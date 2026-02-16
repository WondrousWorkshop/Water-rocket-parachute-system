// Waterraket parachute controller (WiFi Only Edition)
// Geschreven door WondrousWorkshop
// Hardware: Wemos D1 Mini V3, MPU6050, Wemos Battery Shield, MG90S servo.
// voor meer informatie zie de instructable: www.instructable.com/

#include <ESP8266WiFi.h>        
#include <ESP8266WebServer.h>   
#include <Wire.h> 
#include <Servo.h>
#include <Adafruit_MPU6050.h>   

#define SERVO_PIN D5 
#define SERVO_GESLOTEN 0  // gesloten positie van de parachute servo
#define SERVO_OPEN 90     // open positie van de parachute servo

Adafruit_MPU6050 mpu;
Servo parachuteServo;
ESP8266WebServer server(80);

enum vluchtstatus {
  status_op_platform, // staat stil op platform.
  status_vertrokken,  // piek in verticale g kracht, water levert druk en snelheid stijgt
  status_in_vlucht,   // in vlucht, water is op dus geen stuwing meer, snelheid daalt.
  status_parachute,   // parachute is geactiveerd.
};
vluchtstatus huidige_staat = status_op_platform;

float zWaarde = 0;
String mpuStatus = "OK";

// Finetunen van de fases gaat via de volgende drie waardes:
const float grenswaarde_lancering = 20.0; // Waarde is in m/s^2.  9,81 m/s^2 = 1G
const unsigned long parachutebuffer = 1000; // timer voordat de parachute ARMED is in ms
const float grenswaarde_parachute = 0; // 0 is perfecte vrije val (0G). Hierdoor opent de parachute onafhankelijk van rotatie.

float eindTijdParachute = 0.0; // onthoudt de duur van de vlucht.
unsigned long lanceerTijd = 0; // klok voor de parachutebuffer

//begin website--------------------------Geschreven door Gemini---------------------------------------------------------------------------------
const char PAGE_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Raket Ground Control</title>
  <style>
    body { font-family: 'Arial', sans-serif; background-color: #121212; color: #ffffff; text-align: center; margin: 0; padding: 20px; }
    h1 { color: #00ffcc; }
    .box { background-color: #1e1e1e; border-radius: 10px; padding: 15px; margin: 10px auto; max-width: 300px; border: 1px solid #333; }
    .waarde { font-size: 22px; font-weight: bold; color: #ffbf00; display: block; margin-top: 5px; }
    .ok { color: #00ff00; }
    .fout { color: #ff0000; }
    .btn-paniek { background-color: #cc0000; color: white; font-size: 20px; font-weight: bold; padding: 15px 30px; border: none; border-radius: 8px; margin-top: 20px; cursor: pointer; width: 100%; max-width: 330px; box-shadow: 0px 4px 10px rgba(204, 0, 0, 0.5); }
    .btn-paniek:active { background-color: #ff0000; transform: scale(0.95); }
  </style>
  <script>
    let timeoutId;

    function haalData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('mpu').innerHTML = data.mpu;
          document.getElementById('staat').innerHTML = data.staat;
          document.getElementById('tijd').innerHTML = data.tijd + " s";
          
          if(data.mpu === "OK") document.getElementById('mpu').className = "waarde ok";
          else document.getElementById('mpu').className = "waarde fout";

          // --- DE SMART POLLING LOGICA ---
          let wachttijd = 500; // Standaard: 2x per seconde checken (op het platform)

          if (data.staat === "Lancering Detectie" || data.staat === "Coasting") {
             // RAKET IS IN DE LUCHT! We stoppen met het lastigvallen van de Wemos.
             // Geef de processor 100% rust voor de val-detectie.
             wachttijd = 2500; // Wacht 2,5 seconden voordat we weer iets vragen
          } else if (data.staat === "Parachute Open!") {
             wachttijd = 2000; // Vlucht is voorbij, rustig aan doen.
          }

          timeoutId = setTimeout(haalData, wachttijd);
        })
        .catch(() => {
          // WiFi bereik kwijt? Probeer het over 1 seconde nog eens stilzwijgend
          document.getElementById('staat').innerHTML = "VERBINDING WEG...";
          document.getElementById('staat').style.color = "red";
          timeoutId = setTimeout(haalData, 1000);
        });
    }

    window.onload = haalData;

    function noodParachute() {
      if(confirm("🚨 WEET JE ZEKER DAT JE DE PARACHUTE NU WILT OPENEN?")) {
        fetch('/panic');
      }
    }
  </script>
</head>
<body>
  <h1>🚀 Ground Control</h1>
  
  <div class="box">Sensor: <span id="mpu" class="waarde">Laden...</span></div>
  <div class="box">Fase: <span id="staat" class="waarde">Laden...</span></div>
  <div class="box">Vluchttijd: <span id="tijd" class="waarde">0.00 s</span></div>

  <button class="btn-paniek" onclick="noodParachute()">🚨 NOOD-PARACHUTE</button>
</body>
</html>
)=====";
//einde website------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200); 
  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(SERVO_GESLOTEN);

  delay (500); 

  // --- Start Webserver ---
  Serial.println("\nStarten WiFi Access Point...");
  WiFi.softAP("Waterraket", "lancering"); 
  Serial.print("IP Adres: ");
  Serial.println(WiFi.softAPIP());
  
  server.on("/", []() { server.send(200, "text/html", PAGE_HTML); });
  server.on("/data", stuurData); 
  
  server.on("/panic", []() { 
    if(huidige_staat != status_parachute) {
      openParachute(); 
    }
    server.send(200, "text/plain", "Parachute geforceerd open!"); 
  });

  server.begin();
  
  // --- Start MPU6050 ---
  if (!mpu.begin()) {
    Serial.println("MPU6050 niet gevonden!");
    mpuStatus = "ERROR";
  } else {
    Serial.println("MPU ok!");
    mpuStatus = "OK";
    
    // MPU bereik instellen
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G); 
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);  
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  }
} // end setup

void loop() {
  
  server.handleClient();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  zWaarde = a.acceleration.z;
  
  switch (huidige_staat) {
    
    case status_op_platform:
      if (zWaarde > grenswaarde_lancering) {
        huidige_staat = status_vertrokken;
        lanceerTijd = millis();
      } 
      break;

    case status_vertrokken:
      if (millis() - lanceerTijd > parachutebuffer) {
        huidige_staat = status_in_vlucht;
      } 
      break;

    case status_in_vlucht:
      if (zWaarde < grenswaarde_parachute) {
        openParachute();
      } 
      break;

    case status_parachute:
      // Wachten op ground control om de raket te vinden.
      break;
  }
}  // end loop

void openParachute(){
  parachuteServo.write(SERVO_OPEN);
  huidige_staat = status_parachute;
  eindTijdParachute = (millis() - lanceerTijd) / 1000.0;
  
  Serial.print("Parachute geactiveerd na: "); 
  Serial.print(eindTijdParachute);
  Serial.println(" seconden.");
}

String getStaatNaam() {
  if(huidige_staat == status_op_platform) return "Op Platform";
  if(huidige_staat == status_vertrokken) return "Lancering Detectie";
  if(huidige_staat == status_in_vlucht) return "Coasting";
  if(huidige_staat == status_parachute) return "Parachute Open!";
  return "Onbekend";
}

void stuurData() {
  float weergaveTijd = 0.0;
  if (huidige_staat == status_vertrokken || huidige_staat == status_in_vlucht) {
    weergaveTijd = (millis() - lanceerTijd) / 1000.0; 
  } else if (huidige_staat == status_parachute) {
    weergaveTijd = eindTijdParachute; 
  }

  String json = "{";
  json += "\"mpu\":\"" + mpuStatus + "\",";
  json += "\"staat\":\"" + getStaatNaam() + "\",";
  json += "\"tijd\":\"" + String(weergaveTijd, 2) + "\""; 
  json += "}";
  
  server.send(200, "application/json", json);
}
