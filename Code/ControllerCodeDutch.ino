// Waterraket parachute controller
// Geschreven door WondrousWorkshop
// Hardware: Wemos D1 Mini V3, MPU6050, Wemos Battery Shield, Wemos OLED Shield, MG90S servo.
// voor meer informatie zie de instructable: www.instructable.com/

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266WebServer.h>   // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer
#include <Wire.h> 
#include <Servo.h>
#include <Adafruit_MPU6050.h>   // https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_SSD1306.h>   // https://github.com/adafruit/Adafruit_SSD1306  
#include <Adafruit_GFX.h>       // https://github.com/adafruit/Adafruit-GFX-Library

// #Define om ram te besparen al zie ik het nut er niet helemaal van in. Het schijnt een goed idee te zijn om het toch te doen.
#define SERVO_PIN D5 
#define SERVO_GESLOTEN 0  //gesloten positie van de parachute servo
#define SERVO_OPEN 90  //open positie van de parachute servo

#define SCREEN_WIDTH 64  //Voor het OLED shield
#define SCREEN_HEIGHT 48  
#define OLED_RESET -1  //inactief 

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo parachuteServo;
ESP8266WebServer server(80);

enum vluchtstatus {
status_op_platform, //staat stil op platform.
status_vertrokken,  //piek ik verticale g kracht, water levert druk en snelheid stijgt
status_in_vlucht,   //in vlucht, water is op dus geen stuwing meer, snelheid daalt.
status_parachute,   //parachute is geactiveerd.
};
vluchtstatus huidige_staat = status_op_platform;

float zWaarde =0;
String mpuStatus = "OK";

//Finetunen van de fases gaat via de volgende drie waardes:
const float grenswaarde_lancering = 20.0; //Waarde is in m/s^2.  9,81 m/s^2 = 1G
const unsigned long parachutebuffer = 1000; //timer voordat de parachute ARMED is in ms
const float grenswaarde_parachute = 0; //minder dan de zwaartekracht van 9,81m/s^2 dus is vrije val en dus moet de parachute getriggerd worden. Dit kan niet met hellingshoek gedaan worden vanwege hoe de MPU6050 werkt (versnellinsgmeter tov zwaartekracht)

float eindTijdParachute = 0.0; //onthoud de duur van de vlucht van lancering tot eht activeren van de parachute.
unsigned long lanceerTijd =0; //klok voor de parachutebuffer =millies- lanceerTijd > parachutebuffer?

//Plaatje van raket
const unsigned char raketLogo[] PROGMEM = {
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0xc0, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x01, 0xe0, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x06, 0x10, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x0f, 0xfc, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x18, 0x06, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x10, 0x06, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x13, 0xe6, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x16, 0x16, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x16, 0x16, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x13, 0x36, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x11, 0xe6, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x10, 0x86, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x10, 0x06, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x10, 0x06, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x30, 0x07, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0xf0, 0xc6, 0xc0, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0x90, 0xc6, 0x60, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0x98, 0xce, 0x60, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0x8c, 0xcc, 0x60, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0x8c, 0xcc, 0x60, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0x9f, 0xfe, 0x60, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0xb7, 0xf3, 0x60, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0xe7, 0xf1, 0xe0, 0x1f, 0x00, 
  0x00, 0xf0, 0x01, 0xc0, 0xc0, 0xe0, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00
};
//begin website-------------------Geschreven door Gemeni Pro------------------------------------------------------------------------------
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
//einde website----------------------------------------------------

void setup() {
  Serial.begin(115200); //alleen voor testen via USB 
  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(SERVO_GESLOTEN);

  delay (500); // laatste aanpasisng op de Github stelt dit voor.

  //website
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
  
  
  //OLED start en eventuele foutmleding
  while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {   //check of 0x3C klopt op het fysieke Wemos OLED shield
    Serial.println(F("OLED error."));
    Serial.println(F("zoeken.."));
    delay (1000);
    }
  Serial.println(F("OLED verbonden."));  
  
  //laat de raket zien
  display.clearDisplay();
  display.drawBitmap(0, 0, raketLogo, 64, 48, SSD1306_WHITE);
  display.display();
  delay(1000);
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  //MPU6050 start en eventuele foutmleding
  while (!mpu.begin()) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("MPU Error!");
    display.println("zoeken..");
    display.display();
    Serial.println("MPU6050 niet gevonden!");
    delay (1000);
    }
  
  //MPU verbonden bericht
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("MPU ok!");
  display.display();
  delay(1000);

  //MPU bereik instellen
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //max g meting
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);  //demo waarde op de Github van de library
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  //demo waarde op de Githun van de library

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("status:");
  display.println("platform");
  display.display();
}//end setup

void loop() {
  
  server.handleClient();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  zWaarde = a.acceleration.z;
  //display.println("zWaarde"); //Slecht eenmaal gebruiken tijdens orientatietest. Moet een positieve waarde geven. (zwaartekracht trekt namelijk met 9,81m/s^2 aan de sensor)
  
switch (huidige_staat) {
    
    case status_op_platform:
      if (zWaarde > grenswaarde_lancering) {
        huidige_staat = status_vertrokken;
        lanceerTijd = millis();
      } break;

    case status_vertrokken:
      if (millis()-lanceerTijd > parachutebuffer) {
        huidige_staat = status_in_vlucht;
      } break;

    case status_in_vlucht:
      if (zWaarde < grenswaarde_parachute) {
        openParachute();
      } break;

    case status_parachute:
          break;
  }
  
  
}  //end loop

void openParachute(){
  parachuteServo.write(SERVO_OPEN);
  huidige_staat = status_parachute;
  eindTijdParachute = millis() - lanceerTijd)/1000.0;
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(eindTijdParachute);
  display.print(" s vlucht");
  display.display();
  Serial.print("parachute geactiveerd na: "); 
  Serial.println(eindTijdParachute);
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
