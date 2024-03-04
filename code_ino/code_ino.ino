#include "WiFiS3.h"
#include <SimpleDHT.h>
#include <MQ135.h>

#define MIN_SOIL_MOISTURE 0.3

int pinDHT11_1 = 2;
SimpleDHT11 dht11_1(pinDHT11_1);
int pinDHT11_2 = 3;
SimpleDHT11 dht11_2(pinDHT11_2);
typedef struct temperature_umidity {
  byte temperature = 0;
  byte humidity = 0;
} temperatureUmidity;

int led1 = 6;
int led2 = 7;

int pump1 = 12;
int pump2 = 13;

int sensorAir1 = A0;
MQ135 mq135_sensor1(sensorAir1);
int sensorAir2 = A1;
MQ135 mq135_sensor2(sensorAir2);

int pinLight1 = 10;
int pinLight2 = 9;

int pinMoisture1 = A4;
int pinMoisture2 = A5;

bool udare1 = false;
bool udare2 = false;
bool aprindere1 = false;
bool aprindere2 = false;
bool isConnected = false;

char ssid[] = "alex";
char pass[] = "12345678";

int led = LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);


unsigned long lastTimeRead;

temperature_umidity tu1;
temperature_umidity tu2;

float airQuality1;
float airQuality2;

float moisture1;
float moisture2;


void setup() {

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pinLight1, INPUT);
  pinMode(pinLight2, INPUT);

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);      // set the LED pin mode

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(192,48,56,2));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  read_all();
  controll_all();
}
void loop() {
  
  // controll_all();
  // if (millis() - lastTimeRead >= 2500) {
  //   read_all();
  // }
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {  
      //   if (millis() - lastTimeRead >= 2500) {
      //   read_all();
      //   controll_all();
      // }
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            client.print("<html><head><title>Informatii despre Plante</title></head><body>");
            client.print("<div class=\"plant-container\">");
            client.print("<h2>Planta 1</h2>");
            client.print("<div class=\"plant-info\">");
            client.print("<div>Temperatura: " + String(tu1.temperature) + " C</div>");
            client.print("<div>Umiditate: " + String(tu1.humidity) + "%</div>");
            client.print("<div>Calitate Aer: " + String(airQuality1) + "</div>");
            client.print("<div>Umiditate Sol: " + String(moisture1) + "%</div>");
            client.print("</div>");

            client.print("<div class=\"plant-buttons\">");
            client.print("<button><a href=\"/udare1\">Udare</a></button>");
            client.print("<button><a href=\"/aprindere1\">Aprindere Bec</a></button>");
            client.print("</div>");
            client.print("</div>");


            client.print("<div class=\"plant-container\">");
            client.print("<h2>Planta 2</h2>");
            client.print("<div class=\"plant-info\">");
            client.print("<div>Temperatura: " + String(tu2.temperature) + " C</div>");
            client.print("<div>Umiditate: " + String(tu2.humidity) + "%</div>");
            client.print("<div>Calitate Aer: " + String(airQuality2) + "</div>");
            client.print("<div>Umiditate Sol: " + String(moisture2) + "%</div>");
            client.print("</div>");

            client.print("<div class=\"plant-buttons\">");
            client.print("<button><a href=\"/udare2\">Udare</a></button>");
            client.print("<button><a href=\"/aprindere2\">Aprindere Bec</a></button>");
            client.print("</div>");
            client.print("</div>");

            client.print("</body></html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if (currentLine.endsWith("GET /udare1")) {
          udare1 = web_controll(pump1, udare1);
        }
        if (currentLine.endsWith("GET /aprindere1")) {
          aprindere1 = web_controll(led2, aprindere1);
        }
        if (currentLine.endsWith("GET /udare2")) {
          udare2 = web_controll(pump2, udare2);
        }
        if (currentLine.endsWith("GET /aprindere2")) {
          aprindere2 = web_controll(led1, aprindere2);
        }
        //controll_all();
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


void read_all() {
  lastTimeRead = millis();
  tu1 = read_tempereratur_humidity(dht11_1);
  tu2 = read_tempereratur_humidity(dht11_2);

  airQuality1 = read_sensor_air(mq135_sensor1, tu1.temperature, tu1.humidity);
  airQuality2 = read_sensor_air(mq135_sensor2, tu2.temperature, tu2.humidity);

  moisture1 = read_moisture(pinMoisture1);
  moisture2 = read_moisture(pinMoisture2);
}
void controll_all() {
  pumpsControll(moisture1, moisture2);
  light_controll();
}
bool web_controll(int pin, int controll) {
  controll = !controll;
  digitalWrite(pin, controll);
  return controll;
}

//trebuie un delay de cel putin 1.5 secunde pt a reciti acelasi senzor;
temperatureUmidity read_tempereratur_humidity(SimpleDHT11 dht11) {
  int err = SimpleDHTErrSuccess;
  temperatureUmidity aux;
  byte temperature = 0;
  byte humidity = 0;

  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err=");
    Serial.print(SimpleDHTErrCode(err));
    Serial.print(",");
    Serial.println(SimpleDHTErrDuration(err));
    delay(1000);
    return aux;
  }

  aux.humidity = humidity;
  aux.temperature = temperature;
  return aux;
}


void pumpsControll(int moisture1, int moisture2) {
  if (!udare1)
    PumpLogic(moisture1, pump1);
  if (!udare2)
    PumpLogic(moisture2, pump2);
}
void PumpLogic(int moisture, int pump) {
  if (moisture < MIN_SOIL_MOISTURE) {
    Serial.print("Turn on pump ");
    Serial.println(pump);
    digitalWrite(pump, HIGH);
  } else {
    digitalWrite(pump, LOW);
    Serial.print("Turn off pump ");
    Serial.println(pump);
  }
}


float read_sensor_air(MQ135 sensor, float temperature, float humidity) {
  float ppm = sensor.getPPM();
  float correctedPPM = sensor.getCorrectedPPM(temperature, humidity);
  return correctedPPM;
}


void light_controll() {
  if (!aprindere1)
    digitalWrite(led1, digitalRead(pinLight1));
  if (!aprindere2)
    digitalWrite(led2, digitalRead(pinLight2));
}



//se converteste valoare in intervalul [0,1], cu cat e mai aproape de 0 mediul e mai umed
//val de referinta este de 1004 in aer si cu cat mediul e mai umed ea scade
float read_moisture(int pin) {
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    int reading = analogRead(pin);
    sum += reading;
  }
  return (sum / 5.0) / 1004;
}