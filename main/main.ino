#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
//=================INCLUDES======================
#include <WiFi.h>
#include <PubSubClient.h>
#include <CRC32.h>
#include <DES.h>
#include <WebServer.h>
#include <WiFiAP.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
//================================================

//==================DEFINES=======================
#define publishInterval 1000  // 1 second
#define RED 23
#define BLUE 5
#define GREEN 19
#define SPIN 4
#define CPIN 15
#define BUZZER 18
//================================================

//==================WEB INSTANCES==================
WebServer server(80);
String ssidInput = "", passwordInput = "", ipInput = "";
bool wifiConfigured = false;

const char* portal_html = R"rawliteral(
  <html>
  <head>
    <style>
      body {
        font-family: Arial, sans-serif;
        background-color: #f0f2f5;
        padding: 20px;
      }
      h2 {
        color: #333;
      }
      form {
        background-color: #fff;
        padding: 20px;
        border-radius: 10px;
        box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        max-width: 300px;
      }
      input[type="text"], input[type="password"] {
        width: 100%;
        padding: 8px;
        margin: 8px 0;
        border-radius: 5px;
        border: 1px solid #ccc;
      }
      input[type="submit"] {
        width: 100%;
        padding: 10px;
        background-color: #007BFF;
        color: white;
        border: none;
        border-radius: 5px;
        cursor: pointer;
      }
      input[type="submit"]:hover {
        background-color: #0056b3;
      }
      .footer-link {
        margin-top: 15px;
        display: block;
        text-align: center;
        color: #007BFF;
        text-decoration: none;
      }
      .footer-link:hover {
        text-decoration: underline;
      }
    </style>
  </head>
  <body>
    <h2>WiFi Setup</h2>
    <form action="/set">
      SSID:<br><input name="ssid" type="text"><br>
      Password:<br><input name="pass" type="password"><br>
      Server IP:<br><input name="ip" type="text" placeholder="e.g., 192.168.118.102"><br>
      <input type="submit" value="Connect">
    </form>
    <a class="footer-link" href="https://www.google.com" target="_blank">Visit Google</a>
  </body>
  </html>
)rawliteral";
//================================================

//==================CLASSES=======================

class GPSHandler {
private:
  TinyGPSPlus gps;
  HardwareSerial& gpsSerial;

public:
  GPSHandler(HardwareSerial& serial)
    : gpsSerial(serial) {}

  void begin() {
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  }

  void update() {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  }

  double getLat() {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
  }

  double getLon() {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
  }

  bool hasData() {
    return gps.charsProcessed() > 10;
  }
};


class DESHandler {
private:
  DES des;
  uint8_t key[8];

  void applyPKCS7Padding(uint8_t* buffer, int dataLen, int paddedLen) {
    uint8_t padValue = paddedLen - dataLen;
    for (int i = dataLen; i < paddedLen; i++) {
      buffer[i] = padValue;
    }
  }

public:
  DESHandler(const char* keyStr) {
    memcpy(key, keyStr, 8);
  }

  String Encrypt(const String& plaintext) {
    int dataLen = plaintext.length();
    int paddedLen = ((dataLen + 7) / 8) * 8;

    uint8_t* buffer = new uint8_t[paddedLen];
    memset(buffer, 0, paddedLen);
    memcpy(buffer, plaintext.c_str(), dataLen);

    applyPKCS7Padding(buffer, dataLen, paddedLen);

    uint8_t* encrypted = new uint8_t[paddedLen];
    for (int i = 0; i < paddedLen; i += 8) {
      des.encrypt(encrypted + i, buffer + i, key);
    }

    String hexPayload = "";
    for (int i = 0; i < paddedLen; i++) {
      if (encrypted[i] < 16) hexPayload += "0";
      hexPayload += String(encrypted[i], HEX);
    }

    delete[] buffer;
    delete[] encrypted;

    return hexPayload;
  }
};

class MQTTHandler {
private:
  WiFiClient* espClient;
  PubSubClient* client;
  char* topic;
  char* ssid;
  char* pwd;
  char* server;
  int port;
public:
  MQTTHandler() {}
  void init(char* ssid, char* pwd, char* server, int port, char* topic) {
    this->setSSID(ssid);
    this->setPWD(pwd);
    this->setServer(server);
    this->setPort(port);
    this->setTopic(topic);
    this->espClient = new WiFiClient();
    this->client = new PubSubClient(*this->espClient);
  }
  void setTopic(char* topic) {
    this->topic = topic;
  }
  void setSSID(char* ssid) {
    this->ssid = ssid;
  }
  void setPWD(char* pwd) {
    this->pwd = pwd;
  }
  void setServer(char* server) {
    this->server = server;
  }
  void setPort(int port) {
    this->port = port;
  }
  char* getTopic() {
    return this->topic;
  }
  PubSubClient* getClient() {
    return this->client;
  }
  void doConnect() {
    connectWiFi();
    connectMQTT();
  }

  void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(this->ssid, this->pwd);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println(" connected!");
  }

  void connectMQTT() {
    this->client->setServer(this->server, this->port);
    while (!(this->client->connected())) {
      Serial.print("Connecting to MQTT...");
      if (this->client->connect("ESP32Client")) {
        Serial.println(" connected!");
      } else {
        Serial.print(" failed, rc=");
        Serial.print(this->client->state());
        delay(2000);
      }
    }
  }
};

class HeartSensor {
private:
  uint8_t FLAG_MISBPM;
  long irValue;
  MAX30105 particleSensor;
  int beatAvg;
  float beatsPerMinute;
  long lastBeat;
  byte rateSpot;
  byte rates[4];
public:
  HeartSensor() {
    this->lastBeat = 0;
    this->rateSpot = 0;
    this->irValue = 0;
  }
  void init() {
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
      Serial.println("GY-MAX30100 was not found. Please check wiring/power.");
      while (1)
        ;
    }
    this->particleSensor.setup();
    this->particleSensor.setPulseAmplitudeRed(0x0A);
    this->particleSensor.setPulseAmplitudeGreen(0);
  }
  void updateBPM() {
    this->irValue = this->particleSensor.getIR();

    if (checkForBeat(this->irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      this->beatsPerMinute = 60 / (delta / 1000.0);

      if (this->beatsPerMinute < 255 && this->beatsPerMinute > 20) {
        this->rates[this->rateSpot++] = (byte)this->beatsPerMinute;
        this->rateSpot %= 4;

        this->beatAvg = 0;
        for (byte x = 0; x < 4; x++)
          this->beatAvg += rates[x];
        this->beatAvg /= 4;
      }
    }
  }
  void setFLAG_MISBPM(uint8_t FLAG_MISBPM) {
    this->FLAG_MISBPM = FLAG_MISBPM;
  }
  float getBPM() {
    return this->beatsPerMinute;
  }
  float getAVG() {
    return this->beatAvg;
  }
  int checkFinger() {
    return this->irValue < 50000 ? 1 : 0;
  }
  bool isValid() {
    return this->getBPM() >= this->FLAG_MISBPM;
  }
};
//================================================


//==================MAIN INSTANCES=================

HeartSensor hs;
MQTTHandler mqtt_handler;
CRC32 crc;
DESHandler desHandler("12345678");
GPSHandler myGPS(Serial2);
// Variables
String emergencyStatus = "null";
bool statusSent = false;
volatile bool simulate = false;
volatile bool emergency = false;
volatile bool emergencyC = false;
unsigned long lastPublishTime = 0;
unsigned long lastSInterruptTime = 0;
unsigned long lastCInterruptTime = 0;
bool isYellow = false;
bool isGreen = false;
bool isRed = false;
unsigned long yellowStartTime = 0;
unsigned long greenStartTime = 0;
volatile bool cancelRequested = false;
//================================================

void IRAM_ATTR toggleSimulate() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastSInterruptTime > 200) {
    simulate = !simulate;
    lastSInterruptTime = interruptTime;
  }
}

void IRAM_ATTR cancelEmergncy() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastCInterruptTime > 200) {
    cancelRequested = true;
    lastCInterruptTime = interruptTime;
  }
}

bool isCanceled() {
  if (isGreen) {
    return true;
  } else if (isRed) {
    return false;
  }
  return false;  // default
}


//==================ESP32 DEFAULT=================
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  myGPS.begin();
  hs.init();
  hs.setFLAG_MISBPM(60);

  IPAddress local_ip(192, 168, 1, 1);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP("HeartMonitor_Config");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", []() {
    server.send(200, "text/html", portal_html);
  });

  server.on("/set", []() {
    ssidInput = server.arg("ssid");
    passwordInput = server.arg("pass");
    ipInput = server.arg("ip");

    server.send(200, "text/html", "<h1>Trying to connect...</h1>");

    WiFi.begin(ssidInput.c_str(), passwordInput.c_str());
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
      delay(500);
      Serial.print(".");
      tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi!");
      wifiConfigured = true;

      mqtt_handler.init((char*)ssidInput.c_str(), (char*)passwordInput.c_str(),
                        (char*)ipInput.c_str(), 1883, (char*)"heartbit/bpm");
    } else {
      Serial.println("Failed to connect.");
    }
  });

  server.begin();

  while (!wifiConfigured) {
    server.handleClient();
    delay(10);
  }
  pinMode(SPIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);  // Ensure it's off at startup
  attachInterrupt(digitalPinToInterrupt(SPIN), toggleSimulate, FALLING);
  attachInterrupt(digitalPinToInterrupt(CPIN), cancelEmergncy, FALLING);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH);
}

void loop() {

  if (!wifiConfigured) {
    server.handleClient();
    return;
  }

  if (!mqtt_handler.getClient()->connected()) {
    mqtt_handler.doConnect();
  }
  mqtt_handler.getClient()->loop();

  hs.updateBPM();
  myGPS.update();

  if (millis() > 5000 && !myGPS.hasData()) {
    Serial.println("No GPS data received: check wiring");
    while (true)
      ;
  }

  double lat = myGPS.getLat();
  double lon = myGPS.getLon();

  randomSeed(esp_random());

  
  int bpm_value = simulate ? random(160, 201) : hs.getAVG();


  if ((hs.isValid() && !(hs.checkFinger())) || simulate == true) {
    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= publishInterval) {
      unsigned long timestamp = millis();

      String payload = "{\"bpm\":";
      payload += bpm_value;
      payload += ",\"lat\":";
      payload += lat;
      payload += ",\"lon\":";
      payload += lon;
      payload += ",\"status\":";
      payload += emergencyStatus;
      payload += ",\"timestamp\":";
      payload += timestamp;
      payload += "}";

      uint32_t checksum = crc.calculate((uint8_t*)payload.c_str(), payload.length());
      payload = "{\"bpm\":" + String(bpm_value) + ",\"lat\":" + String(lat) + ",\"lon\":" + String(lon) + ",\"status\":\"" + String(emergencyStatus) + "\",\"timestamp\":" + String(timestamp) + ",\"crc32\":\"0x" + String(checksum, HEX) + "\"}";
      Serial.println(payload);
      payload = desHandler.Encrypt(payload);

      mqtt_handler.getClient()->publish(mqtt_handler.getTopic(), payload.c_str());
      lastPublishTime = currentTime;
    }
  }

  delay(20);

  static bool emergencyTriggered = false;

  if (bpm_value >= 110) {
    if (!emergencyTriggered) {
      emergencyTriggered = true;
      isYellow = true;
      yellowStartTime = millis();
      analogWrite(RED, 64);
      analogWrite(GREEN, 128);
      analogWrite(BLUE, 0);
      digitalWrite(BUZZER, HIGH);  // Start buzzer immediately
    }

    if (isYellow) {
      if (cancelRequested) {
        cancelRequested = false;
        isYellow = false;
        isGreen = true;
        greenStartTime = millis();
        analogWrite(RED, 0);
        analogWrite(GREEN, 255);
        analogWrite(BLUE, 0);

        if (!statusSent) {
          emergencyStatus = "false";
          statusSent = true;
        }
      } else if (millis() - yellowStartTime >= 10000) {
        isYellow = false;
        isRed = true;
        analogWrite(RED, 255);
        analogWrite(GREEN, 0);
        analogWrite(BLUE, 0);

        if (!statusSent) {
          emergencyStatus = "true";
          statusSent = true;
        }
      }
    }


    // Green Phase: Cancel successful
    if (isGreen) {
      if (millis() - greenStartTime >= 10000) {
        isGreen = false;
        emergencyTriggered = false;
        toggleSimulate();  // if this is intended
        digitalWrite(BUZZER, LOW);
        analogWrite(RED, 0);
        analogWrite(GREEN, 0);
        analogWrite(BLUE, 255);
        emergencyStatus = "null";
        statusSent = false;
      }
    }

    // Red phase never auto-resets unless you add code here
  }
}
//================================================