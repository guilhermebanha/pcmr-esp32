#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <CRC32.h>
#include <DES.h>
#include <WebServer.h>
#include <WiFiAP.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <string.h>
#include "mbedtls/md5.h"
#include <ArduinoJson.h>
#include <vector>  

#define publishInterval 1000  
#define RED_PIN 23
#define BLUE_PIN 5
#define GREEN_PIN 19
#define SPIN 4
#define CPIN 15
#define BUZZER 18
#define YELLOW_DURATION 10000  
#define GREEN_DURATION 10000   

bool isOffline = false;
unsigned long wifiStartTime = 0;
unsigned long lastOfflineMsgTime = 0;

unsigned long lastCInterruptTime = 0;
unsigned long yellowStartTime = 0;
bool inYellowPhase = false;
bool inGreenPhase = false;
unsigned long greenStartTime = 0;
volatile bool cancelRequested = false;

int bpm_value = 0;
std::vector<std::pair<int, int>> last100Values;
void pushValue(int value, int value2) {
  last100Values.emplace_back(value, value2);

  if (last100Values.size() > 100) {
    last100Values.erase(last100Values.begin());  
  }
}

void clearLast100Values() {
  last100Values.clear();
}

void setColor(int redVal, int greenVal, int blueVal) {
  analogWrite(RED_PIN, redVal);
  analogWrite(GREEN_PIN, greenVal);
  analogWrite(BLUE_PIN, blueVal);
}

void BLUE() {
  digitalWrite(BUZZER, LOW);

  setColor(0, 0, 255);
}

void RED() {
  digitalWrite(BUZZER, HIGH);

  setColor(255, 0, 0);
}

void GREEN() {
  digitalWrite(BUZZER, HIGH);

  setColor(0, 255, 0);
}

void YELLOW() {
  digitalWrite(BUZZER, HIGH);

  setColor(0, 124, 64);

}

String getJsonFromValues() {
  DynamicJsonDocument dataArrayDoc(4096);
  JsonArray dataArray = dataArrayDoc.to<JsonArray>();

  for (const auto& p : last100Values) {
    JsonObject obj = dataArray.createNestedObject();
    obj["bpm"] = p.first;
    obj["milis"] = p.second;
  }

  String dataArrayStr;
  serializeJson(dataArrayDoc, dataArrayStr);

  DynamicJsonDocument fullDoc(4096);
  fullDoc["data"] = dataArray;

  String finalJson;
  serializeJson(fullDoc, finalJson);
  return finalJson;
}

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
    for (int i = dataLen; i < paddedLen; ++i) {
      buffer[i] = padValue;
    }
  }

  int removePKCS7Padding(uint8_t* buffer, int length) {
    if (length == 0) return 0;
    uint8_t padValue = buffer[length - 1];
    if (padValue > 8) return length;  
    return length - padValue;
  }

  uint8_t hexCharToByte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
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
    for (int i = 0; i < paddedLen; ++i) {
      if (encrypted[i] < 16) hexPayload += "0";
      hexPayload += String(encrypted[i], HEX);
    }

    delete[] buffer;
    delete[] encrypted;
    return hexPayload;
  }

  String Decrypt(const String& hexPayload) {
    int len = hexPayload.length();
    if (len % 16 != 0) return "";  

    int byteLen = len / 2;
    uint8_t* encrypted = new uint8_t[byteLen];
    for (int i = 0; i < byteLen; ++i) {
      encrypted[i] = (hexCharToByte(hexPayload[2 * i]) << 4) | hexCharToByte(hexPayload[2 * i + 1]);
    }

    uint8_t* decrypted = new uint8_t[byteLen];
    for (int i = 0; i < byteLen; i += 8) {
      des.decrypt(decrypted + i, encrypted + i, key);
    }

    int unpaddedLen = removePKCS7Padding(decrypted, byteLen);

    String result = "";
    for (int i = 0; i < unpaddedLen; ++i) {
      result += (char)decrypted[i];
    }

    delete[] encrypted;
    delete[] decrypted;
    return result;
  }
};

bool verifyCRC(const String& decryptedJsonStr) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, decryptedJsonStr);
  if (error) {
    Serial.print(F("Deserialization failed: "));
    Serial.println(error.f_str());
    return false;
  }

  const char* receivedCrcStr = doc["crc32"];
  if (!receivedCrcStr) {
    Serial.println("CRC32 field not found in JSON.");
    return false;
  }

  String originalCRC = String(receivedCrcStr);
  doc.remove("crc32");

  String dataToCheck;
  serializeJson(doc, dataToCheck);

  CRC32 crc;
  uint32_t calculatedCRC = crc.calculate((uint8_t*)dataToCheck.c_str(), dataToCheck.length());
  String calculatedCRCStr = "0x" + String(calculatedCRC, HEX);

  return originalCRC.equalsIgnoreCase(calculatedCRCStr);
}
#include <stdio.h>
#include <string.h>
#include "mbedtls/md5.h"

void calculate_md5(const char* input, char* output_hex) {
  unsigned char digest[16];
  mbedtls_md5_context ctx;

  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts(&ctx);  
  mbedtls_md5_update(&ctx, (const unsigned char*)input, strlen(input));
  mbedtls_md5_finish(&ctx, digest);
  mbedtls_md5_free(&ctx);

  for (int i = 0; i < 16; ++i)
    sprintf(output_hex + (i * 2), "%02x", digest[i]);

  output_hex[32] = '\0';  
}

class MQTTHandler {
private:
  WiFiClient* espClient;
  PubSubClient* client;
  char* topic;
  char* ssid;
  char* pwd;
  char* server;
  int port;
  volatile bool* simulatePtr;
  int* bpmVPtr;

public:
  MQTTHandler() {}

  void init(char* ssid, char* pwd, char* server, int port, char* topic, volatile bool* simulatePtr, int* bomVPtr) {
    this->setSSID(ssid);
    this->setPWD(pwd);
    this->setServer(server);
    this->setPort(port);
    this->setTopic(topic);
    this->espClient = new WiFiClient();
    this->client = new PubSubClient(*this->espClient);
    this->simulatePtr = simulatePtr;
    this->bpmVPtr = bomVPtr;
    this->client->setBufferSize(1028);
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
        this->client->subscribe("heartbit/cmd");
        this->client->setCallback([&](char* topic, byte* message, unsigned int length) {
          String messageStr;
          for (unsigned int i = 0; i < length; i++) {
            messageStr += (char)message[i];
          }

          Serial.println(messageStr);

          DESHandler desHandler("12345678");
          String decrypted = desHandler.Decrypt(messageStr);

          if (!verifyCRC(decrypted)) {
            Serial.println("Verificacao CRC Falhada.");
            return;
          }

          StaticJsonDocument<512> doc;
          DeserializationError error = deserializeJson(doc, decrypted);
          if (error) {
            Serial.print(F("Erro de descerializacoa: "));
            Serial.println(error.f_str());
            return;
          }

          if (doc["command"] == "send_offline_data") {
            String finalJson = getJsonFromValues();
            StaticJsonDocument<512> payload;
            DeserializationError erroP = deserializeJson(payload, finalJson);
            if (erroP) {
              Serial.print(F("Erro de descerializacoa: "));
              Serial.println(erroP.f_str());
              return;
            }
            Serial.println("=====");
            Serial.println(finalJson);
            char md5Value[33];
            calculate_md5(finalJson.c_str(), md5Value);
            payload["md5"] = String(md5Value);
            serializeJson(payload, finalJson);
            String encryptedPayload = desHandler.Encrypt(finalJson);
            client->publish("heartbit/offline", encryptedPayload.c_str());
            Serial.println("📤 Sent encrypted offline data.");
            clearLast100Values();
          }
        });
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

  void resetRates() {
    memset(this->rates, 0, sizeof(this->rates));
    this->rateSpot = 0;
    this->beatAvg = 0;
  }

  void setFLAG_MISBPM(uint8_t FLAG_MISBPM) {
    this->FLAG_MISBPM = FLAG_MISBPM;
  }

  int getFLAG_MISBPM() {
    return this->FLAG_MISBPM;
  }

  float getBPM() {
    return this->beatsPerMinute;
  }

  float getAVG() {
    return this->beatAvg;
  }

  int checkFinger() {
    return this->irValue < 50000 ? 0 : 1;
  }

  bool isValid() {
    return this->getBPM() >= this->FLAG_MISBPM;
  }
};

HeartSensor hs;
MQTTHandler mqtt_handler;
CRC32 crc;
DESHandler desHandler("12345678");
GPSHandler myGPS(Serial2);

String emergencyStatus = "null";
bool statusSent = false;
volatile bool simulate = false;
volatile bool hasCache = false;

unsigned long lastPublishTime = 0;
unsigned long lastSInterruptTime = 0;
unsigned long warningStartTime = 0;
unsigned long cancelledStartTime = 0;

volatile bool getSimulate() {
  return simulate;
}

void setSimulate(volatile bool* value) {
  simulate = *value;
}

void IRAM_ATTR toggleSimulate() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastSInterruptTime > 200) {
    simulate = true;
    lastSInterruptTime = interruptTime;
  }
}

void IRAM_ATTR cancelEmergency() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastCInterruptTime > 200) {
    cancelRequested = true;
    simulate = false;
    bpm_value = 0;
    lastCInterruptTime = interruptTime;
    hs.resetRates();
  }
}

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
  wifiStartTime = millis();
  IPAddress IP = WiFi.softAPIP();
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
                        (char*)ipInput.c_str(), 1883, (char*)"heartbit/bpm", &simulate, &bpm_value);
    } else {
      Serial.println("Failed to connect.");
    }
  });

  server.begin();

  pinMode(SPIN, INPUT_PULLUP);
  pinMode(CPIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  attachInterrupt(digitalPinToInterrupt(SPIN), toggleSimulate, FALLING);
  attachInterrupt(digitalPinToInterrupt(CPIN), cancelEmergency, FALLING);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  BLUE();
}

enum EmergencyPhase {
  PHASE_IDLE,
  PHASE_YELLOW,
  PHASE_GREEN,
  PHASE_BLUE,
  PHASE_RED
};

EmergencyPhase emergencyPhase = PHASE_IDLE;
unsigned long phaseStartTime = 0;
bool emergencyTriggered = false;

void loop() {

  unsigned long now = millis();

  if ((bpm_value >= 110 || getSimulate()) && !emergencyTriggered) {
    emergencyTriggered = true;
    cancelRequested = false;
    emergencyPhase = PHASE_YELLOW;
    phaseStartTime = millis();
  }

  if (emergencyTriggered || getSimulate()) {
    switch (emergencyPhase) {

      case PHASE_YELLOW:
        YELLOW();
        if (cancelRequested) {
          emergencyPhase = PHASE_GREEN;
          phaseStartTime = millis();
        } else if (millis() - phaseStartTime >= 10000) {
          emergencyPhase = PHASE_RED;
          phaseStartTime = millis();
        }
        break;

      case PHASE_GREEN:
        GREEN();
        if (millis() - phaseStartTime >= 10000) {
          emergencyPhase = PHASE_BLUE;
          phaseStartTime = millis();
        }
        break;

      case PHASE_BLUE:
        BLUE();
        if (millis() - phaseStartTime >= 10000) {
          emergencyPhase = PHASE_IDLE;
          emergencyTriggered = false;
          cancelRequested = false;
        }
        break;

      case PHASE_RED:
        RED();
        if (millis() - phaseStartTime >= 1000) {
          phaseStartTime = millis();  
        }
        break;

      default:
        break;
    }
  }

  hasCache = last100Values.size() > 0;
  if (simulate) bpm_value = random(160, 201);

  if (!wifiConfigured) {
    server.handleClient();

    if (!isOffline && (now - wifiStartTime > 6000)) {
      isOffline = true;
      Serial.println("⚠️  Entrou em modo offline.");
    }

    if (isOffline && (now - lastOfflineMsgTime > 1000)) {
      if ((hs.isValid() && hs.checkFinger()) || simulate) {
        pushValue(bpm_value, now);
      }
      lastOfflineMsgTime = now;
    }

    return;
  }

  if (!mqtt_handler.getClient()->connected()) {
    mqtt_handler.doConnect();
  }
  mqtt_handler.getClient()->loop();

  hs.updateBPM();
  myGPS.update();

  if (now > 5000 && !myGPS.hasData()) {
    Serial.println("No GPS data received: check wiring");
    while (true)
      ;  
  }
  delay(publishInterval);
  bool c = getSimulate();
  Serial.println(getJsonFromValues());
  if (c) {
    StaticJsonDocument<256> doc;
    doc["bpm"] = bpm_value;
    doc["lat"] = myGPS.getLat();
    doc["lon"] = myGPS.getLon();
    doc["status"] = (emergencyPhase == PHASE_RED) ? "4" : "0" ;
    doc["hasCache"] = hasCache;
    doc["timestamp"] = now;
    doc["finger_present"] = true;

    String jsonStr;
    serializeJson(doc, jsonStr);

    uint32_t checksum = crc.calculate((uint8_t*)jsonStr.c_str(), jsonStr.length());
    doc["crc32"] = "0x" + String(checksum, HEX);

    String finalPayload;
    serializeJson(doc, finalPayload);
    Serial.println(finalPayload);
    String finalString = desHandler.Encrypt(finalPayload);
    mqtt_handler.getClient()->publish(mqtt_handler.getTopic(), finalString.c_str());
    delay(20);
  } else if (emergencyPhase == PHASE_GREEN) {

    StaticJsonDocument<256> doc;
    doc["bpm"] = bpm_value;
    doc["lat"] = myGPS.getLat();
    doc["lon"] = myGPS.getLon();
    doc["status"] = "1";
    doc["hasCache"] = hasCache;
    doc["timestamp"] = now;
    doc["finger_present"] = true;

    String jsonStr;
    serializeJson(doc, jsonStr);

    uint32_t checksum = crc.calculate((uint8_t*)jsonStr.c_str(), jsonStr.length());
    doc["crc32"] = "0x" + String(checksum, HEX);

    String finalPayload;
    serializeJson(doc, finalPayload);
    Serial.println(finalPayload);
    String finalString = desHandler.Encrypt(finalPayload);
    mqtt_handler.getClient()->publish(mqtt_handler.getTopic(), finalString.c_str());
    delay(20);

  } else if ((hs.isValid() && hs.checkFinger())) {

    bpm_value = hs.getAVG();

    StaticJsonDocument<256> doc;
    doc["bpm"] = bpm_value;
    doc["lat"] = myGPS.getLat();
    doc["lon"] = myGPS.getLon();
    doc["status"] = emergencyStatus;
    doc["hasCache"] = hasCache;
    doc["timestamp"] = now;
    doc["finger_present"] = true;

    String jsonStr;
    serializeJson(doc, jsonStr);

    uint32_t checksum = crc.calculate((uint8_t*)jsonStr.c_str(), jsonStr.length());
    doc["crc32"] = "0x" + String(checksum, HEX);

    String finalPayload;
    serializeJson(doc, finalPayload);

    String finalString = desHandler.Encrypt(finalPayload);
    mqtt_handler.getClient()->publish(mqtt_handler.getTopic(), finalString.c_str());

  } else {
    Serial.println(hasCache);
    if (hasCache) {
      StaticJsonDocument<128> doc;
      doc["hasCache"] = true;
      String jsonStr;
      serializeJson(doc, jsonStr);
      uint32_t checksum = crc.calculate((uint8_t*)jsonStr.c_str(), jsonStr.length());
      doc["crc32"] = "0x" + String(checksum, HEX);
      String finalPayload;
      serializeJson(doc, finalPayload);
      mqtt_handler.getClient()->publish(mqtt_handler.getTopic(), (desHandler.Encrypt(finalPayload)).c_str());
    }
  }
}