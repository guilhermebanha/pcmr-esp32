#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <CRC32.h>
#include <DES.h>

#define publishInterval 1000  // 1 second

class DESHandler {
private:
  DES des;
  uint8_t key[8];
  
  // Aplica padding PKCS#7
  void applyPKCS7Padding(uint8_t* buffer, int dataLen, int paddedLen) {
    uint8_t padValue = paddedLen - dataLen;
    for (int i = dataLen; i < paddedLen; i++) {
      buffer[i] = padValue;
    }
  }

public:
  // Construtor: define a chave
  DESHandler(const char* keyStr) {
    memcpy(key, keyStr, 8);
  }

  // Encripta texto completo, retorna HEX
  String Encrypt(const String& plaintext) {
    int dataLen = plaintext.length();
    int paddedLen = ((dataLen + 7) / 8) * 8;

    uint8_t buffer[paddedLen];
    memset(buffer, 0, paddedLen);
    memcpy(buffer, plaintext.c_str(), dataLen);

    applyPKCS7Padding(buffer, dataLen, paddedLen);

    uint8_t encrypted[paddedLen];
    for (int i = 0; i < paddedLen; i += 8) {
      des.encrypt(encrypted + i, buffer + i, key);
    }

    // Converter para HEX
    String hexPayload = "";
    for (int i = 0; i < paddedLen; i++) {
      if (encrypted[i] < 16) hexPayload += "0";
      hexPayload += String(encrypted[i], HEX);
    }

    return hexPayload;
  }
};

// #define WIFI_SSID "Osmanthus Wine"
// #define WIFI_PASSWORD "6GEoarchon03"
// #define MQTT_BROKER "192.168.1.67"
// #define MQTT_PORT 1883
// #define MQTT_TOPIC "heartbit/bpm"
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
    // Default values for first time
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
// HeartSensor class (same as before)
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

HeartSensor hs;
MQTTHandler mqtt_handler;
CRC32 crc;
DESHandler desHandler("12345678");
void setup() {
  Serial.begin(9600);
  hs.init();
  hs.setFLAG_MISBPM(60);
  mqtt_handler.init("Bankai", "alguemnao", "192.168.118.102", 1883, "heartbit/bpm");
}
unsigned long lastPublishTime = 0;

void loop() {
  if (!mqtt_handler.getClient()->connected()) {
    mqtt_handler.doConnect();
  }
  mqtt_handler.getClient()->loop();

  hs.updateBPM();
  int curr = hs.getAVG();

  if (hs.isValid() && !(hs.checkFinger())) {
    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= publishInterval) {

      // Add a timestamp (in milliseconds since program start)
      unsigned long timestamp = millis();

      // Create JSON payload
      String payload = "{\"bpm\":";
      payload += curr;
      payload += ",\"timestamp\":";
      payload += timestamp;
      payload += "}";

      uint32_t checksum = crc.calculate((uint8_t*)payload.c_str(), payload.length());
      payload = "{\"bpm\":" + String(curr) + 
                ",\"timestamp\":" + String(timestamp) + 
                ",\"crc32\":\"0x" + String(checksum, HEX) + "\"}";

      payload = desHandler.Encrypt(payload);

      // Publish encrypted payload
      mqtt_handler.getClient()->publish(mqtt_handler.getTopic(), payload.c_str());

      // Update last publish time
      lastPublishTime = currentTime;
    }
  }

  delay(20);  // Short delay to avoid tight looping
}

