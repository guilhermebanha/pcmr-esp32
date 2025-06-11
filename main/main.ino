#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

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
    this->particleSensor.setup();                     // Configure sensor with default settings
    this->particleSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate sensor is running
    this->particleSensor.setPulseAmplitudeGreen(0);   // Turn off Green LED
  }
  void updateBPM() {
    this->irValue = this->particleSensor.getIR();

    if (checkForBeat(this->irValue) == true) {
      // We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      this->beatsPerMinute = 60 / (delta / 1000.0);

      if (this->beatsPerMinute < 255 && this->beatsPerMinute > 20) {
        this->rates[this->rateSpot++] = (byte)this->beatsPerMinute;  // Store this reading in the array
        this->rateSpot %= 4;                                         // Wrap variable

        // Take average of readings
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
    if (this->beatsPerMinute) return this->beatsPerMinute;
  }
  float getAVG() {
    if (this->beatAvg) return this->beatAvg;
  }
  int checkFinger() {
    if (this->irValue < 50000) return 1;
    else return 0;
  }

  bool isValid() {
    if (this->getBPM() >= this->FLAG_MISBPM) return true;
    return false;
  }
};

HeartSensor hs;
void setup() {
  Serial.begin(9600);
  hs.init();
  hs.setFLAG_MISBPM(60);
}

void loop() {
  hs.updateBPM();
  int curr = hs.getBPM();
  if (hs.isValid() && !(hs.checkFinger())) Serial.println(curr);
  delay(20);
}
