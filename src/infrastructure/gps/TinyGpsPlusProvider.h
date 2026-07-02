#ifndef INFRA_TINYGPSPLUSPROVIDER_H
#define INFRA_TINYGPSPLUSPROVIDER_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#include "domain/interfaces/IGpsProvider.h"

// IGpsProvider backed by TinyGPSPlus over SoftwareSerial. Formerly GpsModule.
class TinyGpsPlusProvider : public IGpsProvider {
public:
  TinyGpsPlusProvider(uint8_t rxPin, uint8_t txPin);

  void begin() override;
  void update() override;
  void readData(GpsLocation &out) override;
  float currentSpeedKmph() override;
  int getSatellites() override;
  bool hasFix() override;
  void diagnose() override;

private:
  TinyGPSPlus gps;
  SoftwareSerial gpsSerial;
};

#endif // INFRA_TINYGPSPLUSPROVIDER_H
