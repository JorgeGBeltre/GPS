#ifndef GPSMODULE_H
#define GPSMODULE_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "Types.h"
#include "config.h"

class GpsModule {
public:
    GpsModule(uint8_t rxPin, uint8_t txPin);
    void begin();
    void update();
    void readData(GPSData& data);
    void diagnose();
    int getSatellites();
    bool hasFix();

    TinyGPSPlus gps;
private:
    SoftwareSerial gpsSerial;
};

extern GpsModule gpsManager;

#endif // GPSMODULE_H
