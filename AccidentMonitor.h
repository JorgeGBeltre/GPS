#ifndef ACCIDENTMONITOR_H
#define ACCIDENTMONITOR_H

#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include "Types.h"
#include "config.h"
#include "DeviceConfig.h"

class AccidentMonitorModule {
public:
    AccidentMonitorModule();
    void begin();
    bool check(AccidentData& outAccident, float currentSpeedGPS);
    
private:
    MPU6050 mpu;
    bool isConnected;
    unsigned long lastAccidentCheck;
    unsigned long lastAccidentEvent;

    int16_t ax, ay, az;
    int16_t oldx, oldy, oldz;
    int vibration, devibrate;
    float acc_history[ACC_FILTER_SIZE];
    int acc_idx;
};

extern AccidentMonitorModule accidentMonitor;

#endif // ACCIDENTMONITOR_H
