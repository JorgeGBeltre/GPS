#ifndef DEVICECONFIG_H
#define DEVICECONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"

class DeviceConfigManager {
public:
    DeviceConfigManager();
    void begin();
    void load();
    void save();
    
    AccidentConfig currentConfig;
    
private:
    const char* configFilePath = "/config.json";
};

extern DeviceConfigManager DeviceConfig;

#endif // DEVICECONFIG_H
