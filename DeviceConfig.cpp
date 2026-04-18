#include "DeviceConfig.h"
#include <LittleFS.h>

DeviceConfigManager DeviceConfig;

DeviceConfigManager::DeviceConfigManager() {
}

void DeviceConfigManager::begin() {
    if (!LittleFS.begin()) {
        LOG_ERROR("Fallo al montar LittleFS");
        return;
    }
    LOG_INFO("LittleFS montado correctamente");
    load();
}

void DeviceConfigManager::load() {
    AccidentConfig defaultConfig;
    if (!LittleFS.exists(configFilePath)) {
        LOG_INFO("Archivo config no existe, creando default...");
        currentConfig = defaultConfig;
        save();
        return;
    }

    File file = LittleFS.open(configFilePath, "r");
    if (!file) {
        LOG_ERROR("Fallo al abrir config para lectura");
        currentConfig = defaultConfig;
        return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        LOG_ERROR("Fallo al parsear config.json, restaurando default");
        currentConfig = defaultConfig;
        save();
        return;
    }

    currentConfig.sensitivity    = doc["sensitivity"] | defaultConfig.sensitivity;
    currentConfig.rollover_angle = doc["rollover_angle"] | defaultConfig.rollover_angle;
    currentConfig.alert_delay    = doc["alert_delay"] | defaultConfig.alert_delay;
    currentConfig.sms_enabled    = doc["sms_enabled"] | defaultConfig.sms_enabled;
    
    if (doc.containsKey("emergency_phone")) {
        strncpy(currentConfig.emergency_phone, doc["emergency_phone"], MAX_PHONE_LENGTH - 1);
        currentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
    } else {
        strncpy(currentConfig.emergency_phone, defaultConfig.emergency_phone, MAX_PHONE_LENGTH - 1);
        currentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
    }

    LOG_INFO("Configuracion cargada desde LittleFS");
}

void DeviceConfigManager::save() {
    File file = LittleFS.open(configFilePath, "w");
    if (!file) {
        LOG_ERROR("Fallo al abrir config para escritura");
        return;
    }

    StaticJsonDocument<512> doc;
    doc["sensitivity"]    = currentConfig.sensitivity;
    doc["rollover_angle"] = currentConfig.rollover_angle;
    doc["alert_delay"]    = currentConfig.alert_delay;
    doc["sms_enabled"]    = currentConfig.sms_enabled;
    doc["emergency_phone"] = currentConfig.emergency_phone;

    if (serializeJson(doc, file) == 0) {
        LOG_ERROR("Fallo al escribir config a file");
    }
    file.close();
    LOG_INFO("Configuracion guardada en LittleFS");
}
