#include "infrastructure/persistence/LittleFsConfigRepository.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "config.h"

LittleFsConfigRepository::LittleFsConfigRepository() {}

void LittleFsConfigRepository::begin() {
  if (!LittleFS.begin()) {
    LOG_ERROR("Fallo al montar LittleFS");
    return;
  }
  LOG_INFO("LittleFS montado correctamente");
  load();
}

void LittleFsConfigRepository::load() {
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

  currentConfig.sensitivity = doc["sensitivity"] | defaultConfig.sensitivity;
  currentConfig.rollover_angle =
      doc["rollover_angle"] | defaultConfig.rollover_angle;
  currentConfig.alert_delay = doc["alert_delay"] | defaultConfig.alert_delay;
  currentConfig.sms_enabled = doc["sms_enabled"] | defaultConfig.sms_enabled;
  currentConfig.sled_online = doc["sled_online"] | defaultConfig.sled_online;

  if (doc.containsKey("emergency_phone")) {
    strncpy(currentConfig.emergency_phone, doc["emergency_phone"],
            MAX_PHONE_LENGTH - 1);
    currentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
  } else {
    strncpy(currentConfig.emergency_phone, defaultConfig.emergency_phone,
            MAX_PHONE_LENGTH - 1);
    currentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
  }

  if (doc.containsKey("wifi_ssid")) {
    strncpy(currentConfig.wifi_ssid, doc["wifi_ssid"],
            sizeof(currentConfig.wifi_ssid) - 1);
    currentConfig.wifi_ssid[sizeof(currentConfig.wifi_ssid) - 1] = '\0';
  } else {
    currentConfig.wifi_ssid[0] = '\0';
  }

  if (doc.containsKey("wifi_pass")) {
    strncpy(currentConfig.wifi_pass, doc["wifi_pass"],
            sizeof(currentConfig.wifi_pass) - 1);
    currentConfig.wifi_pass[sizeof(currentConfig.wifi_pass) - 1] = '\0';
  } else {
    currentConfig.wifi_pass[0] = '\0';
  }

  LOG_INFO("Configuracion cargada desde LittleFS");
}

void LittleFsConfigRepository::save() {
  File file = LittleFS.open(configFilePath, "w");
  if (!file) {
    LOG_ERROR("Fallo al abrir config para escritura");
    return;
  }

  StaticJsonDocument<512> doc;
  doc["sensitivity"] = currentConfig.sensitivity;
  doc["rollover_angle"] = currentConfig.rollover_angle;
  doc["alert_delay"] = currentConfig.alert_delay;
  doc["sms_enabled"] = currentConfig.sms_enabled;
  doc["emergency_phone"] = currentConfig.emergency_phone;
  doc["wifi_ssid"] = currentConfig.wifi_ssid;
  doc["wifi_pass"] = currentConfig.wifi_pass;
  doc["sled_online"] = currentConfig.sled_online;

  if (serializeJson(doc, file) == 0) {
    LOG_ERROR("Fallo al escribir config a file");
  }
  file.close();
  LOG_INFO("Configuracion guardada en LittleFS");
}
