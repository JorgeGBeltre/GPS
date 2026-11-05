#include "infrastructure/persistence/LittleFsConfigRepository.h"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <cstdio>
#include <cstring>

#include "config.h"

namespace {
void populateDefaultDeviceKey(char outKey[37]) {
  const uint32_t chip = ESP.getChipId();
  std::snprintf(outKey, 37, "%08X-0000-4000-8000-%012llX",
                chip,
                static_cast<unsigned long long>(chip) ^ 0x5A5A5A5A5A5AULL);
}

bool hasValidBrokerInConfig(const AccidentConfig &cfg) {
  return std::strlen(cfg.mqtt_server) > 0 &&
         std::strcmp(cfg.mqtt_server, MQTT_SERVER) != 0 &&
         std::strlen(cfg.mqtt_user) > 0 &&
         std::strcmp(cfg.mqtt_user, API_KEY) != 0;
}
} // namespace

LittleFsConfigRepository::LittleFsConfigRepository() {}

void LittleFsConfigRepository::begin() {
  if (!LittleFS.begin()) {
    LOG_ERROR("Fallo al montar LittleFS");
    if (std::strlen(currentConfig.device_key) == 0) {
      populateDefaultDeviceKey(currentConfig.device_key);
    }
    return;
  }
  LOG_INFO("LittleFS montado correctamente");
  load();
}

void LittleFsConfigRepository::load() {
  AccidentConfig defaultConfig;
  populateDefaultDeviceKey(defaultConfig.device_key);

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

  StaticJsonDocument<1024> doc;
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
  currentConfig.mqtt_port   = doc["mqtt_port"]   | defaultConfig.mqtt_port;

  if (doc.containsKey("emergency_phone")) {
    strncpy(currentConfig.emergency_phone, doc["emergency_phone"],
            MAX_PHONE_LENGTH - 1);
    currentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
  } else {
    strncpy(currentConfig.emergency_phone, defaultConfig.emergency_phone,
            MAX_PHONE_LENGTH - 1);
    currentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
  }

  strncpy(currentConfig.wifi_ssid, doc["wifi_ssid"] | "",
          sizeof(currentConfig.wifi_ssid) - 1);
  currentConfig.wifi_ssid[sizeof(currentConfig.wifi_ssid) - 1] = '\0';

  strncpy(currentConfig.wifi_pass, doc["wifi_pass"] | "",
          sizeof(currentConfig.wifi_pass) - 1);
  currentConfig.wifi_pass[sizeof(currentConfig.wifi_pass) - 1] = '\0';

  strncpy(currentConfig.apn, doc["apn"] | defaultConfig.apn,
          sizeof(currentConfig.apn) - 1);
  currentConfig.apn[sizeof(currentConfig.apn) - 1] = '\0';

  strncpy(currentConfig.apn_user, doc["apn_user"] | "",
          sizeof(currentConfig.apn_user) - 1);
  currentConfig.apn_user[sizeof(currentConfig.apn_user) - 1] = '\0';

  strncpy(currentConfig.apn_pass, doc["apn_pass"] | "",
          sizeof(currentConfig.apn_pass) - 1);
  currentConfig.apn_pass[sizeof(currentConfig.apn_pass) - 1] = '\0';

  strncpy(currentConfig.mqtt_server,
          doc["mqtt_server"] | defaultConfig.mqtt_server,
          sizeof(currentConfig.mqtt_server) - 1);
  currentConfig.mqtt_server[sizeof(currentConfig.mqtt_server) - 1] = '\0';

  strncpy(currentConfig.mqtt_user, doc["mqtt_user"] | defaultConfig.mqtt_user,
          sizeof(currentConfig.mqtt_user) - 1);
  currentConfig.mqtt_user[sizeof(currentConfig.mqtt_user) - 1] = '\0';

  strncpy(currentConfig.mqtt_pass, doc["mqtt_pass"] | defaultConfig.mqtt_pass,
          sizeof(currentConfig.mqtt_pass) - 1);
  currentConfig.mqtt_pass[sizeof(currentConfig.mqtt_pass) - 1] = '\0';

  strncpy(currentConfig.device_key, doc["device_key"] | defaultConfig.device_key,
          sizeof(currentConfig.device_key) - 1);
  currentConfig.device_key[sizeof(currentConfig.device_key) - 1] = '\0';
  if (std::strlen(currentConfig.device_key) == 0) {
    populateDefaultDeviceKey(currentConfig.device_key);
  }

  strncpy(currentConfig.bind_token, doc["bind_token"] | "",
          sizeof(currentConfig.bind_token) - 1);
  currentConfig.bind_token[sizeof(currentConfig.bind_token) - 1] = '\0';

  if (doc.containsKey("account_bound")) {
    currentConfig.account_bound = doc["account_bound"].as<bool>();
  } else {
    currentConfig.account_bound = hasValidBrokerInConfig(currentConfig);
  }

  LOG_INFO("Configuracion cargada desde LittleFS");
}

void LittleFsConfigRepository::save() {
  if (std::strlen(currentConfig.device_key) == 0) {
    populateDefaultDeviceKey(currentConfig.device_key);
  }
  if (hasValidBrokerInConfig(currentConfig) || std::strlen(currentConfig.bind_token) > 0) {
    currentConfig.account_bound = true;
  }

  File file = LittleFS.open(configFilePath, "w");
  if (!file) {
    LOG_ERROR("Fallo al abrir config para escritura");
    return;
  }

  StaticJsonDocument<1024> doc;
  doc["sensitivity"] = currentConfig.sensitivity;
  doc["rollover_angle"] = currentConfig.rollover_angle;
  doc["alert_delay"] = currentConfig.alert_delay;
  doc["sms_enabled"] = currentConfig.sms_enabled;
  doc["emergency_phone"] = currentConfig.emergency_phone;
  doc["wifi_ssid"] = currentConfig.wifi_ssid;
  doc["wifi_pass"] = currentConfig.wifi_pass;
  doc["apn"] = currentConfig.apn;
  doc["apn_user"] = currentConfig.apn_user;
  doc["apn_pass"] = currentConfig.apn_pass;
  doc["mqtt_server"] = currentConfig.mqtt_server;
  doc["mqtt_port"] = currentConfig.mqtt_port;
  doc["mqtt_user"] = currentConfig.mqtt_user;
  doc["mqtt_pass"] = currentConfig.mqtt_pass;
  doc["device_key"] = currentConfig.device_key;
  doc["bind_token"] = currentConfig.bind_token;
  doc["sled_online"] = currentConfig.sled_online;
  doc["account_bound"] = currentConfig.account_bound;

  if (serializeJson(doc, file) == 0) {
    LOG_ERROR("Fallo al escribir config a file");
  }
  file.close();
  LOG_INFO("Configuracion guardada en LittleFS");
}

void LittleFsConfigRepository::factoryReset() {
  if (LittleFS.exists(configFilePath)) {
    LittleFS.remove(configFilePath);
  }
  currentConfig = AccidentConfig{};
  populateDefaultDeviceKey(currentConfig.device_key);
  save();
}
