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

  const char *bakPath = "/config.bak";
  const char *tmpPath = "/config.tmp";

  StaticJsonDocument<1024> doc;
  bool loadedOk = false;

  auto tryLoadFrom = [&](const char *path) -> bool {
    if (!LittleFS.exists(path))
      return false;
    File f = LittleFS.open(path, "r");
    if (!f || f.size() < 2) {
      if (f)
        f.close();
      return false;
    }
    doc.clear();
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    return !err;
  };

  if (tryLoadFrom(configFilePath)) {
    loadedOk = true;
  } else if (tryLoadFrom(bakPath)) {
    LOG_WARN("config.json ausente/corrupto; recuperando desde /config.bak");
    LittleFS.remove(configFilePath);
    LittleFS.rename(bakPath, configFilePath);
    loadedOk = true;
  } else if (tryLoadFrom(tmpPath)) {
    LOG_WARN("Recuperando configuracion desde /config.tmp");
    LittleFS.remove(configFilePath);
    LittleFS.rename(tmpPath, configFilePath);
    loadedOk = true;
  }

  if (!loadedOk) {
    LOG_INFO("Archivo config no existe o invalido, creando default...");
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

  const char *tmpPath = "/config.tmp";
  const char *bakPath = "/config.bak";

  File file = LittleFS.open(tmpPath, "w");
  if (!file) {
    LOG_ERROR("Fallo al abrir /config.tmp para escritura");
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

  const size_t written = serializeJson(doc, file);
  file.flush();
  file.close();
  if (written <= 2) {
    LOG_ERROR("Fallo al escribir /config.tmp; conservando config previa");
    LittleFS.remove(tmpPath);
    return;
  }

  if (LittleFS.exists(configFilePath)) {
    LittleFS.remove(bakPath);
    LittleFS.rename(configFilePath, bakPath);
  }
  if (!LittleFS.rename(tmpPath, configFilePath)) {
    LOG_ERROR("Fallo en rename atomico de /config.tmp; restaurando .bak");
    if (LittleFS.exists(bakPath)) {
      LittleFS.rename(bakPath, configFilePath);
    }
    return;
  }
  LOG_INFO("Configuracion guardada atomicamente en LittleFS");
}

void LittleFsConfigRepository::factoryReset() {
  char preservedKey[37] = {0};
  if (std::strlen(currentConfig.device_key) > 0) {
    std::strncpy(preservedKey, currentConfig.device_key,
                 sizeof(preservedKey) - 1);
  }
  LittleFS.remove("/config.tmp");
  LittleFS.remove("/config.bak");
  if (LittleFS.exists(configFilePath)) {
    LittleFS.remove(configFilePath);
  }
  currentConfig = AccidentConfig{};
  if (preservedKey[0] != '\0') {
    std::strncpy(currentConfig.device_key, preservedKey,
                 sizeof(currentConfig.device_key) - 1);
    currentConfig.device_key[sizeof(currentConfig.device_key) - 1] = '\0';
  } else {
    populateDefaultDeviceKey(currentConfig.device_key);
  }
  save();
}
