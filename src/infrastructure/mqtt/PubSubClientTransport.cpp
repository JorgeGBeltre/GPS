#include "infrastructure/mqtt/PubSubClientTransport.h"

#include <ArduinoJson.h>
#include <bearssl/bearssl_hash.h>
#include <cstdio>
#include <cstring>

#include "config.h"
#include "infrastructure/system/EspSysUtils.h"

namespace {
bool verifyBearSslUnlinkProof(const char *deviceKey, const char *chipId,
                              const char *providedHex) {
  if (!deviceKey || !chipId || !providedHex ||
      deviceKey[0] == '\0' || chipId[0] == '\0' ||
      std::strlen(providedHex) != 32) {
    return false;
  }
  char inputBuf[192];
  const int n = std::snprintf(inputBuf, sizeof(inputBuf), "%s:UNLINK:%s",
                              deviceKey, chipId);
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(inputBuf)) {
    return false;
  }
  uint8_t digest[32] = {0};
  br_sha256_context ctx;
  br_sha256_init(&ctx);
  br_sha256_update(&ctx, inputBuf, static_cast<size_t>(n));
  br_sha256_out(&ctx, digest);

  char expectedHex[33] = {0};
  for (size_t i = 0; i < 16; ++i) {
    std::snprintf(expectedHex + (i * 2), 3, "%02x", digest[i]);
  }
  expectedHex[32] = '\0';

  uint8_t diff = 0;
  for (size_t i = 0; i < 32; ++i) {
    char a = providedHex[i];
    if (a >= 'A' && a <= 'F') {
      a = static_cast<char>(a - 'A' + 'a');
    }
    diff |= static_cast<uint8_t>(a ^ expectedHex[i]);
  }
  return diff == 0;
}
} // namespace

PubSubClientTransport *PubSubClientTransport::s_instance = nullptr;

PubSubClientTransport::PubSubClientTransport(IConfigRepository &config)
    : mqtt_client(wifiClient), m_config(config) {
  mqttConnected = false;
  lastReconnectAttempt = 0;
  mqttFailCount = 0;
  currentWiFiStatus = WIFI_DISCONNECTED;
  previousMillis = 0;
  eventCallback = nullptr;
  configCallback = nullptr;
}

void PubSubClientTransport::mqttCallbackWrapper(char *topic, byte *payload,
                                                unsigned int length) {
  if (!s_instance || !topic || !payload || length > MQTT_BUFFER_SIZE)
    return;

  String topicStr = String(topic);
  String message;
  message.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];

  Serial.printf("Tópico recibido: %s\n", topicStr.c_str());

  String unlinkTopic = "devices/" + s_instance->m_chipId + "/unlink/command";
  if (topicStr == unlinkTopic) {
    StaticJsonDocument<320> doc;
    if (deserializeJson(doc, message) == DeserializationError::Ok) {
      const char *proof = doc["unlink_proof"] | "";
      const auto &cfg = s_instance->m_config.config();
      if (verifyBearSslUnlinkProof(cfg.device_key,
                                   s_instance->m_chipId.c_str(), proof)) {
        Serial.println("[UNLINK] Comando verificado criptograficamente -> Quick Pairing...");
        if (s_instance->unlinkCallback_) {
          s_instance->unlinkCallback_();
        }
      } else {
        Serial.println("[UNLINK] AVISO: unlink_proof invalido rechazado.");
      }
    }
    return;
  }

  String deviceEventTopic = "event/" + s_instance->m_chipId;
  if (topicStr == deviceEventTopic) {
    if (s_instance->eventCallback)
      s_instance->eventCallback(message);
  } else if (topicStr == String(FPSTR(MQTT_TOPIC_CONFIG)) + "/" + s_instance->m_chipId) {
    if (s_instance->configCallback)
      s_instance->configCallback(message);
  }
}

void PubSubClientTransport::setEventCallback(MqttEventCallback cb) {
  eventCallback = cb;
}
void PubSubClientTransport::setConfigCallback(MqttConfigCallback cb) {
  configCallback = cb;
}

void PubSubClientTransport::begin(const String &chipId) {
  s_instance = this;
  m_chipId = chipId;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  const auto &cfg = m_config.config();
  if (strlen(cfg.wifi_ssid) > 0) {
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
  } else if (WiFi.SSID().length() > 0) {
    WiFi.begin();
  }

  const char *broker =
      (strlen(cfg.mqtt_server) > 0) ? cfg.mqtt_server : MQTT_SERVER;
  const int port = (cfg.mqtt_port > 0) ? cfg.mqtt_port : MQTT_PORT;
  if (port == 1883) {
    mqtt_client.setClient(plainClient);
  } else {
    mqtt_client.setClient(wifiClient);
  }
  mqtt_client.setServer(broker, port);
  mqtt_client.setCallback(mqttCallbackWrapper);
  mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
  mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);

  static BearSSL::X509List caCert(root_ca_pem);
  wifiClient.setTrustAnchors(&caCert);
}

void PubSubClientTransport::update() {
  const unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    mqttConnected = false;
    if (currentWiFiStatus == WIFI_CONNECTED) {
      currentWiFiStatus = WIFI_DISCONNECTED;
      wifiRetryIntervalMs_ = 15000UL;
      LOG_WARN("WiFi desconectado");
    }
    if (currentWiFiStatus != WIFI_CONNECTING ||
        (now - lastWifiAttemptMs_ >= wifiRetryIntervalMs_)) {
      const auto &cfg = m_config.config();
      if (strlen(cfg.wifi_ssid) > 0) {
        currentWiFiStatus = WIFI_CONNECTING;
        lastWifiAttemptMs_ = now;
        WiFi.disconnect(false);
        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
        wifiRetryIntervalMs_ =
            (wifiRetryIntervalMs_ < 30000UL) ? (wifiRetryIntervalMs_ * 2UL)
                                             : 60000UL;
      } else if (WiFi.SSID().length() > 0) {
        currentWiFiStatus = WIFI_CONNECTING;
        lastWifiAttemptMs_ = now;
        WiFi.begin();
        wifiRetryIntervalMs_ =
            (wifiRetryIntervalMs_ < 30000UL) ? (wifiRetryIntervalMs_ * 2UL)
                                             : 60000UL;
      }
    }
  } else {
    if (currentWiFiStatus != WIFI_CONNECTED) {
      currentWiFiStatus = WIFI_CONNECTED;
      wifiRetryIntervalMs_ = 15000UL;
      LOG_INFO("WiFi conectado");
    }

    if (!mqtt_client.connected()) {
      mqttConnected = false;
      unsigned long reconnectDelay =
          min((unsigned long)(MQTT_RECONNECT_INTERVAL *
                              (1UL << (unsigned)mqttFailCount)),
              60000UL);
      reconnectDelay += ((now * 37UL) % 1500UL);

      if (now - lastReconnectAttempt > reconnectDelay) {
        lastReconnectAttempt = now;
        if (connectMQTT()) {
          mqttFailCount = 0;
          publishBirthOrHeartbeatStatus();
          lastStatusPublishMs_ = now;
          flushOfflineQueue();
        } else {
          mqttFailCount = min(mqttFailCount + 1, 4);
        }
      }
    } else {
      mqtt_client.loop();
      if (queueCount_ > 0) {
        flushOfflineQueue();
      }
      if (now - lastStatusPublishMs_ >= STATUS_PUBLISH_INTERVAL) {
        lastStatusPublishMs_ = now;
        publishBirthOrHeartbeatStatus();
      }
    }
  }
}

bool PubSubClientTransport::connectMQTT() {
  const auto &cfg = m_config.config();
  const char *broker =
      (strlen(cfg.mqtt_server) > 0) ? cfg.mqtt_server : MQTT_SERVER;
  if (!broker || broker[0] == '\0' || strcmp(broker, MQTT_SERVER) == 0) {
    return false;
  }
  const int port = (cfg.mqtt_port > 0) ? cfg.mqtt_port : MQTT_PORT;
  if (port == 1883) {
    mqtt_client.setClient(plainClient);
  } else {
    mqtt_client.setClient(wifiClient);
  }
  mqtt_client.setServer(broker, port);

  const char *user = (strlen(cfg.mqtt_user) > 0) ? cfg.mqtt_user : API_KEY;
  if (!user || user[0] == '\0' || strcmp(user, "EXAMPLE_API_KEY") == 0) {
    return false;
  }
  const char *pass = cfg.mqtt_pass;
  String clientID = m_chipId;
  String statusWillTopic = "status/" + m_chipId;
  String statusWillPayload = "{\"status\":\"offline\",\"chipId\":\"" + m_chipId + "\"}";
  EspSysUtils::heartbeat();
  if (mqtt_client.connect(clientID.c_str(), user, pass,
                          statusWillTopic.c_str(), 1, true,
                          statusWillPayload.c_str())) {
    EspSysUtils::heartbeat();
    mqttConnected = true;

    String deviceEventTopic = "event/" + m_chipId;
    mqtt_client.subscribe(deviceEventTopic.c_str());
    String configTopic = String(FPSTR(MQTT_TOPIC_CONFIG)) + "/" + m_chipId;
    mqtt_client.subscribe(configTopic.c_str());
    String unlinkTopic = "devices/" + m_chipId + "/unlink/command";
    mqtt_client.subscribe(unlinkTopic.c_str());

    return true;
  }
  EspSysUtils::heartbeat();
  return false;
}

void PubSubClientTransport::publishBirthOrHeartbeatStatus() {
  if (!mqtt_client.connected())
    return;
  char topicBuf[40];
  std::snprintf(topicBuf, sizeof(topicBuf), "status/%s", m_chipId.c_str());
  char payloadBuf[320];
  std::snprintf(
      payloadBuf, sizeof(payloadBuf),
      "{\"status\":\"online\",\"chipId\":\"%s\",\"fw\":\"%s\","
      "\"boot_count\":%lu,\"unexpected_reboots\":%u,\"wdt_offences\":%u,"
      "\"offline_queued\":%u,\"free_heap\":%lu,\"reset_reason\":\"%s\"}",
      m_chipId.c_str(), FIRMWARE_VERSION,
      static_cast<unsigned long>(EspSysUtils::bootCount()),
      static_cast<unsigned>(EspSysUtils::unexpectedReboots()),
      static_cast<unsigned>(EspSysUtils::wdtOffences()),
      static_cast<unsigned>(queueCount_),
      static_cast<unsigned long>(ESP.getFreeHeap()),
      EspSysUtils::resetReason());
  mqtt_client.publish(topicBuf, payloadBuf, true);
}

void PubSubClientTransport::enqueueOffline(const char *topic,
                                           const char *message) {
  if (!topic || !message)
    return;
  const size_t msgLen = std::strlen(message);
  if (msgLen == 0 || msgLen > MAX_OFFLINE_PAYLOAD_BYTES) {
    LOG_WARN("Trama offline descartada por limite de heap (MAX 768B)");
    return;
  }
  size_t slot = (queueHead_ + queueCount_) % OFFLINE_QUEUE_CAPACITY;
  if (queueCount_ == OFFLINE_QUEUE_CAPACITY) {
    queueHead_ = (queueHead_ + 1) % OFFLINE_QUEUE_CAPACITY;
  } else {
    queueCount_++;
  }
  offlineQueue_[slot].topic = String(topic);
  offlineQueue_[slot].payload = String(message);
  LOG_INFO("Alerta almacenada en buffer offline (+BUFF)");
}

void PubSubClientTransport::flushOfflineQueue() {
  size_t drained = 0;
  while (drained < 2 && queueCount_ > 0 && mqtt_client.connected()) {
    const auto &item = offlineQueue_[queueHead_];
    if (!mqtt_client.publish(item.topic.c_str(), item.payload.c_str())) {
      break;
    }
    offlineQueue_[queueHead_].topic = String();
    offlineQueue_[queueHead_].payload = String();
    queueHead_ = (queueHead_ + 1) % OFFLINE_QUEUE_CAPACITY;
    queueCount_--;
    drained++;
    EspSysUtils::heartbeat();
    mqtt_client.loop();
  }
}

void PubSubClientTransport::publish(const char *topic, const char *message) {
  if (!mqttConnected || !mqtt_client.connected()) {
    enqueueOffline(topic, message);
    return;
  }
  if (!mqtt_client.publish(topic, message)) {
    enqueueOffline(topic, message);
  }
}

bool PubSubClientTransport::isConnected() { return mqttConnected; }
