#include "infrastructure/mqtt/PubSubClientTransport.h"

#include <ArduinoJson.h>
#include <bearssl/bearssl_hash.h>
#include <cstdio>
#include <cstring>

#include "config.h"

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
  if (!s_instance)
    return;

  String topicStr = String(topic);
  String message;
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
  if (WiFi.status() != WL_CONNECTED) {
    if (currentWiFiStatus == WIFI_CONNECTED) {
      currentWiFiStatus = WIFI_DISCONNECTED;
      LOG_WARN("WiFi desconectado");
    }
    if (currentWiFiStatus != WIFI_CONNECTING) {
      const auto &cfg = m_config.config();
      if (strlen(cfg.wifi_ssid) > 0) {
        currentWiFiStatus = WIFI_CONNECTING;
        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
      } else if (WiFi.SSID().length() > 0) {
        currentWiFiStatus = WIFI_CONNECTING;
        WiFi.begin();
      }
    }
  } else {
    if (currentWiFiStatus != WIFI_CONNECTED) {
      currentWiFiStatus = WIFI_CONNECTED;
      LOG_INFO("WiFi conectado");
    }

    if (!mqtt_client.connected()) {
      mqttConnected = false;
      unsigned long now = millis();
      unsigned long reconnectDelay =
          min((unsigned long)(MQTT_RECONNECT_INTERVAL *
                              (1UL << (unsigned)mqttFailCount)),
              300000UL);
      reconnectDelay += (unsigned long)random(0, 2000);

      if (now - lastReconnectAttempt > reconnectDelay) {
        lastReconnectAttempt = now;
        if (connectMQTT()) {
          mqttFailCount = 0;
          flushOfflineQueue();
        } else {
          mqttFailCount = min(mqttFailCount + 1, 5);
        }
      }
    } else {
      mqtt_client.loop();
      if (queueCount_ > 0) {
        flushOfflineQueue();
      }
    }
  }
}

bool PubSubClientTransport::connectMQTT() {
  const auto &cfg = m_config.config();
  const char *broker =
      (strlen(cfg.mqtt_server) > 0) ? cfg.mqtt_server : MQTT_SERVER;
  const int port = (cfg.mqtt_port > 0) ? cfg.mqtt_port : MQTT_PORT;
  if (port == 1883) {
    mqtt_client.setClient(plainClient);
  } else {
    mqtt_client.setClient(wifiClient);
  }
  mqtt_client.setServer(broker, port);

  const char *user = (strlen(cfg.mqtt_user) > 0) ? cfg.mqtt_user : API_KEY;
  const char *pass = cfg.mqtt_pass;
  String clientID = m_chipId;
  if (mqtt_client.connect(clientID.c_str(), user, pass)) {
    mqttConnected = true;

    String deviceEventTopic = "event/" + m_chipId;
    mqtt_client.subscribe(deviceEventTopic.c_str());
    String configTopic = String(FPSTR(MQTT_TOPIC_CONFIG)) + "/" + m_chipId;
    mqtt_client.subscribe(configTopic.c_str());
    String unlinkTopic = "devices/" + m_chipId + "/unlink/command";
    mqtt_client.subscribe(unlinkTopic.c_str());

    return true;
  }
  return false;
}

void PubSubClientTransport::enqueueOffline(const char *topic,
                                           const char *message) {
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
  while (queueCount_ > 0 && mqtt_client.connected()) {
    const auto &item = offlineQueue_[queueHead_];
    if (!mqtt_client.publish(item.topic.c_str(), item.payload.c_str())) {
      break;
    }
    queueHead_ = (queueHead_ + 1) % OFFLINE_QUEUE_CAPACITY;
    queueCount_--;
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
