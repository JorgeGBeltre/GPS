#ifndef INFRA_PUBSUBCLIENTTRANSPORT_H
#define INFRA_PUBSUBCLIENTTRANSPORT_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <functional>

#include "domain/entities/SystemTypes.h"
#include "domain/interfaces/IConfigRepository.h"
#include "domain/interfaces/IMqttTransport.h"

// IMqttTransport backed by PubSubClient over TLS with an 8-slot offline alert
// ring-buffer (+BUFF style store-and-forward when Wi-Fi/MQTT is disconnected).
class PubSubClientTransport : public IMqttTransport {
public:
  static constexpr size_t OFFLINE_QUEUE_CAPACITY = 8;
  static constexpr size_t MAX_OFFLINE_PAYLOAD_BYTES = 768;

  struct BufferedMqttFrame {
    String topic;
    String payload;
  };

  explicit PubSubClientTransport(IConfigRepository &config);

  void begin(const String &chipId) override;
  void update() override;
  void publish(const char *topic, const char *message) override;
  void setEventCallback(MqttEventCallback cb) override;
  void setConfigCallback(MqttConfigCallback cb) override;
  void setUnlinkCallback(std::function<void()> cb) { unlinkCallback_ = std::move(cb); }
  bool isConnected() override;
  size_t bufferedCount() const { return queueCount_; }

private:
  bool connectMQTT();
  void publishBirthOrHeartbeatStatus();
  void enqueueOffline(const char *topic, const char *message);
  void flushOfflineQueue();
  static void mqttCallbackWrapper(char *topic, byte *payload,
                                  unsigned int length);

  static PubSubClientTransport *s_instance;

  WiFiClient plainClient;
  WiFiClientSecure wifiClient;
  PubSubClient mqtt_client;
  IConfigRepository &m_config;

  String m_chipId;
  bool mqttConnected;
  unsigned long lastReconnectAttempt;
  unsigned long lastWifiAttemptMs_ = 0;
  unsigned long wifiRetryIntervalMs_ = 15000UL;
  unsigned long lastStatusPublishMs_ = 0;
  int mqttFailCount;
  WiFiStatus currentWiFiStatus;
  unsigned long previousMillis;

  BufferedMqttFrame offlineQueue_[OFFLINE_QUEUE_CAPACITY];
  size_t queueHead_ = 0;
  size_t queueCount_ = 0;

  MqttEventCallback eventCallback;
  MqttConfigCallback configCallback;
  std::function<void()> unlinkCallback_;
};

#endif // INFRA_PUBSUBCLIENTTRANSPORT_H
