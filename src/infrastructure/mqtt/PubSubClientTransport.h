#ifndef INFRA_PUBSUBCLIENTTRANSPORT_H
#define INFRA_PUBSUBCLIENTTRANSPORT_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#include "domain/entities/SystemTypes.h"
#include "domain/interfaces/IMqttTransport.h"

// IMqttTransport backed by PubSubClient over TLS. Formerly MqttClientModule.
class PubSubClientTransport : public IMqttTransport {
public:
  PubSubClientTransport();

  void begin(const String &chipId) override;
  void update() override;
  void publish(const char *topic, const char *message) override;
  void setEventCallback(MqttEventCallback cb) override;
  void setConfigCallback(MqttConfigCallback cb) override;
  bool isConnected() override;

private:
  bool connectMQTT();
  static void mqttCallbackWrapper(char *topic, byte *payload,
                                  unsigned int length);

  // PubSubClient's C-style callback cannot bind to an instance, so we keep a
  // single active instance pointer (replaces the old global `mqttManager`).
  static PubSubClientTransport *s_instance;

  WiFiClientSecure wifiClient;
  PubSubClient mqtt_client;

  String m_chipId;
  bool mqttConnected;
  unsigned long lastReconnectAttempt;
  int mqttFailCount;
  WiFiStatus currentWiFiStatus;
  unsigned long previousMillis;

  MqttEventCallback eventCallback;
  MqttConfigCallback configCallback;
};

#endif // INFRA_PUBSUBCLIENTTRANSPORT_H
