#ifndef MQTTCLIENTMODULE_H
#define MQTTCLIENTMODULE_H

#include "Types.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

typedef void (*MqttEventCallback)(const String &message);
typedef void (*MqttConfigCallback)(const String &message);

class MqttClientModule {
public:
  MqttClientModule();
  void begin(const String &chipId);
  void update();
  void publish(const char *topic, const char *message);

  void setEventCallback(MqttEventCallback cb);
  void setConfigCallback(MqttConfigCallback cb);

  bool isConnected();
  WiFiClientSecure wifiClient;
  PubSubClient mqtt_client;

private:
  void reconnectWiFi();
  bool connectMQTT();
  static void mqttCallbackWrapper(char *topic, byte *payload,
                                  unsigned int length);

  String m_chipId;
  bool mqttConnected;
  unsigned long lastReconnectAttempt;
  int mqttFailCount;
  WiFiStatus currentWiFiStatus;
  unsigned long previousMillis;

  MqttEventCallback eventCallback;
  MqttConfigCallback configCallback;
};

extern MqttClientModule mqttManager;

#endif // MQTTCLIENTMODULE_H
