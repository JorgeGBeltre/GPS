#include "MqttClientModule.h"

MqttClientModule mqttManager;

MqttClientModule::MqttClientModule() : mqtt_client(wifiClient) {
  mqttConnected = false;
  lastReconnectAttempt = 0;
  mqttFailCount = 0;
  currentWiFiStatus = WIFI_DISCONNECTED;
  previousMillis = 0;
  eventCallback = nullptr;
  configCallback = nullptr;
}

void MqttClientModule::mqttCallbackWrapper(char *topic, byte *payload,
                                           unsigned int length) {
  String topicStr = String(topic);
  String message;
  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];

  Serial.printf("Tópico recibido: %s\n", topicStr.c_str());

  String deviceEventTopic = "event/" + mqttManager.m_chipId;
  if (topicStr == deviceEventTopic) {
    if (mqttManager.eventCallback)
      mqttManager.eventCallback(message);
  } else if (strcmp_P(topicStr.c_str(), MQTT_TOPIC_CONFIG) == 0) {
    if (mqttManager.configCallback)
      mqttManager.configCallback(message);
  }
}

void MqttClientModule::setEventCallback(MqttEventCallback cb) {
  eventCallback = cb;
}
void MqttClientModule::setConfigCallback(MqttConfigCallback cb) {
  configCallback = cb;
}

void MqttClientModule::begin(const String &chipId) {
  m_chipId = chipId;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  if (WiFi.SSID().length() > 0) {
    WiFi.begin();
  }

  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt_client.setCallback(mqttCallbackWrapper);
  mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
  mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);
  wifiClient.setCACert(root_ca_pem);
}

void MqttClientModule::update() {
  if (WiFi.status() != WL_CONNECTED) {
    if (currentWiFiStatus == WIFI_CONNECTED) {
      currentWiFiStatus = WIFI_DISCONNECTED;
      LOG_WARN("WiFi desconectado");
    }
    if (currentWiFiStatus != WIFI_CONNECTING) {
      currentWiFiStatus = WIFI_CONNECTING;
      WiFi.begin();
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
        } else {
          mqttFailCount = min(mqttFailCount + 1, 5);
        }
      }
    } else {
      mqtt_client.loop();
    }
  }
}

bool MqttClientModule::connectMQTT() {
  String clientID = "ESP8266_" + m_chipId;
  if (mqtt_client.connect(clientID.c_str(), API_KEY, "")) {
    mqttConnected = true;

    String deviceEventTopic = "event/" + m_chipId;
    mqtt_client.subscribe(deviceEventTopic.c_str());
    char configTopic[16];
    strncpy_P(configTopic, MQTT_TOPIC_CONFIG, sizeof(configTopic));
    mqtt_client.subscribe(configTopic);

    return true;
  }
  return false;
}

void MqttClientModule::publish(const char *topic, const char *message) {
  if (!mqttConnected)
    return;
  mqtt_client.publish(topic, message);
}

bool MqttClientModule::isConnected() { return mqttConnected; }
