#include "infrastructure/mqtt/PubSubClientTransport.h"

#include "config.h"

PubSubClientTransport *PubSubClientTransport::s_instance = nullptr;

PubSubClientTransport::PubSubClientTransport() : mqtt_client(wifiClient) {
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

  String deviceEventTopic = "event/" + s_instance->m_chipId;
  if (topicStr == deviceEventTopic) {
    if (s_instance->eventCallback)
      s_instance->eventCallback(message);
  } else if (strcmp_P(topicStr.c_str(), MQTT_TOPIC_CONFIG) == 0) {
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

  if (WiFi.SSID().length() > 0) {
    WiFi.begin();
  }

  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt_client.setCallback(mqttCallbackWrapper);
  mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
  mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);

  // ESP8266 BearSSL equivalent of the ESP32 `setCACert(pem)` the original sketch
  // used (that method does not exist on ESP8266, so the original did not compile
  // here). NOTE: root_ca_pem in config.h is an empty placeholder — fill it with
  // your broker's CA chain for the TLS handshake to succeed. For quick testing
  // only, replace both lines with `wifiClient.setInsecure();`.
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

bool PubSubClientTransport::connectMQTT() {
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

void PubSubClientTransport::publish(const char *topic, const char *message) {
  if (!mqttConnected)
    return;
  mqtt_client.publish(topic, message);
}

bool PubSubClientTransport::isConnected() { return mqttConnected; }
