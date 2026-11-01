#include "infrastructure/mqtt/PubSubClientTransport.h"

#include "config.h"

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
  const auto &cfg = m_config.config();
  const char *broker =
      (strlen(cfg.mqtt_server) > 0) ? cfg.mqtt_server : MQTT_SERVER;
  const int port = (cfg.mqtt_port > 0) ? cfg.mqtt_port : MQTT_PORT;
  mqtt_client.setServer(broker, port);

  const char *user = (strlen(cfg.mqtt_user) > 0) ? cfg.mqtt_user : API_KEY;
  const char *pass = cfg.mqtt_pass;
  String clientID = "ESP8266_" + m_chipId;
  if (mqtt_client.connect(clientID.c_str(), user, pass)) {
    mqttConnected = true;

    String deviceEventTopic = "event/" + m_chipId;
    mqtt_client.subscribe(deviceEventTopic.c_str());
    // Topico de configuracion CON ambito por dispositivo. Antes era el literal "config",
    // compartido por todos los equipos: cualquiera suscrito recibia la configuracion de los
    // demas, y el broker no puede autorizar por propiedad un topico sin identificador.
    String configTopic = String(FPSTR(MQTT_TOPIC_CONFIG)) + "/" + m_chipId;
    mqtt_client.subscribe(configTopic.c_str());

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
