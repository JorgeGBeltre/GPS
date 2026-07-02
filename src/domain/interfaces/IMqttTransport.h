#ifndef DOMAIN_IMQTTTRANSPORT_H
#define DOMAIN_IMQTTTRANSPORT_H

#include <Arduino.h>

typedef void (*MqttEventCallback)(const String &message);
typedef void (*MqttConfigCallback)(const String &message);

// Abstraction over an MQTT client. Concrete impl: PubSubClientTransport.
class IMqttTransport {
public:
  virtual ~IMqttTransport() = default;

  virtual void begin(const String &chipId) = 0;
  virtual void update() = 0;
  virtual void publish(const char *topic, const char *message) = 0;
  virtual void setEventCallback(MqttEventCallback cb) = 0;
  virtual void setConfigCallback(MqttConfigCallback cb) = 0;
  virtual bool isConnected() = 0;
};

#endif // DOMAIN_IMQTTTRANSPORT_H
