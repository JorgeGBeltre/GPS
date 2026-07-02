#ifndef DOMAIN_IGSMALERTSENDER_H
#define DOMAIN_IGSMALERTSENDER_H

#include <Arduino.h>

#include "domain/entities/AccidentEvent.h"
#include "domain/entities/GpsLocation.h"

// Callback invoked when an authorized SMS command is received.
typedef void (*SmsCommandCallback)(const String &numero, const String &comando);

// Abstraction over a GSM modem used for alerting. Concrete impl:
// Sim800lAlertSender.
class IGsmAlertSender {
public:
  virtual ~IGsmAlertSender() = default;

  virtual void begin(const String &initialAuthCode) = 0;
  virtual void update() = 0;
  virtual void sendAlert(const AccidentEvent &accident, const GpsLocation &gps,
                         const String &timestamp, const String &chipId) = 0;
  virtual void sendResponse(const String &numero, const String &mensaje) = 0;
  virtual void setCommandCallback(SmsCommandCallback cb) = 0;
  virtual bool isResponsive() = 0;
};

#endif // DOMAIN_IGSMALERTSENDER_H
