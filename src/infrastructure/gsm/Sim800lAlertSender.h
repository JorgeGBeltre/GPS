#ifndef INFRA_SIM800LALERTSENDER_H
#define INFRA_SIM800LALERTSENDER_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "domain/interfaces/IConfigRepository.h"
#include "domain/interfaces/IGsmAlertSender.h"

// IGsmAlertSender backed by a SIM800L modem. Formerly GsmModule. The emergency
// phone / sms_enabled flag are read from the injected IConfigRepository instead
// of a global singleton.
class Sim800lAlertSender : public IGsmAlertSender {
public:
  Sim800lAlertSender(uint8_t rxPin, uint8_t txPin, IConfigRepository &config);

  void begin(const String &initialAuthCode) override;
  void update() override;
  void sendAlert(const AccidentEvent &accident, const GpsLocation &gps,
                 const String &timestamp, const String &chipId) override;
  void sendResponse(const String &numero, const String &mensaje) override;
  void setCommandCallback(SmsCommandCallback cb) override;
  bool isResponsive() override;

  void checkStatus();

private:
  bool sendATCommand(const String &command, const String &expectedResponse,
                     unsigned long timeoutMs);
  void processReceivedSms();

  IConfigRepository &m_config;
  SoftwareSerial sim800;
  SmsCommandCallback commandCallback;
  String currentAuthCode;

  int smsAuthFailCount;
  unsigned long smsLockoutStart;

  String smsBuffer;
  bool enSMS;
  String currentNumber;
  String currentDate;
  unsigned long lastByteTime;
};

#endif // INFRA_SIM800LALERTSENDER_H
