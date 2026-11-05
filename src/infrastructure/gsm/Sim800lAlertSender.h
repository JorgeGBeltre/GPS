#ifndef INFRA_SIM800LALERTSENDER_H
#define INFRA_SIM800LALERTSENDER_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "domain/interfaces/IConfigRepository.h"
#include "domain/interfaces/IGsmAlertSender.h"

// IGsmAlertSender backed by a SIM800L modem with MCC/MNC automatic APN lookup
// and cellular RF jamming detection (GTJDC).
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
  void probeAutoApn();
  bool isJammingDetected() const { return jammingDetected_; }
  bool isSimPresent() const { return simPresent_; }
  bool isApnAutoDetected() const { return apnAutoDetected_; }
  const String &resolvedApn() const { return resolvedApn_; }

private:
  bool sendATCommand(const String &command, const String &expectedResponse,
                     unsigned long timeoutMs);
  String queryATResponse(const String &command, unsigned long timeoutMs);
  String autoResolveCarrierApn();
  void processReceivedSms();

  IConfigRepository &m_config;
  SoftwareSerial sim800;
  SmsCommandCallback commandCallback;
  String currentAuthCode;
  String resolvedApn_;

  int smsAuthFailCount;
  unsigned long smsLockoutStart;
  int lastRssiCsq_ = 0;
  bool jammingDetected_ = false;
  bool simPresent_ = false;
  bool apnAutoDetected_ = false;

  String smsBuffer;
  bool enSMS;
  String currentNumber;
  String currentDate;
  unsigned long lastByteTime;
};

#endif // INFRA_SIM800LALERTSENDER_H
