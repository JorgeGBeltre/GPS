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
  void enrichWithLbs(GpsLocation &gps) override;

  void checkStatus();
  void probeAutoApn();
  void pollLbsCellInfo(bool requestCoordinatesIfNoGps);
  bool isJammingDetected() const override { return jammingDetected_; }
  uint16_t modemRecoveries() const override { return modemRecoveries_; }
  bool isSimPresent() const { return simPresent_; }
  bool isApnAutoDetected() const { return apnAutoDetected_; }
  const String &resolvedApn() const { return resolvedApn_; }
  int mcc() const { return mcc_; }
  int mnc() const { return mnc_; }
  uint32_t lac() const { return lac_; }
  uint32_t cellId() const { return cellId_; }

private:
  bool sendATCommand(const String &command, const String &expectedResponse,
                     unsigned long timeoutMs);
  String queryATResponse(const String &command, unsigned long timeoutMs);
  String autoResolveCarrierApn();
  void processReceivedSms();
  void recoverUnresponsiveModem();

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

  int mcc_ = 0;
  int mnc_ = 0;
  uint32_t lac_ = 0;
  uint32_t cellId_ = 0;
  bool hasLbsCoords_ = false;
  double lbsLat_ = 0.0;
  double lbsLon_ = 0.0;
  int lbsAccM_ = 0;
  unsigned long lastLbsPollMs_ = 0;

  String smsBuffer;
  bool enSMS;
  String currentNumber;
  String currentDate;
  unsigned long lastByteTime;
  unsigned long lastHealthCheckMs_ = 0;
  unsigned long lastModemRecoveryMs_ = 0;
  uint16_t modemRecoveries_ = 0;
  uint8_t consecutiveAtFailures_ = 0;
};

#endif // INFRA_SIM800LALERTSENDER_H
