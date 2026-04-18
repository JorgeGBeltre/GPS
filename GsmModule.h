#ifndef GSMMODULE_H
#define GSMMODULE_H

#include "DeviceConfig.h"
#include "Types.h"
#include "config.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

typedef void (*SmsCommandCallback)(const String &numero, const String &comando);

class GsmModule {
public:
  GsmModule(uint8_t rxPin, uint8_t txPin);
  void begin(const String &initialAuthCode);
  void update();
  void sendSmsAlert(const AccidentData &accident, const GPSData &gps,
                    const String &timestamp, const String &chipId);
  void sendResponseSms(const String &numero, const String &mensaje);
  bool isResponsive();
  void checkStatus();
  void setCommandCallback(SmsCommandCallback cb);

  static String currentAuthCode;

private:
  SoftwareSerial sim800;
  SmsCommandCallback commandCallback;
  bool sendATCommand(const String &command, const String &expectedResponse,
                     unsigned long timeoutMs);
  void processReceivedSms();

  int smsAuthFailCount;
  unsigned long smsLockoutStart;

  String smsBuffer;
  bool enSMS;
  String currentNumber;
  String currentDate;
  unsigned long lastByteTime;
};

extern GsmModule gsmManager;

#endif // GSMMODULE_H
