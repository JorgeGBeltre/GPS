#ifndef INFRA_ESPCLOCK_H
#define INFRA_ESPCLOCK_H

#include <Arduino.h>

#include "domain/interfaces/IClock.h"

// IClock backed by the ESP8266 SNTP client. Formerly SysUtils NTP helpers.
class EspClock : public IClock {
public:
  bool configureNtp() override;
  String getISOTimestamp() override;
};

#endif // INFRA_ESPCLOCK_H
