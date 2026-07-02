#ifndef DOMAIN_DEVICEINFO_H
#define DOMAIN_DEVICEINFO_H

#include <Arduino.h>

// Domain entity: identity of this device. Formerly `DeviceInfo` in Types.h.
struct DeviceInfo {
  String chipId;
  String chipType;
  String macAddress;
  String clientID;
  String firmwareVersion;
};

#endif // DOMAIN_DEVICEINFO_H
