#ifndef DOMAIN_SYSTEMTYPES_H
#define DOMAIN_SYSTEMTYPES_H

#include <Arduino.h>

// Cross-cutting system value types. Formerly in Types.h.

enum WiFiStatus { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };

struct OTAContext {
  bool inProgress = false;
  String firmwareVersion;
  int currentPart = 0;
  int totalParts = 0;
  unsigned long startTime = 0;
  size_t receivedSize = 0;
  size_t totalSize = 0;
};

#endif // DOMAIN_SYSTEMTYPES_H
