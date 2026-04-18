#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

struct DeviceInfo {
  String chipId;
  String chipType;
  String macAddress;
  String clientID;
  String firmwareVersion;
};

struct GPSData {
  double latitude  = 0.0;
  double longitude = 0.0;
  float  altitude  = 0.0;
  float  speed     = 0.0;
  float  course    = 0.0;
  int    satellites = 0;
  bool   isValid   = false;
  String timestamp = "";
};

struct AccidentData {
  bool  impact_detected  = false;
  bool  rollover_detected = false;
  float impact_magnitude = 0.0f;
  float roll_angle       = 0.0f;
  float pitch_angle      = 0.0f;
  unsigned long detection_time = 0;
};

struct OTAContext {
    bool   inProgress   = false;
    String firmwareVersion;
    int    currentPart  = 0;
    int    totalParts   = 0;
    unsigned long startTime  = 0;
    size_t receivedSize = 0;
    size_t totalSize    = 0;
};

enum WiFiStatus { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };

#endif // TYPES_H
