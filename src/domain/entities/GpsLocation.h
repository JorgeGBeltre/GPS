#ifndef DOMAIN_GPSLOCATION_H
#define DOMAIN_GPSLOCATION_H

#include <Arduino.h>

// Domain entity: a decoded GPS fix. Formerly `GPSData` in Types.h.
struct GpsLocation {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0f;
  float speed = 0.0f;
  float course = 0.0f;
  int satellites = 0;
  bool isValid = false;
  String timestamp = "";
};

#endif // DOMAIN_GPSLOCATION_H
