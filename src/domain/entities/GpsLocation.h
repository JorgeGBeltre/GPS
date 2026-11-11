#ifndef DOMAIN_GPSLOCATION_H
#define DOMAIN_GPSLOCATION_H

#include <Arduino.h>
#include <stdint.h>

// Domain entity: a decoded GPS or cellular LBS fix. Formerly `GPSData` in Types.h.
struct GpsLocation {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0f;
  float speed = 0.0f;
  float course = 0.0f;
  float hdop = 99.9f;
  int satellites = 0;
  bool isValid = false;
  bool driftFiltered = false;
  bool towDetected = false;
  String timestamp = "";

  // LBS (Location-Based Service / Cell ID positioning)
  bool lbsValid = false;
  bool isLbsFallback = false;
  int mcc = 0;
  int mnc = 0;
  uint32_t lac = 0;
  uint32_t cellId = 0;
  int lbsAccuracyM = 0;
};

#endif // DOMAIN_GPSLOCATION_H
