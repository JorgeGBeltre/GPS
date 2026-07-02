#ifndef DOMAIN_IGPSPROVIDER_H
#define DOMAIN_IGPSPROVIDER_H

#include "domain/entities/GpsLocation.h"

// Abstraction over a GPS receiver. Concrete impl: TinyGpsPlusProvider.
class IGpsProvider {
public:
  virtual ~IGpsProvider() = default;

  virtual void begin() = 0;
  virtual void update() = 0;
  virtual void readData(GpsLocation &out) = 0;
  virtual float currentSpeedKmph() = 0;
  virtual int getSatellites() = 0;
  virtual bool hasFix() = 0;
  virtual void diagnose() = 0;
};

#endif // DOMAIN_IGPSPROVIDER_H
