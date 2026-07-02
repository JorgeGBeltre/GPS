#ifndef DOMAIN_IACCIDENTDETECTOR_H
#define DOMAIN_IACCIDENTDETECTOR_H

#include "domain/entities/AccidentEvent.h"

// Abstraction over accident detection. Concrete impl: Mpu6050Detector.
class IAccidentDetector {
public:
  virtual ~IAccidentDetector() = default;

  virtual void begin() = 0;
  // Returns true when a fresh accident event (past cooldown) is detected.
  virtual bool check(AccidentEvent &out, float currentSpeedKmph) = 0;
};

#endif // DOMAIN_IACCIDENTDETECTOR_H
