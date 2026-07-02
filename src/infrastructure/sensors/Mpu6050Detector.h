#ifndef INFRA_MPU6050DETECTOR_H
#define INFRA_MPU6050DETECTOR_H

#include <MPU6050.h>
#include <Wire.h>

#include "application/AccidentAlgorithm.h"
#include "domain/interfaces/IAccidentDetector.h"
#include "domain/interfaces/IConfigRepository.h"

// IAccidentDetector backed by an MPU6050. Formerly AccidentMonitor. Owns the
// hardware (Wire/MPU6050), the check interval and the post-event cooldown;
// delegates the actual impact/rollover math to AccidentAlgorithm.
class Mpu6050Detector : public IAccidentDetector {
public:
  explicit Mpu6050Detector(IConfigRepository &config);

  void begin() override;
  bool check(AccidentEvent &out, float currentSpeedKmph) override;

private:
  IConfigRepository &m_config;
  MPU6050 mpu;
  AccidentAlgorithm algorithm;
  bool isConnected;
  unsigned long lastAccidentCheck;
  unsigned long lastAccidentEvent;
};

#endif // INFRA_MPU6050DETECTOR_H
