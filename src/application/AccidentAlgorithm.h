#ifndef APPLICATION_ACCIDENTALGORITHM_H
#define APPLICATION_ACCIDENTALGORITHM_H

#include <cstdint>

// Pure accident-detection algorithm extracted from the original
// AccidentMonitor. Deliberately hardware-free (no Arduino, no Wire, no MPU6050)
// so it can be unit-tested on the host (see test/test_accident_algorithm).
//
// It owns only the signal-processing state: the vibration-damping counter and
// the moving-average (LTA) filter. Sensor I/O, timing (millis) and the
// post-detection cooldown live in the Infrastructure detector.

struct AccidentSample {
  // False when this sample was swallowed by vibration damping (no evaluation
  // was performed). Callers should treat it like "no reading this tick".
  bool evaluated = false;
  bool impact_detected = false;
  bool rollover_detected = false;
  float impact_magnitude = 0.0f;
  float roll_angle = 0.0f;
  float pitch_angle = 0.0f;
};

class AccidentAlgorithm {
public:
  static constexpr int FILTER_SIZE = 5;

  AccidentAlgorithm();

  // Evaluate one raw accelerometer reading.
  //   sensitivity     : raw impact-magnitude threshold (AccidentConfig)
  //   rolloverAngleDeg: roll/pitch threshold in degrees (AccidentConfig)
  //   speedKmph       : current GPS speed, used to reject high-speed false
  //                     positives (rough roads at constant speed)
  AccidentSample evaluate(int16_t ax, int16_t ay, int16_t az, int sensitivity,
                          int rolloverAngleDeg, float speedKmph);

private:
  int16_t oldx, oldy, oldz;
  int vibration;
  int devibrate;
  float acc_history[FILTER_SIZE];
  int acc_idx;
};

#endif // APPLICATION_ACCIDENTALGORITHM_H
