#ifndef DOMAIN_ACCIDENTEVENT_H
#define DOMAIN_ACCIDENTEVENT_H

// Domain entity: the result of an accident evaluation. Formerly `AccidentData`
// in Types.h. Intentionally free of Arduino/hardware headers so it can be used
// in host (native) unit tests alongside AccidentAlgorithm.
struct AccidentEvent {
  bool impact_detected = false;
  bool rollover_detected = false;
  float impact_magnitude = 0.0f;
  float roll_angle = 0.0f;
  float pitch_angle = 0.0f;
  unsigned long detection_time = 0;
};

#endif // DOMAIN_ACCIDENTEVENT_H
