#include "infrastructure/sensors/Mpu6050Detector.h"

#include "config.h"

Mpu6050Detector::Mpu6050Detector(IConfigRepository &config)
    : m_config(config), isConnected(false), lastAccidentCheck(0),
      lastAccidentEvent(0) {}

void Mpu6050Detector::begin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();

  isConnected = mpu.testConnection();
  if (!isConnected) {
    LOG_ERROR("MPU6050 no conectado!");
  } else {
    LOG_INFO("MPU6050 listo — calibrando...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    LOG_INFO("MPU6050 calibrado");
  }
}

bool Mpu6050Detector::check(AccidentEvent &outAccident,
                            float currentSpeedGPS) {
  if (!isConnected)
    return false;
  if (millis() - lastAccidentCheck < ACCIDENT_CHECK_INTERVAL)
    return false;

  lastAccidentCheck = millis();

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  const AccidentConfig &cfg = m_config.config();
  AccidentSample sample = algorithm.evaluate(ax, ay, az, cfg.sensitivity,
                                             cfg.rollover_angle, currentSpeedGPS);

  if (!sample.evaluated)
    return false; // swallowed by vibration damping

  outAccident.impact_detected = sample.impact_detected;
  outAccident.rollover_detected = sample.rollover_detected;
  outAccident.impact_magnitude = sample.impact_magnitude;
  outAccident.roll_angle = sample.roll_angle;
  outAccident.pitch_angle = sample.pitch_angle;

  // Note: high-speed false-positive rejection now lives inside AccidentAlgorithm.
  if (sample.impact_detected)
    LOG_WARN("Choque VALIDADO por STA/LTA!");
  if (sample.rollover_detected)
    Serial.printf("Rollover detectado! roll=%.1f pitch=%.1f\n",
                  sample.roll_angle, sample.pitch_angle);

  if (outAccident.impact_detected || outAccident.rollover_detected) {
    if (millis() - lastAccidentEvent > ACCIDENT_COOLDOWN) {
      lastAccidentEvent = millis();
      outAccident.detection_time = millis();
      return true;
    } else {
      LOG_WARN("Accidente detectado dentro del cooldown, ignorando");
    }
  }
  return false;
}
