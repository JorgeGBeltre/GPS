#include "AccidentMonitor.h"

AccidentMonitorModule accidentMonitor;

AccidentMonitorModule::AccidentMonitorModule() {
  isConnected = false;
  lastAccidentCheck = 0;
  lastAccidentEvent = 0;
  oldx = 0;
  oldy = 0;
  oldz = 0;
  vibration = 2;
  devibrate = 75;
  acc_idx = 0;
  for (int i = 0; i < ACC_FILTER_SIZE; i++)
    acc_history[i] = 0;
}

void AccidentMonitorModule::begin() {
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

bool AccidentMonitorModule::check(AccidentData &outAccident,
                                  float currentSpeedGPS) {
  if (!isConnected)
    return false;
  if (millis() - lastAccidentCheck < ACCIDENT_CHECK_INTERVAL)
    return false;

  lastAccidentCheck = millis();
  mpu.getAcceleration(&ax, &ay, &az);

  int deltx = ax - oldx;
  int delty = ay - oldy;
  int deltz = az - oldz;

  oldx = ax;
  oldy = ay;
  oldz = az;

  vibration--;
  if (vibration < 0)
    vibration = 0;
  if (vibration > 0)
    return false;

  float rawMag = sqrtf((float)sq(deltx) + (float)sq(delty) + (float)sq(deltz));

  acc_history[acc_idx % ACC_FILTER_SIZE] = rawMag;
  acc_idx++;
  float avgMag = 0.0f;
  for (int i = 0; i < ACC_FILTER_SIZE; i++)
    avgMag += acc_history[i];
  avgMag /= ACC_FILTER_SIZE;

  float STA = rawMag;
  float LTA = avgMag > 0 ? avgMag : 1.0f;

  outAccident.impact_detected = false;
  outAccident.rollover_detected = false;

  bool high_g_impact =
      (rawMag >= (float)DeviceConfig.currentConfig.sensitivity);

  bool anomalia_detectada = (STA / LTA) > 3.0f;

  bool validacion_gps = true;
  if (high_g_impact && currentSpeedGPS > 50.0f) {
    LOG_WARN("Impacto detectado a ALTA VELOCIDAD constante -> Falso positivo "
             "(Adoquines)? Ignorando.");
    validacion_gps = false;
  }

  if (high_g_impact && anomalia_detectada && validacion_gps) {
    outAccident.impact_detected = true;
    outAccident.impact_magnitude = rawMag;
    vibration = devibrate;
    LOG_WARN("Choque VALIDADO por STA/LTA!");
  }

  float axf = ax / 16384.0f;
  float ayf = ay / 16384.0f;
  float azf = az / 16384.0f;

  float roll = atan2f(ayf, sqrtf(axf * axf + azf * azf)) * 180.0f / (float)PI;
  float pitch = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * 180.0f / (float)PI;

  outAccident.roll_angle = roll;
  outAccident.pitch_angle = pitch;

  if (fabsf(roll) > DeviceConfig.currentConfig.rollover_angle ||
      fabsf(pitch) > DeviceConfig.currentConfig.rollover_angle) {
    outAccident.rollover_detected = true;
    Serial.printf("Rollover detectado! roll=%.1f pitch=%.1f\n", roll, pitch);
  }

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
