#include "GpsModule.h"

GpsModule gpsManager(GPS_RX_PIN, GPS_TX_PIN);

GpsModule::GpsModule(uint8_t rxPin, uint8_t txPin) : gpsSerial(rxPin, txPin) {}

void GpsModule::begin() {
  LOG_INFO("Inicializando GPS en 9600 baud...");
  gpsSerial.begin(9600);
  delay(2000);

  int bytesAvailable = gpsSerial.available();
  Serial.println("Bytes disponibles en puerto GPS: " + String(bytesAvailable));
  if (bytesAvailable > 0) {
    LOG_INFO("GPS DETECTADO");
  } else {
    LOG_WARN("GPS NO ENVIA DATOS - Verifica conexiones");
  }
}

void GpsModule::update() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

void GpsModule::readData(GPSData &data) {
  data.isValid = gps.location.isValid();
  if (data.isValid) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.meters();
    data.speed = gps.speed.kmph();
    data.course = gps.course.deg();
    data.satellites = gps.satellites.value();

  } else {
    static unsigned long lastGPSWarning = 0;
    if (millis() - lastGPSWarning > 30000) {
      Serial.printf("GPS sin fix — Satélites: %d\n", gps.satellites.value());
      lastGPSWarning = millis();
    }
  }
}

int GpsModule::getSatellites() { return gps.satellites.value(); }

bool GpsModule::hasFix() { return gps.location.isValid(); }

void GpsModule::diagnose() {
  Serial.println("\n=== ESTADO GPS ===");
  Serial.println("Datos sin procesar disponibles: " +
                 String(gpsSerial.available()));
  Serial.println("Chars procesados: " + String(gps.charsProcessed()));
  Serial.println("Chars con fallos: " + String(gps.failedChecksum()));
  Serial.println("Satélites: " + String(gps.satellites.value()));
  Serial.println("Ubicación válida: " +
                 String(gps.location.isValid() ? "SI" : "NO"));

  if (gps.location.isValid()) {
    Serial.printf("Lat: %.6f, Lon: %.6f\n", gps.location.lat(),
                  gps.location.lng());
    Serial.printf("Altitud: %.1f m, Velocidad: %.1f km/h\n",
                  gps.altitude.meters(), gps.speed.kmph());
  } else {
    Serial.println("GPS no tiene fix aún");
    Serial.println("Tiempo desde último fix: " + String(gps.location.age()) +
                   "ms");
  }
  Serial.println("================================\n");
}
