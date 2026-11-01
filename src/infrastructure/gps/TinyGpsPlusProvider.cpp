#include "infrastructure/gps/TinyGpsPlusProvider.h"

#include "config.h"

TinyGpsPlusProvider::TinyGpsPlusProvider(uint8_t rxPin, uint8_t txPin)
    : gpsSerial(rxPin, txPin) {}

void TinyGpsPlusProvider::begin() {
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

void TinyGpsPlusProvider::update() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

void TinyGpsPlusProvider::readData(GpsLocation &data) {
  data.isValid = gps.location.isValid();
  if (data.isValid) {
    double rawLat = gps.location.lat();
    double rawLon = gps.location.lng();
    float speedKmh = gps.speed.kmph();

    data.altitude = gps.altitude.meters();
    data.speed = speedKmh;
    data.course = gps.course.deg();
    data.hdop = gps.hdop.isValid() ? static_cast<float>(gps.hdop.hdop()) : 99.9f;
    data.satellites = gps.satellites.value();
    data.driftFiltered = false;
    data.towDetected = false;

    if (!hasAnchor_) {
      hasAnchor_ = true;
      anchorLat_ = rawLat;
      anchorLon_ = rawLon;
    }

    double distFromAnchorM = TinyGPSPlus::distanceBetween(
        anchorLat_, anchorLon_, rawLat, rawLon);

    // Static GPS Drift Filter (GTRDF): pin stationary coordinates within 30 m
    if (speedKmh < 3.0f && distFromAnchorM <= 30.0) {
      data.latitude = anchorLat_;
      data.longitude = anchorLon_;
      data.driftFiltered = true;
    } else {
      data.latitude = rawLat;
      data.longitude = rawLon;
      // Parked Tow Detection (GTTOW): displacement > 100 m while low/parked speed
      if (speedKmh < 8.0f && distFromAnchorM >= 100.0) {
        data.towDetected = true;
      }
      if (speedKmh >= 5.0f || distFromAnchorM > 30.0) {
        anchorLat_ = rawLat;
        anchorLon_ = rawLon;
      }
    }
  } else {
    static unsigned long lastGPSWarning = 0;
    if (millis() - lastGPSWarning > 30000) {
      Serial.printf("GPS sin fix — Satélites: %d\n", gps.satellites.value());
      lastGPSWarning = millis();
    }
  }
}

float TinyGpsPlusProvider::currentSpeedKmph() { return gps.speed.kmph(); }

int TinyGpsPlusProvider::getSatellites() { return gps.satellites.value(); }

bool TinyGpsPlusProvider::hasFix() { return gps.location.isValid(); }

void TinyGpsPlusProvider::diagnose() {
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
