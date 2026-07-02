#include "application/services/AccidentService.h"

#include <ArduinoJson.h>

#include "config.h"

AccidentService::AccidentService(IAccidentDetector &detector, IGpsProvider &gps,
                                 IGsmAlertSender &gsm, IMqttTransport &mqtt,
                                 IClock &clock, const DeviceInfo &device)
    : m_detector(detector), m_gps(gps), m_gsm(gsm), m_mqtt(mqtt),
      m_clock(clock), m_device(device) {}

void AccidentService::process() {
  AccidentEvent event;

  if (!m_detector.check(event, m_gps.currentSpeedKmph()))
    return;

  const String ts = m_clock.getISOTimestamp();
  Serial.println(">> ¡ALERTA LANZADA AL SISTEMA!");

  // Read the current fix so the alert carries a real location. (The original
  // sketch passed an always-empty GPSData here; wiring the provider in fixes
  // that latent bug — the SMS/Google-Maps link now reflects the true position.)
  GpsLocation location;
  m_gps.readData(location);

  m_gsm.sendAlert(event, location, ts, m_device.chipId);

  StaticJsonDocument<256> doc;
  doc["type"] = "accident";
  doc["impact"] = event.impact_detected;
  doc["rollover"] = event.rollover_detected;
  doc["id"] = m_device.chipId;
  doc["timestamp"] = ts;

  char buffer[256];
  serializeJson(doc, buffer);
  m_mqtt.publish("telemetry/accidents", buffer);
}
