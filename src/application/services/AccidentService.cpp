#include "application/services/AccidentService.h"

#include <ArduinoJson.h>

#include "config.h"
#include "infrastructure/system/EspSysUtils.h"

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

  // Read the current fix so the alert carries a real location (with cellular LBS fallback).
  GpsLocation location;
  m_gps.readData(location);
  m_gsm.enrichWithLbs(location);
  event.tow_detected = location.towDetected;

  m_gsm.sendAlert(event, location, ts, m_device.chipId);

  StaticJsonDocument<640> doc;
  doc["type"] = "accident";
  doc["impact"] = event.impact_detected;
  doc["rollover"] = event.rollover_detected;
  doc["peak_g"] = event.peak_g;
  doc["harsh_braking"] = event.harsh_braking;
  doc["harsh_accel"] = event.harsh_acceleration;
  doc["harsh_cornering"] = event.harsh_cornering;
  doc["tow"] = event.tow_detected;
  doc["jamming"] = m_gsm.isJammingDetected();
  doc["virtual_ignition"] = event.virtual_ignition;
  doc["pos_src"] = location.isValid
                       ? (location.isLbsFallback ? "LBS" : "GPS")
                       : (location.lbsValid ? "LBS_CELL_ID" : "NONE");
  doc["lbs_fallback"] = location.isLbsFallback;
  if (location.isValid) {
    doc["lat"] = location.latitude;
    doc["lon"] = location.longitude;
    doc["speed_kmh"] = location.speed;
    doc["hdop"] = location.hdop;
  }
  if (location.lbsValid) {
    doc["mcc"] = location.mcc;
    doc["mnc"] = location.mnc;
    doc["lac"] = location.lac;
    doc["cell_id"] = location.cellId;
    if (location.lbsAccuracyM > 0) {
      doc["lbs_acc_m"] = location.lbsAccuracyM;
    }
  }
  doc["boot_count"] = EspSysUtils::bootCount();
  doc["unexpected_reboots"] = EspSysUtils::unexpectedReboots();
  doc["wdt_offences"] = EspSysUtils::wdtOffences();
  doc["modem_recoveries"] = m_gsm.modemRecoveries();
  doc["id"] = m_device.chipId;
  doc["timestamp"] = ts;

  char buffer[640];
  serializeJson(doc, buffer);
  m_mqtt.publish("telemetry/accidents", buffer);
}
