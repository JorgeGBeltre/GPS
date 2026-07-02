// Presentation / Composition Root — GPS Accident Detection System (ESP8266)
//
// This is the only place that knows about concrete implementations. It builds
// the Infrastructure objects, injects them into the Application services
// through their Domain interfaces, and runs the Arduino setup()/loop().
// Formerly GPS.ino.

#include <Arduino.h>

#include "config.h"
#include "domain/entities/DeviceInfo.h"

#include "application/services/AccidentService.h"
#include "infrastructure/gps/TinyGpsPlusProvider.h"
#include "infrastructure/gsm/Sim800lAlertSender.h"
#include "infrastructure/mqtt/PubSubClientTransport.h"
#include "infrastructure/persistence/LittleFsConfigRepository.h"
#include "infrastructure/sensors/Mpu6050Detector.h"
#include "infrastructure/system/EspClock.h"

// ── Infrastructure (concrete implementations) ────────────────────
LittleFsConfigRepository configRepo;
TinyGpsPlusProvider gpsProvider(GPS_RX_PIN, GPS_TX_PIN);
Sim800lAlertSender gsmSender(GSM_RX_PIN, GSM_TX_PIN, configRepo);
PubSubClientTransport mqttTransport;
Mpu6050Detector accidentDetector(configRepo);
EspClock espClock;

// ── Shared state ─────────────────────────────────────────────────
DeviceInfo device;

// ── Application services (depend only on Domain interfaces) ───────
AccidentService accidentService(accidentDetector, gpsProvider, gsmSender,
                                mqttTransport, espClock, device);

// ── Presentation callbacks ───────────────────────────────────────
// Kept as free functions because the GSM/MQTT callbacks are C-style function
// pointers. They dispatch onto the injected objects above.
void onSmsReceived(const String &numero, const String &comando) {
  Serial.println(">>> Procesando SMS desde: " + numero);
  if (comando == "STATUS") {
    gsmSender.sendResponse(numero, "GPS STATUS OK.");
  }
}

void onMqttEvent(const String &message) {
  Serial.println("Evento MQTT recibido: " + message);
}

void onMqttConfig(const String &message) {
  Serial.println("Configuracion MQTT push: " + message);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n*** INICIO SISTEMA VESIS (GRADO INDUSTRIAL) ***");

  device.chipId = String(ESP.getChipId());

  configRepo.begin();

  accidentDetector.begin();
  gpsProvider.begin();

  String authCode = String(ESP.getChipId() ^ (millis() & 0xFFFF));
  gsmSender.begin(authCode);
  gsmSender.setCommandCallback(onSmsReceived);

  mqttTransport.begin(device.chipId);
  mqttTransport.setEventCallback(onMqttEvent);
  mqttTransport.setConfigCallback(onMqttConfig);

  espClock.configureNtp();

  Serial.println("*** SETUP COMPLETADO ***\n");
}

void loop() {
  gpsProvider.update();
  gsmSender.update();
  mqttTransport.update();

  accidentService.process();

  ESP.wdtFeed();
}
