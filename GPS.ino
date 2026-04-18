#include "AccidentMonitor.h"
#include "DeviceConfig.h"
#include "GpsModule.h"
#include "GsmModule.h"
#include "MqttClientModule.h"
#include "SysUtils.h"
#include "Types.h"
#include "config.h"

DeviceInfo device;
GPSData currentGPSData;
AccidentData currentAccidentData;

void onSmsReceived(const String &numero, const String &comando) {

  Serial.println(">>> Procesando SMS desde: " + numero);
  if (comando == "STATUS") {
    gsmManager.sendResponseSms(numero, "GPS STATUS OK.");
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

  DeviceConfig.begin();

  accidentMonitor.begin();
  gpsManager.begin();

  String authCode = String(ESP.getChipId() ^ (millis() & 0xFFFF));
  gsmManager.begin(authCode);
  gsmManager.setCommandCallback(onSmsReceived);

  mqttManager.begin(device.chipId);
  mqttManager.setEventCallback(onMqttEvent);
  mqttManager.setConfigCallback(onMqttConfig);

  SysUtils::configurarTiempoNTP();

  Serial.println("*** SETUP COMPLETADO ***\n");
}

void loop() {

  gpsManager.update();
  gsmManager.update();
  mqttManager.update();

  if (accidentMonitor.check(currentAccidentData, gpsManager.gps.speed.kmph())) {
    String ts = SysUtils::getISOTimestamp();
    Serial.println(">> ¡ALERTA LANZADA AL SISTEMA!");

    gsmManager.sendSmsAlert(currentAccidentData, currentGPSData, ts,
                            device.chipId);

    StaticJsonDocument<256> doc;
    doc["type"] = "accident";
    doc["impact"] = currentAccidentData.impact_detected;
    doc["rollover"] = currentAccidentData.rollover_detected;
    doc["id"] = device.chipId;
    doc["timestamp"] = ts;

    char buffer[256];
    serializeJson(doc, buffer);
    mqttManager.publish("telemetry/accidents", buffer);
  }

  ESP.wdtFeed();
}
