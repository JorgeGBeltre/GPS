// Presentation / Composition Root — GPS Accident Detection System (ESP8266)
//
// Construye las implementaciones de Infraestructura, las inyecta en los
// servicios de Aplicación a través de las interfaces de Dominio y ejecuta
// setup()/loop().

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

#include "config.h"
#include "domain/entities/DeviceInfo.h"

#include "application/services/AccidentService.h"
#include "infrastructure/button/GpioButton.h"
#include "infrastructure/gps/TinyGpsPlusProvider.h"
#include "infrastructure/gsm/Sim800lAlertSender.h"
#include "infrastructure/indicator/GpioIndicator.h"
#include "infrastructure/mqtt/PubSubClientTransport.h"
#include "infrastructure/persistence/LittleFsConfigRepository.h"
#include "infrastructure/provisioning/QuickPairingPortal.h"
#include "infrastructure/sensors/Mpu6050Detector.h"
#include "infrastructure/system/EspClock.h"

// ── Infrastructure (concrete implementations) ────────────────────
LittleFsConfigRepository configRepo;
TinyGpsPlusProvider gpsProvider(GPS_RX_PIN, GPS_TX_PIN);
Sim800lAlertSender gsmSender(GSM_RX_PIN, GSM_TX_PIN, configRepo);
PubSubClientTransport mqttTransport(configRepo);
Mpu6050Detector accidentDetector(configRepo);
EspClock espClock;
gps::infra::GpioIndicator statusLed(wifiLed, true);
gps::infra::GpioButton pairingButton(BUTTON_PIN, BUTTON_DEBOUNCE_MS,
                                     BUTTON_LONG_PRESS_MS);

// ── Shared state ─────────────────────────────────────────────────
DeviceInfo device;

// ── Application services (depend only on Domain interfaces) ───────
AccidentService accidentService(accidentDetector, gpsProvider, gsmSender,
                                mqttTransport, espClock, device);

// ── Presentation callbacks ───────────────────────────────────────
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
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, message)) {
    return;
  }
  auto &cfg = configRepo.config();
  if (doc.containsKey("sensitivity"))
    cfg.sensitivity = doc["sensitivity"].as<int>();
  if (doc.containsKey("rollover_angle"))
    cfg.rollover_angle = doc["rollover_angle"].as<int>();
  if (doc.containsKey("alert_delay"))
    cfg.alert_delay = doc["alert_delay"].as<unsigned long>();
  if (doc.containsKey("sms_enabled"))
    cfg.sms_enabled = doc["sms_enabled"].as<bool>();
  if (doc.containsKey("sled_online")) {
    cfg.sled_online = doc["sled_online"].as<bool>();
    statusLed.setSledOnline(cfg.sled_online);
  }
  if (doc.containsKey("emergency_phone")) {
    strncpy(cfg.emergency_phone, doc["emergency_phone"] | "",
            MAX_PHONE_LENGTH - 1);
    cfg.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
  }
  if (doc.containsKey("apn")) {
    strncpy(cfg.apn, doc["apn"] | "", sizeof(cfg.apn) - 1);
    cfg.apn[sizeof(cfg.apn) - 1] = '\0';
  }
  configRepo.save();
}

static void updateConnectivityIndicator() {
  if (WiFi.status() != WL_CONNECTED) {
    statusLed.show(gps::infra::IndicatorStatus::Disconnected);
  } else if (!mqttTransport.isConnected()) {
    statusLed.show(gps::infra::IndicatorStatus::Connecting);
  } else {
    statusLed.show(gps::infra::IndicatorStatus::Operational);
  }
}

static void executeFactoryUnlinkToQuickPairing() {
  Serial.println("[UNLINK] Dispositivo desvinculado de la cuenta -> Borrando credenciales y entrando en Quick Pairing...");
  configRepo.factoryReset();
  WiFi.disconnect(true);
  ESP.eraseConfig();
  delay(200);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n*** INICIO SISTEMA VESIS (CLEAN ARCHITECTURE) ***");

  char chipHex[16];
  snprintf(chipHex, sizeof(chipHex), "%08X", ESP.getChipId());
  device.chipId = String(chipHex);

  configRepo.begin();
  pairingButton.init();

  const auto &cfg = configRepo.config();
  statusLed.setSledOnline(cfg.sled_online);

  if (strlen(cfg.wifi_ssid) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
  } else if (!cfg.account_bound) {
    Serial.println("[BOOT] Sin credenciales en LittleFS -> Sondeando APN automatico y lanzando Quick Pairing...");
    gsmSender.probeAutoApn();
    gps::infra::QuickPairingPortal::setCellularState(
        gsmSender.isSimPresent(), gsmSender.isApnAutoDetected(),
        gsmSender.resolvedApn());
    gps::infra::QuickPairingPortal::runPairingWorkflow(
        device.chipId, configRepo, statusLed, pairingButton, true);
  }

  accidentDetector.begin();
  gpsProvider.begin();

  String authCode = String(ESP.getChipId() ^ (millis() & 0xFFFF));
  gsmSender.begin(authCode);
  gsmSender.setCommandCallback(onSmsReceived);

  mqttTransport.begin(device.chipId);
  mqttTransport.setEventCallback(onMqttEvent);
  mqttTransport.setConfigCallback(onMqttConfig);
  mqttTransport.setUnlinkCallback(executeFactoryUnlinkToQuickPairing);

  espClock.configureNtp();
  updateConnectivityIndicator();

  Serial.println("*** SETUP COMPLETADO ***\n");
}

void loop() {
  auto btnEv = pairingButton.update();
  if (btnEv == gps::infra::ButtonEvent::LongPress) {
    Serial.println(
        "[BTN] Pulsacion larga (5.0s) detectada -> Iniciando Quick Pairing...");
    gps::infra::QuickPairingPortal::setCellularState(
        gsmSender.isSimPresent(), gsmSender.isApnAutoDetected(),
        gsmSender.resolvedApn());
    gps::infra::QuickPairingPortal::runPairingWorkflow(
        device.chipId, configRepo, statusLed, pairingButton, false);
  }

  updateConnectivityIndicator();
  statusLed.update();

  gpsProvider.update();
  gsmSender.update();
  mqttTransport.update();

  accidentService.process();

  ESP.wdtFeed();
}
