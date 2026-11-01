#include "infrastructure/provisioning/QuickPairingPortal.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>

namespace gps::infra {

QuickPairingResult QuickPairingPortal::runQuickPairing(
    IConfigRepository &configRepo, GpioIndicator &indicator, GpioButton &button,
    unsigned long timeoutMs) {
  Serial.println("\n[PAIRING] =========================================================");
  Serial.println("[PAIRING] Iniciando QUICK PAIRING ESP8266 (SmartConfig ESP-Touch)");
  Serial.println("[PAIRING] - Pulsacion larga (5s): Cambiar a SoftAP VESIS-<ChipID>");
  Serial.println("[PAIRING] - Pulsacion corta:      Salir de modo emparejamiento");
  Serial.println("[PAIRING] =========================================================");

  indicator.show(IndicatorStatus::QuickPairing);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.beginSmartConfig();

  // Esperar a que el usuario suelte el botón de la 1ª pulsación larga
  unsigned long releaseWaitStart = millis();
  while (digitalRead(button.getPin()) == LOW &&
         (millis() - releaseWaitStart) < 3000UL) {
    indicator.update();
    ESP.wdtFeed();
    delay(20);
  }
  button.init();

  const unsigned long start = millis();
  while ((millis() - start) < timeoutMs) {
    indicator.update();
    ESP.wdtFeed();

    if (WiFi.smartConfigDone()) {
      String scSsid = WiFi.SSID();
      String scPass = WiFi.psk();
      if (scSsid.length() > 0) {
        auto &cfg = configRepo.config();
        strncpy(cfg.wifi_ssid, scSsid.c_str(), sizeof(cfg.wifi_ssid) - 1);
        cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
        strncpy(cfg.wifi_pass, scPass.c_str(), sizeof(cfg.wifi_pass) - 1);
        cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
        configRepo.save();
        WiFi.stopSmartConfig();
        Serial.printf("[PAIRING] SmartConfig completado! SSID: %s\n",
                      cfg.wifi_ssid);
        return QuickPairingResult::Provisioned;
      }
    }

    ButtonEvent ev = button.update();
    if (ev == ButtonEvent::LongPress) {
      Serial.println("[PAIRING] 2a pulsacion larga (5s) -> Conmutando a SoftAP VESIS-<ChipID>...");
      WiFi.stopSmartConfig();
      return QuickPairingResult::SwitchToSoftAp;
    } else if (ev == ButtonEvent::ShortPress) {
      Serial.println("[PAIRING] Pulsacion corta detectada -> Cancelando emparejamiento.");
      WiFi.stopSmartConfig();
      return QuickPairingResult::CancelledByButton;
    }

    delay(20);
  }

  WiFi.stopSmartConfig();
  return QuickPairingResult::Timeout;
}

bool QuickPairingPortal::runSoftApPortal(const String &chipId,
                                         IConfigRepository &configRepo,
                                         GpioIndicator &indicator,
                                         GpioButton &button,
                                         unsigned long timeoutMs) {
  indicator.show(IndicatorStatus::CompatibleAp);

  String apSsid = "VESIS-" + chipId;
  const char *apPass = "12345678";

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid.c_str(), apPass);

  DNSServer dnsServer;
  dnsServer.start(53, "*", WiFi.softAPIP());

  ESP8266WebServer server(80);
  bool saved = false;

  server.on("/", HTTP_GET, [&]() {
    const auto &cfg = configRepo.config();
    String html = F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<title>Configuracion VESIS</title>"
                    "<style>body{font-family:Arial;max-width:420px;margin:24px auto;padding:0 16px}"
                    "label{display:block;margin-top:10px;font-weight:bold}"
                    "input{width:100%;padding:8px;margin-top:4px;box-sizing:border-box}"
                    "button{margin-top:16px;width:100%;padding:10px;background:#1976d2;color:#fff;border:none;border-radius:4px}</style>"
                    "</head><body><h2>VESIS GPS Config</h2>"
                    "<form method='POST' action='/save'>");
    html += F("<label>WiFi SSID</label><input name='ssid' value='");
    html += String(cfg.wifi_ssid);
    html += F("' required><label>Clave WiFi</label><input name='pass' type='password' value='");
    html += String(cfg.wifi_pass);
    html += F("'><label>Telefono de Emergencia</label><input name='phone' value='");
    html += String(cfg.emergency_phone);
    html += F("'><label>Sensibilidad Impacto</label><input name='sens' type='number' value='");
    html += String(cfg.sensitivity);
    html += F("'><label>Angulo Volcadura (grados)</label><input name='angle' type='number' value='");
    html += String(cfg.rollover_angle);
    html += F("'><button type='submit'>Guardar y Reiniciar</button></form></body></html>");
    server.send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [&]() {
    auto &cfg = configRepo.config();
    if (server.hasArg("ssid") && server.arg("ssid").length() > 0) {
      strncpy(cfg.wifi_ssid, server.arg("ssid").c_str(),
              sizeof(cfg.wifi_ssid) - 1);
      cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
    }
    if (server.hasArg("pass")) {
      strncpy(cfg.wifi_pass, server.arg("pass").c_str(),
              sizeof(cfg.wifi_pass) - 1);
      cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
    }
    if (server.hasArg("phone") && server.arg("phone").length() > 0) {
      strncpy(cfg.emergency_phone, server.arg("phone").c_str(),
              MAX_PHONE_LENGTH - 1);
      cfg.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
    }
    if (server.hasArg("sens") && server.arg("sens").length() > 0) {
      cfg.sensitivity = server.arg("sens").toInt();
    }
    if (server.hasArg("angle") && server.arg("angle").length() > 0) {
      cfg.rollover_angle = server.arg("angle").toInt();
    }
    configRepo.save();
    server.send(200, "text/html",
                "<html><body><h2>Configuracion guardada. Reiniciando...</h2></body></html>");
    saved = true;
  });

  server.onNotFound([&]() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(),
                      true);
    server.send(302, "text/plain", "");
  });

  unsigned long releaseStart = millis();
  while (digitalRead(button.getPin()) == LOW &&
         (millis() - releaseStart) < 3000UL) {
    indicator.update();
    ESP.wdtFeed();
    delay(20);
  }
  button.init();

  server.begin();
  Serial.printf("[PROVISION] SoftAP '%s' activo (IP: %s)\n", apSsid.c_str(),
                WiFi.softAPIP().toString().c_str());

  const unsigned long start = millis();
  while (!saved && (millis() - start) < timeoutMs) {
    dnsServer.processNextRequest();
    server.handleClient();
    indicator.update();
    ESP.wdtFeed();

    if (button.update() == ButtonEvent::ShortPress) {
      Serial.println("[PROVISION] Pulsacion corta detectada -> Saliendo de SoftAP VESIS.");
      break;
    }
    delay(20);
  }

  delay(300);
  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  return saved;
}

void QuickPairingPortal::runPairingWorkflow(
    const String &chipId, IConfigRepository &configRepo,
    GpioIndicator &indicator, GpioButton &button,
    bool isInitialUnprovisionedBoot) {
  QuickPairingResult res = runQuickPairing(
      configRepo, indicator, button,
      isInitialUnprovisionedBoot ? 60000UL : 180000UL);

  if (res == QuickPairingResult::Provisioned) {
    delay(300);
    ESP.restart();
    return;
  }

  if (res == QuickPairingResult::CancelledByButton) {
    return;
  }

  if (res == QuickPairingResult::SwitchToSoftAp || isInitialUnprovisionedBoot) {
    if (runSoftApPortal(chipId, configRepo, indicator, button, 180000UL)) {
      delay(300);
      ESP.restart();
    }
  }
}

} // namespace gps::infra

