#include "infrastructure/provisioning/QuickPairingPortal.h"
#include "infrastructure/system/EspSysUtils.h"
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <cstring>

namespace gps::infra {

namespace {
bool s_simPresent = false;
bool s_apnAutoDetected = false;
String s_resolvedApn;
} // namespace

void QuickPairingPortal::setCellularState(bool simPresent, bool apnAutoDetected,
                                          const String &resolvedApn) {
  s_simPresent = simPresent;
  s_apnAutoDetected = apnAutoDetected;
  s_resolvedApn = resolvedApn;
}

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

  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  delay(100);
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
                    "<style>body{font-family:Arial;max-width:440px;margin:24px auto;padding:0 16px}"
                    "label{display:block;margin-top:10px;font-weight:bold}"
                    "input,select{width:100%;padding:8px;margin-top:4px;box-sizing:border-box}"
                    "button{margin-top:16px;width:100%;padding:10px;background:#1976d2;color:#fff;border:none;border-radius:4px}</style>"
                    "</head><body><h2>VESIS GPS Config</h2>"
                    "<form method='POST' action='/save'>");
    html += F("<label>WiFi SSID</label><input name='ssid' value='");
    html += String(cfg.wifi_ssid);
    html += F("'><label>Clave WiFi</label><input name='pass' type='password'>"
              "<label>APN Celular (SIM800L)</label><input name='apn' value='");
    html += (std::strlen(cfg.apn) > 0) ? String(cfg.apn) : s_resolvedApn;
    html += F("'><label>Broker MQTT</label><input name='mqtt_server' value='");
    html += String(cfg.mqtt_server);
    html += F("'><label>Puerto MQTT</label><input name='mqtt_port' type='number' value='");
    html += String(cfg.mqtt_port);
    html += F("'><label>Usuario / API Key MQTT</label><input name='mqtt_user' value='");
    html += String(cfg.mqtt_user);
    html += F("'><label>Password MQTT</label><input name='mqtt_pass' type='password'>"
              "<label>Telefono de Emergencia</label><input name='phone' value='");
    html += String(cfg.emergency_phone);
    html += F("'><label>SMS de Emergencia</label><select name='sms_enabled'>");
    html += cfg.sms_enabled ? F("<option value='1' selected>Habilitado</option><option value='0'>Deshabilitado</option>")
                            : F("<option value='1'>Habilitado</option><option value='0' selected>Deshabilitado</option>");
    html += F("</select><label>Sensibilidad Impacto</label><input name='sens' type='number' value='");
    html += String(cfg.sensitivity);
    html += F("'><label>Angulo Volcadura (grados)</label><input name='angle' type='number' value='");
    html += String(cfg.rollover_angle);
    html += F("'><label>Retardo Alerta (ms)</label><input name='alert_delay' type='number' value='");
    html += String(cfg.alert_delay);
    html += F("'><button type='submit'>Guardar y Reiniciar</button></form></body></html>");
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", html);
  });

  auto sendDeviceInfo = [&]() {
    const auto &cfg = configRepo.config();
    const bool wifiConn = (WiFi.status() == WL_CONNECTED);
    const bool hasApn = std::strlen(cfg.apn) > 0 || s_apnAutoDetected;
    const char *credMode =
        (!cfg.account_bound && (s_apnAutoDetected || hasApn || wifiConn))
            ? "adopt_only"
            : (!hasApn ? "apn" : "adopt_only");
    const bool adoptReady = (std::strcmp(credMode, "adopt_only") == 0);
    const String effectiveApn =
        (std::strlen(cfg.apn) > 0) ? String(cfg.apn) : s_resolvedApn;

    StaticJsonDocument<640> doc;
    doc["deviceid"]              = chipId;
    doc["device_id"]             = chipId;
    doc["chipid"]                = chipId;
    doc["apikey"]                = cfg.device_key;
    doc["device_key"]            = cfg.device_key;
    doc["ap"]                    = apSsid;
    doc["device_family"]         = "gps_esp8266";
    doc["device_code"]           = 3;
    doc["device_type"]           = "GPS-ESP8266";
    doc["model"]                 = "VESIS-GPS-ESP8266";
    doc["version"]               = FIRMWARE_VERSION;
    doc["credential_mode"]       = credMode;
    doc["requires_wifi"]         = false;
    doc["requires_apn"]          = (std::strcmp(credMode, "apn") == 0);
    doc["supports_wifi"]         = true;
    doc["supports_apn"]          = true;
    doc["sim_present"]           = s_simPresent;
    doc["apn_auto_detected"]     = s_apnAutoDetected;
    doc["cellular_connected"]    = s_apnAutoDetected;
    doc["adopt_ready"]           = adoptReady;
    doc["apn"]                   = effectiveApn;
    doc["emergency_phone"]       = cfg.emergency_phone;
    doc["account_bound"]         = cfg.account_bound;
    doc["has_wifi_credentials"]  = (std::strlen(cfg.wifi_ssid) > 0);
    doc["accept"]                = "post";
    String out;
    serializeJson(doc, out);
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", out);
  };

  auto handleApPost = [&]() {
    auto &cfg = configRepo.config();
    const String body = server.arg("plain");
    StaticJsonDocument<768> req;
    if (deserializeJson(req, body)) {
      server.sendHeader("Connection", "close");
      server.send(400, "application/json", "{\"error\":400,\"msg\":\"invalid json\"}");
      return;
    }
    JsonVariant payload = req.containsKey("data") ? req["data"].as<JsonVariant>() : req.as<JsonVariant>();

    const char *nextSsid = payload["ssid"] | (payload["wifi_ssid"] | "");
    const char *nextApn  = payload["apn"] | "";
    const bool hasBrokerBundle =
        payload.containsKey("mqtt_server") || payload.containsKey("mqtt_broker") ||
        payload.containsKey("server") || payload.containsKey("serverName") ||
        payload.containsKey("mqtt_user") || payload.containsKey("apikey") ||
        payload.containsKey("bind_token") || payload.containsKey("token");
    const bool canAdopt =
        hasBrokerBundle &&
        (s_apnAutoDetected || std::strlen(cfg.apn) > 0 || std::strlen(cfg.wifi_ssid) > 0);

    if (std::strlen(nextSsid) == 0 && std::strlen(nextApn) == 0 && !canAdopt) {
      server.sendHeader("Connection", "close");
      server.send(400, "application/json", "{\"error\":400,\"msg\":\"missing credentials\"}");
      return;
    }

    if (std::strlen(nextSsid) > 0) {
      strncpy(cfg.wifi_ssid, nextSsid, sizeof(cfg.wifi_ssid) - 1);
      cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
      if (payload.containsKey("password") || payload.containsKey("pass") || payload.containsKey("wifi_pass")) {
        const char *p = payload["password"] | (payload["pass"] | (payload["wifi_pass"] | ""));
        strncpy(cfg.wifi_pass, p, sizeof(cfg.wifi_pass) - 1);
        cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
      }
    }
    if (std::strlen(nextApn) > 0) {
      strncpy(cfg.apn, nextApn, sizeof(cfg.apn) - 1);
      cfg.apn[sizeof(cfg.apn) - 1] = '\0';
    }
    if (payload.containsKey("apn_user")) {
      strncpy(cfg.apn_user, payload["apn_user"] | "", sizeof(cfg.apn_user) - 1);
      cfg.apn_user[sizeof(cfg.apn_user) - 1] = '\0';
    }
    if (payload.containsKey("apn_pass")) {
      strncpy(cfg.apn_pass, payload["apn_pass"] | "", sizeof(cfg.apn_pass) - 1);
      cfg.apn_pass[sizeof(cfg.apn_pass) - 1] = '\0';
    }
    if (payload.containsKey("mqtt_server") || payload.containsKey("mqtt_broker") ||
        payload.containsKey("server") || payload.containsKey("mqttHost") ||
        payload.containsKey("serverName")) {
      const char *m = payload["mqtt_server"] |
                      (payload["mqtt_broker"] |
                       (payload["mqttHost"] |
                        (payload["server"] | (payload["serverName"] | ""))));
      if (std::strlen(m) > 0) {
        strncpy(cfg.mqtt_server, m, sizeof(cfg.mqtt_server) - 1);
        cfg.mqtt_server[sizeof(cfg.mqtt_server) - 1] = '\0';
      }
    }
    if (payload.containsKey("mqtt_port")) {
      cfg.mqtt_port = payload["mqtt_port"].as<int>();
    } else if (payload.containsKey("mqttPort")) {
      cfg.mqtt_port = payload["mqttPort"].as<int>();
    } else if (payload.containsKey("port")) {
      cfg.mqtt_port = payload["port"].as<int>();
    }
    if (payload.containsKey("mqtt_user") || payload.containsKey("apikey") ||
        payload.containsKey("account")) {
      const char *u =
          payload["mqtt_user"] | (payload["apikey"] | (payload["account"] | ""));
      if (std::strlen(u) > 0) {
        strncpy(cfg.mqtt_user, u, sizeof(cfg.mqtt_user) - 1);
        cfg.mqtt_user[sizeof(cfg.mqtt_user) - 1] = '\0';
      }
    }
    if (payload.containsKey("mqtt_pass")) {
      strncpy(cfg.mqtt_pass, payload["mqtt_pass"] | "", sizeof(cfg.mqtt_pass) - 1);
      cfg.mqtt_pass[sizeof(cfg.mqtt_pass) - 1] = '\0';
    }
    if (payload.containsKey("devicekey") || payload.containsKey("device_key")) {
      const char *dk = payload["devicekey"] | (payload["device_key"] | "");
      if (std::strlen(dk) > 0) {
        strncpy(cfg.device_key, dk, sizeof(cfg.device_key) - 1);
        cfg.device_key[sizeof(cfg.device_key) - 1] = '\0';
      }
    }
    if (payload.containsKey("bind_token") || payload.containsKey("token")) {
      const char *bt = payload["bind_token"] | (payload["token"] | "");
      if (std::strlen(bt) > 0) {
        strncpy(cfg.bind_token, bt, sizeof(cfg.bind_token) - 1);
        cfg.bind_token[sizeof(cfg.bind_token) - 1] = '\0';
      }
    }
    if (hasBrokerBundle) {
      cfg.account_bound = true;
    }
    if (payload.containsKey("emergency_phone") || payload.containsKey("phone")) {
      const char *ph = payload["emergency_phone"] | (payload["phone"] | "");
      if (std::strlen(ph) > 0) {
        strncpy(cfg.emergency_phone, ph, MAX_PHONE_LENGTH - 1);
        cfg.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
      }
    }
    if (payload.containsKey("sms_enabled")) {
      cfg.sms_enabled = payload["sms_enabled"].as<bool>();
    }
    if (payload.containsKey("sensitivity")) {
      cfg.sensitivity = payload["sensitivity"].as<int>();
    }
    if (payload.containsKey("rollover_angle")) {
      cfg.rollover_angle = payload["rollover_angle"].as<int>();
    }
    if (payload.containsKey("alert_delay")) {
      cfg.alert_delay = payload["alert_delay"].as<unsigned long>();
    }
    configRepo.save();
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", "{\"error\":0}");
    saved = true;
  };

  server.on("/device", HTTP_GET, sendDeviceInfo);
  server.on("/api/info", HTTP_GET, sendDeviceInfo);
  server.on("/ap", HTTP_POST, handleApPost);
  server.on("/api/wifi", HTTP_POST, handleApPost);

  server.on("/save", HTTP_POST, [&]() {
    auto &cfg = configRepo.config();
    if (server.hasArg("ssid") && server.arg("ssid").length() > 0) {
      strncpy(cfg.wifi_ssid, server.arg("ssid").c_str(),
              sizeof(cfg.wifi_ssid) - 1);
      cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
    }
    if (server.hasArg("pass") && server.arg("pass").length() > 0) {
      strncpy(cfg.wifi_pass, server.arg("pass").c_str(),
              sizeof(cfg.wifi_pass) - 1);
      cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
    }
    if (server.hasArg("apn") && server.arg("apn").length() > 0) {
      strncpy(cfg.apn, server.arg("apn").c_str(), sizeof(cfg.apn) - 1);
      cfg.apn[sizeof(cfg.apn) - 1] = '\0';
    }
    if (server.hasArg("mqtt_server") && server.arg("mqtt_server").length() > 0) {
      strncpy(cfg.mqtt_server, server.arg("mqtt_server").c_str(),
              sizeof(cfg.mqtt_server) - 1);
      cfg.mqtt_server[sizeof(cfg.mqtt_server) - 1] = '\0';
    }
    if (server.hasArg("mqtt_port") && server.arg("mqtt_port").length() > 0) {
      cfg.mqtt_port = server.arg("mqtt_port").toInt();
    }
    if (server.hasArg("mqtt_user") && server.arg("mqtt_user").length() > 0) {
      strncpy(cfg.mqtt_user, server.arg("mqtt_user").c_str(),
              sizeof(cfg.mqtt_user) - 1);
      cfg.mqtt_user[sizeof(cfg.mqtt_user) - 1] = '\0';
    }
    if (server.hasArg("mqtt_pass") && server.arg("mqtt_pass").length() > 0) {
      strncpy(cfg.mqtt_pass, server.arg("mqtt_pass").c_str(),
              sizeof(cfg.mqtt_pass) - 1);
      cfg.mqtt_pass[sizeof(cfg.mqtt_pass) - 1] = '\0';
    }
    if (server.hasArg("phone") && server.arg("phone").length() > 0) {
      strncpy(cfg.emergency_phone, server.arg("phone").c_str(),
              MAX_PHONE_LENGTH - 1);
      cfg.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
    }
    if (server.hasArg("sms_enabled")) {
      cfg.sms_enabled = (server.arg("sms_enabled") == "1");
    }
    if (server.hasArg("sens") && server.arg("sens").length() > 0) {
      cfg.sensitivity = server.arg("sens").toInt();
    }
    if (server.hasArg("angle") && server.arg("angle").length() > 0) {
      cfg.rollover_angle = server.arg("angle").toInt();
    }
    if (server.hasArg("alert_delay") && server.arg("alert_delay").length() > 0) {
      cfg.alert_delay = (unsigned long)server.arg("alert_delay").toInt();
    }
    configRepo.save();
    server.sendHeader("Connection", "close");
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
  WiFi.setAutoReconnect(true);
  return saved;
}

void QuickPairingPortal::runPairingWorkflow(
    const String &chipId, IConfigRepository &configRepo,
    GpioIndicator &indicator, GpioButton &button,
    bool isInitialUnprovisionedBoot) {
  EspSysUtils::setWatchdogPaused(true);
  QuickPairingResult res = runQuickPairing(
      configRepo, indicator, button,
      isInitialUnprovisionedBoot ? 60000UL : 180000UL);

  if (res == QuickPairingResult::Provisioned) {
    delay(300);
    EspSysUtils::markGracefulReboot();
    ESP.restart();
    return;
  }

  if (res == QuickPairingResult::CancelledByButton) {
    EspSysUtils::setWatchdogPaused(false);
    return;
  }

  if (res == QuickPairingResult::SwitchToSoftAp || isInitialUnprovisionedBoot) {
    if (runSoftApPortal(chipId, configRepo, indicator, button, 180000UL)) {
      delay(300);
      EspSysUtils::markGracefulReboot();
      ESP.restart();
      return;
    }
  }
  EspSysUtils::setWatchdogPaused(false);
}

} // namespace gps::infra
