#include "infrastructure/gsm/Sim800lAlertSender.h"

#include "config.h"

namespace {
struct CarrierApnEntry {
  const char *mccMncPrefix;
  const char *apn;
};

constexpr CarrierApnEntry kCarrierApnTable[] = {
    {"37001", "internet.orange.com.do"},
    {"37002", "internet.claro.com.do"},
    {"37003", "avircom.net"},
    {"37004", "data.viva.com.do"},
    {"334020", "internet.itelcel.com"},
    {"33403", "internet.movistar.mx"},
    {"732101", "internet.comcel.com.co"},
    {"732103", "web.colombiamovil.com.co"},
    {"732123", "internet.movistar.com.co"},
    {"71606", "movistar.pe"},
    {"71610", "claro.pe"},
    {"73001", "bam.entelpcs.cl"},
    {"73002", "wap.tmovil.cl"},
    {"72207", "wap.gprs.unifon.com.ar"},
    {"722310", "igprs.claro.com.ar"},
    {"310260", "fast.t-mobile.com"},
    {"310410", "broadband"},
    {"21401", "airtelnet.es"},
    {"21407", "movistar.es"},
};
} // namespace

Sim800lAlertSender::Sim800lAlertSender(uint8_t rxPin, uint8_t txPin,
                                       IConfigRepository &config)
    : m_config(config), sim800(rxPin, txPin) {
  commandCallback = nullptr;
  smsAuthFailCount = 0;
  smsLockoutStart = 0;
  enSMS = false;
  lastByteTime = 0;
}

String Sim800lAlertSender::queryATResponse(const String &command,
                                           unsigned long timeoutMs) {
  while (sim800.available())
    sim800.read();
  sim800.println(command);
  unsigned long startTime = millis();
  String response = "";
  while (millis() - startTime < timeoutMs) {
    while (sim800.available()) {
      response += static_cast<char>(sim800.read());
    }
    if (response.indexOf("OK") >= 0 || response.indexOf("ERROR") >= 0)
      break;
    delay(10);
  }
  response.trim();
  return response;
}

String Sim800lAlertSender::autoResolveCarrierApn() {
  String imsiResp = queryATResponse("AT+CIMI", 1500);
  for (size_t i = 0; i < sizeof(kCarrierApnTable) / sizeof(kCarrierApnTable[0]);
       ++i) {
    if (imsiResp.indexOf(kCarrierApnTable[i].mccMncPrefix) >= 0) {
      return String(kCarrierApnTable[i].apn);
    }
  }
  return "";
}

void Sim800lAlertSender::begin(const String &initialAuthCode) {
  currentAuthCode = initialAuthCode;

  LOG_INFO("Inicializando módulo GSM...");
  sim800.begin(9600);
  delay(3000);
  if (!isResponsive()) {
    LOG_ERROR("No se puede comunicar con módulo GSM");
    return;
  }
  sendATCommand("ATE0", "OK", 2000);
  sendATCommand("AT+CMGF=1", "OK", 2000);
  sendATCommand("AT+CNMI=2,2,0,0,0", "OK", 2000);

  resolvedApn_ = String(m_config.config().apn);
  if (resolvedApn_.length() == 0) {
    resolvedApn_ = autoResolveCarrierApn();
  }
  if (resolvedApn_.length() > 0) {
    String cgdcont = String("AT+CGDCONT=1,\"IP\",\"") + resolvedApn_ + "\"";
    sendATCommand(cgdcont, "OK", 2000);
  }
  LOG_INFO("Módulo GSM inicializado para recibir SMS");
}

void Sim800lAlertSender::setCommandCallback(SmsCommandCallback cb) {
  commandCallback = cb;
}

void Sim800lAlertSender::update() { processReceivedSms(); }

void Sim800lAlertSender::processReceivedSms() {
  while (sim800.available()) {
    char c = sim800.read();
    lastByteTime = millis();

    if (smsBuffer.length() >= SMS_BUFFER_MAX) {
      LOG_WARN("SMS buffer overflow, descartando");
      smsBuffer = "";
      enSMS = false;
      currentNumber = "";
      currentDate = "";
      return;
    }

    smsBuffer += c;

    if (smsBuffer.indexOf("+CMT:") >= 0 && !enSMS) {
      enSMS = true;
      int inicioNum = smsBuffer.indexOf("\"", smsBuffer.indexOf("+CMT:")) + 1;
      int finNum = smsBuffer.indexOf("\"", inicioNum);
      if (inicioNum > 0 && finNum > inicioNum)
        currentNumber = smsBuffer.substring(inicioNum, finNum);

      int inicioFecha = smsBuffer.indexOf("\"", finNum + 1) + 1;
      int finFecha = smsBuffer.indexOf("\"", inicioFecha);
      if (inicioFecha > 0 && finFecha > inicioFecha)
        currentDate = smsBuffer.substring(inicioFecha, finFecha);
    }

    if (enSMS && smsBuffer.indexOf("\r\n\r\n") >= 0) {
      int inicioMensaje = smsBuffer.indexOf("\r\n\r\n") + 4;
      String mensaje = smsBuffer.substring(inicioMensaje);
      mensaje.trim();

      if (mensaje.length() > 0) {
        Serial.println("\n=== SMS RECIBIDO ===");
        Serial.println("De: " + currentNumber);
        Serial.println("Fecha: " + currentDate);
        Serial.println("Mensaje: " + mensaje);
        Serial.println("====================\n");

        if (commandCallback) {
          commandCallback(currentNumber, mensaje);
        }
      }

      smsBuffer = "";
      enSMS = false;
      currentNumber = "";
      currentDate = "";
    }
  }

  if (smsBuffer.length() > 0 && !sim800.available()) {
    if (millis() - lastByteTime > SMS_TIMEOUT_MS) {
      LOG_WARN("SMS timeout, descartando buffer parcial");
      smsBuffer = "";
      enSMS = false;
      currentNumber = "";
      currentDate = "";
    }
  }
}

void Sim800lAlertSender::sendResponse(const String &numero,
                                      const String &mensaje) {
  if (!isResponsive()) {
    LOG_ERROR("No se puede enviar SMS, módulo GSM no responde");
    return;
  }
  Serial.println("Enviando respuesta SMS a: " + numero);
  sendATCommand("AT+CMGF=1", "OK", 3000);
  sendATCommand("AT+CSCS=\"GSM\"", "OK", 3000);

  String cmd = "AT+CMGS=\"" + numero + "\"";
  sim800.println(cmd);
  delay(3000);

  unsigned long timeout = millis();
  bool gotPrompt = false;
  String response = "";

  while (millis() - timeout < 10000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
      if (c == '>') {
        gotPrompt = true;
        break;
      }
    }
    if (gotPrompt)
      break;
    delay(100);
  }
  if (!gotPrompt)
    return;

  sim800.println(mensaje);
  delay(2000);
  sim800.write(0x1A);

  timeout = millis();
  bool smsSent = false;
  while (millis() - timeout < 15000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available()) {
      String resp_str = sim800.readStringUntil('\n');
      if (resp_str.indexOf("OK") >= 0 || resp_str.indexOf("+CMGS") >= 0)
        smsSent = true;
    }
    if (smsSent)
      break;
    delay(100);
  }
}

bool Sim800lAlertSender::isResponsive() {
  while (sim800.available())
    sim800.read();
  sim800.println("AT");
  delay(1000);
  bool responded = false;
  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    if (sim800.available()) {
      String response = sim800.readString();
      if (response.indexOf("OK") >= 0) {
        responded = true;
        break;
      }
    }
    delay(100);
  }
  return responded;
}

void Sim800lAlertSender::checkStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 30000) {
    if (!isResponsive()) {
      LOG_WARN("Reiniciando comunicación GSM...");
      begin(currentAuthCode);
    } else {
      // Cellular RF Jamming Detection (GTJDC): monitor abrupt signal loss + CREG drop
      String csqResp = queryATResponse("AT+CSQ", 1000);
      int csqIdx = csqResp.indexOf("+CSQ:");
      if (csqIdx >= 0) {
        int commaIdx = csqResp.indexOf(",", csqIdx);
        if (commaIdx > csqIdx) {
          int csq = csqResp.substring(csqIdx + 5, commaIdx).toInt();
          String cregResp = queryATResponse("AT+CREG?", 1000);
          bool registered =
              (cregResp.indexOf(",1") >= 0 || cregResp.indexOf(",5") >= 0);
          if (lastRssiCsq_ >= 12 && (csq == 0 || csq == 99) && !registered) {
            jammingDetected_ = true;
            LOG_WARN("Posible interferencia RF GSM detectada (GTJDC)");
          } else if (registered && csq >= 5 && csq != 99) {
            jammingDetected_ = false;
          }
          if (csq != 99)
            lastRssiCsq_ = csq;
        }
      }
    }
    lastCheck = millis();
  }
}

bool Sim800lAlertSender::sendATCommand(const String &command,
                                       const String &expectedResponse,
                                       unsigned long timeoutMs) {
  while (sim800.available())
    sim800.read();
  sim800.println(command);
  unsigned long startTime = millis();
  String response = "";
  while (millis() - startTime < timeoutMs) {
    if (sim800.available()) {
      char c = sim800.read();
      response += c;
      if (response.indexOf(expectedResponse) >= 0)
        return true;
    }
    delay(10);
  }
  return false;
}

void Sim800lAlertSender::sendAlert(const AccidentEvent &accident,
                                   const GpsLocation &gps,
                                   const String &timestamp,
                                   const String &chipId) {
  const AccidentConfig &cfg = m_config.config();
  if (!cfg.sms_enabled || strlen(cfg.emergency_phone) < 10)
    return;

  if (!isResponsive())
    return;

  String sms_data = "ALERTA ACCIDENTE\r\n";
  if (accident.impact_detected)
    sms_data += "Tipo: IMPACTO (" + String(accident.peak_g, 2) + "g)\r\n";
  if (accident.rollover_detected)
    sms_data += "Tipo: VUELCO\r\n";
  if (accident.tow_detected || gps.towDetected)
    sms_data += "Tipo: REMOLQUE NO AUTORIZADO\r\n";

  if (gps.isValid) {
    sms_data += "Ubic: http://maps.google.com/maps?q=loc:";
    sms_data +=
        String(gps.latitude, 6) + "," + String(gps.longitude, 6) + "\r\n";
  } else {
    sms_data += "GPS: No disponible\r\n";
  }

  sms_data += "Hora: " + timestamp + "\r\nID: " + chipId;

  if (!sendATCommand("AT+CMGF=1", "OK", 5000))
    return;
  if (!sendATCommand("AT+CSCS=\"GSM\"", "OK", 3000))
    return;

  String phoneNumber = cfg.emergency_phone;
  phoneNumber.replace(" ", "");
  phoneNumber.replace("-", "");
  phoneNumber.replace("(", "");
  phoneNumber.replace(")", "");
  if (!phoneNumber.startsWith("+"))
    phoneNumber = "+" + phoneNumber;

  String cmd = "AT+CMGS=\"" + phoneNumber + "\"";
  sim800.println(cmd);
  delay(3000);

  unsigned long timeout = millis();
  bool gotPrompt = false;
  while (millis() - timeout < 10000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available()) {
      char c = sim800.read();
      if (c == '>') {
        gotPrompt = true;
        break;
      }
    }
    if (gotPrompt)
      break;
    delay(100);
  }

  if (!gotPrompt)
    return;

  sim800.println(sms_data);
  delay(2000);
  sim800.write(0x1A);

  timeout = millis();
  while (millis() - timeout < 15000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available())
      sim800.read();
    delay(100);
  }
}
