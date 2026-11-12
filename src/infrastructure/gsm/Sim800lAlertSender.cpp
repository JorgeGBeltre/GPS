#include "infrastructure/gsm/Sim800lAlertSender.h"

#include "config.h"
#include "infrastructure/system/EspSysUtils.h"
#include <cmath>
#include <cstdlib>

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

bool parseRegLacCellId(const String &regResp, uint32_t &outLac,
                       uint32_t &outCellId) {
  int q1 = regResp.indexOf('"');
  if (q1 < 0)
    return false;
  int q2 = regResp.indexOf('"', q1 + 1);
  if (q2 < 0)
    return false;
  int q3 = regResp.indexOf('"', q2 + 1);
  if (q3 < 0)
    return false;
  int q4 = regResp.indexOf('"', q3 + 1);
  if (q4 < 0)
    return false;

  String lacHex = regResp.substring(q1 + 1, q2);
  String ciHex = regResp.substring(q3 + 1, q4);
  lacHex.trim();
  ciHex.trim();
  if (lacHex.length() == 0 || ciHex.length() == 0)
    return false;

  uint32_t lac = static_cast<uint32_t>(std::strtoul(lacHex.c_str(), nullptr, 16));
  uint32_t ci = static_cast<uint32_t>(std::strtoul(ciHex.c_str(), nullptr, 16));
  if (ci == 0 || ci == 0xFFFFFFFFUL)
    return false;

  outLac = lac;
  outCellId = ci;
  return true;
}

void extractDigitsMccMnc(const String &raw, int &outMcc, int &outMnc) {
  String digits = "";
  for (int i = 0; i < (int)raw.length(); ++i) {
    if (raw[i] >= '0' && raw[i] <= '9') {
      digits += raw[i];
    } else if (digits.length() >= 5) {
      break;
    } else {
      digits = "";
    }
  }
  if (digits.length() >= 5) {
    outMcc = digits.substring(0, 3).toInt();
    int mncLen = 2;
    for (size_t i = 0; i < sizeof(kCarrierApnTable) / sizeof(kCarrierApnTable[0]);
         ++i) {
      if (digits.startsWith(kCarrierApnTable[i].mccMncPrefix)) {
        int pLen = static_cast<int>(strlen(kCarrierApnTable[i].mccMncPrefix));
        if (pLen >= 5)
          mncLen = pLen - 3;
        break;
      }
    }
    if ((int)digits.length() >= 3 + mncLen) {
      outMnc = digits.substring(3, 3 + mncLen).toInt();
    }
  }
}
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
    EspSysUtils::heartbeat();
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
  String cpinResp = queryATResponse("AT+CPIN?", 800);
  if (cpinResp.indexOf("READY") >= 0) {
    simPresent_ = true;
  }
  String imsiResp = queryATResponse("AT+CIMI", 1200);
  extractDigitsMccMnc(imsiResp, mcc_, mnc_);
  for (size_t i = 0; i < sizeof(kCarrierApnTable) / sizeof(kCarrierApnTable[0]);
       ++i) {
    if (imsiResp.indexOf(kCarrierApnTable[i].mccMncPrefix) >= 0) {
      simPresent_ = true;
      apnAutoDetected_ = true;
      resolvedApn_ = String(kCarrierApnTable[i].apn);
      return resolvedApn_;
    }
  }
  String cgdcontResp = queryATResponse("AT+CGDCONT?", 800);
  int tag = cgdcontResp.indexOf("+CGDCONT:");
  if (tag >= 0) {
    int qCount = 0;
    int qStart = -1;
    for (int i = tag; i < (int)cgdcontResp.length(); ++i) {
      if (cgdcontResp[i] == '"') {
        ++qCount;
        if (qCount == 3) {
          qStart = i + 1;
        } else if (qCount == 4 && qStart > 0) {
          String netApn = cgdcontResp.substring(qStart, i);
          netApn.trim();
          if (netApn.length() > 0) {
            simPresent_ = true;
            apnAutoDetected_ = true;
            resolvedApn_ = netApn;
            return resolvedApn_;
          }
          break;
        }
      }
    }
  }
  return "";
}

void Sim800lAlertSender::probeAutoApn() {
  sim800.begin(9600);
  delay(200);
  String atResp = queryATResponse("AT", 400);
  if (atResp.indexOf("OK") < 0) {
    atResp = queryATResponse("AT", 400);
  }
  if (atResp.indexOf("OK") < 0) {
    return;
  }
  sendATCommand("ATE0", "OK", 400);
  String detected = autoResolveCarrierApn();
  if (detected.length() > 0) {
    String cgdcont = String("AT+CGDCONT=1,\"IP\",\"") + detected + "\"";
    sendATCommand(cgdcont, "OK", 600);
  }
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
  sendATCommand("AT+CREG=2", "OK", 1000);
  sendATCommand("AT+CGREG=2", "OK", 1000);

  resolvedApn_ = String(m_config.config().apn);
  if (resolvedApn_.length() == 0) {
    resolvedApn_ = autoResolveCarrierApn();
  }
  if (resolvedApn_.length() > 0) {
    String cgdcont = String("AT+CGDCONT=1,\"IP\",\"") + resolvedApn_ + "\"";
    sendATCommand(cgdcont, "OK", 2000);
  }
  pollLbsCellInfo(false);
  LOG_INFO("Módulo GSM inicializado para recibir SMS");
}

void Sim800lAlertSender::pollLbsCellInfo(bool requestCoordinatesIfNoGps) {
  lastLbsPollMs_ = millis();

  if (mcc_ == 0) {
    String imsiResp = queryATResponse("AT+CIMI", 1000);
    extractDigitsMccMnc(imsiResp, mcc_, mnc_);
  }

  uint32_t regLac = 0, regCi = 0;
  String cregResp = queryATResponse("AT+CREG?", 800);
  if (!parseRegLacCellId(cregResp, regLac, regCi)) {
    String cgregResp = queryATResponse("AT+CGREG?", 800);
    parseRegLacCellId(cgregResp, regLac, regCi);
  }
  if (regCi > 0) {
    lac_ = regLac;
    cellId_ = regCi;
  }

  if (requestCoordinatesIfNoGps) {
    // 1. Intentar AT+CLBS=1,1 (+CLBS: 0,<lat>,<lon>,<acc>)
    String clbs = queryATResponse("AT+CLBS=1,1", 2500);
    int clbsIdx = clbs.indexOf("+CLBS:");
    if (clbsIdx >= 0) {
      int lineEnd = clbs.indexOf('\n', clbsIdx);
      String line = (lineEnd >= 0) ? clbs.substring(clbsIdx + 6, lineEnd)
                                   : clbs.substring(clbsIdx + 6);
      line.trim();
      int c1 = line.indexOf(',');
      int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
      int c3 = (c2 >= 0) ? line.indexOf(',', c2 + 1) : -1;
      if (c1 >= 0 && c2 >= 0) {
        int code = line.substring(0, c1).toInt();
        double lat = line.substring(c1 + 1, c2).toDouble();
        double lon = (c3 >= 0) ? line.substring(c2 + 1, c3).toDouble()
                               : line.substring(c2 + 1).toDouble();
        int acc = (c3 >= 0) ? line.substring(c3 + 1).toInt() : 500;
        if (code == 0 && (std::fabs(lat) > 0.0001 || std::fabs(lon) > 0.0001)) {
          lbsLat_ = lat;
          lbsLon_ = lon;
          lbsAccM_ = (acc > 0) ? acc : 500;
          hasLbsCoords_ = true;
          return;
        }
      }
    }

    // 2. Fallback SIM800L AT+CIPGSMLOC=1,1 (+CIPGSMLOC: 0,<lon>,<lat>,<date>,<time>)
    String gsmLoc = queryATResponse("AT+CIPGSMLOC=1,1", 2500);
    int locIdx = gsmLoc.indexOf("+CIPGSMLOC:");
    if (locIdx >= 0) {
      int lineEnd = gsmLoc.indexOf('\n', locIdx);
      String line = (lineEnd >= 0) ? gsmLoc.substring(locIdx + 11, lineEnd)
                                   : gsmLoc.substring(locIdx + 11);
      line.trim();
      int c1 = line.indexOf(',');
      int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
      int c3 = (c2 >= 0) ? line.indexOf(',', c2 + 1) : -1;
      if (c1 >= 0 && c2 >= 0 && c3 >= 0) {
        int code = line.substring(0, c1).toInt();
        double lon = line.substring(c1 + 1, c2).toDouble();
        double lat = line.substring(c2 + 1, c3).toDouble();
        if (code == 0 && (std::fabs(lat) > 0.0001 || std::fabs(lon) > 0.0001)) {
          lbsLat_ = lat;
          lbsLon_ = lon;
          lbsAccM_ = 500;
          hasLbsCoords_ = true;
        }
      }
    }
  }
}

void Sim800lAlertSender::enrichWithLbs(GpsLocation &gps) {
  if ((cellId_ == 0 && mcc_ == 0) ||
      (!gps.isValid && (lastLbsPollMs_ == 0 || millis() - lastLbsPollMs_ >= 25000UL))) {
    pollLbsCellInfo(!gps.isValid);
  }
  gps.lbsValid = (mcc_ > 0 && cellId_ > 0) || hasLbsCoords_;
  gps.mcc = mcc_;
  gps.mnc = mnc_;
  gps.lac = lac_;
  gps.cellId = cellId_;
  gps.lbsAccuracyM = lbsAccM_;

  if (!gps.isValid && hasLbsCoords_) {
    gps.isValid = true;
    gps.isLbsFallback = true;
    gps.latitude = lbsLat_;
    gps.longitude = lbsLon_;
  }
}

void Sim800lAlertSender::setCommandCallback(SmsCommandCallback cb) {
  commandCallback = cb;
}

void Sim800lAlertSender::update() {
  processReceivedSms();
  if (!enSMS && smsBuffer.length() == 0) {
    checkStatus();
  }
}

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
  String resp = queryATResponse("AT", 600);
  return resp.indexOf("OK") >= 0;
}

void Sim800lAlertSender::recoverUnresponsiveModem() {
  const unsigned long now = millis();
  if (lastModemRecoveryMs_ != 0 && (now - lastModemRecoveryMs_ < 300000UL)) {
    return;
  }
  lastModemRecoveryMs_ = now;
  modemRecoveries_++;
  consecutiveAtFailures_ = 0;
  LOG_WARN("Auto-recuperando SIM800L no responsivo (AT+CFUN=1,1, cooldown 300s)...");
  sim800.begin(9600);
  sendATCommand("AT+CFUN=1,1", "OK", 1000);
  for (int i = 0; i < 6; ++i) {
    EspSysUtils::heartbeat();
    delay(250);
  }
  if (isResponsive()) {
    sendATCommand("ATE0", "OK", 500);
    sendATCommand("AT+CMGF=1", "OK", 500);
    sendATCommand("AT+CNMI=2,2,0,0,0", "OK", 500);
    sendATCommand("AT+CREG=2", "OK", 500);
    sendATCommand("AT+CGREG=2", "OK", 500);
  }
}

void Sim800lAlertSender::checkStatus() {
  const unsigned long now = millis();
  if (now - lastHealthCheckMs_ < 60000UL)
    return;
  lastHealthCheckMs_ = now;

  String atResp = queryATResponse("AT", 400);
  if (atResp.indexOf("OK") < 0) {
    if (++consecutiveAtFailures_ >= 3) {
      recoverUnresponsiveModem();
    }
    return;
  }
  consecutiveAtFailures_ = 0;

  // Cellular RF Jamming Detection (GTJDC): monitor abrupt signal loss + CREG drop
  String csqResp = queryATResponse("AT+CSQ", 600);
  int csqIdx = csqResp.indexOf("+CSQ:");
  if (csqIdx >= 0) {
    int commaIdx = csqResp.indexOf(",", csqIdx);
    if (commaIdx > csqIdx) {
      int csq = csqResp.substring(csqIdx + 5, commaIdx).toInt();
      String cregResp = queryATResponse("AT+CREG?", 600);
      bool registered =
          (cregResp.indexOf(",1") >= 0 || cregResp.indexOf(",5") >= 0);
      parseRegLacCellId(cregResp, lac_, cellId_);
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

bool Sim800lAlertSender::sendATCommand(const String &command,
                                       const String &expectedResponse,
                                       unsigned long timeoutMs) {
  while (sim800.available())
    sim800.read();
  sim800.println(command);
  unsigned long startTime = millis();
  String response = "";
  while (millis() - startTime < timeoutMs) {
    EspSysUtils::heartbeat();
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
    sms_data += gps.isLbsFallback ? "Ubic(LBS): http://maps.google.com/maps?q=loc:"
                                  : "Ubic: http://maps.google.com/maps?q=loc:";
    sms_data +=
        String(gps.latitude, 6) + "," + String(gps.longitude, 6) + "\r\n";
  } else {
    sms_data += "GPS: No disponible\r\n";
  }
  if (gps.lbsValid && gps.cellId > 0) {
    sms_data += "Celda LBS: " + String(gps.mcc) + "-" + String(gps.mnc) + "-" +
                String(gps.lac) + "-" + String(gps.cellId) + "\r\n";
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
