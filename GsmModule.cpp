#include "GsmModule.h"

String GsmModule::currentAuthCode = "";
GsmModule gsmManager(GSM_RX_PIN, GSM_TX_PIN);

GsmModule::GsmModule(uint8_t rxPin, uint8_t txPin) : sim800(rxPin, txPin) {
  commandCallback = nullptr;
  smsAuthFailCount = 0;
  smsLockoutStart = 0;
  enSMS = false;
  lastByteTime = 0;
}

void GsmModule::begin(const String &initialAuthCode) {
  GsmModule::currentAuthCode = initialAuthCode;

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
  LOG_INFO("Módulo GSM inicializado para recibir SMS");
}

void GsmModule::setCommandCallback(SmsCommandCallback cb) {
  commandCallback = cb;
}

void GsmModule::update() { processReceivedSms(); }

void GsmModule::processReceivedSms() {
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

void GsmModule::sendResponseSms(const String &numero, const String &mensaje) {
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

bool GsmModule::isResponsive() {
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

void GsmModule::checkStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 30000) {
    if (!isResponsive()) {
      LOG_WARN("Reiniciando comunicación GSM...");
      begin(currentAuthCode);
    }
    lastCheck = millis();
  }
}

bool GsmModule::sendATCommand(const String &command,
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

void GsmModule::sendSmsAlert(const AccidentData &accident, const GPSData &gps,
                             const String &timestamp, const String &chipId) {
  if (!DeviceConfig.currentConfig.sms_enabled ||
      strlen(DeviceConfig.currentConfig.emergency_phone) < 10)
    return;

  if (!isResponsive())
    return;

  String sms_data = "ALERTA ACCIDENTE\r\n";
  if (accident.impact_detected)
    sms_data += "Tipo: IMPACTO\r\n";
  if (accident.rollover_detected)
    sms_data += "Tipo: VUELCO\r\n";

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

  String phoneNumber = DeviceConfig.currentConfig.emergency_phone;
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
