#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <MPU6050.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <base64.h>
#include <time.h>

extern "C" {
#include "user_interface.h"
}

static const char MQTT_TOPIC_TELEMETRY[] PROGMEM = "telemetry";
static const char MQTT_TOPIC_EVENTS[] PROGMEM = "events";
static const char MQTT_TOPIC_COMMANDS[] PROGMEM = "commands";
static const char MQTT_TOPIC_OTA[] PROGMEM = "ota";
static const char MQTT_TOPIC_STATUS[] PROGMEM = "status";
static const char MQTT_TOPIC_CONFIG[] PROGMEM = "config";

struct DeviceInfo {
  String chipId;
  String chipType;
  String macAddress;
  String clientID;
  String firmwareVersion;
};

struct GPSData {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0;
  float speed = 0.0;
  float course = 0.0;
  int satellites = 0;
  bool isValid = false;
  String timestamp = "";
};

struct AccidentData {
  bool impact_detected = false;
  bool rollover_detected = false;
  float impact_magnitude = 0.0f;
  float roll_angle = 0.0f;
  float pitch_angle = 0.0f;
  unsigned long detection_time = 0;
};

WiFiClientSecure wifiClient;
PubSubClient mqtt_client(wifiClient);
bool mqttConnected = false;
DeviceInfo device;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
MPU6050 mpu;
SoftwareSerial sim800(GSM_RX_PIN, GSM_TX_PIN);

AccidentConfig accidentConfig;

bool otaInProgress = false;

struct OTAContext {
  bool inProgress = false;
  String firmwareVersion;
  int currentPart = 0;
  int totalParts = 0;
  unsigned long startTime = 0;
  size_t receivedSize = 0;
  size_t totalSize = 0;
};
OTAContext otaContext;

unsigned long lastReconnectAttempt = 0;
int mqttFailCount = 0;

bool alert_triggered = false;
unsigned long alert_time = 0;

int16_t ax, ay, az;
int16_t oldx = 0, oldy = 0, oldz = 0;
int deltx = 0, delty = 0, deltz = 0;
int vibration = 2, devibrate = 75;
float magnitude = 0.0f;
byte updateflag = 0;
unsigned long lastAccidentCheck = 0;

static float acc_history[ACC_FILTER_SIZE] = {0};
static int acc_idx = 0;

static unsigned long lastAccidentEvent = 0;

enum WiFiStatus { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };
WiFiStatus currentWiFiStatus = WIFI_DISCONNECTED;
unsigned long previousMillis = 0;

static int smsAuthFailCount = 0;
static unsigned long smsLockoutStart = 0;

void procesarOTAChunk(const String &message);
bool iniciarOTA(const String &firmwareVersion, int totalParts,
                size_t totalSize);
bool procesarChunkOTA(const String &base64Data, int partIndex);
void finalizarOTA();
void cleanupOTA();
void publishOTAProgress(int progress, const String &firmwareVersion);
void publishOTAError(const String &errorMessage,
                     const String &firmwareVersion = "");
void publishOTASuccess(const String &firmwareVersion);
bool base64DecodeToBuffer(const String &encoded, uint8_t *out, size_t outMax,
                          size_t &outLen);
bool isBase64Char(unsigned char c);

bool configurarTiempoNTP();
String getISOTimestamp();
void verificarYSincronizarTiempo();
DeviceInfo getDeviceInfo();
void publishGPSLocation(const GPSData &data);
void publishAccidentEvent(const AccidentData &accident, const GPSData &gps);
void publishEvent(const char *evento, const char *descripcion);
void publishStatus();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
bool connectMQTT();
void setupOTA();
void updateWiFiLED();
void reconnectWiFi();
void updateWiFiStatus();
void setupWiFi();
void handleButton();
void procesarComandoMQTT(const String &message);
void readGPSData(GPSData &data);
void publishMQTTMessage(const char *topic, const String &message);
void checkAccidents();
void sendSMSAlert(const AccidentData &accident, const GPSData &gps);
void loadConfig();
void saveConfig();
void procesarConfiguracion(const String &message);
void initializeGSM();
bool checkGSMModule();
bool sendATCommand(String command, String expectedResponse,
                   unsigned long timeoutMs);
void checkGSMStatus();
void procesarComandoSerial();
void mostrarMenuSerial();
void cambiarNumeroEmergencia(String nuevoNumero);
void cambiarSensibilidad(String valor);
void cambiarAnguloVuelco(String valor);
void cambiarDelayAlerta(String valor);
void cambiarSMSHabilitado(String valor);
void mostrarConfiguracionActual();
void procesarComando(String comando);
void checkGPSStatus();
void diagnosticarGPSCompleto();
void procesarSMSRecibido();
void enviarRespuestaSMS(String numero, String mensaje);
void procesarComandoSMS(String numero, String comando);
String crearMenuSMS();
String generarCodigoAuth();

void procesarSMSRecibido() {
  static String buffer = "";
  static bool enSMS = false;
  static String numero = "";
  static String fecha = "";
  static unsigned long lastByteTime = 0;

  while (sim800.available()) {
    char c = sim800.read();
    lastByteTime = millis();

    if (buffer.length() >= SMS_BUFFER_MAX) {
      LOG_WARN("SMS buffer overflow, descartando");
      buffer = "";
      enSMS = false;
      numero = "";
      fecha = "";
      return;
    }

    buffer += c;

    if (buffer.indexOf("+CMT:") >= 0 && !enSMS) {
      enSMS = true;
      int inicioNum = buffer.indexOf("\"", buffer.indexOf("+CMT:")) + 1;
      int finNum = buffer.indexOf("\"", inicioNum);
      if (inicioNum > 0 && finNum > inicioNum)
        numero = buffer.substring(inicioNum, finNum);

      int inicioFecha = buffer.indexOf("\"", finNum + 1) + 1;
      int finFecha = buffer.indexOf("\"", inicioFecha);
      if (inicioFecha > 0 && finFecha > inicioFecha)
        fecha = buffer.substring(inicioFecha, finFecha);
    }

    if (enSMS && buffer.indexOf("\r\n\r\n") >= 0) {
      int inicioMensaje = buffer.indexOf("\r\n\r\n") + 4;
      String mensaje = buffer.substring(inicioMensaje);
      mensaje.trim();

      if (mensaje.length() > 0) {
        Serial.println("\n=== SMS RECIBIDO ===");
        Serial.println("De: " + numero);
        Serial.println("Fecha: " + fecha);
        Serial.println("Mensaje: " + mensaje);
        Serial.println("====================\n");
        procesarComandoSMS(numero, mensaje);

        int len = mensaje.length();
        int maxLen = 30;
        if (len > maxLen)
          len = maxLen;
        String evento = "SMS_RECEIVED: " + mensaje.substring(0, len);
        publishEvent("SMS_RECEIVED", evento.c_str());
      }

      buffer = "";
      enSMS = false;
      numero = "";
      fecha = "";
    }
  }

  if (buffer.length() > 0 && !sim800.available()) {
    if (millis() - lastByteTime > SMS_TIMEOUT_MS) {
      LOG_WARN("SMS timeout, descartando buffer parcial");
      buffer = "";
      enSMS = false;
      numero = "";
      fecha = "";
    }
  }
}

void enviarRespuestaSMS(String numero, String mensaje) {
  if (!checkGSMModule()) {
    LOG_ERROR("No se puede enviar SMS, módulo GSM no responde");
    return;
  }

  Serial.println("Enviando respuesta SMS a: " + numero);
  Serial.println("Mensaje: " + mensaje);

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

  if (!gotPrompt) {
    LOG_ERROR("No se recibió prompt > para SMS");
    return;
  }

  sim800.println(mensaje);
  delay(2000);
  sim800.write(0x1A);

  timeout = millis();
  bool smsSent = false;
  response = "";

  while (millis() - timeout < 15000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
      if (response.indexOf("OK") >= 0 || response.indexOf("+CMGS") >= 0) {
        smsSent = true;
        break;
      }
      if (response.indexOf("ERROR") >= 0)
        break;
    }
    if (smsSent)
      break;
    delay(100);
  }

  if (smsSent) {
    Serial.println("Respuesta SMS enviada exitosamente");
    publishEvent("SMS_RESPONSE_SENT", "Respuesta SMS enviada");
  } else {
    LOG_ERROR("No se pudo enviar respuesta SMS");
  }
}

String generarCodigoAuth() {
  uint32_t seed = ESP.getChipId() ^ (millis() & 0xFFFFU);
  return String(seed % 900000UL + 100000UL);
}

void procesarComandoSMS(String numero, String comando) {
  comando.trim();

  if (comando.length() == 0) {
    enviarRespuestaSMS(numero,
                       "Comando vacío. Envía 'HELP' para ver opciones.");
    return;
  }

  Serial.println("Procesando comando SMS: " + comando);

  String comandoUpper = comando;
  comandoUpper.toUpperCase();

  String numeroAutorizado = accidentConfig.emergency_phone;
  numeroAutorizado.replace(" ", "");
  numeroAutorizado.replace("-", "");
  numeroAutorizado.replace("(", "");
  numeroAutorizado.replace(")", "");

  String numeroEntrante = numero;
  numeroEntrante.replace(" ", "");
  numeroEntrante.replace("-", "");
  numeroEntrante.replace("(", "");
  numeroEntrante.replace(")", "");

  if (comandoUpper.startsWith("AUTH ")) {
    if (smsAuthFailCount >= SMS_MAX_AUTH_ATTEMPTS) {
      if (millis() - smsLockoutStart < SMS_LOCKOUT_TIME) {
        enviarRespuestaSMS(numero,
                           "Demasiados intentos. Reintenta en 5 minutos.");
        LOG_WARN("SMS AUTH bloqueado por lockout");
        return;
      } else {
        smsAuthFailCount = 0;
      }
    }

    String codigo = comando.substring(5);
    codigo.trim();
    static String smsAuthCode = "";
    if (smsAuthCode.length() == 0) {
      smsAuthCode = generarCodigoAuth();
      Serial.println("Código SMS AUTH generado: " + smsAuthCode);
    }

    if (codigo == smsAuthCode) {
      smsAuthFailCount = 0;
      smsAuthCode = "";
      enviarRespuestaSMS(numero, "¡Autorizado!");
      String desc1 = "Número autorizado por SMS: " + numero;
      publishEvent("SMS_AUTHORIZED", desc1.c_str());
    } else {
      smsAuthFailCount++;
      if (smsAuthFailCount >= SMS_MAX_AUTH_ATTEMPTS)
        smsLockoutStart = millis();
      enviarRespuestaSMS(numero,
                         "Código incorrecto. Intentos restantes: " +
                             String(SMS_MAX_AUTH_ATTEMPTS - smsAuthFailCount));
    }
    return;
  }

  if (!numeroEntrante.endsWith(numeroAutorizado) &&
      numeroAutorizado.length() >= 10 && numeroEntrante.length() >= 10) {
    String numEntranteSinPrefijo = numeroEntrante;
    if (numEntranteSinPrefijo.length() > 10)
      numEntranteSinPrefijo =
          numEntranteSinPrefijo.substring(numEntranteSinPrefijo.length() - 10);

    String numAutorizadoSinPrefijo = numeroAutorizado;
    if (numAutorizadoSinPrefijo.length() > 10)
      numAutorizadoSinPrefijo = numAutorizadoSinPrefijo.substring(
          numAutorizadoSinPrefijo.length() - 10);

    if (numEntranteSinPrefijo != numAutorizadoSinPrefijo) {
      enviarRespuestaSMS(numero, "No autorizado. Usa AUTH [código] o configura "
                                 "este número como emergencia.");
      String desc2 = "Intento no autorizado de: " + numero;
      publishEvent("SMS_UNAUTHORIZED", desc2.c_str());
      return;
    }
  }

  if (comandoUpper == "HELP" || comandoUpper == "AYUDA" ||
      comandoUpper == "?") {
    enviarRespuestaSMS(numero, crearMenuSMS());
  } else if (comandoUpper == "STATUS" || comandoUpper == "ESTADO") {
    String respuesta = "ESTADO DEL SISTEMA\n";
    respuesta += "ID: " + device.chipId + "\n";
    respuesta += "FW: " + device.firmwareVersion + "\n";
    respuesta +=
        "WiFi: " +
        String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") +
        "\n";
    respuesta +=
        "MQTT: " + String(mqttConnected ? "Conectado" : "Desconectado") + "\n";
    respuesta +=
        "GPS: " + String(gps.location.isValid() ? "Con Fix" : "Sin Fix") + "\n";
    respuesta += "Satélites: " + String(gps.satellites.value()) + "\n";
    respuesta += "Memoria: " + String(ESP.getFreeHeap()) + " bytes\n";
    publishStatus();
    enviarRespuestaSMS(numero, respuesta);
  } else if (comandoUpper == "LOCATION" || comandoUpper == "UBICACION" ||
             comandoUpper == "LOC") {
    GPSData data;
    readGPSData(data);
    String respuesta = "UBICACIÓN GPS\n";
    if (data.isValid) {
      respuesta += "Lat: " + String(data.latitude, 6) + "\n";
      respuesta += "Lon: " + String(data.longitude, 6) + "\n";
      respuesta += "Alt: " + String(data.altitude, 1) + "m\n";
      respuesta += "Vel: " + String(data.speed, 1) + "km/h\n";
      respuesta += "Sat: " + String(data.satellites) + "\n";
      respuesta += "Mapa: http://maps.google.com/maps?q=loc:" +
                   String(data.latitude, 6) + "," + String(data.longitude, 6);
      publishGPSLocation(data);
    } else {
      respuesta += "GPS sin fix\nSatélites: " + String(gps.satellites.value()) +
                   "\nEspere unos minutos...";
    }
    enviarRespuestaSMS(numero, respuesta);
  } else if (comandoUpper == "CONFIG" || comandoUpper == "CONFIGURACION") {
    String respuesta = "CONFIGURACIÓN ACTUAL\n";
    respuesta += "Tel emerg: " + String(accidentConfig.emergency_phone) + "\n";
    respuesta += "Sensibilidad: " + String(accidentConfig.sensitivity) + "\n";
    respuesta +=
        "Ángulo vuelco: " + String(accidentConfig.rollover_angle) + "°\n";
    respuesta +=
        "Delay alerta: " + String(accidentConfig.alert_delay / 1000) + "s\n";
    respuesta +=
        "SMS: " +
        String(accidentConfig.sms_enabled ? "Activado" : "Desactivado") + "\n";
    enviarRespuestaSMS(numero, respuesta);
  } else if (comandoUpper == "TEST" || comandoUpper == "TEST_SMS") {
    enviarRespuestaSMS(numero, "Enviando SMS de prueba...");
    AccidentData testAccident;
    GPSData testGps;
    testAccident.impact_detected = true;
    testAccident.impact_magnitude = 2500;
    testGps.latitude = 18.735693;
    testGps.longitude = -70.162651;
    testGps.isValid = true;
    sendSMSAlert(testAccident, testGps);
    enviarRespuestaSMS(numero,
                       "SMS de prueba enviado al número de emergencia.");
  } else if (comandoUpper.startsWith("SET PHONE ")) {
    String nuevoNumero = comando.substring(10);
    cambiarNumeroEmergencia(nuevoNumero);
    enviarRespuestaSMS(numero, "Número actualizado a: " + nuevoNumero);
  } else if (comandoUpper.startsWith("SET SENS ")) {
    String valor = comando.substring(9);
    cambiarSensibilidad(valor);
    enviarRespuestaSMS(numero, "Sensibilidad actualizada a: " + valor);
  } else if (comandoUpper.startsWith("SET ANGLE ")) {
    String valor = comando.substring(10);
    cambiarAnguloVuelco(valor);
    enviarRespuestaSMS(numero, "Ángulo vuelco actualizado a: " + valor + "°");
  } else if (comandoUpper.startsWith("SET DELAY ")) {
    String valor = comando.substring(10);
    cambiarDelayAlerta(valor);
    enviarRespuestaSMS(numero, "Delay alerta actualizado a: " + valor + "ms");
  } else if (comandoUpper.startsWith("SET SMS ")) {
    String valor = comando.substring(8);
    cambiarSMSHabilitado(valor);
    enviarRespuestaSMS(numero, "SMS " + String(accidentConfig.sms_enabled
                                                   ? "activado"
                                                   : "desactivado"));
  } else if (comandoUpper == "SAVE" || comandoUpper == "GUARDAR") {
    saveConfig();
    enviarRespuestaSMS(numero, "Configuración guardada en EEPROM");
  } else if (comandoUpper == "RESET" || comandoUpper == "REINICIAR") {
    enviarRespuestaSMS(numero, "Reiniciando dispositivo...");
    delay(2000);
    ESP.restart();
  } else if (comandoUpper == "WIFI" || comandoUpper == "WIFI INFO") {
    String respuesta = "INFORMACIÓN WiFi\n";
    respuesta += "SSID: " + WiFi.SSID() + "\n";
    respuesta +=
        "Estado: " +
        String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") +
        "\n";
    if (WiFi.status() == WL_CONNECTED) {
      respuesta += "IP: " + WiFi.localIP().toString() + "\n";
      respuesta += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    }
    enviarRespuestaSMS(numero, respuesta);
  } else if (comandoUpper == "GPS") {
    String respuesta = "ESTADO GPS\n";
    respuesta += "Satélites: " + String(gps.satellites.value()) + "\n";
    respuesta += "Fix: " + String(gps.location.isValid() ? "SI" : "NO") + "\n";
    if (gps.location.isValid()) {
      respuesta += "Lat: " + String(gps.location.lat(), 6) + "\n";
      respuesta += "Lon: " + String(gps.location.lng(), 6) + "\n";
    } else {
      respuesta += "Esperando señal GPS...";
    }
    enviarRespuestaSMS(numero, respuesta);
  } else if (comandoUpper == "GSM") {
    bool gsmOk = checkGSMModule();
    enviarRespuestaSMS(numero, "GSM: " + String(gsmOk ? "OK" : "ERROR"));
  } else if (comandoUpper == "MQTT") {
    String respuesta =
        "MQTT: " + String(mqttConnected ? "Conectado" : "Desconectado");
    if (!mqttConnected) {
      connectMQTT();
      respuesta += "\nIntentando reconectar...";
    }
    enviarRespuestaSMS(numero, respuesta);
  } else {
    enviarRespuestaSMS(
        numero, "Comando no reconocido. Envía 'HELP' para ver opciones.");
  }
}

String crearMenuSMS() {
  String menu = "COMANDOS DISPONIBLES:\n";
  menu += "HELP - Este menú\n";
  menu += "STATUS - Estado sistema\n";
  menu += "LOCATION - Ubicación GPS\n";
  menu += "CONFIG - Config actual\n";
  menu += "TEST - SMS prueba\n";
  menu += "\nCONFIGURAR:\n";
  menu += "SET PHONE [número]\n";
  menu += "SET SENS [valor]\n";
  menu += "SET ANGLE [grados]\n";
  menu += "SET DELAY [ms]\n";
  menu += "SET SMS [ON/OFF]\n";
  menu += "SAVE - Guardar\n";
  menu += "\nSISTEMA:\n";
  menu += "WIFI GPS GSM MQTT RESET\n";
  menu += "\nID: " + device.chipId;
  return menu;
}

void checkGPSStatus() {
  static unsigned long lastGPSDebug = 0;

  if (millis() - lastGPSDebug > 10000) {
    Serial.println("\n=== ESTADO GPS ===");
    Serial.println("Datos sin procesar disponibles: " +
                   String(gpsSerial.available()));
    Serial.println("Chars procesados: " + String(gps.charsProcessed()));
    Serial.println("Chars con fallos: " + String(gps.failedChecksum()));
    Serial.println("Satélites: " + String(gps.satellites.value()));
    Serial.println("Ubicación válida: " +
                   String(gps.location.isValid() ? "SI" : "NO"));

    if (gps.location.isValid()) {
      Serial.printf("Lat: %.6f, Lon: %.6f\n", gps.location.lat(),
                    gps.location.lng());
      Serial.printf("Altitud: %.1f m, Velocidad: %.1f km/h\n",
                    gps.altitude.meters(), gps.speed.kmph());
    } else {
      Serial.println("GPS no tiene fix aún");
      Serial.println("Tiempo desde último fix: " + String(gps.location.age()) +
                     "ms");
    }
    Serial.println("================================\n");
    lastGPSDebug = millis();
  }
}

void diagnosticarGPSCompleto() {
  Serial.println("=== DIAGNÓSTICO GPS COMPLETO ===");

  Serial.println("1. Verificando puerto Serial GPS...");
  gpsSerial.end();
  delay(100);
  gpsSerial.begin(9600);
  Serial.println("Puerto GPS reiniciado");

  pinMode(GPS_TX_PIN, OUTPUT);
  digitalWrite(GPS_TX_PIN, HIGH);
  delay(10);
  int txState = digitalRead(GPS_TX_PIN);
  Serial.println("Pin TX GPS (GPIO16): " +
                 String(txState == HIGH ? "ALTO ✓" : "BAJO ✗"));

  Serial.println("\n2. Leyendo datos GPS por 3 segundos...");
  unsigned long startTime = millis();
  String rawGPSData = "";
  int byteCount = 0;

  while (millis() - startTime < 3000) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      rawGPSData += c;
      byteCount++;
    }
    delay(10);
  }

  Serial.println("Bytes recibidos: " + String(byteCount));

  if (byteCount > 0) {
    Serial.println("\n3. Analizando datos NMEA...");
    Serial.println("Primeros 300 caracteres:");
    int displayLimit = min(300, (int)rawGPSData.length());
    Serial.println(rawGPSData.substring(0, displayLimit));

    int nmeaLines = 0, ggaCount = 0, rmcCount = 0, gsvCount = 0, gsaCount = 0,
        vtgCount = 0;
    for (int i = 0; i < (int)rawGPSData.length(); i++) {
      if (rawGPSData[i] == '$') {
        nmeaLines++;
        if (i + 6 < (int)rawGPSData.length()) {
          String nmeaType = rawGPSData.substring(i, i + 6);
          if (nmeaType.indexOf("GGA") > 0)
            ggaCount++;
          else if (nmeaType.indexOf("RMC") > 0)
            rmcCount++;
          else if (nmeaType.indexOf("GSV") > 0)
            gsvCount++;
          else if (nmeaType.indexOf("GSA") > 0)
            gsaCount++;
          else if (nmeaType.indexOf("VTG") > 0)
            vtgCount++;
        }
      }
    }

    Serial.println("\n4. Estadísticas NMEA:");
    Serial.println("Total líneas NMEA: " + String(nmeaLines));
    Serial.println("  - GGA: " + String(ggaCount));
    Serial.println("  - RMC: " + String(rmcCount));
    Serial.println("  - GSV: " + String(gsvCount));
    Serial.println("  - GSA: " + String(gsaCount));
    Serial.println("  - VTG: " + String(vtgCount));

    if (nmeaLines > 0) {
      Serial.println("\n5. Checksums válidos: " + String(gps.passedChecksum()));
      Serial.println("Checksums fallidos: " + String(gps.failedChecksum()));
    }
  } else {
    Serial.println(
        "\n¡ADVERTENCIA CRÍTICA: NO SE RECIBIÓ NINGÚN DATO DEL GPS!");
  }

  Serial.println("\n6. Estado TinyGPSPlus:");
  Serial.println("Chars procesados: " + String(gps.charsProcessed()));
  Serial.println("Sentencias con fix: " + String(gps.sentencesWithFix()));
  Serial.println("Edad del fix: " + String(gps.location.age()) + "ms");

  if (gps.satellites.isValid()) {
    Serial.println("\n7. Información de satélites:");
    Serial.println("Satélites visibles: " + String(gps.satellites.value()));
    Serial.println("HDOP: " + String(gps.hdop.value() / 100.0));
  }

  Serial.println("\n=== FIN DIAGNÓSTICO GPS ===\n");
}

void procesarComandoSerial() {
  static String inputString = "";

  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        procesarComando(inputString);
        inputString = "";
      }
    } else {
      if (inputString.length() >= SERIAL_CMD_MAX_LEN) {
        LOG_WARN("CMD overflow, descartando");
        inputString = "";
        return;
      }
      inputString += inChar;
    }
  }
}

void procesarComando(String comando) {
  comando.trim();
  if (comando.length() == 0)
    return;

  Serial.println("Comando recibido: " + comando);
  String comandoLower = comando;
  comandoLower.toLowerCase();

  if (comandoLower == "help" || comandoLower == "?")
    mostrarMenuSerial();
  else if (comandoLower == "config" || comandoLower == "show")
    mostrarConfiguracionActual();
  else if (comandoLower == "status") {
    publishStatus();
    Serial.println("Estado publicado por MQTT");
  } else if (comandoLower == "location") {
    GPSData data;
    readGPSData(data);
    if (data.isValid) {
      Serial.printf("Ubicación: Lat: %.6f, Lon: %.6f\n", data.latitude,
                    data.longitude);
      publishGPSLocation(data);
    } else {
      Serial.println("GPS no válido - Satélites: " +
                     String(gps.satellites.value()));
    }
  } else if (comandoLower == "test_sms") {
    Serial.println("Enviando SMS de prueba...");
    AccidentData testAccident;
    GPSData testGps;
    testAccident.impact_detected = true;
    testAccident.impact_magnitude = 2500;
    testGps.latitude = 18.735693;
    testGps.longitude = -70.162651;
    testGps.isValid = true;
    sendSMSAlert(testAccident, testGps);
  } else if (comandoLower.startsWith("set phone "))
    cambiarNumeroEmergencia(comando.substring(10));
  else if (comandoLower.startsWith("set sensitivity "))
    cambiarSensibilidad(comando.substring(16));
  else if (comandoLower.startsWith("set angle "))
    cambiarAnguloVuelco(comando.substring(10));
  else if (comandoLower.startsWith("set delay "))
    cambiarDelayAlerta(comando.substring(10));
  else if (comandoLower.startsWith("set sms "))
    cambiarSMSHabilitado(comando.substring(8));
  else if (comandoLower == "save") {
    saveConfig();
    Serial.println("Configuración guardada en EEPROM");
  } else if (comandoLower == "reset") {
    Serial.println("Reiniciando...");
    delay(1000);
    ESP.restart();
  } else if (comandoLower == "wifi") {
    Serial.println("SSID: " + WiFi.SSID());
    Serial.println("Estado: " + String(WiFi.status() == WL_CONNECTED
                                           ? "Conectado"
                                           : "Desconectado"));
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("IP: " + WiFi.localIP().toString());
      Serial.println("RSSI: " + String(WiFi.RSSI()) + " dBm");
    }
  } else if (comandoLower == "mqtt") {
    Serial.println("Estado MQTT: " +
                   String(mqttConnected ? "Conectado" : "Desconectado"));
    if (!mqttConnected)
      connectMQTT();
  } else if (comandoLower == "gsm") {
    bool gsmOk = checkGSMModule();
    Serial.println("GSM: " + String(gsmOk ? "OK" : "ERROR"));
  } else if (comandoLower == "gps") {
    Serial.println("Satélites: " + String(gps.satellites.value()));
    Serial.println("Fix: " + String(gps.location.isValid() ? "SI" : "NO"));
    if (gps.location.isValid()) {
      Serial.printf("Lat: %.6f  Lon: %.6f\n", gps.location.lat(),
                    gps.location.lng());
      Serial.printf("Alt: %.1f m  Vel: %.1f km/h\n", gps.altitude.meters(),
                    gps.speed.kmph());
    }
    Serial.println("Chars procesados: " + String(gps.charsProcessed()));
  } else if (comandoLower == "gps_debug" || comandoLower == "gps_diagnostic")
    diagnosticarGPSCompleto();
  else if (comandoLower == "gps_raw") {
    Serial.println("Mostrando datos RAW GPS por 5 segundos...");
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
      if (Serial.available()) {
        while (Serial.available())
          Serial.read();
        break;
      }
      while (gpsSerial.available())
        Serial.write(gpsSerial.read());
      delay(10);
    }
    Serial.println("\nFin datos RAW GPS");
  } else if (comandoLower == "sms_test") {
    enviarRespuestaSMS(accidentConfig.emergency_phone,
                       "SMS de prueba desde Serial. ID: " + device.chipId);
  } else {
    Serial.println("Comando no reconocido. Escribe 'help' para ver los "
                   "comandos disponibles.");
  }
}

void mostrarMenuSerial() {
  Serial.println("\n=== COMANDOS DISPONIBLES ===");
  Serial.println("help/?          - Este menú");
  Serial.println("config/show     - Configuración actual");
  Serial.println("status          - Publica estado MQTT");
  Serial.println("location        - Ubicación GPS");
  Serial.println("test_sms        - SMS de prueba");
  Serial.println("sms_test        - SMS al número configurado");
  Serial.println("\n=== CONFIGURACIÓN ===");
  Serial.println("set phone [nro] set sensitivity [val]");
  Serial.println("set angle [deg] set delay [ms]");
  Serial.println("set sms [on/off] save");
  Serial.println("\n=== SISTEMA ===");
  Serial.println("wifi mqtt gsm gps gps_debug gps_raw reset");
  Serial.println("============================\n");
}

void cambiarNumeroEmergencia(String nuevoNumero) {
  nuevoNumero.trim();
  if (nuevoNumero.length() < 10) {
    LOG_ERROR("Número demasiado corto");
    return;
  }

  int digitCount = 0;
  for (size_t i = 0; i < nuevoNumero.length(); i++)
    if (isdigit(nuevoNumero[i]) || nuevoNumero[i] == '+')
      digitCount++;
  if (digitCount < 10) {
    LOG_ERROR("Número debe tener al menos 10 dígitos");
    return;
  }

  if ((int)nuevoNumero.length() >= MAX_PHONE_LENGTH) {
    LOG_WARN("Número truncado a MAX_PHONE_LENGTH");
    nuevoNumero = nuevoNumero.substring(0, MAX_PHONE_LENGTH - 1);
  }

  String oldNumber = accidentConfig.emergency_phone;
  strncpy(accidentConfig.emergency_phone, nuevoNumero.c_str(),
          MAX_PHONE_LENGTH - 1);
  accidentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';

  Serial.println("Número actualizado: " + oldNumber + " → " +
                 String(accidentConfig.emergency_phone));
  saveConfig();
  publishEvent("PHONE_UPDATED_SERIAL",
               ("Número actualizado: " + nuevoNumero).c_str());
}

void cambiarSensibilidad(String valor) {
  int sens = valor.toInt();
  if (sens >= 500 && sens <= 10000) {
    accidentConfig.sensitivity = sens;
    Serial.println("Sensibilidad: " + String(sens));
    saveConfig();
  } else {
    LOG_ERROR("Sensibilidad debe estar entre 500 y 10000");
  }
}

void cambiarAnguloVuelco(String valor) {
  int angulo = valor.toInt();
  if (angulo >= 10 && angulo <= 90) {
    accidentConfig.rollover_angle = angulo;
    Serial.println("Ángulo vuelco: " + String(angulo) + "°");
    saveConfig();
  } else {
    LOG_ERROR("Ángulo debe estar entre 10 y 90 grados");
  }
}

void cambiarDelayAlerta(String valor) {
  unsigned long delayMs = (unsigned long)valor.toInt();
  if (delayMs >= 1000 && delayMs <= 600000) {
    accidentConfig.alert_delay = delayMs;
    Serial.println("Delay alerta: " + String(delayMs) + "ms");
    saveConfig();
  } else {
    LOG_ERROR("Delay debe estar entre 1000 y 600000 ms");
  }
}

void cambiarSMSHabilitado(String valor) {
  valor.toLowerCase();
  if (valor == "on" || valor == "true" || valor == "1" || valor == "si" ||
      valor == "yes") {
    accidentConfig.sms_enabled = true;
    Serial.println("SMS habilitado");
    saveConfig();
  } else if (valor == "off" || valor == "false" || valor == "0" ||
             valor == "no") {
    accidentConfig.sms_enabled = false;
    Serial.println("SMS deshabilitado");
    saveConfig();
  } else {
    LOG_ERROR("Use 'on' o 'off'");
  }
}

void mostrarConfiguracionActual() {
  Serial.println("\n=== CONFIGURACIÓN ACTUAL ===");
  Serial.println("Número emergencia: " +
                 String(accidentConfig.emergency_phone));
  Serial.println("Sensibilidad: " + String(accidentConfig.sensitivity));
  Serial.println("Ángulo vuelco: " + String(accidentConfig.rollover_angle) +
                 "°");
  Serial.println("Delay alerta: " + String(accidentConfig.alert_delay) + "ms");
  Serial.println("SMS habilitado: " +
                 String(accidentConfig.sms_enabled ? "SI" : "NO"));
  Serial.println("ID Dispositivo: " + device.chipId);
  Serial.println("Versión FW: " + device.firmwareVersion);
  Serial.println("\n=== ESTADO GPS ===");
  Serial.println("Fix válido: " + String(gps.location.isValid() ? "SI" : "NO"));
  Serial.println("Satélites: " + String(gps.satellites.value()));
  Serial.println("Chars procesados: " + String(gps.charsProcessed()));
  if (gps.location.isValid())
    Serial.printf("Ubicación: Lat %.6f, Lon %.6f\n", gps.location.lat(),
                  gps.location.lng());
  Serial.println("===========================\n");
}

bool isBase64Char(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

bool base64DecodeToBuffer(const String &encoded, uint8_t *out, size_t outMax,
                          size_t &outLen) {
  outLen = 0;
  int input_len = encoded.length();
  int i = 0, in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];

  const char *b64chars =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  while (input_len-- && (encoded[in_] != '=') && isBase64Char(encoded[in_])) {
    char_array_4[i++] = encoded[in_++];
    if (i == 4) {
      for (int k = 0; k < 4; k++) {
        const char *p = strchr(b64chars, char_array_4[k]);
        char_array_4[k] = p ? (uint8_t)(p - b64chars) : 0;
      }
      char_array_3[0] =
          (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] =
          ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
      for (int k = 0; k < 3; k++) {
        if (outLen >= outMax)
          return false;
        out[outLen++] = char_array_3[k];
      }
      i = 0;
    }
  }

  if (i) {
    for (int j = i; j < 4; j++)
      char_array_4[j] = 0;
    for (int k = 0; k < 4; k++) {
      const char *p = strchr(b64chars, char_array_4[k]);
      char_array_4[k] = p ? (uint8_t)(p - b64chars) : 0;
    }
    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] =
        ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
    for (int j = 0; j < i - 1; j++) {
      if (outLen >= outMax)
        return false;
      out[outLen++] = char_array_3[j];
    }
  }
  return true;
}

void procesarOTAChunk(const String &message) {
  if (ESP.getFreeHeap() < 20000) {
    LOG_ERROR("Memoria insuficiente para OTA");
    return;
  }

  DynamicJsonDocument doc(8192);
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.printf("Error parseando JSON OTA: %s\n", error.c_str());
    return;
  }

  if (!doc.containsKey("EventType") ||
      doc["EventType"] != "UpdateFirmwareDevice")
    return;
  if (!doc.containsKey("Details")) {
    LOG_ERROR("No Details en OTA");
    return;
  }

  JsonObject details = doc["Details"];
  String firmwareVersion = details["FirmwareVersion"].as<String>();
  String base64Part = details["Base64Part"].as<String>();
  int partIndex = details["PartIndex"].as<int>();
  int totalParts = details["TotalParts"].as<int>();
  size_t totalSize = details["TotalSize"].as<size_t>();
  bool isError = details["IsError"] | false;
  String errorMessage = details["ErrorMessage"] | "";

  if (isError) {
    Serial.printf("Error en OTA: %s\n", errorMessage.c_str());
    publishOTAError(errorMessage, firmwareVersion);
    cleanupOTA();
    return;
  }

  if (base64Part.isEmpty() || firmwareVersion.isEmpty()) {
    publishOTAError("Datos OTA incompletos", firmwareVersion);
    cleanupOTA();
    return;
  }

  if (partIndex == 1) {
    if (otaContext.inProgress) {
      LOG_WARN("OTA en progreso, ignorando nuevo inicio");
      return;
    }
    if (firmwareVersion == device.firmwareVersion) {
      publishOTAError("Ya tiene esta versión instalada", firmwareVersion);
      return;
    }
    if (!iniciarOTA(firmwareVersion, totalParts, totalSize))
      return;
  }

  if (!otaContext.inProgress || partIndex != otaContext.currentPart + 1) {
    Serial.printf("Chunk fuera de secuencia. Esperado: %d, Recibido: %d\n",
                  otaContext.currentPart + 1, partIndex);
    publishOTAError("Chunk fuera de secuencia", firmwareVersion);
    cleanupOTA();
    return;
  }

  if (!procesarChunkOTA(base64Part, partIndex)) {
    cleanupOTA();
    return;
  }

  otaContext.currentPart = partIndex;
  int progress = (partIndex * 100) / totalParts;
  publishOTAProgress(progress, firmwareVersion);

  Serial.printf("Chunk %d/%d procesado. Progreso: %d%%, Heap: %d\n", partIndex,
                totalParts, progress, ESP.getFreeHeap());

  if (partIndex == totalParts)
    finalizarOTA();
}

bool iniciarOTA(const String &firmwareVersion, int totalParts,
                size_t totalSize) {
  Serial.printf("Iniciando OTA. Version: %s, Partes: %d, Tamaño: %u\n",
                firmwareVersion.c_str(), totalParts, totalSize);

  if (ESP.getFreeHeap() < 30000) {
    publishOTAError("Memoria insuficiente para iniciar OTA", firmwareVersion);
    return false;
  }

  if (!Update.begin(totalSize, U_FLASH)) {
    String errorMsg = "Error iniciando OTA: ";
    errorMsg += Update.getErrorString();
    publishOTAError(errorMsg, firmwareVersion);
    return false;
  }

  otaContext.inProgress = true;
  otaContext.firmwareVersion = firmwareVersion;
  otaContext.currentPart = 0;
  otaContext.totalParts = totalParts;
  otaContext.startTime = millis();
  otaContext.receivedSize = 0;
  otaContext.totalSize = totalSize;

  publishOTAProgress(0, firmwareVersion);
  LOG_INFO("OTA iniciada en flash");
  return true;
}

bool procesarChunkOTA(const String &base64Data, int partIndex) {
  if (!otaContext.inProgress) {
    LOG_ERROR("OTA no iniciada");
    return false;
  }

  size_t estimatedSize = (base64Data.length() * 3) / 4 + 4;
  uint8_t *buffer = (uint8_t *)malloc(estimatedSize);
  if (!buffer) {
    publishOTAError("Sin memoria para chunk OTA", otaContext.firmwareVersion);
    return false;
  }

  size_t decodedLen = 0;
  if (!base64DecodeToBuffer(base64Data, buffer, estimatedSize, decodedLen) ||
      decodedLen == 0) {
    free(buffer);
    publishOTAError("Error decodificando Base64", otaContext.firmwareVersion);
    return false;
  }

  size_t written = Update.write(buffer, decodedLen);
  free(buffer);

  if (written != decodedLen) {
    String errorMsg = "Error escribiendo chunk: " + String(written) + " de " +
                      String(decodedLen) + " bytes";
    publishOTAError(errorMsg, otaContext.firmwareVersion);
    return false;
  }

  otaContext.receivedSize += written;

  if (millis() - otaContext.startTime > OTA_TIMEOUT) {
    publishOTAError("Timeout en OTA", otaContext.firmwareVersion);
    return false;
  }

  return true;
}

void finalizarOTA() {
  LOG_INFO("Finalizando OTA...");

  if (otaContext.receivedSize < 1000) {
    publishOTAError("Firmware demasiado pequeño", otaContext.firmwareVersion);
    cleanupOTA();
    return;
  }

  if (!Update.end()) {
    String errorMsg = "Error finalizando OTA: ";
    errorMsg += Update.getErrorString();
    publishOTAError(errorMsg, otaContext.firmwareVersion);
    cleanupOTA();
    return;
  }

  if (Update.isFinished()) {
    LOG_INFO("OTA completada exitosamente!");
    publishOTASuccess(otaContext.firmwareVersion);
    Serial.println("Reiniciando en 3 segundos...");
    delay(3000);
    ESP.restart();
  } else {
    publishOTAError("OTA no se completó correctamente",
                    otaContext.firmwareVersion);
    cleanupOTA();
  }
}

void cleanupOTA() {
  if (otaContext.inProgress) {
    Update.end();
    LOG_WARN("OTA abortada");
  }
  otaContext.inProgress = false;
  otaContext.currentPart = 0;
  otaContext.totalParts = 0;
  otaContext.receivedSize = 0;
  otaContext.totalSize = 0;
  otaContext.startTime = 0;
}

void publishOTAProgress(int progress, const String &firmwareVersion) {
  if (!mqttConnected)
    return;
  DynamicJsonDocument doc(1024);
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = "connected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = device.firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["FirmwareVersion"] = firmwareVersion;
  details["Progress"] = progress;
  details["IsError"] = false;
  details["ErrorMessage"] = "";
  String output;
  serializeJson(doc, output);
  publishMQTTMessage(FPSTR(MQTT_TOPIC_EVENTS), output);
  if (progress % 25 == 0 || progress == 100)
    Serial.printf("Progreso OTA: %d%%\n", progress);
}

void publishOTAError(const String &errorMessage,
                     const String &firmwareVersion) {
  if (!mqttConnected)
    return;
  DynamicJsonDocument doc(1024);
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = "connected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = device.firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["FirmwareVersion"] = firmwareVersion.isEmpty()
                                   ? device.firmwareVersion.c_str()
                                   : firmwareVersion.c_str();
  details["IsError"] = true;
  details["ErrorMessage"] = errorMessage;
  details["Progress"] = 0;
  String output;
  serializeJson(doc, output);
  publishMQTTMessage(FPSTR(MQTT_TOPIC_EVENTS), output);
  Serial.printf("Error OTA: %s\n", errorMessage.c_str());
}

void publishOTASuccess(const String &firmwareVersion) {
  if (!mqttConnected)
    return;
  DynamicJsonDocument doc(1024);
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = "connected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["FirmwareVersion"] = firmwareVersion;
  details["IsError"] = false;
  details["ErrorMessage"] = "";
  details["Progress"] = 100;
  String output;
  serializeJson(doc, output);
  publishMQTTMessage(FPSTR(MQTT_TOPIC_EVENTS), output);
  LOG_INFO("Éxito OTA publicado");
}

void initializeGSM() {
  LOG_INFO("Inicializando módulo GSM...");
  delay(3000);
  if (!checkGSMModule()) {
    LOG_ERROR("No se puede comunicar con módulo GSM");
    return;
  }
  sendATCommand("ATE0", "OK", 2000);
  sendATCommand("AT+CMGF=1", "OK", 2000);
  sendATCommand("AT+CNMI=2,2,0,0,0", "OK", 2000);
  LOG_INFO("Módulo GSM inicializado para recibir SMS");
}

void checkGSMStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 30000) {
    if (!checkGSMModule()) {
      LOG_WARN("Reiniciando comunicación GSM...");
      initializeGSM();
    }
    lastCheck = millis();
  }
}

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);

  AccidentConfig defaultConfig;

  char marker[4];
  for (int i = 0; i < 4; i++)
    marker[i] = EEPROM.read(CONFIG_EEPROM_ADDR + i);

  bool configLoadedFromEEPROM = false;

  if (strncmp(marker, "CFG", 3) == 0) {
    int addr = CONFIG_EEPROM_ADDR + 4;

    uint16_t savedSensitivity =
        (uint16_t)EEPROM.read(addr) | ((uint16_t)EEPROM.read(addr + 1) << 8);
    addr += 2;

    int savedRolloverAngle = EEPROM.read(addr);
    addr += 1;

    unsigned long savedAlertDelay = 0;
    for (int i = 0; i < 4; i++)
      savedAlertDelay |= (unsigned long)EEPROM.read(addr + i) << (8 * i);
    addr += 4;

    bool savedSmsEnabled = EEPROM.read(addr);
    addr += 1;

    char savedPhone[MAX_PHONE_LENGTH] = "";
    bool validPhone = true;
    for (int i = 0; i < MAX_PHONE_LENGTH - 1; i++) {
      char ch = (char)EEPROM.read(addr + i);
      if (ch == 0 || ch == (char)0xFF)
        break;
      if (!(isdigit(ch) || ch == '+' || ch == ' ' || ch == '-' || ch == '(' ||
            ch == ')')) {
        validPhone = false;
        break;
      }
      savedPhone[i] = ch;
    }

    if ((int)savedSensitivity >= 500 && (int)savedSensitivity <= 10000 &&
        savedRolloverAngle >= 10 && savedRolloverAngle <= 90 &&
        savedAlertDelay >= 1000 && savedAlertDelay <= 600000 && validPhone &&
        strlen(savedPhone) >= 10) {
      accidentConfig.sensitivity = (int)savedSensitivity;
      accidentConfig.rollover_angle = savedRolloverAngle;
      accidentConfig.alert_delay = savedAlertDelay;
      accidentConfig.sms_enabled = savedSmsEnabled;
      strncpy(accidentConfig.emergency_phone, savedPhone, MAX_PHONE_LENGTH - 1);
      accidentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
      configLoadedFromEEPROM = true;
      LOG_INFO("Configuración cargada de EEPROM");
    } else {
      LOG_WARN("Configuración en EEPROM inválida, usando valores por defecto");
    }
  }

  if (!configLoadedFromEEPROM) {
    accidentConfig = defaultConfig;
    LOG_INFO("Usando configuración por defecto de config.h");
    saveConfig();
  }

  Serial.println("Configuración actual:");
  Serial.printf("  Sensibilidad: %d\n", accidentConfig.sensitivity);
  Serial.printf("  Ángulo vuelco: %d\n", accidentConfig.rollover_angle);
  Serial.printf("  Delay alerta: %lu\n", accidentConfig.alert_delay);
  Serial.printf("  SMS habilitado: %s\n",
                accidentConfig.sms_enabled ? "SI" : "NO");
  Serial.printf("  Teléfono: %s\n", accidentConfig.emergency_phone);
}

void saveConfig() {
  EEPROM.begin(EEPROM_SIZE);

  int addr = CONFIG_EEPROM_ADDR;
  EEPROM.write(addr, 'C');
  EEPROM.write(addr + 1, 'F');
  EEPROM.write(addr + 2, 'G');
  EEPROM.write(addr + 3, '\0');
  addr += 4;

  EEPROM.write(addr, accidentConfig.sensitivity & 0xFF);
  EEPROM.write(addr + 1, (accidentConfig.sensitivity >> 8) & 0xFF);
  addr += 2;

  EEPROM.write(addr, accidentConfig.rollover_angle);
  addr += 1;

  for (int i = 0; i < 4; i++)
    EEPROM.write(addr + i, (accidentConfig.alert_delay >> (8 * i)) & 0xFF);
  addr += 4;

  EEPROM.write(addr, accidentConfig.sms_enabled);
  addr += 1;

  // BUG-05: Limitar escritura del teléfono a MAX_PHONE_LENGTH
  int phoneLen =
      min((int)strlen(accidentConfig.emergency_phone), MAX_PHONE_LENGTH - 1);
  for (int i = 0; i < phoneLen; i++)
    EEPROM.write(addr + i, accidentConfig.emergency_phone[i]);
  EEPROM.write(addr + phoneLen, 0); // null terminator garantizado

  EEPROM.commit();
  LOG_INFO("Configuración guardada en EEPROM");
}

void checkAccidents() {
  if (millis() - lastAccidentCheck < ACCIDENT_CHECK_INTERVAL)
    return;
  lastAccidentCheck = millis();

  mpu.getAcceleration(&ax, &ay, &az);

  deltx = ax - oldx;
  delty = ay - oldy;
  deltz = az - oldz;

  oldx = ax;
  oldy = ay;
  oldz = az;

  vibration--;
  if (vibration < 0)
    vibration = 0;
  if (vibration > 0)
    return;

  float rawMag = sqrtf((float)sq(deltx) + (float)sq(delty) + (float)sq(deltz));

  acc_history[acc_idx % ACC_FILTER_SIZE] = rawMag;
  acc_idx++;
  float avgMag = 0.0f;
  for (int i = 0; i < ACC_FILTER_SIZE; i++)
    avgMag += acc_history[i];
  avgMag /= ACC_FILTER_SIZE;

  magnitude = avgMag;

  AccidentData accident;

  if (rawMag >= (float)accidentConfig.sensitivity) {
    accident.impact_detected = true;
    accident.impact_magnitude = rawMag;
    updateflag = 1;
    vibration = devibrate;
    Serial.println("Choque detectado!");
  }

  float axf = ax / 16384.0f;
  float ayf = ay / 16384.0f;
  float azf = az / 16384.0f;

  float roll = atan2f(ayf, sqrtf(axf * axf + azf * azf)) * 180.0f / (float)PI;
  float pitch = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * 180.0f / (float)PI;

  accident.roll_angle = roll;
  accident.pitch_angle = pitch;

  if (fabsf(roll) > accidentConfig.rollover_angle ||
      fabsf(pitch) > accidentConfig.rollover_angle) {
    accident.rollover_detected = true;
    updateflag = 1;
    Serial.printf("Rollover detectado! roll=%.1f pitch=%.1f\n", roll, pitch);
  }

  if (accident.impact_detected || accident.rollover_detected) {
    if (millis() - lastAccidentEvent > ACCIDENT_COOLDOWN) {
      lastAccidentEvent = millis();
      accident.detection_time = millis();

      GPSData gpsData;
      readGPSData(gpsData);

      publishAccidentEvent(accident, gpsData);

      digitalWrite(BUZZER_PIN, HIGH);
      alert_triggered = true;
      alert_time = millis();
    } else {
      LOG_WARN("Accidente detectado dentro del cooldown, ignorando");
    }
  }
}

void sendSMSAlert(const AccidentData &accident, const GPSData &gps) {
  if (!accidentConfig.sms_enabled ||
      strlen(accidentConfig.emergency_phone) < 10) {
    LOG_WARN("SMS deshabilitado o número no configurado");
    return;
  }

  LOG_INFO("=== INICIANDO ENVÍO DE SMS ===");

  if (!checkGSMModule()) {
    LOG_ERROR("Módulo GSM no responde");
    publishEvent("SMS_ERROR", "Módulo GSM no responde");
    return;
  }

  String sms_data = "ALERTA ACCIDENTE\r\n";
  if (accident.impact_detected)
    sms_data += "Tipo: IMPACTO\r\n";
  if (accident.rollover_detected)
    sms_data += "Tipo: VUELCO\r\n";

  if (gps.isValid) {
    sms_data += "Ubic: http://maps.google.com/maps?q=loc:";
    sms_data += String(gps.latitude, 6) + "," + String(gps.longitude, 6);
    sms_data += "\r\n";
  } else {
    sms_data += "GPS: No disponible\r\n";
  }

  sms_data += "Hora: " + getISOTimestamp();
  sms_data += "\r\nID: " + device.chipId;

  if (!sendATCommand("AT+CMGF=1", "OK", 5000)) {
    LOG_ERROR("No se pudo configurar modo texto");
    return;
  }
  if (!sendATCommand("AT+CSCS=\"GSM\"", "OK", 3000)) {
    LOG_ERROR("No se pudo configurar codificación");
    return;
  }

  String phoneNumber = accidentConfig.emergency_phone;
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
  String response = "";

  while (millis() - timeout < 10000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
      Serial.write(c);
      if (c == '>') {
        gotPrompt = true;
        break;
      }
    }
    if (gotPrompt)
      break;
    delay(100);
  }

  if (!gotPrompt) {
    LOG_ERROR("No se recibió prompt >");
    return;
  }

  sim800.println(sms_data);
  delay(2000);
  sim800.write(0x1A);

  timeout = millis();
  bool smsSent = false;
  response = "";

  while (millis() - timeout < 15000) {
    ESP.wdtFeed();
    yield();
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
      Serial.write(c);
      if (response.indexOf("OK") >= 0 || response.indexOf("+CMGS") >= 0) {
        smsSent = true;
        break;
      }
      if (response.indexOf("ERROR") >= 0)
        break;
    }
    if (smsSent || response.indexOf("ERROR") >= 0)
      break;
    delay(100);
  }

  if (smsSent) {
    LOG_INFO("SMS de emergencia enviado correctamente");
    publishEvent("SMS_SENT", "Alerta enviada exitosamente");
  } else {
    LOG_ERROR("No se confirmó el envío del SMS");
    publishEvent("SMS_ERROR", "Fallo en envío de SMS");
  }

  while (sim800.available())
    sim800.read();
  LOG_INFO("FIN ENVÍO SMS");
}

bool checkGSMModule() {
  Serial.println("Verificando módulo GSM...");
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

  Serial.println(responded ? "Módulo GSM responde correctamente"
                           : "Módulo GSM NO responde");
  return responded;
}

bool sendATCommand(String command, String expectedResponse,
                   unsigned long timeoutMs) {
  Serial.println("Enviando: " + command);
  while (sim800.available())
    sim800.read();
  sim800.println(command);

  unsigned long startTime = millis();
  String response = "";

  while (millis() - startTime < timeoutMs) {
    if (sim800.available()) {
      char c = sim800.read();
      response += c;
      if (response.indexOf(expectedResponse) >= 0) {
        Serial.println("Comando exitoso");
        return true;
      }
    }
    delay(10);
  }

  Serial.println("Timeout: " + command);
  Serial.println("Respuesta: " + response);
  return false;
}

void publishAccidentEvent(const AccidentData &accident, const GPSData &gps) {
  StaticJsonDocument<768> doc;
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = mqttConnected ? "connected" : "disconnected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["IPAddress"] = WiFi.localIP().toString();
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = device.firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["chipId"] = device.chipId;
  JsonObject accObj = details.createNestedObject("accident");
  accObj["impact_detected"] = accident.impact_detected;
  accObj["rollover_detected"] = accident.rollover_detected;
  accObj["impact_magnitude"] = serialized(String(accident.impact_magnitude, 1));
  accObj["roll_angle"] = serialized(String(accident.roll_angle, 1));
  accObj["pitch_angle"] = serialized(String(accident.pitch_angle, 1));
  JsonObject gpsObj = accObj.createNestedObject("gps_data");
  gpsObj["latitude"] = serialized(String(gps.latitude, 6));
  gpsObj["longitude"] = serialized(String(gps.longitude, 6));
  gpsObj["is_valid"] = gps.isValid;
  JsonObject cfgObj = details.createNestedObject("config");
  cfgObj["sensitivity"] = accidentConfig.sensitivity;
  cfgObj["rollover_angle"] = accidentConfig.rollover_angle;
  cfgObj["alert_delay"] = accidentConfig.alert_delay;
  cfgObj["sms_enabled"] = accidentConfig.sms_enabled;

  char output[768];
  serializeJson(doc, output, sizeof(output));
  publishMQTTMessage(FPSTR(MQTT_TOPIC_EVENTS), output);
  LOG_INFO("Evento de accidente publicado por MQTT");

  if (accidentConfig.sms_enabled)
    sendSMSAlert(accident, gps);
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  String topicStr = String(topic);
  String message;
  for (unsigned int i = 0; i < length; i++)
    message += (char)payload[i];

  Serial.printf("Tópico recibido: %s\n", topicStr.c_str());

  String deviceEventTopic = "event/" + device.chipId;
  if (topicStr == deviceEventTopic)
    procesarComandoMQTT(message);
  else if (topicStr == FPSTR(MQTT_TOPIC_CONFIG))
    procesarConfiguracion(message);
}

void procesarConfiguracion(const String &message) {
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.printf("Error parseando configuración: %s\n", error.c_str());
    return;
  }

  bool configChanged = false;

  if (doc.containsKey("sensitivity")) {
    accidentConfig.sensitivity = doc["sensitivity"];
    configChanged = true;
  }
  if (doc.containsKey("rollover_angle")) {
    accidentConfig.rollover_angle = doc["rollover_angle"];
    configChanged = true;
  }
  if (doc.containsKey("alert_delay")) {
    accidentConfig.alert_delay = doc["alert_delay"];
    configChanged = true;
  }
  if (doc.containsKey("sms_enabled")) {
    accidentConfig.sms_enabled = doc["sms_enabled"];
    configChanged = true;
  }
  if (doc.containsKey("emergency_phone")) {
    const char *phone = doc["emergency_phone"];
    if (phone) {
      strncpy(accidentConfig.emergency_phone, phone, MAX_PHONE_LENGTH - 1);
      accidentConfig.emergency_phone[MAX_PHONE_LENGTH - 1] = '\0';
      configChanged = true;
    }
  }

  if (configChanged) {
    saveConfig();
    publishEvent("CONFIG_UPDATED", "Configuración actualizada exitosamente");
  }
}

void procesarComandoMQTT(const String &message) {
  if (message == "reset") {
    publishEvent("SYSTEM_RESET", "Reinicio del sistema solicitado");
    delay(1000);
    ESP.restart();
  } else if (message == "status") {
    publishStatus();
  } else if (message == "location") {
    GPSData data;
    readGPSData(data);
    publishGPSLocation(data);
  } else if (message == "get_config") {
    StaticJsonDocument<256> doc;
    doc["sensitivity"] = accidentConfig.sensitivity;
    doc["rollover_angle"] = accidentConfig.rollover_angle;
    doc["alert_delay"] = accidentConfig.alert_delay;
    doc["sms_enabled"] = accidentConfig.sms_enabled;
    doc["emergency_phone"] = accidentConfig.emergency_phone;
    char output[256];
    serializeJson(doc, output, sizeof(output));
    publishMQTTMessage(FPSTR(MQTT_TOPIC_CONFIG), output);
  } else {
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, message);
    if (!error && doc.containsKey("EventType")) {
      String eventType = doc["EventType"].as<String>();
      if (eventType == "UpdateFirmwareDevice")
        procesarOTAChunk(message);
    }
  }
}

void readGPSData(GPSData &data) {
  data.isValid = gps.location.isValid();
  if (data.isValid) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.meters();
    data.speed = gps.speed.kmph();
    data.course = gps.course.deg();
    data.satellites = gps.satellites.value();
    data.timestamp = getISOTimestamp();
  } else {
    static unsigned long lastGPSWarning = 0;
    if (millis() - lastGPSWarning > 30000) {
      Serial.printf("GPS sin fix — Satélites: %d\n", gps.satellites.value());
      lastGPSWarning = millis();
    }
  }
}

void publishGPSLocation(const GPSData &data) {
  StaticJsonDocument<512> doc;
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = mqttConnected ? "connected" : "disconnected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["IPAddress"] = WiFi.localIP().toString();
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = device.firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["chipId"] = device.chipId;
  JsonObject gpsObj = details.createNestedObject("gps_data");
  gpsObj["latitude"] = serialized(String(data.latitude, 6));
  gpsObj["longitude"] = serialized(String(data.longitude, 6));
  gpsObj["altitude"] = serialized(String(data.altitude, 1));
  gpsObj["speed"] = serialized(String(data.speed, 1));
  gpsObj["course"] = serialized(String(data.course, 1));
  gpsObj["satellites"] = data.satellites;
  gpsObj["is_valid"] = data.isValid;
  gpsObj["timestamp"] = data.timestamp;
  char output[512];
  serializeJson(doc, output, sizeof(output));
  publishMQTTMessage(FPSTR(MQTT_TOPIC_TELEMETRY), output);
}

void publishMQTTMessage(const char *topic, const String &message) {
  if (!mqttConnected || otaInProgress || otaContext.inProgress)
    return;
  bool result = mqtt_client.publish(topic, message.c_str());
  if (!result)
    Serial.printf("Error publicando en %s\n", topic);
}

void publishEvent(const char *evento, const char *descripcion) {
  StaticJsonDocument<512> doc;
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = mqttConnected ? "connected" : "disconnected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["IPAddress"] = WiFi.localIP().toString();
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = device.firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["chipId"] = device.chipId;
  JsonObject info = details.createNestedObject("info");
  info["event"] = evento;
  info["description"] = descripcion;
  info["timestamp"] = (uint32_t)millis();
  char output[512];
  serializeJson(doc, output, sizeof(output));
  publishMQTTMessage(FPSTR(MQTT_TOPIC_EVENTS), output);
}

void publishStatus() {
  StaticJsonDocument<768> doc;
  JsonObject deviceObj = doc.createNestedObject("Device");
  deviceObj["status"] = mqttConnected ? "connected" : "disconnected";
  deviceObj["ChipId"] = device.chipId;
  deviceObj["MacAddress"] = device.macAddress;
  deviceObj["IPAddress"] = WiFi.localIP().toString();
  deviceObj["ChipType"] = device.chipType;
  deviceObj["FirmwareVersion"] = device.firmwareVersion;
  doc["Timestamp"] = getISOTimestamp();
  JsonObject details = doc.createNestedObject("Details");
  details["chipId"] = device.chipId;
  details["wifi_status"] =
      (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
  details["rssi"] = WiFi.RSSI();
  details["free_heap"] = ESP.getFreeHeap();
  details["uptime"] = (uint32_t)millis();
  JsonObject gpsStatus = details.createNestedObject("gps_status");
  gpsStatus["has_fix"] = gps.location.isValid();
  gpsStatus["satellites"] = gps.satellites.value();
  gpsStatus["age"] = (uint32_t)gps.location.age();
  JsonObject cfgObj = details.createNestedObject("accident_config");
  cfgObj["sensitivity"] = accidentConfig.sensitivity;
  cfgObj["rollover_angle"] = accidentConfig.rollover_angle;
  cfgObj["alert_delay"] = accidentConfig.alert_delay;
  cfgObj["sms_enabled"] = accidentConfig.sms_enabled;
  cfgObj["emergency_phone"] = accidentConfig.emergency_phone;
  char output[768];
  serializeJson(doc, output, sizeof(output));
  publishMQTTMessage(FPSTR(MQTT_TOPIC_STATUS), output);
}

bool configurarTiempoNTP() {
  LOG_INFO("Configurando tiempo NTP...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  Serial.print("Obteniendo tiempo desde NTP");
  int intentos = 0;
  while (intentos < 30) {
    delay(1000);
    Serial.print(".");
    time_t now = time(nullptr);
    if (now > 1000000000) {
      Serial.println("\nNTP sincronizado!");
      return true;
    }
    intentos++;
  }
  Serial.println("\nError: No se pudo sincronizar con NTP");
  return false;
}

String getISOTimestamp() {
  time_t now = time(nullptr);

  if (now < 1000000000) {
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "BOOT+%lums", (unsigned long)millis());
    return String(fallback);
  }

  struct tm *timeinfo = localtime(&now);
  if (!timeinfo) {
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "BOOT+%lums", (unsigned long)millis());
    return String(fallback);
  }

  uint32_t ms = (uint32_t)(millis() % 1000UL);
  char timestamp[25];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", timeinfo);
  char fullTimestamp[30];
  snprintf(fullTimestamp, sizeof(fullTimestamp), "%s.%03luZ", timestamp,
           (unsigned long)ms);
  return String(fullTimestamp);
}

void verificarYSincronizarTiempo() {
  static unsigned long ultimaVerificacion = 0;
  if (millis() - ultimaVerificacion > TIME_SYNC_INTERVAL) {
    time_t now = time(nullptr);
    if (now < 1000000000) {
      LOG_WARN("Re-sincronizando tiempo NTP...");
      configurarTiempoNTP();
    }
    ultimaVerificacion = millis();
  }
}

DeviceInfo getDeviceInfo() {
  DeviceInfo info;
  uint32_t chipID = ESP.getChipId();
  char chipIdStr[9];
  snprintf(chipIdStr, sizeof(chipIdStr), "%08X", chipID);
  info.chipId = String(chipIdStr);
  info.chipType = "ESP8266";
  info.macAddress = WiFi.macAddress();
  info.firmwareVersion = FIRMWARE_VERSION;
  info.clientID = "ESP8266_" + info.chipId;
  info.clientID.toUpperCase();
  Serial.printf("Chip ID: %s  MAC: %s  FW: %s\n", info.chipId.c_str(),
                info.macAddress.c_str(), info.firmwareVersion.c_str());
  return info;
}

bool connectMQTT() {
  if (mqttConnected)
    return true;

  Serial.printf("Conectando MQTTS como %s...\n", device.clientID.c_str());

  wifiClient.setCACert(root_ca_pem);

  if (mqtt_client.connect(device.clientID.c_str(), API_KEY, "")) {
    LOG_INFO("MQTTS CONECTADO!");
    mqttConnected = true;
    mqttFailCount = 0;

    String deviceEventTopic = "event/" + device.chipId;
    mqtt_client.subscribe(deviceEventTopic.c_str());
    mqtt_client.subscribe(FPSTR(MQTT_TOPIC_CONFIG));
    Serial.printf("Suscrito a: %s y %s\n", deviceEventTopic.c_str(),
                  FPSTR(MQTT_TOPIC_CONFIG));

    publishStatus();
    publishEvent("MQTT_CONNECTED", "Conexión MQTTS segura establecida");
    return true;
  } else {
    Serial.printf("Error conectando MQTT: %d\n", mqtt_client.state());
    return false;
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(device.clientID.c_str());

  ArduinoOTA.onStart([]() {
    LOG_INFO("Inicio OTA");
    otaInProgress = true;
    publishEvent("OTA_START", "Actualización de firmware OTA iniciada");
  });

  ArduinoOTA.onEnd([]() {
    LOG_INFO("OTA completada");
    otaInProgress = false;
    publishEvent("OTA_END",
                 "Actualización de firmware OTA completada exitosamente");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total > 0)
      Serial.printf("Progreso OTA: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error OTA[%u]: ", error);
    otaInProgress = false;
    const char *errorMsg = "";
    switch (error) {
    case OTA_AUTH_ERROR:
      errorMsg = "Error de autenticación OTA";
      break;
    case OTA_BEGIN_ERROR:
      errorMsg = "Error al iniciar OTA";
      break;
    case OTA_CONNECT_ERROR:
      errorMsg = "Error de conexión OTA";
      break;
    case OTA_RECEIVE_ERROR:
      errorMsg = "Error de recepción OTA";
      break;
    case OTA_END_ERROR:
      errorMsg = "Error al finalizar OTA";
      break;
    }
    publishEvent("OTA_ERROR", errorMsg);
  });

  ArduinoOTA.begin();
  LOG_INFO("OTA configurado");
}

void handleButton() {
  static bool lastButtonState = HIGH;
  static unsigned long buttonPressStart = 0;
  static bool isButtonPressed = false;

  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long nowMs = millis();

  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStart = nowMs;
    isButtonPressed = true;
    Serial.println("Botón presionado - mantenga 3 segundos para modo AP");
  } else if (buttonState == HIGH && lastButtonState == LOW && isButtonPressed) {
    unsigned long pressDuration = nowMs - buttonPressStart;
    isButtonPressed = false;

    if (pressDuration > (unsigned long)(BUTTON_LONG_PRESS_TIME * 1000)) {
      LOG_INFO("Activando modo AP por botón físico...");

      String descripcion = "Modo AP activado - ";
      descripcion +=
          (WiFi.SSID().length() > 0) ? "Con credenciales" : "Sin credenciales";
      publishEvent("AP_MODE_ACTIVATED", descripcion.c_str());

      WiFiManager wifiManager;
      String apSSID = "VESIS-" + String(ESP.getChipId(), HEX);

      String apPassword = WiFi.macAddress();
      apPassword.replace(":", "");
      apPassword = apPassword.substring(6);
      Serial.println("AP SSID: " + apSSID);
      Serial.println("AP Password: " + apPassword);

      wifiManager.setConfigPortalTimeout(180);
      wifiManager.startConfigPortal(apSSID.c_str(), apPassword.c_str());

      LOG_INFO("Reiniciando después de configuración AP...");
      ESP.restart();
    }
  }

  lastButtonState = buttonState;
}

void updateWiFiLED() {
  static bool ledState = LOW;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >=
      (unsigned long)(currentWiFiStatus == WIFI_DISCONNECTED
                          ? FAST_BLINK_INTERVAL
                      : currentWiFiStatus == WIFI_CONNECTING
                          ? SLOW_BLINK_INTERVAL
                          : 0)) {
    previousMillis = currentMillis;
    if (currentWiFiStatus == WIFI_CONNECTED) {
      digitalWrite(wifiLed, HIGH);
    } else {
      ledState = !ledState;
      digitalWrite(wifiLed, ledState);
    }
  }
}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED && currentWiFiStatus != WIFI_CONNECTING) {
    LOG_WARN("Reconectando WiFi...");
    currentWiFiStatus = WIFI_CONNECTING;
    WiFi.begin();
  } else if (WiFi.status() == WL_CONNECTED &&
             currentWiFiStatus != WIFI_CONNECTED) {
    LOG_INFO("WiFi reconectado");
    currentWiFiStatus = WIFI_CONNECTED;
    publishEvent("WIFI_RECONNECTED", "Conexión WiFi restaurada exitosamente");
  }
}

void updateWiFiStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    if (currentWiFiStatus == WIFI_CONNECTED) {
      LOG_WARN("WiFi desconectado");
      currentWiFiStatus = WIFI_DISCONNECTED;
      publishEvent("WIFI_DISCONNECTED", "Conexión WiFi perdida");
    }
    reconnectWiFi();
  }
}

void setupWiFi() {
  LOG_INFO("Iniciando configuración WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  currentWiFiStatus = WIFI_CONNECTING;

  if (WiFi.SSID().length() > 0) {
    Serial.println("Credenciales WiFi encontradas. Conectando...");
    WiFi.begin();

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi conectado! IP: " + WiFi.localIP().toString());
      currentWiFiStatus = WIFI_CONNECTED;
      publishEvent("WIFI_CONNECTED", "Conexión WiFi establecida");
      return;
    } else {
      LOG_WARN("No se pudo conectar con credenciales guardadas");
    }
  } else {
    LOG_WARN("No hay credenciales WiFi guardadas.");
  }

  Serial.println("Use el botón físico (3 segundos) para configurar WiFi");
  currentWiFiStatus = WIFI_DISCONNECTED;
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(wifiLed, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(wifiLed, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  loadConfig();

  Serial.println("\n=== INICIALIZANDO GPS ===");
  gpsSerial.begin(9600);
  delay(2000);

  int bytesAvailable = gpsSerial.available();
  Serial.println("Bytes disponibles en puerto GPS: " + String(bytesAvailable));
  if (bytesAvailable > 0) {
    Serial.println("GPS DETECTADO");
    String initialData = "";
    for (int i = 0; i < min(100, bytesAvailable); i++)
      initialData += (char)gpsSerial.read();
    Serial.println(initialData);
  } else {
    LOG_WARN("GPS NO ENVÍA DATOS — Verifica conexiones");
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();

  bool mpuOk = mpu.testConnection();
  if (!mpuOk) {
    LOG_ERROR("MPU6050 no conectado!");
  } else {
    LOG_INFO("MPU6050 listo — calibrando...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    LOG_INFO("MPU6050 calibrado");
  }

  sim800.begin(9600);
  initializeGSM();

  setupWiFi();

  if (WiFi.status() == WL_CONNECTED)
    configurarTiempoNTP();

  device = getDeviceInfo();

  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
  mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);

  setupOTA();

  Serial.println("\n=================================");
  Serial.println("Sistema de Detección de Accidentes");
  Serial.println("Versión: " + device.firmwareVersion);
  Serial.println("ID: " + device.chipId);
  Serial.println("GPS: " +
                 String(bytesAvailable > 0 ? "DETECTADO" : "NO DETECTADO"));
  Serial.println("MPU6050: " + String(mpuOk ? "OK" : "ERROR"));
  Serial.println("GSM: " + String(checkGSMModule() ? "OK" : "ERROR"));
  Serial.println("=================================\n");

  String authCode = generarCodigoAuth();
  Serial.println(">>> Código SMS AUTH (solo visible acá): " + authCode);

  mostrarMenuSerial();
  mostrarConfiguracionActual();
  publishEvent("SYSTEM_START", "Sistema de detección de accidentes iniciado");
}

void loop() {

  procesarComandoSerial();

  procesarSMSRecibido();

  while (gpsSerial.available() > 0)
    gps.encode(gpsSerial.read());
  checkGPSStatus();

  checkGSMStatus();

  if (alert_triggered && (millis() - alert_time > accidentConfig.alert_delay)) {
    digitalWrite(BUZZER_PIN, LOW);
    alert_triggered = false;
    publishEvent("ALERT_AUTO_OFF",
                 "Buzzer apagado automáticamente por timeout");
  }

  if (alert_triggered && digitalRead(BUTTON_PIN) == LOW) {
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    alert_triggered = false;
    alert_time = 0;
    publishEvent("ALERT_CANCELLED", "Alerta cancelada manualmente");
  }

  if (!otaContext.inProgress)
    checkAccidents();

  updateWiFiLED();
  updateWiFiStatus();
  handleButton();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt_client.connected()) {
      mqttConnected = false;
      unsigned long now = millis();

      unsigned long reconnectDelay = min(
          MQTT_RECONNECT_INTERVAL * (1UL << (unsigned)mqttFailCount), 300000UL);
      reconnectDelay += (unsigned long)random(0, 2000);
      if (now - lastReconnectAttempt > reconnectDelay) {
        lastReconnectAttempt = now;
        if (connectMQTT())
          mqttFailCount = 0;
        else
          mqttFailCount = min(mqttFailCount + 1, 5);
      }
    } else {
      mqtt_client.loop();
    }
  }

  static unsigned long lastGPSPublish = 0;
  if (millis() - lastGPSPublish > GPS_PUBLISH_INTERVAL &&
      !otaContext.inProgress) {
    GPSData data;
    readGPSData(data);
    if (data.isValid)
      publishGPSLocation(data);
    lastGPSPublish = millis();
  }

  static unsigned long lastStatusPublish = 0;
  if (millis() - lastStatusPublish > STATUS_PUBLISH_INTERVAL &&
      !otaContext.inProgress) {
    publishStatus();
    lastStatusPublish = millis();
  }

  verificarYSincronizarTiempo();

  ArduinoOTA.handle();

  if (otaContext.inProgress &&
      (millis() - otaContext.startTime > OTA_TIMEOUT)) {
    publishOTAError("Timeout en actualización OTA", otaContext.firmwareVersion);
    cleanupOTA();
    LOG_WARN("OTA timeout - actualización cancelada");
  }

  delay(10);
}