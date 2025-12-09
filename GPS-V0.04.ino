#include "config.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <FS.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PubSubClient.h>
#include <base64.h>

extern "C" {
#include "user_interface.h"
}

String MQTT_TOPIC_TELEMETRY = "telemetry";
String MQTT_TOPIC_EVENTS = "events";
String MQTT_TOPIC_COMMANDS = "commands";
String MQTT_TOPIC_OTA = "ota";
String MQTT_TOPIC_STATUS = "status";
String MQTT_TOPIC_CONFIG = "config";

struct DeviceInfo {
  String chipId;
  String chipType;
  String macAddress;
  String clientID;
  String firmwareVersion;
};

struct GPSData {
  float latitude = 0.0;
  float longitude = 0.0;
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
  float impact_magnitude = 0.0;
  float roll_angle = 0.0;
  float pitch_angle = 0.0;
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
unsigned long lastReconnectAttempt = 0;
bool alert_triggered = false;
unsigned long alert_time = 0;

int16_t ax, ay, az;
int16_t oldx = 0, oldy = 0, oldz = 0;
int deltx = 0, delty = 0, deltz = 0;
int vibration = 2, devibrate = 75;
int magnitude = 0;
byte updateflag = 0;
unsigned long lastAccidentCheck = 0;

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

enum WiFiStatus {
  WIFI_DISCONNECTED,
  WIFI_CONNECTING,
  WIFI_CONNECTED
};

WiFiStatus currentWiFiStatus = WIFI_DISCONNECTED;
unsigned long previousMillis = 0;


void procesarOTAChunk(const String& message);
bool iniciarOTA(const String& firmwareVersion, int totalParts, size_t totalSize);
bool procesarChunkOTA(const String& base64Data, int partIndex);
void finalizarOTA();
void cleanupOTA();
void publishOTAProgress(int progress, const String& firmwareVersion);
void publishOTAError(const String& errorMessage, const String& firmwareVersion = "");
void publishOTASuccess(const String& firmwareVersion);
String base64Decode(const String& encoded);

String getMQTTClientID();
bool configurarTiempoNTP();
String getISOTimestamp();
void verificarYSincronizarTiempo();
DeviceInfo getDeviceInfo();
void publishGPSLocation(const GPSData& data);
void publishAccidentEvent(const AccidentData& accident, const GPSData& gps);
void publishEvent(const char* evento, const char* descripcion);
void publishStatus();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
bool connectMQTT();
void setupOTA();
void updateWiFiLED();
void reconnectWiFi();
void updateWiFiStatus();
void setupWiFi();
void handleButton();
void procesarComandoMQTT(const String& message);
void readGPSData(GPSData& data);
void publishMQTTMessage(const char* topic, const String& message);
void checkAccidents();
void sendSMSAlert(const AccidentData& accident, const GPSData& gps);
void loadConfig();
void saveConfig();
void procesarConfiguracion(const String& message);
void initializeGSM();
bool checkGSMModule();
bool sendATCommand(String command, String expectedResponse, unsigned long timeoutMs);
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

template<typename T>
T min_val(T a, T b) {
    return (a < b) ? a : b;
}



void procesarSMSRecibido() {
    static String buffer = "";
    static bool enSMS = false;
    static String numero = "";
    static String fecha = "";
    
    while (sim800.available()) {
        char c = sim800.read();
        buffer += c;
        
        
        if (buffer.indexOf("+CMT:") >= 0 && !enSMS) {
            enSMS = true;
           
            int inicioNum = buffer.indexOf("\"", buffer.indexOf("+CMT:")) + 1;
            int finNum = buffer.indexOf("\"", inicioNum);
            if (inicioNum > 0 && finNum > inicioNum) {
                numero = buffer.substring(inicioNum, finNum);
            }
            
           
            int inicioFecha = buffer.indexOf("\"", finNum + 1) + 1;
            int finFecha = buffer.indexOf("\"", inicioFecha);
            if (inicioFecha > 0 && finFecha > inicioFecha) {
                fecha = buffer.substring(inicioFecha, finFecha);
            }
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
                if (len > maxLen) len = maxLen;
                String evento = "SMS_RECEIVED: " + mensaje.substring(0, len);
                publishEvent("SMS_RECEIVED", evento.c_str());
            }
            
            
            buffer = "";
            enSMS = false;
            numero = "";
            fecha = "";
        }
    }
}

void enviarRespuestaSMS(String numero, String mensaje) {
    if (!checkGSMModule()) {
        Serial.println("ERROR: No se puede enviar SMS, módulo GSM no responde");
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
        while (sim800.available()) {
            char c = sim800.read();
            response += c;
            if (c == '>') {
                gotPrompt = true;
                break;
            }
        }
        if (gotPrompt) break;
        delay(100);
    }
    
    if (!gotPrompt) {
        Serial.println("ERROR: No se recibió prompt > para SMS");
        return;
    }
    
    
    sim800.println(mensaje);
    delay(2000);
    
 
    sim800.write(0x1A);
    
    
    timeout = millis();
    bool smsSent = false;
    response = "";
    
    while (millis() - timeout < 15000) {
        while (sim800.available()) {
            char c = sim800.read();
            response += c;
            if (response.indexOf("OK") >= 0 || response.indexOf("+CMGS") >= 0) {
                smsSent = true;
                break;
            }
            if (response.indexOf("ERROR") >= 0) {
                break;
            }
        }
        if (smsSent) break;
        delay(100);
    }
    
    if (smsSent) {
        Serial.println("Respuesta SMS enviada exitosamente");
        publishEvent("SMS_RESPONSE_SENT", "Respuesta SMS enviada");
    } else {
        Serial.println("ERROR: No se pudo enviar respuesta SMS");
    }
}

void procesarComandoSMS(String numero, String comando) {
    comando.trim();
    
    if (comando.length() == 0) {
        enviarRespuestaSMS(numero, "Comando vacío. Envía 'HELP' para ver opciones.");
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
        String codigo = comando.substring(5);
        
        if (codigo == "1234" || codigo == device.chipId.substring(0, 4)) {
            enviarRespuestaSMS(numero, "¡Autorizado! Ahora puedes enviar comandos.");
            
            String desc1 = "Número autorizado por SMS: " + numero;
            publishEvent("SMS_AUTHORIZED", desc1.c_str());
            return;
        } else {
            enviarRespuestaSMS(numero, "Código incorrecto.");
            return;
        }
    }
    
    
    if (!numeroEntrante.endsWith(numeroAutorizado) && 
        numeroAutorizado.length() >= 10 && 
        numeroEntrante.length() >= 10) {
        
        
        String numEntranteSinPrefijo = numeroEntrante;
        if (numEntranteSinPrefijo.length() > 10) {
            numEntranteSinPrefijo = numEntranteSinPrefijo.substring(numEntranteSinPrefijo.length() - 10);
        }
        
        String numAutorizadoSinPrefijo = numeroAutorizado;
        if (numAutorizadoSinPrefijo.length() > 10) {
            numAutorizadoSinPrefijo = numAutorizadoSinPrefijo.substring(numAutorizadoSinPrefijo.length() - 10);
        }
        
        if (numEntranteSinPrefijo != numAutorizadoSinPrefijo) {
            enviarRespuestaSMS(numero, "No autorizado. Usa AUTH [código] o configura este número como emergencia.");
            String desc2 = "Intento no autorizado de: " + numero;
            publishEvent("SMS_UNAUTHORIZED", desc2.c_str());
            return;
        }
    }
    
    
    if (comandoUpper == "HELP" || comandoUpper == "AYUDA" || comandoUpper == "?") {
        enviarRespuestaSMS(numero, crearMenuSMS());
    }
    else if (comandoUpper == "STATUS" || comandoUpper == "ESTADO") {
        String respuesta = "ESTADO DEL SISTEMA\n";
        respuesta += "ID: " + device.chipId + "\n";
        respuesta += "FW: " + device.firmwareVersion + "\n";
        respuesta += "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") + "\n";
        respuesta += "MQTT: " + String(mqttConnected ? "Conectado" : "Desconectado") + "\n";
        respuesta += "GPS: " + String(gps.location.isValid() ? "Con Fix" : "Sin Fix") + "\n";
        respuesta += "Satélites: " + String(gps.satellites.value()) + "\n";
        respuesta += "Memoria: " + String(ESP.getFreeHeap()) + " bytes\n";
        
        
        publishStatus();
        
        enviarRespuestaSMS(numero, respuesta);
    }
    else if (comandoUpper == "LOCATION" || comandoUpper == "UBICACION" || comandoUpper == "LOC") {
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
            respuesta += "GPS sin fix\n";
            respuesta += "Satélites: " + String(gps.satellites.value()) + "\n";
            respuesta += "Espere unos minutos...";
        }
        
        enviarRespuestaSMS(numero, respuesta);
    }
    else if (comandoUpper == "CONFIG" || comandoUpper == "CONFIGURACION") {
        String respuesta = "CONFIGURACIÓN ACTUAL\n";
        respuesta += "Tel emerg: " + accidentConfig.emergency_phone + "\n";
        respuesta += "Sensibilidad: " + String(accidentConfig.sensitivity) + "\n";
        respuesta += "Ángulo vuelco: " + String(accidentConfig.rollover_angle) + "°\n";
        respuesta += "Delay alerta: " + String(accidentConfig.alert_delay / 1000) + "s\n";
        respuesta += "SMS: " + String(accidentConfig.sms_enabled ? "Activado" : "Desactivado") + "\n";
        
        enviarRespuestaSMS(numero, respuesta);
    }
    else if (comandoUpper == "TEST" || comandoUpper == "TEST_SMS") {
        enviarRespuestaSMS(numero, "Enviando SMS de prueba...");
        
        AccidentData testAccident;
        GPSData testGps;
        testAccident.impact_detected = true;
        testAccident.impact_magnitude = 2500;
        testGps.latitude = 18.735693;
        testGps.longitude = -70.162651;
        testGps.isValid = true;
        
        sendSMSAlert(testAccident, testGps);
        enviarRespuestaSMS(numero, "SMS de prueba enviado al número de emergencia.");
    }
    else if (comandoUpper.startsWith("SET PHONE ")) {
        String nuevoNumero = comando.substring(10);
        cambiarNumeroEmergencia(nuevoNumero);
        enviarRespuestaSMS(numero, "Número actualizado a: " + nuevoNumero);
    }
    else if (comandoUpper.startsWith("SET SENS ")) {
        String valor = comando.substring(9);
        cambiarSensibilidad(valor);
        enviarRespuestaSMS(numero, "Sensibilidad actualizada a: " + valor);
    }
    else if (comandoUpper.startsWith("SET ANGLE ")) {
        String valor = comando.substring(10);
        cambiarAnguloVuelco(valor);
        enviarRespuestaSMS(numero, "Ángulo vuelco actualizado a: " + valor + "°");
    }
    else if (comandoUpper.startsWith("SET DELAY ")) {
        String valor = comando.substring(10);
        cambiarDelayAlerta(valor);
        enviarRespuestaSMS(numero, "Delay alerta actualizado a: " + valor + "ms");
    }
    else if (comandoUpper.startsWith("SET SMS ")) {
        String valor = comando.substring(8);
        cambiarSMSHabilitado(valor);
        enviarRespuestaSMS(numero, "SMS " + String(accidentConfig.sms_enabled ? "activado" : "desactivado"));
    }
    else if (comandoUpper == "SAVE" || comandoUpper == "GUARDAR") {
        saveConfig();
        enviarRespuestaSMS(numero, "Configuración guardada en EEPROM");
    }
    else if (comandoUpper == "RESET" || comandoUpper == "REINICIAR") {
        enviarRespuestaSMS(numero, "Reiniciando dispositivo...");
        delay(2000);
        ESP.restart();
    }
    else if (comandoUpper == "WIFI" || comandoUpper == "WIFI INFO") {
        String respuesta = "INFORMACIÓN WiFi\n";
        respuesta += "SSID: " + WiFi.SSID() + "\n";
        respuesta += "Estado: " + String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") + "\n";
        if (WiFi.status() == WL_CONNECTED) {
            respuesta += "IP: " + WiFi.localIP().toString() + "\n";
            respuesta += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
        }
        
        enviarRespuestaSMS(numero, respuesta);
    }
    else if (comandoUpper == "GPS") {
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
    }
    else if (comandoUpper == "GSM") {
        bool gsmOk = checkGSMModule();
        enviarRespuestaSMS(numero, "GSM: " + String(gsmOk ? "OK" : "ERROR"));
    }
    else if (comandoUpper == "MQTT") {
        String respuesta = "MQTT: " + String(mqttConnected ? "Conectado" : "Desconectado");
        if (!mqttConnected) {
            connectMQTT();
            respuesta += "\nIntentando reconectar...";
        }
        enviarRespuestaSMS(numero, respuesta);
    }
    else {
        enviarRespuestaSMS(numero, "Comando no reconocido. Envía 'HELP' para ver opciones.");
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
    menu += "WIFI - Info WiFi\n";
    menu += "GPS - Estado GPS\n";
    menu += "GSM - Estado GSM\n";
    menu += "MQTT - Estado MQTT\n";
    menu += "RESET - Reiniciar\n";
    menu += "\nID: " + device.chipId;
    
    return menu;
}



void checkGPSStatus() {
    static unsigned long lastGPSDebug = 0;
    
    if (millis() - lastGPSDebug > 10000) { 
        Serial.println("\n=== ESTADO GPS ===");
        Serial.println("Datos sin procesar disponibles: " + String(gpsSerial.available()));
        Serial.println("Chars procesados: " + String(gps.charsProcessed()));
        Serial.println("Chars con fallos: " + String(gps.failedChecksum()));
        Serial.println("Satélites: " + String(gps.satellites.value()));
        Serial.println("Ubicación válida: " + String(gps.location.isValid() ? "SI" : "NO"));
        
        if (gps.location.isValid()) {
            Serial.printf("Lat: %.6f, Lon: %.6f\n", gps.location.lat(), gps.location.lng());
            Serial.printf("Altitud: %.1f m, Velocidad: %.1f km/h\n", 
                         gps.altitude.meters(), gps.speed.kmph());
        } else {
            Serial.println("GPS no tiene fix aún");
            Serial.println("Tiempo desde último fix: " + String(gps.location.age()) + "ms");
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
    Serial.println("Pin TX GPS (GPIO16): " + String(txState == HIGH ? "ALTO ✓" : "BAJO ✗"));
    
    
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
        String displayData = rawGPSData.substring(0, displayLimit);
        Serial.println(displayData);
        
        
        int nmeaLines = 0;
        int ggaCount = 0, rmcCount = 0, gsvCount = 0, gsaCount = 0, vtgCount = 0;
        
        for (int i = 0; i < rawGPSData.length(); i++) {
            if (rawGPSData[i] == '$') {
                nmeaLines++;
                
                
                if (i + 6 < rawGPSData.length()) {
                    String nmeaType = rawGPSData.substring(i, i + 6);
                    if (nmeaType.indexOf("GGA") > 0) ggaCount++;
                    else if (nmeaType.indexOf("RMC") > 0) rmcCount++;
                    else if (nmeaType.indexOf("GSV") > 0) gsvCount++;
                    else if (nmeaType.indexOf("GSA") > 0) gsaCount++;
                    else if (nmeaType.indexOf("VTG") > 0) vtgCount++;
                }
            }
        }
        
        Serial.println("\n4. Estadísticas NMEA:");
        Serial.println("Total líneas NMEA: " + String(nmeaLines));
        Serial.println("  - GGA (posicionamiento): " + String(ggaCount));
        Serial.println("  - RMC (mínima recomendada): " + String(rmcCount));
        Serial.println("  - GSV (satélites en vista): " + String(gsvCount));
        Serial.println("  - GSA (activos): " + String(gsaCount));
        Serial.println("  - VTG (velocidad y curso): " + String(vtgCount));
        
       
        if (nmeaLines > 0) {
            Serial.println("\n5. Checksums válidos: " + String(gps.passedChecksum()));
            Serial.println("Checksums fallidos: " + String(gps.failedChecksum()));
        }
        
    } else {
        Serial.println("\n¡ADVERTENCIA CRÍTICA: NO SE RECIBIÓ NINGÚN DATO DEL GPS!");
        Serial.println("\nPosibles problemas:");
        Serial.println("1. Conexiones incorrectas (GPS TX → ESP RX, GPS RX → ESP TX)");
        Serial.println("2. Módulo GPS sin alimentación (ver LED del módulo)");
        Serial.println("3. Baudrate incorrecto (debe ser 9600)");
        Serial.println("4. Módulo GPS defectuoso");
    }
    
    
    Serial.println("\n6. Estado TinyGPSPlus:");
    Serial.println("Chars procesados: " + String(gps.charsProcessed()));
    Serial.println("Sentencias con fix: " + String(gps.sentencesWithFix()));
    Serial.println("Edad del fix: " + String(gps.location.age()) + "ms");
    
    
    if (gps.satellites.isValid()) {
        Serial.println("\n7. Información de satélites:");
        Serial.println("Satélites visibles: " + String(gps.satellites.value()));
        Serial.println("HDOP (precisión horizontal): " + String(gps.hdop.value() / 100.0));
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
            inputString += inChar;
        }
    }
}

void procesarComando(String comando) {
    comando.trim();
    
    if (comando.length() == 0) return;
    
    Serial.println("Comando recibido: " + comando);
    
    
    String comandoLower = comando;
    comandoLower.toLowerCase();
    
    if (comandoLower == "help" || comandoLower == "?") {
        mostrarMenuSerial();
    }
    else if (comandoLower == "config" || comandoLower == "show") {
        mostrarConfiguracionActual();
    }
    else if (comandoLower == "status") {
        publishStatus();
        Serial.println("Estado publicado por MQTT");
    }
    else if (comandoLower == "location") {
        GPSData data;
        readGPSData(data);
        if (data.isValid) {
            Serial.printf("Ubicación: Lat: %.6f, Lon: %.6f\n", data.latitude, data.longitude);
            publishGPSLocation(data);
        } else {
            Serial.println("GPS no válido - Satélites: " + String(gps.satellites.value()));
        }
    }
    else if (comandoLower == "test_sms") {
        Serial.println("Enviando SMS de prueba...");
        AccidentData testAccident;
        GPSData testGps;
        testAccident.impact_detected = true;
        testAccident.impact_magnitude = 2500;
        testGps.latitude = 18.735693;
        testGps.longitude = -70.162651;
        testGps.isValid = true;
        sendSMSAlert(testAccident, testGps);
    }
    else if (comandoLower.startsWith("set phone ")) {
        String nuevoNumero = comando.substring(10);
        cambiarNumeroEmergencia(nuevoNumero);
    }
    else if (comandoLower.startsWith("set sensitivity ")) {
        String valor = comando.substring(16);
        cambiarSensibilidad(valor);
    }
    else if (comandoLower.startsWith("set angle ")) {
        String valor = comando.substring(10);
        cambiarAnguloVuelco(valor);
    }
    else if (comandoLower.startsWith("set delay ")) {
        String valor = comando.substring(10);
        cambiarDelayAlerta(valor);
    }
    else if (comandoLower.startsWith("set sms ")) {
        String valor = comando.substring(8);
        cambiarSMSHabilitado(valor);
    }
    else if (comandoLower == "save") {
        saveConfig();
        Serial.println("Configuración guardada en EEPROM");
    }
    else if (comandoLower == "reset") {
        Serial.println("Reiniciando dispositivo...");
        delay(1000);
        ESP.restart();
    }
    else if (comandoLower == "wifi") {
        Serial.println("Información WiFi:");
        Serial.println("  SSID: " + WiFi.SSID());
        Serial.println("  Estado: " + String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado"));
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("  IP: " + WiFi.localIP().toString());
            Serial.println("  RSSI: " + String(WiFi.RSSI()) + " dBm");
        }
    }
    else if (comandoLower == "mqtt") {
        Serial.println("Estado MQTT: " + String(mqttConnected ? "Conectado" : "Desconectado"));
        if (!mqttConnected) {
            connectMQTT();
        }
    }
    else if (comandoLower == "gsm") {
        Serial.println("Verificando módulo GSM...");
        bool gsmOk = checkGSMModule();
        Serial.println("GSM: " + String(gsmOk ? "OK" : "ERROR"));
    }
    else if (comandoLower == "gps") {
        Serial.println("Estado GPS:");
        Serial.println("  Satélites: " + String(gps.satellites.value()));
        Serial.println("  Ubicación válida: " + String(gps.location.isValid() ? "SI" : "NO"));
        if (gps.location.isValid()) {
            Serial.printf("  Latitud: %.6f\n", gps.location.lat());
            Serial.printf("  Longitud: %.6f\n", gps.location.lng());
            Serial.printf("  Altitud: %.1f m\n", gps.altitude.meters());
            Serial.printf("  Velocidad: %.1f km/h\n", gps.speed.kmph());
        } else {
            Serial.println("  Edad del fix: " + String(gps.location.age()) + "ms");
        }
        Serial.println("  Chars procesados: " + String(gps.charsProcessed()));
    }
    else if (comandoLower == "gps_debug" || comandoLower == "gps_diagnostic") {
        diagnosticarGPSCompleto();
    }
    else if (comandoLower == "gps_raw") {
        Serial.println("Mostrando datos RAW GPS por 5 segundos...");
        Serial.println("(Presiona cualquier tecla para detener)");
        
        unsigned long startTime = millis();
        while (millis() - startTime < 5000) {
            if (Serial.available()) {
                while (Serial.available()) Serial.read();
                break;
            }
            
            while (gpsSerial.available()) {
                char c = gpsSerial.read();
                Serial.write(c);
            }
            delay(10);
        }
        Serial.println("\nFin datos RAW GPS");
    }
    else if (comandoLower == "sms_test") {
        Serial.println("Enviando SMS de prueba a número configurado...");
        enviarRespuestaSMS(accidentConfig.emergency_phone, "SMS de prueba desde comando Serial. ID: " + device.chipId);
    }
    else {
        Serial.println("Comando no reconocido. Escribe 'help' para ver los comandos disponibles.");
    }
}

void mostrarMenuSerial() {
    Serial.println("\n=== COMANDOS DISPONIBLES ===");
    Serial.println("help o ?          - Muestra este menú");
    Serial.println("config o show     - Muestra configuración actual");
    Serial.println("status            - Publica estado por MQTT");
    Serial.println("location          - Obtiene y publica ubicación GPS");
    Serial.println("test_sms          - Envía SMS de prueba");
    Serial.println("sms_test          - Envía SMS test al número config");
    Serial.println("");
    Serial.println("=== CONFIGURACIÓN ===");
    Serial.println("set phone [número] - Cambia número de emergencia");
    Serial.println("                    Ej: set phone +18291234567");
    Serial.println("set sensitivity [valor] - Cambia sensibilidad (500-10000)");
    Serial.println("                    Ej: set sensitivity 2500");
    Serial.println("set angle [grados] - Cambia ángulo de vuelco (10-90)");
    Serial.println("                    Ej: set angle 60");
    Serial.println("set delay [ms]    - Cambia delay de alerta (1000-600000)");
    Serial.println("                    Ej: set delay 30000");
    Serial.println("set sms [on/off]  - Habilita/deshabilita SMS");
    Serial.println("                    Ej: set sms on");
    Serial.println("save              - Guarda configuración en EEPROM");
    Serial.println("");
    Serial.println("=== SISTEMA ===");
    Serial.println("wifi              - Muestra info WiFi");
    Serial.println("mqtt              - Muestra/conecta MQTT");
    Serial.println("gsm               - Verifica módulo GSM");
    Serial.println("gps               - Muestra info GPS");
    Serial.println("gps_debug         - Diagnóstico completo GPS");
    Serial.println("gps_raw           - Muestra datos RAW GPS");
    Serial.println("reset             - Reinicia dispositivo");
    Serial.println("");
    Serial.println("=== COMANDOS SMS ===");
    Serial.println("Los mismos comandos se pueden enviar por SMS");
    Serial.println("al número configurado como emergencia");
    Serial.println("========================\n");
}

void cambiarNumeroEmergencia(String nuevoNumero) {
    nuevoNumero.trim();
    
    
    if (nuevoNumero.length() < 10) {
        Serial.println("ERROR: Número demasiado corto (mínimo 10 dígitos)");
        return;
    }
    
    
    int digitCount = 0;
    for (int i = 0; i < nuevoNumero.length(); i++) {
        if (isdigit(nuevoNumero[i]) || nuevoNumero[i] == '+') {
            digitCount++;
        }
    }
    
    if (digitCount < 10) {
        Serial.println("ERROR: Número debe contener al menos 10 dígitos");
        return;
    }
    
   
    String oldNumber = accidentConfig.emergency_phone;
    accidentConfig.emergency_phone = nuevoNumero;
    
    Serial.println("Número de emergencia actualizado:");
    Serial.println("  Antiguo: " + oldNumber);
    Serial.println("  Nuevo: " + accidentConfig.emergency_phone);
    
    
    saveConfig();
    String mensaje = "Número actualizado vía Serial: " + nuevoNumero;
    publishEvent("PHONE_UPDATED_SERIAL", mensaje.c_str());
}

void cambiarSensibilidad(String valor) {
    int sens = valor.toInt();
    
    if (sens >= 500 && sens <= 10000) {
        accidentConfig.sensitivity = sens;
        Serial.println("Sensibilidad actualizada a: " + String(sens));
        saveConfig();
    } else {
        Serial.println("ERROR: Sensibilidad debe estar entre 500 y 10000");
    }
}

void cambiarAnguloVuelco(String valor) {
    int angulo = valor.toInt();
    
    if (angulo >= 10 && angulo <= 90) {
        accidentConfig.rollover_angle = angulo;
        Serial.println("Ángulo de vuelco actualizado a: " + String(angulo) + "°");
        saveConfig();
    } else {
        Serial.println("ERROR: Ángulo debe estar entre 10 y 90 grados");
    }
}

void cambiarDelayAlerta(String valor) {
    unsigned long delay = valor.toInt();
    
    if (delay >= 1000 && delay <= 600000) {
        accidentConfig.alert_delay = delay;
        Serial.println("Delay de alerta actualizado a: " + String(delay) + "ms");
        saveConfig();
    } else {
        Serial.println("ERROR: Delay debe estar entre 1000 y 600000 ms");
    }
}

void cambiarSMSHabilitado(String valor) {
    valor.toLowerCase();
    
    if (valor == "on" || valor == "true" || valor == "1" || valor == "si" || valor == "yes") {
        accidentConfig.sms_enabled = true;
        Serial.println("SMS habilitado");
        saveConfig();
    }
    else if (valor == "off" || valor == "false" || valor == "0" || valor == "no") {
        accidentConfig.sms_enabled = false;
        Serial.println("SMS deshabilitado");
        saveConfig();
    }
    else {
        Serial.println("ERROR: Use 'on' o 'off'");
    }
}

void mostrarConfiguracionActual() {
    Serial.println("\n=== CONFIGURACIÓN ACTUAL ===");
    Serial.println("Número emergencia: " + accidentConfig.emergency_phone);
    Serial.println("Sensibilidad: " + String(accidentConfig.sensitivity));
    Serial.println("Ángulo vuelco: " + String(accidentConfig.rollover_angle) + "°");
    Serial.println("Delay alerta: " + String(accidentConfig.alert_delay) + "ms");
    Serial.println("SMS habilitado: " + String(accidentConfig.sms_enabled ? "SI" : "NO"));
    Serial.println("ID Dispositivo: " + device.chipId);
    Serial.println("Versión FW: " + device.firmwareVersion);
    
    
    Serial.println("\n=== ESTADO GPS ACTUAL ===");
    Serial.println("Fix válido: " + String(gps.location.isValid() ? "SI" : "NO"));
    Serial.println("Satélites: " + String(gps.satellites.value()));
    Serial.println("Chars procesados: " + String(gps.charsProcessed()));
    if (gps.location.isValid()) {
        Serial.printf("Ubicación: Lat %.6f, Lon %.6f\n", gps.location.lat(), gps.location.lng());
    }
    Serial.println("===========================\n");
}



void procesarOTAChunk(const String& message) {
    if (ESP.getFreeHeap() < 20000) {
        Serial.println("Memoria insuficiente para OTA");
        return;
    }

    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.printf("Error parseando JSON OTA: %s\n", error.c_str());
        return;
    }

    if (!doc.containsKey("EventType") || doc["EventType"] != "UpdateFirmwareDevice") {
        return;
    }

    if (!doc.containsKey("Details")) {
        Serial.println("No se encontraron Details en el mensaje OTA");
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
            Serial.println("OTA en progreso, ignorando nuevo inicio");
            return;
        }
        
        if (firmwareVersion == device.firmwareVersion) {
            publishOTAError("Ya tiene esta versión de firmware instalada", firmwareVersion);
            return;
        }
        
        if (!iniciarOTA(firmwareVersion, totalParts, totalSize)) {
            return;
        }
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

    Serial.printf("Chunk %d/%d procesado. Progreso: %d%%, Memoria: %d\n", 
                 partIndex, totalParts, progress, ESP.getFreeHeap());

    
    if (partIndex == totalParts) {
        finalizarOTA();
    }
}

bool iniciarOTA(const String& firmwareVersion, int totalParts, size_t totalSize) {
    Serial.printf("Iniciando OTA directa. Version: %s, Partes: %d, Tamaño: %d\n", 
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
    Serial.println("OTA iniciada directamente en memoria flash");
    return true;
}

bool procesarChunkOTA(const String& base64Data, int partIndex) {
    if (!otaContext.inProgress) {
        Serial.println("ERROR: OTA no iniciada");
        return false;
    }

    
    String decodedData = base64Decode(base64Data);
    if (decodedData.length() == 0) {
        publishOTAError("Error decodificando Base64", otaContext.firmwareVersion);
        return false;
    }

    size_t dataLength = decodedData.length();
    uint8_t* dataBuffer = (uint8_t*)decodedData.c_str();
    
    size_t written = Update.write(dataBuffer, dataLength);
    
    if (written != decodedData.length()) {
        String errorMsg = "Error escribiendo chunk: ";
        errorMsg += String(written) + " de " + String(decodedData.length()) + " bytes";
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
    Serial.println("Finalizando OTA...");

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
        Serial.println("OTA completada exitosamente!");
        publishOTASuccess(otaContext.firmwareVersion);
        
        Serial.println("Reiniciando en 3 segundos...");
        delay(3000);
        ESP.restart();
    } else {
        publishOTAError("OTA no se completó correctamente", otaContext.firmwareVersion);
        cleanupOTA();
    }
}

void cleanupOTA() {
    if (otaContext.inProgress) {
        Update.end(); 
        Serial.println("OTA abortada");
    }
    
    otaContext.inProgress = false;
    otaContext.currentPart = 0;
    otaContext.totalParts = 0;
    otaContext.receivedSize = 0;
    otaContext.totalSize = 0;
    otaContext.startTime = 0;
}

void publishOTAProgress(int progress, const String& firmwareVersion) {
    if (!mqttConnected) return;

    DynamicJsonDocument doc(1024);
    
    JsonObject deviceObj = doc.createNestedObject("Device");
    deviceObj["status"] = mqttConnected ? "connected" : "disconnected";
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
    
    publishMQTTMessage(MQTT_TOPIC_EVENTS.c_str(), output);
    
    if (progress % 25 == 0 || progress == 100) {
        Serial.printf("Progreso OTA: %d%%\n", progress);
    }
}

void publishOTAError(const String& errorMessage, const String& firmwareVersion) {
    if (!mqttConnected) return;

    DynamicJsonDocument doc(1024);
    
    JsonObject deviceObj = doc.createNestedObject("Device");
    deviceObj["status"] = mqttConnected ? "connected" : "disconnected"; 
    deviceObj["ChipId"] = device.chipId;
    deviceObj["MacAddress"] = device.macAddress;
    deviceObj["ChipType"] = device.chipType;
    deviceObj["FirmwareVersion"] = device.firmwareVersion;

    doc["Timestamp"] = getISOTimestamp();
    
    JsonObject details = doc.createNestedObject("Details");
    details["FirmwareVersion"] = firmwareVersion.isEmpty() ? device.firmwareVersion : firmwareVersion;
    details["IsError"] = true;
    details["ErrorMessage"] = errorMessage;
    details["Progress"] = 0;

    String output;
    serializeJson(doc, output);
    
    publishMQTTMessage(MQTT_TOPIC_EVENTS.c_str(), output);
    Serial.printf("Error OTA: %s\n", errorMessage.c_str());
}

void publishOTASuccess(const String& firmwareVersion) {
    if (!mqttConnected) return;

    DynamicJsonDocument doc(1024);
    
    JsonObject deviceObj = doc.createNestedObject("Device");
    deviceObj["status"] = mqttConnected ? "connected" : "disconnected";  
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
    
    publishMQTTMessage(MQTT_TOPIC_EVENTS.c_str(), output);
    Serial.println("Éxito OTA publicado");
}

String base64Decode(const String& encoded) {
    String decoded;
    int input_len = encoded.length();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    
    const String base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    while (input_len-- && (encoded[in_] != '=') && isBase64(encoded[in_])) {
        char_array_4[i++] = encoded[in_]; in_++;
        if (i == 4) {
            for (i = 0; i <4; i++)
                char_array_4[i] = base64_chars.indexOf(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                decoded += char_array_3[i];
            i = 0;
        }
    }

    if (i) {
        for (j = i; j <4; j++)
            char_array_4[j] = 0;

        for (j = 0; j <4; j++)
            char_array_4[j] = base64_chars.indexOf(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) 
            decoded += char_array_3[j];
    }

    return decoded;
}

bool isBase64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}

void initializeGSM() {
    Serial.println("Inicializando módulo GSM...");
    
    delay(3000);
    
    if (!checkGSMModule()) {
        Serial.println("ERROR: No se puede comunicar con módulo GSM");
        return;
    }
    
    sendATCommand("ATE0", "OK", 2000);  
    sendATCommand("AT+CMGF=1", "OK", 2000);  
    sendATCommand("AT+CNMI=2,2,0,0,0", "OK", 2000);  
    
    Serial.println("Módulo GSM inicializado para recibir SMS");
}

void checkGSMStatus() {
    static unsigned long lastCheck = 0;
    
    if (millis() - lastCheck > 30000) {  
        if (!checkGSMModule()) {
            Serial.println("Reiniciando comunicación GSM...");
            initializeGSM();
        }
        lastCheck = millis();
    }
}

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);

  AccidentConfig defaultConfig; 
  
  char marker[4];
  for (int i = 0; i < 4; i++) {
    marker[i] = EEPROM.read(CONFIG_EEPROM_ADDR + i);
  }
  
  bool configLoadedFromEEPROM = false;
  
  if (String(marker) == "CFG") {
    int addr = CONFIG_EEPROM_ADDR + 4;
    
    int savedSensitivity = EEPROM.read(addr) | (EEPROM.read(addr + 1) << 8);
    addr += 2;
    
    int savedRolloverAngle = EEPROM.read(addr);
    addr += 1;
    
    unsigned long savedAlertDelay = 0;
    for (int i = 0; i < 4; i++) {
      savedAlertDelay |= (unsigned long)EEPROM.read(addr + i) << (8 * i);
    }
    addr += 4;
    
    bool savedSmsEnabled = EEPROM.read(addr);
    addr += 1;
    
    String savedPhone = "";
    char ch;
    int maxPhoneLength = 30;
    bool validPhone = true;
    
    for (int i = 0; i < maxPhoneLength; i++) {
      ch = EEPROM.read(addr + i);
      if (ch == 0 || ch == 0xFF) {
        break;
      }
     
      if (!(isdigit(ch) || ch == '+' || ch == ' ' || ch == '-' || ch == '(' || ch == ')')) {
        validPhone = false;
        break;
      }
      savedPhone += ch;
    }
    
    if (savedSensitivity >= 500 && savedSensitivity <= 10000 &&
        savedRolloverAngle >= 10 && savedRolloverAngle <= 90 &&
        savedAlertDelay >= 1000 && savedAlertDelay <= 600000 &&
        validPhone && savedPhone.length() >= 10) {
      
      accidentConfig.sensitivity = savedSensitivity;
      accidentConfig.rollover_angle = savedRolloverAngle;
      accidentConfig.alert_delay = savedAlertDelay;
      accidentConfig.sms_enabled = savedSmsEnabled;
      accidentConfig.emergency_phone = savedPhone;
      
      configLoadedFromEEPROM = true;
      Serial.println("Configuración cargada de EEPROM");
    } else {
      Serial.println("Configuración en EEPROM inválida, usando valores por defecto");
    }
  }
  
  if (!configLoadedFromEEPROM) {
    accidentConfig = defaultConfig; 
    Serial.println("Usando configuración por defecto de config.h");
    
    saveConfig();
  }
  
  Serial.println("Configuración actual:");
  Serial.printf("  Sensibilidad: %d\n", accidentConfig.sensitivity);
  Serial.printf("  Ángulo vuelco: %d\n", accidentConfig.rollover_angle);
  Serial.printf("  Delay alerta: %lu\n", accidentConfig.alert_delay);
  Serial.printf("  SMS habilitado: %s\n", accidentConfig.sms_enabled ? "SI" : "NO");
  Serial.printf("  Teléfono: %s\n", accidentConfig.emergency_phone.c_str());
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
  
  for (int i = 0; i < 4; i++) {
    EEPROM.write(addr + i, (accidentConfig.alert_delay >> (8 * i)) & 0xFF);
  }
  addr += 4;
  
  EEPROM.write(addr, accidentConfig.sms_enabled);
  addr += 1;
  
  for (int i = 0; i < accidentConfig.emergency_phone.length(); i++) {
    EEPROM.write(addr + i, accidentConfig.emergency_phone[i]);
  }
  EEPROM.write(addr + accidentConfig.emergency_phone.length(), 0);
  
  EEPROM.commit();
  Serial.println("Configuración guardada en EEPROM");
}

void checkAccidents() {
  if (millis() - lastAccidentCheck < ACCIDENT_CHECK_INTERVAL) return;
  lastAccidentCheck = millis();

  mpu.getAcceleration(&ax, &ay, &az);

  deltx = ax - oldx;
  delty = ay - oldy;
  deltz = az - oldz;

  oldx = ax;
  oldy = ay;
  oldz = az;

  vibration--;
  if (vibration < 0) vibration = 0;
  if (vibration > 0) return;

  magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));

  AccidentData accident;
  
  if (magnitude >= accidentConfig.sensitivity) {
    accident.impact_detected = true;
    accident.impact_magnitude = magnitude;
    updateflag = 1;
    vibration = devibrate;
    Serial.println("Choque detectado!");
  }

  float axf = ax / 16384.0;
  float ayf = ay / 16384.0;
  float azf = az / 16384.0;

  float roll  = atan2(ayf, sqrt(axf * axf + azf * azf)) * 180.0 / PI;
  float pitch = atan2(-axf, sqrt(ayf * ayf + azf * azf)) * 180.0 / PI;

  accident.roll_angle = roll;
  accident.pitch_angle = pitch;

  if (abs(roll) > accidentConfig.rollover_angle || abs(pitch) > accidentConfig.rollover_angle) {
    accident.rollover_detected = true;
    updateflag = 1;
    Serial.printf("Rollover detectado! roll=%.1f pitch=%.1f\n", roll, pitch);
  }

  if (accident.impact_detected || accident.rollover_detected) {
    accident.detection_time = millis();
    
    GPSData gpsData;
    readGPSData(gpsData);
    
    publishAccidentEvent(accident, gpsData);
    
    digitalWrite(BUZZER_PIN, HIGH);
    alert_triggered = true;
    alert_time = millis();
  }
}

void sendSMSAlert(const AccidentData& accident, const GPSData& gps) {
    if (!accidentConfig.sms_enabled || accidentConfig.emergency_phone.length() < 10) {
        Serial.println("SMS deshabilitado o número no configurado");
        return;
    }

    Serial.println("=== INICIANDO ENVÍO DE SMS ===");

    if (!checkGSMModule()) {
        Serial.println("ERROR: Módulo GSM no responde");
        publishEvent("SMS_ERROR", "Módulo GSM no responde");
        return;
    }

    String sms_data = "ALERTA ACCIDENTE\r\n";
    if (accident.impact_detected) sms_data += "Tipo: IMPACTO\r\n";
    if (accident.rollover_detected) sms_data += "Tipo: VUELCO\r\n";
    
    if (gps.isValid) {
        sms_data += "Ubic: http://maps.google.com/maps?q=loc:";
        sms_data += String(gps.latitude, 6) + "," + String(gps.longitude, 6);
        sms_data += "\r\n";
    } else {
        sms_data += "GPS: No disponible\r\n";
    }
    
    sms_data += "Hora: " + getISOTimestamp();
    sms_data += "\r\nID: " + device.chipId;

    Serial.println("Mensaje preparado:");
    Serial.println(sms_data);

    Serial.println("Configurando modo texto GSM...");
    if (!sendATCommand("AT+CMGF=1", "OK", 5000)) {
        Serial.println("ERROR: No se pudo configurar modo texto");
        return;
    }

    Serial.println("Configurando codificación GSM...");
    if (!sendATCommand("AT+CSCS=\"GSM\"", "OK", 3000)) {
        Serial.println("ERROR: No se pudo configurar codificación GSM");
        return;
    }

    String phoneNumber = accidentConfig.emergency_phone;
    
    phoneNumber.replace(" ", "");
    phoneNumber.replace("-", "");
    phoneNumber.replace("(", "");
    phoneNumber.replace(")", "");
    
    if (!phoneNumber.startsWith("+")) {
        phoneNumber = "+" + phoneNumber;
    }
    
    Serial.println("Número destino: " + phoneNumber);

    String cmd = "AT+CMGS=\"" + phoneNumber + "\"";
    Serial.println("Enviando comando SMS: " + cmd);
    
    sim800.println(cmd);
    delay(3000); 

    Serial.println("Esperando prompt > ...");
    unsigned long timeout = millis();
    bool gotPrompt = false;
    String response = "";
    
    while (millis() - timeout < 10000) { 
        while (sim800.available()) {
            char c = sim800.read();
            response += c;
            Serial.write(c);  
            
            if (c == '>') {
                gotPrompt = true;
                Serial.println("\n>>> PROMPT > RECIBIDO! <<<");
                break;
            }
        }
        if (gotPrompt) break;
        delay(100);
    }

    if (!gotPrompt) {
        Serial.println("\nERROR: No se recibió prompt > después de 10 segundos");
        Serial.println("Respuesta recibida: " + response);
        return;
    }

    Serial.println("Enviando contenido del SMS...");
    sim800.println(sms_data);
    delay(2000);

    Serial.println("Finalizando con Ctrl+Z...");
    sim800.write(0x1A);  
    
    Serial.println("Esperando confirmación...");
    timeout = millis();
    bool smsSent = false;
    response = "";
    
    while (millis() - timeout < 15000) { 
        while (sim800.available()) {
            char c = sim800.read();
            response += c;
            Serial.write(c);
            
            if (response.indexOf("OK") >= 0 || response.indexOf("+CMGS") >= 0) {
                smsSent = true;
                Serial.println("\n>>> SMS ENVIADO EXITOSAMENTE! <<<");
                break;
            }
            if (response.indexOf("ERROR") >= 0) {
                Serial.println("\n>>> ERROR EN ENVÍO <<<");
                break;
            }
        }
        if (smsSent || response.indexOf("ERROR") >= 0) break;
        delay(100);
    }

    if (smsSent) {
        Serial.println("SMS de emergencia enviado correctamente");
        publishEvent("SMS_SENT", "Alerta enviada exitosamente");
    } else {
        Serial.println("ERROR: No se confirmó el envío del SMS");
        Serial.println("Respuesta final: " + response);
        publishEvent("SMS_ERROR", "Fallo en envío de SMS");
    }
    
    while (sim800.available()) {
        sim800.read();
    }
    
    Serial.println("FIN ENVÍO SMS");
}

bool checkGSMModule() {
    Serial.println("Verificando módulo GSM...");
    
    while (sim800.available()) {
        sim800.read();
    }
    
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
    
    if (responded) {
        Serial.println("Módulo GSM responde correctamente");
    } else {
        Serial.println("Módulo GSM NO responde");
    }
    
    return responded;
}

bool sendATCommand(String command, String expectedResponse, unsigned long timeoutMs) {
    Serial.println("Enviando: " + command);
    
    while (sim800.available()) {
        sim800.read();
    }
    
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
    
    Serial.println("Timeout en comando: " + command);
    Serial.println("Respuesta recibida: " + response);
    return false;
}

void publishAccidentEvent(const AccidentData& accident, const GPSData& gps) {
  String eventData = "{";
  eventData += "\"Device\":{";
  eventData += "\"status\":\"" + String(mqttConnected ? "connected" : "disconnected") + "\",";
  eventData += "\"ChipId\":\"" + device.chipId + "\",";
  eventData += "\"MacAddress\":\"" + device.macAddress + "\",";
  eventData += "\"IPAddress\":\"" + WiFi.localIP().toString() + "\",";
  eventData += "\"ChipType\":\"" + device.chipType + "\",";
  eventData += "\"FirmwareVersion\":\"" + device.firmwareVersion + "\"";
  eventData += "},";
  eventData += "\"Timestamp\":\"" + getISOTimestamp() + "\",";
  eventData += "\"Details\":{";
  eventData += "\"chipId\":\"" + device.chipId + "\",";
  eventData += "\"accident\":{";
  eventData += "\"impact_detected\":" + String(accident.impact_detected ? "true" : "false") + ",";
  eventData += "\"rollover_detected\":" + String(accident.rollover_detected ? "true" : "false") + ",";
  eventData += "\"impact_magnitude\":" + String(accident.impact_magnitude, 1) + ",";
  eventData += "\"roll_angle\":" + String(accident.roll_angle, 1) + ",";
  eventData += "\"pitch_angle\":" + String(accident.pitch_angle, 1) + ",";
  eventData += "\"gps_data\":{";
  eventData += "\"latitude\":" + String(gps.latitude, 6) + ",";
  eventData += "\"longitude\":" + String(gps.longitude, 6) + ",";
  eventData += "\"is_valid\":" + String(gps.isValid ? "true" : "false");
  eventData += "},";
  eventData += "\"config\":{";
  eventData += "\"sensitivity\":" + String(accidentConfig.sensitivity) + ",";
  eventData += "\"rollover_angle\":" + String(accidentConfig.rollover_angle) + ",";
  eventData += "\"alert_delay\":" + String(accidentConfig.alert_delay) + ",";
  eventData += "\"sms_enabled\":" + String(accidentConfig.sms_enabled ? "true" : "false");
  eventData += "}";
  eventData += "}";
  eventData += "}";
  eventData += "}";

  publishMQTTMessage(MQTT_TOPIC_EVENTS.c_str(), eventData);
  Serial.println("Evento de accidente publicado por MQTT");

  if (accidentConfig.sms_enabled) {
    sendSMSAlert(accident, gps);
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    String topicStr = String(topic);
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.printf("Tópico recibido: %s\n", topicStr.c_str());
    Serial.printf("Mensaje: %s\n", message.c_str());
    
    String deviceEventTopic = "event/" + device.chipId;
    if (topicStr == deviceEventTopic) {
        procesarComandoMQTT(message);
    } else if (topicStr == MQTT_TOPIC_CONFIG) {
        procesarConfiguracion(message);
    }
}

void procesarConfiguracion(const String& message) {
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
        Serial.printf("Sensibilidad cambiada a: %d\n", accidentConfig.sensitivity);
    }
    
    if (doc.containsKey("rollover_angle")) {
        accidentConfig.rollover_angle = doc["rollover_angle"];
        configChanged = true;
        Serial.printf("Ángulo vuelco cambiado a: %d\n", accidentConfig.rollover_angle);
    }
    
    if (doc.containsKey("alert_delay")) {
        accidentConfig.alert_delay = doc["alert_delay"];
        configChanged = true;
        Serial.printf("Delay alerta cambiado a: %lu\n", accidentConfig.alert_delay);
    }
    
    if (doc.containsKey("sms_enabled")) {
        accidentConfig.sms_enabled = doc["sms_enabled"];
        configChanged = true;
        Serial.printf("SMS habilitado: %s\n", accidentConfig.sms_enabled ? "SI" : "NO");
    }
    
    if (doc.containsKey("emergency_phone")) {
        accidentConfig.emergency_phone = doc["emergency_phone"].as<String>();
        configChanged = true;
        Serial.printf("Teléfono emergencia cambiado a: %s\n", accidentConfig.emergency_phone.c_str());
    }
    
    if (configChanged) {
        saveConfig();
        publishEvent("CONFIG_UPDATED", "Configuración actualizada exitosamente");
    }
}

void procesarComandoMQTT(const String& message) {
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
        String configJson = "{";
        configJson += "\"sensitivity\":" + String(accidentConfig.sensitivity) + ",";
        configJson += "\"rollover_angle\":" + String(accidentConfig.rollover_angle) + ",";
        configJson += "\"alert_delay\":" + String(accidentConfig.alert_delay) + ",";
        configJson += "\"sms_enabled\":" + String(accidentConfig.sms_enabled ? "true" : "false") + ",";
        configJson += "\"emergency_phone\":\"" + accidentConfig.emergency_phone + "\"";
        configJson += "}";
        
        publishMQTTMessage(MQTT_TOPIC_CONFIG.c_str(), configJson);
    } else {
        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, message);
        if (!error && doc.containsKey("EventType")) {
            String eventType = doc["EventType"].as<String>();
            if (eventType == "UpdateFirmwareDevice") {
                procesarOTAChunk(message);
            }
        }
    }
}

void readGPSData(GPSData& data) {
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
            Serial.printf("GPS aún no tiene fix - Satélites: %d, Edad datos: %dms\n", 
                         gps.satellites.value(), gps.location.age());
            lastGPSWarning = millis();
        }
    }
}

void publishGPSLocation(const GPSData& data) {
    String telemetry = "{";
    telemetry += "\"Device\":{";
    telemetry += "\"status\":\"" + String(mqttConnected ? "connected" : "disconnected") + "\",";
    telemetry += "\"ChipId\":\"" + device.chipId + "\",";
    telemetry += "\"MacAddress\":\"" + device.macAddress + "\",";
    telemetry += "\"IPAddress\":\"" + WiFi.localIP().toString() + "\",";
    telemetry += "\"ChipType\":\"" + device.chipType + "\",";
    telemetry += "\"FirmwareVersion\":\"" + device.firmwareVersion + "\"";
    telemetry += "},";
    telemetry += "\"Timestamp\":\"" + getISOTimestamp() + "\",";
    telemetry += "\"Details\":{";
    telemetry += "\"chipId\":\"" + device.chipId + "\",";
    telemetry += "\"gps_data\":{";
    telemetry += "\"latitude\":" + String(data.latitude, 6) + ",";
    telemetry += "\"longitude\":" + String(data.longitude, 6) + ",";
    telemetry += "\"altitude\":" + String(data.altitude, 1) + ",";
    telemetry += "\"speed\":" + String(data.speed, 1) + ",";
    telemetry += "\"course\":" + String(data.course, 1) + ",";
    telemetry += "\"satellites\":" + String(data.satellites) + ",";
    telemetry += "\"is_valid\":" + String(data.isValid ? "true" : "false") + ",";
    telemetry += "\"timestamp\":\"" + data.timestamp + "\"";
    telemetry += "}";
    telemetry += "}";
    telemetry += "}";

    publishMQTTMessage(MQTT_TOPIC_TELEMETRY.c_str(), telemetry);
}

void publishMQTTMessage(const char* topic, const String& message) {
    if (!mqttConnected || otaInProgress || otaContext.inProgress) return;
    
    bool result = mqtt_client.publish(topic, message.c_str());
    if (!result) {
        Serial.printf("Error publicando en %s\n", topic);
    }
}

void publishEvent(const char* evento, const char* descripcion) {
    String eventData = "{";
    eventData += "\"Device\":{";
    eventData += "\"status\":\"" + String(mqttConnected ? "connected" : "disconnected") + "\",";
    eventData += "\"ChipId\":\"" + device.chipId + "\",";
    eventData += "\"MacAddress\":\"" + device.macAddress + "\",";
    eventData += "\"IPAddress\":\"" + WiFi.localIP().toString() + "\",";
    eventData += "\"ChipType\":\"" + device.chipType + "\",";
    eventData += "\"FirmwareVersion\":\"" + device.firmwareVersion + "\"";
    eventData += "},";
    eventData += "\"Timestamp\":\"" + getISOTimestamp() + "\",";
    eventData += "\"Details\":{";
    eventData += "\"chipId\":\"" + device.chipId + "\",";
    eventData += "\"info\":{";
    eventData += "\"event\":\"" + String(evento) + "\",";
    eventData += "\"description\":\"" + String(descripcion) + "\",";
    eventData += "\"timestamp\":" + String(millis());
    eventData += "}";
    eventData += "}";
    eventData += "}";

    publishMQTTMessage(MQTT_TOPIC_EVENTS.c_str(), eventData);
}

void publishStatus() {
    String status = "{";
    status += "\"Device\":{";
    status += "\"status\":\"" + String(mqttConnected ? "connected" : "disconnected") + "\",";
    status += "\"ChipId\":\"" + device.chipId + "\",";
    status += "\"MacAddress\":\"" + device.macAddress + "\",";
    status += "\"IPAddress\":\"" + WiFi.localIP().toString() + "\",";
    status += "\"ChipType\":\"" + device.chipType + "\",";
    status += "\"FirmwareVersion\":\"" + device.firmwareVersion + "\"";
    status += "},";
    status += "\"Timestamp\":\"" + getISOTimestamp() + "\",";
    status += "\"Details\":{";
    status += "\"chipId\":\"" + device.chipId + "\",";
    status += "\"wifi_status\":\"" + String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + "\",";
    status += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    status += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
    status += "\"uptime\":" + String(millis()) + ",";
    status += "\"gps_status\":{";
    status += "\"has_fix\":" + String(gps.location.isValid() ? "true" : "false") + ",";
    status += "\"satellites\":" + String(gps.satellites.value()) + ",";
    status += "\"age\":" + String(gps.location.age());
    status += "},";
    status += "\"accident_config\":{";
    status += "\"sensitivity\":" + String(accidentConfig.sensitivity) + ",";
    status += "\"rollover_angle\":" + String(accidentConfig.rollover_angle) + ",";
    status += "\"alert_delay\":" + String(accidentConfig.alert_delay) + ",";
    status += "\"sms_enabled\":" + String(accidentConfig.sms_enabled ? "true" : "false") + ",";
    status += "\"emergency_phone\":\"" + accidentConfig.emergency_phone + "\"";
    status += "}";
    status += "}";
    status += "}";

    publishMQTTMessage(MQTT_TOPIC_STATUS.c_str(), status);
}

String getMQTTClientID() {
  return device.clientID;
}

bool configurarTiempoNTP() {
    Serial.println("Configurando tiempo NTP...");
    
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
    
    Serial.print("Obteniendo tiempo desde NTP");
    int intentos = 0;
    
    while (intentos < 30) {
        delay(1000);
        Serial.print(".");
        
        time_t now = time(nullptr);
        if (now > 1000000000) {
            Serial.println("\nNTP sincronizado exitosamente!");
            return true;
        }
        intentos++;
    }
    
    Serial.println("\nError: No se pudo sincronizar con NTP después de 30 segundos");
    return false;
}

String getISOTimestamp() {
    time_t now = time(nullptr);
    
    if (now < 1000000000) {
        unsigned long ms = millis();
        char fallback[30];
        snprintf(fallback, sizeof(fallback), "2025-01-01T00:00:%02lu.%03luZ", 
                (ms / 1000) % 60, ms % 1000);
        return String(fallback);
    }
    
    struct tm* timeinfo = localtime(&now);
    if (!timeinfo) {
        return "2025-01-01T00:00:00.000Z";
    }
    
    uint32_t ms = millis() % 1000;
    char timestamp[25];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", timeinfo);
    
    char fullTimestamp[30];
    snprintf(fullTimestamp, sizeof(fullTimestamp), "%s.%03ldZ", timestamp, ms);
    
    return String(fullTimestamp);
}

void verificarYSincronizarTiempo() {
    static unsigned long ultimaVerificacion = 0;
    
    if (millis() - ultimaVerificacion > TIME_SYNC_INTERVAL) {
        time_t now = time(nullptr);
        if (now < 1000000000) {
            Serial.println("Re-sincronizando tiempo NTP...");
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

  Serial.printf("Información del dispositivo:\n");
  Serial.printf("  Chip ID: %s\n", info.chipId.c_str());
  Serial.printf("  MAC Address: %s\n", info.macAddress.c_str());
  Serial.printf("  Chip Type: %s\n", info.chipType.c_str());
  Serial.printf("  Firmware: %s\n", info.firmwareVersion.c_str());

  return info;
}

bool connectMQTT() {
    if (mqttConnected) return true;
    
    Serial.printf("Conectando MQTTS como %s...\n", device.clientID.c_str());

    wifiClient.setInsecure(); 
    
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt_client.setCallback(mqtt_callback);
    mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
    mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);
    
    if (mqtt_client.connect(device.clientID.c_str(), API_KEY, "")) {
        Serial.println("MQTTS CONECTADO!");
        mqttConnected = true;
        
        String deviceEventTopic = "event/" + device.chipId;
        mqtt_client.subscribe(deviceEventTopic.c_str());
        mqtt_client.subscribe(MQTT_TOPIC_CONFIG.c_str());
        Serial.printf("Suscrito a: %s y %s\n", deviceEventTopic.c_str(), MQTT_TOPIC_CONFIG.c_str());
        
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
        Serial.println("Inicio OTA");
        otaInProgress = true;
        publishEvent("OTA_START", "Actualización de firmware OTA iniciada");
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA completada");
        otaInProgress = false;
        publishEvent("OTA_END", "Actualización de firmware OTA completada exitosamente");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progreso OTA: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error OTA[%u]: ", error);
        otaInProgress = false;
        
        const char* errorMsg = "";
        switch(error) {
            case OTA_AUTH_ERROR: errorMsg = "Error de autenticación OTA"; break;
            case OTA_BEGIN_ERROR: errorMsg = "Error al iniciar actualización OTA"; break;
            case OTA_CONNECT_ERROR: errorMsg = "Error de conexión durante OTA"; break;
            case OTA_RECEIVE_ERROR: errorMsg = "Error de recepción de datos OTA"; break;
            case OTA_END_ERROR: errorMsg = "Error al finalizar actualización OTA"; break;
        }
        
        publishEvent("OTA_ERROR", errorMsg);
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA configurado");
}

void handleButton() {
  static bool lastButtonState = HIGH;
  static unsigned long buttonPressStartTime = 0;
  static bool isButtonPressed = false;
  
  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long actualMillis = millis();

  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStartTime = actualMillis;
    isButtonPressed = true;
    Serial.println("Botón presionado - mantenga 3 segundos para modo AP");
  } 
  
  else if (buttonState == HIGH && lastButtonState == LOW && isButtonPressed) {
    unsigned long pressDuration = actualMillis - buttonPressStartTime;
    isButtonPressed = false;

    if (pressDuration > (BUTTON_LONG_PRESS_TIME * 1000)) {
      Serial.println("Activando modo AP por botón físico...");
      
      String descripcion = "Modo Access Point activado por botón físico - ";
      descripcion += (WiFi.SSID().length() > 0) ? "Con credenciales guardadas" : "Sin credenciales guardadas";
      publishEvent("AP_MODE_ACTIVATED", descripcion.c_str());
      
      WiFiManager wifiManager;
      String apSSID = "VESIS-" + String(ESP.getChipId(), HEX);
      
      wifiManager.setConfigPortalTimeout(180);
      wifiManager.startConfigPortal(apSSID.c_str(), "12345678");
      
      Serial.println("Reiniciando después de configuración AP...");
      ESP.restart();
    }
  }

  lastButtonState = buttonState;
}

void updateWiFiLED() {
  static bool ledState = LOW;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 
      (currentWiFiStatus == WIFI_DISCONNECTED ? FAST_BLINK_INTERVAL : 
       currentWiFiStatus == WIFI_CONNECTING ? SLOW_BLINK_INTERVAL : 0)) {
    
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
    Serial.println("Reconectando WiFi...");
    currentWiFiStatus = WIFI_CONNECTING;
    WiFi.begin();
  } 
  else if (WiFi.status() == WL_CONNECTED && currentWiFiStatus != WIFI_CONNECTED) {
    Serial.println("WiFi reconectado");
    currentWiFiStatus = WIFI_CONNECTED;
    publishEvent("WIFI_RECONNECTED", "Conexión WiFi restaurada exitosamente");
  }
}

void updateWiFiStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    if (currentWiFiStatus == WIFI_CONNECTED) {
      Serial.println("WiFi desconectado");
      currentWiFiStatus = WIFI_DISCONNECTED;
      publishEvent("WIFI_DISCONNECTED", "Conexión WiFi perdida");
    }
    reconnectWiFi();
  }
}

void setupWiFi() {
  Serial.println("Iniciando configuración WiFi...");
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  currentWiFiStatus = WIFI_CONNECTING;

  if (WiFi.SSID().length() > 0) {
    Serial.println("Credenciales WiFi encontradas. Conectando...");
    Serial.println("SSID: " + WiFi.SSID());
    
    WiFi.begin();
    
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi conectado exitosamente!");
      Serial.println("IP address: " + WiFi.localIP().toString());
      currentWiFiStatus = WIFI_CONNECTED;
      publishEvent("WIFI_CONNECTED", "Conexión WiFi establecida con credenciales guardadas");
      return;
    } else {
      Serial.println("\nError: No se pudo conectar con las credenciales guardadas");
    }
  } else {
    Serial.println("No hay credenciales WiFi guardadas.");
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
  delay(1000);
  
  
  Serial.println("Verificando conexión GPS...");
  delay(2000);
  
  int bytesAvailable = gpsSerial.available();
  Serial.println("Bytes disponibles en puerto GPS: " + String(bytesAvailable));
  
  if (bytesAvailable > 0) {
    Serial.println("GPS DETECTADO - Recibiendo datos");
    
    String initialData = "";
    for (int i = 0; i < min(100, bytesAvailable); i++) {
      char c = gpsSerial.read();
      initialData += c;
    }
    Serial.println("Datos iniciales (primeros 100 chars):");
    Serial.println(initialData);
  } else {
    Serial.println("¡ADVERTENCIA: GPS NO ESTÁ ENVIANDO DATOS!");
    Serial.println("Posibles problemas:");
    Serial.println("1. Verifica conexiones: GPS TX → GPIO14, GPS RX → GPIO16");
    Serial.println("2. Verifica alimentación del módulo GPS");
    Serial.println("3. Verifica que el LED del GPS parpadee");
  }
  
 
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 no conectado!");
  } else {
    Serial.println("MPU6050 listo");
  }

  
  sim800.begin(9600);
  initializeGSM();

  setupWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    if (configurarTiempoNTP()) {
      Serial.println("NTP configurado exitosamente");
    }
  }

  device = getDeviceInfo();
  setupOTA();

  Serial.println("\n=================================");
  Serial.println("Sistema de Detección de Accidentes");
  Serial.println("Versión: " + device.firmwareVersion);
  Serial.println("ID: " + device.chipId);
  Serial.println("GPS: " + String(bytesAvailable > 0 ? "DETECTADO" : "NO DETECTADO"));
  Serial.println("MPU6050: " + String(mpu.testConnection() ? "OK" : "ERROR"));
  Serial.println("GSM: " + String(checkGSMModule() ? "OK" : "ERROR"));
  Serial.println("=================================");
  

  mostrarMenuSerial();
  
 
  mostrarConfiguracionActual();

  publishEvent("SYSTEM_START", "Sistema de detección de accidentes iniciado");
}

void loop() {
    
    procesarComandoSerial();
    
   
    procesarSMSRecibido();
    
    
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);
        // Opcional: mostrar datos RAW (descomentar para debug)
        // if (c == '\n' || c == '\r') Serial.println();
        // Serial.write(c);
    }

    
    checkGPSStatus();

    
    checkGSMStatus();

    
    if (!otaContext.inProgress) {
        checkAccidents();
    }

   
    if (alert_triggered && digitalRead(BUTTON_PIN) == LOW) {
        delay(200); 
        digitalWrite(BUZZER_PIN, LOW);
        alert_triggered = false;
        alert_time = 0;
        publishEvent("ALERT_CANCELLED", "Alerta cancelada manualmente");
    }

   
    updateWiFiLED();
    updateWiFiStatus();
    handleButton();

    
    if (WiFi.status() == WL_CONNECTED) {
        if (!mqtt_client.connected()) {
            mqttConnected = false;
            unsigned long now = millis();
            if (now - lastReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
                lastReconnectAttempt = now;
                connectMQTT();
            }
        } else {
            mqtt_client.loop();
        }
    }

   
    static unsigned long lastGPSPublish = 0;
    if (millis() - lastGPSPublish > GPS_PUBLISH_INTERVAL && !otaContext.inProgress) {
        GPSData data;
        readGPSData(data);
        if (data.isValid) {
            publishGPSLocation(data);
        }
        lastGPSPublish = millis();
    }

    
    static unsigned long lastStatusPublish = 0;
    if (millis() - lastStatusPublish > STATUS_PUBLISH_INTERVAL && !otaContext.inProgress) {
        publishStatus();
        lastStatusPublish = millis();
    }

  
    verificarYSincronizarTiempo();

    
    if (otaInProgress) {
        ArduinoOTA.handle();
    }

   
    if (otaContext.inProgress && (millis() - otaContext.startTime > OTA_TIMEOUT)) {
        publishOTAError("Timeout en actualización OTA", otaContext.firmwareVersion);
        cleanupOTA();
        Serial.println("OTA timeout - actualización cancelada");
    }

    delay(10);
}