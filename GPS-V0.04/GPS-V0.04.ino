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
    
    Serial.println("Módulo GSM inicializado");
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
  
  char marker[4];
  for (int i = 0; i < 4; i++) {
    marker[i] = EEPROM.read(CONFIG_EEPROM_ADDR + i);
  }
  
  if (String(marker) != "CFG") {
    Serial.println("Configuración por defecto - primera vez");
    saveConfig(); 
    return;
  }
  
  int addr = CONFIG_EEPROM_ADDR + 4;
  
  
  accidentConfig.sensitivity = EEPROM.read(addr) | (EEPROM.read(addr + 1) << 8);
  addr += 2;
  
  
  accidentConfig.rollover_angle = EEPROM.read(addr);
  addr += 1;
  
  
  accidentConfig.alert_delay = 0;
  for (int i = 0; i < 4; i++) {
    accidentConfig.alert_delay |= (unsigned long)EEPROM.read(addr + i) << (8 * i);
  }
  addr += 4;
  
  
  accidentConfig.sms_enabled = EEPROM.read(addr);
  addr += 1;
  
  
  String phone = "";
  char ch;
  while (addr < EEPROM_SIZE && (ch = EEPROM.read(addr)) != 0 && addr < CONFIG_EEPROM_ADDR + 100) {
    phone += ch;
    addr++;
  }
  accidentConfig.emergency_phone = phone;
  
  Serial.println("Configuración cargada de EEPROM:");
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

 
  gpsSerial.begin(9600);
  Serial.println("GPS inicializado");

 
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 no conectado!");
  } else {
    Serial.println("MPU6050 listo");
  }

  /*
  sim800.begin(9600);
  delay(1000);
  sim800.println("AT");
  delay(1000);
  sim800.println("ATE1");
  delay(1000);
  sim800.println("AT+CMGF=1");
  delay(1000);
  Serial.println("GSM inicializado");
  sim800.begin(9600);
  initializeGSM();
*/ 
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

  Serial.println("\nSistema de Detección de Accidentes - OTA Simplificado");
  Serial.println("Client ID: " + device.chipId);

  publishEvent("SYSTEM_START", "Sistema de detección de accidentes iniciado");
}

void loop() {
    
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

   
    checkGSMStatus();

   
    while (sim800.available()) {
        String response = sim800.readString();
        Serial.print("GSM: ");
        Serial.println(response);
    }

    
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