#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>
#include <EEPROM.h>

#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif

const unsigned long WIFI_CONNECT_TIMEOUT = 3000;

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = -4 * 3600;
const int daylightOffset_sec = 0;

const char* MQTT_SERVER = "example.com";
const int MQTT_PORT = 8084;
const char* MQTT_URI = "example.com:8084";
const char* API_KEY = "EXAMPLE_API_KEY";
const unsigned long MQTT_RECONNECT_INTERVAL = 10000;
const int MQTT_SOCKET_TIMEOUT = 300;
const size_t MQTT_BUFFER_SIZE = 40960;
const unsigned long MQTT_KEEP_ALIVE = 300;

const char* root_ca_pem = \
"-----BEGIN CERTIFICATE-----\n" \
//Propio certificado SSL
"-----END CERTIFICATE-----\n";

// ESP8266 12S
const uint8_t BUTTON_PIN = 0;     // GPIO0 (D3)
const uint8_t wifiLed = 2;        // GPIO2 (D4) - Built-in LED
const uint8_t BUZZER_PIN = 15;    // GPIO15 (D8)
const uint8_t GSM_RX_PIN = 13;    // GPIO13 (D7) - GSM RX
const uint8_t GSM_TX_PIN = 12;    // GPIO12 (D6) - GSM TX
const uint8_t GPS_RX_PIN = 14;    // GPIO14 (D5) - GPS RX
const uint8_t GPS_TX_PIN = 16;    // GPIO16 (D0) - GPS TX


const uint8_t SDA_PIN = 4;        // GPIO4 (D2) - I2C SDA
const uint8_t SCL_PIN = 5;        // GPIO5 (D1) - I2C SCL

struct AccidentConfig {
  int sensitivity = 2000;
  int rollover_angle = 60;
  unsigned long alert_delay = 30000;
  String emergency_phone = "+18296741199"; //Your phoneNumber here that's number is only example +18290000000 
  bool sms_enabled = true;
};

const unsigned long STATUS_PUBLISH_INTERVAL = 3600000;
const unsigned long GPS_PUBLISH_INTERVAL = 30000;
const unsigned long TIME_SYNC_INTERVAL = 3600000;
const unsigned long ACCIDENT_CHECK_INTERVAL = 2000;
const int FAST_BLINK_INTERVAL = 100;
const int SLOW_BLINK_INTERVAL = 400;
const uint8_t BUTTON_LONG_PRESS_TIME = 3;  

const char* FIRMWARE_VERSION = "v0.0.4";

const unsigned long OTA_TIMEOUT = 420000;

const int EEPROM_SIZE = 512;
const int CONFIG_EEPROM_ADDR = 0;

#endif