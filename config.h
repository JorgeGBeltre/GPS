#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>
#include <cstddef>
#include <cstdint>

#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#define LOG_DEBUG(msg)                                                         \
  Serial.print("[DBG] ");                                                      \
  Serial.println(msg)
#define LOG_INFO(msg)                                                          \
  Serial.print("[INFO] ");                                                     \
  Serial.println(msg)
#define LOG_WARN(msg)                                                          \
  Serial.print("[WARN] ");                                                     \
  Serial.println(msg)
#define LOG_ERROR(msg)                                                         \
  Serial.print("[ERR] ");                                                      \
  Serial.println(msg)
#else
#define LOG_DEBUG(msg)
#define LOG_INFO(msg)
#define LOG_WARN(msg)
#define LOG_ERROR(msg)
#endif

constexpr unsigned long WIFI_CONNECT_TIMEOUT = 3000;
constexpr const char *ntpServer1 = "pool.ntp.org";
constexpr const char *ntpServer2 = "time.nist.gov";
constexpr long gmtOffset_sec = -4L * 3600L;
constexpr int daylightOffset_sec = 0;

constexpr const char *MQTT_SERVER = "example.com";
constexpr int MQTT_PORT = 8084;
constexpr const char *API_KEY = "EXAMPLE_API_KEY";
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 10000UL;
constexpr size_t MQTT_BUFFER_SIZE = 40960;
constexpr unsigned long MQTT_KEEP_ALIVE = 300UL;

/*
constexpr const char *root_ca_pem =
    "-----BEGIN CERTIFICATE-----\n" // Your SSL certificate here
    "-----END CERTIFICATE-----\n";
*/

constexpr uint8_t BUTTON_PIN = 0;  // GPIO0 (D3)
constexpr uint8_t wifiLed = 2;     // GPIO2 (D4) - Built-in LED
constexpr uint8_t BUZZER_PIN = 15; // GPIO15 (D8)
constexpr uint8_t GSM_RX_PIN = 13; // GPIO13 (D7)
constexpr uint8_t GSM_TX_PIN = 12; // GPIO12 (D6)
constexpr uint8_t GPS_RX_PIN = 14; // GPIO14 (D5)
constexpr uint8_t GPS_TX_PIN = 16; // GPIO16 (D0)
constexpr uint8_t SDA_PIN = 4;     // GPIO4  (D2)
constexpr uint8_t SCL_PIN = 5;     // GPIO5  (D1)

constexpr int SMS_MAX_AUTH_ATTEMPTS = 3;
constexpr unsigned long SMS_LOCKOUT_TIME = 300000UL; // 5 min
constexpr int MAX_PHONE_LENGTH = 21;
struct AccidentConfig {
  int sensitivity = 2000;
  int rollover_angle = 60;
  unsigned long alert_delay = 30000UL;
  char emergency_phone[MAX_PHONE_LENGTH] = "+18296741199"; // Example
  bool sms_enabled = true;
};

constexpr unsigned long ACCIDENT_COOLDOWN = 60000UL; // 60 seconds

constexpr unsigned long STATUS_PUBLISH_INTERVAL = 3600000UL;
constexpr unsigned long GPS_PUBLISH_INTERVAL = 30000UL;
constexpr unsigned long TIME_SYNC_INTERVAL = 3600000UL;
constexpr unsigned long ACCIDENT_CHECK_INTERVAL = 2000UL;
constexpr int FAST_BLINK_INTERVAL = 100;
constexpr int SLOW_BLINK_INTERVAL = 400;
constexpr uint8_t BUTTON_LONG_PRESS_TIME = 3;

constexpr const char *FIRMWARE_VERSION = "v0.0.5";

constexpr unsigned long OTA_TIMEOUT = 420000UL;

constexpr int EEPROM_SIZE = 512;
constexpr int CONFIG_EEPROM_ADDR = 0;

constexpr size_t SMS_BUFFER_MAX = 512;
constexpr unsigned long SMS_TIMEOUT_MS = 10000UL;
constexpr size_t SERIAL_CMD_MAX_LEN = 128;
constexpr int ACC_FILTER_SIZE = 5;

#endif // CONFIG_H