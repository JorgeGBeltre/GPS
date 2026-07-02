# GPS Accident Detection System - ESP8266

[![Platform](https://img.shields.io/badge/platform-ESP8266-orange)](https://www.espressif.com/en/products/socs/esp8266)
[![MQTT 3.1.1](https://img.shields.io/badge/MQTT-3.1.1-orange)](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/JorgeGBeltre/GPS)

A comprehensive IoT-based vehicle accident detection system that monitors vehicle movements, detects accidents using MPU6050 sensor, provides GPS location tracking, and sends emergency alerts via MQTT and SMS.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Backend Implementation](#backend-implementation)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Installation](#installation)
- [Configuration Commands](#configuration-commands)
- [MQTT Topics](#mqtt-topics)
- [API Reference](#api-reference)
- [OTA Updates](#ota-updates)
- [Common Issues](#common-issues)
- [Troubleshooting](#troubleshooting)
- [Broker Compatibility](#broker-compatibility)
- [License](#license)
- [Contact](#contact)
- [Support](#support)

## Overview

This system uses an ESP8266 microcontroller to monitor vehicle movements in real-time. It detects accidents through impact analysis and rollover detection using an MPU6050 accelerometer/gyroscope, tracks location via GPS, and automatically sends emergency notifications through MQTT and SMS alerts.

### System Evolution
The project evolved from a monolithic prototype → a modular sketch (v0.0.5) →
a **PlatformIO Clean Architecture** project. The latest overhaul organizes the
firmware into four layers with dependencies pointing **inward only**
(Presentation → Application → Domain, and Infrastructure → Domain; the Domain
layer depends on nothing), delivering higher reliability, testable business
logic, and hardware that can be swapped without touching the core.

## Architecture

The firmware is organized into four layers under `src/`:

```
src/
├── domain/                         # Enterprise rules — no hardware, no libs
│   ├── entities/
│   │   ├── AccidentEvent.h         # (hardware-free, host-testable)
│   │   ├── GpsLocation.h
│   │   ├── DeviceInfo.h
│   │   └── SystemTypes.h           # WiFiStatus, OTAContext
│   └── interfaces/                 # Pure abstractions (ports)
│       ├── IGpsProvider.h
│       ├── IGsmAlertSender.h
│       ├── IMqttTransport.h
│       ├── IAccidentDetector.h
│       ├── IConfigRepository.h
│       └── IClock.h
│
├── application/                    # Use cases — depends only on interfaces
│   ├── AccidentAlgorithm.{h,cpp}   # Pure STA/LTA + rollover math (host-tested)
│   └── services/
│       └── AccidentService.{h,cpp} # Orchestrates detect → alert → publish
│
├── infrastructure/                 # Adapters — concrete impls of the ports
│   ├── gps/TinyGpsPlusProvider.{h,cpp}
│   ├── gsm/Sim800lAlertSender.{h,cpp}
│   ├── mqtt/PubSubClientTransport.{h,cpp}
│   ├── sensors/Mpu6050Detector.{h,cpp}
│   ├── persistence/LittleFsConfigRepository.{h,cpp}
│   └── system/{EspClock,EspSysUtils}.{h,cpp}
│
└── main.cpp                        # Presentation / composition root

include/config.h                    # Shared pins & constants (global)
test/test_accident_algorithm/       # Native unit tests (no hardware)
data/config.json                    # Default LittleFS config (uploadfs)
legacy/                             # Original flat sketch (kept, NOT compiled)
```

### Key ideas

- **Dependency inversion.** `AccidentService` and `Mpu6050Detector` never name a
  concrete class; they receive `IGpsProvider`, `IGsmAlertSender`,
  `IMqttTransport`, `IConfigRepository` and `IClock` through their constructors.
  Only `main.cpp` (the composition root) wires the concrete implementations
  together.
- **Swappable adapters.** Changing the GPS chip (NEO-6M → NEO-8M) or the GSM
  modem (SIM800L → SIM7600) means writing one new class in `infrastructure/` and
  changing one line in `main.cpp` — no business-logic changes.
- **Testable core.** `AccidentAlgorithm` was split out of the MPU6050 driver so
  the impact/rollover logic runs on the host with no hardware. See
  `test/test_accident_algorithm/`.

### Legacy module mapping

The original flat modules were reorganized into the layers above (the old files
remain under `legacy/` for reference and are not compiled):

| Legacy file          | Now lives in                                                        |
|----------------------|---------------------------------------------------------------------|
| `GPS.ino`            | `src/main.cpp` (composition root)                                   |
| `Types.h`            | `src/domain/entities/*` + `SystemTypes.h`                           |
| `config.h`           | `include/config.h`                                                  |
| `AccidentMonitor.*`  | `src/infrastructure/sensors/Mpu6050Detector.*` + `src/application/AccidentAlgorithm.*` |
| `GpsModule.*`        | `src/infrastructure/gps/TinyGpsPlusProvider.*`                      |
| `GsmModule.*`        | `src/infrastructure/gsm/Sim800lAlertSender.*`                       |
| `MqttClientModule.*` | `src/infrastructure/mqtt/PubSubClientTransport.*`                   |
| `DeviceConfig.*`     | `src/infrastructure/persistence/LittleFsConfigRepository.*`         |
| `SysUtils.*`         | `src/infrastructure/system/EspClock.*` + `EspSysUtils.*`            |

## Features

### Core Functionality
- **Real-time Accident Detection**: Impact and rollover detection using MPU6050
- **GPS Tracking**: Location monitoring with TinyGPSPlus
- **Dual Alert System**: MQTT notifications + SMS emergency alerts
- **Wireless Connectivity**: WiFi with automatic reconnection
- **Secure Communication**: MQTTS with SSL/TLS encryption
- **Over-the-Air Updates**: Remote firmware updates via MQTT
- **Multiple Configuration Methods**: Serial, SMS, MQTT, and Web Interface

### Technical Features
- **Modular C++ Design (OOD)**: Decoupled hardware managers for high scalability.
- **Configurable Sensitivity**: Adjustable impact sensitivity and rollover angles.
- **Buzzer Alerts**: Audible notifications for accidents.
- **LED Status Indicators**: Visual WiFi connection status.
- **LittleFS JSON Configuration**: Persistent settings stored as JSON (modern alternative to EEPROM).
- **NTP Time Synchronization**: Accurate timestamping for event logs.
- **GSM Module Support**: SIM800L with automated command processing.
- **Multi-protocol Command Interface**: Serial, SMS, and MQTT configuration callbacks.

## Hardware Requirements

### Microcontroller
- **ESP8266 NodeMCU** (ESP-12E/ESP-12S)

### Sensors & Modules
- **MPU6050**: Accelerometer and gyroscope for motion detection
- **GPS Module**: NEO-6M or compatible for location tracking
- **GSM Module**: SIM800L for SMS alerts
- **Buzzer**: Piezo buzzer for audible alerts
- **LED**: Status indicator LED
- **Button**: Physical button for configuration

## Pin Configuration

| Component | GPIO Pin | NodeMCU Pin | Function |
|-----------|----------|-------------|----------|
| Button | GPIO0 | D3 | Configuration/Reset |
| WiFi LED | GPIO2 | D4 | Status indicator |
| Buzzer | GPIO15 | D8 | Audible alerts |
| GSM RX | GPIO13 | D7 | SIM800L RX |
| GSM TX | GPIO12 | D6 | SIM800L TX |
| GPS RX | GPIO14 | D5 | GPS Module RX |
| GPS TX | GPIO16 | D0 | GPS Module TX |
| I2C SDA | GPIO4 | D2 | MPU6050 SDA |
| I2C SCL | GPIO5 | D1 | MPU6050 SCL |

## Installation

This is a **PlatformIO** project. Use the PlatformIO IDE extension (VS Code) or
the `pio` CLI. Dependencies are declared in `platformio.ini` and fetched
automatically — no manual library installation needed.

### Prerequisites
- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) (CLI) or the PlatformIO IDE extension for VS Code
- Library dependencies (auto-installed from `platformio.ini`):
  - `mikalhart/TinyGPSPlus`
  - `knolleary/PubSubClient`
  - `bblanchon/ArduinoJson` (v6)
  - `electroniccats/MPU6050`
- `SoftwareSerial`, `ESP8266WiFi`, `WiFiClientSecure`, `ArduinoOTA`, `LittleFS`
  and `EEPROM` ship with the ESP8266 Arduino core and are **not** listed in
  `lib_deps`.

### Setup Steps
1. Clone the repository and open the `GPS` folder in VS Code (PlatformIO) or a terminal.
2. Configure hardware connections (see [Pin Configuration](#pin-configuration)).
3. Update settings in `include/config.h` (WiFi certificate, MQTT server, pins).
4. Build, upload the filesystem, and flash the firmware (commands below).

### Build & test

```bash
pio run                 # build firmware for nodemcuv2 (ESP8266)
pio run -t upload       # flash firmware
pio run -t uploadfs     # upload data/ (config.json) to LittleFS
pio test -e native      # run host unit tests for AccidentAlgorithm (needs a host GCC — MinGW-w64 on Windows, or WSL)
pio device monitor      # serial monitor @ 115200
```

### Notes / deviations from the original sketch

1. `MQTT_TOPIC_CONFIG` was **referenced but never defined** in the original
   `config.h` (the sketch would not compile as-is). It is now defined in
   `include/config.h` as the PROGMEM `"config"` topic — adjust if needed.
2. The original `loop()` passed an always-empty `GPSData` to the SMS alert, so
   alerts never carried a location. `AccidentService::process()` now reads the
   live fix from `IGpsProvider` before alerting. This is a behavior change (an
   intentional bug fix); revert inside `AccidentService.cpp` if undesired.
3. `Serial`/OTA command handling beyond the `STATUS` SMS was not present in the
   original `loop()` and was **not invented** here. The command set documented
   below is a natural follow-up to wire into a dedicated `CommandRouter`.

## Configuration Commands

The system includes multiple ways to configure accident detection parameters:

### 1. Serial Port Commands (Serial Monitor)

Connect to the serial port (115200 baud) and use the following commands:

#### Basic Configuration
```bash
set phone +18291234567         # Change emergency phone number
set sensitivity 2500           # Change sensitivity (500-10000)
set angle 60                   # Change rollover angle (10-90°)
set delay 30000                # Change alert delay (1000-600000ms)
set sms on                     # Enable/disable SMS alerts (on/off)
save                           # Save configuration to EEPROM
```

#### System Commands
```bash
help or ?                      # Show command menu
config or show                 # Show current configuration
status                         # Publish status via MQTT
location                       # Get and publish GPS location
test_sms                       # Send test SMS
sms_test                       # Send SMS test to configured number
```

#### Diagnostics
```bash
wifi                           # Show WiFi information
mqtt                           # Show/connect MQTT status
gsm                            # Check GSM module
gps                            # Show GPS information
gps_debug                      # Complete GPS diagnostic
gps_raw                        # Show RAW GPS data for 5 seconds
reset                          # Restart device
```

### 2. SMS Commands

Send SMS to the configured emergency phone number:

#### Available SMS Commands
```
HELP or AYUDA or ?             # Show command menu
STATUS or ESTADO               # System status
LOCATION or UBICACION or LOC   # Current GPS location
CONFIG or CONFIGURACION        # Current configuration
TEST or TEST_SMS               # Test SMS to emergency number
```

#### SMS Configuration Commands
```
SET PHONE [number]             # Change emergency phone number
SET SENS [value]               # Change sensitivity (500-10000)
SET ANGLE [degrees]            # Change rollover angle (10-90)
SET DELAY [ms]                 # Change alert delay (1000-600000)
SET SMS [ON/OFF]               # Enable/disable SMS
SAVE or GUARDAR                # Save configuration
```

#### System SMS Commands
```
WIFI or WIFI INFO              # WiFi information
GPS                            # GPS status
GSM                            # GSM module status
MQTT                           # MQTT status
RESET or REINICIAR             # Restart device
```

### 3. MQTT Commands

#### Direct Commands
Send to topic `commands/{device_id}`:
- `reset`: Restart the device
- `status`: Request status report
- `location`: Request immediate GPS location
- `get_config`: Get current configuration

#### JSON Configuration via MQTT
Send JSON to topic `config`:
```json
{
  "sensitivity": 2500,
  "rollover_angle": 60,
  "alert_delay": 30000,
  "sms_enabled": true,
  "emergency_phone": "+18291234567"
}
```

### 4. Web Interface Configuration (WiFiManager)

1. **Activate AP mode**: Press and hold the physical button for 3+ seconds
2. **Connect**: WiFi SSID: `VESIS-{CHIP_ID}`, Password: `12345678`
3. **Configure**: Enter your WiFi SSID and password
4. **Restart**: The device will restart automatically

### Default Values and Ranges

| Parameter | Default Value | Acceptable Range |
|-----------|---------------|------------------|
| Sensitivity | 2000 | 500 - 10000 |
| Rollover Angle | 60° | 10° - 90° |
| Alert Delay | 30000ms | 1000 - 600000ms |
| Emergency Phone | - | Minimum 10 digits |
| SMS Enabled | true | true/false |

### Configuration Examples

#### Example 1: Basic Serial Configuration
```bash
set phone +18295551234
set sensitivity 3000
set angle 70
set delay 15000
set sms on
save
```

#### Example 2: SMS Configuration
```
SET PHONE +18295556789
SET SENS 4000
SET ANGLE 65
SET DELAY 20000
SET SMS ON
SAVE
```

#### Example 3: SMS Verification
```
HELP         # View all commands
STATUS       # View current status
CONFIG       # View configuration
LOCATION     # Get location
```

### Configuration Persistence

- **LittleFS Storage (v0.0.5)**: Configuration is now stored in a `config.json` file. This provides atomic writes and human-readable settings via the file system.
- **EEPROM Migration**: The system has transitioned from raw EEPROM addresses to the LittleFS filesystem for improved reliability and data integrity.
- **Auto-Load**: Current settings are automatically loaded from `/config.json` on startup.
- **Validation**: Strict range validation is performed before any change is persisted.
- **Data Upload**: Remember to upload the `data` folder using the **ESP8266 LittleFS Data Upload** tool before the first run.

### Important Notes

1. **Modular Architecture**: Commands are now handled via a clean callback system (`onSmsReceived`, `onMqttEvent`), making the system highly extensible.
2. **SMS Authorization**: Only the configured emergency number can send SMS commands.
3. **Phone Format**: Numbers must have at least 10 digits and can include `+`, spaces, and dashes.
4. **Restart Required**: Certain changes require a restart to be fully applied to hardware modules.
5. **Confirmation**: Successful configuration changes trigger confirmations via MQTT/SMS/Serial.

### Advanced Diagnostic Commands

```bash
gps_debug     # Complete GPS diagnostic
gps_raw       # View raw NMEA data
gps           # Current GPS status
gsm           # Verify GSM module
```

These commands provide detailed information for troubleshooting hardware and connectivity issues.

### Accident Detection Settings
```cpp
struct AccidentConfig {
  int sensitivity = 2000;        // Impact sensitivity threshold
  int rollover_angle = 60;       // Rollover detection angle
  unsigned long alert_delay = 30000; // Alert delay in milliseconds
  String emergency_phone = "+18290000000"; // Emergency contact number
  bool sms_enabled = true;       // Enable/disable SMS alerts
};
```

### MQTT Configuration
```cpp
const char* MQTT_SERVER = "example.com";
const int MQTT_PORT = 8084;
const char* API_KEY = "EXAMPLE_API_KEY";
```

## MQTT Topics

### Communication Channels
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `telemetry` | Outgoing | GPS location data |
| `events` | Outgoing | System events and accident alerts |
| `commands` | Incoming | Device control commands |
| `ota` | Incoming | Firmware update commands |
| `status` | Outgoing | Device status information |
| `config` | Bidirectional | Configuration management |

## API Reference

### Command Processing
The system responds to various MQTT commands:

#### System Commands
- `reset`: Restart the device
- `status`: Request current status report
- `location`: Request immediate GPS location
- `get_config`: Retrieve current configuration

#### Configuration Commands
Send JSON to `config` topic:
```json
{
  "sensitivity": 2000,
  "rollover_angle": 60,
  "alert_delay": 30000,
  "sms_enabled": true,
  "emergency_phone": "+1234567890"
}
```

### Event Publishing
The system publishes structured JSON events:

#### Accident Event Example
```json
{
  "Device": {
    "status": "connected",
    "ChipId": "ABC123",
    "MacAddress": "AA:BB:CC:DD:EE:FF",
    "IPAddress": "192.168.1.100",
    "ChipType": "ESP8266",
    "FirmwareVersion": "v0.0.4"
  },
  "Timestamp": "2024-01-15T10:30:45.123Z",
  "Details": {
    "chipId": "ABC123",
    "accident": {
      "impact_detected": true,
      "rollover_detected": false,
      "impact_magnitude": 2450.5,
      "roll_angle": 15.2,
      "pitch_angle": 5.8,
      "gps_data": {
        "latitude": 40.7128,
        "longitude": -74.0060,
        "is_valid": true
      },
      "config": {
        "sensitivity": 2000,
        "rollover_angle": 60,
        "alert_delay": 30000,
        "sms_enabled": true
      }
    }
  }
}
```

## OTA Updates

### Firmware Update Process
The system supports chunked OTA updates via MQTT:

#### Update Initiation
```json
{
  "EventType": "UpdateFirmwareDevice",
  "Details": {
    "FirmwareVersion": "v1.0.0",
    "Base64Part": "[base64_encoded_data]",
    "PartIndex": 1,
    "TotalParts": 10,
    "TotalSize": 500000
  }
}
```

#### Update Progress Tracking
- Progress Updates: Published every 25% completion
- Error Handling: Automatic rollback on failure
- Memory Management: Optimized for ESP8266 constraints
- Timeout Protection: 7-minute update timeout

### OTA Features
- Chunked Transfer: Handles large firmware files in parts
- Base64 Encoding: Secure data transmission
- Progress Reporting: Real-time update status
- Error Recovery: Automatic cleanup on failure
- Version Validation: Prevents downgrade attacks

## Common Issues

#### WiFi Connection Problems
- Symptoms: Rapid LED blinking, no MQTT connection
- Solutions: 
  - Check WiFi credentials
  - Use physical button to enter AP mode
  - Verify signal strength

#### GPS Not Working
- Symptoms: Invalid location data, no satellite lock
- Solutions:
  - Check antenna connection
  - Ensure outdoor placement
  - Verify baud rate (9600)

#### MQTT Connection Issues
- Symptoms: No message publishing/subscription
- Solutions:
  - Verify MQTT server credentials
  - Check SSL certificate configuration
  - Confirm network connectivity

#### Accident Detection False Positives
- Solutions:
  - Adjust sensitivity in configuration
  - Modify vibration damping parameters
  - Calibrate MPU6050 sensor

#### Command Interface Issues
If commands don't work:
1. Verify MQTT/WiFi/GPS connection
2. Confirm SMS number is correctly configured
3. Check syntax (uppercase/lowercase)
4. Verify available memory (`status` command)
5. Check serial logs for specific errors

### Debug Mode
Enable debug output by defining `ENABLE_DEBUG` in `include/config.h`:
```cpp
#define ENABLE_DEBUG
```

### Status Indicators
| LED Pattern | Status |
|-------------|--------|
| Solid ON | WiFi connected |
| Slow Blink | WiFi connecting |
| Fast Blink | WiFi disconnected |
| Buzzer Sound | Accident detected |

## Troubleshooting

For issues and questions:
1. Check the troubleshooting section
2. Verify hardware connections
3. Ensure proper power supply
4. Review MQTT broker configuration
5. Use diagnostic commands for detailed debugging


---

## Backend Implementation

To use MQTTOTA in your project, you'll need an MQTT server to manage OTA updates. You can implement your own backend using our reference repository:

**MQTT Broker for OTA Updates**
- **Repository:** [github.com/Ruben890/Mqtt-Broker](https://github.com/Ruben890/Mqtt-Broker)
- **Description:** Complete backend for managing OTA updates via MQTT/MQTTS
- **Features:**
  - Configurable MQTT server
  - IoT device management
  - Firmware update delivery
  - OTA progress tracking
  - Error handling and retry mechanisms

**Steps to use the backend:**
1. Clone the backend repository
2. Configure the MQTT broker according to your needs
3. Implement the update delivery logic
4. Connect your ESP32 devices to the broker
5. Manage OTA updates from a centralized interface

**Example workflow:**
```javascript
// From your backend
1. Prepare firmware in base64 format
2. Publish MQTT message to target device
3. Monitor progress via callbacks
4. Confirm successful completion
5. Log results in database
```

The backend provides a scalable architecture for managing multiple devices simultaneously, with support for mass updates and firmware version management.

---

## Broker Compatibility

Tested with:
- [Mosquitto](https://mosquitto.org/) 2.0+
- [EMQX](https://www.emqx.com/) 5.0+
- [HiveMQ Cloud](https://www.hivemq.com/)
- [AWS IoT Core](https://aws.amazon.com/iot-core/)
- ESP-IDF MQTT client (used in `BasicOTA` example)

---


## License

Licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

## Contact

Author: **Jorge Gaspar Beltre Rivera**  
Project: **GPS Accident Detection System - ESP8266**

 [![GitHub](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/JorgeGBeltre)
 [![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/jorge-gaspar-beltre-rivera/)
 [![Email](https://img.shields.io/badge/Email-EA4335?style=for-the-badge&logo=gmail&logoColor=white)](mailto:Jorgegaspar3021@gmail.com)

---

##  Support

This project is developed independently.

Even a small contribution helps me dedicate more time to development, testing, and releasing new features.

 [![Buy Me a Coffee](https://img.shields.io/badge/Buy_Me_a_Coffee-FFDD00?style=for-the-badge&logo=buy-me-a-coffee&logoColor=black)](https://www.paypal.com/donate/?hosted_button_id=2VLA8BWT967LU)