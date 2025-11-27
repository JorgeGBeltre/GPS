# GPS Accident Detection System - ESP8266

A comprehensive IoT-based vehicle accident detection system that monitors vehicle movements, detects accidents using MPU6050 sensor, provides GPS location tracking, and sends emergency alerts via MQTT and SMS.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Installation](#installation)
- [Configuration](#configuration)
- [MQTT Topics](#mqtt-topics)
- [API Reference](#api-reference)
- [OTA Updates](#ota-updates)
- [Troubleshooting](#troubleshooting)

## Overview

This system uses an ESP8266 microcontroller to monitor vehicle movements in real-time. It detects accidents through impact analysis and rollover detection using an MPU6050 accelerometer/gyroscope, tracks location via GPS, and automatically sends emergency notifications through MQTT and SMS alerts.

## Features

### Core Functionality
- **Real-time Accident Detection**: Impact and rollover detection using MPU6050
- **GPS Tracking**: Location monitoring with TinyGPSPlus
- **Dual Alert System**: MQTT notifications + SMS emergency alerts
- **Wireless Connectivity**: WiFi with automatic reconnection
- **Secure Communication**: MQTTS with SSL/TLS encryption
- **Over-the-Air Updates**: Remote firmware updates via MQTT

### Technical Features
- **Configurable Sensitivity**: Adjustable impact sensitivity and rollover angles
- **Buzzer Alerts**: Audible notifications for accidents
- **LED Status Indicators**: Visual WiFi connection status
- **EEPROM Configuration**: Persistent settings storage
- **NTP Time Synchronization**: Accurate timestamping
- **GSM Module Support**: SIM800L for SMS functionality

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

### Prerequisites
- Arduino IDE with ESP8266 support
- Required libraries:
  - `ESP8266WiFi`
  - `WiFiManager`
  - `PubSubClient`
  - `ArduinoJson`
  - `TinyGPSPlus`
  - `MPU6050`
  - `SoftwareSerial`

### Setup Steps
1. Clone the repository
2. Install required libraries
3. Configure hardware connections
4. Update configuration in `config.h`
5. Upload firmware to ESP8266

## Configuration

### WiFi Setup
- **First Boot**: Press and hold button for 3+ seconds to enter AP mode
- **AP SSID**: `VESIS-{CHIP_ID}`
- **AP Password**: `12345678`
- **Auto-reconnect**: Automatic reconnection with saved credentials

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

## Troubleshooting

### Common Issues

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

### Debug Mode
Enable debug output by defining `ENABLE_DEBUG` in `config.h`:
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

## Support

For issues and questions:
1. Check the troubleshooting section
2. Verify hardware connections
3. Ensure proper power supply
4. Review MQTT broker configuration

---

**Version**: v0.0.4  
**Hardware**: ESP8266 + MPU6050 + GPS + GSM  

##Contact
Author: Jorge Gaspar Beltre Rivera
Project: GPS Accident Detection System - ESP8266
GitHub: github.com/JorgeBeltre
