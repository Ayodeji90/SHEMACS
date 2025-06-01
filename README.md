# Smart Home Energy Monitoring and Control System

A comprehensive IoT-based system for monitoring and controlling home appliances, tracking energy consumption, and providing real-time data through both local display and cloud platforms.

## Features

- **Energy Monitoring**: Track voltage, current, power, and energy consumption for up to 4 devices
- **Cost Calculation**: Automatically calculate energy costs based on consumption
- **Remote Control**: Control devices remotely via Blynk mobile/web app
- **Environment Monitoring**: Measure room temperature, humidity, and light levels
- **Automatic Control**: Devices can be set to respond automatically to environmental conditions
- **Local Display**: View real-time data on an LCD display
- **Telegram Integration**: Receive periodic reports and control devices via Telegram
- **Persistent Storage**: Energy usage data is stored in EEPROM to survive power outages

## Hardware Requirements

- ESP32 Development Board
- 4 x ZMPT101B Voltage Sensor Modules
- 4 x ACS712 Current Sensor Modules
- 4 x Relay Modules (5V)
- DHT11 Temperature and Humidity Sensor
- LDR (Light Dependent Resistor)
- 16x2 I2C LCD Display (0x27 address)
- Breadboard and jumper wires
- Power supply (5V)

## Wiring Diagram

### ESP32 Pin Connections:

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| **Voltage Sensors (ZMPT101B)** | | |
| Device 1 | GPIO39 | Analog pin |
| Device 2 | GPIO25 | Analog pin |
| Device 3 | GPIO26 | Analog pin |
| Device 4 | GPIO27 | Analog pin |
| **Current Sensors (ACS712)** | | |
| Device 1 | GPIO33 | Analog pin |
| Device 2 | GPIO34 | Analog pin |
| Device 3 | GPIO35 | Analog pin |
| Device 4 | GPIO36 | Analog pin |
| **Relay Modules** | | |
| Device 1 | GPIO12 | Digital pin |
| Device 2 | GPIO13 | Digital pin |
| Device 3 | GPIO14 | Digital pin |
| Device 4 | GPIO15 | Digital pin |
| **DHT11 Sensor** | GPIO4 | Digital pin |
| **LDR Sensor** | GPIO32 | Analog pin |
| **I2C LCD** | | |
| SDA | GPIO21 | I2C Data (default for ESP32) |
| SCL | GPIO22 | I2C Clock (default for ESP32) |

### Sensor Wiring:

- **ZMPT101B Voltage Sensors**:
  - VCC → 5V
  - GND → GND
  - OUT → ESP32 analog pin

- **ACS712 Current Sensors**:
  - VCC → 5V
  - GND → GND
  - OUT → ESP32 analog pin

- **Relay Modules**:
  - VCC → 5V
  - GND → GND
  - IN → ESP32 digital pin
  - Connect device load through NO (Normally Open) and COM terminals

- **DHT11 Sensor**:
  - VCC → 3.3V
  - GND → GND
  - DATA → ESP32 GPIO4
  - 10K pull-up resistor between VCC and DATA

- **LDR Circuit**:
  - Connect one terminal of LDR to 3.3V
  - Connect other terminal to ESP32 GPIO32 and to a 10K resistor
  - Connect the other end of the 10K resistor to GND

- **I2C LCD Display**:
  - VCC → 5V
  - GND → GND
  - SDA → ESP32 GPIO21
  - SCL → ESP32 GPIO22

## Software Dependencies

This project uses PlatformIO for development. The following libraries are required:

- Blynk - for IoT connectivity and control
- EmonLib - for energy monitoring calculations
- DHT sensor library - for temperature and humidity sensing
- LiquidCrystal_I2C - for LCD display control
- ArduinoJson - for JSON data handling (Telegram integration)
- PubSubClient - for MQTT communication
- Adafruit Unified Sensor - dependency for DHT sensor library

## Setup Guide

### 1. PlatformIO Setup

1. Install [VS Code](https://code.visualstudio.com/)
2. Install the PlatformIO extension in VS Code
3. Clone or download this repository
4. Open the project folder in VS Code
5. PlatformIO should automatically recognize the project and set it up

### 2. Configuration

Open the `main.cpp` file and update the following values:

```cpp
// WiFi credentials
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

// Blynk authentication token
char auth[] = "YOUR_BLYNK_AUTH_TOKEN";

// Telegram Bot Token and Chat ID
#define BOT_TOKEN "YOUR_TELEGRAM_BOT_TOKEN"
#define CHAT_ID "YOUR_TELEGRAM_CHAT_ID"

// Energy cost rate (per kWh)
const float RATE_PER_KWH = 6.5; // Update with your local electricity rate
```

### 3. Blynk Setup

1. Download the Blynk app ([Android](https://play.google.com/store/apps/details?id=cc.blynk) or [iOS](https://apps.apple.com/us/app/blynk-iot-for-arduino-esp32/id808760481))
2. Create a new account if you don't have one
3. Create a new project and choose ESP32 as your hardware
4. The app will email you an Auth Token - copy this to the `auth[]` variable in your code
5. Set up the following widgets in your Blynk project:

   - **Gauge widgets** for voltage, current, power, energy, and cost monitoring:
     - V0-V4: Device 1 (voltage, current, power, kWh, cost)
     - V5-V9: Device 2 (voltage, current, power, kWh, cost)
     - V10-V14: Device 3 (voltage, current, power, kWh, cost)
     - V15-V19: Device 4 (voltage, current, power, kWh, cost)
     - V20-V22: Environmental data (temperature, humidity, light)

   - **Button widgets** for device control:
     - V30: Device 1 control (Bulb_01)
     - V31: Device 2 control (Bulb_02)
     - V32: Device 3 control (Socket)
     - V33: Device 4 control (Fan)

### 4. Telegram Bot Setup

1. Open Telegram and search for "BotFather"
2. Start a chat with BotFather and create a new bot using the `/newbot` command
3. Follow the prompts to set up your bot name and username
4. Once created, BotFather will provide a token - copy this to the `BOT_TOKEN` definition
5. Create a group or channel for notifications, or use your personal chat
6. To get your Chat ID:
   - Add the bot "@userinfobot" to your chat/group
   - It will automatically send you your Chat ID
   - Copy this to the `CHAT_ID` definition

### 5. Upload the Code

1. Connect your ESP32 to your computer via USB
2. In VS Code with PlatformIO, click the upload button (right arrow icon)
3. Wait for compilation and upload to complete
4. Open the Serial Monitor to verify connection and functionality

## Usage Instructions

### Blynk App Control

- Use the Blynk app to view real-time data for all devices
- Turn devices on/off using the button widgets (V30-V33)
- Monitor energy consumption and costs for each device
- View environmental data (temperature, humidity, light levels)

### LCD Display

The LCD display cycles through three different screens:
1. Voltage and Current for the first two devices
2. Power consumption for the first two devices
3. Total energy consumption and cost for all devices

### Telegram Commands

The system sends periodic reports to your Telegram chat with:
- Energy consumption for each device
- Total energy consumption and cost
- Current temperature and humidity

### Automatic Control

The system includes automatic control features based on environmental conditions:
- The light-dependent feature turns on/off devices based on ambient light levels
- Additional automation can be implemented based on temperature or humidity readings

## Calibration

For accurate energy monitoring, you may need to calibrate the sensors:

1. **Voltage Calibration**:
   - Measure the actual mains voltage with a multimeter
   - Adjust the `VOLTAGE_CALIBRATION` constant in the code
   - Default is 42.5, increase/decrease to match your readings

2. **Current Calibration**:
   - Use a known load with a specific current draw
   - Measure the actual current with a multimeter
   - Adjust the `CURRENT_CALIBRATION` constant in the code
   - Default is 1.80, increase/decrease to match your readings

## Troubleshooting

### Common Issues and Solutions

1. **ESP32 Won't Connect to WiFi**:
   - Verify your WiFi credentials are correct
   - Ensure your router is using 2.4GHz (ESP32 doesn't support 5GHz)
   - Check if your WiFi has special characters in SSID or password

2. **Blynk Connection Issues**:
   - Verify your Auth Token is correct
   - Check if Blynk server is reachable (internet connection)
   - Try restarting the Blynk app and ESP32

3. **Sensor Reading Issues**:
   - **No Voltage Readings**: Check ZMPT101B connections and calibration
   - **No Current Readings**: Verify ACS712 connections and ensure proper current flow
   - **Inaccurate Readings**: Adjust calibration constants in the code

4. **Relay Control Problems**:
   - Verify relay module connections
   - Check if relay is triggered (LED indicator on relay module)
   - Ensure your device is properly connected to relay terminals

5. **LCD Display Issues**:
   - Verify I2C address (default is 0x27, but may vary)
   - Check I2C connections (SDA and SCL)
   - Try running an I2C scanner sketch to confirm address

6. **Telegram Integration Not Working**:
   - Verify Bot Token and Chat ID are correct
   - Check internet connectivity
   - Ensure the bot has permission to post in your group (if using a group)

### Debug Tips

- Enable Serial output for debugging
- Check the Serial Monitor for error messages and connection status
- Use `Serial.println()` statements to track program flow
- Check the ESP32's LED indicators for connection status

## Safety Precautions

- **WARNING**: This project involves working with mains electricity which can be dangerous
- Always ensure proper isolation and insulation of all electrical connections
- Use appropriate enclosures for the final installation
- Consider using optoisolators or other safety measures between sensors and the ESP32
- When possible, use low voltage for testing before implementing with mains voltage
- Follow local electrical codes and regulations

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- OpenEnergyMonitor for the EmonLib library
- Blynk for their IoT platform
- All other open-source libraries used in this project

