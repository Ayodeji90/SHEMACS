// First, creating a new implementation with proper device management

#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_TEMPLATE_NAME"
#define BLYNK_PRINT Serial

// Telegram Bot settings
#define BOT_TOKEN "YOUR_TELEGRAM_BOT_TOKEN"
#define CHAT_ID "YOUR_TELEGRAM_CHAT_ID"

// WiFi credentials
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

// Blynk authentication token
char auth[] = "YOUR_BLYNK_AUTH_TOKEN";

#include <Arduino.h>
#include <EmonLib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// Device structure to manage each device''s data
struct Device {
    const char* name;
    const int voltagePin;
    const int currentPin;
    const int relayPin;
    float voltage;
    float current;
    float power;
    float kWh;
    float cost;
    bool relayState;
    EnergyMonitor emon;
};

// EEPROM address mapping for each device
struct EEPROMAddresses {
    int kWh;
    int cost;
    int relayState;
};

// Pin Definitions
const int VOLTAGE_PINS[] = {39, 25, 26, 27};  // ZMPT101B pins
const int CURRENT_PINS[] = {33, 34, 35, 36};  // ACS712 pins
const int RELAY_PINS[] = {12, 13, 14, 15};    // Relay control pins
const int DHT_PIN = 4;                        // DHT11 sensor
const int LDR_PIN = 32;                       // Light sensor
const int NUM_DEVICES = 4;

// Initialize DHT sensor
DHT dht(DHT_PIN, DHT11);

// Initialize LCD (I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Calibration constants
const float VOLTAGE_CALIBRATION = 42.5;
const float CURRENT_CALIBRATION = 1.80;
const float RATE_PER_KWH = 6.5;

// EEPROM size calculation
const int EEPROM_SIZE = 256;  // Increased to handle all devices

// Initialize devices array
Device devices[NUM_DEVICES] = {
    {"Bulb_01", VOLTAGE_PINS[0], CURRENT_PINS[0], RELAY_PINS[0], 0, 0, 0, 0, 0, false, EnergyMonitor()},
    {"Bulb_02", VOLTAGE_PINS[1], CURRENT_PINS[1], RELAY_PINS[1], 0, 0, 0, 0, 0, false, EnergyMonitor()},
    {"Socket", VOLTAGE_PINS[2], CURRENT_PINS[2], RELAY_PINS[2], 0, 0, 0, 0, 0, false, EnergyMonitor()},
    {"Fan", VOLTAGE_PINS[3], CURRENT_PINS[3], RELAY_PINS[3], 0, 0, 0, 0, 0, false, EnergyMonitor()}
};

// EEPROM addresses for each device
EEPROMAddresses eepromAddr[NUM_DEVICES] = {
    {0, 4, 8},    // Device 1 addresses
    {12, 16, 20}, // Device 2 addresses
    {24, 28, 32}, // Device 3 addresses
    {36, 40, 44}  // Device 4 addresses
};

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long lastDisplayChange = 0;
unsigned long lastTelegramUpdate = 0;
int currentDisplay = 0;
const int DISPLAY_PAGES = 3;  // Number of display pages (0: V/I, 1: Power, 2: Energy/Cost)

// Environment monitoring
float roomTemperature = 0;
float roomHumidity = 0;
int lightLevel = 0;

// Function prototypes
// Function prototypes
void initializeDevices();
void readSensors();
void updateEEPROM();
void updateDisplay();
void updateBlynk();
void sendTelegramUpdate();
void handleEnvironmentControl();
void processMeasurements();
void sendToTelegram(String message);

// Timer object for scheduling tasks
BlynkTimer timer;

// Continuing from previous code...

void setup() {
    Serial.begin(115200);
    
    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    
    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("Initializing...");
    
    // Initialize DHT sensor
    dht.begin();
    
    // Initialize all pins and devices
    initializeDevices();
    
    // Connect to WiFi
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    // Initialize Blynk
    Blynk.begin(auth, ssid, pass);
    
    // Initialize environment sensors
    pinMode(LDR_PIN, INPUT);
    
    // Configure the time
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    // Setup timer intervals
    timer.setInterval(2000L, processMeasurements);    // Process measurements every 2 seconds
    timer.setInterval(5000L, updateDisplay);          // Update display every 5 seconds
    timer.setInterval(10000L, handleEnvironmentControl); // Check environment every 10 seconds
    timer.setInterval(60000L, sendTelegramUpdate);    // Send Telegram updates every minute
}

void loop() {
    Blynk.run();
    timer.run();
}

void initializeDevices() {
    for (int i = 0; i < NUM_DEVICES; i++) {
        // Initialize relay pins
        pinMode(devices[i].relayPin, OUTPUT);
        
        // Initialize energy monitor for each device
        devices[i].emon.voltage(devices[i].voltagePin, VOLTAGE_CALIBRATION, 1.7);
        devices[i].emon.current(devices[i].currentPin, CURRENT_CALIBRATION);
        
        // Read stored values from EEPROM
        EEPROM.get(eepromAddr[i].kWh, devices[i].kWh);
        EEPROM.get(eepromAddr[i].cost, devices[i].cost);
        EEPROM.get(eepromAddr[i].relayState, devices[i].relayState);
        
        // Set initial relay states
        digitalWrite(devices[i].relayPin, devices[i].relayState);
    }
}

void processMeasurements() {
    static unsigned long lastMeasurement = 0;
    unsigned long currentTime = millis();
    float timeDiff = (currentTime - lastMeasurement) / 3600000.0; // Convert to hours
    
    for (int i = 0; i < NUM_DEVICES; i++) {
        // Calculate voltage and current
        devices[i].emon.calcVI(20, 2000);
        
        // Store measurements
        devices[i].voltage = devices[i].emon.Vrms;
        devices[i].current = devices[i].emon.Irms;
        devices[i].power = devices[i].emon.apparentPower;
        
        // Calculate energy consumption
        if (lastMeasurement > 0) {
            devices[i].kWh += (devices[i].power * timeDiff) / 1000.0; // Convert to kWh
            devices[i].cost = devices[i].kWh * RATE_PER_KWH;
        }
    }
    
    lastMeasurement = currentTime;
    updateEEPROM();
    updateBlynk();
}

void updateDisplay() {
    lcd.clear();
    
    switch (currentDisplay) {
        case 0: // Voltage and Current
            for (int i = 0; i < 2; i++) { // Show 2 devices per page
                lcd.setCursor(0, i);
                lcd.printf("%s: %.1fV %.1fA", 
                    devices[i].name, 
                    devices[i].voltage, 
                    devices[i].current);
            }
            break;
            
        case 1: // Power
            for (int i = 0; i < 2; i++) {
                lcd.setCursor(0, i);
                lcd.printf("%s: %.1fW", 
                    devices[i].name, 
                    devices[i].power);
            }
            break;
            
        case 2: // Energy and Cost
            lcd.setCursor(0, 0);
            lcd.printf("Tot kWh: %.2f", 
                devices[0].kWh + devices[1].kWh + 
                devices[2].kWh + devices[3].kWh);
            lcd.setCursor(0, 1);
            lcd.printf("Cost: Rs.%.2f", 
                devices[0].cost + devices[1].cost + 
                devices[2].cost + devices[3].cost);
            break;
    }
    
    currentDisplay = (currentDisplay + 1) % DISPLAY_PAGES;
}

void handleEnvironmentControl() {
    // Read environmental sensors
    roomTemperature = dht.readTemperature();
    roomHumidity = dht.readHumidity();
    lightLevel = analogRead(LDR_PIN);
    
    // Automatic light control for first two devices (bulbs)
    if (lightLevel < 500) { // Threshold for low light
        devices[0].relayState = true;  // Turn on first light
        devices[1].relayState = true;  // Turn on second light
    } else {
        devices[0].relayState = false; // Turn off first light
        devices[1].relayState = false; // Turn off second light
    }
    
    // Automatic control for Socket (device 3) based on time of day
    // Using time patterns to simulate typical usage
    // For example, activate during evening hours (6PM-10PM)
    struct tm timeinfo;
    if(getLocalTime(&timeinfo)) {
        // Socket active during evening hours (18:00 - 22:00)
        if (timeinfo.tm_hour >= 18 && timeinfo.tm_hour < 22) {
            devices[2].relayState = true;  // Turn on socket
        } else {
            devices[2].relayState = false; // Turn off socket
        }
    }
    
    // Automatic control for Fan (device 4) based on temperature
    if (roomTemperature > 28.0) { // Threshold for high temperature
        devices[3].relayState = true;  // Turn on fan
    } else if (roomTemperature < 24.0) { // Threshold for comfortable temperature
        devices[3].relayState = false; // Turn off fan
    }
    // Between 24-28°C, maintain current state (hysteresis to prevent rapid cycling)
    
    // Update relay states for all devices
    for (int i = 0; i < NUM_DEVICES; i++) {
        digitalWrite(devices[i].relayPin, devices[i].relayState);
        
        // Sync Blynk app with the current state
        Blynk.virtualWrite(V30 + i, devices[i].relayState);
    }
}

void updateBlynk() {
    // Send individual device data
    for (int i = 0; i < NUM_DEVICES; i++) {
        Blynk.virtualWrite(V0 + (i*5), devices[i].voltage);
        Blynk.virtualWrite(V1 + (i*5), devices[i].current);
        Blynk.virtualWrite(V2 + (i*5), devices[i].power);
        Blynk.virtualWrite(V3 + (i*5), devices[i].kWh);
        Blynk.virtualWrite(V4 + (i*5), devices[i].cost);
        // Add state sync for each device
        Blynk.virtualWrite(V30 + i, devices[i].relayState);
    }
    
    // Send environmental data
    Blynk.virtualWrite(V20, roomTemperature);
    Blynk.virtualWrite(V21, roomHumidity);
    Blynk.virtualWrite(V22, lightLevel);
}

void updateEEPROM() {
    for (int i = 0; i < NUM_DEVICES; i++) {
        EEPROM.put(eepromAddr[i].kWh, devices[i].kWh);
        EEPROM.put(eepromAddr[i].cost, devices[i].cost);
        EEPROM.put(eepromAddr[i].relayState, devices[i].relayState);
    }
    EEPROM.commit();
}

void sendTelegramUpdate() {
    String message = "Energy Consumption Report\n\n";
    float totalKWh = 0;
    float totalCost = 0;
    
    for (int i = 0; i < NUM_DEVICES; i++) {
        message += String(devices[i].name) + ":\n";
        message += "Energy: " + String(devices[i].kWh, 2) + " kWh\n";
        message += "Cost: Rs." + String(devices[i].cost, 2) + "\n\n";
        totalKWh += devices[i].kWh;
        totalCost += devices[i].cost;
    }
    
    message += "Total Energy: " + String(totalKWh, 2) + " kWh\n";
    message += "Total Cost: Rs." + String(totalCost, 2) + "\n";
    message += "Temperature: " + String(roomTemperature, 1) + "°C\n";
    message += "Humidity: " + String(roomHumidity, 1) + "%";
    
    // Send to Telegram (using your existing Telegram sending code)
    sendToTelegram(message);
}

// Function to send messages to Telegram
void sendToTelegram(String message) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String url = "https://api.telegram.org/bot" + String(BOT_TOKEN) + "/sendMessage";
        
        http.begin(url);
        http.addHeader("Content-Type", "application/json");
        
        // Create JSON payload
        StaticJsonDocument<1024> doc;
        doc["chat_id"] = CHAT_ID;
        doc["text"] = message;
        
        String payload;
        serializeJson(doc, payload);
        
        // Send the request
        int httpResponseCode = http.POST(payload);
        
        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("Telegram response: " + response);
        } else {
            Serial.println("Error sending to Telegram. Code: " + String(httpResponseCode));
        }
        
        http.end();
    } else {
        Serial.println("WiFi not connected. Cannot send Telegram message.");
    }
}

// BLYNK Virtual Pin Handlers

// Add these Blynk handlers after the existing code

// Blynk Virtual Pin Handlers for Device Control
BLYNK_WRITE(V30) { // Control for Device 1 (Bulb_01)
    devices[0].relayState = param.asInt();
    digitalWrite(devices[0].relayPin, devices[0].relayState);
    Serial.printf("Device 1 (%s) state changed to: %d\n", devices[0].name, devices[0].relayState);
}

BLYNK_WRITE(V31) { // Control for Device 2 (Bulb_02)
    devices[1].relayState = param.asInt();
    digitalWrite(devices[1].relayPin, devices[1].relayState);
    Serial.printf("Device 2 (%s) state changed to: %d\n", devices[1].name, devices[1].relayState);
}

BLYNK_WRITE(V32) { // Control for Device 3 (Socket)
    devices[2].relayState = param.asInt();
    digitalWrite(devices[2].relayPin, devices[2].relayState);
    Serial.printf("Device 3 (%s) state changed to: %d\n", devices[2].name, devices[2].relayState);
}

BLYNK_WRITE(V33) { // Control for Device 4 (Fan)
    devices[3].relayState = param.asInt();
    digitalWrite(devices[3].relayPin, devices[3].relayState);
    Serial.printf("Device 4 (%s) state changed to: %d\n", devices[3].name, devices[3].relayState);
}

// Add state sync for each device in the updateBlynk() function and when reconnecting to Blynk

// Add this function to sync states when reconnecting to Blynk
BLYNK_CONNECTED() {
    // Sync all device states when reconnecting to Blynk
    for (int i = 0; i < NUM_DEVICES; i++) {
        Blynk.virtualWrite(V30 + i, devices[i].relayState);
    }
}

