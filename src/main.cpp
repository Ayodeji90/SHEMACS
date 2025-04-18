#include <Arduino.h>         // Core ESP32 functions
#include <WiFi.h>            // ESP32 Wi-Fi (built-in)
#include <PubSubClient.h>    // MQTT
#include <DHT.h>             // DHT11
#include <LiquidCrystal.h>   // 16x2 LCD
#include <Wire.h>            // I2C
#include <Adafruit_Sensor.h> // Sensor library

// PIN DEFINITIONS 
// ACS712 Current Sensors (4 channels)
const int CURRENT_SENSOR_PIN_1 = 33; // ADC pin for ACS712 1
const int CURRENT_SENSOR_PIN_2 = 34; // ADC pin for ACS712 2
const int CURRENT_SENSOR_PIN_3 = 35; // ADC pin for ACS712 3 
const int CURRENT_SENSOR_PIN_4 = 36; // ADC pin for ACS712 4

// ZMPT101B Voltage Sensors (4 channels)
const int VOLTAGE_SENSOR_PIN_1 = 39; // ADC pin for ZMPT101B 1
const int VOLTAGE_SENSOR_PIN_2 = 25; // ADC pin for ZMPT101B 2
const int VOLTAGE_SENSOR_PIN_3 = 26; // ADC pin for ZMPT101B 3
const int VOLTAGE_SENSOR_PIN_4 = 27; // ADC pin for ZMPT101B 4

// 4-Channel Relay Module
const int RELAY_PIN_1 = 12; // Relay for appliance 1
const int RELAY_PIN_2 = 13; // Relay for appliance 2
const int RELAY_PIN_3 = 14; // Relay for appliance 3
const int RELAY_PIN_4 = 15; // Relay for appliance 4

// 16x2 LCD Pins (4-bit mode)
#define LCD_RS 16
#define LCD_EN 17
#define LCD_D4 5
#define LCD_D5 18
#define LCD_D6 19
#define LCD_D7 21

#define DHT11_PIN 4  // GPIO 4 for DHT11 data pin
#define LDR_PIN 32   // GPIO 32 for LDR analog input

// Device Names (to match web interface)
const char* deviceNames[] = {"Bulb_01", "Bulb_02", "Socket", "Fan"};
const int NUM_DEVICES = 4;

// Initialize LCD object
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Wi-Fi and MQTT settings
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";
const char* mqtt_server = "your_mqtt_broker_ip"; // e.g., "192.168.1.100" or "broker.hivemq.com"
const int mqtt_port = 1883;
const char* mqtt_user = "your_mqtt_user"; // Leave empty if not used
const char* mqtt_password = "your_mqtt_password"; // Leave empty if not used

// Objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT11_PIN, DHT11); // DHT11 object

// Sensor data
float temp, light, power[NUM_DEVICES];
int relayState[NUM_DEVICES] = {0, 0, 0, 0};

// Sensor reading functions
float readCurrent(int pin) {
  int value = analogRead(pin);
  float volts = value * 3.3 / 4095.0;
  return abs((volts - 2.5) / 0.185); // Example for 5A ACS712
}

float readVoltage(int pin) {
  int value = analogRead(pin);
  return (value * 3.3 / 4095.0) * 100; // Simplified scaling
}

void updateSensors() {
  temp = dht.readTemperature();
  light = analogRead(LDR_PIN);
  int currentPins[] = {CURRENT_SENSOR_PIN_1, CURRENT_SENSOR_PIN_2, CURRENT_SENSOR_PIN_3, CURRENT_SENSOR_PIN_4};
  int voltagePins[] = {VOLTAGE_SENSOR_PIN_1, VOLTAGE_SENSOR_PIN_2, VOLTAGE_SENSOR_PIN_3, VOLTAGE_SENSOR_PIN_4};
  for (int i = 0; i < NUM_DEVICES; i++) {
    float current = readCurrent(currentPins[i]);
    float voltage = readVoltage(voltagePins[i]);
    power[i] = current * voltage * 0.9; // Assume PF = 0.9
  }
}

void publishDeviceState(int deviceIndex) {
  String stateTopic = "home/relay/" + String(deviceNames[deviceIndex]) + "/state";
  String state = relayState[deviceIndex] ? "ON" : "OFF";
  client.publish(stateTopic.c_str(), state.c_str());
  Serial.print("Published state for ");
  Serial.print(deviceNames[deviceIndex]);
  Serial.print(": ");
  Serial.println(state);
}

void publishSensorData() {
  // Publish temperature
  if (!isnan(temp)) {
    client.publish("home/temp", String(temp).c_str());
    Serial.print("Published temperature: ");
    Serial.println(temp);
  }

  // Publish light level
  client.publish("home/light", String(light).c_str());
  Serial.print("Published light level: ");
  Serial.println(light);

  // Publish power data for each device
  for (int i = 0; i < NUM_DEVICES; i++) {
    String powerTopic = "home/power/" + String(deviceNames[i]);
    client.publish(powerTopic.c_str(), String(power[i]).c_str());
    Serial.print("Published power for ");
    Serial.print(deviceNames[i]);
    Serial.print(": ");
    Serial.println(power[i]);
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temp);
  lcd.print("C L:");
  lcd.print(light);
  lcd.setCursor(0, 1);
  for (int i = 0; i < NUM_DEVICES; i++) {
    lcd.print(relayState[i] ? "1" : "0");
    if (i < NUM_DEVICES - 1) lcd.print(",");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  for (int i = 0; i < NUM_DEVICES; i++) {
    String relayTopic = "home/relay/" + String(deviceNames[i]);
    if (String(topic) == relayTopic) {
      relayState[i] = (msg == "ON") ? 1 : 0;
      digitalWrite(RELAY_PIN_1 + i, relayState[i]); // Adjust pin offset
      publishDeviceState(i);
      updateLCD(); // Update LCD on state change
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      for (int i = 0; i < NUM_DEVICES; i++) {
        String relayTopic = "home/relay/" + String(deviceNames[i]);
        client.subscribe(relayTopic.c_str());
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize DHT11
  dht.begin();

  // Initialize Relay Pins as Outputs
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);

  // Set relays to OFF by default
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  digitalWrite(RELAY_PIN_3, LOW);
  digitalWrite(RELAY_PIN_4, LOW);

  // Initialize LCD
  lcd.begin(16, 2);  // Set up 16 columns, 2 rows
  lcd.print("Smart Home Init");  // Initial message
  delay(2000);       // Display for 2 seconds
  lcd.clear();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // ADC pins (current and voltage sensors) are input by default, no pinMode needed
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 5000) { // Update every 5 seconds
    updateSensors();
    publishSensorData();
    updateLCD();
    lastUpdate = millis();
  }
}