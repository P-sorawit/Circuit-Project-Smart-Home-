# Smart Home Automation Model (ESP32 & Line Bot)

## Project Overview

This project implements a practical **Internet of Things (IoT)** solution to create an automated residential system. It focuses on enhancing convenience, safety, and energy efficiency by using an **ESP32** microcontroller and integrating with a **Line Official Account (Line Bot)** for real-time monitoring and remote control.

## Core Features and Functionality

The system leverages a variety of sensors and actuators to automate key household functions:

| Feature | Sensor/Actuator | Description |
| :--- | :--- | :--- |
| **Automatic Gate Light** | LDR Sensor & IR Remote | Turns light ON/OFF based on ambient light; includes manual override. |
| **Automated Cooling** | NTC Thermistor Sensors | Activates a cooling fan if temperature in 2/3 monitored zones exceeds **30°C**. |
| **Parking Alert System** | Infrared (IR) Sensor & Buzzer | Triggers an audible alert when a vehicle approaches too closely in a confined space. |
| **Internal Light Control** | Touch Sensor | Simple capacitive control for indoor lighting. |

## Technology Stack

| Category | Components / Protocols | Details |
| :--- | :--- | :--- |
| **Microcontroller** | **ESP32** | The central processing unit for data reading and control. |
| **Cloud/Messaging** | **Node-RED, Google Sheets, Line Official Account** | Used for flow-based data processing, logging, and user interface. |
| **Communication** | **MQTT, HTTP** | Protocols for sending sensor data and receiving commands over the network. |

---

## Code Structure

The project primarily uses the Arduino framework for the ESP32. The main code handles sensor reading, decision logic, and communication with the MQTT broker/Line Bot API.

### Main Files

* `SmartHome_ESP32_Main.ino` or `main.cpp`: Contains the core setup and loop logic for the ESP32.
* `NodeRED_Flow.json`: The exported flow file for the Node-RED server (for backend logic).

### Illustrative ESP32 Code Snippet

The following is a simplified structure of the main code, demonstrating the integration of the core features and communication setup:

```cpp
// 1. Include necessary libraries (e.g., WiFi, PubSubClient for MQTT, Line API)
#include <WiFi.h>
#include <PubSubClient.h>
// #include <IRremote.h> // Example library for IR remote

// 2. Define Pin Assignments
#define LDR_PIN 34
#define NTC_TEMP_PIN_1 35
#define IR_SENSOR_PIN 36
#define FAN_CONTROL_PIN 18
#define BUZZER_PIN 19
#define GATE_LIGHT_PIN 23

// 3. Define Network & MQTT Credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqttServer = "YOUR_MQTT_BROKER_IP";
const int mqttPort = 1883;

// 4. Setup Function: Connect to WiFi and MQTT Broker
void setup() {
  Serial.begin(115200);
  // Initialize pins as input/output
  pinMode(FAN_CONTROL_PIN, OUTPUT);
  // ... (Other pin initializations)
  
  // Connect to Wi-Fi
  // ... (WiFi connection logic)

  // Connect to MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback); // Function to handle incoming Line Bot commands
  reconnect(); 
}

// 5. Main Loop Function: Read Sensors, Apply Logic, and Publish Data
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Keeps MQTT connection alive

  // --- A. Read Sensors ---
  int lightValue = analogRead(LDR_PIN);
  float temp1 = readNTC(NTC_TEMP_PIN_1);
  bool parkingObstacle = digitalRead(IR_SENSOR_PIN);
  
  // --- B. Automation Logic ---

  // 1. Gate Light Logic (Check if it's dark)
  if (lightValue < 500) { 
    digitalWrite(GATE_LIGHT_PIN, HIGH);
  } else {
    digitalWrite(GATE_LIGHT_PIN, LOW);
  }
  
  // 2. Cooling Fan Logic (Check temperature)
  if (temp1 > 30.0 && temp2 > 30.0) { // Simplified check for 2 out of 3 sensors
    digitalWrite(FAN_CONTROL_PIN, HIGH); 
  } else {
    digitalWrite(FAN_CONTROL_PIN, LOW);
  }

  // 3. Parking Alert Logic
  if (parkingObstacle) {
    tone(BUZZER_PIN, 1000); // Trigger buzzer
    // Publish alert to MQTT -> Line Bot
    client.publish("home/alerts", "Parking obstacle detected!");
  } else {
    noTone(BUZZER_PIN);
  }

  // --- C. Publish Status to Cloud/Node-RED ---
  // Send temperature status
  String statusPayload = "{\"temp\": " + String(temp1) + ", \"fan\": " + (digitalRead(FAN_CONTROL_PIN) ? "ON" : "OFF") + "}";
  client.publish("home/status", statusPayload.c_str());

  delay(5000); // Update every 5 seconds
}

// 6. MQTT Callback Function (To process commands from Line Bot)
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Check if the message is a command to turn on/off the internal light
  // e.g., if (strcmp(topic, "home/commands/light") == 0) {
  //   // Execute command
  // }
}
