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

* `SmartHome_ESP32_Main.ino` : Contains the core setup and loop logic for the ESP32.
* `node_red_flow.png`: The flow for the Node-RED server (for backend logic).


