# StreamLine: Smart Sprinkler System

## Overview

StreamLine is a smart sprinkler system that utilizes LoRa communication technology to enable seamless interaction between an ESP32 microcontroller and a Raspberry Pi. This system efficiently collects sensor data, controls the sprinkler valve based on the gathered information, and integrates with the Blynk app for real-time monitoring and control.

## Components

**NOTE: Blynk account and dashboard must be set up to use this. Change authentication token in the python code to your unique token from Blynk.**

### 1. ESP32 (SensorValveCode.ino)

The ESP32 microcontroller is responsible for collecting sensor data and controlling the sprinkler valve. The `SensorValveCode.ino` Arduino sketch contains the firmware for the ESP32. This code handles the communication with the Raspberry Pi through LoRa and executes the logic for adjusting the sprinkler system based on the received sensor data and on/off commands from the Blynk app.

### 2. Raspberry Pi (BlynkandCSV.py)

The Raspberry Pi acts as the central hub of the StreamLine system. The `BlynkandCSV.py` Python script manages two main functionalities:

- **Sensor Data Handling:** The script receives sensor data from the ESP32, processes it, and stores it in a CSV file for future analysis.

- **Blynk Integration:** The Blynk app is used for real-time monitoring and control of the smart sprinkler system. The Raspberry Pi updates the Blynk app with the latest sensor data and receives on/off commands from the app. These commands are then transmitted to the ESP32 via LoRa, allowing users to remotely control the sprinkler system.

## Usage

1. **Install Libraries:**
   - Install LoRa module library from Arduino IDE
   - Install Blynk library for python on Raspberry Pi.

2. **ESP32 Setup:**
   - Upload the `SensorValveCode.ino` sketch to your ESP32 using the Arduino IDE or any compatible platform.

3. **Raspberry Pi Setup:**
   - Ensure that Python is installed on your Raspberry Pi.
   - Run the `BlynkandCSV.py` script on the Raspberry Pi.

4. **Blynk App Integration:**
   - Install the Blynk app on your mobile device.
   - Configure the app to connect to the Raspberry Pi and monitor the smart sprinkler system.


## Contributors

- Amna Hayat
- Calyx Ruiz
- Renzheng Zheng

