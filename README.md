# 🛡️ SENTINELARM-6 (v3.9.2)

High-Performance ESP32 Robotic Controller with Dual-Core Safety Architecture.

## 📖 Overview

SentinelArm-6 is a firmware designed for 6-DOF robotic arms that prioritizes safety and smooth motion. Unlike standard Arduino sketches, this firmware utilizes FreeRTOS to separate motion control from safety monitoring onto different CPU cores.

It features Adaptive Collision Detection, ignoring the natural current spikes of motor startup (Inrush Current) while instantly detecting physical obstructions during movement.

## ✨ Key Features

### 🧠 Dual-Core Architecture

* **Core 0 (Safety Task)**: High-frequency monitoring of current sensors (ACS712 & INA219).
* **Core 1 (Motion Task)**: Inverse kinematics, command parsing, and cubic interpolation.

### 🛡️ Advanced Safety Systems

* **Dynamic Deviation Check**: Detects collisions by monitoring unexpected current spikes.
* **Inrush Grace Period** *(New in v3.9.2)*: 300ms blind window prevents false E-Stops.
* **Absolute Current Limiter**: Hard E-Stop if current exceeds 5.0A (configurable).

### 🌊 Cinematic Motion Control

* **Cubic Ease-In-Out** acceleration/deceleration.
* **Hybrid Drive Engine** for Stepper + Servos.

### 💾 Persistence

* EEPROM storage for last position, calibration, and thresholds.

## 🛠️ Hardware Configuration

### Pinout Map (ESP32 Dev Module)

| Component      | ESP32 Pin          | Protocol | Notes                |
| -------------- | ------------------ | -------- | -------------------- |
| Stepper DIR    | GPIO16             | Digital  | Base Motor Direction |
| Stepper STEP   | GPIO17             | Digital  | Step Pulse           |
| Stepper EN     | GPIO4              | Digital  | Active LOW           |
| Microsteps     | 18,19,23           | Digital  | MS1, MS2, MS3        |
| Servo Driver   | 21 (SDA), 22 (SCL) | I2C      | PCA9685 @ 0x40       |
| Main Sensor    | GPIO34             | Analog   | ACS712               |
| Gripper Sensor | 21,22              | I2C      | INA219 @ 0x41        |
| Fan Control    | GPIO26             | PWM      | 12V Fan              |
| Global Enable  | GPIO13             | Digital  | Cuts motor power     |

### Bill of Materials

* ESP32-WROOM-32
* PCA9685 16-Channel PWM Driver
* A4988/DRV8825 Stepper Driver
* ACS712 + INA219 Sensors
* 1× NEMA17 Stepper + 5× Servos (MG996R/RDS3115)

## 🚀 Serial Communication Protocol

* **Baud:** 921600
* **Header:** 0xA5

### Commands

| ID  | Function       | Payload                         |
| --- | -------------- | ------------------------------- |
| 'M' | Move           | Target Angles [6], Speed Factor |
| 'H' | Home           | Move to rest position           |
| 'G' | Gripper        | Target Current Limit            |
| 'E' | E-Stop Release | Unlock motors                   |
| 'S' | Save Pose      | Save to EEPROM                  |
| 'D' | Set Deviation  | Sensitivity value               |
| 'T' | Set Threshold  | Max amps                        |
| 'R' | Report         | Return config                   |

## 💾 Installation

### Prerequisites

* VS Code + PlatformIO.
* Clone the repository.

### Build Instructions

1. Open `platformio.ini`.
2. Connect ESP32 via USB.
3. Upload using PlatformIO.
4. Power motors **after** ESP32 boots.

### Calibration

During the first 3 seconds, the ACS712 baseline is measured.

> ⚠️ **Do NOT move the robot during this time.**

## 📊 Performance Tuning (v3.9.2)

* For false E-Stops → increase Deviation Threshold (`D` command > 1.0).
* Heavy motors? Increase `DEVIATION_GRACE_PERIOD_MS`.

## 📜 License

Distributed under the **MIT License**. See `LICENSE` for details.

**Author:** Farouk Jamali
