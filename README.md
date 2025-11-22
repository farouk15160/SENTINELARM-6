
# SENTINELARM-6: ESP32 SAFETY-FIRST ROBOT CONTROLLER
Project Name:       SentinelArm-6
Version:            3.9.2 (Deviation Grace Period Fix)
Platform:           ESP32 (Arduino Framework / PlatformIO)
Author:             Farouk Jamali
Date:               November 2025

[ DESCRIPTION ]
SentinelArm-6 is a high-performance, fully interruptible firmware for 6-DOF 
robotic arms driven by an ESP32. It features a sophisticated dual-core 
architecture that separates motion control from safety monitoring to ensure 
smooth operation and rapid collision detection.

The system is designed for hybrid stepper/servo configurations (Stepper Base + 
5 Servo Joints) and includes a custom binary serial protocol for robust 
communication with Python or other host clients.

[ KEY FEATURES ]
1. Dual-Core Multitasking:
   - Core 0 (Sentinel): Dedicted high-frequency safety monitoring (Current, 
     Deviation, Thermal).
   - Core 1 (Motion): Command parsing and non-blocking motion interpolation.
   
2. Advanced Safety Systems (E-Stop):
   - Absolute Current Threshold: Triggers if total current exceeds limit (e.g., 2.5A).
   - Dynamic Deviation Check: Triggers if current spikes unexpectedly during 
     movement (Collision Detection).
   - Inrush Grace Period: Ignores startup current spikes (300ms) to prevent 
     false alarms (v3.9.2).
   - IIR Filtering: Smooths sensor noise for reliable readings.

3. Motion Control:
   - Smooth Interpolation: Cubic Ease-In-Out acceleration profiles.
   - Hybrid Drive: Supports Stepper Motor (Base) + PWM Servos (Joints 2-6).
   - Gripper Control: Current-limited gripping to prevent object crushing.

4. Persistence:
   - EEPROM Saving: Remembers last pose and configuration (Limits, Thresholds) 
     after power loss.
   - Auto-Recovery: Soft startup on boot to prevent jerky movements.

[ HARDWARE REQUIREMENTS ]
- Microcontroller: ESP32 Dev Module
- Servo Driver:    PCA9685 (I2C Address 0x40)
- Current Sensor:  ACS712 (Main Power) & INA219 (Gripper, Address 0x41)
- Stepper Driver:  A4988 or DRV8825
- Motors:          1x NEMA17 Stepper (Base), 5x MG996R/RDS3115 Servos
- Cooling:         12V Fan (PWM Controlled)

[ PIN CONFIGURATION ]
| Component       | ESP32 Pin | Notes                        |
|-----------------|-----------|------------------------------|
| Stepper DIR     | GPIO 16   | Base rotation direction      |
| Stepper STEP    | GPIO 17   | Base rotation steps          |
| Stepper ENABLE  | GPIO 04   | Active LOW                   |
| Stepper Microstep| 18, 19, 23| MS1, MS2, MS3                |
| Servo PWM Data  | I2C (21/22)| Via PCA9685                 |
| Output Enable   | GPIO 13   | Global Servo/Stepper Enable  |
| Main Current    | GPIO 34   | Analog Read (ACS712)         |
| Fan PWM         | GPIO 26   | 12V Fan Control              |

[ INSTALLATION & BUILD ]
1. Environment: Install VS Code with the PlatformIO extension.
2. Dependencies:
   - Adafruit PWM Servo Driver Library
   - Adafruit INA219
3. Build:
   - Open the project folder.
   - Select 'env:esp32dev' in PlatformIO.
   - Click "Build" then "Upload".
4. Wiring: Ensure the ACS712 is in series with the main power supply (V+) 
   and the INA219 is in series with the gripper servo V+.

[ SERIAL COMMUNICATION PROTOCOL ]
Baud Rate: 921600
Header Byte: 0xA5
Format: Binary Packets (Struct-based)

Command IDs:
  'M' : Move All Joints (Target Angles, Speed)
  'H' : Home Robot (Move to predefined rest position)
  'G' : Gripper Command (Set target current limit)
  'S' : Save Current Pose to EEPROM
  'E' : Emergency Stop Release (Reset collision flag)
  'T' : Set Absolute Current Threshold (Amps)
  'D' : Set Deviation Threshold (Amps) - [New in v3.9.1]
  'F' : Set Fan Speed (0-255)
  'R' : Request Configuration Report
  'C' : Save Current Configuration (Limits/Thresholds) to EEPROM
  'N'/'X': Set Min/Max Software Limits for Joints

[ CALIBRATION & SAFETY ]
1. Zero-Point: On boot, the firmware calibrates the ACS712 zero point. 
   ENSURE THE ROBOT IS STATIONARY DURING BOOT (Yellow LED/Serial Info).
2. E-Stop Recovery: If the robot hits an obstacle, it enters E-Stop state.
   - Send command 'E' (or use Python `send_command('E')`) to unlock.
   - Check obstruction before unlocking.
3. Tuning Thresholds:
   - If the robot stops falsely during fast moves, increase the Deviation 
     Threshold ('D' command).
   - If it doesn't stop on collision, decrease it.

[ LICENSE ]
Open Source. Please credit the author (Farouk) when using or modifying 
this firmware.
