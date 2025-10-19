/**
 * @file main.cpp
 * @brief Fully interruptible, non-blocking firmware for ROS2 Robot Arm with auto-calibration and optimizations
 * @author Farouk
 * @date 19 Oct 2025
 *
 * @description
 * This firmware uses a non-blocking, dual-task architecture for motion control.
 * - A low-priority task parses incoming serial commands and sets a target pose.
 * - A high-priority "interpolator" task runs at a fixed rate (50Hz) to
 * smoothly move all joints (servos and stepper) from their current angle to the target angle.
 * This allows a new command to interrupt and override an existing motion at any time.
 *
 * Version 3.5 (Stable Release):
 * - Merged known-good stepper logic from successful hardware test.
 * - Added STEPPER_ENABLE_PIN control for safety and power saving.
 * - Implemented reliable, slower stepper pulse timing (500us) to prevent stalls.
 * - Added software Min/Max angle limits for the stepper base (15-345 deg).
 * - Corrected HOME_POSITION bug to prevent joint over-travel.
 * - Implemented a robust, non-jumping startup sequence.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <cmath>

// --- Stepper Motor (Base) Configuration (from successful test) ---
const int STEPPER_DIR_PIN = 16;
const int STEPPER_STEP_PIN = 17;
const int STEPPER_MS1_PIN = 18;
const int STEPPER_MS2_PIN = 19;
const int STEPPER_MS3_PIN = 23;
const int STEPPER_ENABLE_PIN = 4; // NEW: From your test sketch

#define STEPPER_STEPS_PER_REV 200 // 1.8 deg/step motor
#define STEPPER_MICROSTEPS 16     // Using 1/16 microstepping
#define STEPPER_GEAR_RATIO 5.0f   // !!! IMPORTANT: CHANGE THIS to match your robot's gear ratio

// --- Pulse timing for the stepper driver (slower and reliable) ---
#define STEPPER_PULSE_WIDTH_US 50  // Microseconds for step pulse (HIGH)
#define STEPPER_PULSE_DELAY_US 500 // Microseconds between pulses (LOW). Slower is more reliable.

// Steps per degree of the *output shaft*
const float STEPPER_STEPS_PER_DEGREE = (STEPPER_STEPS_PER_REV * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO) / 360.0f;

// ==========================================================================
// --- Multi-Tasking & Safety Configuration ---
// ==========================================================================
TaskHandle_t h_task_command_parser = NULL;
TaskHandle_t h_task_monitoring = NULL;
TaskHandle_t h_task_motion_interpolator = NULL;
SemaphoreHandle_t x_pose_mutex;

volatile float g_collision_current_threshold_A = 5.0f;
volatile bool g_collisionDetected = false;
volatile float g_mainCurrent_A = 0.0;
volatile float g_gripperCurrent_mA = 0.0;
float g_calibrated_zero_voltage = 2.3914f;

// --- Motion Control State (Protected by Mutex) ---
float g_current_angles[6];
float g_target_angles[6];
unsigned long g_move_start_time;
unsigned long g_move_duration_ms;
volatile float g_grip_target_current_mA = -1.0f;
volatile long g_stepper_current_step_pos = 0;

// ==========================================================================
// --- Hardware & Servo Configuration ---
// ==========================================================================
#define ACS712_PIN 34
#define OE_PIN 13
#define BAUD_RATE 921600
#define EEPROM_SIZE 128
#define EEPROM_VALID_FLAG 123

Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOS_NUMBER 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PWM_FREQ 50
#define GRIPPER_SERVO_INDEX 5
#define JOINT_1_SECOND_SERVO 11

// Joint 0: Stepper
// Joint 1: Controls Servos 10 AND 11
// ...
const uint8_t SERVOS[SERVOS_NUMBER] = {0, 10, 12, 13, 14, 15};
// --- NEW: Added Min/Max limits for Stepper (Joint 0) ---
const float SERVOS_MIN[SERVOS_NUMBER] = {15.0f, 40.0f, 0.0f, 30.0f, 0.0f, 0.0f};
const float SERVOS_MAX[SERVOS_NUMBER] = {345.0f, 150.0f, 145.0f, 130.0f, 180.0f, 120.0f};
// --- BUG FIX: Corrected HOME_POSITION[3] to be within its limits ---
const float HOME_POSITION[SERVOS_NUMBER] = {133.0f, 100.00, 110.00, 130.00, 15.00, 60.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {false, true, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {0, 0, 0, 0, 0, 0};

// ==========================================================================
// --- Binary Communication Protocol ---
// ==========================================================================
#define HEADER_BYTE 0xA5
#pragma pack(push, 1)
struct CommandPacket
{
  uint8_t header;
  uint8_t cmd_id;
  float angles[6];
  float speed_factor;
  float gripper_current_ma;
  uint8_t checksum;
};
struct StatusPacket
{
  uint8_t header;
  float main_current_A;
  float gripper_current_mA;
  float angles[6];
  uint8_t checksum;
};
#pragma pack(pop)

// --- Helper Functions ---
uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; ++i)
  {
    checksum ^= data[i];
  }
  return checksum;
}

int angleToPulse(float degrees, int servo_index)
{
  float safe_angle = constrain(degrees, SERVOS_MIN[servo_index], SERVOS_MAX[servo_index]);
  if (SERVO_INVERT[servo_index])
  {
    safe_angle = 180.0f - safe_angle;
  }
  int pulse = map(safe_angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  return pulse + TRIM_US[servo_index];
}

static inline float easeInOutCubic(float t)
{
  t *= 2.0f;
  if (t < 1.0f)
    return 0.5f * t * t * t;
  t -= 2.0f;
  return 0.5f * (t * t * t + 2.0f);
}

// ==========================================================================
// --- SAFETY FUNCTION ---
// ==========================================================================
void emergency_stop()
{
  g_collisionDetected = true;
  digitalWrite(OE_PIN, HIGH);             // Disable servo driver
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // NEW: Disable stepper driver
  Serial.println("\nE-STOP: Main current limit exceeded. Halting.\n");
  if (h_task_motion_interpolator != NULL)
    vTaskSuspend(h_task_motion_interpolator);
  if (h_task_command_parser != NULL)
    vTaskSuspend(h_task_command_parser);
}

// ==========================================================================
// --- Motion Interpolator Task (Core 1, High Priority) ---
// ==========================================================================
void task_motion_interpolator(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    if (g_collisionDetected)
      continue;

    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);

    unsigned long elapsed_time = millis() - g_move_start_time;
    float fraction = (g_move_duration_ms > 0) ? (float)elapsed_time / (float)g_move_duration_ms : 1.0f;
    fraction = constrain(fraction, 0.0f, 1.0f);
    float eased_fraction = easeInOutCubic(fraction);

    // --- Smart Grip Logic ---
    if (g_grip_target_current_mA > 0 && fabsf(g_gripperCurrent_mA) > g_grip_target_current_mA)
    {
      float final_grip_angle = (g_current_angles[GRIPPER_SERVO_INDEX] * (1.0f - eased_fraction)) + (g_target_angles[GRIPPER_SERVO_INDEX] * eased_fraction);
      g_current_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_target_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_grip_target_current_mA = -1.0f;
    }

    // --- STEPPER LOGIC (JOINT 0) ---
    long start_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
    long end_step_pos = (long)(g_target_angles[0] * STEPPER_STEPS_PER_DEGREE);
    long total_steps_for_move = end_step_pos - start_step_pos;
    long current_target_step_pos = start_step_pos + (long)(total_steps_for_move * eased_fraction);
    long steps_to_move = current_target_step_pos - g_stepper_current_step_pos;

    if (steps_to_move != 0)
    {
      digitalWrite(STEPPER_DIR_PIN, (steps_to_move > 0) ? (SERVO_INVERT[0] ? HIGH : LOW) : (SERVO_INVERT[0] ? LOW : HIGH));
      long num_steps_to_pulse = abs(steps_to_move);
      for (long i = 0; i < num_steps_to_pulse; i++)
      {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(STEPPER_PULSE_WIDTH_US);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(STEPPER_PULSE_DELAY_US);
      }
      g_stepper_current_step_pos = current_target_step_pos;
    }

    // --- Servo Joints (1-5) Control ---
    for (int i = 1; i < SERVOS_NUMBER; i++)
    {
      float interpolated_angle = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
      int pulse = angleToPulse(interpolated_angle, i);
      pwm.writeMicroseconds(SERVOS[i], pulse);
      if (i == 1)
      {
        pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulse);
      }
    }

    // --- Move Completion ---
    if (fraction >= 1.0f)
    {
      g_grip_target_current_mA = -1.0;
      for (int i = 0; i < SERVOS_NUMBER; i++)
      {
        g_current_angles[i] = g_target_angles[i];
      }
      g_stepper_current_step_pos = end_step_pos;
    }
    xSemaphoreGive(x_pose_mutex);
  }
}

// ==========================================================================
// --- Command Parser Task (Core 1, Low Priority) ---
// ==========================================================================
void parse_command(const CommandPacket &cmd)
{
  if (g_collisionDetected)
    return;
  switch (cmd.cmd_id)
  {
  case 'M':
  case 'H':
  case 'G':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    if (cmd.cmd_id == 'M' || cmd.cmd_id == 'H')
    {
      g_grip_target_current_mA = -1.0f;
    }
    unsigned long elapsed_time = millis() - g_move_start_time;
    float fraction = g_move_duration_ms > 0 ? (float)elapsed_time / (float)g_move_duration_ms : 1.0f;
    fraction = constrain(fraction, 0.0f, 1.0f);
    float eased_fraction = easeInOutCubic(fraction);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      g_current_angles[i] = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
    }
    g_stepper_current_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
    float max_angle_delta = 0.0f;
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      float target = (cmd.cmd_id == 'H') ? HOME_POSITION[i] : cmd.angles[i];
      g_target_angles[i] = constrain(target, SERVOS_MIN[i], SERVOS_MAX[i]); // Constrain here
      float delta = std::abs(g_target_angles[i] - g_current_angles[i]);
      if (delta > max_angle_delta)
      {
        max_angle_delta = delta;
      }
    }
    if (cmd.cmd_id == 'G')
    {
      g_grip_target_current_mA = cmd.gripper_current_ma;
    }
    g_move_start_time = millis();
    float speed = (cmd.speed_factor > 0.01f) ? cmd.speed_factor : 10.0f;
    g_move_duration_ms = max_angle_delta * speed;
    if (cmd.cmd_id == 'H')
      g_move_duration_ms = 3000;
    xSemaphoreGive(x_pose_mutex);
    break;
  }
  case 'S':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    unsigned long elapsed_time = millis() - g_move_start_time;
    float fraction = g_move_duration_ms > 0 ? (float)elapsed_time / (float)g_move_duration_ms : 1.0f;
    fraction = constrain(fraction, 0.0f, 1.0f);
    float eased_fraction = easeInOutCubic(fraction);
    float angles_to_save[SERVOS_NUMBER];
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      angles_to_save[i] = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
    }
    EEPROM.write(0, EEPROM_VALID_FLAG);
    EEPROM.put(1, angles_to_save);
    EEPROM.commit();
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: Current pose saved to EEPROM.");
    break;
  }
  }
}

// --- Robust byte-by-byte serial parser ---
void task_command_parser(void *pvParameters)
{
  uint8_t serial_buffer[sizeof(CommandPacket)];
  int buffer_pos = 0;
  for (;;)
  {
    while (Serial.available() > 0)
    {
      uint8_t byte_in = Serial.read();
      if (buffer_pos == 0)
      {
        if (byte_in == HEADER_BYTE)
        {
          serial_buffer[buffer_pos++] = byte_in;
        }
      }
      else
      {
        serial_buffer[buffer_pos++] = byte_in;
        if (buffer_pos >= sizeof(CommandPacket))
        {
          CommandPacket received_cmd;
          memcpy(&received_cmd, serial_buffer, sizeof(CommandPacket));
          uint8_t calculated_cs = calculate_checksum(serial_buffer, sizeof(CommandPacket) - 1);
          if (calculated_cs == received_cmd.checksum)
          {
            parse_command(received_cmd);
          }
          else
          {
            Serial.println("WARN: Checksum failed!");
          }
          buffer_pos = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ==========================================================================
// --- Monitoring Task (Core 0) ---
// ==========================================================================
void task_monitoring(void *pvParameters)
{
  long lastStatusSendTime = 0;
  const int nSamples = 250;
  const float sens = 0.066;
  const float vcc = 3.3;
  const int adcMax = 4095;
  for (;;)
  {
    long val = 0;
    for (int i = 0; i < nSamples; i++)
      val += analogRead(ACS712_PIN);
    float measured_voltage = ((float)val / nSamples / adcMax) * vcc;
    g_mainCurrent_A = (measured_voltage - g_calibrated_zero_voltage) / sens;
    g_gripperCurrent_mA = ina219.getCurrent_mA();
    if (fabsf(g_mainCurrent_A) > g_collision_current_threshold_A)
    {
      emergency_stop();
      continue;
    }
    if (millis() - lastStatusSendTime > 100 && !g_collisionDetected)
    {
      StatusPacket status;
      status.header = HEADER_BYTE;
      status.main_current_A = g_mainCurrent_A;
      status.gripper_current_mA = g_gripperCurrent_mA;
      xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
      unsigned long elapsed_time = millis() - g_move_start_time;
      float fraction = g_move_duration_ms > 0 ? (float)elapsed_time / (float)g_move_duration_ms : 1.0f;
      fraction = constrain(fraction, 0.0f, 1.0f);
      float eased_fraction = easeInOutCubic(fraction);
      for (int i = 0; i < SERVOS_NUMBER; i++)
      {
        status.angles[i] = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
      }
      xSemaphoreGive(x_pose_mutex);
      status.checksum = calculate_checksum((uint8_t *)&status, sizeof(StatusPacket) - 1);
      Serial.write((uint8_t *)&status, sizeof(StatusPacket));
      lastStatusSendTime = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==========================================================================
// --- SETUP (ROBUST STARTUP) ---
// ==========================================================================
void setup()
{
  // --- Step 1: Disable all motors IMMEDIATELY ---
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH); // Disable servos
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // Disable stepper

  delay(2000); // Wait for power to stabilize

  Serial.begin(BAUD_RATE);
  Serial.println("\n--- ESP32 Robot Arm Firmware (v3.5 Stable) ---");
  EEPROM.begin(EEPROM_SIZE);

  // --- Step 2: Initialize Stepper Pins ---
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_MS1_PIN, OUTPUT);
  pinMode(STEPPER_MS2_PIN, OUTPUT);
  pinMode(STEPPER_MS3_PIN, OUTPUT);
  digitalWrite(STEPPER_MS1_PIN, HIGH);
  digitalWrite(STEPPER_MS2_PIN, HIGH);
  digitalWrite(STEPPER_MS3_PIN, HIGH);
  Serial.println("INFO: Stepper microstepping set to 1/16.");

  // --- Step 3: Initialize I2C Devices ---
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);
  ina219.begin();
  // --- CALIBRATION STEP ---
  Serial.println("Calibrating current sensor... Keep arm power off or unloaded.");
  long adc_sum = 0;
  const int n_cal_samples = 1000;
  for (int i = 0; i < n_cal_samples; i++)
  {
    adc_sum += analogRead(ACS712_PIN);
    delay(1);
  }
  g_calibrated_zero_voltage = ((float)adc_sum / n_cal_samples / 4095.0f) * 3.3f;
  Serial.print("Calibration complete. Zero-point voltage: ");
  Serial.println(g_calibrated_zero_voltage, 4);

  // --- Step 4: Load and Validate Startup Position ---
  if (EEPROM.read(0) == EEPROM_VALID_FLAG)
  {
    Serial.println("INFO: Restoring last saved pose from EEPROM.");
    float temp_angles[SERVOS_NUMBER];
    EEPROM.get(1, temp_angles);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      // PERMANENT FIX: Constrain all loaded values to be safe
      g_current_angles[i] = constrain(temp_angles[i], SERVOS_MIN[i], SERVOS_MAX[i]);
      g_target_angles[i] = g_current_angles[i];
    }
  }
  else
  {
    Serial.println("INFO: No valid pose in EEPROM. Starting at HOME position.");
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      // PERMANENT FIX: Constrain all home values to be safe
      g_current_angles[i] = constrain(HOME_POSITION[i], SERVOS_MIN[i], SERVOS_MAX[i]);
      g_target_angles[i] = g_current_angles[i];
    }
  }

  // --- Step 5: Initialize Global Motion State ---
  g_stepper_current_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
  Serial.print("INFO: Initializing stepper to angle ");
  Serial.print(g_current_angles[0]);
  Serial.print(" (");
  Serial.print(g_stepper_current_step_pos);
  Serial.println(" steps).");
  g_move_start_time = millis();
  g_move_duration_ms = 0; // No move on boot

  // --- Step 6: Pre-load servos with start position BEFORE enabling ---
  Serial.println("INFO: Pre-loading servo positions...");
  for (int i = 1; i < SERVOS_NUMBER; i++)
  {
    int pulse = angleToPulse(g_current_angles[i], i);
    pwm.writeMicroseconds(SERVOS[i], pulse);
    if (i == 1)
    {
      pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulse);
    }
  }
  delay(500); // Give servos time to receive the signal

  // --- Step 7: Enable all motors ---
  digitalWrite(STEPPER_ENABLE_PIN, LOW); // Enable stepper
  digitalWrite(OE_PIN, LOW);             // Enable servos
  Serial.println("Motors enabled. Startup should be smooth.");

  // --- Step 8: Start Tasks ---
  x_pose_mutex = xSemaphoreCreateMutex();
  Serial.println("Initializing tasks...");
  xTaskCreatePinnedToCore(task_monitoring, "Monitoring", 4096, NULL, 1, &h_task_monitoring, 0);
  xTaskCreatePinnedToCore(task_command_parser, "CmdParser", 4096, NULL, 2, &h_task_command_parser, 1);
  xTaskCreatePinnedToCore(task_motion_interpolator, "Motion", 4096, NULL, 3, &h_task_motion_interpolator, 1);
  Serial.println("Initialization Complete. Ready for commands.");
}

void loop()
{
  vTaskDelete(NULL); // FreeRTOS handles everything
}