/**
 * @file main.cpp
 * @brief Fully interruptible, non-blocking firmware for ROS2 Robot Arm with auto-calibration and optimizations
 * @author Farouk & Gemini
 * @date 06 Oct 2025
 *
 * @description
 * This firmware uses a non-blocking, dual-taask architecture for motion control.
 * - A low-priority task parses incoming serial commands and sets a target pose.
 * - A high-priority "interpolator" task runs at a fixed rate (50Hz) to
 * smoothly move the servos from their current angle to the target angle.
 * This allows a new command to interrupt and override an existing motion at any time.
 *
 * Optimizations in this version:
 * - Added more efficient easing functions for smoother, less CPU-intensive motion.
 * - EEPROM writes are now on-demand via a dedicated 'S' command to prevent motion stutter
 * and reduce flash memory wear.
 * - Added EEPROM loading on boot to restore the last saved position.
 * - Re-implemented non-blocking, current-based smart gripper functionality.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <cmath>

// ==========================================================================
// --- Multi-Tasking & Safety Configuration ---
// ==========================================================================
TaskHandle_t h_task_command_parser = NULL;
TaskHandle_t h_task_monitoring = NULL;
TaskHandle_t h_task_motion_interpolator = NULL; // High-priority motion task
SemaphoreHandle_t x_pose_mutex;                 // Mutex to protect shared pose data

volatile float g_collision_current_threshold_A = 2.5f;
volatile bool g_collisionDetected = false;
volatile float g_mainCurrent_A = 0.0;
volatile float g_gripperCurrent_mA = 0.0;
float g_calibrated_zero_voltage = 2.4038f;

// --- Motion Control State (Protected by Mutex) ---
float g_current_angles[6];
float g_target_angles[6];
unsigned long g_move_start_time;
unsigned long g_move_duration_ms;
volatile float g_grip_target_current_mA = -1.0f; // NEW: Target current for smart grip. -1 means inactive.

// ==========================================================================
// --- Hardware & Servo Configuration ---
// ==========================================================================
#define ACS712_PIN 34
#define OE_PIN 13
#define BAUD_RATE 921600
#define EEPROM_SIZE 128
#define EEPROM_VALID_FLAG 123 // Magic number to check if EEPROM data is valid

Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOS_NUMBER 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PWM_FREQ 50
#define GRIPPER_SERVO_INDEX 5

const uint8_t SERVOS[SERVOS_NUMBER] = {10, 11, 12, 13, 14, 15};
const float SERVOS_MIN[SERVOS_NUMBER] = {40.0f, 40.0f, 0.0f, 30.0f, 0.0f, 0.0f};
const float SERVOS_MAX[SERVOS_NUMBER] = {150.0f, 150.0f, 145.0f, 130.0f, 120.0f, 120.0f};
const float HOME_POSITION[SERVOS_NUMBER] = {70.00, 70.00, 120.00, 140.00, 80.00, 30.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {true, false, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {0, 0, 0, 0, 0, 0};

// ==========================================================================
// --- Binary Communication Protocol ---
// ==========================================================================
#define HEADER_BYTE 0xA5

#pragma pack(push, 1)
struct CommandPacket
{
  uint8_t header;
  uint8_t cmd_id; // 'M'ove, 'H'ome, 'G'rip, 'S'ave
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

float pulseToAngle(int pulse, int servo_index)
{
  int pulse_without_trim = pulse - TRIM_US[servo_index];
  float degrees = (float)(pulse_without_trim - SERVO_MIN_US) * 180.0f / (float)(SERVO_MAX_US - SERVO_MIN_US);
  return SERVO_INVERT[servo_index] ? 180.0f - degrees : degrees;
}

static inline float easeInOutCubic(float t)
{
  t *= 2.0f;
  if (t < 1.0f)
  {
    return 0.5f * t * t * t;
  }
  t -= 2.0f;
  return 0.5f * (t * t * t + 2.0f);
}

// ==========================================================================
// --- SAFETY FUNCTION ---
// ==========================================================================
void emergency_stop()
{
  g_collisionDetected = true;
  digitalWrite(OE_PIN, HIGH); // Disable servo driver
  Serial.println("\nE-STOP: Main current limit exceeded. Halting.\n");
  // Suspend motion and command tasks for safety
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
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz update rate
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    if (g_collisionDetected)
      continue;

    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);

    unsigned long elapsed_time = millis() - g_move_start_time;
    float fraction = 1.0f;
    if (g_move_duration_ms > 0)
    {
      fraction = (float)elapsed_time / (float)g_move_duration_ms;
    }
    fraction = constrain(fraction, 0.0f, 1.0f);
    float eased_fraction = easeInOutCubic(fraction);

    // --- Smart Grip Logic ---
    if (g_grip_target_current_mA > 0 && fabsf(g_gripperCurrent_mA) > g_grip_target_current_mA)
    {
      // Gripper has made contact and exceeded current threshold
      float final_grip_angle = (g_current_angles[GRIPPER_SERVO_INDEX] * (1.0f - eased_fraction)) + (g_target_angles[GRIPPER_SERVO_INDEX] * eased_fraction);

      // Stop the gripper's motion immediately by setting its target to its current location
      g_current_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_target_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;

      g_grip_target_current_mA = -1.0f; // Deactivate smart grip mode
    }

    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      // Interpolate from current to target using linear interpolation (lerp)
      float interpolated_angle = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
      pwm.writeMicroseconds(SERVOS[i], angleToPulse(interpolated_angle, i));
    }

    // If move is complete, update the "current" angles to match the target
    if (fraction >= 1.0f)
    {
      g_grip_target_current_mA = -1.0; // Ensure grip mode is off after any move completes
      for (int i = 0; i < SERVOS_NUMBER; i++)
      {
        g_current_angles[i] = g_target_angles[i];
      }
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
  case 'M': // MOVE
  case 'H': // HOME
  case 'G': // GRIP
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);

    // A regular move cancels any active grip command
    if (cmd.cmd_id == 'M' || cmd.cmd_id == 'H')
    {
      g_grip_target_current_mA = -1.0f;
    }

    // The move starts NOW from the current interpolated position.
    // First, calculate the true current position to ensure a smooth transition.
    unsigned long elapsed_time = millis() - g_move_start_time;
    float fraction = g_move_duration_ms > 0 ? (float)elapsed_time / (float)g_move_duration_ms : 1.0f;
    fraction = constrain(fraction, 0.0f, 1.0f);
    float eased_fraction = easeInOutCubic(fraction);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      // Update g_current_angles to the actual current position before setting a new target
      g_current_angles[i] = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);
    }

    // Set the new target
    float max_angle_delta = 0.0f;
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      float target = (cmd.cmd_id == 'H') ? HOME_POSITION[i] : cmd.angles[i];
      float delta = std::abs(target - g_current_angles[i]);
      if (delta > max_angle_delta)
      {
        max_angle_delta = delta;
      }
      g_target_angles[i] = constrain(target, SERVOS_MIN[i], SERVOS_MAX[i]);
    }

    if (cmd.cmd_id == 'G')
    {
      g_grip_target_current_mA = cmd.gripper_current_ma;
    }

    g_move_start_time = millis();
    g_move_duration_ms = max_angle_delta * cmd.speed_factor;
    if (cmd.cmd_id == 'H')
      g_move_duration_ms = 4000; // Homing is always slow

    xSemaphoreGive(x_pose_mutex);
    break;
  }

  case 'S': // SAVE
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    EEPROM.write(0, EEPROM_VALID_FLAG);
    EEPROM.put(1, g_current_angles);
    EEPROM.commit();
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: Current pose saved to EEPROM.");
    break;
  }
  }
}

// --- FIXED: Reverted to robust byte-by-byte serial parser ---
void task_command_parser(void *pvParameters)
{
  uint8_t serial_buffer[sizeof(CommandPacket)];
  int buffer_pos = 0;

  for (;;)
  {
    while (Serial.available() > 0)
    {
      uint8_t byte_in = Serial.read();

      // If the buffer is empty, we must wait for a header byte
      if (buffer_pos == 0)
      {
        if (byte_in == HEADER_BYTE)
        {
          serial_buffer[buffer_pos++] = byte_in;
        }
        // If it's not a header, we just ignore it and continue waiting
      }
      else
      {
        // If we are already filling the buffer, add the new byte
        serial_buffer[buffer_pos++] = byte_in;

        // If the buffer is now full, process the packet
        if (buffer_pos >= sizeof(CommandPacket))
        {
          CommandPacket received_cmd;
          memcpy(&received_cmd, serial_buffer, sizeof(CommandPacket));

          uint8_t calculated_cs = calculate_checksum(serial_buffer, sizeof(CommandPacket) - 1);

          if (calculated_cs == received_cmd.checksum)
          {
            // Command is valid, process it
            parse_command(received_cmd);
          }
          else
          {
            // Checksum failed
            Serial.println("WARN: Checksum failed!");
          }

          // Reset buffer for the next packet
          buffer_pos = 0;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // Yield to other tasks
  }
}

// ==========================================================================
// --- Monitoring Task (Core 0) ---
// ==========================================================================
void task_monitoring(void *pvParameters)
{
  long lastStatusSendTime = 0;
  const int nSamples = 250;
  const float sens = 0.066; // For 30A ACS712 model
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
    { // 10 Hz
      StatusPacket status;
      status.header = HEADER_BYTE;
      status.main_current_A = g_mainCurrent_A;
      status.gripper_current_mA = g_gripperCurrent_mA;

      xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
      // Calculate real-time interpolated angles for feedback
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
// --- SETUP ---
// ==========================================================================
void setup()
{
  delay(4000);
  Serial.begin(BAUD_RATE);
  Serial.println("\n--- ESP32 Robot Arm Firmware (Optimized Non-Blocking Edition) ---");

  EEPROM.begin(EEPROM_SIZE);

  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH); // Servos disabled initially for safety

  // --- CALIBRATION STEP (Uncomment if needed) ---
  // Serial.println("Calibrating current sensor... Keep arm power off or unloaded.");
  // long adc_sum = 0;
  // const int n_cal_samples = 1000;
  // for (int i = 0; i < n_cal_samples; i++)
  // {
  //   adc_sum += analogRead(ACS712_PIN);
  //   delay(1);
  // }
  // g_calibrated_zero_voltage = ((float)adc_sum / n_cal_samples / 4095.0f) * 3.3f;
  // Serial.print("Calibration complete. Zero-point voltage: ");
  // Serial.println(g_calibrated_zero_voltage, 4);

  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);
  ina219.begin();

  // --- MODIFIED: Initialize pose from EEPROM or HOME position ---
  if (EEPROM.read(0) == EEPROM_VALID_FLAG)
  {
    Serial.println("INFO: Restoring last saved pose from EEPROM.");
    EEPROM.get(1, g_current_angles);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      g_target_angles[i] = g_current_angles[i];
    }
  }
  else
  {
    Serial.println("INFO: No valid pose in EEPROM. Starting at HOME position.");
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      g_current_angles[i] = HOME_POSITION[i];
      g_target_angles[i] = HOME_POSITION[i];
    }
  }

  g_move_start_time = millis();
  g_move_duration_ms = 0;

  digitalWrite(OE_PIN, LOW); // Enable servos
  Serial.println("Servos enabled.");
  delay(500);

  x_pose_mutex = xSemaphoreCreateMutex();

  Serial.println("Initializing tasks...");
  xTaskCreatePinnedToCore(task_monitoring, "Monitoring", 4096, NULL, 1, &h_task_monitoring, 0);
  xTaskCreatePinnedToCore(task_command_parser, "CmdParser", 4096, NULL, 2, &h_task_command_parser, 1);
  xTaskCreatePinnedToCore(task_motion_interpolator, "Motion", 4096, NULL, 3, &h_task_motion_interpolator, 1); // Highest priority

  Serial.println("Initialization Complete. Ready for commands.");
}

void loop()
{
  vTaskDelete(NULL); // FreeRTOS handles everything.
}
