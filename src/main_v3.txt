/**
 * @file robot_arm_firmware_ros2.ino
 * @brief High-Performance ESP32 Firmware for ROS2 Real-Time Control
 * @author Gemini
 * @date 27 Sep 2025
 *
 * @description
 * This firmware controls a 6-axis robot arm using an ESP32, optimized for
 * real-time, low-latency control via a high-speed binary ROS2 bridge.
 *
 * Key Optimizations:
 * - Replaced ASCII serial protocol with a compact binary protocol to drastically
 * reduce parsing overhead and increase throughput.
 * - Baud rate increased to 921600 for high-speed communication.
 * - Command parsing is now a direct memory copy, eliminating string operations.
 * - Added a checksum for data integrity.
 * - Maintained the dual-core architecture for concurrent monitoring and control.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Arduino.h"
#include <Adafruit_INA219.h>
#include <EEPROM.h>

// ==========================================================================
// --- SERIAL PROTOCOL DEFINITION ---
// ==========================================================================
#define SERIAL_BAUD_RATE 921600
const uint8_t HEADER_BYTE = 0xA5;

// Command from PC to ESP32 (total 15 bytes)
struct __attribute__((packed)) CommandPacket
{
  uint8_t header;        // Should be 0xA5
  uint8_t command_id;    // 'M' for move, 'G' for grip, etc.
  float angles[6];       // Target angles for each joint
  float speed_factor;    // Movement speed
  float gripper_current; // Current for gripper
  uint8_t checksum;
};

// Status from ESP32 to PC (total 31 bytes)
struct __attribute__((packed)) StatusPacket
{
  uint8_t header; // Should be 0xA5
  float main_current_A;
  float gripper_current_mA;
  float actual_angles[6];
  uint8_t checksum;
};

CommandPacket g_rx_packet;
StatusPacket g_tx_packet;

// ==========================================================================
// --- Multi-Tasking & Safety Configuration ---
// ==========================================================================
TaskHandle_t h_task_robot_control = NULL;
TaskHandle_t h_task_monitoring = NULL;
SemaphoreHandle_t x_currentPulse_mutex;

volatile float g_collision_current_threshold_A = 2.5f;
volatile bool g_collisionDetected = false;
volatile float g_mainCurrent_A = 0.0;
volatile float g_gripperCurrent_mA = 0.0;

#define GRIPPER_AVG_SAMPLES 5
volatile float g_gripper_current_samples[GRIPPER_AVG_SAMPLES] = {0.0};
volatile int g_gripper_sample_idx = 0;
#define EEPROM_SIZE 128

// ==========================================================================
// --- SENSOR & HARDWARE CONFIGURATION ---
// ==========================================================================
#define ACS712_PIN 34
const float sens = 0.66; // 30A version is 66 mV/A
const float vcc = 3.3;
const int adcMax = 4095;
const int nSamples = 250;
float zero_point_voltage = 1.65;

Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define OE_PIN 13
#define AUTO_HOME_ON_BOOT 1

#define SERVOS_NUMBER 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PWM_FREQ 50
#define GRIPPER_SERVO_INDEX 5

// --- Speed Configuration ---
float g_speed_factor_ms_per_degree = 150.0f;
#define DURATION_MIN_MS 100
#define DURATION_MAX_MS 4000

// --- Servo Configuration ---
const uint8_t SERVOS[SERVOS_NUMBER] = {10, 11, 12, 13, 14, 15};
const float SERVOS_MIN[SERVOS_NUMBER] = {40.0f, 40.0f, 0.0f, 30.0f, 0.0f, 20.0f};
const float SERVOS_MAX[SERVOS_NUMBER] = {150.0f, 150.0f, 145.0f, 130.0f, 120.0f, 93.0f};
const float HOME_POSITION[SERVOS_NUMBER] = {90.00, 90.00, 90.00, 90.00, 90.00, 30.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {true, false, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {0, 0, 0, 0, 0, 0};

int currentPulse[16];

// --- Math & Helper Functions ---
uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++)
  {
    checksum ^= data[i];
  }
  return checksum;
}

int angleToPulse(float degrees)
{
  return map(degrees, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
}

float pulseToAngle(int pulse, int servoIndex)
{
  int pulseWithoutTrim = pulse - TRIM_US[servoIndex];
  float degrees = (float)(pulseWithoutTrim - SERVO_MIN_US) * 180.0f / (float)(SERVO_MAX_US - SERVO_MIN_US);
  return SERVO_INVERT[servoIndex] ? 180.0f - degrees : degrees;
}

static inline float easeInOutQuint(float t)
{
  if (t < 0.5f)
    return 16.0f * t * t * t * t * t;
  float f = t - 1.0f;
  return 1.0f + 16.0f * f * f * f * f * f;
}

// --- Core Safety & Motion ---
void emergency_stop()
{
  g_collisionDetected = true;
  digitalWrite(OE_PIN, HIGH);
  Serial.println("\nE-STOP\n");
  if (h_task_robot_control != NULL)
    vTaskSuspend(h_task_robot_control);
  if (h_task_monitoring != NULL)
    vTaskSuspend(h_task_monitoring);
}

void moveServosSmoothly(const float targetAngles[], uint32_t override_duration = 0)
{
  if (g_collisionDetected)
    return;

  int startPulse[SERVOS_NUMBER];
  int targetPulse[SERVOS_NUMBER];
  int deltaPulse[SERVOS_NUMBER];
  float max_angle_delta = 0.0f;

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
  for (int i = 0; i < SERVOS_NUMBER; i++)
  {
    uint8_t ch = SERVOS[i];
    startPulse[i] = currentPulse[ch];
    float startAngle = pulseToAngle(startPulse[i], i);
    float angle_delta = abs(targetAngles[i] - startAngle);
    if (angle_delta > max_angle_delta)
      max_angle_delta = angle_delta;

    float safeAngle = constrain(targetAngles[i], SERVOS_MIN[i], SERVOS_MAX[i]);
    if (SERVO_INVERT[i])
      safeAngle = 180.0f - safeAngle;
    targetPulse[i] = angleToPulse(safeAngle) + TRIM_US[i];
    deltaPulse[i] = targetPulse[i] - startPulse[i];
  }
  xSemaphoreGive(x_currentPulse_mutex);

  uint32_t move_duration_ms = (override_duration > 0) ? override_duration : (max_angle_delta * g_speed_factor_ms_per_degree);
  move_duration_ms = constrain(move_duration_ms, DURATION_MIN_MS, DURATION_MAX_MS);

  unsigned long startTime = millis();
  unsigned long currentTime;
  do
  {
    if (g_collisionDetected)
      return;
    currentTime = millis();
    float fraction = (float)(currentTime - startTime) / (float)move_duration_ms;
    if (fraction > 1.0f)
      fraction = 1.0f;
    float eased_fraction = easeInOutQuint(fraction);

    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      uint8_t ch = SERVOS[i];
      int pulse = startPulse[i] + (int)((float)deltaPulse[i] * eased_fraction);
      pwm.writeMicroseconds(ch, pulse);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  } while (currentTime - startTime < move_duration_ms);

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
  for (int i = 0; i < SERVOS_NUMBER; i++)
  {
    uint8_t ch = SERVOS[i];
    pwm.writeMicroseconds(ch, targetPulse[i]);
    currentPulse[ch] = targetPulse[i];
  }
  EEPROM.write(0, 123);
  EEPROM.put(1, currentPulse);
  EEPROM.commit();
  xSemaphoreGive(x_currentPulse_mutex);
}

void gripWithCurrentSensing(float targetAngle, float currentThreshold_mA)
{
  if (g_collisionDetected)
    return;

  const uint8_t gripperChannel = SERVOS[GRIPPER_SERVO_INDEX];
  const int gripReleaseSteps = 30;

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
  int startPulse = currentPulse[gripperChannel];
  xSemaphoreGive(x_currentPulse_mutex);

  float safeAngle = constrain(targetAngle, SERVOS_MIN[GRIPPER_SERVO_INDEX], SERVOS_MAX[GRIPPER_SERVO_INDEX]);
  if (SERVO_INVERT[GRIPPER_SERVO_INDEX])
    safeAngle = 180.0f - safeAngle;
  int targetPulseVal = angleToPulse(safeAngle) + TRIM_US[GRIPPER_SERVO_INDEX];
  int step = (targetPulseVal > startPulse) ? 1 : -1;

  for (int p = startPulse; (step > 0) ? (p <= targetPulseVal) : (p >= targetPulseVal); p += step)
  {
    if (g_collisionDetected)
      return;
    pwm.writeMicroseconds(gripperChannel, p);
    vTaskDelay(pdMS_TO_TICKS(15));

    if (abs(g_gripperCurrent_mA) > currentThreshold_mA)
    {
      int finalPulse = p - (step * gripReleaseSteps);
      pwm.writeMicroseconds(gripperChannel, finalPulse);

      xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
      currentPulse[gripperChannel] = finalPulse;
      EEPROM.write(0, 123);
      EEPROM.put(1, currentPulse);
      EEPROM.commit();
      xSemaphoreGive(x_currentPulse_mutex);
      return;
    }
  }

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
  currentPulse[gripperChannel] = targetPulseVal;
  EEPROM.write(0, 123);
  EEPROM.put(1, currentPulse);
  EEPROM.commit();
  xSemaphoreGive(x_currentPulse_mutex);
}

void goHome(uint16_t duration)
{
  moveServosSmoothly(HOME_POSITION, duration);
}

// --- CORE 1: Robot Control Task (Handles incoming commands) ---
void task_robot_control(void *pvParameters)
{
  Serial.println("INFO: Control Task started on Core 1.");
  uint8_t buffer[sizeof(CommandPacket)];
  int buffer_pos = 0;

  for (;;)
  {
    while (Serial.available())
    {
      uint8_t byte_in = Serial.read();
      if (buffer_pos == 0 && byte_in != HEADER_BYTE)
      {
        continue; // Wait for header
      }

      buffer[buffer_pos++] = byte_in;

      if (buffer_pos == sizeof(CommandPacket))
      {
        // Full packet received, cast to struct
        memcpy(&g_rx_packet, buffer, sizeof(CommandPacket));

        // Verify checksum
        uint8_t calculated_checksum = calculate_checksum(buffer, sizeof(CommandPacket) - 1);
        if (g_rx_packet.checksum == calculated_checksum)
        {
          // Command is valid, process it
          g_speed_factor_ms_per_degree = g_rx_packet.speed_factor;

          switch (g_rx_packet.command_id)
          {
          case 'M': // Move all joints
            moveServosSmoothly(g_rx_packet.angles);
            break;
          case 'G': // Grip
            gripWithCurrentSensing(g_rx_packet.angles[GRIPPER_SERVO_INDEX], g_rx_packet.gripper_current);
            break;
          case 'H': // Go Home
            goHome(2500);
            break;
          }
        }
        else
        {
          // Checksum failed, discard packet
        }
        buffer_pos = 0; // Reset for next packet
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to prevent busy-waiting
  }
}

// --- CORE 0: Monitoring Task (Sends status back to PC) ---
void task_monitoring(void *pvParameters)
{
  Serial.println("INFO: Monitoring Task started on Core 0.");
  long lastStatusSendTime = 0;

  for (;;)
  {
    // Read main current
    long val = 0;
    for (int i = 0; i < nSamples; i++)
      val += analogRead(ACS712_PIN);
    float avg_adc = (float)val / nSamples;
    float measured_voltage = (avg_adc / adcMax) * vcc;
    g_mainCurrent_A = (measured_voltage - zero_point_voltage) / sens;

    // Read and average gripper current
    g_gripper_current_samples[g_gripper_sample_idx] = ina219.getCurrent_mA();
    g_gripper_sample_idx = (g_gripper_sample_idx + 1) % GRIPPER_AVG_SAMPLES;
    float avg_current = 0.0;
    for (int i = 0; i < GRIPPER_AVG_SAMPLES; i++)
      avg_current += g_gripper_current_samples[i];
    g_gripperCurrent_mA = avg_current / GRIPPER_AVG_SAMPLES;

    if (abs(g_mainCurrent_A) > g_collision_current_threshold_A)
    {
      emergency_stop();
    }

    // Send status to ROS host periodically (e.g., at 50 Hz)
    if (millis() - lastStatusSendTime > 20)
    {
      g_tx_packet.header = HEADER_BYTE;
      g_tx_packet.main_current_A = g_mainCurrent_A;
      g_tx_packet.gripper_current_mA = g_gripperCurrent_mA;

      xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
      for (int i = 0; i < SERVOS_NUMBER; i++)
      {
        g_tx_packet.actual_angles[i] = pulseToAngle(currentPulse[SERVOS[i]], i);
      }
      xSemaphoreGive(x_currentPulse_mutex);

      uint8_t *tx_buffer = (uint8_t *)&g_tx_packet;
      g_tx_packet.checksum = calculate_checksum(tx_buffer, sizeof(StatusPacket) - 1);

      Serial.write(tx_buffer, sizeof(StatusPacket));
      lastStatusSendTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==========================================================================
// --- SETUP & MAIN LOOP ---
// ==========================================================================
void setup()
{
  delay(1000);
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
  Serial.println("\n--- ESP32 Robot Arm Firmware (ROS2 Binary Protocol) ---");

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("FATAL: Failed to initialise EEPROM!");
    while (1)
      ;
  }

  pinMode(OE_PIN, OUTPUT);
  pinMode(ACS712_PIN, INPUT);
  digitalWrite(OE_PIN, HIGH);
  Serial.println("Step 1: Outputs DISABLED.");

  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);

  if (!ina219.begin())
  {
    Serial.println("FATAL: Failed to find INA219 chip for gripper!");
    while (1)
      ;
  }
  ina219.setCalibration_32V_1A();
  Serial.println("Step 2: Sensors and PWM driver initialized.");

  if (EEPROM.read(0) == 123)
  {
    Serial.println("Step 3: Restored last pose from EEPROM.");
    EEPROM.get(1, currentPulse);
  }
  else
  {
    Serial.println("Step 3: No EEPROM pose found. Defaulting to HOME.");
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      float safeAngle = HOME_POSITION[i];
      if (SERVO_INVERT[i])
        safeAngle = 180.0f - safeAngle;
      currentPulse[SERVOS[i]] = angleToPulse(safeAngle) + TRIM_US[i];
    }
  }

  digitalWrite(OE_PIN, LOW);
  Serial.println("Step 4: Outputs ENABLED.");
  delay(500);

  x_currentPulse_mutex = xSemaphoreCreateMutex();

#if AUTO_HOME_ON_BOOT
  Serial.println("Step 5: Auto slow-move to HOME.");
  goHome(4000);
#endif

  Serial.println("\n--- Initializing Dual-Core Tasks ---");
  xTaskCreatePinnedToCore(task_monitoring, "Monitoring", 4096, NULL, 2, &h_task_monitoring, 0);
  xTaskCreatePinnedToCore(task_robot_control, "Control", 4096, NULL, 1, &h_task_robot_control, 1);

  Serial.println("\n--- Initialization Complete. Ready for commands. ---");
}

void loop()
{
  vTaskDelete(NULL); // FreeRTOS tasks handle everything.
}
