/**
 * @file main.cpp
 * @brief Fully interruptible, non-blocking firmware for ROS2 Robot Arm
 * @author Farouk
 * @date 26 Oct 2025
 *
 * @description
 * Version 3.9.2 (Deviation Grace Period Fix):
 * - [FIX] Behebt fälschliche E-Stops bei normalen Bewegungen.
 * - [NEU] Fügt eine 300ms "Grace Period" (DEVIATION_GRACE_PERIOD_MS) in task_monitoring hinzu.
 * - [FIX] Der Deviation-Check wird nun erst 300ms *nach* dem Start einer Bewegung
 * aktiviert, um den normalen Anlaufstrom (inrush current) zu ignorieren.
 * - [NEU] Fügt "Deviation Threshold" (g_collision_deviation_threshold_A) hinzu.
 * - [NEU] Fügt einen IIR-Filter in task_monitoring hinzu.
 * - [NEU] Implementiert Befehl 'D' zum Setzen des Deviation Threshold.
 * - [NEU] Aktualisiert ConfigPacket, ArmConfig und EEPROM-Logik.
 * - [FIX] Behebt den "Status Timeout" E-Stop-Fehler (collision_flag).
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <cmath>

// --- Stepper Motor (Base) Configuration ---
const int STEPPER_DIR_PIN = 16;
const int STEPPER_STEP_PIN = 17;
const int STEPPER_MS1_PIN = 18;
const int STEPPER_MS2_PIN = 19;
const int STEPPER_MS3_PIN = 23;
const int STEPPER_ENABLE_PIN = 4;

#define STEPPER_STEPS_PER_REV 200
#define STEPPER_MICROSTEPS 16
#define STEPPER_GEAR_RATIO 5.0f
#define STEPPER_PULSE_WIDTH_US 10
#define STEPPER_PULSE_DELAY_US 500

const float STEPPER_STEPS_PER_DEGREE = (STEPPER_STEPS_PER_REV * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO) / 360.0f;

// --- 12V Lüfter PWM Konfiguration ---
#define FAN_PWM_PIN 26
#define FAN_PWM_CHANNEL 0
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8
#define FAN_DUTY_CYCLE_7_PERCENT 18

// ==========================================================================
// --- Multi-Tasking & Safety Configuration ---
// ==========================================================================
TaskHandle_t h_task_command_parser = NULL;
TaskHandle_t h_task_monitoring = NULL;
TaskHandle_t h_task_motion_interpolator = NULL;
SemaphoreHandle_t x_pose_mutex;
SemaphoreHandle_t x_i2c_mutex;

volatile float g_collision_current_threshold_A = 5.0f;
volatile float g_collision_deviation_threshold_A = 1.0f; // --- NEU V3.9.1 ---
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
volatile int g_fan_duty_cycle = FAN_DUTY_CYCLE_7_PERCENT;

// ==========================================================================
// --- Hardware & Servo Configuration ---
// ==========================================================================
#define ACS712_PIN 34
#define OE_PIN 13
#define BAUD_RATE 921600
#define PWM_FREQ 50
#define GRIPPER_SERVO_INDEX 5
#define JOINT_1_SECOND_SERVO 11
#define SERVOS_NUMBER 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t SERVOS[SERVOS_NUMBER] = {0, 10, 12, 13, 14, 15};
float SERVOS_MIN[SERVOS_NUMBER] = {15.0f, 40.0f, 0.0f, 30.0f, 0.0f, 0.0f};
float SERVOS_MAX[SERVOS_NUMBER] = {345.0f, 150.0f, 145.0f, 130.0f, 180.0f, 120.0f};

const float HOME_POSITION[SERVOS_NUMBER] = {133.0f, 100.00, 110.00, 130.00, 15.00, 60.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {false, true, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {0, 0, 0, 0, 0, 0};

// ==========================================================================
// --- EEPROM Configuration (v3.9.1) ---
// ==========================================================================
#define EEPROM_SIZE 128
#define EEPROM_VALID_FLAG 123
#define EEPROM_VALID_FLAG_ADDR 0
#define EEPROM_POSE_ADDR 1
#define EEPROM_CONFIG_ADDR (EEPROM_POSE_ADDR + sizeof(float) * SERVOS_NUMBER)

struct ArmConfig
{
  float min_limits[SERVOS_NUMBER];
  float max_limits[SERVOS_NUMBER];
  float collision_threshold;
  float deviation_threshold; // --- NEU V3.9.1 ---
};

// ==========================================================================
// --- Binary Communication Protocol (v3.9.1) ---
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
  uint8_t cmd_id;
  float main_current_A;
  float gripper_current_mA;
  float angles[6];
  uint8_t collision_flag; // --- NEU V3.9.1 (E-Stop Fix) ---
  uint8_t checksum;
};
struct ConfigPacket
{
  uint8_t header;
  uint8_t cmd_id;
  float min_limits[6];
  float max_limits[6];
  float collision_threshold;
  float deviation_threshold; // --- NEU V3.9.1 ---
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
  xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
  float min_angle = SERVOS_MIN[servo_index];
  float max_angle = SERVOS_MAX[servo_index];
  xSemaphoreGive(x_pose_mutex);

  float safe_angle = constrain(degrees, min_angle, max_angle);
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
  digitalWrite(OE_PIN, HIGH);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
  // Serial.println("\nE-STOP: Halting.\n"); // Note: Serial prints in emergency_stop can be risky

  if (h_task_motion_interpolator != NULL)
    vTaskSuspend(h_task_motion_interpolator);

  // The command parser task MUST remain active to receive the 'E' (release) command.
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

    if (g_grip_target_current_mA > 0 && fabsf(g_gripperCurrent_mA) > g_grip_target_current_mA)
    {
      float final_grip_angle = (g_current_angles[GRIPPER_SERVO_INDEX] * (1.0f - eased_fraction)) + (g_target_angles[GRIPPER_SERVO_INDEX] * eased_fraction);
      g_current_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_target_angles[GRIPPER_SERVO_INDEX] = final_grip_angle;
      g_grip_target_current_mA = -1.0f;
    }

    long start_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
    long end_step_pos = (long)(g_target_angles[0] * STEPPER_STEPS_PER_DEGREE);
    long total_steps_for_move = end_step_pos - start_step_pos;
    long current_target_step_pos = start_step_pos + (long)(total_steps_for_move * eased_fraction);
    long steps_to_move = current_target_step_pos - g_stepper_current_step_pos;

    // --- Stepper-Logik (Kein I2C, sicher) ---
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

    // --- Servo-Logik (NUTZT I2C, MUSS GESCHÜTZT WERDEN) ---
    for (int i = 1; i < SERVOS_NUMBER; i++)
    {
      float interpolated_angle = (g_current_angles[i] * (1.0f - eased_fraction)) + (g_target_angles[i] * eased_fraction);

      xSemaphoreGive(x_pose_mutex);
      int pulse = angleToPulse(interpolated_angle, i);
      xSemaphoreTake(x_pose_mutex, portMAX_DELAY);

      xSemaphoreTake(x_i2c_mutex, portMAX_DELAY);
      pwm.writeMicroseconds(SERVOS[i], pulse);
      if (i == 1)
      {
        pwm.writeMicroseconds(JOINT_1_SECOND_SERVO, pulse);
      }
      xSemaphoreGive(x_i2c_mutex);
    }

    if (fraction >= 1.0f)
    {
      g_grip_target_current_mA = -1.0;
      for (int i = 0; i < SERVOS_NUMBER; i++)
      {
        g_current_angles[i] = g_target_angles[i];
      }
      g_stepper_current_step_pos = end_step_pos;
      g_move_duration_ms = 0; // --- WICHTIG: Bewegung als beendet markieren ---
    }
    xSemaphoreGive(x_pose_mutex);
  }
}

// ==========================================================================
// --- Command Parser Task (Core 1, Low Priority) ---
// ==========================================================================
void parse_command(const CommandPacket &cmd)
{
  if (g_collisionDetected && cmd.cmd_id != 'E')
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
      g_target_angles[i] = constrain(target, SERVOS_MIN[i], SERVOS_MAX[i]);
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
    if (g_move_duration_ms < 50)
      g_move_duration_ms = 50; // Mindestbewegungszeit
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
    EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
    EEPROM.put(EEPROM_POSE_ADDR, angles_to_save);
    EEPROM.commit();
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: Current pose saved to EEPROM.");
    break;
  }
  case 'N':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      if (cmd.angles[i] < SERVOS_MAX[i])
      {
        SERVOS_MIN[i] = cmd.angles[i];
      }
    }
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: SERVOS_MIN limits updated in RAM.");
    break;
  }
  case 'X':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      if (cmd.angles[i] > SERVOS_MIN[i])
      {
        SERVOS_MAX[i] = cmd.angles[i];
      }
    }
    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: SERVOS_MAX limits updated in RAM.");
    break;
  }
  case 'T':
  {
    float new_threshold = cmd.speed_factor;
    if (new_threshold > 0.1f && new_threshold < 20.0f)
    {
      g_collision_current_threshold_A = new_threshold;
      Serial.print("INFO: Main current threshold set to: ");
      Serial.println(g_collision_current_threshold_A);
    }
    break;
  }
  // --- NEU V3.9.1: Deviation Threshold setzen ---
  case 'D':
  {
    float new_threshold = cmd.speed_factor;
    if (new_threshold > 0.05f && new_threshold < 10.0f)
    {
      g_collision_deviation_threshold_A = new_threshold;
      Serial.print("INFO: Deviation current threshold set to: ");
      Serial.println(g_collision_deviation_threshold_A);
    }
    break;
  }
  case 'C':
  {
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    ArmConfig cfg;
    memcpy(cfg.min_limits, (void *)SERVOS_MIN, sizeof(SERVOS_MIN));
    memcpy(cfg.max_limits, (void *)SERVOS_MAX, sizeof(SERVOS_MAX));
    cfg.collision_threshold = g_collision_current_threshold_A;
    cfg.deviation_threshold = g_collision_deviation_threshold_A; // --- NEU V3.9.1 ---

    EEPROM.put(EEPROM_CONFIG_ADDR, cfg);
    EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
    EEPROM.commit();

    xSemaphoreGive(x_pose_mutex);
    Serial.println("INFO: Configuration saved to EEPROM.");
    break;
  }
  case 'R':
  {
    ConfigPacket packet;
    packet.header = HEADER_BYTE;
    packet.cmd_id = 'R';

    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    memcpy(packet.min_limits, (void *)SERVOS_MIN, sizeof(SERVOS_MIN));
    memcpy(packet.max_limits, (void *)SERVOS_MAX, sizeof(SERVOS_MAX));
    packet.collision_threshold = g_collision_current_threshold_A;
    packet.deviation_threshold = g_collision_deviation_threshold_A; // --- NEU V3.9.1 ---
    xSemaphoreGive(x_pose_mutex);

    packet.checksum = calculate_checksum((uint8_t *)&packet, sizeof(ConfigPacket) - 1);
    Serial.write((uint8_t *)&packet, sizeof(ConfigPacket));
    Serial.println("INFO: Sent config report.");
    break;
  }
  case 'E':
  {
    Serial.println("INFO: E-Stop Release command received.");
    g_collisionDetected = false;

    if (h_task_motion_interpolator != NULL)
      vTaskResume(h_task_motion_interpolator);

    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    digitalWrite(OE_PIN, LOW);
    Serial.println("INFO: Tasks resumed and motors re-enabled.");
    break;
  }

  case 'F':
  {
    int dutyCycle = (int)cmd.angles[0];
    g_fan_duty_cycle = constrain(dutyCycle, 0, 255);

    Serial.print("INFO: Fan duty cycle set to ");
    Serial.println(g_fan_duty_cycle);
    break;
  }
  } // end switch
}

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
// --- Monitoring Task (Core 0, Low Priority) ---
// ==========================================================================

// --- NEU V3.9.2: Grace Period (in ms) ---
// Zeit, die nach dem Start einer Bewegung gewartet wird, bevor die
// Abweichungsprüfung (Deviation Check) aktiviert wird, um den
// normalen Anlaufstrom (Inrush Current) zu ignorieren.
#define DEVIATION_GRACE_PERIOD_MS 300

void task_monitoring(void *pvParameters)
{
  long lastStatusSendTime = 0;
  const int nSamples = 250;
  const float sens = 0.066;
  const float vcc = 3.3;
  const int adcMax = 4095;

  int last_fan_duty_cycle = -1;

  // --- NEU V3.9.1: Deviation-Logik ---
  static float g_avgCurrent_A = 0.0f;
  const float IIR_ALPHA = 0.1f; // 0.1 = schnelle Anpassung, 0.01 = glatter

  for (;;)
  {
    // --- 1. Strommessung (Analog, sicher) ---
    long val = 0;
    for (int i = 0; i < nSamples; i++)
      val += analogRead(ACS712_PIN);
    float measured_voltage = ((float)val / nSamples / adcMax) * vcc;
    g_mainCurrent_A = (measured_voltage - g_calibrated_zero_voltage) / sens;

    // --- NEU V3.9.1: Deviation-Logik ---
    float current_deviation = fabsf(g_mainCurrent_A - g_avgCurrent_A);
    // Aktualisiere den Durchschnitt IMMER, damit er der Bewegung folgt
    g_avgCurrent_A = (g_avgCurrent_A * (1.0f - IIR_ALPHA)) + (g_mainCurrent_A * IIR_ALPHA);

    // --- 2. Greiferstrom (I2C, muss geschützt werden) ---
    xSemaphoreTake(x_i2c_mutex, portMAX_DELAY);
    float raw_grip_current = ina219.getCurrent_mA();
    xSemaphoreGive(x_i2c_mutex);

    if (isnan(raw_grip_current) || isinf(raw_grip_current))
    {
      g_gripperCurrent_mA = 0.0f;
    }
    else
    {
      g_gripperCurrent_mA = raw_grip_current;
    }

    // --- 3. Lüftersteuerung (PWM, sicher auf Core 0) ---
    if (g_fan_duty_cycle != last_fan_duty_cycle)
    {
      ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);
      last_fan_duty_cycle = g_fan_duty_cycle;
    }

    // --- 4. Sicherheitscheck (Absoluter Grenzwert) ---
    if (fabsf(g_mainCurrent_A) > g_collision_current_threshold_A)
    {
      Serial.println("E-STOP: Absolute threshold exceeded!");
      emergency_stop();
      continue; // Gehe zum Anfang der Schleife, Status wird gesendet
    }

    // --- 5. Sicherheitscheck (Abweichungs-Grenzwert) ---
    xSemaphoreTake(x_pose_mutex, portMAX_DELAY);
    unsigned long current_duration = g_move_duration_ms;
    unsigned long current_start_time = g_move_start_time;
    xSemaphoreGive(x_pose_mutex);

    bool is_moving = (millis() - current_start_time) < current_duration;

    // --- NEU V3.9.2: Grace Period Check ---
    // Prüfe die Abweichung nur, wenn:
    // 1. Der Arm sich bewegen SOLL (is_moving == true)
    // 2. Die Grace Period (Anlaufstrom-Phase) VORBEI ist.
    bool grace_period_over = (millis() - current_start_time) > DEVIATION_GRACE_PERIOD_MS;

    if (is_moving && grace_period_over && (current_deviation > g_collision_deviation_threshold_A))
    {
      Serial.print("E-STOP: Deviation threshold exceeded! Dev: ");
      Serial.println(current_deviation);
      emergency_stop();
      continue;
    }

    // --- 6. Statuspaket senden ---
    // --- E-STOP TIMEOUT FIX: Sende Status IMMER ---
    if (millis() - lastStatusSendTime > 100)
    {
      StatusPacket status;
      status.header = HEADER_BYTE;
      status.cmd_id = 'S';
      status.main_current_A = g_mainCurrent_A;
      status.gripper_current_mA = g_gripperCurrent_mA;

      // --- NEU V3.9.1: E-Stop-Statusflag setzen ---
      status.collision_flag = g_collisionDetected ? 1 : 0;

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

// I2C-Scanner-Funktion
void scan_i2c_availble_address()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
}

void setup()
{
  // --- Step 1: Disable all motors IMMEDIATELY ---
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);

  delay(2000);

  Serial.begin(BAUD_RATE);
  Serial.println("\n--- ESP32 Robot Arm Firmware (v3.9.2 Grace Period Fix) ---"); // Version aktualisiert
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

  // --- Step 3: Initialize I2C Devices ---
  Wire.begin();
  // scan_i2c_availble_address();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);

  if (!ina219.begin())
  {
    Serial.println("WARN: Failed to find INA219 sensor at 0x41. Gripper current will read 0.");
  }

  // --- Lüfter-PWM initialisieren ---
  Serial.println("INFO: Initializing 12V Fan PWM...");
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  Serial.println("INFO: Setting fan speed to 7% at startup.");
  ledcWrite(FAN_PWM_CHANNEL, g_fan_duty_cycle);

  // --- Step 4: Calibrate Current Sensor ---
  Serial.println("INFO: Calibrating current sensor. Do not move the robot...");
  long adc_sum = 0;
  const int n_cal_samples = 1000;
  const float vcc = 3.3;
  const int adcMax = 4095;
  for (int i = 0; i < n_cal_samples; i++)
  {
    adc_sum += analogRead(ACS712_PIN);
    delay(1);
  }
  g_calibrated_zero_voltage = ((float)adc_sum / n_cal_samples / adcMax) * vcc;
  Serial.print("INFO: Calibration complete. Zero-point voltage: ");
  Serial.println(g_calibrated_zero_voltage, 4);

  // --- Step 5: Load and Validate Startup Pose AND Config from EEPROM ---
  if (EEPROM.read(EEPROM_VALID_FLAG_ADDR) == EEPROM_VALID_FLAG)
  {
    Serial.println("INFO: Restoring pose and config from EEPROM.");
    float temp_angles[SERVOS_NUMBER];
    EEPROM.get(EEPROM_POSE_ADDR, temp_angles);

    ArmConfig cfg;
    EEPROM.get(EEPROM_CONFIG_ADDR, cfg);
    memcpy(SERVOS_MIN, cfg.min_limits, sizeof(SERVOS_MIN));
    memcpy(SERVOS_MAX, cfg.max_limits, sizeof(SERVOS_MAX));
    g_collision_current_threshold_A = cfg.collision_threshold;
    g_collision_deviation_threshold_A = cfg.deviation_threshold; // --- NEU V3.9.1 ---

    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      g_current_angles[i] = constrain(temp_angles[i], SERVOS_MIN[i], SERVOS_MAX[i]);
      g_target_angles[i] = g_current_angles[i];
    }
  }
  else
  {
    Serial.println("INFO: No valid config. Saving defaults to EEPROM.");
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      g_current_angles[i] = constrain(HOME_POSITION[i], SERVOS_MIN[i], SERVOS_MAX[i]);
      g_target_angles[i] = g_current_angles[i];
    }
    EEPROM.put(EEPROM_POSE_ADDR, g_current_angles);
    ArmConfig cfg;
    memcpy(cfg.min_limits, (void *)SERVOS_MIN, sizeof(SERVOS_MIN));
    memcpy(cfg.max_limits, (void *)SERVOS_MAX, sizeof(SERVOS_MAX));
    cfg.collision_threshold = g_collision_current_threshold_A;
    cfg.deviation_threshold = g_collision_deviation_threshold_A; // --- NEU V3.9.1 ---
    EEPROM.put(EEPROM_CONFIG_ADDR, cfg);

    EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_VALID_FLAG);
    EEPROM.commit();
  }
  Serial.println("INFO: Config loaded:");
  Serial.print("  Abs. Threshold: ");
  Serial.println(g_collision_current_threshold_A);
  Serial.print("  Dev. Threshold: ");
  Serial.println(g_collision_deviation_threshold_A); // --- NEU V3.9.1 ---

  // --- Step 6: Initialize Global Motion State ---
  g_stepper_current_step_pos = (long)(g_current_angles[0] * STEPPER_STEPS_PER_DEGREE);
  g_move_start_time = millis();
  g_move_duration_ms = 0;

  // --- Step 7: Create Mutexes ---
  x_pose_mutex = xSemaphoreCreateMutex();
  x_i2c_mutex = xSemaphoreCreateMutex();

  if (x_pose_mutex == NULL || x_i2c_mutex == NULL)
  {
    Serial.println("FATAL: Could not create mutex. Halting.");
    while (1)
      ;
  }

  // --- Step 8: Pre-load servos (Sicher, da Tasks noch nicht laufen) ---
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
  delay(500);

  // --- Step 9: Enable all motors ---
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  digitalWrite(OE_PIN, LOW);
  Serial.println("Motors enabled. Startup should be smooth.");

  // --- Step 10: Start Tasks ---
  Serial.println("Initializing tasks...");
  xTaskCreatePinnedToCore(task_monitoring, "Monitoring", 4096, NULL, 1, &h_task_monitoring, 0);
  xTaskCreatePinnedToCore(task_command_parser, "CmdParser", 4096, NULL, 2, &h_task_command_parser, 1);
  xTaskCreatePinnedToCore(task_motion_interpolator, "Motion", 4096, NULL, 3, &h_task_motion_interpolator, 1);
  Serial.println("Initialization Complete. Ready for commands.");
}

void loop()
{
  vTaskDelete(NULL);
}