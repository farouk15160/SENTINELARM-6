#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Arduino.h"
#include <Adafruit_INA219.h>
#include <EEPROM.h>

// ==========================================================================
// --- Multi-Tasking & Safety Configuration ---
// ==========================================================================
TaskHandle_t h_task_robot_control = NULL;
TaskHandle_t h_task_monitoring = NULL;
SemaphoreHandle_t x_currentPulse_mutex;

// SAFETY: If main current exceeds this (in Amps) for any reason, arm will stop.
// This is now a variable that can be changed with the 'C' command.
volatile float g_collision_current_threshold_A = 2.5f;

// Global state flags - volatile because they are shared between cores
volatile bool g_collisionDetected = false;
volatile float g_mainCurrent_A = 0.0;
volatile float g_gripperCurrent_mA = 0.0;

// --- Gripper Current Averaging ---
#define GRIPPER_AVG_SAMPLES 5
volatile float g_gripper_current_samples[GRIPPER_AVG_SAMPLES] = {0.0};
volatile int g_gripper_sample_idx = 0;

// --- EEPROM Configuration ---
#define EEPROM_SIZE 128

// ==========================================================================
// --- ACS712 Main Power Current Sensor Configuration ---
// ==========================================================================
#define ACS712_PIN 34    // Use an ADC1 pin for better stability (e.g., 34)
const float sens = 0.66; // CORRECTED: 30A version is 0.66 V/A
const float vcc = 3.3;   // ESP32 operates at 3.3V
const int adcMax = 4095; // ESP32 has a 12-bit ADC (0-4095)
const int nSamples = 250;
float zero_point_voltage = 1.65;

// ==========================================================================
// --- SENSOR & HARDWARE OBJECTS ---
// ==========================================================================
Adafruit_INA219 ina219 = Adafruit_INA219(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
void scan_i2c_availble_address();

// --- Core Safety & Configuration ---
#define OE_PIN 13
#define AUTO_HOME_ON_BOOT 1

#define SERVOS_NUMBER 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PWM_FREQ 50
#define GRIPPER_SERVO_INDEX 5

// --- NEW: Speed Configuration (replaces fixed duration) ---
// This factor determines how many milliseconds it takes to move one degree.
float g_speed_factor_ms_per_degree = 20.0f;
#define DURATION_MIN_MS 100
#define DURATION_MAX_MS 4000 // Increased max duration for slow, large moves

const uint8_t SERVOS[SERVOS_NUMBER] = {10, 11, 12, 13, 14, 15};
const float SERVOS_MIN[SERVOS_NUMBER] = {40.0f, 40.0f, 0.0f, 30.0f, 0.0f, 7.0f};
const float SERVOS_MAX[SERVOS_NUMBER] = {150.0f, 150.0f, 145.0f, 130.0f, 120.0f, 118.0f};
const float HOME_POSITION[SERVOS_NUMBER] = {90.00, 90.00, 90.00, 90.00, 90.00, 90.00};
const bool SERVO_INVERT[SERVOS_NUMBER] = {true, false, false, false, false, false};
const int16_t TRIM_US[SERVOS_NUMBER] = {52, 333, 0, 0, 0, 0};

String serialBuffer = "";
int currentPulse[16];

// --- Math & Conversion Helpers ---
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

// --- IMPROVED: Replaced Cubic with Quintic easing for a smoother motion profile ---
static inline float easeInOutQuint(float t)
{
  if (t < 0.5f)
  {
    return 16.0f * t * t * t * t * t;
  }
  float f = t - 1.0f;
  return 1.0f + 16.0f * f * f * f * f * f;
}

// ==========================================================================
// --- SAFETY FUNCTION ---
// ==========================================================================
void emergency_stop()
{
  g_collisionDetected = true;
  digitalWrite(OE_PIN, HIGH); // Disable servo driver outputs
  Serial.println("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.printf("!! E-STOP TRIGGERED: Main current exceeded %.2f A\n", g_collision_current_threshold_A);
  Serial.println("!! Servos disabled. Please reset the device.");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
  vTaskSuspend(h_task_robot_control); // Suspend both tasks
  vTaskSuspend(h_task_monitoring);
}

// ==========================================================================
// --- Core Motion Functions (Now faster and safer) ---
// ==========================================================================
void moveServosSmoothly(const uint8_t channels[], const float targetAngles[], uint8_t num_channels, uint32_t override_duration = 0)
{
  if (g_collisionDetected)
    return;

  int startPulse[num_channels];
  int targetPulse[num_channels];
  int deltaPulse[num_channels];
  float max_angle_delta = 0.0f;

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY); // Lock mutex
  for (int i = 0; i < num_channels; i++)
  {
    uint8_t ch = channels[i];
    int servo_idx = -1;
    for (int j = 0; j < SERVOS_NUMBER; j++)
      if (SERVOS[j] == ch)
        servo_idx = j;
    if (servo_idx == -1)
      continue;

    startPulse[i] = currentPulse[ch];
    float startAngle = pulseToAngle(startPulse[i], servo_idx);
    float angle_delta = abs(targetAngles[i] - startAngle);
    if (angle_delta > max_angle_delta)
    {
      max_angle_delta = angle_delta;
    }

    float safeAngle = constrain(targetAngles[i], SERVOS_MIN[servo_idx], SERVOS_MAX[servo_idx]);
    if (SERVO_INVERT[servo_idx])
      safeAngle = 180.0f - safeAngle;
    targetPulse[i] = angleToPulse(safeAngle) + TRIM_US[servo_idx];
    deltaPulse[i] = targetPulse[i] - startPulse[i];
  }
  xSemaphoreGive(x_currentPulse_mutex); // Unlock mutex

  // --- NEW: Calculate move duration based on speed factor and angle distance ---
  uint32_t move_duration_ms;
  if (override_duration > 0)
  {
    move_duration_ms = override_duration;
  }
  else
  {
    move_duration_ms = max_angle_delta * g_speed_factor_ms_per_degree;
    move_duration_ms = constrain(move_duration_ms, DURATION_MIN_MS, DURATION_MAX_MS);
  }

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

    for (int i = 0; i < num_channels; i++)
    {
      uint8_t ch = channels[i];
      int pulse = startPulse[i] + (int)((float)deltaPulse[i] * eased_fraction);
      pwm.writeMicroseconds(ch, pulse);
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // Use FreeRTOS delay
  } while (currentTime - startTime < move_duration_ms);

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY); // Lock mutex
  for (int i = 0; i < num_channels; i++)
  {
    uint8_t ch = channels[i];
    pwm.writeMicroseconds(ch, targetPulse[i]);
    currentPulse[ch] = targetPulse[i];
  }
  EEPROM.write(0, 123);
  EEPROM.put(1, currentPulse);
  EEPROM.commit();
  xSemaphoreGive(x_currentPulse_mutex); // Unlock mutex
}

void gripWithCurrentSensing(float targetAngle, float currentThreshold_mA)
{
  if (g_collisionDetected)
    return;

  const uint8_t gripperChannel = SERVOS[GRIPPER_SERVO_INDEX];
  const int gripReleaseSteps = 30;

  Serial.printf("INFO: Gripping to angle %.1f with %.1f mA threshold.\n", targetAngle, currentThreshold_mA);

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
  int startPulse = currentPulse[gripperChannel];
  xSemaphoreGive(x_currentPulse_mutex);

  float safeAngle = constrain(targetAngle, SERVOS_MIN[GRIPPER_SERVO_INDEX], SERVOS_MAX[GRIPPER_SERVO_INDEX]);
  if (SERVO_INVERT[GRIPPER_SERVO_INDEX])
    safeAngle = 180.0f - safeAngle;
  int targetPulse = angleToPulse(safeAngle) + TRIM_US[GRIPPER_SERVO_INDEX];
  int step = (targetPulse > startPulse) ? 1 : -1;

  for (int p = startPulse; (step > 0) ? (p <= targetPulse) : (p >= targetPulse); p += step)
  {
    if (g_collisionDetected)
      return;

    pwm.writeMicroseconds(gripperChannel, p);
    vTaskDelay(pdMS_TO_TICKS(15));

    if (abs(g_gripperCurrent_mA) > currentThreshold_mA)
    {
      Serial.printf("INFO: Object gripped! Current of %.2f mA exceeded threshold of %.1f mA.\n", g_gripperCurrent_mA, currentThreshold_mA);
      int finalPulse = p - (step * gripReleaseSteps);
      if ((step > 0 && finalPulse < startPulse) || (step < 0 && finalPulse > startPulse))
        finalPulse = startPulse;

      pwm.writeMicroseconds(gripperChannel, finalPulse);

      xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
      currentPulse[gripperChannel] = finalPulse;
      EEPROM.write(0, 123);
      EEPROM.put(1, currentPulse);
      EEPROM.commit();
      xSemaphoreGive(x_currentPulse_mutex);

      Serial.printf("INFO: Backing off to a final pulse of %d to hold object securely.\n", finalPulse);
      return;
    }
  }

  Serial.println("INFO: Gripper reached target angle without exceeding current threshold.");

  xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
  currentPulse[gripperChannel] = targetPulse;
  EEPROM.write(0, 123);
  EEPROM.put(1, currentPulse);
  EEPROM.commit();
  xSemaphoreGive(x_currentPulse_mutex);
}

void goHome(uint16_t duration)
{
  float home_pos_copy[SERVOS_NUMBER];
  for (int i = 0; i < SERVOS_NUMBER; i++)
    home_pos_copy[i] = HOME_POSITION[i];
  home_pos_copy[1] = home_pos_copy[0];
  // Use the duration override for homing
  moveServosSmoothly(SERVOS, home_pos_copy, SERVOS_NUMBER, duration);
}

void parseSerialCommand(String cmd)
{
  cmd.trim();
  if (cmd.length() == 0)
    return;
  char commandType = cmd.charAt(0);
  String args = cmd.substring(cmd.indexOf(',') + 1);

  switch (commandType)
  {
  case 'B':
    Serial.println("INFO: BEGIN received. Performing slow move to HOME.");
    goHome(4000);
    break;
  case 'H':
    Serial.println("INFO: Moving to home position.");
    goHome(1500);
    break;
  case 'M':
  {
    const int expected_angles = SERVOS_NUMBER - 1;
    float receivedAngles[expected_angles];
    int argCount = 0;
    int lastIndex = 0;
    for (int i = 0; i < args.length(); i++)
    {
      if (args.charAt(i) == ',')
      {
        if (argCount < expected_angles)
        {
          receivedAngles[argCount++] = args.substring(lastIndex, i).toFloat();
          lastIndex = i + 1;
        }
      }
    }
    if (argCount < expected_angles)
    {
      receivedAngles[argCount++] = args.substring(lastIndex).toFloat();
    }
    if (argCount != expected_angles)
    {
      Serial.printf("ERROR: M command expects %d angles, got %d.\n", expected_angles, argCount);
      return;
    }
    float targetAngles[SERVOS_NUMBER];
    targetAngles[0] = receivedAngles[0];
    targetAngles[1] = receivedAngles[0];
    for (int i = 1; i < expected_angles; i++)
    {
      targetAngles[i + 1] = receivedAngles[i];
    }
    Serial.println("Moving all joints...");
    moveServosSmoothly(SERVOS, targetAngles, SERVOS_NUMBER);
    Serial.println("Move complete.");
    break;
  }
  case 'J':
  {
    int first_comma = args.indexOf(',');
    if (first_comma == -1)
      return;
    int joint_idx = args.substring(0, first_comma).toInt();
    float target_angle = args.substring(first_comma + 1).toFloat();
    if (joint_idx < 0 || joint_idx >= SERVOS_NUMBER)
      return;

    Serial.printf("Moving joint %d...\n", joint_idx);
    if (joint_idx == 0 || joint_idx == 1)
    {
      uint8_t channels[] = {SERVOS[0], SERVOS[1]};
      float angles[] = {target_angle, target_angle};
      moveServosSmoothly(channels, angles, 2);
    }
    else
    {
      uint8_t channels[] = {SERVOS[joint_idx]};
      float angles[] = {target_angle};
      moveServosSmoothly(channels, angles, 1);
    }
    Serial.println("Move complete.");
    break;
  }
  case 'G':
  {
    int first_comma = args.indexOf(',');
    if (first_comma == -1)
      return;
    float target_angle = args.substring(0, first_comma).toFloat();
    float current_threshold = args.substring(first_comma + 1).toFloat();
    if (current_threshold <= 0)
      return;
    gripWithCurrentSensing(target_angle, current_threshold);
    break;
  }
  // --- UPDATED: 'S' now sets speed factor ---
  case 'S':
  {
    float speed_factor = args.toFloat();
    if (speed_factor < 1.0f)
    { // Prevent division by zero or too fast moves
      Serial.println("ERROR: Speed factor must be 1.0 or greater.");
      return;
    }
    g_speed_factor_ms_per_degree = speed_factor;
    Serial.printf("Speed factor set to %.2f ms/degree.\n", g_speed_factor_ms_per_degree);
    break;
  }
  // --- NEW: 'C' sets the collision threshold ---
  case 'C':
  {
    float threshold = args.toFloat();
    if (threshold < 0.5f || threshold > 5.0f)
    { // Safety limits
      Serial.println("ERROR: Threshold must be between 0.5 and 5.0 Amps.");
      return;
    }
    g_collision_current_threshold_A = threshold;
    Serial.printf("Collision threshold set to %.2f A.\n", g_collision_current_threshold_A);
    break;
  }
  case 'T':
  {
    Serial.println("--- Current Robot Position (Angles) ---");
    xSemaphoreTake(x_currentPulse_mutex, portMAX_DELAY);
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      float currentAngle = pulseToAngle(currentPulse[SERVOS[i]], i);
      Serial.printf("Servo %d: %.2f deg\n", i, currentAngle);
    }
    xSemaphoreGive(x_currentPulse_mutex);
    Serial.println("------------------------------------");
    break;
  }
  default:
    Serial.printf("ERROR: Unknown command '%c'.\n", commandType);
    break;
  }
}

// ==========================================================================
// --- CORE 1: Robot Control Task ---
// ==========================================================================
void task_robot_control(void *pvParameters)
{
  Serial.println("Control Task started on Core 1.");
  for (;;)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      if (c == '\n')
      {
        if (serialBuffer.length() > 0)
        {
          parseSerialCommand(serialBuffer);
        }
        serialBuffer = "";
      }
      else
      {
        serialBuffer += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==========================================================================
// --- CORE 0: Monitoring Task ---
// ==========================================================================
void task_monitoring(void *pvParameters)
{
  Serial.println("Monitoring Task started on Core 0.");
  long lastPrintTime = 0;

  for (;;)
  {
    long val = 0;
    for (int i = 0; i < nSamples; i++)
    {
      val += analogRead(ACS712_PIN);
    }
    float avg_adc = (float)val / nSamples;
    float measured_voltage = (avg_adc / adcMax) * vcc;
    g_mainCurrent_A = (measured_voltage - zero_point_voltage) / sens;

    g_gripper_current_samples[g_gripper_sample_idx] = ina219.getCurrent_mA();
    g_gripper_sample_idx = (g_gripper_sample_idx + 1) % GRIPPER_AVG_SAMPLES;

    float avg_current = 0.0;
    for (int i = 0; i < GRIPPER_AVG_SAMPLES; i++)
    {
      avg_current += g_gripper_current_samples[i];
    }
    g_gripperCurrent_mA = avg_current / GRIPPER_AVG_SAMPLES;

    if (abs(g_mainCurrent_A) > g_collision_current_threshold_A)
    {
      emergency_stop();
    }

    if (millis() - lastPrintTime > 500)
    {
      Serial.printf("Status -> Main Current: %.3f A | Gripper Current: %.2f mA\n", g_mainCurrent_A, g_gripperCurrent_mA);
      lastPrintTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==========================================================================
// --- SETUP ---
// ==========================================================================
void setup()
{
  delay(1000);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n--- Safe Servo Boot Initializing for ESP32 (Dual-Core) ---");

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
  scan_i2c_availble_address();
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
  Serial.println("INA219 Gripper sensor initialized and calibrated for 32V/1A range.");

  delay(100);
  Serial.println("Step 2: PCA9685 initialized.");

  byte eepromFlag = EEPROM.read(0);
  if (eepromFlag == 123)
  {
    Serial.println("Step 3: Found and restored last pose from EEPROM.");
    EEPROM.get(1, currentPulse);
  }
  else
  {
    Serial.println("Step 3: No EEPROM pose found. Defaulting to HOME position.");
    for (int i = 0; i < SERVOS_NUMBER; i++)
    {
      float safeAngle = HOME_POSITION[i];
      if (SERVO_INVERT[i])
        safeAngle = 180.0f - safeAngle;
      currentPulse[SERVOS[i]] = angleToPulse(safeAngle) + TRIM_US[i];
    }
  }

  digitalWrite(OE_PIN, LOW);
  Serial.println("Step 4: Outputs ENABLED. Arm is holding position.");
  delay(500);

  x_currentPulse_mutex = xSemaphoreCreateMutex();

#if AUTO_HOME_ON_BOOT
  Serial.println("Step 5: Auto slow-move to HOME from last known position.");
  goHome(4000);
#else
  Serial.println("SAFE MODE: Waiting for BEGIN ('B') command. No auto movement.");
#endif

  Serial.println("\n--- Initializing Dual-Core Tasks ---");

  xTaskCreatePinnedToCore(
      task_monitoring, "Monitoring Task", 4096, NULL, 2, &h_task_monitoring, 0);

  xTaskCreatePinnedToCore(
      task_robot_control, "Robot Control Task", 4096, NULL, 1, &h_task_robot_control, 1);

  Serial.println("\n--- Initialization Complete. Ready for commands. ---");
}

void loop()
{
  vTaskDelete(NULL);
}

// --- Utility Functions ---
void scan_i2c_availble_address()
{
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning...");
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
