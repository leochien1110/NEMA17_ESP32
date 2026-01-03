#include <AS5600.h>
#include <Arduino.h>
#include <Wire.h>

// --- SHARED DATA ---
// Accessed by both Core 0 (ROS) and Core 1 (Motor)
volatile long targetPos = 0;
volatile long currentPos = 0;

// --- INCLUDES ---
// Must be included after globals are defined if they use them (though they used
// extern)
#include "motor_driver.h"
#include "ros_task.h"

// --- PID & CONTROL CONSTANTS ---
// Copied/Adapted from NEMA17_PID_Control
const float COUNTS_TO_DEG = 360.0 / 4096.0;
const float DEG_TO_COUNTS = 4096.0 / 360.0;

float Kp = 12.0;
float Kd = 2.2;
float deadbandDeg = 0.5;
float maxVel = 8000.0; // steps/sec (High Speed)

// S-Curve Variables
float currentVel = 0;   // steps/sec
float currentAccel = 0; // steps/sec²
float maxAccel = 25000.0;
float maxDecel = 25000.0;
float jerkRatio = 1.0; // S-curve jerk ratio

AS5600 as5600;

// --- CONTROL LOGIC (Core 1) ---
void runPID(float dt) {
  // 1. Calculate Error
  float errorCounts = (float)targetPos - (float)currentPos;
  float errorDeg = errorCounts * COUNTS_TO_DEG;

  // 2. Deadband
  if (abs(errorDeg) < deadbandDeg) {
    currentVel = 0;
    currentAccel = 0;
    setMotorSpeed(0);
    return;
  }

  // 3. PD Controller
  static long lastPos = 0;
  float measuredVel = (currentPos - lastPos) / dt;
  lastPos = currentPos;

  float pTerm = Kp * errorCounts;
  float dTerm = -Kd * measuredVel;
  float desiredVel = pTerm + dTerm;

  // 4. Velocity Limits
  desiredVel = constrain(desiredVel, -maxVel, maxVel);

  // 5. S-Curve / Ramping
  float velChange = desiredVel - currentVel;

  // Decide Accel Limit
  bool isDecelerating =
      (abs(desiredVel) < abs(currentVel)) && (abs(velChange) > 100);
  float accelLimit = isDecelerating ? maxDecel : maxAccel;

  // Kickstart friction
  if (abs(currentVel) < 50 && abs(desiredVel) > 100) {
    currentVel = (desiredVel > 0) ? 200 : -200;
    currentAccel = 0;
  } else {
    // S-Curve Logic
    // Calculate required accel
    float desiredAccel = velChange / dt;

    // Clamp Accel
    if (abs(desiredAccel) > accelLimit) {
      desiredAccel = (desiredAccel > 0) ? accelLimit : -accelLimit;
    }

    // Apply Jerk-limited Accel
    float accelChange = desiredAccel - currentAccel;
    float maxJerk = accelLimit / (jerkRatio * 0.5);
    float maxAccelChange = maxJerk * dt;

    if (abs(accelChange) > maxAccelChange) {
      currentAccel += (accelChange > 0) ? maxAccelChange : -maxAccelChange;
    } else {
      currentAccel = desiredAccel;
    }

    currentVel += currentAccel * dt;

    // Clamp Velocity if we overshot
    if ((velChange > 0 && currentVel > desiredVel) ||
        (velChange < 0 && currentVel < desiredVel)) {
      currentVel = desiredVel;
      currentAccel = 0;
    }
  }

  // 6. Hardware Command
  setMotorSpeed(currentVel);
}

// --- TASK 1: MOTOR CONTROL (Core 1) ---
void TaskMotorControl(void *pvParameters) {
  // Hardwire I2C pins for AS5600 if different from default
  // Node32s default: SDA=21, SCL=22
  Wire.begin(21, 22);
  Wire.setClock(1000000); // 1MHz I2C

  as5600.begin();
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);
  // Wait for sensor?
  delay(100);
  if (!as5600.isConnected()) {
    // Handle error? For now just continue or loop
  }

  // Initialize Hardware Timer
  setupMotorTimer();

  // Loop Timing
  unsigned long lastTime = micros();
  const int LOOP_INTERVAL_US = 1000; // 1kHz Control Loop

  for (;;) {
    unsigned long now = micros();
    if (now - lastTime >= LOOP_INTERVAL_US) {
      float dt = (now - lastTime) / 1000000.0;
      // Catch large dt (overflow or lag) or tiny dt
      if (dt > 0.01)
        dt = 0.01;
      if (dt < 0.0001)
        dt = 0.001;

      lastTime = now;

      // 1. Read Sensor
      // getCumulativePosition is blocking I2C, usually takes ~200-300us
      currentPos = as5600.getCumulativePosition();

      // 2. Run Control Algorithm
      runPID(dt);
    } else {
      // Yield slightly if we have time, but at 1kHz we are tight.
      // vTaskDelay(1) is 1ms, which might be too long if we want precise 1kHz.
      // Better to just busy wait or short yield.
      // Since we are pinned to Core 1 and nothing else is heavy there, busy
      // wait is OK-ish but `vTaskDelay` allows IDLE task to reset watchdog.
      // Let's rely on the `if` check and vTaskDelay(1) if we have plenty of
      // time, but strictly:
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // --- DEBUG PRINT (Every 500ms) ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print("Tgt:");
      Serial.print(targetPos * COUNTS_TO_DEG);
      Serial.print(" Cur:");
      Serial.print(currentPos * COUNTS_TO_DEG);
      Serial.print(" Vel:");
      Serial.println(currentVel);
    }
  }
}

void setup() {
  Serial.begin(115200); // For Debugging

  // Create Tasks
  // Task 1: Motor Control (High Priority, Core 1)
  xTaskCreatePinnedToCore(TaskMotorControl, "MotorCtrl", 4096, NULL, 10, NULL,
                          1);

  // Task 2: Micro-ROS (Medium Priority, Core 0)
  xTaskCreatePinnedToCore(TaskMicroROS, "RosComm", 10000, NULL, 5, NULL, 0);
}

void loop() {
  vTaskDelete(NULL); // Loop is not used
}