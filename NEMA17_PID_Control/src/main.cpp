#include <AS5600.h>
#include <Arduino.h>
#include <Wire.h>

// Pin definitions
const int STEP_PIN = 14;
const int DIR_PIN = 12;
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Motor/Encoder Parameters
const float DEG_TO_COUNTS = 4096.0 / 360.0;
const float COUNTS_TO_DEG = 360.0 / 4096.0;

// PD Constants
float Kp = 15.0; // Lower to prevent stalling at start
float Kd = 0.5;  // Lower to prevent overshooting
float deadbandDeg = 0.5; // Stop if within this range (degrees)
float maxVel = 10000.0;  // Max velocity (steps/sec) - prevents decel stalling

// State Variables
long targetPos = 0; // Target position in encoder counts
long stepCount = 0; // Total steps sent

// Control Variables
long lastPos = 0; // Last position for velocity calculation
unsigned long lastPIDTime = 0;
const int PID_INTERVAL_US = 500; // 2kHz Loop

// Velocity Ramping (Acceleration Limiting)
float currentVel = 0;   // Current commanded velocity (steps/sec)
float currentAccel = 0; // Current acceleration (steps/sec²) - for jerk control
float maxAccel = 25000.0; // Max acceleration (steps/sec²) - adjustable
float maxDecel = 25000.0; // Max deceleration (steps/sec²) - slower for safety
float jerkRatio =
    0.33; // S-curve jerk ratio (0-1, where 0.66 = 66% smooth ramping)

unsigned long lastStepTime = 0;
unsigned long stepIntervalUs = 0;
unsigned long lastReportTime = 0;

AS5600 as5600;

void runPulsing() {
  if (stepIntervalUs > 0) {
    unsigned long nowUs = micros();
    if (nowUs - lastStepTime >= stepIntervalUs) {
      lastStepTime = nowUs;
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      stepCount++;
    }
  }
}

void updateControl(float dt) {
  long currentPos = as5600.getCumulativePosition();

  // PD Control Loop
  float errorCounts = (float)targetPos - (float)currentPos;
  float errorDeg = errorCounts * COUNTS_TO_DEG;

  // 1. Deadband Check: Stop if close enough
  if (abs(errorDeg) < deadbandDeg) {
    stepIntervalUs = 0;
    currentVel = 0;
    lastPos = currentPos;
    return;
  }

  // Calculate actual velocity (change in position over time)
  float measuredVel = (currentPos - lastPos) / dt;
  lastPos = currentPos;

  // PD Controller: P drives toward target, D opposes velocity (damping)
  float pTerm = Kp * errorCounts;
  float dTerm = -Kd * measuredVel; // Negative because we oppose velocity

  float desiredVel = pTerm + dTerm;

  // Velocity limit: prevent going too fast (prevents decel stalling)
  desiredVel = constrain(desiredVel, -maxVel, maxVel);

  // Only move if desired velocity is meaningful (avoid tiny drifts)
  if (abs(desiredVel) < 10) {
    stepIntervalUs = 0;
    currentVel = 0; // Reset ramping
    return;
  }

  // **S-Curve Jerk Control**: Smooth acceleration ramping
  float velChange = desiredVel - currentVel;

  // Use different limits for acceleration vs deceleration
  bool isDecelerating =
      (abs(desiredVel) < abs(currentVel)) && (abs(velChange) > 100);
  float accelLimit = isDecelerating ? maxDecel : maxAccel;

  // Starting boost: overcome static friction when starting from rest
  if (abs(currentVel) < 50 && abs(desiredVel) > 100) {
    // Jump to minimum velocity to break static friction
    currentVel = (desiredVel > 0) ? 200 : -200;
    currentAccel = 0; // Reset acceleration for jerk control
  } else {
    // Calculate desired acceleration based on velocity error
    float desiredAccel = (velChange / dt); // Ideal accel to reach target

    // Limit to max accel/decel
    if (abs(desiredAccel) > accelLimit) {
      desiredAccel = (desiredAccel > 0) ? accelLimit : -accelLimit;
    }

    // **Jerk Limiting**: Smooth the acceleration change (S-curve)
    float accelChange = desiredAccel - currentAccel;
    float maxJerk =
        accelLimit / (jerkRatio * 0.5); // Jerk = accel_rate / time_fraction
    float maxAccelChange = maxJerk * dt;

    if (abs(accelChange) > maxAccelChange) {
      // Ramp acceleration smoothly (S-curve phase)
      currentAccel += (accelChange > 0) ? maxAccelChange : -maxAccelChange;
    } else {
      // Can reach desired acceleration
      currentAccel = desiredAccel;
    }

    // Apply the smoothed acceleration to velocity
    currentVel += currentAccel * dt;

    // Safety clamp
    if ((velChange > 0 && currentVel > desiredVel) ||
        (velChange < 0 && currentVel < desiredVel)) {
      currentVel = desiredVel;
      currentAccel = 0;
    }
  }

  // Convert Ramped Velocity to Step Interval
  stepIntervalUs = 1000000.0 / abs(currentVel);
  digitalWrite(DIR_PIN, currentVel > 0 ? LOW : HIGH);
}

void setup() {
  Serial.begin(921600);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(1000000);

  as5600.begin();
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);
  as5600.resetCumulativePosition();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  lastPIDTime = micros();

  Serial.println("\n--- Simplified NEMA17 PD Control ---");
  Serial.println("Commands: T<deg>, P<val>, D<val>, B<val>, A<val>, V<val>");
  Serial.println("  T=Target, P=Kp, D=Kd, B=Deadband, A=Accel, V=MaxVel");
}

void loop() {
  // ====== CRITICAL: Step Generation FIRST (Never Block This!) ======
  runPulsing();

  unsigned long nowUs = micros();

  // ====== PID Control Loop (2kHz) ======
  if (nowUs - lastPIDTime >= PID_INTERVAL_US) {
    float dt = (nowUs - lastPIDTime) / 1000000.0;
    lastPIDTime = nowUs;
    updateControl(dt);
  }

  // ====== Telemetry - ONLY print when moving slow or stopped ======
  // Serial.print() blocks for ~700μs, causing step starvation at high speeds!
  if (millis() - lastReportTime > 100) { // Reduced to 10Hz
    if (abs(currentVel) < 1000) {        // Only print if slow/stopped
      lastReportTime = millis();
      float currentDeg = as5600.getCumulativePosition() * COUNTS_TO_DEG;
      float targetDeg = targetPos * COUNTS_TO_DEG;

      Serial.print("Ang: ");
      Serial.print(currentDeg, 2);
      Serial.print(" | Tgt: ");
      Serial.print(targetDeg, 2);
      Serial.print(" | Steps: ");
      Serial.println(stepCount);
    } else {
      lastReportTime = millis(); // Update timer even if we skip
    }
  }

  // ====== Command Parsing (keep minimal) ======
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.length() > 0) {
      char cmd = toupper(in[0]);
      float val = in.substring(1).toFloat();

      if (cmd == 'T') {
        targetPos = val * DEG_TO_COUNTS;
        stepCount = 0;
        currentVel = 0; // Reset velocity ramping
      } else if (cmd == 'P') {
        Kp = val;
        Serial.print("Kp set to: ");
        Serial.println(Kp);
      } else if (cmd == 'D') {
        Kd = val;
        Serial.print("Kd set to: ");
        Serial.println(Kd);
      } else if (cmd == 'B') {
        deadbandDeg = val;
        Serial.print("Deadband set to: ");
        Serial.println(deadbandDeg);
      } else if (cmd == 'A') {
        maxAccel = val;
        Serial.print("Max Accel set to: ");
        Serial.println(maxAccel);
      } else if (cmd == 'V') {
        maxVel = val;
        Serial.print("Max Vel set to: ");
        Serial.println(maxVel);
      }
    }
  }
}