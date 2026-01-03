#include <AS5600.h>
#include <Arduino.h>
#include <Wire.h>

// Pin definitions
const int STEP_PIN = 14;
const int DIR_PIN = 12;
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Motor parameters
const int STEPS_PER_REV = 1600; // 1/8 microstepping (200 * 8)
const int CW_DELAY = 1000;      // 1000us total delay (Speed V)
const int CCW_DELAY = 500;      // 500us total delay (Speed 2V)
const int NUM_REVOLUTIONS = 5;

AS5600 as5600;

// TRUE CLOSED LOOP: Move until the encoder reaches a specific target position
void moveMotorToTarget(long targetPos, int stepDelay) {
  long currentPos = as5600.getCumulativePosition();
  bool clockwise = (targetPos > currentPos);

  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);
  delayMicroseconds(5);

  uint32_t lastStepTime = micros();
  int pulseCount = 0;

  // Movement Loop
  while (clockwise ? (currentPos < targetPos) : (currentPos > targetPos)) {
    uint32_t now = micros();

    if (now - lastStepTime >= stepDelay) {
      lastStepTime += stepDelay;

      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      pulseCount++;

      // Update position from encoder (every 4 steps for efficiency)
      if (pulseCount % 4 == 0) {
        currentPos = as5600.getCumulativePosition();
      }

      // High-frequency telemetry
      if (pulseCount % 200 == 0) {
        float currentDeg = currentPos * (360.0 / 4096.0);
        Serial.print(clockwise ? "CW  " : "CCW ");
        Serial.print("Pulses: ");
        Serial.print(pulseCount);
        Serial.print(" | Deg: ");
        Serial.print(currentDeg, 1);
        Serial.print(" | Target: ");
        Serial.print(targetPos * (360.0 / 4096.0), 1);
        Serial.print(" | Diff: ");
        Serial.println(targetPos - currentPos);
      }
    }
  }
  Serial.println(">> Target Reached!");
}

void setup() {
  Serial.begin(921600);
  delay(1000);
  Serial.println("\n--- NEMA17 Accurate Closed-Loop ---");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  as5600.begin();

  // Software flip: ensure CW results in positive values
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);

  if (as5600.isConnected()) {
    Serial.println("Encoder: Online");
  } else {
    Serial.println("Encoder: ERROR");
  }

  as5600.resetCumulativePosition();
}

void loop() {
  // 1. Move CW to +5 Revolutions (1800 degrees)
  long target = 5 * 4096;
  Serial.println("\n>> Moving to +1800.0 Degrees");
  moveMotorToTarget(target, CW_DELAY);
  delay(1000);

  // 2. Move CCW back to 0 Degrees
  Serial.println("\n>> Moving back to 0.0 Degrees");
  moveMotorToTarget(0, CCW_DELAY);
  delay(2000);
}