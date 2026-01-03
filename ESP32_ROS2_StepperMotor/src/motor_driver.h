#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// --- Pin Definitions ---
const int STEP_PIN = 14;
const int DIR_PIN = 12;

// --- Timer Objects ---
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Volatile Variables (Shared between ISR and Main Code) ---
volatile bool stepState = false;
volatile unsigned long stepIntervalTicks = 0; // 0 = STOP
volatile bool stopMotor = true;

// --- The Interrupt Service Routine (ISR) ---
// This runs strictly on hardware timing. NO Serial prints here!
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  if (!stopMotor && stepIntervalTicks > 50) { // Min limit to prevent crash
    stepState = !stepState;                   // Toggle state
    digitalWrite(STEP_PIN, stepState);

    // Reset timer for next toggle
    // Note: We toggle twice per step (High -> Low), so interval is halved
    // effectively But usually we just set the period for the full pulse if
    // using specific modes. For manual toggling:
    if (stepState) {
      // Logic High: Hold for 2us (very short)
      // Since this ISR overhead is >2us, we can just set LOW immediately if we
      // want short pulses But for simplicity of this toggle method, we control
      // frequency.
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// --- Setup Function ---
void setupMotorTimer() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // 1. Initialize Timer 0, Prescaler 80 (80MHz / 80 = 1MHz = 1us per tick)
  timer = timerBegin(0, 80, true);

  // 2. Attach ISR
  timerAttachInterrupt(timer, &onTimer, true);

  // 3. Set Initial Alarm (Arbitrary slow speed)
  timerAlarmWrite(timer, 1000000, true); // 1 second

  // 4. Enable
  timerAlarmEnable(timer);
}

// --- Speed Setter (Call this from your PID loop) ---
void setMotorSpeed(float stepsPerSec) {
  portENTER_CRITICAL(&timerMux);

  if (abs(stepsPerSec) < 10.0) {
    stopMotor = true;
  } else {
    stopMotor = false;

    // Set Direction
    digitalWrite(DIR_PIN, stepsPerSec > 0
                              ? LOW
                              : HIGH); // Invert if needed checking direction

    // Calculate Ticks: 1,000,000 ticks/sec / steps/sec
    // Since ISR toggles (High/Low), we need 2 interrupts per step
    // So divide period by 2.
    unsigned long newInterval = (1000000.0 / abs(stepsPerSec)) / 2.0;

    // Safety Clamp (Don't go faster than hardware can handle, e.g. 20us)
    if (newInterval < 20)
      newInterval = 20;

    if (newInterval != stepIntervalTicks) {
      stepIntervalTicks = newInterval;
      timerAlarmWrite(timer, stepIntervalTicks, true);
    }
  }

  portEXIT_CRITICAL(&timerMux);
}

#endif
