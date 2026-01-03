#include <Arduino.h>

// Pin definitions for A4988 stepper driver
const int STEP_PIN = 14;    // Step pin connected to GPIO 14
const int DIR_PIN = 12;     // Direction pin connected to GPIO 12
const int BUTTON_PIN = 0;   // Button pin connected to GPIO 0 for revolution adjustment

// Motor parameters
const int STEPS_PER_REV = 200;           // Standard NEMA17 motor (1.8° per step)
const int CLOCKWISE_DELAY = 1000;        // Microseconds between steps for clockwise (safe speed)
const int COUNTERCLOCKWISE_DELAY = 2000; // Microseconds between steps for counterclockwise (2x slower)
const int REVOLUTION_PAUSE = 1000;       // Milliseconds to pause between revolutions

// Revolution count options (1, 2, 5, 10)
const int REV_OPTIONS[] = {1, 2, 5, 10};
const int NUM_REV_OPTIONS = 4;

// Button debouncing
const int BUTTON_DEBOUNCE_DELAY = 200;  // Milliseconds

// Variables
bool clockwise = true;
int currentRevIndex = 0;  // Index into REV_OPTIONS array (starts with 1 revolution)
int currentRevolutions = REV_OPTIONS[currentRevIndex];  // Current revolution count

// Button state tracking
unsigned long lastButtonPress = 0;
int lastButtonState = HIGH;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("A4988 NEMA17 Stepper Motor Controller");
  Serial.println("Step Pin: 14, Direction Pin: 12, Button Pin: 0");
  Serial.println("Clockwise: Fast, Counterclockwise: 2x Slower");
  Serial.println("Short press GPIO0 button to cycle revolutions: 1, 2, 5, 10");
  Serial.print("Current revolutions: ");
  Serial.println(currentRevolutions);
  Serial.println("Starting motor control...\n");

  // Configure pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Configure button pin as input with pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Set initial direction (LOW = clockwise, HIGH = counterclockwise)
  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);
}

void loop() {
  // Check for button press to change revolution count
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH &&
      (millis() - lastButtonPress) > BUTTON_DEBOUNCE_DELAY) {

    // Cycle to next revolution option
    currentRevIndex = (currentRevIndex + 1) % NUM_REV_OPTIONS;
    currentRevolutions = REV_OPTIONS[currentRevIndex];

    // Print new setting
    Serial.print("Revolution count changed to: ");
    Serial.println(currentRevolutions);

    lastButtonPress = millis();
  }
  lastButtonState = buttonState;

  // Print current direction and revolution count
  Serial.print("Starting ");
  Serial.print(currentRevolutions);
  Serial.print(" revolution(s) - Direction: ");
  Serial.println(clockwise ? "Clockwise (Fast)" : "Counterclockwise (Slow)");

  // Set direction pin (swapped to correct rotation direction)
  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);

  // Small delay to allow direction change to settle
  delayMicroseconds(100);

  // Choose speed based on direction
  int stepDelay = clockwise ? CLOCKWISE_DELAY : COUNTERCLOCKWISE_DELAY;

  // Complete the specified number of revolutions
  int totalSteps = currentRevolutions * STEPS_PER_REV;
  for (int step = 0; step < totalSteps; step++) {
    // Generate one step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay / 2);
  }
  
  // Print completion message
  Serial.print(currentRevolutions);
  Serial.println(" revolution(s) completed!");
  Serial.println("Pausing for 1 second...\n");
  
  // Pause between revolutions
  delay(REVOLUTION_PAUSE);
  
  // Change direction for next revolution
  clockwise = !clockwise;
}