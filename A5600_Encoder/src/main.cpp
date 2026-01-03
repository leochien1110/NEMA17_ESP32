#include <AS5600.h>
#include <Arduino.h>
#include <Wire.h>

/**
 * AS5600 to ESP32 Pin Mapping:
 *
 * AS5600 | ESP32
 * -------|-------
 * VCC    | 3V3
 * GND    | GND
 * SDA    | GPIO 21
 * SCL    | GPIO 22
 * DIR    | GND (Sets direction - GND is usually Clockwise)
 */

#define I2C_SDA 21
#define I2C_SCL 22

AS5600 as5600;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give time for serial monitor to stabilize

  Serial.println("\n--- AS5600 Encoder Initialization ---");

  // Initialize I2C with specified pins
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize AS5600
  as5600.begin();

  if (as5600.isConnected()) {
    Serial.println("Connection status: AS5600 detected!");
  } else {
    Serial.println(
        "Connection status: AS5600 NOT found! Please check your wiring.");
    Serial.print("SDA: ");
    Serial.println(I2C_SDA);
    Serial.print("SCL: ");
    Serial.println(I2C_SCL);
  }
}

void loop() {
  static uint32_t lastTime = 0;
  if (millis() - lastTime > 100) {
    lastTime = millis();

    if (as5600.isConnected()) {
      // Raw 12-bit value (0-4095)
      uint16_t rawAngle = as5600.readAngle();
      // Degrees (0-360)
      float degrees = rawAngle * (360.0 / 4095.0);

      Serial.print("Raw: ");
      Serial.print(rawAngle);
      Serial.print("\tDeg: ");
      Serial.print(degrees, 2);
      Serial.print("\tPos: ");
      Serial.println(as5600.getCumulativePosition());
    } else {
      Serial.println("Waiting for AS5600...");
    }
  }
}