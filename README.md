# NEMA17 ESP32 Stepper Control Collection

A collection of PlatformIO projects for controlling NEMA17 stepper motors using ESP32, various drivers, and the AS5600 encoder.

## Projects included:

### 1. [NEMA17_TMC2209_CloseLoopControl](./NEMA17_TMC2209_CloseLoopControl)
- **Status:** Active / Latest
- **Hardware:** ESP32, TMC2209 Driver, AS5600 Encoder.
- **Features:** True closed-loop control, absolute angle tracking (360+ degrees), high-speed serial monitoring (921600 baud).

### 2. [A5600_Encoder](./A5600_Encoder)
- Basic interface and testing for the AS5600 magnetic encoder.

### 3. [NEMA17_A4988_BasicControl](./NEMA17_A4988_BasicControl)
- Open-loop control using the A4988 driver.

### 4. [NEMA17_TMC2209_BasicControl](./NEMA17_TMC2209_BasicControl)
- Basic open-loop control using the TMC2209 driver without encoder feedback.

## How to use:
1. Install [PlatformIO](https://platformio.org/).
2. Open any project folder in VS Code.
3. Configure `platformio.ini` if necessary.
4. Build and Upload.
