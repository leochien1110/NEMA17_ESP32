# ESP32 Dual-Core ROS 2 Stepper Motor Control

This project implements a precise stepper motor controller using an ESP32.
- **Core 1**: Dedicated to Real-Time Motor Control (PID + Hardware Timer).
- **Core 0**: Dedicated to Micro-ROS communications over WiFi.

## 1. Setup & Build (MacOS/Linux)

### Prerequisites
**MacOS**:
```bash
brew install cmake binutils
```

**Ubuntu**:
```bash
sudo apt update && sudo apt install build-essential cmake binutils
```

### Build & Upload
1.  Open Project in VSCode with PlatformIO.
2.  **Build**: Click the "Checkmark" icon (or run `pio run`).
3.  **Upload**: Connect ESP32 and click "Arrow" icon (or `pio run -t upload`).
4.  **Monitor**: Click "Plug" icon (or `pio device monitor`).
    - *Note:* Ensure you see "Connected to WiFi!" and "Micro-ROS Agent Connected!".

## 2. Run Micro-ROS Agent (On MacOS)

The agent acts as the bridge between the ESP32 (WiFi) and your ROS 2 Desktop environment.

**Run this in a Terminal on MacOS:**
```bash
# Allow network traffic (Open port 8888/UDP)
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888
```
*Wait until you see the ESP32 connect in the agent logs.*

## 3. Test Control (On MacOS)

Since you likely don't have ROS 2 installed natively on Mac, use a docker container to run ROS 2 commands.

**Open a NEW Terminal:**
```bash
# Note: On Mac, --net=host works differently. 
# For simple topic pub/sub with the agent on the same host, this usually works:
docker run -it --rm --net=host ros:humble-ros-base /bin/bash
```

### Commands inside the Container:
**0. Source ROS Setup**
```bash
source /opt/ros/humble/setup.bash
```

**1. Check Connection**
```bash
ros2 topic list
# You should see:
# /motor/encoder
# /motor/target_pos
```

**2. Monitor Position (Encoder)**
```bash
ros2 topic echo /motor/encoder
```
*Rotate the motor manually, and you should see the angle (in degrees) update.*

**3. Move Motor (Control)**
Run these commands in another terminal to send targets (in Degrees):
```bash
# Move to 360 degrees
ros2 topic pub --once /motor/target_pos std_msgs/msg/Float32 "data: 360.0"

# Move back to 0
ros2 topic pub --once /motor/target_pos std_msgs/msg/Float32 "data: 0.0"

# Spin continuously (Simulate by updating target far away)
ros2 topic pub --once /motor/target_pos std_msgs/msg/Float32 "data: 5000.0"
```

## Build with PlatformIO
```bash
pio run
pio run -t upload
pio device monitor
```

On MacOS,
```bash
/Users/wychien/.platformio/penv/bin/pio run --target upload -d /Users/wychien/Documents/PlatformIO/Projects/ESP32_ROS2_StepperMotor
/Users/wychien/.platformio/penv/bin/pio device monitor -d /Users/wychien/Documents/PlatformIO/Projects/ESP32_ROS2_StepperMotor
```

## 4. Future Roadmap: Cart Control (Nav2 Integration)
To upgrade this simple Motor Driver into a full **Rail Cart Controller** compatible with ROS 2 Navigation Stack (Nav2), the following features are required:

### 1. Homing / Zeroing Logic (Critical)
*   **Problem**: `currentPos` starts at 0 on power-up, regardless of actual cart position.
*   **Solution**: Implement a Homing Routine.
    *   **Logic**: Drive Left until Limit Switch hit -> Set `offset` -> Drive to center.
    *   **ROS**: Add a Service (e.g., `/motor/home`) to trigger calibration.

### 2. Position Limits (Soft Stops)
*   **Problem**: Motor can spin indefinitely (7200+ degrees), risking physical collision.
*   **Solution**: Define `MIN_POS_M` and `MAX_POS_M` (e.g., 0 to 1.5 meters).
    *   Reject ROS commands outside this safe range.

### 3. Unit Conversion (Degrees to Meters)
*   **Problem**: Current ROS interface uses **Degrees** (`Float32`). Nav2 uses **Meters**.
*   **Solution**:
    *   Define `COUNTS_PER_METER` based on Wheel Diameter & Gear Ratio.
    *   Update ROS topic to accept Meters or handle conversion in ESP32.

### 4. Velocity Command Interface (Nav2 Standard)
*   **Problem**: Nav2 outputs velocity commands, not position targets.
*   **Solution**:
    *   Add Subscriber to `/cmd_vel` (`geometry_msgs/Twist`).
    *   Implement "Velocity Mode" where PID maintains RPM instead of Angle.

### 5. Odometry Feedback
*   **Problem**: `/motor/encoder` is just a Float. Nav2 needs full Odometry.
*   **Solution**:
    *   Publish `/odom` (`nav_msgs/Odometry`) containing Position (X) and Velocity (Linear X).
    *   Publish simple TF transform (Odom -> Base Link).