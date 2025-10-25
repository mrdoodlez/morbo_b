# Morbo-B
### Autonomous Tag-Following Rover

---

## Overview

**Morbo-B** is an experimental rover platform designed to follow a visual tag in real time.
The system combines low-level motion control on **STM32 + FreeRTOS** with a host-side vision and navigation stack (currently running on PC, later migrating to **Raspberry Pi**).

---

## System Architecture

### Onboard (STM32 + FreeRTOS)
- PWM motor control for differential drive wheels
- Wheel encoders for odometry
- 6DoF IMU with **ST MotionFX** sensor fusion
- Command interpreter capable of:
  - Relative moves (`dx`, `dy`)
  - Absolute positioning within the odom frame

### Host Side (PC / Raspberry Pi)
- **Python communication utility**
  - Sends motion commands via Bluetooth or serial link
  - Visualizes rover pose and trajectory in real time
- **OpenCV-based vision module**
  - Detects and tracks a red circular tag in the camera frame
  - Estimates tag position and orientation relative to the rover

---

## Current Capabilities
- Low-level motion control
- IMU fusion and odometry
- Tag detection via camera
- Command and visualization tools

---

## Work in Progress
- Integrate visual tracking output into motion commands
  - Convert tag position in camera frame → rover movement in odom frame
- Migrate host functionality to Raspberry Pi for onboard autonomy
- Improve latency and robustness of video streaming and processing

---

## Technologies
- **Embedded:** STM32, FreeRTOS, C/C++, ST MotionFX / MotionAC / MotionGC
- **Host:** Python, OpenCV, Matplotlib
- **Communication:** Bluetooth / Serial link

---

## Next Steps
1. Implement closed-loop tag-following logic
2. Optimize frame-to-command latency
3. Migrate host logic to Raspberry Pi
4. Add waypoint and trajectory tracking

---

## Demo
*Coming soon — real-time video of the rover following a colored tag.*

---

**Morbo-B** — experimental platform for vision-guided mobile robotics.
