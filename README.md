# **Quadcopter Flight Stabilization Project**
_Developing experience in IMU processing and closed-loop flight control_

## **Project Overview**
This is an **on-going project** focused on **gaining experience** in IMU sensor processing and designing a **closed-loop flight stabilization controller**. The goal is **not** to build a fully functional flight controller but to ensure that the quadcopter can maintain stable flight (hovering) without falling.

## **Key Features**
âœ… **Real-time IMU data processing** with sensor fusion  
âœ… **250 Hz closed-loop flight stability control**  
âœ… **Two operation modes: Flight control & Calibration**  
âœ… **3D visualization of telemetry data using a Python tool**  

## **Hardware Components**
- **MCU**: STM32G474 Nucleo  
- **ESC**: Speedy-bee (PWM control protocol)  
- **IMU Sensor**: ST LSM6D (Accelerometer + Gyroscope)  
- **Communication**: UARTâ†”Bluetooth module (host communication)  
- **Power Supply**: DC/DC module  

## **Software & Technology Stack**
- **Firmware Language**: C  
- **MCU Frameworks**: STM32 HAL, FreeRTOS  
- **Sensor Fusion**: ST MotionGC, MotionAC, MotionFX  
- **Control Loop Frequency**: 250 Hz  
- **Communication Protocols**:  
  - **MCUâ†”IMU** via 4-wire **SPI (2.5 Mbit/s)**  
  - **MCUâ†”Host** via **UARTâ†”Bluetooth**  

## **Host Test Utility**
- **Written in Python**  
- **Visualizes telemetry in 3D**  
- **Displays raw sensor data in calibration mode**  

## **Next Steps (TODO)**
- Implement **short trajectory following** for the quadrotor  
