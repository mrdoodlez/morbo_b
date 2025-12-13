# MorBo-B Beer Buddy Rover Project

This is a small, home-built rover prototype I’m working on for fun and experimentation.  
Nothing fancy — just a simple setup where everything is kept as straightforward as possible.

The original idea was to have something that could carry a huge bag full of beer for me to the beach.

## What the rover can do (so far)

- Follows an ArUco tag in the camera view using a very simple proportional controller.
- Keeps its orientation stable using an IMU processed with MotionFX on an STM32.
- Streams live video from the onboard camera.
- Sends back basic telemetry (angles, speeds, PWM, battery ADC values).

## How it’s put together

### Raspberry Pi — “lightweight perception”

All OpenCV runs on the Raspberry Pi:

- Camera → OpenCV → ArUco detection
- Computes tag position in rover frame and sends it to STM32
- Streams video through ffmpeg (low-latency RTMP)

### STM32 — “small realtime brain”

- Runs MotionFX for IMU orientation
- Runs a tiny P-controller computing velocities
- Drives motors through a custom PWM setup
- Uses a small driver for external pulse counters (optical encoders)
- Handles incoming commands from the Raspberry Pi and sends telemetry back

## Mechanics (all hand-built)

I built all the mechanical parts myself — nothing off-the-shelf except motors and bearings.

There are two physical versions of the rover:

1. A small indoor debugging rover for algorithm testing.
2. A larger, high-torque rover with custom 3D-printed gearboxes and stronger drive modules.

## Host side

On the PC there is a lightweight viewer that:

- Plots position, orientation and motor commands  
- Shows the incoming video stream  
- Visualises telemetry in real time

## Android monitoring app

There is also a very simple Android application used purely as a **telemetry monitor**:

- Connects to the rover over BLE
- Receives telemetry packets
- Plots basic rover data (orientation, velocities, tag tracking status) in real time
- Intended only for quick field diagnostics and status monitoring

**Disclaimer**

The Android application is a pure product of *vibe coding* and was generated almost entirely with the help of AI tools.
It exists solely as a practical utility for this project.

I do **not** consider it representative of my skills or experience in Android development, and it is **not** meant to showcase Android programming expertise in any way.

## Demo
https://www.linkedin.com/posts/stanislav-raskov-8b927817_robotics-embedded-opencv-activity-7396827851494297600-C0tn?utm_source=social_share_send&utm_medium=member_desktop_web&rcm=ACoAAANrug0B6drm2dej2f2d9ZgyykN8vfbWWls
