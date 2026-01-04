# Digital Twin

## Overview
The project uses a **digital twin workflow**: joint motion is validated in simulation (Isaac Sim) and then sent to real hardware using the same joint naming and control pipeline.

---

<p align="center">
  <img src="https://github.com/DmitriyB51/3d-Printed-5DOF-robotic-arm-Isaac-Simulator-integration-inverse-kinematics/raw/main/media/twin.gif" width="600">
</p>

## Data Flow (Simulation → Hardware)
1. **Isaac Sim** publishes simulated joint angles to `/joint_states` (`sensor_msgs/JointState`)
2. ROS2 node converts joint angles (rad → deg) and applies servo calibration offsets
3. Converted angles are published to `/servo_angles` (`std_msgs/Float32MultiArray`)
4. **ESP32 (micro-ROS)** subscribes to `servo_angles` and drives 5 servos using PWM microseconds

**ROS2 Conversion Node:** [joint_reader.py](../src/my_servo_pkg/my_servo_pkg/joint_reader.py) - Converts joint states from simulation to servo angles for hardware.
---

## ROS2 Conversion Node (JointToTwoServos)
- Subscribes: `/joint_states`
- Publishes: `/servo_angles`
- Target joints:
  - `Revolute_1`, `Revolute_6`, `Revolute_2`, `Revolute_4`, `Revolute_5`
- Converts radians to degrees and applies per-joint offsets to match real servo zero positions:
  - `Revolute_1`, `Revolute_6`: `(deg - 90) * (-1)`
  - `Revolute_2`, `Revolute_4`: `deg + 90`

---
**ESP32 Firmware:** [usb_final.ino](../../arduino_ide_src/usb_final/usb_final.ino) - micro-ROS node that receives servo angles and controls 5 MG996R servos via PWM.

## micro-ROS ESP32 Servo Node
- Transport: **serial** (`MICRO_ROS_TRANSPORT_SERIAL`)
- Subscribes: `servo_angles` (`Float32MultiArray`)
- Expects 5 angles (degrees)
- Clamps each angle to `[0, 180]`
- Maps degrees to PWM pulse width:
  - `SERVO_MIN_US = 500`
  - `SERVO_MAX_US = 2500`
- Outputs PWM via `ESP32Servo` on pins:
  - S1=13, S2=14, S3=15, S4=12, S5=19

---

## Purpose
- Use simulation as a safe test environment
- Keep joint naming consistent between simulation and hardware
- Convert simulation joint angles into real servo commands
- Run the same motion pipeline for both digital and physical robot
