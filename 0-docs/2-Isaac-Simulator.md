# Isaac Sim Integration

## Overview
The robot is simulated in **NVIDIA Isaac Sim** using a URDF model generated from the Fusion 360 design.

<p align="center">
  <img src="../images/sim.jpg" width="600">
</p> 

---

## URDF
- Matches mechanical structure
- Correct joint axes and limits
- Used as the simulation base model

---

## Control
- Isaac Sim **Action Graph**
- ROS2 communication

<p align="center">
  <img src="../images/graph.jpg" width="600">
</p>

**Data flow:**
1. Target joint values received from ROS2 / micro-ROS
2. Joint positions applied in simulation
3. Current joint states published to `/joint_states`






## Purpose
- Validate inverse kinematics
- Test motion before hardware
- Digital twin verification
