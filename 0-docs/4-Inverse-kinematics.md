# Inverse Kinematics


 
## Overview
Inverse kinematics is solved in Python using **IKPy** based on the robot **URDF**.  
The IK output (joint angles) is published to **ROS2** as `sensor_msgs/JointState` and used as a command for Isaac Sim.

**Implementation:** [IK_ROS.py](../src/my_servo_pkg/my_servo_pkg/IK_ROS.py) - ROS2 node for inverse kinematics computation and joint command publishing.

<p align="center">
  <img src="https://github.com/DmitriyB51/3d-Printed-5DOF-robotic-arm-Isaac-Simulator-integration-inverse-kinematics/raw/main/media/IK.gif" width="600">
</p> 


---

## Inputs / Outputs
**Input:** target end-effector position `(X, Y, Z)` from terminal  
**Output:** joint angles for 4 revolute joints published to `/joint_command`

---

## IK Model
- IK chain loaded from URDF using `ikpy.chain.Chain.from_urdf_file(...)`
- Active joints are selected via `active_links_mask`
- Joint names used in ROS2 message:
  - `Revolute_1`, `Revolute_6`, `Revolute_2`, `Revolute_4`

---

## Solver Logic
1. Read `(X, Y, Z)`
2. Try multiple candidate orientations (fallback list)
3. Run IKPy `inverse_kinematics()` with `orientation_mode="Y"`
4. Validate solution using forward kinematics:
   - compute end-effector position from `forward_kinematics()`
   - compare with target using a position tolerance

If a valid solution is found, publish the joint angles.

---

## Reachability Check
A solution is accepted only if the computed EE position is within:
- `POSITION_TOLERANCE = 0.04` (meters)

If no orientation reaches the target within tolerance:
- the target is treated as **not reachable**

---

## ROS2 Command Publishing
- Publisher topic: `/joint_command`
- Message type: `sensor_msgs/JointState`
- Publish rate: `30 Hz`
- Each valid solution is published for ~0.5 seconds to ensure Isaac Sim receives it

---

## Notes
- Only joint indices `ik[1:5]` are used (4 joints sent to simulation)
- Angles are published in **radians** (ROS standard)
- Terminal printing converts angles to degrees for readability
