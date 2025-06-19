# Phantom to Interbotix Teleoperation (ROS 2)

This repository enables real-time teleoperation of an **Interbotix RX200** robotic arm using a **Geomagic Touch** haptic device through ROS 2.

It includes:

* Pose-based end-effector control
* Wrist joint control via stylus orientation
* Button-based gripper operation
* Test scripts to verify driver and topic functionality

---

## Repository Overview

```
Phantom_to_Interbotix_teleop/
├── test1/                  # Test subscribers for /phantom topics
├── teleop_node.py          # Main teleoperation script
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Dependencies

* ROS 2 Humble (or compatible)
* [interbotix\_xs\_ros2](https://github.com/Interbotix/interbotix_ros_core/tree/ros2)
* [Geomagic\_Touch\_ROS2](https://github.com/Drunk-ed/Geomagic_Touch_ROS2)

---

##  Setup Instructions

### 1. Clone Repositories

```bash
mkdir -p ~/interbotix_ws1/src
cd ~/interbotix_ws1/src

# Clone this repo
git clone https://github.com/Drunk-ed/Phantom_to_Interbotix_teleop.git
```

### 2. Build

```bash
colcon build
source install/setup.bash
```

---

## Testing Geomagic Touch Drivers

To check if the drivers are working correctly and get familiar with the Geomagic device, run the test subscribers from the `test1` package:

```bash
# Pose topic test
ros2 run test1 sub_phantom_pose

# Button state test
ros2 run test1 sub_phantom_button

# Joint state test
ros2 run test1 sub_phantom_joint_state
```

Each will print data to the terminal to verify the `/phantom/pose`, `/phantom/button`, and `/phantom/joint_states` topics.

---

## Running the Teleop Node

```bash
ros2 run phantom_to_interbotix teleop_node
```

### Controls:

* Move stylus → Move robot end-effector (XYZ)
* Hold **white button** → Enter joint control mode
* In joint mode: rotate stylus → control wrist\_angle and wrist\_rotate
* Press **grey button** → Gripper toggle (close/open)

> ⚠ Make sure robot is powered on and motors are enabled via `robot_startup()`.

---

## Notes

* Teleop frequency is throttled to 50Hz for stability.
* You can edit `teleop.py` to change frame offsets or joint mappings.
* Gripper control uses built-in Interbotix grasp/release functions.

---

## References

* [Geomagic Touch ROS 2 Driver (custom)](https://github.com/Drunk-ed/Geomagic_Touch_ROS2)
* [Interbotix ROS 2 SDK](https://github.com/Interbotix/interbotix_ros_core/tree/ros2)
* [3D Systems OpenHaptics SDK](https://www.3dsystems.com/haptics-devices/openhaptics)
