
# ROS2 Gesture Control for Dual UR5 with Robotiq Grippers

This repository provides ROS2 packages to control a dual-arm UR5 robotic system with Robotiq grippers using hand gestures. It integrates MoveIt2 for motion planning and uses MediaPipe + OpenCV for gesture recognition.

This work is part of the project: Gesture-based teleoperation for mobile manipulation with dual arms.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt2
- Python 3
- OpenCV (via pip)
- MediaPipe (via pip)

## Packages in this Repository

- `ur_description` – Official URDF/XACRO description package for the UR5 robot.
- `robotiq_description` – ROS 2-compatible Robotiq gripper description (sourced from [this repo](https://github.com/JuoTungChen/ROS2_pick_and_place_UR5)).
- `dual_ur5_description` – XACRO-based model of two UR5 arms with Robotiq grippers.
- `dual_ur5_moveit_config` – MoveIt2 configuration generated using MoveIt Setup Assistant.
- `hand_gesture_control` – Python package using OpenCV and MediaPipe to recognize hand gestures and publish control commands.
- `pymoveit2` – Lightweight Python wrapper for MoveIt 2 (from [github.com/AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2)).

## Installation

After unzipping the project, you will have a complete ROS 2 workspace at:

`~/dual_ur5/`

There is no need to create a new workspace manually.

1. Verify source packages:

Make sure the following folders exist under `src/` (already included in this zip):

- `universal_robot`
- `robotiq_description`

2. Install dependencies:

```
sudo apt update
pip install opencv-python
pip install mediapipe
```

3. Build the workspace:

```
cd ~/dual_ur5_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Test MoveIt2 motion planning in RViz:

```
ros2 launch dual_ur5_moveit_config demo.launch.py
```

### 2. Launch the full system including RViz + gesture control:

```
ros2 launch hand_gesture_control gesture_control_dual_ur5.launch.py
```

This will start:

- MoveIt2 RViz demo for dual UR5 + gripper
- Gesture recognition node (`hand_gesture_publisher`)
- Gesture control node (`hand_gesture_subscriber`)

## TODO

- Final motion control integration is not yet implemented.
- Tried using `pymoveit2` and `moveit_commander`, but they are not fully compatible with ROS 2 Humble.
- Also explored MoveGroup action interface in Python, but no successful implementation yet.
- Future plan: migrate motion control to C++ or use `rclpy` action client.

## Author

SunYi
