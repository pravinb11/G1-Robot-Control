# Humanoid Walking Control Setup

## Setup

First source the workspace:

```bash
source ~/ros2_control_ws/install/setup.bash
```

Then launch the Gazebo simulation:

```bash
ros2 launch g1_description g1_gazebo.launch.py
```

---

## Objectives

1. Stabilize the robot.
2. Implement the Kajita paper for walking control.
