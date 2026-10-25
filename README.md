# Lab 1: ROS 2, Turtlesim, Actions & Services

**Course:** Design of Mechatronics Systems, Faculty of Engineering, Alexandria University  
**ROS 2 distribution:** Jazzy (Ubuntu 24.04)

A custom ROS 2 mechatronics system that controls the `turtlesim` simulator. It implements custom interface definitions (`c_interface`), a navigation action server (`/go_to_pose`), live ROS speed parameters with a dynamic update service, automated launch files, and sequential path execution.

## Team

| Name | ID |
|------|----|
| Youssif Ahmed | 9522 |
| Yahia Haythem | 9613 |
| Adham Ramadan| 9598|

## Repository contents

This repository contains only the two source ROS 2 packages (no `build/`, `install/` or `log/` folders).

```
design_lab_assignment1/
├── README.md
├── .gitignore
├── c_interface/             # Custom interface definitions
│   ├── action/Navigate.action
│   ├── srv/Setspeed.srv
│   └── srv/Clearc.srv
└── pkg1/                    # Control nodes, launch files, and parameters
    ├── pkg1/
    │   └── control.py
    ├── launch/
    │   ├── bringup.launch.py
    │   └── square.launch.py
    ├── setup.py
    └── package.xml
```

## Package descriptions

### `c_interface`

| Interface | Type | Fields / Description |
|-----------|------|----------------------|
| `Navigate` | action | Goal: `float32 x`, `float32 y`, `float32 yaw`. Result/Feedback integration for goal-reaching. |
| `Setspeed` | srv | Request: `float32 max_linear_speed`, `float32 max_angular_speed`. Response: `bool success`, `string message`. |
| `Clearc` | srv | Request: empty. Clears canvas and resets turtle position without drawing unwanted tracking lines. |

### `pkg1`

| Node / File | Provides | Description |
|-------------|----------|-------------|
| `control` (node) | action `/go_to_pose`, services `/set_speed`, `/clear_canvas`, parameters `max_linear_speed`, `max_angular_speed` | Core controller node using a multi-threaded execution architecture (`MultiThreadedExecutor`) to handle concurrent service calls and action goals asynchronously. |
| `bringup.launch.py` | launch file | Starts the `turtlesim` simulation and the custom controller node with specified configuration parameters. |
| `square.launch.py` | launch file | Automates system bringup and sequences action goals to drive the turtle in a square/path trajectory. |

---

## Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy installed
- Turtlesim and build dependencies:

```bash
sudo apt install ros-jazzy-turtlesim ros-jazzy-ros2launch python3-colcon-common-extensions
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Youssif-Ahmed/design_lab_assignment1.git .
cd ~/ros2_ws
colcon build --packages-select c_interface pkg1
source install/setup.bash
```

> **Note:** Every new terminal session requires sourcing your workspace via `source ~/ros2_ws/install/setup.bash` before running any ROS 2 commands utilizing these packages.

---

## How to Run Each Part

### Part 1: System Bringup
Start `turtlesim` and the custom controller node together with default speed parameters:
```bash
ros2 launch pkg1 bringup.launch.py
```

### Part 2: Automated Square / Navigation Launch
Start the system bringup and automatically trigger path/square navigation execution:
```bash
ros2 launch pkg1 square.launch.py
```

---

## Commands to Try Every Requirement

Open a separate terminal, source your workspace, and use the commands below to test individual features:

### 1. Test Navigation Action (`/go_to_pose`)
Drives the turtle to target coordinates (`x=8.0, y=2.0, yaw=0.0`) using the custom action server:
```bash
ros2 action send_goal /go_to_pose c_interface/action/Navigate "{x: 8.0, y: 2.0, yaw: 0.0}"
```

### 2. Test Speed-Setting Service (`/set_speed`)
Dynamically updates the controller's maximum speed limits while active:
```bash
ros2 service call /set_speed c_interface/srv/Setspeed "{max_linear_speed: 2.0, max_angular_speed: 1.5}"
```

### 3. Test Clear / Reset Service (`/clear_canvas`)
Clears the background canvas and safely resets the turtle to its home position:
```bash
ros2 service call /reset_turtle c_interface/srv/Clearc "{}"
```

###
---
