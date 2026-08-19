# MFJA 3rd Floor Gazebo Simulation

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for the MFJA 3rd floor. 

## 📖 General Overview

The simulation environment provides a comprehensive digital twin of the MFJA 3rd floor, featuring multiple work cells, industrial robotic arms (KUKA, Stäubli, Yaskawa), and mobile robots (TIAGo). 

A major focus of this repository is the **Room 315 flexible rail system**, which currently utilizes a highly reliable **kinematic shuttle simulation**. Instead of relying on complex physics interactions like wheel friction, shuttles move along arc-length paths generated from a calibrated explicit rail graph, ensuring smooth and predictable behavior for testing routing logic, multi-shuttle interactions, and switch controls.

Whether you are testing mobile robot navigation on the full floor, running pick-and-place tasks with a single robotic arm, or orchestrating a complex multi-shuttle logistics scenario in Room 315, this repository provides the necessary models and launch configurations.

The two HPP training packages are additions from the `psardin001` fork; they
are not part of the original repository's `main` branch. The edited Staubli
gripper CAD and Gazebo integration came from Ali's `main_ali` branch and are
now consumed by the HPP examples through the canonical description package.
The later MFJA split body, custom-jaw, and adapter files are preserved there as
source geometry; they improve the collision and Gazebo models but do not yet
define the physical Staubli `tool0` transform or grasp TCP.

---

## 🛠️ Installation Guide

The project requires **Ubuntu 24.04** and **ROS 2 Jazzy**. The repository acts as a meta-repository and must be built inside a colcon workspace.

### 1. Install Prerequisites
Make sure ROS 2 Jazzy is installed, along with essential build tools:
```bash
sudo apt update
sudo apt install -y build-essential cmake git ninja-build pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  ros-jazzy-desktop ros-jazzy-robot-state-publisher ros-jazzy-ros-gz

# Initialize rosdep if you haven't already
sudo rosdep init || true
rosdep update
```

### 2. Clone the Repository
Create a workspace and clone this meta-repository inside its `src/` folder:
```bash
export MFJA_WS=~/mfja_ws
mkdir -p "$MFJA_WS/src"
cd "$MFJA_WS/src"
git clone https://github.com/psardin001/mfja_3rd_floor_gz.git
```

### 3. Build and Source
Install ROS dependencies, build the workspace, and source it:
```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash

# Install dependencies defined in package.xml files
rosdep install --from-paths src/mfja_3rd_floor_gz -y --ignore-src --rosdistro jazzy

# Build the workspace
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz

# Source the installed environment
source install/setup.bash
```
In every new terminal, run:

```bash
source "${MFJA_WS:-$HOME/mfja_ws}/install/setup.bash"
```

---

## ⚡ Basic Commands & Quick Start

The repository offers multiple run modes depending on what you want to test.

### 1. Launching the Full Floor
To run the complete 3rd-floor environment with all rooms, you can launch the `full_floor.launch.py`. You can choose to load all robots or none:
```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```
*(Change `robots:=none` to `robots:=all` to spawn TIAGo, KUKA, Stäubli, and Yaskawa robots).*

### 2. Launching Room 315 (Rail Simulation)
If you only want to focus on the flexible rail system and shuttles in Room 315:
```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

### 3. Launching a Single Industrial Robot
For isolated testing of a specific robotic arm (e.g., KUKA) without the rest of the floor:
```bash
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py \
  robot:=kuka \
  gui:=true
```
*(Other options for `robot` include `staubli`, `hc10`, and `hc10dt`).*

### 4. Basic Shuttle Control (Room 315)
If you launched the Room 315 shuttles, you can control them via ROS topics:

**Turn ON a shuttle:**
```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON'}"
```

**Control rail switches (e.g., switch all to interior):**
```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

---

## 📂 Repository Layout

*   `mfja_3rd_floor_description/`: Gazebo worlds, models, meshes, and URDF/SDF assets.
*   `mfja_rail_interfaces/`: Custom ROS 2 interfaces for commands, states, and sensors.
*   `mfja_robot_control_config/`: Shuttle/switch scripts, bridge configurations, and rail kinematic settings.
*   `mfja_3rd_floor_bringup/`: Centralized launch entry points for the full floor, Room 315, and single robot setups.
*   `mfja_staubli_demos/`: Introductory Staubli HPP arm-planning exercise.
*   `mfja_staubli_manipulation_demos/`: Advanced Room 315 manipulation-graph and shuttle exercise.

---

## 📚 Detailed Documentation

For a deep dive into advanced features, please refer to our dedicated documentation files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**: Includes step-by-step guides for adding shuttles dynamically, reading sensor feedback, testing industrial robots, and troubleshooting.
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Technical details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[Room 315 Staubli Manipulation Demo](mfja_staubli_manipulation_demos/README.md)**: New-PC setup, classroom runbook, HPP planning checks, and Gazebo/real-output boundaries.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.
