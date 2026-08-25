# MFJA 3rd Floor Gazebo Simulation

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for the MFJA 3rd floor. 

## 📖 General Overview

The simulation environment provides a comprehensive digital twin of the MFJA 3rd floor, featuring multiple work cells, industrial robotic arms (KUKA, Stäubli, Yaskawa), and mobile robots (TIAGo). 

A major focus of this repository is the **Room 315 flexible rail system**, which currently utilizes a highly reliable **kinematic shuttle simulation**. Instead of relying on complex physics interactions like wheel friction, shuttles move along arc-length paths generated from a calibrated explicit rail graph, ensuring smooth and predictable behavior for testing routing logic, multi-shuttle interactions, and switch controls.

Whether you are testing mobile robot navigation on the full floor, running pick-and-place tasks with a single robotic arm, or orchestrating a complex multi-shuttle logistics scenario in Room 315, this repository provides the necessary models and launch configurations.

The `psardin001` fork adds a two-stage HPP course:

1. [Cartesian line planning](mfja_staubli_demos/README.md) introduces the HPP
   robot, constraints, and continuous collision checking.
2. [Manipulation planning](mfja_staubli_manipulation_demos/README.md) adds a
   gripper, payload, contact graph, and the Room 315 shuttles. Instructors use
   its [short classroom runbook](mfja_staubli_manipulation_demos/docs/room315_training_runbook.md).

Ali's edited Staubli/gripper assembly and the later MFJA split gripper CAD are
shared through `mfja_3rd_floor_description`. The split meshes improve Gazebo
fidelity and collision envelopes, but do not calibrate the physical
Staubli-to-gripper transform or grasp TCP.

---

## 🛠️ Installation Guide

The default setup is a standard ROS 2 colcon underlay/overlay. HPP is built
from local sources for ROS Jazzy's Python 3.12, then MFJA and the pinned
Staubli driver stack are built on top. Docker and Nix are not part of the
runtime.

Keep one source checkout and initialize its Staubli submodule:

```bash
cd /home/psardin/devel/mfja_3rd_floor_gz
git submodule update --init --recursive
```

Install the host HPP dependencies once, then build both workspaces:

```bash
sudo apt install ros-jazzy-coal ros-jazzy-jrl-cmakemodules \
  ros-jazzy-pinocchio ros-jazzy-proxsuite

cd /home/psardin/devel/mfja_3rd_floor_gz
./mfja_staubli_demos/scripts/room315_build_hpp_underlay.sh
./mfja_staubli_demos/scripts/room315_build_overlay.sh
source /home/psardin/devel/mfja_ws/install/setup.bash
./mfja_staubli_demos/scripts/room315_check_integration.sh
```

The HPP builder reads the canonical local sources below
`/home/psardin/devel/nix-hpp/src`, including `hpp-exec`, and installs them in
`hpp_jazzy_ws`. The directory name does not make this host build depend on
Nix. After modifying `hpp-exec`, rerun `room315_build_hpp_underlay.sh`.

For normal work, every new terminal needs only:

```bash
source /home/psardin/devel/mfja_ws/install/setup.bash
```

To build against another compatible HPP installation, point the overlay
builder at its setup file:

```bash
HPP_SETUP=/path/to/hpp/environment.sh \
  ./mfja_staubli_demos/scripts/room315_build_overlay.sh
```

That environment must expose `pyhpp`, `hpp_exec`, and `rclpy` to the same
Python interpreter.

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
*   `mfja_staubli_demos/`: Stage 1 Staubli HPP arm-planning exercise.
*   `mfja_staubli_manipulation_demos/`: Stage 2 Room 315 manipulation-graph and shuttle exercise.

---

## 📚 Detailed Documentation

For a deep dive into advanced features, please refer to our dedicated documentation files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**: Includes step-by-step guides for adding shuttles dynamically, reading sensor feedback, testing industrial robots, and troubleshooting.
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Technical details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[Stage 1: Staubli Cartesian HPP Demo](mfja_staubli_demos/README.md)**: Constrained arm planning and collision checking.
*   **[Stage 2: Room 315 Staubli Manipulation Demo](mfja_staubli_manipulation_demos/README.md)**: Manipulation graph, classroom runbook, and Gazebo/real-output boundaries.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.
