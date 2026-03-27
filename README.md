Gazebo simulation meta-repository for the 3rd floor of the Maison de la Formation Jacqueline Auriol (MFJA)
===========================================================================================================

## Overview

This repository has been refactored from a single large ROS 2 package into a meta-repository with five ROS 2 packages inside the same Git repository.

The goal of the refactor is to separate:

- shared environment and robot description assets;
- shared robot runtime and control configuration;
- full third-floor bringup;
- room-315-only bringup.

An additional umbrella package named `mfja_3rd_floor_gz` is also provided inside the repository so the package layout includes one subdirectory with the same name as the Git repository itself. This package mainly exists for compatibility and convenience, while the functional responsibilities stay split across the dedicated packages below.

The repository root is no longer a ROS 2 package by itself. The ROS 2 packages are the subdirectories listed below.

## Packages

### `mfja_3rd_floor_gz`

Umbrella and compatibility package:

- gives the repository a subpackage with the same name as the parent Git directory;
- forwards launch entry points to the dedicated bringup packages;
- keeps simple compatibility commands available.

Main content:

- `mfja_3rd_floor_gz/launch/full_floor.launch.py`
- `mfja_3rd_floor_gz/launch/room_315_only.launch.py`

### `mfja_3rd_floor_description`

Shared simulation assets:

- Gazebo models;
- meshes and STL files;
- URDF files;
- world files;
- room models such as room 315;
- shared static content reused by different simulation modes.

Main content:

- `mfja_3rd_floor_description/models/`
- `mfja_3rd_floor_description/urdf/`
- `mfja_3rd_floor_description/worlds/`

### `mfja_robot_control_config`

Shared runtime and control layer:

- robot YAML spawn configuration files;
- shared multi-robot launch logic;
- shared Gazebo-related config files;
- synchronized multi-robot control utility.

Main content:

- `mfja_robot_control_config/config/`
- `mfja_robot_control_config/launch/multi_robot_sim.launch.py`
- `mfja_robot_control_config/scripts/multi_robot_sync_demo.py`

### `mfja_3rd_floor_bringup`

Bringup package for the complete third-floor simulation.

Main content:

- `mfja_3rd_floor_bringup/launch/full_floor.launch.py`

### `mfja_room_315_bringup`

Bringup package for the isolated room-315-only simulation.

Main content:

- `mfja_room_315_bringup/launch/room_315_only.launch.py`

## Build

The examples below use a project-specific workspace variable to keep the setup easy to copy and unlikely to conflict with an existing generic workspace:

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
mkdir -p "$MFJA_WS/src"
cd "$MFJA_WS/src"
git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## If You Are Updating From The Old Single-Package Layout

If your workspace was built before the refactor, your `build/`, `install/`, or `log/` directories may still contain artifacts from the old single-package layout.

The safest update procedure is to rebuild from a clean workspace state:

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
cd "$MFJA_WS"
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

This is recommended because the new meta-repository now legitimately contains a package named `mfja_3rd_floor_gz`, so removing only `build/mfja_3rd_floor_gz` and `install/mfja_3rd_floor_gz` is no longer a reliable way to distinguish old artifacts from the new umbrella package.

You can check that the new packages are visible with:

```bash
ros2 pkg list | grep mfja
```

Expected packages:

- `mfja_3rd_floor_gz`
- `mfja_3rd_floor_description`
- `mfja_robot_control_config`
- `mfja_3rd_floor_bringup`
- `mfja_room_315_bringup`

## Run Modes

### Full third floor

Preferred entry point:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py
```

Umbrella-package compatibility entry point:

```bash
ros2 launch mfja_3rd_floor_gz full_floor.launch.py
```

This mode loads:

- the complete MFJA third-floor world;
- all shared environment assets referenced by the full world;
- robots defined in `mfja_robot_control_config/config/robots.yaml`.

Optional robot selection override:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py robots:=kuka,tiago
```

World-only mode through the unified entry point:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py robots:=none
```

Equivalent umbrella-package command:

```bash
ros2 launch mfja_3rd_floor_gz full_floor.launch.py robots:=none
```

### Room 315 only

Preferred entry point:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py
```

Umbrella-package compatibility entry point:

```bash
ros2 launch mfja_3rd_floor_gz room_315_only.launch.py
```

This mode loads:

- the dedicated room 315 world;
- the dedicated room 315 mesh and shared models referenced by that world;
- robots defined in `mfja_robot_control_config/config/robots_room_315_only.yaml`.

It is intended to stay lighter and easier to tune for robot and control experiments than the full-floor simulation.

Optional robot selection override:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py robots:=staubli
```

More examples:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py robots:=1,5
```

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py robots:=all
```

## Step-by-Step Usage

The following workflow is the recommended way to run the project from a clean terminal session.

### 1. Open Terminal 1 and build the workspace

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If the repository was recently refactored and your workspace still contains old artifacts, use the clean rebuild procedure shown earlier in this README before continuing.

### 2. Choose one simulation mode

#### Option A: run the full third floor

Preferred command:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py
```

Equivalent umbrella-package command:

```bash
ros2 launch mfja_3rd_floor_gz full_floor.launch.py
```

#### Option B: run room 315 only

Preferred command:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py
```

Equivalent umbrella-package command:

```bash
ros2 launch mfja_3rd_floor_gz room_315_only.launch.py
```

### 3. Optional: start only selected robots

You can add `robots:=...` to either launch mode.

Examples:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py robots:=kuka,tiago
```

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py robots:=staubli
```

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py robots:=1,5
```

### 4. Open Terminal 2 for commands and checks

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 5. Verify that the robots are ready

List MFJA-related packages:

```bash
ros2 pkg list | grep mfja
```

List the main robot topics:

```bash
ros2 topic list | grep -E '^/(kuka1|staubli1|yaskawa_hc10_1|yaskawa_hc10dt_1|tiago1)/'
```

Check one command topic:

```bash
ros2 topic info /kuka1/joint_trajectory
```

### 6. Move one robot only

Example: move only `kuka1`

```bash
ros2 topic pub --once /kuka1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6'], points: [{positions: [0.0,-0.8,1.2,0.0,0.6,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

Example: move only `staubli1`

```bash
ros2 topic pub --once /staubli1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], points: [{positions: [0.0,0.3,-0.5,0.0,0.6,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### 7. Move several robots together from one command

First, inspect the expected joint order:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py --list-joints
```

Then command a subset directly:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py \
  --goal kuka1=1.2,-1.2,1.4,0.0,0.3,0.0 \
  --goal staubli1=0.1,0.4,-0.6,0.0,0.5,0.0 \
  --trajectory-duration 4.0
```

The synchronized tool behavior is:

- if no `--goal` is given, it commands all enabled robots that have built-in presets;
- if one or more `--goal` arguments are given, it commands only the listed robots.

### 8. Control TIAGo base motion

Move the TIAGo base continuously:

```bash
ros2 topic pub -r 20 /tiago1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}"
```

Stop TIAGo:

```bash
ros2 topic pub --once /tiago1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 9. Stop the simulation

In the terminal where Gazebo is running, press:

```bash
Ctrl+C
```

Then relaunch with another mode or another robot selection if needed.

## Robot Selection

The shared multi-robot launch logic supports:

- full robot names, for example `kuka1,tiago1`;
- short aliases, for example `kuka,tiago`;
- numeric selection by YAML order, for example `1,5`;
- `all`.

Current shortcuts:

- `1` or `kuka` -> `kuka1`
- `2` or `staubli` -> `staubli1`
- `3` or `hc10` -> `yaskawa_hc10_1`
- `4` or `hc10dt` -> `yaskawa_hc10dt_1`
- `5` or `tiago` -> `tiago1`

If `robots:=...` is omitted, the enabled robots from the selected YAML file are used.

## Shared Robot Control Utility

The synchronized command-line control utility is installed from `mfja_robot_control_config`.

Show expected joints:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py --list-joints
```

Command specific robots:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py \
  --goal kuka1=1.2,-1.2,1.4,0.0,0.3,0.0 \
  --goal staubli1=0.1,0.4,-0.6,0.0,0.5,0.0 \
  --trajectory-duration 4.0
```

## Recommended Usage

For new work, the recommended launch commands are:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py
ros2 launch mfja_room_315_bringup room_315_only.launch.py
```

The umbrella package remains available for convenience and compatibility:

```bash
ros2 launch mfja_3rd_floor_gz full_floor.launch.py
ros2 launch mfja_3rd_floor_gz room_315_only.launch.py
```

## Design Notes

- Shared robot models, URDF files, worlds, and meshes are stored only once in `mfja_3rd_floor_description`.
- Full-floor and room-315-only modes reuse the same shared multi-robot launch logic from `mfja_robot_control_config`.
- This separation reduces future Git conflicts between work on the full third-floor setup and work focused only on room 315.
- The repository remains a single Git repository, but it is now organized as multiple ROS 2 packages with clearer responsibilities.
