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
- the rotating rail-switch blade model;
- shared static content reused by different simulation modes.

Main content:

- `mfja_3rd_floor_description/models/`
- `mfja_3rd_floor_description/models/rail_switch_3pos_left/`
- `mfja_3rd_floor_description/urdf/`
- `mfja_3rd_floor_description/worlds/`

### `mfja_robot_control_config`

Shared runtime and control layer:

- robot YAML spawn configuration files;
- shared multi-robot launch logic;
- shared Gazebo-related config files;
- synchronized multi-robot control utility;
- three-position rail-switch control utility.

Main content:

- `mfja_robot_control_config/config/`
- `mfja_robot_control_config/launch/multi_robot_sim.launch.py`
- `mfja_robot_control_config/scripts/multi_robot_sync_demo.py`
- `mfja_robot_control_config/scripts/three_position_switch_demo.py`

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

The following workflow is the recommended way to use the current repository, including the new rotating rail switch and the robot command tools.

### Terminal 1: build and launch the simulation

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export GZ_PARTITION=rail_switch_demo
```

Run the complete third floor with all robots:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py robots:=all gz_partition:=rail_switch_demo
```

Equivalent umbrella-package command:

```bash
ros2 launch mfja_3rd_floor_gz full_floor.launch.py robots:=all gz_partition:=rail_switch_demo
```

Run room 315 only with all robots:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py robots:=all gz_partition:=rail_switch_demo
```

Equivalent umbrella-package command:

```bash
ros2 launch mfja_3rd_floor_gz room_315_only.launch.py robots:=all gz_partition:=rail_switch_demo
```

If you want only some robots, replace `robots:=all` with one of the following:

```bash
robots:=kuka1
robots:=staubli1
robots:=yaskawa_hc10_1
robots:=yaskawa_hc10dt_1
robots:=tiago1
robots:=kuka,tiago
robots:=1,5
robots:=none
```

### Terminal 2: source the workspace for robot commands

Open a second terminal and prepare it for robot commands:

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_PARTITION=rail_switch_demo
```

### Terminal 3: optional switch commands

If you want to move the rotating rail switch independently from the robot commands, open a third terminal:

```bash
export MFJA_WS=~/mfja_3rd_floor_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_PARTITION=rail_switch_demo
```

### Quick checks before commanding anything

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

## Ready-to-Use Robot Command Examples

The examples below are intended to be executed from Terminal 2 after the simulation is already running.

### KUKA KR6 R900 sixx

```bash
ros2 topic pub --once /kuka1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6'], points: [{positions: [0.6,-1.0,1.1,0.0,0.6,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### Staeubli TX2-60L

```bash
ros2 topic pub --once /staubli1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], points: [{positions: [0.1,0.4,-0.6,0.0,0.5,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### Yaskawa HC10

```bash
ros2 topic pub --once /yaskawa_hc10_1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'], points: [{positions: [0.2,-0.7,0.9,0.0,0.4,0.2], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### Yaskawa HC10DT

```bash
ros2 topic pub --once /yaskawa_hc10dt_1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'], points: [{positions: [-0.2,-0.5,0.8,0.0,0.5,-0.2], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### TIAGo arm and head

```bash
ros2 topic pub --once /tiago1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['torso_lift_joint','arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint','head_1_joint','head_2_joint'], points: [{positions: [0.10,0.3,-0.5,-0.4,1.0,0.2,-0.2,0.1,0.2,-0.2], time_from_start: {sec: 4, nanosec: 0}}]}"
```

### TIAGo base motion

Move TIAGo forward while rotating:

```bash
ros2 topic pub -r 20 /tiago1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}"
```

Stop TIAGo:

```bash
ros2 topic pub --once /tiago1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Ready-to-Use Rail Switch Commands

The rotating rail switch is controlled with the dedicated tool below. These commands are intended to be executed from Terminal 3.

For the full-floor world:

```bash
ros2 run mfja_robot_control_config three_position_switch_demo.py 0 --world default --partition rail_switch_demo --pos -16.73 -4.42 0.73
ros2 run mfja_robot_control_config three_position_switch_demo.py 1 --world default --partition rail_switch_demo --pos -16.73 -4.42 0.73
ros2 run mfja_robot_control_config three_position_switch_demo.py 2 --world default --partition rail_switch_demo --pos -16.73 -4.42 0.73
```

For the room-315-only world:

```bash
ros2 run mfja_robot_control_config three_position_switch_demo.py 0 --world room_315_only --partition rail_switch_demo --pos -16.73 -4.42 0.73
ros2 run mfja_robot_control_config three_position_switch_demo.py 1 --world room_315_only --partition rail_switch_demo --pos -16.73 -4.42 0.73
ros2 run mfja_robot_control_config three_position_switch_demo.py 2 --world room_315_only --partition rail_switch_demo --pos -16.73 -4.42 0.73
```

Current calibrated switch positions:

- `0 -> 2.6 rad`
- `1 -> -1.59 rad`
- `2 -> 0.50666 rad`

## Multi-Robot Synchronized Motion Examples

First, inspect the expected joint order:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py --list-joints
```

Run the default preset motion for all enabled robots that have presets:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py
```

Command a subset directly:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py \
  --goal kuka1=1.2,-1.2,1.4,0.0,0.3,0.0 \
  --goal staubli1=0.1,0.4,-0.6,0.0,0.5,0.0 \
  --trajectory-duration 4.0
```

Command all five robots explicitly:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py \
  --goal kuka1=0.8,-1.0,1.2,0.0,0.4,0.0 \
  --goal staubli1=0.0,0.3,-0.5,0.0,0.6,0.0 \
  --goal yaskawa_hc10_1=0.0,-0.6,0.8,0.0,0.5,0.0 \
  --goal yaskawa_hc10dt_1=0.0,-0.5,0.7,0.0,0.4,0.0 \
  --goal tiago1=0.10,0.3,-0.5,-0.4,1.0,0.2,-0.2,0.1,0.2,-0.2 \
  --trajectory-duration 4.0
```

The synchronized tool behavior is:

- if no `--goal` is given, it commands all enabled robots that have built-in presets;
- if one or more `--goal` arguments are given, it commands only the listed robots;
- if `--tiago-base-duration` is positive, it can also move the TIAGo base during the synchronized demo.

## Stopping the Simulation

In Terminal 1, press:

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
