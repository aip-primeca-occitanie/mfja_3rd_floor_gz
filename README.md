Gazebo simulation for the 3rd floor of the Maison de la Formation Jacqueline Auriol (MFJA)
================================================================================================

## Introduction

This repository provides the Gazebo Harmonic world, robot models, launch files, and control utilities used to simulate the third floor of the Maison de la Formation Jacqueline Auriol (MFJA).

The current workspace supports:

- the MFJA third-floor map using the updated `mfja_3rd_floor_v4.stl` mesh;
- multi-robot spawning from one YAML configuration file;
- independent control of KUKA, Staubli, Yaskawa HC10, Yaskawa HC10DT, and TIAGo;
- synchronized motion commands for several robots from one ROS 2 node;
- static environment assets such as robot tables, carters, rails, shuttles, and lab furniture.

## Install

```bash
mkdir -p ~/mfja_ws/src
cd ~/mfja_ws/src
git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git
cd ..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Repository structure

- `config/robots.yaml`: robot registry, poses, model selection, and activation flags
- `launch/mfja_3rdf.launch.py`: launch the MFJA world without robots
- `launch/mfja_3rdf_kuka.launch.py`: launch the world and spawn robots from YAML
- `models/`: Gazebo models for robots and static scene assets
- `scripts/multi_robot_sync_demo.py`: synchronized multi-robot command-line tool
- `worlds/mfja_3rd_floor.world`: static scene composition

## Simulation

### Walls only

```bash
cd ~/mfja_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mfja_3rd_floor_gz mfja_3rdf.launch.py world_name:=mfja_3rd_floor
```

### Full multi-robot simulation

Launch the MFJA world with robots spawned from `config/robots.yaml`:

```bash
cd ~/mfja_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mfja_3rd_floor_gz mfja_3rdf_kuka.launch.py
```

By default, the launch file spawns all robots with `enabled: true` in `config/robots.yaml`.

## Robot registry

The robot list is defined in `config/robots.yaml`.

Supported `model` values are:

- `kuka_kr6r900sixx`
- `staubli_tx2_60l`
- `yaskawa_hc10`
- `yaskawa_hc10dt`
- `tiago`

Each robot entry defines:

- `name`: ROS namespace and Gazebo entity name
- `model`: SDF and URDF pair to load
- `x_pose`, `y_pose`, `z_pose`: spawn position
- `yaw`: heading angle
- `enabled`: whether the robot is spawned by default

## Selecting robots from the command line

The launch file accepts a `robots:=...` argument to override the YAML activation set without editing the file.

It supports:

- full names, for example `kuka1,tiago1`
- short aliases, for example `kuka,tiago`
- numeric shortcuts by YAML order, for example `1,5`
- `all`

Examples:

```bash
ros2 launch mfja_3rd_floor_gz mfja_3rdf_kuka.launch.py robots:=kuka,tiago
```

```bash
ros2 launch mfja_3rd_floor_gz mfja_3rdf_kuka.launch.py robots:=1,5
```

```bash
ros2 launch mfja_3rd_floor_gz mfja_3rdf_kuka.launch.py robots:=all
```

Current shortcuts:

- `1` or `kuka` -> `kuka1`
- `2` or `staubli` -> `staubli1`
- `3` or `hc10` -> `yaskawa_hc10_1`
- `4` or `hc10dt` -> `yaskawa_hc10dt_1`
- `5` or `tiago` -> `tiago1`

## ROS topics

Each robot has its own namespace.

For all articulated robots:

- `/<name>/joint_trajectory`
- `/<name>/joint_states`
- `/<name>/joint_trajectory_progress`

For TIAGo:

- `/<name>/cmd_vel`
- `/<name>/odom`
- `/<name>/tf`

## Single-robot control

### KUKA

```bash
ros2 topic pub --once /kuka1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6'], points: [{positions: [0.0,-0.8,1.2,0.0,0.6,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### Staubli

```bash
ros2 topic pub --once /staubli1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], points: [{positions: [0.0,0.3,-0.5,0.0,0.6,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### Yaskawa HC10

```bash
ros2 topic pub --once /yaskawa_hc10_1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'], points: [{positions: [0.0,-0.6,0.8,0.0,0.5,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### Yaskawa HC10DT

```bash
ros2 topic pub --once /yaskawa_hc10dt_1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'], points: [{positions: [0.0,-0.5,0.7,0.0,0.4,0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

### TIAGo arm

```bash
ros2 topic pub --once /tiago1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['torso_lift_joint','arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint','head_1_joint','head_2_joint'], points: [{positions: [0.10,0.3,-0.5,-0.4,1.0,0.2,-0.2,0.1,0.2,-0.2], time_from_start: {sec: 4, nanosec: 0}}]}"
```

### TIAGo base

Move:

```bash
ros2 topic pub -r 20 /tiago1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}"
```

Stop:

```bash
ros2 topic pub --once /tiago1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Read odometry:

```bash
ros2 topic echo /tiago1/odom
```

## Synchronized multi-robot control

The repository includes `scripts/multi_robot_sync_demo.py` for coordinated motion from one ROS 2 node.

### Show expected joint order

```bash
cd ~/mfja_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mfja_3rd_floor_gz multi_robot_sync_demo.py --list-joints
```

### Default synchronized demo

If no `--goal` arguments are given, the tool commands all enabled robots with built-in default targets:

```bash
ros2 run mfja_3rd_floor_gz multi_robot_sync_demo.py
```

### Command only selected robots

If one or more `--goal` arguments are given, only the listed robots move.

Example with two robots:

```bash
ros2 run mfja_3rd_floor_gz multi_robot_sync_demo.py \
  --goal kuka1=1.2,-1.2,1.4,0.0,0.3,0.0 \
  --goal staubli1=0.1,0.4,-0.6,0.0,0.5,0.0 \
  --trajectory-duration 4.0
```

Example with three robots:

```bash
ros2 run mfja_3rd_floor_gz multi_robot_sync_demo.py \
  --goal kuka1=1.2,-1.2,1.4,0.0,0.3,0.0 \
  --goal staubli1=0.1,0.4,-0.6,0.0,0.5,0.0 \
  --goal yaskawa_hc10_1=0.0,-0.7,0.9,0.0,0.4,0.0 \
  --trajectory-duration 4.0
```

Example with TIAGo base motion:

```bash
ros2 run mfja_3rd_floor_gz multi_robot_sync_demo.py \
  --goal kuka1=1.2,-1.2,1.4,0.0,0.3,0.0 \
  --goal staubli1=0.1,0.4,-0.6,0.0,0.5,0.0 \
  --goal tiago1=0.10,0.2,-0.4,-0.3,0.9,0.1,-0.1,0.0,0.1,-0.1 \
  --trajectory-duration 4.0 \
  --tiago-base-linear 0.20 \
  --tiago-base-angular 0.25 \
  --tiago-base-duration 4.0
```

## Static scene assets

The world now includes:

- MFJA third-floor map from `mfja_3rd_floor_v4.stl`
- KUKA, Staubli, and Yaskawa support tables
- carters
- Montratec rails
- shuttle elements
- additional lab tables and chairs

Static scene placement is defined in `worlds/mfja_3rd_floor.world`.

## Updating positions

To move robots:

- edit `config/robots.yaml`
- update `x_pose`, `y_pose`, `z_pose`, and `yaw`
- relaunch the simulation

To move static scene assets:

- edit `worlds/mfja_3rd_floor.world`
- update the `<pose>` of the corresponding `<include>`
- relaunch the simulation

If only `robots.yaml` or `mfja_3rd_floor.world` changes, a rebuild is usually not required.
