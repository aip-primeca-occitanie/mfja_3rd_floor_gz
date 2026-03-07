Gazebo simulation for the 3rd floor of the Maison de la Formation Jacqueline Auriol (MFJA)
=================================================================================================================


# Introduction
The aim of this repository is to provide the world, models, and links towards other repositories to simulate robots at the third floor of the Maison de la Formation Jacqueline Auriol.

# Install

```
mkdir -p ~/mfja_ws/src
cd ~/mfja_ws/src
git clone https://github.com/olivier-stasse/mfja_3rd_floor_gz.git
cd ..
colcon build --symlink-install
```

# Simulation

## Walls only


```
ros2 launch mfja_3rd_floor_gz mfja_3rdf.launch.py world_name:=mfja_3rd_floor
```

## Robots (KUKA + Stäubli + Yaskawa + TIAGo)

Launch MFJA 3rd floor with robots spawning from `config/robots.yaml`:

```
ros2 launch mfja_3rd_floor_gz mfja_3rdf_kuka.launch.py
```

To add more robots, edit `config/robots.yaml` and append new entries with different `name` values.
Supported `model` values:

- `kuka_kr6r900sixx`
- `staubli_tx2_60l`
- `yaskawa_hc10`
- `yaskawa_hc10dt`
- `tiago`

Each `name` becomes an isolated namespace:

- `/<name>/joint_trajectory`
- `/<name>/joint_states`

For mobile-capable models (currently `tiago`), additional topics are available:

- `/<name>/cmd_vel`
- `/<name>/odom`
- `/<name>/tf`

Example command (robot `kuka1`):

```
ros2 topic pub --once /kuka1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6'], points: [{positions: [0.0, -0.8, 1.2, 0.0, 0.6, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

Example command (robot `staubli1`):

```
ros2 topic pub --once /staubli1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], points: [{positions: [0.0, 0.3, -0.5, 0.0, 0.6, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

Example command (robot `yaskawa_hc10_1`):

```
ros2 topic pub --once /yaskawa_hc10_1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'], points: [{positions: [0.0, -0.6, 0.8, 0.0, 0.5, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

Example command (robot `yaskawa_hc10dt_1`):

```
ros2 topic pub --once /yaskawa_hc10dt_1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t'], points: [{positions: [0.0, -0.5, 0.7, 0.0, 0.4, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}"
```

Example command (robot `tiago1`):

```
ros2 topic pub --once /tiago1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['torso_lift_joint','arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint','head_1_joint','head_2_joint'], points: [{positions: [0.10, 0.3, -0.5, -0.4, 1.0, 0.2, -0.2, 0.1, 0.2, -0.2], time_from_start: {sec: 4, nanosec: 0}}]}"
```

Move TIAGo base with velocity command:

```
ros2 topic pub -r 20 /tiago1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.35}}"
```

Stop TIAGo base:

```
ros2 topic pub --once /tiago1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Check TIAGo odometry:

```
ros2 topic echo /tiago1/odom
```
