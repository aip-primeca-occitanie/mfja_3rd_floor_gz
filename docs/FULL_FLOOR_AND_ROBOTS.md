# Full Floor and Robot Reference

## Full Floor

The full-floor world file is `mfja_3rd_floor.world`, and its internal Gazebo
world name must be:

```xml
<world name="mfja_3rd_floor">
```

That is why the shuttle node must use:

```bash
-p gazebo_world_name:=mfja_3rd_floor
```

Terminal 1 - start the full floor with the Room 315 rail stack:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

The same startup shuttle controls work on the full floor:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

Optional advanced mode - start one kinematic shuttle on the full floor after
launching with `enable_room315_kinematic_shuttles:=false`:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=mfja_3rd_floor \
  -p start_slot:=2 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

## Check Gazebo Services

The launch automatically starts ROS-Gazebo service bridges for:

- `/world/<world_name>/set_pose`
- `/world/<world_name>/create`
- `/world/<world_name>/remove`

Check them with:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service list | grep -E "set_pose|create|remove"
```

For Room 315 only, expected services:

```text
/world/room_315_only/set_pose
/world/room_315_only/create
/world/room_315_only/remove
```

For the full floor, expected services:

```text
/world/mfja_3rd_floor/set_pose
/world/mfja_3rd_floor/create
/world/mfja_3rd_floor/remove
```

If you see `/world/default/set_pose` while running the full floor, Gazebo is
using the wrong world name. Stop Gazebo, rebuild if needed, and restart the launch.

## Robot Spawning and Control

The same launch files can run the world with or without industrial robots. For
shuttle-only testing, use `robots:=none`. For robot experiments, use `robots:=all`
or select only the robots you need.

Full floor with all configured robots:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=all \
  start_paused:=false \
  gui:=true
```

Room 315 shuttles use the same `use_sim_time` clock as the rest of the full-floor
simulation, so robot motion and shuttle motion stay synchronized. If the full
scene runs below real time, the shuttle will look slower in wall-clock time
because the whole simulation is slower.

Room 315 only with all configured robots:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=all \
  start_paused:=false \
  gui:=true
```

Robot selection supports full names, short aliases, numeric YAML order, `all`,
and `none`.

Common selectors:

```text
robots:=kuka1
robots:=staubli1
robots:=yaskawa_hc10_1
robots:=yaskawa_hc10dt_1
robots:=tiago1
robots:=kuka,tiago
robots:=1,5
robots:=all
robots:=none
```

Current shortcut mapping:

| Selector | Robot |
| --- | --- |
| `1`, `kuka` | `kuka1` |
| `2`, `staubli` | `staubli1` |
| `3`, `hc10` | `yaskawa_hc10_1` |
| `4`, `hc10dt` | `yaskawa_hc10dt_1` |
| `5`, `tiago` | `tiago1` |

The full-floor launch uses:

```text
mfja_robot_control_config/config/robots.yaml
```

The room-only launch uses:

```text
mfja_robot_control_config/config/robots_room_315_only.yaml
```

### Single Industrial Robot Mode

Use this mode when you want only one industrial robot, its support table, and
the ground plane. It does not load Room 315, rails, shuttles, sensors, lab
furniture, or other robots. This mode is only for the four industrial robots:
`kuka`, `staubli`, `hc10`, and `hc10dt`.

```bash
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py \
  robot:=kuka \
  start_paused:=false \
  gui:=true
```

Supported selectors:

```text
robot:=kuka
robot:=staubli
robot:=hc10
robot:=hc10dt
```

### Robot Topic Checks

After launching the simulation with robots enabled, open a new terminal:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

List the main robot topics:

```bash
ros2 topic list | grep -E '^/(kuka1|staubli1|yaskawa_hc10_1|yaskawa_hc10dt_1|tiago1)/'
```

Check a command topic:

```bash
ros2 topic info /kuka1/joint_trajectory
```

Most fixed-base robots are controlled through:

```text
/<robot_name>/joint_trajectory
```

TIAGo additionally exposes a mobile-base command topic:

```text
/tiago1/cmd_vel
```

### KUKA KR6 R900 Sixx

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

### TIAGo Arm and Head

```bash
ros2 topic pub --once /tiago1/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['torso_lift_joint','arm_1_joint','arm_2_joint','arm_3_joint','arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint','head_1_joint','head_2_joint'], points: [{positions: [0.10,0.3,-0.5,-0.4,1.0,0.2,-0.2,0.1,0.2,-0.2], time_from_start: {sec: 4, nanosec: 0}}]}"
```

### TIAGo Base Motion

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
