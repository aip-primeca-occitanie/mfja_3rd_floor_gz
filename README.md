# MFJA 3rd Floor Gazebo - Room 315 Kinematic Shuttle

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for
the MFJA 3rd floor, with the current focus on the Room 315 flexible rail system.

The current project state is a **kinematic-first shuttle simulation**. The
shuttle does not currently use contact dynamics or wheel physics. Instead, it
moves along an arc-length path backend generated from calibrated CSV rail
geometry and an explicit rail graph, then updates the Gazebo model pose through
`/world/<world_name>/set_pose`.

Dynamic shuttle work is intentionally not used in the current main version. The
current version focuses on kinematic shuttle motion, switch routing,
independent stoppers before switches, multi-shuttle operation, runtime spawning,
explicit shuttle enable/disable commands, simple collision avoidance, and
continuous cubic Hermite path sampling for smoother generalized motion.

## Repository Layout

This git repository is intentionally organized as a **meta-repository**. The
repository root is not a ROS 2 package. Do not add root-level `package.xml`,
`CMakeLists.txt`, `launch/`, `config/`, `models/`, `worlds/`, or `CSV/`
directories. Package-specific files must live inside the package that owns them.

- `mfja_3rd_floor_description/`: models, meshes, worlds, and URDF/SDF assets.
- `mfja_robot_control_config/`: launch base, bridge config, shuttle/switch scripts, and Room 315 kinematic config.
- `mfja_room_315_bringup/`: launch entry point for Room 315 only.
- `mfja_3rd_floor_bringup/`: launch entry point for the full floor.
- `mfja_3rd_floor_gz/`: compatibility package that forwards to the bringup packages.
- `mfja_robot_control_config/config/room_315_kinematics/raw_segments/`: source rail segment CSV files for the Room 315 kinematic rail network.

The detailed Room 315 kinematic artifacts are also documented here:

```text
mfja_robot_control_config/config/room_315_kinematics/README.md
```

## Build

Use this build command from the workspace root. The explicit `--base-paths` is
important because this repository contains several ROS packages inside one git
repository.

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build --base-paths \
  src/mfja_3rd_floor_gz/mfja_robot_control_config \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_description \
  src/mfja_3rd_floor_gz/mfja_room_315_bringup \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_bringup \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_gz \
  --packages-select \
  mfja_robot_control_config \
  mfja_3rd_floor_description \
  mfja_room_315_bringup \
  mfja_3rd_floor_bringup \
  mfja_3rd_floor_gz \
  --symlink-install

source install/setup.bash
```

If you only edit README files, no rebuild is required.

## Room 315 Continuous Path Backend

The Room 315 rail geometry still starts from measured CSV segment files. The
runtime no longer has to treat those CSV rows as isolated pose steps. It can
sample each segment through a path backend:

- `cubic_hermite`: recommended default. It builds a continuous arc-length
  parameterized path from the CSV points and tangents.
- `polyline`: direct CSV polyline interpolation. Keep this for debugging and
  comparing against the measured source data.

Normal demos should use:

```bash
-p path_backend:=cubic_hermite
```

To compare against the direct CSV interpolation, use:

```bash
-p path_backend:=polyline
```

Validate the continuous path backend after changing rail geometry:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_continuous_path_validator.py
```

Expected result:

```text
Status: PASS (0 warnings)
```

Generated outputs:

- `mfja_robot_control_config/config/room_315_kinematics/continuous_path_report.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/continuous_path_validation.png`

## Room 315 Only

Terminal 1 - start Gazebo:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_room_315_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```

Terminal 2 - start one kinematic shuttle:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=2 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

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

Terminal 1 - start the full floor:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```

Terminal 2 - start one kinematic shuttle on the full floor:

```bash
cd /home/tiago/ALI_ros2_ws
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

Check them with:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service list | grep -E "set_pose|create"
```

For Room 315 only, expected services:

```text
/world/room_315_only/set_pose
/world/room_315_only/create
```

For the full floor, expected services:

```text
/world/mfja_3rd_floor/set_pose
/world/mfja_3rd_floor/create
```

If you see `/world/default/set_pose` while running the full floor, Gazebo is
using an old world name. Stop Gazebo, rebuild if needed, and restart the launch.

## Robot Spawning and Control

The same launch files can run the world with or without industrial robots. For
shuttle-only testing, use `robots:=none`. For robot experiments, use `robots:=all`
or select only the robots you need.

Full floor with all configured robots:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=all \
  start_paused:=false \
  gui:=true
```

Room 315 only with all configured robots:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_room_315_bringup room_315_only.launch.py \
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

### Robot Topic Checks

After launching the simulation with robots enabled, open a new terminal:

```bash
cd /home/tiago/ALI_ros2_ws
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

### Multi-Robot Synchronized Motion

Inspect the expected joint order for enabled robots:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py --list-joints
```

Run the default preset motion for all enabled robots with built-in presets:

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

- If no `--goal` is provided, it commands all enabled robots that have built-in presets.
- If one or more `--goal` arguments are provided, it commands only the listed robots.
- If `--tiago-base-duration` is positive, it can also publish TIAGo base `cmd_vel` during the synchronized demo.

Example with TIAGo base motion:

```bash
ros2 run mfja_robot_control_config multi_robot_sync_demo.py \
  --goal tiago1=0.10,0.3,-0.5,-0.4,1.0,0.2,-0.2,0.1,0.2,-0.2 \
  --tiago-base-linear 0.15 \
  --tiago-base-angular 0.20 \
  --tiago-base-duration 3.0
```

## Allowed Shuttle Start Slots

Only these four start slots are allowed:

| Slot | Gazebo pose |
| --- | --- |
| `1` | `-14.95 -3.86 0.84 0 0 3.14` |
| `2` | `-15.43 -3.86 0.84 0 0 3.14` |
| `3` | `-15.24 -5.54 0.84 0 0 0` |
| `4` | `-14.77 -5.54 0.84 0 0 0` |

Example:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=3 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

For the full floor, change only:

```bash
-p gazebo_world_name:=mfja_3rd_floor
```

## Start Multiple Shuttles

Start four shuttles from the four start slots:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p shuttle_count:=4 \
  -p start_slots:=1,2,3,4 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

Full-floor version:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=mfja_3rd_floor \
  -p shuttle_count:=4 \
  -p start_slots:=1,2,3,4 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

There is no hard software limit on shuttle count during runtime. At startup,
each initial shuttle must use a unique, unoccupied start slot. Additional
shuttles can be added later after a start slot becomes free.

## Add Shuttles During Runtime

After Gazebo and the shuttle node are running, publish to:

```text
/room_315/shuttle/add_cmd
```

Add a shuttle at slot 3:

```bash
ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: 'slot=3'}"
```

Add a shuttle with a specific entity name and speed:

```bash
ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: 'entity=room315_shuttle_5 slot=3 speed=0.2'}"
```

Short form:

```bash
ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: '4'}"
```

Notes:

- `room315_shuttle_1` to `room315_shuttle_4` are preloaded in the worlds.
- Shuttles beyond the preloaded count are spawned through `/world/<world_name>/create`.
- If the requested start slot is occupied, the node rejects the add command and does not create a new shuttle.
- A slot is considered occupied when an existing shuttle is within `start_slot_occupancy_radius_m` of that start pose.

## Shuttle ON/OFF Control

Each shuttle can be independently enabled or disabled through:

```text
/room_315/shuttle/control_cmd
```

Disabling a shuttle keeps the model in place and stops its kinematic motion.
Enabling it again lets it continue from its current segment and arc-length
position.

Turn one shuttle off:

```bash
ros2 topic pub --once /room_315/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_shuttle_2=OFF'}"
```

Turn it back on:

```bash
ros2 topic pub --once /room_315/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_shuttle_2=ON'}"
```

Control all shuttles at once:

```bash
ros2 topic pub --once /room_315/shuttle/control_cmd std_msgs/msg/String "{data: 'ALL=OFF'}"
ros2 topic pub --once /room_315/shuttle/control_cmd std_msgs/msg/String "{data: 'ALL=ON'}"
```

JSON form:

```bash
ros2 topic pub --once /room_315/shuttle/control_cmd std_msgs/msg/String "{data: '{\"entity\":\"room315_shuttle_3\",\"enabled\":\"OFF\"}'}"
```

## Stopper Control and Sensor Workflow

Stopper logic is independent from switch logic. A stopper is a binary primitive:

- `0`, `OPEN`, `RELEASE`, `OFF`: the stopper is open and shuttles may pass.
- `1`, `STOP`, `CLOSED`, `ON`: the stopper stops a shuttle before the switch.

The public stopper labels now follow the real switch labels used in the cell,
while the internal routing stays remapped under the hood so the motion does not
change:

| Stopper | Before switch | Stop segments |
| --- | --- | --- |
| `A1` | `A1` | `A23` |
| `A2` | `A2` | `A34E`, `A34I` |
| `A3` | `A3` | `A14` |
| `A4` | `A4` | `A12E`, `A12I` |

Stopper commands use:

```text
/room_315/stopper_states
```

Close one stopper:

```bash
ros2 topic pub --once /room_315/stopper_states std_msgs/msg/String "{data: 'A1=1'}"
```

Open one stopper:

```bash
ros2 topic pub --once /room_315/stopper_states std_msgs/msg/String "{data: 'A1=0'}"
```

Close or open all stoppers:

```bash
ros2 topic pub --once /room_315/stopper_states std_msgs/msg/String "{data: 'ALL=1'}"
ros2 topic pub --once /room_315/stopper_states std_msgs/msg/String "{data: 'ALL=0'}"
```

The educational sensor workflow is exposed on:

```text
/room_315/sensors/switch_approach
```

Echo the sensor events:

```bash
ros2 topic echo /room_315/sensors/switch_approach
```

Each message is JSON text. The `sensors` field is a list because multiple
shuttles can trigger approach sensors at the same time. To know which shuttle an
event belongs to, read the `entity_name` field in that event.

Example single-shuttle event:

```json
{
  "sensors": [
    {
      "before_switch": "A1",
      "distance_m": 0.247,
      "entity_name": "room315_shuttle_4",
      "segment": "A23",
      "sensor": "A1_APPROACH",
      "stopper": "A1",
      "stopper_state": "0",
      "workflow": "sensor -> stop shuttle -> move switch -> unstop shuttle"
    }
  ],
  "stopper_states": {
    "A1": "0",
    "A2": "0",
    "A3": "0",
    "A4": "0"
  }
}
```

This means `room315_shuttle_4` is on segment `A23`, approaching switch `A1`,
and is about `0.247 m` before the A1 stop point. If the printed distance keeps
decreasing, the shuttle is moving toward that stopper.

Example with two simultaneous sensor events:

```json
{
  "sensors": [
    {
      "before_switch": "A3",
      "distance_m": 0.18,
      "entity_name": "room315_shuttle_2",
      "segment": "A14",
      "stopper": "A3"
    },
    {
      "before_switch": "A1",
      "distance_m": 0.24,
      "entity_name": "room315_shuttle_4",
      "segment": "A23",
      "stopper": "A1"
    }
  ]
}
```

In that case, handle each event by its own `entity_name` and matching
`stopper`.

The intended manual workflow is:

1. Watch the sensor event for a shuttle approaching a switch.
2. Close the matching stopper, for example `A1=1`.
3. Move the switch, for example `A1=S`.
4. Open the stopper again, for example `A1=0`.

Example sequence:

```bash
ros2 topic echo /room_315/sensors/switch_approach
ros2 topic pub --once /room_315/stopper_states std_msgs/msg/String "{data: 'A1=1'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=S'}"
ros2 topic pub --once /room_315/stopper_states std_msgs/msg/String "{data: 'A1=0'}"
```

## Collision Avoidance

Collision avoidance is enabled by default:

```text
enable_collision_avoidance=true
shuttle_collision_distance_m=0.33
```

The shuttle STL length is approximately `0.343 m`, so the default `0.33 m`
distance is used as a practical center-distance stop threshold. If a moving
shuttle gets too close to another shuttle, it enters `WAITING` and stops at the
last safe pose instead of merging through the other shuttle.

You usually do not need to pass these parameters, but they can be overridden:

```bash
-p enable_collision_avoidance:=true \
-p shuttle_collision_distance_m:=0.33
```

## Switch Control

Send switch commands to:

```text
/room_315/switch_states
```

Supported states:

- `G`, `GRAND`, `GRAND_BOUCLE`, `BIG`, `LARGE`: big loop.
- `S`, `PETIT`, `PETIT_BOUCLE`, `SMALL`: small loop.

Switch selectors:

- Public station labels: `A1`, `A2`, `A3`, `A4`
- Visual right/left aliases: `A1R`, `A1L`, `A2R`, `A2L`, `A3R`, `A3L`, `A4R`, `A4L`
- Groups: `ALL`, `RIGHT`, `LEFT`

All public switch labels now follow the real switch stickers. The routing layer
is remapped internally so the shuttle keeps the same motion as before:

- Right rail: `A1R -> former A3`, `A2R -> former A4`, `A3R -> former A1`, `A4R -> former A2`
- Left rail: `A1L -> former A4`, `A2L -> former A3`, `A3L -> former A2`, `A4L -> former A1`
- Public route/stopper labels: `A1 -> former A3`, `A2 -> former A4`, `A3 -> former A1`, `A4 -> former A2`

Set all switches to the big loop:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'ALL=G'}"
```

Set all switches to the small loop:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'ALL=S'}"
```

Switch one station:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A2=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A2=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A3=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A3=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A4=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A4=S'}"
```

Use right/left visual aliases:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1R=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1R=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1L=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1L=S'}"
```

`A?R` updates the routed line and Gazebo together. `A?L` only moves the left
visual switch in Gazebo, because the current shuttle routing still follows the
right rail set.

Send multiple updates in one command:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=S A2=G A3=S A4=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1R=S A2R=S A3R=G A4R=G'}"
```

Always prefer `/room_315/switch_states`. It updates the route logic and also
publishes visual switch commands to Gazebo.

The node also listens to:

```text
/mfja/conveyor/switch_states
```

This lets a restarted shuttle node sync from the latest visual switch state, if
the visual switch controller is still running.

## Runtime Pose Calibration

The current CSV files are already calibrated, so offset and scale should normally
be `1.0` and `0.0`. For runtime testing, publish to:

```text
/room_315/shuttle/pose_offset_cmd
```

Examples:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'dx=0.01'}"
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'dy=-0.02'}"
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'x=0.0 y=0.0 z=0.0'}"
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'sx=1.0 sy=1.0'}"
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'reset'}"
```

## State and Debug Topics

State topic:

```bash
ros2 topic echo /room_315/shuttle/state --once
```

Approach sensor events:

```bash
ros2 topic echo /room_315/sensors/switch_approach --once
```

First shuttle pose:

```bash
ros2 topic echo /room_315/shuttle/pose_cmd --once
```

Specific shuttle pose:

```bash
ros2 topic echo /room_315/shuttles/room315_shuttle_3/pose_cmd --once
```

Visual switch state:

```bash
ros2 topic echo /mfja/conveyor/switch_states --once
```

Room 315 topics:

```bash
ros2 topic list | grep room_315
```

## CSV Preprocessing and Validation

Run these after changing CSV geometry or `rail_network.yaml`:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_csv_preprocessor.py
ros2 run mfja_robot_control_config room_315_network_validator.py
ros2 run mfja_robot_control_config room_315_continuous_path_validator.py
ros2 run mfja_robot_control_config room_315_segment_plot.py
```

Generated outputs:

- `mfja_robot_control_config/config/room_315_kinematics/raw_segments/`
- `mfja_robot_control_config/config/room_315_kinematics/segment_summary.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/validation_report.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/continuous_path_report.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/network_validation.png`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/continuous_path_validation.png`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/room_315_segments_overview.png`

Offline kinematic core test:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle.py \
  --path-backend cubic_hermite \
  --switch A1=G \
  --switch A2=G \
  --switch A3=G \
  --switch A4=G
```

## Important Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gazebo_world_name` | `room_315_only` | Gazebo world used to derive `/world/<name>/set_pose` and `/world/<name>/create`. |
| `enable_gazebo_set_pose` | `false` | If `true`, the node moves Gazebo shuttle models. |
| `enable_gazebo_spawn` | `true` | Allows runtime spawning of shuttles beyond the preloaded models. |
| `start_slot` | `2` | Start slot for a single shuttle. |
| `start_slots` | empty | Comma-separated start slots for multiple shuttles, for example `1,2,3,4`. |
| `shuttle_count` | `1` | Initial shuttle count. |
| `gazebo_entity_name` | `room315_shuttle_1` | Gazebo entity name for a single shuttle. |
| `gazebo_entity_names` | empty | Comma-separated names for multiple shuttles. |
| `preloaded_shuttle_count` | `4` | Number of shuttle models already present in the world. |
| `reject_occupied_start_slots` | `true` | Reject runtime add commands when the requested start slot is occupied. |
| `start_slot_occupancy_radius_m` | `0.33` | Radius used to decide if a start slot is occupied. |
| `speed` | `0.25` | Shuttle speed in m/s. |
| `update_rate_hz` | `30.0` | Internal kinematic update rate. |
| `gazebo_set_pose_rate_hz` | `10.0` | Rate for Gazebo `set_pose` calls. |
| `path_backend` | `cubic_hermite` | Geometry sampler used by the shuttle core. Use `cubic_hermite` for normal continuous motion or `polyline` for direct CSV comparison. |
| `arc_length_samples_per_edge` | `16` | Sub-samples per CSV edge used to parameterize the continuous path by arc length. |
| `enable_collision_avoidance` | `true` | Stop before center-distance collision. |
| `shuttle_collision_distance_m` | `0.33` | Minimum allowed center distance between shuttles. |
| `switch_command_topic` | `/room_315/switch_states` | Route and visual switch command topic. |
| `stopper_command_topic` | `/room_315/stopper_states` | Independent binary stopper command topic. |
| `sensor_state_topic` | `/room_315/sensors/switch_approach` | Sensor-style approach event topic for switch workflow testing. |
| `add_shuttle_command_topic` | `/room_315/shuttle/add_cmd` | Runtime shuttle add command topic. |
| `shuttle_control_command_topic` | `/room_315/shuttle/control_cmd` | Per-shuttle ON/OFF control topic. |
| `state_topic` | `/room_315/shuttle/state` | Combined shuttle state topic. |
| `pose_offset_command_topic` | `/room_315/shuttle/pose_offset_cmd` | Runtime pose calibration topic. |
| `publish_visual_switch_commands` | `true` | Also move the visible Gazebo switch models. |
| `sync_from_visual_switch_states` | `true` | Sync route logic from the latest visual switch state. |

## Troubleshooting

- If Gazebo does not open, start the room-only or full-floor launch before the shuttle node.
- If `Gazebo set_pose service is not ready yet`, check `gazebo_world_name` and `ros2 service list`.
- If full-floor services appear as `/world/default/...`, restart Gazebo after ensuring the world contains `<world name="mfja_3rd_floor">`.
- If runtime-spawned shuttles do not appear, check `/world/<world_name>/create`.
- If an add command is rejected, check whether another shuttle is still inside `start_slot_occupancy_radius_m` of that slot.
- If the rail path was edited, run `room_315_csv_preprocessor.py`, `room_315_network_validator.py`, and `room_315_continuous_path_validator.py` before testing in Gazebo.
- If you need to compare the continuous path against the measured CSV path, rerun the shuttle node with `-p path_backend:=polyline`.
- If a shuttle stops with `stopped_by` set to a stopper name, open that stopper with `/room_315/stopper_states`.
- If a shuttle stops in `WAITING`, it is likely blocked by another shuttle within `shuttle_collision_distance_m`.
- If a shuttle enters `FALLING`, the graph has no valid successor for the current switch configuration.
- If a switch moves visually but the shuttle route does not change, send commands to `/room_315/switch_states`, not directly to `/mfja/conveyor/switch_cmd`.
