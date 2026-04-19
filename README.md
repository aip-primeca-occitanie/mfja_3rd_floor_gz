# MFJA 3rd Floor Gazebo - Room 315 Kinematic Shuttle

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for
the MFJA 3rd floor, with the current focus on the Room 315 flexible rail system.

الحالة الحالية للمشروع: **kinematic-first shuttle simulation**. يعني الشاتل لا
يعتمد حاليًا على contact dynamics أو wheel physics، بل يتحرك على مسار محسوب من
ملفات CSV وشبكة explicit graph، ثم يتم تحديث pose داخل Gazebo باستخدام
`/world/<world_name>/set_pose`.

مهم: شغل الداينمك غير مستخدم حاليًا. النسخة الحالية موجهة للـ kinematic shuttle،
السويتشات، تعدد الشاتلات، وتجنب التصادم البسيط.

## Repository Layout

- `mfja_3rd_floor_description/`: models, meshes, worlds, and URDF/SDF assets.
- `mfja_robot_control_config/`: launch base, bridge config, shuttle/switch scripts, and Room 315 kinematic config.
- `mfja_room_315_bringup/`: launch entry point for Room 315 only.
- `mfja_3rd_floor_bringup/`: launch entry point for the full floor.
- `mfja_3rd_floor_gz/`: compatibility package that forwards to the bringup packages.
- `CSV/`: source rail segment CSV files for the Room 315 kinematic rail network.

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
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

There is no hard software limit on shuttle count. The practical limit is Gazebo
performance and manual collision management.

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
- Reusing the same start slot is allowed, but collision avoidance may stop the new shuttle if the slot is still occupied.

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

- Logical: `A1`, `A2`, `A3`, `A4`
- Visual right/left aliases: `A1R`, `A1L`, `A2R`, `A2L`, `A3R`, `A3L`, `A4R`, `A4L`
- Groups: `ALL`, `RIGHT`, `LEFT`

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
ros2 run mfja_robot_control_config room_315_segment_plot.py
```

Generated outputs:

- `mfja_robot_control_config/config/room_315_kinematics/segment_summary.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/validation_report.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/network_validation.png`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/room_315_segments_overview.png`

Offline kinematic core test:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle.py \
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
| `speed` | `0.25` | Shuttle speed in m/s. |
| `update_rate_hz` | `30.0` | Internal kinematic update rate. |
| `gazebo_set_pose_rate_hz` | `10.0` | Rate for Gazebo `set_pose` calls. |
| `enable_collision_avoidance` | `true` | Stop before center-distance collision. |
| `shuttle_collision_distance_m` | `0.33` | Minimum allowed center distance between shuttles. |
| `switch_command_topic` | `/room_315/switch_states` | Route and visual switch command topic. |
| `add_shuttle_command_topic` | `/room_315/shuttle/add_cmd` | Runtime shuttle add command topic. |
| `state_topic` | `/room_315/shuttle/state` | Combined shuttle state topic. |
| `pose_offset_command_topic` | `/room_315/shuttle/pose_offset_cmd` | Runtime pose calibration topic. |
| `publish_visual_switch_commands` | `true` | Also move the visible Gazebo switch models. |
| `sync_from_visual_switch_states` | `true` | Sync route logic from the latest visual switch state. |

## Troubleshooting

- If Gazebo does not open, start the room-only or full-floor launch before the shuttle node.
- If `Gazebo set_pose service is not ready yet`, check `gazebo_world_name` and `ros2 service list`.
- If full-floor services appear as `/world/default/...`, restart Gazebo after ensuring the world contains `<world name="mfja_3rd_floor">`.
- If runtime-spawned shuttles do not appear, check `/world/<world_name>/create`.
- If a shuttle stops in `WAITING`, it is likely blocked by another shuttle within `shuttle_collision_distance_m`.
- If a shuttle enters `FALLING`, the graph has no valid successor for the current switch configuration.
- If a switch moves visually but the shuttle route does not change, send commands to `/room_315/switch_states`, not directly to `/mfja/conveyor/switch_cmd`.
