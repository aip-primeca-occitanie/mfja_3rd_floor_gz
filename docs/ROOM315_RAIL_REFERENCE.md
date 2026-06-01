# Room 315 Rail Reference

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

## Room 315 Only

This launch starts Gazebo and, by default, also starts the Room 315 right and
left rail nodes. That means the YAML devices, visual markers, typed
command/state topics, and `/room_315/rails/{right,left}/...` namespaces are
available from the same launch. Initial shuttle count defaults to `0`, so no
shuttle moves until you add one or request startup shuttles with launch
arguments.

Terminal 1 - start Room 315 with the rail stack:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

Start Room 315 with one right shuttle and one left shuttle visible but waiting
for your `ON` command:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

Start Room 315 with one right shuttle and one left shuttle moving immediately:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=true
```

To start Gazebo without the rail shuttle nodes:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=false
```

Optional advanced mode - start one right-rail kinematic shuttle directly after
launching Gazebo with `enable_room315_kinematic_shuttles:=false`:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p rail_side:=right \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=2 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

Optional advanced mode - start one left-rail kinematic shuttle directly:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p rail_side:=left \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=1 \
  -p path_backend:=cubic_hermite \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

Optional advanced mode - start the ready-made dual launch separately:

Right only:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_robot_control_config room_315_dual_kinematic_shuttles.launch.py \
  gazebo_world_name:=room_315_only \
  enable_right:=true \
  enable_left:=false \
  right_start_slot:=2 \
  speed:=0.2
```

Left only:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_robot_control_config room_315_dual_kinematic_shuttles.launch.py \
  gazebo_world_name:=room_315_only \
  enable_right:=false \
  enable_left:=true \
  left_start_slot:=1 \
  speed:=0.2
```

Both rails together:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_robot_control_config room_315_dual_kinematic_shuttles.launch.py \
  gazebo_world_name:=room_315_only \
  enable_right:=true \
  enable_left:=true \
  right_start_slot:=2 \
  left_start_slot:=1 \
  speed:=0.2
```

## Right and Left Rail Quick Commands

Open one extra terminal for commands:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

In the integrated room-only and full-floor launches, the rail nodes start by
default but `room315_right_shuttle_count:=0` and
`room315_left_shuttle_count:=0`, so no shuttle is created at startup. Add a
shuttle with `/room_315/rails/{right,left}/shuttles/add` while Gazebo is
already running. Use `start_enabled: false` when you want the shuttle to appear
and wait for a later `ON` command, or `start_enabled: true` when you want it to
move immediately.
If you start initial shuttles with a nonzero count, use
`room315_shuttles_start_enabled:=false` to make them wait for `ON`, or
`room315_shuttles_start_enabled:=true` to make them move immediately.
Initial shuttles are always deployed visibly in Gazebo; the startup choice is
only waiting versus moving.

Default first entity names:

- Right rail: `room315_right_shuttle_1`
- Left rail: `room315_left_shuttle_1`

Main per-rail APIs use `mfja_rail_interfaces` messages/services and the Phase 5
rail subsystem namespace:

| Purpose | Right rail | Interface |
| --- | --- | --- |
| Shuttle state | `/room_315/rails/right/shuttles/state` | `mfja_rail_interfaces/msg/ShuttleState` |
| Shuttle control | `/room_315/rails/right/shuttles/command` | `mfja_rail_interfaces/msg/ShuttleCommand` |
| Add shuttle | `/room_315/rails/right/shuttles/add` | `mfja_rail_interfaces/srv/AddShuttle` |
| Switch commands | `/room_315/rails/right/switches/command` | `mfja_rail_interfaces/msg/SwitchCommand` |
| Switch state | `/room_315/rails/right/switches/state` | `mfja_rail_interfaces/msg/SwitchState` |
| Stopper commands | `/room_315/rails/right/stoppers/command` | `mfja_rail_interfaces/msg/StopperCommand` |
| Stopper state | `/room_315/rails/right/stoppers/state` | `mfja_rail_interfaces/msg/StopperState` |
| Rail sensors | `/room_315/rails/right/sensors/feedback` | `mfja_rail_interfaces/msg/SensorFeedback` |

Use the same names under `/room_315/rails/left/...` for the left rail.

Right rail basic commands:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'ON'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'RESET'}"
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_5', start_slot: '2', speed: 0.2, start_enabled: false}"
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_6', start_slot: '3', speed: 0.2, start_enabled: true}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '1'}]}"
ros2 topic echo /room_315/rails/right/shuttles/state mfja_rail_interfaces/msg/ShuttleState
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
```

Left rail basic commands:

```bash
ros2 topic pub --once /room_315/rails/left/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_left_shuttle_1', command: 'ON'}"
ros2 service call /room_315/rails/left/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_left_shuttle_2', start_slot: '1', speed: 0.2, start_enabled: false}"
ros2 service call /room_315/rails/left/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_left_shuttle_3', start_slot: '2', speed: 0.2, start_enabled: true}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
ros2 topic echo /room_315/rails/left/shuttles/state mfja_rail_interfaces/msg/ShuttleState
ros2 topic echo /room_315/rails/left/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
```

Common slot notes:

- Right rail uses its own `slot 1..4` set from `rail_network_right.yaml`.
- Left rail uses its own `slot 1..4` set from `rail_network_left.yaml`.
- If a shuttle enters `FALLING`, use `RESET` on that rail's `shuttles/command` topic.

Unless a later example explicitly uses `/room_315/rails/left/...`, the remaining
examples in this README refer to the right rail.

## Room 315 Typed Interfaces

Phase 4 adds the `mfja_rail_interfaces` package and migrates the Room 315
rail/shuttle topics away from raw `std_msgs/msg/String` payloads. Phase 5 moves
the canonical topics under `/room_315/rails/{right,left}/...`.

Messages:

- `NamedState`: `name`, `state`.
- `SwitchCommand` / `SwitchState`: arrays of `NamedState` switches.
- `StopperCommand` / `StopperState`: arrays of `NamedState` stoppers.
- `ShuttleCommand`: `name`, `command`, optional `start_slot`, optional `speed`.
- `AddShuttle` service: `name`, `start_slot`, `speed`, and `start_enabled`.
  Use `start_enabled: false` to create a waiting shuttle, or
  `start_enabled: true` to create one that starts immediately.
- `ShuttleState`: one shuttle pose/state sample.
- `SensorFeedback`: array of `SensorReading` entries for variable sensor counts.
  Each `SensorReading.active` value is `0` or `1` only; no continuous
  distance is published. Each `SensorReading.sensor_type` value is always
  `sensor`; use `name` to identify what that sensor is for.

Typed examples:

```bash
ros2 interface show mfja_rail_interfaces/msg/SwitchCommand
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
ros2 topic echo /room_315/rails/right/switches/state mfja_rail_interfaces/msg/SwitchState

ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '1'}]}"
ros2 topic echo /room_315/rails/right/stoppers/state mfja_rail_interfaces/msg/StopperState

ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'ON'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'REMOVE'}"
ros2 topic echo /room_315/rails/right/shuttles/state mfja_rail_interfaces/msg/ShuttleState

ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
```

The supported public rail API is the typed
`/room_315/rails/{right,left}/...` topic set shown above.
