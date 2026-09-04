# Shuttle, Switch, Stopper, and Debug Reference

## Allowed Shuttle Start Slots

Only these four start slots are allowed:

| Slot | Gazebo pose |
| --- | --- |
| `1` | `-15.43 -3.86 0.84 0 0 3.14` |
| `2` | `-14.95 -3.86 0.84 0 0 3.14` |
| `3` | `-14.77 -5.54 0.84 0 0 0` |
| `4` | `-15.24 -5.54 0.84 0 0 0` |

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

After Gazebo and the shuttle node are running, call this service:

```text
/room_315/rails/right/shuttles/add
```

Add a stopped shuttle at slot 3, then start it later:

```bash
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_5', start_slot: '3', speed: 0.2, start_enabled: false}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_5', command: 'ON'}"
```

Add a shuttle with a specific entity name and speed, moving immediately:

```bash
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_6', start_slot: '4', speed: 0.2, start_enabled: true}"
```

Supported add-service modes:

- Stopped/waiting: `start_enabled: false`.
- Moving immediately: `start_enabled: true`.

Notes:

- `room315_right_shuttle_1` to `room315_right_shuttle_4` are preloaded in the worlds.
- Shuttles beyond the preloaded count are spawned through `/world/<world_name>/create`.
- If the requested start slot is occupied, the node rejects the add service request and does not create a new shuttle.
- A slot is considered occupied when an existing shuttle is within `start_slot_occupancy_radius_m` of that start pose.
- Start-slot labels in this README follow the current cell numbering.

## Shuttle ON/OFF Control

Each shuttle can be independently enabled, disabled, reset to its start slot,
or removed from Gazebo through:

```text
/room_315/rails/right/shuttles/command
```

For the left rail, use the same commands on `/room_315/rails/left/shuttles/command`
with entity names such as `room315_left_shuttle_1`.

Disabling a shuttle keeps the model in place and stops its kinematic motion.
Enabling it again lets it continue from its current segment and arc-length
position. `RESET` re-snaps the shuttle to its configured start slot without
restarting Gazebo. `REMOVE` deletes the shuttle model from Gazebo and
unregisters it from the node.

Turn one shuttle off:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'OFF'}"
```

Turn it back on:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'ON'}"
```

Reset a shuttle after it entered `FALLING`:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'RESET'}"
```

Remove a shuttle completely from the simulation:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'REMOVE'}"
```

Add the same shuttle back after removal:

```bash
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_2', start_slot: '2', start_enabled: false}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'ON'}"
```

Control all shuttles at once:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'ALL', command: 'OFF'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'ALL', command: 'ON'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'ALL', command: 'RESET'}"
```

## Stopper Control and Sensor Workflow

Stopper logic is independent from switch logic. A stopper is a binary primitive:

- `0`, `OPEN`, `RELEASE`, `OFF`: the stopper is open and shuttles may pass.
- `1`, `STOP`, `CLOSED`, `ON`: the stopper stops a shuttle before the switch.

Public stopper labels:

| Stopper | Before switch | Stop segments |
| --- | --- | --- |
| `A1` | `A1` | `A14` |
| `A2` | `A2` | `A12E`, `A12I` |
| `A3` | `A3` | `A23` |
| `A4` | `A4` | `A34E`, `A34I` |

Stopper commands use:

```text
/room_315/rails/right/stoppers/command
```

Close one stopper:

```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '1'}]}"
```

Open one stopper:

```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '0'}]}"
```

Close or open all stoppers:

```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'ALL', state: '1'}]}"
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'ALL', state: '0'}]}"
```

Binary rail sensor occupancy is exposed on:

```text
/room_315/rails/right/sensors/feedback
```

Echo the sensor readings:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
```

Each message contains one `SensorReading` per configured rail sensor.
Rail sensors are binary occupancy sensors, not distance sensors: for normal
sensors, `active: 1` means a shuttle is on top of that sensor within its YAML
`radius_m`; `A*_STOPPER_SENSOR` names are regular position sensors whose point is
derived from the matching stopper minus `before_stopper_m`. `active: 0` means
the sensor is clear. The published `sensor_type` is
always `sensor`. When active, `shuttle_name` identifies the detected shuttle.

Closing one stopper is not sufficient preparation for changing a route: a
shuttle can already be inside the next branch/guard segment. For a manual
simulation test, use this safe sequence:

1. Stop the current high-level launch.
2. Restart it with both initial shuttle counts set to `0`.
3. Change the coordinated `A1`/`A2` or `A3`/`A4` pair in one command.
4. Wait for the switch-state topic to report the requested actual state.
5. Use a validated, route-compatible scenario before adding motion.

Normal numbered start slots are on exterior guard segments. Resetting a
shuttle to one of those slots does not make it safe to select an interior
route.

Example after launching an empty rail:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'INTERIOR'}]}"
ros2 topic echo /room_315/rails/right/switches/state mfja_rail_interfaces/msg/SwitchState
```

Virtual position detector names are published on the same sensor feedback topic:

```text
/room_315/rails/right/sensors/feedback
```

These detector names follow the same public `A1` to `A4` structure already used
for switches and stoppers:

- `DZI2R`, `DZI1R`, `DZI4R`, `DZI3R`: right-rail indexing-zone detectors for
  `slot 1`, `slot 2`, `slot 3`, and `slot 4`.
- `DA1R`, `DA2R`, `DA3R`, `DA4R`: right-rail detector on the single-track side
  of each switch.
- `DA1ER`, `DA2ER`, `DA3ER`, `DA4ER`: right-rail detector on the `EXTERIOR`
  branch.
- `DA1IR`, `DA2IR`, `DA3IR`, `DA4IR`: right-rail detector on the `INTERIOR`
  branch.

Echo the position detectors:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
```

Typical manual checks:

```bash
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_1', start_slot: '1', speed: 0.05, start_enabled: true}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

Expected detector families:

- `slot 1`, `slot 2`, `slot 3`, and `slot 4` startup positions trigger
  `DZI2R`, `DZI1R`, `DZI4R`, and `DZI3R`.
- `ALL=EXTERIOR` makes the shuttle pass through `...ER` branch detectors.
- `ALL=INTERIOR` makes the shuttle pass through `...IR` branch detectors.

Position detectors use the same `sensor_type: sensor` and binary `active`
semantics as every other rail sensor.

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

`WAITING` still means the drive is enabled and may resume when the stopper or
collision clears. An accepted `OFF` command instead reports `DISABLED`. The
published `speed` remains the configured travel-speed setting in both states;
it is retained for the next `ON` command and is not instantaneous velocity.

You usually do not need to pass these parameters, but they can be overridden:

```bash
-p enable_collision_avoidance:=true \
-p shuttle_collision_distance_m:=0.33
```

## Robot-Shuttle Gazebo Collision

The Room 315 shuttle model has a simple box collision volume for robot contact.
Room 315 rail path and switch collisions use a separate Gazebo
`collide_bitmask`, so shuttles do not collide with the rail geometry they are
kinematically following. Robot collision models keep the default Gazebo mask, so
robot links still collide with the shuttle body.

Visual-only device markers remain collision-free.

## Switch Control

Each rail has its own switch-command topic:

```text
/room_315/rails/right/switches/command
/room_315/rails/left/switches/command
```

Supported states:

- `EXTERIOR` or `E`
- `INTERIOR` or `I`

Switch selectors for normal operation:

- Public station labels: `A1`, `A2`, `A3`, `A4`
- Group selector: `ALL`

The rail-specific topic determines whether the command applies to the right or
left rail, so prefer the public labels `A1`, `A2`, `A3`, and `A4`.

The typed command topic is a low-level simulation interface and bypasses the
supervised route-planning boundary. Treat `A1`/`A2` and `A3`/`A4` as coordinated
pairs. Apply the empty-rail procedure above before any of the following manual
commands; do not reroute moving shuttles on the basis of a fixed timer.

Set all switches to the exterior branch:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
```

Set all switches to the interior branch:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

Switch one coordinated pair on either rail:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'EXTERIOR'}, {name: 'A2', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'INTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'EXTERIOR'}, {name: 'A2', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'INTERIOR'}]}"
```

Send multiple updates in one command:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'INTERIOR'}, {name: 'A3', state: 'EXTERIOR'}, {name: 'A4', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'INTERIOR'}, {name: 'A3', state: 'EXTERIOR'}, {name: 'A4', state: 'EXTERIOR'}]}"
```

Use the rail-specific command topics `/room_315/rails/right/switches/command` or
`/room_315/rails/left/switches/command`. Route logic and Gazebo switch visuals
update only when the delayed actual switch state is applied.

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
/room_315/rails/right/shuttles/pose_offset_command
```

Examples:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/pose_offset_command std_msgs/msg/String "{data: 'dx=0.01'}"
ros2 topic pub --once /room_315/rails/right/shuttles/pose_offset_command std_msgs/msg/String "{data: 'dy=-0.02'}"
ros2 topic pub --once /room_315/rails/right/shuttles/pose_offset_command std_msgs/msg/String "{data: 'x=0.0 y=0.0 z=0.0'}"
ros2 topic pub --once /room_315/rails/right/shuttles/pose_offset_command std_msgs/msg/String "{data: 'sx=1.0 sy=1.0'}"
ros2 topic pub --once /room_315/rails/right/shuttles/pose_offset_command std_msgs/msg/String "{data: 'reset'}"
```

## State and Debug Topics

State topic:

```bash
ros2 topic echo /room_315/rails/right/shuttles/state --once
```

Rail sensor events:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback --once
```

First shuttle pose:

```bash
ros2 topic echo /room_315/rails/right/shuttles/pose_cmd --once
```

Specific shuttle pose:

```bash
ros2 topic echo /room_315/rails/right/shuttles/room315_right_shuttle_3/pose_cmd --once
```

Visual switch state:

```bash
ros2 topic echo /mfja/conveyor/switch_states --once
```

Room 315 topics:

```bash
ros2 topic list | grep room_315
```

Offline kinematic core test:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle.py \
  --path-backend cubic_hermite \
  --switch A1=E \
  --switch A2=E \
  --switch A3=E \
  --switch A4=E
```

## Important Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gazebo_world_name` | `room_315_only` | Gazebo world used to derive `/world/<name>/set_pose`, `/world/<name>/create`, and `/world/<name>/remove`. |
| `enable_gazebo_set_pose` | `false` | If `true`, the node moves Gazebo shuttle models. |
| `enable_gazebo_spawn` | `true` | Allows runtime spawning of shuttles beyond the preloaded models. |
| `start_slot` | `2` | Start slot for a single shuttle. |
| `start_slots` | empty | Comma-separated start slots for multiple shuttles, for example `1,2,3,4`. |
| `shuttle_count` | `1` | Initial shuttle count for the node. The integrated room/full-floor launches pass `0` by default. |
| `start_enabled` | `false` | Initial shuttles are always visible. If `true`, they start moving immediately. If `false`, they wait for an `ON` command. |
| `gazebo_entity_name` | `room315_right_shuttle_1` | Gazebo entity name for a single shuttle on the right rail. The left rail default is `room315_left_shuttle_1`. |
| `gazebo_entity_names` | empty | Comma-separated names for multiple shuttles. |
| `preloaded_shuttle_count` | `4` | Number of shuttle models already present in the world on the right rail. The left rail currently preloads `1`. |
| `reject_occupied_start_slots` | `true` | Reject runtime add service requests when the requested start slot is occupied. |
| `start_slot_occupancy_radius_m` | `0.33` | Radius used to decide if a start slot is occupied. |
| `speed` | `0.25` | Shuttle speed in m/s. |
| `update_rate_hz` | `30.0` | Internal kinematic update rate. |
| `gazebo_set_pose_rate_hz` | `10.0` | Rate for Gazebo `set_pose` calls. |
| `sensor_publish_rate_hz` | `10.0` | Publish rate for binary `SensorFeedback` messages. |
| `show_device_markers` | `true` | Spawn visual markers for position sensors and stoppers. Switch bodies are colored separately by switch state. |
| `device_marker_dynamic_refresh` | `false` | If `true`, delete and respawn device markers when their active/inactive color changes. Disabled by default to avoid Gazebo `remove` errors for markers that have not been inserted yet. |
| `device_marker_refresh_grace_period_s` | `0.5` | Minimum delay after a successful marker spawn before deleting/recreating it for a color refresh. This avoids Gazebo remove requests racing ahead of marker insertion. |
| `path_backend` | `cubic_hermite` | Geometry sampler used by the shuttle core. Use `cubic_hermite` for normal continuous motion or `polyline` for direct CSV comparison. |
| `arc_length_samples_per_edge` | `16` | Sub-samples per CSV edge used to parameterize the continuous path by arc length. |
| `enable_collision_avoidance` | `true` | Stop before center-distance collision. |
| `shuttle_collision_distance_m` | `0.33` | Minimum allowed center distance between shuttles. |
| `switch_command_topic` | `/room_315/rails/right/switches/command` | Switch command topic for the right rail. The left rail default is `/room_315/rails/left/switches/command`. |
| `switch_state_topic` | `/room_315/rails/right/switches/state` | Actual delayed switch state topic for the right rail. The left rail default is `/room_315/rails/left/switches/state`. |
| `stopper_command_topic` | `/room_315/rails/right/stoppers/command` | Independent binary stopper command topic for the right rail. The left rail default is `/room_315/rails/left/stoppers/command`. |
| `stopper_state_topic` | `/room_315/rails/right/stoppers/state` | Actual delayed stopper state topic for the right rail. The left rail default is `/room_315/rails/left/stoppers/state`. |
| `sensor_feedback_topic` | `/room_315/rails/right/sensors/feedback` | Unified binary sensor occupancy for before-stopper sensors, `DZI*`, and `DA*`. The left rail default is `/room_315/rails/left/sensors/feedback`. |
| `add_shuttle_service` | `/room_315/rails/right/shuttles/add` | Runtime shuttle add service for the right rail. The left rail default is `/room_315/rails/left/shuttles/add`. |
| `shuttle_control_command_topic` | `/room_315/rails/right/shuttles/command` | Per-shuttle ON/OFF/RESET/REMOVE control topic for the right rail. The left rail default is `/room_315/rails/left/shuttles/command`. |
| `shuttle_state_topic` | `/room_315/rails/right/shuttles/state` | Combined shuttle state topic for the right rail. The left rail default is `/room_315/rails/left/shuttles/state`. |
| `pose_offset_command_topic` | `/room_315/rails/right/shuttles/pose_offset_command` | Runtime pose calibration topic for the right rail. The left rail default is `/room_315/rails/left/shuttles/pose_offset_command`. |
| `switch_motion_delay_s` | `0.3` | Delay before requested switch state becomes actual and the visible Gazebo switch model moves. |
| `stopper_motion_delay_s` | `0.1` | Delay before requested stopper state becomes actual. |
| `publish_visual_switch_commands` | `true` | Move the visible Gazebo switch models when delayed actual switch states are applied. |
| `sync_from_visual_switch_states` | `true` | Sync route logic from the latest visual switch state. |

## Runtime Troubleshooting

- If Gazebo does not open, start the room-only or full-floor launch before the shuttle node.
- If `Gazebo set_pose service is not ready yet`, check `gazebo_world_name` and `ros2 service list`.
- If full-floor services appear as `/world/default/...`, restart Gazebo after ensuring the world contains `<world name="mfja_3rd_floor">`.
- If runtime-spawned shuttles do not appear, check `/world/<world_name>/create`.
- If `REMOVE` does not delete the shuttle model, check `/world/<world_name>/remove` in `ros2 service list`.
- If an add service request is rejected, check whether another shuttle is still inside `start_slot_occupancy_radius_m` of that slot.
- If the rail path was edited, rebuild and run the offline kinematic core test
  before testing in Gazebo.
- If you need to compare the continuous path against the measured CSV path, rerun the shuttle node with `-p path_backend:=polyline`.
- If a shuttle stops with `stopped_by` set to a stopper name, open that stopper with `/room_315/rails/right/stoppers/command`.
- If a shuttle stops in `WAITING`, it is likely blocked by another shuttle within `shuttle_collision_distance_m`.
- If a shuttle was commanded `OFF`, require a newer state with mode `DISABLED`;
  `WAITING` is not a drive-disable acknowledgement.
- If a shuttle enters `FALLING`, the graph has no valid successor for the current switch configuration. Reset it with `/room_315/rails/right/shuttles/command`, for example `room315_right_shuttle_2=RESET`.
- If a switch moves visually but the shuttle route does not change, send commands to `/room_315/rails/right/switches/command`, not directly to `/mfja/conveyor/switch_cmd`.
