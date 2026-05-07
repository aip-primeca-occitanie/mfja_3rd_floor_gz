# Room 315 Kinematic Rail Network

This directory contains the Room 315 kinematic-first shuttle network. The
shuttle is controlled by graph routing and an arc-length path backend generated
from CSV rail geometry, then pushed into Gazebo with `set_pose`. Contact
dynamics, wheel physics, and rail contact are intentionally not part of this
phase.

The repository root `README.md` contains the full operator guide for launching
Gazebo, running one or more shuttles, adding shuttles at runtime, changing
switch states, and controlling the robots.

## Files

- `raw_segments/`: source CSV files for the directed rail segments.
- `normalized_segments/`: preprocessed CSV files with duplicate points removed,
  arc length, tangent, and yaw columns.
- `rail_network_right.yaml`: explicit graph topology for the right rail: nodes,
  segments, switches, fixed transitions, and block placeholders.
- `rail_network_left.yaml`: explicit graph topology for the left rail with its
  own switch labeling and routing layout.
- `rail_devices_right.yaml`: user-editable right-rail slots, position sensors,
  approach sensors, and stoppers.
- `rail_devices_left.yaml`: user-editable left-rail slots, position sensors,
  approach sensors, and stoppers.
- `segment_summary.yaml`: preprocessing summary generated from the raw CSVs.
- `validation_report.yaml`: validation results for lengths, snap distances,
  endpoint gaps, and tangent mismatches.
- `continuous_path_report.yaml`: validation results comparing the continuous
  path backend against the direct CSV polyline reference.
- `debug_plots/`: generated visual debug plots of the rail network.

## Segment Direction

Every rail segment is one-way only. Motion always follows the CSV order from
`index=0` to the last row in that file. The geometry is used for pose
interpolation only. Routing is decided by `rail_network_right.yaml`, not by exact
endpoint equality.

If a shuttle reaches the end of a segment and the graph has no valid successor
for the current switch state, the shuttle enters `FALLING` mode instead of
silently correcting or teleporting.

## Path Backends

The calibrated CSV files remain the source of truth, but runtime motion can be
sampled in two ways:

- `cubic_hermite`: recommended default. It creates a continuous path from CSV
  points and tangents, then reparameterizes it by arc length.
- `polyline`: direct CSV polyline interpolation used as a reference and debug
  backend.

Use `cubic_hermite` for normal demos and `polyline` only when comparing the
continuous path against the measured CSV points.

## Rail Device YAML

Phase 1 moves Room 315 rail devices out of the topology YAMLs and into:

```text
rail_devices_right.yaml
rail_devices_left.yaml
```

The shuttle node has a `devices_yaml` parameter. If it is empty, the node picks
`rail_devices_right.yaml` for `rail_side:=right` and `rail_devices_left.yaml`
for `rail_side:=left`. To test a custom device file, pass:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p devices_yaml:=/absolute/path/to/my_devices.yaml
```

Each device is placed on a rail segment with `segment` and `s_ratio`:

```yaml
position_sensors:
  - name: DZI1R
    segment: A12E
    s_ratio: 0.411866742
    radius_m: 0.09
```

`segment` must match a segment in the active rail network YAML. `s_ratio` is a
normalized distance along that segment, where `0.0` is the start of the segment
and `1.0` is the end. Runtime arc length is computed as:

```text
s = s_ratio * segment.length
```

The node then resolves device pose from the existing
`SegmentGeometry.sample(s)` method.

To move a sensor, stopper, or slot, edit only its `segment` and `s_ratio` in the
matching rail device YAML. For example, moving `DA1R` slightly upstream means
decreasing its `s_ratio` on `A14`. For stoppers or approach sensors that cover
two converging branch segments, keep one public device name and edit the
individual entries under `points:`.

You can also compute `segment` and `s_ratio` from a Gazebo XYZ coordinate with
`room_315_device_position_tool.py`. This is useful when you pick a sensor
position visually in Gazebo:

```bash
cd /home/tiago/ALI_ros2_ws/src/mfja_3rd_floor_gz
python3 mfja_robot_control_config/scripts/room_315_device_position_tool.py \
  --side right \
  --category position_sensors \
  --name DZI1R \
  --x -14.95 \
  --y -3.86 \
  --z 0.84
```

The command prints the nearest rail `segment`, `s`, `s_ratio`, and the distance
from the requested Gazebo point to the rail. It does not edit YAML unless you
add `--write`:

```bash
python3 mfja_robot_control_config/scripts/room_315_device_position_tool.py \
  --side right \
  --category position_sensors \
  --name DZI1R \
  --x -14.95 \
  --y -3.86 \
  --z 0.84 \
  --write
```

For devices that use `points:`, select the point with `--point-index`.
The tool refuses to write when the point is far from the rail unless `--force`
is passed.

Rail device YAML does not change sensor semantics. Phase 5 publishes the
canonical rail sensor topics under `/room_315/rails/{right,left}/...`: approach
feedback is on `/room_315/rails/right/sensors/feedback` or
`/room_315/rails/left/sensors/feedback`, and position feedback is on
`/room_315/rails/right/sensors/position_feedback` or
`/room_315/rails/left/sensors/position_feedback`.
Phase 4 migrates the rail and shuttle topics to `mfja_rail_interfaces` messages.
Deprecated `std_msgs/msg/String` aliases remain for migration where practical.

## Validate Rail Devices

Run the device validator after editing either rail device YAML:

```bash
cd /home/tiago/ALI_ros2_ws/src/mfja_3rd_floor_gz
python3 mfja_robot_control_config/scripts/room_315_devices_validator.py
```

The validator checks both `rail_devices_right.yaml` and
`rail_devices_left.yaml` against their matching rail network files. It reports
`PASS`, `WARN`, or `FAIL` and checks required fields, segment references,
`s_ratio`, duplicate names, marker radii/distances, and stopper default states.

## Gazebo Device Markers

The kinematic shuttle node spawns visual-only Gazebo markers from the loaded
device YAML when `enable_device_markers` is true. This is enabled by default and
uses the existing Gazebo create/remove services. Markers are static SDF models
with a single visual and no collision element, so they do not affect physics.
Markers are spawned gradually instead of all at once so Gazebo has time to
accept every create request.

Colors:

- slots: green
- position sensors: blue
- approach sensors: cyan
- stoppers: red

Example marker names:

```text
marker_right_DZI1R
marker_right_stopper_A1
marker_left_slot_1
```

To test marker movement, edit a device `segment` or `s_ratio` in the matching
`rail_devices_*.yaml`, rebuild or use a symlink install, then relaunch Gazebo
and the shuttle node. The marker will be recreated at the new
`SegmentGeometry.sample(s_ratio * segment.length)` position.

## Regenerate Preprocessed Data

Run these commands from the repository root after editing any file in
`raw_segments/` or after changing the network topology:

```bash
cd /home/tiago/ALI_ros2_ws/src/mfja_3rd_floor_gz

python3 mfja_robot_control_config/scripts/room_315_csv_preprocessor.py
python3 mfja_robot_control_config/scripts/room_315_network_validator.py
python3 mfja_robot_control_config/scripts/room_315_continuous_path_validator.py
python3 mfja_robot_control_config/scripts/room_315_segment_plot.py
```

Expected validation result:

```text
Validated 14 segments and 12 nodes.
Status: PASS (0 warnings)
```

The continuous path validator also prints:

```text
Validated continuous paths for 14 segments.
Status: PASS (0 warnings)
```

## Build After Changes

Use the meta-repository build command from the workspace root:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build --base-paths \
  src/mfja_3rd_floor_gz/mfja_rail_interfaces \
  src/mfja_3rd_floor_gz/mfja_robot_control_config \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_description \
  src/mfja_3rd_floor_gz/mfja_room_315_bringup \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_bringup \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_gz \
  --packages-select \
  mfja_rail_interfaces \
  mfja_robot_control_config \
  mfja_3rd_floor_description \
  mfja_room_315_bringup \
  mfja_3rd_floor_bringup \
  mfja_3rd_floor_gz \
  --symlink-install

source install/setup.bash
```

## Quick Runtime Example

Start Room 315 only. This starts Gazebo and the Room 315 right/left rail stack
by default, including YAML devices, visual markers, typed topics, command/state
separation, and `/room_315/rails/{right,left}/...` namespaces. Initial shuttle
count defaults to `0`, so no shuttle moves until you add one or request startup
shuttles:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_room_315_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

Start with one right shuttle and one left shuttle visible but waiting for `ON`:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_deployed:=true \
  room315_shuttles_start_enabled:=false
```

Start with one right shuttle and one left shuttle moving immediately:

```bash
ros2 launch mfja_room_315_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=true
```

Start the full floor with the same Room 315 rail stack enabled:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

To run Gazebo without the Room 315 rail stack, pass
`enable_room315_kinematic_shuttles:=false` to either launch.

Optional advanced mode: start a kinematic shuttle node manually after launching
Gazebo with `enable_room315_kinematic_shuttles:=false`:

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

For manual full-floor operation, use the same shuttle node and change only the
world name:

```bash
-p gazebo_world_name:=mfja_3rd_floor
```

## Quick Check of Recent Additions

The current runtime additions can be validated with four quick checks:

1. Switch commands:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'EXTERIOR'}, {name: 'A2', state: 'INTERIOR'}, {name: 'A3', state: 'E'}, {name: 'A4', state: 'I'}]}"
ros2 topic echo /room_315/rails/right/switches/state mfja_rail_interfaces/msg/SwitchState
```

2. Reset after `FALLING` without restarting Gazebo:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'RESET'}"
```

3. Remove and re-add a shuttle:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'REMOVE'}"
ros2 topic pub --once /room_315/rails/right/shuttles/add_command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'ADD', start_slot: '1', speed: 0.2}"
```

4. Watch the new right-rail detector stream:

```bash
ros2 topic echo /room_315/rails/right/sensors/position_feedback mfja_rail_interfaces/msg/SensorFeedback
```

Start slots are numbered as follows:

- `slot 1`: upper indexing pair, right physical position.
- `slot 2`: upper indexing pair, left physical position.
- `slot 3`: lower indexing pair, left physical position.
- `slot 4`: lower indexing pair, right physical position.

## Command and State Topics

Phase 3 separates requested commands from actual states. Phase 4 changes the
main command/state topics from `std_msgs/msg/String` to typed
`mfja_rail_interfaces` messages. Phase 5 moves the canonical topics under
`/room_315/rails/{right,left}/...`. Commands ask for a change. State topics
report the actual state after the configured motion delay has elapsed. Rail
routing uses the actual switch state, not the raw command payload.

Right rail topics:

```text
switch commands:    /room_315/rails/right/switches/command          mfja_rail_interfaces/msg/SwitchCommand
switch state:       /room_315/rails/right/switches/state            mfja_rail_interfaces/msg/SwitchState
stopper commands:   /room_315/rails/right/stoppers/command          mfja_rail_interfaces/msg/StopperCommand
stopper state:      /room_315/rails/right/stoppers/state            mfja_rail_interfaces/msg/StopperState
shuttle commands:   /room_315/rails/right/shuttles/command          mfja_rail_interfaces/msg/ShuttleCommand
add shuttle:        /room_315/rails/right/shuttles/add_command      mfja_rail_interfaces/msg/ShuttleCommand
shuttle state:      /room_315/rails/right/shuttles/state            mfja_rail_interfaces/msg/ShuttleState
approach feedback:  /room_315/rails/right/sensors/feedback          mfja_rail_interfaces/msg/SensorFeedback
position feedback:  /room_315/rails/right/sensors/position_feedback mfja_rail_interfaces/msg/SensorFeedback
```

Left rail topics use the same names under `/room_315/rails/left/...`.

The interface package is `mfja_rail_interfaces`. Its Room 315 messages are:

- `NamedState`
- `SwitchCommand`, `SwitchState`
- `StopperCommand`, `StopperState`
- `ShuttleCommand`, `ShuttleState`
- `SensorReading`, `SensorFeedback`

Old Phase 4 typed topics and the older mixed String topics are still accepted
or published as deprecated migration aliases. The left rail keeps the matching
`/room_315_left/...` aliases:

```text
/room_315_right/switch_cmd                 mfja_rail_interfaces/msg/SwitchCommand
/room_315_right/switch_state               mfja_rail_interfaces/msg/SwitchState
/room_315_right/stopper_cmd                mfja_rail_interfaces/msg/StopperCommand
/room_315_right/stopper_state              mfja_rail_interfaces/msg/StopperState
/room_315_right/shuttle/control_cmd        mfja_rail_interfaces/msg/ShuttleCommand
/room_315_right/shuttle/add_cmd            mfja_rail_interfaces/msg/ShuttleCommand
/room_315_right/shuttle/state              mfja_rail_interfaces/msg/ShuttleState
/room_315_right/sensors/switch_approach    mfja_rail_interfaces/msg/SensorFeedback
/room_315_right/sensors/position           mfja_rail_interfaces/msg/SensorFeedback
/room_315_right/switch_states              std_msgs/msg/String
/room_315_right/stopper_states             std_msgs/msg/String
/room_315_right/shuttle/control_cmd_string std_msgs/msg/String
/room_315_right/shuttle/add_cmd_string     std_msgs/msg/String
```

Use public switch labels `A1` through `A4` with `EXTERIOR` / `INTERIOR` or the
short `E` / `I` forms:

```bash
ros2 topic echo /room_315/rails/right/switches/state mfja_rail_interfaces/msg/SwitchState
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'EXTERIOR'}, {name: 'A2', state: 'INTERIOR'}, {name: 'A3', state: 'EXTERIOR'}, {name: 'A4', state: 'INTERIOR'}]}"
```

The node also republishes visual switch commands on `/mfja/conveyor/switch_cmd`
when the delayed actual switch state is applied, so the Gazebo switch visuals
rotate after the same `switch_motion_delay_s`.

Switch and stopper motion delays are configurable:

```bash
ros2 launch mfja_robot_control_config room_315_dual_kinematic_shuttles.launch.py \
  gazebo_world_name:=room_315_only \
  switch_motion_delay_s:=0.3 \
  stopper_motion_delay_s:=0.1
```

The delay is measured on the node ROS clock, so with `use_sim_time:=true` it
follows Gazebo simulation time.

At runtime, the same parameters can be changed on each shuttle node:

```bash
ros2 param set /room_315/rails/right/room_315_kinematic_shuttle switch_motion_delay_s 0.5
ros2 param set /room_315/rails/right/room_315_kinematic_shuttle stopper_motion_delay_s 0.2
```

## Stopper and Sensor Workflow

Stoppers are independent from switches. Each stopper has a binary state:

- `0`: open/released.
- `1`: stop/closed.

The public stopper set is `A1`, `A2`, `A3`, and `A4`. The approach sensor topic is:

```text
/room_315/rails/right/sensors/feedback
```

The manual teaching workflow is:

```text
sensor -> stop shuttle -> move switch -> unstop shuttle
```

Sensor messages use `mfja_rail_interfaces/msg/SensorFeedback`. Its
`readings` field is a list, so the same message can report several shuttles at
once. Use `shuttle_name` to identify the shuttle associated with each event.
Deprecated JSON mirrors are published on topics ending in `_json`.

Example:

```json
{
  "sensors": [
    {
      "before_switch": "A1",
      "distance_m": 0.247,
      "entity_name": "room315_right_shuttle_4",
      "segment": "A14",
      "stopper": "A1"
    }
  ]
}
```

This means `room315_right_shuttle_4` is approaching the A1 stopper on segment `A14`,
and the distance to the stop point is about `0.247 m`.

Example:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
ros2 topic echo /room_315/rails/right/stoppers/state mfja_rail_interfaces/msg/StopperState
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '1'}]}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '0'}]}"
```

The rail device YAML also defines virtual shuttle position detectors on:

```text
/room_315/rails/right/sensors/position_feedback
```

The public detector set is:

- `DZI2R`, `DZI1R`, `DZI4R`, `DZI3R` for the right-rail indexing-zone
  detector positions near the four slot areas.
- `DA1R`, `DA2R`, `DA3R`, `DA4R` on the single-track side of each switch.
- `DA1GR`, `DA2GR`, `DA3GR`, `DA4GR` on the `EXTERIOR` branch.
- `DA1SR`, `DA2SR`, `DA3SR`, `DA4SR` on the `INTERIOR` branch.

Practical use:

- Spawn or reset on `slot 1`, `slot 2`, `slot 3`, and `slot 4` to check the
  nearby `DZI...R` indexing-zone detectors.
- Send `ALL=EXTERIOR` on `/room_315/rails/right/switches/command` with
  `mfja_rail_interfaces/msg/SwitchCommand` to observe the `...GR` branch
  detectors.
- Send `ALL=INTERIOR` on `/room_315/rails/right/switches/command` with
  `mfja_rail_interfaces/msg/SwitchCommand` to observe the `...SR` branch
  detectors.

## Start Slots

The four allowed start slots are defined in `rail_devices_right.yaml` and
`rail_devices_left.yaml` with `segment` + `s_ratio`. The legacy right-rail
physical positions correspond approximately to:

```text
slot 1: -14.95 -3.86 0.84 0 0 3.14
slot 2: -15.43 -3.86 0.84 0 0 3.14
slot 3: -15.24 -5.54 0.84 0 0 0
slot 4: -14.77 -5.54 0.84 0 0 0
```

Multiple shuttles can be started with `shuttle_count` and `start_slots`, or
added while the node is running through `/room_315/rails/right/shuttles/add_command`.

Runtime add commands are rejected when the selected start slot is occupied.

## Shuttle ON/OFF Control

Per-shuttle motion control is available through:

```text
/room_315/rails/right/shuttles/command
```

The actual shuttle state is published separately on:

```text
/room_315/rails/right/shuttles/state
```

Examples:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'OFF'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'ON'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'RESET'}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_2', command: 'REMOVE'}"
```

## Collision Avoidance

Simple center-distance collision avoidance is enabled by default. The default
distance is `0.33 m`. This is not full block occupancy yet; it only prevents
kinematic shuttles from overlapping by stopping a following shuttle when it gets
too close to another active shuttle.
