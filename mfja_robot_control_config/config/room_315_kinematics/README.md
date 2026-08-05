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
- `rail_network_right.yaml`: explicit graph topology for the right rail: nodes,
  segments, switches, fixed transitions, and block placeholders.
- `rail_network_left.yaml`: explicit graph topology for the left rail with its
  own switch labeling and routing layout.
- `rail_devices_right.yaml`: user-editable right-rail slots, position sensors,
  stopper-linked position sensors, and stoppers.
- `rail_devices_left.yaml`: user-editable left-rail slots, position sensors,
  stopper-linked position sensors, and stoppers.

## Segment CSV Naming

The live right and left topology YAML files use normalized, explicit CSV
references. Every CSV filename stem matches its public segment name; for
example, public segment `A23` explicitly declares
`csv: raw_segments/A23.csv`. The
`csv_reference_schema: public_segment_filename_v1` marker distinguishes this
layout from historical topology snapshots.

The explicit `csv` field remains authoritative. Loader code must read that
field and must not infer a filename from the segment key merely because the
two names now match.

The kinematic loader contains a narrowly scoped compatibility table for the
exact legacy public-segment/CSV-reference pairs found in immutable historical
schema snapshots. It redirects those old references to the normalized files
and emits a deprecation warning. Live YAML and newly exported snapshots always
write normalized references; no duplicate legacy CSV files are retained.

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

Slots, position sensors, and stoppers are placed on a rail segment with
`segment` and `s_ratio`:

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

For `position_sensors`, binary feedback is driven only by `segment`,
`s_ratio`, and `radius_m`. Fields such as `switch` and `branch` are descriptive
YAML labels kept for readability and compatibility; changing them does not move
the detector. Older custom files may also contain `index_zone`, `start_slot`, or
`aliases`, but those legacy labels do not move the detector either. To move a
`DZI*` or `DA*` detector, edit its `segment` and `s_ratio`, or use the device
position tool.

To move a position sensor, stopper, or slot, edit only its `segment` and
`s_ratio` in the matching rail device YAML. For example, moving `DA1R` slightly
upstream means decreasing its `s_ratio` on `A14`. For stoppers that cover two
converging branch segments, keep one public stopper name and edit the individual
entries under `stoppers[].points`.

Before-stopper detectors are regular `position_sensors` entries linked to a
matching stopper. They define `stopper`, `before_stopper_m`, and `radius_m`.
Their runtime point is derived from the stopper point minus `before_stopper_m`,
so moving the stopper also moves the linked detector. Do not put `segment`,
`s_ratio`, or `points` on a stopper-linked position sensor.

All position sensor entries use `radius_m` as their occupancy radius. Missing
`radius_m` is a configuration error so sensor behavior stays explicit in YAML.
The canonical rail sensor topic is
`/room_315/rails/right/sensors/feedback` or
`/room_315/rails/left/sensors/feedback`; it contains both before-stopper
detectors and rail-point sensors.
The public rail API uses typed `mfja_rail_interfaces` messages only.

### Moving Single-Point and Multi-Point Devices

A single-point device has `segment` and `s_ratio` directly on the device entry:

```yaml
- name: DZI1R
  segment: A12E
  s_ratio: 0.411866742
  radius_m: 0.09
```

A stopper can keep one public name while using multiple physical points:

```yaml
- name: A4
  before_switch: A4
  default_state: '0'
  points:
  - segment: A34E
    s_ratio: 0.964078719
  - segment: A34I
    s_ratio: 0.943533612
```

The matching before-stopper position sensor follows those stopper points
automatically:

```yaml
- name: A4_STOPPER_SENSOR
  stopper: A4
  before_stopper_m: 0.1
  radius_m: 0.08
```

Rail sensors publish feedback; the stopper is what actually stops the shuttle
when its state is `1`.

## Gazebo Device Markers

The kinematic shuttle node spawns visual-only Gazebo markers from the loaded
device YAML when `enable_device_markers` is true. This is enabled by default and
uses the existing Gazebo create/remove services. Markers are static SDF models
with a single visual and no collision element, so they do not affect physics.
Markers are spawned gradually instead of all at once so Gazebo has time to
accept every create request.
Runtime delete/recreate color refreshes are disabled by default to avoid Gazebo
remove errors when a marker has not yet been inserted into the world.

Colors:

- position sensors: blue when inactive, green when active (`active=1`)
- stoppers: amber when released (`state=0`), red when active (`state=1`)
- shuttles: black normally, red in `FALLING` mode
- switch bodies: green for state `I` / `INTERIOR`,
  orange for state `E` / `EXTERIOR`

Position sensor markers sit slightly above the rail so a visible part remains
above the shuttle body while a shuttle is crossing the sensor. Stopper-linked
position sensors are normal position sensors, so they also publish normal
sensor feedback and can spawn normal sensor markers. The old continuous sensor
distance field has been fully removed from the sensor interface.

Example marker names:

```text
marker_right_DZI1R
marker_right_stopper_A1
```

To test marker movement, edit a position sensor or stopper `segment` or
`s_ratio` in the matching `rail_devices_*.yaml`, rebuild or use a symlink
install, then relaunch Gazebo and the shuttle node. The marker will be recreated
at the new
`SegmentGeometry.sample(s_ratio * segment.length)` position.

## Build After Changes

Use the meta-repository build command from the workspace root:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash

colcon build --base-paths \
  src/mfja_3rd_floor_gz/mfja_rail_interfaces \
  src/mfja_3rd_floor_gz/mfja_robot_control_config \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_description \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_bringup \
  --packages-select \
  mfja_rail_interfaces \
  mfja_robot_control_config \
  mfja_3rd_floor_description \
  mfja_3rd_floor_bringup \
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
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

Start with one right shuttle and one left shuttle visible but waiting for `ON`.
Initial shuttles are always deployed visibly; `room315_shuttles_start_enabled`
only chooses whether they wait or move immediately:

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

Start with one right shuttle and one left shuttle moving immediately:

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

Start the full floor with the same Room 315 rail stack enabled:

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

To run Gazebo without the Room 315 rail stack, pass
`enable_room315_kinematic_shuttles:=false` to either launch.

Optional advanced mode: start a kinematic shuttle node manually after launching
Gazebo with `enable_room315_kinematic_shuttles:=false`:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
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
ros2 service call /room_315/rails/right/shuttles/add mfja_rail_interfaces/srv/AddShuttle "{name: 'room315_right_shuttle_1', start_slot: '1', speed: 0.2, start_enabled: false}"
ros2 topic pub --once /room_315/rails/right/shuttles/command mfja_rail_interfaces/msg/ShuttleCommand "{name: 'room315_right_shuttle_1', command: 'ON'}"
```

4. Watch the new right-rail detector stream:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
```

## Sensor Test Cookbook

Use these checks after editing `rail_devices_right.yaml` or
`rail_devices_left.yaml`, or when you want to prove that the sensor model is
working in Gazebo.

### Test All Right-Rail Position Sensors

Launch the right rail with four slow shuttles:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_right_rail:=true \
  enable_room315_left_rail:=false \
  room315_right_shuttle_count:=4 \
  room315_shuttles_start_enabled:=true \
  room315_shuttle_speed:=0.08
```

To start with visual payload boxes on selected shuttles, add:

```bash
  room315_enable_payload_visuals:=true \
  room315_right_loaded_shuttles:=R2 \
  room315_left_loaded_shuttles:=L2
```

The payload local X offset defaults to `-0.08`, which centers the cube on the
shuttle. Override `room315_payload_pose_x_offset_m` only when intentionally
moving the box forward or backward relative to the shuttle.

Loaded shuttles publish privileged payload state on
`/room_315/rails/<side>/shuttles/payload_state`; change it at runtime with
`/room_315/rails/<side>/shuttles/payload_command`, for example
`{"shuttle":"R2","loaded":false}`. Payload state is for supervisor/debug and
dataset metadata only; VLA `model_input` still contains only language, overhead
images, and last command.

In a command terminal, sweep the switches through exterior and interior routes:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"

sleep 45

ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

Watch the detector stream:

```bash
timeout 120s ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Expected right-rail position sensor families:

- Indexing zone sensors: `DZI1R`, `DZI2R`, `DZI3R`, `DZI4R`.
- Main switch sensors: `DA1R`, `DA2R`, `DA3R`, `DA4R`.
- Exterior branch sensors: `DA1ER`, `DA2ER`, `DA3ER`, `DA4ER`.
- Interior branch sensors: `DA1IR`, `DA2IR`, `DA3IR`, `DA4IR`.

### Test All Right-Rail Before-Stopper Position Sensors

Use the same slow right-rail launch and route sweep, but watch the normal sensor
feedback topic:

```bash
timeout 120s ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Expected right-rail stopper-linked position sensors:

- `A1_STOPPER_SENSOR`
- `A2_STOPPER_SENSOR`
- `A3_STOPPER_SENSOR`
- `A4_STOPPER_SENSOR`

These sensors use the same code type as all other rail sensors:
`sensor_type=sensor`, `active=1` when a shuttle is on top of the derived
before-stopper detector point within the YAML `radius_m`, and `active=0`
otherwise.

### Test All Left-Rail Sensors

Launch only the left rail:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_right_rail:=false \
  enable_room315_left_rail:=true \
  room315_left_shuttle_count:=4 \
  room315_shuttles_start_enabled:=true \
  room315_shuttle_speed:=0.08
```

Sweep left switches:

```bash
ros2 topic pub --once /room_315/rails/left/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"

sleep 45

ros2 topic pub --once /room_315/rails/left/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

Watch both left-rail sensor streams:

```bash
timeout 120s ros2 topic echo /room_315/rails/left/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback

timeout 120s ros2 topic echo /room_315/rails/left/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
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

Right rail APIs:

```text
switch commands:    /room_315/rails/right/switches/command          mfja_rail_interfaces/msg/SwitchCommand
switch state:       /room_315/rails/right/switches/state            mfja_rail_interfaces/msg/SwitchState
stopper commands:   /room_315/rails/right/stoppers/command          mfja_rail_interfaces/msg/StopperCommand
stopper state:      /room_315/rails/right/stoppers/state            mfja_rail_interfaces/msg/StopperState
shuttle commands:   /room_315/rails/right/shuttles/command          mfja_rail_interfaces/msg/ShuttleCommand
add shuttle:        /room_315/rails/right/shuttles/add              mfja_rail_interfaces/srv/AddShuttle
shuttle state:      /room_315/rails/right/shuttles/state            mfja_rail_interfaces/msg/ShuttleState
sensor feedback:  /room_315/rails/right/sensors/feedback          mfja_rail_interfaces/msg/SensorFeedback
```

Left rail topics use the same names under `/room_315/rails/left/...`.

The interface package is `mfja_rail_interfaces`. Its Room 315 messages are:

- `NamedState`
- `SwitchCommand`, `SwitchState`
- `StopperCommand`, `StopperState`
- `ShuttleCommand`, `ShuttleState`
- `SensorReading`, `SensorFeedback`

The supported public rail API is the typed
`/room_315/rails/{right,left}/...` topic set shown above.

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

With the current rail device YAML, a closed stopper stops the shuttle at the
matching stopper-linked position sensor point, which is derived from
`before_stopper_m`.

## Stopper and Sensor Workflow

Stoppers are independent from switches. Each stopper has a binary state:

- `0`: open/released.
- `1`: stop/closed.

The public stopper set is `A1`, `A2`, `A3`, and `A4`. The unified sensor topic is:

```text
/room_315/rails/right/sensors/feedback
```

The manual teaching workflow is:

```text
sensor -> stop shuttle -> move switch -> unstop shuttle
```

Sensor messages use `mfja_rail_interfaces/msg/SensorFeedback`. Its `readings`
field contains binary occupancy readings. For normal rail-point sensors,
`active=1` means the named sensor is occupied within the configured YAML
`radius_m`. `A*_STOPPER_SENSOR` names are regular position sensors linked to their
matching stoppers; their point is the stopper point minus
`before_stopper_m`. `active=0` means the detector is clear.
When active,
`shuttle_name` identifies the detected shuttle. Sensors do not publish
continuous distance values.

Example:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
ros2 topic echo /room_315/rails/right/stoppers/state mfja_rail_interfaces/msg/StopperState
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '1'}]}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '0'}]}"
```

The rail device YAML also defines virtual shuttle position detector names on the same topic:

```text
/room_315/rails/right/sensors/feedback
```

The public detector set is:

- `DZI2R`, `DZI1R`, `DZI4R`, `DZI3R` for the right-rail indexing-zone
  detector positions near the four slot areas.
- `DA1R`, `DA2R`, `DA3R`, `DA4R` on the single-track side of each switch.
- `DA1ER`, `DA2ER`, `DA3ER`, `DA4ER` on the `EXTERIOR` branch.
- `DA1IR`, `DA2IR`, `DA3IR`, `DA4IR` on the `INTERIOR` branch.

Practical use:

- Spawn or reset on `slot 1`, `slot 2`, `slot 3`, and `slot 4` to check the
  nearby `DZI...R` indexing-zone detectors.
- Send `ALL=EXTERIOR` on `/room_315/rails/right/switches/command` with
  `mfja_rail_interfaces/msg/SwitchCommand` to observe the `...ER` branch
  detectors.
- Send `ALL=INTERIOR` on `/room_315/rails/right/switches/command` with
  `mfja_rail_interfaces/msg/SwitchCommand` to observe the `...IR` branch
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
added while the node is running through `/room_315/rails/right/shuttles/add`.
Use `start_enabled: false` to create a shuttle that waits for a later `ON`
command, or `start_enabled: true` to create one that starts immediately.

Runtime add service requests are rejected when the selected start slot is occupied.

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

## Robot-Shuttle Gazebo Collision

The `room315_shuttle` model has a simple box collision volume for robot contact.
Room 315 rail path and switch collisions use a separate Gazebo
`collide_bitmask`, so shuttles do not collide with the rail geometry they are
kinematically following. Robot collision models keep the default Gazebo mask, so
robot links still collide with the shuttle body.

Visual-only device markers remain collision-free.
