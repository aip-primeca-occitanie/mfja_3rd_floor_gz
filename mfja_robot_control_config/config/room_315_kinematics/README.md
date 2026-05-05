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
  segments, switches, independent stoppers, fixed transitions, start slots, and
  block placeholders.
- `rail_network_left.yaml`: explicit graph topology for the left rail with its
  own slots, sensors, switch labeling, and stopper layout.
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

## Quick Runtime Example

Start Room 315 only:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_room_315_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```

Start the kinematic shuttle node:

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

For full-floor operation, use the same shuttle node and change only the world
name:

```bash
-p gazebo_world_name:=mfja_3rd_floor
```

## Quick Check of Recent Additions

The current runtime additions can be validated with four quick checks:

1. Switch commands:

```bash
ros2 topic pub --once /room_315_right/switch_states std_msgs/msg/String "{data: 'A1=EXTERIOR A2=INTERIOR A3=E A4=I'}"
```

2. Reset after `FALLING` without restarting Gazebo:

```bash
ros2 topic pub --once /room_315_right/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_right_shuttle_1=RESET'}"
```

3. Remove and re-add a shuttle:

```bash
ros2 topic pub --once /room_315_right/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_right_shuttle_1=REMOVE'}"
ros2 topic pub --once /room_315_right/shuttle/add_cmd std_msgs/msg/String "{data: 'entity=room315_right_shuttle_1 slot=1'}"
```

4. Watch the new right-rail detector stream:

```bash
ros2 topic echo /room_315_right/sensors/position
```

Start slots are numbered as follows:

- `slot 1`: upper indexing pair, left physical position.
- `slot 2`: upper indexing pair, right physical position.
- `slot 3`: lower indexing pair, right physical position.
- `slot 4`: lower indexing pair, left physical position.

## Switch Commands

Switch states are controlled through `/room_315_right/switch_states`. Use
`EXTERIOR` / `INTERIOR` or the short `E` / `I` forms. Visual switch selectors
stay `A1R` / `A1L` through `A4R` / `A4L`.

Example:

```bash
ros2 topic pub --once /room_315_right/switch_states std_msgs/msg/String "{data: 'A1=EXTERIOR A2=INTERIOR A3=EXTERIOR A4=INTERIOR'}"
```

The node also republishes visual switch commands on
`/mfja/conveyor/switch_cmd` so the Gazebo switch visuals rotate with the logical
state.

## Stopper and Sensor Workflow

Stoppers are independent from switches. Each stopper has a binary state:

- `0`: open/released.
- `1`: stop/closed.

The public stopper set is `A1`, `A2`, `A3`, and `A4`. The approach sensor topic is:

```text
/room_315_right/sensors/switch_approach
```

The manual teaching workflow is:

```text
sensor -> stop shuttle -> move switch -> unstop shuttle
```

Sensor messages are JSON strings. The `sensors` field is a list, so the same
message can report several shuttles at once. Use `entity_name` to identify the
shuttle associated with each event.

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
ros2 topic echo /room_315_right/sensors/switch_approach
ros2 topic pub --once /room_315_right/stopper_states std_msgs/msg/String "{data: 'A1=1'}"
ros2 topic pub --once /room_315_right/switch_states std_msgs/msg/String "{data: 'A1=INTERIOR'}"
ros2 topic pub --once /room_315_right/stopper_states std_msgs/msg/String "{data: 'A1=0'}"
```

The same network file also defines virtual shuttle position detectors on:

```text
/room_315_right/sensors/position
```

The public detector set is:

- `DZI2R`, `DZI1R`, `DZI4R`, `DZI3R` for the right-rail indexing-zone
  positions of `slot 1`, `slot 2`, `slot 3`, and `slot 4`.
- `DA1R`, `DA2R`, `DA3R`, `DA4R` on the single-track side of each switch.
- `DA1GR`, `DA2GR`, `DA3GR`, `DA4GR` on the `EXTERIOR` branch.
- `DA1SR`, `DA2SR`, `DA3SR`, `DA4SR` on the `INTERIOR` branch.

Practical use:

- Spawn or reset on `slot 1`, `slot 2`, `slot 3`, and `slot 4` to check
  `DZI2R`, `DZI1R`, `DZI4R`, and `DZI3R`.
- Send `ALL=EXTERIOR` to observe the `...GR` branch detectors.
- Send `ALL=INTERIOR` to observe the `...SR` branch detectors.

## Start Slots

The four allowed start slots are defined in `rail_network_right.yaml`:

```text
slot 1: -15.43 -3.86 0.84 0 0 3.14
slot 2: -14.95 -3.86 0.84 0 0 3.14
slot 3: -14.77 -5.54 0.84 0 0 0
slot 4: -15.24 -5.54 0.84 0 0 0
```

Multiple shuttles can be started with `shuttle_count` and `start_slots`, or
added while the node is running through `/room_315_right/shuttle/add_cmd`.

Runtime add commands are rejected when the selected start slot is occupied.

## Shuttle ON/OFF Control

Per-shuttle motion control is available through:

```text
/room_315_right/shuttle/control_cmd
```

Examples:

```bash
ros2 topic pub --once /room_315_right/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_right_shuttle_2=OFF'}"
ros2 topic pub --once /room_315_right/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_right_shuttle_2=ON'}"
ros2 topic pub --once /room_315_right/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_right_shuttle_2=RESET'}"
ros2 topic pub --once /room_315_right/shuttle/control_cmd std_msgs/msg/String "{data: 'room315_right_shuttle_2=REMOVE'}"
```

## Collision Avoidance

Simple center-distance collision avoidance is enabled by default. The default
distance is `0.33 m`. This is not full block occupancy yet; it only prevents
kinematic shuttles from overlapping by stopping a following shuttle when it gets
too close to another active shuttle.
