# Quick Start and Feature Guide

## Tested Launch Commands

GUI:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

Headless:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=false \
  enable_room315_kinematic_shuttles:=true
```

If you only edit README files, no rebuild is required. If you edit launch files,
Python scripts, package metadata, interfaces, models, worlds, URDF, SDF, or
config files, rebuild and source again.

## Multiple TIAGo Robots From YAML

`mfja_robot_control_config/config/robots.yaml` and
`mfja_robot_control_config/config/robots_room_315_only.yaml` are the robot spawn
lists. Add or remove TIAGo robots by editing the `robots:` list. Each entry must
have a unique `name`.

Supported TIAGo variants:

- `model: tiago_with_arm` for the mobile TIAGo with torso, head, and arm
  (`tiago` and `tiago_arm` are accepted aliases).
- `model: tiago_base` for the mobile TIAGo body without the arm or head
  (`tiago_no_arm` and `tiago_mobile_base` are accepted aliases).

Example:

```yaml
robots:
  - name: tiago_arm1
    model: tiago_with_arm
    x_pose: -6.4
    y_pose: -3.0
    z_pose: 0.0
    yaw: 1.57
    enabled: true

  - name: tiago_base1
    model: tiago_base
    x_pose: -7.2
    y_pose: -3.0
    z_pose: 0.0
    yaw: 1.57
    enabled: true
```

You can also keep `model: tiago` and switch the variant with `arm: true` or
`arm: false`.

## Step-by-Step Feature Guide

This section is the practical runbook. Use it when you want to test one feature
quickly without searching through the reference sections below.

### 1. Build and Source the Workspace

Use this terminal before any launch or topic command:

```bash
export MFJA_WS=~/test_mfja_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz
source install/setup.bash
```

If the workspace is already built and you only opened a new terminal, use:

```bash
export MFJA_WS=~/test_mfja_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 2. Launch Room 315 Only

Terminal 1 - start Room 315 with rails, device YAML, markers, typed topics,
and shuttle nodes enabled, but with no initial shuttles:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_sensor_publish_rate_hz:=10.0 \
  room315_show_device_markers:=true
```

Terminal 2 - check that the rail topics exist:

```bash
source /opt/ros/jazzy/setup.bash
source "${MFJA_WS:-$HOME/test_mfja_ws}/install/setup.bash"

ros2 topic list | grep /room_315/rails
```

Expected namespaces:

```text
/room_315/rails/right/...
/room_315/rails/left/...
```

### 3. Launch the Full Floor with the Same Room 315 Rail Features

Terminal 1:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_sensor_publish_rate_hz:=10.0 \
  room315_show_device_markers:=true
```

Terminal 2 - verify the full-floor Gazebo services:

```bash
source /opt/ros/jazzy/setup.bash
source "${MFJA_WS:-$HOME/test_mfja_ws}/install/setup.bash"

ros2 service list | grep /world/mfja_3rd_floor
```

Expected services include:

```text
/world/mfja_3rd_floor/create
/world/mfja_3rd_floor/remove
/world/mfja_3rd_floor/set_pose
```

### 4. Launch Only One Industrial Robot and Its Table

This mode loads only the ground plane, one selected industrial robot, and that
robot's support table. It does not load Room 315, rails, shuttles, sensors,
fixtures, other robots, or TIAGo.

Terminal 1 - choose exactly one robot:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py \
  robot:=kuka \
  gui:=true \
  start_paused:=false
```

Other valid selectors:

```bash
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py robot:=staubli gui:=true start_paused:=false
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py robot:=hc10 gui:=true start_paused:=false
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py robot:=hc10dt gui:=true start_paused:=false
```

Terminal 2 - check the selected robot topics. Example for KUKA:

```bash
source /opt/ros/jazzy/setup.bash
source "${MFJA_WS:-$HOME/test_mfja_ws}/install/setup.bash"

ros2 topic list | grep kuka1
```

### 5. Start Shuttles Hidden, Visible-but-Stopped, or Moving

No initial shuttles, but rail nodes are running:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=0 \
  room315_left_shuttle_count:=0
```

One right shuttle and one left shuttle visible, waiting for your `ON` command:

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

One right shuttle and one left shuttle moving immediately:

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

The same arguments work with the full-floor launch:

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

### 6. Add a Stopped or Moving Shuttle During Runtime

Terminal 1 - keep Room 315 running.

Terminal 2 - add a right-rail shuttle at slot 2, keep it stopped, then start it
with an `ON` command:

```bash
source /opt/ros/jazzy/setup.bash
source "${MFJA_WS:-$HOME/test_mfja_ws}/install/setup.bash"

ros2 service call /room_315/rails/right/shuttles/add \
  mfja_rail_interfaces/srv/AddShuttle \
  "{name: 'room315_right_shuttle_1', start_slot: '2', speed: 0.2, start_enabled: false}"

ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON'}"
```

Add a left-rail shuttle at slot 3 and make it move immediately:

```bash
ros2 service call /room_315/rails/left/shuttles/add \
  mfja_rail_interfaces/srv/AddShuttle \
  "{start_slot: '3', speed: 0.2, start_enabled: true}"
```

### 7. Turn Shuttles ON, OFF, RESET, or REMOVE

Turn one right shuttle on:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON'}"
```

Stop it:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'OFF'}"
```

Reset it to its start slot:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'RESET'}"
```

Remove it from Gazebo:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'REMOVE'}"
```

Control all shuttles on one rail:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'ALL', command: 'OFF'}"
```

Echo actual shuttle state:

```bash
ros2 topic echo /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

### 8. Move Switches with Command and State Topics

Commands are requests. State topics report the actual state after the configured
motion delay. Switch states accept `I`/`INTERIOR` and `E`/`EXTERIOR`.
Stopper states accept `0`/`PASS`/`OPEN` and
`1`/`STOP`/`CLOSED`.

Terminal 2 - watch actual switch state:

```bash
ros2 topic echo /room_315/rails/right/switches/state \
  mfja_rail_interfaces/msg/SwitchState
```

Terminal 3 - command switch A1 to the interior branch:

```bash
source /opt/ros/jazzy/setup.bash
source "${MFJA_WS:-$HOME/test_mfja_ws}/install/setup.bash"

ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
```

Command all right-rail switches to exterior:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"
```

Run with a longer switch delay so the visual delay is easy to see:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  room315_switch_motion_delay_s:=2.0
```

### 9. Open and Close Stoppers with Command and State Topics

Terminal 2 - watch actual stopper state:

```bash
ros2 topic echo /room_315/rails/right/stoppers/state \
  mfja_rail_interfaces/msg/StopperState
```

Terminal 3 - close stopper A1:

```bash
source /opt/ros/jazzy/setup.bash
source "${MFJA_WS:-$HOME/test_mfja_ws}/install/setup.bash"

ros2 topic pub --once /room_315/rails/right/stoppers/command \
  mfja_rail_interfaces/msg/StopperCommand \
  "{stoppers: [{name: 'A1', state: '1'}]}"
```

Open stopper A1:

```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command \
  mfja_rail_interfaces/msg/StopperCommand \
  "{stoppers: [{name: 'A1', state: '0'}]}"
```

Run with a longer stopper delay:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  room315_stopper_motion_delay_s:=1.0
```

### 10. Read Sensor Feedback

Room 315 rail sensors are binary occupancy sensors, not distance sensors.
For normal rail-point sensors, `active: 1` means a shuttle is on top of the
sensor point within its YAML `radius_m`. `A*_STOPPER_SENSOR` names are regular
position sensors linked to their matching stoppers; their point is the stopper
point minus `before_stopper_m`. `active: 0` means the sensor is clear.
A sensor only reports occupancy; stopping is controlled by the matching stopper
state and uses the linked before-stopper sensor point as the stop trigger.
All rail readings use `sensor_type: sensor`; the sensor name explains the
purpose of the detector.

All rail sensor occupancy is published on one topic:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Left rail uses the same names under `/room_315/rails/left/...`.

Quick empty-rail check:

Terminal 1:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_right_rail:=true \
  enable_room315_left_rail:=false \
  room315_right_shuttle_count:=0 \
  room315_sensor_publish_rate_hz:=2.0
```

Terminal 2:

```bash
timeout 8s ros2 topic echo --once /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Expected result: every right-rail position sensor is present and publishes
`active: 0`.

Quick occupied-sensor check:

Terminal 2:

```bash
ros2 service call /room_315/rails/right/shuttles/add \
  mfja_rail_interfaces/srv/AddShuttle \
  "{name: 'room315_right_shuttle_1', start_slot: '2', speed: 0.2, start_enabled: false}"

timeout 8s ros2 topic echo --once /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Expected result: `DZI2R` publishes `active: 1` and
`shuttle_name: room315_right_shuttle_1`. Remove the test shuttle when finished:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'REMOVE'}"
```

To test all right-rail position sensors, run a slow sweep and watch
`/room_315/rails/right/sensors/feedback`:

```bash
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

```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"

sleep 45

ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

```bash
timeout 120s ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Expected right-rail position sensor families:

- `DZI1R`, `DZI2R`, `DZI3R`, `DZI4R`
- `DA1R`, `DA2R`, `DA3R`, `DA4R`
- `DA1ER`, `DA2ER`, `DA3ER`, `DA4ER`
- `DA1IR`, `DA2IR`, `DA3IR`, `DA4IR`

To test all right-rail before-stopper sensors, watch `/sensors/feedback` during the
same route sweep. Expected names are `A1_STOPPER_SENSOR`, `A2_STOPPER_SENSOR`,
`A3_STOPPER_SENSOR`, and `A4_STOPPER_SENSOR`:

```bash
timeout 120s ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

For the left rail, use the same commands with `/room_315/rails/left/...`,
`enable_room315_right_rail:=false`, `enable_room315_left_rail:=true`, and
`room315_left_shuttle_count:=4`.
