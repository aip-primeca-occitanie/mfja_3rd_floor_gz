# Room 315 Rail Devices and Tests

### 12. Edit Rail Device YAML and Move Markers

Device YAML files:

```text
mfja_robot_control_config/config/room_315_kinematics/rail_devices_right.yaml
mfja_robot_control_config/config/room_315_kinematics/rail_devices_left.yaml
```

Example device entry:

```yaml
position_sensors:
  - name: DZI1R
    segment: A23
    s_ratio: 0.35
    radius_m: 0.08
```

Example stopper and matching before-stopper position sensor:

```yaml
position_sensors:
  - name: A1_STOPPER_SENSOR
    stopper: A1
    before_stopper_m: 0.1
    radius_m: 0.08

stoppers:
  - name: A1
    before_switch: A1
    segment: A23
    s_ratio: 0.711151221
    default_state: '0'
```

To move a position sensor, stopper, or slot:

1. Choose the correct YAML file for the rail side.
2. Change `segment` or `s_ratio` on that position sensor, stopper, or slot.
3. Save the file.
4. Relaunch Gazebo.
5. The runtime device position and visual marker move together.

Stopper-linked position sensors do not have their own `segment`, `s_ratio`, or
`points`; each one is derived from the matching stopper and
`before_stopper_m`. Move the stopper entry under `stoppers`, and the linked
sensor moves with it. If a stopper has multiple physical points, edit only
`stoppers[].points`.

### 13. Check Visual Device Markers in Gazebo

Launch Room 315 or the full floor with the rail stack enabled. Markers are
spawned from the YAML-resolved positions:

- position sensors: blue when inactive, green when active
- stoppers: amber when released, red when active
- shuttles: black normally, red in `FALLING` mode
- switch bodies: green for state `I` / `INTERIOR`,
  orange for state `E` / `EXTERIOR`

Position sensor markers sit slightly above the rail so a visible part remains
above the shuttle body while a shuttle is crossing the sensor. Approach sensor
definitions remain in YAML and feedback, inherit stopper locations, and do not
spawn visual markers. The old continuous sensor distance field has been fully
removed from the sensor interface.

Hide device markers when you want a cleaner Gazebo scene:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_show_device_markers:=false
```

If a marker does not appear immediately, wait a few seconds. The node retries
Gazebo create requests while the `/world/<world_name>/create` bridge becomes
ready.

### 14. Test Shuttle-Shuttle Collision Avoidance

Launch two shuttles on one rail:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=2 \
  room315_left_shuttle_count:=0 \
  room315_shuttles_start_enabled:=true
```

Watch the right-rail shuttle state:

```bash
ros2 topic echo /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

When one shuttle gets too close to another, it should stop at a safe pose
instead of passing through it.

### 15. Test Robot-Shuttle Gazebo Collision

Launch Room 315 with one industrial robot and one visible shuttle:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=kuka \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=0 \
  room315_shuttles_start_enabled:=false
```

Then move the robot in Gazebo or with its ROS trajectory interface toward the
shuttle body. The shuttle has a conservative robot-contact collision volume.
Rail path geometry and rail switch geometry use a separate collision bitmask, so
the shuttle should not collide with the rail it follows.

### 16. Show Message Types and Topic Types

Inspect custom interfaces:

```bash
ros2 interface show mfja_rail_interfaces/msg/ShuttleCommand
ros2 interface show mfja_rail_interfaces/msg/SwitchCommand
ros2 interface show mfja_rail_interfaces/msg/SensorFeedback
```

Check live topic types:

```bash
ros2 topic info /room_315/rails/right/shuttles/command
ros2 topic info /room_315/rails/right/switches/state
ros2 topic info /room_315/rails/right/sensors/feedback
```

Canonical topics use `mfja_rail_interfaces` messages under
`/room_315/rails/{right,left}/...`.

### 17. Launch Names

All high-level launch entry points live in `mfja_3rd_floor_bringup/launch`:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py robot:=kuka
```
