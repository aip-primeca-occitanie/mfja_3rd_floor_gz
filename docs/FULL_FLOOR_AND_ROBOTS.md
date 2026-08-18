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

> **Obstacle-cache warning:** Both high-level floor launches default to
> `room315_clear_vla_obstacle_pose_cache:=true`. At startup, that setting
> deletes `~/.ros/room315_vla_obstacles.json`. Add
> `room315_clear_vla_obstacle_pose_cache:=false` to preserve the cache. If you
> override `room315_vla_obstacle_pose_file`, point it only at a file that is
> intentionally disposable; the same startup cleanup applies to the override.

Terminal 1 - start the full floor with the Room 315 rail stack:

```bash
cd "${MFJA_WS:-$HOME/mfja_ws}"
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
cd "${MFJA_WS:-$HOME/mfja_ws}"
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
cd "${MFJA_WS:-$HOME/mfja_ws}"
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

The same launch files can run the world with or without configured robots. For
shuttle-only testing, use `robots:=none`. For robot experiments, explicitly
select only the robots you need.

Do **not** use `robots:=all` with the current full-floor configuration.
`robots:=all` selects every YAML entry regardless of its `enabled` value, and
the full-floor file currently contains both `tiago1` and `tiago_base1`. Those
mobile variants publish duplicate, unprefixed TF frame names and therefore must
not run in the same ROS graph. Select exactly one TIAGo variant until the mobile
TF frames are instance-prefixed.

Full floor with all four industrial robots and the arm-equipped TIAGo:

```bash
cd "${MFJA_WS:-$HOME/mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=kuka1,staubli1,yaskawa_hc10_1,yaskawa_hc10dt_1,tiago1 \
  start_paused:=false \
  gui:=true
```

Room 315 shuttles use the same `use_sim_time` clock as the rest of the full-floor
simulation, so robot motion and shuttle motion stay synchronized. If the full
scene runs below real time, the shuttle will look slower in wall-clock time
because the whole simulation is slower.

Room 315 only with all configured robots:

```bash
cd "${MFJA_WS:-$HOME/mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=all \
  start_paused:=false \
  gui:=true
```

The room-only YAML currently has only one TIAGo entry, so its `robots:=all`
selection does not create the two-mobile-robot TF conflict. Recheck that file
before using `all` after adding another mobile robot.

Robot selection supports exact names, short aliases, numeric YAML order,
`all`, and `none`. Prefer exact names, especially in multi-robot commands;
numeric selectors depend on YAML order, and a short alias is rejected when it
matches more than one configured robot. In particular, `robots:=tiago` is
ambiguous in the current full-floor configuration.

Common selectors:

```text
robots:=kuka1
robots:=staubli1
robots:=yaskawa_hc10_1
robots:=yaskawa_hc10dt_1
robots:=tiago1
robots:=tiago_base1
robots:=kuka1,tiago1
robots:=1,5
robots:=none
```

Current numeric mapping in the full-floor YAML:

| Selector | Robot |
| --- | --- |
| `1` | `kuka1` |
| `2` | `staubli1` |
| `3` | `yaskawa_hc10_1` |
| `4` | `yaskawa_hc10dt_1` |
| `5` | `tiago1` |
| `6` | `tiago_base1` |

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
cd "${MFJA_WS:-$HOME/mfja_ws}"
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

The recommended helper command publishes the correct `JointTrajectory` message
for each robot and accepts angular values in radians or degrees:

```bash
ros2 run mfja_robot_control_config robot_joint_command.py --list
ros2 run mfja_robot_control_config robot_joint_command.py kuka --unit rad --positions 0.6 -1.0 1.1 0.0 0.6 0.0
ros2 run mfja_robot_control_config robot_joint_command.py kuka --unit deg --positions 34.38 -57.30 63.03 0.0 34.38 0.0
```

When `--unit deg` is used, angular joints are converted to radians before
publishing. Linear joints, such as TIAGo's `torso_lift_joint`, stay in meters.

### Industrial Gripper Motion

Each industrial robot exposes a separate gripper position command:

```text
/<robot_name>/gripper/position_command
```

The ROS bridge forwards this command to one local symmetric controller inside
Gazebo, which applies the same bounded per-jaw target to both opposite-axis jaws.

Use the gripper helper to list the configured travel ranges or to open and
close each gripper:

```bash
ros2 run mfja_robot_control_config robot_gripper_command.py --list

ros2 run mfja_robot_control_config robot_gripper_command.py kuka open
ros2 run mfja_robot_control_config robot_gripper_command.py kuka close

ros2 run mfja_robot_control_config robot_gripper_command.py staubli open
ros2 run mfja_robot_control_config robot_gripper_command.py staubli close

ros2 run mfja_robot_control_config robot_gripper_command.py hc10 open
ros2 run mfja_robot_control_config robot_gripper_command.py hc10 close

ros2 run mfja_robot_control_config robot_gripper_command.py hc10dt open
ros2 run mfja_robot_control_config robot_gripper_command.py hc10dt close
```

An optional percentage can follow the action. The percentage is an absolute
fraction of the interval configured for that robot: `open 100` targets its
configured 100% position, `close 100` targets its configured 0% position, and
both `open 50` and `close 50` target the midpoint.

```bash
ros2 run mfja_robot_control_config robot_gripper_command.py kuka open 75
ros2 run mfja_robot_control_config robot_gripper_command.py kuka close 100
ros2 run mfja_robot_control_config robot_gripper_command.py hc10 open 50
ros2 run mfja_robot_control_config robot_gripper_command.py hc10 close 100
```

The per-robot percentage endpoints and the percentages used by bare `open` and
`close` commands are read from:

```text
mfja_robot_control_config/config/gripper_command_defaults.yaml
```

Each robot entry has four editable values:

```yaml
position_at_0_percent_m: 0.0
position_at_100_percent_m: 0.010
default_open_percentage: 100.0
default_close_percentage: 100.0
```

The two `position_at_*_percent_m` values are per-jaw positions in meters. For
example, setting Stäubli's `position_at_100_percent_m` to `0.020` gives each
jaw 20 mm of configured travel, so the total full opening
becomes 40 mm because the jaws move symmetrically. The 0% position must be zero or
greater, and the 100% position must be greater than the 0% position.

The command mapping is:

```text
open(P)  = q0 + (q100 - q0) * P / 100
close(P) = q100 - (q100 - q0) * P / 100
```

where `q0` and `q100` are the configured endpoint positions. All command modes,
including an explicit percentage and `--position`, load this file. `--position`
must be inside `[q0, q100]`.

Default action percentages of 100 preserve fully-open / fully-closed behavior.
At launch, the configured endpoints are also applied to the active gripper joint
and controller limits, so restart the robot simulation after changing them. The
launch files accept `gripper_config:=PATH`, while the command helper accepts
`--defaults-file PATH`. Use the same absolute path for both when overriding the
default: relative launch paths are resolved inside the package share, while
relative helper paths use the terminal's current directory. With a non-symlink
installation, rebuild the package after editing the source file. Show the
currently configured endpoints instead of relying on fixed documentation:

```bash
ros2 run mfja_robot_control_config robot_gripper_command.py --list
```

For a partially open gripper, send a configured-range per-jaw position in meters:

```bash
ros2 run mfja_robot_control_config robot_gripper_command.py hc10 --position 0.012
```

Preview the resolved topic and target without publishing:

```bash
ros2 run mfja_robot_control_config robot_gripper_command.py kuka open --dry-run
```

The helper waits for the gripper command subscriber and live robot joint states,
then publishes a short command burst. These commands animate the jaws only; the
models do not yet provide physical grasping or payload attachment.

Several supplied gripper CAD files are monolithic assemblies with the fingers
baked into one mesh. Those source files remain untouched, but they are not used
as the runtime gripper visuals. The simulation uses separate local body and
left/right jaw visuals instead, which prevents fixed fingers from remaining
visible underneath the moving jaws. For KUKA, the body remains simplified while
both moving jaws reuse the unmodified `jaw_kuka.stl` asset at unit scale; the
opposite jaw is rotated 180 degrees at runtime.

### KUKA KR6 R900 Sixx

```bash
ros2 run mfja_robot_control_config robot_joint_command.py kuka --unit rad \
  --positions 0.6 -1.0 1.1 0.0 0.6 0.0

ros2 run mfja_robot_control_config robot_joint_command.py kuka --unit deg \
  --positions 34.38 -57.30 63.03 0.0 34.38 0.0
```

### Staeubli TX2-60L

```bash
ros2 run mfja_robot_control_config robot_joint_command.py staubli --unit rad \
  --positions 0.1 0.4 -0.6 0.0 0.5 0.0

ros2 run mfja_robot_control_config robot_joint_command.py staubli --unit deg \
  --positions 5.73 22.92 -34.38 0.0 28.65 0.0
```

### Yaskawa HC10

```bash
ros2 run mfja_robot_control_config robot_joint_command.py hc10 --unit rad \
  --positions 0.2 -0.7 0.9 0.0 0.4 0.2

ros2 run mfja_robot_control_config robot_joint_command.py hc10 --unit deg \
  --positions 11.46 -40.11 51.57 0.0 22.92 11.46
```

### Yaskawa HC10DT

```bash
ros2 run mfja_robot_control_config robot_joint_command.py hc10dt --unit rad \
  --positions -0.2 -0.5 0.8 0.0 0.5 -0.2

ros2 run mfja_robot_control_config robot_joint_command.py hc10dt --unit deg \
  --positions -11.46 -28.65 45.84 0.0 28.65 -11.46
```

### TIAGo Arm and Head

```bash
ros2 run mfja_robot_control_config robot_joint_command.py tiago1 --unit rad \
  --positions 0.10 0.3 -0.5 -0.4 1.0 0.2 -0.2 0.1 0.2 -0.2

ros2 run mfja_robot_control_config robot_joint_command.py tiago1 --unit deg \
  --positions 0.10 17.19 -28.65 -22.92 57.30 11.46 -11.46 5.73 11.46 -11.46
```

### TIAGo Base Torso

```bash
ros2 run mfja_robot_control_config robot_joint_command.py tiago_base1 --unit rad \
  --positions 0.10
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
