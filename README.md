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
- `mfja_rail_interfaces/`: typed ROS 2 message interfaces for Room 315 rail commands, states, and sensors.
- `mfja_robot_control_config/`: launch base, bridge config, shuttle/switch scripts, and Room 315 kinematic config.
- `mfja_3rd_floor_bringup/`: launch entry points for Room 315, the full floor, and isolated industrial robot runs.
- `mfja_robot_control_config/config/room_315_kinematics/raw_segments/`: source rail segment CSV files for the Room 315 kinematic rail network.

The detailed Room 315 kinematic artifacts are also documented here:

```text
mfja_robot_control_config/config/room_315_kinematics/README.md
```

The focused HTML runbook is available at:

```text
runbook.html
```

## Installation

Use Ubuntu 24.04 with ROS 2 Jazzy. This repository is a meta-repository, so
clone it inside a colcon workspace `src/` directory and build from the
workspace root.

### 1. Install ROS 2 And Build Tools

If ROS 2 Jazzy is not installed yet, configure the ROS apt repository:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install the required packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  ninja-build \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-jazzy-desktop \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz

# Run this only if rosdep has not already been initialized on the machine.
sudo rosdep init || true
rosdep update
```

### 2. Clone The Repository

```bash
export MFJA_WS=~/test_mfja_ws
mkdir -p "$MFJA_WS/src"
cd "$MFJA_WS/src"
git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git
```

### 3. Optional: Enter The Nix Shell

If you use Nix, install it once:

```bash
sh <(curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install) --daemon
```

Then enter the shell from the repository directory before building:

```bash
cd "$MFJA_WS/src/mfja_3rd_floor_gz"
nix develop
```

### 4. Build The Workspace

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src/mfja_3rd_floor_gz -y --ignore-src --rosdistro jazzy
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz
```

### 5. Source The Workspace

```bash
source install/setup.bash
```

### 6. Every New Terminal

Without Nix:

```bash
export MFJA_WS=~/test_mfja_ws
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

With hybrid Nix:

```bash
export MFJA_WS=~/test_mfja_ws
cd "$MFJA_WS/src/mfja_3rd_floor_gz"
nix develop

cd "$MFJA_WS"
source install/setup.bash
```

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
  room315_sensor_detection_tolerance_m:=0.01 \
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
  room315_sensor_detection_tolerance_m:=0.01 \
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
motion delay. Switch states accept `0`/`INTERIOR`/`SMALL`/`S` and
`1`/`EXTERIOR`/`BIG`/`G`. Stopper states accept `0`/`PASS`/`OPEN` and
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
Each `SensorReading` has `active: 1` when a shuttle is on top of the
configured sensor point within `room315_sensor_detection_tolerance_m`, and
`active: 0` otherwise. A sensor only reports occupancy; it does not stop a
shuttle. Stopping is controlled independently by the matching stopper state.
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
- `DA1GR`, `DA2GR`, `DA3GR`, `DA4GR`
- `DA1SR`, `DA2SR`, `DA3SR`, `DA4SR`

To test all right-rail before-stopper sensors, watch `/sensors/feedback` during the
same route sweep. Expected names are `A1_APPROACH`, `A2_APPROACH`,
`A3_APPROACH`, and `A4_APPROACH`:

```bash
timeout 120s ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

For the left rail, use the same commands with `/room_315/rails/left/...`,
`enable_room315_right_rail:=false`, `enable_room315_left_rail:=true`, and
`room315_left_shuttle_count:=4`.

### 11. Edit Rail Device YAML and Move Markers

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

To move a device:

1. Choose the correct YAML file for the rail side.
2. Change `segment` or `s_ratio`.
3. Save the file.
4. Relaunch Gazebo.
5. The runtime device position and visual marker move together.

### 12. Check Visual Device Markers in Gazebo

Launch Room 315 or the full floor with the rail stack enabled. Markers are
spawned from the YAML-resolved positions:

- slots and sensors: blue
- stoppers: red
- switch bodies: green for state `0` / `INTERIOR` / `S`,
  orange for state `1` / `EXTERIOR` / `G`

All rail sensors use the same marker color because they are all binary
occupancy sensors. The old continuous sensor distance field has been fully
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

### 13. Test Shuttle-Shuttle Collision Avoidance

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

### 14. Test Robot-Shuttle Gazebo Collision

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

### 15. Show Message Types and Topic Types

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

### 16. Launch Names

All high-level launch entry points live in `mfja_3rd_floor_bringup/launch`:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py robot:=kuka
```

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

Terminal 1 - start the full floor with the Room 315 rail stack:

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
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
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
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
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

The same launch files can run the world with or without industrial robots. For
shuttle-only testing, use `robots:=none`. For robot experiments, use `robots:=all`
or select only the robots you need.

Full floor with all configured robots:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=all \
  start_paused:=false \
  gui:=true
```

Room 315 only with all configured robots:

```bash
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
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
cd "${MFJA_WS:-$HOME/test_mfja_ws}"
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
Rail sensors are binary occupancy sensors, not distance sensors: `active: 1`
means a shuttle is on top of that sensor within `sensor_detection_tolerance_m`,
and `active: 0` means the sensor is clear. The published `sensor_type` is
always `sensor`. When active, `shuttle_name` identifies the detected shuttle.

The intended manual workflow is:

1. Watch the binary sensor reading for a shuttle at the sensor.
2. Close the matching stopper, for example `A1=1`.
3. Move the switch, for example `A1=INTERIOR`.
4. Open the stopper again, for example `A1=0`.

Example sequence:

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback mfja_rail_interfaces/msg/SensorFeedback
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '1'}]}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/stoppers/command mfja_rail_interfaces/msg/StopperCommand "{stoppers: [{name: 'A1', state: '0'}]}"
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
- `DA1GR`, `DA2GR`, `DA3GR`, `DA4GR`: right-rail detector on the `EXTERIOR`
  branch.
- `DA1SR`, `DA2SR`, `DA3SR`, `DA4SR`: right-rail detector on the `INTERIOR`
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
- `ALL=EXTERIOR` makes the shuttle pass through `...GR` branch detectors.
- `ALL=INTERIOR` makes the shuttle pass through `...SR` branch detectors.

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

Switch one station on either rail:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}]}"
```

Send multiple updates in one command:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'EXTERIOR'}, {name: 'A3', state: 'INTERIOR'}, {name: 'A4', state: 'EXTERIOR'}]}"
ros2 topic pub --once /room_315/rails/left/switches/command mfja_rail_interfaces/msg/SwitchCommand "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'EXTERIOR'}, {name: 'A3', state: 'INTERIOR'}, {name: 'A4', state: 'EXTERIOR'}]}"
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
  --switch A1=G \
  --switch A2=G \
  --switch A3=G \
  --switch A4=G
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
| `sensor_detection_tolerance_m` | `0.01` | Binary rail sensor detection tolerance. A sensor is active only when a shuttle is within this tolerance of the configured sensor point. |
| `sensor_publish_rate_hz` | `10.0` | Publish rate for binary `SensorFeedback` messages. |
| `show_device_markers` | `true` | Spawn visual markers for slots, sensors, and stoppers. Switch bodies are colored separately by switch state. |
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
- If a shuttle enters `FALLING`, the graph has no valid successor for the current switch configuration. Reset it with `/room_315/rails/right/shuttles/command`, for example `room315_right_shuttle_2=RESET`.
- If a switch moves visually but the shuttle route does not change, send commands to `/room_315/rails/right/switches/command`, not directly to `/mfja/conveyor/switch_cmd`.
