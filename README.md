# MFJA 3rd Floor

This repository contains the ROS 2 and Gazebo models for the MFJA third floor.
Its simplest Staubli example plans one table pick-and-place with HPP and uses
the same plan in Viser, Gazebo, or through the direct VAL3 robot driver.

## Install the Staubli pick-and-place

The installer supports Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic.

First install ROS 2 Jazzy from the
[official Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Then install the remaining packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential doxygen git python3-venv ros-dev-tools \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-coal ros-jazzy-jrl-cmakemodules \
  ros-jazzy-pinocchio ros-jazzy-proxsuite
```

Create one folder, clone the repository, and run the installer:

```bash
export MFJA_WORK_DIR="$HOME/mfja"
mkdir -p "$MFJA_WORK_DIR"
git clone --recurse-submodules \
  https://github.com/psardin001/mfja_3rd_floor_gz.git \
  "$MFJA_WORK_DIR/mfja_3rd_floor_gz"
"$MFJA_WORK_DIR/mfja_3rd_floor_gz/install.sh" "$MFJA_WORK_DIR"
source "$MFJA_WORK_DIR/setup.bash"
```

The installer imports the exact HPP revisions from `hpp_jazzy.repos` and
creates this single-folder layout:

```text
mfja/
├── .venv/
├── hpp_sources/
├── hpp_ws/
├── mfja_3rd_floor_gz/
├── mfja_ws/
└── setup.bash
```

Source `$HOME/mfja/setup.bash` in every new terminal. It sets `MFJA_WORK_DIR`
to its own folder, so the commands below also work in a fresh shell. The source
checkout may be placed elsewhere by passing that work folder to `install.sh`.
The first HPP build is lengthy. On a machine with enough memory, prefix the
install command with `CMAKE_BUILD_PARALLEL_LEVEL=2` to use two compiler jobs.

## Test the pick-and-place

Planning computes and validates the trajectory locally:

```bash
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh
```

Viser opens the same planned paths at `http://localhost:8000`:

```bash
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --viser
```

For Gazebo, use the same isolated ROS domain in both terminals:

```bash
# Terminal 1
source "$MFJA_WORK_DIR/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_pick_place_sim.launch.py
```

```bash
# Terminal 2
source "$MFJA_WORK_DIR/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --execute
```

For an authorized physical robot, use the cell's ROS domain in both terminals
and enter the controller address when launching the driver:

```bash
# Terminal 1
source "$MFJA_WORK_DIR/setup.bash"
read -r -p "Staubli controller IP: " ROBOT_IP
ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_hardware.launch.py robot_ip:="$ROBOT_IP"
```

```bash
# Terminal 2: read the current joint positions, then plan from them
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_demos room315_read_configuration.py

# Copy the six values from the reported "positions" array.
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh \
  --execution-profile hardware \
  --q-start Q1 Q2 Q3 Q4 Q5 Q6
```

After commissioning, path review, and operator approval, add `--execute` to the
terminal 2 command. Automated checks cover planning and simulation; the
operator validates physical execution in the commissioned cell.

## Other simulations

### 1. Launching the Full Floor
To run the complete 3rd-floor environment with all rooms, you can launch the `full_floor.launch.py`. You can choose to load all robots or none:
```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```
*(Change `robots:=none` to `robots:=all` to spawn TIAGo, KUKA, Stäubli, and Yaskawa robots).*

### 2. Launching Room 315 (Rail Simulation)
Launch the flexible rail system and shuttles in Room 315:
```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

### 3. Launching a Single Industrial Robot
Launch one industrial robot and its support table (for example, KUKA):
```bash
ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py \
  robot:=kuka \
  gui:=true
```
*(Other options for `robot` include `staubli`, `hc10`, and `hc10dt`).*

### 4. Basic Shuttle Control (Room 315)
If you launched the Room 315 shuttles, you can control them via ROS topics:

**Turn ON a shuttle:**
```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON'}"
```

**Control rail switches (e.g., switch all to interior):**
```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'INTERIOR'}]}"
```

---

## 📂 Repository Layout

*   `mfja_3rd_floor_description/`: Gazebo worlds, models, meshes, and URDF/SDF assets.
*   `mfja_rail_interfaces/`: Custom ROS 2 interfaces for commands, states, and sensors.
*   `mfja_robot_control_config/`: Shuttle/switch scripts, bridge configurations, and rail kinematic settings.
*   `mfja_3rd_floor_bringup/`: Centralized launch entry points for the full floor, Room 315, and single robot setups.
*   `mfja_staubli_demos/`: Stage 1 Staubli HPP arm-planning exercise.
*   `mfja_staubli_manipulation_demos/`: Staubli HPP table pick-and-place.
*   `hpp_jazzy.repos`: exact HPP source manifest for a reproducible host underlay.

---

## 📚 Detailed Documentation

For a deep dive into advanced features, please refer to our dedicated documentation files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**: Includes step-by-step guides for adding shuttles dynamically, reading sensor feedback, testing industrial robots, and troubleshooting.
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Technical details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[Stage 1: Staubli Cartesian HPP Demo](mfja_staubli_demos/README.md)**: Constrained arm planning and collision checking.
*   **[Stage 2: Room 315 Staubli Manipulation Demo](mfja_staubli_manipulation_demos/README.md)**: Table pick-and-place in Gazebo or through the direct VAL3 driver.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.
