# MFJA 3rd Floor

This repository contains the ROS 2 and Gazebo models for the MFJA third floor.
Its simplest Staubli example plans one table pick-and-place with HPP and uses
the same plan in Viser, Gazebo, or through the direct VAL3 robot driver.

## Installation

These instructions install the Staubli pick-and-place on Ubuntu 24.04 with
ROS 2 Jazzy and Gazebo Harmonic.

### Install ROS 2 and Robotpkg HPP

Configure the ROS 2 Jazzy apt repository using the
[official Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then add the Robotpkg Noble repository:

```bash
sudo apt update
sudo apt install -y curl
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
  | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" \
  | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
```

Install ROS 2, the build tools, HPP 9.0.2, and the HPP viewer:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake doxygen git python3-venv ros-dev-tools \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-control-msgs \
  robotpkg-py312-hpp-python=9.0.2 \
  robotpkg-py312-qt5-hpp-gepetto-viewer=9.0.2
```

### Clone the repository

Choose a work directory. The commands below use `$HOME/mfja`:

```bash
export MFJA_WORK_DIR="$HOME/mfja"
mkdir -p "$MFJA_WORK_DIR"
git clone --recurse-submodules \
  https://github.com/psardin001/mfja_3rd_floor_gz.git \
  "$MFJA_WORK_DIR/mfja_3rd_floor_gz"
```

### Build the HPP additions and MFJA

Run the installer from the cloned repository:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 \
  "$MFJA_WORK_DIR/mfja_3rd_floor_gz/install.sh" "$MFJA_WORK_DIR"
```

Robotpkg supplies the HPP Python and viewer stack. The installer imports and
builds the pinned TOPPRA, `hpp-toppra`, and `hpp-exec` sources from
`hpp_jazzy.repos`, builds the MFJA overlay, installs the Viser environment, and
creates `$MFJA_WORK_DIR/setup.bash`.

The generated files use this layout:

```text
mfja/
├── .venv/
├── hpp/
│   ├── build/
│   ├── install/
│   └── src/
├── mfja_3rd_floor_gz/
├── mfja_ws/
└── setup.bash
```

### Load the environment

Source the generated setup file in each terminal:

```bash
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
```

## Running the pick-and-place

### Planning

Planning computes and validates the trajectory locally:

```bash
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh
```

### Viser

Plan and open the paths in Viser:

```bash
ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --viser
```

Open `http://localhost:8000` in a browser.

### Gazebo

Use the same ROS domain in both terminals.

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

### Robot

An authorized operator uses the cell's ROS domain in both terminals and enters
the controller address when launching the driver.

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
terminal 2 command.

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
*   `hpp_jazzy.repos`: exact TOPPRA, `hpp-toppra`, and `hpp-exec` source revisions.

---

## 📚 Detailed Documentation

For a deep dive into advanced features, please refer to our dedicated documentation files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**: Includes step-by-step guides for adding shuttles dynamically, reading sensor feedback, testing industrial robots, and troubleshooting.
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Technical details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[Stage 1: Staubli Cartesian HPP Demo](mfja_staubli_demos/README.md)**: Constrained arm planning and collision checking.
*   **[Stage 2: Room 315 Staubli Manipulation Demo](mfja_staubli_manipulation_demos/README.md)**: Table pick-and-place in Gazebo or through the direct VAL3 driver.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.
