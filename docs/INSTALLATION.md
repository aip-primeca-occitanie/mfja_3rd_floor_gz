# Installation and Workspace Setup

This guide installs the supported base simulation and explains the additional
requirements for research features. The base target is Ubuntu 24.04, ROS 2
Jazzy, and Gazebo Harmonic.

## What Works After the Base Installation

The base installation supports:

- Room 315 and full-floor Gazebo launches;
- right/left rail shuttle control;
- switches, stoppers, sensors, markers, and payload visuals;
- industrial robot and TIAGo spawning/control;
- VLA camera bridge and primitive supervisor;
- most source and package tests.

Active learned V4 inference and language-to-motion execution additionally need
external model/runtime artifacts. They are intentionally not bundled into a
normal Git clone.

## System Requirements

- Ubuntu 24.04 (Noble), `amd64` or `arm64` supported by the ROS installation.
- ROS 2 Jazzy Desktop.
- Gazebo Harmonic through the Jazzy `ros_gz`/vendor packages.
- Python 3.12.
- CMake 3.28 or newer for the description, control, and bringup packages.
- Enough disk space for the repository, colcon outputs, and optional datasets.
  Large research datasets/checkpoints can require many additional gigabytes.
- A working OpenGL display for the Gazebo GUI. `gui:=false` suppresses the GUI
  client, but RGB-D camera sensors can still require a working EGL/headless
  rendering backend on a display-less host.

A CUDA GPU is optional. Basic simulation, rail control, CPU tests, and many data
tools do not require one.

## 1. Install ROS 2 Jazzy and Development Tools

If ROS 2 is not installed, follow the
[official ROS 2 Jazzy Ubuntu deb installation procedure](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
to configure the ROS apt repository and install `ros-jazzy-desktop`. The
[official Gazebo/ROS installation guide](https://gazebosim.org/docs/harmonic/ros_installation/)
documents the recommended Jazzy and Harmonic pairing.

After the repository is configured, install the MFJA base tools and ROS-Gazebo
pairing:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  ninja-build \
  pkg-config \
  python3-colcon-common-extensions \
  python3-pip \
  python3-pytest \
  python3-rosdep \
  python3-venv \
  python3-yaml \
  ripgrep \
  ros-jazzy-desktop \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz
```

Gazebo Harmonic is the recommended Gazebo pairing for ROS 2 Jazzy. Do not add a
different Gazebo generation unless you are deliberately porting and retesting
the project.

Initialize `rosdep` once per machine. If it was already initialized, skip the
`sudo rosdep init` command:

```bash
sudo rosdep init
rosdep update
```

Verify the base environment:

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
ros2 --help
gz sim --versions
cmake --version
python3 --version
```

Expected ROS distribution: `jazzy`. Gazebo should report the Harmonic/gz-sim8
generation.

## 2. Create a Workspace and Clone the Repository

The repository root is not a ROS package. Clone it under a colcon workspace
`src/` directory. The normal clone follows GitHub's default branch. During the
current integration window, the marker check below switches to
`INTERNSHIP-ALI-2026` only when the default branch does not yet
contain the launch stack documented here. After the merge, the same commands
remain on `main`.

Run this block only for a new checkout. If
`$HOME/mfja_ws/src/mfja_3rd_floor_gz` already exists, skip it and use
[Updating an Existing Checkout](#updating-an-existing-checkout).

```bash
(
  set -euo pipefail

  MFJA_WS="$HOME/mfja_ws"
  MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

  mkdir -p "$MFJA_WS/src"
  git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git \
    "$MFJA_REPO"

  if [ ! -f "$MFJA_REPO/docs/INSTALLATION.md" ] ||
     [ ! -f "$MFJA_REPO/mfja_3rd_floor_bringup/launch/room_315_floor_common.py" ]; then
    git -C "$MFJA_REPO" switch --track \
      origin/INTERNSHIP-ALI-2026
  fi

  test -f "$MFJA_REPO/docs/INSTALLATION.md"
  test -f "$MFJA_REPO/mfja_3rd_floor_bringup/launch/room_315_floor_common.py"
  git -C "$MFJA_REPO" status --short --branch
)
```

The final command prints the integration branch before the merge and `main`
after it. The two file checks prevent continuing with a revision that does not
match this guide.

Define the workspace paths in the current terminal for the remaining steps:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
```

If you already cloned it elsewhere, either move it under `src/` or set
`MFJA_REPO` to its absolute path and use the explicit `--paths` build commands
below.

## 3. Install Repository Dependencies

Source ROS and let `rosdep` resolve the four real packages:

```bash
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy -y \
  --skip-keys "python3-torch python3-torchvision"
```

This installs the base simulation dependencies, including the ROS-Gazebo
bridges/interfaces, OpenCV/NumPy/Pillow, and PlanSys2 packages available through
the Ubuntu/ROS repositories. The two skipped rosdep keys are optional learned
visual-runtime dependencies. On the supported Ubuntu 24.04 host, rosdep may map
them to `python3-torch` and `python3-torchvision` even when apt has no installable
candidate. Install them in the optional environment below only if you need
training, V4 inference, or the Torch-gated tests.

Verify dependency state:

```bash
rosdep check --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy \
  --skip-keys "python3-torch python3-torchvision"
```

Do not place an extracted reproduction dataset or frozen source tree under the
workspace `src/` directory. If it contains `package.xml`, colcon/rosdep may find
a duplicate `mfja_robot_control_config`.

## 4. Build

Build all packages from the workspace root:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
```

The explicit paths keep colcon focused on the four packages even if external
research material exists elsewhere near the repository.

On a low-memory machine, reduce parallelism:

```bash
colcon build --symlink-install --executor sequential --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
```

## 5. Source and Verify the Installation

```bash
source "$MFJA_WS/install/setup.bash"

ros2 pkg prefix mfja_3rd_floor_bringup
ros2 pkg prefix mfja_3rd_floor_description
ros2 pkg prefix mfja_rail_interfaces
ros2 pkg prefix mfja_robot_control_config

ros2 interface show mfja_rail_interfaces/msg/ShuttleCommand
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
```

If any prefix command fails, the build did not complete or this terminal is
using the wrong overlay. See [Troubleshooting](TROUBLESHOOTING.md).

## 6. Every New Terminal

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
```

Source the ROS base first and the workspace overlay second.

Optional convenience for Bash:

```bash
alias mfja_source='source /opt/ros/jazzy/setup.bash && source "$HOME/mfja_ws/install/setup.bash"'
```

Add an alias to your shell startup only if the workspace path is stable. Avoid
automatically sourcing multiple ROS workspaces whose packages have the same
names.

## 7. First Smoke Run

The high-level launch clears `~/.ros/room315_vla_obstacles.json` by default.
This file is a disposable pose cache. Add
`room315_clear_vla_obstacle_pose_cache:=false` to preserve it. If you override
`room315_vla_obstacle_pose_file`, ensure that it names only the intended cache
file because the default startup action unlinks the configured path.

Start a lightweight server-only Room 315 runtime. This disables the Gazebo GUI
client; a display-less machine may still need a working EGL/headless rendering
backend because the world includes camera sensors:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=false \
  start_paused:=false \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=0 \
  room315_shuttles_start_enabled:=false
```

After about five seconds, use another sourced terminal:

```bash
ros2 topic echo --once /clock
ros2 service list | grep '^/world/room_315_only/'
ros2 topic echo --once /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Stop the launch with `Ctrl-C`. Continue with the
[Quick Start and Feature Guide](QUICK_START_AND_FEATURE_GUIDE.md).

## Optional Hybrid Nix Shell

`flake.nix` provides a hybrid development shell for build tools and selected
libraries. It does not install ROS 2 or Gazebo. The host must already contain
`/opt/ros/jazzy` and the ROS-Gazebo packages.

Install Nix using its official installation instructions, then:

```bash
cd "$MFJA_REPO"
nix develop
```

The shell:

- sets `ROS_DISTRO=jazzy`;
- sources `/opt/ros/jazzy/setup.bash` when present;
- defaults `RMW_IMPLEMENTATION` to Fast DDS when not already set;
- provides CMake, GCC, Ninja, Git, pkg-config, and a small Python environment;
- delegates `colcon` to the host `/usr/bin/colcon`.

Build from the workspace root and source its overlay while inside the shell:

```bash
cd "$MFJA_WS"
colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
source "$MFJA_WS/install/setup.bash"
```

## Optional Feature Requirements

### PlanSys2 and Live Task Planning

The control package declares `plansys2_msgs`, `plansys2_bringup`, and
`plansys2_planner`, so the normal `rosdep install` should install their Jazzy
packages. Before live planning:

```bash
ros2 pkg prefix plansys2_planner
ros2 pkg prefix plansys2_bringup
```

The planner backend must also be available through the installed PlanSys2
configuration. Verify lifecycle/service readiness at runtime rather than
assuming package presence proves a working planner.

### Visual Training and V4 Inference

Torch and TorchVision are not required for the base simulator. For the exact
CPU package pair recorded by the checked-in V4 evidence, create an isolated
environment that can still see the system ROS Python packages:

```bash
python3 -m venv --system-site-packages "$HOME/.venvs/mfja-visual"
source "$HOME/.venvs/mfja-visual/bin/activate"
python -m pip install --upgrade pip
python -m pip install \
  torch==2.10.0 torchvision==0.25.0 \
  --index-url https://download.pytorch.org/whl/cpu

python -c 'import torch, torchvision; print(torch.__version__, torchvision.__version__)'
python -c 'import rclpy; print(rclpy.__file__)'
```

Activate this environment before configuring/building if you want CMake to
register the Torch-gated tests. If the package was already configured, rebuild
with `--cmake-clean-cache`. For GPU/CUDA, choose a wheel command compatible with
the host driver from the
[official PyTorch installer](https://pytorch.org/get-started/locally/) instead
of reusing the CPU command. The exact version pairing is also listed on the
[official previous-versions page](https://pytorch.org/get-started/previous-versions/).
Keep virtual environments outside the repository and colcon source tree.

Active V4 inference also needs a complete external promotion bundle. Read
[Visual-State Runtime Integration](room315_visual_runtime_integration.md) before
launching it.

### Local English Intent Model

The setup tool downloads a pinned 1.04 GiB (1.12 GB) GGUF checkpoint. Its
default dependency installer uses the user package site and can retry with
`--break-system-packages` on a PEP 668 system, so the recommended workflow is a
dedicated environment plus `--skip-dependency-install`:

```bash
export ROOM315_INTENT_DIR="$HOME/models/room315_intent"
python3 -m venv --system-site-packages "$HOME/.venvs/room315-intent"
source "$HOME/.venvs/room315-intent/bin/activate"
python -m pip install --upgrade pip
python -m pip install 'llama-cpp-python==0.3.16'

python3 "$MFJA_REPO/mfja_robot_control_config/scripts/setup_room315_intent_model.py" \
  --model-dir "$ROOM315_INTENT_DIR" \
  --skip-dependency-install
source "$ROOM315_INTENT_DIR/room315_intent.env"
```

Initial setup needs network access unless the exact file is already present.
The generated config is offline-only and verifies the pinned SHA-256 before
loading. Reactivate the same virtual environment before running the semantic
model. Do not commit the checkpoint, environment file, or local config.

### LeRobot Export

The core repository does not declare the external `lerobot` Python package.
Only the optional conversion tool requires it. Install a compatible LeRobot
version in a dedicated environment according to its upstream instructions, then
confirm that environment can import both `lerobot` and any ROS/Python modules
needed by the selected workflow.

### Datasets and Reproduction Artifacts

Download datasets, checkpoints, and release archives outside both the Git
working tree and colcon `src` tree, for example:

```bash
export MFJA_DATA_ROOT="$HOME/mfja_data"
mkdir -p "$MFJA_DATA_ROOT"
```

Verify published `SHA256SUMS` before extraction/use. Dataset availability does
not grant a license beyond the terms attached to that release.

## Updating an Existing Checkout

First preserve or commit your own work and confirm that `git status --short` is
clean. To update the currently checked-out branch without creating a merge
commit:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

git -C "$MFJA_REPO" fetch origin
git -C "$MFJA_REPO" pull --ff-only
```

After the integration work reaches `main`, a checkout created by the temporary
fallback in Step 2 can move to the default branch with:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

git -C "$MFJA_REPO" fetch origin
git -C "$MFJA_REPO" switch main
git -C "$MFJA_REPO" pull --ff-only origin main
```

Then reinstall dependency changes and rebuild:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy -y \
  --skip-keys "python3-torch python3-torchvision"

colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"

source "$MFJA_WS/install/setup.bash"
```

Use `--cmake-clean-cache` when dependency/CMake discovery changed. Do not edit
or copy files directly into `install/` to avoid rebuilding.

## Installation Checklist

- [ ] Ubuntu 24.04 is in use.
- [ ] `/opt/ros/jazzy/setup.bash` exists.
- [ ] `gz sim --versions` reports the Harmonic/gz-sim8 generation.
- [ ] `rosdep check` succeeds with only the two intentionally skipped optional
      Torch keys excluded.
- [ ] Colcon discovers exactly the four MFJA packages from the intended paths.
- [ ] The full build completes without errors.
- [ ] All four `ros2 pkg prefix` checks succeed after sourcing.
- [ ] `ShuttleCommand` can be shown through `ros2 interface show`.
- [ ] The Room 315 launch arguments can be displayed.
- [ ] The server-only smoke publishes `/clock`, world services, and shuttle
      state.
- [ ] Optional model/data artifacts are stored outside Git/workspace `src`.

For any failed item, use [Troubleshooting](TROUBLESHOOTING.md).
