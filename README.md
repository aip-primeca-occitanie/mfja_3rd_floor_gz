# MFJA 3rd Floor Gazebo Simulation

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for the MFJA 3rd floor. 

## 📖 General Overview

The simulation environment provides a comprehensive digital twin of the MFJA 3rd floor, featuring multiple work cells, industrial robotic arms (KUKA, Stäubli, Yaskawa), and mobile robots (TIAGo). 

A major focus of this repository is the **Room 315 flexible rail system**, which currently utilizes a highly reliable **kinematic shuttle simulation**. Instead of relying on complex physics interactions like wheel friction, shuttles move along arc-length paths generated from a calibrated explicit rail graph, ensuring smooth and predictable behavior for testing routing logic, multi-shuttle interactions, and switch controls.

Whether you are testing mobile robot navigation on the full floor, running pick-and-place tasks with a single robotic arm, or orchestrating a complex multi-shuttle logistics scenario in Room 315, this repository provides the necessary models and launch configurations.

**Navigation:** [requirements](#requirements-and-installation-choices)
· [native Ubuntu setup](#method-a-native-ubuntu-setup)
· [Nix setup](#method-b-hybrid-nix-setup)
· [first run](#a6-start-room-315-terminal-1)
· [quick commands](#basic-commands-and-quick-start)
· [detailed guide](DETAILED_GUIDE.md)

---

## Requirements and Installation Choices

This guide installs the `main_ali` branch on a new laptop. The repository root
is a meta-repository, not a ROS package, so it must be cloned below a colcon
workspace `src/` directory and built from the workspace root.

| Component | Supported requirement |
| --- | --- |
| Operating system | Ubuntu 24.04 Noble; `x86_64` is the safest target |
| ROS | ROS 2 Jazzy Desktop |
| Simulator | Gazebo Harmonic / `gz-sim8` through `ros_gz` |
| Python | Python 3.12 |
| CMake | 3.28 or newer |
| Graphics | OpenGL-capable display for the Gazebo GUI |

The base clone contains all project worlds, models, meshes, URDF/SDF assets,
rail CSV/YAML configuration, launch files, and typed interfaces. It has no Git
submodules and does not require a dataset, checkpoint, Torch, CUDA, or a
discrete GPU.

The direct project dependencies fall into these groups:

| Layer | Dependencies | Installation source |
| --- | --- | --- |
| Host tools | Git, GCC/build-essential, CMake, Ninja, pkg-config, colcon, rosdep, Python, PyYAML | Ubuntu `apt` |
| ROS build/interfaces | `ament_cmake`, ROSIDL generators/runtime, `std_msgs` | `rosdep` |
| Gazebo development libraries | `gz-common5`, `gz-msgs10`, `gz-plugin2`, `gz-sim8`, `gz-transport13` vendor packages | `rosdep` / `ros-jazzy-ros-gz` |
| ROS runtime | `rclpy`, launch/launch_ros, standard ROS messages, robot state publisher, `ros_gz_bridge`, `ros_gz_interfaces`, `ros_gz_sim` | `rosdep` |

Do not install every ROS dependency manually. The `rosdep install` step below
reads the four `package.xml` files and installs the correct Jazzy/Noble package
names.

Choose one complete installation path:

| Path | Build environment | ROS/Gazebo source |
| --- | --- | --- |
| [Method A: native Ubuntu](#method-a-native-ubuntu-setup) | Ubuntu packages | Ubuntu packages |
| [Method B: hybrid Nix](#method-b-hybrid-nix-setup) | Nix Bash/Ninja/Make/Git plus Ubuntu CMake, compiler, and Python | Ubuntu packages |

> **Important Nix boundary:** `flake.nix` is hybrid. Nix supplies Bash, Ninja,
> Make, and Git only. Ubuntu supplies CMake, GCC/G++, Python, pkg-config,
> colcon, ROS 2 Jazzy, Gazebo Harmonic, rosdep, and all ROS package
> dependencies. Keeping CMake and the compiler/runtime stack entirely on
> Ubuntu avoids mixing Nix and Noble discovery paths, system headers, or shared
> libraries. This is not a pure-Nix, NixOS, macOS, or Windows setup.

## Method A: Native Ubuntu Setup

Follow A1 through A8 in order. Commands marked one-time are not repeated after
the laptop is configured.

### A1. Confirm Ubuntu and Architecture

```bash
grep -E '^(NAME|VERSION_ID)=' /etc/os-release
uname -m
```

Continue only with Ubuntu `24.04`. The ROS/Nix files support `x86_64-linux` and
`aarch64-linux`, but Gazebo rendering on ARM is best-effort and must be
validated locally.

### A2. Install ROS 2, Gazebo, and Build Tools (One Time)

The following block updates Ubuntu, configures the
[official ROS 2 Jazzy apt source](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
when needed, and installs the complete host tool set. It changes system
packages and requires `sudo`:

```bash
(
  set -euo pipefail

  . /etc/os-release
  MFJA_UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
  test "$MFJA_UBUNTU_CODENAME" = "noble"

  sudo apt update
  sudo apt upgrade -y
  sudo apt install -y \
    ca-certificates \
    curl \
    locales \
    software-properties-common

  if ! locale | grep -qi 'utf-8'; then
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    export LC_ALL=en_US.UTF-8
  fi

  sudo add-apt-repository -y universe

  if ! apt-cache show ros-jazzy-desktop >/dev/null 2>&1; then
    MFJA_ROS_APT_SOURCE_VERSION="$(
      curl -fsSL \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p'
    )"
    test -n "$MFJA_ROS_APT_SOURCE_VERSION"

    MFJA_ROS_APT_DEB="$(mktemp --suffix=.deb)"
    trap 'rm -f -- "$MFJA_ROS_APT_DEB"' EXIT
    curl -fL -o "$MFJA_ROS_APT_DEB" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${MFJA_ROS_APT_SOURCE_VERSION}/ros2-apt-source_${MFJA_ROS_APT_SOURCE_VERSION}.${MFJA_UBUNTU_CODENAME}_all.deb"
    sudo dpkg -i "$MFJA_ROS_APT_DEB"
  fi

  sudo apt update
  sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    python3-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-venv \
    python3-yaml \
    ros-jazzy-desktop \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz

  printf 'MFJA host setup completed successfully.\n'
)
```

Initialize rosdep once per machine, then verify the supported versions:

```bash
(
  set -eo pipefail

  if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  rosdep update

  source /opt/ros/jazzy/setup.bash
  test "$ROS_DISTRO" = jazzy
  gz sim --versions
  cmake --version
  python3 --version
)
```

Gazebo must report the Harmonic / `gz-sim8` generation, CMake must be at least
3.28, and Python must be 3.12.

### A3. Create the Workspace and Clone `main_ali`

The repository's default GitHub branch is not `main_ali`, so the branch must be
selected explicitly:

```bash
(
  set -euo pipefail

  MFJA_WS="$HOME/mfja_ws"
  MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

  mkdir -p "$MFJA_WS/src"
  test ! -e "$MFJA_REPO"

  git clone --branch main_ali --single-branch \
    https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git \
    "$MFJA_REPO"

  test "$(git -C "$MFJA_REPO" branch --show-current)" = main_ali
  git -C "$MFJA_REPO" status --short --branch
)
```

Do not clone over an existing checkout. To update an existing `main_ali`
checkout, preserve local work first, then run `git pull --ff-only` from that
repository.

### A4. Install the Project Dependencies

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy -y

rosdep check --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy
```

The dependency check must finish with `All system dependencies have been
satisfied`.

### A5. Build and Verify the Four Packages

Build from the workspace root. The explicit paths prevent colcon from finding
duplicate packages in unrelated datasets or copied source trees:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"

source "$MFJA_WS/install/setup.bash"

ros2 pkg prefix mfja_3rd_floor_bringup
ros2 pkg prefix mfja_3rd_floor_description
ros2 pkg prefix mfja_rail_interfaces
ros2 pkg prefix mfja_robot_control_config
ros2 interface show mfja_rail_interfaces/msg/ShuttleCommand
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
```

Every package prefix must point inside `$MFJA_WS/install`. On a low-memory
laptop, add `--executor sequential` after `--symlink-install`.

### A6. Start Room 315 (Terminal 1)

This first run disables the heavier robot models, starts Gazebo unpaused, and
creates one stopped right-rail shuttle:

```bash
export MFJA_WS="$HOME/mfja_ws"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  start_paused:=false \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=0 \
  room315_shuttles_start_enabled:=false
```

Leave Terminal 1 running and wait about five seconds for the delayed rail nodes
to start.

### A7. Verify and Move the Shuttle (Terminal 2)

```bash
export MFJA_WS="$HOME/mfja_ws"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"

ros2 topic echo --once /clock
ros2 service list | grep '^/world/room_315_only/'
ros2 topic echo --once /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Start the shuttle and confirm that it moves in Gazebo:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON', speed: 0.2}"

ros2 topic echo --once /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Stop the shuttle with:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'OFF'}"
```

Stop the launch with `Ctrl-C` in Terminal 1.

### A8. Source Every New Terminal

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
```

Source the ROS base first and the workspace overlay second before running any
`ros2` command.

## Method B: Hybrid Nix Setup

Follow N1 through N7 in order. The checked-in `flake.lock` pins Nix Bash,
Ninja, Make, and Git. It does not pin Ubuntu CMake, the compiler, Python,
pkg-config, colcon, or ROS/Gazebo packages installed through apt.

### N1. Install the Required Host Runtime

Complete [A1](#a1-confirm-ubuntu-and-architecture) and
[A2](#a2-install-ros-2-gazebo-and-build-tools-one-time) first. Confirm that the
host compiler/runtime tools used by the flake exist:

```bash
test -f /opt/ros/jazzy/setup.bash
test -x /usr/bin/cmake
test -x /usr/bin/gcc
test -x /usr/bin/g++
test -x /usr/bin/python3
test -x /usr/bin/pkg-config
test -x /usr/bin/colcon
```

### N2. Install Nix (One Time)

The following command downloads and executes the multi-user installer from the
[official Nix download page](https://nixos.org/download/), creates `/nix`, and
configures the Nix daemon:

```bash
sudo apt update
sudo apt install -y ca-certificates curl xz-utils

curl --proto '=https' --tlsv1.2 -L \
  https://nixos.org/nix/install \
  | sh -s -- --daemon
```

Run it as the normal user, not with `sudo`; the installer requests elevated
permission itself. Close the terminal, open a new one, and verify:

```bash
nix --version
```

### N3. Enable Flake Commands (One Time)

```bash
MFJA_NIX_CONF="$HOME/.config/nix/nix.conf"
mkdir -p "$(dirname "$MFJA_NIX_CONF")"
touch "$MFJA_NIX_CONF"

if ! grep -Eq \
  '^[[:space:]]*(extra-)?experimental-features[[:space:]]*=.*nix-command.*flakes' \
  "$MFJA_NIX_CONF"; then
  printf '%s\n' \
    'extra-experimental-features = nix-command flakes' \
    >> "$MFJA_NIX_CONF"
fi

nix flake --help >/dev/null
```

If the configuration cannot be changed, add
`--extra-experimental-features 'nix-command flakes'` to every Nix command.

### N4. Clone `main_ali` and Install ROS Dependencies

Nix does not download or clone the project repository. On a new laptop, create
the checkout explicitly before entering the development shell:

```bash
(
  set -euo pipefail

  MFJA_WS="$HOME/mfja_ws"
  MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

  mkdir -p "$MFJA_WS/src"

  if [ ! -e "$MFJA_REPO" ]; then
    git clone --branch main_ali --single-branch \
      https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git \
      "$MFJA_REPO"
  fi

  test "$(git -C "$MFJA_REPO" rev-parse --is-inside-work-tree)" = true
  test "$(git -C "$MFJA_REPO" branch --show-current)" = main_ali
  test -f "$MFJA_REPO/flake.nix"
  test -f "$MFJA_REPO/flake.lock"
  git -C "$MFJA_REPO" status --short --branch
)
```

After the clone succeeds, complete
[A4](#a4-install-the-project-dependencies). Nix does not replace rosdep.

### N5. Enter and Verify the Development Shell

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
cd "$MFJA_REPO"
nix flake show
nix develop
```

The first entry requires network access and space in `/nix/store`. Once the new
interactive shell opens, verify both sides of the hybrid environment:

```bash
(
  set -euo pipefail

  test "$MFJA_NIX_MODE" = hybrid
  test "$ROS_DISTRO" = jazzy
  test -x /usr/bin/colcon

  test "$CC" = /usr/bin/gcc
  test "$CXX" = /usr/bin/g++
  test "$PYTHON" = /usr/bin/python3
  test "$PKG_CONFIG" = /usr/bin/pkg-config
  test "$(command -v cmake)" = /usr/bin/cmake
  test "$(command -v gcc)" = /usr/bin/gcc
  test "$(command -v g++)" = /usr/bin/g++
  test "$(command -v python3)" = /usr/bin/python3
  test "$(command -v pkg-config)" = /usr/bin/pkg-config
  test "$(command -v colcon)" = /usr/bin/colcon
  test -z "${NIX_CFLAGS_COMPILE:-}"
  test -z "${NIXPKGS_CMAKE_PREFIX_PATH:-}"

  MFJA_CMAKE_VERSION="$(cmake --version | awk 'NR == 1 { print $3 }')"
  test -n "$MFJA_CMAKE_VERSION"
  /usr/bin/dpkg --compare-versions "$MFJA_CMAKE_VERSION" ge 3.28
  printf 'Ubuntu CMake version: %s\n' "$MFJA_CMAKE_VERSION"
  unset MFJA_CMAKE_VERSION

  for MFJA_NIX_TOOL in ninja make git; do
    case "$(command -v "$MFJA_NIX_TOOL")" in
      /nix/store/*/bin/*) ;;
      *) printf 'ERROR: %s is not coming from Nix.\n' \
           "$MFJA_NIX_TOOL" >&2; false ;;
    esac
  done
  unset MFJA_NIX_TOOL
  command -v ros2
  command -v gz

  "$PKG_CONFIG" --exists uuid
  "$PKG_CONFIG" --atleast-version=4 libzmq
  test -f /usr/include/zmq.hpp
  test -f /usr/include/python3.12/Python.h
  test -f "/usr/include/$(/usr/bin/gcc -print-multiarch)/python3.12/pyconfig.h"

  printf '%s\n' \
    '#include <cstdlib>' \
    '#include <chrono>' \
    '#include <sys/types.h>' \
    '#include <Python.h>' \
    '#include <uuid/uuid.h>' \
    'int main() { time_t value{}; (void)value; return 0; }' | \
    "$CXX" -std=c++17 -x c++ -fsyntax-only \
      -isystem /usr/include/python3.12 -
)
```

CMake, `CC`, `CXX`, `PYTHON`, and `PKG_CONFIG` must resolve to the exact
`/usr/bin` tools shown above, and Ubuntu CMake must be version 3.28 or newer.
The shell removes inherited Nix discovery/compiler flags and old ROS overlays
before sourcing Jazzy. Nix remains responsible only for Bash, Ninja, Make, and
Git. The final compile-only smoke test deliberately combines C++, glibc,
Python, and UUID headers; success confirms they all come from the compatible
Ubuntu toolchain.

### N6. Build and Verify Inside Nix

Stay inside `nix develop`:

```bash
cd "$MFJA_WS"

if colcon build --symlink-install --paths \
    "$MFJA_REPO/mfja_rail_interfaces" \
    "$MFJA_REPO/mfja_3rd_floor_description" \
    "$MFJA_REPO/mfja_robot_control_config" \
    "$MFJA_REPO/mfja_3rd_floor_bringup" \
    --cmake-args \
      -DCMAKE_C_COMPILER=/usr/bin/gcc \
      -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
      -DPython3_EXECUTABLE=/usr/bin/python3; then
  source "$MFJA_WS/install/setup.bash"

  ros2 pkg prefix mfja_3rd_floor_bringup
  ros2 pkg prefix mfja_3rd_floor_description
  ros2 pkg prefix mfja_rail_interfaces
  ros2 pkg prefix mfja_robot_control_config
  ros2 interface show mfja_rail_interfaces/msg/ShuttleCommand
  ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
else
  printf '%s\n' \
    'ERROR: colcon build failed; the install overlay was not sourced and ROS checks were skipped.' \
    >&2
  false
fi
```

### N7. Run from Nix and Open Later Terminals

For Terminal 1 and every later terminal, enter the Nix shell first:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
cd "$MFJA_REPO"
nix develop
```

Then, inside the Nix shell, source the overlay and use the launch/verification
commands from [A6](#a6-start-room-315-terminal-1) and
[A7](#a7-verify-and-move-the-shuttle-terminal-2):

```bash
source "$MFJA_WS/install/setup.bash"
```

Run `exit` when you want to leave the interactive Nix shell.

## Installation Troubleshooting

- `Package ... not found`: source `/opt/ros/jazzy/setup.bash`, then the intended
  workspace's `install/setup.bash` in the same terminal.
- Colcon reports a duplicate package: keep copied source trees and datasets
  outside workspace `src/`, and use the four explicit `--paths` above instead
  of a broad `--base-paths` scan.
- A low-memory build is killed: rebuild with `--executor sequential`.
- `nix: command not found` after installation: close the terminal and open a
  new one so the Nix profile is loaded.
- Nix says flakes are disabled: repeat N3 or pass the temporary experimental
  feature flag documented there.
- The Nix shell cannot find ROS or colcon: return to N1; the flake intentionally
  does not supply those host packages.
- CMake reports a missing `UUID`, `ZeroMQ`, or another Gazebo dependency:
  repeat A4 and the `$PKG_CONFIG` checks in N5. Install the missing dependency
  through Ubuntu/rosdep; do not add a second Nix copy of an Ubuntu Gazebo
  runtime library.
- Compilation cannot find `<multiarch>/python3.12/pyconfig.h`: rerun the Python
  header checks in the Nix verification step. If either file is absent,
  install/reinstall `python3-dev` and `libpython3.12-dev`, then enter a new Nix
  shell and repeat the mixed-header smoke test.
- Compilation reports `__time64_t does not name a type`, errors inside both
  `/nix/store/.../glibc...` and `/usr/include/...`, or a cached Nix compiler:
  update the branch, exit the old shell, enter `nix develop` again, and run the
  following clean transition build once. It clears CMake's old compiler cache
  and old objects without deleting the checkout or `/nix/store`:

  ```bash
  cd "$MFJA_WS"

  colcon build \
    --symlink-install \
    --executor sequential \
    --cmake-clean-cache \
    --cmake-clean-first \
    --paths \
    "$MFJA_REPO/mfja_rail_interfaces" \
    "$MFJA_REPO/mfja_3rd_floor_description" \
    "$MFJA_REPO/mfja_robot_control_config" \
    "$MFJA_REPO/mfja_3rd_floor_bringup" \
    --cmake-args \
      -DCMAKE_C_COMPILER=/usr/bin/gcc \
      -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
      -DPython3_EXECUTABLE=/usr/bin/python3
  ```

  After this succeeds, use the normal N6 build command for later changes.
- A new terminal or `nix develop` prints a missing workspace path such as
  `bash: /home/tiago/sri2_g2_hela_ws/install/setup.bash: No such file or
  directory`: a shell startup file contains a stale `source` command. Follow
  [Remove a stale workspace source](#remove-a-stale-workspace-source) below;
  cloning or rebuilding this repository does not repair an unrelated path.
- Gazebo opens with a rendering error: verify `DISPLAY`, OpenGL acceleration,
  and the graphics driver. `gui:=false` disables the client, although sensor
  rendering can still require a working EGL/headless backend.
- No shuttle is visible: initial counts default to zero; pass a shuttle count as
  shown in A6.
- A shuttle is visible but does not move: set `start_paused:=false`, then publish
  the `ON` command from A7.

### Remove a Stale Workspace Source

First locate startup lines that automatically source a workspace overlay. This
check is read-only and tolerates startup files that do not exist:

```bash
grep -nHE \
  '^[[:space:]]*(source|\.)[[:space:]].*/install/(local_)?setup\.bash' \
  "$HOME/.bashrc" "$HOME/.bash_aliases" "$HOME/.bash_profile" \
  "$HOME/.bash_login" "$HOME/.profile" \
  2>/dev/null || true
```

Choose the file reported by `grep`, back it up, and open it. Replace `.bashrc`
below if the stale line is in a different startup file:

```bash
MFJA_STARTUP_FILE="$HOME/.bashrc"
test -f "$MFJA_STARTUP_FILE"
MFJA_STARTUP_BACKUP="${MFJA_STARTUP_FILE}.mfja-backup.$(date +%Y%m%d-%H%M%S)"

cp -a -- "$MFJA_STARTUP_FILE" "$MFJA_STARTUP_BACKUP"
printf 'Backup: %s\n' "$MFJA_STARTUP_BACKUP"
"${EDITOR:-nano}" "$MFJA_STARTUP_FILE"
```

Comment out the stale line. If that old workspace is still intentionally used,
guard it instead so a missing checkout does not break every new shell:

```bash
# Preferred for an obsolete workspace:
# source "$HOME/sri2_g2_hela_ws/install/setup.bash"

# Alternative only when the old workspace is still intentionally used:
MFJA_LEGACY_SETUP="$HOME/sri2_g2_hela_ws/install/setup.bash"
if [ -f "$MFJA_LEGACY_SETUP" ]; then
  source "$MFJA_LEGACY_SETUP"
fi
unset MFJA_LEGACY_SETUP
```

Validate the edited file before opening another terminal:

```bash
if ! bash -n "$MFJA_STARTUP_FILE"; then
  cp -a -- "$MFJA_STARTUP_BACKUP" "$MFJA_STARTUP_FILE"
  printf 'Syntax error: restored %s from the backup.\n' "$MFJA_STARTUP_FILE" >&2
  false
fi
```

Do not automatically source any colcon workspace's `install/setup.bash` from
`.bashrc` or another login file. Workspace overlays are order-sensitive and
become stale when a directory is renamed or removed. Enter `nix develop` first,
then source only the intended `$MFJA_WS/install/setup.bash` explicitly after a
successful build, as shown in N6 and N7.

---

## Basic Commands and Quick Start

The repository offers multiple run modes depending on what you want to test.

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
If you only want to focus on the flexible rail system and shuttles in Room 315:
```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  start_paused:=false \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

### 3. Launching a Single Industrial Robot
For isolated testing of a specific robotic arm (e.g., KUKA) without the rest of the floor:
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

---

## 📚 Detailed Documentation

For a deep dive into advanced features, please refer to our dedicated documentation files:

*   **[Detailed Feature & API Guide (DETAILED_GUIDE.md)](DETAILED_GUIDE.md)**: Includes step-by-step guides for adding shuttles dynamically, reading sensor feedback, testing industrial robots, and troubleshooting.
*   **[Room 315 Kinematic Rail Network Specs](mfja_robot_control_config/config/room_315_kinematics/README.md)**: Technical details about segment directions, device YAMLs, and sensor cookbook testing.
*   **[HTML Runbook](runbook.html)**: A focused visualization and operational guide.
