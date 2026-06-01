# Installation and Workspace Setup

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
