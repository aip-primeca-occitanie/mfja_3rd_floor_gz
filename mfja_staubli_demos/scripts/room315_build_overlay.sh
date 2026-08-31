#!/usr/bin/env bash
# Build MFJA as a standard colcon overlay on an installed HPP/ROS underlay.
set -euo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(cd -- "$(dirname -- "$script_path")" && pwd -P)
mfja_root=$(cd -- "$script_dir/../.." && pwd -P)
devel_root=$(dirname "$mfja_root")
workspace=${MFJA_WS:-$devel_root/mfja_ws}
hpp_setup=${HPP_SETUP:-$devel_root/hpp/install/setup.bash}
install_pip_dir=${INSTALL_PIP_DIR:-}
robotpkg=${ROBOTPKG:-/opt/openrobots}
ros_setup=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
staubli_source=${STAUBLI_ROS2_SOURCE:-$mfja_root/Staubli_ROS2}
build_base=${MFJA_BUILD_BASE:-$workspace/build}
install_base=${MFJA_INSTALL_BASE:-$workspace/install}
log_base=${MFJA_LOG_BASE:-$workspace/log}

if [[ "${ROOM315_OVERLAY_BUILD_ENV:-}" != "1" ]]; then
  exec /usr/bin/env -i \
    HOME="$HOME" \
    USER="${USER:-$(id -un)}" \
    LOGNAME="${LOGNAME:-${USER:-$(id -un)}}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    HPP_SETUP="$hpp_setup" \
    INSTALL_PIP_DIR="$install_pip_dir" \
    MFJA_BUILD_BASE="$build_base" \
    MFJA_INSTALL_BASE="$install_base" \
    MFJA_LOG_BASE="$log_base" \
    MFJA_WS="$workspace" \
    ROBOTPKG="$robotpkg" \
    ROS_SETUP="$ros_setup" \
    STAUBLI_ROS2_SOURCE="$staubli_source" \
    CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}" \
    ROOM315_OVERLAY_BUILD_ENV=1 \
    /bin/bash "$script_path" "$@"
fi

if [[ ! -f "$hpp_setup" ]]; then
  echo "HPP environment setup not found at $hpp_setup." >&2
  echo "Set HPP_SETUP to any compatible HPP/ROS setup.bash." >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090,SC1091
source "$hpp_setup"
set -u
export PYTHONNOUSERSITE=1

python3 - <<'PY'
import sys

import hpp_exec
import pyhpp
import pyhpp_toppra
import rclpy
from pyhpp.manipulation import Device

print(f"Overlay ABI: Python {sys.version.split()[0]}")
print(f"pyhpp: {pyhpp.__file__}")
print(f"pyhpp_toppra: {pyhpp_toppra.__file__}")
print(f"hpp-exec: {hpp_exec.__file__}")
print(f"rclpy: {rclpy.__file__}")
PY

mkdir -p "$workspace/src"
workspace_link="$workspace/src/mfja_3rd_floor_gz"
if [[ ! -e "$workspace_link" && ! -L "$workspace_link" ]]; then
  ln -s "$mfja_root" "$workspace_link"
fi
workspace_source=$(readlink -f "$workspace_link")
if [[ "$workspace_source" != "$mfja_root" ]]; then
  echo "$workspace_link does not resolve to $mfja_root" >&2
  exit 1
fi

base_paths=(
  "$mfja_root/mfja_rail_interfaces"
  "$mfja_root/mfja_3rd_floor_description"
  "$mfja_root/mfja_robot_control_config"
  "$mfja_root/mfja_3rd_floor_bringup"
  "$mfja_root/mfja_staubli_demos"
  "$mfja_root/mfja_staubli_manipulation_demos"
)
packages=(
  mfja_rail_interfaces
  mfja_3rd_floor_description
  mfja_robot_control_config
  mfja_3rd_floor_bringup
  mfja_staubli_demos
  mfja_staubli_manipulation_demos
)

staubli_packages=(
  industrial_msgs
  industrial_robot_client
  industrial_utils
  simple_message
  staubli_msgs
  staubli_tx2_60l_description
  staubli_val3_driver
  urdf_extention
)
staubli_package_dirs=(
  industrial_msgs
  industrial_robot_client
  industrial_utils
  simple_message
  staubli_msgs
  staubli_tx2_60l_description
  staubli_val3_driver
  urdf_extention
)

missing_staubli_sources=()
for package_dir in "${staubli_package_dirs[@]}"; do
  if [[ ! -f "$staubli_source/$package_dir/package.xml" ]]; then
    missing_staubli_sources+=("$package_dir/package.xml")
  fi
done

if [[ ${#missing_staubli_sources[@]} -eq 0 ]]; then
  base_paths=(
    "$staubli_source"
    "${base_paths[@]}"
  )
  packages=("${staubli_packages[@]}" "${packages[@]}")
elif [[ -d "$staubli_source" ]]; then
  echo "The Staubli_ROS2 checkout at $staubli_source is incomplete:" >&2
  printf '  %s\n' "${missing_staubli_sources[@]}" >&2
  exit 1
else
  required_installed_staubli_packages=(
    industrial_msgs
    industrial_robot_client
    industrial_utils
    simple_message
    staubli_msgs
    staubli_tx2_60l_description
    staubli_val3_driver
    urdf_extention
  )
  missing_installed_staubli_packages=()
  for package in "${required_installed_staubli_packages[@]}"; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
      missing_installed_staubli_packages+=("$package")
    fi
  done
  if [[ ${#missing_installed_staubli_packages[@]} -gt 0 ]]; then
    echo "The Staubli source checkout is unavailable at $staubli_source," >&2
    echo "and these real-robot packages are not installed:" >&2
    printf '  %s\n' "${missing_installed_staubli_packages[@]}" >&2
    echo "Set STAUBLI_ROS2_SOURCE to a complete Staubli_ROS2 checkout." >&2
    exit 1
  fi
fi

if [[ -f "$install_base/.colcon_install_layout" ]] \
  && [[ "$(<"$install_base/.colcon_install_layout")" != "merged" ]]; then
  echo "$install_base uses a non-merged layout; preserve it before rebuilding." >&2
  exit 1
fi

obsolete_staubli_packages=(
  motion_control_msgs
  moveit_interface
  robot_middleware
  staubli_support
  staubli_tx2_60l_moveit_config
)
stale_staubli_packages=()
for package in "${obsolete_staubli_packages[@]}"; do
  marker="$install_base/share/ament_index/resource_index/packages/$package"
  if [[ -e "$marker" || -L "$marker" ]]; then
    stale_staubli_packages+=("$package")
  fi
done
if [[ ${#stale_staubli_packages[@]} -gt 0 ]]; then
  echo "$install_base contains packages removed from the lean overlay:" >&2
  printf '  %s\n' "${stale_staubli_packages[@]}" >&2
  echo "Preserve that generated install and rebuild into an empty install base." >&2
  echo "Alternatively, set MFJA_INSTALL_BASE to a clean directory." >&2
  exit 1
fi

legacy_manipulation_artifacts=(
  "$install_base/lib/mfja_staubli_manipulation_demos/room315_demo.sh"
  "$install_base/lib/mfja_staubli_manipulation_demos/room315_hpp_manipulation.sh"
  "$install_base/lib/mfja_staubli_manipulation_demos/room315_manipulation_demo.sh"
  "$install_base/lib/mfja_staubli_manipulation_demos/room315_moving_shuttle_demo.sh"
  "$install_base/share/mfja_staubli_manipulation_demos/launch/room_315_staubli_shuttle_manipulation_demo.launch.py"
  "$install_base/share/mfja_staubli_manipulation_demos/models/room315_pick_support.sdf"
  "$install_base/share/mfja_staubli_manipulation_demos/hpp/room315_shuttle_manipulation.py"
  "$install_base/share/mfja_staubli_manipulation_demos/hpp/room315_shuttle_deck.urdf"
  "$install_base/share/mfja_staubli_manipulation_demos/hpp/room315_shuttle_deck.srdf"
)
for artifact in "${legacy_manipulation_artifacts[@]}"; do
  if [[ -e "$artifact" || -L "$artifact" ]]; then
    echo "$install_base contains obsolete manipulation files." >&2
    echo "Preserve that generated install and rebuild into an empty install base." >&2
    exit 1
  fi
done

mkdir -p "$build_base" "$install_base" "$log_base"

export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-1}
export MAKEFLAGS="-j$CMAKE_BUILD_PARALLEL_LEVEL"
python_executable=$(command -v python3)
colcon --log-base "$log_base" build \
  --base-paths "${base_paths[@]}" \
  --build-base "$build_base" \
  --install-base "$install_base" \
  --merge-install \
  --symlink-install \
  --executor sequential \
  --cmake-force-configure \
  --packages-select "${packages[@]}" \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DPYTHON_EXECUTABLE="$python_executable" \
    -DPython3_EXECUTABLE="$python_executable"

set +u
# shellcheck disable=SC1090,SC1091
source "$install_base/setup.bash"
set -u
for package in \
  industrial_robot_client \
  staubli_tx2_60l_description \
  staubli_val3_driver; do
  ros2 pkg prefix "$package" >/dev/null
done
python3 - "$install_base" <<'PY'
import sys
from pathlib import Path

import hpp_exec
import pyhpp
import pyhpp_toppra
import rclpy
from ament_index_python.packages import get_package_share_path
from pyhpp.manipulation import Device

prefix = Path(sys.argv[1]).resolve()
model = (
    get_package_share_path("mfja_staubli_manipulation_demos")
    / "models"
    / "room315_payload_box.sdf"
)
if not model.is_file():
    raise SystemExit(f"installed payload model is missing: {model}")
print(f"MFJA overlay ready: {prefix}")
print(f"Python: {sys.version.split()[0]}")
print(f"payload model: {model}")
PY
ros2 run mfja_staubli_manipulation_demos \
  room315_pick_place.sh --help >/dev/null

echo "Run 'source $install_base/setup.bash' to use MFJA."
