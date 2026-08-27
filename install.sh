#!/usr/bin/env bash
# Install the Room 315 Staubli pick-and-place in adjacent workspaces.
set -euo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
mfja_root=$(cd -- "$(dirname -- "$script_path")" && pwd -P)
work_dir=${1:-${MFJA_WORK_DIR:-$(dirname "$mfja_root")}}
work_dir=$(readlink -m "$work_dir")
ros_setup=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
hpp_source_root="$work_dir/hpp_sources"
hpp_workspace="$work_dir/hpp_ws"
mfja_workspace="$work_dir/mfja_ws"
venv="$work_dir/.venv"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Jazzy was not found at $ros_setup." >&2
  echo "Install the packages listed in README.md, or set ROS_SETUP." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "$ros_setup"
set -u

missing_commands=()
for command in colcon git gz python3 ros2 vcs; do
  if ! command -v "$command" >/dev/null 2>&1; then
    missing_commands+=("$command")
  fi
done
if ((${#missing_commands[@]})); then
  echo "Missing install tools: ${missing_commands[*]}" >&2
  echo "Install the Ubuntu packages listed in README.md." >&2
  exit 1
fi

if [[ "$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" != "3.12" ]]; then
  echo "ROS 2 Jazzy and this HPP build require Python 3.12." >&2
  exit 1
fi

mkdir -p "$work_dir" "$hpp_source_root"
git -C "$mfja_root" submodule update --init --recursive

required_driver_file="$mfja_root/Staubli_ROS2/staubli_val3_driver/config/tx2_60l_streaming.yaml"
if [[ ! -f "$required_driver_file" ]]; then
  echo "The pinned Staubli_ROS2 revision lacks the MoveIt-free driver configuration:" >&2
  echo "  $required_driver_file" >&2
  echo "Publish and pin the commissioned driver revision before installing on another machine." >&2
  exit 1
fi

required_hpp_sources=(
  example-robot-data
  hpp-environments
  hpp-util
  hpp-pinocchio
  hpp-statistics
  hpp-constraints
  hpp-core
  hpp-manipulation
  hpp-manipulation-urdf
  hpp-python
  hpp-gepetto-viewer
  hpp-exec
)
missing_hpp_sources=()
for directory in "${required_hpp_sources[@]}"; do
  if [[ ! -f "$hpp_source_root/$directory/package.xml" ]]; then
    missing_hpp_sources+=("$directory")
  fi
done
if ((${#missing_hpp_sources[@]})); then
  vcs import "$hpp_source_root" < "$mfja_root/hpp_jazzy.repos"
fi
for directory in "${required_hpp_sources[@]}"; do
  if [[ ! -f "$hpp_source_root/$directory/package.xml" ]]; then
    echo "HPP source import is incomplete: $hpp_source_root/$directory" >&2
    exit 1
  fi
done

HPP_SOURCE_ROOT="$hpp_source_root" \
HPP_EXEC_SOURCE="$hpp_source_root/hpp-exec" \
HPP_WS="$hpp_workspace" \
ROS_SETUP="$ros_setup" \
  "$mfja_root/mfja_staubli_demos/scripts/room315_build_hpp_underlay.sh"

MFJA_WS="$mfja_workspace" \
HPP_SETUP="$hpp_workspace/install/setup.bash" \
STAUBLI_ROS2_SOURCE="$mfja_root/Staubli_ROS2" \
  "$mfja_root/mfja_staubli_demos/scripts/room315_build_overlay.sh"

if [[ ! -x "$venv/bin/python" ]]; then
  python3 -m venv --system-site-packages "$venv"
fi
PYTHONNOUSERSITE=1 "$venv/bin/python" -m pip install \
  --disable-pip-version-check \
  --requirement "$mfja_root/requirements-viser.txt"

cp "$mfja_root/setup.bash.in" "$work_dir/setup.bash"
# shellcheck disable=SC1090
set +u
source "$work_dir/setup.bash"
set -u
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh

printf '\nInstallation complete. In each terminal, run:\n'
printf '  source %q\n' "$work_dir/setup.bash"
