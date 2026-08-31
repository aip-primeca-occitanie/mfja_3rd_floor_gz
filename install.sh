#!/usr/bin/env bash
# Install the Room 315 Staubli pick-and-place in adjacent workspaces.
set -euo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
mfja_root=$(cd -- "$(dirname -- "$script_path")" && pwd -P)
work_dir=${1:-${MFJA_WORK_DIR:-$(dirname "$mfja_root")}}
work_dir=$(readlink -m "$work_dir")
ros_setup=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
hpp_devel="$work_dir/hpp"
hpp_source_root="$hpp_devel/src"
hpp_build_root="$hpp_devel/build"
hpp_install="$hpp_devel/install"
mfja_workspace="$work_dir/mfja_ws"
venv="$work_dir/.venv"
robotpkg=/opt/openrobots
required_hpp_version=9.0.2

platform=$(
  # shellcheck disable=SC1091
  source /etc/os-release
  printf '%s:%s' "${ID:-}" "${VERSION_ID:-}"
)
if [[ "$platform" != "ubuntu:24.04" ]]; then
  echo "This installer requires Ubuntu 24.04; found $platform." >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "The Robotpkg Noble packages used here require amd64." >&2
  exit 1
fi

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Jazzy was not found at $ros_setup." >&2
  echo "Install the packages listed in README.md, or set ROS_SETUP." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090,SC1091
source "$ros_setup"
set -u

missing_commands=()
for command in cmake colcon git gz python3 ros2 vcs; do
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
  echo "ROS 2 Jazzy and Robotpkg HPP require Python 3.12." >&2
  exit 1
fi

required_robotpkg_packages=(
  robotpkg-py312-hpp-python
  robotpkg-py312-qt5-hpp-gepetto-viewer
)
invalid_robotpkg_packages=()
for package in "${required_robotpkg_packages[@]}"; do
  installed_record=$(
    dpkg-query -W -f='${Status}|${Version}' "$package" 2>/dev/null || true
  )
  if [[ "$installed_record" != "install ok installed|$required_hpp_version" ]]; then
    invalid_robotpkg_packages+=("$package=$required_hpp_version")
  fi
done
if ((${#invalid_robotpkg_packages[@]})); then
  echo "Install the pinned Robotpkg HPP packages listed in README.md:" >&2
  printf '  sudo apt install -y' >&2
  printf ' %q' "${invalid_robotpkg_packages[@]}" >&2
  printf '\n' >&2
  exit 2
fi
if [[ ! -d "$robotpkg" ]]; then
  echo "Robotpkg prefix not found at $robotpkg." >&2
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
  toppra
  hpp-toppra
  hpp-exec
)
required_source_files=(
  cpp/CMakeLists.txt
  package.xml
  package.xml
)
required_source_commits=(
  2dfb9729d2ba2bbc962d512cc3a73b264a5ab466
  83efa3beb861132484364c46d28b4ee642830c10
  d8e7c5a38e073c919326c6e55a358eee5db6751d
)
import_sources=0
for directory in "${required_hpp_sources[@]}"; do
  source_directory="$hpp_source_root/$directory"
  if [[ ! -d "$source_directory/.git" ]]; then
    if [[ -e "$source_directory" ]]; then
      echo "$source_directory exists and is not a Git checkout." >&2
      exit 1
    fi
    import_sources=1
  fi
done
if ((import_sources)); then
  vcs import --skip-existing "$hpp_source_root" < "$mfja_root/hpp_jazzy.repos"
fi

for index in "${!required_hpp_sources[@]}"; do
  directory=${required_hpp_sources[$index]}
  source_directory="$hpp_source_root/$directory"
  required_file=${required_source_files[$index]}
  expected_commit=${required_source_commits[$index]}
  if [[ ! -f "$source_directory/$required_file" ]]; then
    echo "HPP source import is incomplete: $source_directory/$required_file" >&2
    exit 1
  fi
  actual_commit=$(git -C "$source_directory" rev-parse HEAD)
  if [[ "$actual_commit" != "$expected_commit" ]]; then
    echo "$source_directory is at $actual_commit; expected $expected_commit." >&2
    echo "Preserve or move that checkout before reinstalling." >&2
    exit 1
  fi
  if [[ -n "$(git -C "$source_directory" status --porcelain)" ]]; then
    echo "$source_directory has local changes; preserve them before reinstalling." >&2
    exit 1
  fi
done

if [[ ! -x "$venv/bin/python" ]]; then
  python3 -m venv --system-site-packages "$venv"
fi
PYTHONNOUSERSITE=1 "$venv/bin/python" -m pip install \
  --disable-pip-version-check \
  --requirement "$mfja_root/requirements-viser.txt"

DEVEL_HPP_DIR="$hpp_devel" \
HPP_SOURCE_ROOT="$hpp_source_root" \
HPP_BUILD_DIR="$hpp_build_root" \
INSTALL_HPP_DIR="$hpp_install" \
INSTALL_PIP_DIR="$venv" \
ROBOTPKG="$robotpkg" \
ROS_SETUP="$ros_setup" \
  "$mfja_root/mfja_staubli_demos/scripts/room315_build_hpp_underlay.sh"

MFJA_WS="$mfja_workspace" \
HPP_SETUP="$hpp_install/setup.bash" \
INSTALL_PIP_DIR="$venv" \
ROBOTPKG="$robotpkg" \
ROS_SETUP="$ros_setup" \
STAUBLI_ROS2_SOURCE="$mfja_root/Staubli_ROS2" \
  "$mfja_root/mfja_staubli_demos/scripts/room315_build_overlay.sh"

cp "$mfja_root/setup.bash.in" "$work_dir/setup.bash"
unset HPP_SETUP
set +u
# shellcheck disable=SC1090,SC1091
source "$work_dir/setup.bash"
set -u
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh

printf '\nInstallation complete. In each terminal, run:\n'
printf '  source %q\n' "$work_dir/setup.bash"
