#!/usr/bin/env bash
# Build the local HPP sources as a ROS Jazzy/Python 3.12 colcon underlay.
set -euo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(cd -- "$(dirname -- "$script_path")" && pwd -P)
mfja_root=$(cd -- "$script_dir/../.." && pwd -P)
devel_root=$(dirname "$mfja_root")
hpp_source_root=${HPP_SOURCE_ROOT:-$devel_root/hpp_sources}
hpp_exec_source=${HPP_EXEC_SOURCE:-$hpp_source_root/hpp-exec}
hpp_workspace=${HPP_WS:-$devel_root/hpp_ws}
ros_setup=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}

if [[ "${ROOM315_HOST_BUILD_ENV:-}" != "1" ]]; then
  exec /usr/bin/env -i \
    HOME="$HOME" \
    USER="${USER:-$(id -un)}" \
    LOGNAME="${LOGNAME:-${USER:-$(id -un)}}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    HPP_EXEC_SOURCE="$hpp_exec_source" \
    HPP_SOURCE_ROOT="$hpp_source_root" \
    HPP_WS="$hpp_workspace" \
    ROS_SETUP="$ros_setup" \
    CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}" \
    ROOM315_HOST_BUILD_ENV=1 \
    /bin/bash "$script_path" "$@"
fi

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found at $ros_setup; set ROS_SETUP." >&2
  exit 1
fi
# shellcheck disable=SC1090
set +u
source "$ros_setup"
set -u

required_debian_packages=(
  ros-jazzy-coal
  ros-jazzy-jrl-cmakemodules
  ros-jazzy-pinocchio
  ros-jazzy-proxsuite
)
missing_packages=()
for package in "${required_debian_packages[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null \
    | grep -Fq 'install ok installed'; then
    missing_packages+=("$package")
  fi
done
if ((${#missing_packages[@]})); then
  echo "Install the missing Jazzy HPP dependencies once:" >&2
  printf '  sudo apt install' >&2
  printf ' %q' "${required_debian_packages[@]}" >&2
  printf '\n' >&2
  exit 2
fi

python3 - <<'PY'
import sys

import coal
import eigenpy
import pinocchio
import rclpy

if sys.version_info[:2] != (3, 12):
    raise SystemExit(f"ROS Jazzy host build requires Python 3.12, got {sys.version}")
major = int(pinocchio.__version__.split(".", 1)[0])
if major < 4:
    raise SystemExit(
        f"Pinocchio 4 is required with Coal 3, got Pinocchio {pinocchio.__version__}"
    )
print(
    f"Host ABI: Python {sys.version.split()[0]}, EigenPy {eigenpy.__version__}, "
    f"Coal {coal.__version__}, Pinocchio {pinocchio.__version__}"
)
PY

source_directories=(
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

mkdir -p "$hpp_workspace/src"
for directory in "${source_directories[@]}"; do
  source_candidate="$hpp_source_root/$directory"
  if [[ "$directory" == "hpp-exec" ]]; then
    source_candidate="$hpp_exec_source"
  fi
  if [[ ! -d "$source_candidate" ]]; then
    echo "HPP source package not found at $source_candidate" >&2
    exit 1
  fi
  source_path=$(readlink -f "$source_candidate")
  link_path="$hpp_workspace/src/$directory"
  if [[ ! -f "$source_path/package.xml" ]]; then
    echo "HPP source package not found at $source_path" >&2
    exit 1
  fi
  if [[ -L "$link_path" ]]; then
    if [[ "$(readlink -f "$link_path")" != "$source_path" ]]; then
      echo "$link_path points somewhere else; preserve or correct it first." >&2
      exit 1
    fi
  elif [[ -e "$link_path" ]]; then
    echo "$link_path already exists and is not a symlink." >&2
    exit 1
  else
    ln -s "$source_path" "$link_path"
  fi
done

build_base="$hpp_workspace/build"
install_base="$hpp_workspace/install"
log_base="$hpp_workspace/log"
mkdir -p "$build_base" "$install_base" "$log_base"

export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-1}
export MAKEFLAGS="-j$CMAKE_BUILD_PARALLEL_LEVEL"
export PYTHONNOUSERSITE=1
python_executable=$(command -v python3)
base_paths=("${source_directories[@]/#/$hpp_workspace/src/}")
cmake_args=(
  -DBUILD_PYTHON_INTERFACE=OFF
  -DBUILD_TESTING=OFF
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
  -DGENERATE_PYTHON_STUBS=OFF
  -DINSTALL_DOCUMENTATION=OFF
  -DPYTHON_EXECUTABLE="$python_executable"
  -DPython3_EXECUTABLE="$python_executable"
  -DPYTHON_STANDARD_LAYOUT=ON
  -DUSE_QPOASES=OFF
)

build_stage() {
  colcon --log-base "$log_base" build \
    --base-paths "${base_paths[@]}" \
    --build-base "$build_base" \
    --install-base "$install_base" \
    --merge-install \
    --executor sequential \
    --cmake-force-configure \
    --packages-select "$@" \
    --cmake-args "${cmake_args[@]}"

  # Colcon cannot infer every HPP dependency because older package manifests
  # mix hyphenated and underscored names. Expose each completed stage to CMake
  # before configuring the next one.
  # shellcheck disable=SC1090
  set +u
  source "$install_base/local_setup.bash"
  set -u
}

build_stage example-robot-data
build_stage hpp-environments hpp-util
build_stage hpp_pinocchio hpp-statistics
build_stage hpp-constraints
build_stage hpp_core
build_stage hpp_manipulation
build_stage hpp-manipulation-urdf
build_stage hpp_python
build_stage hpp_gepetto_viewer
build_stage hpp-exec

# shellcheck disable=SC1090
set +u
source "$install_base/setup.bash"
set -u
python3 - "$install_base" "$hpp_exec_source" <<'PY'
import sys
from pathlib import Path

import coal
import hpp_exec
import pyhpp
sys.modules.setdefault("hppfcl", coal)
import pyhpp_viser
import rclpy
from pyhpp.manipulation import Device

prefix = Path(sys.argv[1]).resolve()
source = Path(sys.argv[2]).resolve()
if sys.version_info[:2] != (3, 12):
    raise SystemExit(f"unexpected runtime Python: {sys.version}")
for module in (hpp_exec, pyhpp, pyhpp_viser):
    path = Path(module.__file__).resolve()
    if not path.is_relative_to(prefix):
        raise SystemExit(f"{module.__name__} is outside {prefix}: {path}")
installed = Path(hpp_exec.__file__).resolve().parent
for name in ("__init__.py", "joint_state.py"):
    if (source / "hpp_exec" / name).read_bytes() != (installed / name).read_bytes():
        raise SystemExit(f"installed hpp-exec {name} does not match {source}")
print(f"HPP underlay ready: {prefix}")
print(f"Python: {sys.version.split()[0]}")
print(f"pyhpp: {Path(pyhpp.__file__).resolve()}")
print(f"hpp-exec: {Path(hpp_exec.__file__).resolve()}")
print(f"rclpy: {Path(rclpy.__file__).resolve()}")
PY

echo "After editing hpp-exec, rerun this script to reinstall the changes."
