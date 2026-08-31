#!/usr/bin/env bash
# Build the source-only HPP additions on the Robotpkg HPP installation.
set -euo pipefail

script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(cd -- "$(dirname -- "$script_path")" && pwd -P)
mfja_root=$(cd -- "$script_dir/../.." && pwd -P)
devel_root=$(dirname "$mfja_root")
hpp_devel=${DEVEL_HPP_DIR:-$devel_root/hpp}
hpp_source_root=${HPP_SOURCE_ROOT:-$hpp_devel/src}
hpp_build_root=${HPP_BUILD_DIR:-$hpp_devel/build}
hpp_install=${INSTALL_HPP_DIR:-$hpp_devel/install}
install_pip_dir=${INSTALL_PIP_DIR:-}
robotpkg=${ROBOTPKG:-/opt/openrobots}
ros_setup=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}

if [[ "${ROOM315_HOST_BUILD_ENV:-}" != "1" ]]; then
  exec /usr/bin/env -i \
    HOME="$HOME" \
    USER="${USER:-$(id -un)}" \
    LOGNAME="${LOGNAME:-${USER:-$(id -un)}}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    DEVEL_HPP_DIR="$hpp_devel" \
    HPP_BUILD_DIR="$hpp_build_root" \
    HPP_SOURCE_ROOT="$hpp_source_root" \
    INSTALL_HPP_DIR="$hpp_install" \
    INSTALL_PIP_DIR="$install_pip_dir" \
    ROBOTPKG="$robotpkg" \
    ROS_SETUP="$ros_setup" \
    CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}" \
    ROOM315_HOST_BUILD_ENV=1 \
    /bin/bash "$script_path" "$@"
fi

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found at $ros_setup; set ROS_SETUP." >&2
  exit 1
fi
if [[ ! -d "$robotpkg" ]]; then
  echo "Robotpkg prefix not found at $robotpkg; set ROBOTPKG." >&2
  exit 1
fi

toppra_source="$hpp_source_root/toppra"
hpp_toppra_source="$hpp_source_root/hpp-toppra"
hpp_exec_source="$hpp_source_root/hpp-exec"
for source_file in \
  "$toppra_source/cpp/CMakeLists.txt" \
  "$hpp_toppra_source/CMakeLists.txt" \
  "$hpp_exec_source/CMakeLists.txt"; do
  if [[ ! -f "$source_file" ]]; then
    echo "HPP source file not found at $source_file" >&2
    exit 1
  fi
done

mkdir -p "$hpp_build_root" "$hpp_install"
cp "$mfja_root/hpp_setup.bash.in" "$hpp_install/setup.bash"

set +u
# shellcheck disable=SC1090,SC1091
source "$hpp_install/setup.bash"
set -u

/usr/bin/python3 - "$robotpkg" <<'PY'
import sys
from pathlib import Path

import coal
import eigenpy
import pinocchio
import pyhpp
sys.modules.setdefault("hppfcl", coal)
import pyhpp_viser
import rclpy
from pyhpp.manipulation import Device

prefix = Path(sys.argv[1]).resolve()
if sys.version_info[:2] != (3, 12):
    raise SystemExit(f"Robotpkg HPP requires Python 3.12, got {sys.version}")
for module in (coal, eigenpy, pinocchio, pyhpp, pyhpp_viser):
    path = Path(module.__file__).resolve()
    if not path.is_relative_to(prefix):
        raise SystemExit(f"{module.__name__} is outside {prefix}: {path}")
print(f"Robotpkg HPP ready: {prefix}")
print(f"Python: {sys.version.split()[0]}")
print(f"pyhpp: {Path(pyhpp.__file__).resolve()}")
print(f"pyhpp_viser: {Path(pyhpp_viser.__file__).resolve()}")
print(f"rclpy: {Path(rclpy.__file__).resolve()}")
PY

export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-1}
export MAKEFLAGS="-j$CMAKE_BUILD_PARALLEL_LEVEL"
export PYTHONNOUSERSITE=1
common_cmake_args=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_LIBDIR=lib
  -DCMAKE_INSTALL_PREFIX="$hpp_install"
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
)

cmake \
  -S "$toppra_source/cpp" \
  -B "$hpp_build_root/toppra" \
  "${common_cmake_args[@]}" \
  -DBUILD_TESTS=OFF \
  -DBUILD_WITH_PINOCCHIO=OFF \
  -DPYTHON_BINDINGS=OFF
cmake --build "$hpp_build_root/toppra" --target install
for toppra_artifact in \
  "$hpp_install/lib/libtoppra.so" \
  "$hpp_install/lib/cmake/toppra/toppraConfig.cmake"; do
  if [[ ! -f "$toppra_artifact" ]]; then
    echo "TOPPRA installation is incomplete: $toppra_artifact" >&2
    exit 1
  fi
done

cmake \
  -S "$hpp_toppra_source" \
  -B "$hpp_build_root/hpp-toppra" \
  "${common_cmake_args[@]}" \
  -DBUILD_TESTING=OFF \
  -DGENERATE_PYTHON_STUBS=OFF \
  -DINSTALL_DOCUMENTATION=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_STANDARD_LAYOUT=ON
cmake --build "$hpp_build_root/hpp-toppra" --target install

cmake \
  -S "$hpp_exec_source" \
  -B "$hpp_build_root/hpp-exec" \
  "${common_cmake_args[@]}" \
  -DBUILD_TESTING=OFF \
  -DGENERATE_PYTHON_STUBS=OFF \
  -DINSTALL_DOCUMENTATION=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_STANDARD_LAYOUT=ON
cmake --build "$hpp_build_root/hpp-exec" --target install

/usr/bin/python3 - "$hpp_install" "$robotpkg" "$hpp_exec_source" <<'PY'
import sys
from pathlib import Path

import coal
import hpp_exec
import pyhpp
sys.modules.setdefault("hppfcl", coal)
import pyhpp_toppra
import pyhpp_viser
import rclpy
from pyhpp.manipulation import Device
from pyhpp_toppra import Toppra

local_prefix = Path(sys.argv[1]).resolve()
robotpkg_prefix = Path(sys.argv[2]).resolve()
source = Path(sys.argv[3]).resolve()
if sys.version_info[:2] != (3, 12):
    raise SystemExit(f"unexpected runtime Python: {sys.version}")
for module in (hpp_exec, pyhpp_toppra):
    path = Path(module.__file__).resolve()
    if not path.is_relative_to(local_prefix):
        raise SystemExit(f"{module.__name__} is outside {local_prefix}: {path}")
for module in (pyhpp, pyhpp_viser):
    path = Path(module.__file__).resolve()
    if not path.is_relative_to(robotpkg_prefix):
        raise SystemExit(f"{module.__name__} is outside {robotpkg_prefix}: {path}")
rclpy_path = Path(rclpy.__file__).resolve()
if not rclpy_path.is_relative_to(Path("/opt/ros/jazzy")):
    raise SystemExit(f"rclpy is outside /opt/ros/jazzy: {rclpy_path}")
installed = Path(hpp_exec.__file__).resolve().parent
for name in ("__init__.py", "joint_state.py"):
    if (source / "hpp_exec" / name).read_bytes() != (installed / name).read_bytes():
        raise SystemExit(f"installed hpp-exec {name} does not match {source}")
print(f"HPP source overlay ready: {local_prefix}")
print(f"Python: {sys.version.split()[0]}")
print(f"pyhpp: {Path(pyhpp.__file__).resolve()}")
print(f"pyhpp_toppra: {Path(pyhpp_toppra.__file__).resolve()}")
print(f"pyhpp_viser: {Path(pyhpp_viser.__file__).resolve()}")
print(f"hpp-exec: {Path(hpp_exec.__file__).resolve()}")
print(f"rclpy: {Path(rclpy.__file__).resolve()}")
PY

echo "After editing hpp-exec, rerun this script to reinstall the changes."
