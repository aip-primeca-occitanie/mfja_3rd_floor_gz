#!/usr/bin/env bash
# Check the installed environment needed by the Room 315 manipulation demo.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=room315_env.sh
# shellcheck disable=SC1091
source "$SCRIPT_DIR/room315_env.sh"

failures=0

pass() {
  printf '  [OK] %s\n' "$1"
}

fail() {
  printf '  [--] %s\n' "$1"
  failures=$((failures + 1))
}

printf 'Room 315 setup check\n'
MFJA_PREFIX=$(ros2 pkg prefix mfja_staubli_manipulation_demos 2>/dev/null || true)
pass "MFJA overlay: $MFJA_PREFIX"
pass "ROS domain: ${ROS_DOMAIN_ID:-0}"

for command in ros2 gz python3; do
  if command -v "$command" >/dev/null 2>&1; then
    pass "command: $command"
  else
    fail "missing command: $command"
  fi
done

for subcommand in interface service; do
  if ros2 "$subcommand" --help >/dev/null 2>&1; then
    pass "ROS CLI: ros2 $subcommand"
  else
    fail "missing ROS CLI: ros2 $subcommand"
  fi
done

for package in \
  mfja_staubli_manipulation_demos \
  mfja_3rd_floor_bringup \
  mfja_3rd_floor_description \
  mfja_robot_control_config \
  ros_gz_sim \
  staubli_msgs; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    pass "ROS package: $package"
  else
    fail "missing ROS package: $package"
  fi
done

installed_executables=$(ros2 pkg executables mfja_staubli_manipulation_demos 2>/dev/null || true)
for executable in \
  room315_check_setup.sh \
  room315_pick_place.sh; do
  if grep -Fq "mfja_staubli_manipulation_demos $executable" <<<"$installed_executables"; then
    pass "ROS executable: $executable"
  else
    fail "missing ROS executable: $executable; rebuild the MFJA overlay"
  fi
done

if MODULE_REPORT=$(python3 - \
  "${INSTALL_HPP_DIR:-}" \
  "${ROBOTPKG:-}" \
  "${ROS_SETUP:-}" <<'PY'
import sys
from pathlib import Path

import coal

sys.modules.setdefault("hppfcl", coal)

import hpp_exec
import pyhpp
import pyhpp_toppra
import pyhpp_viser
import rclpy
import trimesh
import viser
from pyhpp.manipulation import Device
from pyhpp_toppra import Toppra


def require_prefix(module, prefix):
    path = Path(module.__file__).resolve()
    if prefix and not path.is_relative_to(prefix):
        raise SystemExit(f"{module.__name__} is outside {prefix}: {path}")
    return path


local_prefix = Path(sys.argv[1]).resolve() if sys.argv[1] else None
robotpkg_prefix = Path(sys.argv[2] or "/opt/openrobots").resolve()
ros_setup = Path(sys.argv[3] or "/opt/ros/jazzy/setup.bash").resolve()
ros_prefix = ros_setup.parent

paths = {
    "hpp-exec": require_prefix(hpp_exec, local_prefix),
    "hpp-toppra": require_prefix(pyhpp_toppra, local_prefix),
    "pyhpp": require_prefix(pyhpp, robotpkg_prefix),
    "pyhpp-viser": require_prefix(pyhpp_viser, robotpkg_prefix),
    "rclpy": require_prefix(rclpy, ros_prefix),
}
for name, path in paths.items():
    print(f"{name}: {path}")
PY
); then
  pass "HPP manipulation, TOPPRA, viewer, and ROS Python imports"
  while IFS= read -r module_path; do
    pass "$module_path"
  done <<<"$MODULE_REPORT"
else
  fail "Python modules are missing or resolve from unexpected prefixes"
fi

if ((failures)); then
  printf '\n%d check(s) need attention.\n' "$failures" >&2
  exit 1
fi

printf '\nSetup is ready. Planning-only check:\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --build-only\n'
printf '\nViser:\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --viser\n'
printf '\nSimulation:\n'
printf '  ros2 launch mfja_staubli_manipulation_demos room_315_staubli_pick_place_sim.launch.py\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_pick_place.sh --execute\n'
printf '\nAuthorized hardware execution requires the hardware launch, hardware profile, and measured --q-start.\n'
