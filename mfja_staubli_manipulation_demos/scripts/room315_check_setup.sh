#!/usr/bin/env bash
# Check the installed environment needed by the Room 315 manipulation demo.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=room315_env.sh
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

if python3 -c 'import coal, sys; sys.modules.setdefault("hppfcl", coal); import hpp_exec, pyhpp, pyhpp_viser, rclpy, trimesh, viser; from pyhpp.manipulation import Device' >/dev/null; then
  pass "HPP manipulation and ROS Python imports"
else
  fail "HPP manipulation and ROS Python imports"
fi

HPP_EXEC_PATH=$(python3 -c 'import hpp_exec; print(hpp_exec.__file__)' 2>/dev/null || true)
if [[ -n "$HPP_EXEC_PATH" && -f "$HPP_EXEC_PATH" ]]; then
  pass "installed hpp-exec: $HPP_EXEC_PATH"
else
  fail "hpp-exec is unavailable from the active HPP underlay"
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
