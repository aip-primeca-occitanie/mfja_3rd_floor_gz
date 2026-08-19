#!/bin/bash
# Check the host setup needed by the complete Room 315 manipulation demo.
SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
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
pass "ROS setup: $ROS_SETUP"
pass "MFJA setup: $MFJA_SETUP"
pass "ROS domain: $ROS_DOMAIN_ID"

for command in ros2 gz docker; do
  if command -v "$command" >/dev/null 2>&1; then
    pass "command: $command"
  else
    fail "missing command: $command"
  fi
done

for package in \
  mfja_staubli_manipulation_demos \
  mfja_3rd_floor_bringup \
  mfja_3rd_floor_description \
  mfja_rail_interfaces \
  mfja_robot_control_config \
  ros_gz_sim; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    pass "ROS package: $package"
  else
    fail "missing ROS package: $package"
  fi
done

installed_executables=$(ros2 pkg executables mfja_staubli_manipulation_demos 2>/dev/null || true)
for executable in \
  room315_check_setup.sh \
  room315_demo.sh \
  room315_hpp_manipulation.sh \
  room315_manipulation_demo.sh \
  room315_moving_shuttle_demo.sh; do
  if grep -Fq "mfja_staubli_manipulation_demos $executable" <<<"$installed_executables"; then
    pass "ROS executable: $executable"
  else
    fail "missing ROS executable: $executable; rebuild and source the MFJA workspace"
  fi
done

if [[ -x "${HPP_EXEC_DIR:-}/run.sh" ]]; then
  pass "hpp-exec: $HPP_EXEC_DIR"
  if [[ -d "$HPP_EXEC_DIR/docker/devel/install/share/hpp-manipulation" ]]; then
    pass "HPP manipulation stack is built"
  else
    fail "HPP stack is not built; run hpp-exec once, then 'cd ~/devel/src && make all' in the container"
  fi
else
  fail "hpp-exec not found; set HPP_EXEC_DIR"
fi

if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    pass "Docker daemon is accessible"
  else
    fail "Docker daemon is not accessible by this user"
  fi
fi

if ((failures)); then
  printf '\n%d check(s) need attention.\n' "$failures" >&2
  exit 1
fi

printf '\nSetup is ready. Fixed-support test:\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_demo.sh gui:=false right_start_slot:=3\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_manipulation_demo.sh\n'
printf '\nFull moving-shuttle demo:\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_demo.sh gui:=false\n'
printf '  ros2 run mfja_staubli_manipulation_demos room315_moving_shuttle_demo.sh\n'
