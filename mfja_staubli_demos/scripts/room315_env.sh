# shellcheck shell=bash
# Internal environment check used by the Room 315 scripts.
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS 2 is unavailable; source the MFJA workspace install/setup.bash." >&2
  exit 1
fi
if ! python3 -c 'import hpp_exec, pyhpp, rclpy' >/dev/null 2>&1; then
  echo "HPP and ROS Python modules are unavailable in the same interpreter." >&2
  echo "Source the MFJA workspace install/setup.bash built on your HPP underlay." >&2
  exit 1
fi
