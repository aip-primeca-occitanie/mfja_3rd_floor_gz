# shellcheck shell=bash
# Internal environment helper sourced by the Room 315 scripts.
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
PACKAGE_NAME=mfja_staubli_manipulation_demos
if [[ -f "$SCRIPT_DIR/../package.xml" ]]; then
  ROOM315_PACKAGE_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd -P)
elif [[ -d "$SCRIPT_DIR/../../share/$PACKAGE_NAME" ]]; then
  ROOM315_PACKAGE_DIR=$(cd -- "$SCRIPT_DIR/../../share/$PACKAGE_NAME" && pwd -P)
else
  echo "Cannot locate the $PACKAGE_NAME package from $SCRIPT_DIR." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS 2 is unavailable; source the MFJA workspace install/setup.bash." >&2
  exit 1
fi
if ! python3 -c 'import hpp_exec, pyhpp, rclpy; from pyhpp.manipulation import Device' >/dev/null 2>&1; then
  echo "HPP manipulation and ROS Python modules are unavailable together." >&2
  echo "Source the MFJA workspace install/setup.bash built on your HPP underlay." >&2
  exit 1
fi

export ROOM315_PACKAGE_DIR
set -u
