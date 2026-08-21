#!/bin/bash
# HPP-planned Cartesian line for the Room 315 Staubli simulation.
# Arguments are forwarded to hpp/room315_hpp_line.py.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPT_DIR/room315_env.sh"

if [[ -z "${HPP_EXEC_DIR:-}" ]]; then
  echo "HPP_EXEC_DIR is not set; point it to your hpp-exec checkout." >&2
  echo "Example: export HPP_EXEC_DIR=\$HOME/hpp-exec" >&2
  exit 1
fi

if [[ ! -x "$HPP_EXEC_DIR/run.sh" ]]; then
  echo "hpp-exec run.sh not found; set HPP_EXEC_DIR=/path/to/hpp-exec." >&2
  exit 1
fi

DEMO_PACKAGE=mfja_staubli_demos
DESCRIPTION_PACKAGE=mfja_3rd_floor_description
DEMO_SHARE=$(ros2 pkg prefix --share "$DEMO_PACKAGE")
DESCRIPTION_SHARE=$(ros2 pkg prefix --share "$DESCRIPTION_PACKAGE")
DEMO_DIR=$(dirname -- "$(readlink -f "$DEMO_SHARE/package.xml")")
DESCRIPTION_DIR=$(dirname -- "$(readlink -f "$DESCRIPTION_SHARE/package.xml")")
CONTAINER_PACKAGES=/home/user/mfja_packages

# The MFJA workspace sets ROS_DOMAIN_ID=7; the container must match.
# /dev/shm shared with the host so Fast DDS discovers the host simulation.
EXTRA_DOCKER_ARGS="-v $DEMO_DIR:$CONTAINER_PACKAGES/$DEMO_PACKAGE:ro -v $DESCRIPTION_DIR:$CONTAINER_PACKAGES/$DESCRIPTION_PACKAGE:ro -v /dev/shm:/dev/shm" \
exec "$HPP_EXEC_DIR/run.sh" --domain-id "${ROS_DOMAIN_ID:-7}" bash -c "
  source /home/user/devel/config.sh &&
  export ROS_PACKAGE_PATH=$CONTAINER_PACKAGES\${ROS_PACKAGE_PATH:+:\$ROS_PACKAGE_PATH} &&
  python3 $CONTAINER_PACKAGES/$DEMO_PACKAGE/hpp/room315_hpp_line.py \"\$@\"" \
  bash "$@"
