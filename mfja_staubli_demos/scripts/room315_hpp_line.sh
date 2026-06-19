#!/bin/bash
# HPP-planned Cartesian line for the Room 315 Staubli: plans inside the
# hpp-exec container (host network reaches the simulation) and executes on
# the live robot. Arguments are forwarded to hpp/room315_hpp_line.py.
SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
MFJA_REPO=$(cd -- "$SCRIPT_DIR/../.." && pwd)
CONTAINER_REPO=/home/user/mfja_3rd_floor_gz

LOCAL_ENV="$SCRIPT_DIR/room315_local_env.sh"
if [[ -f "$LOCAL_ENV" ]]; then
  source "$LOCAL_ENV"
fi

if [[ -z "${HPP_EXEC_DIR:-}" ]]; then
  echo "HPP_EXEC_DIR is not set; point it to your hpp-exec checkout." >&2
  echo "Example: export HPP_EXEC_DIR=\$HOME/hpp-exec" >&2
  exit 1
fi

if [[ ! -x "$HPP_EXEC_DIR/run.sh" ]]; then
  echo "hpp-exec run.sh not found; set HPP_EXEC_DIR=/path/to/hpp-exec." >&2
  exit 1
fi

# The MFJA workspace sets ROS_DOMAIN_ID=7; the container must match.
# /dev/shm shared with the host so Fast DDS discovers the host simulation.
EXTRA_DOCKER_ARGS="-v $MFJA_REPO:$CONTAINER_REPO:ro -v /dev/shm:/dev/shm" \
exec "$HPP_EXEC_DIR/run.sh" --domain-id "${ROS_DOMAIN_ID:-7}" bash -c "
  source /home/user/devel/config.sh &&
  export ROS_PACKAGE_PATH=$CONTAINER_REPO\${ROS_PACKAGE_PATH:+:\$ROS_PACKAGE_PATH} &&
  python3 $CONTAINER_REPO/mfja_staubli_demos/hpp/room315_hpp_line.py \"\$@\"" \
  bash "$@"
