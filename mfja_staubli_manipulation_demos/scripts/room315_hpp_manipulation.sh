#!/bin/bash
# Build the HPP manipulation scene inside the hpp-exec container.
# Arguments are forwarded to hpp/room315_shuttle_manipulation.py.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
MFJA_REPO=$(cd -- "$SCRIPT_DIR/../.." && pwd)
CONTAINER_REPO=/home/user/mfja_3rd_floor_gz

LOCAL_ENV="$SCRIPT_DIR/room315_local_env.sh"
if [[ -f "$LOCAL_ENV" ]]; then
  source "$LOCAL_ENV"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}

if [[ -z "${HPP_EXEC_DIR:-}" ]]; then
  echo "HPP_EXEC_DIR is not set; point it to your hpp-exec checkout." >&2
  echo "Example: export HPP_EXEC_DIR=\$HOME/devel/nix-hpp/src/hpp-exec" >&2
  exit 1
fi

if [[ ! -x "$HPP_EXEC_DIR/run.sh" ]]; then
  echo "hpp-exec run.sh not found; set HPP_EXEC_DIR=/path/to/hpp-exec." >&2
  exit 1
fi

EXTRA_DOCKER_ARGS="-v $MFJA_REPO:$CONTAINER_REPO:ro -v /dev/shm:/dev/shm" \
exec "$HPP_EXEC_DIR/run.sh" --domain-id "${ROS_DOMAIN_ID:-7}" bash -c "
  source /home/user/devel/config.sh &&
  export LD_LIBRARY_PATH=/home/user/devel/install/lib:/home/user/devel/install/lib64:/opt/openrobots/lib:\${LD_LIBRARY_PATH:-} &&
  export PYTHONPATH=/home/user/devel/install/lib/python3.12/site-packages:/opt/openrobots/lib/python3.12/site-packages:/home/user/devel/hpp-exec:\${PYTHONPATH:-} &&
  export ROS_PACKAGE_PATH=$CONTAINER_REPO\${ROS_PACKAGE_PATH:+:\$ROS_PACKAGE_PATH} &&
  python3 -u $CONTAINER_REPO/mfja_staubli_manipulation_demos/hpp/room315_shuttle_manipulation.py \"\$@\"" \
  bash "$@"
