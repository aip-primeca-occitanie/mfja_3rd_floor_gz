#!/bin/bash
# Build the HPP manipulation scene inside the hpp-exec container.
# Arguments are forwarded to hpp/room315_shuttle_manipulation.py.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPT_DIR/room315_env.sh"

if [[ -z "${HPP_EXEC_DIR:-}" ]]; then
  echo "HPP_EXEC_DIR is not set; point it to your hpp-exec checkout." >&2
  echo "Example: export HPP_EXEC_DIR=\$HOME/devel/hpp-exec" >&2
  exit 1
fi

if [[ ! -x "$HPP_EXEC_DIR/run.sh" ]]; then
  echo "hpp-exec run.sh not found; set HPP_EXEC_DIR=/path/to/hpp-exec." >&2
  exit 1
fi

DEMO_PACKAGE=mfja_staubli_manipulation_demos
DESCRIPTION_PACKAGE=mfja_3rd_floor_description
DEMO_SHARE=$(ros2 pkg prefix --share "$DEMO_PACKAGE")
DESCRIPTION_SHARE=$(ros2 pkg prefix --share "$DESCRIPTION_PACKAGE")
DEMO_DIR=$(dirname -- "$(readlink -f "$DEMO_SHARE/package.xml")")
DESCRIPTION_DIR=$(dirname -- "$(readlink -f "$DESCRIPTION_SHARE/package.xml")")
CONTAINER_PACKAGES=/home/user/mfja_packages

GRIPPER_OUTPUT=
EXPECT_GRIPPER_OUTPUT=false
EXECUTE=false
for argument in "$@"; do
  if $EXPECT_GRIPPER_OUTPUT; then
    GRIPPER_OUTPUT=$argument
    EXPECT_GRIPPER_OUTPUT=false
    continue
  fi
  case "$argument" in
    --execute)
      EXECUTE=true
      ;;
    --gripper-output)
      EXPECT_GRIPPER_OUTPUT=true
      ;;
    --gripper-output=*)
      GRIPPER_OUTPUT=${argument#*=}
      ;;
  esac
done

STAUBLI_DOCKER_ARGS=
STAUBLI_CONTAINER_SETUP=true
if [[ "$GRIPPER_OUTPUT" == "staubli-io" && "$EXECUTE" == true ]]; then
  if [[ -n "${STAUBLI_SETUP:-}" ]]; then
    if [[ ! -f "$STAUBLI_SETUP" ]]; then
      echo "Staubli workspace setup not found at '$STAUBLI_SETUP'." >&2
      exit 1
    fi
    STAUBLI_SETUP=$(readlink -f "$STAUBLI_SETUP")
    set +u
    source "$STAUBLI_SETUP"
    set -u
  fi

  if ! STAUBLI_MSGS_PREFIX=$(ros2 pkg prefix staubli_msgs); then
    echo "staubli_msgs is unavailable; set STAUBLI_SETUP=/path/to/staubli_ws/install/local_setup.bash." >&2
    exit 1
  fi
  STAUBLI_MSGS_PREFIX=$(readlink -f "$STAUBLI_MSGS_PREFIX")
  STAUBLI_PACKAGE_SETUP=$STAUBLI_MSGS_PREFIX/share/staubli_msgs/local_setup.bash
  if [[ ! -f "$STAUBLI_PACKAGE_SETUP" ]]; then
    echo "staubli_msgs setup not found at '$STAUBLI_PACKAGE_SETUP'." >&2
    exit 1
  fi

  STAUBLI_PREFIX_PARENT=$(dirname -- "$STAUBLI_MSGS_PREFIX")
  if [[ -f "$STAUBLI_MSGS_PREFIX/.colcon_install_layout" ]]; then
    STAUBLI_INSTALL_ROOT=$STAUBLI_MSGS_PREFIX
  elif [[ -f "$STAUBLI_PREFIX_PARENT/.colcon_install_layout" ]]; then
    STAUBLI_INSTALL_ROOT=$STAUBLI_PREFIX_PARENT
  else
    echo "Cannot determine the colcon install root for '$STAUBLI_MSGS_PREFIX'." >&2
    exit 1
  fi

  STAUBLI_WORKSPACE_ROOT=$(dirname -- "$STAUBLI_INSTALL_ROOT")
  if [[ ! -d "$STAUBLI_WORKSPACE_ROOT/build" ]]; then
    echo "Staubli workspace build directory not found at '$STAUBLI_WORKSPACE_ROOT/build'." >&2
    exit 1
  fi

  STAUBLI_DOCKER_ARGS="-v $STAUBLI_WORKSPACE_ROOT:$STAUBLI_WORKSPACE_ROOT:ro"
  STAUBLI_CONTAINER_SETUP="source '$STAUBLI_PACKAGE_SETUP'"
fi

if docker ps --format '{{.Names}}' | grep -Fxq hpp-exec; then
  echo "The hpp-exec container is already running and cannot receive the Room 315 bind mounts." >&2
  echo "Exit that container before running this command." >&2
  exit 1
fi

EXTRA_DOCKER_ARGS="-v $DEMO_DIR:$CONTAINER_PACKAGES/$DEMO_PACKAGE:ro -v $DESCRIPTION_DIR:$CONTAINER_PACKAGES/$DESCRIPTION_PACKAGE:ro -v /dev/shm:/dev/shm $STAUBLI_DOCKER_ARGS" \
exec "$HPP_EXEC_DIR/run.sh" --domain-id "${ROS_DOMAIN_ID:-7}" bash -c "
  source /home/user/devel/config.sh &&
  $STAUBLI_CONTAINER_SETUP &&
  export ROS_PACKAGE_PATH=$CONTAINER_PACKAGES\${ROS_PACKAGE_PATH:+:\$ROS_PACKAGE_PATH} &&
  python3 -u $CONTAINER_PACKAGES/$DEMO_PACKAGE/hpp/room315_shuttle_manipulation.py \"\$@\"" \
  bash "$@"
