# Internal environment helper sourced by the Room 315 scripts.
#
# Re-executes the calling script in a clean environment so shell
# customizations (direnv, nix, conda, venvs, ...) cannot leak into the system
# ROS/Gazebo stack, then sources the ROS and MFJA workspaces.
if [[ -z "${ROOM315_CLEAN_ENV:-}" ]]; then
  clean_env=(ROOM315_CLEAN_ENV=1 PATH=/usr/local/bin:/usr/bin:/bin)
  for var in HOME USER LOGNAME TERM LANG DISPLAY XAUTHORITY WAYLAND_DISPLAY \
    XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS ROS_SETUP MFJA_SETUP MFJA_WS \
    STAUBLI_SETUP DEVEL_HPP_DIR HPP_EXEC_DIR \
    "${!DOCKER_@}" "${!ROS_@}"; do
    if [[ -n "${!var:-}" ]]; then
      clean_env+=("$var=${!var}")
    fi
  done
  exec /usr/bin/env -i "${clean_env[@]}" /bin/bash "$0" "$@"
fi
set +u
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
PACKAGE_NAME=mfja_staubli_manipulation_demos
if [[ -f "$SCRIPT_DIR/../package.xml" ]]; then
  ROOM315_PACKAGE_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
elif [[ -d "$SCRIPT_DIR/../../share/$PACKAGE_NAME" ]]; then
  ROOM315_PACKAGE_DIR=$(cd -- "$SCRIPT_DIR/../../share/$PACKAGE_NAME" && pwd)
else
  echo "Cannot locate the $PACKAGE_NAME package from $SCRIPT_DIR." >&2
  exit 1
fi

PACKAGE_PREFIX=
SOURCE_REPO=
if [[ "$ROOM315_PACKAGE_DIR" == */share/$PACKAGE_NAME ]]; then
  PACKAGE_PREFIX=${ROOM315_PACKAGE_DIR%/share/$PACKAGE_NAME}
else
  SOURCE_REPO=$(cd -- "$ROOM315_PACKAGE_DIR/.." && pwd)
fi

LOCAL_ENV="$SCRIPT_DIR/room315_local_env.sh"
if [[ -f "$LOCAL_ENV" ]]; then
  source "$LOCAL_ENV"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
if [[ -z "${MFJA_SETUP:-}" ]]; then
  setup_candidates=()
  if [[ -n "${MFJA_WS:-}" ]]; then
    setup_candidates+=("${MFJA_WS%/}/install/setup.bash")
  fi
  if [[ -n "$SOURCE_REPO" ]]; then
    setup_candidates+=(
      "$SOURCE_REPO/../../install/setup.bash"
      "$SOURCE_REPO/../mfja_ws/install/setup.bash"
    )
  fi
  if [[ -n "$PACKAGE_PREFIX" ]]; then
    setup_candidates+=(
      "$PACKAGE_PREFIX/../setup.bash"
      "$PACKAGE_PREFIX/setup.bash"
    )
  fi
  setup_candidates+=("$HOME/devel/mfja_ws/install/setup.bash")
  for setup in "${setup_candidates[@]}"; do
    if [[ -f "$setup" ]]; then
      MFJA_SETUP=$setup
      break
    fi
  done
fi
if [[ ! -f "${ROS_SETUP:-}" ]]; then
  echo "ROS setup not found at '$ROS_SETUP'; set ROS_SETUP." >&2
  exit 1
fi
if [[ ! -f "${MFJA_SETUP:-}" ]]; then
  echo "MFJA workspace setup not found; set MFJA_SETUP or MFJA_WS." >&2
  exit 1
fi
ROS_SETUP=$(readlink -f "$ROS_SETUP")
MFJA_SETUP=$(readlink -f "$MFJA_SETUP")
source "$ROS_SETUP"
source "$MFJA_SETUP"

if [[ -z "${HPP_EXEC_DIR:-}" ]]; then
  for directory in \
    "${DEVEL_HPP_DIR:+$DEVEL_HPP_DIR/src/hpp-exec}" \
    "$HOME/devel/nix-hpp/src/hpp-exec" \
    "$HOME/devel/hpp-exec" \
    "$HOME/nix-hpp/src/hpp-exec"; do
    if [[ -x "$directory/run.sh" ]]; then
      HPP_EXEC_DIR=$directory
      break
    fi
  done
fi
if [[ -d "${HPP_EXEC_DIR:-}" ]]; then
  HPP_EXEC_DIR=$(cd -- "$HPP_EXEC_DIR" && pwd)
fi

export ROS_SETUP MFJA_SETUP ROOM315_PACKAGE_DIR HPP_EXEC_DIR
set -u
