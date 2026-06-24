# Source this first (not executable on its own).
#
# Re-executes the calling script in a clean environment so shell
# customizations (direnv, nix, conda, venvs, ...) cannot leak into the system
# ROS/Gazebo stack, then sources the ROS and MFJA workspaces.
if [[ -z "${ROOM315_CLEAN_ENV:-}" ]]; then
  clean_env=(ROOM315_CLEAN_ENV=1 PATH=/usr/local/bin:/usr/bin:/bin)
  for var in HOME USER LOGNAME TERM LANG DISPLAY XAUTHORITY WAYLAND_DISPLAY \
    XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS MFJA_SETUP MFJA_WS HPP_EXEC_DIR \
    "${!ROS_@}"; do
    if [[ -n "${!var:-}" ]]; then
      clean_env+=("$var=${!var}")
    fi
  done
  exec /usr/bin/env -i "${clean_env[@]}" /bin/bash "$0" "$@"
fi
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
MFJA_REPO=$(cd -- "$SCRIPT_DIR/../.." && pwd)
LOCAL_ENV="$SCRIPT_DIR/room315_local_env.sh"
if [[ -f "$LOCAL_ENV" ]]; then
  source "$LOCAL_ENV"
fi
ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
if [[ -z "${MFJA_SETUP:-}" ]]; then
  for setup in \
    "${MFJA_WS:-}/install/setup.bash" \
    "$MFJA_REPO/../../install/setup.bash" \
    "$MFJA_REPO/../mfja_ws/install/setup.bash" \
    "$HOME/devel/mfja_ws/install/setup.bash"; do
    if [[ -f "$setup" ]]; then
      MFJA_SETUP=$setup
      break
    fi
  done
fi
if [[ ! -f "$MFJA_SETUP" ]]; then
  echo "MFJA workspace setup not found; set MFJA_SETUP or MFJA_WS." >&2
  exit 1
fi
source "$ROS_SETUP"
source "$MFJA_SETUP"
set -u
