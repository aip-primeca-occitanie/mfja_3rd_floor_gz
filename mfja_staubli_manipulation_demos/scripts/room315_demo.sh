#!/bin/bash
# Launch the Room 315 Staubli/shuttle manipulation scene in a clean environment.
# Arguments are forwarded to ros2 launch.
SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPT_DIR/room315_env.sh"

existing=$(pgrep -af 'gz sim .*room_315' || true)
if [[ -n "$existing" ]]; then
  echo "A Room 315 simulation is already running:" >&2
  echo "$existing" >&2
  echo "Stop it first: two simulations interleave robot states and shuttle poses." >&2
  exit 1
fi

exec ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_shuttle_manipulation_demo.launch.py "$@"
