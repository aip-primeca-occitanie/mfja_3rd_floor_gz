#!/bin/bash
# Run the moving-shuttle Room 315 Staubli manipulation sequence.
# Default scenario: pick from one right-rail shuttle and place on a second one.
# Arguments are consumed by room315_moving_shuttle_sequence.py.
SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPT_DIR/room315_env.sh"

exec python3 -u \
  "$ROOM315_PACKAGE_DIR/scripts/room315_moving_shuttle_sequence.py" "$@"
