#!/bin/bash
# Run the moving-shuttle Room 315 Staubli manipulation sequence.
# Default scenario: pick from one right-rail shuttle and place on a second one.
# Arguments are split by room315_moving_shuttle_sequence.py: known shuttle-demo
# options are consumed there, while remaining options are forwarded to HPP.
SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPT_DIR/room315_env.sh"

exec python3 -u "$SCRIPT_DIR/room315_moving_shuttle_sequence.py" \
  --hpp-script "$SCRIPT_DIR/room315_hpp_manipulation.sh" "$@"
