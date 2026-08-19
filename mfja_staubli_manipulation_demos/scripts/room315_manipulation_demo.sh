#!/bin/bash
# Run one shuttle-to-table manipulation cycle without commanding the rail.
SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPT_DIR/room315_env.sh"

exec python3 -u \
  "$ROOM315_PACKAGE_DIR/scripts/room315_manipulation_sequence.py" "$@"
