#!/usr/bin/env bash
# Run the Room 315 Staubli table pick-and-place planner.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=room315_env.sh
source "$SCRIPT_DIR/room315_env.sh"

exec python3 -u "$ROOM315_PACKAGE_DIR/hpp/room315_pick_place.py" "$@"
