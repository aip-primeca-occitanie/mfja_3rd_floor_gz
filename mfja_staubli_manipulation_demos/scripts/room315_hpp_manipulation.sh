#!/usr/bin/env bash
# Run the Room 315 HPP manipulation planner in the active colcon environment.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=room315_env.sh
source "$SCRIPT_DIR/room315_env.sh"

GRIPPER_OUTPUT=
EXPECT_GRIPPER_OUTPUT=false
for argument in "$@"; do
  if $EXPECT_GRIPPER_OUTPUT; then
    GRIPPER_OUTPUT=$argument
    EXPECT_GRIPPER_OUTPUT=false
    continue
  fi
  case "$argument" in
    --gripper-output)
      EXPECT_GRIPPER_OUTPUT=true
      ;;
    --gripper-output=*)
      GRIPPER_OUTPUT=${argument#*=}
      ;;
  esac
done

if [[ "$GRIPPER_OUTPUT" == "staubli-io" ]]; then
  if ! python3 -c 'import staubli_msgs.srv'; then
    echo "staubli_msgs is unavailable in the MFJA workspace." >&2
    echo "Rebuild the MFJA overlay, then source its install/setup.bash." >&2
    exit 1
  fi
fi

exec python3 -u "$ROOM315_PACKAGE_DIR/hpp/room315_shuttle_manipulation.py" "$@"
