#!/usr/bin/env bash
# Run the installed Room 315 HPP planner.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=room315_env.sh
source "$script_dir/room315_env.sh"

package_share=$(python3 -c \
  'from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory("mfja_staubli_demos"))')
planner="$package_share/hpp/room315_hpp_line.py"
if [[ ! -f "$planner" ]]; then
  echo "Installed HPP planner not found at $planner; rebuild the MFJA workspace." >&2
  exit 1
fi

exec python3 "$planner" "$@"
