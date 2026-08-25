#!/bin/bash
# Compatibility wrapper for the Python trajectory exporter.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
exec python3 "$SCRIPT_DIR/room315_export_staubli_line.py" "$@"
