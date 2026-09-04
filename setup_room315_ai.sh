#!/usr/bin/env bash
set -euo pipefail

MFJA_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOM315_MODEL_ROOT="${ROOM315_MODEL_ROOT:-$HOME/mfja_models}"
ROOM315_CONFIG_ROOT="$HOME/.config/mfja"
ROOM315_ENV_FILE="$ROOM315_CONFIG_ROOT/room315_ai.env"
VISUAL_ROOT="$ROOM315_MODEL_ROOT/visual_v4"
RELEASE_ROOT="$VISUAL_ROOT/room315_visual_v4_full_evidence_2026-08-11"
ACTIVE_BUNDLE="$RELEASE_ROOT/runtime/active_closed_loop_runtime"
ARCHIVE="$VISUAL_ROOT/room315_visual_v4_full_evidence_2026-08-11.tar.zst"
ARCHIVE_SHA256="35b583baca4f45eed6aad659c253180d00f1a5830ce389266ba714a4445a8ecc"
RELEASE_URL="https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/room315-visual-v4-evidence-2026-08-11/room315_visual_v4_full_evidence_2026-08-11.tar.zst"

command -v curl >/dev/null
command -v zstd >/dev/null
test -f /opt/ros/jazzy/setup.bash
mkdir -p \
  "$VISUAL_ROOT" \
  "$ROOM315_MODEL_ROOT/room315_intent" \
  "$HOME/.venvs" \
  "$ROOM315_CONFIG_ROOT"

python3 -m venv --system-site-packages "$HOME/.venvs/mfja-visual"
"$HOME/.venvs/mfja-visual/bin/python" -m pip install --upgrade pip
"$HOME/.venvs/mfja-visual/bin/python" -m pip install \
  torch==2.10.0 torchvision==0.25.0 \
  --index-url https://download.pytorch.org/whl/cpu

python3 -m venv --system-site-packages "$HOME/.venvs/room315-intent"
"$HOME/.venvs/room315-intent/bin/python" -m pip install --upgrade pip
"$HOME/.venvs/room315-intent/bin/python" -m pip install 'llama-cpp-python==0.3.16'
"$HOME/.venvs/room315-intent/bin/python" \
  "$MFJA_REPO/mfja_robot_control_config/scripts/setup_room315_intent_model.py" \
  --model-dir "$ROOM315_MODEL_ROOT/room315_intent" --skip-dependency-install

if [ ! -f "$ARCHIVE" ]; then
  curl -fL -o "$ARCHIVE.part" "$RELEASE_URL"
  mv "$ARCHIVE.part" "$ARCHIVE"
fi
printf '%s  %s\n' "$ARCHIVE_SHA256" "$ARCHIVE" | sha256sum --check -
if [ ! -f "$ACTIVE_BUNDLE/checkpoint_epoch_011.pt" ]; then
  tar --zstd -xf "$ARCHIVE" -C "$VISUAL_ROOT"
fi
(cd "$RELEASE_ROOT" && sha256sum --strict -c SHA256SUMS >/dev/null)

TASK_CONFIG="$ROOM315_MODEL_ROOT/task_execution_runtime.local.yaml"
cat >"$TASK_CONFIG" <<EOF
room_315_task_execution_node:
  ros__parameters:
    use_sim_time: true
    execution_enabled: false
    task_execution_authorization_path: $ACTIVE_BUNDLE/candidate_state.json
    task_execution_authorization_sha256: 14cedafe28c999786a66934a523db5757e1ccdd7ae34705d5a2df58488fc8df1
    task_execution_promotion_manifest_path: $ACTIVE_BUNDLE/runtime_promotion_manifest.json
    allowed_visual_schema_version: room315.visual_state.v4
    allowed_visual_checkpoint_sha256: 869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d
    external_obstacles_disabled: true
EOF

cat >"$ROOM315_ENV_FILE" <<EOF
export ROOM315_MODEL_ROOT="$ROOM315_MODEL_ROOT"
export ROOM315_ACTIVE_BUNDLE="$ACTIVE_BUNDLE"
export ROOM315_ACTIVE_MANIFEST="$ACTIVE_BUNDLE/runtime_promotion_manifest.json"
export ROOM315_TASK_RUNTIME="$TASK_CONFIG"
source "$ROOM315_MODEL_ROOT/room315_intent/room315_intent.env"
EOF

cp "$ROOM315_ENV_FILE" "$ROOM315_MODEL_ROOT/room315_ai.env"

echo "Room 315 AI setup completed: $ROOM315_MODEL_ROOT"
echo "Environment file: $ROOM315_ENV_FILE"
