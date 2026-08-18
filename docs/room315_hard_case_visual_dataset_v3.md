# Room 315 hard-case visual dataset V3

> **Historical workflow:** the commands and absolute paths below reproduce the
> dated V3 experiment; they are not the current V4 runtime procedure. Use the
> [Documentation Hub](README.md) to select the maintained workflow.

This workflow creates a new, perception-only development dataset without
reading or creating a Test split. It keeps `room315.visual_state.v3`, the
fixed identity order `L1,L2,L3,L4,R1,R2,R3,R4`, the authoritative 14-block
vocabulary, and paired 640×480 overhead RGB inputs.

All commands below run from:

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz
source /opt/ros/jazzy/setup.bash
source /home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/mfja_robot_control_config/scripts:${PYTHONPATH:-}"
```

## Quota and smoke

Create the deterministic quota plan:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_quota_planner.py \
  --output /home/tiago/room315_hard_case_visual_v3_guard_seed31520260730/room315_visual_v3_quota_plan.json
```

Materialise the guarded 32-scenario smoke manifest:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_generator.py \
  --mode smoke \
  --smoke-count 32
```

Capture it:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_capture.py \
  capture \
  --profile smoke \
  --resume
```

Audit it:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_audit.py \
  --mode smoke
```

The full manifest/capture must not start unless the smoke report says `PASS`.

## Full generation and safe resume

Materialise all manifests (4,000 train, 512 validation, and a separate 256
development canary):

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_generator.py \
  --mode full
```

Capture each package. Re-running the same commands with `--resume` verifies
every existing completed episode before continuing at the first incomplete
scenario:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_capture.py \
  capture --profile train --resume

python3 mfja_robot_control_config/scripts/room_315_visual_v3_capture.py \
  capture --profile validation --resume

python3 mfja_robot_control_config/scripts/room_315_visual_v3_capture.py \
  capture --profile canary --resume
```

For the approximately 16-hour sequential run, the guarded pipeline can be
started unattended after the smoke report passes:

```bash
nohup python3 \
  mfja_robot_control_config/scripts/room_315_visual_v3_pipeline.py \
  --start-at train \
  > /home/tiago/room315_hard_case_visual_v3_guard_seed31520260730/full_pipeline_console.log \
  2>&1 &
```

It runs train, validation, canary, grouped split creation, and the final audit
in that order. Any non-zero stage stops all later stages. Monitor it with:

```bash
cat /home/tiago/room315_hard_case_visual_v3_guard_seed31520260730/full_pipeline_state.json

find /home/tiago/room315_hard_case_visual_v3_capture_seed31520260730/dataset/episodes \
  -mindepth 1 -maxdepth 1 -type d ! -name .capture_tmp | wc -l
```

Inspect progress without changing state:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_capture.py \
  status --profile train
```

## Grouped split and audit

Create and verify the train/validation-only package:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_splitter.py
```

This compares validation semantic families with both the newly generated
training families and the allowed old replay-training files. It does not read,
hash, copy, or enumerate the consumed legacy Test.

Run the complete conditional and image audit:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3_audit.py \
  --mode full
```

## Tests

```bash
python3 -m py_compile \
  mfja_robot_control_config/scripts/room_315_visual_v3_*.py

pytest -q \
  mfja_robot_control_config/test/test_room315_visual_v3_dataset.py \
  mfja_robot_control_config/test/test_room315_visual_state_capture.py \
  mfja_robot_control_config/test/test_room315_visual_scenario_runner.py

colcon test \
  --packages-select mfja_robot_control_config \
  --pytest-args -q \
  --event-handlers console_direct+

git diff --check
```

## Output roots

- capture:
  `/home/tiago/room315_hard_case_visual_v3_capture_seed31520260730`
- grouped train/validation:
  `/home/tiago/room315_hard_case_visual_v3_splits_seed31520260730`
- development canary:
  `/home/tiago/room315_hard_case_visual_v3_canary_seed31520260730`
- smoke, quota, and audits:
  `/home/tiago/room315_hard_case_visual_v3_guard_seed31520260730`

The Experiment-A source-balanced policy is in
`config/room_315_vla/visual_state_experiment_a_dataset_v3.yaml`. It samples
approximately 50% old replay training and 50% new hard-case training per
epoch without duplicating JSONL rows. Its `PENDING_CAPTURE_AND_FINAL_AUDIT`
hashes deliberately fail closed until the captured files exist and the final
audit replaces them with exact SHA-256 values.
