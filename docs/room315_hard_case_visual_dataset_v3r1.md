# Room 315 hard-case visual dataset V3R1

> **Historical workflow:** this file preserves a dated V3R1 experiment and its
> qualification-host paths. It is not a current installation or V4 runtime
> guide. Use the [Documentation Hub](README.md) for maintained procedures.

V3R1 corrects the missing deliberate `R4` approach coverage for right slot 3
without changing `room315.visual_state.v3`. The authoritative location is
segment `A34E` at ratio `0.447469343`; the dedicated buckets are `-0.15`,
`-0.10`, `-0.05`, `-0.02`, `0.00`, `+0.02`, `+0.05`, `+0.10`, and `+0.15`.

The package reuses only complete, hash-verified V3 train episodes. Historical
V3 roots are read-only. The consumed legacy Test is never read, copied,
hashed, evaluated, or recreated.

## Environment

Run repository commands from:

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz
source /opt/ros/jazzy/setup.bash
source /home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/mfja_robot_control_config/scripts:${PYTHONPATH:-}"
```

## Immutable roots

- V3R1 capture:
  `/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730`
- V3R1 grouped train/validation:
  `/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730`
- V3R1 development canary:
  `/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730`
- V3R1 gates, smoke, and audits:
  `/home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730`

## Static generation and gates

Generate or verify the immutable full manifests and import reusable V3
episodes:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_generator.py \
  --mode full --resume

python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_audit.py \
  --mode static
```

The required deliberate counts are 540 train, 270 validation, and 108
canary. The audit reports deliberate exact-offset samples separately from
incidental nearby `R4/A34E` samples.

Generate, capture, and audit the isolated 36-scenario correction smoke:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_generator.py \
  --mode smoke --resume

python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_capture.py \
  capture --profile smoke --resume

python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_audit.py \
  --mode smoke
```

Full capture is forbidden unless the reuse audit, static audit, correction
smoke report, and family-overlap audit all pass.

## Full capture service

The dedicated user service name is:

```text
room315-visual-v3r1-full-31520260730.service
```

Start it only after all four gates pass:

```bash
systemd-run --user \
  --unit=room315-visual-v3r1-full-31520260730 \
  --collect \
  /bin/bash -lc \
  'cd /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz && source /opt/ros/jazzy/setup.bash && source /home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash && export PYTHONPATH="$PWD/mfja_robot_control_config/scripts:${PYTHONPATH:-}" && exec python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_pipeline.py --start-at train'
```

Monitor without modifying capture state:

```bash
systemctl --user status room315-visual-v3r1-full-31520260730.service

journalctl --user \
  -u room315-visual-v3r1-full-31520260730.service \
  -f

python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_capture.py \
  status --profile train

cat /home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730/full_pipeline_state.json
```

Safe resume after an interruption uses the same guarded pipeline:

```bash
systemd-run --user \
  --unit=room315-visual-v3r1-full-31520260730 \
  --collect \
  /bin/bash -lc \
  'cd /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz && source /opt/ros/jazzy/setup.bash && source /home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash && export PYTHONPATH="$PWD/mfja_robot_control_config/scripts:${PYTHONPATH:-}" && exec python3 mfja_robot_control_config/scripts/room_315_visual_v3r1_pipeline.py --start-at train'
```

Resume validates every existing episode and skips the 1,356 imported V3
episodes. The pipeline then captures missing train scenarios, validation, and
canary; creates train/validation only; verifies grouping and image overlap;
and runs the final audit.

## Experiment A

`config/room_315_vla/visual_state_experiment_a_dataset_v3r1.yaml` uses
deterministic 50/50 source-balanced sampling between approved old
training-replay data and V3R1 hard-case train. Canary is excluded from
training and checkpoint selection. Pending hashes deliberately fail closed
until full capture and final audit finish.
