# Room 315 Payload Case Planning

The active planning workflow is case based. The source of truth is the curated
160 payload speed-sweep cases:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml
```

Each case declares the side, target slot, loaded shuttle candidates, starting
slots, optional blocker-clearance strategy, launch parameters, and expected
selected shuttle. The scenario generator resolves one case into symbolic steps,
primitive commands, expected event targets, and schema-v3 action vectors.

## Dry Run

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --dry-run
```

## Preflight

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --command-timeout-s 8 \
  --preflight-only \
  --ready-line
```

## Execute

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --command-timeout-s 30 \
  --arrival-timeout-s 120 \
  --require-dataset-recorder \
  --output /tmp/right_loaded_r1_s1_to_slot3_no_blocker_speed008_execute.json \
  --quiet \
  --execute
```

## Batch Runner

```bash
ros2 run mfja_robot_control_config room_315_payload_case_batch_runner.py \
  --case-config mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml \
  --dataset-dir ~/room315_payload_expanded_160_speed_sweep \
  --results-dir /tmp/room315_payload_expanded_160_speed_sweep
```

The batch runner launches Room 315 for each selected case, checks preflight,
executes the generated primitive sequence, waits for a complete dataset episode,
and writes `payload_case_batch_summary.json`.
