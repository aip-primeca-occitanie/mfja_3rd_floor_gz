# Room 315 Payload Case Planning

The active planning workflow is case based. The checked-in source of truth for
regression is the curated 160 payload speed-sweep cases:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml
```

Each case declares the side, target slot, loaded shuttle candidates, starting
slots, optional blocker-clearance strategy, launch parameters, and expected
selected shuttle. The scenario generator resolves one case into symbolic steps,
primitive commands, expected event targets, and planner provenance.

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

## Seeded Benchmark Expansion

Use the benchmark suite to create a deterministic extension manifest without
committing generated artifacts:

```bash
ros2 run mfja_robot_control_config room_315_vla_benchmark_suite.py generate-cases \
  --extension-case-count 320 \
  --seed 315 \
  --output /tmp/room315_seeded_balanced_cases.yaml
```

The generated file embeds the 160-case regression subset and adds 100 to 1000
balanced extension cases. The extension explicitly covers 4+4 fleets,
loaded/empty shuttle selection, blockers, occupied targets, unknown positions,
sensor dropout, obstacles, inspection, and simultaneous requests.

Comparison reports should keep `oracle_plansys2`, `frozen_visual_plansys2`, and
`lora_visual_plansys2` in separate rows, and should keep Gazebo planning
evidence separate from real-image perception claims. See
[ROOM315_BENCHMARK_RUNBOOK.md](ROOM315_BENCHMARK_RUNBOOK.md).
