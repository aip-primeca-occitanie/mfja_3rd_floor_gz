# Room 315 VLA Operations

Room 315 VLA operation keeps the curated 160 payload speed-sweep cases as the
regression subset in:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml
```

The supervisor accepts direct primitive JSON commands and schema-v3 action
vectors only. Generated payload cases execute as `switches`, `stoppers`,
`shuttle`, `DONE`, `stop_all`, and `emergency_stop` commands through
`/room_315/vla/command`.

For seeded benchmark expansion and method comparison, use
[ROOM315_BENCHMARK_RUNBOOK.md](ROOM315_BENCHMARK_RUNBOOK.md). The benchmark
generator writes YAML manifests only; generated datasets and checkpoints stay
outside the repository.

For the current production task-goal understanding layer, including
`TaskGoalDraft`, parser boundaries, clarification, confirmation, and local-model
output restrictions, use
[ROOM315_TASK_GOAL_UNDERSTANDING.md](ROOM315_TASK_GOAL_UNDERSTANDING.md).

The task-goal semantic runtime is configured by:

```text
mfja_robot_control_config/config/room_315_vla/task_goal_understanding.yaml
```

Install or verify the real local offline intent checkpoint outside Git:

```bash
python3 mfja_robot_control_config/scripts/setup_room315_intent_model.py
source /home/tiago/models/room315_intent/room315_intent.env
```

Run a real semantic health/parse smoke test with:

```bash
PYTHONPATH=mfja_robot_control_config/scripts \
python3 mfja_robot_control_config/scripts/room_315_task_goal_semantic_smoke.py \
  --require-real-model \
  --expect-semantic \
  --expect-draft-field selection_strategy=nearest \
  --expect-draft-field payload_filter=loaded \
  --expect-draft-field side=right \
  --expect-draft-field target_slot=3 \
  --text "Could you send whichever carrier is closest and holding a component to the third position on the right-hand line?"
```

Run the interactive user-facing command interface:

```bash
PYTHONPATH=mfja_robot_control_config/scripts \
python3 mfja_robot_control_config/scripts/room_315_task_goal_cli.py
```

Run the English corpus benchmark with:

```bash
ros2 run mfja_robot_control_config room_315_task_goal_benchmark.py \
  --corpus mfja_robot_control_config/config/room_315_vla/task_goal_english_benchmark.yaml \
  --output /tmp/room315_task_goal_benchmark_report.json
```

## Launch

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true
```

Watch supervisor status:

```bash
ros2 topic echo /room_315/vla/status std_msgs/msg/String
```

## Generate One Case

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --dry-run
```

Execute a case with the supervisor and dataset recorder running:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --arrival-timeout-s 120 \
  --require-dataset-recorder \
  --execute
```

## Run The 160 Cases

```bash
ros2 run mfja_robot_control_config room_315_payload_case_batch_runner.py \
  --case-config mfja_robot_control_config/config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml \
  --dataset-dir ~/room315_payload_expanded_160_speed_sweep \
  --results-dir /tmp/room315_payload_expanded_160_speed_sweep
```

Use `--dry-run` to inspect launch arguments without starting Gazebo.

## Generate The Seeded Benchmark Manifest

```bash
ros2 run mfja_robot_control_config room_315_vla_benchmark_suite.py generate-cases \
  --extension-case-count 320 \
  --seed 315 \
  --output /tmp/room315_seeded_balanced_cases.yaml
```

The output retains the 160 regression cases and adds balanced stress-family
coverage for 4+4 fleets, loaded/empty selection, blockers, occupied targets,
unknown positions, dropout, obstacles, inspection, and simultaneous requests.
Report Gazebo planning results separately from real-image perception claims.

## Manual Primitive Commands

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"switches\",\"side\":\"right\",\"switches\":{\"A3\":\"INTERIOR\"}}'}"

ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"stoppers\",\"side\":\"right\",\"stoppers\":{\"A3\":\"0\"}}'}"

ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"shuttle\",\"side\":\"right\",\"shuttle\":\"room315_right_shuttle_1\",\"command\":\"ON\",\"speed\":0.08}'}"
```

The safety decoder validates side, target, switch and stopper masks, emergency
state, falling state, headway, block reservation, and required wait targets
before publishing rail commands.
