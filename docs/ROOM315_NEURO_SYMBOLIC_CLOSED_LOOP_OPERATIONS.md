# Room 315 Neuro-Symbolic Closed-Loop Operations

Room 315 uses four separate closed-loop components: a local language model that
proposes task-goal fields, a visual-state model that estimates scene facts,
PlanSys2 for symbolic planning, and a deterministic rail-safety supervisor that
alone may publish typed rail commands. The curated 160 payload speed-sweep
cases remain the regression subset in:

```text
mfja_robot_control_config/config/room_315_payload_cases/payload_training_cases_expanded_160_speed_sweep.yaml
```

The supervisor accepts primitive JSON commands from the planner/executive path.
Generated payload cases execute as `switches`, `stoppers`, `shuttle`, `DONE`,
`stop_all`, and `emergency_stop` commands through `/room_315/rail_safety/primitive_command`.
Learned components do not publish this topic directly.

For seeded benchmark expansion and method comparison, use
[ROOM315_BENCHMARK_RUNBOOK.md](ROOM315_BENCHMARK_RUNBOOK.md). The benchmark
generator writes YAML manifests only; generated datasets and checkpoints stay
outside the repository.

For the current task-goal understanding layer, including
`TaskGoalDraft`, parser boundaries, clarification, confirmation, and local-model
output restrictions, use
[ROOM315_TASK_GOAL_UNDERSTANDING.md](ROOM315_TASK_GOAL_UNDERSTANDING.md).

The task-goal semantic runtime is configured by:

```text
mfja_robot_control_config/config/room_315_task_goal/task_goal_understanding.yaml
```

## Common Terminal Setup

Run this setup in every terminal used by the commands below. Source-relative
scripts and configuration paths assume the repository root is the current
directory.

```bash
export MFJA_WS="${MFJA_WS:-$HOME/mfja_ws}"
export MFJA_REPO="${MFJA_REPO:-$MFJA_WS/src/mfja_3rd_floor_gz}"
export ROOM315_INTENT_DIR="${ROOM315_INTENT_DIR:-$HOME/models/room315_intent}"

cd "$MFJA_REPO"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
if [[ -f "$HOME/.venvs/room315-intent/bin/activate" ]]; then
  source "$HOME/.venvs/room315-intent/bin/activate"
fi
if [[ -f "$ROOM315_INTENT_DIR/room315_intent.env" ]]; then
  source "$ROOM315_INTENT_DIR/room315_intent.env"
fi
```

Install or verify the real local offline intent checkpoint outside Git:

```bash
export ROOM315_INTENT_DIR="$HOME/models/room315_intent"
python3 -m venv --system-site-packages "$HOME/.venvs/room315-intent"
source "$HOME/.venvs/room315-intent/bin/activate"
python -m pip install --upgrade pip
python -m pip install 'llama-cpp-python==0.3.16'

python3 mfja_robot_control_config/scripts/setup_room315_intent_model.py \
  --model-dir "$ROOM315_INTENT_DIR" \
  --skip-dependency-install
source "$ROOM315_INTENT_DIR/room315_intent.env"
```

The setup command requires network access on its first run, downloads about
1.04 GiB (1.12 GB), and verifies the pinned checkpoint. The explicit virtual
environment avoids the setup script's user-package/`--break-system-packages`
fallback. Reactivate it before using the semantic model. The generated
environment file pins the verified local checkpoint and enables offline model
loading after setup succeeds.

Run a real semantic health/parse smoke test with:

```bash
PYTHONPATH=mfja_robot_control_config/scripts \
python3 mfja_robot_control_config/scripts/room_315_task_goal_semantic_smoke.py \
  --config "$ROOM315_TASK_GOAL_LOCAL_CONFIG" \
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
python3 mfja_robot_control_config/scripts/room_315_task_goal_cli.py \
  --config "$ROOM315_TASK_GOAL_LOCAL_CONFIG"
```

Run the English corpus benchmark with:

```bash
ros2 run mfja_robot_control_config room_315_task_goal_benchmark.py \
  --corpus mfja_robot_control_config/config/room_315_task_goal/task_goal_english_benchmark.yaml \
  --output /tmp/room315_task_goal_benchmark_report.json
```

## Run One Curated Case

The case generator always uses PlanSys2, including `--dry-run`. A runnable
single-case workflow therefore needs three sourced terminals.

Terminal 1 uses the high-level launch, which clears the disposable
`~/.ros/room315_visual_obstacles.json` cache by default. This case disables the
external obstacle feature and expects that clean state. Add
`room315_clear_visual_obstacle_pose_cache:=false` only for a reviewed workflow that
must preserve the cache; never repoint the cache argument to an unrelated file.

### Terminal 1: Simulation, Supervisor, Recorder, and Required Shuttle

```bash
export ROOM315_CASE_DATASET="$HOME/room315_data/manual_case"

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_rail_safety_supervisor:=true \
  enable_room315_visual_state_dataset_recorder:=true \
  room315_visual_dataset_dir:="$ROOM315_CASE_DATASET" \
  enable_room315_visual_obstacles:=false \
  room315_identity_selection_mode:=explicit \
  room315_right_shuttle_count:=1 \
  room315_right_active_identities:=R1 \
  room315_right_start_slot:=1 \
  room315_right_loaded_shuttles:=R1 \
  room315_left_shuttle_count:=0 \
  room315_shuttles_start_enabled:=false
```

This state matches
`right_loaded_r1_s1_to_slot3_no_blocker_speed008`. Wait at least five seconds.

### Terminal 2: PlanSys2

Start the repository's planner launch with task actuation disabled. No visual
runtime artifact is required for this curated-case generator path.

```bash
ros2 launch mfja_robot_control_config room_315_task_execution.launch.py \
  use_sim_time:=true \
  execution_enabled:=false \
  enable_plansys2:=true
```

### Terminal 3: Preflight, Generate, and Execute

Verify the planner, supervisor, recorder, and initial shuttle before execution:

```bash
ros2 lifecycle get /planner
ros2 topic echo --once /room_315/rail_safety/status std_msgs/msg/String
ros2 topic echo --once /room_315/visual_dataset/status std_msgs/msg/String
ros2 topic echo --once /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Generate the planned case without publishing supervisor commands:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --dry-run
```

Execute it only after the preflight state matches the YAML:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker_speed008 \
  --language-template-id loaded_shuttle_to_slot \
  --arrival-timeout-s 120 \
  --require-dataset-recorder \
  --execute
```

## Run The 160 Cases

Stop the manual Terminal 1 floor launch before starting the batch runner; the
runner owns one Room 315 launch at a time. Keep the PlanSys2 process from
Terminal 2 running because the batch runner does not start a planner and has no
fallback planning backend.

```bash
ros2 run mfja_robot_control_config room_315_payload_case_batch_runner.py \
  --case-config mfja_robot_control_config/config/room_315_payload_cases/payload_training_cases_expanded_160_speed_sweep.yaml \
  --dataset-dir ~/room315_payload_expanded_160_speed_sweep \
  --results-dir /tmp/room315_payload_expanded_160_speed_sweep
```

Use `--dry-run` to inspect launch arguments without starting Gazebo.

## Generate The Seeded Benchmark Manifest

```bash
ros2 run mfja_robot_control_config room_315_visual_planning_benchmark_suite.py generate-cases \
  --extension-case-count 320 \
  --seed 315 \
  --output /tmp/room315_seeded_balanced_cases.yaml
```

The output retains the 160 regression cases and adds balanced stress-family
coverage for 4+4 fleets, loaded/empty selection, blockers, occupied targets,
unknown positions, dropout, obstacles, inspection, and simultaneous requests.
Report Gazebo planning results separately from real-image perception claims.

## Manual Primitive Commands

These debug examples request Gazebo rail actuation through the supervisor. They
bypass TaskGoal confirmation, PlanSys2, and task-execution authorization; they
are not equivalent to the closed-loop language-to-motion path. Use them only in
the simulation after verifying the selected side, active shuttle, current
device state, supervisor status, and emergency-stop status. The supervisor
still validates each primitive before forwarding typed rail commands.

```bash
ros2 topic pub --once /room_315/rail_safety/primitive_command std_msgs/msg/String \
  "{data: '{\"action\":\"switches\",\"side\":\"right\",\"switches\":{\"A3\":\"INTERIOR\"}}'}"

ros2 topic pub --once /room_315/rail_safety/primitive_command std_msgs/msg/String \
  "{data: '{\"action\":\"stoppers\",\"side\":\"right\",\"stoppers\":{\"A3\":\"0\"}}'}"

ros2 topic pub --once /room_315/rail_safety/primitive_command std_msgs/msg/String \
  "{data: '{\"action\":\"shuttle\",\"side\":\"right\",\"shuttle\":\"room315_right_shuttle_1\",\"command\":\"ON\",\"speed\":0.08}'}"

ros2 topic pub --once /room_315/rail_safety/primitive_command std_msgs/msg/String \
  "{data: '{\"action\":\"stop_all\"}'}"
```

Use `stop_all` to request an ordinary supervised stop. Use the separate
`/room_315/rail_safety/emergency_stop` Boolean topic for emergency-stop testing, and
follow supervisor status before attempting any reset or clear operation.

The safety decoder validates side, target, switch and stopper masks, emergency
state, falling state, headway, block reservation, and required wait targets
before publishing rail commands.
