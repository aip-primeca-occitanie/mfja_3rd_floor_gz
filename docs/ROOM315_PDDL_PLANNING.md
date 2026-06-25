# Room 315 PDDL / PlanSys Scenario Generation

This document describes the focused Room 315 VLA research pipeline:

```text
PDDL goal/problem
  -> symbolic plan
  -> primitive VLA commands
  -> supervisor safety execution
  -> events.jsonl training rows
  -> dataset coverage report
```

PDDL/PlanSys is an expert scenario-generation layer. It is not the learned
model, and it is not passed directly as `model_input`. The learned VLA model
still predicts event-level `action_vector` targets from only language, overhead
images, and the previous command.

## Why PDDL / PlanSys

Room 315 scenarios are easiest to scale when the high-level task intent is
separate from low-level rail execution. PDDL gives the project a compact symbolic
language for goals such as "move the right shuttle from Yaskawa to Staubli."
PlanSys2 is now the required planning backend for replacing hand-written
scenario scripts with generated symbolic plans. The earlier deterministic
fallback and external command adapters are not runtime paths anymore.

The current scaffold deliberately starts small:

```text
prepare_switches -> open_stoppers -> move_shuttle -> stop_shuttle -> finish_task
```

The domain models rail sides, shuttles, stations, switch groups, and stopper
groups. It does not yet model every A1..A4 switch detail, visual obstacle
planning, KUKA/MoveIt coordination, or robot handshakes. Those remain explicit
future extensions.

## How Goals Become Symbolic Plans

Each PDDL problem defines initial station state and a goal such as:

```lisp
(task_done right_shuttle right_staubli)
```

The scenario generator accepts either a problem file or a supported symbolic
goal ID:

```text
right_yaskawa_to_staubli
right_staubli_to_yaskawa
left_yaskawa_to_kuka
left_kuka_to_yaskawa
```

The PlanSys2 backend implements the planner interface:

```text
plan(goal_or_problem) -> symbolic_plan
```

The only supported runtime backend is:

```text
plansys
```

`plansys` calls the PlanSys2 `planner/get_plan` service with the Room 315 PDDL
domain and the selected PDDL problem text. The returned PlanSys2 timed plan is
normalized into the internal symbolic plan format used by the existing
translator. If PlanSys2 packages are not installed or `/planner/get_plan` is not
available, generation fails clearly. There is no silent fallback planner.

## How Symbolic Plans Become VLA Commands

The translator converts each symbolic action into two aligned artifacts:

```text
symbolic plan step -> primitive JSON command -> event-level action/action_vector
```

Examples:

```text
prepare_switches right yaskawa staubli
  -> {"action": "switches", "side": "right", ...}
  -> SET_SWITCHES action_vector target

open_stoppers right
  -> {"action": "stoppers", "side": "right", ...}
  -> SET_STOPPERS action_vector target

move_shuttle right right_shuttle yaskawa staubli speed=0.3
  -> {"action": "shuttle", "side": "right", "command": "ON", "speed": 0.3}
  -> SHUTTLE_ON action_vector target

stop_shuttle right right_shuttle
  -> {"action": "shuttle", "side": "right", "command": "OFF"}
  -> STOP_NOW action_vector target

finish_task right_shuttle staubli
  -> {"action": "DONE", "status": "success", ...}
  -> DONE action_vector target
```

The PDDL-derived fields are retained as metadata for replay and auditing, not as
model inputs.

## How Commands Are Executed

Execution never bypasses the Room 315 VLA supervisor. In `--execute` mode the
generator:

1. Generates or loads a symbolic plan.
2. Runs static validation on the symbolic plan, primitive commands, schema-v3
   action vectors, shuttle identity, and model-input boundary.
3. Generates deterministic task language.
4. Publishes `start <language>` to `/room_315/vla/episode_control`.
5. Publishes each primitive command to `/room_315/vla/command`.
6. Waits for `/room_315/vla/status` safety-decoder feedback.
7. For shuttle motion steps, waits for the target station sensor before
   publishing the following `stop_shuttle` / `shuttle OFF` command.
8. Stops immediately with `stop failure` if the supervisor rejects a command,
   if the target station sensor is not reached before the arrival timeout, or
   if fleet safety metrics report wrong-shuttle, headway, block, deadlock, or
   emergency-stop violations.

For example, `right_yaskawa_to_staubli` waits for `DZI3R` or `DZI4R` before
publishing `shuttle OFF`. This keeps the PDDL plan symbolic while the runtime
still uses real supervisor status to decide when to stop the moving shuttle.
9. Verifies the final goal from runtime status and publishes `stop success`
   only after all planned steps complete.

Every generated scenario is a candidate until the validation gate writes:

```text
episodes/<episode_id>/validation.json
```

Only `validation_status: approved` with `approved_for_training: true` may enter
the default flat training export. Failed episodes remain on disk for debug.

The supervisor remains responsible for safety decoding and conversion to rail
switch, stopper, and shuttle commands. Reset is not recorded as part of the task.

## How events.jsonl Is Generated

The dataset recorder listens to the same supervisor command and episode-control
topics used by manual or teleop scenarios. During a planned episode it writes
event rows to:

```text
episodes/<episode_id>/events.jsonl
```

Each row captures the observation before a decision and the event-level target
for that decision. PDDL planning metadata may be copied into the row for
provenance, but it stays outside `model_input`.

## Model Input, Target, And Metadata

The learned model input remains exactly:

```text
model_input.language
model_input.overhead_images
model_input.last_command
```

The learned model target remains:

```text
action_vector
```

or an equivalent event-level primitive action. The model should learn to predict
the next event-level command from language, overhead camera images, and the
previous command.

Metadata-only fields include:

```text
pddl_goal
pddl_problem
symbolic_plan
planning_source
plan_step_index
generated_language
generated_language_template_id
language_template_id
structured_rail_state
privileged_eval
observation.state
debug
```

These fields are for scenario provenance, coverage reporting, privileged
evaluation, ablations, and debugging. They must not be fed to the main learned
VLA policy.

Example event shape:

```json
{
  "model_input": {
    "language": "move the right shuttle from Yaskawa to Staubli",
    "overhead_images": {
      "right_rail_rgb": "episodes/.../right_rail_rgb/000000.jpg",
      "left_rail_rgb": "episodes/.../left_rail_rgb/000000.jpg"
    },
    "last_command": {
      "action": "START"
    }
  },
  "action": {
    "primitive": "SET_SWITCHES",
    "side": "right",
    "target_id": "ALL_SWITCHES",
    "reason": "switch_update"
  },
  "action_vector": [2.0, 0.0],
  "planning_source": "pddl",
  "pddl_problem": "problem_right_yaskawa_to_staubli.pddl",
  "pddl_goal": "right_shuttle at staubli",
  "symbolic_plan": [
    "prepare_switches right yaskawa staubli",
    "open_stoppers right yaskawa staubli",
    "move_shuttle right right_shuttle yaskawa staubli speed=0.3",
    "stop_shuttle right right_shuttle",
    "finish_task right_shuttle staubli"
  ],
  "plan_step_index": 0,
  "language_template_id": "move_from_to"
}
```

The example `action_vector` is shortened for readability. Actual datasets use
the full Room 315 event-level action schema.

## Multi-Shuttle Extension

Room 315 now has a research scaffold for up to four shuttles per rail side:

```text
right_shuttle_1..right_shuttle_4  -> R1..R4
left_shuttle_1..left_shuttle_4    -> L1..L4
```

Launch arguments expose both the shuttle count and explicit start slots:

```text
room315_right_shuttle_count:=0..4
room315_left_shuttle_count:=0..4
room315_right_start_slots:=1,2,3,4
room315_left_start_slots:=1,2,3,4
room315_enable_payload_visuals:=true
room315_payload_pose_x_offset_m:=-0.08  # optional; default centers the payload
room315_right_loaded_shuttles:=R2
room315_left_loaded_shuttles:=L2
```

When more than one shuttle exists on a side, motion commands must identify the
target shuttle. Ambiguous commands such as “turn on the right shuttle” are
rejected by the fleet-aware safety layer. Explicit commands can name `R2`,
`right_shuttle_2`, or `room315_right_shuttle_2`.

The multi-shuttle event target is action schema v3. It uses the canonical
event-level primitive action format with:

```text
shuttle_index
target_id values such as right_shuttle_2 / left_shuttle_3
coordination_mode
fleet-oriented reason IDs
```

The model input schema remains version 3 and is unchanged:

```text
language
overhead_images
last_command
```

Payload state is represented symbolically outside `model_input` with predicates
such as:

```lisp
(loaded right_shuttle_2)
(empty right_shuttle_1)
```

The scenario generator includes payload-specific goals such as
`right_loaded_r2_to_staubli`, `right_loaded_to_slot3`,
`right_loaded_to_slot3_clear_blocker`, `right_empty_r1_to_yaskawa`, and
`left_loaded_l2_to_kuka`. Generated route commands still use schema v3 with
the selected `shuttle_index`/`target_id`, for example R2 maps to
`shuttle_index=1` and `target_id=right_shuttle_2`.

`right_loaded_to_slot3` is the first deterministic multi-match payload policy:
R1 starts loaded in slot 1, R2 starts loaded in slot 2, and the target is slot
3. When the language says “move the loaded right shuttle to slot 3”, the
scenario generator selects the loaded shuttle nearest to the target slot, with
lowest shuttle ID as the tie-breaker. The selected task still executes through
the supervisor/schema-v3 path, while `selection_policy`,
`selection_candidates`, `target_slot`, and payload state remain outside
`model_input`.

The supervisor accepts task language such as “move the loaded shuttle to
Staubli”, “move the empty shuttle to Yaskawa”, or “move R2 carrying a part to
Staubli”. If more than one source shuttle matches the requested loaded/empty
condition and no shuttle ID is given, the command is rejected as ambiguous and
the failed episode is not approved for training export by default.

`right_loaded_to_slot3_clear_blocker` is the blocked visual payload training
scenario. It starts R1 empty in slot 3 as the blocker and R2 loaded in slot 2
as the selected shuttle. The generated primitive plan closes A4, moves R1
forward only to the A4 stopper clearance point, opens the stoppers again, then
moves the loaded R2 to slot 3. Per-step metadata records `coordination_phase`,
`plan_step_target_shuttle_id`, `target_slot`, `target_sensors`, and
`blocker_clearance` outside `model_input`.

Current scope: blocker restoration uses free station slots. General restoration
to the interior loop is the next planning phase.

## Visual Identity And Payload Occlusion

The deployable VLA model should learn shuttle identity from the overhead RGB
images. Structured detector output is privileged. The shuttle identity config is:

```text
mfja_robot_control_config/config/room_315_vla/shuttle_identity.yaml
```

Each shuttle has perimeter identity regions, not a single center marker. The
center is reserved as the payload zone, while corner/edge regions carry tag IDs
and large human-readable labels such as `R2` or `L3`. This keeps identity cues
visible when a centered payload partially occludes the shuttle.

The optional privileged tracker publishes:

```text
/room_315/vla/shuttle_identity_tracks
/room_315/vla/shuttle_identity_debug
```

These tracks may include visible marker IDs, confidence, approximate image
regions, and occlusion state. They are for supervisor checks, dataset metadata,
evaluation, debug visualization, and detector-assisted ablations. They must not
be copied into `model_input`.

## Language Generation From PDDL

The first language layer uses deterministic templates, not an external LLM.
Examples:

```text
move the right shuttle from Yaskawa to Staubli
send the right shuttle to the Staubli station
move the left shuttle from Yaskawa to KUKA
route the left shuttle from KUKA station to Yaskawa station
move R2 to the Staubli station
move the shuttle labeled L3 to KUKA
move R4 to Staubli even though it is carrying a part
```

The generator supports fixed seeds and explicit template IDs so data generation
is reproducible while still covering several paraphrases per goal. Identity-aware
language is still task language. The resolved `target_shuttle_id`, marker IDs,
and tracker state remain metadata/evaluation fields outside `model_input`.

## Dry-Run Generation

Dry-run mode produces planned episode JSON without Gazebo execution, but it
still obtains the symbolic plan from PlanSys2:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --goal right_yaskawa_to_staubli \
  --planner-backend plansys \
  --language-seed 42 \
  --dry-run
```

Using a problem file:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --problem mfja_robot_control_config/config/room_315_vla/pddl/problem_right_yaskawa_to_staubli.pddl \
  --planner-backend plansys \
  --dry-run
```

Before running dry-run or execute mode, start/source an environment where the
PlanSys2 planner service is available. The generator uses `/planner/get_plan` by
default. Use `--planner-service` only if your PlanSys2 planner is exposed under a
different service name.

## Execute Generation

Execution mode requires the VLA supervisor and dataset recorder to be running:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --goal right_yaskawa_to_staubli \
  --planner-backend plansys \
  --language-seed 42 \
  --arrival-timeout-s 120 \
  --execute
```

All commands go through:

```text
/room_315/vla/command
/room_315/vla/episode_control
/room_315/vla/status
/room_315/vla/dataset_status
```

The generator stops the episode with failure if the supervisor rejects a command
or if no safety decision arrives before the command timeout.

## Batch Generation

Batch generation expands a YAML list of goals into many planned episodes. The
default config is:

```text
mfja_robot_control_config/config/room_315_vla/pddl_scenario_batch.yaml
```

Dry-run a batch:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --batch-config mfja_robot_control_config/config/room_315_vla/pddl_scenario_batch.yaml \
  --planner-backend plansys \
  --dry-run
```

Execute a batch:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --batch-config mfja_robot_control_config/config/room_315_vla/pddl_scenario_batch.yaml \
  --planner-backend plansys \
  --arrival-timeout-s 120 \
  --execute
```

The batch config controls:

```text
goals
repetitions_per_goal
language_seed
speed_values
output_dataset_dir
dry_run
execute
shuffle
```

The current batch enables the four simple station shuttle goals. Later goals
such as `right_full_exterior_loop`, `left_full_exterior_loop`, and
`emergency_stop_all` are listed as future work until their PDDL and translator
support is implemented.

## Dataset Coverage Report

After recording manual and/or PDDL-generated episodes, run:

```bash
ros2 run mfja_robot_control_config room_315_pddl_dataset_report.py \
  ~/room315_smolvla_demo \
  --output ~/room315_pddl_report.json
```

The report compares manual and PDDL-generated data with:

```text
dataset_source
total_episodes
approved_episodes
failed_episodes
approval_rate
failure_reasons_distribution
number_of_events
approved_event_count
skipped_event_count
goals_covered
language_variants_count
average_plan_length
action_primitive_distribution
side_distribution
speed_distribution
rejected_action_rate
task_success
shuttle_id_accuracy
wrong_shuttle_command_rate
identity_grounding_accuracy
target_shuttle_selection_accuracy
visible_marker_count_distribution
identity_occlusion_level_distribution
occluded_identity_success_rate
partial_occlusion_success_rate
loaded_shuttle_success_rate
unloaded_shuttle_success_rate
collision_or_near_collision_count
headway_violation_rate
block_reservation_success_rate
deadlock_rate
deadlock_avoidance_success_rate
average_wait_time
fleet_throughput_tasks_per_minute
per_side_success_rate
per_shuttle_success_rate
```

Rows with `planning_source: pddl` or PDDL fields are counted as PDDL-generated.
Rows without PDDL metadata are counted as manual. This is dataset/evaluation
tooling only; it does not train a model.
