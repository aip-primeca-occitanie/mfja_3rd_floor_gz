# Room 315 Visual VLA with expert sensor supervision

The current Room 315 research setup is framed as a rail-cell
Vision-Language-Action problem with expert-side binary sensor supervision. The
scenario generator and supervisor may use slot sensors, stopper sensors, switch
state, and Gazebo state internally so demonstrations stop at correct, safe
places. The learned policy should not receive those expert shortcuts. It
observes a language goal, overhead camera images, and the previous command, then
predicts the next event-level direct symbolic action.

This gives the project a clear research boundary: the model must solve a
partially observable routing and safety task from realistic rail-cell
observations, while the supervisor remains responsible for deterministic
execution and safety gating.

## Focused Direction: PDDL / PlanSys Scenario Generation

The current data-generation direction is to use PDDL with a real PlanSys2
planner as the expert scenario layer for Room 315. The planner describes what
task should happen, not what the learned policy sees. Runtime generation no
longer uses a deterministic fallback planner or an external command adapter.

The pipeline is:

```text
PDDL goal/problem
  -> PlanSys2 /planner/get_plan
  -> symbolic plan
  -> primitive VLA commands
  -> supervisor safety execution
  -> events.jsonl rows
  -> dataset coverage report
```

A goal such as `right_yaskawa_to_staubli` or a PDDL problem with
`(task_done right_shuttle right_staubli)` becomes a symbolic plan such as:

```text
prepare_switches right yaskawa staubli
open_stoppers right yaskawa staubli
move_shuttle right right_shuttle yaskawa staubli speed=0.3
stop_shuttle right right_shuttle
finish_task right_shuttle staubli
```

The scenario generator calls PlanSys2, normalizes the returned timed plan into
those symbolic steps, and translates them into the same primitive VLA commands
that manual tools use: `switches`, `stoppers`, shuttle `ON/OFF`, and terminal
`DONE`. In execution mode, those commands are published only to
`/room_315/vla/command`, so the VLA supervisor and safety decoder remain in the
loop. Dataset episodes are started and stopped through
`/room_315/vla/episode_control`, and the recorder writes `events.jsonl`.

This creates a clean comparison between manual scenarios and PDDL-generated
scenarios. The `room_315_pddl_dataset_report.py` tool summarizes
`dataset_source`, goal coverage, language variants, plan length, action
primitive distribution, side distribution, speed distribution, rejected action
rate, and task success.

PDDL fields are metadata only. `pddl_goal`, `symbolic_plan`, `planning_source`,
`structured_rail_state`, and `privileged_eval` may be used for generation,
auditing, and evaluation, but they are not input to the learned model. The policy
still learns:

```text
model_input.language + model_input.overhead_images + model_input.last_command + model_input.observable_state
  -> action_vector
```

## Multi-Shuttle Benchmark Direction

The current scaffold extends Room 315 from one shuttle per rail side toward a
multi-shuttle VLA benchmark with up to four shuttles on the right rail and four
on the left rail:

```text
R1..R4 = right_shuttle_1..right_shuttle_4
L1..L4 = left_shuttle_1..left_shuttle_4
```

The launch layer accepts `room315_right_shuttle_count`,
`room315_left_shuttle_count`, `room315_right_start_slots`, and
`room315_left_start_slots`. Existing one-shuttle scenarios remain valid. When a
side has multiple shuttles, a shuttle motion command must specify the target
identity; otherwise the fleet-aware supervisor rejects it as ambiguous.

Action schema v3 keeps the event-level target contract but adds shuttle identity
and coordination fields:

```text
primitive_id
side_id
shuttle_index
switch/stopper masks and values
speed_mps
wait_condition_id
target_id
reason_id
coordination_mode
```

This is still a direct primitive-action target, not a route template and not a
PDDL action. The model-facing input remains deployable-only: `language`,
`overhead_images`, `last_command`, and `observable_state`.

## Visual Shuttle Identity Under Occlusion

Shuttle identity is intended to be learned visually. The project now defines a
perimeter identity mapping in
`mfja_robot_control_config/config/room_315_vla/shuttle_identity.yaml`.
Each shuttle has large text labels such as `R2`/`L3` plus multiple perimeter
fiducial IDs. The center of the shuttle is treated as the payload zone, so a
carried part can partially occlude the center without covering every identity
cue.

The optional `room_315_vla_shuttle_identity_tracker.py` node fuses privileged
fiducial detections into `/room_315/vla/shuttle_identity_tracks` for safety,
debugging, evaluation, and detector-assisted baselines. These tracks are never
model input. They may appear in metadata or `privileged_eval` as labels such as
`visible_marker_count`, `identity_occlusion_level`, `payload_present`, and
`target_shuttle_id`.

## Model Input vs Privileged Eval

Training and online inference must use only `model_input`. Its schema is version
3 and contains exactly:

```text
language
overhead_images
last_command
observable_state
```

`observable_state` contains only the real deploy-time binary sensor bits plus
current switch and stopper states. Shuttle command state,
time-since-last-sensor-event, exact Gazebo pose, true shuttle segment,
arc-length position, distance-to-switch, normalized rail position, payload
labels, and reset/evaluation internals are not part of `model_input`. Those
values may appear only under `privileged_eval`, `structured_rail_state`,
`observation.state`, or debug fields. Use them for expert execution, offline
scoring, oracle baselines, reset labels, and auditing; do not feed them to a
learned VLA policy.

## Why Images Matter

The rail sensors are sparse binary sensors and are useful model-facing
observations because the real system exposes them. The overhead images still
provide the visual signal the policy must use for shuttle location between
sensors, loaded/empty payload status, independently movable right/left obstacle
markers, and station occupancy inferred from the black shuttle covering or
revealing slot fiducials.
Station-specific colored strips and green inspection disks are not used as
model-facing visual shortcuts.

This is why the project evaluates at least two policy families:

- `state_only`: language plus binary sensor/switch/stopper state, used only as
  an ablation baseline.
- `vla`: language plus overhead images, matching the deployable policy input.

The expected research signal is the gap between these two policies on tasks
where binary sensors alone are under-informative.

## Event-Level Action Schema V3

Training labels are event-level direct decisions, not high-level route templates
and not repeated framewise commands. Each training row represents:

```text
observation_before_decision -> next_direct_symbolic_event_action
```

The schema-v3 primitive set is:

```text
WAIT
DONE
SET_SWITCHES
SET_STOPPERS
SHUTTLE_ON
STOP_NOW
EMERGENCY_STOP
```

Partial device commands are encoded with per-device masks and values:

```text
switch_mask[A1,A2,A3,A4] + switch_value[A1,A2,A3,A4]
stopper_mask[A1,A2,A3,A4] + stopper_value[A1,A2,A3,A4]
speed_mps
```

Unselected devices are `UNCHANGED`, so an action such as "set only A3 to
INTERIOR" cannot accidentally change A1, A2, or A4. The action vector also
stores `side`, explicit shuttle `speed_mps`, `wait_condition`, `target_id`,
and `reason`.

This action vector is the intended learned-model output. `route_template`
commands remain useful for expert demonstration generation and benchmark
orchestration, but the model should ultimately emit the event-level vector or an
equivalent primitive JSON command.

## Scenario Families

Use the scenario families to build a balanced dataset and report per-family
metrics:

```text
transport
loop_entry
station_navigation
stopper
exterior_loop
visual_target
obstacle_stop
emergency
```

The retained perception/recovery scenario is `left_slot3_kuka_then_slot2`.
Plain stopper-stop and station-navigation motions remain in the rail
scenario group, so they are not duplicated as visual scenarios. Synthetic
unknown-position recovery and sensor-dropout cases are intentionally excluded
from the research set because they do not add a clear model-facing visual
question. Stopper-only visual obstacle stops are excluded for the same reason.
The obstacle-approach set adds
`right_obstacle_aware_route` and `left_obstacle_aware_route`, where the expert
reads a terminal-controlled visual-only obstacle pose. The scenario itself does
not move or hide the obstacle. The task is one big exterior-loop lap: if the
obstacle is close enough to the exterior rail path, the shuttle stops at the
expert-computed point just before it and the episode ends there; if the obstacle
is clear of the exterior loop, the shuttle completes the lap. These scenarios
are designed so privileged Gazebo state can be used for reset/evaluation, while
the model still receives only schema-v3 visual
`model_input`.

## Dataset Export

Record episodes with the dataset recorder, then train from event rows only:

```bash
ros2 run mfja_robot_control_config room_315_vla_event_extractor.py \
  ~/room315_smolvla_demo \
  --output meta/training_events.jsonl
```

For PDDL/PlanSys2 episodes, `meta/training_events.jsonl` includes only episodes
whose `episodes/<episode_id>/validation.json` is approved for training. Failed
or unvalidated episodes are skipped unless an explicit debug flag such as
`--include-failed` is used.

The dataset layout is:

```text
episodes/<episode_id>/validation.json # scenario approval gate
episodes/<episode_id>/events.jsonl     # training labels
episodes/<episode_id>/data.jsonl       # raw replay/debug only
episodes/<episode_id>/images/...       # overhead camera frames
meta/training_events.jsonl             # flattened training export
```

Each training row contains `episode_id`, `step_index`, `task`,
`model_input`, `observation.images.*`, `observation.state`, `action`, and
`privileged_eval`. Use only `model_input` plus the event action label for learned
VLA policies. `observation.state`, `structured_rail_state`, and
`privileged_eval` are for audits, ablations, and oracle evaluation. Use
`events.jsonl` or `meta/training_events.jsonl` for learning. Do not train on
`data.jsonl`, because it is framewise replay and can repeat the same command
many times.

For PDDL-generated datasets, planner metadata may appear beside the event row as
`planning_source`, `pddl_goal`, `pddl_problem`, `symbolic_plan`,
`plan_step_index`, `language_template_id`, `target_shuttle_id`,
`visible_marker_count`, `payload_present`, `identity_occlusion_level`, and
`expected_visible_ids`. These are provenance/evaluation fields. They must remain
outside `model_input` and should be excluded from the learned policy input just
like `privileged_eval` and `structured_rail_state`.

## Baselines

Run the lightweight evaluator before training SmolVLA/LeRobot policies:

```bash
ros2 run mfja_robot_control_config room_315_vla_baseline_eval.py \
  ~/room315_smolvla_demo \
  --output-dir ~/room315_vla_baselines \
  --holdout-fraction 0.2
```

It writes:

```text
baseline_metrics.json
baseline_metrics.csv
baseline_task_family_metrics.csv
task_family_metrics.csv
```

The baseline set is:

- `state_only`: language plus sparse binary state predicts the event action as a
  non-deployable ablation.
- `vla`: language plus overhead images predicts the event action.
- `oracle`: privileged replay upper bound for checking the evaluation pipeline.

Report `task_success`, `action_accuracy`, `primitive_accuracy`,
`side_accuracy`, `device_accuracy`, `completion_time`, `command_count`,
`illegal_proposal_rate`, and `rejected_action_rate` per task family. Also report
`visual_target_success` and `obstacle_stop_success` for the perception-heavy
families.

For multi-shuttle PDDL datasets, also report shuttle identity and fleet metrics:
`shuttle_id_accuracy`, `wrong_shuttle_command_rate`,
`identity_grounding_accuracy`, `target_shuttle_selection_accuracy`,
`visible_marker_count_distribution`, `identity_occlusion_level_distribution`,
`partial_occlusion_success_rate`, `loaded_shuttle_success_rate`, `headway_violation_rate`,
`block_reservation_success_rate`, `deadlock_rate`,
`deadlock_avoidance_success_rate`, `fleet_throughput_tasks_per_minute`,
`per_side_success_rate`, and `per_shuttle_success_rate`.

## Identity-Aware Multi-Shuttle Milestone

Room 315 now includes Gazebo-visible shuttle identity models:

```text
room315_shuttle_R1 ... room315_shuttle_R4
room315_shuttle_L1 ... room315_shuttle_L4
```

The world files preload four right and four left shuttle entities with distinct
R1-R4/L1-L4 models. Each model keeps the center as the payload zone and places
four physical identity regions on the perimeter/corners:

```text
front_left
front_right
rear_left
rear_right
```

Each region uses RGB-visible geometry/material colors and a fiducial-like
placeholder. The labels are part of the shuttle body; they are not simulator
overlays, do not use SVG albedo texture maps, and are not structured model
input. The learned policy must infer the target shuttle from the overhead images
plus task language.

Payload models are available for identity occlusion experiments:

```text
room315_vla_payload_small_box
room315_vla_payload_small_box as carried_box
room315_vla_payload_medium_box
room315_vla_payload_tall_box
room315_vla_payload_wide_box_within_keepout
room315_vla_payload_partial_marker_occluder
```

Their metadata lives in
`mfja_robot_control_config/config/room_315_vla/payload_scenarios.yaml`. Normal
payloads stay inside the center keep-out zone and preserve all perimeter identity
regions. The partial occluder is a controlled test case for one-corner
occlusion.

Room 315 shuttles can also carry a visual payload box during normal shuttle
motion. The kinematic shuttle node spawns `room315_vla_payload_small_box` as a
separate Gazebo model named `<shuttle_entity>_payload`, keeps it pose-synced
with the shuttle, and publishes a privileged payload-state JSON topic:

```text
/room_315/rails/right/shuttles/payload_state
/room_315/rails/left/shuttles/payload_state
```

Initial loaded shuttles are launch-configurable:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_enable_payload_visuals:=true \
  room315_right_shuttle_count:=4 \
  room315_left_shuttle_count:=4 \
  room315_right_loaded_shuttles:=R2 \
  room315_left_loaded_shuttles:=L2
```

Payload state can be changed while a scenario is running:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/payload_command \
  std_msgs/msg/String \
  "{data: '{\"shuttle\":\"R2\",\"loaded\":true,\"payload_type\":\"box\"}'}"

ros2 topic pub --once /room_315/rails/right/shuttles/payload_command \
  std_msgs/msg/String \
  "{data: '{\"shuttle\":\"R2\",\"loaded\":false}'}"
```

The deployable model input remains exactly:

```text
model_input.language
model_input.overhead_images
model_input.last_command
model_input.observable_state
```

Payload state, payload type, visible marker count, expected visible IDs,
`target_shuttle_id`, identity tracker state, rail occupancy, and block
reservations are privileged metadata for safety/evaluation only. They may appear
in top-level event metadata or `privileged_eval`, never inside `model_input`.

## Real VLA Agent Contract

The real VLA HTTP agent asks for schema-v3 action vectors. Schema-v3 includes
the explicit `shuttle_index` and coordination fields needed for
identity-aware fleet control:

```json
{
  "action_vector_schema_version": 3,
  "action_vector": [4, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25, 3, 29, 14, 1]
}
```

The agent rejects multi-shuttle movement vectors that omit `shuttle_index`, and
it rejects primitive JSON shuttle commands unless they include `shuttle_id`,
`shuttle`, `name`, or `shuttle_index`. It also rejects model outputs containing
privileged fields such as `structured_rail_state`, `target_shuttle_id`,
`symbolic_plan`, `shuttle_identity_tracks`, or PDDL internals.

## Fleet Safety Runtime State

The VLA supervisor now builds runtime fleet-safety state from the rail shuttle
status. It checks occupied blocks, reservations, station-slot target conflicts,
and minimum headway before accepting shuttle motion. Accepted movement commands
can reserve `next_block` and `target_slot`; stop/reset/remove releases the
owner's reservations. Emergency stop and stop-all remain globally allowed.

The supervisor status includes privileged fleet state under
`safety_decoder.fleet_state` with `model_input_exposure: excluded`. Rejection
metrics include block occupancy, reservation, headway, wrong-shuttle, and
deadlock counters.

## Eval Commands

Run a task-level benchmark with the supervisor and recorder enabled:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_right_shuttle_count:=1 \
  room315_right_start_slot:=1 \
  room315_left_shuttle_count:=1 \
  room315_left_start_slot:=1 \
  room315_shuttles_start_enabled:=false \
  enable_room315_vla:=true \
  enable_room315_vla_dataset_recorder:=true \
  room315_vla_dataset_dir:=~/room315_smolvla_demo \
  enable_room315_vla_benchmark_runner:=true \
  room315_vla_benchmark_tasks:=all \
  room315_vla_benchmark_report_dir:=~/room315_vla_benchmarks
```

Watch benchmark status:

```bash
ros2 topic echo /room_315/vla/benchmark_status std_msgs/msg/String
```

Run the offline evaluator on the recorded dataset:

```bash
ros2 run mfja_robot_control_config room_315_vla_baseline_eval.py \
  ~/room315_smolvla_demo \
  --output-dir ~/room315_vla_baselines
```

## Future 4-Robot Extension Roadmap

The current rail-only VLA task already encodes the four station identities:

```text
Right slots 1-2: Yaskawa HC10DT
Right slots 3-4: Staubli TX2
Left slots 1-2: Yaskawa HC10
Left slots 3-4: KUKA KR6
```

A realistic extension path is:

1. Keep the rail-only policy as the low-level logistics layer and preserve the
   same schema-v3 action interface.
2. Add robot availability and station readiness as sparse binary inputs, not as
   privileged robot poses.
3. Add task templates such as "deliver part to KUKA, wait for robot done, return
   to Yaskawa" while keeping shuttle/switch/stopper execution safety-gated.
4. Add visual station state labels for part present/absent, blocked station, and
   inspection target.
5. Evaluate multi-station coordination with the same per-family metrics plus
   robot-handshake success, station wait time, and rail utilization.

This roadmap keeps the research contribution centered on VLA under sparse
industrial sensing rather than on hand-coded Gazebo pose control.
