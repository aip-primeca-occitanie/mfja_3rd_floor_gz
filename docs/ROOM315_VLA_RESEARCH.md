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

## Model Input vs Privileged Eval

Training and online inference must use only `model_input`. Its schema is version
3 and contains exactly:

```text
language
overhead_images
last_command
```

Binary sensor bits, switch/stopper states, shuttle command state,
time-since-last-sensor-event, exact Gazebo pose, true shuttle segment,
arc-length position, distance-to-switch, normalized rail position, and
reset/evaluation internals are not part of `model_input`. Those values may
appear only under `privileged_eval`, `structured_rail_state`, `observation.state`,
or debug fields. Use them for expert execution, offline scoring, oracle
baselines, reset labels, and auditing; do not feed them to a learned VLA policy.

## Why Images Matter

The rail sensors are sparse binary sensors and are still useful for generating
correct demonstrations. They are not model-facing policy input. The overhead
images provide the visual signal the policy must use: shuttle location between
sensors, independently movable right/left obstacle markers, and station
occupancy inferred from the black shuttle covering or revealing slot fiducials.
Station-specific colored strips and green inspection disks are not used as
model-facing visual shortcuts.

This is why the project evaluates at least two policy families:

- `state_only`: language plus binary sensor/switch/stopper state, used only as
  an ablation baseline.
- `vla`: language plus overhead images, matching the deployable policy input.

The expected research signal is the gap between these two policies on tasks
where binary sensors alone are under-informative.

## Event-Level Action Schema V2

Training labels are event-level direct decisions, not high-level route templates
and not repeated framewise commands. Each training row represents:

```text
observation_before_decision -> next_direct_symbolic_event_action
```

The schema-v2 primitive set is:

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

The dataset layout is:

```text
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
   same schema-v2 action interface.
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
