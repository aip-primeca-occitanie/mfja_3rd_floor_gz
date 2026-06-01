# Room 315 VLA Operations

### 11. Run the Room 315 VLA Supervisor

The VLA experiment layer adds independent right-rail and left-rail RGB-D
cameras and a ROS action supervisor. The supervisor supports two command
levels:

- **High-level research/benchmark commands**: `route_template` tasks such as
  `right_yaskawa_to_staubli`. These are the preferred VLA learning and
  evaluation targets. The supervisor expands them into deterministic primitive
  rail commands, tracks task phases, verifies success/failure, and publishes
  `active_tasks` plus bounded `completed_tasks` in `/room_315/vla/status`.
- **Primitive debugging/internal commands**: `switches`, `stoppers`, `shuttle`,
  `route_shuttle`, `stop_all`, and emergency-stop commands. These remain
  backward-compatible and are useful when manually inspecting switch, stopper,
  and shuttle behavior.

The supervisor never executes raw model actions directly. Incoming symbolic
actions first pass through the Room 315 safety decoder, which normalizes the
command, validates the type/side/targets/masks, checks the current shuttle,
switch, stopper, emergency, and falling state, and then either returns an
accepted `corrected_action` or rejects the proposal with a clear reason. Safe
corrections are limited to explicit normalization such as `switch` ->
`switches` or `I` -> `INTERIOR`; unsafe actions are not silently modified.
For schema-v2 model outputs, the supervisor also accepts an `action_vector`,
decodes it to an event-level symbolic action, validates it, and only then
produces the executable ROS command. The validator rejects switch changes near
occupied guarded switch segments, rejects loop transitions unless the shuttle is
stopped at the side-specific gate, rejects unsafe shuttle `ON` primitives that
do not include a wait condition and target, and always allows emergency-stop
proposals. Each decision log includes `raw_action`, `illegal_proposal`,
`rejected_action`, and `executed_action`.
Decoder metrics are published in `/room_315/vla/status` under
`safety_decoder.metrics`, including `total_proposed_actions`,
`accepted_actions`, `rejected_actions`, `illegal_proposal_rate`, and
`rejection_reasons`.

An optional VLA agent can now sit in front of the supervisor: it reads the
camera image plus `/room_315/vla/status`, accepts a human goal on
`/room_315/vla/user_goal`, and publishes the selected JSON action to
`/room_315/vla/command`.

Terminal 1 - launch Room 315 with the VLA bridge and supervisor:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true
```

Terminal 2 - watch the VLA status:

```bash
ros2 topic echo /room_315/vla/status std_msgs/msg/String
```

For model-facing experiments, run the HTTP VLA agent with a local or remote
endpoint. The endpoint receives `model_input_schema_version: 2`,
`model_input`, and the allowed action names, then returns a JSON action:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true \
  enable_room315_real_vla_agent:=true \
  room315_vla_agent_provider:=http \
  room315_vla_agent_http_endpoint:=http://127.0.0.1:8000/plan
```

Without a model endpoint, publish a task template directly to the supervisor.
With the HTTP agent running, publish high-level user goals to the agent:

```bash
ros2 topic pub --once /room_315/vla/user_goal std_msgs/msg/String \
  "{data: 'move the right shuttle from Yaskawa to Staubli'}"

ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"route_template\",\"template\":\"right_yaskawa_to_staubli\"}'}"
```

The VLA layer uses task-level templates instead of robot-specific shortcut names.
For transport tasks, the supervisor rejects the command unless it currently
detects an available shuttle on one of the source slot sensors. When the selected
shuttle reaches one of the target slot sensors, the supervisor stops it and marks
the task complete.

The Room 315 VLA station mapping is:

```text
Right rail slots 1-2: Yaskawa HC10DT
Right rail slots 3-4: Staubli TX2
Left rail slots 1-2: Yaskawa HC10
Left rail slots 3-4: KUKA KR6
```

Canonical task templates are defined in:

```text
mfja_robot_control_config/config/room_315_vla/vla_supervisor.yaml
```

Task templates:

```text
right_yaskawa_to_staubli: right slots 1-2 -> right slots 3-4
right_staubli_to_yaskawa: right slots 3-4 -> right slots 1-2
left_yaskawa_to_kuka: left slots 1-2 -> left slots 3-4
left_kuka_to_yaskawa: left slots 3-4 -> left slots 1-2
right_enter_interior_loop: right slots 1-2, stop before A3, set all switches INTERIOR, verify interior-loop entry, keep circulating
left_enter_interior_loop: left slots 1-2, stop before A1, set all switches INTERIOR, verify interior-loop entry, keep circulating
```

Task status includes fields such as `task_id`, `template`, `phase`, `status`,
`duration_s`, `failure_reason`, and the primitive commands emitted for that
task.

Watch the agent decision stream:

```bash
ros2 topic echo /room_315/vla/agent_status std_msgs/msg/String
```

To collect demonstrations for an open-source SmolVLA/LeRobot pipeline, enable
the dataset recorder. It stores each episode as raw replay rows, event-level
training labels, and JPEG camera frames under the selected dataset directory.
Raw frame rows can include `observation.images.right_rail_rgb` and
`observation.images.left_rail_rgb`:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  room315_visual_debug_colors:=false \
  enable_room315_vla:=true \
  enable_room315_real_vla_agent:=true \
  room315_vla_agent_provider:=http \
  enable_room315_vla_dataset_recorder:=true \
  room315_vla_dataset_dir:=~/room315_smolvla_demo
```

Set `room315_visual_debug_colors:=false` for realistic VLA datasets: switches
stay rail-colored and shuttles stay black even if the simulated shuttle enters
`FALLING` mode. Use `room315_visual_debug_colors:=true` to restore the older
debug colors for quick visual troubleshooting.

With `dataset_auto_start_on_goal` enabled by default, a new episode starts when
a goal arrives on `/room_315/vla/user_goal`. The benchmark runner starts and
stops episodes explicitly through `/room_315/vla/episode_control` while sending
`route_template` commands directly to `/room_315/vla/command`. Stop or mark an
episode manually from another terminal:

```bash
ros2 topic pub --once /room_315/vla/episode_control std_msgs/msg/String \
  "{data: 'stop success'}"
```

Recorder status is published on:

```bash
ros2 topic echo /room_315/vla/dataset_status std_msgs/msg/String
```

The recorded action vector follows
`mfja_robot_control_config/config/room_315_vla/action_space.yaml`. This keeps
the learning target compact and reproducible: a policy predicts the next
event-level symbolic action, then a post-processor converts that accepted event
action to the JSON command accepted by the supervisor.

Action space schema v2 uses only these primitives: `WAIT`, `DONE`,
`SET_SWITCHES`, `SET_STOPPERS`, `SHUTTLE_ON_FAST`, `SHUTTLE_ON_SLOW`,
`STOP_NOW`, and `EMERGENCY_STOP`. The vector fields are `primitive_id`,
`side_id`, per-device `switch_mask_A1..A4`/`switch_value_A1..A4`,
per-device `stopper_mask_A1..A4`/`stopper_value_A1..A4`,
`wait_condition_id`, `target_id`, and `reason_id`. A mask value of `0` means
`UNCHANGED`; only devices with mask `1` are decoded as selected devices.
This represents partial decisions such as “set only A3 to INTERIOR” or “close
only A4” without accidentally changing A1/A2/A3/A4 together. Fast versus slow
shuttle movement is represented by the primitive itself, not by a continuous
speed field.

The same vector can be sent directly to the supervisor for guarded execution:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action_vector\":[2,0,0,0,1,0,0,0,2,0,0,0,0,0,0,0,0,0,1,3,8]}'}"
```

That example decodes to `SET_SWITCHES` on the right rail, selecting only A3 and
setting it to `INTERIOR`; the supervisor still safety-checks it before
publishing the ROS switch command.

Each episode now has two JSONL files:

- `events.jsonl`: the training file. Each row is one decision event:
  `observation_before_decision -> next_symbolic_action`. Events are written only
  for meaningful actions such as `route_template`, `route_template_phase`,
  `switches`, `stoppers`, `shuttle` `ON/OFF`, `route_shuttle`, and terminal
  success/failure/stopped labels, but the training `next_action` is normalized
  into the schema-v2 primitive set.
- `data.jsonl`: raw framewise replay. It keeps camera references, structured
  status, and the latest command for auditing and temporal reconstruction, but
  rows are marked `raw_replay_only` and should not be used as repeated
  supervised labels.

Use `events.jsonl` for model training. Each event row includes `episode_id`,
`step_index`, `task`, flattened `observation.images.*`, `observation.state`,
schema-v2 `action`, and `privileged_eval`. The richer event rows still retain
debug fields such as `legacy_next_action`, `original_command`, `action_vector`,
and `wait_condition`, but those are not repeated frame labels. Episode summaries
and raw/event rows also include the current safety-decoder metrics so datasets
can report how many model proposals were rejected by the execution guard.

To build a flat training JSONL across all recorded episodes, run the extractor.
It reads only `episodes/*/events.jsonl` and ignores `episodes/*/data.jsonl`:

```bash
ros2 run mfja_robot_control_config room_315_vla_event_extractor.py \
  ~/room315_smolvla_demo \
  --output meta/training_events.jsonl
```

Each extracted row has the training shape:

```json
{
  "episode_id": "episode_000001_task",
  "step_index": 0,
  "task": "move the right shuttle from Yaskawa to Staubli",
  "observation.images.right_rail_rgb": "episodes/.../right_rail_rgb/000000.jpg",
  "observation.images.left_rail_rgb": "episodes/.../left_rail_rgb/000000.jpg",
  "observation.state": [],
  "action": {"primitive": "SET_SWITCHES", "side": "right"},
  "privileged_eval": {}
}
```

The model-facing observation is `model_input_schema_version: 2`. Train and serve
models from the `model_input` object only. Its fields are intentionally limited
to `language`, `overhead_images`, `binary_sensor_bits`, `switch_states`,
`stopper_states`, `last_command`, `shuttle_command_state`, and
`time_since_last_sensor_event`. Exact Gazebo pose, true shuttle segment,
distance-to-switch, and normalized-position values are excluded from
`model_input`; when needed for offline analysis they live under
`privileged_eval`/`debug`, not in the policy input.

The training observation vector is identity-aware. It no longer uses sensor
counts as primary features. Instead, `observation.state` contains a stable
multi-hot feature for each known right/left rail sensor, including slot sensors
such as `DZI2R`/`DZI3R`, switch approach sensors such as `DA3IR`, and
`A*_STOPPER_SENSOR` entries. Switch states are normalized before encoding:
`E`, `EXTERIOR`, and `exterior` all become `EXTERIOR`; `I`, `INTERIOR`, and
`interior` all become `INTERIOR`; unknown or missing values become `UNKNOWN`.
Old count-style values are retained only in `observation.debug_counts` for
auditing, not as the primary training state.

The teleoperation scenario generator now includes perception/recovery tasks
that are useful for training beyond simple route templates:
`unknown_position_recovery`, `visual_stop_before_A3`,
`visual_stop_before_A4`, `visual_center_at_station`,
`sensor_dropout_route`, `visual_marker_target`, and
`visual_obstacle_stop`. These scenarios still publish ordinary primitive VLA
commands, but their labels emphasize visual localization, binary sensor
reacquisition, stopper-based fallback, and obstacle stopping. Expert-side
Gazebo state may be used by the generator for safe reset/recovery and by the
recorder under `privileged_eval`, but the policy input remains limited to
schema-v2 `model_input`: language, overhead images, binary sensor bits,
switch/stopper states, last command, shuttle command state, and sensor-event
timing.

The overhead-camera scene also contains visual-only evaluation markers:
colored station strips, green inspection disks, empty/occupied station legend
markers, and a removable magenta obstacle marker entity named
`room315_vla_removable_obstacle_marker`. These objects are deliberately not
added to `observation.state` or `model_input`; they are learned only through
the camera images. Their definitions and station-occupancy labels are written
under `privileged_eval.visual_eval_labels` for evaluation and auditing.

Run the lightweight baseline evaluator before training a larger policy:

```bash
ros2 run mfja_robot_control_config room_315_vla_baseline_eval.py \
  ~/room315_smolvla_demo \
  --output-dir ~/room315_vla_baselines \
  --holdout-fraction 0.2
```

It evaluates three deterministic baselines on event-level labels:

- `state_only`: `language + binary_state -> event action`, using binary sensor
  bits plus normalized switch/stopper/last-command state, with no images.
- `vla`: `language + overhead_images + binary_state -> event action`, using the
  same state plus simple overhead-image color/content features.
- `oracle`: a privileged replay upper bound from the expert event labels. This
  is not a deployable policy; it checks the evaluation pipeline and defines the
  offline upper bound.

The evaluator writes:

```text
baseline_metrics.json
baseline_metrics.csv
baseline_task_family_metrics.csv
task_family_metrics.csv
```

Report `task_success`, `action_accuracy`, `primitive_accuracy`,
`side_accuracy`, `device_accuracy`, `completion_time`, `command_count`,
`illegal_proposal_rate`, and `rejected_action_rate` per task family. The
scenario-family success metrics are also broken out as
`unknown_position_success`, `sensor_dropout_success`, `visual_target_success`,
and `obstacle_stop_success`. Use the oracle gap when comparing learned VLA
policies.

To evaluate the task-level VLA layer as a research baseline, enable the
benchmark runner. It dispatches `route_template` commands automatically, waits
for supervisor task success/failure from `active_tasks` and `completed_tasks`,
optionally marks dataset episodes as success/failure, and writes JSONL metrics:

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
  room315_vla_benchmark_tasks:=transport \
  room315_vla_benchmark_report_dir:=~/room315_vla_benchmarks
```

Use `room315_vla_benchmark_tasks:=all`, `transport`, `loop_entry`, or a
comma-separated list such as
`right_yaskawa_to_staubli,left_kuka_to_yaskawa`. The runner publishes live
status on:

```bash
ros2 topic echo /room_315/vla/benchmark_status std_msgs/msg/String
```

Each benchmark run writes:

```text
~/room315_vla_benchmarks/room315_vla_benchmark_<timestamp>.jsonl
~/room315_vla_benchmarks/room315_vla_benchmark_<timestamp>_summary.json
```

Benchmark rows and summaries include `safety_decoder_metrics`; report the
`illegal_proposal_rate` alongside task success rate when evaluating a learned
policy. A model that completes tasks only by proposing many illegal actions is
therefore visible in the benchmark output.

For another VLA server, use `room315_vla_agent_provider:=http` and provide
`room315_vla_agent_http_endpoint:=http://host:port/path`. The endpoint receives
the schema-v2 `model_input` object with language, overhead image JPEGs as
base64, binary sensor bits, normalized switches/stoppers, last command, shuttle
command state, and sensor-event timing. It does not receive exact Gazebo pose or
true segment state.

Send a high-level route template:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"route_template\",\"template\":\"left_yaskawa_to_kuka\"}'}"
```

Send a primitive debugging command:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"switches\",\"side\":\"right\",\"switches\":{\"ALL\":\"EXTERIOR\"}}'}"
```

Primitive route debugging is still available:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"route_shuttle\",\"side\":\"right\",\"loop\":\"exterior\",\"start\":true}'}"
```

Send an interior-loop entry task:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"route_template\",\"template\":\"right_enter_interior_loop\"}'}"
```

Trigger and clear the virtual emergency stop:

```bash
ros2 topic pub --once /room_315/vla/emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"clear_emergency_stop\"}'}"
```

The independent rail-focused RGB-D cameras are bridged under:

```text
/room_315/vla/right_rail_rgbd/image
/room_315/vla/right_rail_rgbd/camera_info
/room_315/vla/right_rail_rgbd/depth_image
/room_315/vla/right_rail_rgbd/points
/room_315/vla/left_rail_rgbd/image
/room_315/vla/left_rail_rgbd/camera_info
/room_315/vla/left_rail_rgbd/depth_image
/room_315/vla/left_rail_rgbd/points
```
