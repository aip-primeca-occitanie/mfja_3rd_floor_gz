# Room 315 VLA Operations

### 11. Run the Room 315 VLA Supervisor

The VLA experiment layer adds independent right-rail and left-rail RGB-D
cameras and a ROS action supervisor. The learned VLA policy is intended to
output event-level direct symbolic commands: preferably a schema-v3
`action_vector`, or an equivalent JSON primitive command such as `switches`,
`stoppers`, `shuttle`, `stop_all`, or `emergency_stop`. `route_template` tasks
such as `right_yaskawa_to_staubli` are still supported for expert
demonstration generation, manual checks, and benchmark orchestration, but they
are not the final model-output interface.

The supervisor never executes raw model actions directly. Incoming symbolic
actions first pass through the Room 315 safety decoder, which normalizes the
command, validates the type/side/targets/masks, checks the current shuttle,
switch, stopper, emergency, and falling state, and then either returns an
accepted `corrected_action` or rejects the proposal with a clear reason. Safe
corrections are limited to explicit normalization such as `switch` ->
`switches` or `I` -> `INTERIOR`; unsafe actions are not silently modified.
For schema-v3 model outputs, the supervisor also accepts an `action_vector`,
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
endpoint. The endpoint receives `model_input_schema_version: 3`,
`model_input`, the event-level action-vector contract, and allowed fallback JSON
commands. The `allowed_actions` sent to the model intentionally excludes
`route_template`; the preferred response is `{"action_vector": [...]}`:

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

Without a model endpoint, publish a task template directly to the supervisor to
exercise the expert/demo path. With the HTTP agent running, publish high-level
user goals to the agent; the model should answer with an event-level command:

```bash
ros2 topic pub --once /room_315/vla/user_goal std_msgs/msg/String \
  "{data: 'move the right shuttle from Yaskawa to Staubli'}"

ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"route_template\",\"template\":\"right_yaskawa_to_staubli\"}'}"
```

The demo and benchmark layer uses task-level templates instead of
robot-specific shortcut names. For transport templates, the supervisor rejects
the command unless it currently detects an available shuttle on one of the source
slot sensors. When the selected shuttle reaches one of the target slot sensors,
the supervisor stops it and marks the task complete.

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
the learning target compact and reproducible: a policy predicts the next direct
event-level symbolic action, then the supervisor decodes and safety-checks that
action before publishing rail commands.

For multi-shuttle experiments, launch up to four shuttles per rail side:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true \
  room315_right_shuttle_count:=4 \
  room315_right_start_slots:=1,2,3,4 \
  room315_left_shuttle_count:=4 \
  room315_left_start_slots:=1,2,3,4
```

Stable identities are `R1..R4` and `L1..L4`. If more than one shuttle exists on
a side, the supervisor rejects shuttle-motion commands that do not identify the
target shuttle. Use `shuttle_id: "R2"`, `shuttle: "right_shuttle_2"`, or the
Gazebo entity name `room315_right_shuttle_2`.

Action space schema v3 uses only these primitives: `WAIT`, `DONE`,
`SET_SWITCHES`, `SET_STOPPERS`, `SHUTTLE_ON`, `STOP_NOW`, and
`EMERGENCY_STOP`. The vector fields are `primitive_id`,
`side_id`, `shuttle_index`, per-device
`switch_mask_A1..A4`/`switch_value_A1..A4`,
per-device `stopper_mask_A1..A4`/`stopper_value_A1..A4`,
`speed_mps`, `wait_condition_id`, `target_id`, `reason_id`, and
`coordination_mode`. A mask value of `0` means
`UNCHANGED`; only devices with mask `1` are decoded as selected devices.
This represents partial decisions such as “set only A3 to INTERIOR” or “close
only A4” without accidentally changing A1/A2/A3/A4 together. Shuttle movement
uses the `SHUTTLE_ON` primitive plus an explicit `speed_mps` value, so policies
can request the actual shuttle speed in meters per second.

Multi-shuttle targets use the same schema-v3 event-level representation with
`shuttle_index`, `coordination_mode`, and target IDs such as
`right_shuttle_2` and `left_shuttle_3`. These are target labels, not extra
model input. The model-facing input remains:

```text
language
overhead_images
last_command
```

The same vector can be sent directly to the supervisor for guarded execution:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action_vector_schema_version\":3,\"action_vector\":[2,0,-1,0,0,1,0,0,0,2,0,0,0,0,0,0,0,0,0,0.0,1,3,8,0]}'}"
```

That example decodes to `SET_SWITCHES` on the right rail, selecting only A3 and
setting it to `INTERIOR`; the supervisor still safety-checks it before
publishing the ROS switch command.

Each episode now has two JSONL files:

- `events.jsonl`: the training file. Each row is one decision event:
  `observation_before_decision -> next_symbolic_action`. Even when demos are
  triggered by `route_template`, the trainable `action` and `action_vector` are
  normalized into the schema-v3 event-level primitive set. The learned model
  should emit this event-level action, not repeat the route template.
- `data.jsonl`: raw framewise replay. It keeps camera references, structured
  status, and the latest command for auditing and temporal reconstruction, but
  rows are marked `raw_replay_only` and should not be used as repeated
  supervised labels.

Use `events.jsonl` for model training. Each event row includes `episode_id`,
`step_index`, `task`, flattened `observation.images.*`, `observation.state`,
schema-v3 `action`, and `privileged_eval`. The richer event rows still retain
debug fields such as `symbolic_next_action`, `original_command`, `action_vector`,
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

The model-facing observation is `model_input_schema_version: 3`. Train and serve
learned VLA policies from the `model_input` object only. Its fields are
intentionally limited to `language`, `overhead_images`, and `last_command`.
Binary sensor bits, switch/stopper states, shuttle command state,
time-since-last-sensor-event, exact Gazebo pose, true shuttle segment,
distance-to-switch, and normalized-position values are excluded from
`model_input`; when needed for expert execution, offline analysis, or debugging
they live under `privileged_eval`, `structured_rail_state`, `observation.state`,
or debug fields, not in the policy input.

The raw event rows still retain an identity-aware `observation.state` vector for
auditing and state-only ablations. It contains stable multi-hot features for each
known right/left rail sensor, including slot sensors such as `DZI2R`/`DZI3R`,
switch approach sensors such as `DA3IR`, and `A*_STOPPER_SENSOR` entries. Do not
use `observation.state` when training the deployable VLA policy; use it only for
diagnostics and baseline comparisons.

The teleoperation scenario generator now keeps the nontrivial visual research
tasks: `left_slot3_kuka_then_slot2`, `right_obstacle_aware_route`, and
`left_obstacle_aware_route`. These scenarios still publish ordinary primitive
VLA commands, but their labels emphasize camera-visible shuttle motion,
KUKA/left-shuttle sequencing, and obstacle-aware go/stop decisions. Synthetic
unknown-position recovery, stopper-only visual obstacle stops, and sensor-dropout
cases are intentionally excluded from the research scenario set because they do
not add a clear model-facing visual question. Expert-side sensors and Gazebo
state may still be used by the generator for safe reset/recovery and by the
recorder under `privileged_eval`, but the policy input remains limited to
schema-v3 `model_input`: language, overhead images, and last command.

The overhead-camera scene keeps station cues minimal: slot fiducials remain
visible, while dedicated colored station strips, green inspection disks, and
empty/occupied station legend markers are not present. Station occupancy is
meant to be inferred visually from whether the black shuttle covers or reveals
the station/slot fiducials. Two independent visual obstacle entities are still
available for obstacle-stop scenarios: `room315_vla_right_obstacle_marker` and
`room315_vla_left_obstacle_marker`. These visual cues are deliberately not
added to `observation.state` or `model_input`; they are learned only through
the camera images. Occupancy labels derived from binary slot sensors are
written only under `privileged_eval.visual_eval_labels` for evaluation and
auditing.

Move the right or left visual obstacle with explicit coordinates while Gazebo
is running. The tool calls Gazebo `set_pose` and also writes
`~/.ros/room315_vla_obstacles.json`, which `m10`/`m11` read at scenario start
inside the same Gazebo run:

```bash
ros2 run mfja_robot_control_config room_315_vla_obstacle_tool.py \
  --side right --x -14.18 --y -4.68

ros2 run mfja_robot_control_config room_315_vla_obstacle_tool.py \
  --side left --x -9.86 --y -4.68 --z 0.856 --yaw 0.0
```

Use a negative `--z`, such as `--z -5.0`, to hide one obstacle without deleting
the model. To create a non-blocking example, move the obstacle away from the rail
path, for example `--side right --x -15.24 --y -5.54`; then run `m10` and the
shuttle should complete one exterior-loop lap.

The teleop generator first reads the pose cache written by
`room_315_vla_obstacle_tool.py`; it does not call `set_pose` or move the
obstacle itself. `room_315_only.launch.py` and `full_floor.launch.py` clear this
cache by default at simulation startup, because Gazebo resets the obstacle model
to the world-file pose on every restart. This prevents `m10`/`m11` from stopping
at a stale obstacle position from a previous run. Use
`room315_clear_vla_obstacle_pose_cache:=false` only if you intentionally want to
keep the old cache across launches.

If the obstacle is within
`*_manual_obstacle_path_threshold_m` of the exterior rail path, it runs the
shuttle on the exterior loop and sends `OFF` when the shuttle reaches
`*_manual_obstacle_stop_before_m` before the obstacle. The episode ends there.
If the obstacle is farther away, or hidden below
`*_manual_obstacle_hidden_z_threshold_m`, it opens the route and completes one
exterior-loop lap.

```bash
ros2 run mfja_robot_control_config vla_teleop_generator.py --ros-args \
  -p scenario_names:=m10
```

Use `m11` for the same obstacle-aware behavior on the left rail.

Use `-p right_manual_obstacle_use_pose_file:=false` if you want to ignore the
cache and provide coordinates directly with
`right_manual_obstacle_x/y/z/yaw`. The obstacle-aware scenarios always use the
exterior loop; they do not reroute through the interior branch.
Use `right_manual_obstacle_stop_before_m` or
`left_manual_obstacle_stop_before_m` to tune how close the shuttle stops before
the obstacle. The default is `0.20`.
If the teleop log says `source=ros_parameters`, the scenario is using the
default/parameter pose rather than the cache from `room_315_vla_obstacle_tool.py`.
That is expected immediately after a fresh simulation launch. After running the
obstacle tool inside the same launch, the log should say `source=pose_file:...`.

```bash
ros2 run mfja_robot_control_config vla_teleop_generator.py --ros-args \
  -p scenario_names:=m11 \
  -p left_manual_obstacle_stop_before_m:=0.18
```

Run the lightweight baseline evaluator before training a larger policy:

```bash
ros2 run mfja_robot_control_config room_315_vla_baseline_eval.py \
  ~/room315_smolvla_demo \
  --output-dir ~/room315_vla_baselines \
  --holdout-fraction 0.2
```

It evaluates three deterministic baselines on event-level labels:

- `state_only`: `language + binary_state -> event action`, using binary sensor
  bits plus normalized switch/stopper state as a non-deployable ablation.
- `vla`: `language + overhead_images -> event action`, using the deployable
  visual policy input plus simple overhead-image color/content features.
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
`visual_target_success` and `obstacle_stop_success`. Use the oracle gap when
comparing learned VLA policies.

To evaluate the expert/demo task layer, enable the benchmark runner. It
dispatches `route_template` commands automatically, waits for supervisor task
success/failure from `active_tasks` and `completed_tasks`, optionally marks
dataset episodes as success/failure, and writes JSONL metrics. This benchmark is
useful for collecting and checking episodes; it is separate from the final
event-level model-output contract:

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
the schema-v3 `model_input` object with language, overhead image JPEGs as
base64, and last command. It does not receive binary sensor bits,
switch/stopper states, exact Gazebo pose, or true segment state.

## Multi-Shuttle Identity And Payload Runs

Two-shuttle identity plus payload experiment:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true \
  enable_room315_vla_dataset_recorder:=true \
  room315_right_shuttle_count:=2 \
  room315_right_start_slots:=1,3 \
  room315_left_shuttle_count:=0 \
  room315_visual_debug_colors:=false \
  room315_show_device_markers:=false \
  room315_vla_dataset_dir:=~/room315_multi_shuttle_vla
```

The world preloads R1/R2 with physical perimeter identity frames. Use the
payload models in `mfja_3rd_floor_description/models/room315_vla_payload_*` and
the metadata definitions in
`mfja_robot_control_config/config/room_315_vla/payload_scenarios.yaml` for
controlled loaded/unloaded and partial-occlusion scenarios.

Four-plus-four visual identity smoke test:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true \
  room315_right_shuttle_count:=4 \
  room315_right_start_slots:=1,2,3,4 \
  room315_left_shuttle_count:=4 \
  room315_left_start_slots:=1,2,3,4 \
  room315_visual_debug_colors:=false \
  room315_show_device_markers:=false
```

Check that status registers all shuttles and that the visible labels are mounted
on the shuttle bodies:

```bash
ros2 topic echo /room_315/vla/status std_msgs/msg/String
```

Fleet-safety rejection smoke test:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"shuttle\",\"side\":\"right\",\"shuttle_id\":\"R2\",\"command\":\"ON\",\"next_block\":\"A12E\",\"headway_blocks_ahead\":0}'}"
```

If another shuttle occupies or reserves the target block, the supervisor rejects
the command and increments the relevant safety metric.

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
