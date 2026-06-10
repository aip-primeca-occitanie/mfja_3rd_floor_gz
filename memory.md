# MFJA Room 315 VLA Project Memory

Use this file as a context handoff for another AI assistant. It summarizes the
project, current architecture, important commands, current VLA research intent,
and the latest design decisions. The strongest focus is the Room 315
Vision-Language-Action layer.

## One-Sentence Summary

This repository is a ROS 2 Jazzy + Gazebo Harmonic simulation of the MFJA third
floor, with a Room 315 rail-cell research setup where a VLA policy should learn
from language, overhead camera images, and previous command only, while expert
code may use sensors and Gazebo state internally to generate safe demonstrations
and evaluation labels.

## Repository Location and Shape

Typical workspace:

```text
~/mfja_3rd_floor_ros2_ws
```

Repository root:

```text
~/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz
```

The repository root is not itself a ROS package. It contains multiple packages:

```text
mfja_3rd_floor_description/     Gazebo worlds, SDF models, URDF, meshes, tests.
mfja_3rd_floor_bringup/         Main launch entry points.
mfja_robot_control_config/      Rail control, kinematic shuttle nodes, VLA stack.
mfja_rail_interfaces/           Custom ROS 2 messages/services for rail control.
mfja_room_315_bringup/          Room 315-related bringup package.
docs/                           Technical docs.
runbook.html                    General operation runbook.
runbookvla.html                 VLA-specific runbook.
```

Build from the workspace root, not from the repo root:

```bash
cd ~/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz
source install/setup.bash
```

Every new terminal must source:

```bash
cd ~/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Project Goal

The project models a manufacturing/education rail cell in Room 315. Shuttles
move around two rail loops, with switches, stoppers, binary sensors, station
slots, overhead VLA cameras, and optional industrial robots around the stations.

The VLA research goal is not to expose perfect simulator state to the learned
model. The expert/supervisor may use binary sensors and Gazebo state to produce
correct demonstrations, but the deployable VLA model should infer the useful
state visually from camera frames and language.

## Room 315 Rail Cell

There are two rail sides:

```text
right
left
```

Each side has:

```text
shuttle(s)
switches A1..A4
stoppers A1..A4
slot sensors DZI1..DZI4
approach / branch sensors such as DA3IR and DA3IL
```

Station mapping:

```text
Right rail slots 1-2: Yaskawa HC10DT
Right rail slots 3-4: Staubli TX2
Left rail slots 1-2: Yaskawa HC10
Left rail slots 3-4: KUKA KR6
```

The shuttle motion is kinematic-first. The shuttle follows a calibrated
arc-length rail graph from CSV path geometry. Gazebo model poses are updated
through `/world/<world_name>/set_pose`; wheel physics/contact dynamics are not
the main rail-motion mechanism.

## Important Launch Files

Room 315 only:

```text
mfja_3rd_floor_bringup/launch/room_315_only.launch.py
```

Full floor:

```text
mfja_3rd_floor_bringup/launch/full_floor.launch.py
```

Base Gazebo + robot launch:

```text
mfja_robot_control_config/launch/multi_robot_sim.launch.py
```

Room 315 dual kinematic shuttles:

```text
mfja_robot_control_config/launch/room_315_dual_kinematic_shuttles.launch.py
```

VLA supervisor/agent/recorder/benchmark launch:

```text
mfja_robot_control_config/launch/room_315_vla_supervisor.launch.py
```

## Lightweight Gazebo / Visual Settings

For VLA data collection, prefer realistic and uncluttered visuals:

```text
room315_visual_debug_colors:=false
room315_show_device_markers:=false
```

`room315_show_device_markers:=false` hides rail sensor and stopper markers from
the camera. This is important: the model should not learn from artificial sensor
markers in the image.

To make Gazebo lighter, use:

```text
gui_config:=config/mfja_light.gui.config
```

The world files already disable scene shadows with:

```xml
<shadows>false</shadows>
```

The light GUI config is better for performance, but it does not show the full
move/rotate toolbar. To move objects interactively and see coordinates, use:

```text
gui_config:=config/mfja_default.gui.config
```

If the obstacle is moved from the Gazebo GUI only, the VLA obstacle pose cache is
not updated. For m10/m11 to use the new obstacle position, move the obstacle via
the obstacle tool described below.

## Basic VLA Launch

Terminal 1:

```bash
cd ~/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

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
  room315_visual_debug_colors:=false \
  room315_show_device_markers:=false \
  enable_room315_vla:=true
```

Terminal 2, watch VLA status:

```bash
ros2 topic echo /room_315/vla/status std_msgs/msg/String
```

## VLA Design Boundary

This is the most important research rule:

The learned policy must use:

```text
model_input_schema_version: 3
model_input.language
model_input.overhead_images
model_input.last_command
```

The learned policy must not use:

```text
binary sensor bits
switch/stopper states
shuttle command state
true shuttle segment
arc-length / normalized position
Gazebo pose
distance to switch
reset internals
privileged evaluation labels
```

Those privileged values may still appear in:

```text
privileged_eval
structured_rail_state
observation.state
debug fields
```

They are for expert generation, reset, audits, baseline ablations, and oracle
evaluation. They are not deployable VLA policy input.

## VLA Cameras

The VLA uses independent right/left overhead RGB-D cameras.

ROS image topics:

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

In model input, the image names are:

```text
right_rail_rgb
left_rail_rgb
```

## VLA Supervisor

Core file:

```text
mfja_robot_control_config/scripts/room_315_vla_supervisor.py
```

Config:

```text
mfja_robot_control_config/config/room_315_vla/vla_supervisor.yaml
```

Action space:

```text
mfja_robot_control_config/config/room_315_vla/action_space.yaml
```

Main command topic:

```text
/room_315/vla/command
```

Main status topic:

```text
/room_315/vla/status
```

The supervisor supports:

```text
Event-level direct model outputs via action_vector
Equivalent primitive JSON commands
High-level route_template tasks for demos and benchmarks
Safety decoder and guarded execution
Task status tracking
Active/completed tasks in JSON status
```

Allowed action names include:

```text
route_template
route_shuttle
switches
stoppers
shuttle
add_shuttle
stop_all
emergency_stop
clear_emergency_stop
status
```

The preferred learned-model output is an event-level direct symbolic command:
usually `{"action_vector": [...]}` following schema v2, or an equivalent
primitive JSON command such as `switches`, `stoppers`, `shuttle`, `stop_all`, or
`emergency_stop`. `route_template` commands are for expert demonstration
generation, manual checks, and benchmark orchestration; they are not the final
model-output interface.

Canonical route templates:

```text
right_yaskawa_to_staubli
right_staubli_to_yaskawa
left_yaskawa_to_kuka
left_kuka_to_yaskawa
right_enter_interior_loop
left_enter_interior_loop
```

Example model-style event-level action vector sent directly to the supervisor:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action_vector\":[2,0,0,0,1,0,0,0,2,0,0,0,0,0,0,0,0,0,0.0,1,3,8]}'}"
```

This decodes to `SET_SWITCHES` on the right side, selecting A3 as `INTERIOR`,
then passes through the same safety decoder. Example demo/benchmark task
template command:

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"route_template\",\"template\":\"right_yaskawa_to_staubli\"}'}"
```

## Safety Decoder

The supervisor never blindly executes a model proposal. It normalizes and
validates commands, then either accepts/corrects safe normalizations or rejects
unsafe proposals.

Important safety behavior:

```text
Rejects unsafe switch changes near occupied guarded switch segments.
Rejects loop transitions unless the shuttle is stopped at the side-specific gate.
Rejects unsafe shuttle ON primitives without wait condition/target.
Always allows emergency stop proposals.
Publishes safety metrics in /room_315/vla/status.
```

Safety metrics include:

```text
total_proposed_actions
accepted_actions
rejected_actions
illegal_proposal_rate
rejection_reasons
```

## Event-Level Action Schema

Action schema:

```text
schema_version: 2
model_input_schema_version: 3
```

The policy predicts event-level direct symbolic actions, not high-level
`route_template` tasks, not repeated framewise Gazebo commands, and not raw
Gazebo controls.

Primitive IDs:

```text
WAIT: 0
DONE: 1
SET_SWITCHES: 2
SET_STOPPERS: 3
SHUTTLE_ON: 4
STOP_NOW: 5
EMERGENCY_STOP: 6
```

Action vector fields:

```text
primitive_id
side_id
switch_mask_A1..A4
switch_value_A1..A4
stopper_mask_A1..A4
stopper_value_A1..A4
speed_mps
wait_condition_id
target_id
reason_id
```

Device masks matter. A mask of `0` means `UNCHANGED`; only selected devices are
changed. This prevents a model action like "set A3 interior" from accidentally
changing A1/A2/A4.

## Real VLA Agent

Core file:

```text
mfja_robot_control_config/scripts/room_315_real_vla_agent.py
```

The agent reads:

```text
/room_315/vla/user_goal
/room_315/vla/status
right/left overhead images
```

It sends:

```text
/room_315/vla/command
/room_315/vla/agent_status
```

Only HTTP provider is currently supported.
The preferred HTTP model response is:

```json
{"action_vector": [2, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 1, 3, 8]}
```

Equivalent primitive JSON commands are also accepted for compatibility, but the
research target is the event-level action vector. The agent sends the provider
`preferred_model_output: "action_vector"`, `event_action_vector_fields`, and
`event_primitive_ids`. The `allowed_actions` sent to the model intentionally
excludes `route_template`; route templates remain accepted by the supervisor for
manual/expert paths, but are not offered as the model contract.

Launch with agent:

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
  room315_visual_debug_colors:=false \
  room315_show_device_markers:=false \
  enable_room315_vla:=true \
  enable_room315_real_vla_agent:=true \
  room315_vla_agent_provider:=http \
  room315_vla_agent_http_endpoint:=http://127.0.0.1:8000/plan
```

Send a high-level user goal:

```bash
ros2 topic pub --once /room_315/vla/user_goal std_msgs/msg/String \
  "{data: 'move the right shuttle from Yaskawa to Staubli'}"
```

## Dataset Recorder

Core file:

```text
mfja_robot_control_config/scripts/room_315_vla_dataset_recorder.py
```

Episode control topic:

```text
/room_315/vla/episode_control
```

Dataset status topic:

```text
/room_315/vla/dataset_status
```

Launch recorder:

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
  room315_visual_debug_colors:=false \
  room315_show_device_markers:=false \
  enable_room315_vla:=true \
  enable_room315_vla_dataset_recorder:=true \
  room315_vla_dataset_dir:=~/room315_smolvla_demo
```

Manual episode control:

```bash
ros2 topic pub --once /room_315/vla/episode_control std_msgs/msg/String \
  "{data: 'start manual VLA test'}"

ros2 topic pub --once /room_315/vla/episode_control std_msgs/msg/String \
  "{data: 'stop success'}"
```

Dataset layout:

```text
episodes/<episode_id>/events.jsonl
episodes/<episode_id>/data.jsonl
episodes/<episode_id>/images/...
meta/training_events.jsonl
```

Training rule:

```text
Train on events.jsonl or meta/training_events.jsonl.
Do not train on data.jsonl.
```

`events.jsonl` contains decision events:

```text
observation_before_decision -> next_direct_symbolic_event_action
```

`data.jsonl` is raw framewise replay/debug. It can repeat commands and should
not be used as supervised action labels.

## Reset and Dataset Recording

The teleop generator resets shuttles after scenarios, but reset must not be
recorded as part of the learned task. The generator waits for the recorder to
confirm stop before reset. This avoids recording reset frames or reset commands.

Important:

```text
Reset is not part of the user command.
Reset should not be model training data.
Do not include reset frames in the VLA learning target.
```

## Event Extractor and Baseline Eval

Flatten training events:

```bash
ros2 run mfja_robot_control_config room_315_vla_event_extractor.py \
  ~/room315_smolvla_demo \
  --output meta/training_events.jsonl
```

Run baseline evaluator:

```bash
ros2 run mfja_robot_control_config room_315_vla_baseline_eval.py \
  ~/room315_smolvla_demo \
  --output-dir ~/room315_vla_baselines \
  --holdout-fraction 0.2
```

Baseline families:

```text
state_only   Non-deployable ablation using sparse binary state.
vla          Language + overhead images, matching deployable input.
oracle       Privileged replay upper bound.
```

Metrics include:

```text
task_success
action_accuracy
primitive_accuracy
side_accuracy
device_accuracy
completion_time
command_count
illegal_proposal_rate
rejected_action_rate
visual_target_success
obstacle_stop_success
```

## Benchmark Runner

Core file:

```text
mfja_robot_control_config/scripts/room_315_vla_benchmark_runner.py
```

Status topic:

```text
/room_315/vla/benchmark_status
```

Launch benchmark:

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
  room315_visual_debug_colors:=false \
  room315_show_device_markers:=false \
  enable_room315_vla:=true \
  enable_room315_vla_dataset_recorder:=true \
  room315_vla_dataset_dir:=~/room315_smolvla_demo \
  enable_room315_vla_benchmark_runner:=true \
  room315_vla_benchmark_tasks:=all \
  room315_vla_benchmark_report_dir:=~/room315_vla_benchmarks
```

Task selection examples:

```text
room315_vla_benchmark_tasks:=all
room315_vla_benchmark_tasks:=transport
room315_vla_benchmark_tasks:=loop_entry
room315_vla_benchmark_tasks:=right_yaskawa_to_staubli,left_yaskawa_to_kuka
```

## Teleop Scenario Generator

Core file:

```text
mfja_robot_control_config/scripts/vla_teleop_generator.py
```

It generates expert demonstrations by issuing deterministic VLA JSON commands.
Internally it may use sensors and rail state to ensure correctness. The model
must still learn only from model_input.

Run one scenario by code:

```bash
ros2 run mfja_robot_control_config vla_teleop_generator.py --ros-args \
  -p scenario_names:=r02
```

Run one scenario by name:

```bash
ros2 run mfja_robot_control_config vla_teleop_generator.py --ros-args \
  -p scenario_names:=right_obstacle_aware_route
```

Run multiple selected scenarios:

```bash
ros2 run mfja_robot_control_config vla_teleop_generator.py --ros-args \
  -p scenario_names:=r02,m08,m10
```

Reset after each scenario is enabled by default:

```text
reset_after_each_scenario:=true
```

Disable only for special debugging:

```bash
ros2 run mfja_robot_control_config vla_teleop_generator.py --ros-args \
  -p scenario_names:=r02,r03 \
  -p reset_after_each_scenario:=false
```

## Current Teleop Scenarios

Right rail scenarios:

```text
r01 move_right_shuttle_full_exterior_loop
r02 move_right_shuttle_from_yaskawa_to_staubli
r03 move_right_shuttle_from_staubli_to_yaskawa
r04 go_to_staubli_on_right_rail
r05 go_to_yaskawa_on_right_rail
r06 stop_right_shuttle_at_stopper_A1
r07 stop_right_shuttle_at_stopper_A2
r08 stop_right_shuttle_at_stopper_A3
r09 stop_right_shuttle_at_stopper_A4
r10 right_shuttle_enter_interior_loop_from_exterior
r11 move_right_shuttle_on_interior_loop
r12 route_right_shuttle_through_A3_into_the_interior_branch
r13 pass_right_shuttle_through_A4_from_the_interior_approach
r14 right_shuttle_stop_at_A3_then_resume_and_stop_at_A4
r15 right_shuttle_stop_at_A2_then_resume_and_stop_at_A4
r16 complete_one_fast_right_exterior_loop
r17 complete_one_slow_right_exterior_loop
```

Left rail scenarios:

```text
l01 move_left_shuttle_full_exterior_loop
l02 move_left_shuttle_from_yaskawa_to_kuka
l03 move_left_shuttle_from_kuka_to_yaskawa
l04 go_to_kuka_on_left_rail
l05 go_to_yaskawa_on_left_rail
l06 stop_left_shuttle_at_stopper_A1
l07 stop_left_shuttle_at_stopper_A2
l08 stop_left_shuttle_at_stopper_A3
l09 stop_left_shuttle_at_stopper_A4
l10 left_shuttle_enter_interior_loop_from_exterior
l11 move_left_shuttle_on_interior_loop
l12 route_left_shuttle_through_A3_into_the_interior_branch
l13 pass_left_shuttle_through_A4_from_the_interior_approach
```

Mixed / visual research scenarios currently kept:

```text
m02 emergency_stop_all
m08 left_slot3_kuka_then_slot2
m10 right_obstacle_aware_route
m11 left_obstacle_aware_route
```

Removed visual/recovery scenarios:

```text
m01 removed: stopper-only/no real visual research value.
m03 removed: unknown-position recovery was synthetic and relied too much on sensors.
m07 removed: sensor-dropout route was internal and not model-facing.
m09 removed: stopper-only visual obstacle stop was not a real movable visual obstacle task.
```

## Meaning of Current m Scenarios

`m02 emergency_stop_all`:

```text
Prepares the right rail, starts motion, then sends stop_all. Useful for safety
command data.
```

`m08 left_slot3_kuka_then_slot2`:

```text
Moves the left shuttle to slot 3, sends a KUKA joint trajectory
KUKA_SLOT3_INTERLOCK_POSITIONS_RAD for 4 seconds, then moves the left shuttle to
slot 2. This tests rail + robot sequencing.
```

`m10 right_obstacle_aware_route`:

```text
Right rail exterior-loop task. If the right visual obstacle is close enough to
the exterior-loop path, the shuttle stops before it and the scenario ends. If
the obstacle is clear or hidden, the shuttle completes one exterior loop.
```

`m11 left_obstacle_aware_route`:

```text
Same as m10, but for the left rail and left visual obstacle.
```

## Visual Obstacle System

Obstacle model:

```text
mfja_3rd_floor_description/models/room315_vla_removable_obstacle_marker/model.sdf
```

Current obstacle body size:

```text
0.13 x 0.13 x 0.13 m cube
```

Obstacle entity names in Gazebo:

```text
room315_vla_right_obstacle_marker
room315_vla_left_obstacle_marker
```

Default obstacle poses in the worlds:

```text
right: x=-14.18, y=-4.68, z=0.856, yaw=0.0
left:  x=-9.86,  y=-4.68, z=0.856, yaw=0.0
```

Obstacle tool:

```text
mfja_robot_control_config/scripts/room_315_vla_obstacle_tool.py
```

Move right obstacle:

```bash
ros2 run mfja_robot_control_config room_315_vla_obstacle_tool.py \
  --side right --x -14.18 --y -4.68 --z 0.856 --yaw 0.0
```

Move left obstacle:

```bash
ros2 run mfja_robot_control_config room_315_vla_obstacle_tool.py \
  --side left --x -9.86 --y -4.68 --z 0.856 --yaw 0.0
```

Move right obstacle away so m10 completes a loop:

```bash
ros2 run mfja_robot_control_config room_315_vla_obstacle_tool.py \
  --side right --x -15.24 --y -5.54 --z 0.856 --yaw 0.0
```

Hide obstacle without deleting it:

```bash
ros2 run mfja_robot_control_config room_315_vla_obstacle_tool.py \
  --side right --x -14.18 --y -4.68 --z -5.0 --yaw 0.0
```

The tool calls Gazebo `/world/<world>/set_pose` and writes:

```text
~/.ros/room315_vla_obstacles.json
```

m10/m11 read this pose file inside the same Gazebo run. The scenarios do not
move the obstacle themselves.

Important obstacle cache behavior:

```text
room_315_only.launch.py and full_floor.launch.py clear the obstacle pose cache
at simulation startup by default.
```

This is intentional. Gazebo resets the obstacle to the world-file pose on
restart, so keeping the old cache would make m10/m11 stop at stale obstacle
coordinates. To intentionally preserve the cache across launches:

```text
room315_clear_vla_obstacle_pose_cache:=false
```

m10/m11 obstacle parameters:

```text
manual_obstacle_pose_file: ~/.ros/room315_vla_obstacles.json
<side>_manual_obstacle_use_pose_file: true
<side>_manual_obstacle_path_threshold_m: 0.45
<side>_manual_obstacle_hidden_z_threshold_m: 0.2
<side>_manual_obstacle_stop_before_m: 0.20
```

Obstacle logic:

```text
If obstacle z < hidden_z_threshold, treat as hidden/non-blocking.
Convert obstacle Gazebo xy to rail xy using side-specific calibration.
Project obstacle onto the exterior-loop polyline.
If path offset <= path_threshold, obstacle blocks the path.
Run exterior loop and stop stop_before_m before the obstacle projection.
If not blocking, complete one exterior-loop lap.
```

## Why Sensors Are Still Used Internally

Expert generation should use sensors internally to make correct demonstrations.
This is good and intentional. The expert can use:

```text
slot sensors
stopper sensors
switch state
segment/arc-length
Gazebo state
```

The learned policy should not receive those as policy input. It should receive
only:

```text
language
overhead camera images
last command
```

This separation gives the research setup a clean oracle/expert vs learned-policy
boundary.

The learned policy output should also stay clean: it should emit the next
event-level command/action vector directly. Do not make the learned policy output
`route_template` as its final interface; route templates are a demo/benchmark
tool used to generate and validate episodes.

## Why m03 Was Removed

m03 represented unknown-position recovery. It had low research value because it
made the expert rely too directly on sensors to recover a state that the learned
model should infer from camera images. It risked teaching a sensor shortcut
rather than a visual policy.

## Why m09 Was Removed

m09 used a stopper as a visual/physical obstacle. It was not the same as the
movable visual obstacle marker used in m10/m11. Since it was stopper-only and
not a true obstacle-aware visual task, it was removed to keep the scenario set
research-clean.

## What an AI Assistant Should Not Do

Do not:

```text
Do not re-add m09 unless explicitly requested.
Do not train on reset frames.
Do not train on data.jsonl repeated frame rows.
Do not put binary sensors or Gazebo pose into model_input.
Do not make the learned model output route_template as the final action.
Do not make the obstacle scenario move the obstacle internally.
Do not leave duplicate scenarios that teach the same thing.
Do not expose sensor/stopper markers in VLA camera datasets.
Do not assume GUI obstacle moves update the pose cache.
```

Do:

```text
Keep model_input schema version 3 limited to language, overhead_images, last_command.
Make the learned model output event-level action_vector or equivalent primitive JSON.
Use privileged state only for expert generation, reset, audits, and oracle evaluation.
Use event-level labels from events.jsonl.
Use room315_visual_debug_colors:=false for realistic datasets.
Use room315_show_device_markers:=false for realistic datasets.
Use m10/m11 for real visual obstacle-aware behavior.
```

## Important Files for VLA Work

```text
runbookvla.html
docs/ROOM315_VLA_OPERATIONS.md
docs/ROOM315_VLA_RESEARCH.md

mfja_robot_control_config/scripts/room_315_vla_supervisor.py
mfja_robot_control_config/scripts/room_315_real_vla_agent.py
mfja_robot_control_config/scripts/room_315_vla_dataset_recorder.py
mfja_robot_control_config/scripts/room_315_vla_event_extractor.py
mfja_robot_control_config/scripts/room_315_vla_baseline_eval.py
mfja_robot_control_config/scripts/room_315_vla_benchmark_runner.py
mfja_robot_control_config/scripts/vla_teleop_generator.py
mfja_robot_control_config/scripts/room_315_vla_obstacle_tool.py

mfja_robot_control_config/config/room_315_vla/vla_supervisor.yaml
mfja_robot_control_config/config/room_315_vla/action_space.yaml

mfja_3rd_floor_bringup/launch/room_315_only.launch.py
mfja_3rd_floor_bringup/launch/full_floor.launch.py
mfja_robot_control_config/launch/room_315_vla_supervisor.launch.py

mfja_3rd_floor_description/worlds/room_315_only.world
mfja_3rd_floor_description/worlds/mfja_3rd_floor.world
mfja_3rd_floor_description/models/room315_vla_overhead_devices/model.sdf
mfja_3rd_floor_description/models/room315_vla_removable_obstacle_marker/model.sdf
```

## Useful Tests

Run focused tests after VLA/scenario changes:

```bash
cd ~/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz
pytest -q \
  mfja_robot_control_config/test/test_vla_teleop_generator.py \
  mfja_robot_control_config/test/test_room315_vla_smoke.py \
  mfja_robot_control_config/test/test_room315_vla_baseline_eval.py \
  mfja_3rd_floor_description/test/test_room315_vla_camera_model.py
```

Run broader tests:

```bash
pytest -q mfja_robot_control_config/test
pytest -q mfja_3rd_floor_description/test
git diff --check
```

Rebuild relevant packages after code/launch/model changes:

```bash
cd ~/mfja_3rd_floor_ros2_ws
colcon build --packages-select mfja_robot_control_config
colcon build --packages-select mfja_3rd_floor_bringup
colcon build --packages-select mfja_3rd_floor_description
source install/setup.bash
```

## Common Debug Commands

Watch VLA status:

```bash
ros2 topic echo /room_315/vla/status std_msgs/msg/String
```

Watch dataset status:

```bash
ros2 topic echo /room_315/vla/dataset_status std_msgs/msg/String
```

Watch benchmark status:

```bash
ros2 topic echo /room_315/vla/benchmark_status std_msgs/msg/String
```

Watch agent status:

```bash
ros2 topic echo /room_315/vla/agent_status std_msgs/msg/String
```

Check launch arguments:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
```

## Current Research Interpretation

This project is best understood as a controlled VLA benchmark, not as a pure
physics robotics simulator. The rail world provides rich enough perception:

```text
shuttle location between sparse sensors
station occupancy from visual fiducials
switch/stopper consequences
obstacle go/stop decisions
left/right rail identity
station-to-station language goals
```

The deterministic expert should generate clean, safe behavior. The learned VLA
policy should learn the task from images and language while the supervisor
remains a safety gate.

For research credibility, keep the split clear:

```text
Expert: can use privileged sensors/state.
Model: language + images + previous command.
Evaluation: may use privileged labels, oracle, and state-only ablations.
Deployment path: model proposes symbolic action; supervisor validates before execution.
```
