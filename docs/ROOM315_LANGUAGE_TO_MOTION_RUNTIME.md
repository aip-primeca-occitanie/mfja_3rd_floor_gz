# Room 315 English command-to-motion runtime

This runtime connects a confirmed English command to supervised shuttle motion:

```text
English text
  -> validated TaskGoal
  -> accepted visual state + deterministic presence registry
  -> Room 315 PDDL problem
  -> PlanSys2/POPF plan
  -> one supervised primitive
  -> accepted visual re-observation
  -> replan or success
```

The current runtime accepts transport goals to slots 1 through 4. Explicit
identity, nearest, loaded, empty, and any-shuttle selection are grounded from
the accepted visual facts before planning. Unsupported goals fail closed.

## Source ownership

- Shuttle presence is read from
  `/room_315/rails/left/shuttles/state` and
  `/room_315/rails/right/shuttles/state`.
- Presence gating uses only `ShuttleState.name`, `header.stamp`, and ROS receive
  time.
- Block/segment, bounding box, `s_m`, `s_ratio`, segment length, and loaded
  state come from the visual model.
- `ShuttleState.current_segment`, `s`, `x`, `y`, `z`, `yaw`, and `speed` are
  never used for visual localization.
- Controller mode is used only to confirm that a supervised `OFF` command took
  effect after visual arrival.
- Switch and stopper state come from their deterministic controllers.

The Gazebo `ShuttleState` topics are acceptable deterministic presence sources
for this simulation integration. They are not proof that the physical
installation has equivalent sensing. Real hardware must provide a
PLC/controller inventory or shuttle-presence provider through the same
`PresenceProvider` boundary.

## Safety behavior

Execution is disabled by default and must be enabled explicitly. The gateway
rejects the complete observation and does not command motion when visual state,
presence, supervisor state, identity mapping, topology, or target occupancy is
unknown or stale. It sends `stop_all` on an execution exception or shutdown
during an active goal.

The current Gazebo operator recipe disables removable VLA obstacles and hides
device markers. Hiding markers does not disable the real switch/stopper
controller state used by the supervisor.

The visual decoder may project the redundant `s_ratio` coordinate from the
learned `s_m` and learned segment length when their mismatch is no more than
the configured bound. This projection does not read controller pose, Gazebo
pose, slot priors, or oracle labels. Larger mismatches remain rejected.

Initial planner occupancy and target stopping use separate visual tolerances.
Target arrival requires the learned segment to match the target segment and
the learned `s_ratio` to remain within `0.05` of the authoritative slot ratio
for three consecutive accepted camera observations. Predicted segment length
is not used to widen this stopping window.

## Terminal 1: Gazebo, cameras, supervisor, and deterministic presence

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  start_paused:=false \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true \
  enable_room315_vla_camera_bridge:=true \
  enable_room315_vla_obstacles:=false \
  room315_show_device_markers:=false \
  room315_visual_debug_colors:=false \
  room315_identity_selection_mode:=explicit \
  room315_left_shuttle_count:=2 \
  room315_left_active_identities:=L2,L4 \
  room315_left_start_slots:=1,3 \
  room315_left_loaded_shuttles:=L4 \
  room315_right_shuttle_count:=1 \
  room315_right_active_identities:=R4 \
  room315_right_start_slot:=2 \
  room315_right_loaded_shuttles:=R4 \
  room315_shuttles_start_enabled:=false
```

## Terminal 2: visual-state inference

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH="/usr/lib/python3/dist-packages:/home/tiago/room315_local_training/venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

ros2 launch mfja_robot_control_config room_315_visual_state_runtime.launch.py \
  use_sim_time:=true \
  enable_camera_bridge:=false \
  device:=cuda \
  checkpoint_path:=/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run/best.pt \
  sidecar_directory:=/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run \
  dry_run_state_fusion:=true \
  plansys2_update_enabled:=false
```

Wait until diagnostics report at least one accepted frame:

```bash
ros2 topic echo /room_315/visual_state/observed_state \
  mfja_rail_interfaces/msg/VisualStateObservation --once
```

The message must contain `accepted: true`, `presence_ready: true`, and
`state_fusion_ready: true`.

## Terminal 3: PlanSys2 and closed-loop execution gateway

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

ros2 launch mfja_robot_control_config room_315_task_execution.launch.py \
  use_sim_time:=true \
  execution_enabled:=true \
  enable_plansys2:=true \
  external_obstacles_disabled:=true
```

Readiness checks:

```bash
ros2 lifecycle get /planner
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
```

The planner must be `active`.

## Terminal 4: interactive English commands

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz
source /opt/ros/jazzy/setup.bash
source /home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash
source /home/tiago/models/room315_intent/room315_intent.env

export PYTHONPATH="/usr/lib/python3/dist-packages:$PWD/mfja_robot_control_config/scripts:${PYTHONPATH:-}"

python3 mfja_robot_control_config/scripts/room_315_task_goal_cli.py \
  --config "$ROOM315_TASK_GOAL_LOCAL_CONFIG" \
  --publish-topic /room_315/task_goal \
  --result-topic /room_315/task_goal/status \
  --wait-for-result \
  --result-timeout-s 180
```

Examples:

```text
Move shuttle R4 to slot 3 on the right rail.
Move the nearest loaded shuttle on the right rail to slot 3.
Move an empty shuttle on the left rail to slot 2.
```

Reply `yes` after checking the confirmation summary.

## Optional Terminal 5: status monitor

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /room_315/task_goal/status std_msgs/msg/String
```

Terminal results are `succeeded`, `aborted`, `rejected`, or `failed`, with a
fail-closed reason.
