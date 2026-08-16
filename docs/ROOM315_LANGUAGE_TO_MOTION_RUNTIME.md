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

The runtime deliberately exposes a finite, testable request contract. It does
not claim that arbitrary English is automatically a safe robot program.

Supported atomic requests are:

- transport one present shuttle to an exact slot (`1` through `4`) or a
  configured station;
- select that shuttle by explicit identity (`L1` through `L4`, `R1` through
  `R4`), `any`, or topology-nearest;
- optionally require `loaded`, `empty`, or `any` payload state; and
- inspect the Room 315 system, one rail, a present explicitly named shuttle,
  a slot, a configured station, or any present shuttle matching an optional
  payload filter, without actuating equipment.

`nearest` inspection is intentionally rejected before confirmation because an
atomic inspection goal has no separate slot/station reference from which
nearest could be measured. Name the shuttle, or use `any` with `loaded`,
`empty`, or `any`. Selector and payload fields are invalid for system, rail,
slot, and station inspections; they are never silently ignored.

Inspection is non-actuating and completes only after a newer accepted visual
observation is received. Replaying the same state ID or timestamp is treated as
unknown freshness and fails closed; it is never reported as a successful fresh
inspection. A successful inspection includes an `inspection_report` built from
that exact accepted observation, rather than from a later read of the visual
state topic. The report carries the observation state ID so the displayed facts
can be tied to the frame that satisfied the inspection.

The report is filtered to the requested scope. `Inspect the Room 315 system.`
shows the in-scope observation for every canonical shuttle, including explicit
absent or unknown states. `Inspect shuttle R2.` shows only R2. Rail, slot, and
station inspections are filtered in the same way. Missing or invalid visual
facts are shown as unknown; the runtime does not fill them from controller pose
or from a newer frame.

An explicit identity determines its authoritative rail, so commands such as
`Move L4 to slot 2` do not require the operator to repeat `left`. Colour aliases
are resolved to an authoritative identity before PDDL grounding; colours are
not PDDL objects. Station goals are grounded to a currently feasible,
sensor-backed station slot. Selection, payload state, and location are grounded
from one accepted visual observation before a planning problem is emitted.

Compound agendas, swaps, “move all”, load/unload operations, arbitrary metric
coordinates, deadlines, and unconstrained speed objectives are outside the
current `TaskGoal` schema. They must be clarified or rejected, not approximated
by an unrelated plan. A physically impossible request, insufficient rail
capacity, stale observation, unknown presence, unresolved identity, or unsafe
device state also fails closed.

## Topology and blocker planning

The planning problem is generated from the authoritative left/right rail
networks. It covers all 14 public segments and all 16 assignments of the four
switches on each rail. A shuttle observed only on a continuous segment uses a
segment-origin topology action. A shuttle with an exact occupied slot uses a
slot-origin topology action that removes the source occupancy atomically; this
prevents the symbolic state from leaving one shuttle in two slots.

Routes are occupancy-aware. For an obstructed route, the executive executes at
most one supervised blocker relocation, obtains a fresh accepted visual frame,
and replans. It never assumes that the rest of a stale multi-step plan is still
valid. Segment-origin and slot-origin topology setup both require the verified
normal route; after an interior relocation, PDDL must therefore emit the
matching `finish_*_route_clearance` action before configuring the goal route.
The A34I interior branch has bounded physical capacity. When a
three-blocker route cannot safely stage another shuttle there, an explicit
`pause_route_clearance` phase is permitted only after all staged shuttles have
matching stop certificates. The normal route is then verified, a staged token
is advanced to a free intermediate slot, and clearance resumes. This avoids
overlapping shuttles in A34I while retaining a complete path for full-rail
requests.

The expert and runtime PDDL domains share one executable action surface. Every
symbolic action is translated, validated, supervised, effect-checked, and
followed by re-observation before subsequent motion.

The switch supervisor independently measures each controller shuttle state's
along-rail distance to the authoritative controlled switch node. A switch
change is rejected at or below `0.35 m`; missing, invalid, or unmappable
controller coordinates fail closed. These controller fields are a safety veto
only and never replace or improve the learned visual localization used by the
planner.

## Source ownership

- Shuttle presence is read from
  `/room_315/rails/left/shuttles/state` and
  `/room_315/rails/right/shuttles/state`.
- Presence gating uses only `ShuttleState.name`, `header.stamp`, and ROS receive
  time.
- Block/segment, bounding box, `s_m`, `s_ratio`, segment length, and loaded
  state come from the visual model.
- Inspection output preserves that distinction: presence is controller-derived,
  while block, along-rail position, bounding box, and payload classification are
  visual-model facts from the report's accepted frame.
- The displayed payload score is an uncalibrated loaded/empty decision score.
  It is labelled `payload decision score`, not payload confidence or a
  calibrated probability.
- `ShuttleState.current_segment`, `s`, `x`, `y`, `z`, `yaw`, and `speed` are
  never used for visual localization.
- A fresh controller `DISABLED` mode is used only to confirm that a supervised
  `OFF` command took effect after visual arrival. `WAITING` is not accepted as
  that proof because an enabled shuttle held by a stopper or collision also
  reports `WAITING`.
- In the Gazebo controller, `ShuttleState.speed` is the retained travel-speed
  setting for the next `ON` command, not an instantaneous velocity. It may
  remain non-zero while mode is `DISABLED`; stop verification must therefore
  require a fresh explicit `DISABLED` state and must not reinterpret this field
  as motion evidence.
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

Initial planner occupancy comes from the accepted visual state. Final actuation
stopping is a separate effect-verification boundary: the target's
identity-bearing deterministic slot sensor and a confirmed controller `OFF`
state must agree. A fresh visual observation is retained as model evidence and
for replanning, but controller position fields never replace learned visual
localization. Predicted segment length is not used to widen the target.

When a completed topology-origin move leaves the rail in a mixed switch
configuration, PlanSys2 may select `restore_normal_route` before the next slot
motion. The executive expands that action into supervised all-exterior switch
and all-open stopper commands, verifies both device effects, waits for a fresh
accepted visual frame, and then replans. Ordinary recovery is unavailable
during an active blocker-clearance phase; the bounded-capacity pause described
above is a distinct guarded transition. Restoration is also unavailable with a
non-open stopper, an external obstacle, or when an interior shuttle lacks a
matching sensor-derived stop certificate. A certificate is bound to the
current accepted visual segment and is invalidated before any later `ON`
command for that shuttle.

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
Move L4 to slot 2.
Inspect shuttle R2.
Inspect any loaded shuttle on the right rail.
Inspect the Room 315 system.
```

Reply `yes` after checking the confirmation summary.

For a successful `Inspect ...` command, the CLI prints both the complete
published result as JSON for audit and automation, including
`result.inspection_report`, and a human-readable summary of that same report.
Each in-scope shuttle row identifies its presence state and source, the
available visual block and along-rail position, payload classification, and
model scores. The enclosing report records the exact observation state ID.
Thus a system inspection lists the Room 315 shuttle observations, while an R2
inspection displays only what the accepted frame says about R2. The human
summary is a rendering of the JSON report; it does not perform a second topic
read.

## Optional Terminal 5: status monitor

```bash
cd /home/tiago/mfja_3rd_floor_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /room_315/task_goal/status std_msgs/msg/String
```

Terminal results are `succeeded`, `aborted`, `rejected`, or `failed`, with a
fail-closed reason. Successful inspection results additionally contain the
same-frame machine-readable inspection report described above.
