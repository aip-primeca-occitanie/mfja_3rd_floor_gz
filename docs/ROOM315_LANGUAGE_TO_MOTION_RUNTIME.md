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

The current Gazebo operator recipe disables removable visual obstacles and hides
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

## Prerequisites for Closed-Loop Execution

This workflow is optional and artifact-dependent. A fresh clone can run the
simulation and deterministic rails, but it cannot start the learned
command-to-motion path by itself. Before continuing, obtain all of the
following through the project's controlled release process:

- a complete active V4 promotion bundle, including the checkpoint and every
  required artifact resolved by `runtime_promotion_manifest.json`;
- the independently published SHA-256 of that manifest;
- the matching task-execution authorization file;
- the independently published SHA-256 of the task authorization;
- the local intent-model checkpoint if semantic English parsing is required;
  and
- the isolated visual Python environment described in
  [Installation](INSTALLATION.md#visual-training-and-v4-inference).

The active V4 authorization is Gazebo-only. It is not approval for physical
hardware. Never replace the published digests with digests calculated from
untrusted files merely to make validation pass.

The task gateway is not generic across arbitrary V4 models. The current code
admits only candidate
`room315_visual_runtime_candidate_v4_seed31520260811_epoch11_869d6404_shadow`,
visual schema `room315.visual_state.v4`, and checkpoint SHA-256
`869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d`.
The active promotion and task authorization must be later, matching records
for that exact candidate. A different valid V4 visual bundle may run inference
but will be rejected by this task gateway.

Set portable workspace and artifact variables:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
export ROOM315_V4_BUNDLE="$HOME/room315_artifacts/visual_v4_active"
export ROOM315_V4_MANIFEST="$ROOM315_V4_BUNDLE/runtime_promotion_manifest.json"
export ROOM315_V4_MANIFEST_SHA256='<approved-64-character-sha256>'
export ROOM315_TASK_AUTH="$ROOM315_V4_BUNDLE/candidate_state.json"
export ROOM315_TASK_AUTH_SHA256='<approved-64-character-sha256>'
export ROOM315_TASK_EXEC_CONFIG="$HOME/.config/mfja/task_execution_runtime.yaml"
```

Verify both top-level files before launch:

```bash
printf '%s  %s\n' \
  "$ROOM315_V4_MANIFEST_SHA256" "$ROOM315_V4_MANIFEST" | sha256sum --check -
printf '%s  %s\n' \
  "$ROOM315_TASK_AUTH_SHA256" "$ROOM315_TASK_AUTH" | sha256sum --check -
```

Create a host-local task-execution configuration:

```bash
mkdir -p "$(dirname "$ROOM315_TASK_EXEC_CONFIG")"
cp "$MFJA_REPO/mfja_robot_control_config/config/room_315_task_execution/task_execution_runtime.yaml" \
  "$ROOM315_TASK_EXEC_CONFIG"
${EDITOR:-nano} "$ROOM315_TASK_EXEC_CONFIG"
```

Replace these three values in the copied YAML with literal absolute paths and
the published digest; ROS parameter YAML does not expand shell variables:

```yaml
task_execution_authorization_path: /absolute/path/to/candidate_state.json
task_execution_authorization_sha256: <approved-64-character-sha256>
task_execution_promotion_manifest_path: /absolute/path/to/runtime_promotion_manifest.json
```

Do not edit the source configuration to store a personal home-directory path.
The authorization, manifest, checkpoint allowlist, and runtime mode must all
describe the same approved V4 candidate.

Shell variables are local to one terminal. The blocks below repeat the values
needed by each foreground process; replace every placeholder consistently.

## Terminal 1: Gazebo, Cameras, Supervisor, and Presence

This high-level launch clears the disposable
`~/.ros/room315_visual_obstacles.json` cache by default. That matches the disabled
external-obstacle assumption used below. Add
`room315_clear_visual_obstacle_pose_cache:=false` only when a reviewed workflow
must preserve the cache, and never point the configurable cache path at an
unrelated file.

```bash
export MFJA_WS="$HOME/mfja_ws"

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  start_paused:=false \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_rail_safety_supervisor:=true \
  enable_room315_rgbd_camera_bridge:=true \
  enable_room315_visual_obstacles:=false \
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

Wait at least five seconds. Confirm `/clock`, both camera image topics, both
rail shuttle-state topics, and `/room_315/rail_safety/status` before continuing.

## Terminal 2: Active V4 Visual-State Inference

```bash
export MFJA_WS="$HOME/mfja_ws"
export ROOM315_V4_BUNDLE="$HOME/room315_artifacts/visual_v4_active"
export ROOM315_V4_MANIFEST="$ROOM315_V4_BUNDLE/runtime_promotion_manifest.json"
export ROOM315_V4_MANIFEST_SHA256='<approved-64-character-sha256>'

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
source "$HOME/.venvs/mfja-visual/bin/activate"
printf '%s  %s\n' \
  "$ROOM315_V4_MANIFEST_SHA256" "$ROOM315_V4_MANIFEST" | sha256sum --check -

ros2 launch mfja_robot_control_config room_315_visual_state_runtime.launch.py \
  use_sim_time:=true \
  enable_camera_bridge:=false \
  runtime_mode:=active \
  v4_promotion_manifest:="$ROOM315_V4_MANIFEST" \
  v4_promotion_manifest_sha256:="$ROOM315_V4_MANIFEST_SHA256" \
  device:=auto \
  dry_run_state_fusion:=true \
  plansys2_update_enabled:=false
```

The launch accepts `runtime_config`, `runtime_mode`,
`v4_promotion_manifest`, and `v4_promotion_manifest_sha256`; historical
`checkpoint_path` and `sidecar_directory` launch arguments are not supported.
The manifest itself binds the checkpoint and all supporting artifacts.

Wait for an accepted frame:

```bash
ros2 topic echo /room_315/visual_state/observed_state \
  mfja_rail_interfaces/msg/VisualStateObservation --once
```

The message must contain `accepted: true`, `presence_ready: true`, and
`state_fusion_ready: true`. If it does not appear, inspect
`/room_315/visual_state/validation` and `/diagnostics`; do not proceed to
execution.

## Terminal 3: PlanSys2 and the Closed-Loop Gateway

Enabling this launch opts into supervised Gazebo shuttle actuation. Keep it
disabled until the previous readiness checks pass.

```bash
export MFJA_WS="$HOME/mfja_ws"
export ROOM315_TASK_EXEC_CONFIG="$HOME/.config/mfja/task_execution_runtime.yaml"

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"

ros2 launch mfja_robot_control_config room_315_task_execution.launch.py \
  use_sim_time:=true \
  runtime_config:="$ROOM315_TASK_EXEC_CONFIG" \
  execution_enabled:=true \
  enable_plansys2:=true \
  external_obstacles_disabled:=true
```

The flags are intentionally inverse: Terminal 1 uses
`enable_room315_visual_obstacles:=false`, so the gateway must use
`external_obstacles_disabled:=true`. The gateway revalidates its immutable
execution authorization for every goal and rejects the goal if any path,
digest, schema, candidate, or runtime guard differs.

Readiness checks:

```bash
ros2 lifecycle get /planner
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
```

The planner must be `active`, and diagnostics must not report an authorization,
observation, supervisor, or sensor fault.

## Terminal 4: Interactive English Commands

Install the optional local intent model once. This step requires network
access and disk space for the GGUF checkpoint. Use a dedicated virtual
environment so the setup does not modify the user/system Python package site;
offline operation begins only after setup completes.

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
export ROOM315_INTENT_DIR="$HOME/models/room315_intent"

cd "$MFJA_REPO"
python3 -m venv --system-site-packages "$HOME/.venvs/room315-intent"
source "$HOME/.venvs/room315-intent/bin/activate"
python -m pip install --upgrade pip
python -m pip install 'llama-cpp-python==0.3.16'

python3 mfja_robot_control_config/scripts/setup_room315_intent_model.py \
  --model-dir "$ROOM315_INTENT_DIR" \
  --skip-dependency-install
```

Then start the CLI:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
export ROOM315_INTENT_DIR="$HOME/models/room315_intent"

cd "$MFJA_REPO"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
source "$HOME/.venvs/room315-intent/bin/activate"
source "$ROOM315_INTENT_DIR/room315_intent.env"

export PYTHONPATH="$MFJA_REPO/mfja_robot_control_config/scripts:${PYTHONPATH:-}"

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

## Optional Terminal 5: Status Monitor

```bash
export MFJA_WS="$HOME/mfja_ws"

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"

ros2 topic echo /room_315/task_goal/status std_msgs/msg/String
```

Terminal results are `succeeded`, `aborted`, `rejected`, or `failed`, with a
fail-closed reason. Successful inspection results additionally contain the
same-frame machine-readable inspection report described above.

For visual-runtime details, see
[`room315_visual_runtime_integration.md`](room315_visual_runtime_integration.md).
For task-goal parser setup and security boundaries, see
[`ROOM315_TASK_GOAL_UNDERSTANDING.md`](ROOM315_TASK_GOAL_UNDERSTANDING.md).
