# Room 315 Visual-State Runtime Integration

This guide documents the current `room315.visual_state.v4` runtime. It is for
operators who already have a qualified V4 runtime bundle and for maintainers
who need to understand the boundary between learned perception and
deterministic control.

The basic Gazebo, robot, and rail simulation does not require this runtime. A
fresh clone does not contain the large checkpoint or every authorization
artifact referenced by the checked-in runtime configuration. Do not expect the
advanced visual runtime to become ready until a complete, host-local promotion
bundle has been supplied.

## Safety Boundary

`room_315_visual_state_inference_node.py`:

1. verifies an immutable V4 promotion manifest and every required contract
   artifact referenced by it;
2. checks that the manifest authorizes the requested `active` or `shadow`
   mode;
3. loads the exact checkpoint declared by the manifest;
4. synchronizes the left and right RGB images;
5. obtains shuttle presence from the deterministic rail-state inventory;
6. performs V4 inference, acceptance checks, and deterministic validation;
7. publishes raw, validation, accepted, and diagnostic outputs; and
8. enforces the manifest's runtime guards. All currently supported V4 scopes
   require dry-run state fusion and disable direct PlanSys2 mutation.

The node does not plan, publish rail commands, bypass the supervisor, or
authorize task execution. Task execution is a separate, fail-closed gateway.
The checked-in launch defaults keep state fusion dry-run and PlanSys2 mutation
disabled.

The supplied active authorization is for Gazebo only. It is not approval for a
physical deployment.

## V4 Model Contract

The deployable runtime accepts only:

- schema `room315.visual_state.v4`;
- model kind `room315_visual_state_resnet18_split_rails_v4`;
- fixed identity order `L1,L2,L3,L4,R1,R2,R3,R4`;
- the checkpoint, preprocessing, topology, thresholds, and calibration
  declared by the promotion manifest; and
- exact SHA-256 matches for all required artifacts.

The current V4 output is segment-first. It predicts segment class, loaded
state, bounding box, and normalized position for present shuttle identities.
The runtime derives any compatibility representation only after V4 acceptance;
it does not re-enable the historical V3 runtime.

## Promotion Bundle Requirement

A usable promotion bundle is a directory containing the promotion manifest
and every required artifact that the runtime verifier resolves from it.
Depending on the authorization, this includes the checkpoint, model/config
evidence, topology contract, calibration records, and a manual decision record.
Copying only
`runtime_promotion_manifest.json` is insufficient.

Keep the bundle outside Git. Obtain it from the project's authorized artifact
custodian or reconstruct it through the locked V4 qualification workflow. The
release and evidence overview is in
[`report/evidence/ROOM315_VISUAL_V4_RELEASE.md`](../report/evidence/ROOM315_VISUAL_V4_RELEASE.md).

For any selected bundle, verify the manifest against its independently
published digest in the same terminal that will start inference:

```bash
export ROOM315_SELECTED_V4_BUNDLE='/absolute/path/to/selected-v4-bundle'
export ROOM315_SELECTED_V4_MANIFEST="$ROOM315_SELECTED_V4_BUNDLE/runtime_promotion_manifest.json"
export ROOM315_SELECTED_V4_MANIFEST_SHA256='<approved-64-character-sha256>'

test -f "$ROOM315_SELECTED_V4_MANIFEST"
printf '%s  %s\n' \
  "$ROOM315_SELECTED_V4_MANIFEST_SHA256" \
  "$ROOM315_SELECTED_V4_MANIFEST" | sha256sum --check -
```

Do not compute a digest from an untrusted manifest and then use that same
digest as authorization. Obtain the expected digest through the release or
review channel that approved the bundle.

The manifest's `deployment_mode` must equal the launch `runtime_mode`. A
shadow manifest cannot be converted to active operation by changing a launch
argument. An active manifest also binds the permitted runtime guards.

## Build and Environment

From the workspace root:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$HOME/.venvs/mfja-visual/bin/activate"
colcon build --symlink-install \
  --packages-select mfja_rail_interfaces mfja_robot_control_config \
  --paths \
    "$MFJA_REPO/mfja_rail_interfaces" \
    "$MFJA_REPO/mfja_robot_control_config"
source install/setup.bash
```

The ROS Python environment must contain Torch, TorchVision, NumPy, Pillow,
`cv_bridge`, and `message_filters`. Complete the base dependency step and the
isolated visual environment in
[`INSTALLATION.md`](INSTALLATION.md#visual-training-and-v4-inference) first.
Every terminal that starts visual inference must activate that environment.

## Start the Simulation Inputs

The high-level launch below starts Gazebo, the paired RGB-D camera bridge, the
deterministic rail nodes, and the supervisor. It intentionally does not start
the visual inference process. It also clears the disposable
`~/.ros/room315_visual_obstacles.json` cache by default. Add
`room315_clear_visual_obstacle_pose_cache:=false` only if the cache must be
preserved, and keep any configured cache path limited to that intended file.

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
  room315_left_shuttle_count:=2 \
  room315_right_shuttle_count:=2 \
  room315_shuttles_start_enabled:=false
```

Wait at least five seconds, then verify that `/clock`, both camera image topics,
both shuttle-state topics, and `/room_315/rail_safety/status` are active.

## Start a Shadow Runtime

Use a promotion bundle whose manifest declares `deployment_mode: shadow`:

```bash
export MFJA_WS="$HOME/mfja_ws"
export ROOM315_SHADOW_V4_BUNDLE="$HOME/room315_artifacts/visual_v4_shadow"
export ROOM315_SHADOW_V4_MANIFEST="$ROOM315_SHADOW_V4_BUNDLE/runtime_promotion_manifest.json"
export ROOM315_SHADOW_V4_MANIFEST_SHA256='<approved-shadow-manifest-sha256>'

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
source "$HOME/.venvs/mfja-visual/bin/activate"
printf '%s  %s\n' \
  "$ROOM315_SHADOW_V4_MANIFEST_SHA256" \
  "$ROOM315_SHADOW_V4_MANIFEST" | sha256sum --check -

ros2 launch mfja_robot_control_config room_315_visual_state_runtime.launch.py \
  use_sim_time:=true \
  enable_camera_bridge:=false \
  runtime_mode:=shadow \
  v4_promotion_manifest:="$ROOM315_SHADOW_V4_MANIFEST" \
  v4_promotion_manifest_sha256:="$ROOM315_SHADOW_V4_MANIFEST_SHA256" \
  device:=auto \
  dry_run_state_fusion:=true \
  plansys2_update_enabled:=false
```

Because the high-level simulation launch already created the camera bridge,
`enable_camera_bridge` is false here. Set it to true only when no other process
bridges the two Gazebo image topics.

Shadow mode publishes under `/room_315/visual_state/shadow_v4/*` and never
updates PlanSys2. Inspect it with:

```bash
ros2 topic list | sort | rg '/room_315/visual_state/shadow_v4'
ros2 topic echo /room_315/visual_state/shadow_v4/validation \
  mfja_rail_interfaces/msg/VisualStateObservation --once
ros2 topic echo /room_315/visual_state/shadow_v4/diagnostics \
  diagnostic_msgs/msg/DiagnosticArray
```

## Start an Active, Dry-Run Runtime

Use only a promotion bundle whose manifest declares `deployment_mode: active`
and whose manual decision authorizes the exact guard values below:

```bash
export MFJA_WS="$HOME/mfja_ws"
export ROOM315_ACTIVE_V4_BUNDLE="$HOME/room315_artifacts/visual_v4_active"
export ROOM315_ACTIVE_V4_MANIFEST="$ROOM315_ACTIVE_V4_BUNDLE/runtime_promotion_manifest.json"
export ROOM315_ACTIVE_V4_MANIFEST_SHA256='<approved-active-manifest-sha256>'

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
source "$HOME/.venvs/mfja-visual/bin/activate"
printf '%s  %s\n' \
  "$ROOM315_ACTIVE_V4_MANIFEST_SHA256" \
  "$ROOM315_ACTIVE_V4_MANIFEST" | sha256sum --check -

ros2 launch mfja_robot_control_config room_315_visual_state_runtime.launch.py \
  use_sim_time:=true \
  enable_camera_bridge:=false \
  runtime_mode:=active \
  v4_promotion_manifest:="$ROOM315_ACTIVE_V4_MANIFEST" \
  v4_promotion_manifest_sha256:="$ROOM315_ACTIVE_V4_MANIFEST_SHA256" \
  device:=auto \
  dry_run_state_fusion:=true \
  plansys2_update_enabled:=false
```

If the requested mode or guards differ from the immutable authorization, the
process stays alive but the model remains unready. It publishes an ERROR
diagnostic and no accepted observations. Do not wait for a launch-process exit,
weaken the checks, or edit the manifest in place.

Active outputs are:

| Topic | Type | Meaning |
|---|---|---|
| `/room_315/visual_state/raw` | `VisualStateObservation` | Pre-validation compatibility observation |
| `/room_315/visual_state/raw_model_prediction` | `std_msgs/String` | V4 diagnostic model payload |
| `/room_315/visual_state/validation` | `VisualStateObservation` | Acceptance or rejection result |
| `/room_315/visual_state/observed_state` | `VisualStateObservation` | Accepted fused state only |
| `/diagnostics` | `DiagnosticArray` | Artifact, input, presence, validation, and safety health |

Verify one accepted observation:

```bash
ros2 topic echo /room_315/visual_state/observed_state \
  mfja_rail_interfaces/msg/VisualStateObservation --once
```

An accepted message must report `accepted: true`, `presence_ready: true`, and
`state_fusion_ready: true`. Absence of an accepted message is a fail-closed
condition; inspect the validation topic and diagnostics instead of assuming
that the model is working.

## Deterministic Presence Contract

The Gazebo presence provider subscribes to:

- `/room_315/rails/left/shuttles/state`;
- `/room_315/rails/right/shuttles/state`; and
- `mfja_rail_interfaces/msg/ShuttleState` on both topics.

It deliberately uses only the shuttle name, message timestamp, and ROS receive
time. It does not use controller `mode`, segment, position, pose, yaw, or speed
to improve model predictions.

| Presence state | Meaning | Runtime behavior |
|---|---|---|
| `present` | A fresh, correctly mapped rail-state message exists | Decode and validate that identity |
| `absent` | Both inventories are fresh and the identity has no current report | Mask all visual fields for that identity |
| `unknown` | Startup, staleness, duplicate identity, wrong side, or mapping failure | Reject the complete observation |

The simulation provider does not prove that a physical cell has equivalent
presence sensing. Physical integration requires a trusted PLC/controller
provider implementing the same boundary.

## Validation and Failure Behavior

The model remains unready when an artifact is missing, a digest is wrong, or
the manifest mode/guards do not authorize the request. The node stays alive so
diagnostics remain available.

After a model is ready, it rejects a frame when any of these conditions occurs:

- image pairs are stale, future-dated, or too far apart;
- presence is incomplete, stale, duplicated, unmapped, or on the wrong side;
- a segment or loaded-state class is invalid;
- a bounding box or normalized position is non-finite or out of bounds;
- the V4 confidence and frame-level acceptance rules fail; or
- deterministic visual validation fails.

Rejection means no accepted observation is published. Shadow mode and the
checked-in dry-run guards also prevent PlanSys2 mutation.

Supervisor status and emergency-stop readiness do not decide whether a visual
frame is accepted. They gate downstream side effects and task execution. It is
therefore possible to observe a valid visual message while actuation remains
blocked; use supervisor and task-gateway diagnostics for the actuation state.

## Configuration and Environment Overrides

The source configuration is
`mfja_robot_control_config/config/room_315_visual_state/visual_state_runtime.yaml`.
Its checked-in artifact path is host-specific evidence, not a portable
default. Prefer launch overrides or a host-local copy rather than committing a
personal path.

The runtime also recognizes these artifact environment overrides:

- `ROOM315_VISUAL_V4_PROMOTION_MANIFEST_PATH` or
  `ROOM315_VISUAL_V4_MANIFEST_PATH`;
- `ROOM315_VISUAL_EXPECTED_V4_PROMOTION_MANIFEST_SHA256` or
  `ROOM315_VISUAL_V4_EXPECTED_PROMOTION_MANIFEST_SHA256`.

Defining conflicting aliases is an error. The launch arguments are clearer for
operator runbooks and are preferred in the examples above.

See [`CONFIGURATION.md`](CONFIGURATION.md) for ownership rules and
[`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) for symptom-based diagnosis.
