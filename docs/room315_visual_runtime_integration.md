# Room 315 visual-state runtime integration

Date: 2026-07-30

## Runtime boundary

`room_315_visual_state_inference_node.py` is the paired-camera inference and
validation boundary for the approved Room 315 fixed-eight model. It:

1. verifies the checkpoint and all four sidecars by exact SHA-256;
2. strictly reconstructs the approved paired-view ResNet-18 architecture;
3. synchronizes the left and right RGB images;
4. gates fixed identity slots through a deterministic `PresenceProvider`;
5. decodes model values only for present slots;
6. validates topology, image timestamps, bounding boxes, categorical values,
   and continuous rail-position consistency;
7. optionally applies a deterministic temporal filter, disabled by default;
8. converts accepted visual fields to the existing `ObservedFact` and
   `ObservedState` contracts;
9. publishes typed raw, validation, and accepted observation messages plus
   standard ROS diagnostics;
10. optionally updates existing PlanSys2 problem-expert predicates.

The node does not plan, execute, publish rail commands, bypass the supervisor,
or replace deterministic safety. Its PlanSys2 integration only changes
problem-expert facts and is disabled by default.

## Approved model contract

- schema: `room315.visual_state.v3`
- architecture:
  `structured_visual_state_torchvision_resnet18_fixed8_v3`
- backbone: TorchVision ResNet-18, partial fine-tuning of `layer4`
- input: synchronized left/right RGB, concatenated as `[1,6,224,224]`
- resize: direct PIL bilinear
- normalization per RGB view:
  mean `(0.485, 0.456, 0.406)`, standard deviation
  `(0.229, 0.224, 0.225)`
- output dimension: 200
- fixed identity order: `L1,L2,L3,L4,R1,R2,R3,R4`
- decoded visual fields: side, block, bbox, `s_m`, `s_ratio`,
  `segment_length_m`, and loaded/empty

The training target mask is label-side metadata. It is never interpreted as a
runtime prediction.

## Deterministic presence contract

The Gazebo provider consumes:

- `/room_315/rails/left/shuttles/state`
- `/room_315/rails/right/shuttles/state`
- message type `mfja_rail_interfaces/msg/ShuttleState`

Only these values are read:

- `msg.name`
- `msg.header.stamp`
- ROS receive time

The provider deliberately never reads `mode`, `current_segment`, `s`, `x`,
`y`, `z`, `yaw`, or `speed`. These deterministic controller fields cannot
replace or improve the model's visual block, bbox, position, segment length,
or loaded/empty predictions.

Identity resolution reuses the repository's authoritative shuttle registry:
`all_shuttle_specs()` and `normalize_shuttle_ref()`. No second identity map is
maintained. The fixed order remains:

`L1,L2,L3,L4,R1,R2,R3,R4`.

`presence_state_timeout_s` defaults to 1.0 seconds.
`presence_warmup_s` defaults to 0.5 seconds. Both side sources must initialize,
complete warm-up, and remain fresh according to both source and receive
timestamps.

The three presence states have distinct semantics:

| State | Meaning | Runtime action |
|---|---|---|
| `present` | a fresh mapped state message exists | decode, validate, and fuse its visual slot |
| `absent` | the complete registry is fresh but that identity has no fresh report | ignore all bbox/location/payload model values for that slot |
| `unknown` | source startup, staleness, identity, duplicate, or side validation failed | reject the complete observation |

The runtime fails closed if either topic is not initialized, either source is
stale, an entity cannot be mapped, duplicate aliases identify the same active
slot, an identity appears on the wrong side, or any slot is unknown. In these
cases no accepted observation is published and PlanSys2 is not updated.

The current Gazebo `ShuttleState` topics are acceptable deterministic
controller-state inventory sources for simulation. This does not prove that
the physical installation has equivalent presence sensing. A real deployment
must provide a deterministic PLC/controller shuttle inventory through another
`PresenceProvider` implementation. The model runtime, validator, fusion, and
PlanSys2 boundary do not need to change when that provider is replaced.

## Validation and rejection behavior

The validator rejects:

- missing or unhealthy artifacts/model;
- stale, future-dated, or excessively skewed image pairs;
- unknown, duplicate, reordered, or side-conflicting identities;
- blocks outside the authoritative 14-block vocabulary;
- invalid loaded-state classes;
- non-finite or non-positive boxes and boxes wholly outside their rail camera;
- non-finite, negative, or out-of-range rail-position values;
- `s_m` inconsistent with `s_ratio * segment_length_m`;
- any presence registry that is not fully ready.

Small configurable boundary excursions can be clamped and are listed in
`clamped_fields`. Validation reasons and counters are published in both the
typed validation message and `/diagnostics`.

The optional temporal filter uses categorical majority and numeric EMA only
after validation. It resets after any rejected observation and removes state
for inactive identities, so it cannot create a shuttle.

## Field ownership and confidence

Presence facts are deterministic facts:

- source: `trusted_device`
- confidence: `1.0`
- owner: deterministic presence provider

Visual facts are:

- source: `visual_model`
- fields: side/block, bbox, rail position, segment length, loaded/empty
- confidence: `0.0` sentinel
- metadata: `confidence_available=false`

The approved model has no calibrated confidence head. The runtime does not
invent confidence from logits or regression magnitudes.

Absent identities receive only `present=false`; they contribute no visual
location, payload, bbox, or PlanSys2 predicates. Unknown presence blocks fusion
and PlanSys2 updates. The PDDL problem builder likewise skips explicitly absent
shuttles before requiring payload or location facts.

## Topics

Inputs:

- `/room_315/vla/left_rail_rgbd/image`
- `/room_315/vla/right_rail_rgbd/image`
- `/room_315/rails/left/shuttles/state`
- `/room_315/rails/right/shuttles/state`
- `/room_315/vla/status` for independent supervisor readiness only

Outputs:

- `/room_315/visual_state/raw`
- `/room_315/visual_state/validation`
- `/room_315/visual_state/observed_state` for accepted fused observations only
- `/diagnostics`

The three visual topics use
`mfja_rail_interfaces/msg/VisualStateObservation`. Each message includes the
schema/checkpoint identity, both image stamps, readiness and acceptance flags,
reasons, clamped fields, latency and frame counters, and all eight shuttle
slots with explicit presence state.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select mfja_rail_interfaces mfja_robot_control_config \
  --allow-overriding mfja_robot_control_config
source install/setup.bash
```

Runtime Python requires Torch and TorchVision in the Python environment used
by ROS 2, plus NumPy, Pillow, `cv_bridge`, `message_filters`, and the declared
ROS message packages.

## Launch

The checked-in YAML contains the local approved artifact path. On another
host, override the checkpoint and sidecar directory:

```bash
ros2 launch mfja_robot_control_config room_315_visual_state_runtime.launch.py \
  checkpoint_path:=/absolute/path/to/best.pt \
  sidecar_directory:=/absolute/path/to/run \
  device:=auto \
  dry_run_state_fusion:=true \
  plansys2_update_enabled:=false
```

Keep dry-run mode enabled while validating the live graph. PlanSys2 mutation
requires all of:

- `dry_run_state_fusion:=false`
- `plansys2_update_enabled:=true`
- a healthy model and synchronized inputs
- a complete fresh presence registry
- an accepted deterministic validation result
- a fresh supervisor status with emergency stop clear
- available PlanSys2 problem-expert add/remove predicate services

This still does not authorize planning or actuation from the inference node.

## Validation-only CPU smoke

The smoke command requires explicit validation files and rejects filenames
that identify the locked Test split:

```bash
python3 mfja_robot_control_config/scripts/room_315_visual_runtime_cpu_smoke.py \
  --checkpoint /absolute/path/to/best.pt \
  --sidecar-directory /absolute/path/to/run \
  --checkpoint-sha256 8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d \
  --target-stats-sha256 2d48078641842aa2db7a59b9285fc5bbedaaa3a0039fc39986ca230db983b18c \
  --vectorizer-sha256 637c854556f3331c4e187db4aa7fc70457f01df8877947b9a0e988a543f7113e \
  --training-config-sha256 5c45544af7766afff397dafa7c14c0b3b05083f07a93122308ef50c2e8f452eb \
  --run-metadata-sha256 d86c0ebfda3f5b174fc3c06f4ce8a3e083d2048db7b44d20efe951aaa7e5428d \
  --validation-split /absolute/path/to/validation.jsonl \
  --validation-labels /absolute/path/to/validation_visual_labels.jsonl \
  --dataset-root /absolute/path/to/dataset
```

This command performs inference only. It does not train, select a checkpoint,
or evaluate the locked Test split.

## Diagnostics

Inspect readiness and rejection causes with:

```bash
ros2 topic echo /diagnostics \
  diagnostic_msgs/msg/DiagnosticArray
ros2 topic echo /room_315/visual_state/validation \
  mfja_rail_interfaces/msg/VisualStateObservation
```

Key diagnostics include model, input, presence, fusion, and safety readiness;
artifact error; device and checkpoint SHA-256; last inference and acceptance
times; latencies; accepted/rejected/stale counters; presence reasons; and
PlanSys2 update status.
