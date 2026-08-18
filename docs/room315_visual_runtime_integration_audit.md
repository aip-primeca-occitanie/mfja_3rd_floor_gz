# Room 315 visual runtime integration audit

> **Historical record:** this dated audit preserves an earlier integration
> decision. Do not use it as a current launch runbook. Follow
> [Visual-State Runtime Integration](room315_visual_runtime_integration.md) and
> verify the installed launch arguments with `--show-args`.

Date: 2026-07-30

## Scope and repository state

The requested runtime must load the frozen paired-camera visual-state model,
validate its outputs deterministically, and expose only accepted visual facts
to the existing deterministic state-fusion and PlanSys2 boundary. It must not
train, access the locked Test split, publish commands, or replace independent
safety logic.

The working tree is currently on `ali/neuro-symbolic-closed-loop` at
`9a27b20`, not on the requested `vla-experiment` branch. The requested branch
at `b111e3d` is an ancestor of the current branch. The current branch contains
the later ObservedState, fusion, PlanSys2, eight-shuttle visual schema, and
training changes required by this integration. The tree also contains
pre-existing uncommitted dataset-generation work, so switching back to the
older branch would be unsafe and would remove required interfaces. No branch
change was made.

## Frozen model artifact audit

The approved directory is:

`/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run`

The required files exist and their SHA-256 values match:

| Artifact | Verified SHA-256 |
|---|---|
| `best.pt` | `8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d` |
| `target_stats.json` | `2d48078641842aa2db7a59b9285fc5bbedaaa3a0039fc39986ca230db983b18c` |
| `visual_label_vectorizer.json` | `637c854556f3331c4e187db4aa7fc70457f01df8877947b9a0e988a543f7113e` |
| `training_config.json` | `5c45544af7766afff397dafa7c14c0b3b05083f07a93122308ef50c2e8f452eb` |
| `run_metadata.json` | `d86c0ebfda3f5b174fc3c06f4ce8a3e083d2048db7b44d20efe951aaa7e5428d` |

The sidecars confirm:

- schema `room315.visual_state.v3`;
- model kind
  `structured_visual_state_torchvision_resnet18_fixed8_v3`;
- TorchVision ResNet-18 with partial `layer4` fine-tuning;
- paired `[B,6,224,224]` RGB input;
- fixed identity order `L1,L2,L3,L4,R1,R2,R3,R4`;
- 200 output dimensions;
- best epoch 14;
- target mean/std vectors of length 200;
- global 14-block vocabulary
  `A12E,A12I,A14,A1E,A1I,A23,A2E,A2I,A34E,A34I,A3E,A3I,A4E,A4I`.

The checkpoint and sidecars were read only for integrity and contract
metadata. No training or Test-split evaluation was run.

## Existing reusable implementation

### Contracts and state fusion

- `mfja_robot_control_config/scripts/room_315_contracts.py`
  defines the ROS-independent immutable `ObservedFact` and `ObservedState`
  contracts. Learned facts must use source `visual_model`; direct command-like
  or privileged fields are rejected. `ObservedState` keeps visual inputs
  separate from `fused_planner_state`.
- `mfja_robot_control_config/scripts/room_315_observed_state_provider.py`
  defines `ObservedStateProvider`, `FusedObservedStateProvider`, and
  `fuse_observed_facts`. Fusion emits source `state_fuser`, records
  `selected_source`, and gives `trusted_device` priority over `visual_model`.
- `mfja_robot_control_config/scripts/room_315_visual_observed_state_provider.py`
  contains a previous compact RGB-D detection adapter and calibration
  provider. Its existing `ObservedFact` construction and fusion boundary are
  reusable, but its compact detector schema, identity-confidence fields,
  depth projection, switch prediction, and obstacle prediction do not match
  the approved fixed-slot paired-RGB model. It must not be used as the new
  checkpoint decoder.

### Model and vectorizer

- `mfja_robot_control_config/scripts/room_315_vla_train_local.py:157-226`
  contains the authoritative direct bilinear RGB resize, `[0,1]` conversion,
  left/right channel concatenation, and ImageNet normalization behavior.
- `mfja_robot_control_config/scripts/room_315_vla_train_local.py:369-421`
  constructs the paired-view ResNet-18 architecture.
- `mfja_robot_control_config/scripts/room_315_visual_state_dataset.py:1195-1415`
  defines `VisualStateLabelVectorizer`, the authoritative fixed identity
  order, topology vocabulary, vector names, and training target masks.

The shared model constructor is now in
`mfja_robot_control_config/scripts/room_315_visual_model.py`. The training
wrapper and the pure runtime call the same constructor, so strict checkpoint
loading cannot drift from the training architecture.

### PlanSys2 and safety boundary

- `mfja_robot_control_config/scripts/room_315_pddl_scenario_generator.py`
  maps known `fused_planner_state` facts to the existing PDDL predicates.
  Relevant existing facts are `present`, `location_block`, `location_slot`,
  `loaded`, and occupancy. Switch, stopper, obstacle, and safety state are
  required separately from trusted deterministic sources.
- `mfja_robot_control_config/scripts/room_315_closed_loop_executive.py`
  consumes an `ObservedStateProvider`, builds a PDDL problem, asks PlanSys2,
  translates one symbolic step, and submits it to the supervisor. This is the
  correct downstream boundary; the inference node must never call its planner,
  translator, transport, or executor directly.
- `mfja_robot_control_config/scripts/room_315_vla_supervisor.py` owns the
  emergency stop, deterministic safety decoder, workspace/fleet checks,
  actuator feedback, and rail command publishers. The new visual runtime must
  expose readiness only and must not publish to `/room_315/vla/command` or any
  rail device command topic.

### ROS topics and launch conventions

The overhead camera SDF publishes 10 Hz RGB-D sensor bases:

- `/room_315/vla/left_rail_rgbd`
- `/room_315/vla/right_rail_rgbd`

`room_315_vla_supervisor.launch.py` already bridges and uses these RGB topics:

- `/room_315/vla/left_rail_rgbd/image`
- `/room_315/vla/right_rail_rgbd/image`

The project uses `ament_cmake`, executable Python scripts installed into
`lib/${PROJECT_NAME}`, package-share YAML configuration, `launch_ros.actions.Node`,
and `ament_cmake_pytest`.

Available typed image input is `sensor_msgs/msg/Image`. Existing rail messages
cover devices and shuttle commands/state, but there is no typed message for a
decoded fixed-slot visual observation, validation result, or runtime health.
Standard `diagnostic_msgs/msg/DiagnosticArray` is available for health but
does not replace a typed visual-state observation.

## Selected integration point

The implemented boundary is:

1. a pure paired-image model runtime;
2. a pure fixed-slot decoder;
3. a pure deterministic validator;
4. an optional disabled-by-default deterministic stabilizer;
5. conversion of accepted shuttle facts to `ObservedFact(source="visual_model")`;
6. `FusedObservedStateProvider` with trusted device facts retaining priority;
7. publication of a traceable typed observation and diagnostics;
8. optional deterministic handoff to the existing ObservedState/PlanSys2
   provider boundary, without planner or command calls from inference.

No simulator position, segment, motion, payload, or other oracle field belongs
in steps 1-6.

## Resolved immutable-contract mismatch: no presence output

The approved model cannot predict shuttle presence.

Direct evidence:

- `visual_label_vectorizer.json` has `dim=200` and no output name containing
  `presence` or `present`.
- Its `prediction_target_fields` are only `bbox`, `loaded_state`,
  `location.block`, `location.side`, `rail_position.s_m`,
  `rail_position.s_ratio`, and `rail_position.segment_length_m`.
- The 200 dimensions are 56 numeric values plus 144 categorical values.
- `VisualStateLabelVectorizer.target_mask` at
  `room_315_visual_state_dataset.py:1294-1331` uses ground-truth
  `presence` only to remove absent slots from the training loss.
- Presence is not passed to the model input and is not encoded as a target.

Consequently, every fixed output slot emits bbox/location/payload values at
runtime even when that identity is absent. There is no calibrated presence
score, objectness head, or runtime target mask. The following proposed
shortcuts would violate the request:

- treating all eight output slots as present would invent absent shuttles;
- treating bbox validity as presence would add an untrained, uncalibrated
  heuristic and can accept hallucinated slots;
- using simulator identity/presence truth would leak debug oracle data into
  production;
- adding a presence output would change the architecture and require training,
  which this phase forbids.

This originally blocked the requirements that absent identities produce no
accepted location/payload facts and that the temporal layer never invents a
shuttle.

## Selected presence contract

The user selected the external deterministic presence gate. The Gazebo/ROS 2
implementation subscribes to:

- `/room_315/rails/left/shuttles/state`;
- `/room_315/rails/right/shuttles/state`.

Only `ShuttleState.name`, `ShuttleState.header.stamp`, and ROS receive time
enter the presence provider. Identity mapping reuses `all_shuttle_specs()` and
`normalize_shuttle_ref()` from the existing fleet registry. The provider never
accepts the deterministic position or motion fields. It keeps all eight slots
as `present`, `absent`, or `unknown`; stale or uninitialized sources make the
complete registry unknown.

This source is acceptable as deterministic Gazebo controller inventory. It is
not evidence of physical presence sensing. Real hardware must supply an
equivalent PLC/controller inventory provider through the same
`PresenceProvider` abstraction.

## Implemented changes

- pure modules for artifact/model runtime, deterministic validation, temporal
  stabilization, and accepted-fact/state fusion;
- `PresenceProvider` and a Gazebo `ShuttleStatePresenceProvider`;
- typed `VisualShuttleState` and `VisualStateObservation` interfaces plus
  standard diagnostics;
- one `rclpy` approximate-time paired-image inference node;
- one YAML configuration and one launch file using the existing camera topics;
- package dependency/install updates for Torch/TorchVision,
  `cv_bridge`, `message_filters`, and diagnostics;
- unit tests using synthetic and validation-only fixtures;
- a validation-only CPU checkpoint smoke test;
- a ROS integration test that checks raw/validation/accepted-or-rejected
  outputs and absence of command publication;
- `docs/room315_visual_runtime_integration.md`.

The PDDL problem builder was corrected to stop requiring or emitting payload
and location predicates for `present=false` shuttles. The PDDL domain, safety
implementation, planner, executor, command publishers, datasets, checkpoint,
and sidecars were not modified.
