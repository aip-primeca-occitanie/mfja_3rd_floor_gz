# Room 315 hard-case visual dataset V3: initial repository audit

Date: 2026-07-30
Repository: `/home/tiago/mfja_3rd_floor_ros2_ws/src/mfja_3rd_floor_gz`
Branch at audit time: `ali/neuro-symbolic-closed-loop`
Requested generation seed: `31520260730`

## Scope and immutable-data boundary

This audit was completed before implementing the V3 hard-case generator. The
new work must create only newly rendered Gazebo observations in new output
roots. Existing capture packages, splits, JSONL files, images, checkpoints, and
sidecars are immutable.

The consumed legacy Test split is prohibited. In particular, the following
files must never be opened, parsed, enumerated, copied, imported, hashed, or
used for generation, tuning, coverage planning, or evaluation:

- `/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/test.jsonl`
- `/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/test_visual_labels.jsonl`

New tooling must fail closed for either resolved basename and for an explicitly
supplied file whose hash equals either known forbidden hash. It must not scan
existing image trees to try to recover Test membership. The old training split
and labels may be used only as a replay reference and for semantic-family
overlap auditing. The old validation split may be read only for historical
coverage auditing.

## Authoritative inventory and topology

`mfja_robot_control_config/scripts/room_315_visual_fleet.py` is the
fail-closed inventory boundary. It reconciles:

- `MAX_SHUTTLES_PER_SIDE`;
- `config/room_315_vla/shuttle_identity.yaml`;
- `mfja_3rd_floor_description/worlds/room_315_only.world`;
- `LEFT_ENTITY_DEFAULTS` and `RIGHT_ENTITY_DEFAULTS`.

Its fixed visual order is declared at lines 28-31:

`L1, L2, L3, L4, R1, R2, R3, R4`.

The same module derives the global block vocabulary from the authoritative
left and right rail-network YAML files. The public vocabulary is:

`A12E, A12I, A14, A1E, A1I, A23, A2E, A2I, A34E, A34I, A3E, A3I, A4E, A4I`.

Relevant topology sources and helpers are:

- `mfja_robot_control_config/config/room_315_rails/left_rail_network.yaml`;
- `mfja_robot_control_config/config/room_315_rails/right_rail_network.yaml`;
- `mfja_robot_control_config/scripts/room_315_rail_defaults.py`;
- `public_rail_segment_lengths()`;
- `default_rail_network_path()`;
- `room_315_shuttle_geometry.py`.

All eight identities are physically applicable to every public segment on
their own rail. Side is fixed by identity: `L* -> left`, `R* -> right`.
Physically valid positions must additionally satisfy segment length, endpoint,
same-segment separation, and topology constraints.

## Existing scenario generator

`mfja_robot_control_config/scripts/room_315_visual_scenario_generator.py`
defines capture schema `room315.visual_capture_scenario.v1` and requires both
`left_rail_rgb` and `right_rail_rgb`. Reusable implementation includes:

- seeded selection and ratio sampling;
- authoritative segment lengths;
- `_zone_segments()` and `_sample_ratio()`;
- `_blocker_positions()` for relation geometry;
- `_place_neutral_shuttles()` for unrelated active identities;
- `_launch_arguments()` and `_switch_command()`;
- `scenario_physical_conflicts()`;
- `validate_scenario()` and `validate_scenarios()`.

The established spatial zones are:

- `boundary`;
- `switch`;
- `slot`;
- `merge_conflict`;
- `buffer`;
- `ordinary`.

The established relation families are:

- `blocker_ahead_same_segment`;
- `nonblocker_behind_same_segment`;
- `blocker_intermediate_segment`;
- `nonblocker_adjacent_branch`;
- `multi_blocker`.

`mfja_robot_control_config/scripts/room_315_arbitrary_subset_visual.py` adds the
neutral observation name `no_relation_observation`, enumerates every non-empty
identity subset, and provides stable configuration IDs and relation
eligibility helpers.

Authoritative operational ratios currently used by the generator are:

| Side | Segment | Ratios |
|---|---|---|
| right | A12E | 0.411866742, 0.653073633 |
| right | A34E | 0.447469343, 0.683726992 |
| left | A12E | 0.428330934, 0.674370792 |
| left | A34E | 0.427586575, 0.668424664 |

These ratios are defined in `SLOT_RATIOS` at lines 120-129. V3 must add the
requested dense offset buckets without changing the authoritative centres.

`mfja_robot_control_config/scripts/room_315_arbitrary_subset_visual_v2.py`
contains reusable feasibility logic:

- topology/zone compatibility;
- position materialisation;
- projectability checks;
- position-zone validation;
- relation feasibility;
- physical-separation validation.

Its command-line package assumptions are tied to the older 2,040-scenario
plan, so V3 should reuse validators and topology functions, not its fixed plan
or output constants.

## Capture and label generation

`mfja_robot_control_config/scripts/room_315_visual_state_capture.py` is the
authoritative two-camera/oracle capture implementation. It subscribes to:

- `/room_315/vla/left_rail_rgbd/image`;
- `/room_315/vla/right_rail_rgbd/image`;
- `/room_315/rails/left/shuttles/state`;
- `/room_315/rails/right/shuttles/state`;
- the corresponding payload-state and switch-state topics.

`capture_scenario()` waits for complete fresh state, verifies identity,
payload, camera, and topology consistency, renders both images, derives
per-camera bounding boxes through `room_315_visual_label_exporter.py`, writes a
temporary episode, validates it, and atomically promotes the completed
episode. It does not use blank placeholder images.

`mfja_robot_control_config/scripts/room_315_visual_scenario_runner.py` launches
Room 315 Gazebo for each manifest scenario and invokes the capture script. Its
resume path accepts an episode only after revalidating the existing completed
episode. This mechanism is correct but potentially expensive for 4,768 new
scenarios because it launches Gazebo per scenario; the V3 orchestrator must
preserve fail-closed validation and resumability even if it wraps this runner.

## Existing dataset schema and vectorizer

`mfja_robot_control_config/scripts/room_315_visual_state_dataset.py` defines:

- `room315.visual_state.v3`;
- model inputs limited to two keys under `overhead_images`;
- fixed identities supplied by `room_315_visual_fleet.py`;
- one authoritative global union block vocabulary for every slot;
- perception targets only: side, block, bounding box, `s_m`, `s_ratio`,
  `segment_length_m`, and loaded state;
- oracle-only `position_uncertainty_m`, explicitly excluded from prediction
  targets;
- `s_ratio ~= s_m / segment_length_m` validation.

The fixed V3 vector has 200 values: 8 shuttle slots multiplied by presence,
side, global block, payload, bounding-box, and rail-position fields. V3
generation must not alter that schema or add planning, routing, safety, task,
command, or trajectory targets.

The shared `write_jsonl()` helper writes directly. The new package tooling
therefore needs its own atomic JSON/JSONL promotion rather than modifying or
manually editing generated rows.

## Existing split and audit mechanisms

`mfja_robot_control_config/scripts/room_315_grouped_visual_splits.py` provides
useful patterns for:

- canonical configuration hashing;
- atomic text, JSON, and JSONL writes;
- image decode/non-empty verification;
- label/model-input separation;
- row and file hashing;
- grouped overlap audits;
- immutable package manifests.

It is intentionally hard-coded to the older 2,040-scenario package and creates
train, validation, and Test outputs. It must not be invoked for V3. The V3
splitter will create train and validation only and must reject any Test path or
Test-evaluation option.

Existing audit logic in
`mfja_robot_control_config/scripts/room_315_visual_dataset_audit.py` verifies
visual schema validity, segment coverage, payload counts, ratios, excluded
prediction fields, and scenario families. V3 needs a separate conditional
audit layer because the existing report does not enumerate the requested
identity/payload/block/position cells or semantic family leakage against old
replay training.

## Current coverage weakness

The older 1,528/256 development split was balanced mainly in marginal counts.
It did not guarantee conditional coverage across identity, payload, block,
position, active subset, co-present identities, loaded-identity set, relation,
occlusion, or render bucket.

The live hard case `{L2,L4,R4}` had eight old-training occurrences and no old
validation occurrences. The exact payload/block/position scene was absent.
Consequently, high aggregate loaded-state accuracy did not establish that L4
loaded on A34E or R4 loaded on A12E was represented under that joint context.
Continuous position coverage was also too sparse around operational stopping
ratios such as right slot 3.

## Planned V3 changes

The implementation will add isolated tools for:

1. a deterministic feasible quota plan over all authoritative conditional
   cells;
2. hard-case and counterfactual scenario materialisation using existing
   topology and relation validators;
3. guarded, atomic, resumable capture orchestration;
4. train/validation-only grouped assignment with semantic-family isolation
   from new training and old replay training;
5. a separate 256-scenario development canary;
6. conditional JSON/Markdown/CSV audits and immutable SHA-256 manifests;
7. denylist enforcement for the consumed legacy Test;
8. an Experiment-A source-balanced dataset configuration.

The new family signature will include active identities, loaded identities,
identity-to-block and identity-to-position-bin assignments, relation family,
target-zone family, occlusion class, and discrete render bucket. Timestamps
and random image noise will not split equivalent semantic families.

## New output structure

No existing output is overwritten. Default roots are:

- capture: `/home/tiago/room315_hard_case_visual_v3_capture_seed31520260730`;
- grouped splits: `/home/tiago/room315_hard_case_visual_v3_splits_seed31520260730`;
- canary: `/home/tiago/room315_hard_case_visual_v3_canary_seed31520260730`;
- guard/audit: `/home/tiago/room315_hard_case_visual_v3_guard_seed31520260730`.

The capture root will contain immutable generation configuration, quota plan,
scenario manifests, per-scenario atomic episode directories, consolidated
model-input JSONL and visual-label JSONL, capture state/failures, environment
metadata, and SHA-256 manifests. The split root will contain only `train` and
`validation` JSONL/labels plus grouping and overlap audits. The canary root is
development-regression data only and is excluded from both splits. The guard
root contains the smoke package and conditional reports.

Safe resume is allowed only when schema, seed, configuration hash, and manifest
hash match, and every claimed completed episode passes image and label
validation. No V3 command creates a final Test split.
