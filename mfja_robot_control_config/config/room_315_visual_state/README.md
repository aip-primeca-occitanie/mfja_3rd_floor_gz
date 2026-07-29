# Room 315 visual-state scenarios

This directory defines the new image-to-visual-state data path. It is
independent of language tasks, PDDL problems, previous commands, observations,
and action targets.

## Data boundary

The scenario manifest is simulator setup metadata. It is never passed to the
model.

After a scenario is captured, one training sample has two physically separated
parts:

```text
model input
  left overhead image
  right overhead image

oracle label
  fixed L1-L4/R1-R4 entry, presence/visibility masks
  side, global rail block, bounding box, and payload state
  continuous rail position: s_m, s_ratio, and segment length
```

The current label schema is `room315.visual_state.v3`. Every label contains
exactly eight entries in this order:

```text
L1, L2, L3, L4, R1, R2, R3, R4
```

Entry identity is structural and is not a classification target. Every entry
uses the same 14-block vocabulary loaded from the authoritative left/right
rail topology. Present visible entries must have one valid block, bbox,
payload state, `s_m`, `s_ratio`, and segment length. Absent or off-camera
entries remain in the fixed schema but are excluded by explicit masks. The
resulting model target has 200 outputs; its capacity is not fit from a dataset
subset.

`position_uncertainty_m = 0.0` is retained in the separated oracle file only as
Gazebo provenance. The visual target vectorizer explicitly excludes it, along
with confidence, switches, obstacles, planning fields, and safety-derived
fields.

The blocker-localization profile adds route-specific hard negatives. A shuttle
behind the selected shuttle or on the unused interior branch is labelled with
its exact position but is not considered a route blocker. The `relation_probe`
field exists only in the simulator setup manifest and is excluded from model
input and visual labels.

## Generate a pilot

```bash
ros2 run mfja_robot_control_config room_315_visual_scenario_generator.py \
  --config "$(ros2 pkg prefix mfja_robot_control_config)/share/mfja_robot_control_config/config/room_315_visual_state/training_scenarios.yaml" \
  --output-dir ~/room315_visual_state_v3/scenarios \
  --count 50
```

The result contains:

```text
~/room315_visual_state_v3/scenarios/
  scenario_manifest.jsonl   exact Gazebo setup for every scenario
  scenario_summary.json     balance and quality-gate report
```

Validate it at any time:

```bash
ros2 run mfja_robot_control_config room_315_visual_scenario_generator.py \
  --validate-manifest ~/room315_visual_state_v3/scenarios/scenario_manifest.jsonl
```

## Generate the blocker-localization pilot

```bash
ros2 run mfja_robot_control_config room_315_visual_scenario_generator.py \
  --config "$(ros2 pkg prefix mfja_robot_control_config)/share/mfja_robot_control_config/config/room_315_visual_state/blocker_training_scenarios.yaml" \
  --output-dir ~/room315_visual_state_v4_blockers/scenarios \
  --count 50
```

This profile starts shuttles at continuous `SEGMENT@S_RATIO` positions and
balances five cases: blocker ahead, blocker on an intermediate segment,
non-blocker behind, non-blocker on the adjacent branch, and multiple blockers.
Every active relation rail launches all four of its physical shuttles. Targets
rotate across suffixes 1-4, required relation actors are assigned separately,
and remaining shuttles are placed as collision-free relation-neutral actors.
The configured rail scopes are 40% left-only 4+0, 40% right-only 0+4, and 20%
simultaneous 4+4. All shuttles receive seeded randomized positions. The full
configuration covers all 14 public segments on both rails and
balances targets across boundaries, switches, slots, merge/conflict zones,
buffers, and ordinary segment regions.

Every generated continuous position is also projected through the calibrated
rail geometry into Gazebo world coordinates. The generator rejects and
deterministically resamples any pair whose oriented `0.36 m x 0.22 m` shuttle
footprints do not retain at least `0.04 m` clearance. This applies across
different segments as well as on the same segment. Alternative paths inside
the short A1-A4 switch pieces are not treated as simultaneously occupiable;
adjacent-branch negatives use the physically separable A12/A34 branches.
The dataset audit repeats this geometry check on both the scenario manifest
and the captured oracle-label JSONL.

Audit the generated scenario plan without changing it:

```bash
ros2 run mfja_robot_control_config room_315_visual_dataset_audit.py \
  --config "$(ros2 pkg prefix mfja_robot_control_config)/share/mfja_robot_control_config/config/room_315_visual_state/blocker_training_scenarios.yaml" \
  --scenario-manifest ~/room315_visual_state_v4_blockers/scenarios/scenario_manifest.jsonl
```

After capture and splitting, run the complete audit, including label JSONL and
train/validation/test scenario-family isolation:

```bash
ros2 run mfja_robot_control_config room_315_visual_dataset_audit.py \
  --config "$(ros2 pkg prefix mfja_robot_control_config)/share/mfja_robot_control_config/config/room_315_visual_state/blocker_training_scenarios.yaml" \
  --scenario-manifest ~/room315_visual_state_v4_blockers/scenarios/scenario_manifest.jsonl \
  --splits-dir ~/room315_visual_state_v4_blockers/splits \
  --require-complete \
  --report ~/room315_visual_state_v4_blockers/dataset_audit.json
```

## Capture the pilot

First inspect the exact commands for one scenario without starting Gazebo:

```bash
ros2 run mfja_robot_control_config room_315_visual_scenario_runner.py \
  --scenario-manifest ~/room315_visual_state_v3/scenarios/scenario_manifest.jsonl \
  --output-dataset ~/room315_visual_state_v3 \
  --limit 1 \
  --dry-run
```

Capture one scenario with the Gazebo GUI so it can be checked visually:

```bash
ros2 run mfja_robot_control_config room_315_visual_scenario_runner.py \
  --scenario-manifest ~/room315_visual_state_v3/scenarios/scenario_manifest.jsonl \
  --output-dataset ~/room315_visual_state_v3 \
  --limit 1 \
  --gui
```

After verifying that sample, capture all remaining pilot scenarios headlessly:

```bash
ros2 run mfja_robot_control_config room_315_visual_scenario_runner.py \
  --scenario-manifest ~/room315_visual_state_v3/scenarios/scenario_manifest.jsonl \
  --output-dataset ~/room315_visual_state_v3 \
  --resume \
  --keep-going
```

The runner launches every scene, applies its switch settings, waits for the
scene to settle, and calls `room_315_visual_state_capture.py`. The capture tool
rejects the sample if an image or simulator state is missing, the cameras are
not synchronized, scene state differs from the manifest, a projected shuttle
is outside the camera view, or the exact image pair already exists.

The final curated dataset uses this layout:

```text
room315_visual_state_v3/
  episodes/<episode_id>/images/<camera>/<frame>.jpg
  meta/training_events.jsonl
  meta/visual_label_export_summary.json
```

`meta/training_events.jsonl` is the join point between image paths and labels
before splitting. The splitter then writes model-input JSONL and label JSONL as
separate files.
