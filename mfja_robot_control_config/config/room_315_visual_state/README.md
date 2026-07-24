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
  shuttle identities, bounding boxes, rail blocks, and payload states
  switch states
  obstacle bounding boxes when obstacle labelling is enabled
```

The current pilot deliberately sets `obstacles: []`. Obstacle scenes must not be
added until the exporter can provide a real bounding box or segmentation mask.

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
