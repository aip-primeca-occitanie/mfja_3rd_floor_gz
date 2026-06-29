# Room 315 Visual VLA Research

The current Room 315 research setup is focused on the 40 successful payload
training cases. The learned policy observes task language, overhead camera
images, and the previous command, then predicts the next event-level direct
symbolic action.

Expert-only rail state remains outside `model_input`. Binary sensors, exact
Gazebo pose, true shuttle segment, payload identity metadata, and validation
signals are kept in privileged metadata for audit and evaluation.

## Current Pipeline

```text
payload_training_cases.yaml
  -> case resolver
  -> symbolic primitive sequence
  -> schema-v3 event actions
  -> supervisor safety decoder
  -> Room 315 rail execution
  -> dataset recorder
  -> training_events.jsonl
```

The model-facing output is a schema-v3 `action_vector` or an equivalent
primitive JSON command. The supervisor validates every proposal before any rail
command is published.

## Case Matrix

The active case matrix covers right and left rails, loaded shuttle selection,
nearest loaded shuttle selection, no-blocker moves, blocker clearance to a
stopper, and blocker clearance into the interior loop.

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases.yaml
```

## Evaluation

Evaluation should be run on successful payload-case episodes exported by:

```bash
ros2 run mfja_robot_control_config room_315_vla_event_extractor.py \
  ~/room315_payload_all_cases \
  --output meta/training_events.jsonl
```

The primary boundary to preserve is simple: model input receives only language,
overhead images, last command, and observable state. All expert routing facts
remain outside the model input.
