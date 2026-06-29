# Room 315 VLA Operations

Room 315 VLA operation is currently scoped to the curated 40 payload training
cases in:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases.yaml
```

The supervisor accepts direct primitive JSON commands and schema-v3 action
vectors only. Generated payload cases execute as `switches`, `stoppers`,
`shuttle`, `DONE`, `stop_all`, and `emergency_stop` commands through
`/room_315/vla/command`.

## Launch

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true \
  enable_room315_vla:=true
```

Watch supervisor status:

```bash
ros2 topic echo /room_315/vla/status std_msgs/msg/String
```

## Generate One Case

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker \
  --language-template-id loaded_shuttle_to_slot \
  --dry-run
```

Execute a case with the supervisor and dataset recorder running:

```bash
ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \
  --case-id right_loaded_r1_s1_to_slot3_no_blocker \
  --language-template-id loaded_shuttle_to_slot \
  --arrival-timeout-s 120 \
  --require-dataset-recorder \
  --execute
```

## Run The 40 Cases

```bash
ros2 run mfja_robot_control_config room_315_payload_case_batch_runner.py \
  --case-config mfja_robot_control_config/config/room_315_vla/payload_training_cases.yaml \
  --dataset-dir ~/room315_payload_all_cases \
  --results-dir /tmp/room315_payload_case_batch
```

Use `--dry-run` to inspect launch arguments without starting Gazebo.

## Manual Primitive Commands

```bash
ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"switches\",\"side\":\"right\",\"switches\":{\"A3\":\"INTERIOR\"}}'}"

ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"stoppers\",\"side\":\"right\",\"stoppers\":{\"A3\":\"0\"}}'}"

ros2 topic pub --once /room_315/vla/command std_msgs/msg/String \
  "{data: '{\"action\":\"shuttle\",\"side\":\"right\",\"shuttle\":\"room315_right_shuttle_1\",\"command\":\"ON\",\"speed\":0.08}'}"
```

The safety decoder validates side, target, switch and stopper masks, emergency
state, falling state, headway, block reservation, and required wait targets
before publishing rail commands.
