# MFJA 3rd Floor Gazebo - Room 315 Kinematic Shuttle

This repository contains Gazebo Harmonic / ROS 2 Jazzy simulation assets for the
MFJA 3rd floor. The current focus is Room 315: a flexible rail-cell simulation
with kinematic shuttles, switches, stoppers, binary sensors, overhead cameras,
and a rail-only Vision-Language-Action research layer.

The shuttle simulation is kinematic-first. Shuttles do not currently use wheel
physics or contact dynamics for rail motion. Instead, each shuttle follows a
calibrated arc-length rail graph generated from Room 315 CSV geometry, then the
simulation publishes Gazebo model poses through `/world/<world_name>/set_pose`.

For more background on the rail backend, go to
[docs/ROOM315_RAIL_REFERENCE.md](docs/ROOM315_RAIL_REFERENCE.md).

## Repository Layout

This repository is a meta-repository. The repository root is not a ROS 2
package, so package-specific launch, model, config, and source files should stay
inside their owning package.

- `mfja_3rd_floor_description/`: Gazebo models, meshes, worlds, URDF, and SDF assets.
- `mfja_rail_interfaces/`: typed ROS 2 messages for rail commands, states, switches, stoppers, shuttles, and sensors.
- `mfja_robot_control_config/`: rail controllers, kinematic shuttle nodes, VLA supervisor, dataset recorder, benchmark runner, evaluator tools, launch files, and Room 315 config.
- `mfja_3rd_floor_bringup/`: launch entry points for Room 315, the full floor, and isolated industrial robot runs.
- `mfja_robot_control_config/config/room_315_kinematics/raw_segments/`: source rail segment CSV files.
- `docs/`: split documentation for setup, VLA, rail operation, robot spawning, and troubleshooting.

For the full documentation map, see [Documentation Map](#documentation-map).

## Install and Build

Use Ubuntu 24.04 with ROS 2 Jazzy. Clone this repository inside a colcon
workspace `src/` directory, install dependencies with `rosdep`, build from the
workspace root, and source the overlay.

```bash
export MFJA_WS=~/test_mfja_ws
mkdir -p "$MFJA_WS/src"
cd "$MFJA_WS/src"
git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/mfja_3rd_floor_gz -y --ignore-src --rosdistro jazzy
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz
source install/setup.bash
```

Every new terminal must source ROS and the workspace again:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

For Nix shell usage, full dependency setup, and build variants, go to
[docs/INSTALLATION.md](docs/INSTALLATION.md).

## Run Room 315

The common Room 315 launch starts Gazebo, the Room 315 rail stack, and the
kinematic shuttle simulation:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

For headless runs, set `gui:=false`. To launch the full floor with the same Room
315 rail features, use `mfja_3rd_floor_bringup full_floor.launch.py`.

For more launch variants, hidden/stopped/moving shuttle startup modes, switch
commands, stopper commands, and sensor checks, go to
[docs/QUICK_START_AND_FEATURE_GUIDE.md](docs/QUICK_START_AND_FEATURE_GUIDE.md).

## Room 315 Rail Basics

Each Room 315 rail side has one or more shuttles, switches `A1` to `A4`,
stoppers `A1` to `A4`, slot sensors such as `DZI1R`/`DZI2R`, and approach or
interior/exterior branch sensors such as `DA3IR` and `DA3IL`.

The main rail control concepts are:

- Shuttles are enabled, disabled, reset, added, or removed through typed ROS
  topics.
- Switches choose exterior/interior branches.
- Stoppers hold or release the shuttle before switch areas.
- Sensors are binary occupancy events, not continuous distance measurements.
- The kinematic node publishes state and sensor feedback for the supervisor,
  recorder, benchmark runner, and VLA agent.

For device YAML, marker behavior, collision tests, message types, launch names,
and detailed rail reference commands, go to
[docs/ROOM315_RAIL_DEVICES_AND_TESTS.md](docs/ROOM315_RAIL_DEVICES_AND_TESTS.md)
and [docs/ROOM315_RAIL_REFERENCE.md](docs/ROOM315_RAIL_REFERENCE.md).

## Rail-only VLA Layer

The Room 315 VLA layer treats the rail cell as a sparse-sensing research task:
language plus overhead images plus the previous command predicts event-level
symbolic actions. Binary rail state, exact Gazebo pose, true shuttle segment,
distance-to-switch, and normalized rail position stay out of `model_input`;
those values are kept only in `privileged_eval` for reset, auditing, and
evaluation.

The VLA stack includes:

- Independent right-rail and left-rail overhead RGB-D cameras.
- A high-level VLA supervisor on `/room_315/vla/command`.
- Primitive debugging commands for switches, stoppers, shuttles, stop-all, and
  emergency stop.
- A safety decoder between model actions and rail execution.
- Event-level symbolic action schema v3 with switch/stopper masks, shuttle
  identity, and coordination mode values.
- Dataset recording plus the curated 40-case payload batch runner.

Canonical station mapping:

```text
Right rail slots 1-2: Yaskawa HC10DT
Right rail slots 3-4: Staubli TX2
Left rail slots 1-2: Yaskawa HC10
Left rail slots 3-4: KUKA KR6
```

The active training/evaluation surface is the curated payload case matrix:

```text
mfja_robot_control_config/config/room_315_vla/payload_training_cases.yaml
```

Run one case with `room_315_pddl_scenario_generator.py --case-id ...`, or run
the 40-case batch with `room_315_payload_case_batch_runner.py`.

For supervisor operation, VLA topics, action vectors, and manual commands, go to
[docs/ROOM315_VLA_OPERATIONS.md](docs/ROOM315_VLA_OPERATIONS.md).
For the research formulation, model input, privileged evaluation, scenario
families, metrics, and roadmap, go to
[docs/ROOM315_VLA_RESEARCH.md](docs/ROOM315_VLA_RESEARCH.md).

## VLA Dataset and Evaluation Basics

The dataset recorder writes two streams per episode:

```text
events.jsonl   # event-level training labels
data.jsonl     # raw framewise replay/debug only
images/...     # overhead camera frames
```

Train on `events.jsonl`, not on repeated framewise `data.jsonl` labels. Each
training event represents:

```text
observation_before_decision -> next_symbolic_event_action
```

To export a flat training file:

```bash
ros2 run mfja_robot_control_config room_315_vla_event_extractor.py \
  ~/room315_smolvla_demo \
  --output meta/training_events.jsonl
```

For PDDL/PlanSys2 generated scenarios, the extractor is fail-closed: by default
it includes only episodes with `episodes/<episode_id>/validation.json` marked
`approved_for_training: true`. Use `--include-failed` or `--allow-unvalidated`
only for explicit debug exports.

To run the lightweight baseline evaluator:

```bash
ros2 run mfja_robot_control_config room_315_vla_baseline_eval.py \
  ~/room315_smolvla_demo \
  --output-dir ~/room315_vla_baselines \
  --holdout-fraction 0.2
```

The built-in evaluator reports `state_only`, `vla`, and `oracle` baselines with
task-level, action-level, device-level, timing, and safety metrics per task
family.

For recorder launch commands, benchmark runner usage, output file formats, and
metric definitions, go to [docs/ROOM315_VLA_OPERATIONS.md](docs/ROOM315_VLA_OPERATIONS.md)
and [docs/ROOM315_VLA_RESEARCH.md](docs/ROOM315_VLA_RESEARCH.md).

## Full Floor and Robots

Room 315 can be launched alone or as part of the full MFJA third-floor world.
The repository also includes industrial robot spawn/config workflows for KUKA
KR6, Staubli TX2, Yaskawa HC10, Yaskawa HC10DT, and TIAGo-related examples.

Robot lists are configured from YAML files such as:

```text
mfja_robot_control_config/config/robots.yaml
mfja_robot_control_config/config/robots_room_315_only.yaml
```

For full-floor launch, Gazebo services, robot spawning, isolated robot runs, and
robot topic checks, go to [docs/FULL_FLOOR_AND_ROBOTS.md](docs/FULL_FLOOR_AND_ROBOTS.md).

## Development and Tests

If you edit Python scripts, launch files, package metadata, worlds, models,
URDF/SDF, interfaces, or config files, rebuild and source the workspace. If you
only edit Markdown docs, no rebuild is required.

Useful checks:

```bash
python3 -m pytest mfja_robot_control_config/test
python3 -m pytest mfja_3rd_floor_description/test
git diff --check
colcon build --symlink-install --packages-select mfja_robot_control_config
colcon test --packages-select mfja_robot_control_config
```

For troubleshooting topics, runtime parameters, pose calibration, state/debug
topics, switch control, stopper workflow, and shuttle control details, go to
[docs/SHUTTLE_SWITCH_STOPPER_REFERENCE.md](docs/SHUTTLE_SWITCH_STOPPER_REFERENCE.md).

## Documentation Map

Use this README for the project overview and first commands. Detailed
instructions are split by topic:

- Installation, rosdep, Nix, build, source, and every-new-terminal setup:
  [docs/INSTALLATION.md](docs/INSTALLATION.md)
- Basic launch commands, Room 315 workflows, runtime shuttle commands,
  switches, stoppers, and sensor checks:
  [docs/QUICK_START_AND_FEATURE_GUIDE.md](docs/QUICK_START_AND_FEATURE_GUIDE.md)
- VLA supervisor operation, task templates, dataset recorder usage,
  event-level labels, model input, action vectors, benchmark runner, and VLA
  topics: [docs/ROOM315_VLA_OPERATIONS.md](docs/ROOM315_VLA_OPERATIONS.md)
- Rail-only VLA research formulation, `model_input` vs `privileged_eval`,
  baselines, metrics, scenario families, and 4-robot roadmap:
  [docs/ROOM315_VLA_RESEARCH.md](docs/ROOM315_VLA_RESEARCH.md)
- Rail device YAML, marker behavior, collision tests, message types, and launch
  names: [docs/ROOM315_RAIL_DEVICES_AND_TESTS.md](docs/ROOM315_RAIL_DEVICES_AND_TESTS.md)
- Room 315 path backend, Room 315-only launch reference, right/left rail quick
  commands, and typed interface details:
  [docs/ROOM315_RAIL_REFERENCE.md](docs/ROOM315_RAIL_REFERENCE.md)
- Full-floor launch, Gazebo service checks, robot spawning, and industrial
  robot/TIAGo command references:
  [docs/FULL_FLOOR_AND_ROBOTS.md](docs/FULL_FLOOR_AND_ROBOTS.md)
- Shuttle start slots, multiple shuttles, ON/OFF/RESET/REMOVE, stopper
  workflow, collision avoidance, switch control, pose calibration, debug topics,
  parameters, and troubleshooting:
  [docs/SHUTTLE_SWITCH_STOPPER_REFERENCE.md](docs/SHUTTLE_SWITCH_STOPPER_REFERENCE.md)
- Detailed Room 315 kinematic artifact notes:
  [mfja_robot_control_config/config/room_315_kinematics/README.md](mfja_robot_control_config/config/room_315_kinematics/README.md)
- Focused HTML runbook: [runbook.html](runbook.html)
