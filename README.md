# MFJA Third-Floor ROS 2 and Gazebo Simulation

This repository provides the ROS 2 Jazzy and Gazebo Harmonic simulation of the
MFJA third floor, with a detailed Room 315 rail cell, industrial and mobile
robots, typed rail-control interfaces, and an optional neuro-symbolic
Vision-Language-Action (VLA) research stack.

The default Room 315 rail motion is kinematic. Shuttles follow calibrated,
directed rail geometry and are moved through Gazebo pose services; wheel and
rail contact dynamics are not used to propel them.

> **Important scope:** this is a simulation and research repository. It is not
> a safety-certified controller for physical equipment. VLA task execution is
> fail-closed, disabled by default, and requires separately supplied,
> checksum-verified runtime artifacts.

## Start Here

Choose the path that matches your goal:

| Goal | Read this |
| --- | --- |
| Install the prerequisites and build a clean workspace | [Installation](docs/INSTALLATION.md) |
| Start Room 315 and try the rail controls | [Quick Start and Feature Guide](docs/QUICK_START_AND_FEATURE_GUIDE.md) |
| Understand the packages, runtime processes, and data flow | [System Architecture](docs/SYSTEM_ARCHITECTURE.md) |
| Change robots, worlds, rail geometry, sensors, or VLA settings | [Configuration and Customization](docs/CONFIGURATION.md) |
| Maintain, test, and extend the repository | [Maintenance Guide](docs/MAINTENANCE.md) |
| Diagnose a build or runtime problem | [Troubleshooting](docs/TROUBLESHOOTING.md) |
| Find every user, operator, research, and maintainer document | [Documentation Hub](docs/README.md) |
| Look up project terminology | [Glossary](docs/GLOSSARY.md) |

## What the Repository Contains

- Room 315 and complete third-floor Gazebo worlds.
- Kinematic right- and left-rail shuttle fleets with four public shuttle
  identities per side.
- Configurable switches, stoppers, indexing sensors, approach sensors, payload
  visuals, collision spacing, and runtime shuttle creation/removal.
- Typed ROS 2 messages and services under
  `/room_315/rails/{right,left}/...`.
- KUKA KR6, Staeubli TX2, Yaskawa HC10/HC10DT, and TIAGo simulation assets.
- Joint and industrial-gripper command helpers.
- Independent overhead RGB-D cameras for the Room 315 rail sides.
- Dataset capture, validation, splitting, training, evaluation, benchmark, and
  evidence tooling.
- English task-goal parsing, visual-state inference, PlanSys2 planning, a
  supervised primitive-action boundary, and closed-loop re-observation.

The principal limitations are documented in
[System Architecture: Scope and Boundaries](docs/SYSTEM_ARCHITECTURE.md#scope-and-boundaries).

## Supported Platform

The maintained platform is:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic through `ros_gz`
- Python 3.12
- CMake 3.28 or newer for the three packages that declare that minimum

The optional Nix development shell supplies build tools only. ROS 2 and Gazebo
still come from the host installation.

## Quick Installation

The commands below use a dedicated colcon workspace. Change `MFJA_WS` if you
prefer another location.

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"

mkdir -p "$MFJA_WS/src"
git clone https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz.git \
  "$MFJA_REPO"

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy -y \
  --skip-keys "python3-torch python3-torchvision"

cd "$MFJA_WS"
colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"

source "$MFJA_WS/install/setup.bash"
```

Using the four explicit package paths prevents downloaded datasets or frozen
source snapshots placed near the repository from being discovered as duplicate
colcon packages.

Every new terminal must source both ROS 2 and this workspace:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
```

See [Installation](docs/INSTALLATION.md) for host packages, `rosdep`, Nix,
verification, updating, optional Torch setup, and common installation failures.

## First Run: Room 315

By default, every high-level Room 315/full-floor launch removes the disposable
obstacle-pose cache at `~/.ros/room315_vla_obstacles.json`. Preserve it with
`room315_clear_vla_obstacle_pose_cache:=false`. If you override
`room315_vla_obstacle_pose_file`, point it only at the intended cache file: the
default clearing action unlinks that path before startup.

Start Room 315 without the heavier robot models, with one stopped shuttle on
each rail:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  start_paused:=false \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

The simulation starts the world immediately. Rail nodes are added after a short
startup delay. In another sourced terminal, verify the API:

```bash
ros2 topic list | grep '^/room_315/rails/'
ros2 service list | grep '^/world/room_315_only/'
ros2 topic echo --once /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Start the right shuttle:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON', speed: 0.2}"
```

Stop the launch with `Ctrl-C`. Do not start `room_315_only.launch.py` and
`full_floor.launch.py` at the same time: the high-level launch owns an exclusive
Room 315 runtime lock because the ROS topics and Gazebo world services are
process-global.

## Launch Profiles

| Profile | Command | Default behavior |
| --- | --- | --- |
| Room 315 only | `ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py` | GUI on, simulation running, rails enabled, zero initial shuttles, robots selected from the Room 315 YAML unless `robots:=none` is passed |
| Complete third floor | `ros2 launch mfja_3rd_floor_bringup full_floor.launch.py` | GUI on, simulation paused, rails enabled, zero initial shuttles, robots selected from the full-floor YAML |
| One industrial robot | `ros2 launch mfja_3rd_floor_bringup single_industrial_robot.launch.py robot:=kuka` | Minimal world with one robot and its table; selectors are `kuka`, `staubli`, `hc10`, and `hc10dt` |
| Headless Room 315 | Add `gui:=false start_paused:=false robots:=none` | Server-only simulation suitable for automated runs |
| Room 315 with VLA bridge/supervisor | Add `enable_room315_vla:=true` | Camera bridge and primitive supervisor enabled; learned visual inference and task execution remain separate processes |

Inspect the authoritative arguments and current defaults from the installed
launch file:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
```

The current full-floor YAML enables both `tiago1` and `tiago_base1`, but mobile
frames are not instance-prefixed. Launching both produces duplicate TF frame
IDs such as `odom` and `base_footprint`. For TF-dependent work, select at most
one mobile robot by exact name; the generic `tiago` selector is ambiguous when
both entries exist. Avoid `robots:=all` until mobile-frame namespacing is added.

## Rail Control Essentials

Right-rail examples are shown below. Replace `right` with `left` for the other
rail.

### Shuttle Commands

```bash
# Stop, reset to the configured start position, or remove a shuttle.
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'OFF'}"

ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'RESET'}"

ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'REMOVE'}"
```

Add a shuttle at runtime:

```bash
ros2 service call /room_315/rails/right/shuttles/add \
  mfja_rail_interfaces/srv/AddShuttle \
  "{name: 'room315_right_shuttle_2', start_slot: '1', speed: 0.2, start_enabled: false}"
```

### Switches and Stoppers

Switch state `E`/`EXTERIOR` selects the exterior branch;
`I`/`INTERIOR` selects the interior branch. Stopper state `0` is open/pass and
`1` is closed/stop.

Switches form coordinated pairs: `A1`/`A2` and `A3`/`A4`. The typed command
topic is a low-level simulation interface and bypasses the VLA route-planning
boundary. Use it to move a route only on an empty rail. Normal numbered start
slots lie on exterior guard segments, so resetting a shuttle there is not safe
preparation for an interior route. For this switch-only example, stop the prior
launch and restart it with both shuttle counts set to `0`. Never reroute a
moving shuttle after a fixed timer.

```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'A1', state: 'INTERIOR'}, {name: 'A2', state: 'INTERIOR'}]}"

ros2 topic pub --once /room_315/rails/right/stoppers/command \
  mfja_rail_interfaces/msg/StopperCommand \
  "{stoppers: [{name: 'A1', state: '1'}]}"
```

Commands are requests. The corresponding `.../state` topic reports the actual
state after the configured motion delay.

### Sensor Feedback

```bash
ros2 topic echo /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Sensors report binary occupancy. `active: 1` means a shuttle is inside the
configured sensor radius; it is not a distance measurement. The message also
carries privileged segment/arc-length fields for debugging and evaluation;
those fields are not additional physical distance sensors and are excluded from
the learned visual-model input.

The complete operator procedure is in
[Quick Start and Feature Guide](docs/QUICK_START_AND_FEATURE_GUIDE.md), and the
full API is in [Room 315 Rail Reference](docs/ROOM315_RAIL_REFERENCE.md).

## Robots

Robot spawn lists live in:

- `mfja_robot_control_config/config/robots.yaml` for the complete floor.
- `mfja_robot_control_config/config/robots_room_315_only.yaml` for Room 315.

The `robots` launch argument accepts `none`, `all`, an exact robot name, a
supported unambiguous alias, a YAML list index, or a comma-separated
combination. Prefer exact names for mobile robots. Examples:

```bash
ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=kuka1,tiago1 gui:=true start_paused:=false

ros2 run mfja_robot_control_config robot_joint_command.py --list
ros2 run mfja_robot_control_config robot_gripper_command.py --list
```

Grippers are animated symmetric mechanisms; they do not currently attach or
physically grasp payloads. See
[Full Floor and Robot Reference](docs/FULL_FLOOR_AND_ROBOTS.md).

## VLA and Language-to-Motion Workflows

The current control boundary is intentionally neuro-symbolic:

```text
confirmed English TaskGoal
  -> accepted visual facts and deterministic presence
  -> PlanSys2 problem and plan
  -> one validated primitive
  -> rail supervisor
  -> effect check, new observation, and replan
```

Learned components may propose task-goal fields or produce visual facts. They
do not publish rail commands directly. Exact simulator pose, true segment, and
binary rail signals remain outside learned visual-model input and are used only
in declared safety, presence, or evaluation boundaries.

Basic VLA camera/supervisor launch:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=true \
  start_paused:=false \
  enable_room315_vla:=true
```

Active visual inference and motion execution are not turnkey on a fresh clone.
They require an external V4 promotion manifest, the artifacts referenced by
that manifest, exact checksums, and a host-local task-execution authorization
file. The checked-in runtime YAML records a qualified project environment and
contains site-specific absolute paths; do not enable execution by merely
changing `execution_enabled`.

Read these in order for the advanced runtime:

1. [VLA Operations](docs/ROOM315_VLA_OPERATIONS.md)
2. [Visual Runtime Integration](docs/room315_visual_runtime_integration.md)
3. [Task-Goal Understanding](docs/ROOM315_TASK_GOAL_UNDERSTANDING.md)
4. [PDDL Planning](docs/ROOM315_PDDL_PLANNING.md)
5. [Language-to-Motion Runtime](docs/ROOM315_LANGUAGE_TO_MOTION_RUNTIME.md)

Dataset and checkpoint releases are described in the
[Documentation Hub](docs/README.md#datasets-models-and-evidence). Large
datasets, checkpoints, caches, and generated experiment output should remain
outside the Git working tree.

## Repository Layout

This repository is a colcon meta-repository, not a ROS package itself.

| Path | Responsibility |
| --- | --- |
| `mfja_3rd_floor_bringup/` | Stable high-level launch entry points |
| `mfja_3rd_floor_description/` | Gazebo worlds, SDF models, meshes, URDF, and the symmetric-gripper C++ plugin |
| `mfja_rail_interfaces/` | Typed Room 315 messages and the runtime shuttle-add service |
| `mfja_robot_control_config/` | Robot spawning, rail runtime, VLA/planning nodes, tools, configuration, launch files, and most tests |
| `config/room_315_vla/` | Top-level experiment configuration retained for specific dataset work |
| `docs/` | User, operator, architecture, maintenance, research, and audit documentation |
| `report/` | English report sources, figures, and checksum-bound evidence records |
| `flake.nix` | Optional hybrid Nix development shell |

See [System Architecture](docs/SYSTEM_ARCHITECTURE.md) for package dependencies,
launch sequencing, ROS interfaces, data ownership, and runtime files.

## Common Configuration Entry Points

| Change | Source of truth |
| --- | --- |
| Enable, disable, rename, or reposition robots | `mfja_robot_control_config/config/robots*.yaml` |
| Change industrial gripper travel/defaults | `mfja_robot_control_config/config/gripper_command_defaults.yaml` |
| Change Room 315 slots, sensors, stoppers, or radii | `mfja_robot_control_config/config/room_315_kinematics/rail_devices_{right,left}.yaml` |
| Change rail topology or segment-to-CSV mapping | `mfja_robot_control_config/config/room_315_kinematics/rail_network_{right,left}.yaml` |
| Change measured rail geometry | `mfja_robot_control_config/config/room_315_kinematics/raw_segments/*.csv` |
| Change a world or model | `mfja_3rd_floor_description/worlds/` or `models/` |
| Change launch defaults and feature wiring | `mfja_3rd_floor_bringup/launch/room_315_floor_common.py` |
| Change shuttle identity/color mapping | `mfja_robot_control_config/config/room_315_vla/shuttle_identity.yaml` |
| Change supervisor defaults | `mfja_robot_control_config/config/room_315_vla/vla_supervisor.yaml` |
| Change visual-runtime or execution policy | `mfja_robot_control_config/config/room_315_vla/*.yaml`, preserving the artifact and authorization contracts |

Before editing any of these files, use
[Configuration and Customization](docs/CONFIGURATION.md), which explains the
required companion changes, validation, rebuild, and restart for each case.

## Development and Verification

Documentation-only changes do not require a ROS rebuild. Source, launch,
interface, model, world, and installed configuration changes do.

Typical checks from the repository root are:

```bash
git diff --check
python3 -m pytest mfja_3rd_floor_description/test
python3 -m pytest \
  mfja_robot_control_config/test/test_room315_kinematic_shuttle_core.py
```

The full direct control-package collection requires the optional Torch
environment; without Torch, module-level dependency guards can make collection
exit with "no tests collected." See the Maintenance Guide for the focused and
full matrices.

After a colcon build:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
colcon test --packages-select \
  mfja_3rd_floor_description \
  mfja_robot_control_config
colcon test-result --verbose
```

The control package has a large research test suite, and optional Torch-backed
tests are registered only when Torch is importable at CMake configure time.
Start with a focused test for the component you changed, then run the package or
full suite before merging. Direct `pytest` and `colcon test` cover different
sets, so use the matrix in [Maintenance Guide](docs/MAINTENANCE.md).

## Fast Troubleshooting

- `Package ... not found`: build from the workspace root and source the same
  workspace's `install/setup.bash` in the current terminal.
- A new executable is missing: rebuild `mfja_robot_control_config`, source the
  overlay again, and confirm that the script is listed in its `CMakeLists.txt`.
- No shuttle is visible: integrated launches default to zero initial shuttles;
  set a shuttle count or call the add service.
- A shuttle is visible but stationary: initial shuttles default to disabled;
  publish `ON`, or launch with `room315_shuttles_start_enabled:=true`.
- A shuttle reports `WAITING`: inspect stopper state and shuttle spacing.
- A shuttle reports `FALLING`: the selected switch configuration has no valid
  successor; correct the route and issue `RESET`.
- Gazebo pose/create/remove services are missing: verify the internal world
  name and the `/world/<world_name>/...` service namespace.
- A second Room 315 launch is rejected: stop the first high-level Room 315 or
  full-floor launch. The lock file itself is harmless after its owner exits.
- VLA inference cannot start on another computer: replace the site-specific
  runtime paths with a valid local, checksum-matching promotion bundle.

For diagnosis commands and a symptom-to-fix table, see
[Troubleshooting](docs/TROUBLESHOOTING.md).

## Documentation and Attribution

The complete, categorized documentation index is
[docs/README.md](docs/README.md). When code and prose disagree, the installed
launch arguments (`--show-args`), interface definitions, and versioned
configuration are authoritative; update the documentation in the same change.

The ROS package manifests declare Apache License 2.0 for package code. Imported
robot meshes and CAD assets retain their own upstream terms. Review
[Third-Party Asset Attribution](mfja_3rd_floor_description/THIRD_PARTY.md)
before redistributing or replacing assets.
