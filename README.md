# MFJA 3rd Floor Gazebo - Room 315 Kinematic Shuttle

This repository contains the Gazebo Harmonic / ROS 2 Jazzy simulation assets for
the MFJA 3rd floor, with the current focus on the Room 315 flexible rail system.

The current project state is a **kinematic-first shuttle simulation**. The
shuttle follows calibrated Room 315 rail geometry through an explicit rail graph
and publishes Gazebo poses through `/world/<world_name>/set_pose`. The project
also contains the Room 315 rail-only VLA research layer for sparse binary
sensing, overhead images, event-level actions, safety decoding, dataset
recording, and benchmark evaluation.

## Repository Layout

This git repository is a **meta-repository**. The repository root is not a ROS 2
package. Package-specific files must live inside the package that owns them.

- `mfja_3rd_floor_description/`: models, meshes, worlds, and URDF/SDF assets.
- `mfja_rail_interfaces/`: typed ROS 2 message interfaces for Room 315 rail commands, states, and sensors.
- `mfja_robot_control_config/`: launch base, bridge config, shuttle/switch scripts, Room 315 kinematic config, VLA supervisor, recorder, benchmark, and evaluator tools.
- `mfja_3rd_floor_bringup/`: launch entry points for Room 315, the full floor, and isolated industrial robot runs.
- `mfja_robot_control_config/config/room_315_kinematics/raw_segments/`: source rail segment CSV files for the Room 315 kinematic rail network.

## Documentation Map

Use this README as the entry point only. Detailed instructions are split by
topic:

- Need installation, rosdep, Nix, build, source, and every-new-terminal setup? Go to [docs/INSTALLATION.md](docs/INSTALLATION.md).
- Need the basic launch commands, Room 315 launch workflow, runtime shuttle commands, switches, stoppers, and sensor checks? Go to [docs/QUICK_START_AND_FEATURE_GUIDE.md](docs/QUICK_START_AND_FEATURE_GUIDE.md).
- Need VLA supervisor operation, task templates, dataset recorder usage, event-level labels, model input, action vectors, benchmark runner, and VLA topics? Go to [docs/ROOM315_VLA_OPERATIONS.md](docs/ROOM315_VLA_OPERATIONS.md).
- Need the research formulation for rail-only VLA under sparse binary sensing, model_input vs privileged_eval, baselines, metrics, scenario families, and the 4-robot roadmap? Go to [docs/ROOM315_VLA_RESEARCH.md](docs/ROOM315_VLA_RESEARCH.md).
- Need rail device YAML, marker behavior, shuttle-shuttle collision tests, robot-shuttle collision tests, message types, or launch names? Go to [docs/ROOM315_RAIL_DEVICES_AND_TESTS.md](docs/ROOM315_RAIL_DEVICES_AND_TESTS.md).
- Need the Room 315 path backend, Room 315-only launch reference, right/left rail quick commands, and typed interface details? Go to [docs/ROOM315_RAIL_REFERENCE.md](docs/ROOM315_RAIL_REFERENCE.md).
- Need full-floor launch, Gazebo service checks, robot spawning, and industrial robot/TIAGo command references? Go to [docs/FULL_FLOOR_AND_ROBOTS.md](docs/FULL_FLOOR_AND_ROBOTS.md).
- Need allowed shuttle start slots, multiple shuttle runtime control, ON/OFF/RESET/REMOVE, stopper workflow, collision avoidance, switch control, pose calibration, state/debug topics, parameters, and troubleshooting? Go to [docs/SHUTTLE_SWITCH_STOPPER_REFERENCE.md](docs/SHUTTLE_SWITCH_STOPPER_REFERENCE.md).
- Need the detailed Room 315 kinematic artifact notes? Go to [mfja_robot_control_config/config/room_315_kinematics/README.md](mfja_robot_control_config/config/room_315_kinematics/README.md).
- Need the focused HTML runbook? Open [runbook.html](runbook.html).

## Quick Workspace Reminder

For a fresh workspace, clone this repository inside a colcon `src/` directory,
install dependencies with rosdep, build from the workspace root, then source the
workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/mfja_3rd_floor_gz -y --ignore-src --rosdistro jazzy
colcon build --symlink-install --base-paths src/mfja_3rd_floor_gz
source install/setup.bash
```

Full setup details are in [docs/INSTALLATION.md](docs/INSTALLATION.md).

## Quick Room 315 Launch Pointer

The common Room 315 launch entry point is:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true \
  enable_room315_kinematic_shuttles:=true
```

More launch variants and test commands are in
[docs/QUICK_START_AND_FEATURE_GUIDE.md](docs/QUICK_START_AND_FEATURE_GUIDE.md).
