# Configuration and Customization

This guide identifies the source of truth for common changes and explains the
companion edits, rebuilds, restarts, and tests each change requires. Use it
before editing a generated file or changing one part of a multi-file contract.

## Configuration Rules

1. Edit source files under the four ROS packages, not `build/`, `install/`,
   `log/`, `/tmp`, or generated dataset/evidence directories.
2. Keep large datasets, checkpoints, virtual environments, and runtime
   candidates outside the repository and colcon source tree.
3. Use a symlink install during development, but still rebuild after changing
   launch files, package metadata, interfaces, C++, models, worlds, or installed
   configuration.
4. Restart the affected launch after changing startup configuration. Most YAML
   files are read only when a node is created.
5. Run the focused tests listed here before the complete package test suite.
6. Preserve immutable hashes and evidence. Create a new versioned artifact
   rather than editing a qualified manifest or frozen split in place.

## Configuration Precedence

For the normal high-level Room 315/full-floor launch, values flow through these
layers:

```text
command-line launch argument
  -> default in room_315_floor_common.py
  -> lower-level launch argument
  -> ROS node parameter
  -> implementation default only when no launch value was supplied
```

Use high-level `room315_*` arguments with
`mfja_3rd_floor_bringup`. Unprefixed arguments such as `speed` belong to the
lower-level rail launch and may also appear in `--show-args` because launch
descriptions are included. The high-level mapping wins for the integrated
runtime.

Important scope differences:

| Setting | Standalone rail-node default | Integrated floor-launch default |
| --- | --- | --- |
| Initial shuttle count | `1` | `0` per side |
| Speed | `0.25 m/s` | `0.2 m/s` |
| Gazebo pose update rate | `30 Hz` | `30 Hz` |
| Sensor feedback rate | `10 Hz` | `30 Hz` |
| Start enabled | `false` | `false` |
| Path backend | `cubic_hermite` | `cubic_hermite` |
| Device markers | `true` | `true` |
| Shuttle spacing threshold | `0.33 m` | `0.33 m` |

### Launch-Time Obstacle Cache

The high-level Room 315/full-floor launch defaults
`room315_clear_visual_obstacle_pose_cache:=true`, which unlinks the configured
`room315_visual_obstacle_pose_file` before startup. The normal path is the
disposable cache `~/.ros/room315_visual_obstacles.json`. Set the clear argument to
`false` when the cache must survive a restart. If you override the path, verify
that it identifies only the intended cache file; the clearing function does not
validate its purpose.

Always inspect the installed entry point you intend to run:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
ros2 param dump /room_315/rails/right/room_315_kinematic_shuttle
```

## Quick Source-of-Truth Matrix

| Desired change | Primary source | Typical companion files |
| --- | --- | --- |
| Robot instance, pose, or enabled state | `mfja_robot_control_config/config/robots*.yaml` | Gripper config for renamed industrial robots |
| Robot geometry/controller | `mfja_3rd_floor_description/models/<robot>/model.sdf` | Matching URDF, meshes, launch model sets, attribution |
| Industrial gripper range | `mfja_robot_control_config/config/gripper_command_defaults.yaml` | No manual SDF patch; launch materializes it |
| Static room/furniture composition | `mfja_3rd_floor_description/worlds/*.world` | Referenced model directory |
| Reusable model | `mfja_3rd_floor_description/models/<name>/` | World include or spawn logic, attribution |
| Rail path samples | `config/room_315_kinematics/raw_segments/*.csv` | Explicit network `csv:` mapping and topology validation |
| Rail nodes/routing/switch topology | `rail_network_{right,left}.yaml` | CSVs, left public-name mapping, PDDL if topology contract changes |
| Slot/sensor/stopper position | `rail_devices_{right,left}.yaml` | Marker/sensor tests and scenario assumptions |
| Persistent rail-to-Gazebo calibration | `scripts/room_315_rail_defaults.py` | Both side-specific tests and device coordinates |
| Startup shuttle identities/payloads | Launch arguments | `shuttle_identity.yaml` and identity models for structural changes |
| Shuttle identity colors/tags | `config/room_315_shuttle_identity/shuttle_identity.yaml` | Identity models, world includes, multi-shuttle conventions |
| Camera pose/spec/topic | `models/room315_visual_observation_rig/model.sdf` | Both camera bridge launches, runtime config/calibration, tests |
| GUI layout | `config/room315_runtime_safe.gui.config` or `mfja_light.gui.config` | Launch profile when adding a new config |
| Primitive-supervisor defaults | `config/room_315_task_execution/rail_safety_supervisor.yaml` | Supervisor tests and operations docs |
| Live planning domain | `config/room_315_planning/pddl/domain_room315_runtime.pddl` | Translator, validation gate, executive, tests |
| Visual runtime | `config/room_315_visual_state/visual_state_runtime.yaml` | External promotion bundle and exact SHA-256 |
| Task execution policy | `task_execution_runtime*.yaml` | External authorization and promotion manifests |
| English intent runtime | `task_goal_understanding.yaml` or generated local config | External GGUF path and pinned hash |

Paths beginning with `config/` in the table are under
`mfja_robot_control_config/` unless stated otherwise.

## Robots

### Enable, Disable, Select, or Reposition a Robot

Edit one of:

- `mfja_robot_control_config/config/robots.yaml` for the full floor.
- `mfja_robot_control_config/config/robots_room_315_only.yaml` for Room 315.

Each entry uses this shape:

```yaml
robots:
  - name: tiago1
    model: tiago_with_arm
    x_pose: -3.0
    y_pose: -3.0
    z_pose: 0.0
    yaw: 1.57
    enabled: true
```

Rules:

- `name` must be non-empty and unique within the file.
- `enabled` controls selection only when the launch `robots` argument is empty.
- An explicit `robots:=...` selection can select an entry even when its
  `enabled` field is false.
- Industrial model values are `kuka_kr6r900sixx`, `staubli_tx2_60l`,
  `yaskawa_hc10`, and `yaskawa_hc10dt`.
- TIAGo-with-arm aliases include `tiago`, `tiago_with_arm`, and `tiago_arm`.
- Base-only aliases include `tiago_base`, `tiago_no_arm`, and
  `tiago_mobile_base`.
- Pose units are meters and radians.
- If an industrial instance is renamed, add the same instance key under
  `grippers:` in `gripper_command_defaults.yaml`; a missing entry aborts the
  launch. Update the hard-coded instance/topic profiles in
  `robot_joint_command.py` and `robot_gripper_command.py` if the helpers must
  support the new name.
- Mobile robot-state and DiffDrive frames are prefixed with the unique instance
  name, so multiple TIAGo variants can share one ROS graph. Keep every robot
  name unique and use exact selectors when a short alias is ambiguous.

After editing, rebuild `mfja_robot_control_config`, source the overlay, and
relaunch. Verify with:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=tiago1 gui:=true start_paused:=false
ros2 topic list | grep '^/tiago1/'
```

### Change an Industrial Gripper Range

Edit:

```text
mfja_robot_control_config/config/gripper_command_defaults.yaml
```

Each configured position is a per-jaw prismatic distance in meters:

```yaml
grippers:
  kuka1:
    position_at_0_percent_m: 0.0
    position_at_100_percent_m: 0.030
    default_open_percentage: 100.0
    default_close_percentage: 100.0
```

Constraints:

- The 0% position must be non-negative.
- The 100% position must be greater than the 0% position.
- Default percentages must remain in `[0, 100]`.
- Both jaws move symmetrically, so the full opening is twice the per-jaw travel
  when both jaws have equal opposing motion.
- Restart the robot launch after a range change. The launcher writes a patched
  temporary SDF and supplies the patched URDF in memory as `robot_description`;
  edit the source templates rather than generated process assets.

Validate without publishing:

```bash
python3 mfja_robot_control_config/scripts/robot_gripper_command.py \
  --defaults-file mfja_robot_control_config/config/gripper_command_defaults.yaml \
  --list
python3 mfja_robot_control_config/scripts/robot_gripper_command.py \
  --defaults-file mfja_robot_control_config/config/gripper_command_defaults.yaml \
  kuka open 75 --dry-run
```

Then run the gripper config, bridge, command, and articulation tests listed in
[Maintenance Guide](MAINTENANCE.md#change-to-test-matrix).

### Add a New Robot Model

A new robot is a cross-package feature, not a single YAML entry. At minimum:

1. Add `models/<model_name>/model.config`, `model.sdf`, and all referenced
   meshes under `mfja_3rd_floor_description`.
2. Add `urdf/<model_name>.urdf`. The current launch resolves both SDF and URDF
   for every supported robot and aborts if either is absent.
3. Add the model to the supported mobile or industrial model set and bridge
   behavior in `multi_robot_sim.launch.py`.
4. Add aliases only when they remain unambiguous.
5. If isolated mode is required, add a table/layout mapping in
   `isolated_industrial_robot.launch.py`.
6. Add an instance to the appropriate robot YAML.
7. Add gripper configuration and update the joint/gripper helper profiles if
   the model has an industrial gripper or should be addressable by those tools.
8. Record imported asset provenance and terms in `THIRD_PARTY.md`.
9. Add spawn, collision, bridge, joint, and model-structure tests.

Do not assume that placing a directory under `models/` automatically makes a
robot spawnable; launch support and a matching URDF are required.

## Worlds and Reusable Models

### Change a Static Object or Room Layout

Edit the applicable source world:

```text
mfja_3rd_floor_description/worlds/room_315_only.world
mfja_3rd_floor_description/worlds/mfja_3rd_floor.world
```

Use `<include>` for reusable models and keep each entity name unique. When a
change belongs to a reusable object rather than one world instance, edit that
model's `model.sdf` instead.

World invariants:

- Keep the file stem and internal `<world name>` equal.
- Preserve the required Gazebo system plugins for physics, user commands,
  scene broadcast, and sensors unless the launch/runtime is changed with them.
- Preserve camera and switch entity names if existing bridges/controllers refer
  to them.
- Test both Room 315 and the full floor when editing an asset shared by both.

### Add a Reusable Gazebo Model

Create:

```text
mfja_3rd_floor_description/models/<name>/model.config
mfja_3rd_floor_description/models/<name>/model.sdf
mfja_3rd_floor_description/models/<name>/meshes/...
```

Use package-relative/model URIs rather than author-specific absolute paths.
Reference it from a world with `model://<name>` or spawn it through launch/code.
The description package installs the entire `models/` directory, so no CMake
list update is normally needed for a new file inside it. A new third-party asset
still requires attribution review.

Rebuild the description package, source the overlay, relaunch, and check the
Gazebo console for missing URI/mesh errors.

## Rail Geometry and Topology

### What Each File Owns

- `raw_segments/*.csv`: sampled geometric centerlines.
- `rail_network_right.yaml` and `rail_network_left.yaml`: explicit `csv:`
  binding, directed nodes, segment endpoints, switch rules, fixed transitions,
  and routing tables.
- `rail_devices_right.yaml` and `rail_devices_left.yaml`: user-editable slots,
  binary sensors, stoppers, radii, and default states.
- `room_315_rail_defaults.py`: rail-to-Gazebo transforms, public topic/entity
  defaults, left internal/public segment mapping, and visual constants.

The `csv:` field is authoritative. Never infer a CSV filename from a segment
name. Read [Segment CSV Migration](../mfja_robot_control_config/config/room_315_kinematics/SEGMENT_CSV_MIGRATION.md)
before changing the schema.

### Change a Rail CSV

Preserve a consistent numeric row format and forward traversal direction. Then
verify that:

- each network segment references the intended CSV explicitly;
- the first and last rows still correspond to the declared start/end nodes;
- adjacent endpoints remain within the configured snap tolerance;
- switch routing remains complete for all supported states;
- slots, sensors, stoppers, and start positions still lie on valid segments;
- left public/internal names are interpreted through the explicit mapping.

Run normalization, segment-name, topology, route, kinematic-core, device, and
collision tests before a visual launch.

### Change Rail Topology or Switch Routing

Edit the relevant `rail_network_*.yaml`. Every segment is forward-only. A
diverging switch maps a state to an outgoing segment; a converging gate validates
which incoming branch is open. Missing successors intentionally result in
`FALLING`.

A topology contract change may also require updates to:

- `room_315_rail_defaults.py` public-name conversion.
- device YAML segment references.
- PDDL domains and topology problem generation.
- scenario cases and expected planner provenance.
- supervisor safety maps and tests.
- diagrams and rail reference documentation.

Treat this as a versioned behavior change, not a cosmetic YAML edit.

## Slots, Sensors, and Stoppers

Device entries use a segment plus normalized arc-length ratio:

```yaml
- name: DZI1R
  segment: A12E
  s_ratio: 0.411866742
  radius_m: 0.09
```

A multi-branch device uses `points:`. A stopper-linked sensor derives its point
from its matching stopper and `before_stopper_m`:

```yaml
- name: A4_STOPPER_SENSOR
  stopper: A4
  before_stopper_m: 0.1
  radius_m: 0.08
```

Rules:

- Keep `s_ratio` in `[0, 1]`.
- Use a valid segment from the corresponding network.
- Keep public names unique in their category.
- A sensor radius controls binary occupancy and must not be described as a
  continuous distance range.
- State `0` opens a stopper; state `1` closes it.
- Move a linked sensor by moving its stopper or changing
  `before_stopper_m`; do not add a conflicting direct point.

### Convert a Gazebo Coordinate to `segment + s_ratio`

The device-position tool is dry-run by default. Always pass explicit source
paths so an installed copy cannot be edited accidentally:

```bash
python3 mfja_robot_control_config/scripts/room_315_device_position_tool.py \
  --side right \
  --x -14.95 --y -3.86 --z 0.84 \
  --network mfja_robot_control_config/config/room_315_kinematics/rail_network_right.yaml \
  --devices mfja_robot_control_config/config/room_315_kinematics/rail_devices_right.yaml
```

Review the reported distance and proposed point. To update a specific entry,
add the category/name and only then add `--write`:

```bash
python3 mfja_robot_control_config/scripts/room_315_device_position_tool.py \
  --side right \
  --x -14.95 --y -3.86 --z 0.84 \
  --category position_sensors --name DZI1R \
  --network mfja_robot_control_config/config/room_315_kinematics/rail_network_right.yaml \
  --devices mfja_robot_control_config/config/room_315_kinematics/rail_devices_right.yaml \
  --write
```

Inspect the diff, rebuild the control package, relaunch, and validate both the
marker and binary feedback. Do not use `--force` unless the measured offset has
been independently reviewed.

## Rail-to-Gazebo Calibration

Runtime pose offsets can be sent to each rail's
`shuttles/pose_offset_command` topic for temporary visual calibration. They are
not a persistent source of truth.

Persistent affine/scale/rotation/offset values live in
`room_315_rail_defaults.py` under separate right and left calibration defaults.
A persistent calibration change moves shuttles and derived device markers, so
validate:

- slot alignment;
- both rail loops and every branch;
- device marker placement;
- collision clearance;
- camera framing and visual calibration assumptions;
- left public/internal mapping behavior.

Record the measurement method and update any affected figures/calibration
contracts in the same change.

## Shuttle Identities and Payload Visuals

The structural identity contract spans:

- `config/room_315_shuttle_identity/shuttle_identity.yaml`;
- `scripts/room_315_multi_shuttle.py` conventions;
- identity-specific model directories `room315_shuttle_R1` through `R4` and
  `room315_shuttle_L1` through `L4`;
- the preloaded world includes;
- launch counts/explicit identity selection;
- visual labels, keepout zones, scenario generators, and tests.

For an exact subset, use:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  room315_identity_selection_mode:=explicit \
  room315_right_shuttle_count:=2 \
  room315_right_active_identities:=R2,R4 \
  room315_right_start_slots:=1,3 \
  room315_right_loaded_shuttles:=R4
```

Counts, identity lists, and start-slot lists must describe the same intended
fleet. Identity/color/tag changes require visual-pipeline and dataset-contract
review; do not reuse an old label meaning for a new physical identity.

Payload state topics are privileged simulation/debug metadata. A loaded visual
box does not mean the gripper or shuttle has a physics attachment, and it does
not become a learned input merely because it is published by the controller.

## Cameras

The Room 315 overhead camera model is:

```text
mfja_3rd_floor_description/models/room315_visual_observation_rig/model.sdf
```

It owns camera pose, resolution, update rate, field of view, clipping planes,
Gazebo topic roots, and optical frame names. A topic or frame change must be
coordinated with:

- `room_315_perception_and_safety.launch.py`;
- `room_315_visual_state_runtime.launch.py`;
- dataset recorder and visual inference parameters;
- visual calibration/config files;
- tests and any frozen model contract that names the old camera/frame.

Do not assume that a similarly named calibration frame already equals the SDF
optical frame. Verify both strings and the transform semantics before changing
or documenting them.

## Launch Defaults

The shared high-level floor arguments live in:

```text
mfja_3rd_floor_bringup/launch/room_315_floor_common.py
```

Lower-level behavior is implemented in the control package launch files. When
adding or renaming an argument:

1. declare it at the correct layer;
2. forward it explicitly to the included launch;
3. preserve its type with `ParameterValue` where scalar-looking strings are
   expected;
4. update both profiles if the behavior applies to both;
5. run `--show-args` on every affected public entry point;
6. update operational documentation and launch-argument tests.

The visual loop controller accepts `auto`, `INTERIOR`, or `EXTERIOR` for
`initial_loop_mode`.

## ROS Interfaces

Interface sources live in `mfja_rail_interfaces/msg` and `srv`. To change one:

1. edit or add the `.msg`/`.srv` source;
2. update `rosidl_generate_interfaces` in
   `mfja_rail_interfaces/CMakeLists.txt`;
3. add dependency declarations when the interface imports another package;
4. update every publisher, subscriber, serializer, fixture, and documentation
   consumer;
5. rebuild `mfja_rail_interfaces` and all dependent packages;
6. source the new overlay before inspecting the type;
7. decide whether the change is backward compatible or needs a versioned
   message/schema/topic.

```bash
ros2 interface show mfja_rail_interfaces/msg/ShuttleCommand
ros2 interface show mfja_rail_interfaces/msg/VisualStateObservation
```

Generated Python/C++ interface bindings under `build/` and `install/` must
never be edited.

## Visual State, Planning, Safety, and Language Configuration

### Supervisor and Cases

- `rail_safety_supervisor.yaml`: default shuttle names, speed, and supervisor history.
- `payload_training_cases_expanded_160_speed_sweep.yaml`: curated 160-case
  regression matrix.
- `payload_scenarios.yaml`: payload scenario definitions.
- `shuttle_identity.yaml`: canonical identity/visual mapping.
- `visual_observed_state_calibration.yaml`: visual observation calibration.

Changes to a regression case should preserve its stable identifier or create a
new identifier when semantics change. Generated benchmark extensions belong in
an external output directory unless intentionally versioned.

### PDDL and Execution

- `domain_room315.pddl` is used by expert/dataset planning workflows.
- `domain_room315_runtime.pddl` is the live task-execution domain.
- `task_execution_runtime.yaml` and `task_execution_runtime_v4.yaml` contain
  fail-closed execution policy and qualified authorization references.

A PDDL action is executable only when the translator, validation gate,
supervisor, effect checker, and tests support it. Adding syntax to a domain
alone does not create a safe runtime capability.

### Visual Runtime Artifacts

`visual_state_runtime.yaml` points to a V4 promotion manifest and expected hash.
The manifest in turn identifies the approved model and contract artifacts.
Checked-in absolute paths document one qualification environment; they are not
portable defaults.

For another machine:

1. obtain the complete approved runtime candidate outside Git;
2. verify its published checksums;
3. use a host-local runtime YAML or launch overrides for the promotion-manifest
   path and expected SHA-256;
4. run validation-only/CPU smoke and shadow checks;
5. create a new authorization only through the documented qualification flow;
6. keep execution disabled until every gate passes.

Never make the system start by deleting hash checks or copying an unrelated
hash into the config.

### Local English Intent Model

The setup tool downloads a pinned GGUF file. Its built-in dependency path can
modify the user Python site and retry with `--break-system-packages`, so use a
dedicated environment, install the pinned binding there, and explicitly skip
the script's dependency installer:

```bash
export ROOM315_INTENT_DIR="$HOME/models/room315_intent"
python3 -m venv --system-site-packages "$HOME/.venvs/room315-intent"
source "$HOME/.venvs/room315-intent/bin/activate"
python -m pip install --upgrade pip
python -m pip install 'llama-cpp-python==0.3.16'

python3 "$MFJA_REPO/mfja_robot_control_config/scripts/setup_room315_intent_model.py" \
  --model-dir "$ROOM315_INTENT_DIR" \
  --skip-dependency-install
source "$ROOM315_INTENT_DIR/room315_intent.env"
```

Allow 1.04 GiB (1.12 GB) for the checkpoint plus package/build cache space.
Network access is required for initial setup unless the exact verified file is
already present. After setup, the generated environment enables offline model
loading. Reactivate the same virtual environment before using it. Keep the
model and generated local config outside Git.

## GUI Configuration

The active profiles use:

- `room315_runtime_safe.gui.config` for Room 315.
- `mfja_light.gui.config` for the full floor and isolated robot.

`mfja_default.gui.config` and `gazebo_params.yaml` are installed but are not the
active high-level defaults. Pass `gui_config:=<path>` to test an alternative.
Relative launch paths are resolved inside the installed control package;
absolute paths can point to a local experimental config.

## After Any Configuration Change

Use this minimum sequence:

```bash
git diff --check

cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to \
  mfja_3rd_floor_bringup \
  --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
source "$MFJA_WS/install/setup.bash"
```

Then run the focused tests for the edited surface, inspect the installed launch
arguments or interface, and perform a headless or GUI smoke run. See
[Maintenance Guide](MAINTENANCE.md) for optimized package-specific commands.
