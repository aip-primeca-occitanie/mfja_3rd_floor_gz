# Maintenance Guide

This guide is for future maintainers who need to update, test, extend, or hand
over the MFJA simulation without losing the contracts that connect Gazebo, ROS
2, rail control, visual perception, planning, and evidence artifacts.

## Maintainer Responsibilities

A complete change normally preserves all of the following:

- the source/install separation of a colcon workspace;
- public ROS topic, service, message, and launch contracts;
- valid Gazebo resource paths and world/entity names;
- directed rail geometry, device, calibration, and public-name consistency;
- explicit learned-input versus privileged/safety-data boundaries;
- fail-closed visual-model artifact and task-execution authorization checks;
- dataset split and evidence immutability;
- third-party asset provenance and redistribution terms;
- English operational documentation that matches the installed code.

Read [System Architecture](SYSTEM_ARCHITECTURE.md) and
[Configuration and Customization](CONFIGURATION.md) before making a structural
change.

## Before Editing

1. Confirm the repository and branch:

   ```bash
   pwd
   git status --short --branch
   ```

2. Treat every existing modified, deleted, or untracked file as user-owned
   unless the current task explicitly includes it.
3. Identify the source of truth and all consumers using `rg`.
4. Decide whether the change is documentation-only, package-local, interface
   affecting, or a versioned research/runtime contract change.
5. Record the pre-change command or test that demonstrates the issue when the
   change is a bug fix.
6. Keep external datasets/checkpoints out of the repository and colcon source
   tree.

Do not edit `build/`, `install/`, `log/`, `__pycache__`, `.pytest_cache`, or
generated `/tmp` files. Do not repair a source problem by modifying an installed
copy.

## Repository Ownership

| Area | Owning package/path | Notes |
| --- | --- | --- |
| Public launch entry points | `mfja_3rd_floor_bringup/launch` | Wrappers plus shared Room 315/full-floor policy |
| Gazebo worlds/models and URDF | `mfja_3rd_floor_description` | Includes C++ gripper plugin and attribution |
| Rail and visual messages/services | `mfja_rail_interfaces` | Build-generated bindings depend on these files |
| Runtime nodes and tools | `mfja_robot_control_config/scripts` | Executable installation is an explicit CMake list |
| Lower-level launch | `mfja_robot_control_config/launch` | Directory is installed automatically |
| Installed runtime config | `mfja_robot_control_config/config` | Most files are listed explicitly in CMake; PDDL directory is installed as a directory |
| Operational docs | `README.md`, `docs/` | Documentation-only changes need no ROS rebuild |
| Detailed rail config docs | `config/room_315_kinematics/*.md` | Installed README is part of the package reference |
| Research report/evidence | `report/` | Many files are generated or checksum-bound; do not casually rewrite |

## Standard Change Cycle

### 1. Make the Smallest Source Change

Use existing helpers, schemas, assets, and conventions. Avoid copying runtime
logic into documentation or duplicating a source-of-truth value in another
config file unless the contract explicitly requires it.

### 2. Inspect the Diff

```bash
git diff --check
git diff --stat
git diff -- README.md docs mfja_3rd_floor_bringup \
  mfja_3rd_floor_description mfja_rail_interfaces \
  mfja_robot_control_config
```

Limit the final diff to intended files and preserve unrelated worktree changes.

### 3. Run Focused Source Tests

Run the narrowest test that covers the change first:

```bash
python3 -m pytest -q \
  mfja_robot_control_config/test/test_room315_rail_devices.py
```

The direct source-tree suite and the ament/colcon suite are not identical; see
[Testing Strategy](#testing-strategy).

### 4. Build the Affected Packages

Build from the workspace root, never from inside one package:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
```

Use one of the build recipes below.

### 5. Source the New Overlay

```bash
source "$MFJA_WS/install/setup.bash"
```

Sourcing another workspace or an older terminal is a common cause of testing
stale code.

### 6. Verify the Installed Surface

Depending on the change:

```bash
ros2 pkg prefix mfja_robot_control_config
ros2 pkg executables mfja_robot_control_config
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
ros2 interface show mfja_rail_interfaces/msg/ShuttleCommand
```

### 7. Run Package Tests and a Runtime Smoke

Use the test matrix and smoke checklist below. A source test alone does not prove
that CMake installed the changed script/config/model.

### 8. Update Documentation and Handover Notes

Update the operational guide, configuration reference, and documentation index
when a public behavior, file, topic, argument, or workflow changes.

## Build Recipes

All recipes use explicit package paths to avoid accidental discovery of a
frozen package copy inside an extracted dataset.

### Full Development Build

```bash
colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
```

### Control or Config Change

Use this for control-package Python, YAML/JSON/PDDL, or lower-level launch files:

```bash
colcon build --symlink-install \
  --paths "$MFJA_REPO/mfja_robot_control_config" \
  --packages-select mfja_robot_control_config
```

If the change is consumed by a public bringup wrapper, also build bringup:

```bash
colcon build --symlink-install \
  --paths \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  --packages-select mfja_robot_control_config mfja_3rd_floor_bringup
```

### Description, World, Model, URDF, or Plugin Change

```bash
colcon build --symlink-install \
  --paths "$MFJA_REPO/mfja_3rd_floor_description" \
  --packages-select mfja_3rd_floor_description
```

Rebuild consumers as well when a package-facing path/library contract changes.

### Interface Change

An interface change must rebuild the interface package and its consumers:

```bash
colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  --packages-up-to mfja_3rd_floor_bringup
```

### CMake or Dependency Metadata Change

Force package reconfiguration when cached CMake discovery could hide a change:

```bash
colcon build --symlink-install --cmake-clean-cache --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
```

### Documentation-Only Change

No colcon build is required. Run the Markdown, English-language, and diff checks
under [Documentation Maintenance](#documentation-maintenance).

## Testing Strategy

### Dependency Check

Install dependencies before interpreting a skipped/empty test run:

```bash
rosdep check --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy \
  --skip-keys "python3-torch python3-torchvision"
```

Torch and TorchVision are declared runtime dependencies, but the supported
Ubuntu 24.04 apt resolver may have no installable candidates for their rosdep
keys. The base dependency check therefore skips them explicitly; use the
isolated environment in [Installation](INSTALLATION.md#visual-training-and-v4-inference)
when they are needed. Some test modules use module-level optional dependency
checks; without Torch, a broad direct pytest collection may finish with "no
tests collected" rather than prove the rest of the directory passed.

### Focused Source Tests

Direct pytest sees every `test_*.py` in the requested path, including tests not
registered in CMake:

```bash
python3 -m pytest -q \
  mfja_3rd_floor_description/test/test_room315_visual_observation_assets.py

python3 -m pytest -q \
  mfja_robot_control_config/test/test_room315_kinematic_shuttle_core.py
```

Use `-k <expression>` or an exact test path during iteration.

### Package/Ament Tests

After building and sourcing:

```bash
cd "$MFJA_WS"
colcon test --packages-select \
  mfja_3rd_floor_description \
  mfja_robot_control_config
colcon test-result --verbose
```

`colcon test` runs only tests registered by the package CMake files. It is the
best installed-package test, but it is not a substitute for the broader direct
pytest collection.

At the time of this documentation update, the control package contains 97
`test_*.py` files. Ament registers 83 without Torch and 88 when Torch is
available at CMake configure time. The source-only set includes
coverage of floor/runtime singleton behavior, scenario validation, rail names
and topology, runtime contracts, switch authority, task-execution singleton
behavior, and V4 coverage tools. Future maintainers should either register an
appropriate test or document why it must remain source-only.

### Optional/Long Tests

The control suite includes Torch-dependent, ROS integration, artifact-dependent,
and long-timeout tests. Some registered cases have timeouts up to 600 seconds.
Do not label a missing external model/dataset as a code regression, and do not
claim a full suite pass when those prerequisites were absent. Record:

- command used;
- package/source revision;
- installed optional dependencies;
- external artifact identifiers/hashes;
- passed, failed, skipped, and uncollected counts.

### Runtime Smoke Test

Start a lightweight headless system:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  gui:=false \
  start_paused:=false \
  room315_clear_visual_obstacle_pose_cache:=false \
  room315_right_shuttle_count:=1 \
  room315_left_shuttle_count:=0 \
  room315_shuttles_start_enabled:=false
```

In another sourced terminal, after five seconds:

```bash
ros2 topic echo --once /clock
ros2 service list | grep '^/world/room_315_only/'
ros2 topic echo --once /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
ros2 topic echo --once /room_315/rails/right/sensors/feedback \
  mfja_rail_interfaces/msg/SensorFeedback
```

Stop with `Ctrl-C` and confirm no second Room 315 launch remains before the next
test.

## Change-to-Test Matrix

The listed tests are starting points; use `rg` to find additional consumers.

| Change | Focused validation |
| --- | --- |
| Robot list, selectors, or launch | `test_room315_full_package_static.py`, `test_room315_multi_shuttle_launch_args.py`, `robot_joint_command.py --list`, `--show-args`, headless spawn |
| Robot model/URDF | Description collision tests, `test_industrial_gripper_articulation.py`, `test_kuka_initial_joint_positions.py`, robot topic smoke |
| Gripper config/plugin/helper | `test_gripper_range_launch.py`, `test_robot_gripper_bridge.py`, `test_robot_gripper_command.py`, description articulation test |
| World or physics | `test_lightweight_world_physics.py`, camera/model/collision tests, room-only and full-floor launch |
| Camera model/topic/calibration | `test_room315_visual_observation_assets.py`, camera bbox/calibration/visual runtime tests, live image and camera-info topics |
| Raw rail CSV | segment normalization/names, route topology, kinematic core, rail devices, collision tests |
| Network/routing | route topology, PDDL planning, multi-shuttle planning, kinematic core, switch feedback authority |
| Device YAML | `test_room315_rail_devices.py`, marker lifecycle, kinematic core, live sensor/stopper checks |
| Calibration | kinematic core, devices, collision, payload/identity visual tests, full GUI inspection |
| Shuttle identity/payload | identity config/assets/tracker, keepout zones, payload models/visuals/occlusion, eight-shuttle visual pipeline |
| Message/service | dependent package build, `ros2 interface show`, contracts tests, live topic/service type checks |
| Public launch argument | launch-argument/static/singleton tests, `--show-args`, headless launch |
| rail-safety supervisor/safety | rail-safety supervisor, fleet safety, block reservation, headway, model-input boundary tests |
| Task goal/PDDL/executive | task-goal builder/semantic/validation, PDDL generator/translator/planner, task execution/authorization, closed-loop tests |
| V4 visual runtime/artifact | contract/model/calibration/acceptance/runtime/shadow/fault/promotion tests, CPU smoke, hash verification |
| Dataset pipeline | recorder/extractor/split/audit/leakage/role-specific tests plus output manifest verification |
| Documentation | link checker, English scan, `git diff --check`; no ROS build |

## Extension Recipes

### Add a Python Tool or Node

1. Add the script under `mfja_robot_control_config/scripts` with a Python 3
   shebang and executable bit.
2. Give command-line tools useful `--help`, validation, nonzero failure codes,
   and a safe dry-run when they can mutate data or runtime state.
3. Add it to the explicit `install(PROGRAMS ...)` list in
   `mfja_robot_control_config/CMakeLists.txt` if users should call it through
   `ros2 run`.
4. Declare all ROS/system dependencies in `package.xml`.
5. Add a test and register it with `ament_add_pytest_test` when it belongs in
   `colcon test`.
6. Rebuild, source, and verify it appears in:

   ```bash
   ros2 pkg executables mfja_robot_control_config
   ```

7. Document its inputs, outputs, side effects, and output directory.

Four specialized source scripts currently are not installed through `ros2 run`:

- `room_315_experiment_a_smoke_v2_package.py`
- `room_315_visual_final_test_v4.py`
- `room_315_visual_v4_final_test_coverage_compat.py`
- `room_315_visual_v4_final_test_coverage_extension.py`

Run them from the source tree only when their historical/current workflow calls
for it, or deliberately add and test an install rule before advertising
`ros2 run`.

### Add a Launch File or Argument

Launch directories are installed as directories, so a new `.launch.py` is
included after rebuild. A new argument still needs:

- a clear default, choices, units, and description;
- explicit forwarding through every wrapper/include layer;
- correct string/boolean/numeric parameter typing;
- tests for default and override behavior;
- `--show-args` verification;
- an operational documentation update.

Do not expose a lower-level parameter with a high-level name unless its mapping
is explicit.

### Add an Installed Configuration File

Most control-package config files are installed through explicit CMake `FILES`
lists. Adding a YAML/JSON/config file to the source directory does not guarantee
it will exist in the package share.

1. Add the source file.
2. Add it to the appropriate CMake install block, or deliberately install its
   containing directory.
3. Add schema/static tests.
4. Rebuild and verify the installed path:

   ```bash
   ros2 pkg prefix --share mfja_robot_control_config
   ```

5. Ensure no author-specific path is presented as a portable default.

### Add or Change a ROS Interface

Follow [Configuration: ROS Interfaces](CONFIGURATION.md#ros-interfaces). Treat
field removal/renaming/type changes as compatibility changes. Update JSON
serializers and dataset schemas separately where they mirror the ROS message.

### Add a Gazebo Model or Asset

Follow [Configuration: Worlds and Reusable Models](CONFIGURATION.md#worlds-and-reusable-models).
Check mesh scale, pose, collision cost, URI resolution, and upstream terms.
Binary CAD/mesh modifications must retain provenance; do not overwrite an
upstream original when a local derived asset can be added separately.

### Change Rail Geometry or Devices

Follow the detailed kinematic README and CSV migration guide. Use explicit
source paths with write-capable tools, inspect the diff, and validate both
topological correctness and Gazebo alignment. A visually plausible route can
still be topologically invalid.

### Change PDDL or Task Semantics

Update the complete executable surface:

```text
TaskGoal schema and validation
  -> problem builder
  -> expert/runtime PDDL domains
  -> plan translator
  -> validation gate
  -> closed-loop executive
  -> supervisor command contract
  -> effect verification and tests
```

Unsupported compound goals must continue to be clarified or rejected, not
silently approximated.

### Change a V4 Runtime Artifact or Policy

Use a new candidate directory and immutable manifests. The safe sequence is:

1. build and hash the candidate;
2. run validation and CPU smoke;
3. run shadow comparison;
4. run acceptance and fault campaigns;
5. record the promotion decision;
6. create a separate execution authorization;
7. deploy with exact expected hashes;
8. retain the previous promoted bundle for rollback.

Never mutate the promoted checkpoint, sidecars, manifest, or authorization
file in place. Never bypass a failed hash or set `execution_enabled: true`
without the qualified external files.

## Source, Generated, External, and Immutable Files

| Class | Examples | Maintenance rule |
| --- | --- | --- |
| Source | package code, launch, YAML, SDF, URDF, Markdown | Edit, review, build, and test normally |
| Build output | `build/`, `install/`, `log/`, Python caches | Regenerate; never hand-edit or commit |
| Runtime temporary | `/tmp/*_bridge.yaml`, patched SDF, filtered world | Process-owned; inspect for diagnosis only |
| User data | `~/.ros/...datasets`, benchmark outputs | Keep outside Git; back up according to experiment needs |
| External model/runtime | GGUF, `.pt`, `.safetensors`, promotion bundle | Keep outside Git; verify exact hash and provenance |
| Immutable research | frozen splits, final-test inputs, evidence manifests/checksums | Create a new version; do not rewrite in place |
| Vendor/upstream asset | imported CAD, meshes, URDF derivatives | Preserve original terms and attribution |

If an external extracted dataset contains a frozen ROS package, do not put it
under the colcon `src` tree. Colcon follows enough of that tree to detect a
duplicate package even when the copy is only reproduction material.

## Documentation Maintenance

Operational documentation must be English and should use portable paths such
as `$HOME`, `$MFJA_WS`, and `$MFJA_REPO`. A historical audit may record an
original absolute path as provenance, but it must be labelled historical in the
documentation hub.

For every documentation change:

1. Confirm commands against source or the installed `--help`/`--show-args`.
2. Use relative links for repository documents.
3. Label values as node defaults, high-level launch defaults, or examples.
4. State external prerequisites and side effects before commands that download,
   install, write, actuate, or enable execution.
5. Keep one canonical detailed explanation and link to it rather than copying a
   large command block into many files.
6. Run:

   ```bash
   git diff --check
   git grep -nIP '[\x{0600}-\x{06ff}]' -- \
     README.md docs \
     mfja_robot_control_config/config/room_315_kinematics/README.md \
     mfja_robot_control_config/config/room_315_visual_state/README.md
   ```

   `git grep` deliberately checks maintained tracked documentation, including
   tracked HTML even when the global ignore rules contain `*.html`.

7. Run a Markdown link checker over the edited files and record the command and
   result. The repository does not currently ship a dedicated link-check tool,
   so at minimum resolve every edited relative target from its containing file.

Do not regenerate or overwrite standalone HTML/report output unless the change
explicitly includes that artifact and its source/generation procedure.

## Dependency Maintenance

- Runtime dependencies belong in the owning `package.xml`.
- CMake compile/link dependencies belong in the owning `CMakeLists.txt`.
- Python imports used only by optional research tools must fail with a clear
  prerequisite message or be documented as optional; do not silently remove
  checks.
- Keep the Ubuntu/ROS/Gazebo version contract synchronized among package code,
  `flake.nix`, installation docs, and CI if CI is added later.
- The Nix shell is hybrid: it must not be documented as providing ROS or
  Gazebo.
- Run `rosdep check` after dependency metadata changes.

## Release and Handover Checklist

Before handing the repository to another maintainer or creating a release:

- [ ] The working tree contains only intentional changes.
- [ ] `git diff --check` passes.
- [ ] Every new executable/config/test is installed or explicitly documented as
      source-only.
- [ ] `rosdep check` passes on the supported environment, or missing optional
      dependencies are recorded.
- [ ] Focused tests pass.
- [ ] Registered package tests pass, with results captured.
- [ ] Broader source tests were run or their exclusions are recorded.
- [ ] Room 315 headless smoke passes after a fresh build/source.
- [ ] Full-floor or isolated-robot smoke was run when affected.
- [ ] Public launch arguments and interfaces match documentation.
- [ ] External artifact paths are portable or clearly host-local templates.
- [ ] Checkpoint, manifest, authorization, dataset, and evidence hashes are
      preserved and recorded.
- [ ] No build output, cache, checkpoint, local dataset, or virtual environment
      is staged.
- [ ] Third-party attribution is updated for new imported assets.
- [ ] The documentation hub links and classifies every new document.
- [ ] Known limitations and unverified tests are stated without overstating
      readiness.

## Recovery and Rollback

- Stop a running launch with `Ctrl-C` and allow ROS/Gazebo children to exit.
- Re-source the intended overlay before deciding that rollback failed.
- For configuration/code, use version control to apply a reviewed revert; do
  not modify generated install files.
- For V4 runtime deployment, point the host-local config back to the previous
  intact promoted bundle and verify its expected hash. Do not edit either
  bundle.
- A leftover Room 315 lock-file pathname is not itself a problem; the live
  operating-system lock is what prevents a second launch. Find and stop the
  owning process rather than deleting the file under it.
- Preserve failed logs and result manifests until the cause is understood.

Use [Troubleshooting](TROUBLESHOOTING.md) for symptom-specific diagnostic
commands.
