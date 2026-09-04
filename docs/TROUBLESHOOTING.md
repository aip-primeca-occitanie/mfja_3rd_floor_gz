# Troubleshooting

Use this guide to diagnose installation, build, Gazebo, rail, robot, visual
state, language, planning, safety, data, and test failures. Work from the first
relevant section and verify each boundary
before changing configuration.

## Collect a Baseline

In the terminal where the problem occurs:

```bash
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
which ros2
ros2 pkg prefix mfja_3rd_floor_bringup
ros2 pkg prefix mfja_robot_control_config
ros2 node list
ros2 topic list
ros2 service list
```

Confirm which overlay is sourced:

```bash
printenv AMENT_PREFIX_PATH | tr ':' '\n'
```

For a standard workspace, start every terminal with:

```bash
export MFJA_WS="$HOME/mfja_ws"
export MFJA_REPO="$MFJA_WS/src/mfja_3rd_floor_gz"
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
```

ROS logs are normally under `~/.ros/log`. Colcon build/test logs are under the
workspace `log/` directory.

## Installation and Build Problems

### `ros2: command not found`

Cause: ROS 2 is not installed or its base environment is not sourced.

```bash
test -f /opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

If the file is absent, follow [Installation](INSTALLATION.md) on Ubuntu 24.04.

### `Package 'mfja_...' not found`

Likely causes:

- the workspace has not been built;
- the current terminal did not source `install/setup.bash`;
- a different workspace overlay was sourced later;
- the build failed before installing the package.

Diagnose:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon list --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
```

Build, source, and verify:

```bash
colcon build --symlink-install --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
source "$MFJA_WS/install/setup.bash"
ros2 pkg prefix mfja_3rd_floor_bringup
```

### Duplicate package `mfja_robot_control_config`

Cause: colcon discovered a frozen/source copy inside an extracted dataset,
release archive, symlink, or nested workspace.

Locate all discovered copies:

```bash
cd "$MFJA_WS"
colcon list --base-paths src
find src -name package.xml -print
```

Keep extracted datasets and reproduction trees outside the colcon `src`
directory. If a user-owned archive must remain there, isolate its frozen source
tree with a `COLCON_IGNORE` file or build only the four explicit package paths.
Do not alter an immutable archive without recording that change.

### `rosdep check` reports Torch/TorchVision or another dependency

On Ubuntu 24.04, rosdep may resolve `python3-torch` and
`python3-torchvision` to apt package names that have no installable candidate.
They are optional for the base simulator. Install every other dependency while
explicitly skipping those two keys:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  "$MFJA_REPO/mfja_3rd_floor_bringup" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_robot_control_config" \
  --ignore-src --rosdistro jazzy -y \
  --skip-keys "python3-torch python3-torchvision"
```

Use the isolated CPU/GPU setup in
[Installation: Visual Training and V4 Inference](INSTALLATION.md#visual-training-and-v4-inference)
if the feature actually needs Torch. Run `rosdep check` with the same
`--skip-keys` option; a Python virtual environment does not make apt/rosdep
consider those system keys installed.

Do not confuse the optional Nix shell with dependency installation: it supplies
build tools but expects host ROS/Gazebo packages.

### A new script is missing from `ros2 run`

Check the installed executable list:

```bash
ros2 pkg executables mfja_robot_control_config | sort
```

If the source script should be public, confirm that it is executable and listed
under `install(PROGRAMS ...)` in
`mfja_robot_control_config/CMakeLists.txt`. Rebuild the control package and
source the overlay again. Some specialized scripts are intentionally
source-only; see [Maintenance Guide](MAINTENANCE.md#add-a-python-tool-or-node).

### A changed config appears to be ignored

Likely causes:

- the process was not restarted;
- the config is not included in the package CMake install list;
- the terminal is using an older overlay;
- the launch uses another profile/config path;
- a command-line launch override replaces the file value.

Check the package share and launch arguments:

```bash
ros2 pkg prefix --share mfja_robot_control_config
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
```

With a symlink install, installed data is often linked to source, but a rebuild
is still required to add a newly listed file or change CMake/package metadata.

### CMake still uses old dependency or test discovery

Reconfigure the affected package rather than editing build output:

```bash
cd "$MFJA_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-clean-cache --paths \
  "$MFJA_REPO/mfja_rail_interfaces" \
  "$MFJA_REPO/mfja_3rd_floor_description" \
  "$MFJA_REPO/mfja_robot_control_config" \
  "$MFJA_REPO/mfja_3rd_floor_bringup"
```

## Launch and Gazebo Problems

### A second Room 315 launch is rejected

The high-level Room 315/full-floor launch intentionally holds an exclusive host
lock. The fixed ROS rail topics and Gazebo services cannot safely be owned by
two floor runtimes.

Find likely owners:

```bash
ps -ef | grep -E 'room_315_only.launch.py|full_floor.launch.py' | grep -v grep
```

Return to the original launch terminal and stop it with `Ctrl-C`. Do not delete
the lock file or bypass the lock while a process owns it. A leftover pathname
after the owner exits is harmless because the operating-system lock is gone.

### A saved visual-obstacle pose disappears at launch

High-level Room 315/full-floor launches clear the disposable obstacle-pose
cache by default. The normal path is
`~/.ros/room315_visual_obstacles.json`. Preserve an existing cache with:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none \
  room315_clear_visual_obstacle_pose_cache:=false
```

If `room315_visual_obstacle_pose_file` is overridden, verify its exact value before
launch: the default clearing action unlinks the configured path and does not
validate whether it is the intended cache.

### Gazebo GUI does not open

Try a headless launch to separate display problems from server problems:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none gui:=false start_paused:=false
```

If headless works, inspect `DISPLAY`, Wayland/X11 permissions, GPU/driver logs,
and Gazebo client output. Remote systems normally need headless mode or a
working display-forwarding setup.

### Gazebo opens but nothing moves

Check simulation time:

```bash
ros2 topic echo --once /clock
```

The full-floor profile starts paused by default. Press Play in the GUI or launch
with `start_paused:=false`. If `/clock` is not advancing, timers using
`use_sim_time:=true` do not advance either.

Room 315 starts running by default, but initial shuttle counts are zero and
selected shuttles start disabled unless overridden.

### Topics/services are missing immediately after launch

Robot actions start at approximately 3 seconds, rail nodes at 4 seconds, and
optional perception-and-safety processes at 5 seconds. Wait at least five
seconds, then check:

```bash
ros2 node list
ros2 topic list | grep '^/room_315/'
ros2 service list | grep '^/world/'
```

Inspect the launch terminal for a process exit rather than repeatedly starting
another high-level launch.

### Gazebo reports a missing model or mesh URI

Confirm the source asset exists and the description package being used is the
one you built:

```bash
ros2 pkg prefix --share mfja_3rd_floor_description
printenv GZ_SIM_MODEL_PATH | tr ':' '\n'
printenv GZ_SIM_RESOURCE_PATH | tr ':' '\n'
```

The high-level launch sets the package model path for its processes. Absolute
author-specific mesh paths in SDF/URDF are not portable; use model/package
resources.

### `/world/<name>/set_pose`, `create`, or `remove` is missing

List actual services:

```bash
ros2 service list | grep -E '/world/.+/(set_pose|create|remove)$'
```

Expected names:

```text
/world/room_315_only/set_pose
/world/room_315_only/create
/world/room_315_only/remove
```

or:

```text
/world/mfja_3rd_floor/set_pose
/world/mfja_3rd_floor/create
/world/mfja_3rd_floor/remove
```

If services appear under `/world/default` or another name, the world file's
internal `<world name>` does not match the expected runtime name. For custom
worlds, match the filename stem, XML world name, and shuttle
`gazebo_world_name`.

### The full-floor simulation is much slower than wall time

Shuttle timers use simulation time. A heavy scene can run below real-time
factor 1, so motion looks slower in wall-clock time even when its simulated
speed is correct. Use the lighter Room 315 world, `robots:=none`, `gui:=false`,
or a lighter GUI config when appropriate.

## Rail and Shuttle Problems

### No shuttle is visible

The integrated launch defaults to zero initial shuttles. Either set a count:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none start_paused:=false \
  room315_right_shuttle_count:=1 \
  room315_shuttles_start_enabled:=false
```

or add one at runtime:

```bash
ros2 service call /room_315/rails/right/shuttles/add \
  mfja_rail_interfaces/srv/AddShuttle \
  "{name: 'room315_right_shuttle_1', start_slot: '2', speed: 0.2, start_enabled: false}"
```

### The add-shuttle service rejects a request

Check the returned message and current fleet:

```bash
ros2 topic echo /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Common causes are an invalid slot/name, a duplicate active entity, an occupied
start slot within the configured occupancy radius, or a count/identity mismatch.

### A shuttle is visible but does not move

Initial shuttles default to disabled. Publish `ON`:

```bash
ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'ON', speed: 0.2}"
```

Then inspect the newest state. `DISABLED` means drive is off; `WAITING` means
the drive may be enabled but motion is held by another condition.

### Shuttle mode is `WAITING`

Inspect stopper and shuttle state:

```bash
ros2 topic echo --once /room_315/rails/right/stoppers/state \
  mfja_rail_interfaces/msg/StopperState
ros2 topic echo /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Open a deliberately closed stopper only after confirming the route is safe:

```bash
ros2 topic pub --once /room_315/rails/right/stoppers/command \
  mfja_rail_interfaces/msg/StopperCommand \
  "{stoppers: [{name: 'A1', state: '0'}]}"
```

If no stopper is closed, another shuttle may be inside the configured
`shuttle_collision_distance_m`/headway boundary.

### Shuttle mode is `FALLING`

The directed graph had no valid successor for the actual switch configuration.
Correct the switch route, then reset the shuttle:

```bash
ros2 topic pub --once /room_315/rails/right/switches/command \
  mfja_rail_interfaces/msg/SwitchCommand \
  "{switches: [{name: 'ALL', state: 'EXTERIOR'}]}"

ros2 topic pub --once /room_315/rails/right/shuttles/command \
  mfja_rail_interfaces/msg/ShuttleCommand \
  "{name: 'room315_right_shuttle_1', command: 'RESET'}"
```

If a recently edited network causes repeat faults, run the offline kinematic
core and topology tests before relaunching.

### `OFF` was sent but stop acknowledgement is unclear

Wait for a newer `ShuttleState` with mode `DISABLED`. `WAITING` is not an
explicit drive-disable acknowledgement, and a nonzero `speed` field is only the
retained configured speed, not proof of current motion.

### A switch moves visually but routing does not change

Publish to the typed rail command topic, not directly to the visual controller:

```text
/room_315/rails/right/switches/command
```

Verify the delayed actual state:

```bash
ros2 topic echo /room_315/rails/right/switches/state \
  mfja_rail_interfaces/msg/SwitchState
```

Routing uses actual state after the configured delay.

The typed switch topic is a low-level simulation interface that bypasses the
supervised route-planning boundary. Change `A1`/`A2` or `A3`/`A4` as coordinated pairs
only on an empty rail. Normal numbered start slots are exterior guard segments,
so resetting there is not safe preparation for selecting an interior route. A
fixed wall-time delay is not evidence that a moving shuttle has cleared the
guard; use a validated route-compatible scenario for motion.

### A switch does not move visually

Confirm the typed state changes, the visual controller node exists, and the
visual bridge topics exist:

```bash
ros2 node list | grep conveyor
ros2 topic echo --once /mfja/conveyor/switch_states std_msgs/msg/String
```

If typed state is correct but the entity does not move, check that the world
still uses the expected switch entity names and that controller selectors/yaw
maps were updated with any rename.

### Sensor always reads clear or always occupied

Inspect the actual device YAML loaded by the node and the shuttle state. Verify:

- segment name;
- `s_ratio` in `[0, 1]`;
- `radius_m` is reasonable;
- linked stopper name/point;
- right versus left public segment naming;
- rail calibration after geometry changes.

Use the device-position tool in dry-run mode and turn device markers on:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none start_paused:=false \
  room315_show_device_markers:=true
```

The binary result is `active`. Segment/arc-length fields in the reading are
privileged simulator metadata, not a second continuous-sensor measurement.

### Device markers are missing

Check `room315_show_device_markers:=true`, the world create service, and launch
logs. Markers are spawned gradually, so allow startup time. If a marker file was
edited, rebuild/source/relaunch. Visual marker color refresh is intentionally
guarded to avoid create/remove races.

### A runtime-spawned shuttle is invisible or cannot be removed

Verify the corresponding Gazebo services and exact entity name:

```bash
ros2 service list | grep -E '/world/.+/(create|remove)$'
ros2 topic echo /room_315/rails/right/shuttles/state \
  mfja_rail_interfaces/msg/ShuttleState
```

Check launch logs for spawn/delete response messages and model URI failures.

## Robot and Gripper Problems

### A requested robot does not spawn

Check the selected config and accepted selectors:

```bash
sed -n '1,220p' \
  "$MFJA_REPO/mfja_robot_control_config/config/robots_room_315_only.yaml"
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py --show-args
```

Use an exact name when a shortcut is ambiguous. Every YAML robot name must be
unique. Confirm the model SDF and matching URDF both exist for supported robot
spawning.

### Robot topics are missing

Allow the three-second spawn delay, then check:

```bash
ros2 topic list | grep '^/kuka1/'
ros2 topic info /kuka1/joint_trajectory
```

Inspect the launch output for a generated bridge configuration error, robot
spawn failure, or `robot_state_publisher` exit.

### Joint command helper rejects positions

List the authoritative profile/order:

```bash
ros2 run mfja_robot_control_config robot_joint_command.py --list
ros2 run mfja_robot_control_config robot_joint_command.py \
  kuka --unit rad --positions 0 0 0 0 0 0 --dry-run
```

Provide the exact number of values. Angular values are radians unless
`--unit deg` is passed; TIAGo torso lift remains meters.

### Gripper does not move

Check the configured range and dry-run target:

```bash
ros2 run mfja_robot_control_config robot_gripper_command.py --list
ros2 run mfja_robot_control_config robot_gripper_command.py \
  kuka open 100 --dry-run
ros2 topic info /kuka1/gripper/position_command
ros2 topic echo --once /kuka1/joint_states sensor_msgs/msg/JointState
```

Restart the simulation after changing gripper endpoints; the launch applies
ranges while generating the process-specific robot description. Verify that a
renamed industrial robot has a matching config entry. Remember that movement is
visual articulation, not physical grasp/attachment.

## Visual Runtime, Planning, and Safety Problems

### RGB-D camera bridge/rail-safety supervisor topics are absent

The high-level default is disabled. Launch with:

```bash
ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \
  robots:=none start_paused:=false \
  enable_room315_rail_safety_supervisor:=true \
  enable_room315_rgbd_camera_bridge:=true
```

After five seconds:

```bash
ros2 topic list | grep -E '^/room_315/(perception|rail_safety)/'
ros2 topic echo --once /room_315/perception/right_rail_rgbd/image \
  sensor_msgs/msg/Image
```

### Visual inference launch rejects `checkpoint_path` or `sidecar_directory`

Those are superseded launch arguments. The current V4-only launch accepts
`runtime_config`, `runtime_mode`, `v4_promotion_manifest`,
`v4_promotion_manifest_sha256`, `device`, and state-fusion/update switches.

```bash
ros2 launch mfja_robot_control_config \
  room_315_visual_state_runtime.launch.py --show-args
```

Use [Visual-State Runtime Integration](room315_visual_runtime_integration.md)
and a complete V4 promotion bundle.

### Visual inference cannot find `/home/tiago/...`

The checked-in runtime YAML records the qualified host and is not a portable
artifact installation. Do not create an empty file or change a hash to make the
error disappear.

Obtain the complete approved candidate, verify its checksums, then supply a
host-local runtime config or launch overrides for:

- promotion-manifest path;
- expected promotion-manifest SHA-256;
- device/runtime mode as appropriate.

The manifest must reference all required model/contract artifacts.

### Visual observations are rejected or never become ready

Inspect:

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
ros2 topic echo /room_315/visual_state/validation \
  mfja_rail_interfaces/msg/VisualStateObservation
ros2 topic echo /room_315/visual_state/observed_state \
  mfja_rail_interfaces/msg/VisualStateObservation
```

Check camera freshness, both rail presence streams, checkpoint/contract hashes,
runtime mode, preprocessing compatibility, validation reasons, and stale-frame
counters. Do not route a rejected raw observation to planning.

### Task execution stays disabled

This is the safe default. Enabling the launch argument alone is insufficient:
the runtime re-hashes the configured task-execution authorization and promotion
manifest. A fresh clone does not include those host-local artifacts.

Follow the qualification/promotion/authorization workflow. Do not bypass the
checks or reuse an authorization for a different candidate.

### PlanSys2 planner is not active

Check lifecycle and services:

```bash
ros2 lifecycle get /planner
ros2 service list | grep '/planner/get_plan'
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
```

Verify PlanSys2/POPF dependencies, planner logs, the runtime domain path, and
that only one task-execution runtime owns the fixed node/topic names.

### Local English intent model setup uses the wrong home directory

Pass a portable directory explicitly because older/default project paths may
refer to the qualification host. Use a dedicated environment so the setup does
not install into the user/system Python package environment:

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

Initial setup needs network access, 1.04 GiB (1.12 GB) for the GGUF checkpoint,
and space for `llama-cpp-python` installation/build caches. Reactivate the same
virtual environment before using the semantic runtime. If the checkpoint hash
does not match, do not run it; remove/quarantine the bad download through your
normal data-management process and repeat verified setup.

## Dataset and Experiment Problems

### Recorder creates no episode or frames

Check that the recorder is enabled, camera images arrive, and the episode
control/goal behavior matches the launch:

```bash
ros2 topic echo /room_315/visual_dataset/status std_msgs/msg/String
ros2 topic hz /room_315/perception/right_rail_rgbd/image
```

Verify write permission and free space for the configured
`room315_visual_dataset_dir`. Keep the directory outside Git.

### Event extractor produces fewer events than raw frames

This is expected. `data.jsonl` is a framewise replay/debug stream;
`events.jsonl` is the event-level training surface. For PDDL-generated episodes,
the extractor also excludes failed or unapproved episodes by default. Inspect
each episode's validation record before using debug override flags.

### A hash/checksum verification fails

Stop. Identify whether the source file, transfer, extraction, path, or manifest
is wrong. Do not update an expected hash just because the current file differs.
Qualified runtime and published evidence derive meaning from exact byte
identity. Obtain the correct artifact or create a new version through the
documented workflow.

## Test Problems

### Direct pytest reports no tests collected

Check dependency/import output. Some broad control-suite collection paths can
terminate early when Torch is unavailable. Install declared dependencies, run a
specific dependency-independent test, and compare with registered colcon tests:

```bash
python3 -m pytest -q \
  mfja_robot_control_config/test/test_room315_rail_devices.py

cd "$MFJA_WS"
colcon test --packages-select mfja_robot_control_config
colcon test-result --verbose
```

Do not interpret exit code 5/no collection as a pass.

### `colcon test` misses a source test

Only tests registered with `ament_add_pytest_test` run under colcon. Search
`mfja_robot_control_config/CMakeLists.txt` for the test name. Run an unregistered
test directly and decide whether it should be added to the CMake test surface.

### ROS integration tests fail while unit tests pass

Check for an already running Room 315 launch, fixed namespace collisions, stale
overlay, inactive `/clock`, and missing optional dependencies/artifacts. The
singleton lock and fixed topic names intentionally prevent overlapping floor
runtimes.

## Safe Shutdown

1. Publish an `OFF` or supervisor `stop_all` when a procedure requires a
   controlled simulated stop.
2. Stop interactive helper processes.
3. Stop the high-level ROS launch with `Ctrl-C` and wait for shutdown messages.
4. Confirm no launch process remains before starting another floor runtime.
5. Preserve logs/evidence for unresolved faults.

If the problem remains after these checks, report the exact command, source
revision, environment versions, launch arguments, relevant log excerpt,
interface/config hashes, and whether the issue reproduces in the lightweight
headless Room 315 profile.
