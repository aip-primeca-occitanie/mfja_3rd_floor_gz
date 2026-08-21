# Room 315 Classroom Runbook

This is the operator procedure for the simulated two-shuttle exercise. It does
not authorize real Staubli motion.

## Before learners arrive

Source the workspace and validate the PC:

```bash
export MFJA_WS=${MFJA_WS:-$HOME/mfja_ws}
source "$MFJA_WS/install/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
```

Give each workstation on the network a different `ROS_DOMAIN_ID`.

After changing HPP code or geometry, run a dry plan:

```bash
ros2 run mfja_staubli_manipulation_demos \
  room315_hpp_manipulation.sh \
  --direction shuttle-to-shuttle \
  --shuttle-pose -15.310 -5.536 0.839346 0 0 -0.002 \
  --destination-shuttle-pose -14.770 -5.536 0.839346 0 0 -0.0014
```

Both checks must finish successfully.

## Learner exercise

Use two terminals sourced from the same workspace.

Terminal 1:

```bash
ros2 run mfja_staubli_manipulation_demos room315_demo.sh
```

Wait until Room 315, the Staubli, and the pickup shuttle are visible, and the
arm has settled at `[0, 50, 70, 0, 55, 0]` degrees.

Terminal 2:

```bash
ros2 run mfja_staubli_manipulation_demos \
  room315_moving_shuttle_demo.sh
```

Expected sequence:

1. A destination shuttle and payload appear.
2. Rail switches and stoppers acknowledge the route.
3. The shuttle stops and HPP plans from its measured pose.
4. The arm approaches, closes, transfers, opens, and retreats.
5. Terminal 2 prints `two-shuttle manipulation demo complete`.

The payload attachment is kinematic in Gazebo. HPP remains the collision and
contact source of truth.

The classroom launch uses a 0.1 m/s shuttle and 30 Hz rail-sensor feedback.
DZI3R marks entry into the pickup zone; the coordinator then continues to the
canonical `[-15.240, -5.536, 0.839]` m support position before sending `OFF`.
This keeps the plan on the compact HPP IK branch. If HPP reports only distant
IK branches, reset the scene instead of executing from that stop.

## Stop and reset

Stop terminal 2 first with Ctrl-C; the coordinator sends `OFF` to any shuttle
it started. Then stop terminal 1.

Relaunch both terminals before the next learner. After any failure, preserve
the complete error output and reset rather than continuing from a partly moved
scene.

Only one Room 315 simulation may run per PC.

## Diagnostic topics

```bash
ros2 topic echo /staubli1/joint_states
ros2 topic echo /room_315/rails/right/shuttles/state
ros2 topic echo /room_315/rails/right/shuttles/pose_cmd
ros2 topic echo /room_315/rails/right/sensors/feedback
```

## Real-robot boundary

The Gazebo model's startup controller targets do not command hardware. The real
arm action and gripper IO are now known, and the simulation staging
configuration `[0, 50, 70, 0, 55, 0]` degrees passes the current HPP arm/cell
configuration validation with the corrected robot world pose. Real motion
remains blocked until the physical tool/workpiece model is commissioned and the
motion is authorized on site.

The ordered status and exact interfaces are maintained in the package
[README](../README.md#known-real-robot-interfaces).
