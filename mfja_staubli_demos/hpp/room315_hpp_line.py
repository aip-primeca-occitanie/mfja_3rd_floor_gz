#!/usr/bin/python3
"""Plan a straight Cartesian tool0 line with HPP and send it to Gazebo.
"""

import argparse
import time

import numpy as np
import pinocchio as pin
import rclpy
from hpp_exec import configs_to_joint_trajectory, read_current_configuration
from pinocchio import StdVec_Bool as Mask
from pyhpp.constraints import (
    ComparisonType,
    ComparisonTypes,
    Implicit,
    Orientation,
    Position,
)
from pyhpp.core import ConfigProjector, ConstraintSet, Dichotomy, Problem, Straight
from pyhpp.pinocchio import Device, urdf
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
FRAME = "staubli/tool0"
# Start configuration with tool0 inside the cell enclosure at 1/4 of the
# robot height (0.32 m), 0.5 m in front of the base, so the default line
# goes straight up to 3/4 of the robot height (0.97 m) inside the glass.
DEFAULT_Q_START = np.array(
    [-1.56136443, 0.47307870, 2.04964315, -0.00130315, -0.32991444, 0.00524110]
)
DEFAULT_LINE = np.array([0.0, 0.0, 0.6475])
ROOM315_ROBOT_POSE = (-15.1622, -6.0, 1.0, 0.0, 0.0, 1.57)

ROBOT_URDF = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
ROBOT_SRDF = "package://mfja_staubli_demos/hpp/staubli_tx2_60l.srdf"
CELL_URDF = "package://mfja_staubli_demos/hpp/room315_cell.urdf"
CELL_SRDF = "package://mfja_staubli_demos/hpp/room315_cell.srdf"

START_HOLD = 1.0
JOINT_STATE_TIMEOUT = 10.0
SUBSCRIBER_TIMEOUT = 5.0


def build_problem():
    robot = Device("staubli")
    urdf.loadModel(
        robot, 0, "staubli", "anchor", ROBOT_URDF, ROBOT_SRDF, pin.SE3.Identity()
    )

    x, y, z, roll, pitch, yaw = ROOM315_ROBOT_POSE
    robot_world = pin.SE3(
        pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z])
    )
    # Cell link origins are world poses; placing the cell at the inverse of
    # the robot world pose expresses everything in the robot base frame.
    urdf.loadModel(
        robot, 0, "room315", "anchor", CELL_URDF, CELL_SRDF, robot_world.inverse()
    )

    problem = Problem(robot)
    problem.addConfigValidation("CollisionValidation")
    problem.addConfigValidation("JointBoundValidation")
    problem.steeringMethod(Straight(problem))
    problem.pathValidation(Dichotomy(robot, 0.0))
    return robot, problem


def sample_path(path, samples):
    length = path.length()
    configs = []
    for i in range(samples):
        q, ok = path(i / (samples - 1) * length)
        if not ok:
            raise RuntimeError(f"path evaluation failed at sample {i}")
        configs.append(np.asarray(q).flatten())
    return configs


def publish_trajectory(node, topic, trajectory):
    publisher = node.create_publisher(JointTrajectory, topic, 10)
    deadline = time.monotonic() + SUBSCRIBER_TIMEOUT
    while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print(f"warning: no subscriber detected on {topic}")
    # Publish exactly once: the Gazebo controller restarts the trajectory on
    # every received message.
    publisher.publish(trajectory)
    rclpy.spin_once(node, timeout_sec=0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", default="staubli1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=80)
    parser.add_argument(
        "--line",
        nargs=3,
        metavar=("DX", "DY", "DZ"),
        default=DEFAULT_LINE,
        type=float,
        help="Cartesian line in the Staubli base frame, meters.",
    )
    parser.add_argument(
        "--q-start",
        nargs=6,
        metavar=tuple(JOINT_NAMES),
        default=DEFAULT_Q_START,
        type=float,
        help="Start configuration for --plan-only (live runs start from the "
        "current robot configuration).",
    )
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument(
        "--goto-start",
        action="store_true",
        help="Move smoothly to the --q-start configuration (collision-checked "
        "joint motion) instead of planning a line. Use once after the "
        "simulation starts: lines are not reachable from the upright spawn "
        "pose.",
    )
    args = parser.parse_args()
    if args.goto_start and args.plan_only:
        parser.error("--goto-start needs the live robot")

    robot, problem = build_problem()
    model = robot.model()
    frame_id = model.getFrameId(FRAME)
    frame = model.frames[frame_id]
    joint_id = frame.parentJoint
    tool_in_joint = frame.placement
    data = model.createData()
    line = np.array(args.line)

    node = None
    if args.plan_only:
        q_start = np.array(args.q_start)
    else:
        rclpy.init()
        node = Node("room315_hpp_line")
        q_start = read_current_configuration(
            node,
            JOINT_NAMES,
            f"/{args.robot_name}/joint_states",
            timeout_sec=JOINT_STATE_TIMEOUT,
            strip_prefix=True,
            require_single_publisher=True,
        )
        if q_start is None:
            raise RuntimeError(
                f"could not read /{args.robot_name}/joint_states; "
                "is the Room 315 simulation running?"
            )

    valid, report = problem.isConfigValid(q_start)

    if args.goto_start:
        q_target = np.array(args.q_start)
        if valid:
            success, path, report = problem.directPath(q_start, q_target, True)
            if not success:
                raise RuntimeError(
                    f"motion to the start configuration is blocked: {report}"
                )
        else:
            print(f"warning: recovering from invalid configuration ({report})")
            _, path, _ = problem.directPath(q_start, q_target, False)
        configs = sample_path(path, 25)
        duration = max(3.0, float(np.max(np.abs(q_target - q_start))) / 0.3)
        print(f"moving to the start configuration ({duration:.1f} s)")
    else:
        if not valid:
            raise RuntimeError(f"start configuration is invalid: {report}")
        pin.forwardKinematics(model, data, q_start)
        pin.updateFramePlacements(model, data)
        start_pose = data.oMf[frame_id].copy()
        goal_pose = pin.SE3(start_pose.rotation, start_pose.translation + line)

        xyz = Mask()
        xyz[:] = (True, True, True)
        line_xy = Mask()
        line_xy[:] = (True, True, False)
        active_xy = Mask()
        active_xy[:] = (True, True)
        equal3 = ComparisonTypes()
        equal3[:] = (ComparisonType.EqualToZero,) * 3
        equal2 = ComparisonTypes()
        equal2[:] = (ComparisonType.EqualToZero,) * 2

        goal_projector = ConfigProjector(robot, "goal_projector", 1e-4, 200)
        goal_projector.add(
            Implicit(
                Position(
                    "tool0_goal_position",
                    robot,
                    joint_id,
                    tool_in_joint,
                    goal_pose,
                    xyz,
                ),
                equal3,
                xyz,
            ),
            0,
        )
        goal_projector.add(
            Implicit(
                Orientation(
                    "tool0_goal_orientation",
                    robot,
                    joint_id,
                    pin.SE3(tool_in_joint.rotation, np.zeros(3)),
                    pin.SE3(goal_pose.rotation, np.zeros(3)),
                    xyz,
                ),
                equal3,
                xyz,
            ),
            0,
        )
        goal_constraints = ConstraintSet(robot, "goal")
        goal_constraints.addConstraint(goal_projector)
        problem.setConstraints(goal_constraints)
        success, q_goal, residual = problem.applyConstraints(q_start)
        if not success:
            raise RuntimeError(
                f"line end {np.round(goal_pose.translation, 3)} is not reachable "
                f"from the current configuration (HPP projection residual "
                f"{residual:.3g}). The line starts wherever the robot currently "
                "is: move back first (e.g. the opposite --line), or use "
                "--goto-start to return to the working pose."
            )
        q_goal = np.asarray(q_goal).flatten()
        valid, report = problem.isConfigValid(q_goal)
        if not valid:
            raise RuntimeError(f"goal configuration is invalid: {report}")

        # setConstraints installs the projector into the core problem, so the
        # steering method projects every configuration onto the line.
        direction = line / np.linalg.norm(line)
        z = direction
        x = np.cross([0.0, 0.0, 1.0], z)
        if np.linalg.norm(x) < 1e-6:
            x = np.cross([0.0, 1.0, 0.0], z)
        x /= np.linalg.norm(x)
        line_frame = pin.SE3(
            np.column_stack([x, np.cross(z, x), z]), start_pose.translation
        )

        projector = ConfigProjector(robot, "line_projector", 1e-4, 40)
        projector.add(
            Implicit(
                Position(
                    "tool0_on_line",
                    robot,
                    joint_id,
                    tool_in_joint,
                    line_frame,
                    line_xy,
                ),
                equal2,
                active_xy,
            ),
            0,
        )
        projector.add(
            Implicit(
                Orientation(
                    "tool0_orientation",
                    robot,
                    joint_id,
                    pin.SE3(tool_in_joint.rotation, np.zeros(3)),
                    pin.SE3(start_pose.rotation, np.zeros(3)),
                    xyz,
                ),
                equal3,
                xyz,
            ),
            0,
        )
        constraint_set = ConstraintSet(robot, "line")
        constraint_set.addConstraint(projector)
        problem.setConstraints(constraint_set)

        success, path, report = problem.directPath(q_start, q_goal, True)
        if not success:
            raise RuntimeError(
                f"HPP could not find a collision-free line motion: {report}"
            )

        configs = sample_path(path, args.samples)
        positions = []
        for config in configs:
            pin.forwardKinematics(model, data, config)
            pin.updateFramePlacements(model, data)
            positions.append(data.oMf[frame_id].translation.copy())
        positions = np.array(positions)
        offsets = positions - start_pose.translation
        closest = np.outer(offsets @ direction, direction)
        deviation = np.max(np.linalg.norm(offsets - closest, axis=1))
        print(f"line start position: {start_pose.translation}")
        print(f"line end position: {start_pose.translation + line}")
        print(f"max straight-line deviation: {deviation:.6f} m")
        duration = args.duration

    if args.plan_only:
        return 0

    topic = f"/{args.robot_name}/joint_trajectory"
    times = [0.0] + np.linspace(
        START_HOLD, START_HOLD + duration, len(configs)
    ).tolist()
    publish_trajectory(
        node,
        topic,
        configs_to_joint_trajectory([configs[0]] + configs, times, JOINT_NAMES),
    )
    print(f"published {len(configs) + 1} points to {topic}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
