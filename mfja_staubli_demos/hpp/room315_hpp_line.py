#!/usr/bin/python3
"""Plan a straight Cartesian tool0 line with HPP for Gazebo or Staubli export."""

import argparse
import sys
import time

import numpy as np
import rclpy
from hpp_exec import (
    configs_to_joint_trajectory,
    read_current_configuration,
)
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from room315_cartesian_line import (
    DEFAULT_LINE,
    DEFAULT_Q_START,
    JOINT_NAMES,
    START_HOLD,
    build_problem,
    plan_cartesian_line,
    sample_path,
)
from staubli_trajectory_export import render_joint_trajectory

JOINT_STATE_TIMEOUT = 10.0
SUBSCRIBER_TIMEOUT = 5.0


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
        default=None,
        type=float,
        help="Start configuration for --plan-only or offline Staubli export "
        "(live Gazebo runs start from the current robot configuration).",
    )
    parser.add_argument(
        "--joint-states-topic",
        help=(
            "Read the export start configuration from this JointState topic "
            "instead of --q-start. Only valid with "
            "--print-joint-trajectory."
        ),
    )
    offline_mode = parser.add_mutually_exclusive_group()
    offline_mode.add_argument("--plan-only", action="store_true")
    offline_mode.add_argument(
        "--print-joint-trajectory",
        "--print-joint-path-command",
        dest="print_joint_trajectory",
        action="store_true",
        help=(
            "Compute and print the JointTrajectory JSON payload for the "
            "Staubli /joint_path_command topic."
        ),
    )
    parser.add_argument(
        "--goto-start",
        action="store_true",
        help=(
            "Simulation-only setup motion to --q-start. A valid spawn uses "
            "collision checking; recovery from the invalid upright Gazebo "
            "spawn is deliberately unchecked."
        ),
    )
    args = parser.parse_args()
    q_start_was_explicit = args.q_start is not None
    if args.joint_states_topic and not args.print_joint_trajectory:
        parser.error(
            "--joint-states-topic is only valid with "
            "--print-joint-trajectory"
        )
    if (
        args.print_joint_trajectory
        and q_start_was_explicit
        and args.joint_states_topic
    ):
        parser.error(
            "--q-start and --joint-states-topic cannot be used together"
        )
    if args.q_start is None:
        args.q_start = DEFAULT_Q_START.copy()
    if args.goto_start and (args.plan_only or args.print_joint_trajectory):
        parser.error("--goto-start needs the live Gazebo robot")
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and positive")
    if not np.all(np.isfinite(args.q_start)):
        parser.error("--q-start values must be finite")
    if not args.goto_start and (
        not np.all(np.isfinite(args.line)) or np.linalg.norm(args.line) <= 0.0
    ):
        parser.error("--line must be finite and non-zero")

    line = np.array(args.line)

    node = None
    if args.plan_only or (
        args.print_joint_trajectory and not args.joint_states_topic
    ):
        q_start = np.array(args.q_start)
    elif args.print_joint_trajectory:
        rclpy.init()
        reader = Node("room315_staubli_joint_state_reader")
        try:
            q_start = read_current_configuration(
                reader,
                JOINT_NAMES,
                topic=args.joint_states_topic,
                timeout_sec=JOINT_STATE_TIMEOUT,
                require_single_publisher=True,
            )
        finally:
            reader.destroy_node()
            rclpy.shutdown()
        if q_start is None:
            raise RuntimeError(
                f"could not read {args.joint_states_topic}; verify that the "
                "topic produces messages and ROS_DOMAIN_ID matches the "
                "Staubli driver"
            )
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

    if not np.all(np.isfinite(q_start)):
        raise RuntimeError("the start configuration contains non-finite values")

    if args.goto_start:
        robot, problem = build_problem()
        valid, report = problem.isConfigValid(q_start)
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
        plan = plan_cartesian_line(
            q_start=q_start,
            line=line,
            samples=args.samples,
        )
        configs = plan.configurations
        diagnostics = sys.stderr if args.print_joint_trajectory else sys.stdout
        print(f"line start position: {plan.start_position}", file=diagnostics)
        print(f"line end position: {plan.end_position}", file=diagnostics)
        print(
            f"max straight-line deviation: {plan.max_deviation:.6f} m",
            file=diagnostics,
        )
        duration = args.duration

    if args.plan_only:
        return 0

    times = [0.0] + np.linspace(
        START_HOLD, START_HOLD + duration, len(configs)
    ).tolist()
    trajectory_configs = [configs[0]] + configs
    if args.print_joint_trajectory:
        if args.joint_states_topic:
            print(
                f"# Start configuration read from {args.joint_states_topic}.",
                file=sys.stderr,
            )
        elif q_start_was_explicit:
            print(
                "# Start configuration supplied with --q-start.", file=sys.stderr
            )
        else:
            print(
                "# Start configuration is the demo DEFAULT_Q_START.",
                file=sys.stderr,
            )
        print("# Joint positions are radians.", file=sys.stderr)
        print(
            "# Verify the first point against fresh /joint_states before use.",
            file=sys.stderr,
        )
        print(
            "# Zero velocity arrays select the driver's 10% fallback speed.",
            file=sys.stderr,
        )
        print(
            "# Direct topic publication is fire-and-forget; use an action "
            "interface for results and goal-scoped cancellation.",
            file=sys.stderr,
        )
        print(
            "# The current VAL3 driver treats --duration as trajectory metadata.",
            file=sys.stderr,
        )
        print(render_joint_trajectory(trajectory_configs, times, JOINT_NAMES))
        return 0

    topic = f"/{args.robot_name}/joint_trajectory"
    publish_trajectory(
        node,
        topic,
        configs_to_joint_trajectory(trajectory_configs, times, JOINT_NAMES),
    )
    print(f"published {len(configs) + 1} points to {topic}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
