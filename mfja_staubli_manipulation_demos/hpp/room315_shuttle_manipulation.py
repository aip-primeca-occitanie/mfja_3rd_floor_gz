#!/usr/bin/python3
"""Plan and execute the Room 315 Staubli shuttle payload manipulation demo."""

import argparse
from pathlib import Path

import numpy as np

from room315_execution import execute_plan, wait_for_execution_start
from room315_planning import (
    build_execution_phases,
    direction_endpoints,
    format_plan,
    plan_manipulation,
)
from room315_problem import (
    BOX_ENTITY_NAME,
    DEFAULT_Q_START,
    DEFAULT_SHUTTLE_SLOT3_POSE,
    DEFAULT_SHUTTLE_SLOT4_POSE,
    GAZEBO_GRIPPER_CLOSE_POSITIONS,
    GAZEBO_GRIPPER_JOINTS,
    GAZEBO_GRIPPER_OPEN_POSITIONS,
    GRAPH_NAME,
    JOINT_NAMES,
    WORLD_NAME,
    box_configuration_from_world_pose,
    build_problem,
    mapping_names,
    project_free_configuration,
    shuttle_box_world_pose,
    table_box_world_pose,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", default="staubli1")
    parser.add_argument("--world-name", default=WORLD_NAME)
    parser.add_argument("--box-entity-name", default=BOX_ENTITY_NAME)
    parser.add_argument(
        "--trajectory-topic",
        default=None,
        help="Arm JointTrajectory topic. Defaults to /<robot-name>/joint_trajectory.",
    )
    parser.add_argument(
        "--joint-state-topic",
        default=None,
        help="JointState topic. Defaults to /<robot-name>/joint_states.",
    )
    parser.add_argument(
        "--payload-output",
        choices=["gazebo", "none"],
        default="gazebo",
        help=(
            "How to realize the payload during execution. 'gazebo' spawns and "
            "kinematically follows the visible box; 'none' leaves payload "
            "handling to the physical world."
        ),
    )
    parser.add_argument(
        "--gripper-output",
        choices=["bool", "joint-trajectory", "staubli-io", "none"],
        default="none",
        help=(
            "Output used for semantic grasp/release pre-actions. Gazebo uses "
            "passive gripper geometry by default; select 'bool', "
            "'joint-trajectory', or 'staubli-io' for an actuated gripper."
        ),
    )
    parser.add_argument(
        "--gripper-command-topic",
        default=None,
        help=(
            "Bool topic used when --gripper-output bool. Defaults to "
            "/<robot-name>/gripper/command."
        ),
    )
    parser.add_argument(
        "--gripper-trajectory-topic",
        default=None,
        help=(
            "JointTrajectory topic used when --gripper-output joint-trajectory. "
            "Defaults to /<robot-name>/gripper_joint_trajectory."
        ),
    )
    parser.add_argument(
        "--gripper-joints",
        nargs="+",
        default=GAZEBO_GRIPPER_JOINTS,
        help="Actuated gripper joint names for --gripper-output joint-trajectory.",
    )
    parser.add_argument(
        "--gripper-open-positions",
        nargs="+",
        type=float,
        default=GAZEBO_GRIPPER_OPEN_POSITIONS,
        help="Open joint positions for --gripper-output joint-trajectory.",
    )
    parser.add_argument(
        "--gripper-close-positions",
        nargs="+",
        type=float,
        default=GAZEBO_GRIPPER_CLOSE_POSITIONS,
        help="Closed joint positions for --gripper-output joint-trajectory.",
    )
    parser.add_argument("--gripper-motion-duration", type=float, default=0.15)
    parser.add_argument(
        "--staubli-io-service",
        default="/io_interface/write_single_io",
        help="Staubli VAL3 IO service used when --gripper-output staubli-io.",
    )
    parser.add_argument(
        "--staubli-io-pin",
        type=int,
        default=None,
        help="Digital output pin used for the pneumatic gripper valve.",
    )
    parser.add_argument(
        "--staubli-io-module-id",
        type=int,
        default=None,
        help="Staubli IO module id. Defaults to staubli_msgs/msg/IOModule.VALVE_OUT.",
    )
    parser.add_argument(
        "--staubli-io-inverted",
        action="store_true",
        help="Use state=False to close and state=True to open the gripper.",
    )
    parser.add_argument("--staubli-io-timeout", type=float, default=5.0)
    parser.add_argument(
        "--shuttle-pose",
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        type=float,
        default=DEFAULT_SHUTTLE_SLOT3_POSE,
        help="Gazebo/world pose of the shuttle model at the pickup slot.",
    )
    parser.add_argument(
        "--destination-shuttle-pose",
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        type=float,
        default=None,
        help="Gazebo/world pose of a second shuttle deck used as the drop support.",
    )
    parser.add_argument(
        "--q-start",
        nargs=6,
        metavar=tuple(JOINT_NAMES),
        default=DEFAULT_Q_START,
        type=float,
        help="Staubli joint configuration used to seed the shuttle and table placements.",
    )
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--direction",
        choices=["shuttle-to-table", "table-to-shuttle", "shuttle-to-shuttle"],
        default="shuttle-to-table",
        help="Manipulation direction for this one-cycle HPP plan.",
    )
    parser.add_argument("--target-attempts", type=int, default=30)
    parser.add_argument("--transition-iterations", type=int, default=1000)
    parser.add_argument("--transition-timeout", type=float, default=25.0)
    parser.add_argument("--samples-per-path-unit", type=int, default=30)
    parser.add_argument("--min-segment-samples", type=int, default=8)
    parser.add_argument("--max-joint-speed", type=float, default=0.50)
    parser.add_argument("--min-sample-dt", type=float, default=0.03)
    parser.add_argument("--phase-start-hold", type=float, default=0.2)
    parser.add_argument("--gripper-settle-s", type=float, default=0.5)
    parser.add_argument("--box-rate", type=float, default=30.0)
    parser.add_argument("--joint-state-timeout", type=float, default=10.0)
    parser.add_argument("--joint-state-stale-timeout", type=float, default=5.0)
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument(
        "--start-mode",
        choices=["check", "snap", "move"],
        default="check",
        help=(
            "How execution reaches the first planned HPP configuration. "
            "'check' requires the arm to already be there, 'move' uses the "
            "conservative pre-position trajectory, and 'snap' uses a brief "
            "simulation-only command."
        ),
    )
    parser.add_argument("--snap-start-duration", type=float, default=2.0)
    parser.add_argument("--snap-start-timeout", type=float, default=8.0)
    parser.add_argument("--start-joint-speed", type=float, default=0.15)
    parser.add_argument("--start-samples-per-second", type=int, default=30)
    parser.add_argument("--min-start-duration", type=float, default=5.0)
    parser.add_argument("--initial-hold", type=float, default=1.5)
    parser.add_argument("--start-tolerance", type=float, default=0.06)
    parser.add_argument("--segment-tolerance", type=float, default=0.08)
    parser.add_argument("--execution-timeout-scale", type=float, default=6.0)
    parser.add_argument("--payload-sync-error", type=float, default=0.50)
    parser.add_argument("--payload-sync-lookahead", type=int, default=80)
    parser.add_argument("--payload-sync-report-period", type=float, default=5.0)
    parser.add_argument("--payload-final-snap-samples", type=int, default=6)
    parser.add_argument("--payload-pose-epsilon", type=float, default=1e-4)
    parser.add_argument(
        "--replace-box",
        action="store_true",
        help="Remove an existing Gazebo payload entity before spawning a fresh one.",
    )
    parser.add_argument(
        "--ready-file",
        type=Path,
        default=None,
        help="Touch this file after HPP planning and execution sampling are ready.",
    )
    parser.add_argument(
        "--start-file",
        type=Path,
        default=None,
        help="When executing, wait for this file before publishing to Gazebo.",
    )
    parser.add_argument(
        "--abort-file",
        type=Path,
        default=None,
        help="When waiting on --start-file, exit cleanly if this file appears.",
    )
    args = parser.parse_args()

    if args.plan_only and args.execute:
        parser.error("--plan-only and --execute are mutually exclusive")
    if args.gripper_output == "none" and args.staubli_io_pin is not None:
        args.gripper_output = "staubli-io"
    if args.gripper_output == "none" and args.gripper_command_topic is not None:
        args.gripper_output = "bool"
    if args.gripper_output == "none" and args.gripper_trajectory_topic is not None:
        args.gripper_output = "joint-trajectory"
    if args.gripper_output == "staubli-io" and args.staubli_io_pin is None:
        parser.error("--staubli-io-pin is required with --gripper-output staubli-io")
    if args.direction == "shuttle-to-shuttle" and args.destination_shuttle_pose is None:
        args.destination_shuttle_pose = DEFAULT_SHUTTLE_SLOT4_POSE
    return args


def main():
    args = parse_args()
    destination_shuttle_pose = (
        tuple(args.destination_shuttle_pose)
        if args.destination_shuttle_pose is not None
        else None
    )
    robot, problem, graph = build_problem(tuple(args.shuttle_pose), destination_shuttle_pose)
    print("HPP manipulation scene initialized")
    print(f"config size: {robot.configSize()}")
    print(f"grippers: {mapping_names(robot.grippers())}")
    print(f"handles: {mapping_names(robot.handles())}")
    print(f"contact surfaces: {mapping_names(robot.contactSurfaces())}")
    print(f"graph: {GRAPH_NAME}")
    if args.build_only:
        return 0

    q_arm = np.asarray(args.q_start, dtype=float)
    q_shuttle_guess = box_configuration_from_world_pose(
        q_arm, shuttle_box_world_pose(tuple(args.shuttle_pose))
    )
    q_table_guess = box_configuration_from_world_pose(q_arm, table_box_world_pose())
    q_shuttle = project_free_configuration(problem, graph, q_shuttle_guess, "shuttle")
    q_table = project_free_configuration(problem, graph, q_table_guess, "table")
    q_drop_shuttle = None
    if destination_shuttle_pose is not None:
        q_drop_guess = box_configuration_from_world_pose(
            q_arm, shuttle_box_world_pose(destination_shuttle_pose)
        )
        q_drop_shuttle = project_free_configuration(
            problem, graph, q_drop_guess, "drop_shuttle"
        )
    q_source, q_destination, source_label, destination_label = direction_endpoints(
        args.direction, q_shuttle, q_table, q_drop_shuttle
    )
    print(f"direction: {args.direction} ({source_label} -> {destination_label})")

    segments = plan_manipulation(
        robot,
        problem,
        graph,
        q_source,
        q_destination,
        source_label=source_label,
        destination_label=destination_label,
        target_attempts=args.target_attempts,
        transition_iterations=args.transition_iterations,
        transition_timeout=args.transition_timeout,
    )
    format_plan(segments)
    phases = build_execution_phases(
        robot,
        graph,
        segments,
        q_source,
        q_destination,
        source_label,
        destination_label,
        args,
    )

    if args.execute:
        if not wait_for_execution_start(args):
            return 2
        execute_plan(
            robot,
            phases,
            q_source,
            args,
        )
    else:
        print("pass --execute to publish the phases to Gazebo")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
