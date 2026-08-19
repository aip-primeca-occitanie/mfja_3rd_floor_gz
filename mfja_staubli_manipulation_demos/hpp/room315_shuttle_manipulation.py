#!/usr/bin/python3
"""Plan and execute the Room 315 Staubli shuttle payload manipulation demo."""

import argparse

import numpy as np

from room315_execution import execute_plan
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
    project_free_configuration,
    shuttle_box_world_pose,
    table_box_world_pose,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", default="staubli1")
    parser.add_argument("--world-name", default=WORLD_NAME)
    parser.add_argument("--box-entity-name", default=BOX_ENTITY_NAME)
    arm_output = parser.add_mutually_exclusive_group()
    arm_output.add_argument(
        "--trajectory-topic",
        default=None,
        help="Arm JointTrajectory topic. Defaults to /<robot-name>/joint_trajectory.",
    )
    arm_output.add_argument(
        "--trajectory-action",
        default=None,
        help="Arm FollowJointTrajectory action, for example the Staubli driver.",
    )
    parser.add_argument(
        "--joint-state-topic",
        default=None,
        help=(
            "JointState topic. Defaults to /joint_states for action output and "
            "/<robot-name>/joint_states for topic output."
        ),
    )
    parser.add_argument(
        "--payload-output",
        choices=["gazebo", "none"],
        default="gazebo",
        help=(
            "How to realize the payload during execution. 'gazebo' updates the "
            "existing visible box; 'none' leaves payload handling to the "
            "physical world."
        ),
    )
    parser.add_argument(
        "--gripper-output",
        choices=["joint-trajectory", "staubli-io", "none"],
        default=None,
        help=(
            "How to close and open the gripper. Gazebo uses passive gripper "
            "geometry by default. When omitted, a gripper trajectory argument "
            "selects its output; otherwise no command is sent."
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
        help=(
            "Staubli joint configuration in radians used to seed the shuttle "
            "and table placements."
        ),
    )
    parser.add_argument("--start-tolerance", type=float, default=0.06)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--build-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--direction",
        choices=["shuttle-to-table", "table-to-shuttle", "shuttle-to-shuttle"],
        default="shuttle-to-shuttle",
        help="Manipulation direction for this one-cycle HPP plan.",
    )
    parser.set_defaults(
        gripper_joints=GAZEBO_GRIPPER_JOINTS,
        gripper_open_positions=GAZEBO_GRIPPER_OPEN_POSITIONS,
        gripper_close_positions=GAZEBO_GRIPPER_CLOSE_POSITIONS,
        gripper_motion_duration=0.15,
        gripper_settle_s=0.5,
        staubli_io_service="/io_interface/write_single_io",
        staubli_io_timeout=5.0,
        target_attempts=30,
        target_pair_attempts=6,
        transition_iterations=1000,
        transition_timeout=25.0,
        samples_per_path_unit=30,
        min_segment_samples=8,
        max_joint_speed=0.50,
        min_sample_dt=0.03,
        phase_start_hold=0.2,
        box_rate=30.0,
        joint_state_timeout=10.0,
        joint_state_stale_timeout=5.0,
        subscriber_timeout=5.0,
        segment_tolerance=0.08,
        execution_timeout_scale=6.0,
        payload_sync_error=0.50,
        payload_sync_lookahead=80,
        payload_sync_report_period=5.0,
        payload_final_snap_samples=6,
        payload_pose_epsilon=1e-4,
    )
    args = parser.parse_args(argv)

    if args.gripper_output is None:
        if args.gripper_trajectory_topic is not None:
            args.gripper_output = "joint-trajectory"
        else:
            args.gripper_output = "none"
    if args.trajectory_action is not None and args.payload_output != "none":
        parser.error("--trajectory-action requires --payload-output none")
    if args.gripper_output == "staubli-io" and args.trajectory_action is None:
        parser.error("--gripper-output staubli-io requires --trajectory-action")
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
    print(f"grippers: {sorted(entry.key() for entry in robot.grippers())}")
    print(f"handles: {sorted(entry.key() for entry in robot.handles())}")
    print(f"contact surfaces: {sorted(robot.contactSurfaces())}")
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
        target_pair_attempts=args.target_pair_attempts,
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
        execute_plan(
            robot,
            phases,
            q_source,
            args,
        )
    else:
        print("planning complete; pass --execute to run the phases")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
