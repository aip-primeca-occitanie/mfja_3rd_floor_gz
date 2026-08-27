#!/usr/bin/python3
"""Plan and optionally execute one table pick-and-place."""

import argparse
import sys
import threading

import numpy as np

from room315_execution import execute_plan
from room315_execution_profiles import (
    apply_execution_profile,
    EXECUTION_PROFILES,
    requires_explicit_measured_start,
)
from room315_planning import (
    build_execution_plan,
    format_plan,
    plan_manipulation,
)
from room315_problem import (
    BOX_ENTITY_NAME,
    DEFAULT_Q_START,
    GAZEBO_GRIPPER_CLOSE_POSITIONS,
    GAZEBO_GRIPPER_JOINTS,
    GAZEBO_GRIPPER_OPEN_POSITIONS,
    JOINT_NAMES,
    WORLD_NAME,
    box_configuration_from_world_pose,
    build_problem,
    project_free_configuration,
    table_box_world_pose,
)


PICK_OFFSET_X = -0.10
PLACE_OFFSET_X = 0.10


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execution-profile",
        choices=sorted(EXECUTION_PROFILES),
        default="simulation",
    )
    parser.add_argument(
        "--q-start",
        nargs=6,
        metavar=tuple(JOINT_NAMES),
        type=float,
        help="Measured Staubli joint configuration in radians.",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--build-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument(
        "--viser",
        action="store_true",
        help="open the planned paths in a browser at http://localhost:8000",
    )
    parser.set_defaults(
        robot_name="staubli1",
        world_name=WORLD_NAME,
        box_entity_name=BOX_ENTITY_NAME,
        trajectory_topic=None,
        trajectory_action=None,
        joint_state_topic=None,
        payload_output=None,
        gripper_output=None,
        gripper_trajectory_topic=None,
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
        segment_start_hold=0.2,
        box_rate=30.0,
        start_tolerance=0.06,
        joint_state_timeout=10.0,
        joint_state_stale_timeout=5.0,
        subscriber_timeout=5.0,
        segment_tolerance=0.08,
        execution_timeout_scale=6.0,
        payload_sync_error=0.50,
        payload_sync_lookahead=80,
        payload_final_snap_samples=6,
        payload_pose_epsilon=1e-4,
    )
    args = parser.parse_args(argv)

    q_start_was_explicit = args.q_start is not None
    if args.q_start is None:
        args.q_start = DEFAULT_Q_START
    apply_execution_profile(args)
    if requires_explicit_measured_start(args, q_start_was_explicit):
        parser.error("hardware execution requires an explicit measured --q-start")
    return args


def show_in_viser(robot, problem, segments, q_source):
    """Open the HPP scene and planned segments in the browser viewer."""
    import coal
    import webbrowser

    # The pinned viewer still uses Coal's former Python module name.
    sys.modules.setdefault("hppfcl", coal)
    from pyhpp_viser import Viewer

    viewer = Viewer(robot, problem)
    viewer.start(open=False)
    viewer(q_source)
    for index, segment in enumerate(segments, start=1):
        viewer.loadPath(
            segment.path,
            name=f"{index}: {segment.transition_name}",
        )
    url = "http://localhost:8000"
    print(f"Viser is ready at {url} (Ctrl-C to stop)")
    webbrowser.open(url)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass


def run(args):
    robot, problem, graph = build_problem()
    print("HPP table pick-and-place scene initialized")
    if args.build_only:
        return 0

    q_arm = np.asarray(args.q_start, dtype=float)
    q_source = project_free_configuration(
        problem,
        graph,
        box_configuration_from_world_pose(
            q_arm, table_box_world_pose(x_offset=PICK_OFFSET_X)
        ),
        "pick",
    )
    q_destination = project_free_configuration(
        problem,
        graph,
        box_configuration_from_world_pose(
            q_arm, table_box_world_pose(x_offset=PLACE_OFFSET_X)
        ),
        "place",
    )
    segments = plan_manipulation(
        robot,
        problem,
        graph,
        q_source,
        q_destination,
        source_label="pick",
        destination_label="place",
        target_attempts=args.target_attempts,
        target_pair_attempts=args.target_pair_attempts,
        transition_iterations=args.transition_iterations,
        transition_timeout=args.transition_timeout,
    )
    format_plan(segments)
    execution_plan = build_execution_plan(
        robot,
        graph,
        segments,
        q_source,
        q_destination,
        "pick",
        "place",
        args,
    )

    if args.viser:
        show_in_viser(robot, problem, segments, q_source)
    elif args.execute:
        execute_plan(robot, execution_plan, q_source, args)
    else:
        print("planning complete; pass --execute to run the plan")
    return 0


def main(argv=None):
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
