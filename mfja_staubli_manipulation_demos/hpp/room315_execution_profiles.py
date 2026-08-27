"""Named ROS execution routes for the Room 315 manipulation planner."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ExecutionProfile:
    trajectory_topic: str | None
    trajectory_action: str | None
    joint_state_topic: str
    payload_output: str
    gripper_output: str


EXECUTION_PROFILES = {
    "simulation": ExecutionProfile(
        trajectory_topic="/{robot_name}/joint_trajectory",
        trajectory_action=None,
        joint_state_topic="/{robot_name}/joint_states",
        payload_output="gazebo",
        gripper_output="joint-trajectory",
    ),
    "hardware": ExecutionProfile(
        trajectory_topic=None,
        trajectory_action="/manipulator_controller/joint_trajectory_action",
        joint_state_topic="/joint_states",
        payload_output="none",
        gripper_output="staubli-io",
    ),
}


def apply_execution_profile(args):
    """Apply the ROS routes for the selected execution profile."""
    profile = EXECUTION_PROFILES[args.execution_profile]
    substitutions = {"robot_name": args.robot_name}
    args.trajectory_topic = (
        profile.trajectory_topic.format(**substitutions)
        if profile.trajectory_topic is not None
        else None
    )
    args.trajectory_action = (
        profile.trajectory_action.format(**substitutions)
        if profile.trajectory_action is not None
        else None
    )
    args.joint_state_topic = profile.joint_state_topic.format(**substitutions)
    args.payload_output = profile.payload_output
    args.gripper_output = profile.gripper_output
    return args


def requires_explicit_measured_start(args, q_start_was_explicit):
    """Require a measured start for physical execution."""
    return (
        args.execute
        and args.execution_profile == "hardware"
        and not q_start_was_explicit
    )
