"""Named ROS execution routes for the Room 315 manipulation planner."""

from room315_config import load_config

profiles = load_config()["execution"]["profiles"]


def apply_execution_profile(args):
    """Apply the ROS routes for the selected execution profile."""
    profile = profiles[args.execution_profile]
    substitutions = {"robot_name": args.robot_name}
    args.trajectory_topic = (
        profile["trajectory_topic"].format(**substitutions)
        if profile["trajectory_topic"] is not None
        else None
    )
    args.trajectory_action = (
        profile["trajectory_action"].format(**substitutions)
        if profile["trajectory_action"] is not None
        else None
    )
    args.joint_state_topic = profile["joint_state_topic"].format(**substitutions)
    args.payload_output = profile["payload_output"]
    args.gripper_output = profile["gripper_output"]
    return args


def requires_explicit_measured_start(args, q_start_was_explicit):
    """Require a measured start for physical execution."""
    return (
        args.execute
        and args.execution_profile == "hardware"
        and not q_start_was_explicit
    )
