"""Staubli table pick-and-place scene without rail or shuttle nodes."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PAYLOAD_POSE = (-14.75, -5.84, 1.033, 0.0, 0.0, 0.0)
WORLD_NAME = "room_315_only"


def spawn_model(name, model_file, pose):
    x, y, z, roll, pitch, yaw = pose
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        output="screen",
        parameters=[
            {
                "world": WORLD_NAME,
                "file": model_file,
                "name": name,
                "allow_renaming": False,
                "x": x,
                "y": y,
                "z": z,
                "R": roll,
                "P": pitch,
                "Y": yaw,
            }
        ],
    )


def generate_launch_description():
    package_share = FindPackageShare("mfja_staubli_manipulation_demos")
    simulator_launch = PathJoinSubstitution(
        [
            FindPackageShare("mfja_robot_control_config"),
            "launch",
            "multi_robot_sim.launch.py",
        ]
    )
    robot_config = PathJoinSubstitution(
        [package_share, "config", "robots_room315_gripper.yaml"]
    )
    payload = PathJoinSubstitution(
        [package_share, "models", "room315_payload_box.sdf"]
    )
    gz_partition = LaunchConfiguration("gz_partition")
    gui = LaunchConfiguration("gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_partition",
                default_value=f"room315_pick_place_{os.getpid()}",
                description="Gazebo transport partition isolating this scene.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                choices=["true", "false"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch),
                launch_arguments={
                    "world_name": WORLD_NAME,
                    "robot_config": robot_config,
                    "robots": "staubli1",
                    "gz_partition": gz_partition,
                    "use_sim_time": "true",
                    "gui": gui,
                    "start_paused": "false",
                    "enable_conveyor_controller": "false",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="staubli1_gripper_bridge",
                output="screen",
                arguments=[
                    "/staubli1/gripper_joint_trajectory"
                    "@trajectory_msgs/msg/JointTrajectory"
                    "]gz.msgs.JointTrajectory"
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[spawn_model("room315_payload_box", payload, PAYLOAD_POSE)],
            ),
        ]
    )
