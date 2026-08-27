"""Direct Staubli TX2-60L driver bringup for authorized hardware use."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    joint_config = LaunchConfiguration("joint_config")
    enable_io = LaunchConfiguration("enable_io")
    enable_system = LaunchConfiguration("enable_system")
    publish_description = LaunchConfiguration("publish_robot_description")
    driver_launch = PathJoinSubstitution(
        [
            FindPackageShare("staubli_val3_driver"),
            "launch",
            "robot_interface_streaming.launch.py",
        ]
    )
    robot_xacro = PathJoinSubstitution(
        [
            FindPackageShare("staubli_tx2_60l_description"),
            "urdf",
            "tx2_60l.xacro",
        ]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_xacro]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                description="IP address of the authorized Staubli controller",
            ),
            DeclareLaunchArgument(
                "joint_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("staubli_val3_driver"),
                        "config",
                        "tx2_60l_streaming.yaml",
                    ]
                ),
                description=(
                    "TX2-60L joint names and commissioned velocity limits"
                ),
            ),
            DeclareLaunchArgument(
                "enable_io",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "enable_system",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "publish_robot_description",
                default_value="true",
                choices=["true", "false"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(driver_launch),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "joint_config": joint_config,
                    "enable_io": enable_io,
                    "enable_system": enable_system,
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="staubli_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(publish_description),
            ),
        ]
    )
