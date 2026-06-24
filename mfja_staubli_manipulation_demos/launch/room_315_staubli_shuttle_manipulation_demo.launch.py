"""Room 315 Staubli manipulation scene with one stopped right-rail shuttle."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_partition = LaunchConfiguration("gz_partition")
    description_model_path = PathJoinSubstitution(
        [FindPackageShare("mfja_3rd_floor_description"), "models"]
    )
    demo_model_path = PathJoinSubstitution(
        [FindPackageShare("mfja_staubli_manipulation_demos"), "models"]
    )
    robot_config = PathJoinSubstitution(
        [
            FindPackageShare("mfja_staubli_manipulation_demos"),
            "config",
            "robots_room315_gripper.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_partition",
                default_value=f"room_315_staubli_manipulation_{os.getpid()}",
                description="Gazebo transport partition isolating this instance.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "gui_render_engine",
                default_value="ogre",
                description="Render engine for the Gazebo GUI client.",
            ),
            DeclareLaunchArgument(
                "right_start_slot",
                default_value="3",
                description="Initial right-rail shuttle slot near the Staubli.",
            ),
            DeclareLaunchArgument(
                "shuttle_speed",
                default_value="0.3",
                description="Right-rail shuttle speed in meters per second.",
            ),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable(
                "GZ_SIM_MODEL_PATH",
                [
                    description_model_path,
                    os.pathsep,
                    demo_model_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_MODEL_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    description_model_path,
                    os.pathsep,
                    demo_model_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("mfja_3rd_floor_bringup"),
                                    "launch",
                                    "room_315_only.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "robots": "staubli",
                            "robot_config": robot_config,
                            "gui": "false",
                            "start_paused": "false",
                            "enable_room315_kinematic_shuttles": "true",
                            "enable_room315_right_rail": "true",
                            "enable_room315_left_rail": "false",
                            "room315_right_start_slot": LaunchConfiguration(
                                "right_start_slot"
                            ),
                            "room315_right_shuttle_count": "1",
                            "room315_left_shuttle_count": "0",
                            "room315_shuttles_start_enabled": "false",
                            "room315_shuttle_speed": LaunchConfiguration(
                                "shuttle_speed"
                            ),
                            "gz_partition": gz_partition,
                        }.items(),
                    ),
                ],
                scoped=True,
                forwarding=True,
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
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "gz",
                            "sim",
                            "-g",
                            "--gui-config",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("mfja_robot_control_config"),
                                    "config",
                                    "mfja_default.gui.config",
                                ]
                            ),
                            "--render-engine-gui",
                            LaunchConfiguration("gui_render_engine"),
                            "--force-version",
                            "8",
                        ],
                        condition=IfCondition(LaunchConfiguration("gui")),
                    ),
                ],
            ),
        ]
    )
