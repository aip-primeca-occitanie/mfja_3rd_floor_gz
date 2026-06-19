"""Room 315 simulation for the Staubli Cartesian demos.

Starts the Gazebo server through the bringup launch with gui:=false, then the
GUI as a separate process so the simulation survives a GUI crash. Send motions
with: scripts/room315_hpp_line.sh
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_partition = LaunchConfiguration("gz_partition")
    model_path = PathJoinSubstitution(
        [FindPackageShare("mfja_3rd_floor_description"), "models"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_partition",
                default_value=f"room_315_staubli_demo_{os.getpid()}",
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
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable(
                "GZ_SIM_MODEL_PATH",
                [
                    model_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_MODEL_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    model_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            # Scoped so the include's gui:=false does not overwrite this
            # file's own gui configuration (launch configurations are global).
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
                            "gui": "false",
                            "start_paused": "false",
                            "enable_room315_kinematic_shuttles": "false",
                            "gz_partition": gz_partition,
                        }.items(),
                    ),
                ],
                scoped=True,
                forwarding=True,
            ),
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
        ]
    )
