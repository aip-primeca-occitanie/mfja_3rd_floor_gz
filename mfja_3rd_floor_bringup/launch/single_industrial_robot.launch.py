import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_pkg_path = get_package_share_directory('mfja_robot_control_config')
    base_launch = os.path.join(
        control_pkg_path,
        'launch',
        'isolated_industrial_robot.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='kuka',
            description='One industrial robot selector: kuka, staubli, hc10, hc10dt, or exact name.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/robots_room_315_only.yaml',
            description='Robot spawn YAML relative to mfja_robot_control_config.',
        ),
        DeclareLaunchArgument(
            'gripper_config',
            default_value='config/gripper_command_defaults.yaml',
            description=(
                'Per-robot gripper percentage ranges. Relative paths are '
                'resolved inside mfja_robot_control_config.'
            ),
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='isolated_industrial_robot',
            description='Minimal world file from mfja_3rd_floor_description/worlds.',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'isolated_industrial_robot_{os.getpid()}',
            description='Gazebo transport partition used to isolate this launch instance.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            choices=['true', 'false'],
            description='Start Gazebo GUI client.',
        ),
        DeclareLaunchArgument(
            'start_paused',
            default_value='false',
            choices=['true', 'false'],
            description='Start Gazebo paused so the user can press play manually.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'robot_config': LaunchConfiguration('robot_config'),
                'gripper_config': LaunchConfiguration('gripper_config'),
                'world_name': LaunchConfiguration('world_name'),
                'gz_partition': LaunchConfiguration('gz_partition'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'start_paused': LaunchConfiguration('start_paused'),
            }.items(),
        ),
    ])
