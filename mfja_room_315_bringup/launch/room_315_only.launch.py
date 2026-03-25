import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_pkg_path = get_package_share_directory('mfja_robot_control_config')
    base_launch = os.path.join(control_pkg_path, 'launch', 'multi_robot_sim.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='room_315_only',
            description='World file name from mfja_3rd_floor_description/worlds.',
        ),
        DeclareLaunchArgument(
            'robots',
            default_value='',
            description=(
                'Comma-separated robot selection list. Supports full names, '
                'short aliases, numeric indices, or "all".'
            ),
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/robots_room_315_only.yaml',
            description='Robot spawn YAML relative to mfja_robot_control_config.',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'room_315_only_{os.getpid()}',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'robot_config': LaunchConfiguration('robot_config'),
                'robots': LaunchConfiguration('robots'),
                'gz_partition': LaunchConfiguration('gz_partition'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
    ])
