import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_pkg_path = get_package_share_directory('mfja_robot_control_config')
    base_launch = os.path.join(control_pkg_path, 'launch', 'multi_robot_sim.launch.py')
    shuttles_launch = os.path.join(
        control_pkg_path,
        'launch',
        'room_315_dual_kinematic_shuttles.launch.py',
    )

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
                'short aliases, numeric indices, "all", or "none".'
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
        DeclareLaunchArgument(
            'start_paused',
            default_value='true',
            choices=['true', 'false'],
            description='Start Gazebo paused so the user can press play manually.',
        ),
        DeclareLaunchArgument(
            'initial_loop_mode',
            default_value='auto',
            description='Startup loop mode: auto, PETIT_BOUCLE, or GRAND_BOUCLE.',
        ),
        DeclareLaunchArgument(
            'pause_during_switch_update',
            default_value='false',
            choices=['true', 'false'],
            description='Pause Gazebo while applying visual switch pose updates.',
        ),
        DeclareLaunchArgument(
            'enable_room315_kinematic_shuttles',
            default_value='true',
            choices=['true', 'false'],
            description='Start the Room 315 right/left kinematic rail shuttle nodes.',
        ),
        DeclareLaunchArgument(
            'enable_room315_right_rail',
            default_value='true',
            choices=['true', 'false'],
            description='Start the Room 315 right rail shuttle node.',
        ),
        DeclareLaunchArgument(
            'enable_room315_left_rail',
            default_value='true',
            choices=['true', 'false'],
            description='Start the Room 315 left rail shuttle node.',
        ),
        DeclareLaunchArgument(
            'room315_right_start_slot',
            default_value='2',
            description='Startup slot for the Room 315 right rail shuttle.',
        ),
        DeclareLaunchArgument(
            'room315_left_start_slot',
            default_value='2',
            description='Startup slot for the Room 315 left rail shuttle.',
        ),
        DeclareLaunchArgument(
            'room315_shuttle_speed',
            default_value='0.2',
            description='Common Room 315 shuttle speed in meters per second.',
        ),
        DeclareLaunchArgument(
            'room315_shuttles_start_enabled',
            default_value='false',
            choices=['true', 'false'],
            description='Start initial Room 315 shuttles moving without waiting for ON.',
        ),
        DeclareLaunchArgument(
            'room315_right_shuttle_count',
            default_value='0',
            description='Number of initial shuttles on the Room 315 right rail.',
        ),
        DeclareLaunchArgument(
            'room315_left_shuttle_count',
            default_value='0',
            description='Number of initial shuttles on the Room 315 left rail.',
        ),
        DeclareLaunchArgument(
            'room315_switch_motion_delay_s',
            default_value='0.3',
            description='Room 315 switch motion delay in seconds.',
        ),
        DeclareLaunchArgument(
            'room315_stopper_motion_delay_s',
            default_value='0.1',
            description='Room 315 stopper motion delay in seconds.',
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
                'start_paused': LaunchConfiguration('start_paused'),
                'initial_loop_mode': LaunchConfiguration('initial_loop_mode'),
                'pause_during_switch_update': LaunchConfiguration('pause_during_switch_update'),
            }.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(shuttles_launch),
                    condition=IfCondition(
                        LaunchConfiguration('enable_room315_kinematic_shuttles')
                    ),
                    launch_arguments={
                        'gazebo_world_name': LaunchConfiguration('world_name'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'speed': LaunchConfiguration('room315_shuttle_speed'),
                        'start_enabled': LaunchConfiguration(
                            'room315_shuttles_start_enabled'
                        ),
                        'switch_motion_delay_s': LaunchConfiguration(
                            'room315_switch_motion_delay_s'
                        ),
                        'stopper_motion_delay_s': LaunchConfiguration(
                            'room315_stopper_motion_delay_s'
                        ),
                        'right_start_slot': LaunchConfiguration(
                            'room315_right_start_slot'
                        ),
                        'left_start_slot': LaunchConfiguration(
                            'room315_left_start_slot'
                        ),
                        'right_shuttle_count': LaunchConfiguration(
                            'room315_right_shuttle_count'
                        ),
                        'left_shuttle_count': LaunchConfiguration(
                            'room315_left_shuttle_count'
                        ),
                        'enable_right': LaunchConfiguration('enable_room315_right_rail'),
                        'enable_left': LaunchConfiguration('enable_room315_left_rail'),
                    }.items(),
                ),
            ],
        ),
    ])
