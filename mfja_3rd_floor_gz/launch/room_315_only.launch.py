import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_directory('mfja_room_315_bringup')
    base_launch = os.path.join(pkg_path, 'launch', 'room_315_only.launch.py')

    return LaunchDescription([
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
        DeclareLaunchArgument(
            'room315_sensor_detection_tolerance_m',
            default_value='0.01',
            description='Room 315 binary sensor detection tolerance in meters.',
        ),
        DeclareLaunchArgument(
            'room315_sensor_publish_rate_hz',
            default_value='10.0',
            description='Room 315 binary sensor feedback publish rate.',
        ),
        DeclareLaunchArgument(
            'room315_show_device_markers',
            default_value='true',
            choices=['true', 'false'],
            description='Show Room 315 slot, sensor, and stopper markers.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'pause_during_switch_update': LaunchConfiguration('pause_during_switch_update'),
                'enable_room315_kinematic_shuttles': LaunchConfiguration(
                    'enable_room315_kinematic_shuttles'
                ),
                'enable_room315_right_rail': LaunchConfiguration(
                    'enable_room315_right_rail'
                ),
                'enable_room315_left_rail': LaunchConfiguration(
                    'enable_room315_left_rail'
                ),
                'room315_right_start_slot': LaunchConfiguration(
                    'room315_right_start_slot'
                ),
                'room315_left_start_slot': LaunchConfiguration(
                    'room315_left_start_slot'
                ),
                'room315_shuttle_speed': LaunchConfiguration('room315_shuttle_speed'),
                'room315_shuttles_start_enabled': LaunchConfiguration(
                    'room315_shuttles_start_enabled'
                ),
                'room315_right_shuttle_count': LaunchConfiguration(
                    'room315_right_shuttle_count'
                ),
                'room315_left_shuttle_count': LaunchConfiguration(
                    'room315_left_shuttle_count'
                ),
                'room315_switch_motion_delay_s': LaunchConfiguration(
                    'room315_switch_motion_delay_s'
                ),
                'room315_stopper_motion_delay_s': LaunchConfiguration(
                    'room315_stopper_motion_delay_s'
                ),
                'room315_sensor_detection_tolerance_m': LaunchConfiguration(
                    'room315_sensor_detection_tolerance_m'
                ),
                'room315_sensor_publish_rate_hz': LaunchConfiguration(
                    'room315_sensor_publish_rate_hz'
                ),
                'room315_show_device_markers': LaunchConfiguration(
                    'room315_show_device_markers'
                ),
            }.items(),
        ),
    ])
