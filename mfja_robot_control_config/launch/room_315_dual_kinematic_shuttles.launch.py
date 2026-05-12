from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common_parameters = {
        'path_backend': LaunchConfiguration('path_backend'),
        'speed': LaunchConfiguration('speed'),
        'start_enabled': LaunchConfiguration('start_enabled'),
        'gazebo_world_name': LaunchConfiguration('gazebo_world_name'),
        'enable_gazebo_set_pose': True,
        'enable_gazebo_spawn': True,
        'enable_gazebo_delete': True,
        'sync_from_visual_switch_states': True,
        'publish_visual_switch_commands': True,
        'switch_motion_delay_s': LaunchConfiguration('switch_motion_delay_s'),
        'stopper_motion_delay_s': LaunchConfiguration('stopper_motion_delay_s'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    right_node = Node(
        package='mfja_robot_control_config',
        executable='room_315_kinematic_shuttle_node.py',
        namespace='room_315/rails/right',
        name='room_315_kinematic_shuttle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_right')),
        parameters=[
            common_parameters,
            {
                'rail_side': 'right',
                'start_slot': LaunchConfiguration('right_start_slot'),
                'shuttle_count': LaunchConfiguration('right_shuttle_count'),
            },
        ],
    )

    left_node = Node(
        package='mfja_robot_control_config',
        executable='room_315_kinematic_shuttle_node.py',
        namespace='room_315/rails/left',
        name='room_315_kinematic_shuttle',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_left')),
        parameters=[
            common_parameters,
            {
                'rail_side': 'left',
                'start_slot': LaunchConfiguration('left_start_slot'),
                'shuttle_count': LaunchConfiguration('left_shuttle_count'),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_world_name',
            default_value='room_315_only',
            description='Gazebo world entity name used by the shuttle nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'path_backend',
            default_value='cubic_hermite',
            description='Path backend: polyline or cubic_hermite.',
        ),
        DeclareLaunchArgument(
            'speed',
            default_value='0.2',
            description='Common shuttle speed for both rails in meters per second.',
        ),
        DeclareLaunchArgument(
            'start_enabled',
            default_value='false',
            choices=['true', 'false'],
            description='Start initial shuttles moving without waiting for ON.',
        ),
        DeclareLaunchArgument(
            'switch_motion_delay_s',
            default_value='0.3',
            description='Delay between a switch command and actual switch state.',
        ),
        DeclareLaunchArgument(
            'stopper_motion_delay_s',
            default_value='0.1',
            description='Delay between a stopper command and actual stopper state.',
        ),
        DeclareLaunchArgument(
            'right_start_slot',
            default_value='2',
            description='Startup slot for the right-rail shuttle.',
        ),
        DeclareLaunchArgument(
            'right_shuttle_count',
            default_value='0',
            description='Number of initial right-rail shuttles. Use 0 to start the rail with no shuttle.',
        ),
        DeclareLaunchArgument(
            'left_start_slot',
            default_value='2',
            description='Startup slot for the left-rail shuttle.',
        ),
        DeclareLaunchArgument(
            'left_shuttle_count',
            default_value='0',
            description='Number of initial left-rail shuttles. Use 0 to start the rail with no shuttle.',
        ),
        DeclareLaunchArgument(
            'enable_right',
            default_value='true',
            choices=['true', 'false'],
            description='Start the right-rail shuttle node.',
        ),
        DeclareLaunchArgument(
            'enable_left',
            default_value='true',
            choices=['true', 'false'],
            description='Start the left-rail shuttle node.',
        ),
        right_node,
        left_node,
    ])
