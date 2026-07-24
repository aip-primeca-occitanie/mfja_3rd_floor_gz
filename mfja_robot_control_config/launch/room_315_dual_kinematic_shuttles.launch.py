from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common_parameters = {
        'path_backend': LaunchConfiguration('path_backend'),
        'speed': LaunchConfiguration('speed'),
        'falling_stop_offset_m': LaunchConfiguration('falling_stop_offset_m'),
        'shuttle_collision_distance_m': LaunchConfiguration('shuttle_collision_distance_m'),
        'start_enabled': LaunchConfiguration('start_enabled'),
        'gazebo_world_name': LaunchConfiguration('gazebo_world_name'),
        'enable_gazebo_set_pose': True,
        'enable_gazebo_spawn': True,
        'enable_gazebo_delete': True,
        'sync_from_visual_switch_states': True,
        'publish_visual_switch_commands': True,
        'switch_motion_delay_s': LaunchConfiguration('switch_motion_delay_s'),
        'stopper_motion_delay_s': LaunchConfiguration('stopper_motion_delay_s'),
        'sensor_publish_rate_hz': LaunchConfiguration('sensor_publish_rate_hz'),
        'sync_sensor_feedback_to_motion_tick': LaunchConfiguration(
            'sync_sensor_feedback_to_motion_tick'
        ),
        'gazebo_set_pose_rate_hz': LaunchConfiguration('gazebo_set_pose_rate_hz'),
        'show_device_markers': LaunchConfiguration('show_device_markers'),
        'sensor_marker_visual_hold_s': LaunchConfiguration(
            'sensor_marker_visual_hold_s'
        ),
        'visual_debug_colors': LaunchConfiguration('visual_debug_colors'),
        'enable_payload_visuals': LaunchConfiguration('enable_payload_visuals'),
        'payload_type': LaunchConfiguration('payload_type'),
        'payload_pose_x_offset_m': LaunchConfiguration('payload_pose_x_offset_m'),
        'payload_pose_z_offset_m': LaunchConfiguration('payload_pose_z_offset_m'),
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
                'start_slots': LaunchConfiguration('right_start_slots'),
                'start_positions': LaunchConfiguration('right_start_positions'),
                'shuttle_count': LaunchConfiguration('right_shuttle_count'),
                'loaded_shuttles': LaunchConfiguration('right_loaded_shuttles'),
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
                'start_slots': LaunchConfiguration('left_start_slots'),
                'start_positions': LaunchConfiguration('left_start_positions'),
                'shuttle_count': LaunchConfiguration('left_shuttle_count'),
                'loaded_shuttles': LaunchConfiguration('left_loaded_shuttles'),
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
            'falling_stop_offset_m',
            default_value='0.0',
            description='Distance before an invalid route endpoint where FALLING mode is latched.',
        ),
        DeclareLaunchArgument(
            'shuttle_collision_distance_m',
            default_value='0.33',
            description='Minimum separation between Room 315 shuttles before collision avoidance stops motion.',
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
            'sensor_publish_rate_hz',
            default_value='30.0',
            description='Publish rate for binary sensor feedback.',
        ),
        DeclareLaunchArgument(
            'sync_sensor_feedback_to_motion_tick',
            default_value='true',
            choices=['true', 'false'],
            description='Publish sensor feedback from the same tick that updates shuttle motion.',
        ),
        DeclareLaunchArgument(
            'gazebo_set_pose_rate_hz',
            default_value='30.0',
            description='Maximum Gazebo set_pose rate for visible shuttle motion.',
        ),
        DeclareLaunchArgument(
            'show_device_markers',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn visual markers for position sensors and stoppers.',
        ),
        DeclareLaunchArgument(
            'sensor_marker_visual_hold_s',
            default_value='0.35',
            description='Minimum seconds a crossed position sensor marker stays green.',
        ),
        DeclareLaunchArgument(
            'visual_debug_colors',
            default_value='true',
            choices=['true', 'false'],
            description='Use debug colors for shuttle mode; false keeps shuttles black.',
        ),
        DeclareLaunchArgument(
            'enable_payload_visuals',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn carried payload boxes for loaded Room 315 shuttles.',
        ),
        DeclareLaunchArgument(
            'payload_type',
            default_value='box',
            description='Structured payload type used for loaded shuttles.',
        ),
        DeclareLaunchArgument(
            'payload_pose_x_offset_m',
            default_value='-0.08',
            description='Additional x offset applied to carried payload model poses.',
        ),
        DeclareLaunchArgument(
            'payload_pose_z_offset_m',
            default_value='0.0',
            description='Additional z offset applied to carried payload model poses.',
        ),
        DeclareLaunchArgument(
            'right_start_slot',
            default_value='2',
            description='Startup slot for the right-rail shuttle.',
        ),
        DeclareLaunchArgument(
            'right_start_slots',
            default_value='',
            description='Comma-separated startup slots for right-rail multi-shuttle mode.',
        ),
        DeclareLaunchArgument(
            'right_start_positions',
            default_value='',
            description=(
                'Optional comma-separated SEGMENT@S_RATIO positions for right shuttles.'
            ),
        ),
        DeclareLaunchArgument(
            'right_shuttle_count',
            default_value='0',
            description='Number of initial right-rail shuttles. Use 0 to start the rail with no shuttle.',
        ),
        DeclareLaunchArgument(
            'right_loaded_shuttles',
            default_value='',
            description='Comma-separated loaded right shuttles at startup, e.g. R2,4,all.',
        ),
        DeclareLaunchArgument(
            'left_start_slot',
            default_value='2',
            description='Startup slot for the left-rail shuttle.',
        ),
        DeclareLaunchArgument(
            'left_start_slots',
            default_value='',
            description='Comma-separated startup slots for left-rail multi-shuttle mode.',
        ),
        DeclareLaunchArgument(
            'left_start_positions',
            default_value='',
            description=(
                'Optional comma-separated SEGMENT@S_RATIO positions for left shuttles.'
            ),
        ),
        DeclareLaunchArgument(
            'left_shuttle_count',
            default_value='0',
            description='Number of initial left-rail shuttles. Use 0 to start the rail with no shuttle.',
        ),
        DeclareLaunchArgument(
            'left_loaded_shuttles',
            default_value='',
            description='Comma-separated loaded left shuttles at startup, e.g. L1,L4,all.',
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
