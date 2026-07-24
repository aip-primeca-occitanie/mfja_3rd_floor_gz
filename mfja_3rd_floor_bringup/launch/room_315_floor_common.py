import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_launch_bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _clear_vla_obstacle_pose_cache(context, *args, **kwargs):
    clear_cache = LaunchConfiguration(
        'room315_clear_vla_obstacle_pose_cache'
    ).perform(context)
    if not _as_launch_bool(clear_cache):
        return []

    pose_file = Path(
        LaunchConfiguration('room315_vla_obstacle_pose_file').perform(context)
    ).expanduser()
    try:
        pose_file.unlink(missing_ok=True)
    except OSError as exc:
        return [
            LogInfo(msg=f'Could not clear VLA obstacle pose cache {pose_file}: {exc}')
        ]
    return [LogInfo(msg=f'Cleared VLA obstacle pose cache: {pose_file}')]


FLOOR_PROFILES = {
    'room_315_only': {
        'world_name': 'room_315_only',
        'robot_config': 'config/robots_room_315_only.yaml',
        'gz_partition_prefix': 'room_315_only',
        'start_paused': 'false',
        'start_paused_description': (
            'Start Gazebo paused so the user can press play manually. '
            'Room 315 defaults to running so shuttle timers publish visible poses.'
        ),
        'gui_config': None,
    },
    'full_floor': {
        'world_name': 'mfja_3rd_floor',
        'robot_config': 'config/robots.yaml',
        'gz_partition_prefix': 'mfja_3rd_floor',
        'start_paused': 'true',
        'start_paused_description': (
            'Start Gazebo paused so the user can press play manually.'
        ),
        'gui_config': 'config/mfja_light.gui.config',
    },
}


def generate_floor_launch_description(profile_name):
    try:
        profile = FLOOR_PROFILES[profile_name]
    except KeyError as exc:
        raise ValueError(f'Unknown floor launch profile: {profile_name!r}') from exc

    control_pkg_path = get_package_share_directory('mfja_robot_control_config')
    base_launch = os.path.join(control_pkg_path, 'launch', 'multi_robot_sim.launch.py')
    shuttles_launch = os.path.join(
        control_pkg_path,
        'launch',
        'room_315_dual_kinematic_shuttles.launch.py',
    )
    vla_launch = os.path.join(
        control_pkg_path,
        'launch',
        'room_315_vla_supervisor.launch.py',
    )

    launch_arguments = [
        DeclareLaunchArgument(
            'world_name',
            default_value=profile['world_name'],
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
            default_value=profile['robot_config'],
            description='Robot spawn YAML relative to mfja_robot_control_config.',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'{profile["gz_partition_prefix"]}_{os.getpid()}',
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
            default_value=profile['start_paused'],
            choices=['true', 'false'],
            description=profile['start_paused_description'],
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
            'room315_visual_debug_colors',
            default_value='true',
            choices=['true', 'false'],
            description='Use switch/shuttle debug colors; false keeps rail-like colors for VLA data.',
        ),
        DeclareLaunchArgument(
            'enable_room315_vla_obstacles',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Load Room 315 VLA removable obstacle markers. Defaults to false '
                'so Gazebo starts without obstacles unless explicitly requested.'
            ),
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
            'room315_right_start_slots',
            default_value='',
            description='Comma-separated startup slots for Room 315 right rail shuttles.',
        ),
        DeclareLaunchArgument(
            'room315_right_start_positions',
            default_value='',
            description=(
                'Optional comma-separated SEGMENT@S_RATIO positions for Room 315 '
                'right rail shuttles.'
            ),
        ),
        DeclareLaunchArgument(
            'room315_left_start_slot',
            default_value='2',
            description='Startup slot for the Room 315 left rail shuttle.',
        ),
        DeclareLaunchArgument(
            'room315_left_start_slots',
            default_value='',
            description='Comma-separated startup slots for Room 315 left rail shuttles.',
        ),
        DeclareLaunchArgument(
            'room315_left_start_positions',
            default_value='',
            description=(
                'Optional comma-separated SEGMENT@S_RATIO positions for Room 315 '
                'left rail shuttles.'
            ),
        ),
        DeclareLaunchArgument(
            'room315_shuttle_speed',
            default_value='0.2',
            description='Common Room 315 shuttle speed in meters per second.',
        ),
        DeclareLaunchArgument(
            'room315_falling_stop_offset_m',
            default_value='0.0',
            description='Distance before an invalid route endpoint where Room 315 shuttles enter FALLING.',
        ),
        DeclareLaunchArgument(
            'room315_shuttle_collision_distance_m',
            default_value='0.33',
            description='Minimum separation between Room 315 shuttles before collision avoidance stops motion.',
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
            'room315_sensor_publish_rate_hz',
            default_value='30.0',
            description='Room 315 binary sensor feedback publish rate.',
        ),
        DeclareLaunchArgument(
            'room315_sync_sensor_feedback_to_motion_tick',
            default_value='true',
            choices=['true', 'false'],
            description='Publish Room 315 sensor feedback from the same tick that updates shuttle motion.',
        ),
        DeclareLaunchArgument(
            'room315_gazebo_set_pose_rate_hz',
            default_value='30.0',
            description='Room 315 visible shuttle pose update rate in Gazebo.',
        ),
        DeclareLaunchArgument(
            'room315_show_device_markers',
            default_value='true',
            choices=['true', 'false'],
            description='Show Room 315 position sensor and stopper markers.',
        ),
        DeclareLaunchArgument(
            'room315_enable_payload_visuals',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn carried payload boxes for loaded Room 315 shuttles.',
        ),
        DeclareLaunchArgument(
            'room315_payload_pose_x_offset_m',
            default_value='-0.08',
            description='Additional x offset for carried Room 315 payload boxes.',
        ),
        DeclareLaunchArgument(
            'room315_right_loaded_shuttles',
            default_value='',
            description='Comma-separated loaded right shuttles at startup, e.g. R2,4,all.',
        ),
        DeclareLaunchArgument(
            'room315_left_loaded_shuttles',
            default_value='',
            description='Comma-separated loaded left shuttles at startup, e.g. L1,L4,all.',
        ),
        DeclareLaunchArgument(
            'room315_sensor_marker_visual_hold_s',
            default_value='0.35',
            description='Minimum seconds a crossed Room 315 sensor marker stays green.',
        ),
        DeclareLaunchArgument(
            'enable_room315_vla',
            default_value='false',
            choices=['true', 'false'],
            description='Start the Room 315 VLA camera bridge and action supervisor.',
        ),
        DeclareLaunchArgument(
            'room315_clear_vla_obstacle_pose_cache',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Clear the VLA obstacle pose cache at simulation startup so '
                'stale obstacle moves from a previous Gazebo run are not reused.'
            ),
        ),
        DeclareLaunchArgument(
            'room315_vla_obstacle_pose_file',
            default_value='~/.ros/room315_vla_obstacles.json',
            description='Pose cache written by room_315_vla_obstacle_tool.py.',
        ),
        DeclareLaunchArgument(
            'enable_room315_vla_camera_bridge',
            default_value='true',
            choices=['true', 'false'],
            description='Bridge the Room 315 VLA rail-focused RGB-D cameras to ROS.',
        ),
        DeclareLaunchArgument(
            'enable_room315_vla_dataset_recorder',
            default_value='false',
            choices=['true', 'false'],
            description='Record Room 315 visual-state episodes for LeRobot conversion.',
        ),
        DeclareLaunchArgument(
            'room315_vla_dataset_dir',
            default_value='~/.ros/room315_visual_state_datasets/demo',
            description='Output directory for Room 315 visual-state demonstrations.',
        ),
        DeclareLaunchArgument(
            'room315_vla_dataset_sample_period_s',
            default_value='0.2',
            description='Sample period in seconds for Room 315 VLA demonstrations.',
        ),
    ]
    base_launch_arguments = {
        'world_name': LaunchConfiguration('world_name'),
        'robot_config': LaunchConfiguration('robot_config'),
        'robots': LaunchConfiguration('robots'),
        'gz_partition': LaunchConfiguration('gz_partition'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'gui': LaunchConfiguration('gui'),
        'start_paused': LaunchConfiguration('start_paused'),
        'initial_loop_mode': LaunchConfiguration('initial_loop_mode'),
        'pause_during_switch_update': LaunchConfiguration('pause_during_switch_update'),
        'visual_debug_colors': LaunchConfiguration('room315_visual_debug_colors'),
        'enable_room315_vla_obstacles': LaunchConfiguration(
            'enable_room315_vla_obstacles'
        ),
    }
    if profile['gui_config']:
        launch_arguments.insert(
            7,
            DeclareLaunchArgument(
                'gui_config',
                default_value=profile['gui_config'],
                description=(
                    'Gazebo GUI config path. Relative paths are resolved inside '
                    'mfja_robot_control_config.'
                ),
            ),
        )
        base_launch_arguments['gui_config'] = LaunchConfiguration('gui_config')

    return LaunchDescription([
        *launch_arguments,
        OpaqueFunction(function=_clear_vla_obstacle_pose_cache),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments=base_launch_arguments.items(),
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
                        'falling_stop_offset_m': LaunchConfiguration(
                            'room315_falling_stop_offset_m'
                        ),
                        'shuttle_collision_distance_m': LaunchConfiguration(
                            'room315_shuttle_collision_distance_m'
                        ),
                        'start_enabled': LaunchConfiguration(
                            'room315_shuttles_start_enabled'
                        ),
                        'switch_motion_delay_s': LaunchConfiguration(
                            'room315_switch_motion_delay_s'
                        ),
                        'stopper_motion_delay_s': LaunchConfiguration(
                            'room315_stopper_motion_delay_s'
                        ),
                        'sensor_publish_rate_hz': LaunchConfiguration(
                            'room315_sensor_publish_rate_hz'
                        ),
                        'sync_sensor_feedback_to_motion_tick': LaunchConfiguration(
                            'room315_sync_sensor_feedback_to_motion_tick'
                        ),
                        'gazebo_set_pose_rate_hz': LaunchConfiguration(
                            'room315_gazebo_set_pose_rate_hz'
                        ),
                        'show_device_markers': LaunchConfiguration(
                            'room315_show_device_markers'
                        ),
                        'sensor_marker_visual_hold_s': LaunchConfiguration(
                            'room315_sensor_marker_visual_hold_s'
                        ),
                        'visual_debug_colors': LaunchConfiguration(
                            'room315_visual_debug_colors'
                        ),
                        'enable_payload_visuals': LaunchConfiguration(
                            'room315_enable_payload_visuals'
                        ),
                        'payload_pose_x_offset_m': LaunchConfiguration(
                            'room315_payload_pose_x_offset_m'
                        ),
                        'right_loaded_shuttles': LaunchConfiguration(
                            'room315_right_loaded_shuttles'
                        ),
                        'left_loaded_shuttles': LaunchConfiguration(
                            'room315_left_loaded_shuttles'
                        ),
                        'right_start_slot': LaunchConfiguration(
                            'room315_right_start_slot'
                        ),
                        'right_start_slots': LaunchConfiguration(
                            'room315_right_start_slots'
                        ),
                        'right_start_positions': LaunchConfiguration(
                            'room315_right_start_positions'
                        ),
                        'left_start_slot': LaunchConfiguration(
                            'room315_left_start_slot'
                        ),
                        'left_start_slots': LaunchConfiguration(
                            'room315_left_start_slots'
                        ),
                        'left_start_positions': LaunchConfiguration(
                            'room315_left_start_positions'
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
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(vla_launch),
                    condition=IfCondition(LaunchConfiguration('enable_room315_vla')),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'enable_camera_bridge': LaunchConfiguration(
                            'enable_room315_vla_camera_bridge'
                        ),
                        'enable_supervisor': 'true',
                        'enable_dataset_recorder': LaunchConfiguration(
                            'enable_room315_vla_dataset_recorder'
                        ),
                        'dataset_dir': LaunchConfiguration(
                            'room315_vla_dataset_dir'
                        ),
                        'dataset_sample_period_s': LaunchConfiguration(
                            'room315_vla_dataset_sample_period_s'
                        ),
                    }.items(),
                ),
            ],
        ),
    ])
