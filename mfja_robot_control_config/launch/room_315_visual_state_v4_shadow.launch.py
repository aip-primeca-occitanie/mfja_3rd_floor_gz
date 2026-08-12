"""Compare the explicit V3 rollback runtime with an isolated V4 shadow."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _runtime_launch(**arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mfja_robot_control_config'),
            '/launch/room_315_visual_state_runtime.launch.py',
        ]),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    v3 = _runtime_launch(
        # Keep the V3 rollback node name so its node-scoped YAML is applied
        # verbatim. The V4 node is the one deliberately renamed.
        node_name='room_315_visual_state_inference_node',
        runtime_config=LaunchConfiguration('v3_runtime_config'),
        runtime_generation='v3',
        runtime_mode='active',
        enable_camera_bridge=LaunchConfiguration('enable_camera_bridge'),
        use_sim_time=LaunchConfiguration('use_sim_time'),
        device=LaunchConfiguration('v3_device'),
        dry_run_state_fusion='true',
        plansys2_update_enabled='false',
    )
    v4 = _runtime_launch(
        node_name='room_315_visual_state_v4_shadow',
        runtime_config=LaunchConfiguration('v4_runtime_config'),
        runtime_generation='v4',
        runtime_mode='shadow',
        v4_promotion_manifest=LaunchConfiguration(
            'v4_promotion_manifest'
        ),
        v4_promotion_manifest_sha256=LaunchConfiguration(
            'v4_promotion_manifest_sha256'
        ),
        enable_camera_bridge='false',
        use_sim_time=LaunchConfiguration('use_sim_time'),
        device=LaunchConfiguration('v4_device'),
        dry_run_state_fusion='true',
        plansys2_update_enabled='false',
    )
    comparator = Node(
        package='mfja_robot_control_config',
        executable='room_315_visual_shadow_compare.py',
        name='room_315_visual_shadow_compare',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'v3_validation_topic': '/room_315/visual_state/validation',
            'v4_validation_topic': '/room_315/visual_state/shadow_v4/validation',
            'expected_v3_checkpoint_sha256': LaunchConfiguration(
                'expected_v3_checkpoint_sha256'
            ),
            'expected_v4_checkpoint_sha256': LaunchConfiguration(
                'expected_v4_checkpoint_sha256'
            ),
            'minimum_paired_frames': LaunchConfiguration(
                'minimum_paired_frames'
            ),
            'duration_s': LaunchConfiguration('duration_s'),
            'output_file': LaunchConfiguration('shadow_report_path'),
        }],
    )
    stop_after_comparison = RegisterEventHandler(OnProcessExit(
        target_action=comparator,
        on_exit=lambda event, _context: [EmitEvent(event=Shutdown(
            reason=(
                'V4 shadow comparison completed'
                if event.returncode == 0
                else 'V4 shadow comparison failed closed'
            ),
        ))],
    ))
    return LaunchDescription([
        DeclareLaunchArgument(
            'v3_runtime_config',
            default_value=[
                FindPackageShare('mfja_robot_control_config'),
                '/config/room_315_vla/visual_state_runtime_v3_rollback.yaml',
            ],
        ),
        DeclareLaunchArgument(
            'v4_runtime_config',
            description='Immutable candidate V4 ROS parameter YAML.',
        ),
        DeclareLaunchArgument(
            'v4_promotion_manifest',
            description=(
                'Immutable V4 promotion manifest passed explicitly because '
                'the shadow node is deliberately renamed.'
            ),
        ),
        DeclareLaunchArgument(
            'v4_promotion_manifest_sha256',
            description='Expected SHA-256 of the immutable V4 manifest.',
        ),
        DeclareLaunchArgument(
            'shadow_report_path',
            description='New path for the immutable shadow comparison report.',
        ),
        DeclareLaunchArgument(
            'expected_v3_checkpoint_sha256',
            default_value=(
                '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
            ),
        ),
        DeclareLaunchArgument(
            'expected_v4_checkpoint_sha256',
            default_value=(
                '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
            ),
        ),
        DeclareLaunchArgument('minimum_paired_frames', default_value='20'),
        DeclareLaunchArgument('duration_s', default_value='30.0'),
        DeclareLaunchArgument(
            'enable_camera_bridge',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'v3_device',
            default_value='cuda',
            choices=['auto', 'cpu', 'cuda'],
        ),
        DeclareLaunchArgument(
            'v4_device',
            default_value='cuda',
            choices=['auto', 'cpu', 'cuda'],
        ),
        v3,
        v4,
        comparator,
        stop_after_comparison,
    ])
