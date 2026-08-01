from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_inference(context):
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'device': LaunchConfiguration('device'),
        'dry_run_state_fusion': LaunchConfiguration('dry_run_state_fusion'),
        'plansys2_update_enabled': LaunchConfiguration(
            'plansys2_update_enabled'
        ),
    }
    checkpoint = LaunchConfiguration('checkpoint_path').perform(context).strip()
    sidecars = LaunchConfiguration('sidecar_directory').perform(context).strip()
    if checkpoint:
        overrides['checkpoint_path'] = checkpoint
    if sidecars:
        overrides['sidecar_directory'] = sidecars
    for argument, parameter in (
        ('checkpoint_sha256', 'expected_checkpoint_sha256'),
        ('target_stats_sha256', 'expected_target_stats_sha256'),
        ('vectorizer_sha256', 'expected_vectorizer_sha256'),
        ('training_config_sha256', 'expected_training_config_sha256'),
        ('run_metadata_sha256', 'expected_run_metadata_sha256'),
        (
            'runtime_configuration_sha256',
            'expected_runtime_configuration_sha256',
        ),
    ):
        value = LaunchConfiguration(argument).perform(context).strip()
        if value:
            overrides[parameter] = value
    return [Node(
        package='mfja_robot_control_config',
        executable='room_315_visual_state_inference_node.py',
        name='room_315_visual_state_inference_node',
        output='screen',
        parameters=[LaunchConfiguration('runtime_config'), overrides],
    )]


def generate_launch_description():
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='room315_visual_runtime_camera_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera_bridge')),
        arguments=[
            '/room_315/vla/right_rail_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/room_315/vla/left_rail_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_camera_bridge',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'runtime_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mfja_robot_control_config'),
                'config',
                'room_315_vla',
                'visual_state_runtime.yaml',
            ]),
            description='Installed or source-tree visual runtime YAML path.',
        ),
        DeclareLaunchArgument(
            'checkpoint_path',
            default_value='',
            description='Optional override; empty keeps the YAML value.',
        ),
        DeclareLaunchArgument(
            'sidecar_directory',
            default_value='',
            description='Optional override; empty keeps the YAML value.',
        ),
        DeclareLaunchArgument('checkpoint_sha256', default_value=''),
        DeclareLaunchArgument('target_stats_sha256', default_value=''),
        DeclareLaunchArgument('vectorizer_sha256', default_value=''),
        DeclareLaunchArgument('training_config_sha256', default_value=''),
        DeclareLaunchArgument('run_metadata_sha256', default_value=''),
        DeclareLaunchArgument(
            'runtime_configuration_sha256',
            default_value='',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='auto',
            choices=['auto', 'cpu', 'cuda'],
        ),
        DeclareLaunchArgument(
            'dry_run_state_fusion',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'plansys2_update_enabled',
            default_value='false',
            choices=['true', 'false'],
        ),
        camera_bridge,
        OpaqueFunction(function=_launch_inference),
    ])
