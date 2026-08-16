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
        'runtime_generation': LaunchConfiguration('runtime_generation'),
        'dry_run_state_fusion': LaunchConfiguration('dry_run_state_fusion'),
        'plansys2_update_enabled': LaunchConfiguration(
            'plansys2_update_enabled'
        ),
    }
    for argument, parameter in (
        ('runtime_mode', 'runtime_mode'),
        ('v4_promotion_manifest', 'v4_promotion_manifest_path'),
        (
            'v4_promotion_manifest_sha256',
            'expected_v4_promotion_manifest_sha256',
        ),
    ):
        value = LaunchConfiguration(argument).perform(context).strip()
        if value:
            overrides[parameter] = value
    return [Node(
        package='mfja_robot_control_config',
        executable='room_315_visual_state_inference_node.py',
        name=LaunchConfiguration('node_name'),
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
            'node_name',
            default_value='room_315_visual_state_inference_node',
        ),
        DeclareLaunchArgument(
            'runtime_generation',
            default_value='v4',
            choices=['v4'],
            description='The deployable visual runtime is V4 only.',
        ),
        DeclareLaunchArgument(
            'runtime_mode',
            default_value='',
            description='Optional active/shadow override; empty keeps YAML.',
        ),
        DeclareLaunchArgument(
            'v4_promotion_manifest',
            default_value='',
            description='Optional immutable V4 promotion-manifest path.',
        ),
        DeclareLaunchArgument(
            'v4_promotion_manifest_sha256',
            default_value='',
            description='Expected SHA-256 of the V4 promotion manifest.',
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
