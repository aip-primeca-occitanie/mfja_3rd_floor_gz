from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='room315_vla_camera_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera_bridge')),
        arguments=[
            '/room_315/vla/right_rail_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/room_315/vla/right_rail_rgbd/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/room_315/vla/right_rail_rgbd/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/room_315/vla/right_rail_rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/room_315/vla/left_rail_rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/room_315/vla/left_rail_rgbd/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/room_315/vla/left_rail_rgbd/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/room_315/vla/left_rail_rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
    )

    supervisor = Node(
        package='mfja_robot_control_config',
        executable='room_315_vla_supervisor.py',
        name='room_315_vla_supervisor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_supervisor')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'command_topic': LaunchConfiguration('command_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'emergency_stop_topic': LaunchConfiguration('emergency_stop_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
        }],
    )

    real_vla_agent = Node(
        package='mfja_robot_control_config',
        executable='room_315_real_vla_agent.py',
        name='room_315_real_vla_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_real_vla_agent')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'provider': LaunchConfiguration('vla_agent_provider'),
            'user_goal_topic': LaunchConfiguration('user_goal_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'command_topic': LaunchConfiguration('command_topic'),
            'agent_status_topic': LaunchConfiguration('agent_status_topic'),

            'http_endpoint': LaunchConfiguration('vla_agent_http_endpoint'),
            'decision_period_s': LaunchConfiguration('vla_agent_decision_period_s'),
            'request_timeout_s': LaunchConfiguration('vla_agent_request_timeout_s'),
        }],
    )

    dataset_recorder = Node(
        package='mfja_robot_control_config',
        executable='room_315_vla_dataset_recorder.py',
        name='room_315_vla_dataset_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_dataset_recorder')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dataset_dir': LaunchConfiguration('dataset_dir'),
            'image_topic': LaunchConfiguration('image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'status_topic': LaunchConfiguration('status_topic'),
            'goal_topic': LaunchConfiguration('user_goal_topic'),
            'command_topic': LaunchConfiguration('command_topic'),
            'control_topic': LaunchConfiguration('episode_control_topic'),
            'recorder_status_topic': LaunchConfiguration('dataset_status_topic'),
            'sample_period_s': LaunchConfiguration('dataset_sample_period_s'),
            'auto_start_on_goal': LaunchConfiguration('dataset_auto_start_on_goal'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'enable_camera_bridge',
            default_value='true',
            choices=['true', 'false'],
            description='Bridge the Room 315 VLA rail-focused RGB-D cameras to ROS.',
        ),
        DeclareLaunchArgument(
            'enable_supervisor',
            default_value='true',
            choices=['true', 'false'],
            description='Start the Room 315 VLA action supervisor.',
        ),
        DeclareLaunchArgument(
            'enable_real_vla_agent',
            default_value='false',
            choices=['true', 'false'],
            description='Start the optional model-facing VLA agent.',
        ),
        DeclareLaunchArgument(
            'enable_dataset_recorder',
            default_value='false',
            choices=['true', 'false'],
            description='Record Room 315 VLA episodes for SmolVLA/LeRobot fine-tuning.',
        ),
        DeclareLaunchArgument(
            'vla_agent_provider',
            default_value='http',
            description='VLA agent provider (only http is supported now).',
        ),
        DeclareLaunchArgument(
            'vla_agent_http_endpoint',
            default_value='',
            description='HTTP endpoint for provider:=http.',
        ),
        DeclareLaunchArgument(
            'vla_agent_decision_period_s',
            default_value='0.5',
            description='How often the VLA agent checks for new user goals.',
        ),
        DeclareLaunchArgument(
            'vla_agent_request_timeout_s',
            default_value='20.0',
            description='Timeout in seconds for external VLA provider requests.',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description='Optional YAML config path for Room 315 VLA supervisor defaults.',
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/room_315/vla/command',
            description='std_msgs/String command input for text or JSON VLA actions.',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/room_315/vla/status',
            description='std_msgs/String JSON status output from the VLA supervisor.',
        ),
        DeclareLaunchArgument(
            'user_goal_topic',
            default_value='/room_315/vla/user_goal',
            description='std_msgs/String high-level goal input for the VLA agent.',
        ),
        DeclareLaunchArgument(
            'agent_status_topic',
            default_value='/room_315/vla/agent_status',
            description='std_msgs/String JSON status output from the VLA agent.',
        ),
        DeclareLaunchArgument(
            'episode_control_topic',
            default_value='/room_315/vla/episode_control',
            description='std_msgs/String dataset episode control input.',
        ),
        DeclareLaunchArgument(
            'dataset_status_topic',
            default_value='/room_315/vla/dataset_status',
            description='std_msgs/String JSON status output from the VLA dataset recorder.',
        ),
        DeclareLaunchArgument(
            'dataset_dir',
            default_value='~/.ros/room315_vla_datasets/smolvla_demo',
            description='Output directory for recorded VLA demonstrations.',
        ),
        DeclareLaunchArgument(
            'dataset_sample_period_s',
            default_value='0.2',
            description='Sample period in seconds for recorded VLA frames.',
        ),
        DeclareLaunchArgument(
            'dataset_auto_start_on_goal',
            default_value='true',
            choices=['true', 'false'],
            description='Start a new dataset episode when a VLA user goal arrives.',
        ),
        DeclareLaunchArgument(
            'emergency_stop_topic',
            default_value='/room_315/vla/emergency_stop',
            description='std_msgs/Bool virtual emergency stop input.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='',
            description='Optional legacy primary VLA image topic. Empty disables it.',
        ),
        DeclareLaunchArgument(
            'right_image_topic',
            default_value='/room_315/vla/right_rail_rgbd/image',
            description='ROS image topic for the independent right-rail VLA camera.',
        ),
        DeclareLaunchArgument(
            'left_image_topic',
            default_value='/room_315/vla/left_rail_rgbd/image',
            description='ROS image topic for the independent left-rail VLA camera.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='',
            description='Optional legacy primary VLA camera info topic. Empty disables it.',
        ),
        DeclareLaunchArgument(
            'right_camera_info_topic',
            default_value='/room_315/vla/right_rail_rgbd/camera_info',
            description='ROS camera info topic for the independent right-rail VLA camera.',
        ),
        DeclareLaunchArgument(
            'left_camera_info_topic',
            default_value='/room_315/vla/left_rail_rgbd/camera_info',
            description='ROS camera info topic for the independent left-rail VLA camera.',
        ),
        camera_bridge,
        supervisor,
        real_vla_agent,
        dataset_recorder,
    ])
