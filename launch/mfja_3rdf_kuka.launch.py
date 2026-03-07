import os
import tempfile
import xml.etree.ElementTree as ET
from os import environ, pathsep

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MOBILE_MODELS = {'tiago'}


def _load_robots(config_path):
    with open(config_path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}

    robots = [r for r in config.get('robots', []) if r.get('enabled', True)]
    if not robots:
        raise RuntimeError(
            f'No enabled robots in "{config_path}". '
            'Set at least one robot with enabled: true.'
        )

    return robots


def _make_bridge_yaml(robot_name, world_name, model_name):
    bridge_config = [
        {
            'ros_topic_name': f'/{robot_name}/joint_trajectory',
            'gz_topic_name': f'/model/{robot_name}/joint_trajectory',
            'ros_type_name': 'trajectory_msgs/msg/JointTrajectory',
            'gz_type_name': 'gz.msgs.JointTrajectory',
            'direction': 'ROS_TO_GZ',
        },
        {
            'ros_topic_name': f'/{robot_name}/joint_states',
            'gz_topic_name': f'/world/{world_name}/model/{robot_name}/joint_state',
            'ros_type_name': 'sensor_msgs/msg/JointState',
            'gz_type_name': 'gz.msgs.Model',
            'direction': 'GZ_TO_ROS',
        },
        {
            'ros_topic_name': f'/{robot_name}/joint_trajectory_progress',
            'gz_topic_name': f'/model/{robot_name}/joint_trajectory_progress',
            'ros_type_name': 'std_msgs/msg/Float64',
            'gz_type_name': 'gz.msgs.Double',
            'direction': 'GZ_TO_ROS',
        },
    ]

    if model_name in MOBILE_MODELS:
        bridge_config.extend([
            {
                'ros_topic_name': f'/{robot_name}/cmd_vel',
                'gz_topic_name': f'/model/{robot_name}/cmd_vel',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
            },
            {
                'ros_topic_name': f'/{robot_name}/odom',
                'gz_topic_name': f'/model/{robot_name}/odom',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{robot_name}/tf',
                'gz_topic_name': f'/model/{robot_name}/tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        ])

    output_path = os.path.join(
        tempfile.gettempdir(), f'{robot_name}_bridge.yaml')
    with open(output_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(bridge_config, stream, sort_keys=False)

    return output_path


def _materialize_mobile_model_sdf(model_sdf_path, robot_name):
    with open(model_sdf_path, 'r', encoding='utf-8') as infp:
        sdf_text = infp.read()

    replacements = {
        '<topic>cmd_vel</topic>': f'<topic>/model/{robot_name}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>': f'<odom_topic>/model/{robot_name}/odom</odom_topic>',
        '<tf_topic>tf</tf_topic>': f'<tf_topic>/model/{robot_name}/tf</tf_topic>',
    }

    for source, target in replacements.items():
        if source not in sdf_text:
            raise RuntimeError(
                f'Expected token "{source}" not found in mobile model SDF: {model_sdf_path}'
            )
        sdf_text = sdf_text.replace(source, target, 1)

    output_path = os.path.join(
        tempfile.gettempdir(), f'{robot_name}_mobile_model.sdf')
    with open(output_path, 'w', encoding='utf-8') as outfp:
        outfp.write(sdf_text)

    return output_path


def _get_world_entity_name(world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_element = root.find('world')
    if world_element is None:
        raise RuntimeError(f'No <world> element found in: {world_path}')
    return world_element.attrib.get('name', 'default')


def _resolve_robot_assets(pkg_path, model_name):
    model_sdf = os.path.join(pkg_path, 'models', model_name, 'model.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', f'{model_name}.urdf')

    if not os.path.exists(model_sdf):
        raise RuntimeError(
            f'Missing model file for "{model_name}": {model_sdf}. '
            'Add models/<model_name>/model.sdf.'
        )
    if not os.path.exists(urdf_path):
        raise RuntimeError(
            f'Missing URDF file for "{model_name}": {urdf_path}. '
            'Add urdf/<model_name>.urdf.'
        )

    return model_sdf, urdf_path


def _launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('mfja_3rd_floor_gz')
    world_file_name = LaunchConfiguration('world_name').perform(context)
    world = os.path.join(pkg_path, 'worlds', world_file_name + '.world')
    world_entity_name = _get_world_entity_name(world)
    gz_partition = LaunchConfiguration('gz_partition').perform(context).strip()
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true')
    enable_gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    robot_config = LaunchConfiguration('robot_config').perform(context)

    if not os.path.isabs(robot_config):
        robot_config = os.path.join(pkg_path, robot_config)

    robots = _load_robots(robot_config)
    model_path = os.path.join(pkg_path, 'models')
    resource_path = model_path

    if 'GZ_SIM_MODEL_PATH' in environ:
        model_path += pathsep + environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GZ_SIM_RESOURCE_PATH']

    robot_descriptions = {}

    actions = [
        # Use a dedicated transport partition per launch to avoid
        # cross-spawning into an already running Gazebo server.
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                )
            ),
            launch_arguments={
                'gz_args': f'-r -s {world}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ]

    if enable_gui:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py',
                    )
                ),
                launch_arguments={
                    'gz_args': '-g',
                    'on_exit_shutdown': 'true',
                }.items(),
            )
        )

    spawn_actions = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
    ]

    for robot in robots:
        robot_name = str(robot['name'])
        model_name = str(robot.get('model', 'kuka_kr6r900sixx'))
        x_pose = float(robot.get('x_pose', 0.0))
        y_pose = float(robot.get('y_pose', 0.0))
        z_pose = float(robot.get('z_pose', 0.0))
        yaw = float(robot.get('yaw', 0.0))
        model_sdf, urdf_path = _resolve_robot_assets(pkg_path, model_name)
        spawn_sdf = model_sdf
        frame_prefix = '' if model_name in MOBILE_MODELS else f'{robot_name}/'

        if model_name in MOBILE_MODELS:
            spawn_sdf = _materialize_mobile_model_sdf(model_sdf, robot_name)

        if urdf_path not in robot_descriptions:
            with open(urdf_path, 'r', encoding='utf-8') as infp:
                robot_descriptions[urdf_path] = infp.read()

        bridge_file = _make_bridge_yaml(robot_name, world_entity_name, model_name)

        spawn_actions.extend([
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{robot_name}',
                output='screen',
                parameters=[{
                    'world': world_entity_name,
                    'file': spawn_sdf,
                    'name': robot_name,
                    'allow_renaming': False,
                    'x': x_pose,
                    'y': y_pose,
                    'z': z_pose,
                    'Y': yaw,
                }],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                output='screen',
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_descriptions[urdf_path],
                    'frame_prefix': frame_prefix,
                }],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{robot_name}_bridge',
                output='screen',
                arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
            ),
        ])

    actions.append(TimerAction(period=3.0, actions=spawn_actions))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='mfja_3rd_floor',
            description='World file name from mfja_3rd_floor_gz/worlds (without extension).',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'mfja_3rd_floor_gz_{os.getpid()}',
            description='Gazebo transport partition used to isolate this launch instance.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/robots.yaml',
            description='Absolute path or package-relative path to robot spawn YAML.',
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
        OpaqueFunction(function=_launch_setup),
    ])
