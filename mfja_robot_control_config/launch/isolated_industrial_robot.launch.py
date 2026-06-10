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


DESCRIPTION_PACKAGE = 'mfja_3rd_floor_description'
CONTROL_CONFIG_PACKAGE = 'mfja_robot_control_config'
INDUSTRIAL_MODELS = {
    'kuka_kr6r900sixx',
    'staubli_tx2_60l',
    'yaskawa_hc10',
    'yaskawa_hc10dt',
}

ISOLATED_LAYOUTS = {
    'kuka_kr6r900sixx': {
        'table_model': 'kuka_table',
        'table_pose': (0.0, 0.0, 0.5, -1.57, 0.0, 1.57),
        'robot_pose': (-0.027, 0.0, 1.0, 0.0, 0.0, 3.14),
    },
    'staubli_tx2_60l': {
        'table_model': 'staubli_table',
        'table_pose': (0.0, 0.0, 0.5, -1.57, 0.0, 1.57),
        'robot_pose': (-0.256, 0.0, 1.0, 0.0, 0.0, 1.57),
    },
    'yaskawa_hc10': {
        'table_model': 'table_yaskawa',
        'table_pose': (0.0, 0.0, 0.5, 0.0, 0.0, 1.57),
        'robot_pose': (0.207, -0.03, 0.62, 0.0, 0.0, 1.57),
    },
    'yaskawa_hc10dt': {
        'table_model': 'table_yaskawa',
        'table_pose': (0.0, 0.0, 0.5, 0.0, 0.0, 1.57),
        'robot_pose': (-0.246, -0.03, 0.62, 0.0, 0.0, 1.57),
    },
}


def _robot_shortcuts(robot, index):
    name = str(robot.get('name', '')).strip()
    model = str(robot.get('model', '')).strip().lower()
    shortcuts = {str(index), name.lower(), model}

    if name:
        base_name = name.lower().rstrip('0123456789').rstrip('_')
        if base_name:
            shortcuts.add(base_name)

    if model.startswith('kuka_'):
        shortcuts.add('kuka')
    elif model.startswith('staubli_'):
        shortcuts.add('staubli')
    elif model == 'yaskawa_hc10':
        shortcuts.add('hc10')
    elif model == 'yaskawa_hc10dt':
        shortcuts.add('hc10dt')

    return shortcuts


def _load_robot_config(config_path):
    with open(config_path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    return config.get('robots', [])


def _select_single_industrial_robot(config_path, selector):
    robots = _load_robot_config(config_path)
    exact_name_map = {}
    selector_map = {}
    for index, robot in enumerate(robots, start=1):
        model = str(robot.get('model', '')).strip()
        if model not in INDUSTRIAL_MODELS:
            continue

        name = str(robot.get('name', '')).strip()
        if not name:
            raise RuntimeError(
                f'Robot entry #{index} in "{config_path}" is missing name.'
            )

        exact_name_map[name.lower()] = robot
        for shortcut in _robot_shortcuts(robot, index):
            selector_map.setdefault(shortcut, []).append(robot)

    normalized = selector.strip().lower()
    candidates = (
        [exact_name_map[normalized]]
        if normalized in exact_name_map
        else selector_map.get(normalized, [])
    )

    available = ', '.join(
        f'{robot["name"]} ({robot.get("model", "")})'
        for robot in exact_name_map.values()
    )
    if not candidates:
        raise RuntimeError(
            f'Unknown industrial robot selector {selector!r}. '
            f'Available: {available}. Shortcuts: kuka, staubli, hc10, hc10dt.'
        )
    if len(candidates) > 1:
        names = ', '.join(str(robot['name']) for robot in candidates)
        raise RuntimeError(
            f'Ambiguous industrial robot selector {selector!r}: {names}. '
            'Use the exact robot name.'
        )
    return candidates[0]


def _get_world_entity_name(world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_element = root.find('world')
    if world_element is None:
        raise RuntimeError(f'No <world> element found in: {world_path}')
    return world_element.attrib.get('name', 'default')


def _resolve_robot_assets(description_pkg_path, model_name):
    model_sdf = os.path.join(description_pkg_path, 'models', model_name, 'model.sdf')
    urdf_path = os.path.join(description_pkg_path, 'urdf', f'{model_name}.urdf')
    if not os.path.exists(model_sdf):
        raise RuntimeError(f'Missing model file for "{model_name}": {model_sdf}')
    if not os.path.exists(urdf_path):
        raise RuntimeError(f'Missing URDF file for "{model_name}": {urdf_path}')
    return model_sdf, urdf_path


def _make_bridge_yaml(robot_name, world_name):
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
            'ros_type_name': 'std_msgs/msg/Float32',
            'gz_type_name': 'gz.msgs.Float',
            'direction': 'GZ_TO_ROS',
        },
    ]
    output_path = os.path.join(tempfile.gettempdir(), f'{robot_name}_isolated_bridge.yaml')
    with open(output_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(bridge_config, stream, sort_keys=False)
    return output_path


def _create_entity_node(entity_name, sdf_path, pose, world_name):
    x, y, z, roll, pitch, yaw = pose
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{entity_name}',
        output='screen',
        parameters=[{
            'world': world_name,
            'file': sdf_path,
            'name': entity_name,
            'allow_renaming': False,
            'x': x,
            'y': y,
            'z': z,
            'R': roll,
            'P': pitch,
            'Y': yaw,
        }],
    )


def _launch_setup(context, *args, **kwargs):
    description_pkg_path = get_package_share_directory(DESCRIPTION_PACKAGE)
    control_pkg_path = get_package_share_directory(CONTROL_CONFIG_PACKAGE)
    world_name = LaunchConfiguration('world_name').perform(context)
    world_path = os.path.join(description_pkg_path, 'worlds', f'{world_name}.world')
    world_entity_name = _get_world_entity_name(world_path)
    gui_config_file = LaunchConfiguration('gui_config').perform(context).strip()
    if not os.path.isabs(gui_config_file):
        gui_config_file = os.path.join(control_pkg_path, gui_config_file)

    gz_partition = LaunchConfiguration('gz_partition').perform(context).strip()
    enable_gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    start_paused = LaunchConfiguration('start_paused').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    robot_config = LaunchConfiguration('robot_config').perform(context)
    robot_selector = LaunchConfiguration('robot').perform(context)

    if not os.path.isabs(robot_config):
        robot_config = os.path.join(control_pkg_path, robot_config)

    robot = _select_single_industrial_robot(robot_config, robot_selector)
    robot_name = str(robot['name'])
    model_name = str(robot['model'])
    layout = ISOLATED_LAYOUTS[model_name]

    robot_sdf, urdf_path = _resolve_robot_assets(description_pkg_path, model_name)
    table_sdf = os.path.join(
        description_pkg_path,
        'models',
        layout['table_model'],
        'model.sdf',
    )
    if not os.path.exists(table_sdf):
        raise RuntimeError(f'Missing table model file: {table_sdf}')

    with open(urdf_path, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    bridge_file = _make_bridge_yaml(robot_name, world_entity_name)
    model_path = os.path.join(description_pkg_path, 'models')
    resource_path = model_path
    if 'GZ_SIM_MODEL_PATH' in environ:
        model_path += pathsep + environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GZ_SIM_RESOURCE_PATH']

    gz_server_args = f'-s {world_path}' if start_paused else f'-r -s {world_path}'
    actions = [
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
                'gz_args': gz_server_args,
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
                    'gz_args': f'-g --gui-config {gui_config_file}',
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
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{robot_name}_bridge',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={bridge_file}'],
        ),
        _create_entity_node(
            entity_name=f'{robot_name}_table',
            sdf_path=table_sdf,
            pose=layout['table_pose'],
            world_name=world_entity_name,
        ),
        _create_entity_node(
            entity_name=robot_name,
            sdf_path=robot_sdf,
            pose=layout['robot_pose'],
            world_name=world_entity_name,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': f'{robot_name}/',
            }],
        ),
    ]
    actions.append(TimerAction(period=3.0, actions=spawn_actions))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='kuka',
            description='One industrial robot selector: kuka, staubli, hc10, hc10dt, or exact name.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/robots_room_315_only.yaml',
            description='Absolute path or path relative to mfja_robot_control_config.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='isolated_industrial_robot',
            description='Minimal world file from mfja_3rd_floor_description/worlds.',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'isolated_industrial_robot_{os.getpid()}',
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
            'gui_config',
            default_value='config/mfja_light.gui.config',
            description=(
                'Gazebo GUI config path. Relative paths are resolved inside '
                'mfja_robot_control_config.'
            ),
        ),
        DeclareLaunchArgument(
            'start_paused',
            default_value='false',
            choices=['true', 'false'],
            description='Start Gazebo paused so the user can press play manually.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
