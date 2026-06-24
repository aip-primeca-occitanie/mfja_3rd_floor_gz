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

TIAGO_WITH_ARM_ALIASES = {'tiago', 'tiago_with_arm', 'tiago_arm'}
TIAGO_BASE_ALIASES = {'tiago_base', 'tiago_no_arm', 'tiago_mobile_base'}
MODEL_ALIASES = {
    **{alias: 'tiago' for alias in TIAGO_WITH_ARM_ALIASES},
    **{alias: 'tiago_base' for alias in TIAGO_BASE_ALIASES},
}
MOBILE_MODELS = {'tiago', 'tiago_base'}
DESCRIPTION_PACKAGE = 'mfja_3rd_floor_description'
CONTROL_CONFIG_PACKAGE = 'mfja_robot_control_config'


def _canonical_model_name(model_name):
    normalized = str(model_name or '').strip().lower()
    return MODEL_ALIASES.get(normalized, normalized)


def _parse_bool_field(value, field_name, robot_name):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {'true', 'yes', 'y', '1', 'on'}:
            return True
        if normalized in {'false', 'no', 'n', '0', 'off'}:
            return False
    raise RuntimeError(
        f'Invalid boolean "{field_name}" value for robot "{robot_name}": '
        f'{value!r}. Use true/false.'
    )


def _resolve_robot_model_name(robot):
    robot_name = str(robot.get('name', '<unnamed>')).strip() or '<unnamed>'
    raw_model_name = str(robot.get('model', 'kuka_kr6r900sixx')).strip().lower()
    model_name = _canonical_model_name(raw_model_name)

    if model_name not in MOBILE_MODELS:
        return model_name

    variant = str(
        robot.get('variant', robot.get('type', robot.get('tiago_variant', '')))
    ).strip().lower().replace('-', '_')
    if variant in {'base', 'mobile_base', 'no_arm', 'without_arm', 'armless'}:
        return 'tiago_base'
    if variant in {'arm', 'with_arm', 'full', 'manipulator'}:
        return 'tiago'

    for arm_field in ('with_arm', 'has_arm', 'arm'):
        if arm_field in robot:
            return (
                'tiago'
                if _parse_bool_field(robot[arm_field], arm_field, robot_name)
                else 'tiago_base'
            )

    return model_name


def _parse_selected_robots(raw_value):
    selected = (raw_value or '').strip()
    if not selected:
        return None
    if selected.lower() == 'all':
        return 'all'
    if selected.lower() == 'none':
        return 'none'
    return [token.strip() for token in selected.split(',') if token.strip()]


def _robot_shortcuts(robot, index):
    name = str(robot.get('name', '')).strip()
    raw_model = str(robot.get('model', '')).strip().lower()
    model = _resolve_robot_model_name(robot)
    shortcuts = {str(index), name.lower(), raw_model, model}

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
    elif model in MOBILE_MODELS:
        shortcuts.add('tiago')
        if model == 'tiago_base':
            shortcuts.update({'tiago_base', 'tiago_no_arm', 'mobile_base'})
        else:
            shortcuts.update({'tiago_with_arm', 'tiago_arm'})

    return shortcuts


def _resolve_selected_robots(all_robots, selected_tokens, config_path):
    exact_name_map = {}
    selector_map = {}

    for index, robot in enumerate(all_robots, start=1):
        name = str(robot.get('name', '')).strip()
        if not name:
            raise RuntimeError(
                f'Robot entry #{index} in "{config_path}" is missing the "name" field.'
            )

        normalized_name = name.lower()
        if normalized_name in exact_name_map:
            raise RuntimeError(
                f'Duplicate robot name "{name}" in "{config_path}". '
                'Every robot entry must have a unique name.'
            )

        exact_name_map[normalized_name] = robot
        for shortcut in _robot_shortcuts(robot, index):
            selector_map.setdefault(shortcut, []).append(robot)

    resolved = []
    seen = set()
    missing = []
    ambiguous = []

    for token in selected_tokens:
        normalized = token.lower()
        candidates = []

        if normalized in exact_name_map:
            candidates = [exact_name_map[normalized]]
        else:
            candidates = selector_map.get(normalized, [])

        if not candidates:
            missing.append(token)
            continue
        if len(candidates) > 1:
            ambiguous.append(
                f'{token} -> {", ".join(str(robot["name"]) for robot in candidates)}'
            )
            continue

        robot_name = str(candidates[0]['name'])
        if robot_name not in seen:
            seen.add(robot_name)
            resolved.append(candidates[0])

    if missing or ambiguous:
        available = ', '.join(
            f'{index}={robot["name"]}'
            for index, robot in enumerate(all_robots, start=1)
        ) or '(none)'
        shortcut_help = 'kuka, staubli, hc10, hc10dt, tiago, tiago_base'
        errors = []
        if missing:
            errors.append('Unknown selection(s): ' + ', '.join(missing))
        if ambiguous:
            errors.append('Ambiguous selection(s): ' + '; '.join(ambiguous))
        errors.append(f'Available robots in "{config_path}": {available}')
        errors.append(f'Useful shortcuts: {shortcut_help}, or use "all" or "none"')
        raise RuntimeError('. '.join(errors))

    return resolved


def _validate_robot_entries(all_robots, config_path):
    seen_names = set()
    for index, robot in enumerate(all_robots, start=1):
        if not isinstance(robot, dict):
            raise RuntimeError(
                f'Robot entry #{index} in "{config_path}" must be a YAML map.'
            )

        name = str(robot.get('name', '')).strip()
        if not name:
            raise RuntimeError(
                f'Robot entry #{index} in "{config_path}" is missing the "name" field.'
            )

        normalized_name = name.lower()
        if normalized_name in seen_names:
            raise RuntimeError(
                f'Duplicate robot name "{name}" in "{config_path}". '
                'Every robot entry must have a unique name.'
            )
        seen_names.add(normalized_name)


def _load_robots(config_path, selected_names=None):
    with open(config_path, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}

    all_robots = config.get('robots', [])
    _validate_robot_entries(all_robots, config_path)

    if selected_names == 'all':
        robots = list(all_robots)
    elif selected_names == 'none':
        robots = []
    elif selected_names:
        robots = _resolve_selected_robots(all_robots, selected_names, config_path)
    else:
        robots = [r for r in all_robots if r.get('enabled', True)]

    if not robots and selected_names != 'none':
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
            'ros_topic_name': f'/{robot_name}/joint_trajectory_progress',
            'gz_topic_name': f'/model/{robot_name}/joint_trajectory_progress',
            'ros_type_name': 'std_msgs/msg/Float64',
            'gz_type_name': 'gz.msgs.Double',
            'direction': 'GZ_TO_ROS',
        },
        {
            'ros_topic_name': f'/{robot_name}/joint_states',
            'gz_topic_name': f'/world/{world_name}/model/{robot_name}/joint_state',
            'ros_type_name': 'sensor_msgs/msg/JointState',
            'gz_type_name': 'gz.msgs.Model',
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

    output_path = os.path.join(tempfile.gettempdir(), f'{robot_name}_bridge.yaml')
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

    output_path = os.path.join(tempfile.gettempdir(), f'{robot_name}_mobile_model.sdf')
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


def _robot_asset_package(robot):
    return str(robot.get('asset_package', DESCRIPTION_PACKAGE)).strip() or DESCRIPTION_PACKAGE


def _robot_asset_package_path(robot, description_pkg_path):
    asset_package = _robot_asset_package(robot)
    if asset_package == DESCRIPTION_PACKAGE:
        return description_pkg_path
    return get_package_share_directory(asset_package)


def _resolve_robot_assets(robot, description_pkg_path, model_name):
    asset_pkg_path = _robot_asset_package_path(robot, description_pkg_path)
    model_sdf = os.path.join(asset_pkg_path, 'models', model_name, 'model.sdf')
    urdf_path = os.path.join(asset_pkg_path, 'urdf', f'{model_name}.urdf')

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
    description_pkg_path = get_package_share_directory(DESCRIPTION_PACKAGE)
    control_pkg_path = get_package_share_directory(CONTROL_CONFIG_PACKAGE)
    world_file_name = LaunchConfiguration('world_name').perform(context)
    world = os.path.join(description_pkg_path, 'worlds', world_file_name + '.world')
    world_entity_name = _get_world_entity_name(world)
    gui_config_file = os.path.join(
        control_pkg_path,
        'config',
        'mfja_default.gui.config',
    )
    gz_partition = LaunchConfiguration('gz_partition').perform(context).strip()
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )
    enable_gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    start_paused = LaunchConfiguration('start_paused').perform(context).lower() == 'true'
    pause_during_switch_update = (
        LaunchConfiguration('pause_during_switch_update').perform(context).lower() == 'true'
    )
    robot_config = LaunchConfiguration('robot_config').perform(context)
    selected_robots = _parse_selected_robots(
        LaunchConfiguration('robots').perform(context)
    )
    initial_loop_mode = LaunchConfiguration('initial_loop_mode').perform(context).strip()

    if not os.path.isabs(robot_config):
        robot_config = os.path.join(control_pkg_path, robot_config)

    robots = _load_robots(robot_config, selected_robots)
    model_paths = [os.path.join(description_pkg_path, 'models')]
    for robot in robots:
        asset_model_path = os.path.join(
            _robot_asset_package_path(robot, description_pkg_path),
            'models',
        )
        if asset_model_path not in model_paths:
            model_paths.append(asset_model_path)

    model_path = pathsep.join(model_paths)
    resource_path = model_path

    if 'GZ_SIM_MODEL_PATH' in environ:
        model_path += pathsep + environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GZ_SIM_RESOURCE_PATH']

    robot_descriptions = {}
    conveyor_controller_arguments = [
        '--world', world_entity_name,
        '--world-file', world,
        '--partition', gz_partition,
        '--initial-loop-mode', initial_loop_mode,
    ]
    if start_paused:
        conveyor_controller_arguments.append('--keep-paused-after-initial-loop')
    if not pause_during_switch_update:
        conveyor_controller_arguments.append('--no-pause-during-switch-update')

    gz_server_args = f'-s {world}' if start_paused else f'-r -s {world}'

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
        Node(
            package=CONTROL_CONFIG_PACKAGE,
            executable='conveyor_loop_mode_controller.py',
            name='conveyor_loop_mode_controller',
            output='screen',
            arguments=conveyor_controller_arguments,
            parameters=[{'use_sim_time': use_sim_time}],
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
            name='room315_world_service_bridge',
            output='screen',
            arguments=[
                f'/world/{world_entity_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                f'/world/{world_entity_name}/create@ros_gz_interfaces/srv/SpawnEntity',
                f'/world/{world_entity_name}/remove@ros_gz_interfaces/srv/DeleteEntity',
            ],
        )
    ]

    for robot in robots:
        robot_name = str(robot['name'])
        model_name = _resolve_robot_model_name(robot)
        x_pose = float(robot.get('x_pose', 0.0))
        y_pose = float(robot.get('y_pose', 0.0))
        z_pose = float(robot.get('z_pose', 0.0))
        yaw = float(robot.get('yaw', 0.0))
        model_sdf, urdf_path = _resolve_robot_assets(
            robot, description_pkg_path, model_name
        )
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
            description='World file name from mfja_3rd_floor_description/worlds (without extension).',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'mfja_multi_robot_sim_{os.getpid()}',
            description='Gazebo transport partition used to isolate this launch instance.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/robots.yaml',
            description='Absolute path or path relative to mfja_robot_control_config.',
        ),
        DeclareLaunchArgument(
            'robots',
            default_value='',
            description=(
                'Comma-separated robot selection list. Supports full names '
                '("kuka1,tiago1"), short aliases ("kuka,tiago,tiago_base"), numeric '
                'indices by YAML order ("1,5"), "all", or "none". Leave empty '
                'to use enabled flags from the YAML.'
            ),
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
            default_value='true',
            choices=['true', 'false'],
            description='Start Gazebo paused so the user can press play manually.',
        ),
        DeclareLaunchArgument(
            'initial_loop_mode',
            default_value='auto',
            description='Startup loop mode: auto, INTERIOR, or EXTERIOR.',
        ),
        DeclareLaunchArgument(
            'pause_during_switch_update',
            default_value='false',
            choices=['true', 'false'],
            description='Pause Gazebo while applying visual switch pose updates.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
