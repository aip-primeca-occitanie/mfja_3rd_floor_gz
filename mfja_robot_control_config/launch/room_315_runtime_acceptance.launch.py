"""Launch one guarded, observation-only Room 315 V4 acceptance scenario."""

import hashlib
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


V4_CANDIDATE_STATE_SCHEMA = 'room315.deployment_candidate_state.v4.v1'
V4_RAW_PREDICTION_SCHEMA = 'room315.visual_runtime_v4.diagnostic.v1'


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _require_v4_candidate_state(state):
    if not isinstance(state, dict):
        raise RuntimeError('candidate_state.json must contain a JSON object')
    candidate_state_schema = str(state.get('schema_version') or '').strip()
    if candidate_state_schema != V4_CANDIDATE_STATE_SCHEMA:
        raise RuntimeError(
            'acceptance requires the exact V4 candidate-state schema: '
            f'{candidate_state_schema!r} != {V4_CANDIDATE_STATE_SCHEMA!r}'
        )
    return V4_RAW_PREDICTION_SCHEMA


def _after_success(phase, actions):
    def handler(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'ERROR: Acceptance readiness failed closed: {phase}'),
                EmitEvent(event=Shutdown(
                    reason=f'Room 315 acceptance {phase} readiness failed',
                )),
            ]
        return [LogInfo(msg=f'Acceptance readiness PASS: {phase}'), *actions]
    return handler


def _scenario_launch_arguments(scenario):
    setup = scenario.get('gazebo_setup') or {}
    left = [str(value) for value in setup.get('left_active_identities') or []]
    right = [str(value) for value in setup.get('right_active_identities') or []]
    if not left or not right:
        raise RuntimeError(
            'acceptance requires initialized left and right presence sources'
        )
    fields = (
        'left_start_positions',
        'right_start_positions',
        'left_loaded_identities',
        'right_loaded_identities',
    )
    for field in fields:
        if not isinstance(setup.get(field), list):
            raise RuntimeError(f'acceptance scenario is missing {field}')
    if len(setup['left_start_positions']) != len(left):
        raise RuntimeError('left identity/start-position cardinality mismatch')
    if len(setup['right_start_positions']) != len(right):
        raise RuntimeError('right identity/start-position cardinality mismatch')
    return {
        'identity_selection_mode': 'explicit',
        'left_active_identities': ','.join(left),
        'right_active_identities': ','.join(right),
        'left_shuttle_count': str(len(left)),
        'right_shuttle_count': str(len(right)),
        'left_start_positions': ','.join(setup['left_start_positions']),
        'right_start_positions': ','.join(setup['right_start_positions']),
        'left_loaded_shuttles': ','.join(setup['left_loaded_identities']),
        'right_loaded_shuttles': ','.join(setup['right_loaded_identities']),
    }


def _acceptance_actions(context):
    candidate = Path(
        LaunchConfiguration('candidate_directory').perform(context)
    ).expanduser().resolve()
    scenario_id = LaunchConfiguration('scenario_id').perform(context).strip()
    output_root = Path(
        LaunchConfiguration('output_root').perform(context)
    ).expanduser().resolve()
    if output_root.exists():
        raise RuntimeError(f'refusing to reuse acceptance output: {output_root}')
    if (
        LaunchConfiguration('enable_task_execution').perform(context) != 'false'
        or LaunchConfiguration('execution_enabled').perform(context) != 'false'
    ):
        raise RuntimeError(
            'this acceptance workflow is observation-only; execution is forbidden'
        )

    manifest_path = candidate / 'acceptance_scenarios.json'
    runtime_config = candidate / 'runtime_ros_parameters.yaml'
    state_path = candidate / 'candidate_state.json'
    for required in (manifest_path, runtime_config, state_path):
        if not required.is_file():
            raise RuntimeError(f'candidate artifact is missing: {required}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    state = json.loads(state_path.read_text(encoding='utf-8'))
    expected_raw_prediction_schema = _require_v4_candidate_state(state)
    checkpoint_filename = str(
        state.get('checkpoint_filename') or 'best.pt'
    ).strip()
    if Path(checkpoint_filename).name != checkpoint_filename:
        raise RuntimeError('candidate checkpoint_filename must be a basename')
    checkpoint_path = candidate / checkpoint_filename
    if not checkpoint_path.is_file():
        raise RuntimeError(f'candidate artifact is missing: {checkpoint_path}')
    matches = [
        row for row in manifest.get('scenarios', [])
        if row.get('scenario_id') == scenario_id
    ]
    if len(matches) != 1:
        raise RuntimeError(f'unknown or duplicate acceptance scenario: {scenario_id}')
    scenario = matches[0]
    launch_values = _scenario_launch_arguments(scenario)
    expected_checkpoint_sha256 = str(state.get('checkpoint_sha256') or '')
    if _sha256(checkpoint_path) != expected_checkpoint_sha256:
        raise RuntimeError('candidate checkpoint SHA-256 failed before Gazebo startup')
    default_topics = {
        'raw_prediction': '/room_315/visual_state/raw_model_prediction',
        'raw_observation': '/room_315/visual_state/raw',
        'validation': '/room_315/visual_state/validation',
        'accepted_observed_state': '/room_315/visual_state/observed_state',
    }
    configured_topics = state.get('runtime_topics') or default_topics
    if not isinstance(configured_topics, dict):
        raise RuntimeError('candidate runtime_topics must be a mapping')
    runtime_topics = {
        name: str(configured_topics.get(name) or '').strip()
        for name in default_topics
    }
    if any(
        not topic.startswith('/room_315/visual_state/')
        for topic in runtime_topics.values()
    ):
        raise RuntimeError('candidate runtime topics are missing or outside visual state')

    readiness_dir = output_root / 'readiness'
    event_path = output_root / 'events' / f'{scenario_id}.json'
    report_path = output_root / 'acceptance_report.json'

    def readiness_node(phase, timeout_argument):
        return Node(
            package='mfja_robot_control_config',
            executable='room_315_runtime_acceptance_readiness.py',
            name=f'room_315_acceptance_{phase}_readiness',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'phase': phase,
                'scenario_manifest_path': str(manifest_path),
                'scenario_id': scenario_id,
                'proof_path': str(readiness_dir / f'{phase}.json'),
                'world_name': 'room_315_only',
                'timeout_s': LaunchConfiguration(timeout_argument),
                'expected_checkpoint_sha256': expected_checkpoint_sha256,
                'expected_raw_prediction_schema': (
                    expected_raw_prediction_schema
                ),
                'raw_prediction_topic': runtime_topics['raw_prediction'],
            }],
        )

    floor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mfja_3rd_floor_bringup'),
            '/launch/room_315_only.launch.py',
        ]),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': 'true',
            'start_paused': 'false',
            'robots': 'none',
            'enable_room315_kinematic_shuttles': 'false',
            'enable_room315_vla': 'false',
            'enable_room315_vla_dataset_recorder': 'false',
            'enable_room315_vla_obstacles': 'false',
            'room315_visual_debug_colors': 'false',
            'room315_show_device_markers': 'false',
        }.items(),
    )
    world_gate = readiness_node('world', 'world_readiness_timeout_s')

    shuttles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mfja_robot_control_config'),
            '/launch/room_315_dual_kinematic_shuttles.launch.py',
        ]),
        launch_arguments={
            'gazebo_world_name': 'room_315_only',
            'use_sim_time': 'true',
            'start_enabled': 'false',
            'identity_selection_mode': launch_values['identity_selection_mode'],
            'left_active_identities': launch_values['left_active_identities'],
            'right_active_identities': launch_values['right_active_identities'],
            'left_shuttle_count': launch_values['left_shuttle_count'],
            'right_shuttle_count': launch_values['right_shuttle_count'],
            'left_start_positions': launch_values['left_start_positions'],
            'right_start_positions': launch_values['right_start_positions'],
            'left_loaded_shuttles': launch_values['left_loaded_shuttles'],
            'right_loaded_shuttles': launch_values['right_loaded_shuttles'],
            'show_device_markers': 'false',
            'visual_debug_colors': 'false',
            'enable_payload_visuals': 'true',
        }.items(),
    )
    scene_gate = readiness_node('scene', 'scene_readiness_timeout_s')

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mfja_robot_control_config'),
            '/launch/room_315_vla_supervisor.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'enable_camera_bridge': 'true',
            'enable_supervisor': 'true',
            'enable_dataset_recorder': 'false',
        }.items(),
    )
    camera_gate = readiness_node('camera', 'camera_readiness_timeout_s')

    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mfja_robot_control_config'),
            '/launch/room_315_visual_state_runtime.launch.py',
        ]),
        launch_arguments={
            'runtime_config': str(runtime_config),
            'enable_camera_bridge': 'false',
            'use_sim_time': 'true',
            'dry_run_state_fusion': 'true',
            'plansys2_update_enabled': 'false',
        }.items(),
    )
    runtime_gate = readiness_node('runtime', 'runtime_readiness_timeout_s')

    recorder = Node(
        package='mfja_robot_control_config',
        executable='room_315_runtime_acceptance_recorder.py',
        name='room_315_runtime_acceptance_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scenario_manifest_path': str(manifest_path),
            'scenario_id': scenario_id,
            'output_file': str(event_path),
            'readiness_proof_path': str(readiness_dir / 'runtime.json'),
            'expected_checkpoint_sha256': expected_checkpoint_sha256,
            'record_duration_s': LaunchConfiguration('record_duration_s'),
            'minimum_record_duration_s': 3.0,
            'minimum_accepted_observations': 3,
            'observation_only': True,
            'raw_model_prediction_topic': runtime_topics['raw_prediction'],
            'raw_observation_topic': runtime_topics['raw_observation'],
            'validation_topic': runtime_topics['validation'],
            'accepted_observed_state_topic': (
                runtime_topics['accepted_observed_state']
            ),
        }],
    )
    report = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mfja_robot_control_config',
            'room_315_runtime_acceptance_report.py',
            '--candidate-directory', str(candidate),
            '--event-directory', str(output_root / 'events'),
            '--output', str(report_path),
        ],
        output='screen',
    )

    def after_recorder(event, _context):
        return [
            LogInfo(msg=(
                'Acceptance observation completed; generating report.'
                if event.returncode == 0
                else (
                    'ERROR: Acceptance observation failed closed; '
                    'generating failure report.'
                )
            )),
            report,
        ]

    def after_report(event, _context):
        success = event.returncode == 0
        message = (
            f'Acceptance report written: {report_path}'
            if success else 'Acceptance report generation failed'
        )
        return [
            LogInfo(msg=message if success else f'ERROR: {message}'),
            EmitEvent(event=Shutdown(reason=message)),
        ]

    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=world_gate,
            on_exit=_after_success('world', [shuttles, scene_gate]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=scene_gate,
            on_exit=_after_success('scene', [cameras, camera_gate]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=camera_gate,
            on_exit=_after_success('camera', [runtime, runtime_gate]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=runtime_gate,
            on_exit=_after_success('runtime', [recorder]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=after_recorder,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=report,
            on_exit=after_report,
        )),
    ]
    return [*handlers, floor, world_gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('candidate_directory'),
        DeclareLaunchArgument('scenario_id'),
        DeclareLaunchArgument('output_root'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
        ),
        DeclareLaunchArgument('world_readiness_timeout_s', default_value='45.0'),
        DeclareLaunchArgument('scene_readiness_timeout_s', default_value='45.0'),
        DeclareLaunchArgument('camera_readiness_timeout_s', default_value='45.0'),
        DeclareLaunchArgument('runtime_readiness_timeout_s', default_value='120.0'),
        DeclareLaunchArgument('record_duration_s', default_value='60.0'),
        DeclareLaunchArgument(
            'enable_task_execution',
            default_value='false',
            choices=['true', 'false'],
            description='Compatibility guard; true is rejected by this workflow.',
        ),
        DeclareLaunchArgument(
            'execution_enabled',
            default_value='false',
            choices=['true', 'false'],
            description='Compatibility guard; true is rejected by this workflow.',
        ),
        OpaqueFunction(function=_acceptance_actions),
    ])
