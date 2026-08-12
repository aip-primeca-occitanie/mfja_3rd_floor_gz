"""Run one immutable Room 315 scenario through the V3/V4 shadow pair.

This launch is deliberately observation-only.  It stages the Gazebo world,
fixed shuttle scene, and paired cameras behind readiness gates before starting
the explicit V3 rollback runtime and the isolated V4 shadow runtime. The comparator in
``room_315_visual_state_v4_shadow.launch.py`` owns final shutdown.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, NamedTuple

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
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


HEX_SHA256 = re.compile(r'[0-9a-f]{64}')
V4_CANDIDATE_STATE_SCHEMA = 'room315.deployment_candidate_state.v4.v1'
V4_PROMOTION_SCHEMA = 'room315.visual_runtime_promotion.v4.v1'
SCENARIO_MANIFEST_SCHEMA = 'room315.runtime_acceptance_scenarios.v1'
REQUIRED_PROMOTION_ARTIFACTS = frozenset({
    'checkpoint',
    'training_final_report',
    'canary_final_report',
    'canary_completion_ledger',
    'effective_config',
    'validation_acceptance',
    'validation_segment_calibration',
    'public_topology_contract',
})


class VerifiedShadowScenario(NamedTuple):
    candidate_directory: Path
    runtime_config: Path
    promotion_manifest: Path
    promotion_manifest_sha256: str
    checkpoint_sha256: str
    candidate_id: str
    scenario_id: str
    scenario: dict[str, Any]
    shuttle_launch_arguments: dict[str, str]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _required_sha256(value: str, label: str) -> str:
    normalized = str(value or '').strip().lower()
    if not HEX_SHA256.fullmatch(normalized):
        raise RuntimeError(f'{label} must be a lowercase 64-character SHA-256')
    return normalized


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f'{label} is not valid readable JSON: {path}') from exc
    if not isinstance(value, dict):
        raise RuntimeError(f'{label} must contain a JSON object: {path}')
    return value


def _require_regular_file(path: Path, label: str) -> None:
    if not path.is_file() or path.is_symlink():
        raise RuntimeError(f'{label} is missing or not a regular file: {path}')


def _verify_candidate_sums(candidate: Path) -> None:
    sums_path = candidate / 'SHA256SUMS'
    _require_regular_file(sums_path, 'candidate SHA256SUMS')
    entries: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        sums_path.read_text(encoding='utf-8').splitlines(),
        start=1,
    ):
        digest, separator, filename = raw_line.partition('  ')
        if (
            separator != '  '
            or not HEX_SHA256.fullmatch(digest)
            or not filename
            or Path(filename).name != filename
            or filename == 'SHA256SUMS'
            or filename in entries
        ):
            raise RuntimeError(
                f'invalid candidate SHA256SUMS entry on line {line_number}'
            )
        entries[filename] = digest

    payloads = {
        path.name
        for path in candidate.iterdir()
        if path.name != 'SHA256SUMS'
    }
    if set(entries) != payloads:
        raise RuntimeError('candidate SHA256SUMS inventory mismatch')
    for filename, expected in entries.items():
        path = candidate / filename
        _require_regular_file(path, f'candidate payload {filename}')
        if _sha256(path) != expected:
            raise RuntimeError(f'candidate payload SHA-256 mismatch: {filename}')


def _scenario_launch_arguments(scenario: dict[str, Any]) -> dict[str, str]:
    setup = scenario.get('gazebo_setup')
    if not isinstance(setup, dict):
        raise RuntimeError('shadow scenario gazebo_setup must be an object')

    list_fields = (
        'left_active_identities',
        'right_active_identities',
        'left_start_positions',
        'right_start_positions',
        'left_loaded_identities',
        'right_loaded_identities',
    )
    for field in list_fields:
        if not isinstance(setup.get(field), list):
            raise RuntimeError(f'shadow scenario is missing list field {field}')

    left = [str(value).strip().upper() for value in setup['left_active_identities']]
    right = [
        str(value).strip().upper() for value in setup['right_active_identities']
    ]
    left_loaded = {
        str(value).strip().upper() for value in setup['left_loaded_identities']
    }
    right_loaded = {
        str(value).strip().upper() for value in setup['right_loaded_identities']
    }
    if (
        not left
        or not right
        or len(left) != len(set(left))
        or len(right) != len(set(right))
        or any(identity not in {'L1', 'L2', 'L3', 'L4'} for identity in left)
        or any(identity not in {'R1', 'R2', 'R3', 'R4'} for identity in right)
    ):
        raise RuntimeError(
            'shadow scenario requires unique initialized identities on both rails'
        )
    if left_loaded - set(left) or right_loaded - set(right):
        raise RuntimeError('loaded identities must be active in the shadow scenario')
    if len(setup['left_start_positions']) != len(left):
        raise RuntimeError('left identity/start-position cardinality mismatch')
    if len(setup['right_start_positions']) != len(right):
        raise RuntimeError('right identity/start-position cardinality mismatch')
    if any(not str(value).strip() for value in (
        *setup['left_start_positions'],
        *setup['right_start_positions'],
    )):
        raise RuntimeError('shadow scenario start positions must be non-empty')

    return {
        'identity_selection_mode': 'explicit',
        'left_active_identities': ','.join(left),
        'right_active_identities': ','.join(right),
        'left_shuttle_count': str(len(left)),
        'right_shuttle_count': str(len(right)),
        'left_start_positions': ','.join(
            str(value).strip() for value in setup['left_start_positions']
        ),
        'right_start_positions': ','.join(
            str(value).strip() for value in setup['right_start_positions']
        ),
        'left_loaded_shuttles': ','.join(sorted(left_loaded)),
        'right_loaded_shuttles': ','.join(sorted(right_loaded)),
    }


def _verify_shadow_candidate(
    candidate_directory: Path | str,
    *,
    scenario_id: str,
    expected_manifest_sha256: str,
    expected_checkpoint_sha256: str,
) -> VerifiedShadowScenario:
    candidate = Path(candidate_directory).expanduser().resolve()
    if not candidate.is_dir() or candidate.is_symlink():
        raise RuntimeError(f'V4 shadow candidate directory is invalid: {candidate}')

    scenario_id = str(scenario_id or '').strip()
    if not scenario_id:
        raise RuntimeError('shadow scenario_id is required')
    expected_manifest_sha = _required_sha256(
        expected_manifest_sha256,
        'expected V4 promotion manifest SHA-256',
    )
    expected_checkpoint_sha = _required_sha256(
        expected_checkpoint_sha256,
        'expected V4 checkpoint SHA-256',
    )

    state_path = candidate / 'candidate_state.json'
    scenario_path = candidate / 'acceptance_scenarios.json'
    runtime_config = candidate / 'runtime_ros_parameters.yaml'
    promotion_path = candidate / 'runtime_promotion_manifest.json'
    for path, label in (
        (state_path, 'candidate state'),
        (scenario_path, 'scenario manifest'),
        (runtime_config, 'V4 runtime configuration'),
        (promotion_path, 'V4 promotion manifest'),
    ):
        _require_regular_file(path, label)

    _verify_candidate_sums(candidate)
    actual_manifest_sha = _sha256(promotion_path)
    if actual_manifest_sha != expected_manifest_sha:
        raise RuntimeError('V4 promotion manifest SHA-256 mismatch')

    state = _load_json_object(state_path, 'candidate state')
    promotion = _load_json_object(promotion_path, 'V4 promotion manifest')
    scenarios = _load_json_object(scenario_path, 'scenario manifest')
    candidate_id = str(state.get('candidate_id') or '').strip()
    if (
        state.get('schema_version') != V4_CANDIDATE_STATE_SCHEMA
        or not candidate_id
        or state.get('deployment_mode') != 'shadow'
        or state.get('shadow_execution_authorized') is not True
        or state.get('automatic_promotion_allowed') is not False
        or state.get('active_runtime_selected') is not False
    ):
        raise RuntimeError('candidate state does not authorize isolated V4 shadow use')
    if (
        promotion.get('schema_version') != V4_PROMOTION_SCHEMA
        or promotion.get('candidate_id') != candidate_id
        or promotion.get('immutable') is not True
        or promotion.get('deployment_mode') != 'shadow'
        or promotion.get('shadow_execution_authorized') is not True
        or promotion.get('automatic_promotion_allowed') is not False
        or promotion.get('manual_review_approved') is not False
        or promotion.get('manual_runtime_review_status') != 'pending'
    ):
        raise RuntimeError('promotion manifest does not authorize pending V4 shadow use')

    artifacts = promotion.get('artifacts')
    if not isinstance(artifacts, dict) or not REQUIRED_PROMOTION_ARTIFACTS <= set(
        artifacts
    ):
        raise RuntimeError('V4 promotion manifest artifact inventory is incomplete')
    for name in REQUIRED_PROMOTION_ARTIFACTS:
        entry = artifacts.get(name)
        if not isinstance(entry, dict):
            raise RuntimeError(f'V4 promotion artifact is invalid: {name}')
        filename = str(entry.get('path') or '').strip()
        digest = _required_sha256(
            str(entry.get('sha256') or ''),
            f'V4 promotion artifact {name} SHA-256',
        )
        if not filename or Path(filename).name != filename:
            raise RuntimeError(f'V4 promotion artifact path is unsafe: {name}')
        artifact_path = candidate / filename
        _require_regular_file(artifact_path, f'V4 promotion artifact {name}')
        if _sha256(artifact_path) != digest:
            raise RuntimeError(f'V4 promotion artifact SHA-256 mismatch: {name}')

    checkpoint_entry = artifacts['checkpoint']
    checkpoint_filename = str(state.get('checkpoint_filename') or '').strip()
    state_checkpoint_sha = _required_sha256(
        str(state.get('checkpoint_sha256') or ''),
        'candidate-state checkpoint SHA-256',
    )
    manifest_checkpoint_sha = str(checkpoint_entry.get('sha256') or '').strip()
    if (
        Path(checkpoint_filename).name != checkpoint_filename
        or checkpoint_filename != checkpoint_entry.get('path')
        or state_checkpoint_sha != manifest_checkpoint_sha
        or state_checkpoint_sha != expected_checkpoint_sha
    ):
        raise RuntimeError('candidate/checkpoint SHA-256 manifest binding mismatch')
    checkpoint_path = candidate / checkpoint_filename
    _require_regular_file(checkpoint_path, 'V4 checkpoint')
    if _sha256(checkpoint_path) != expected_checkpoint_sha:
        raise RuntimeError('V4 checkpoint SHA-256 mismatch')

    runtime_candidate = scenarios.get('runtime_candidate')
    rows = scenarios.get('scenarios')
    if (
        scenarios.get('schema_version') != SCENARIO_MANIFEST_SCHEMA
        or scenarios.get('candidate_id') != candidate_id
        or runtime_candidate != {
            'runtime_generation': 'v4',
            'runtime_mode': 'shadow',
            'automatic_promotion_allowed': False,
        }
        or not isinstance(rows, list)
    ):
        raise RuntimeError('scenario manifest does not match the V4 shadow candidate')
    matches = [
        row for row in rows
        if isinstance(row, dict) and row.get('scenario_id') == scenario_id
    ]
    if len(matches) != 1:
        raise RuntimeError(f'unknown or duplicate shadow scenario: {scenario_id}')
    shuttle_arguments = _scenario_launch_arguments(matches[0])
    return VerifiedShadowScenario(
        candidate_directory=candidate,
        runtime_config=runtime_config,
        promotion_manifest=promotion_path,
        promotion_manifest_sha256=actual_manifest_sha,
        checkpoint_sha256=expected_checkpoint_sha,
        candidate_id=candidate_id,
        scenario_id=scenario_id,
        scenario=matches[0],
        shuttle_launch_arguments=shuttle_arguments,
    )


def _after_success(phase: str, actions: list[Any]):
    def handler(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'ERROR: V4 shadow {phase} readiness failed closed'),
                EmitEvent(event=Shutdown(
                    reason=f'Room 315 V4 shadow {phase} readiness failed',
                )),
            ]
        return [LogInfo(msg=f'V4 shadow readiness PASS: {phase}'), *actions]

    return handler


def _new_shadow_output_root(
    output_root: Path | str,
    *,
    candidate_directory: Path,
) -> Path:
    output = Path(output_root).expanduser().resolve()
    if output.exists():
        raise RuntimeError(f'refusing to reuse V4 shadow output: {output}')
    if output == candidate_directory or candidate_directory in output.parents:
        raise RuntimeError('shadow output cannot be written inside the candidate')
    return output


def _scenario_actions(context):
    verified = _verify_shadow_candidate(
        LaunchConfiguration('candidate_directory').perform(context),
        scenario_id=LaunchConfiguration('scenario_id').perform(context),
        expected_manifest_sha256=LaunchConfiguration(
            'v4_promotion_manifest_sha256'
        ).perform(context),
        expected_checkpoint_sha256=LaunchConfiguration(
            'expected_v4_checkpoint_sha256'
        ).perform(context),
    )
    output_root = _new_shadow_output_root(
        LaunchConfiguration('output_root').perform(context),
        candidate_directory=verified.candidate_directory,
    )

    v3_runtime_config = Path(
        LaunchConfiguration('v3_runtime_config').perform(context)
    ).expanduser().resolve()
    _require_regular_file(v3_runtime_config, 'explicit V3 rollback configuration')
    expected_v3_sha = _required_sha256(
        LaunchConfiguration('expected_v3_checkpoint_sha256').perform(context),
        'expected V3 checkpoint SHA-256',
    )
    minimum_frames_text = LaunchConfiguration(
        'minimum_paired_frames'
    ).perform(context).strip()
    duration_text = LaunchConfiguration('duration_s').perform(context).strip()
    try:
        minimum_frames = int(minimum_frames_text)
        duration_s = float(duration_text)
    except ValueError as exc:
        raise RuntimeError('shadow frame minimum and duration must be numeric') from exc
    if minimum_frames < 1:
        raise RuntimeError('minimum_paired_frames must be at least one')
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise RuntimeError('duration_s must be finite and positive')

    readiness_directory = output_root / 'readiness'
    report_path = output_root / 'shadow_comparison.json'
    if report_path.exists():
        raise RuntimeError(f'refusing to reuse shadow report: {report_path}')

    def readiness_node(phase: str, timeout_argument: str) -> Node:
        return Node(
            package='mfja_robot_control_config',
            executable='room_315_runtime_acceptance_readiness.py',
            name=f'room_315_v4_shadow_{phase}_readiness',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'phase': phase,
                'scenario_manifest_path': str(
                    verified.candidate_directory / 'acceptance_scenarios.json'
                ),
                'scenario_id': verified.scenario_id,
                'proof_path': str(readiness_directory / f'{phase}.json'),
                'world_name': 'room_315_only',
                'timeout_s': LaunchConfiguration(timeout_argument),
                'expected_checkpoint_sha256': verified.checkpoint_sha256,
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
            **verified.shuttle_launch_arguments,
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

    shadow_pair = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mfja_robot_control_config'),
            '/launch/room_315_visual_state_v4_shadow.launch.py',
        ]),
        launch_arguments={
            'v3_runtime_config': str(v3_runtime_config),
            'v4_runtime_config': str(verified.runtime_config),
            'v4_promotion_manifest': str(verified.promotion_manifest),
            'v4_promotion_manifest_sha256': (
                verified.promotion_manifest_sha256
            ),
            'shadow_report_path': str(report_path),
            'expected_v3_checkpoint_sha256': expected_v3_sha,
            'expected_v4_checkpoint_sha256': verified.checkpoint_sha256,
            'minimum_paired_frames': str(minimum_frames),
            'duration_s': str(duration_s),
            'enable_camera_bridge': 'false',
            'use_sim_time': 'true',
            'v3_device': LaunchConfiguration('v3_device'),
            'v4_device': LaunchConfiguration('v4_device'),
        }.items(),
    )

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
            on_exit=_after_success('camera', [shadow_pair]),
        )),
    ]
    return [*handlers, floor, world_gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'candidate_directory',
            description='Immutable V4 shadow candidate directory.',
        ),
        DeclareLaunchArgument(
            'scenario_id',
            default_value='accept_dense',
            description='One exact scenario ID from acceptance_scenarios.json.',
        ),
        DeclareLaunchArgument(
            'output_root',
            description='A new directory for readiness proofs and shadow report.',
        ),
        DeclareLaunchArgument(
            'v4_promotion_manifest_sha256',
            description='Externally expected immutable V4 manifest SHA-256.',
        ),
        DeclareLaunchArgument(
            'expected_v4_checkpoint_sha256',
            default_value=(
                '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
            ),
        ),
        DeclareLaunchArgument(
            'v3_runtime_config',
            default_value=[
                FindPackageShare('mfja_robot_control_config'),
                '/config/room_315_vla/visual_state_runtime_v3_rollback.yaml',
            ],
            description='Explicit V3 rollback runtime configuration.',
        ),
        DeclareLaunchArgument(
            'expected_v3_checkpoint_sha256',
            default_value=(
                '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
            ),
        ),
        DeclareLaunchArgument(
            'minimum_paired_frames',
            default_value='20',
        ),
        DeclareLaunchArgument('duration_s', default_value='30.0'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
        ),
        DeclareLaunchArgument('world_readiness_timeout_s', default_value='45.0'),
        DeclareLaunchArgument('scene_readiness_timeout_s', default_value='45.0'),
        DeclareLaunchArgument('camera_readiness_timeout_s', default_value='45.0'),
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
        OpaqueFunction(function=_scenario_actions),
    ])
