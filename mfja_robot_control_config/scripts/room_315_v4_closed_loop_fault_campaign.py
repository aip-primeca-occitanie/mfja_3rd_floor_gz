#!/usr/bin/env python3
"""Qualify fail-closed Room 315 V4 behaviour in Gazebo, never on hardware.

The campaign has two deliberately separate steps.  ``prepare-source-lock``
captures the exact runtime sources after review.  ``run`` accepts only that
immutable lock, re-hashes every source and fixed qualification artifact, and
then runs each fault in a fresh ROS domain and a fresh Gazebo process.  Output
directories are single-use and are made read-only after finalisation.

This runner is scoped to the immutable V4 closed-loop *qualification* bundle.
It does not grant or claim physical-deployment authority.
"""

from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
import re
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


SCRIPT_PATH = Path(__file__).resolve()
REPO = SCRIPT_PATH.parents[2]
WORKSPACE = REPO.parents[1]
INSTALL_ROOT = WORKSPACE / 'install'
INSTALL_SETUP = WORKSPACE / 'install' / 'setup.bash'
PACKAGE = REPO / 'mfja_robot_control_config'
V4_BUNDLE = Path(
    '/home/tiago/room315_visual_runtime_candidate_v4_seed31520260811_'
    'epoch11_869d6404_closed_loop_qualification_attempt1'
)
V4_CHECKPOINT = V4_BUNDLE / 'checkpoint_epoch_011.pt'
V4_PROMOTION_MANIFEST = V4_BUNDLE / 'runtime_promotion_manifest.json'
V4_MANUAL_DECISION = V4_BUNDLE / 'manual_decision_record.json'
V4_RUNTIME_CONFIG = V4_BUNDLE / 'runtime_ros_parameters.yaml'
V4_AUTHORIZATION = V4_BUNDLE / 'candidate_state.json'
TASK_RUNTIME_CONFIG = (
    PACKAGE / 'config/room_315_task_execution/task_execution_runtime_v4.yaml'
)
V4_SCHEMA = 'room315.visual_state.v4'
V4_CHECKPOINT_SHA256 = (
    '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
)
V4_PROMOTION_MANIFEST_SHA256 = (
    '6f9828219c22599825f5a14e405c8f11ce017984cc0d65821a357240d6529e2a'
)
V4_MANUAL_DECISION_SHA256 = (
    '296c9768cfa8da2ff2507d6ad9aabb10046489e8653b4dd6f13f40c1e4e096e2'
)
V4_RUNTIME_CONFIG_SHA256 = (
    'e6f8506f352bcc1f09b84707705ecaa5498b60e339865f8b7eca5c92a5aa5a6d'
)
V4_AUTHORIZATION_SHA256 = (
    '3181aab44b0b70f1394537f778f9ac361e3fc3627cfe9c16b99e6d33529cf94b'
)
TASK_RUNTIME_CONFIG_SHA256 = (
    'f2282108cb058caf400846b5b187b85c6a0e849b936cb77623267f06929669e2'
)
AUTHORIZATION_SCOPE = 'gazebo_v4_closed_loop_qualification_only'
SOURCE_LOCK_SCHEMA = 'room315.v4_closed_loop_fault_source_lock.v1'
CAMPAIGN_SCHEMA = 'room315.v4_closed_loop_fault_campaign.v1'
CASE_SCHEMA = 'room315.v4_closed_loop_fault_case.v1'
RUNTIME_LOCK = Path('/tmp/mfja_room315_floor_runtime.lock')
TERMINAL_STATUSES = frozenset({'succeeded', 'aborted', 'failed', 'rejected'})
NON_SUCCESS_STATUSES = frozenset({'aborted', 'failed', 'rejected'})
TASK_GOAL_ACKNOWLEDGEMENT_STATUSES = frozenset({
    'accepted', 'running', *TERMINAL_STATUSES,
})
SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')


@dataclass(frozen=True)
class FixedArtifact:
    role: str
    path: Path
    size_bytes: int
    sha256: str
    immutable_required: bool


FIXED_ARTIFACTS = (
    FixedArtifact(
        'visual_checkpoint', V4_CHECKPOINT, 85876329,
        V4_CHECKPOINT_SHA256, True,
    ),
    FixedArtifact(
        'visual_promotion_manifest', V4_PROMOTION_MANIFEST, 8445,
        V4_PROMOTION_MANIFEST_SHA256, True,
    ),
    FixedArtifact(
        'visual_manual_decision', V4_MANUAL_DECISION, 1141,
        V4_MANUAL_DECISION_SHA256, True,
    ),
    FixedArtifact(
        'visual_runtime_configuration', V4_RUNTIME_CONFIG, 827,
        V4_RUNTIME_CONFIG_SHA256, True,
    ),
    FixedArtifact(
        'task_execution_authorization', V4_AUTHORIZATION, 797,
        V4_AUTHORIZATION_SHA256, True,
    ),
    FixedArtifact(
        'task_execution_runtime_configuration', TASK_RUNTIME_CONFIG, 2148,
        TASK_RUNTIME_CONFIG_SHA256, False,
    ),
)


CRITICAL_SOURCE_PATHS = (
    'mfja_3rd_floor_bringup/launch/room_315_only.launch.py',
    'mfja_rail_interfaces/msg/ShuttleCommand.msg',
    'mfja_rail_interfaces/msg/ShuttleState.msg',
    'mfja_rail_interfaces/msg/VisualStateObservation.msg',
    'mfja_rail_interfaces/msg/VisualShuttleState.msg',
    'mfja_robot_control_config/launch/room_315_task_execution.launch.py',
    'mfja_robot_control_config/launch/room_315_visual_state_runtime.launch.py',
    'mfja_robot_control_config/scripts/room_315_closed_loop_executive.py',
    'mfja_robot_control_config/scripts/room_315_kinematic_shuttle_node.py',
    'mfja_robot_control_config/scripts/room_315_pddl_scenario_generator.py',
    'mfja_robot_control_config/scripts/room_315_task_execution.py',
    'mfja_robot_control_config/scripts/room_315_task_execution_config.py',
    'mfja_robot_control_config/scripts/room_315_task_execution_node.py',
    'mfja_robot_control_config/scripts/room_315_v4_closed_loop_fault_campaign.py',
    'mfja_robot_control_config/scripts/room_315_visual_runtime_v4.py',
    'mfja_robot_control_config/scripts/room_315_visual_state_inference_node.py',
    'mfja_robot_control_config/scripts/room_315_rail_safety_supervisor.py',
)


@dataclass(frozen=True)
class FaultScenario:
    scenario_id: str
    fault: str
    planner_enabled: bool
    expected_terminal: str
    expect_gateway_start: bool
    expect_actuating_command: bool
    inject_emergency_stop: bool
    require_disabled_controller: bool
    sensor_feedback_available: bool


FAULT_SCENARIOS = (
    FaultScenario(
        'F01', 'corrupt_authorization', True, 'gateway_refused', False,
        False, False, True, True,
    ),
    FaultScenario(
        'F02', 'wrong_promotion_manifest', True, 'gateway_refused', False,
        False, False, True, True,
    ),
    FaultScenario(
        'F03', 'planner_unavailable', False, 'non_success', True,
        False, False, True, True,
    ),
    FaultScenario(
        'F04', 'emergency_stop_during_long_move', True, 'non_success', True,
        True, True, True, True,
    ),
    FaultScenario(
        'F05', 'right_sensor_feedback_loss', True, 'non_success', True,
        True, False, True, False,
    ),
)


class QualificationError(RuntimeError):
    """Raised when a declared fail-closed qualification invariant fails."""


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    process: subprocess.Popen[str]
    log_path: Path
    log_stream: Any


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace('+00:00', 'Z')


def canonical_json(value: Any) -> bytes:
    return (
        json.dumps(value, indent=2, sort_keys=True) + '\n'
    ).encode('utf-8')


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_bytes_exclusive(path: Path, value: bytes, mode: int = 0o644) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
    try:
        with os.fdopen(descriptor, 'wb', closefd=False) as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(descriptor)


def write_json_exclusive(path: Path, value: Any, mode: int = 0o644) -> None:
    write_bytes_exclusive(path, canonical_json(value), mode=mode)


def write_json_replace(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_bytes(canonical_json(value))
    temporary.replace(path)


def read_json_object(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QualificationError(f'{label} is not valid UTF-8 JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise QualificationError(f'{label} must contain one JSON object')
    return value


def require_immutable_regular_file(path: Path, *, label: str) -> os.stat_result:
    try:
        info = path.stat()
    except OSError as exc:
        raise QualificationError(f'{label} is not readable: {path}: {exc}') from exc
    if not stat.S_ISREG(info.st_mode):
        raise QualificationError(f'{label} is not a regular file: {path}')
    if info.st_mode & 0o022:
        raise QualificationError(
            f'{label} must not be group- or world-writable: {path}'
        )
    return info


def verify_fixed_artifacts() -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for artifact in FIXED_ARTIFACTS:
        if artifact.immutable_required:
            info = require_immutable_regular_file(
                artifact.path, label=artifact.role,
            )
        else:
            try:
                info = artifact.path.stat()
            except OSError as exc:
                raise QualificationError(
                    f'{artifact.role} is not readable: {artifact.path}: {exc}'
                ) from exc
            if not stat.S_ISREG(info.st_mode):
                raise QualificationError(
                    f'{artifact.role} is not a regular file: {artifact.path}'
                )
        actual_sha256 = sha256_file(artifact.path)
        if info.st_size != artifact.size_bytes or actual_sha256 != artifact.sha256:
            raise QualificationError(
                f'fixed artifact drift for {artifact.role}: '
                f'size={info.st_size}/{artifact.size_bytes}, '
                f'sha256={actual_sha256}/{artifact.sha256}'
            )
        rows.append({
            'role': artifact.role,
            'path': str(artifact.path),
            'size_bytes': info.st_size,
            'sha256': actual_sha256,
            'immutable_required': artifact.immutable_required,
            'mode': stat.S_IMODE(info.st_mode),
        })
    verify_qualification_semantics()
    return {
        'status': 'passed',
        'authorization_scope': AUTHORIZATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'physical_deployment_approved': False,
        'artifacts': rows,
    }


def verify_qualification_semantics() -> None:
    manifest = read_json_object(
        V4_PROMOTION_MANIFEST, label='V4 qualification manifest',
    )
    authorization = read_json_object(
        V4_AUTHORIZATION, label='V4 task authorization',
    )
    decision = read_json_object(
        V4_MANUAL_DECISION, label='V4 manual decision',
    )
    qualification = manifest.get('closed_loop_qualification')
    if not isinstance(qualification, dict):
        raise QualificationError('manifest has no closed_loop_qualification object')
    expected_guards = {
        'actuation_enabled': True,
        'dry_run_state_fusion': True,
        'plansys2_update_enabled': False,
    }
    checks = (
        (manifest.get('deployment_mode') == 'active', 'manifest is not active'),
        (
            qualification.get('qualification_only') is True,
            'manifest is not qualification-only',
        ),
        (
            qualification.get('physical_deployment_approved') is False,
            'manifest must explicitly reject physical deployment',
        ),
        (
            (manifest.get('manual_decision_record') or {}).get('sha256')
            == V4_MANUAL_DECISION_SHA256,
            'manifest manual-decision binding drifted',
        ),
        (
            authorization.get('authorization_scope') == AUTHORIZATION_SCOPE,
            'authorization scope drifted',
        ),
        (
            authorization.get('checkpoint_sha256') == V4_CHECKPOINT_SHA256,
            'authorization checkpoint drifted',
        ),
        (
            authorization.get('physical_deployment_approved') is False,
            'authorization must explicitly reject physical deployment',
        ),
        (
            authorization.get('runtime_guards') == expected_guards,
            'authorization runtime guards drifted',
        ),
        (
            decision.get('scope') == AUTHORIZATION_SCOPE,
            'manual decision scope drifted',
        ),
        (
            decision.get('physical_deployment_approved') is False,
            'manual decision must explicitly reject physical deployment',
        ),
        (
            decision.get('runtime_guards') == expected_guards,
            'manual decision runtime guards drifted',
        ),
    )
    for accepted, reason in checks:
        if not accepted:
            raise QualificationError(reason)


def source_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for relative in CRITICAL_SOURCE_PATHS:
        path = REPO / relative
        if not path.is_file():
            raise QualificationError(f'critical runtime source is missing: {path}')
        installed = installed_path_for_source(relative)
        if not installed.is_file():
            raise QualificationError(
                f'critical installed runtime file is missing: {installed}; '
                'rebuild the workspace before preparing a source lock'
            )
        source_sha256 = sha256_file(path)
        installed_sha256 = sha256_file(installed)
        if path.stat().st_size != installed.stat().st_size or (
            source_sha256 != installed_sha256
        ):
            raise QualificationError(
                f'source/install drift for {relative}: '
                f'source={source_sha256}, installed={installed_sha256}; '
                'rebuild before qualification'
            )
        rows.append({
            'path': relative,
            'size_bytes': path.stat().st_size,
            'sha256': source_sha256,
            'installed_path': str(installed),
            'installed_size_bytes': installed.stat().st_size,
            'installed_sha256': installed_sha256,
        })
    return rows


def installed_path_for_source(relative: str) -> Path:
    parts = Path(relative).parts
    if len(parts) < 3:
        raise QualificationError(f'cannot map source into install tree: {relative}')
    package, category, *remainder = parts
    if category == 'scripts' and package == 'mfja_robot_control_config':
        return INSTALL_ROOT / package / 'lib' / package / Path(*remainder)
    if category in {'launch', 'msg'}:
        return INSTALL_ROOT / package / 'share' / package / category / Path(
            *remainder
        )
    raise QualificationError(f'cannot map source into install tree: {relative}')


def prepare_source_lock(output: Path) -> dict[str, Any]:
    output = output.expanduser().resolve()
    if output.exists():
        raise QualificationError(f'refusing to overwrite source lock: {output}')
    fixed = verify_fixed_artifacts()
    rows = source_rows()
    payload = {
        'schema_version': SOURCE_LOCK_SCHEMA,
        'created_at_utc': utc_now(),
        'authorization_scope': AUTHORIZATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'visual_checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'qualification_manifest_sha256': V4_PROMOTION_MANIFEST_SHA256,
        'task_runtime_config_sha256': TASK_RUNTIME_CONFIG_SHA256,
        'physical_deployment_approved': False,
        'automatic_execution_authority': False,
        'fixed_artifacts': fixed['artifacts'],
        'sources': rows,
        'source_count': len(rows),
        'source_set_sha256': sha256_bytes(canonical_json(rows)),
    }
    write_json_exclusive(output, payload, mode=0o444)
    os.chmod(output, 0o444)
    return payload


def verify_source_lock(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    require_immutable_regular_file(path, label='source lock')
    payload = read_json_object(path, label='source lock')
    expected_scalars = {
        'schema_version': SOURCE_LOCK_SCHEMA,
        'authorization_scope': AUTHORIZATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'visual_checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'qualification_manifest_sha256': V4_PROMOTION_MANIFEST_SHA256,
        'task_runtime_config_sha256': TASK_RUNTIME_CONFIG_SHA256,
        'physical_deployment_approved': False,
        'automatic_execution_authority': False,
        'source_count': len(CRITICAL_SOURCE_PATHS),
    }
    for name, expected in expected_scalars.items():
        if payload.get(name) != expected:
            raise QualificationError(
                f'source lock {name} mismatch: {payload.get(name)!r} != {expected!r}'
            )
    locked_rows = payload.get('sources')
    if not isinstance(locked_rows, list):
        raise QualificationError('source lock sources must be a list')
    if [row.get('path') for row in locked_rows if isinstance(row, dict)] != list(
        CRITICAL_SOURCE_PATHS
    ):
        raise QualificationError('source lock does not cover the exact source set')
    if payload.get('source_set_sha256') != sha256_bytes(canonical_json(locked_rows)):
        raise QualificationError('source lock aggregate hash is invalid')
    current_rows = source_rows()
    if locked_rows != current_rows:
        locked = {row['path']: row for row in locked_rows if isinstance(row, dict)}
        current = {row['path']: row for row in current_rows}
        changed = sorted(
            path_name for path_name in set(locked) | set(current)
            if locked.get(path_name) != current.get(path_name)
        )
        raise QualificationError(f'critical runtime source drift: {changed}')
    fixed = verify_fixed_artifacts()
    if payload.get('fixed_artifacts') != fixed['artifacts']:
        raise QualificationError('fixed-artifact rows differ from the source lock')
    result = dict(payload)
    result['source_lock_path'] = str(path)
    result['source_lock_sha256'] = sha256_file(path)
    result['verified_at_utc'] = utc_now()
    return result


def validate_scenarios(scenarios: Iterable[FaultScenario]) -> None:
    rows = list(scenarios)
    expected_ids = ['F01', 'F02', 'F03', 'F04', 'F05']
    if [item.scenario_id for item in rows] != expected_ids:
        raise QualificationError('fault scenario identifiers or order drifted')
    if len({item.fault for item in rows}) != len(rows):
        raise QualificationError('fault names must be unique')
    by_fault = {item.fault: item for item in rows}
    if by_fault['corrupt_authorization'].expect_gateway_start:
        raise QualificationError('corrupt authorization must refuse gateway startup')
    if by_fault['wrong_promotion_manifest'].expect_gateway_start:
        raise QualificationError('wrong manifest must refuse gateway startup')
    if by_fault['planner_unavailable'].planner_enabled:
        raise QualificationError('planner-unavailable scenario must disable PlanSys2')
    if not by_fault['emergency_stop_during_long_move'].expect_actuating_command:
        raise QualificationError('emergency-stop scenario must begin a long move')
    if not by_fault['emergency_stop_during_long_move'].inject_emergency_stop:
        raise QualificationError('emergency-stop scenario must inject the e-stop')
    feedback_loss = by_fault['right_sensor_feedback_loss']
    if feedback_loss.sensor_feedback_available:
        raise QualificationError('feedback-loss scenario must use a black-hole topic')
    if not feedback_loss.expect_actuating_command:
        raise QualificationError('feedback-loss scenario must begin a long move')
    if feedback_loss.inject_emergency_stop:
        raise QualificationError('feedback-loss scenario must not inject an e-stop')
    if sum(int(item.inject_emergency_stop) for item in rows) != 1:
        raise QualificationError('exactly one fault scenario must inject an e-stop')
    if any(item.expected_terminal == 'succeeded' for item in rows):
        raise QualificationError('no fault scenario may expect success')


def scenario_by_id(scenario_id: str) -> FaultScenario:
    for scenario in FAULT_SCENARIOS:
        if scenario.scenario_id == scenario_id:
            return scenario
    raise QualificationError(f'unknown fault scenario: {scenario_id}')


def floor_command(speed_mps: float = 0.05) -> list[str]:
    return [
        'ros2', 'launch', 'mfja_3rd_floor_bringup', 'room_315_only.launch.py',
        'robots:=none',
        'gui:=false',
        'start_paused:=false',
        'enable_room315_kinematic_shuttles:=true',
        'enable_room315_right_rail:=true',
        'enable_room315_left_rail:=true',
        'enable_room315_rail_safety_supervisor:=true',
        'enable_room315_rgbd_camera_bridge:=true',
        'enable_room315_visual_state_dataset_recorder:=false',
        'enable_room315_visual_obstacles:=false',
        'room315_show_device_markers:=false',
        'room315_visual_debug_colors:=false',
        'room315_enable_payload_visuals:=true',
        'room315_identity_selection_mode:=explicit',
        'room315_shuttles_start_enabled:=false',
        f'room315_shuttle_speed:={speed_mps}',
        'room315_right_shuttle_count:=1',
        'room315_right_active_identities:=R4',
        'room315_right_start_slots:=1',
        'room315_left_shuttle_count:=0',
    ]


def visual_command() -> list[str]:
    return [
        'ros2', 'launch', 'mfja_robot_control_config',
        'room_315_visual_state_runtime.launch.py',
        'use_sim_time:=true',
        'enable_camera_bridge:=false',
        'device:=cuda',
        f'runtime_config:={V4_RUNTIME_CONFIG}',
        'runtime_generation:=v4',
        'runtime_mode:=active',
        f'v4_promotion_manifest:={V4_PROMOTION_MANIFEST}',
        f'v4_promotion_manifest_sha256:={V4_PROMOTION_MANIFEST_SHA256}',
        'dry_run_state_fusion:=true',
        'plansys2_update_enabled:=false',
    ]


def execution_command(
    *, runtime_config: Path, planner_enabled: bool,
) -> list[str]:
    return [
        'ros2', 'launch', 'mfja_robot_control_config',
        'room_315_task_execution.launch.py',
        'use_sim_time:=true',
        f'runtime_config:={runtime_config}',
        'execution_enabled:=true',
        f'enable_plansys2:={str(planner_enabled).lower()}',
        'external_obstacles_disabled:=true',
    ]


def long_move_task_goal() -> dict[str, Any]:
    return {
        'confidence': 1.0,
        'constraints': {
            'goal_type': 'transport',
            'payload_filter': 'any',
            'selection_strategy': 'explicit',
            'shuttle_selection': 'explicit',
            'side': 'right',
            'target_kind': 'slot',
            'target_shuttle': 'room315_right_shuttle_4',
            'target_slot': '4',
        },
        'contract_type': 'TaskGoal',
        'description': 'fault qualification: move R4 from slot 1 to slot 4',
        'goal_id': 'room315-v4-fault-r4-slot1-to-slot4',
        'schema_version': 1,
        'source': 'human',
        'timestamp': 0.0,
    }


def verify_command_templates() -> dict[str, Any]:
    validate_scenarios(FAULT_SCENARIOS)
    visual = visual_command()
    floor = floor_command()
    task = execution_command(
        runtime_config=TASK_RUNTIME_CONFIG,
        planner_enabled=True,
    )
    joined_visual = ' '.join(visual)
    joined_floor = ' '.join(floor)
    joined_task = ' '.join(task)
    required_visual = (
        'runtime_generation:=v4',
        'runtime_mode:=active',
        f'v4_promotion_manifest:={V4_PROMOTION_MANIFEST}',
        f'v4_promotion_manifest_sha256:={V4_PROMOTION_MANIFEST_SHA256}',
        'dry_run_state_fusion:=true',
        'plansys2_update_enabled:=false',
    )
    if any(token not in joined_visual for token in required_visual):
        raise QualificationError('visual command lost a V4 qualification guard')
    if 'gui:=false' not in joined_floor or 'robots:=none' not in joined_floor:
        raise QualificationError('floor command is not the Gazebo-only headless scene')
    if 'execution_enabled:=true' not in joined_task:
        raise QualificationError('task command does not explicitly opt into execution')
    forbidden = ('physical', 'hardware', 'real_robot')
    if any(token in joined_visual.casefold() for token in forbidden):
        raise QualificationError('visual command contains a physical-deployment token')
    return {'floor': floor, 'visual': visual, 'execution': task}


def clean_environment(domain_id: int, ros_log_dir: Path) -> dict[str, str]:
    env = dict(os.environ)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    env.update({
        'ROS_DOMAIN_ID': str(domain_id),
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS2CLI_NO_DAEMON': '1',
        'ROS_LOG_DIR': str(ros_log_dir),
        'HF_HUB_OFFLINE': '1',
        'TRANSFORMERS_OFFLINE': '1',
    })
    env.pop('ROS_LOCALHOST_ONLY', None)
    return env


def visual_environment(env: dict[str, str]) -> dict[str, str]:
    result = dict(env)
    venv_packages = Path(
        '/home/tiago/room315_local_training/venv/lib/python3.12/site-packages'
    )
    current = result.get('PYTHONPATH', '')
    parts = ['/usr/lib/python3/dist-packages', str(venv_packages)]
    if current:
        parts.append(current)
    result['PYTHONPATH'] = ':'.join(parts)
    return result


def run_capture(
    command: list[str],
    *,
    env: dict[str, str],
    timeout_s: float = 30.0,
    cwd: Path = WORKSPACE,
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=False,
        )
        return {
            'command': command,
            'returncode': completed.returncode,
            'stdout': completed.stdout,
            'stderr': completed.stderr,
            'timed_out': False,
            'duration_s': time.monotonic() - started,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            'command': command,
            'returncode': None,
            'stdout': exc.stdout or '',
            'stderr': exc.stderr or '',
            'timed_out': True,
            'duration_s': time.monotonic() - started,
        }


def start_process(
    name: str,
    command: list[str],
    *,
    env: dict[str, str],
    log_path: Path,
) -> ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open('w', encoding='utf-8', buffering=1)
    process = subprocess.Popen(
        command,
        cwd=WORKSPACE,
        env=env,
        text=True,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return ManagedProcess(name, command, process, log_path, stream)


def stop_process(
    managed: ManagedProcess,
    *,
    timeout_s: float = 15.0,
) -> dict[str, Any]:
    signals: list[str] = []
    if managed.process.poll() is None:
        for sig, wait_s in (
            (signal.SIGINT, timeout_s),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 2.0),
        ):
            try:
                os.killpg(managed.process.pid, sig)
                signals.append(sig.name)
            except ProcessLookupError:
                break
            try:
                managed.process.wait(timeout=wait_s)
                break
            except subprocess.TimeoutExpired:
                continue
    returncode = managed.process.poll()
    managed.log_stream.flush()
    managed.log_stream.close()
    return {
        'name': managed.name,
        'pid': managed.process.pid,
        'command': managed.command,
        'signals': signals,
        'returncode': returncode,
    }


def require_alive(managed: ManagedProcess, *, label: str) -> None:
    returncode = managed.process.poll()
    if returncode is None:
        return
    log_tail = managed.log_path.read_text(
        encoding='utf-8', errors='replace',
    )[-4000:]
    raise QualificationError(
        f'{label} exited early with {returncode}:\n{log_tail}'
    )


def wait_for_process_exit(
    managed: ManagedProcess,
    *,
    timeout_s: float,
) -> int:
    try:
        return managed.process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        raise QualificationError(
            f'{managed.name} did not exit within {timeout_s:.1f}s'
        ) from exc


def topic_echo_once(
    topic: str,
    message_type: str,
    *,
    env: dict[str, str],
    timeout_s: float,
    filter_expression: str = '',
) -> dict[str, Any]:
    command = [
        'ros2', 'topic', 'echo', '--no-daemon', '--full-length',
        topic, message_type, '--once',
    ]
    if filter_expression:
        command.extend(['--filter', filter_expression])
    return run_capture(command, env=env, timeout_s=timeout_s)


def parse_ros_yaml(text: str) -> dict[str, Any]:
    cleaned = text.strip()
    if cleaned.endswith('---'):
        cleaned = cleaned[:-3].rstrip()
    try:
        payload = yaml.safe_load(cleaned) if cleaned else None
    except yaml.YAMLError as exc:
        raise QualificationError(f'ROS output is invalid YAML: {exc}') from exc
    if not isinstance(payload, dict):
        raise QualificationError('ROS output is not one YAML mapping')
    return payload


def wait_topic(
    topic: str,
    message_type: str,
    *,
    env: dict[str, str],
    timeout_s: float,
) -> dict[str, Any]:
    result = topic_echo_once(
        topic, message_type, env=env, timeout_s=timeout_s,
    )
    if result.get('returncode') != 0:
        raise QualificationError(
            f'{topic} did not become ready: '
            f'{result.get("stderr") or result.get("stdout")}'
        )
    return parse_ros_yaml(str(result.get('stdout') or ''))


def wait_for_subscription(
    topic: str,
    *,
    env: dict[str, str],
    timeout_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    attempts: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        result = run_capture(
            ['ros2', 'topic', 'info', '--no-daemon', topic],
            env=env,
            timeout_s=min(5.0, max(0.5, deadline - time.monotonic())),
        )
        attempts.append(result)
        match = re.search(
            r'Subscription count:\s*(\d+)', str(result.get('stdout') or ''),
        )
        if result.get('returncode') == 0 and match and int(match.group(1)) >= 1:
            return {'status': 'ready', 'attempts': attempts}
        time.sleep(0.25)
    raise QualificationError(f'no subscriber became ready for {topic}')


def wait_for_unpublished_feedback_subscription(
    topic: str,
    *,
    env: dict[str, str],
    timeout_s: float,
) -> dict[str, Any]:
    """Prove the gateway listens on a feedback topic with zero publishers."""

    deadline = time.monotonic() + timeout_s
    attempts: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        result = run_capture(
            ['ros2', 'topic', 'info', '--no-daemon', topic],
            env=env,
            timeout_s=min(5.0, max(0.5, deadline - time.monotonic())),
        )
        attempts.append(result)
        output = str(result.get('stdout') or '')
        publisher_match = re.search(r'Publisher count:\s*(\d+)', output)
        subscription_match = re.search(r'Subscription count:\s*(\d+)', output)
        if (
            result.get('returncode') == 0
            and publisher_match
            and int(publisher_match.group(1)) == 0
            and subscription_match
            and int(subscription_match.group(1)) >= 1
        ):
            return {
                'status': 'unavailable',
                'topic': topic,
                'publisher_count': 0,
                'subscription_count': int(subscription_match.group(1)),
                'attempts': attempts,
            }
        time.sleep(0.25)
    raise QualificationError(
        f'feedback-loss black-hole topic was not proved unavailable: {topic}'
    )


def capture_shuttle_state(
    *,
    env: dict[str, str],
    timeout_s: float,
    require_mode: str = '',
) -> dict[str, Any]:
    expression = "m.name == 'room315_right_shuttle_4'"
    if require_mode:
        expression += f" and m.mode == {require_mode!r}"
    result = topic_echo_once(
        '/room_315/rails/right/shuttles/state',
        'mfja_rail_interfaces/msg/ShuttleState',
        env=env,
        timeout_s=timeout_s,
        filter_expression=expression,
    )
    if result.get('returncode') != 0:
        raise QualificationError(
            'R4 controller state was not captured: '
            f'{result.get("stderr") or result.get("stdout")}'
        )
    state = parse_ros_yaml(str(result.get('stdout') or ''))
    if state.get('name') != 'room315_right_shuttle_4':
        raise QualificationError(f'captured wrong shuttle state: {state}')
    return state


def verify_stationary(
    before: dict[str, Any],
    after: dict[str, Any],
    *,
    tolerance_m: float = 1.0e-6,
) -> dict[str, Any]:
    before_s = float(before.get('s'))
    after_s = float(after.get('s'))
    if not math.isfinite(before_s) or not math.isfinite(after_s):
        raise QualificationError('stationary proof contains non-finite s')
    delta = abs(after_s - before_s)
    if delta > tolerance_m:
        raise QualificationError(
            f'zero-motion invariant failed: |delta_s|={delta:.9f}m '
            f'> {tolerance_m:.9f}m'
        )
    if str(after.get('mode') or '').upper() != 'DISABLED':
        raise QualificationError(
            f'zero-motion final controller mode is not DISABLED: {after}'
        )
    return {
        'before_s_m': before_s,
        'after_s_m': after_s,
        'absolute_delta_s_m': delta,
        'tolerance_m': tolerance_m,
        'final_mode': after.get('mode'),
    }


def publish_string(
    topic: str,
    payload: dict[str, Any],
    *,
    env: dict[str, str],
) -> dict[str, Any]:
    message = json.dumps(json.dumps(payload, sort_keys=True))
    return run_capture(
        [
            'ros2', 'topic', 'pub', '--once', topic, 'std_msgs/msg/String',
            f'{{data: {message}}}',
        ],
        env=env,
        timeout_s=15.0,
    )


def parse_task_goal_acknowledgement(
    raw: str,
    *,
    goal_id: str,
) -> dict[str, Any] | None:
    """Return only an explicit status acknowledgement for the exact goal."""

    try:
        payload = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get('goal_id') != goal_id:
        return None
    status = str(payload.get('status') or '').strip().casefold()
    if status not in TASK_GOAL_ACKNOWLEDGEMENT_STATUSES:
        return None
    result = dict(payload)
    result['status'] = status
    return result


def publish_task_goal_once_wait_ack(
    payload: dict[str, Any],
    *,
    timeout_s: float,
    rclpy_module: Any | None = None,
    string_type: Any | None = None,
) -> dict[str, Any]:
    """Publish exactly once through a persistent DDS endpoint and await status.

    A missing acknowledgement is deliberately fatal: retrying the same goal
    after an uncertain delivery could create a duplicate live execution.
    """

    goal_id = str(payload.get('goal_id') or '').strip()
    if not goal_id:
        raise QualificationError('TaskGoal publication requires a goal_id')
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise QualificationError('TaskGoal acknowledgement timeout must be positive')
    if rclpy_module is None:
        import rclpy as rclpy_module  # type: ignore[no-redef]
    if string_type is None:
        from std_msgs.msg import String as string_type  # type: ignore[no-redef]

    task_topic = '/room_315/task_goal'
    status_topic = '/room_315/task_goal/status'
    node = None
    acknowledgement: dict[str, Any] | None = None
    published_count = 0
    discovery_started = time.monotonic()
    rclpy_module.init(args=[])
    try:
        node = rclpy_module.create_node(
            f'room315_v4_fault_goal_publisher_{os.getpid()}'
        )
        publisher = node.create_publisher(string_type, task_topic, 10)

        def receive_status(message: Any) -> None:
            nonlocal acknowledgement
            if published_count != 1 or acknowledgement is not None:
                return
            acknowledgement = parse_task_goal_acknowledgement(
                getattr(message, 'data', ''), goal_id=goal_id,
            )

        status_subscription = node.create_subscription(
            string_type, status_topic, receive_status, 10,
        )
        discovery_deadline = time.monotonic() + timeout_s
        while True:
            task_endpoints = node.get_subscriptions_info_by_topic(task_topic)
            status_endpoints = node.get_publishers_info_by_topic(status_topic)
            gateway_task_endpoints = [
                endpoint for endpoint in task_endpoints
                if str(endpoint.node_name) == 'room_315_task_execution_node'
            ]
            gateway_status_endpoints = [
                endpoint for endpoint in status_endpoints
                if str(endpoint.node_name) == 'room_315_task_execution_node'
            ]
            if gateway_task_endpoints and gateway_status_endpoints:
                break
            remaining = discovery_deadline - time.monotonic()
            if remaining <= 0.0:
                raise QualificationError(
                    'the exact task execution gateway DDS endpoints did not '
                    'match before publication; goal was not published'
                )
            rclpy_module.spin_once(
                node, timeout_sec=min(0.1, remaining),
            )

        message = string_type()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)
        published_count = 1
        published_at = time.monotonic()

        acknowledgement_deadline = published_at + timeout_s
        while acknowledgement is None:
            remaining = acknowledgement_deadline - time.monotonic()
            if remaining <= 0.0:
                raise QualificationError(
                    'TaskGoal was published exactly once but no matching status '
                    'acknowledgement arrived; refusing unsafe retry'
                )
            rclpy_module.spin_once(
                node, timeout_sec=min(0.1, remaining),
            )

        # Keep both endpoints referenced until after acknowledgement.
        _ = status_subscription
        return {
            'schema_version': 'room315.v4_fault_task_goal_publication.v1',
            'status': 'acknowledged',
            'goal_id': goal_id,
            'task_goal_topic': task_topic,
            'status_topic': status_topic,
            'published_count': published_count,
            'unsafe_retry_performed': False,
            'matched_gateway_task_subscription_count': len(
                gateway_task_endpoints
            ),
            'matched_gateway_status_publisher_count': len(
                gateway_status_endpoints
            ),
            'total_task_subscription_count': int(
                publisher.get_subscription_count()
            ),
            'total_status_publisher_count': int(
                node.count_publishers(status_topic)
            ),
            'discovery_duration_s': published_at - discovery_started,
            'acknowledgement_duration_s': time.monotonic() - published_at,
            'acknowledgement_status': acknowledgement['status'],
            'acknowledgement': acknowledgement,
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_module.ok():
            rclpy_module.shutdown()


def publish_task_goal_with_ack(
    payload: dict[str, Any],
    *,
    env: dict[str, str],
    timeout_s: float = 30.0,
) -> dict[str, Any]:
    """Run the persistent publisher in the selected isolated ROS domain."""

    command = [
        '/usr/bin/python3', str(SCRIPT_PATH), '_publish_task_goal',
        '--goal-json', json.dumps(payload, sort_keys=True),
        '--timeout-s', str(timeout_s),
    ]
    process = run_capture(
        command,
        env=env,
        timeout_s=(2.0 * timeout_s) + 15.0,
    )
    if process.get('returncode') != 0:
        detail = str(process.get('stderr') or process.get('stdout') or '').strip()
        raise QualificationError(
            f'persistent TaskGoal publication failed: {detail}'
        )
    lines = [
        line.strip() for line in str(process.get('stdout') or '').splitlines()
        if line.strip()
    ]
    try:
        evidence = json.loads(lines[-1]) if lines else None
    except json.JSONDecodeError as exc:
        raise QualificationError(
            f'TaskGoal publication evidence is invalid JSON: {exc}'
        ) from exc
    if not isinstance(evidence, dict):
        raise QualificationError('TaskGoal publication evidence is not an object')
    if (
        evidence.get('status') != 'acknowledged'
        or evidence.get('goal_id') != payload.get('goal_id')
        or evidence.get('published_count') != 1
        or evidence.get('unsafe_retry_performed') is not False
        or evidence.get('acknowledgement_status')
        not in TASK_GOAL_ACKNOWLEDGEMENT_STATUSES
    ):
        raise QualificationError(
            f'TaskGoal publication evidence is incompatible: {evidence}'
        )
    evidence['publisher_process'] = {
        'command': command,
        'returncode': process.get('returncode'),
        'timed_out': process.get('timed_out'),
        'duration_s': process.get('duration_s'),
        'stderr': process.get('stderr'),
    }
    return evidence


def parse_emergency_stop_acknowledgement(
    raw: str,
) -> dict[str, Any] | None:
    """Accept only the supervisor snapshot emitted after stop-all handling."""

    try:
        payload = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get('emergency_stop') is not True:
        return None
    expected_result = 'external emergency stop: all shuttles OFF'
    if payload.get('last_result') != expected_result:
        return None
    return {
        'emergency_stop': True,
        'last_result': expected_result,
    }


def publish_estop_once_wait_ack(
    *,
    timeout_s: float,
    delivery_hold_s: float,
    rclpy_module: Any | None = None,
    bool_type: Any | None = None,
    string_type: Any | None = None,
) -> dict[str, Any]:
    """Publish one e-stop after both safety-critical subscribers are matched."""

    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise QualificationError('emergency-stop acknowledgement timeout is invalid')
    if not math.isfinite(delivery_hold_s) or delivery_hold_s < 0.0:
        raise QualificationError('emergency-stop delivery hold is invalid')
    if rclpy_module is None:
        import rclpy as rclpy_module  # type: ignore[no-redef]
    if bool_type is None:
        from std_msgs.msg import Bool as bool_type  # type: ignore[no-redef]
    if string_type is None:
        from std_msgs.msg import String as string_type  # type: ignore[no-redef]

    estop_topic = '/room_315/rail_safety/emergency_stop'
    status_topic = '/room_315/rail_safety/status'
    node = None
    acknowledgement: dict[str, Any] | None = None
    published_count = 0
    discovery_started = time.monotonic()
    rclpy_module.init(args=[])
    try:
        node = rclpy_module.create_node(
            f'room315_v4_fault_estop_publisher_{os.getpid()}'
        )
        publisher = node.create_publisher(bool_type, estop_topic, 10)

        def receive_status(message: Any) -> None:
            nonlocal acknowledgement
            if published_count != 1 or acknowledgement is not None:
                return
            acknowledgement = parse_emergency_stop_acknowledgement(
                getattr(message, 'data', ''),
            )

        status_subscription = node.create_subscription(
            string_type, status_topic, receive_status, 10,
        )
        discovery_deadline = time.monotonic() + timeout_s
        while True:
            estop_endpoints = node.get_subscriptions_info_by_topic(estop_topic)
            status_endpoints = node.get_publishers_info_by_topic(status_topic)
            supervisor_estop_endpoints = [
                endpoint for endpoint in estop_endpoints
                if str(endpoint.node_name) == 'room_315_rail_safety_supervisor'
            ]
            recorder_estop_endpoints = [
                endpoint for endpoint in estop_endpoints
                if str(endpoint.node_name) == 'rosbag2_recorder'
            ]
            supervisor_status_endpoints = [
                endpoint for endpoint in status_endpoints
                if str(endpoint.node_name) == 'room_315_rail_safety_supervisor'
            ]
            compatible_estop_subscriptions = int(
                publisher.get_subscription_count()
            )
            compatible_status_publishers = int(
                node.count_publishers(status_topic)
            )
            if (
                supervisor_estop_endpoints
                and recorder_estop_endpoints
                and supervisor_status_endpoints
                and compatible_estop_subscriptions >= 2
                and compatible_status_publishers >= 1
            ):
                break
            remaining = discovery_deadline - time.monotonic()
            if remaining <= 0.0:
                raise QualificationError(
                    'supervisor and rosbag emergency-stop DDS endpoints did '
                    'not both match before publication; e-stop was not published'
                )
            rclpy_module.spin_once(
                node, timeout_sec=min(0.1, remaining),
            )

        message = bool_type()
        message.data = True
        publisher.publish(message)
        published_count = 1
        published_at = time.monotonic()

        acknowledgement_deadline = published_at + timeout_s
        while acknowledgement is None:
            remaining = acknowledgement_deadline - time.monotonic()
            if remaining <= 0.0:
                raise QualificationError(
                    'emergency stop was published exactly once but the '
                    'supervisor stop-all acknowledgement did not arrive; '
                    'refusing unsafe retry'
                )
            rclpy_module.spin_once(
                node, timeout_sec=min(0.1, remaining),
            )

        acknowledged_at = time.monotonic()
        hold_deadline = time.monotonic() + delivery_hold_s
        while True:
            remaining = hold_deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy_module.spin_once(
                node, timeout_sec=min(0.1, remaining),
            )

        _ = status_subscription
        return {
            'schema_version': 'room315.v4_fault_estop_publication.v1',
            'status': 'acknowledged',
            'emergency_stop': True,
            'published_count': published_count,
            'unsafe_retry_performed': False,
            'matched_supervisor_subscription_count': len(
                supervisor_estop_endpoints
            ),
            'matched_recorder_subscription_count': len(
                recorder_estop_endpoints
            ),
            'matched_supervisor_status_publisher_count': len(
                supervisor_status_endpoints
            ),
            'compatible_emergency_stop_subscription_count': (
                compatible_estop_subscriptions
            ),
            'compatible_supervisor_status_publisher_count': (
                compatible_status_publishers
            ),
            'delivery_hold_s': delivery_hold_s,
            'discovery_duration_s': published_at - discovery_started,
            'acknowledgement_duration_s': acknowledged_at - published_at,
            'acknowledgement': acknowledgement,
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_module.ok():
            rclpy_module.shutdown()


def publish_estop(
    *,
    env: dict[str, str],
    timeout_s: float = 30.0,
    delivery_hold_s: float = 1.0,
) -> dict[str, Any]:
    """Run the persistent e-stop publisher in the isolated ROS domain."""

    command = [
        '/usr/bin/python3', str(SCRIPT_PATH), '_publish_estop',
        '--timeout-s', str(timeout_s),
        '--delivery-hold-s', str(delivery_hold_s),
    ]
    process = run_capture(
        command,
        env=env,
        timeout_s=(2.0 * timeout_s) + delivery_hold_s + 15.0,
    )
    if process.get('returncode') != 0:
        detail = str(process.get('stderr') or process.get('stdout') or '').strip()
        raise QualificationError(
            f'persistent emergency-stop publication failed: {detail}'
        )
    lines = [
        line.strip() for line in str(process.get('stdout') or '').splitlines()
        if line.strip()
    ]
    try:
        evidence = json.loads(lines[-1]) if lines else None
    except json.JSONDecodeError as exc:
        raise QualificationError(
            f'emergency-stop publication evidence is invalid JSON: {exc}'
        ) from exc
    if not isinstance(evidence, dict):
        raise QualificationError(
            'emergency-stop publication evidence is not an object'
        )
    if (
        evidence.get('status') != 'acknowledged'
        or evidence.get('emergency_stop') is not True
        or evidence.get('published_count') != 1
        or evidence.get('unsafe_retry_performed') is not False
        or int(evidence.get('matched_supervisor_subscription_count') or 0) < 1
        or int(evidence.get('matched_recorder_subscription_count') or 0) < 1
        or int(
            evidence.get('matched_supervisor_status_publisher_count') or 0
        ) < 1
        or int(
            evidence.get('compatible_emergency_stop_subscription_count') or 0
        ) < 2
        or int(
            evidence.get('compatible_supervisor_status_publisher_count') or 0
        ) < 1
        or evidence.get('acknowledgement') != {
            'emergency_stop': True,
            'last_result': 'external emergency stop: all shuttles OFF',
        }
    ):
        raise QualificationError(
            f'emergency-stop publication evidence is incompatible: {evidence}'
        )
    evidence['publisher_process'] = {
        'command': command,
        'returncode': process.get('returncode'),
        'timed_out': process.get('timed_out'),
        'duration_s': process.get('duration_s'),
        'stderr': process.get('stderr'),
    }
    return evidence


def start_terminal_status_capture(
    goal_id: str,
    *,
    env: dict[str, str],
    log_path: Path,
) -> ManagedProcess:
    # The filter uses only String.data operations supported by ros2 topic echo.
    expression = (
        f'{goal_id!r} in m.data and '
        '(\'"status": "succeeded"\' in m.data or '
        '\'"status": "aborted"\' in m.data or '
        '\'"status": "failed"\' in m.data or '
        '\'"status": "rejected"\' in m.data)'
    )
    return start_process(
        'terminal_status_echo',
        [
            'ros2', 'topic', 'echo', '--no-daemon', '--full-length',
            '/room_315/task_goal/status', 'std_msgs/msg/String', '--once',
            '--filter', expression,
        ],
        env=env,
        log_path=log_path,
    )


def read_terminal_status(managed: ManagedProcess, *, timeout_s: float) -> dict[str, Any]:
    try:
        managed.process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        raise QualificationError('terminal TaskGoal status timed out') from exc
    managed.log_stream.flush()
    output = managed.log_path.read_text(encoding='utf-8', errors='replace')
    message = parse_ros_yaml(output)
    raw = message.get('data')
    if not isinstance(raw, str):
        raise QualificationError('terminal status has no String.data')
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise QualificationError(f'terminal status JSON is invalid: {exc}') from exc
    if not isinstance(payload, dict):
        raise QualificationError('terminal status payload is not an object')
    return payload


def start_actuating_command_capture(
    *,
    env: dict[str, str],
    log_path: Path,
) -> ManagedProcess:
    return start_process(
        'actuating_command_echo',
        [
            'ros2', 'topic', 'echo', '--no-daemon', '--full-length',
            '/room_315/rails/right/shuttles/command',
            'mfja_rail_interfaces/msg/ShuttleCommand', '--once',
            '--filter',
            (
                "m.name == 'room315_right_shuttle_4' and "
                "m.command in ['ON', 'ENABLE', 'ENABLED', 'START', 'RUN']"
            ),
        ],
        env=env,
        log_path=log_path,
    )


def read_actuating_command(
    managed: ManagedProcess,
    *,
    timeout_s: float,
) -> dict[str, Any]:
    try:
        managed.process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        stop_process(managed, timeout_s=1.0)
        raise QualificationError('long move never emitted an actuating command') from exc
    managed.log_stream.flush()
    payload = parse_ros_yaml(
        managed.log_path.read_text(encoding='utf-8', errors='replace')
    )
    stop_process(managed, timeout_s=1.0)
    if str(payload.get('command') or '').upper() not in {
        'ON', 'ENABLE', 'ENABLED', 'START', 'RUN',
    }:
        raise QualificationError(f'captured command is not actuating: {payload}')
    return payload


def create_fault_runtime_config(
    scenario: FaultScenario,
    *,
    case_dir: Path,
) -> tuple[Path, list[Path]]:
    try:
        root = yaml.safe_load(TASK_RUNTIME_CONFIG.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise QualificationError(f'cannot load bound task runtime config: {exc}') from exc
    if not isinstance(root, dict) or len(root) != 1:
        raise QualificationError('task runtime config has an unexpected root')
    node_name, node_payload = next(iter(root.items()))
    if not isinstance(node_payload, dict):
        raise QualificationError('task runtime node payload is invalid')
    parameters = node_payload.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise QualificationError('task runtime parameters are missing')
    derived_files: list[Path] = []
    parameters['execution_enabled'] = True
    if scenario.fault == 'corrupt_authorization':
        corrupt = case_dir / 'fault_inputs' / 'corrupt_authorization.json'
        corrupt.parent.mkdir(parents=True, exist_ok=True)
        write_json_exclusive(corrupt, {'corrupt': True}, mode=0o444)
        os.chmod(corrupt, 0o444)
        parameters['task_execution_authorization_path'] = str(corrupt)
        parameters['task_execution_authorization_sha256'] = sha256_file(corrupt)
        derived_files.append(corrupt)
    elif scenario.fault == 'wrong_promotion_manifest':
        wrong = case_dir / 'fault_inputs' / 'wrong_promotion_manifest.json'
        wrong.parent.mkdir(parents=True, exist_ok=True)
        write_json_exclusive(
            wrong,
            {
                'schema_version': 'room315.visual_runtime_promotion.v4.v1',
                'deployment_mode': 'shadow',
                'physical_deployment_approved': False,
            },
            mode=0o444,
        )
        os.chmod(wrong, 0o444)
        parameters['task_execution_promotion_manifest_path'] = str(wrong)
        derived_files.append(wrong)
    elif scenario.fault == 'right_sensor_feedback_loss':
        parameters['right_sensor_feedback_topic'] = feedback_loss_topic(scenario)
    elif scenario.fault not in {
        'planner_unavailable', 'emergency_stop_during_long_move',
    }:
        raise QualificationError(f'unsupported fault mutation: {scenario.fault}')
    derived = case_dir / 'fault_inputs' / 'task_execution_runtime.yaml'
    derived.parent.mkdir(parents=True, exist_ok=True)
    raw = yaml.safe_dump(
        {node_name: node_payload}, sort_keys=False,
    ).encode('utf-8')
    write_bytes_exclusive(derived, raw, mode=0o444)
    os.chmod(derived, 0o444)
    derived_files.append(derived)
    return derived, derived_files


def feedback_loss_topic(scenario: FaultScenario) -> str:
    return f'/room_315/fault/{scenario.scenario_id}/right_sensor_blackhole'


def verify_feedback_loss_runtime_config(
    scenario: FaultScenario,
    runtime_config: Path,
) -> dict[str, Any]:
    if scenario.fault != 'right_sensor_feedback_loss':
        raise QualificationError('feedback configuration proof requires F05')
    try:
        root = yaml.safe_load(runtime_config.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise QualificationError(
            f'cannot verify feedback-loss runtime config: {exc}'
        ) from exc
    if not isinstance(root, dict) or len(root) != 1:
        raise QualificationError('feedback-loss runtime config root is invalid')
    node_payload = next(iter(root.values()))
    parameters = (
        node_payload.get('ros__parameters')
        if isinstance(node_payload, dict) else None
    )
    configured = (
        parameters.get('right_sensor_feedback_topic')
        if isinstance(parameters, dict) else None
    )
    expected = feedback_loss_topic(scenario)
    if configured != expected:
        raise QualificationError(
            f'feedback-loss topic is not the exact black hole: {configured!r}'
        )
    return {
        'status': 'configured',
        'topic': expected,
        'runtime_config_path': str(runtime_config),
        'runtime_config_sha256': sha256_file(runtime_config),
    }


RECORDED_TOPICS = (
    '/clock',
    '/diagnostics',
    '/room_315/task_goal',
    '/room_315/task_goal/status',
    '/room_315/visual_state/observed_state',
    '/room_315/rail_safety/primitive_command',
    '/room_315/rail_safety/status',
    '/room_315/rail_safety/emergency_stop',
    '/room_315/rails/right/shuttles/state',
    '/room_315/rails/right/shuttles/command',
    '/room_315/rails/right/sensors/feedback',
)


def bag_command(case_dir: Path, scenario: FaultScenario) -> list[str]:
    return [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--storage-preset-profile', 'zstd_fast',
        '--use-sim-time',
        '--disable-keyboard-controls',
        '--output', str(case_dir / 'rosbag'),
        '--custom-data', 'campaign=room315_v4_closed_loop_faults',
        f'scenario_id={scenario.scenario_id}',
        '--topics', *RECORDED_TOPICS,
    ]


def audit_rosbag(bag_dir: Path) -> dict[str, Any]:
    """Deserialize safety-critical topics and reject mixed visual provenance."""

    if not (bag_dir / 'metadata.yaml').is_file():
        raise QualificationError(f'rosbag metadata is missing: {bag_dir}')
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise QualificationError(
            f'ROS bag Python bindings are unavailable: {exc}'
        ) from exc

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir), storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required_topics = {
        '/room_315/visual_state/observed_state',
        '/room_315/rails/right/shuttles/state',
    }
    missing = sorted(required_topics - set(topic_types))
    if missing:
        raise QualificationError(f'rosbag is missing required topics: {missing}')
    message_types: dict[str, Any] = {}
    visual_count = 0
    v3_count = 0
    wrong_visual_rows: list[dict[str, Any]] = []
    shuttle_commands: list[dict[str, Any]] = []
    shuttle_states: list[dict[str, Any]] = []
    task_statuses: list[dict[str, Any]] = []
    estop_values: list[bool] = []
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        type_name = topic_types.get(topic)
        if not type_name:
            continue
        message_type = message_types.get(type_name)
        if message_type is None:
            message_type = get_message(type_name)
            message_types[type_name] = message_type
        message = deserialize_message(data, message_type)
        if topic == '/room_315/visual_state/observed_state':
            visual_count += 1
            schema = str(message.schema_version)
            checkpoint = str(message.checkpoint_sha256)
            if schema == 'room315.visual_state.v3':
                v3_count += 1
            if schema != V4_SCHEMA or checkpoint != V4_CHECKPOINT_SHA256:
                wrong_visual_rows.append({
                    'timestamp_ns': int(timestamp_ns),
                    'schema_version': schema,
                    'checkpoint_sha256': checkpoint,
                })
        elif topic == '/room_315/rails/right/shuttles/command':
            shuttle_commands.append({
                'timestamp_ns': int(timestamp_ns),
                'name': str(message.name),
                'command': str(message.command),
                'speed': float(message.speed),
                'target_slot': str(message.target_slot),
            })
        elif topic == '/room_315/rails/right/shuttles/state':
            if str(message.name) == 'room315_right_shuttle_4':
                shuttle_states.append({
                    'timestamp_ns': int(timestamp_ns),
                    'name': str(message.name),
                    'mode': str(message.mode),
                    's': float(message.s),
                    'reached_target_slot': str(message.reached_target_slot),
                })
        elif topic == '/room_315/task_goal/status':
            try:
                payload = json.loads(str(message.data))
            except json.JSONDecodeError:
                payload = {'invalid_json': str(message.data)}
            if isinstance(payload, dict):
                payload = dict(payload)
                payload['bag_timestamp_ns'] = int(timestamp_ns)
                task_statuses.append(payload)
        elif topic == '/room_315/rail_safety/emergency_stop':
            estop_values.append(bool(message.data))

    if visual_count < 1:
        raise QualificationError('rosbag contains no accepted V4 observation')
    if v3_count or wrong_visual_rows:
        raise QualificationError(
            'rosbag visual provenance violation: '
            f'v3_count={v3_count}, wrong={wrong_visual_rows[:5]}'
        )
    actuating_values = {'ON', 'ENABLE', 'ENABLED', 'START', 'RUN'}
    actuating = [
        item for item in shuttle_commands
        if str(item['command']).upper() in actuating_values
    ]
    terminal = [
        item for item in task_statuses
        if str(item.get('status') or '').casefold() in TERMINAL_STATUSES
    ]
    successes = [
        item for item in task_statuses
        if str(item.get('status') or '').casefold() == 'succeeded'
    ]
    return {
        'status': 'passed',
        'visual_observation_count': visual_count,
        'v4_observation_count': visual_count,
        'v3_observation_count': v3_count,
        'wrong_visual_provenance_count': len(wrong_visual_rows),
        'visual_schema_version': V4_SCHEMA,
        'visual_checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'shuttle_command_count': len(shuttle_commands),
        'actuating_command_count': len(actuating),
        'shuttle_commands': shuttle_commands,
        'r4_state_count': len(shuttle_states),
        'r4_states': shuttle_states,
        'task_status_count': len(task_statuses),
        'terminal_status_count': len(terminal),
        'success_status_count': len(successes),
        'terminal_statuses': terminal,
        'emergency_stop_values': estop_values,
    }


def validate_feedback_unavailable_proof(
    scenario: FaultScenario,
    proof: dict[str, Any] | None,
) -> bool:
    if scenario.fault != 'right_sensor_feedback_loss':
        return False
    if not isinstance(proof, dict) or proof.get('status') != 'passed':
        raise QualificationError(
            f'{scenario.scenario_id} has no feedback unavailability proof'
        )
    expected_topic = feedback_loss_topic(scenario)
    configuration = proof.get('configuration')
    if (
        not isinstance(configuration, dict)
        or configuration.get('status') != 'configured'
        or configuration.get('topic') != expected_topic
        or not SHA256_PATTERN.fullmatch(
            str(configuration.get('runtime_config_sha256') or '')
        )
    ):
        raise QualificationError(
            f'{scenario.scenario_id} feedback black-hole configuration is unproved'
        )
    for phase in ('before_goal', 'after_terminal'):
        observation = proof.get(phase)
        if (
            not isinstance(observation, dict)
            or observation.get('status') != 'unavailable'
            or observation.get('topic') != expected_topic
            or observation.get('publisher_count') != 0
            or int(observation.get('subscription_count') or 0) < 1
        ):
            raise QualificationError(
                f'{scenario.scenario_id} feedback unavailability is unproved '
                f'at {phase}'
            )
    return True


def validate_fault_outcome(
    scenario: FaultScenario,
    *,
    gateway_returncode: int | None,
    terminal_status: dict[str, Any] | None,
    bag_audit: dict[str, Any],
    stationary_proof: dict[str, Any] | None,
    final_controller_state: dict[str, Any],
    gateway_refusal_reason: str = '',
    feedback_unavailable_proof: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Apply the same strict assertions in live runs and focused tests."""

    success_count = int(bag_audit.get('success_status_count') or 0)
    actuating_count = int(bag_audit.get('actuating_command_count') or 0)
    if success_count != 0:
        raise QualificationError(
            f'{scenario.scenario_id} produced a false success status'
        )
    if int(bag_audit.get('v3_observation_count') or 0) != 0:
        raise QualificationError(f'{scenario.scenario_id} recorded a V3 observation')
    if int(bag_audit.get('wrong_visual_provenance_count') or 0) != 0:
        raise QualificationError(
            f'{scenario.scenario_id} recorded wrong V4 provenance'
        )
    if int(bag_audit.get('v4_observation_count') or 0) < 1:
        raise QualificationError(
            f'{scenario.scenario_id} recorded no V4 observation'
        )
    final_mode = str(final_controller_state.get('mode') or '').upper()
    if scenario.require_disabled_controller and final_mode != 'DISABLED':
        raise QualificationError(
            f'{scenario.scenario_id} controller final mode is {final_mode!r}'
        )

    feedback_unavailable_proved = False
    if scenario.expected_terminal == 'gateway_refused':
        if gateway_returncode is None or not gateway_refusal_reason.strip():
            raise QualificationError(
                f'{scenario.scenario_id} gateway did not fail closed at startup'
            )
        if terminal_status is not None:
            raise QualificationError(
                f'{scenario.scenario_id} unexpectedly received a task terminal status'
            )
        if actuating_count != 0 or stationary_proof is None:
            raise QualificationError(
                f'{scenario.scenario_id} gateway refusal did not prove zero motion'
            )
    else:
        if gateway_returncode is not None:
            raise QualificationError(
                f'{scenario.scenario_id} gateway exited before outcome: '
                f'{gateway_returncode}'
            )
        if not isinstance(terminal_status, dict):
            raise QualificationError(
                f'{scenario.scenario_id} has no terminal TaskGoal response'
            )
        status = str(terminal_status.get('status') or '').casefold()
        if status not in NON_SUCCESS_STATUSES:
            raise QualificationError(
                f'{scenario.scenario_id} terminal status is not non-success: {status}'
            )
        if scenario.expect_actuating_command:
            if actuating_count < 1:
                raise QualificationError(
                    f'{scenario.scenario_id} never began its deliberate long move'
                )
            estop_values = bag_audit.get('emergency_stop_values', [])
            if scenario.inject_emergency_stop:
                if True not in estop_values:
                    raise QualificationError(
                        f'{scenario.scenario_id} did not record the injected e-stop'
                    )
            else:
                if True in estop_values:
                    raise QualificationError(
                        f'{scenario.scenario_id} unexpectedly injected an e-stop'
                    )
                if not scenario.sensor_feedback_available:
                    feedback_unavailable_proved = (
                        validate_feedback_unavailable_proof(
                            scenario, feedback_unavailable_proof,
                        )
                    )
        elif actuating_count != 0 or stationary_proof is None:
            raise QualificationError(
                f'{scenario.scenario_id} did not remain stationary before rejection'
            )

    return {
        'status': 'passed',
        'scenario_id': scenario.scenario_id,
        'fault': scenario.fault,
        'gateway_refused': scenario.expected_terminal == 'gateway_refused',
        'terminal_status': (
            terminal_status.get('status') if terminal_status else None
        ),
        'false_success_count': success_count,
        'actuating_command_count': actuating_count,
        'zero_motion_proved': stationary_proof is not None,
        'emergency_stop_injected': scenario.inject_emergency_stop,
        'feedback_unavailable_proved': feedback_unavailable_proved,
        'controller_final_mode': final_mode,
        'v4_observation_count': bag_audit['v4_observation_count'],
        'v3_observation_count': bag_audit['v3_observation_count'],
        'physical_deployment': False,
    }


def verify_runtime_lock_free() -> None:
    descriptor = os.open(RUNTIME_LOCK, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.lseek(descriptor, 0, os.SEEK_SET)
            owner = os.read(descriptor, 64).decode(
                'ascii', errors='ignore',
            ).strip()
            raise QualificationError(
                'Room 315 runtime lock is already held; '
                f'owner={owner or "unknown"}'
            ) from exc
        fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        os.close(descriptor)


def verify_empty_graph(env: dict[str, str]) -> dict[str, Any]:
    nodes = run_capture(
        ['ros2', 'node', 'list', '--no-daemon'], env=env, timeout_s=10.0,
    )
    topics = run_capture(
        ['ros2', 'topic', 'list', '--no-daemon'], env=env, timeout_s=10.0,
    )
    if nodes.get('returncode') != 0 or topics.get('returncode') != 0:
        raise QualificationError('could not inspect the pre-existing ROS graph')
    node_names = sorted(
        line.strip() for line in str(nodes.get('stdout') or '').splitlines()
        if line.strip()
    )
    topic_names = sorted(
        line.strip() for line in str(topics.get('stdout') or '').splitlines()
        if line.strip() and line.strip() not in {'/parameter_events', '/rosout'}
    )
    if node_names or topic_names:
        raise QualificationError(
            f'ROS domain is not empty: nodes={node_names}, topics={topic_names}'
        )
    return {'status': 'empty', 'nodes': node_names, 'topics': topic_names}


def wait_for_planner(env: dict[str, str], timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    attempts: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        lifecycle = run_capture(
            ['ros2', 'lifecycle', 'get', '/planner'],
            env=env,
            timeout_s=5.0,
        )
        service = run_capture(
            ['ros2', 'service', 'type', '/planner/get_plan'],
            env=env,
            timeout_s=5.0,
        )
        attempts.append({'lifecycle': lifecycle, 'service': service})
        if (
            lifecycle.get('returncode') == 0
            and re.fullmatch(
                r'active\s+\[(\d+)\]',
                str(lifecycle.get('stdout') or '').strip(),
                flags=re.IGNORECASE,
            )
            and service.get('returncode') == 0
            and str(service.get('stdout') or '').strip()
            == 'plansys2_msgs/srv/GetPlan'
        ):
            return {'status': 'ready', 'attempts': attempts}
        time.sleep(0.25)
    raise QualificationError('PlanSys2 did not become active')


def validate_live_visual(payload: dict[str, Any]) -> dict[str, Any]:
    required_true = (
        'accepted', 'model_ready', 'input_ready', 'presence_ready',
        'state_fusion_ready',
    )
    missing = [name for name in required_true if payload.get(name) is not True]
    if missing:
        raise QualificationError(f'V4 observation is not ready: {missing}')
    if payload.get('schema_version') != V4_SCHEMA:
        raise QualificationError(
            f'live observation schema is {payload.get("schema_version")!r}'
        )
    if payload.get('checkpoint_sha256') != V4_CHECKPOINT_SHA256:
        raise QualificationError('live observation checkpoint hash drifted')
    if payload.get('stage') != 'fused_observed_state':
        raise QualificationError('live observation is not the fused state')
    if payload.get('stale') is not False:
        raise QualificationError('live observation is stale')
    if payload.get('validation_reasons') != []:
        raise QualificationError(
            f'live observation has validation reasons: '
            f'{payload.get("validation_reasons")}'
        )
    present = {
        str(item.get('identity') or '')
        for item in payload.get('shuttles') or []
        if isinstance(item, dict)
        and str(item.get('presence_state') or '').casefold() == 'present'
    }
    if present != {'R4'}:
        raise QualificationError(f'live V4 presence is not exactly R4: {present}')
    return {
        'status': 'passed',
        'schema_version': payload['schema_version'],
        'checkpoint_sha256': payload['checkpoint_sha256'],
        'accepted_frame_count': int(payload.get('accepted_frame_count') or 0),
        'present_identities': sorted(present),
    }


def gateway_refusal_reason(
    scenario: FaultScenario,
    log_text: str,
) -> str:
    lowered = log_text.casefold()
    if scenario.fault == 'corrupt_authorization':
        required = ('authorization',)
        rejection_markers = (
            'mismatch',
            'invalid',
            'reject',
            'missing field',
            'incompatible field set',
        )
    elif scenario.fault == 'wrong_promotion_manifest':
        required = ('promotion manifest', 'sha-256')
        rejection_markers = ('mismatch',)
    else:
        return ''
    if (
        all(value in lowered for value in required)
        and any(value in lowered for value in rejection_markers)
    ):
        matching = [
            line.strip() for line in log_text.splitlines()
            if all(value in line.casefold() for value in required[:1])
        ]
        return matching[-1][-2000:] if matching else scenario.fault
    return ''


def publish_stop_all(env: dict[str, str], reason: str) -> dict[str, Any]:
    return publish_string(
        '/room_315/rail_safety/primitive_command',
        {
            'action': 'stop_all',
            'reason': reason,
            'source': 'room315_v4_closed_loop_fault_campaign',
        },
        env=env,
    )


def chmod_tree_read_only(root: Path) -> None:
    for path in sorted(root.rglob('*'), reverse=True):
        try:
            if path.is_dir():
                os.chmod(path, 0o555)
            elif path.is_file():
                executable = bool(path.stat().st_mode & stat.S_IXUSR)
                os.chmod(path, 0o555 if executable else 0o444)
        except FileNotFoundError:
            continue
    os.chmod(root, 0o555)


def finalise_evidence(root: Path, summary: dict[str, Any]) -> None:
    write_json_replace(root / 'summary.json', summary)
    files = sorted(
        path for path in root.rglob('*')
        if path.is_file() and path.name not in {'manifest.json', 'SHA256SUMS'}
    )
    rows = [
        {
            'path': str(path.relative_to(root)),
            'size_bytes': path.stat().st_size,
            'sha256': sha256_file(path),
        }
        for path in files
    ]
    manifest = {
        'schema_version': 'room315.v4_closed_loop_fault_evidence.v1',
        'created_at_utc': utc_now(),
        'campaign_schema': CAMPAIGN_SCHEMA,
        'campaign_status': summary.get('status'),
        'physical_deployment': False,
        'file_count_excluding_manifest_and_checksums': len(rows),
        'files': rows,
    }
    write_json_replace(root / 'manifest.json', manifest)
    checksum_paths = sorted(
        path for path in root.rglob('*')
        if path.is_file() and path.name != 'SHA256SUMS'
    )
    lines = [
        f'{sha256_file(path)}  {path.relative_to(root)}'
        for path in checksum_paths
    ]
    checksum_path = root / 'SHA256SUMS'
    checksum_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    chmod_tree_read_only(root)


def run_worker(
    scenario: FaultScenario,
    *,
    case_dir: Path,
    domain_id: int,
) -> dict[str, Any]:
    if not case_dir.is_dir():
        raise QualificationError(f'worker case directory is missing: {case_dir}')
    started = time.monotonic()
    runtime_root = Path(
        tempfile.mkdtemp(prefix=f'room315_v4_fault_{scenario.scenario_id}_')
    )
    base_env = clean_environment(domain_id, runtime_root / 'ros_logs_base')
    processes: list[ManagedProcess] = []
    auxiliary_processes: list[ManagedProcess] = []
    bag: ManagedProcess | None = None
    bag_stopped = False
    teardown: list[dict[str, Any]] = []
    terminal_status: dict[str, Any] | None = None
    stationary_proof: dict[str, Any] | None = None
    feedback_unavailable_proof: dict[str, Any] | None = None
    final_state: dict[str, Any] = {}
    gateway_returncode: int | None = None
    refusal_reason = ''
    result: dict[str, Any] = {
        'schema_version': CASE_SCHEMA,
        'scenario_id': scenario.scenario_id,
        'fault': scenario.fault,
        'status': 'failed',
        'ros_domain_id': domain_id,
        'authorization_scope': AUTHORIZATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'qualification_manifest_sha256': V4_PROMOTION_MANIFEST_SHA256,
        'task_runtime_config_sha256': TASK_RUNTIME_CONFIG_SHA256,
        'physical_deployment': False,
        'started_at_utc': utc_now(),
    }
    try:
        verify_runtime_lock_free()
        graph = verify_empty_graph(base_env)
        write_json_replace(case_dir / 'preexisting_graph.json', graph)
        runtime_config, derived_files = create_fault_runtime_config(
            scenario, case_dir=case_dir,
        )
        if scenario.fault == 'right_sensor_feedback_loss':
            feedback_unavailable_proof = {
                'schema_version': 'room315.v4_feedback_unavailable_proof.v1',
                'status': 'pending',
                'configuration': verify_feedback_loss_runtime_config(
                    scenario, runtime_config,
                ),
            }
        write_json_replace(case_dir / 'derived_input_hashes.json', {
            'files': [
                {
                    'path': str(path),
                    'size_bytes': path.stat().st_size,
                    'sha256': sha256_file(path),
                    'mode': stat.S_IMODE(path.stat().st_mode),
                }
                for path in derived_files
            ],
        })
        floor = start_process(
            'floor', floor_command(),
            env=clean_environment(domain_id, runtime_root / 'ros_logs_floor'),
            log_path=case_dir / 'logs' / 'floor.log',
        )
        processes.append(floor)
        clock = wait_topic(
            '/clock', 'rosgraph_msgs/msg/Clock',
            env=base_env, timeout_s=120.0,
        )
        write_json_replace(case_dir / 'readiness_clock.json', clock)
        require_alive(floor, label='Gazebo floor')
        initial_state = capture_shuttle_state(
            env=base_env, timeout_s=60.0, require_mode='DISABLED',
        )
        write_json_replace(case_dir / 'initial_controller_state.json', initial_state)

        bag = start_process(
            'rosbag', bag_command(case_dir, scenario),
            env=clean_environment(domain_id, runtime_root / 'ros_logs_bag'),
            log_path=case_dir / 'logs' / 'rosbag.log',
        )
        time.sleep(2.0)
        require_alive(bag, label='rosbag recorder')

        visual = start_process(
            'visual', visual_command(),
            env=visual_environment(
                clean_environment(domain_id, runtime_root / 'ros_logs_visual')
            ),
            log_path=case_dir / 'logs' / 'visual.log',
        )
        processes.append(visual)
        visual_payload = wait_topic(
            '/room_315/visual_state/observed_state',
            'mfja_rail_interfaces/msg/VisualStateObservation',
            env=base_env, timeout_s=120.0,
        )
        visual_check = validate_live_visual(visual_payload)
        write_json_replace(case_dir / 'visual_readiness.json', visual_check)
        require_alive(visual, label='V4 visual runtime')

        execution = start_process(
            'execution',
            execution_command(
                runtime_config=runtime_config,
                planner_enabled=scenario.planner_enabled,
            ),
            env=clean_environment(domain_id, runtime_root / 'ros_logs_execution'),
            log_path=case_dir / 'logs' / 'execution.log',
        )
        processes.append(execution)

        if not scenario.expect_gateway_start:
            gateway_returncode = wait_for_process_exit(execution, timeout_s=30.0)
            execution.log_stream.flush()
            log_text = execution.log_path.read_text(
                encoding='utf-8', errors='replace',
            )
            refusal_reason = gateway_refusal_reason(scenario, log_text)
            if not refusal_reason:
                raise QualificationError(
                    f'{scenario.scenario_id} gateway exit lacked the declared '
                    'authorization/manifest refusal evidence'
                )
            time.sleep(3.0)
        else:
            subscriber = wait_for_subscription(
                '/room_315/task_goal', env=base_env, timeout_s=60.0,
            )
            write_json_replace(case_dir / 'task_subscriber.json', subscriber)
            require_alive(execution, label='task execution gateway')
            if feedback_unavailable_proof is not None:
                feedback_unavailable_proof['before_goal'] = (
                    wait_for_unpublished_feedback_subscription(
                        feedback_loss_topic(scenario),
                        env=base_env,
                        timeout_s=30.0,
                    )
                )
                write_json_replace(
                    case_dir / 'feedback_unavailable_proof.json',
                    feedback_unavailable_proof,
                )
            if scenario.planner_enabled:
                planner = wait_for_planner(base_env, timeout_s=60.0)
                write_json_replace(case_dir / 'planner_readiness.json', planner)
            else:
                planner_service = run_capture(
                    ['ros2', 'service', 'type', '/planner/get_plan'],
                    env=base_env,
                    timeout_s=5.0,
                )
                write_json_replace(
                    case_dir / 'planner_unavailable_proof.json', planner_service,
                )
                if planner_service.get('returncode') == 0:
                    raise QualificationError(
                        'planner-unavailable scenario unexpectedly found the service'
                    )
            goal = long_move_task_goal()
            terminal_capture = start_terminal_status_capture(
                goal['goal_id'],
                env=base_env,
                log_path=case_dir / 'logs' / 'terminal_status_echo.log',
            )
            auxiliary_processes.append(terminal_capture)
            actuating_capture: ManagedProcess | None = None
            if scenario.expect_actuating_command:
                actuating_capture = start_actuating_command_capture(
                    env=base_env,
                    log_path=case_dir / 'logs' / 'actuating_command_echo.log',
                )
                auxiliary_processes.append(actuating_capture)
            time.sleep(1.0)
            publish_result = publish_task_goal_with_ack(
                goal, env=base_env,
            )
            write_json_replace(case_dir / 'task_publish.json', publish_result)
            if scenario.expect_actuating_command:
                if actuating_capture is None:
                    raise QualificationError(
                        'internal error: actuating command capture was not started'
                    )
                actuating = read_actuating_command(
                    actuating_capture,
                    timeout_s=120.0,
                )
                auxiliary_processes.remove(actuating_capture)
                write_json_replace(case_dir / 'actuating_command.json', actuating)
                if scenario.inject_emergency_stop:
                    estop = publish_estop(env=base_env)
                    write_json_replace(
                        case_dir / 'emergency_stop_publish.json', estop,
                    )
            terminal_status = read_terminal_status(
                terminal_capture,
                timeout_s=120.0,
            )
            stop_process(terminal_capture, timeout_s=1.0)
            auxiliary_processes.remove(terminal_capture)
            write_json_replace(case_dir / 'terminal_status.json', terminal_status)
            if terminal_status.get('goal_id') != goal['goal_id']:
                raise QualificationError('terminal status belongs to another TaskGoal')
            require_alive(execution, label='task execution gateway after fault')
            if feedback_unavailable_proof is not None:
                feedback_unavailable_proof['after_terminal'] = (
                    wait_for_unpublished_feedback_subscription(
                        feedback_loss_topic(scenario),
                        env=base_env,
                        timeout_s=30.0,
                    )
                )
                feedback_unavailable_proof['status'] = 'passed'
                write_json_replace(
                    case_dir / 'feedback_unavailable_proof.json',
                    feedback_unavailable_proof,
                )

        final_state = capture_shuttle_state(
            env=base_env, timeout_s=30.0, require_mode='DISABLED',
        )
        write_json_replace(case_dir / 'final_controller_state.json', final_state)
        if not scenario.expect_actuating_command:
            stationary_proof = verify_stationary(initial_state, final_state)
            write_json_replace(case_dir / 'stationary_proof.json', stationary_proof)

        if bag is None:
            raise QualificationError('internal error: rosbag was never started')
        bag_stop = stop_process(bag, timeout_s=20.0)
        bag_stopped = True
        teardown.append(bag_stop)
        write_json_replace(case_dir / 'rosbag_stop.json', bag_stop)
        if bag_stop.get('returncode') != 0:
            raise QualificationError('rosbag did not stop cleanly')
        bag_audit = audit_rosbag(case_dir / 'rosbag')
        write_json_replace(case_dir / 'rosbag_audit.json', bag_audit)
        validation = validate_fault_outcome(
            scenario,
            gateway_returncode=gateway_returncode,
            terminal_status=terminal_status,
            bag_audit=bag_audit,
            stationary_proof=stationary_proof,
            final_controller_state=final_state,
            gateway_refusal_reason=refusal_reason,
            feedback_unavailable_proof=feedback_unavailable_proof,
        )
        write_json_replace(case_dir / 'validation.json', validation)
        result.update(validation)
        result['status'] = 'passed'
        result['gateway_returncode'] = gateway_returncode
        result['gateway_refusal_reason'] = refusal_reason
        result['bag_audit'] = bag_audit
        return result
    except Exception as exc:
        result['status'] = 'failed'
        result['failure_reason'] = str(exc)
        raise
    finally:
        if bag is not None and not bag_stopped:
            try:
                teardown.append(stop_process(bag, timeout_s=20.0))
            except Exception as exc:  # noqa: BLE001 - continue safe teardown
                teardown.append({'name': 'rosbag', 'teardown_error': str(exc)})
        try:
            stop_all = publish_stop_all(
                base_env,
                f'v4_fault_qualification_{scenario.scenario_id}_complete',
            )
            write_json_replace(case_dir / 'stop_all.json', stop_all)
        except Exception as exc:  # noqa: BLE001 - process teardown must continue
            write_json_replace(case_dir / 'stop_all_error.json', {'error': str(exc)})
        for managed in reversed(auxiliary_processes):
            try:
                teardown.append(stop_process(managed, timeout_s=1.0))
            except Exception as exc:  # noqa: BLE001 - preserve remaining logs
                teardown.append({
                    'name': managed.name,
                    'teardown_error': str(exc),
                })
        for managed in reversed(processes):
            try:
                teardown.append(stop_process(managed))
            except Exception as exc:  # noqa: BLE001 - preserve remaining logs
                teardown.append({
                    'name': managed.name,
                    'teardown_error': str(exc),
                })
        result['finished_at_utc'] = utc_now()
        result['duration_s'] = time.monotonic() - started
        write_json_replace(case_dir / 'teardown.json', teardown)
        write_json_replace(case_dir / 'case_summary.json', result)
        shutil.rmtree(runtime_root, ignore_errors=True)


def _worker_shell_command(
    *,
    scenario: FaultScenario,
    case_dir: Path,
    domain_id: int,
) -> list[str]:
    # The positional arguments avoid interpolating paths into shell source.
    shell = (
        'set -e; '
        'source "$1"; '
        'exec /usr/bin/python3 "$2" _worker '
        '--scenario "$3" --case-dir "$4" --domain-id "$5"'
    )
    return [
        'bash', '-c', shell, 'room315-v4-fault-worker',
        str(INSTALL_SETUP), str(SCRIPT_PATH), scenario.scenario_id,
        str(case_dir), str(domain_id),
    ]


def run_campaign(
    *,
    source_lock: Path,
    output_root: Path,
    base_domain_id: int,
    selected_ids: list[str],
) -> dict[str, Any]:
    validate_scenarios(FAULT_SCENARIOS)
    if not INSTALL_SETUP.is_file():
        raise QualificationError(f'workspace install is missing: {INSTALL_SETUP}')
    locked = verify_source_lock(source_lock)
    templates = verify_command_templates()
    output_root = output_root.expanduser().resolve()
    if output_root.exists():
        raise QualificationError(
            f'refusing to reuse qualification output: {output_root}'
        )
    selected = (
        [scenario_by_id(value) for value in selected_ids]
        if selected_ids else list(FAULT_SCENARIOS)
    )
    if len({item.scenario_id for item in selected}) != len(selected):
        raise QualificationError('selected fault scenarios must be unique')
    if base_domain_id < 0 or base_domain_id + len(selected) - 1 > 232:
        raise QualificationError('ROS domain allocation must stay in 0..232')
    output_root.mkdir(parents=True, exist_ok=False)
    write_bytes_exclusive(
        output_root / 'source_lock.json', source_lock.read_bytes(), mode=0o444,
    )
    write_json_replace(output_root / 'source_lock_verification.json', locked)
    write_json_replace(output_root / 'fixed_artifact_verification.json',
                       verify_fixed_artifacts())
    write_json_replace(output_root / 'command_templates.json', templates)
    declaration = {
        'schema_version': CAMPAIGN_SCHEMA,
        'created_at_utc': utc_now(),
        'authorization_scope': AUTHORIZATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'visual_checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'qualification_manifest_sha256': V4_PROMOTION_MANIFEST_SHA256,
        'task_runtime_config_sha256': TASK_RUNTIME_CONFIG_SHA256,
        'physical_deployment': False,
        'independent_cold_start_per_case': True,
        'reuse_existing_runtime_state': False,
        'fail_on_first_unexpected_outcome': True,
        'selected_scenario_ids': [item.scenario_id for item in selected],
        'scenarios': [item.__dict__ for item in selected],
    }
    write_json_replace(output_root / 'declaration.json', declaration)
    started_at_utc = utc_now()
    results: list[dict[str, Any]] = []
    status = 'passed' if len(selected) == len(FAULT_SCENARIOS) else 'partial'
    failure_reason = ''
    try:
        for index, scenario in enumerate(selected):
            case_dir = output_root / 'cases' / scenario.scenario_id
            case_dir.mkdir(parents=True, exist_ok=False)
            command = _worker_shell_command(
                scenario=scenario,
                case_dir=case_dir,
                domain_id=base_domain_id + index,
            )
            log_path = case_dir / 'worker_process.log'
            with log_path.open('w', encoding='utf-8') as stream:
                completed = subprocess.run(
                    command,
                    cwd=WORKSPACE,
                    text=True,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            summary_path = case_dir / 'case_summary.json'
            if summary_path.is_file():
                case_result = read_json_object(
                    summary_path, label=f'{scenario.scenario_id} case summary',
                )
                results.append(case_result)
            if completed.returncode != 0:
                status = 'failed'
                failure_reason = (
                    case_result.get('failure_reason', '')
                    if summary_path.is_file() else
                    f'worker exited with {completed.returncode}'
                )
                break
        if status in {'passed', 'partial'}:
            locked_after = verify_source_lock(source_lock)
            write_json_replace(
                output_root / 'source_lock_post_verification.json', locked_after,
            )
    except Exception as exc:
        status = 'failed'
        failure_reason = str(exc)
    summary = {
        'schema_version': CAMPAIGN_SCHEMA,
        'status': status,
        'authorization_scope': AUTHORIZATION_SCOPE,
        'visual_schema_version': V4_SCHEMA,
        'visual_checkpoint_sha256': V4_CHECKPOINT_SHA256,
        'qualification_manifest_sha256': V4_PROMOTION_MANIFEST_SHA256,
        'task_runtime_config_sha256': TASK_RUNTIME_CONFIG_SHA256,
        'physical_deployment': False,
        'started_at_utc': started_at_utc,
        'finished_at_utc': utc_now(),
        'declared_scenario_count': len(FAULT_SCENARIOS),
        'selected_scenario_count': len(selected),
        'completed_scenario_count': len(results),
        'passed_scenario_count': sum(
            item.get('status') == 'passed' for item in results
        ),
        'failed_scenario_count': sum(
            item.get('status') != 'passed' for item in results
        ),
        'all_false_success_counts_zero': bool(results) and all(
            int(item.get('false_success_count') or 0) == 0 for item in results
        ),
        'all_controllers_disabled': bool(results) and all(
            item.get('controller_final_mode') == 'DISABLED' for item in results
        ),
        'v3_observation_count': sum(
            int(item.get('v3_observation_count') or 0) for item in results
        ),
        'v4_observation_count': sum(
            int(item.get('v4_observation_count') or 0) for item in results
        ),
        'failure_reason': failure_reason,
        'results': results,
    }
    finalise_evidence(output_root, summary)
    if status == 'failed':
        raise QualificationError(f'fault campaign failed: {failure_reason}')
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    prepare = subparsers.add_parser(
        'prepare-source-lock',
        help='write one immutable exact-source review lock',
    )
    prepare.add_argument('--output', type=Path, required=True)
    run = subparsers.add_parser(
        'run', help='run the Gazebo-only fail-closed qualification campaign',
    )
    run.add_argument('--source-lock', type=Path, required=True)
    run.add_argument('--output-root', type=Path, required=True)
    run.add_argument('--base-domain-id', type=int, default=181)
    run.add_argument('--scenario', action='append', default=[])
    worker = subparsers.add_parser('_worker', help=argparse.SUPPRESS)
    worker.add_argument('--scenario', required=True)
    worker.add_argument('--case-dir', type=Path, required=True)
    worker.add_argument('--domain-id', type=int, required=True)
    publisher = subparsers.add_parser(
        '_publish_task_goal', help=argparse.SUPPRESS,
    )
    publisher.add_argument('--goal-json', required=True)
    publisher.add_argument('--timeout-s', type=float, required=True)
    estop_publisher = subparsers.add_parser(
        '_publish_estop', help=argparse.SUPPRESS,
    )
    estop_publisher.add_argument('--timeout-s', type=float, required=True)
    estop_publisher.add_argument(
        '--delivery-hold-s', type=float, required=True,
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.command == 'prepare-source-lock':
            payload = prepare_source_lock(args.output)
            print(json.dumps({
                'status': 'prepared',
                'source_lock': str(args.output.expanduser().resolve()),
                'source_count': payload['source_count'],
                'physical_deployment': False,
            }, sort_keys=True))
            return 0
        if args.command == 'run':
            summary = run_campaign(
                source_lock=args.source_lock,
                output_root=args.output_root,
                base_domain_id=args.base_domain_id,
                selected_ids=args.scenario,
            )
            print(json.dumps(summary, sort_keys=True))
            return 0
        if args.command == '_worker':
            scenario = scenario_by_id(args.scenario)
            result = run_worker(
                scenario,
                case_dir=args.case_dir.expanduser().resolve(),
                domain_id=args.domain_id,
            )
            print(json.dumps(result, sort_keys=True))
            return 0
        if args.command == '_publish_task_goal':
            try:
                goal = json.loads(args.goal_json)
            except json.JSONDecodeError as exc:
                raise QualificationError(
                    f'TaskGoal helper input is invalid JSON: {exc}'
                ) from exc
            if not isinstance(goal, dict):
                raise QualificationError('TaskGoal helper input is not an object')
            evidence = publish_task_goal_once_wait_ack(
                goal, timeout_s=args.timeout_s,
            )
            print(json.dumps(evidence, sort_keys=True))
            return 0
        if args.command == '_publish_estop':
            evidence = publish_estop_once_wait_ack(
                timeout_s=args.timeout_s,
                delivery_hold_s=args.delivery_hold_s,
            )
            print(json.dumps(evidence, sort_keys=True))
            return 0
    except QualificationError as exc:
        print(f'qualification error: {exc}', file=sys.stderr)
        return 2
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
