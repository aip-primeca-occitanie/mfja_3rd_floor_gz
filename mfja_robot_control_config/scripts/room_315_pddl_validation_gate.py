#!/usr/bin/env python3
"""Strict validation gate for Room 315 PDDL/PlanSys2 VLA scenarios."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_multi_shuttle import ACTION_SCHEMA_VERSION
from room_315_multi_shuttle import ACTION_VECTOR_V3_FIELDS
from room_315_multi_shuttle import COORDINATION_MODE_IDS
from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import MAX_SHUTTLES_PER_SIDE
from room_315_multi_shuttle import PRIMITIVE_IDS
from room_315_multi_shuttle import REASON_IDS
from room_315_multi_shuttle import SIDE_IDS
from room_315_multi_shuttle import STOPPER_VALUE_IDS
from room_315_multi_shuttle import SWITCH_VALUE_IDS
from room_315_multi_shuttle import TARGET_IDS
from room_315_multi_shuttle import WAIT_CONDITION_IDS
from room_315_multi_shuttle import decode_action_v3


SUPPORTED_SYMBOLIC_ACTIONS = {
    'prepare_switches',
    'open_stoppers',
    'move_shuttle',
    'stop_shuttle',
    'finish_task',
}
MODEL_INPUT_ALLOWED_KEYS = {'language', 'overhead_images', 'last_command'}
FORBIDDEN_MODEL_INPUT_KEYS = {
    'auxiliary_targets',
    'binary_sensor_bits',
    'block_reservations',
    'distance_to_switch',
    'evaluator_label',
    'evaluator_labels',
    'gazebo_pose',
    'identity_tracks',
    'model_input_exposure',
    'normalized_rail_position',
    'observation.state',
    'payload',
    'payload_condition',
    'payload_present',
    'payload_state',
    'payload_type',
    'pddl_domain',
    'pddl_facts',
    'pddl_goal',
    'pddl_problem',
    'plan_step_index',
    'planning_source',
    'plansys_trace',
    'privileged_eval',
    'raw_shuttle_states',
    'shuttle_identity_tracks',
    'status',
    'stopper_states',
    'structured_rail_state',
    'switch_states',
    'true_shuttle_segment',
}
SAFETY_COUNT_FIELDS = (
    'wrong_shuttle_command_count',
    'headway_violation_count',
    'block_occupancy_violation_count',
    'block_reservation_rejection_count',
    'deadlock_detected_count',
)
SAFETY_INFO_FIELDS = (
    'deadlock_avoided_count',
)
TARGET_SENSORS_BY_SIDE_AND_STATION = {
    ('right', 'yaskawa'): ('DZI1R', 'DZI2R'),
    ('right', 'staubli'): ('DZI3R', 'DZI4R'),
    ('left', 'yaskawa'): ('DZI1L', 'DZI2L'),
    ('left', 'kuka'): ('DZI3L', 'DZI4L'),
}


def validate_candidate_scenario(
    scenario: dict[str, Any],
    *,
    multi_shuttle_active: bool | None = None,
) -> dict[str, Any]:
    """Validate the static Generate -> Validate stage before ROS execution."""

    issues: list[str] = []
    symbolic_plan = list(scenario.get('symbolic_plan') or [])
    commands = list(scenario.get('primitive_commands') or [])
    action_vectors = list(scenario.get('action_vectors') or [])
    event_targets = list(scenario.get('expected_event_targets') or [])

    if not symbolic_plan:
        issues.append('PlanSys2 returned an empty symbolic plan')
    for index, step in enumerate(symbolic_plan):
        action_name = _symbolic_action_name(step)
        if action_name not in SUPPORTED_SYMBOLIC_ACTIONS:
            issues.append(f'unknown symbolic PDDL step at index {index}: {step!r}')

    if not commands:
        issues.append('scenario has no translated primitive commands')
    if len(commands) != len(symbolic_plan):
        issues.append(
            'translated primitive command count does not match symbolic plan '
            f'({len(commands)} != {len(symbolic_plan)})'
        )
    if len(action_vectors) != len(commands):
        issues.append(
            'action_vector count does not match primitive command count '
            f'({len(action_vectors)} != {len(commands)})'
        )

    for index, command in enumerate(commands):
        issues.extend(_validate_primitive_command(
            command,
            index=index,
            scenario=scenario,
            multi_shuttle_active=multi_shuttle_active,
        ))

    for index, vector in enumerate(action_vectors):
        expected = event_targets[index] if index < len(event_targets) else {}
        issues.extend(_validate_action_vector(
            vector,
            index=index,
            expected_action=expected if isinstance(expected, dict) else {},
            scenario=scenario,
            multi_shuttle_active=multi_shuttle_active,
        ))

    for path in privileged_model_input_paths(scenario):
        issues.append(f'privileged field inside model_input: {path}')

    return {
        'valid': not issues,
        'failure_reasons': issues,
        'checked_symbolic_steps': len(symbolic_plan),
        'checked_primitive_commands': len(commands),
        'checked_action_vectors': len(action_vectors),
    }


def build_validation_result(
    scenario: dict[str, Any],
    *,
    execution_result: dict[str, Any] | None = None,
    supervisor_metrics: dict[str, Any] | None = None,
    dataset_status: dict[str, Any] | None = None,
    status: dict[str, Any] | None = None,
    recorded_event_count: Any = None,
    multi_shuttle_active: bool | None = None,
) -> dict[str, Any]:
    """Return the persisted validation.json payload for one episode.

    Approval is fail-closed: missing execution, final-goal, task-success, or
    recording signals keep approved_for_training false.
    """

    execution_result = dict(execution_result or {})
    supervisor_metrics = _merge_metrics(supervisor_metrics, execution_result, status)
    dataset_status = dict(dataset_status or {})
    static = validate_candidate_scenario(
        scenario,
        multi_shuttle_active=multi_shuttle_active,
    )
    missing: list[str] = []
    failure_reasons: list[str] = list(static['failure_reasons'])

    task_success = _optional_bool(
        execution_result.get('task_success'),
        execution_result.get('success'),
    )
    final_goal_satisfied = _optional_bool(execution_result.get('final_goal_satisfied'))
    if execution_result == {}:
        missing.append('execution_result')
    if task_success is None:
        missing.append('task_success')
    if final_goal_satisfied is None:
        missing.append('final_goal_satisfied')

    supervisor_rejected_count = _int_metric(
        supervisor_metrics,
        'rejected_actions',
        execution_result.get('supervisor_rejected_count'),
    )
    if supervisor_rejected_count is None:
        missing.append('supervisor_rejected_count')
        supervisor_rejected_count = 0
    rejected_action_rate = _float_metric(
        supervisor_metrics,
        'rejected_action_rate',
        execution_result.get('rejected_action_rate'),
    )
    if rejected_action_rate is None:
        total = _float_metric(supervisor_metrics, 'total_proposed_actions')
        rejected = float(supervisor_rejected_count)
        rejected_action_rate = 0.0 if total == 0.0 else (
            round(rejected / total, 6) if total and total > 0 else None
        )
    if rejected_action_rate is None:
        missing.append('rejected_action_rate')
        rejected_action_rate = 0.0

    arrival_timeout = bool(
        execution_result.get('arrival_timeout')
        or _arrival_wait_failed_by_timeout(execution_result.get('arrival_wait'))
    )
    emergency_stop_triggered = bool(
        execution_result.get('emergency_stop_triggered')
        or _truthy_metric(supervisor_metrics, 'emergency_stop_triggered')
        or _truthy_status_flag(status, ('emergency_stop', 'emergency_stop_triggered'))
    )
    executed_command_count = _optional_int(
        execution_result.get('executed_command_count'),
        len(execution_result.get('published_commands') or []),
    )
    if executed_command_count is None:
        missing.append('executed_command_count')
        executed_command_count = 0
    recorded_event_count_value = _optional_int(
        recorded_event_count,
        execution_result.get('recorded_event_count'),
        dataset_status.get('event_index'),
    )
    if recorded_event_count_value is None:
        missing.append('recorded_event_count')
        recorded_event_count_value = 0

    safety_counts = {
        field: _int_metric(supervisor_metrics, field, execution_result.get(field), default=0)
        for field in (*SAFETY_COUNT_FIELDS, *SAFETY_INFO_FIELDS)
    }
    rejected_reasons = _rejected_reasons(supervisor_metrics, execution_result)

    if not execution_result.get('success', False):
        reason = str(execution_result.get('failure_reason') or 'execution did not succeed')
        failure_reasons.append(reason)
    if task_success is False:
        failure_reasons.append('task_success is false')
    if final_goal_satisfied is False:
        failure_reasons.append('final_goal_satisfied is false')
    if supervisor_rejected_count > 0:
        failure_reasons.append('supervisor rejected at least one command')
    if rejected_action_rate > 0.0:
        failure_reasons.append('rejected_action_rate is nonzero')
    if arrival_timeout:
        failure_reasons.append('arrival timeout')
    if emergency_stop_triggered:
        failure_reasons.append('emergency stop triggered')
    for field in SAFETY_COUNT_FIELDS:
        if int(safety_counts[field]) > 0:
            failure_reasons.append(f'{field} is nonzero')
    for field in missing:
        failure_reasons.append(f'missing required validation signal: {field}')

    approved = (
        static['valid']
        and bool(execution_result.get('success', False))
        and task_success is True
        and final_goal_satisfied is True
        and supervisor_rejected_count == 0
        and rejected_action_rate == 0.0
        and not arrival_timeout
        and not emergency_stop_triggered
        and all(int(safety_counts[field]) == 0 for field in SAFETY_COUNT_FIELDS)
        and executed_command_count > 0
        and recorded_event_count_value > 0
        and not missing
    )

    target = _target_metadata(scenario)
    result = {
        'scenario_id': str(scenario.get('scenario_id') or ''),
        'goal_id': str(scenario.get('goal_id') or scenario.get('scenario_id') or ''),
        'source': 'pddl_plansys',
        'validation_status': 'approved' if approved else 'failed',
        'approved_for_training': bool(approved),
        'failure_reason': '' if approved else _first_failure(failure_reasons),
        'failure_reasons': _unique_strings(failure_reasons),
        'task_success': bool(task_success) if task_success is not None else False,
        'final_goal_satisfied': bool(final_goal_satisfied)
        if final_goal_satisfied is not None
        else False,
        'supervisor_rejected_count': int(supervisor_rejected_count),
        'rejected_action_rate': float(rejected_action_rate),
        'rejected_reasons': rejected_reasons,
        'arrival_timeout': bool(arrival_timeout),
        'emergency_stop_triggered': bool(emergency_stop_triggered),
        'wrong_shuttle_command_count': int(safety_counts['wrong_shuttle_command_count']),
        'headway_violation_count': int(safety_counts['headway_violation_count']),
        'block_occupancy_violation_count': int(safety_counts['block_occupancy_violation_count']),
        'block_reservation_rejection_count': int(
            safety_counts['block_reservation_rejection_count']
        ),
        'deadlock_detected_count': int(safety_counts['deadlock_detected_count']),
        'deadlock_avoided_count': int(safety_counts['deadlock_avoided_count']),
        'target_shuttle_id': target['target_shuttle_id'],
        'final_target_station': target['final_target_station'],
        'final_expected_sensor': target['final_expected_sensor'],
        'executed_command_count': int(executed_command_count),
        'recorded_event_count': int(recorded_event_count_value),
        'static_validation': static,
        'missing_validation_signals': missing,
    }
    return result


def write_validation_result(
    dataset_dir: Path | str,
    episode_id: str,
    validation: dict[str, Any],
) -> Path:
    if not str(episode_id or '').strip():
        raise ValueError('episode_id is required to write validation.json')
    episode_dir = Path(dataset_dir).expanduser() / 'episodes' / str(episode_id)
    episode_dir.mkdir(parents=True, exist_ok=True)
    output = episode_dir / 'validation.json'
    output.write_text(
        json.dumps(validation, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return output


def load_validation_result(episode_dir: Path | str) -> dict[str, Any] | None:
    path = Path(episode_dir) / 'validation.json'
    if not path.exists():
        return None
    try:
        parsed = json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError:
        return {
            'validation_status': 'failed',
            'approved_for_training': False,
            'failure_reason': 'validation.json is not valid JSON',
        }
    return parsed if isinstance(parsed, dict) else None


def validation_approves_training(validation: dict[str, Any] | None) -> bool:
    return (
        isinstance(validation, dict)
        and validation.get('validation_status') == 'approved'
        and validation.get('approved_for_training') is True
    )


def privileged_model_input_paths(value: Any, *, root: str = '$') -> list[str]:
    paths: list[str] = []
    if isinstance(value, dict):
        if _looks_like_model_input(value):
            paths.extend(_model_input_privileged_paths(value, root=root))
        for key, child in value.items():
            key_path = f'{root}.{key}'
            if str(key) == 'model_input' and isinstance(child, dict):
                paths.extend(_model_input_privileged_paths(child, root=key_path))
            paths.extend(privileged_model_input_paths(child, root=key_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            paths.extend(privileged_model_input_paths(child, root=f'{root}[{index}]'))
    return _unique_strings(paths)


def runtime_failure_reason(
    *,
    status: dict[str, Any] | None = None,
    supervisor_metrics: dict[str, Any] | None = None,
) -> str:
    metrics = _merge_metrics(supervisor_metrics, {}, status)
    if _truthy_status_flag(status, ('emergency_stop', 'emergency_stop_triggered')):
        return 'emergency stop triggered'
    if _truthy_metric(metrics, 'emergency_stop_triggered'):
        return 'emergency stop triggered'
    for field in SAFETY_COUNT_FIELDS:
        value = _int_metric(metrics, field, default=0)
        if value and value > 0:
            return f'{field} is nonzero'
    return ''


def _validate_action_vector(
    vector: Any,
    *,
    index: int,
    expected_action: dict[str, Any],
    scenario: dict[str, Any],
    multi_shuttle_active: bool | None,
) -> list[str]:
    issues: list[str] = []
    try:
        values = [float(value) for value in list(vector)]
    except (TypeError, ValueError):
        return [f'action_vector at index {index} is not a numeric sequence']
    if len(values) != len(ACTION_VECTOR_V3_FIELDS):
        return [
            f'action_vector at index {index} length {len(values)} does not match '
            f'schema v{ACTION_SCHEMA_VERSION} length {len(ACTION_VECTOR_V3_FIELDS)}'
        ]

    field_values = dict(zip(ACTION_VECTOR_V3_FIELDS, values))
    issues.extend(_validate_enum_field(index, field_values, 'primitive_id', PRIMITIVE_IDS.values()))
    issues.extend(_validate_enum_field(index, field_values, 'side_id', SIDE_IDS.values()))
    issues.extend(_validate_enum_field(
        index,
        field_values,
        'wait_condition_id',
        WAIT_CONDITION_IDS.values(),
    ))
    issues.extend(_validate_enum_field(index, field_values, 'target_id', TARGET_IDS.values()))
    issues.extend(_validate_enum_field(index, field_values, 'reason_id', REASON_IDS.values()))
    issues.extend(_validate_enum_field(
        index,
        field_values,
        'coordination_mode',
        COORDINATION_MODE_IDS.values(),
    ))
    shuttle_index = _rounded_int(field_values.get('shuttle_index'))
    if shuttle_index is None or shuttle_index < -1 or shuttle_index >= MAX_SHUTTLES_PER_SIDE:
        issues.append(
            f'action_vector at index {index} has invalid shuttle_index '
            f'{field_values.get("shuttle_index")!r}'
        )
    for name in DEVICE_NAMES:
        for prefix in ('switch_mask', 'stopper_mask'):
            mask = field_values[f'{prefix}_{name}']
            if mask not in {0.0, 1.0}:
                issues.append(f'action_vector at index {index} has invalid {prefix}_{name}={mask!r}')
        issues.extend(_validate_enum_field(
            index,
            field_values,
            f'switch_value_{name}',
            SWITCH_VALUE_IDS.values(),
        ))
        issues.extend(_validate_enum_field(
            index,
            field_values,
            f'stopper_value_{name}',
            STOPPER_VALUE_IDS.values(),
        ))
    try:
        decoded = decode_action_v3(values)
    except ValueError as exc:
        issues.append(f'action_vector at index {index} failed schema-v3 decode: {exc}')
        return issues
    primitive = str(decoded.get('primitive') or '')
    side = str(decoded.get('side') or '')
    if expected_action and str(expected_action.get('primitive') or '') != primitive:
        issues.append(
            f'action_vector at index {index} primitive {primitive!r} does not match '
            f'expected action {expected_action.get("primitive")!r}'
        )
    if primitive in {'SHUTTLE_ON', 'STOP_NOW'} and _multi_shuttle_for_side(
        scenario,
        side,
        explicit=multi_shuttle_active,
    ):
        if shuttle_index is None or shuttle_index < 0:
            issues.append(
                f'schema-v3 movement action_vector at index {index} is missing '
                'a valid shuttle_index in multi-shuttle mode'
            )
    return issues


def _validate_primitive_command(
    command: Any,
    *,
    index: int,
    scenario: dict[str, Any],
    multi_shuttle_active: bool | None,
) -> list[str]:
    if not isinstance(command, dict):
        return [f'primitive command at index {index} is not a JSON object']
    issues: list[str] = []
    action = str(command.get('action') or '').strip()
    if action not in {'switches', 'stoppers', 'shuttle', 'DONE', 'stop_all', 'emergency_stop'}:
        issues.append(f'primitive command at index {index} has unknown action {action!r}')
        return issues
    if action in {'switches', 'stoppers', 'shuttle'}:
        side = str(command.get('side') or '').strip().casefold()
        if side not in SIDE_IDS:
            issues.append(f'primitive command at index {index} has invalid side {side!r}')
    if action == 'shuttle':
        command_name = str(command.get('command') or '').strip().upper()
        if command_name not in {'ON', 'OFF'}:
            issues.append(f'shuttle command at index {index} has invalid command {command_name!r}')
        side = str(command.get('side') or '').strip().casefold()
        if command_name in {'ON', 'OFF'} and _multi_shuttle_for_side(
            scenario,
            side,
            explicit=multi_shuttle_active,
        ):
            if not _command_has_explicit_shuttle_identity(command, side=side):
                issues.append(
                    f'shuttle movement command at index {index} does not identify '
                    'the target shuttle in multi-shuttle mode'
                )
    return issues


def _validate_enum_field(
    index: int,
    fields: dict[str, float],
    name: str,
    allowed_values,
) -> list[str]:
    value = _rounded_int(fields.get(name))
    if value not in set(int(item) for item in allowed_values):
        return [f'action_vector at index {index} has invalid {name}={fields.get(name)!r}']
    return []


def _model_input_privileged_paths(model_input: dict[str, Any], *, root: str) -> list[str]:
    paths: list[str] = []
    for key, value in model_input.items():
        normalized = _normalized_key(key)
        if key not in MODEL_INPUT_ALLOWED_KEYS or normalized in FORBIDDEN_MODEL_INPUT_KEYS:
            paths.append(f'{root}.{key}')
        paths.extend(_privileged_paths_by_key(value, root=f'{root}.{key}'))
    return paths


def _privileged_paths_by_key(value: Any, *, root: str) -> list[str]:
    paths: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            normalized = _normalized_key(key)
            child_root = f'{root}.{key}'
            if normalized in FORBIDDEN_MODEL_INPUT_KEYS:
                paths.append(child_root)
            paths.extend(_privileged_paths_by_key(child, root=child_root))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            paths.extend(_privileged_paths_by_key(child, root=f'{root}[{index}]'))
    return paths


def _looks_like_model_input(value: dict[str, Any]) -> bool:
    keys = set(value)
    return bool(keys & MODEL_INPUT_ALLOWED_KEYS) and 'last_command' in keys


def _symbolic_action_name(step: Any) -> str:
    text = str(step or '').strip()
    text = re.sub(r'^\s*\d+(?:\.\d+)?:\s*', '', text)
    text = text.strip().strip('()')
    if not text:
        return ''
    return text.split()[0].strip().casefold()


def _multi_shuttle_for_side(
    scenario: dict[str, Any],
    side: str,
    *,
    explicit: bool | None,
) -> bool:
    if explicit is True:
        return True
    counts = scenario.get('shuttle_counts')
    if isinstance(counts, dict):
        try:
            return int(counts.get(side) or 0) > 1
        except (TypeError, ValueError):
            return False
    key = f'{side}_shuttle_count'
    try:
        return int(scenario.get(key) or 0) > 1
    except (TypeError, ValueError):
        return False


def _command_has_explicit_shuttle_identity(command: dict[str, Any], *, side: str) -> bool:
    for key in ('shuttle_id', 'name'):
        if _is_explicit_shuttle_ref(command.get(key)):
            return True
    if _optional_int(command.get('shuttle_index')) is not None:
        return True
    shuttle = command.get('shuttle')
    text = str(shuttle or '').strip()
    if not text:
        return False
    generic = {f'{side}_shuttle', 'right_shuttle', 'left_shuttle'}
    if text.casefold() in generic:
        return False
    return _is_explicit_shuttle_ref(text)


def _is_explicit_shuttle_ref(value: Any) -> bool:
    text = str(value or '').strip()
    if not text:
        return False
    lowered = text.casefold()
    return bool(
        re.fullmatch(r'[rl][1-4]', lowered)
        or re.fullmatch(r'(?:room315_)?(?:right|left)_shuttle_?[1-4]', lowered)
    )


def _merge_metrics(
    supervisor_metrics: dict[str, Any] | None,
    execution_result: dict[str, Any],
    status: dict[str, Any] | None,
) -> dict[str, Any]:
    merged: dict[str, Any] = {}
    status_metrics = _metrics_from_status(status)
    merged.update(status_metrics)
    if isinstance(supervisor_metrics, dict):
        merged.update(supervisor_metrics)
    for key, value in execution_result.items():
        if key.endswith('_count') or key.endswith('_rate') or key in {
            'rejected_actions',
            'total_proposed_actions',
            'emergency_stop_triggered',
        }:
            merged.setdefault(key, value)
    return merged


def _metrics_from_status(status: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(status, dict):
        return {}
    safety = status.get('safety_decoder')
    if isinstance(safety, dict) and isinstance(safety.get('metrics'), dict):
        return dict(safety['metrics'])
    metrics = status.get('safety_decoder_metrics')
    return dict(metrics) if isinstance(metrics, dict) else {}


def _target_metadata(scenario: dict[str, Any]) -> dict[str, str]:
    side = ''
    target_station = ''
    target_shuttle_id = ''
    for target in scenario.get('expected_event_targets') or []:
        if not isinstance(target, dict):
            continue
        if str(target.get('primitive') or '') == 'SHUTTLE_ON':
            side = str(target.get('side') or side)
            target_shuttle_id = str(target.get('shuttle_id') or target_shuttle_id)
            target_id = str(target.get('target_id') or '')
            if not target_shuttle_id and _is_explicit_shuttle_ref(target_id):
                target_shuttle_id = target_id
    goal_text = str(scenario.get('pddl_goal') or '')
    match = re.search(r'\bat\s+([A-Za-z0-9_-]+)\b', goal_text, re.IGNORECASE)
    if match:
        target_station = _station_symbol(match.group(1))
    if not side:
        side = _side_from_goal_or_scenario(scenario)
    sensors = TARGET_SENSORS_BY_SIDE_AND_STATION.get((side, target_station), ())
    return {
        'target_shuttle_id': target_shuttle_id,
        'final_target_station': target_station,
        'final_expected_sensor': ','.join(sensors),
    }


def _side_from_goal_or_scenario(scenario: dict[str, Any]) -> str:
    for key in ('side', 'scenario_id', 'goal_id', 'pddl_goal'):
        text = str(scenario.get(key) or '').casefold()
        if 'right' in text:
            return 'right'
        if 'left' in text:
            return 'left'
    return ''


def _station_symbol(value: str) -> str:
    text = str(value or '').strip().casefold()
    if text.startswith('right_') or text.startswith('left_'):
        text = text.split('_', 1)[1]
    return text if text in {'yaskawa', 'staubli', 'kuka'} else ''


def _rejected_reasons(
    supervisor_metrics: dict[str, Any],
    execution_result: dict[str, Any],
) -> dict[str, Any]:
    for source in (execution_result, supervisor_metrics):
        for key in ('rejected_reasons', 'rejection_reasons'):
            value = source.get(key)
            if isinstance(value, dict):
                return dict(value)
            if isinstance(value, list):
                return {str(item): value.count(item) for item in sorted(set(value))}
    decision = execution_result.get('supervisor_decision')
    if isinstance(decision, dict) and decision.get('reason'):
        return {str(decision['reason']): 1}
    return {}


def _arrival_wait_failed_by_timeout(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    if value.get('arrived') is True:
        return False
    return 'timeout' in str(value.get('reason') or '').casefold()


def _truthy_status_flag(status: dict[str, Any] | None, keys: tuple[str, ...]) -> bool:
    if not isinstance(status, dict):
        return False
    for key in keys:
        value = status.get(key)
        if isinstance(value, bool) and value:
            return True
    emergency = status.get('emergency')
    if isinstance(emergency, dict):
        return any(bool(emergency.get(key)) for key in keys)
    return False


def _truthy_metric(metrics: dict[str, Any], key: str) -> bool:
    value = metrics.get(key)
    if isinstance(value, bool):
        return value
    parsed = _optional_int(value)
    return bool(parsed and parsed > 0)


def _first_failure(reasons: list[str]) -> str:
    unique = _unique_strings(reasons)
    return unique[0] if unique else 'validation did not approve scenario'


def _unique_strings(values: list[Any]) -> list[str]:
    seen: set[str] = set()
    output: list[str] = []
    for value in values:
        text = str(value or '').strip()
        if not text or text in seen:
            continue
        seen.add(text)
        output.append(text)
    return output


def _int_metric(metrics: dict[str, Any], key: str, *fallbacks: Any, default: Any = None) -> int | None:
    return _optional_int(metrics.get(key), *fallbacks, default=default)


def _float_metric(metrics: dict[str, Any], key: str, *fallbacks: Any) -> float | None:
    for value in (metrics.get(key), *fallbacks):
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return None


def _optional_bool(*values: Any) -> bool | None:
    for value in values:
        if isinstance(value, bool):
            return value
    return None


def _optional_int(*values: Any, default: Any = None) -> int | None:
    for value in values:
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    try:
        return int(default)
    except (TypeError, ValueError):
        return None


def _rounded_int(value: Any) -> int | None:
    try:
        return int(round(float(value)))
    except (TypeError, ValueError):
        return None


def _normalized_key(key: Any) -> str:
    return str(key or '').strip().casefold().replace('-', '_')
