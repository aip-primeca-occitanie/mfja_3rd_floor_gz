#!/usr/bin/env python3

import json
import os
import time
from datetime import datetime
from datetime import timezone
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


SIDES = ('right', 'left')
DEVICE_NAMES = ('A1', 'A2', 'A3', 'A4')
SENSOR_IDS_BY_SIDE = {
    'right': (
        'DZI1R',
        'DZI2R',
        'DZI3R',
        'DZI4R',
        'DA1R',
        'DA1ER',
        'DA1IR',
        'DA2R',
        'DA2ER',
        'DA2IR',
        'DA3R',
        'DA3ER',
        'DA3IR',
        'DA4R',
        'DA4ER',
        'DA4IR',
        'A1_STOPPER_SENSOR',
        'A2_STOPPER_SENSOR',
        'A3_STOPPER_SENSOR',
        'A4_STOPPER_SENSOR',
    ),
    'left': (
        'DZI1L',
        'DZI2L',
        'DZI3L',
        'DZI4L',
        'DA1L',
        'DA1EL',
        'DA1IL',
        'DA2L',
        'DA2EL',
        'DA2IL',
        'DA3L',
        'DA3EL',
        'DA3IL',
        'DA4L',
        'DA4EL',
        'DA4IL',
        'A1_STOPPER_SENSOR',
        'A2_STOPPER_SENSOR',
        'A3_STOPPER_SENSOR',
        'A4_STOPPER_SENSOR',
    ),
}


COMMAND_ACTIONS = {
    'status',
    'route_template',
    'route_template_phase',
    'route_shuttle',
    'switches',
    'stoppers',
    'shuttle',
    'add_shuttle',
    'stop_all',
    'emergency_stop',
    'clear_emergency_stop',
}

PRIMITIVE_IDS = {
    'WAIT': 0,
    'DONE': 1,
    'SET_SWITCHES': 2,
    'SET_STOPPERS': 3,
    'SHUTTLE_ON_FAST': 4,
    'SHUTTLE_ON_SLOW': 5,
    'STOP_NOW': 6,
    'EMERGENCY_STOP': 7,
}

SIDE_IDS = {'right': 0, 'left': 1}
SWITCH_VALUE_IDS = {'UNCHANGED': 0, 'EXTERIOR': 1, 'INTERIOR': 2}
STOPPER_VALUE_IDS = {'UNCHANGED': 0, 'open': 1, 'closed': 2}
WAIT_CONDITION_IDS = {
    'none': 0,
    'switch_state_match': 1,
    'stopper_state_match': 2,
    'shuttle_command_applied': 3,
    'task_terminal_status': 4,
    'task_phase_observed': 5,
    'terminal': 6,
    'target_sensor_active': 7,
}
TARGET_IDS = {
    'none': 0,
    'A1': 1,
    'A2': 2,
    'A3': 3,
    'A4': 4,
    'ALL_SWITCHES': 5,
    'ALL_STOPPERS': 6,
    'MULTIPLE_DEVICES': 7,
    'right_shuttle': 8,
    'left_shuttle': 9,
    'right_yaskawa_to_staubli': 10,
    'right_staubli_to_yaskawa': 11,
    'left_yaskawa_to_kuka': 12,
    'left_kuka_to_yaskawa': 13,
    'right_enter_interior_loop': 14,
    'left_enter_interior_loop': 15,
    'task_phase': 16,
    'terminal': 17,
    'DZI1R': 18,
    'DZI2R': 19,
    'DZI3R': 20,
    'DZI4R': 21,
    'DZI1L': 22,
    'DZI2L': 23,
    'DZI3L': 24,
    'DZI4L': 25,
    'DA3IR': 26,
    'DA3IL': 27,
}
REASON_IDS = {
    'none': 0,
    'command_event': 1,
    'route_template_requested': 2,
    'task_phase': 3,
    'task_succeeded': 4,
    'task_failed': 5,
    'episode_stopped': 6,
    'episode_discarded': 7,
    'switch_update': 8,
    'stopper_update': 9,
    'shuttle_start': 10,
    'shuttle_stop': 11,
    'emergency': 12,
    'unsupported_command': 13,
}
FAST_SPEED_THRESHOLD_MPS = 0.3
PRIMITIVE_IDS_BY_VALUE = {value: key for key, value in PRIMITIVE_IDS.items()}
SIDE_IDS_BY_VALUE = {value: key for key, value in SIDE_IDS.items()}
SWITCH_VALUE_IDS_BY_VALUE = {value: key for key, value in SWITCH_VALUE_IDS.items()}
STOPPER_VALUE_IDS_BY_VALUE = {value: key for key, value in STOPPER_VALUE_IDS.items()}
WAIT_CONDITION_IDS_BY_VALUE = {value: key for key, value in WAIT_CONDITION_IDS.items()}
TARGET_IDS_BY_VALUE = {value: key for key, value in TARGET_IDS.items()}
REASON_IDS_BY_VALUE = {value: key for key, value in REASON_IDS.items()}

ACTION_VECTOR_FIELDS = [
    'primitive_id',
    'side_id',
    *[f'switch_mask_{name}' for name in DEVICE_NAMES],
    *[f'switch_value_{name}' for name in DEVICE_NAMES],
    *[f'stopper_mask_{name}' for name in DEVICE_NAMES],
    *[f'stopper_value_{name}' for name in DEVICE_NAMES],
    'wait_condition_id',
    'target_id',
    'reason_id',
]
EVENT_ACTION_FIELDS = [
    'primitive',
    'side',
    'switch_mask',
    'switch_values',
    'stopper_mask',
    'stopper_values',
    'wait_condition',
    'target_id',
    'reason',
]

OBSERVATION_STATE_FIELDS = [
    'emergency_stop',
    'vision_image_frames',
    *[
        f'{side}_sensor_{sensor_name}'
        for side in SIDES
        for sensor_name in SENSOR_IDS_BY_SIDE[side]
    ],
    *[f'right_switch_{name}' for name in DEVICE_NAMES],
    *[f'left_switch_{name}' for name in DEVICE_NAMES],
    *[f'right_stopper_{name}' for name in DEVICE_NAMES],
    *[f'left_stopper_{name}' for name in DEVICE_NAMES],
]

DEBUG_OBSERVATION_FIELDS = [
    f'{side}_{field}'
    for side in SIDES
    for field in (
        'shuttle_count',
        'active_sensors_count',
        'active_position_sensors_count',
    )
]
MODEL_INPUT_SCHEMA_VERSION = 2
MODEL_INPUT_FIELDS = [
    'language',
    'overhead_images',
    'binary_sensor_bits',
    'switch_states',
    'stopper_states',
    'last_command',
    'shuttle_command_state',
    'time_since_last_sensor_event',
]
PRIVILEGED_EVAL_FIELDS = [
    'supervisor_status',
    'raw_shuttle_states',
    'raw_active_sensor_readings',
    'visual_eval_labels',
]
OVERHEAD_IMAGE_NAMES = {'right_rail_rgb', 'left_rail_rgb'}
STATION_SENSOR_GROUPS = {
    'right': {
        'yaskawa_hc10dt': ['DZI1R', 'DZI2R'],
        'staubli_tx2': ['DZI3R', 'DZI4R'],
    },
    'left': {
        'yaskawa_hc10': ['DZI1L', 'DZI2L'],
        'kuka_kr6': ['DZI3L', 'DZI4L'],
    },
}
VISUAL_EVAL_MARKERS = {
    'colored_station_markers': [
        {
            'id': 'right_yaskawa_station_marker',
            'side': 'right',
            'station': 'yaskawa_hc10dt',
            'color': 'blue',
        },
        {
            'id': 'right_staubli_station_marker',
            'side': 'right',
            'station': 'staubli_tx2',
            'color': 'amber',
        },
        {
            'id': 'left_yaskawa_station_marker',
            'side': 'left',
            'station': 'yaskawa_hc10',
            'color': 'blue',
        },
        {
            'id': 'left_kuka_station_marker',
            'side': 'left',
            'station': 'kuka_kr6',
            'color': 'orange',
        },
    ],
    'inspection_markers': [
        {'id': 'right_green_inspection_marker', 'side': 'right', 'color': 'green'},
        {'id': 'left_green_inspection_marker', 'side': 'left', 'color': 'green'},
    ],
    'station_status_markers': [
        {'id': 'right_station_empty_marker', 'side': 'right', 'meaning': 'empty'},
        {'id': 'right_station_occupied_marker', 'side': 'right', 'meaning': 'occupied'},
        {'id': 'left_station_empty_marker', 'side': 'left', 'meaning': 'empty'},
        {'id': 'left_station_occupied_marker', 'side': 'left', 'meaning': 'occupied'},
    ],
    'removable_obstacle_marker': {
        'entity': 'room315_vla_removable_obstacle_marker',
        'visual_ids': [
            'right_removable_obstacle_marker',
            'left_removable_obstacle_marker',
        ],
        'default_present': True,
        'visual_only': True,
    },
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _safe_filename(text: str, fallback: str) -> str:
    cleaned = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in text.strip())
    cleaned = '_'.join(part for part in cleaned.split('_') if part)
    return cleaned[:80] or fallback


def _parse_json_or_text(raw: str) -> Any:
    text = raw.strip()
    if not text:
        return {}
    if text[0] in '{[':
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {'text_command': text}
    return {'text_command': text}


def _as_command_dict(command: Any) -> dict[str, Any]:
    if isinstance(command, list) and command and isinstance(command[0], dict):
        return command[0]
    if isinstance(command, dict):
        return command
    return {'action': 'status'}


def _normalize_action_name(command: dict[str, Any]) -> str:
    action = str(command.get('action') or command.get('intent') or command.get('type') or 'status')
    action = action.lower().strip()
    if action in {'route', 'start_route'}:
        return 'route_shuttle'
    if action in {'template', 'task'}:
        return 'route_template'
    if action in {'route_template_phase', 'task_phase'}:
        return 'route_template_phase'
    if action in {'switch'}:
        return 'switches'
    if action in {'stopper'}:
        return 'stoppers'
    if action in {'shuttle_command'}:
        return 'shuttle'
    if action in {'spawn_shuttle'}:
        return 'add_shuttle'
    if action in {'all_off'}:
        return 'stop_all'
    if action in {'estop'}:
        return 'emergency_stop'
    if action in {'clear_estop', 'reset_estop'}:
        return 'clear_emergency_stop'
    return action if action in COMMAND_ACTIONS else 'status'


def _normalized_name_map(raw: Any) -> dict[str, str]:
    if not isinstance(raw, dict):
        return {}
    return {
        str(name).strip().upper(): str(state).strip().upper()
        for name, state in raw.items()
        if str(name).strip()
    }


def _normalize_symbolic_action(command: Any) -> dict[str, Any]:
    command_dict = _as_command_dict(command)
    action = _normalize_action_name(command_dict)
    side = str(command_dict.get('side') or '').strip().lower()

    if str(command_dict.get('action') or '').strip().upper() == 'DONE':
        return {
            'action': 'DONE',
            'status': str(command_dict.get('status') or '').strip(),
            'template': str(command_dict.get('template') or '').strip(),
            'task_id': str(command_dict.get('task_id') or '').strip(),
            'failure_reason': str(command_dict.get('failure_reason') or '').strip(),
        }

    if action == 'route_template':
        return {
            'action': 'route_template',
            'template': str(
                command_dict.get('template')
                or command_dict.get('template_id')
                or command_dict.get('name')
                or ''
            ).strip(),
        }

    if action == 'route_template_phase':
        return {
            'action': 'route_template_phase',
            'template': str(command_dict.get('template') or '').strip(),
            'task_id': str(command_dict.get('task_id') or '').strip(),
            'phase': str(command_dict.get('phase') or '').strip(),
            'status': str(command_dict.get('status') or '').strip(),
        }

    if action == 'route_shuttle':
        normalized = {
            'action': 'route_shuttle',
            'side': side or 'right',
            'loop': str(command_dict.get('loop') or '').strip().lower(),
            'start': bool(command_dict.get('start', command_dict.get('start_after_prepare', True))),
        }
        if command_dict.get('start_slot') is not None:
            normalized['start_slot'] = str(command_dict.get('start_slot')).strip()
        if command_dict.get('speed') is not None:
            normalized['speed'] = float(command_dict.get('speed') or 0.0)
        return normalized

    if action == 'switches':
        switches = command_dict.get('switches')
        if not isinstance(switches, dict) and 'name' in command_dict and 'state' in command_dict:
            switches = {str(command_dict['name']): command_dict['state']}
        return {
            'action': 'switches',
            'side': side or 'right',
            'switches': _normalized_name_map(switches),
        }

    if action == 'stoppers':
        stoppers = command_dict.get('stoppers')
        if not isinstance(stoppers, dict) and 'name' in command_dict and 'state' in command_dict:
            stoppers = {str(command_dict['name']): command_dict['state']}
        return {
            'action': 'stoppers',
            'side': side or 'right',
            'stoppers': _normalized_name_map(stoppers),
        }

    if action == 'shuttle':
        normalized = {
            'action': 'shuttle',
            'side': side or 'right',
            'command': str(command_dict.get('command') or '').strip().upper(),
        }
        if command_dict.get('shuttle') or command_dict.get('name'):
            normalized['shuttle'] = str(command_dict.get('shuttle') or command_dict.get('name')).strip()
        if command_dict.get('start_slot') is not None:
            normalized['start_slot'] = str(command_dict.get('start_slot')).strip()
        if command_dict.get('speed') is not None:
            normalized['speed'] = float(command_dict.get('speed') or 0.0)
        return normalized

    return {'action': action}


def _is_meaningful_event_action(next_action: dict[str, Any]) -> bool:
    action = str(next_action.get('action') or '')
    if action in {'switches', 'stoppers', 'route_shuttle', 'route_template'}:
        return True
    if action == 'route_template_phase':
        return bool(next_action.get('phase'))
    if action == 'DONE':
        return True
    if action == 'shuttle':
        return str(next_action.get('command') or '').upper() in {'ON', 'OFF'}
    return False


def _action_vector_or_none(next_action: dict[str, Any]) -> list[float] | None:
    return _encode_action(next_action)


def _round_index(raw: Any) -> int:
    try:
        return int(round(float(raw)))
    except (TypeError, ValueError):
        return 0


def _side_from_id(raw: Any) -> str:
    return SIDE_IDS_BY_VALUE.get(_round_index(raw), 'right')


def _assignment_dict(command: dict[str, Any], key: str) -> dict[str, Any]:
    assignments = command.get(key)
    if isinstance(assignments, dict):
        return dict(assignments)
    if 'name' in command and 'state' in command:
        return {str(command['name']): command['state']}
    return {}


def _switch_assignment_dict(command: dict[str, Any], action: str) -> dict[str, Any]:
    assignments = _assignment_dict(command, 'switches')
    if assignments:
        return assignments
    if action != 'switches':
        return {}
    loop = str(command.get('loop') or '').strip().lower()
    if loop == 'exterior':
        return {'ALL': 'EXTERIOR'}
    if loop == 'interior':
        return {'ALL': 'INTERIOR'}
    return {}


def _ordered_device_dict(default: Any) -> dict[str, Any]:
    return {name: default for name in DEVICE_NAMES}


def _device_mask_values(
    assignments: dict[str, Any],
    *,
    value_kind: str,
) -> tuple[list[float], list[float]]:
    mask = [0.0 for _name in DEVICE_NAMES]
    values = [0.0 for _name in DEVICE_NAMES]

    def value_id(raw_value: Any) -> int:
        if value_kind == 'switch':
            return SWITCH_VALUE_IDS.get(_normalize_switch_state(raw_value), 0)
        if value_kind == 'stopper':
            return STOPPER_VALUE_IDS.get(_normalize_stopper_state(raw_value), 0)
        return 0

    normalized = {
        str(name).strip().upper(): state
        for name, state in assignments.items()
        if str(name).strip()
    }
    all_value = normalized.get('ALL')
    if all_value is not None:
        encoded = value_id(all_value)
        if encoded:
            mask = [1.0 for _name in DEVICE_NAMES]
            values = [float(encoded) for _name in DEVICE_NAMES]

    for index, name in enumerate(DEVICE_NAMES):
        if name not in normalized:
            continue
        encoded = value_id(normalized[name])
        if encoded:
            mask[index] = 1.0
            values[index] = float(encoded)
        else:
            mask[index] = 0.0
            values[index] = 0.0
    return mask, values


def _device_mask_value_dicts(
    assignments: dict[str, Any],
    *,
    value_kind: str,
) -> tuple[dict[str, int], dict[str, str]]:
    mask_values, encoded_values = _device_mask_values(assignments, value_kind=value_kind)
    value_lookup = SWITCH_VALUE_IDS_BY_VALUE if value_kind == 'switch' else STOPPER_VALUE_IDS_BY_VALUE
    mask = {
        name: int(mask_values[index])
        for index, name in enumerate(DEVICE_NAMES)
    }
    values = {
        name: value_lookup.get(int(encoded_values[index]), 'UNCHANGED')
        for index, name in enumerate(DEVICE_NAMES)
    }
    for name in DEVICE_NAMES:
        if not mask[name]:
            values[name] = 'UNCHANGED'
    return mask, values


def _infer_side(command: dict[str, Any], task_context: dict[str, Any] | None = None) -> str:
    side = str(command.get('side') or '').strip().lower()
    if side in SIDE_IDS:
        return side
    template = str(
        command.get('template')
        or command.get('template_id')
        or (task_context or {}).get('template')
        or ''
    ).strip().lower()
    if template.startswith('left_'):
        return 'left'
    if template.startswith('right_'):
        return 'right'
    return 'right'


def _normalize_wait_condition(raw: Any) -> str:
    if isinstance(raw, dict):
        raw = raw.get('type')
    condition = str(raw or 'none').strip()
    return condition if condition in WAIT_CONDITION_IDS else 'none'


def _normalize_target_id(raw: Any) -> str:
    target = str(raw or 'none').strip()
    if target in TARGET_IDS:
        return target
    upper = target.upper()
    if upper in TARGET_IDS:
        return upper
    lower = target.lower()
    if lower in TARGET_IDS:
        return lower
    return 'none'


def _normalize_reason(raw: Any) -> str:
    reason = str(raw or 'none').strip().lower()
    return reason if reason in REASON_IDS else 'none'


def _target_from_assignments(assignments: dict[str, Any], *, device_kind: str) -> str:
    if not assignments:
        return 'none'
    normalized_names = {
        str(name).strip().upper()
        for name in assignments
        if str(name).strip()
    }
    if 'ALL' in normalized_names:
        return 'ALL_SWITCHES' if device_kind == 'switch' else 'ALL_STOPPERS'
    selected = sorted(name for name in normalized_names if name in DEVICE_NAMES)
    if len(selected) == 1:
        return selected[0]
    if len(selected) > 1:
        return 'MULTIPLE_DEVICES'
    return 'none'


def _target_from_wait_condition(wait_condition: dict[str, Any]) -> str:
    target = wait_condition.get('target') if isinstance(wait_condition, dict) else None
    if isinstance(target, dict):
        if wait_condition.get('type') == 'switch_state_match':
            return _target_from_assignments(target, device_kind='switch')
        if wait_condition.get('type') == 'stopper_state_match':
            return _target_from_assignments(target, device_kind='stopper')
    if isinstance(target, str):
        return _normalize_target_id(target)
    phase = wait_condition.get('phase') if isinstance(wait_condition, dict) else ''
    if phase:
        return 'task_phase'
    return 'none'


def _reason_for_action(
    command: dict[str, Any],
    *,
    event_type: str = '',
    status_text: str = '',
) -> str:
    action = str(command.get('action') or '').strip()
    if action == 'DONE':
        status = str(command.get('status') or status_text).strip().lower()
        if status in {'succeeded', 'success'}:
            return 'task_succeeded'
        if status in {'failed', 'failure'}:
            return 'task_failed'
        if status == 'discarded':
            return 'episode_discarded'
        return 'episode_stopped'
    if action == 'route_template_phase' or event_type == 'task_phase':
        return 'task_phase'
    if action == 'route_template':
        return 'route_template_requested'
    if action == 'switches':
        return 'switch_update'
    if action == 'stoppers':
        return 'stopper_update'
    if action == 'shuttle':
        command_name = str(command.get('command') or '').strip().upper()
        return 'shuttle_stop' if command_name == 'OFF' else 'shuttle_start'
    if action in {'stop_all'}:
        return 'shuttle_stop'
    if action == 'emergency_stop':
        return 'emergency'
    if event_type:
        return 'command_event'
    return 'none'


def _event_action_v2_from_symbolic_action(
    command: Any,
    *,
    wait_condition: dict[str, Any] | None = None,
    task_context: dict[str, Any] | None = None,
    event_type: str = '',
    status_text: str = '',
) -> dict[str, Any]:
    command_dict = _as_command_dict(command)
    action = _normalize_action_name(command_dict)
    if str(command_dict.get('action') or '').strip().upper() == 'DONE':
        action = 'DONE'

    side = _infer_side(command_dict, task_context)
    switch_assignments = _switch_assignment_dict(command_dict, action)
    stopper_assignments = _assignment_dict(command_dict, 'stoppers')
    switch_mask, switch_values = _device_mask_value_dicts(
        switch_assignments,
        value_kind='switch',
    )
    stopper_mask, stopper_values = _device_mask_value_dicts(
        stopper_assignments,
        value_kind='stopper',
    )

    primitive = 'WAIT'
    if action == 'switches':
        primitive = 'SET_SWITCHES'
    elif action == 'stoppers':
        primitive = 'SET_STOPPERS'
    elif action == 'shuttle':
        shuttle_command = str(command_dict.get('command') or '').strip().upper()
        if shuttle_command == 'OFF':
            primitive = 'STOP_NOW'
        elif shuttle_command == 'ON':
            try:
                speed = float(command_dict.get('speed', 0.0) or 0.0)
            except (TypeError, ValueError):
                speed = 0.0
            primitive = 'SHUTTLE_ON_FAST' if speed >= FAST_SPEED_THRESHOLD_MPS else 'SHUTTLE_ON_SLOW'
    elif action == 'route_shuttle':
        start = command_dict.get('start', command_dict.get('start_after_prepare', True))
        if _as_bool(start):
            try:
                speed = float(command_dict.get('speed', 0.0) or 0.0)
            except (TypeError, ValueError):
                speed = 0.0
            primitive = 'SHUTTLE_ON_FAST' if speed >= FAST_SPEED_THRESHOLD_MPS else 'SHUTTLE_ON_SLOW'
        else:
            primitive = 'STOP_NOW'
    elif action == 'stop_all':
        primitive = 'STOP_NOW'
    elif action == 'emergency_stop':
        primitive = 'EMERGENCY_STOP'
    elif action == 'DONE':
        primitive = 'DONE'

    wait_condition_type = _normalize_wait_condition(wait_condition or {})
    if primitive == 'WAIT' and not wait_condition_type:
        wait_condition_type = 'none'

    target_id = _target_from_wait_condition(wait_condition or {})
    if target_id == 'none':
        if primitive == 'SET_SWITCHES':
            target_id = _target_from_assignments(switch_assignments, device_kind='switch')
        elif primitive == 'SET_STOPPERS':
            target_id = _target_from_assignments(stopper_assignments, device_kind='stopper')
        elif primitive in {'SHUTTLE_ON_FAST', 'SHUTTLE_ON_SLOW', 'STOP_NOW'}:
            target_id = f'{side}_shuttle'
        elif primitive == 'DONE':
            target_id = _normalize_target_id(command_dict.get('template')) or 'terminal'
        elif action == 'route_template':
            target_id = _normalize_target_id(command_dict.get('template'))
        elif action == 'route_template_phase':
            target_id = 'task_phase'
    if target_id == 'none' and primitive == 'DONE':
        target_id = 'terminal'

    return {
        'primitive': primitive,
        'side': side,
        'switch_mask': switch_mask,
        'switch_values': switch_values,
        'stopper_mask': stopper_mask,
        'stopper_values': stopper_values,
        'wait_condition': wait_condition_type,
        'target_id': _normalize_target_id(target_id),
        'reason': _reason_for_action(
            {**command_dict, 'action': action},
            event_type=event_type,
            status_text=status_text,
        ),
    }


def _mask_value_dicts_from_event_action(
    action: dict[str, Any],
    *,
    mask_key: str,
    value_key: str,
    value_kind: str,
) -> tuple[list[float], list[float]]:
    raw_mask = action.get(mask_key, {})
    raw_values = action.get(value_key, {})
    if not isinstance(raw_mask, dict):
        raw_mask = {}
    if not isinstance(raw_values, dict):
        raw_values = {}
    masks: list[float] = []
    values: list[float] = []
    for name in DEVICE_NAMES:
        selected = _as_bool(raw_mask.get(name, False))
        masks.append(1.0 if selected else 0.0)
        if not selected:
            values.append(0.0)
            continue
        raw_value = raw_values.get(name, 'UNCHANGED')
        if value_kind == 'switch':
            value_id = SWITCH_VALUE_IDS.get(_normalize_switch_state(raw_value), 0)
        else:
            value_id = STOPPER_VALUE_IDS.get(_normalize_stopper_state(raw_value), 0)
        if value_id == 0:
            raise ValueError(f'{mask_key}_{name} selected but value is UNCHANGED')
        values.append(float(value_id))
    return masks, values


def _normalize_event_action_v2(action: Any) -> dict[str, Any]:
    action_dict = _as_command_dict(action)
    if 'primitive' not in action_dict:
        return _event_action_v2_from_symbolic_action(action_dict)
    primitive = str(action_dict.get('primitive') or 'WAIT').strip().upper()
    if primitive not in PRIMITIVE_IDS:
        primitive = 'WAIT'
    side = str(action_dict.get('side') or 'right').strip().lower()
    if side not in SIDE_IDS:
        side = 'right'
    switch_mask, switch_values = _mask_value_dicts_from_event_action(
        action_dict,
        mask_key='switch_mask',
        value_key='switch_values',
        value_kind='switch',
    )
    stopper_mask, stopper_values = _mask_value_dicts_from_event_action(
        action_dict,
        mask_key='stopper_mask',
        value_key='stopper_values',
        value_kind='stopper',
    )
    switch_mask_dict = {
        name: int(switch_mask[index])
        for index, name in enumerate(DEVICE_NAMES)
    }
    switch_value_dict = {
        name: SWITCH_VALUE_IDS_BY_VALUE.get(int(switch_values[index]), 'UNCHANGED')
        for index, name in enumerate(DEVICE_NAMES)
    }
    stopper_mask_dict = {
        name: int(stopper_mask[index])
        for index, name in enumerate(DEVICE_NAMES)
    }
    stopper_value_dict = {
        name: STOPPER_VALUE_IDS_BY_VALUE.get(int(stopper_values[index]), 'UNCHANGED')
        for index, name in enumerate(DEVICE_NAMES)
    }
    return {
        'primitive': primitive,
        'side': side,
        'switch_mask': switch_mask_dict,
        'switch_values': switch_value_dict,
        'stopper_mask': stopper_mask_dict,
        'stopper_values': stopper_value_dict,
        'wait_condition': _normalize_wait_condition(action_dict.get('wait_condition')),
        'target_id': _normalize_target_id(action_dict.get('target_id')),
        'reason': _normalize_reason(action_dict.get('reason')),
    }


def _encode_action(command: Any) -> list[float]:
    action = _normalize_event_action_v2(command)
    switch_mask, switch_values = _mask_value_dicts_from_event_action(
        action,
        mask_key='switch_mask',
        value_key='switch_values',
        value_kind='switch',
    )
    stopper_mask, stopper_values = _mask_value_dicts_from_event_action(
        action,
        mask_key='stopper_mask',
        value_key='stopper_values',
        value_kind='stopper',
    )
    return [
        float(PRIMITIVE_IDS[action['primitive']]),
        float(SIDE_IDS[action['side']]),
        *switch_mask,
        *switch_values,
        *stopper_mask,
        *stopper_values,
        float(WAIT_CONDITION_IDS[action['wait_condition']]),
        float(TARGET_IDS[action['target_id']]),
        float(REASON_IDS[action['reason']]),
    ]


def _decode_device_assignments(
    action_vector: list[float],
    mask_prefix: str,
    value_prefix: str,
    value_lookup: dict[int, str],
) -> dict[str, str]:
    assignments: dict[str, str] = {}
    for name in DEVICE_NAMES:
        mask_index = ACTION_VECTOR_FIELDS.index(f'{mask_prefix}_{name}')
        value_index = ACTION_VECTOR_FIELDS.index(f'{value_prefix}_{name}')
        selected = float(action_vector[mask_index]) >= 0.5
        if not selected:
            continue
        value_id = _round_index(action_vector[value_index])
        value = value_lookup.get(value_id, 'UNCHANGED')
        if value == 'UNCHANGED':
            raise ValueError(f'{mask_prefix}_{name} selected but value is UNCHANGED')
        assignments[name] = value
    return assignments


def _decode_action(action_vector: Any) -> dict[str, Any]:
    values = [float(value) for value in list(action_vector)]
    if len(values) != len(ACTION_VECTOR_FIELDS):
        raise ValueError(
            f'action vector length {len(values)} does not match schema '
            f'length {len(ACTION_VECTOR_FIELDS)}'
        )

    def field(name: str) -> float:
        return values[ACTION_VECTOR_FIELDS.index(name)]

    switch_assignments = _decode_device_assignments(
        values,
        'switch_mask',
        'switch_value',
        SWITCH_VALUE_IDS_BY_VALUE,
    )
    stopper_assignments = _decode_device_assignments(
        values,
        'stopper_mask',
        'stopper_value',
        STOPPER_VALUE_IDS_BY_VALUE,
    )
    switch_mask = _ordered_device_dict(0)
    switch_values = _ordered_device_dict('UNCHANGED')
    stopper_mask = _ordered_device_dict(0)
    stopper_values = _ordered_device_dict('UNCHANGED')
    for name, value in switch_assignments.items():
        switch_mask[name] = 1
        switch_values[name] = value
    for name, value in stopper_assignments.items():
        stopper_mask[name] = 1
        stopper_values[name] = value

    return {
        'primitive': PRIMITIVE_IDS_BY_VALUE.get(_round_index(field('primitive_id')), 'WAIT'),
        'side': _side_from_id(field('side_id')),
        'switch_mask': switch_mask,
        'switch_values': switch_values,
        'stopper_mask': stopper_mask,
        'stopper_values': stopper_values,
        'wait_condition': WAIT_CONDITION_IDS_BY_VALUE.get(
            _round_index(field('wait_condition_id')),
            'none',
        ),
        'target_id': TARGET_IDS_BY_VALUE.get(_round_index(field('target_id')), 'none'),
        'reason': REASON_IDS_BY_VALUE.get(_round_index(field('reason_id')), 'none'),
    }


def _normalize_switch_state(raw: Any) -> str:
    state = str(raw or '').strip().upper()
    if state in {'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if state in {'I', 'INTERIOR'}:
        return 'INTERIOR'
    return 'UNKNOWN'


def _switch_state_value(raw: Any) -> float:
    return float({'UNKNOWN': 0, 'EXTERIOR': 1, 'INTERIOR': 2}[_normalize_switch_state(raw)])


def _normalize_stopper_state(raw: Any) -> str:
    state = str(raw or '').strip().lower()
    if state in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return 'open'
    if state in {'1', 'closed', 'close', 'stop', 'blocked', 'on', 'true'}:
        return 'closed'
    return 'unknown'


def _stopper_state_value(raw: Any) -> float:
    state = _normalize_stopper_state(raw)
    if state == 'open':
        return 1.0
    if state == 'closed':
        return 2.0
    return 0.0


def _rails_from_status(status: dict[str, Any]) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status.get('rails'), dict) else {}
    return rails


def _rail_from_status(status: dict[str, Any], side: str) -> dict[str, Any]:
    rail = _rails_from_status(status).get(side, {})
    return rail if isinstance(rail, dict) else {}


def _active_sensor_ids_from_readings(raw_readings: Any) -> set[str]:
    sensor_ids: set[str] = set()
    if isinstance(raw_readings, dict):
        iterable = raw_readings.values()
    elif isinstance(raw_readings, list):
        iterable = raw_readings
    else:
        return sensor_ids
    for reading in iterable:
        if isinstance(reading, dict):
            name = str(reading.get('name') or reading.get('sensor') or '').strip()
        else:
            name = str(reading or '').strip()
        if name:
            sensor_ids.add(name.upper())
    return sensor_ids


def _active_sensor_ids(rail: dict[str, Any]) -> set[str]:
    return (
        _active_sensor_ids_from_readings(rail.get('active_sensors'))
        | _active_sensor_ids_from_readings(rail.get('active_position_sensors'))
    )


def _binary_sensor_bits(status: dict[str, Any]) -> dict[str, dict[str, int]]:
    bits: dict[str, dict[str, int]] = {}
    for side in SIDES:
        active_ids = _active_sensor_ids(_rail_from_status(status, side))
        bits[side] = {
            sensor_name: 1 if sensor_name in active_ids else 0
            for sensor_name in SENSOR_IDS_BY_SIDE[side]
        }
    return bits


def _station_occupancy_eval_labels(status: dict[str, Any]) -> dict[str, dict[str, Any]]:
    sensor_bits = _binary_sensor_bits(status)
    labels: dict[str, dict[str, Any]] = {}
    for side, stations in STATION_SENSOR_GROUPS.items():
        labels[side] = {}
        for station, sensors in stations.items():
            active_sensors = [
                sensor_name
                for sensor_name in sensors
                if sensor_bits.get(side, {}).get(sensor_name, 0) == 1
            ]
            labels[side][station] = {
                'label': 'occupied' if active_sensors else 'empty',
                'active_slot_sensors': active_sensors,
                'source': 'binary_slot_sensors',
            }
    return labels


def _visual_eval_labels_from_status(status: dict[str, Any]) -> dict[str, Any]:
    return {
        'marker_definitions': VISUAL_EVAL_MARKERS,
        'station_occupancy': _station_occupancy_eval_labels(status),
        'policy_visibility': 'visual_input_only',
        'model_input_exposure': 'excluded',
    }


def _switch_states(status: dict[str, Any]) -> dict[str, dict[str, str]]:
    states: dict[str, dict[str, str]] = {}
    for side in SIDES:
        rail = _rail_from_status(status, side)
        switches = rail.get('switches', {}) if isinstance(rail.get('switches'), dict) else {}
        states[side] = {
            name: _normalize_switch_state(switches.get(name))
            for name in DEVICE_NAMES
        }
    return states


def _stopper_states(status: dict[str, Any]) -> dict[str, dict[str, str]]:
    states: dict[str, dict[str, str]] = {}
    for side in SIDES:
        rail = _rail_from_status(status, side)
        stoppers = rail.get('stoppers', {}) if isinstance(rail.get('stoppers'), dict) else {}
        states[side] = {
            name: _normalize_stopper_state(stoppers.get(name))
            for name in DEVICE_NAMES
        }
    return states


def _overhead_images(image_refs: dict[str, Any]) -> dict[str, Any]:
    return {
        camera_name: image_ref
        for camera_name, image_ref in image_refs.items()
        if camera_name in OVERHEAD_IMAGE_NAMES
    }


def _shuttle_command_state(status: dict[str, Any]) -> dict[str, dict[str, str]]:
    state = {
        side: {
            'last_command': 'UNKNOWN',
            'last_shuttle': '',
        }
        for side in SIDES
    }
    primitive = status.get('last_primitive_command')
    if not isinstance(primitive, dict):
        return state
    if str(primitive.get('action') or '').strip().lower() != 'shuttle':
        return state
    side = str(primitive.get('side') or '').strip().lower()
    if side not in SIDES:
        return state
    state[side] = {
        'last_command': str(primitive.get('command') or '').strip().upper() or 'UNKNOWN',
        'last_shuttle': str(primitive.get('shuttle') or '').strip(),
    }
    return state


def _time_since_last_sensor_event(
    sensor_event_times: dict[str, float | None] | None,
    now_s: float | None,
) -> dict[str, float | None]:
    now_value = time.monotonic() if now_s is None else float(now_s)
    times = sensor_event_times or {}
    elapsed: dict[str, float | None] = {}
    side_values = []
    for side in SIDES:
        event_time = times.get(side)
        if event_time is None:
            elapsed[side] = None
            continue
        value = round(max(now_value - float(event_time), 0.0), 3)
        elapsed[side] = value
        side_values.append(value)
    elapsed['any'] = None if not side_values else min(side_values)
    return elapsed


def _model_input_from_status(
    status: dict[str, Any],
    *,
    language: str,
    overhead_images: dict[str, Any],
    last_command: Any,
    sensor_event_times: dict[str, float | None] | None = None,
    now_s: float | None = None,
) -> dict[str, Any]:
    return {
        'language': str(language or ''),
        'overhead_images': _overhead_images(overhead_images),
        'binary_sensor_bits': _binary_sensor_bits(status),
        'switch_states': _switch_states(status),
        'stopper_states': _stopper_states(status),
        'last_command': _normalize_symbolic_action(last_command),
        'shuttle_command_state': _shuttle_command_state(status),
        'time_since_last_sensor_event': _time_since_last_sensor_event(
            sensor_event_times,
            now_s,
        ),
    }


def _privileged_eval_from_status(status: dict[str, Any]) -> dict[str, Any]:
    rails = _rails_from_status(status)
    return {
        'supervisor_status': status,
        'raw_shuttle_states': {
            side: (
                rails.get(side, {}).get('shuttles', {})
                if isinstance(rails.get(side, {}), dict)
                else {}
            )
            for side in SIDES
        },
        'raw_active_sensor_readings': {
            side: {
                'active_sensors': (
                    rails.get(side, {}).get('active_sensors', [])
                    if isinstance(rails.get(side, {}), dict)
                    else []
                ),
                'active_position_sensors': (
                    rails.get(side, {}).get('active_position_sensors', [])
                    if isinstance(rails.get(side, {}), dict)
                    else []
                ),
            }
            for side in SIDES
        },
        'visual_eval_labels': _visual_eval_labels_from_status(status),
    }


def _debug_observation_counts(status: dict[str, Any]) -> dict[str, int]:
    rails = _rails_from_status(status)
    counts: dict[str, int] = {}
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        counts[f'{side}_shuttle_count'] = len(rail.get('shuttles', {}) or {})
        counts[f'{side}_active_sensors_count'] = len(rail.get('active_sensors', []) or [])
        counts[f'{side}_active_position_sensors_count'] = len(
            rail.get('active_position_sensors', []) or []
        )
    return counts


def _encode_state(status: dict[str, Any]) -> list[float]:
    rails = _rails_from_status(status)
    vision = status.get('vision', {}) if isinstance(status.get('vision'), dict) else {}
    values = [
        1.0 if status.get('emergency_stop') else 0.0,
        float(vision.get('image_frames', 0) or 0),
    ]
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        active_ids = _active_sensor_ids(rail)
        values.extend(
            1.0 if sensor_name in active_ids else 0.0
            for sensor_name in SENSOR_IDS_BY_SIDE[side]
        )
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        switches = rail.get('switches', {}) if isinstance(rail.get('switches'), dict) else {}
        values.extend(_switch_state_value(switches.get(name)) for name in DEVICE_NAMES)
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        stoppers = rail.get('stoppers', {}) if isinstance(rail.get('stoppers'), dict) else {}
        values.extend(_stopper_state_value(stoppers.get(name)) for name in DEVICE_NAMES)
    return values


def _task_context_from_status(
    status: dict[str, Any],
    command: Any,
) -> dict[str, Any]:
    command_dict = _as_command_dict(command)
    requested_template = str(
        command_dict.get('template')
        or command_dict.get('template_id')
        or ''
    ).strip()
    active_tasks = status.get('active_tasks', {})
    if isinstance(active_tasks, dict):
        for task in active_tasks.values():
            if not isinstance(task, dict):
                continue
            if requested_template and task.get('template') != requested_template:
                continue
            return task
    completed_tasks = status.get('completed_tasks', [])
    if isinstance(completed_tasks, list):
        for task in reversed(completed_tasks):
            if not isinstance(task, dict):
                continue
            if requested_template and task.get('template') != requested_template:
                continue
            return task
    return {}


def _structured_rail_state(status: dict[str, Any]) -> dict[str, Any]:
    structured: dict[str, Any] = {
        'emergency_stop': bool(status.get('emergency_stop')),
        'rails': {},
    }
    sensor_bits = _binary_sensor_bits(status)
    switch_states = _switch_states(status)
    stopper_states = _stopper_states(status)
    for side in SIDES:
        structured['rails'][side] = {
            'switches': switch_states[side],
            'stoppers': stopper_states[side],
            'sensor_multi_hot': sensor_bits[side],
            'debug_counts': {
                key.removeprefix(f'{side}_'): value
                for key, value in _debug_observation_counts(status).items()
                if key.startswith(f'{side}_')
            },
        }
    return structured


def _event_signature(next_action: dict[str, Any]) -> str:
    return _json_dumps(next_action)


def _wait_condition_for_action(
    next_action: dict[str, Any],
    task_context: dict[str, Any],
) -> dict[str, Any]:
    action = str(next_action.get('action') or '')
    if action == 'switches':
        return {'type': 'switch_state_match', 'target': next_action.get('switches', {})}
    if action == 'stoppers':
        return {'type': 'stopper_state_match', 'target': next_action.get('stoppers', {})}
    if action == 'shuttle':
        command = str(next_action.get('command') or '')
        return {'type': 'shuttle_command_applied', 'target': command}
    if action == 'route_template':
        return {'type': 'task_terminal_status', 'target': 'succeeded_or_failed'}
    if action == 'route_template_phase':
        return {
            'type': 'task_phase_observed',
            'phase': next_action.get('phase', ''),
            'task_status': next_action.get('status', ''),
        }
    if action == 'DONE':
        return {'type': 'terminal', 'status': next_action.get('status', '')}
    if task_context:
        return {
            'type': 'task_context',
            'phase': task_context.get('phase', ''),
            'task_status': task_context.get('status', ''),
        }
    return {}


class Room315VlaDatasetRecorder(Node):
    def __init__(self) -> None:
        super().__init__('room_315_vla_dataset_recorder')

        self.declare_parameter('dataset_dir', '~/.ros/room315_vla_datasets/smolvla_demo')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('right_image_topic', '/room_315/vla/right_rail_rgbd/image')
        self.declare_parameter('left_image_topic', '/room_315/vla/left_rail_rgbd/image')
        self.declare_parameter('status_topic', '/room_315/vla/status')
        self.declare_parameter('goal_topic', '/room_315/vla/user_goal')
        self.declare_parameter('command_topic', '/room_315/vla/command')
        self.declare_parameter('control_topic', '/room_315/vla/episode_control')
        self.declare_parameter('recorder_status_topic', '/room_315/vla/dataset_status')
        self.declare_parameter('sample_period_s', 0.2)
        self.declare_parameter('auto_start_on_goal', True)
        self.declare_parameter('image_max_width', 640)
        self.declare_parameter('image_jpeg_quality', 85)
        self.declare_parameter('camera_name', 'legacy_primary_rgb')

        raw_dataset_dir = str(self.get_parameter('dataset_dir').value)
        self.dataset_dir = Path(os.path.expandvars(raw_dataset_dir)).expanduser()
        self.meta_dir = self.dataset_dir / 'meta'
        self.episodes_dir = self.dataset_dir / 'episodes'
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        self.episodes_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        primary_camera_name = str(self.get_parameter('camera_name').value)
        self.camera_topics = {}
        for camera_name, topic in {
            primary_camera_name: str(self.get_parameter('image_topic').value),
            'right_rail_rgb': str(self.get_parameter('right_image_topic').value),
            'left_rail_rgb': str(self.get_parameter('left_image_topic').value),
        }.items():
            topic = topic.strip()
            if topic:
                self.camera_topics[camera_name] = topic
        self.latest_images: dict[str, Image] = {}
        self.latest_status: dict[str, Any] = {}
        self.latest_goal = 'unspecified Room 315 rail task'
        self.latest_task_index = -1
        self.latest_command: Any = {'action': 'status'}

        self.active = False
        self.episode_index = self._next_episode_index()
        self.episode_dir: Path | None = None
        self.image_dirs: dict[str, Path] = {}
        self.data_stream = None
        self.event_stream = None
        self.frame_index = 0
        self.event_index = 0
        self.episode_id = ''
        self.started_at = ''
        self.last_error = ''
        self.last_event_signature = ''
        self.last_primitive_signature = ''
        self.last_task_phase_by_id: dict[str, str] = {}
        self.completed_task_signatures: set[str] = set()
        self.last_sensor_signature_by_side: dict[str, str] = {side: '' for side in SIDES}
        self.last_sensor_event_time_by_side: dict[str, float | None] = {
            side: None for side in SIDES
        }

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('recorder_status_topic').value),
            10,
        )
        for camera_name, topic in self.camera_topics.items():
            self._subscribe_image(camera_name, topic)
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('goal_topic').value),
            self._on_goal,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('command_topic').value),
            self._on_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('control_topic').value),
            self._on_control,
            10,
        )

        self._write_dataset_info()
        sample_period_s = max(float(self.get_parameter('sample_period_s').value), 0.05)
        self.create_timer(sample_period_s, self._record_sample)
        self.create_timer(1.0, lambda: self._publish_status('idle' if not self.active else 'recording'))
        self._publish_status('ready')
        self.get_logger().info(f'Room 315 VLA dataset recorder ready: {self.dataset_dir}')

    def _subscribe_image(self, camera_name: str, topic: str) -> None:
        topic = topic.strip()
        if not topic:
            return
        self.create_subscription(
            Image,
            topic,
            lambda msg, name=camera_name: self._on_image(name, msg),
            10,
        )

    def _on_image(self, camera_name: str, msg: Image) -> None:
        self.latest_images[camera_name] = msg

    def _on_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.latest_status = parsed
            self._update_sensor_event_tracking(parsed)
            self._record_status_events()

    def _on_goal(self, msg: String) -> None:
        goal = msg.data.strip()
        if not goal:
            return
        self.latest_goal = goal
        self.latest_task_index = self._task_index(goal)
        if _as_bool(self.get_parameter('auto_start_on_goal').value) and not self.active:
            self._start_episode(goal)
        self._publish_status('goal_received')

    def _on_command(self, msg: String) -> None:
        self.latest_command = _parse_json_or_text(msg.data)
        self._record_command_event(self.latest_command)
        self._publish_status('command_received')

    def _on_control(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return
        lowered = command.lower()
        if lowered.startswith('start'):
            goal = command[5:].strip() or self.latest_goal
            self.latest_goal = goal
            self.latest_task_index = self._task_index(goal)
            self._start_episode(goal)
        elif lowered.startswith('stop'):
            success = None
            if 'success' in lowered:
                success = True
            elif 'failure' in lowered or 'fail' in lowered:
                success = False
            self._stop_episode(success=success, discarded=False)
        elif lowered.startswith('discard'):
            self._stop_episode(success=None, discarded=True)
        else:
            self.last_error = f'unknown episode control command: {command}'
            self.get_logger().warning(self.last_error)
            self._publish_status('error')

    def _start_episode(self, goal: str) -> None:
        if self.active:
            self._stop_episode(success=None, discarded=False)
        self.latest_goal = goal.strip() or 'unspecified Room 315 rail task'
        self.latest_task_index = self._task_index(self.latest_goal)
        suffix = _safe_filename(goal, 'room315_task')
        episode_name = f'episode_{self.episode_index:06d}_{suffix}'
        self.episode_dir = self.episodes_dir / episode_name
        self.episode_id = episode_name
        self.image_dirs = {}
        for camera_name in self.camera_topics:
            image_dir = self.episode_dir / 'images' / camera_name
            image_dir.mkdir(parents=True, exist_ok=True)
            self.image_dirs[camera_name] = image_dir
        self.data_stream = (self.episode_dir / 'data.jsonl').open('w', encoding='utf-8')
        self.event_stream = (self.episode_dir / 'events.jsonl').open('w', encoding='utf-8')
        self.frame_index = 0
        self.event_index = 0
        self.started_at = _utc_now()
        self.last_event_signature = ''
        self.last_primitive_signature = ''
        self.last_task_phase_by_id = {}
        self.completed_task_signatures = set()
        self.last_sensor_signature_by_side = {side: '' for side in SIDES}
        self.last_sensor_event_time_by_side = {side: None for side in SIDES}
        self.active = True
        self.last_error = ''
        self.get_logger().info(f'Started VLA dataset episode {self.episode_index}: {goal}')
        self._publish_status('started')

    def _stop_episode(self, *, success: bool | None, discarded: bool) -> None:
        if not self.active:
            return
        terminal_status = 'discarded' if discarded else (
            'success' if success is True else 'failure' if success is False else 'stopped'
        )
        self._record_event(
            {
                'action': 'DONE',
                'status': terminal_status,
                'template': _as_command_dict(self.latest_command).get('template', ''),
            },
            original_command={'action': 'DONE', 'status': terminal_status},
            event_type='episode_terminal',
            status_text=terminal_status,
        )
        if self.data_stream is not None:
            self.data_stream.close()
            self.data_stream = None
        if self.event_stream is not None:
            self.event_stream.close()
            self.event_stream = None
        summary = {
            'episode_index': self.episode_index,
            'episode_id': self.episode_id,
            'task': self.latest_goal,
            'task_index': self.latest_task_index,
            'started_at': self.started_at,
            'ended_at': _utc_now(),
            'frames': self.frame_index,
            'events': self.event_index,
            'success': success,
            'discarded': discarded,
            'format': 'room315_vla_event_labeled_jsonl_v2',
            'training_labels': 'events.jsonl',
            'raw_replay': 'data.jsonl',
            'safety_decoder_metrics': self._safety_decoder_metrics(),
        }
        if self.episode_dir is not None:
            with (self.episode_dir / 'episode.json').open('w', encoding='utf-8') as stream:
                json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
        self.get_logger().info(
            f'Stopped VLA dataset episode {self.episode_index}: frames={self.frame_index}'
        )
        self.episode_index += 1
        self.active = False
        self.episode_dir = None
        self.episode_id = ''
        self.image_dirs = {}
        self._publish_status('discarded' if discarded else 'stopped')

    def _record_sample(self) -> None:
        self._write_raw_sample(self.latest_command)

    def _update_sensor_event_tracking(self, status: dict[str, Any]) -> None:
        now = time.monotonic()
        sensor_bits = _binary_sensor_bits(status)
        for side in SIDES:
            signature = _json_dumps(sensor_bits.get(side, {}))
            if signature == self.last_sensor_signature_by_side.get(side):
                continue
            self.last_sensor_signature_by_side[side] = signature
            self.last_sensor_event_time_by_side[side] = now

    def _safety_decoder_metrics(self) -> dict[str, Any]:
        safety = self.latest_status.get('safety_decoder', {})
        if isinstance(safety, dict) and isinstance(safety.get('metrics'), dict):
            return dict(safety['metrics'])
        metrics = self.latest_status.get('safety_decoder_metrics', {})
        return dict(metrics) if isinstance(metrics, dict) else {}

    def _write_raw_sample(
        self,
        command: Any,
        task_context: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        if not self.active or not self.latest_images or self.data_stream is None:
            return None
        try:
            image_relpaths = {
                camera_name: self._write_image(camera_name, image)
                for camera_name, image in self.latest_images.items()
                if camera_name in self.image_dirs
            }
            task_context = task_context or _task_context_from_status(
                self.latest_status,
                command,
            )
            frame_index = self.frame_index
            model_input = _model_input_from_status(
                self.latest_status,
                language=self.latest_goal,
                overhead_images=image_relpaths,
                last_command=command,
                sensor_event_times=getattr(self, 'last_sensor_event_time_by_side', {}),
                now_s=time.monotonic(),
            )
            row = {
                'episode_index': self.episode_index,
                'episode_id': self.episode_id,
                'frame_index': frame_index,
                'timestamp': self._now_seconds(),
                'task': self.latest_goal,
                'task_index': self.latest_task_index,
                'task_id': task_context.get('task_id', ''),
                'task_template': task_context.get('template', ''),
                'task_phase': task_context.get('phase', ''),
                'task_status': task_context.get('status', ''),
                'task_failure_reason': task_context.get('failure_reason', ''),
                'observation.state': _encode_state(self.latest_status),
                'observation.state_schema': OBSERVATION_STATE_FIELDS,
                'observation.debug_counts': _debug_observation_counts(self.latest_status),
                'model_input_schema_version': MODEL_INPUT_SCHEMA_VERSION,
                'model_input': model_input,
                'action': _encode_action(command),
                'action_schema': ACTION_VECTOR_FIELDS,
                'command': command,
                'raw_replay_only': True,
                'high_level_template': (
                    task_context.get('template')
                    or _as_command_dict(command).get('template')
                    or ''
                ),
                'primitive_commands': task_context.get('primitive_commands', []),
                'last_primitive_command': self.latest_status.get('last_primitive_command'),
                'safety_decoder_metrics': self._safety_decoder_metrics(),
                'privileged_eval': _privileged_eval_from_status(self.latest_status),
                'debug': {
                    'supervisor_status': self.latest_status,
                    'observation_counts': _debug_observation_counts(self.latest_status),
                },
            }
            for camera_name, image_relpath in image_relpaths.items():
                row[f'observation.images.{camera_name}'] = image_relpath
            self.data_stream.write(_json_dumps(row) + '\n')
            self.data_stream.flush()
            self.frame_index += 1
            return {
                'frame_index': frame_index,
                'image_frame_refs': image_relpaths,
            }
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().error(f'Failed to record VLA dataset sample: {exc}')
            self._publish_status('error')
            return None

    def _record_command_event(self, command: Any) -> None:
        next_action = _normalize_symbolic_action(command)
        if not _is_meaningful_event_action(next_action):
            return
        self._record_event(
            next_action,
            original_command=command,
            event_type='command',
        )

    def _record_status_events(self) -> None:
        if not self.active:
            return
        primitive = self.latest_status.get('last_primitive_command')
        if isinstance(primitive, dict):
            next_action = _normalize_symbolic_action(primitive)
            primitive_signature = _event_signature(next_action)
            if (
                _is_meaningful_event_action(next_action)
                and primitive_signature != self.last_primitive_signature
            ):
                self.last_primitive_signature = primitive_signature
                self._record_event(
                    next_action,
                    original_command=primitive,
                    event_type='supervisor_primitive',
                    task_context=self._task_context_for_primitive(primitive),
                )

        active_tasks = self.latest_status.get('active_tasks', {})
        if isinstance(active_tasks, dict):
            for task in active_tasks.values():
                if not isinstance(task, dict):
                    continue
                task_id = str(task.get('task_id') or '')
                phase = str(task.get('phase') or '')
                if not task_id or not phase:
                    continue
                if self.last_task_phase_by_id.get(task_id) == phase:
                    continue
                self.last_task_phase_by_id[task_id] = phase
                self._record_event(
                    {
                        'action': 'route_template_phase',
                        'template': task.get('template', ''),
                        'task_id': task_id,
                        'phase': phase,
                        'status': task.get('status', ''),
                    },
                    original_command=self.latest_command,
                    event_type='task_phase',
                    task_context=task,
                )

        completed_tasks = self.latest_status.get('completed_tasks', [])
        if isinstance(completed_tasks, list):
            for task in completed_tasks:
                if not isinstance(task, dict):
                    continue
                task_id = str(task.get('task_id') or '')
                status = str(task.get('status') or '')
                signature = f'{task_id}:{status}:{task.get("phase", "")}'
                if not task_id or signature in self.completed_task_signatures:
                    continue
                self.completed_task_signatures.add(signature)
                self._record_event(
                    {
                        'action': 'DONE',
                        'status': status,
                        'template': task.get('template', ''),
                        'task_id': task_id,
                        'failure_reason': task.get('failure_reason', ''),
                    },
                    original_command=self.latest_command,
                    event_type='task_terminal',
                    task_context=task,
                    status_text=status,
                )

    def _task_context_for_primitive(self, primitive: dict[str, Any]) -> dict[str, Any]:
        task_id = str(primitive.get('task_id') or '')
        active_tasks = self.latest_status.get('active_tasks', {})
        if task_id and isinstance(active_tasks, dict):
            task = active_tasks.get(task_id)
            if isinstance(task, dict):
                return task
        completed_tasks = self.latest_status.get('completed_tasks', [])
        if task_id and isinstance(completed_tasks, list):
            for task in reversed(completed_tasks):
                if isinstance(task, dict) and str(task.get('task_id') or '') == task_id:
                    return task
        return _task_context_from_status(self.latest_status, self.latest_command)

    def _record_event(
        self,
        next_action: Any,
        *,
        original_command: Any,
        event_type: str,
        task_context: dict[str, Any] | None = None,
        status_text: str = '',
    ) -> None:
        if not self.active or self.event_stream is None:
            return
        normalized_action = _normalize_symbolic_action(next_action)
        if not _is_meaningful_event_action(normalized_action):
            return
        task_context = task_context or _task_context_from_status(self.latest_status, original_command)
        wait_condition = _wait_condition_for_action(normalized_action, task_context)
        event_action = _event_action_v2_from_symbolic_action(
            normalized_action,
            wait_condition=wait_condition,
            task_context=task_context,
            event_type=event_type,
            status_text=status_text,
        )
        signature = _event_signature(event_action)
        if signature == self.last_event_signature:
            return
        raw_sample = self._write_raw_sample(original_command, task_context)
        image_frame_refs = {} if raw_sample is None else raw_sample['image_frame_refs']
        model_input = _model_input_from_status(
            self.latest_status,
            language=self.latest_goal,
            overhead_images=image_frame_refs,
            last_command=original_command,
            sensor_event_times=getattr(self, 'last_sensor_event_time_by_side', {}),
            now_s=time.monotonic(),
        )
        row = {
            'episode_index': self.episode_index,
            'episode_id': self.episode_id,
            'event_index': self.event_index,
            'step_index': self.event_index,
            'event_type': event_type,
            'timestamp': self._now_seconds(),
            'task': self.latest_goal,
            'task_index': self.latest_task_index,
            'task_id': task_context.get('task_id', normalized_action.get('task_id', '')),
            'template': (
                task_context.get('template')
                or normalized_action.get('template')
                or _as_command_dict(original_command).get('template')
                or ''
            ),
            'phase': task_context.get('phase', normalized_action.get('phase', '')),
            'task_status': task_context.get('status', normalized_action.get('status', '')),
            'failure_reason': (
                task_context.get('failure_reason')
                or normalized_action.get('failure_reason')
                or ''
            ),
            'observation_frame_index': None if raw_sample is None else raw_sample['frame_index'],
            'image_frame_refs': image_frame_refs,
            'structured_rail_state': _structured_rail_state(self.latest_status),
            'observation.state': _encode_state(self.latest_status),
            'observation.state_schema': OBSERVATION_STATE_FIELDS,
            'observation.debug_counts': _debug_observation_counts(self.latest_status),
            'model_input_schema_version': MODEL_INPUT_SCHEMA_VERSION,
            'model_input': model_input,
            'original_command': original_command,
            'legacy_next_action': normalized_action,
            'next_action': event_action,
            'action': event_action,
            'action_vector': _action_vector_or_none(event_action),
            'action_schema': ACTION_VECTOR_FIELDS,
            'wait_condition': wait_condition,
            'safety_decoder_metrics': self._safety_decoder_metrics(),
            'privileged_eval': _privileged_eval_from_status(self.latest_status),
            'debug': {
                'observation_counts': _debug_observation_counts(self.latest_status),
            },
            'status': {
                'event_status': status_text,
                'supervisor_last_result': self.latest_status.get('last_result', ''),
            },
        }
        for camera_name, image_relpath in image_frame_refs.items():
            row[f'observation.images.{camera_name}'] = image_relpath
        self.event_stream.write(_json_dumps(row) + '\n')
        self.event_stream.flush()
        self.event_index += 1
        self.last_event_signature = signature

    def _write_image(self, camera_name: str, image: Image) -> str:
        if camera_name not in self.image_dirs or self.episode_dir is None:
            raise RuntimeError('episode image directory is not ready')
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        max_width = int(self.get_parameter('image_max_width').value)
        if max_width > 0 and frame.shape[1] > max_width:
            scale = max_width / float(frame.shape[1])
            frame = cv2.resize(
                frame,
                (max_width, max(1, int(frame.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )
        filename = f'{self.frame_index:06d}.jpg'
        image_path = self.image_dirs[camera_name] / filename
        quality = min(max(int(self.get_parameter('image_jpeg_quality').value), 1), 100)
        ok = cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError(f'failed to write image {image_path}')
        return str(image_path.relative_to(self.dataset_dir))

    def _now_seconds(self) -> float:
        try:
            return self.get_clock().now().nanoseconds / 1e9
        except AttributeError:
            return time.time()

    def _write_dataset_info(self) -> None:
        info = {
            'format': 'room315_vla_event_labeled_jsonl_v2',
            'model_input_schema_version': MODEL_INPUT_SCHEMA_VERSION,
            'created_or_updated_at': _utc_now(),
            'description': (
                'Room 315 VLA demonstrations for adapting open-source policies '
                'such as SmolVLA/LeRobot to discrete industrial rail-cell control.'
            ),
            'training_split_note': (
                'Use per-episode events.jsonl for supervised training labels. '
                'Per-frame data.jsonl is retained for raw replay, visual auditing, '
                'and temporal reconstruction only. Do not train on framewise '
                'repeated current_command labels.'
            ),
            'training_label_file': 'events.jsonl',
            'training_extractor': 'room_315_vla_event_extractor.py',
            'raw_replay_file': 'data.jsonl',
            'fps': round(1.0 / max(float(self.get_parameter('sample_period_s').value), 0.05), 3),
            'image_features': {
                f'observation.images.{camera_name}': {
                    'encoding': 'jpg',
                    'source_topic': topic,
                }
                for camera_name, topic in self.camera_topics.items()
            },
            'state_features': OBSERVATION_STATE_FIELDS,
            'model_input_features': MODEL_INPUT_FIELDS,
            'model_input_note': (
                'model_input is the only model-facing observation object. It contains '
                'language, overhead image references, binary sensor bits, normalized '
                'switch/stopper states, last_command, shuttle_command_state, and '
                'time_since_last_sensor_event. Exact Gazebo pose, true segment, '
                'distance-to-switch, and normalized position values are excluded.'
            ),
            'privileged_eval_fields': PRIVILEGED_EVAL_FIELDS,
            'visual_eval_markers': VISUAL_EVAL_MARKERS,
            'visual_eval_note': (
                'Visual station/status/inspection/obstacle markers are visible in '
                'overhead images and mirrored only in privileged_eval labels. They '
                'are intentionally excluded from observation.state and model_input.'
            ),
            'sensor_features_by_side': {
                side: list(sensor_names)
                for side, sensor_names in SENSOR_IDS_BY_SIDE.items()
            },
            'debug_observation_fields': DEBUG_OBSERVATION_FIELDS,
            'action_features': ACTION_VECTOR_FIELDS,
            'symbolic_action_features': EVENT_ACTION_FIELDS,
            'action_encodings': {
                'primitive_id': PRIMITIVE_IDS,
                'side_id': SIDE_IDS,
                'device_mask': {'unchanged': 0, 'selected': 1},
                'switch_value': SWITCH_VALUE_IDS,
                'stopper_value': STOPPER_VALUE_IDS,
                'wait_condition_id': WAIT_CONDITION_IDS,
                'target_id': TARGET_IDS,
                'reason_id': REASON_IDS,
            },
            'state_encodings': {
                'sensor_multi_hot': {'inactive': 0, 'active': 1},
                'switch': {'UNKNOWN': 0, 'EXTERIOR': 1, 'INTERIOR': 2},
                'stopper': {'unknown': 0, 'open': 1, 'closed': 2},
            },
            'event_features': [
                'episode_id',
                'step_index',
                'timestamp',
                'task',
                'observation.images.*',
                'observation.state',
                'action',
                'privileged_eval',
            ],
            'topics': {
                'goal': str(self.get_parameter('goal_topic').value),
                'command': str(self.get_parameter('command_topic').value),
                'status': str(self.get_parameter('status_topic').value),
                'control': str(self.get_parameter('control_topic').value),
                'right_image': str(self.get_parameter('right_image_topic').value),
                'left_image': str(self.get_parameter('left_image_topic').value),
            },
        }
        with (self.meta_dir / 'info.json').open('w', encoding='utf-8') as stream:
            json.dump(info, stream, ensure_ascii=False, indent=2, sort_keys=True)

    def _task_index(self, task: str) -> int:
        tasks_path = self.meta_dir / 'tasks.jsonl'
        task = task.strip() or 'unspecified Room 315 rail task'
        tasks: dict[str, int] = {}
        if tasks_path.exists():
            with tasks_path.open('r', encoding='utf-8') as stream:
                for line in stream:
                    try:
                        item = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if isinstance(item, dict) and isinstance(item.get('task'), str):
                        tasks[item['task']] = int(item.get('task_index', len(tasks)))
        if task in tasks:
            return tasks[task]
        task_index = len(tasks)
        with tasks_path.open('a', encoding='utf-8') as stream:
            stream.write(_json_dumps({'task_index': task_index, 'task': task}) + '\n')
        return task_index

    def _next_episode_index(self) -> int:
        max_index = -1
        for path in self.episodes_dir.glob('episode_*'):
            parts = path.name.split('_')
            if len(parts) >= 2 and parts[1].isdigit():
                max_index = max(max_index, int(parts[1]))
        return max_index + 1

    def _publish_status(self, state: str) -> None:
        msg = String()
        msg.data = _json_dumps({
            'state': state,
            'active': self.active,
            'dataset_dir': str(self.dataset_dir),
            'episode_index': self.episode_index,
            'episode_id': self.episode_id,
            'frame_index': self.frame_index,
            'event_index': self.event_index,
            'task': self.latest_goal,
            'task_index': self.latest_task_index,
            'available_cameras': sorted(self.latest_images),
            'last_error': self.last_error,
        })
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop_episode(success=None, discarded=False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Room315VlaDatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
