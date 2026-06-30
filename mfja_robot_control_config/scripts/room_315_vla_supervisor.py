#!/usr/bin/env python3

import json
import re
import time
from pathlib import Path
import sys
from typing import Any

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String

from mfja_rail_interfaces.msg import NamedState
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleCommand
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import StopperCommand
from mfja_rail_interfaces.msg import StopperState
from mfja_rail_interfaces.msg import SwitchCommand
from mfja_rail_interfaces.msg import SwitchState
from mfja_rail_interfaces.srv import AddShuttle


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_multi_shuttle import ACTION_VECTOR_V3_FIELDS as EVENT_ACTION_VECTOR_V3_FIELDS
from room_315_multi_shuttle import ShuttleRegistry
from room_315_multi_shuttle import decode_action_v3
from room_315_multi_shuttle import empty_fleet_safety_metrics
from room_315_multi_shuttle import fleet_safety_state_from_rails
from room_315_multi_shuttle import gazebo_entity_name as multi_gazebo_entity_name
from room_315_multi_shuttle import normalize_fleet_block_id
from room_315_multi_shuttle import normalize_fleet_slot_id
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_multi_shuttle import validate_fleet_command


SIDES = ('right', 'left')
SWITCHES = ('A1', 'A2', 'A3', 'A4')
DEFAULT_SLOT_SENSOR_BY_SIDE = {
    'right': {
        '1': 'DZI1R',
        '2': 'DZI2R',
        '3': 'DZI3R',
        '4': 'DZI4R',
    },
    'left': {
        '1': 'DZI1L',
        '2': 'DZI2L',
        '3': 'DZI3L',
        '4': 'DZI4L',
    },
}
STOPPER_SENSOR_BY_STOPPER = {
    'A1': 'A1_STOPPER_SENSOR',
    'A2': 'A2_STOPPER_SENSOR',
    'A3': 'A3_STOPPER_SENSOR',
    'A4': 'A4_STOPPER_SENSOR',
}
TASK_TERMINAL_STATES = {'succeeded', 'failed'}
SAFETY_ACTION_ALIASES = {
    'switch': 'switches',
    'stopper': 'stoppers',
    'shuttle_command': 'shuttle',
    'spawn_shuttle': 'add_shuttle',
    'all_off': 'stop_all',
    'estop': 'emergency_stop',
    'clear_estop': 'clear_emergency_stop',
    'reset_estop': 'clear_emergency_stop',
}
SAFETY_ACTIONS = {
    'status',
    'snapshot',
    'switches',
    'stoppers',
    'shuttle',
    'add_shuttle',
    'stop_all',
    'emergency_stop',
    'clear_emergency_stop',
}
SAFE_STOPPED_MODES = {'', 'STOPPED', 'WAITING', 'DISABLED', 'OFF', 'IDLE'}
FALLING_MODES = {'FALLING', 'FALLEN'}
SWITCH_NEAR_SEGMENTS = {
    'A1': {'A12', 'A12E', 'A12I', 'A14'},
    'A2': {'A12', 'A12E', 'A12I', 'A23'},
    'A3': {'A23', 'A34', 'A34E', 'A34I'},
    'A4': {'A14', 'A34', 'A34E', 'A34I'},
}
SWITCH_SENSOR_PREFIX_BY_SIDE = {
    'right': {
        'A1': ('DZI1R', 'DA1R', 'DA1ER', 'DA1IR', 'A1_STOPPER_SENSOR'),
        'A2': ('DZI2R', 'DA2R', 'DA2ER', 'DA2IR', 'A2_STOPPER_SENSOR'),
        'A3': ('DZI3R', 'DA3R', 'DA3ER', 'DA3IR', 'A3_STOPPER_SENSOR'),
        'A4': ('DZI4R', 'DA4R', 'DA4ER', 'DA4IR', 'A4_STOPPER_SENSOR'),
    },
    'left': {
        'A1': ('DZI1L', 'DA1L', 'DA1EL', 'DA1IL', 'A1_STOPPER_SENSOR'),
        'A2': ('DZI2L', 'DA2L', 'DA2EL', 'DA2IL', 'A2_STOPPER_SENSOR'),
        'A3': ('DZI3L', 'DA3L', 'DA3EL', 'DA3IL', 'A3_STOPPER_SENSOR'),
        'A4': ('DZI4L', 'DA4L', 'DA4EL', 'DA4IL', 'A4_STOPPER_SENSOR'),
    },
}
SWITCH_CLEAR_DISTANCE_M = 0.35
SWITCH_VALUE_BY_ID = {
    1: 'EXTERIOR',
    2: 'INTERIOR',
}
STOPPER_VALUE_BY_ID = {
    1: '0',
    2: '1',
}
EVENT_PRIMITIVE_BY_ID = {
    0: 'WAIT',
    1: 'DONE',
    2: 'SET_SWITCHES',
    3: 'SET_STOPPERS',
    4: 'SHUTTLE_ON',
    5: 'STOP_NOW',
    6: 'EMERGENCY_STOP',
}
EVENT_SIDE_BY_ID = {
    0: 'right',
    1: 'left',
}
EVENT_WAIT_CONDITION_BY_ID = {
    0: 'none',
    1: 'switch_state_match',
    2: 'stopper_state_match',
    3: 'shuttle_command_applied',
    4: 'task_terminal_status',
    5: 'reserved_legacy_wait_5',
    6: 'terminal',
    7: 'target_sensor_active',
}
EVENT_TARGET_BY_ID = {
    0: 'none',
    1: 'A1',
    2: 'A2',
    3: 'A3',
    4: 'A4',
    5: 'ALL_SWITCHES',
    6: 'ALL_STOPPERS',
    7: 'MULTIPLE_DEVICES',
    8: 'right_shuttle',
    9: 'left_shuttle',
    10: 'reserved_legacy_target_10',
    11: 'reserved_legacy_target_11',
    12: 'reserved_legacy_target_12',
    13: 'reserved_legacy_target_13',
    14: 'reserved_legacy_target_14',
    15: 'reserved_legacy_target_15',
    16: 'reserved_legacy_target_16',
    17: 'terminal',
    18: 'DZI1R',
    19: 'DZI2R',
    20: 'DZI3R',
    21: 'DZI4R',
    22: 'DZI1L',
    23: 'DZI2L',
    24: 'DZI3L',
    25: 'DZI4L',
    26: 'DA3IR',
    27: 'DA3IL',
}
EVENT_REASON_BY_ID = {
    0: 'none',
    1: 'command_event',
    2: 'reserved_legacy_reason_2',
    3: 'reserved_legacy_reason_3',
    4: 'task_succeeded',
    5: 'task_failed',
    6: 'episode_stopped',
    7: 'episode_discarded',
    8: 'switch_update',
    9: 'stopper_update',
    10: 'shuttle_start',
    11: 'shuttle_stop',
    12: 'emergency',
    13: 'unsupported_command',
}
EVENT_ACTION_VECTOR_FIELDS = EVENT_ACTION_VECTOR_V3_FIELDS



def _default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_vla'
            / 'vla_supervisor.yaml'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'room_315_vla'
            / 'vla_supervisor.yaml'
        )


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open('r', encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f'{path} must contain a YAML mapping.')
    return loaded


def _clean_token(value: Any) -> str:
    return str(value).strip()


def _normalize_side(raw: Any, default: str = 'right') -> str:
    value = _clean_token(raw).lower()
    if value in {'right', 'r', 'droit', 'droite'}:
        return 'right'
    if value in {'left', 'l', 'gauche'}:
        return 'left'
    return default


def _normalize_loop(raw: Any | None) -> str | None:
    if raw is None:
        return None
    value = _clean_token(raw).lower()
    if value in {'g', 'e', 'exterior', 'external', 'grand', 'grand_boucle', 'big'}:
        return 'exterior'
    if value in {'s', 'i', 'interior', 'internal', 'petit', 'petit_boucle', 'small'}:
        return 'interior'
    return None


def _switch_state_for_loop(loop: str | None) -> str | None:
    if loop == 'exterior':
        return 'EXTERIOR'
    if loop == 'interior':
        return 'INTERIOR'
    return None


def _canonical_switch_state(raw: Any) -> str:
    value = _clean_token(raw).upper()
    if value in {'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if value in {'I', 'INTERIOR'}:
        return 'INTERIOR'
    return value


def _normalize_stopper_state(raw: Any) -> str:
    value = _clean_token(raw).lower()
    if value in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return '0'
    if value in {'1', 'close', 'closed', 'stop', 'blocked', 'on', 'true'}:
        return '1'
    return _clean_token(raw)


def _strict_side(raw: Any) -> str:
    value = _clean_token(raw).lower()
    if value in {'right', 'r', 'droit', 'droite'}:
        return 'right'
    if value in {'left', 'l', 'gauche'}:
        return 'left'
    return ''


def _normalize_safety_action(raw: Any) -> str:
    action = _clean_token(raw).lower()
    return SAFETY_ACTION_ALIASES.get(action, action)


def _empty_safety_metrics() -> dict[str, Any]:
    metrics = {
        'total_proposed_actions': 0,
        'accepted_actions': 0,
        'rejected_actions': 0,
        'illegal_proposal_rate': 0.0,
        'rejected_action_rate': 0.0,
        'rejection_reasons': {},
    }
    metrics.update(empty_fleet_safety_metrics())
    return metrics


def _safety_decision(
    *,
    accepted: bool,
    original_action: Any,
    corrected_action: Any | None = None,
    reason: str = '',
) -> dict[str, Any]:
    if corrected_action is None and accepted:
        corrected_action = original_action
    safe_correction = accepted and corrected_action != original_action
    return {
        'accepted': bool(accepted),
        'reason': reason,
        'original_action': original_action,
        'corrected_action': corrected_action if accepted else None,
        'safe_correction': bool(safe_correction),
        'raw_action': original_action,
        'illegal_proposal': not bool(accepted),
        'rejected_action': None if accepted else original_action,
        'executed_action': corrected_action if accepted else None,
    }


def _rail_snapshot(rails: dict[str, Any], side: str) -> dict[str, Any]:
    rail = rails.get(side, {}) if isinstance(rails, dict) else {}
    return rail if isinstance(rail, dict) else {}


def _rail_shuttles(rails: dict[str, Any], side: str) -> dict[str, Any]:
    shuttles = _rail_snapshot(rails, side).get('shuttles', {})
    return shuttles if isinstance(shuttles, dict) else {}


def _shuttle_mode_from_state(state: dict[str, Any]) -> str:
    return _clean_token(state.get('mode', '')).upper() if isinstance(state, dict) else ''


def _shuttle_is_falling(state: dict[str, Any]) -> bool:
    return _shuttle_mode_from_state(state) in FALLING_MODES


def _shuttle_is_moving(state: dict[str, Any]) -> bool:
    if not isinstance(state, dict):
        return False
    mode = _shuttle_mode_from_state(state)
    if mode in FALLING_MODES or mode in SAFE_STOPPED_MODES:
        return False
    try:
        if abs(float(state.get('speed', 0.0) or 0.0)) > 0.001:
            return True
    except (TypeError, ValueError):
        pass
    return mode in {'MOVING', 'ENABLED', 'ENABLE', 'RUNNING', 'ON', 'ACTIVE'}


def _any_falling_shuttle(rails: dict[str, Any]) -> str:
    for side in SIDES:
        for shuttle_name, state in _rail_shuttles(rails, side).items():
            if _shuttle_is_falling(state):
                return f'{side} shuttle {shuttle_name} is in {state.get("mode")} mode'
    return ''


def _active_sensor_readings_from_rail(rail: dict[str, Any]) -> list[dict[str, Any]]:
    readings: list[dict[str, Any]] = []
    for key in ('active_sensors', 'active_position_sensors'):
        raw_readings = rail.get(key, [])
        if not isinstance(raw_readings, list):
            continue
        readings.extend(item for item in raw_readings if isinstance(item, dict))
    return readings


def _active_sensor_names(rail: dict[str, Any]) -> set[str]:
    return {
        _clean_token(reading.get('name', '')).upper()
        for reading in _active_sensor_readings_from_rail(rail)
        if _clean_token(reading.get('name', ''))
    }


def _shuttle_near_switch(shuttle_state: dict[str, Any], switch_name: str) -> bool:
    segment = _clean_token(shuttle_state.get('segment', '')).upper()
    if not segment:
        return False
    if _shuttle_segment_cleared_switch(shuttle_state, switch_name, segment):
        return False
    near_segments = SWITCH_NEAR_SEGMENTS.get(switch_name, set())
    return segment in near_segments or any(segment.startswith(prefix) for prefix in near_segments)


def _shuttle_segment_cleared_switch(
    shuttle_state: dict[str, Any],
    switch_name: str,
    segment: str,
) -> bool:
    try:
        s = float(shuttle_state.get('s'))
    except (TypeError, ValueError):
        return False
    if switch_name == 'A3' and segment in {'A34E', 'A34I'}:
        return s > SWITCH_CLEAR_DISTANCE_M
    return False


def _active_sensor_near_switch(rail: dict[str, Any], side: str, switch_name: str) -> bool:
    names = _active_sensor_names(rail)
    near_names = {
        name.upper()
        for name in SWITCH_SENSOR_PREFIX_BY_SIDE.get(side, {}).get(switch_name, ())
    }
    return bool(names & near_names)


def _unsafe_switch_change_reason(
    rails: dict[str, Any],
    side: str,
    switch_name: str,
) -> str:
    rail = _rail_snapshot(rails, side)
    moving_near = []
    for shuttle_name, shuttle_state in _rail_shuttles(rails, side).items():
        if _shuttle_is_moving(shuttle_state) and _shuttle_near_switch(shuttle_state, switch_name):
            moving_near.append(str(shuttle_name))
    if moving_near:
        return (
            f'unsafe switch change: {side} switch {switch_name} is near moving '
            f'shuttle(s) {", ".join(moving_near)}'
        )
    if _active_sensor_near_switch(rail, side, switch_name):
        moving = [
            str(shuttle_name)
            for shuttle_name, shuttle_state in _rail_shuttles(rails, side).items()
            if _shuttle_is_moving(shuttle_state)
        ]
        if moving:
            return (
                f'unsafe switch change: {side} switch {switch_name} has active '
                f'nearby sensors while shuttle(s) {", ".join(moving)} are moving'
            )
    return ''


def _occupied_guarded_segment_reason(
    rails: dict[str, Any],
    side: str,
    switch_name: str,
) -> str:
    rail = _rail_snapshot(rails, side)
    occupied_shuttles = []
    for shuttle_name, shuttle_state in _rail_shuttles(rails, side).items():
        if _shuttle_is_falling(shuttle_state):
            continue
        if _shuttle_near_switch(shuttle_state, switch_name):
            occupied_shuttles.append(str(shuttle_name))
    if occupied_shuttles:
        return (
            f'unsafe switch change: {side} switch {switch_name} guarded segment '
            f'is occupied by shuttle(s) {", ".join(occupied_shuttles)}'
        )
    if _active_sensor_near_switch(rail, side, switch_name):
        return (
            f'unsafe switch change: {side} switch {switch_name} guarded segment '
            'has active occupancy sensors'
        )
    return ''


def _closed_stoppers(rails: dict[str, Any], side: str) -> list[str]:
    stoppers = _rail_snapshot(rails, side).get('stoppers', {})
    if not isinstance(stoppers, dict):
        return []
    return [
        str(name).upper()
        for name, state in stoppers.items()
        if str(name).upper() in SWITCHES and _normalize_stopper_state(state) == '1'
    ]


def _rail_has_moving_shuttle(rails: dict[str, Any], side: str) -> bool:
    return any(_shuttle_is_moving(state) for state in _rail_shuttles(rails, side).values())


def _shuttle_exists_on_side(
    rails: dict[str, Any],
    side: str,
    shuttle_name: str,
) -> bool:
    shuttles = _rail_shuttles(rails, side)
    if shuttle_name == 'ALL':
        return bool(shuttles)
    return shuttle_name in shuttles


def _shuttle_name_matches_side(shuttle_name: str, side: str) -> bool:
    lowered = shuttle_name.lower()
    if 'left' in lowered and side != 'left':
        return False
    if 'right' in lowered and side != 'right':
        return False
    return True


def _sensor_for_slot(
    slot_sensor_by_side: dict[str, dict[str, str]],
    side: str,
    slot: str,
) -> str:
    return (slot_sensor_by_side.get(side, {}) or {}).get(str(slot), '')


def _find_source_shuttle_for_slots(
    rails: dict[str, Any],
    slot_sensor_by_side: dict[str, dict[str, str]],
    side: str,
    slots: list[str],
) -> tuple[str, str]:
    matches = _find_source_shuttles_for_slots(
        rails,
        slot_sensor_by_side,
        side,
        slots,
    )
    return matches[0] if matches else ('', '')


def _find_source_shuttles_for_slots(
    rails: dict[str, Any],
    slot_sensor_by_side: dict[str, dict[str, str]],
    side: str,
    slots: list[str],
) -> list[tuple[str, str]]:
    rail = _rail_snapshot(rails, side)
    readings = _active_sensor_readings_from_rail(rail)
    matches: list[tuple[str, str]] = []
    for slot in slots:
        wanted_sensor = _sensor_for_slot(slot_sensor_by_side, side, str(slot)).casefold()
        if not wanted_sensor:
            continue
        for reading in readings:
            if str(reading.get('name', '')).casefold() != wanted_sensor:
                continue
            shuttle = _clean_token(reading.get('shuttle', ''))
            if shuttle and (shuttle, str(slot)) not in matches:
                matches.append((shuttle, str(slot)))
    return matches


def _payload_condition_from_command(command: dict[str, Any]) -> str:
    raw_condition = (
        command.get('payload_condition')
        or command.get('payload_state')
        or command.get('payload')
        or ''
    )
    if raw_condition == '' and 'loaded' in command:
        raw_condition = 'loaded' if bool(command.get('loaded')) else 'empty'
    text = _clean_token(raw_condition).casefold()
    if text in {'loaded', 'load', 'with_payload', 'with payload', 'carrying', 'part', 'box'}:
        return 'loaded'
    if text in {'empty', 'unloaded', 'without_payload', 'without payload', 'none', 'no_payload'}:
        return 'empty'
    return ''


def _payloads_for_side(rails: dict[str, Any], side: str) -> dict[str, Any]:
    rail = _rail_snapshot(rails, side)
    payloads = rail.get('payloads', {})
    return payloads if isinstance(payloads, dict) else {}


def _payload_loaded_for_shuttle(rails: dict[str, Any], side: str, shuttle_name: str) -> bool:
    payloads = _payloads_for_side(rails, side)
    entry = payloads.get(shuttle_name, {})
    if not isinstance(entry, dict):
        return False
    return bool(entry.get('loaded', False))


def _payload_condition_matches(
    rails: dict[str, Any],
    side: str,
    shuttle_name: str,
    condition: str,
) -> bool:
    if not condition:
        return True
    loaded = _payload_loaded_for_shuttle(rails, side, shuttle_name)
    return loaded if condition == 'loaded' else not loaded


def _source_shuttles_matching_payload(
    rails: dict[str, Any],
    slot_sensor_by_side: dict[str, dict[str, str]],
    side: str,
    source_slots: list[str],
    condition: str,
) -> list[tuple[str, str]]:
    return [
        (shuttle, slot)
        for shuttle, slot in _find_source_shuttles_for_slots(
            rails,
            slot_sensor_by_side,
            side,
            source_slots,
        )
        if _payload_condition_matches(rails, side, shuttle, condition)
    ]


def _mask_assignments_from_command(
    command: dict[str, Any],
    *,
    mask_prefix: str,
    value_prefix: str,
    value_by_id: dict[int, str],
) -> dict[str, Any]:
    assignments: dict[str, Any] = {}
    mask_map = command.get(mask_prefix)
    value_map = command.get(value_prefix)
    if isinstance(mask_map, dict) or isinstance(value_map, dict):
        mask_map = mask_map if isinstance(mask_map, dict) else {}
        value_map = value_map if isinstance(value_map, dict) else {}
        for name in SWITCHES:
            selected = str(mask_map.get(name, '')).lower() in {'1', 'true', 'yes', 'on'}
            if not selected:
                continue
            raw_value = value_map.get(name)
            if isinstance(raw_value, (int, float)):
                raw_value = value_by_id.get(int(raw_value), '')
            assignments[name] = raw_value
    for name in SWITCHES:
        selected = str(command.get(f'{mask_prefix}_{name}', '')).lower() in {
            '1',
            'true',
            'yes',
            'on',
        }
        if not selected:
            continue
        raw_value = command.get(f'{value_prefix}_{name}', '')
        if isinstance(raw_value, (int, float)):
            raw_value = value_by_id.get(int(raw_value), '')
        assignments[name] = raw_value
    return assignments


def _switch_assignments_for_safety(command: dict[str, Any]) -> dict[str, Any]:
    switches = command.get('switches')
    if isinstance(switches, dict):
        return dict(switches)
    if 'name' in command and 'state' in command:
        return {str(command['name']): command['state']}
    masked = _mask_assignments_from_command(
        command,
        mask_prefix='switch_mask',
        value_prefix='switch_value',
        value_by_id=SWITCH_VALUE_BY_ID,
    )
    if masked:
        return masked
    loop_state = _switch_state_for_loop(_normalize_loop(command.get('loop')))
    if loop_state:
        return {'ALL': loop_state}
    return {}


def _stopper_assignments_for_safety(command: dict[str, Any]) -> dict[str, Any]:
    stoppers = command.get('stoppers')
    if isinstance(stoppers, dict):
        return dict(stoppers)
    if 'name' in command and 'state' in command:
        return {str(command['name']): command['state']}
    return _mask_assignments_from_command(
        command,
        mask_prefix='stopper_mask',
        value_prefix='stopper_value',
        value_by_id=STOPPER_VALUE_BY_ID,
    )


def _canonical_switch_assignments(assignments: dict[str, Any]) -> tuple[dict[str, str], str]:
    canonical: dict[str, str] = {}
    for raw_name, raw_state in assignments.items():
        name = _clean_token(raw_name).upper()
        if name not in {'ALL', *SWITCHES}:
            return {}, f'invalid switch target {raw_name!r}; allowed A1-A4 or ALL'
        state = _canonical_switch_state(raw_state)
        if state not in {'EXTERIOR', 'INTERIOR'}:
            return {}, f'invalid switch state {raw_state!r}; allowed EXTERIOR/INTERIOR'
        canonical[name] = state
    return canonical, ''


def _canonical_stopper_assignments(assignments: dict[str, Any]) -> tuple[dict[str, str], str]:
    canonical: dict[str, str] = {}
    for raw_name, raw_state in assignments.items():
        name = _clean_token(raw_name).upper()
        if name not in {'ALL', *SWITCHES}:
            return {}, f'invalid stopper target {raw_name!r}; allowed A1-A4 or ALL'
        state = _normalize_stopper_state(raw_state)
        if state not in {'0', '1'}:
            return {}, f'invalid stopper state {raw_state!r}; allowed open/close or 0/1'
        canonical[name] = state
    return canonical, ''


def _expanded_device_assignments(assignments: dict[str, str]) -> dict[str, str]:
    expanded: dict[str, str] = {}
    for name, state in assignments.items():
        if name == 'ALL':
            for device_name in SWITCHES:
                expanded[device_name] = state
        elif name in SWITCHES:
            expanded[name] = state
    return expanded


def _is_all_switch_loop_transition(expanded: dict[str, str]) -> bool:
    if set(expanded) != set(SWITCHES):
        return False
    return len(set(expanded.values())) == 1


def _single_gate_switch_change_is_staged(
    expanded: dict[str, str],
    *,
    rails: dict[str, Any],
    side: str,
) -> bool:
    if len(expanded) != 1:
        return False
    switch_name = next(iter(expanded))
    if switch_name != _gate_for_side(side):
        return False
    if _rail_has_moving_shuttle(rails, side):
        return False
    return _rail_stopped_on_gate_approach(rails, side, switch_name)


def _rail_stopped_on_gate_approach(rails: dict[str, Any], side: str, gate: str) -> bool:
    approach_segments = {
        ('right', 'A3'): {'A23'},
        ('left', 'A3'): {'A23'},
    }.get((side, gate), set())
    for shuttle_state in _rail_shuttles(rails, side).values():
        if _shuttle_is_moving(shuttle_state) or _shuttle_is_falling(shuttle_state):
            continue
        segment = _clean_token(shuttle_state.get('segment', '')).upper()
        if segment in approach_segments:
            return True
    rail = _rail_snapshot(rails, side)
    active_names = _active_sensor_names(rail)
    gate_sensor = f'{gate}_STOPPER_SENSOR'
    return gate_sensor in active_names


def _switch_assignments_are_noop(
    rails: dict[str, Any],
    side: str,
    expanded: dict[str, str],
) -> bool:
    switches = _rail_snapshot(rails, side).get('switches', {})
    if not isinstance(switches, dict) or not expanded:
        return False
    for switch_name, desired_state in expanded.items():
        current_state = _canonical_switch_state(switches.get(switch_name))
        if current_state != desired_state:
            return False
    return True


def _gate_for_side(side: str) -> str:
    return {'right': 'A3', 'left': 'A3'}[side]


def _rail_stopped_at_gate(rails: dict[str, Any], side: str, gate: str) -> bool:
    for shuttle_state in _rail_shuttles(rails, side).values():
        if _shuttle_is_moving(shuttle_state) or _shuttle_is_falling(shuttle_state):
            continue
        if _shuttle_near_switch(shuttle_state, gate):
            return True
    active_names = _active_sensor_names(_rail_snapshot(rails, side))
    gate_sensors = {
        name.upper()
        for name in SWITCH_SENSOR_PREFIX_BY_SIDE.get(side, {}).get(gate, ())
    }
    return bool(active_names & gate_sensors)


def _valid_slot(raw: Any) -> bool:
    return str(raw).strip() in {'1', '2', '3', '4'}


def _round_index(raw: Any) -> int:
    try:
        return int(round(float(raw)))
    except (TypeError, ValueError):
        return 0


def _safe_int(raw: Any, default: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return default


def _is_action_vector(raw: Any) -> bool:
    if not isinstance(raw, list) or len(raw) != len(EVENT_ACTION_VECTOR_FIELDS):
        return False
    try:
        [float(value) for value in raw]
    except (TypeError, ValueError):
        return False
    return True


def _decode_event_action_vector(action_vector: Any) -> dict[str, Any]:
    values = [float(value) for value in list(action_vector)]
    if len(values) != len(EVENT_ACTION_VECTOR_FIELDS):
        raise ValueError(
            f'action_vector length {len(values)} does not match schema v3 length '
            f'{len(EVENT_ACTION_VECTOR_FIELDS)}'
        )
    return decode_action_v3(values)


def _selected_assignments_from_event_action(
    event_action: dict[str, Any],
    *,
    mask_key: str,
    value_key: str,
) -> dict[str, str]:
    mask = event_action.get(mask_key, {})
    values = event_action.get(value_key, {})
    if not isinstance(mask, dict) or not isinstance(values, dict):
        return {}
    assignments: dict[str, str] = {}
    for name in SWITCHES:
        selected = str(mask.get(name, '')).lower() in {'1', 'true', 'yes', 'on'}
        if selected:
            assignments[name] = _clean_token(values.get(name, ''))
    return assignments


def _event_action_to_ros_command(
    event_action: dict[str, Any],
    *,
    default_shuttle_name_by_side: dict[str, str],
) -> tuple[dict[str, Any] | None, str]:
    primitive = _clean_token(event_action.get('primitive', '')).upper()
    side = _strict_side(event_action.get('side', 'right')) or 'right'
    target_id = _clean_token(event_action.get('target_id', 'none'))
    wait_condition = _clean_token(event_action.get('wait_condition', 'none'))
    shuttle_name = _event_action_shuttle_name(
        event_action,
        side=side,
        default_shuttle_name_by_side=default_shuttle_name_by_side,
    )

    if primitive in {'WAIT', 'DONE'}:
        return {'action': 'status'}, ''
    if primitive == 'EMERGENCY_STOP':
        return {'action': 'emergency_stop'}, ''
    if primitive == 'STOP_NOW':
        return {
            'action': 'shuttle',
            'side': side,
            'shuttle': shuttle_name,
            'command': 'OFF',
        }, ''
    if primitive == 'SHUTTLE_ON':
        if wait_condition == 'none' or target_id == 'none':
            return None, 'unsafe shuttle ON: missing wait_condition or target_id'
        expected_target = f'{side}_shuttle'
        specific_targets = {
            f'{side}_shuttle_{index}'
            for index in range(1, 5)
        }
        if target_id not in {expected_target, 'MULTIPLE_DEVICES', *specific_targets}:
            return None, (
                f'unsafe shuttle ON: target_id {target_id!r} does not match '
                f'{expected_target!r}'
            )
        try:
            speed_mps = float(event_action.get('speed_mps', 0.0) or 0.0)
        except (TypeError, ValueError):
            speed_mps = 0.0
        if speed_mps <= 0.0:
            return None, 'unsafe shuttle ON: speed_mps must be > 0'
        return {
            'action': 'shuttle',
            'side': side,
            'shuttle': shuttle_name,
            'command': 'ON',
            'speed': speed_mps,
        }, ''
    if primitive == 'SET_SWITCHES':
        assignments = _selected_assignments_from_event_action(
            event_action,
            mask_key='switch_mask',
            value_key='switch_values',
        )
        if not assignments:
            return None, 'SET_SWITCHES needs at least one selected switch'
        return {'action': 'switches', 'side': side, 'switches': assignments}, ''
    if primitive == 'SET_STOPPERS':
        assignments = _selected_assignments_from_event_action(
            event_action,
            mask_key='stopper_mask',
            value_key='stopper_values',
        )
        if not assignments:
            return None, 'SET_STOPPERS needs at least one selected stopper'
        return {'action': 'stoppers', 'side': side, 'stoppers': assignments}, ''
    return None, f'unsupported primitive {primitive!r}'


def _event_action_shuttle_name(
    event_action: dict[str, Any],
    *,
    side: str,
    default_shuttle_name_by_side: dict[str, str],
) -> str:
    spec = normalize_shuttle_ref(
        event_action.get('shuttle_id') or event_action.get('target_id'),
        side=side,
    )
    if spec is None:
        try:
            shuttle_index = int(event_action.get('shuttle_index'))
        except (TypeError, ValueError):
            shuttle_index = -1
        if shuttle_index >= 0:
            return multi_gazebo_entity_name(side, shuttle_index + 1)
    if spec is not None:
        return spec.gazebo_entity_name
    return default_shuttle_name_by_side.get(side, f'room315_{side}_shuttle_1')


def _resolve_shuttle_command_name(raw: str, *, side: str) -> str:
    spec = normalize_shuttle_ref(raw, side=side)
    if spec is not None:
        return spec.gazebo_entity_name
    return _clean_token(raw)


def decode_and_validate(
    action_vector: Any,
    *,
    rails: dict[str, Any],
    emergency_stop: bool,
    active_tasks: dict[str, dict[str, Any]],
    slot_sensor_by_side: dict[str, dict[str, str]],
    default_shuttle_name_by_side: dict[str, str],
    block_reservations: dict[str, str] | None = None,
    station_slot_targets: dict[str, str] | None = None,
    min_headway_blocks: int = 1,
) -> dict[str, Any]:
    raw_action = action_vector
    try:
        event_action = _decode_event_action_vector(action_vector)
    except ValueError as exc:
        return _safety_decision(
            accepted=False,
            original_action=raw_action,
            reason=str(exc),
        )

    ros_command, reason = _event_action_to_ros_command(
        event_action,
        default_shuttle_name_by_side=default_shuttle_name_by_side,
    )
    if reason:
        return _safety_decision(
            accepted=False,
            original_action=raw_action,
            reason=reason,
        )
    if ros_command is None:
        return _safety_decision(
            accepted=False,
            original_action=raw_action,
            reason='action_vector did not decode to a ROS command',
        )

    primitive = event_action['primitive']
    side = event_action['side']
    if primitive == 'SET_SWITCHES':
        assignments, reason = _canonical_switch_assignments(ros_command.get('switches', {}))
        if reason:
            return _safety_decision(accepted=False, original_action=raw_action, reason=reason)
        expanded = _expanded_device_assignments(assignments)
        is_transition = _is_all_switch_loop_transition(expanded)
        assignments_are_noop = _switch_assignments_are_noop(rails, side, expanded)
        if is_transition and not assignments_are_noop:
            if _rail_has_moving_shuttle(rails, side):
                return _safety_decision(
                    accepted=False,
                    original_action=raw_action,
                    reason=f'unsafe loop transition: {side} shuttle must STOP before switching loop mode',
                )
            gate = _gate_for_side(side)
            if not _rail_stopped_at_gate(rails, side, gate):
                return _safety_decision(
                    accepted=False,
                    original_action=raw_action,
                    reason=(
                        f'unsafe loop transition: {side} rail must be staged at '
                        f'side-specific gate {gate} before switching loop mode'
                    ),
                )
        elif not assignments_are_noop:
            if not _single_gate_switch_change_is_staged(
                expanded,
                rails=rails,
                side=side,
            ):
                for switch_name in expanded:
                    reason = _occupied_guarded_segment_reason(rails, side, switch_name)
                    if reason:
                        return _safety_decision(
                            accepted=False,
                            original_action=raw_action,
                            reason=reason,
                        )

    decision = _decode_room315_vla_action(
        ros_command,
        rails=rails,
        emergency_stop=emergency_stop,
        active_tasks=active_tasks,
        slot_sensor_by_side=slot_sensor_by_side,
        default_shuttle_name_by_side=default_shuttle_name_by_side,
        block_reservations=block_reservations,
        station_slot_targets=station_slot_targets,
        min_headway_blocks=min_headway_blocks,
    )
    decision['raw_action'] = raw_action
    decision['decoded_action'] = event_action
    decision['executed_action'] = decision.get('corrected_action') if decision.get('accepted') else None
    decision['illegal_proposal'] = not bool(decision.get('accepted'))
    decision['rejected_action'] = None if decision.get('accepted') else raw_action
    return decision


def _decode_room315_vla_action(
    command: dict[str, Any],
    *,
    rails: dict[str, Any],
    emergency_stop: bool,
    active_tasks: dict[str, dict[str, Any]],
    slot_sensor_by_side: dict[str, dict[str, str]],
    default_shuttle_name_by_side: dict[str, str],
    block_reservations: dict[str, str] | None = None,
    station_slot_targets: dict[str, str] | None = None,
    min_headway_blocks: int = 1,
) -> dict[str, Any]:
    if not isinstance(command, dict):
        return _safety_decision(
            accepted=False,
            original_action=command,
            reason='model action must be a JSON object',
        )

    action = _normalize_safety_action(command.get('action') or command.get('intent') or command.get('type') or 'status')
    if action == 'snapshot':
        action = 'status'
    if action not in SAFETY_ACTIONS:
        return _safety_decision(
            accepted=False,
            original_action=command,
            reason=f'unknown command type {action!r}',
        )

    corrected = dict(command)
    corrected['action'] = action

    if action in {'status', 'clear_emergency_stop', 'emergency_stop', 'stop_all'}:
        return _safety_decision(accepted=True, original_action=command, corrected_action=corrected)

    if emergency_stop:
        return _safety_decision(
            accepted=False,
            original_action=command,
            reason='emergency stop is active; only status, stop_all, or clear_emergency_stop are allowed',
        )

    falling_reason = _any_falling_shuttle(rails)
    shuttle_command = _clean_token(command.get('command', '')).upper()
    if falling_reason and not (action == 'shuttle' and shuttle_command in {'OFF', 'RESET'}):
        return _safety_decision(
            accepted=False,
            original_action=command,
            reason=f'falling state rejection: {falling_reason}',
        )

    if action in {'switches', 'stoppers', 'shuttle', 'add_shuttle'}:
        side = _strict_side(command.get('side', 'right'))
        if side not in SIDES:
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'invalid side {command.get("side")!r}; allowed right/left',
            )
        corrected['side'] = side

    if action == 'switches':
        assignments, reason = _canonical_switch_assignments(_switch_assignments_for_safety(command))
        if reason:
            return _safety_decision(accepted=False, original_action=command, reason=reason)
        if not assignments:
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason='switch action needs switches, loop, or switch mask/value fields',
            )
        expanded = _expanded_device_assignments(assignments)
        side = corrected['side']
        assignments_are_noop = _switch_assignments_are_noop(rails, side, expanded)
        if (
            _is_all_switch_loop_transition(expanded)
            and not assignments_are_noop
            and _rail_has_moving_shuttle(rails, side)
        ):
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'unsafe loop transition: {side} shuttle must be staged/stopped before changing all switches',
            )
        if not assignments_are_noop:
            for switch_name in expanded:
                reason = _unsafe_switch_change_reason(rails, side, switch_name)
                if reason:
                    return _safety_decision(accepted=False, original_action=command, reason=reason)
        corrected['switches'] = assignments
        return _safety_decision(accepted=True, original_action=command, corrected_action=corrected)

    if action == 'stoppers':
        assignments, reason = _canonical_stopper_assignments(_stopper_assignments_for_safety(command))
        if reason:
            return _safety_decision(accepted=False, original_action=command, reason=reason)
        if not assignments:
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason='stopper action needs stoppers or stopper mask/value fields',
            )
        expanded = _expanded_device_assignments(assignments)
        side = corrected['side']
        if _rail_has_moving_shuttle(rails, side):
            for stopper_name, state in expanded.items():
                if state != '1':
                    continue
                reason = _unsafe_switch_change_reason(rails, side, stopper_name)
                if reason:
                    return _safety_decision(
                        accepted=False,
                        original_action=command,
                        reason=reason.replace('switch change', 'stopper close'),
                    )
        corrected['stoppers'] = assignments
        return _safety_decision(accepted=True, original_action=command, corrected_action=corrected)

    if action == 'shuttle':
        side = corrected['side']
        command_name = _clean_token(command.get('command', '')).upper()
        if command_name not in {'ON', 'OFF', 'RESET', 'REMOVE', 'ADD_STOPPED', 'ADD_MOVING'}:
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'invalid shuttle command {command_name!r}',
            )
        registry = ShuttleRegistry.from_rails(rails)
        fleet_state = fleet_safety_state_from_rails(
            rails,
            block_reservations=block_reservations,
            station_slot_targets=station_slot_targets,
            min_headway_blocks=min_headway_blocks,
        )
        fleet_ok, fleet_reason = validate_fleet_command(
            command,
            registry=registry,
            fleet_state=fleet_state,
        )
        if not fleet_ok:
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=fleet_reason,
            )
        shuttle_name = _clean_token(
            command.get('shuttle')
            or command.get('shuttle_id')
            or command.get('name')
            or default_shuttle_name_by_side.get(side, '')
        )
        shuttle_name = _resolve_shuttle_command_name(shuttle_name, side=side)
        if not shuttle_name:
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason='shuttle action needs shuttle/name',
            )
        if not _shuttle_name_matches_side(shuttle_name, side):
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'wrong side: shuttle {shuttle_name!r} does not belong on {side} rail',
            )
        if command_name in {'ON', 'OFF', 'RESET', 'REMOVE'} and not _shuttle_exists_on_side(rails, side, shuttle_name):
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'missing shuttle {shuttle_name!r} on {side} rail',
            )
        if command_name == 'ON':
            blocked = _closed_stoppers(rails, side)
            if blocked:
                target_stopper = _clean_token(
                    command.get('target_stopper')
                    or command.get('stopper_target')
                    or command.get('target')
                    or ''
                ).upper()
                if target_stopper not in blocked or len(blocked) != 1:
                    return _safety_decision(
                        accepted=False,
                        original_action=command,
                        reason=f'path blocked by closed stopper(s) on {side}: {", ".join(blocked)}',
                    )
                corrected['target_stopper'] = target_stopper
        if command.get('start_slot') is not None and not _valid_slot(command.get('start_slot')):
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'invalid start_slot {command.get("start_slot")!r}; allowed 1-4',
            )
        corrected['command'] = command_name
        corrected['shuttle'] = shuttle_name
        return _safety_decision(accepted=True, original_action=command, corrected_action=corrected)

    if action == 'add_shuttle':
        side = corrected['side']
        start_slot = str(command.get('start_slot', '2')).strip()
        if not _valid_slot(start_slot):
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'invalid start_slot {start_slot!r}; allowed 1-4',
            )
        moving = bool(command.get('moving', command.get('start', False)))
        if moving and _closed_stoppers(rails, side):
            return _safety_decision(
                accepted=False,
                original_action=command,
                reason=f'cannot add moving shuttle because {side} rail has closed stopper(s)',
            )
        corrected['start_slot'] = start_slot
        return _safety_decision(accepted=True, original_action=command, corrected_action=corrected)

    return _safety_decision(accepted=True, original_action=command, corrected_action=corrected)


def _named_states(assignments: dict[str, Any], state_normalizer=lambda value: _clean_token(value)):
    states = []
    for name, raw_state in assignments.items():
        item = NamedState()
        item.name = _clean_token(name).upper()
        item.state = state_normalizer(raw_state)
        states.append(item)
    return states


class Room315VlaSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('room_315_vla_supervisor')

        self.declare_parameter('config_path', str(_default_config_path()))
        self.declare_parameter('command_topic', '/room_315/vla/command')
        self.declare_parameter('status_topic', '/room_315/vla/status')
        self.declare_parameter('emergency_stop_topic', '/room_315/vla/emergency_stop')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('right_image_topic', '/room_315/vla/right_rail_rgbd/image')
        self.declare_parameter('left_image_topic', '/room_315/vla/left_rail_rgbd/image')
        self.declare_parameter('right_camera_info_topic', '/room_315/vla/right_rail_rgbd/camera_info')
        self.declare_parameter('left_camera_info_topic', '/room_315/vla/left_rail_rgbd/camera_info')
        self.declare_parameter('publish_status_period_s', 1.0)
        self.declare_parameter('completed_task_limit', 20)
        self.declare_parameter('safety_decision_log_limit', 20)

        raw_config_path = str(self.get_parameter('config_path').value).strip()
        config_path = Path(raw_config_path) if raw_config_path else _default_config_path()
        self.config = _load_yaml(config_path)
        self.defaults = self.config.get('defaults', {})
        if not isinstance(self.defaults, dict):
            self.defaults = {}

        self.slot_sensor_by_side = self._slot_sensor_map_from_config()

        self.emergency_stop = False
        self.last_command = ''
        self.last_result = 'initialized'
        self.last_primitive_command: dict[str, Any] | None = None
        self.last_image_time: float | None = None
        self.last_camera_info_time: float | None = None
        self.image_frame_count = 0
        self.camera_info_frame_id = ''
        self.camera_vision: dict[str, dict[str, Any]] = {}
        self.active_tasks: dict[str, dict[str, Any]] = {}
        self.completed_tasks: list[dict[str, Any]] = []
        self.task_counter = 0
        self.safety_metrics = _empty_safety_metrics()
        self.block_reservations: dict[str, str] = {}
        self.station_slot_targets: dict[str, str] = {}
        self.min_headway_blocks = int(self.defaults.get('min_headway_blocks', 1) or 1)
        self.last_safety_decision: dict[str, Any] | None = None
        self.safety_decisions: list[dict[str, Any]] = []
        self.safety_decision_log_limit = max(
            int(self.get_parameter('safety_decision_log_limit').value),
            1,
        )
        self.completed_task_limit = max(
            int(self.defaults.get(
                'completed_task_limit',
                self.get_parameter('completed_task_limit').value,
            )),
            1,
        )

        self.rails: dict[str, dict[str, Any]] = {
            side: {
                'shuttles': {},
                'switches': {},
                'stoppers': {},
                'payloads': {},
                'active_sensors': [],
                'active_position_sensors': [],
            }
            for side in SIDES
        }

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            str(self.get_parameter('command_topic').value),
            self._on_command,
            10,
        )
        self.estop_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            self._on_emergency_stop,
            10,
        )
        self.image_subs = []
        self.camera_info_subs = []
        self._subscribe_image_topic('legacy_primary', str(self.get_parameter('image_topic').value))
        self._subscribe_image_topic('right_rail_rgb', str(self.get_parameter('right_image_topic').value))
        self._subscribe_image_topic('left_rail_rgb', str(self.get_parameter('left_image_topic').value))
        self._subscribe_camera_info_topic(
            'legacy_primary',
            str(self.get_parameter('camera_info_topic').value),
        )
        self._subscribe_camera_info_topic(
            'right_rail_rgb',
            str(self.get_parameter('right_camera_info_topic').value),
        )
        self._subscribe_camera_info_topic(
            'left_rail_rgb',
            str(self.get_parameter('left_camera_info_topic').value),
        )

        self.shuttle_command_pubs: dict[str, Any] = {}
        self.shuttle_add_clients: dict[str, Any] = {}
        self.switch_pubs: dict[str, Any] = {}
        self.stopper_pubs: dict[str, Any] = {}

        for side in SIDES:
            prefix = f'/room_315/rails/{side}'
            self.shuttle_command_pubs[side] = self.create_publisher(
                ShuttleCommand,
                f'{prefix}/shuttles/command',
                10,
            )
            self.shuttle_add_clients[side] = self.create_client(
                AddShuttle,
                f'{prefix}/shuttles/add',
            )
            self.switch_pubs[side] = self.create_publisher(
                SwitchCommand,
                f'{prefix}/switches/command',
                10,
            )
            self.stopper_pubs[side] = self.create_publisher(
                StopperCommand,
                f'{prefix}/stoppers/command',
                10,
            )

            self.create_subscription(
                ShuttleState,
                f'{prefix}/shuttles/state',
                lambda msg, rail_side=side: self._on_shuttle_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                SwitchState,
                f'{prefix}/switches/state',
                lambda msg, rail_side=side: self._on_switch_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                StopperState,
                f'{prefix}/stoppers/state',
                lambda msg, rail_side=side: self._on_stopper_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                SensorFeedback,
                f'{prefix}/sensors/feedback',
                lambda msg, rail_side=side: self._on_sensor_feedback(rail_side, msg, 'active_sensors'),
                10,
            )
            self.create_subscription(
                SensorFeedback,
                f'{prefix}/sensors/position_feedback',
                lambda msg, rail_side=side: self._on_sensor_feedback(
                    rail_side,
                    msg,
                    'active_position_sensors',
                ),
                10,
            )
            self.create_subscription(
                String,
                f'{prefix}/shuttles/payload_state',
                lambda msg, rail_side=side: self._on_payload_state(rail_side, msg),
                10,
            )

        period_s = max(float(self.get_parameter('publish_status_period_s').value), 0.1)
        self.create_timer(period_s, self._on_status_timer)
        self.get_logger().info(
            f'Room 315 VLA supervisor ready. Command topic: '
            f'{self.get_parameter("command_topic").value}'
        )

    def _subscribe_image_topic(self, camera_name: str, topic: str) -> None:
        topic = topic.strip()
        if not topic:
            return
        self.image_subs.append(
            self.create_subscription(
                Image,
                topic,
                lambda msg, name=camera_name: self._on_image(name, msg),
                10,
            )
        )

    def _subscribe_camera_info_topic(self, camera_name: str, topic: str) -> None:
        topic = topic.strip()
        if not topic:
            return
        self.camera_info_subs.append(
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, name=camera_name: self._on_camera_info(name, msg),
                10,
            )
        )

    def _on_image(self, camera_name: str, _msg: Image) -> None:
        now = time.monotonic()
        self.last_image_time = now
        self.image_frame_count += 1
        camera = self.camera_vision.setdefault(camera_name, {'image_frames': 0})
        camera['image_frames'] = int(camera.get('image_frames', 0)) + 1
        camera['last_image_time'] = now

    def _on_camera_info(self, camera_name: str, msg: CameraInfo) -> None:
        now = time.monotonic()
        self.last_camera_info_time = now
        self.camera_info_frame_id = msg.header.frame_id
        camera = self.camera_vision.setdefault(camera_name, {'image_frames': 0})
        camera['last_camera_info_time'] = now
        camera['camera_info_frame_id'] = msg.header.frame_id

    def _on_shuttle_state(self, side: str, msg: ShuttleState) -> None:
        self.rails[side]['shuttles'][msg.name] = {
            'mode': msg.mode,
            'segment': msg.current_segment,
            's': round(float(msg.s), 4),
            'x': round(float(msg.x), 4),
            'y': round(float(msg.y), 4),
            'z': round(float(msg.z), 4),
            'yaw': round(float(msg.yaw), 4),
            'speed': round(float(msg.speed), 4),
        }

    def _on_payload_state(self, side: str, msg: String) -> None:
        try:
            parsed = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            return
        payloads = self._payload_entries_from_status_message(side, parsed)
        if payloads:
            self.rails[side]['payloads'] = payloads

    def _payload_entries_from_status_message(
        self,
        side: str,
        parsed: Any,
    ) -> dict[str, dict[str, Any]]:
        if isinstance(parsed, list):
            raw_entries = parsed
        elif isinstance(parsed, dict):
            raw_entries = parsed.get('shuttles', [])
            if not raw_entries and isinstance(parsed.get('by_shuttle'), dict):
                raw_entries = list(parsed['by_shuttle'].values())
        else:
            raw_entries = []
        if not isinstance(raw_entries, list):
            return {}

        entries: dict[str, dict[str, Any]] = {}
        for raw_entry in raw_entries:
            if not isinstance(raw_entry, dict):
                continue
            try:
                entry_side = _normalize_side(raw_entry.get('side', side))
            except ValueError:
                continue
            if entry_side != side:
                continue
            entity_name = _clean_token(
                raw_entry.get('entity_name')
                or raw_entry.get('name')
                or raw_entry.get('shuttle')
                or ''
            )
            if not entity_name:
                spec = normalize_shuttle_ref(raw_entry.get('shuttle_id'), side=side)
                entity_name = spec.gazebo_entity_name if spec is not None else ''
            if not entity_name:
                continue
            loaded = bool(raw_entry.get('loaded', False))
            spec = normalize_shuttle_ref(entity_name, side=side)
            try:
                fallback_index = int(raw_entry.get('shuttle_index', -1))
            except (TypeError, ValueError):
                fallback_index = -1
            entries[entity_name] = {
                'shuttle_id': (
                    spec.shuttle_id
                    if spec is not None
                    else str(raw_entry.get('shuttle_id') or entity_name)
                ),
                'short_id': (
                    spec.short_id
                    if spec is not None
                    else str(raw_entry.get('short_id') or '')
                ),
                'side': side,
                'shuttle_index': (
                    spec.shuttle_index
                    if spec is not None
                    else fallback_index
                ),
                'entity_name': entity_name,
                'loaded': loaded,
                'payload_type': (
                    str(raw_entry.get('payload_type') or 'box').strip()
                    if loaded
                    else 'none'
                ),
                'model_input_exposure': 'excluded',
            }
        return entries

    def _on_switch_state(self, side: str, msg: SwitchState) -> None:
        self.rails[side]['switches'] = {
            item.name: item.state
            for item in msg.switches
        }

    def _on_stopper_state(self, side: str, msg: StopperState) -> None:
        self.rails[side]['stoppers'] = {
            item.name: item.state
            for item in msg.stoppers
        }

    def _on_sensor_feedback(self, side: str, msg: SensorFeedback, key: str) -> None:
        active = []
        for reading in msg.readings:
            if reading.active:
                item = {
                    'name': reading.name,
                    'type': reading.sensor_type,
                    'shuttle': reading.shuttle_name,
                    'segment': reading.segment,
                    's': round(float(reading.s), 4),
                    's_ratio': round(float(reading.s_ratio), 4),
                }
                distance_m = getattr(reading, 'distance_m', None)
                if distance_m is not None:
                    item['distance_m'] = round(float(distance_m), 4)
                active.append(item)
        self.rails[side][key] = active


    def _on_emergency_stop(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self._stop_all(close_stoppers=True, reason='external emergency stop')
        else:
            self._set_result('emergency stop cleared')
        self._publish_status()

    def _on_command(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return
        self.last_command = raw
        try:
            command = self._parse_command(raw)
            decision = self._decode_and_record_safety(command)
            if not decision.get('accepted'):
                self._set_result(f'command rejected by safety decoder: {decision.get("reason", "")}')
                self.get_logger().warning(self.last_result)
                self._publish_status()
                return
            self._execute(decision.get('corrected_action'))
        except Exception as exc:
            self._set_result(f'command rejected: {exc}')
            self.get_logger().warning(self.last_result)
        self._publish_status()

    def _parse_command(self, raw: str) -> dict[str, Any] | list[Any]:
        if raw.startswith('{') or raw.startswith('['):
            return json.loads(raw)
        return self._parse_text_command(raw)

    def _parse_text_command(self, raw: str) -> dict[str, Any]:
        text = raw.casefold()
        side = self._infer_side(text)
        loop = self._infer_loop(text)

        if 'clear' in text and ('emergency' in text or 'estop' in text):
            return {'action': 'clear_emergency_stop'}
        if 'stop all' in text or 'all off' in text:
            return {'action': 'stop_all'}
        if 'emergency stop' in text or 'estop' in text:
            return {'action': 'emergency_stop'}



        switch_names = self._infer_switch_names(text)
        if 'switch' in text or switch_names:
            state = _switch_state_for_loop(loop)
            if state is None:
                raise ValueError('switch command needs exterior/interior state')
            names = switch_names or ['ALL']
            return {
                'action': 'switches',
                'side': side,
                'switches': {name: state for name in names},
            }

        if 'stopper' in text or 'stoppers' in text:
            names = switch_names or ['ALL']
            if 'close' in text or 'block' in text:
                state = '1'
            elif 'open' in text or 'release' in text:
                state = '0'
            else:
                raise ValueError('stopper command needs open/close state')
            return {
                'action': 'stoppers',
                'side': side,
                'stoppers': {name: state for name in names},
            }

        return {'action': 'status'}

    @staticmethod
    def _payload_condition_from_text(text: str) -> str:
        if any(token in text for token in ('loaded', 'carrying', 'with a part', 'with payload')):
            return 'loaded'
        if any(token in text for token in ('empty', 'unloaded', 'without payload', 'no payload')):
            return 'empty'
        return ''

    @staticmethod
    def _shuttle_ref_from_text(text: str) -> str:
        match = re.search(r'\b([rl][1-4])\b', text, re.IGNORECASE)
        if match:
            return match.group(1).upper()
        match = re.search(
            r'\b((?:right|left)_shuttle_?[1-4])\b',
            text,
            re.IGNORECASE,
        )
        return match.group(1) if match else ''

    @staticmethod
    def _side_from_shuttle_ref(shuttle_ref: str) -> str:
        text = str(shuttle_ref or '').casefold()
        if re.fullmatch(r'r[1-4]', text) or text.startswith('right'):
            return 'right'
        if re.fullmatch(r'l[1-4]', text) or text.startswith('left'):
            return 'left'
        return ''

    def _infer_side(self, text: str) -> str:
        if any(token in text for token in ('left', 'gauche')):
            return 'left'
        if any(token in text for token in ('right', 'droit', 'droite')):
            return 'right'
        return 'right'

    def _infer_loop(self, text: str) -> str | None:
        if any(token in text for token in ('interior', 'internal', 'petit', 'small')):
            return 'interior'
        if any(token in text for token in ('exterior', 'external', 'grand', 'large')):
            return 'exterior'
        return None

    def _infer_slot(self, text: str) -> str | None:
        match = re.search(r'(?:slot|start_slot|from)\s*_?\s*([1-4])', text)
        if match:
            return match.group(1)
        match = re.search(r'\b([1-4])\b', text)
        if match and 'slot' in text:
            return match.group(1)
        return None

    def _infer_switch_names(self, text: str) -> list[str]:
        names = sorted({match.group(0).upper() for match in re.finditer(r'\bA[1-4]\b', text, re.IGNORECASE)})
        if 'all' in text:
            return ['ALL']
        return names

    def _default_shuttle_names_by_side(self) -> dict[str, str]:
        return {side: self._default_shuttle_name(side) for side in SIDES}

    def _decode_and_record_safety(self, command: dict[str, Any] | list[Any]) -> dict[str, Any]:
        if _is_action_vector(command):
            decision = self.decode_and_validate(command)
            self._record_safety_decision(decision)
            return decision
        if isinstance(command, dict) and 'action_vector' in command:
            target_stopper = (
                command.get('target_stopper')
                or command.get('stopper_target')
                or command.get('target')
            )
            if (
                str(command.get('action') or '').casefold() == 'shuttle'
                and str(command.get('command') or '').upper() == 'ON'
                and target_stopper
            ):
                command_for_safety = dict(command)
                raw_action = command_for_safety.pop('action_vector')
                decision = self._safety_decode_command(command_for_safety)
                decision['raw_action'] = raw_action
                decision['executed_action'] = (
                    decision.get('corrected_action') if decision.get('accepted') else None
                )
                decision['illegal_proposal'] = not bool(decision.get('accepted'))
            else:
                decision = self.decode_and_validate(command.get('action_vector'))
                decision['raw_action'] = command.get('action_vector')
            decision['original_action'] = command
            if not decision.get('accepted'):
                decision['rejected_action'] = command.get('action_vector')
            self._record_safety_decision(decision)
            return decision
        if isinstance(command, list):
            corrected_items = []
            for item in command:
                decision = self._safety_decode_command(item)
                self._record_safety_decision(decision)
                if not decision.get('accepted'):
                    return {
                        'accepted': False,
                        'reason': decision.get('reason', 'list item rejected'),
                        'original_action': command,
                        'corrected_action': None,
                        'safe_correction': False,
                    }
                corrected_items.append(decision.get('corrected_action'))
            aggregate = {
                'accepted': True,
                'reason': '',
                'original_action': command,
                'corrected_action': corrected_items,
                'safe_correction': corrected_items != command,
                'raw_action': command,
                'illegal_proposal': False,
                'rejected_action': None,
                'executed_action': corrected_items,
            }
            self.last_safety_decision = aggregate
            return aggregate

        decision = self._safety_decode_command(command)
        self._record_safety_decision(decision)
        return decision

    def decode_and_validate(self, action_vector: Any) -> dict[str, Any]:
        return decode_and_validate(
            action_vector,
            rails=self.rails,
            emergency_stop=self.emergency_stop,
            active_tasks=self.active_tasks,
            slot_sensor_by_side=self.slot_sensor_by_side,
            default_shuttle_name_by_side=self._default_shuttle_names_by_side(),
            block_reservations=getattr(self, 'block_reservations', {}),
            station_slot_targets=getattr(self, 'station_slot_targets', {}),
            min_headway_blocks=getattr(self, 'min_headway_blocks', 1),
        )

    def _safety_decode_command(self, command: Any) -> dict[str, Any]:
        if _is_action_vector(command):
            return self.decode_and_validate(command)
        if isinstance(command, dict) and 'action_vector' in command:
            return self.decode_and_validate(command.get('action_vector'))
        return _decode_room315_vla_action(
            command,
            rails=self.rails,
            emergency_stop=self.emergency_stop,
            active_tasks=self.active_tasks,
            slot_sensor_by_side=self.slot_sensor_by_side,
            default_shuttle_name_by_side=self._default_shuttle_names_by_side(),
            block_reservations=getattr(self, 'block_reservations', {}),
            station_slot_targets=getattr(self, 'station_slot_targets', {}),
            min_headway_blocks=getattr(self, 'min_headway_blocks', 1),
        )

    def _record_safety_decision(self, decision: dict[str, Any]) -> None:
        now = time.monotonic()
        entry = dict(decision)
        entry['time_s'] = round(now, 6)
        accepted = bool(entry.get('accepted'))
        self.safety_metrics['total_proposed_actions'] += 1
        if accepted:
            self.safety_metrics['accepted_actions'] += 1
            self._update_runtime_fleet_safety(entry)
        else:
            self.safety_metrics['rejected_actions'] += 1
            reason = str(entry.get('reason') or 'unknown')
            reasons = self.safety_metrics.setdefault('rejection_reasons', {})
            reasons[reason] = int(reasons.get(reason, 0)) + 1
            reason_text = reason.casefold()
            if 'ambiguous' in reason_text or 'wrong shuttle' in reason_text:
                wrong = int(self.safety_metrics.get('wrong_shuttle_command_count') or 0) + 1
                self.safety_metrics['wrong_shuttle_command_count'] = wrong
            if 'occupied' in reason_text:
                self.safety_metrics['block_occupancy_violation_count'] = (
                    int(self.safety_metrics.get('block_occupancy_violation_count') or 0) + 1
                )
            if 'reserved' in reason_text:
                self.safety_metrics['block_reservation_rejection_count'] = (
                    int(self.safety_metrics.get('block_reservation_rejection_count') or 0) + 1
                )
            if 'headway' in reason_text:
                self.safety_metrics['headway_violation_count'] = (
                    int(self.safety_metrics.get('headway_violation_count') or 0) + 1
                )
            if 'deadlock' in reason_text:
                self.safety_metrics['deadlock_detected_count'] = (
                    int(self.safety_metrics.get('deadlock_detected_count') or 0) + 1
                )
        total = int(self.safety_metrics.get('total_proposed_actions') or 0)
        rejected = int(self.safety_metrics.get('rejected_actions') or 0)
        self.safety_metrics['illegal_proposal_rate'] = 0.0 if total == 0 else round(rejected / total, 4)
        self.safety_metrics['rejected_action_rate'] = 0.0 if total == 0 else round(rejected / total, 4)
        wrong = int(self.safety_metrics.get('wrong_shuttle_command_count') or 0)
        self.safety_metrics['wrong_shuttle_command_rate'] = (
            0.0 if total == 0 else round(wrong / total, 4)
        )
        self.last_safety_decision = entry
        self.safety_decisions.append(entry)
        if len(self.safety_decisions) > self.safety_decision_log_limit:
            self.safety_decisions = self.safety_decisions[-self.safety_decision_log_limit:]

    def _update_runtime_fleet_safety(self, decision: dict[str, Any]) -> None:
        corrected = decision.get('corrected_action')
        if isinstance(corrected, list):
            for item in corrected:
                self._update_runtime_fleet_safety({'accepted': True, 'corrected_action': item})
            return
        if not isinstance(corrected, dict):
            return
        action = _normalize_safety_action(corrected.get('action'))
        if action != 'shuttle':
            return
        side = _strict_side(corrected.get('side', 'right')) or 'right'
        spec = normalize_shuttle_ref(
            corrected.get('shuttle') or corrected.get('shuttle_id') or corrected.get('name'),
            side=side,
        )
        if spec is None:
            return
        owner = spec.short_id
        command_name = _clean_token(corrected.get('command', '')).upper()
        if command_name == 'ON':
            next_block = normalize_fleet_block_id(
                corrected.get('next_block') or corrected.get('target_block') or '',
                side=side,
            )
            if next_block:
                previous_owner = self.block_reservations.get(next_block)
                self.block_reservations[next_block] = owner
                if previous_owner != owner:
                    self.safety_metrics['block_reservation_success_count'] = (
                        int(self.safety_metrics.get('block_reservation_success_count') or 0) + 1
                    )
            target_slot = normalize_fleet_slot_id(
                corrected.get('target_slot') or corrected.get('slot') or '',
                side=side,
            )
            if target_slot:
                self.station_slot_targets[target_slot] = owner
            return
        if command_name in {'OFF', 'RESET', 'REMOVE'}:
            self._release_runtime_fleet_owner(owner)

    def _release_runtime_fleet_owner(self, owner: str) -> None:
        self.block_reservations = {
            block: reserved_owner
            for block, reserved_owner in self.block_reservations.items()
            if reserved_owner != owner
        }
        self.station_slot_targets = {
            slot: reserved_owner
            for slot, reserved_owner in self.station_slot_targets.items()
            if reserved_owner != owner
        }

    def _execute(self, command: dict[str, Any] | list[Any]) -> None:
        if isinstance(command, list):
            for item in command:
                if not isinstance(item, dict):
                    raise ValueError('list commands must contain objects')
                self._execute(item)
            return

        action = str(command.get('action') or command.get('intent') or command.get('type') or 'status')
        action = action.lower()

        if action in {'status', 'snapshot'}:
            self._set_result('status requested')
            return
        if action in {'clear_emergency_stop', 'clear_estop', 'reset_estop'}:
            self.emergency_stop = False
            self._set_result('emergency stop cleared')
            return
        if action in {'emergency_stop', 'estop'}:
            self.emergency_stop = True
            self._stop_all(close_stoppers=True, reason='commanded emergency stop')
            return
        if action in {'stop_all', 'all_off'}:
            self._stop_all(close_stoppers=bool(command.get('close_stoppers', False)), reason='stop all command')
            return

        if self.emergency_stop:
            raise RuntimeError('emergency stop is active; clear it before motion commands')

        if action in {'add_shuttle', 'spawn_shuttle'}:
            self._execute_add_shuttle(command)
            return
        if action in {'shuttle', 'shuttle_command'}:
            self._execute_shuttle_command(command)
            return
        if action in {'switches', 'switch'}:
            self._execute_switches(command)
            return
        if action in {'stoppers', 'stopper'}:
            self._execute_stoppers(command)
            return
        raise ValueError(f'unknown VLA action {action!r}')

    def _execute_add_shuttle(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        moving = bool(command.get('moving', command.get('start', False)))
        self._request_add_shuttle(
            side,
            command.get('shuttle') or command.get('name') or self._default_shuttle_name(side),
            start_slot=str(command.get('start_slot', '2')),
            speed=float(command.get('speed', self._default_speed())),
            start_enabled=moving,
        )
        self._set_result(f'added shuttle on {side}')

    def _execute_shuttle_command(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        shuttle_command = str(command.get('command', 'ON')).upper()
        if shuttle_command in {'ADD_MOVING', 'ADD_STOPPED'}:
            self._request_add_shuttle(
                side,
                command.get('shuttle') or command.get('name') or self._default_shuttle_name(side),
                start_slot=str(command.get('start_slot', '2')),
                speed=float(command.get('speed', self._default_speed())),
                start_enabled=shuttle_command == 'ADD_MOVING',
            )
            self._set_result(f'add shuttle request sent on {side}')
            return
        self._publish_shuttle_command(
            side,
            command.get('shuttle') or command.get('name') or self._default_shuttle_name(side),
            shuttle_command,
            start_slot=str(command.get('start_slot', '')),
            speed=float(command.get('speed', self._default_speed())),
        )
        self._set_result(f'shuttle command sent on {side}')

    def _execute_switches(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        loop = _normalize_loop(command.get('loop'))
        switches = self._switch_assignments_from_command(command, loop)
        if not switches:
            raise ValueError('switch action needs "switches" or "loop"')
        self._publish_switches(side, switches)
        self._set_result(f'switches commanded on {side}: {switches}')

    def _execute_stoppers(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        stoppers = dict(command.get('stoppers') or {})
        if not stoppers and 'name' in command and 'state' in command:
            stoppers = {str(command['name']): command['state']}
        if not stoppers:
            raise ValueError('stopper action needs "stoppers"')
        self._publish_stoppers(side, stoppers)
        self._set_result(f'stoppers commanded on {side}: {stoppers}')

    def _slot_sensor_map_from_config(self) -> dict[str, dict[str, str]]:
        mapping = {
            side: dict(DEFAULT_SLOT_SENSOR_BY_SIDE[side])
            for side in SIDES
        }
        configured = self.defaults.get('slot_sensor_by_side')
        if not isinstance(configured, dict):
            return mapping
        for side in SIDES:
            side_mapping = configured.get(side)
            if not isinstance(side_mapping, dict):
                continue
            for slot, sensor_name in side_mapping.items():
                slot_name = str(slot).strip()
                if slot_name in {'1', '2', '3', '4'}:
                    mapping[side][slot_name] = str(sensor_name).strip()
        return mapping

    def _normalize_slots(self, raw_slots: Any) -> list[str]:
        if raw_slots is None:
            return []
        if isinstance(raw_slots, str):
            values = re.split(r'[\s,]+', raw_slots.strip())
        else:
            try:
                values = list(raw_slots)
            except TypeError:
                values = [raw_slots]
        slots = []
        for value in values:
            slot = str(value).strip()
            if slot.startswith('slot_'):
                slot = slot.rsplit('_', 1)[-1]
            if slot in {'1', '2', '3', '4'} and slot not in slots:
                slots.append(slot)
        return slots

    def _normalize_segments(self, raw_segments: Any) -> list[str]:
        if raw_segments is None:
            return []
        if isinstance(raw_segments, str):
            values = re.split(r'[\s,]+', raw_segments.strip())
        else:
            try:
                values = list(raw_segments)
            except TypeError:
                values = [raw_segments]
        segments = []
        for value in values:
            segment = str(value).strip().upper()
            if segment and segment not in segments:
                segments.append(segment)
        return segments

    def _normalize_names(self, raw_names: Any) -> list[str]:
        if raw_names is None:
            return []
        if isinstance(raw_names, str):
            values = re.split(r'[\s,]+', raw_names.strip())
        else:
            try:
                values = list(raw_names)
            except TypeError:
                values = [raw_names]
        names = []
        for value in values:
            name = str(value).strip()
            if name and name not in names:
                names.append(name)
        return names

    def _slot_sensor_name(self, side: str, slot: str) -> str:
        return self.slot_sensor_by_side.get(side, {}).get(str(slot), '')

    def _sensor_names_for_slots(self, side: str, slots: list[str]) -> list[str]:
        return [
            sensor_name
            for sensor_name in (self._slot_sensor_name(side, slot) for slot in slots)
            if sensor_name
        ]

    def _find_shuttle_in_slots(self, side: str, slots: list[str]) -> tuple[str, str, str]:
        for slot in slots:
            sensor_name = self._slot_sensor_name(side, slot)
            if not sensor_name:
                continue
            reading = self._active_sensor_reading(side, sensor_name)
            if not reading:
                continue
            shuttle_name = str(reading.get('shuttle') or '').strip()
            if shuttle_name:
                return shuttle_name, slot, sensor_name
        return '', '', ''

    def _find_specific_shuttle_in_slots(
        self,
        side: str,
        slots: list[str],
        shuttle_name: str,
    ) -> tuple[str, str, str]:
        name = str(shuttle_name or '').strip()
        if not name:
            return '', '', ''
        for slot in slots:
            sensor_name = self._slot_sensor_name(side, slot)
            if not sensor_name:
                continue
            reading = self._active_sensor_reading(side, sensor_name, name)
            if reading:
                return name, slot, sensor_name
        return '', '', ''

    def _active_slot_for_shuttle(
        self,
        side: str,
        slots: list[str],
        shuttle_name: str,
    ) -> tuple[str, str]:
        for slot in slots:
            sensor_name = self._slot_sensor_name(side, slot)
            if not sensor_name:
                continue
            if self._active_sensor_reading(side, sensor_name, shuttle_name):
                return slot, sensor_name
        return '', ''

    def _active_named_sensor_for_shuttle(
        self,
        side: str,
        sensor_names: list[str],
        shuttle_name: str,
    ) -> str:
        for sensor_name in sensor_names:
            if self._active_sensor_reading(side, sensor_name, shuttle_name):
                return sensor_name
        return ''

    def _active_sensor_reading(
        self,
        side: str,
        sensor_name: str,
        shuttle_name: str = '',
    ) -> dict[str, Any] | None:
        wanted = sensor_name.strip().casefold()
        for key in ('active_position_sensors', 'active_sensors'):
            for reading in self.rails.get(side, {}).get(key, []) or []:
                if str(reading.get('name') or '').casefold() != wanted:
                    continue
                reading_shuttle = str(reading.get('shuttle') or '').strip()
                if shuttle_name and reading_shuttle != shuttle_name:
                    continue
                return reading
        return None

    def _shuttle_state(self, side: str, shuttle_name: str) -> dict[str, Any]:
        shuttles = self.rails.get(side, {}).get('shuttles', {}) or {}
        if shuttle_name and shuttle_name in shuttles:
            return shuttles[shuttle_name]
        if shuttles:
            return next(iter(shuttles.values()))
        return {}

    def _shuttle_segment(self, side: str, shuttle_name: str) -> str:
        return str(self._shuttle_state(side, shuttle_name).get('segment') or '').strip().upper()

    def _shuttle_mode(self, side: str, shuttle_name: str) -> str:
        return str(self._shuttle_state(side, shuttle_name).get('mode') or '').strip().upper()

    def _shuttle_waiting_at_stopper(self, side: str, shuttle_name: str, stopper: str) -> bool:
        sensor_name = STOPPER_SENSOR_BY_STOPPER.get(stopper, '')
        if sensor_name and self._active_sensor_reading(side, sensor_name, shuttle_name):
            return True
        return self._shuttle_mode(side, shuttle_name) == 'WAITING'

    def _switches_match(self, side: str, assignments: dict[str, Any]) -> bool:
        expected = self._expanded_switch_assignments(assignments)
        actual = self.rails.get(side, {}).get('switches', {}) or {}
        return all(
            _canonical_switch_state(actual.get(name)) == expected_state
            for name, expected_state in expected.items()
        )

    def _stoppers_match(self, side: str, assignments: dict[str, Any]) -> bool:
        expected = self._expanded_stopper_assignments(assignments)
        actual = self.rails.get(side, {}).get('stoppers', {}) or {}
        return all(
            _normalize_stopper_state(actual.get(name, '')) == expected_state
            for name, expected_state in expected.items()
        )

    def _expanded_switch_assignments(self, assignments: dict[str, Any]) -> dict[str, str]:
        expanded: dict[str, str] = {}
        for raw_name, raw_state in assignments.items():
            name = str(raw_name).strip().upper()
            state = _canonical_switch_state(raw_state)
            if name == 'ALL':
                for switch_name in SWITCHES:
                    expanded[switch_name] = state
            elif name in SWITCHES:
                expanded[name] = state
        return expanded

    def _expanded_stopper_assignments(self, assignments: dict[str, Any]) -> dict[str, str]:
        expanded: dict[str, str] = {}
        for raw_name, raw_state in assignments.items():
            name = str(raw_name).strip().upper()
            state = _normalize_stopper_state(raw_state)
            if name == 'ALL':
                for stopper_name in SWITCHES:
                    expanded[stopper_name] = state
            elif name in SWITCHES:
                expanded[name] = state
        return expanded

    def _on_status_timer(self) -> None:
        self._publish_status()



    def _switch_assignments_from_command(
        self,
        command: dict[str, Any],
        loop: str | None,
    ) -> dict[str, Any]:
        switches = dict(command.get('switches') or {})
        if not switches and 'name' in command and 'state' in command:
            switches = {str(command['name']): command['state']}
        loop_state = _switch_state_for_loop(loop)
        if not switches and loop_state:
            switches = {'ALL': loop_state}
        return switches

    def _publish_switches(
        self,
        side: str,
        assignments: dict[str, Any],
        *,
        task_id: str = '',
    ) -> None:
        msg = SwitchCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.switches = _named_states(assignments)
        self.switch_pubs[side].publish(msg)
        self._record_primitive_command(task_id, 'switches', side, {'switches': assignments})

    def _publish_stoppers(
        self,
        side: str,
        assignments: dict[str, Any],
        *,
        task_id: str = '',
    ) -> None:
        msg = StopperCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stoppers = _named_states(assignments, _normalize_stopper_state)
        self.stopper_pubs[side].publish(msg)
        self._record_primitive_command(task_id, 'stoppers', side, {'stoppers': assignments})

    def _publish_shuttle_command(
        self,
        side: str,
        name: Any,
        command: str,
        *,
        start_slot: str = '',
        speed: float | None = None,
        task_id: str = '',
    ) -> None:
        msg = ShuttleCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = _clean_token(name)
        msg.command = command.upper()
        msg.start_slot = start_slot
        msg.speed = float(self._default_speed() if speed is None else speed)
        self.shuttle_command_pubs[side].publish(msg)
        payload = {
            'shuttle': msg.name,
            'command': msg.command,
            'start_slot': msg.start_slot,
            'speed': round(float(msg.speed), 4),
        }
        self._record_primitive_command(task_id, 'shuttle', side, payload)

    def _record_primitive_command(
        self,
        task_id: str,
        action: str,
        side: str,
        payload: dict[str, Any],
    ) -> None:
        entry = {
            'time_s': round(time.monotonic(), 6),
            'task_id': task_id,
            'action': action,
            'side': side,
            **payload,
        }
        self.last_primitive_command = entry
        if task_id and task_id in self.active_tasks:
            primitives = self.active_tasks[task_id].setdefault('primitive_commands', [])
            primitives.append(entry)

    def _request_add_shuttle(
        self,
        side: str,
        name: Any,
        *,
        start_slot: str,
        speed: float,
        start_enabled: bool,
    ) -> None:
        client = self.shuttle_add_clients[side]
        if not client.service_is_ready():
            self.get_logger().warning(
                f'AddShuttle service for {side} rail is not ready yet; request queued by ROS.'
            )
        request = AddShuttle.Request()
        request.name = _clean_token(name)
        request.start_slot = _clean_token(start_slot)
        request.speed = float(speed)
        request.start_enabled = bool(start_enabled)
        future = client.call_async(request)
        future.add_done_callback(
            lambda result, rail_side=side: self._on_add_shuttle_response(rail_side, result)
        )

    def _on_add_shuttle_response(self, side: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'AddShuttle service call failed on {side}: {exc}')
            return
        if response.success:
            self.get_logger().info(f'AddShuttle succeeded on {side}: {response.message}')
        else:
            self.get_logger().error(f'AddShuttle rejected on {side}: {response.message}')

    def _stop_all(self, close_stoppers: bool, reason: str) -> None:
        for side in SIDES:
            self._publish_shuttle_command(side, 'ALL', 'OFF')
            if close_stoppers:
                self._publish_stoppers(side, {'ALL': '1'})
        self.active_tasks.clear()
        self._set_result(f'{reason}: all shuttles OFF')

    def _default_speed(self) -> float:
        return float(self.defaults.get('speed', 0.2))

    def _default_shuttle_name(self, side: str) -> str:
        names = self.defaults.get('shuttle_name_by_side', {})
        if isinstance(names, dict) and side in names:
            return str(names[side])
        return f'room315_{side}_shuttle_1'

    def _set_result(self, result: str) -> None:
        self.last_result = result
        self.get_logger().info(result)

    def _task_status_snapshot(self, task: dict[str, Any]) -> dict[str, Any]:
        now = time.monotonic()
        started_s = float(task.get('started_s', now))
        updated_s = float(task.get('updated_s', now))
        return {
            'task_id': str(task.get('task_id') or ''),
            'template': str(task.get('template') or ''),
            'type': str(task.get('type') or ''),
            'side': str(task.get('side') or ''),
            'status': str(task.get('status') or ''),
            'phase': str(task.get('phase') or ''),
            'duration_s': round(updated_s - started_s, 3)
            if str(task.get('status')) in TASK_TERMINAL_STATES
            else round(now - started_s, 3),
            'failure_reason': str(task.get('failure_reason') or ''),
            'shuttle': str(task.get('shuttle') or ''),
            'source_slots': list(task.get('source_slots') or []),
            'source_slot': str(task.get('source_slot') or ''),
            'target_slots': list(task.get('target_slots') or []),
            'target_slot': str(task.get('target_slot') or ''),
            'target_sensor': str(task.get('target_sensor') or ''),
            'primitive_commands': list(task.get('primitive_commands') or []),
        }

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._snapshot(), sort_keys=True)
        self.status_pub.publish(msg)



    def _snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        fleet_state = fleet_safety_state_from_rails(
            self.rails,
            block_reservations=getattr(self, 'block_reservations', {}),
            station_slot_targets=getattr(self, 'station_slot_targets', {}),
            min_headway_blocks=getattr(self, 'min_headway_blocks', 1),
        )
        image_age = None if self.last_image_time is None else round(now - self.last_image_time, 3)
        camera_info_age = (
            None
            if self.last_camera_info_time is None
            else round(now - self.last_camera_info_time, 3)
        )
        cameras = {}
        for camera_name, camera in self.camera_vision.items():
            last_image_time = camera.get('last_image_time')
            last_camera_info_time = camera.get('last_camera_info_time')
            cameras[camera_name] = {
                'image_frames': int(camera.get('image_frames', 0)),
                'last_image_age_s': (
                    None if last_image_time is None else round(now - float(last_image_time), 3)
                ),
                'last_camera_info_age_s': (
                    None
                    if last_camera_info_time is None
                    else round(now - float(last_camera_info_time), 3)
                ),
                'camera_info_frame_id': str(camera.get('camera_info_frame_id', '')),
            }
        return {
            'emergency_stop': self.emergency_stop,
            'last_command': self.last_command,
            'last_result': self.last_result,
            'last_primitive_command': self.last_primitive_command,
            'active_tasks': {
                task_id: self._task_status_snapshot(task)
                for task_id, task in self.active_tasks.items()
            },
            'completed_tasks': list(self.completed_tasks),
            'safety_decoder': {
                'metrics': self.safety_metrics,
                'last_decision': self.last_safety_decision,
                'recent_decisions': list(self.safety_decisions),
                'fleet_state': {
                    'block_occupancy': fleet_state.block_occupancy,
                    'block_reservations': dict(getattr(self, 'block_reservations', {})),
                    'station_slot_targets': dict(getattr(self, 'station_slot_targets', {})),
                    'shuttle_blocks': fleet_state.shuttle_blocks,
                    'min_headway_blocks': getattr(self, 'min_headway_blocks', 1),
                    'model_input_exposure': 'excluded',
                },
            },
            'safety_decoder_metrics': self.safety_metrics,
            'payload_state': self._payload_state_snapshot(),
            'vision': {
                'image_frames': self.image_frame_count,
                'last_image_age_s': image_age,
                'last_camera_info_age_s': camera_info_age,
                'camera_info_frame_id': self.camera_info_frame_id,
                'cameras': cameras,
            },
            'rails': self.rails,
        }

    def _payload_state_snapshot(self) -> dict[str, Any]:
        shuttles = []
        by_shuttle: dict[str, dict[str, Any]] = {}
        for side in SIDES:
            for entity_name, entry in (self.rails.get(side, {}).get('payloads', {}) or {}).items():
                if not isinstance(entry, dict):
                    continue
                item = {
                    'shuttle_id': str(entry.get('shuttle_id') or entity_name),
                    'short_id': str(entry.get('short_id') or ''),
                    'side': side,
                    'shuttle_index': _safe_int(entry.get('shuttle_index'), -1),
                    'entity_name': entity_name,
                    'loaded': bool(entry.get('loaded', False)),
                    'payload_type': (
                        str(entry.get('payload_type') or 'box')
                        if bool(entry.get('loaded', False))
                        else 'none'
                    ),
                    'model_input_exposure': 'excluded',
                }
                shuttles.append(item)
                by_shuttle[entity_name] = item
        return {
            'shuttles': sorted(shuttles, key=lambda item: (item['side'], item['shuttle_index'])),
            'by_shuttle': by_shuttle,
            'model_input_exposure': 'excluded',
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Room315VlaSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
