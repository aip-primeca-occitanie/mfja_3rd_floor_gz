#!/usr/bin/env python3

import heapq
import json
import math
import re
import time
from functools import lru_cache
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

from room_315_multi_shuttle import ShuttleRegistry
from room_315_multi_shuttle import empty_fleet_safety_metrics
from room_315_multi_shuttle import fleet_safety_state_from_rails
from room_315_multi_shuttle import normalize_fleet_block_id
from room_315_multi_shuttle import normalize_fleet_slot_id
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_multi_shuttle import validate_fleet_command
from room_315_pddl_scenario_generator import INTERIOR_HOLDING_BRANCH_BY_GATE
from room_315_pddl_scenario_generator import INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE
from room_315_pddl_scenario_generator import SLOT_SENSOR_BY_SIDE_AND_SLOT
from room_315_rail_defaults import LEFT_PUBLIC_SEGMENT_NAME_MAP
from room_315_rail_defaults import default_rail_network_path
from room_315_runtime_contracts import CONTROLLER_DISABLED_MODE
from room_315_runtime_contracts import normalize_runtime_clearance_certificate


SIDES = ('right', 'left')
SWITCHES = ('A1', 'A2', 'A3', 'A4')
STOPPER_SENSOR_BY_STOPPER = {
    'A1': 'A1_STOPPER_SENSOR',
    'A2': 'A2_STOPPER_SENSOR',
    'A3': 'A3_STOPPER_SENSOR',
    'A4': 'A4_STOPPER_SENSOR',
}
TASK_TERMINAL_STATES = {'succeeded', 'failed'}
RECOVERABLE_SAFETY_KEYWORDS = (
    'stale',
    'conflicting',
    'unknown localization',
    'sensor dropout',
    'timeout',
    'obstacle',
    'occupied',
    'reserved',
    'headway',
    'deadlock',
)
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
CONTROLLER_S_RANGE_TOLERANCE_M = 0.001
ROUTE_NORMALIZATION_ACTION_BY_MODE = {
    'restore_normal_route_before_slot_motion': 'restore_normal_route',
    'restore_normal_route_after_interior_clearance': 'finish_route_clearance',
    'pause_clearance_after_interior_capacity_exhausted': 'pause_route_clearance',
}
GUARDED_ROUTE_RECONFIGURATION_ACTION_BY_MODE = {
    'begin_route_clearance_hold_interior': 'begin_route_clearance',
}
GUARDED_SWITCH_PROOF_ACTION_BY_MODE = {
    **ROUTE_NORMALIZATION_ACTION_BY_MODE,
    **GUARDED_ROUTE_RECONFIGURATION_ACTION_BY_MODE,
}
ROUTE_NORMALIZATION_SYMBOLIC_ACTION_ALIASES = {
    # Segment-origin clearance has the same supervised physical restoration
    # macro as exact-slot clearance, while retaining distinct PDDL semantics.
    'finish_segment_route_clearance': 'finish_route_clearance',
}
INTERIOR_GATE_BY_PUBLIC_SEGMENT = {
    str(branch['target_segment']): gate
    for gate, branch in INTERIOR_HOLDING_BRANCH_BY_GATE.items()
}
SWITCH_VALUE_BY_ID = {
    1: 'EXTERIOR',
    2: 'INTERIOR',
}
STOPPER_VALUE_BY_ID = {
    1: '0',
    2: '1',
}
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


def _shuttle_command_speed(
    command: Any,
    requested_speed: float | None,
    default_speed: float,
) -> float:
    """Return an ON-only typed-command speed without changing retained state."""

    if _clean_token(command).upper() != 'ON':
        return 0.0
    return float(default_speed if requested_speed is None else requested_speed)


def _empty_safety_metrics() -> dict[str, Any]:
    metrics = {
        'total_proposed_actions': 0,
        'accepted_actions': 0,
        'rejected_actions': 0,
        'illegal_proposal_rate': 0.0,
        'rejected_action_rate': 0.0,
        'rejection_reasons': {},
        'trusted_state_rejection_count': 0,
        'unknown_localization_rejection_count': 0,
        'obstacle_stop_count': 0,
        'sensor_dropout_count': 0,
        'timeout_rejection_count': 0,
        'occupied_target_rejection_count': 0,
        'safety_recovery_count': 0,
        'fail_safe_abort_count': 0,
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


def _is_recoverable_safety_reason(reason: Any) -> bool:
    text = str(reason or '').casefold()
    return any(keyword in text for keyword in RECOVERABLE_SAFETY_KEYWORDS)


def _explicit_state_quality_reason(container: Any, context: str) -> str:
    if not isinstance(container, dict):
        return ''
    for key in ('status', 'state_status', 'trusted_state_status', 'safety_status'):
        status = _clean_token(container.get(key)).casefold()
        if status in {'stale', 'conflicting'}:
            return f'{context} is {status}; safe stop, reobserve, and replan required'
    for key in ('timed_out', 'timeout', 'sensor_timeout', 'observation_timeout'):
        if bool(container.get(key)):
            return f'{context} timeout; safe stop, reobserve, and replan required'
    for key in ('age_s', 'observation_age_s', 'last_update_age_s'):
        if key not in container:
            continue
        try:
            age_s = float(container.get(key))
        except (TypeError, ValueError):
            continue
        max_age_s = float(container.get('max_age_s', container.get('stale_after_s', 2.0)) or 2.0)
        if age_s > max_age_s:
            return (
                f'{context} stale: age {age_s:.3f}s exceeds {max_age_s:.3f}s; '
                'safe stop, reobserve, and replan required'
            )
    return ''


def _sensor_dropout_reason(rail: dict[str, Any], side: str) -> str:
    if bool(rail.get('sensor_dropout') or rail.get('position_sensor_dropout')):
        return f'sensor dropout on {side} rail; safe stop, reobserve, and replan required'
    status = _clean_token(
        rail.get('sensor_status')
        or rail.get('position_sensor_status')
        or rail.get('trusted_sensor_status')
    ).casefold()
    if status in {'dropout', 'lost', 'missing', 'unavailable'}:
        return f'sensor dropout on {side} rail ({status}); safe stop, reobserve, and replan required'
    return ''


def _obstacle_appearance_reason(rail: dict[str, Any], side: str) -> str:
    raw_obstacles = (
        rail.get('obstacles')
        or rail.get('present_obstacles')
        or rail.get('obstacle_markers')
        or []
    )
    if isinstance(raw_obstacles, dict):
        obstacles = [name for name, present in raw_obstacles.items() if bool(present)]
    elif isinstance(raw_obstacles, list):
        obstacles = [item for item in raw_obstacles if item]
    elif raw_obstacles:
        obstacles = [raw_obstacles]
    else:
        obstacles = []
    if obstacles or bool(rail.get('obstacle_present')):
        names = ', '.join(str(item) for item in obstacles) or 'unidentified obstacle'
        return f'obstacle appearance on {side} rail: {names}; safe stop and replan required'
    for reading in _active_sensor_readings_from_rail(rail):
        sensor_type = _clean_token(reading.get('type') or reading.get('sensor_type')).casefold()
        if sensor_type == 'obstacle':
            name = _clean_token(reading.get('name') or 'obstacle')
            return f'obstacle appearance on {side} rail: {name}; safe stop and replan required'
    return ''


def _unknown_localization_reason(
    rails: dict[str, Any],
    side: str,
    shuttle_name: str,
) -> str:
    state = _rail_shuttles(rails, side).get(shuttle_name, {})
    if not isinstance(state, dict):
        return f'unknown localization for {shuttle_name!r}: missing trusted shuttle state'
    quality = _explicit_state_quality_reason(state, f'{shuttle_name} trusted shuttle state')
    if quality:
        return quality
    localized = any(
        state.get(key) is not None and _clean_token(state.get(key)) != ''
        for key in ('segment', 'current_segment', 'block', 'current_block')
    )
    if localized:
        return ''
    for reading in _active_sensor_readings_from_rail(_rail_snapshot(rails, side)):
        if _clean_token(reading.get('shuttle')) == shuttle_name:
            return ''
    return (
        f'unknown localization for {shuttle_name!r}: no segment/block or active '
        'position sensor; safe stop, reobserve, and replan required'
    )


def _slot_number(raw: Any) -> str:
    text = _clean_token(raw)
    if not text:
        return ''
    if ':slot:' in text:
        return text.rsplit(':slot:', 1)[-1]
    if text.startswith(('right_slot_', 'left_slot_', 'slot_')):
        return text.rsplit('_', 1)[-1]
    return text if text in {'1', '2', '3', '4'} else ''


def _owner_labels_for_safety(raw_name: Any, *, side: str) -> set[str]:
    labels = {_clean_token(raw_name)} if _clean_token(raw_name) else set()
    spec = normalize_shuttle_ref(raw_name, side=side)
    if spec is not None:
        labels.update({spec.short_id, spec.shuttle_id, spec.gazebo_entity_name})
    return {label for label in labels if label}


def _target_slot_occupied_reason(
    rails: dict[str, Any],
    slot_sensor_by_side: dict[str, dict[str, str]],
    side: str,
    command: dict[str, Any],
    shuttle_name: str,
) -> str:
    slot = _slot_number(command.get('target_slot') or command.get('slot'))
    if not slot:
        return ''
    rail = _rail_snapshot(rails, side)
    slot_id = f'{side}:slot:{slot}'
    expected_labels = _owner_labels_for_safety(shuttle_name, side=side)
    occupancy_maps = [
        rail.get('slot_occupancy', {}),
        rail.get('slots', {}),
    ]
    for mapping in occupancy_maps:
        if not isinstance(mapping, dict):
            continue
        for key in (slot, slot_id, f'{side}_slot_{slot}', f'slot_{slot}'):
            if key not in mapping:
                continue
            raw_value = mapping.get(key)
            if isinstance(raw_value, dict):
                raw_value = raw_value.get('shuttle') or raw_value.get('occupant')
            occupant = _clean_token(raw_value)
            if occupant and occupant not in expected_labels:
                return f'target slot {slot_id} is occupied by {occupant}'
    target_sensor = _sensor_for_slot(slot_sensor_by_side, side, slot).casefold()
    if not target_sensor:
        return ''
    for reading in _active_sensor_readings_from_rail(rail):
        if _clean_token(reading.get('name')).casefold() != target_sensor:
            continue
        occupant = _clean_token(reading.get('shuttle'))
        if occupant and occupant not in expected_labels:
            return f'target slot {slot_id} is occupied by {occupant}'
        if not occupant:
            return f'target slot {slot_id} occupancy is active but shuttle identity is unknown'
    return ''


def _active_sensor_readings_from_rail(rail: dict[str, Any]) -> list[dict[str, Any]]:
    readings: list[dict[str, Any]] = []
    for key in ('active_sensors', 'active_position_sensors'):
        raw_readings = rail.get(key, [])
        if not isinstance(raw_readings, list):
            continue
        readings.extend(item for item in raw_readings if isinstance(item, dict))
    return readings


def _shuttle_safety_segments(
    shuttle_state: dict[str, Any],
    *,
    side: str,
) -> set[str]:
    """Return conservative segment aliases for switch-safety checks only.

    ``ShuttleState.current_segment`` uses the public topology.  The left rail
    has a mirrored internal topology, so keeping both aliases lets certificate
    checks compare the two vocabularies. Neither value is exported to the
    planner or used to replace visual localization.
    """

    segment = _clean_token(shuttle_state.get('segment', '')).upper()
    if not segment:
        return set()
    segments = {segment}
    if side == 'left':
        segments.add(LEFT_PUBLIC_SEGMENT_NAME_MAP.get(segment, segment))
    return segments


def _left_controller_segment_to_internal() -> dict[str, str]:
    """Return the validated inverse of the public left-segment vocabulary."""

    inverse = {
        str(public).strip().upper(): str(internal).strip().upper()
        for internal, public in LEFT_PUBLIC_SEGMENT_NAME_MAP.items()
    }
    if len(inverse) != len(LEFT_PUBLIC_SEGMENT_NAME_MAP):
        raise ValueError('left public segment map is not one-to-one')
    return inverse


LEFT_CONTROLLER_SEGMENT_TO_INTERNAL = _left_controller_segment_to_internal()


@lru_cache(maxsize=2)
def _rail_switch_distance_geometry(side: str) -> dict[str, Any]:
    """Load authoritative metric rail geometry for the safety veto only."""

    from room_315_kinematic_shuttle import CUBIC_HERMITE_PATH_BACKEND
    from room_315_kinematic_shuttle import RailNetwork

    normalized_side = _strict_side(side)
    if normalized_side not in SIDES:
        raise ValueError(f'unsupported rail side {side!r}')
    network = RailNetwork.from_yaml(
        default_rail_network_path(normalized_side),
        path_backend=CUBIC_HERMITE_PATH_BACKEND,
    )
    raw_segments = network.config.get('segments')
    if not isinstance(raw_segments, dict):
        raise ValueError('authoritative rail geometry has no segment definitions')

    adjacency: dict[str, list[tuple[str, float]]] = {}
    segments: dict[str, tuple[str, str, float]] = {}
    for raw_name, raw_segment in raw_segments.items():
        name = _clean_token(raw_name).upper()
        if not isinstance(raw_segment, dict) or name not in network.segments:
            raise ValueError(f'invalid authoritative segment definition {name!r}')
        start = _clean_token(raw_segment.get('start_node')).upper()
        end = _clean_token(raw_segment.get('end_node')).upper()
        length = float(network.segments[name].length)
        if not start or not end or not math.isfinite(length) or length <= 0.0:
            raise ValueError(f'invalid authoritative segment geometry {name!r}')
        segments[name] = (start, end, length)
        adjacency.setdefault(start, []).append((end, length))
        adjacency.setdefault(end, []).append((start, length))

    switch_node_distances: dict[str, dict[str, float]] = {}
    for switch_name in SWITCHES:
        switch = network.switches.get(switch_name)
        if not isinstance(switch, dict):
            raise ValueError(f'missing authoritative switch {switch_name!r}')
        target = _clean_token(switch.get('controlled_node')).upper()
        if target not in adjacency:
            raise ValueError(
                f'authoritative switch {switch_name!r} has invalid controlled node'
            )
        distances = {target: 0.0}
        queue: list[tuple[float, str]] = [(0.0, target)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance > distances.get(node, math.inf):
                continue
            for neighbor, edge_length in adjacency.get(node, ()):
                candidate = distance + edge_length
                if candidate >= distances.get(neighbor, math.inf):
                    continue
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
        switch_node_distances[switch_name] = distances

    return {
        'segments': segments,
        'switch_node_distances': switch_node_distances,
        'source': str(network.network_path),
    }


def _controller_segment_for_geometry(raw_segment: Any, *, side: str) -> str:
    segment = _clean_token(raw_segment).upper()
    if side == 'left':
        return LEFT_CONTROLLER_SEGMENT_TO_INTERNAL.get(segment, '')
    return segment


def _shuttle_switch_distance_m(
    shuttle_state: dict[str, Any],
    switch_name: str,
    *,
    side: str,
) -> tuple[float | None, str]:
    """Measure controller-state distance to a switch as an independent veto.

    ``segment`` and ``s`` originate only from deterministic controller state.
    This result is never exported as visual localization and never replaces the
    model state used by the planner or state-fusion runtime.
    """

    raw_segment = shuttle_state.get('segment')
    if raw_segment in (None, ''):
        raw_segment = shuttle_state.get('current_segment')
    if raw_segment in (None, ''):
        return None, 'missing controller segment'
    if 's' not in shuttle_state or shuttle_state.get('s') in (None, ''):
        return None, 'missing controller s'
    try:
        s = float(shuttle_state.get('s'))
    except (TypeError, ValueError):
        return None, 'invalid controller s'
    if not math.isfinite(s):
        return None, 'non-finite controller s'
    try:
        geometry = _rail_switch_distance_geometry(side)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        return None, f'authoritative rail geometry unavailable: {error}'
    segment = _controller_segment_for_geometry(raw_segment, side=side)
    segment_geometry = geometry['segments'].get(segment)
    if segment_geometry is None:
        return None, f'unknown controller segment {_clean_token(raw_segment)!r}'
    switch_distances = geometry['switch_node_distances'].get(switch_name)
    if not isinstance(switch_distances, dict):
        return None, f'unknown controlled switch {switch_name!r}'
    start, end, length = segment_geometry
    if (
        s < -CONTROLLER_S_RANGE_TOLERANCE_M
        or s > length + CONTROLLER_S_RANGE_TOLERANCE_M
    ):
        return None, (
            f'controller s={s:.6g} is outside segment {segment} '
            f'length={length:.6g}'
        )
    s = max(0.0, min(s, length))
    start_distance = switch_distances.get(start)
    end_distance = switch_distances.get(end)
    if start_distance is None or end_distance is None:
        return None, f'segment {segment} is disconnected from switch {switch_name}'
    distance = min(s + start_distance, length - s + end_distance)
    if not math.isfinite(distance):
        return None, 'non-finite switch clearance distance'
    return distance, ''


def _controller_state_for_sensor_identity(
    rails: dict[str, Any],
    *,
    side: str,
    raw_identity: Any,
) -> tuple[str, dict[str, Any] | None, str]:
    spec = normalize_shuttle_ref(raw_identity, side=side)
    if spec is None or spec.side != side:
        return '', None, 'unknown or wrong-side shuttle identity'
    matches = []
    for raw_name, state in _rail_shuttles(rails, side).items():
        candidate = normalize_shuttle_ref(raw_name, side=side)
        if candidate is None or candidate.shuttle_id != spec.shuttle_id:
            continue
        if isinstance(state, dict):
            matches.append((str(raw_name), state))
    if not matches:
        return spec.shuttle_id, None, 'identity has no controller shuttle state'
    if len(matches) != 1:
        return spec.shuttle_id, None, 'identity has duplicate controller states'
    return matches[0][0], matches[0][1], ''


def _active_sensor_switch_occupancy_reason(
    rails: dict[str, Any],
    *,
    side: str,
    switch_name: str,
) -> str:
    rail = _rail_snapshot(rails, side)
    near_names = {
        name.upper()
        for name in SWITCH_SENSOR_PREFIX_BY_SIDE.get(side, {}).get(switch_name, ())
    }
    for reading in _active_sensor_readings_from_rail(rail):
        sensor_name = _clean_token(reading.get('name')).upper()
        if not sensor_name or sensor_name not in near_names:
            continue
        raw_identity = _clean_token(reading.get('shuttle'))
        if not raw_identity:
            return (
                f'active {switch_name} guard sensor {sensor_name} has unknown '
                'shuttle identity'
            )
        shuttle_name, state, reason = _controller_state_for_sensor_identity(
            rails,
            side=side,
            raw_identity=raw_identity,
        )
        if reason or state is None:
            return (
                f'active {switch_name} guard sensor {sensor_name} is not '
                f'identity-bound: {reason}'
            )
        distance, reason = _shuttle_switch_distance_m(
            state,
            switch_name,
            side=side,
        )
        if reason or distance is None:
            return (
                f'active {switch_name} guard sensor {sensor_name} cannot prove '
                f'{shuttle_name} clear: {reason}'
            )
        if distance <= SWITCH_CLEAR_DISTANCE_M:
            return (
                f'active {switch_name} guard sensor {sensor_name} is '
                f'identity-bound to {shuttle_name} at {distance:.3f}m'
            )
    return ''


def _unsafe_switch_change_reason(
    rails: dict[str, Any],
    side: str,
    switch_name: str,
) -> str:
    moving_near = []
    for shuttle_name, shuttle_state in _rail_shuttles(rails, side).items():
        if _shuttle_is_falling(shuttle_state):
            continue
        distance, reason = _shuttle_switch_distance_m(
            shuttle_state,
            switch_name,
            side=side,
        )
        if reason or distance is None:
            return (
                f'unsafe switch change: {side} switch {switch_name} cannot prove '
                f'controller safety distance for {shuttle_name}: {reason}'
            )
        if (
            _shuttle_is_moving(shuttle_state)
            and distance <= SWITCH_CLEAR_DISTANCE_M
        ):
            moving_near.append(f'{shuttle_name}@{distance:.3f}m')
    if moving_near:
        return (
            f'unsafe switch change: {side} switch {switch_name} is near moving '
            f'shuttle(s) {", ".join(moving_near)}'
        )
    return ''


def _occupied_guarded_segment_reason(
    rails: dict[str, Any],
    side: str,
    switch_name: str,
) -> str:
    occupied_shuttles = []
    for shuttle_name, shuttle_state in _rail_shuttles(rails, side).items():
        if _shuttle_is_falling(shuttle_state):
            continue
        distance, reason = _shuttle_switch_distance_m(
            shuttle_state,
            switch_name,
            side=side,
        )
        if reason or distance is None:
            return (
                f'{side} switch {switch_name} clearance is unknown for '
                f'{shuttle_name}: {reason}'
            )
        if distance <= SWITCH_CLEAR_DISTANCE_M:
            occupied_shuttles.append(f'{shuttle_name}@{distance:.3f}m')
    if occupied_shuttles:
        return (
            f'unsafe switch change: {side} switch {switch_name} guarded segment '
            f'is occupied within {SWITCH_CLEAR_DISTANCE_M:.3f}m by shuttle(s) '
            f'{", ".join(occupied_shuttles)}'
        )
    sensor_reason = _active_sensor_switch_occupancy_reason(
        rails,
        side=side,
        switch_name=switch_name,
    )
    if sensor_reason:
        return (
            f'unsafe switch change: {side} switch {switch_name} guarded segment '
            f'has active occupancy evidence: {sensor_reason}'
        )
    return ''


def _changed_switch_names(
    rails: dict[str, Any],
    side: str,
    expanded: dict[str, str],
) -> tuple[str, ...]:
    switches = _rail_snapshot(rails, side).get('switches', {})
    switches = switches if isinstance(switches, dict) else {}
    return tuple(
        switch_name
        for switch_name, desired_state in expanded.items()
        if _canonical_switch_state(switches.get(switch_name)) != desired_state
    )


def _normalization_symbolic_step_reason(
    metadata: dict[str, Any],
    *,
    side: str,
    mode: str,
) -> str:
    expected_action = GUARDED_SWITCH_PROOF_ACTION_BY_MODE[mode]
    text = _clean_token(metadata.get('symbolic_step')).lower()
    text = re.sub(r'^\s*\d+(?:\.\d+)?\s*:\s*', '', text)
    text = re.sub(r'\s*\[[^\]]*\]\s*$', '', text).strip()
    if text.startswith('(') and text.endswith(')'):
        text = text[1:-1].strip()
    tokens = text.split()
    actual_action = (
        ROUTE_NORMALIZATION_SYMBOLIC_ACTION_ALIASES.get(tokens[0], tokens[0])
        if tokens
        else ''
    )
    if not tokens or actual_action != expected_action:
        return (
            f'symbolic_step must be {expected_action!r} for mode {mode!r}'
        )
    side_index = (
        2
        if expected_action in {'begin_route_clearance', 'finish_route_clearance'}
        else 1
    )
    if len(tokens) <= side_index or tokens[side_index] != side:
        return 'symbolic_step side does not match the switch command side'
    return ''


def _identity_set_for_normalization(
    raw: Any,
    *,
    side: str,
    field: str,
) -> tuple[set[str], str]:
    if not isinstance(raw, (list, tuple)):
        return set(), f'{field} must be a list'
    identities: set[str] = set()
    for raw_identity in raw:
        spec = normalize_shuttle_ref(raw_identity, side=side)
        if spec is None or spec.side != side:
            return set(), f'{field} contains an unknown or wrong-side identity'
        if spec.shuttle_id in identities:
            return set(), f'{field} contains duplicate identity {spec.short_id}'
        identities.add(spec.shuttle_id)
    return identities, ''


def _current_interior_safety_occupants(
    rails: dict[str, Any],
    *,
    side: str,
) -> tuple[dict[str, dict[str, Any]], str]:
    """Return controller-confirmed stopped shuttles on the interior branch.

    ``ShuttleState.speed`` is the retained travel-speed setting used by the
    Gazebo controller when a shuttle is enabled.  It intentionally remains
    non-zero after an OFF command so the next ON command can reuse that speed;
    it is not an instantaneous velocity measurement.  Consequently the
    explicit ``DISABLED`` controller mode, not ``WAITING`` and not the
    retained speed setting, is the authoritative OFF effect at this boundary.
    An enabled shuttle held by a stopper or collision also reports ``WAITING``
    and must not authorize a route change that could release it.
    """

    occupants: dict[str, dict[str, Any]] = {}
    for raw_identity, state in _rail_shuttles(rails, side).items():
        if _shuttle_is_falling(state):
            continue
        if not (
            _shuttle_safety_segments(state, side=side)
            & set(INTERIOR_GATE_BY_PUBLIC_SEGMENT)
        ):
            continue
        spec = normalize_shuttle_ref(raw_identity, side=side)
        if spec is None or spec.side != side:
            return {}, 'current interior safety state has an unknown identity'
        if spec.shuttle_id in occupants:
            return {}, f'current interior safety state duplicates {spec.short_id}'
        mode = _shuttle_mode_from_state(state)
        if mode != CONTROLLER_DISABLED_MODE:
            return {}, f'{spec.short_id} has no explicit disabled controller mode'
        occupants[spec.shuttle_id] = state
    if not occupants:
        return {}, 'route-normalization exception has no stopped interior shuttle'
    return occupants, ''


def _normalization_device_snapshot_reason(
    proof: dict[str, Any],
    *,
    rails: dict[str, Any],
    side: str,
) -> str:
    rail = _rail_snapshot(rails, side)
    current_switches = rail.get('switches', {})
    current_stoppers = rail.get('stoppers', {})
    proof_switches = proof.get('switches')
    proof_stoppers = proof.get('stoppers')
    if not all(
        isinstance(value, dict)
        for value in (
            current_switches,
            current_stoppers,
            proof_switches,
            proof_stoppers,
        )
    ):
        return 'route-normalization device snapshots must be mappings'
    expected_switches = {
        name: _canonical_switch_state(current_switches.get(name)).lower()
        for name in SWITCHES
    }
    certified_switches = {
        _clean_token(name).upper(): _canonical_switch_state(value).lower()
        for name, value in proof_switches.items()
    }
    if certified_switches != expected_switches:
        return 'route-normalization switch snapshot does not match current state'
    expected_stoppers = {
        name: 'open' if _normalize_stopper_state(current_stoppers.get(name)) == '0'
        else 'closed' if _normalize_stopper_state(current_stoppers.get(name)) == '1'
        else _clean_token(current_stoppers.get(name)).lower()
        for name in SWITCHES
    }
    certified_stoppers = {
        _clean_token(name).upper(): (
            'open' if _normalize_stopper_state(value) == '0'
            else 'closed' if _normalize_stopper_state(value) == '1'
            else _clean_token(value).lower()
        )
        for name, value in proof_stoppers.items()
    }
    if certified_stoppers != expected_stoppers:
        return 'route-normalization stopper snapshot does not match current state'
    return ''


def _normalization_certificates_reason(
    metadata: dict[str, Any],
    proof: dict[str, Any],
    *,
    rails: dict[str, Any],
    side: str,
) -> str:
    interior_states, reason = _current_interior_safety_occupants(
        rails,
        side=side,
    )
    if reason:
        return reason
    interior_ids = set(interior_states)
    exact_fields = [
        'interior_shuttles',
        'certified_interior_shuttles',
        'clearance_lifecycle_certified_stopped_interior_shuttles',
    ]
    for field in exact_fields:
        identities, reason = _identity_set_for_normalization(
            proof.get(field),
            side=side,
            field=field,
        )
        if reason:
            return reason
        if identities != interior_ids:
            return f'{field} does not exactly match current interior occupants'
    subset_fields: dict[str, set[str]] = {}
    for field in (
        'visually_interior_shuttles',
        'certified_stopped_interior_shuttles',
    ):
        identities, reason = _identity_set_for_normalization(
            proof.get(field),
            side=side,
            field=field,
        )
        if reason:
            return reason
        if not identities.issubset(interior_ids):
            return f'{field} contains a non-interior identity'
        subset_fields[field] = identities
    for field in (
        'external_obstacles',
        'clearance_lifecycle_uncertified_interior_shuttles',
    ):
        value = proof.get(field)
        if not isinstance(value, list) or value:
            return f'{field} must be an empty list'
    uncertified_interior, reason = _identity_set_for_normalization(
        proof.get('uncertified_interior_shuttles'),
        side=side,
        field='uncertified_interior_shuttles',
    )
    if reason:
        return reason
    expected_uncertified = (
        interior_ids
        - subset_fields['certified_stopped_interior_shuttles']
    )
    if uncertified_interior != expected_uncertified:
        return (
            'uncertified_interior_shuttles does not match the strict '
            'visual/certificate disagreement set'
        )
    certified_visual_disagreements, reason = (
        _identity_set_for_normalization(
            proof.get('clearance_lifecycle_visual_disagreements'),
            side=side,
            field='clearance_lifecycle_visual_disagreements',
        )
    )
    if reason:
        return reason
    mismatches, reason = _identity_set_for_normalization(
        proof.get('certificate_segment_mismatches'),
        side=side,
        field='certificate_segment_mismatches',
    )
    if reason:
        return reason
    if (
        mismatches != certified_visual_disagreements
        or mismatches != uncertified_interior
    ):
        return 'visual/certificate disagreements are not exactly certified'
    if (
        proof.get('clearance_lifecycle_visual_prediction_preserved')
        is not True
        or proof.get(
            'clearance_lifecycle_certificate_used_as_localization'
        )
        is not False
    ):
        return 'clearance-lifecycle visual provenance is invalid'

    raw_consistency = proof.get('certificate_segment_consistency')
    if not isinstance(raw_consistency, dict):
        return 'certificate_segment_consistency must be a mapping'
    consistency_by_id: dict[str, dict[str, Any]] = {}
    for raw_identity, entry in raw_consistency.items():
        spec = normalize_shuttle_ref(raw_identity, side=side)
        if spec is None or spec.side != side or not isinstance(entry, dict):
            return 'certificate_segment_consistency contains an invalid identity'
        if spec.shuttle_id in consistency_by_id:
            return 'certificate_segment_consistency contains duplicate identities'
        consistency_by_id[spec.shuttle_id] = entry
    if set(consistency_by_id) != interior_ids:
        return 'certificate_segment_consistency does not match interior occupants'
    for identity, consistency in consistency_by_id.items():
        expected_public_segment = _clean_token(
            consistency.get('certificate_target_public_segment')
        ).upper()
        expected_visual_segment = (
            LEFT_PUBLIC_SEGMENT_NAME_MAP.get(
                expected_public_segment,
                expected_public_segment,
            )
            if side == 'left'
            else expected_public_segment
        )
        strict_visual_match = (
            consistency.get('required') is not True
            or expected_public_segment not in INTERIOR_GATE_BY_PUBLIC_SEGMENT
            or _clean_token(
                consistency.get('certificate_target_public_segment')
            ).upper() != expected_public_segment
            or _clean_token(
                consistency.get('certificate_target_internal_segment')
            ).upper() != expected_visual_segment
            or consistency.get('certificate_used_as_localization') is not False
        )
        if strict_visual_match:
            return f'visual/certificate segment proof is invalid for {identity}'
        accepted_visual_segment = _clean_token(
            consistency.get('accepted_visual_internal_segment')
        ).upper()
        if consistency.get('satisfied') is True:
            if accepted_visual_segment != expected_visual_segment:
                return f'visual/certificate segment proof is invalid for {identity}'
        elif not (
            identity in certified_visual_disagreements
            and accepted_visual_segment
            and accepted_visual_segment != expected_visual_segment
            and consistency.get(
                'certificate_used_as_persisted_execution_effect'
            )
            is True
            and consistency.get('raw_visual_prediction_preserved') is True
        ):
            return f'visual/certificate segment proof is invalid for {identity}'

    raw_certificates = metadata.get('runtime_clearance_certificates')
    if not isinstance(raw_certificates, dict):
        return 'runtime_clearance_certificates must be a mapping'
    certificates: dict[str, dict[str, Any]] = {}
    for raw_identity, raw_certificate in raw_certificates.items():
        try:
            certificate = normalize_runtime_clearance_certificate(
                raw_identity,
                raw_certificate,
            )
        except ValueError as exc:
            return f'runtime clearance certificate is invalid: {exc}'
        spec = normalize_shuttle_ref(certificate['identity'])
        if spec is None or spec.side != side:
            return 'runtime clearance certificate has a side conflict'
        if spec.shuttle_id in certificates:
            return 'runtime clearance certificates contain duplicate identities'
        certificates[spec.shuttle_id] = certificate
    if set(certificates) != interior_ids:
        return 'runtime clearance certificates do not match interior occupants'
    for identity, certificate in certificates.items():
        spec = normalize_shuttle_ref(identity, side=side)
        target_segment = _clean_token(
            certificate.get('target_segment')
        ).upper()
        gate = INTERIOR_GATE_BY_PUBLIC_SEGMENT.get(target_segment, '')
        expected_sensor = INTERIOR_LOOP_ENTRY_SENSOR_BY_SIDE_AND_GATE.get(
            (side, gate),
            '',
        )
        try:
            target_s_m = float(certificate['target_s_m'])
        except (KeyError, TypeError, ValueError):
            return f'runtime clearance target_s_m is invalid for {identity}'
        if (
            spec is None
            or not gate
            or target_segment not in INTERIOR_GATE_BY_PUBLIC_SEGMENT
            or _clean_token(certificate.get('entry_sensor')).upper() != expected_sensor
            or not math.isfinite(target_s_m)
            or target_s_m <= 0.0
        ):
            return f'runtime clearance certificate is invalid for {identity}'
    return ''


def _clearance_route_switch_proof_reason(
    metadata: dict[str, Any],
    *,
    side: str,
    expanded: dict[str, str],
) -> str:
    route_switch_proof = metadata.get('clearance_route_switch_proof')
    if not isinstance(route_switch_proof, dict):
        return 'clearance_route_switch_proof must be a mapping'
    if (
        route_switch_proof.get('side') != side
        or route_switch_proof.get('route_specific_switch_assignment') is not True
        or route_switch_proof.get(
            'controller_position_fields_used_for_localization'
        )
        is not False
    ):
        return 'clearance route switch proof provenance is invalid'
    raw_required = route_switch_proof.get('required_switches')
    if not isinstance(raw_required, dict):
        return 'clearance route required_switches must be a mapping'
    required_switches = {
        _clean_token(name).upper(): _canonical_switch_state(state)
        for name, state in raw_required.items()
    }
    if (
        set(required_switches) != set(SWITCHES)
        or expanded != required_switches
    ):
        return 'switch command does not match the proved clearance route'
    target_segment = _clean_token(
        route_switch_proof.get('target_segment')
    ).upper()
    gate = INTERIOR_GATE_BY_PUBLIC_SEGMENT.get(target_segment, '')
    branch = INTERIOR_HOLDING_BRANCH_BY_GATE.get(gate)
    exit_switch = _clean_token(
        route_switch_proof.get('exit_switch')
    ).upper()
    if (
        not branch
        or _clean_token(route_switch_proof.get('gate_switch')).upper() != gate
        or exit_switch != _clean_token(branch.get('exit_switch')).upper()
        or required_switches.get(gate) != 'INTERIOR'
        or required_switches.get(exit_switch) != 'INTERIOR'
    ):
        return 'clearance route does not prove its target interior branch'
    return ''


def _clearance_motion_route_proof_reason(
    metadata: dict[str, Any],
    *,
    rails: dict[str, Any],
    side: str,
) -> str:
    """Bind an interior shuttle ON command to the actual device route."""

    proof = metadata.get('clearance_motion_route_proof')
    if not isinstance(proof, dict):
        return 'clearance_motion_route_proof must be a mapping'
    if (
        proof.get('side') != side
        or proof.get('route_specific_switch_assignment') is not True
        or proof.get('controller_position_fields_used_for_localization')
        is not False
    ):
        return 'clearance motion route proof provenance is invalid'
    raw_switches = proof.get('required_switches')
    raw_stoppers = proof.get('required_stoppers')
    if not isinstance(raw_switches, dict) or not isinstance(raw_stoppers, dict):
        return 'clearance motion route device requirements must be mappings'
    required_switches = {
        _clean_token(device).upper(): _canonical_switch_state(state)
        for device, state in raw_switches.items()
    }
    required_stoppers = {
        _clean_token(device).upper(): _normalize_stopper_state(state)
        for device, state in raw_stoppers.items()
    }
    if (
        set(required_switches) != set(SWITCHES)
        or set(required_stoppers) != set(SWITCHES)
    ):
        return 'clearance motion route proof is not a complete device assignment'
    target_segment = _clean_token(proof.get('target_segment')).upper()
    gate = INTERIOR_GATE_BY_PUBLIC_SEGMENT.get(target_segment, '')
    branch = INTERIOR_HOLDING_BRANCH_BY_GATE.get(gate)
    exit_switch = _clean_token(proof.get('exit_switch')).upper()
    if (
        not branch
        or _clean_token(proof.get('gate_switch')).upper() != gate
        or exit_switch != _clean_token(branch.get('exit_switch')).upper()
        or required_switches.get(gate) != 'INTERIOR'
        or required_switches.get(exit_switch) != 'INTERIOR'
        or required_stoppers
        != {device: ('1' if device == exit_switch else '0') for device in SWITCHES}
    ):
        return 'clearance motion route does not hold its target branch safely'
    rail = _rail_snapshot(rails, side)
    actual_switches = {
        device: _canonical_switch_state(
            dict(rail.get('switches') or {}).get(device)
        )
        for device in SWITCHES
    }
    actual_stoppers = {
        device: _normalize_stopper_state(
            dict(rail.get('stoppers') or {}).get(device)
        )
        for device in SWITCHES
    }
    if actual_switches != required_switches:
        return (
            'clearance motion required switch assignment is not active:'
            f'required={required_switches},actual={actual_switches}'
        )
    if actual_stoppers != required_stoppers:
        return (
            'clearance motion required stopper assignment is not active:'
            f'required={required_stoppers},actual={actual_stoppers}'
        )
    return ''


def _guarded_route_normalization_proof_reason(
    command: dict[str, Any],
    *,
    rails: dict[str, Any],
    side: str,
    expanded: dict[str, str],
    changed_switches: tuple[str, ...],
) -> str:
    metadata = command.get('closed_loop_executive')
    if not isinstance(metadata, dict):
        return 'missing closed-loop route-normalization proof'
    mode = _clean_token(metadata.get('mode'))
    if mode not in GUARDED_SWITCH_PROOF_ACTION_BY_MODE:
        return 'closed-loop mode is not authorized for route normalization'
    if mode == 'begin_route_clearance_hold_interior':
        reason = _clearance_route_switch_proof_reason(
            metadata,
            side=side,
            expanded=expanded,
        )
        if reason:
            return reason
    elif expanded != {name: 'EXTERIOR' for name in SWITCHES}:
        return (
            'guarded occupancy permits only all-switch EXTERIOR '
            'normalization or an exact proved clearance route'
        )
    if (
        metadata.get('localization_source') != 'accepted_visual_state'
        or metadata.get('controller_position_fields_used_for_localization') is not False
    ):
        return 'route normalization lacks accepted-visual localization provenance'
    if not _clean_token(metadata.get('problem_name')):
        return 'route normalization lacks a bound planning problem'
    reason = _normalization_symbolic_step_reason(
        metadata,
        side=side,
        mode=mode,
    )
    if reason:
        return reason
    proof = metadata.get('route_normalization_proof')
    if not isinstance(proof, dict):
        return 'route_normalization_proof must be a mapping'
    if proof.get('side') != side:
        return 'route-normalization proof side does not match command side'
    if proof.get('controller_position_fields_used_for_localization') is not False:
        return 'route-normalization proof used forbidden controller localization'
    reason = _normalization_device_snapshot_reason(
        proof,
        rails=rails,
        side=side,
    )
    if reason:
        return reason
    if _obstacle_appearance_reason(_rail_snapshot(rails, side), side):
        return 'current rail has an obstacle during route normalization'
    if mode == 'begin_route_clearance_hold_interior':
        if (
            proof.get('normal_route') is not True
            or proof.get('all_stoppers_open') is not True
            or proof.get('clearance_mode') is not False
        ):
            return 'clearance-route reconfiguration did not start from a proved normal route'
    elif mode == 'restore_normal_route_before_slot_motion':
        if (
            proof.get('reconfiguration_required') is not True
            or proof.get('reconfiguration_safe') is not True
            or proof.get('clearance_mode') is not False
            or proof.get('all_stoppers_open') is not True
        ):
            return 'mixed-route normalization is not proven safe'
    elif (
        proof.get('clearance_mode') is not True
        or proof.get('clearance_pause_safe') is not True
    ):
        return 'active-clearance normalization is not proven safe'
    reason = _normalization_certificates_reason(
        metadata,
        proof,
        rails=rails,
        side=side,
    )
    if reason:
        return reason

    rail = _rail_snapshot(rails, side)
    interior_ids, reason = _current_interior_safety_occupants(
        rails,
        side=side,
    )
    if reason:
        return reason
    interior_ids = set(interior_ids)
    for switch_name in changed_switches:
        near_names = {
            name.upper()
            for name in SWITCH_SENSOR_PREFIX_BY_SIDE.get(side, {}).get(
                switch_name,
                (),
            )
        }
        for reading in _active_sensor_readings_from_rail(rail):
            if _clean_token(reading.get('name')).upper() not in near_names:
                continue
            shuttle_name, state, sensor_reason = (
                _controller_state_for_sensor_identity(
                    rails,
                    side=side,
                    raw_identity=reading.get('shuttle'),
                )
            )
            if sensor_reason or state is None:
                return (
                    f'active {switch_name} guard sensor is not identity-bound '
                    f'to controller state: {sensor_reason}'
                )
            distance, distance_reason = _shuttle_switch_distance_m(
                state,
                switch_name,
                side=side,
            )
            if distance_reason or distance is None:
                return (
                    f'active {switch_name} guard sensor cannot prove '
                    f'{shuttle_name} clear: {distance_reason}'
                )
            if distance > SWITCH_CLEAR_DISTANCE_M:
                continue
            spec = normalize_shuttle_ref(shuttle_name, side=side)
            if spec is None or spec.shuttle_id not in interior_ids:
                return (
                    f'active {switch_name} guard sensor is not identity-bound '
                    'to a certified interior shuttle'
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


def _looks_like_numeric_vector(raw: Any) -> bool:
    if not isinstance(raw, list) or not raw:
        return False
    try:
        [float(value) for value in raw]
    except (TypeError, ValueError):
        return False
    return True


def _removed_action_vector_decision(raw: Any) -> dict[str, Any]:
    return _safety_decision(
        accepted=False,
        original_action=raw,
        reason=(
            'removed action_vector commands are not supported; production execution '
            'must use PlanSys2 primitive command JSON'
        ),
    )


def _resolve_shuttle_command_name(raw: str, *, side: str) -> str:
    spec = normalize_shuttle_ref(raw, side=side)
    if spec is not None:
        return spec.gazebo_entity_name
    return _clean_token(raw)


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
    if _looks_like_numeric_vector(command):
        return _removed_action_vector_decision(command)
    if not isinstance(command, dict):
        return _safety_decision(
            accepted=False,
            original_action=command,
            reason='model action must be a JSON object',
        )
    if 'action_vector' in command:
        return _removed_action_vector_decision(command)

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
        rail = _rail_snapshot(rails, side)
        reason = (
            _explicit_state_quality_reason(rail, f'{side} trusted safety state')
            or _sensor_dropout_reason(rail, side)
        )
        if reason:
            return _safety_decision(accepted=False, original_action=command, reason=reason)
        assignments_are_noop = _switch_assignments_are_noop(rails, side, expanded)
        changed_switches = _changed_switch_names(rails, side, expanded)
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
            guarded_occupancy_reasons = []
            for switch_name in changed_switches:
                reason = _unsafe_switch_change_reason(rails, side, switch_name)
                if reason:
                    return _safety_decision(accepted=False, original_action=command, reason=reason)
                occupied_reason = _occupied_guarded_segment_reason(
                    rails,
                    side,
                    switch_name,
                )
                if occupied_reason:
                    guarded_occupancy_reasons.append(occupied_reason)
            metadata = command.get('closed_loop_executive')
            normalization_mode = (
                _clean_token(metadata.get('mode'))
                if isinstance(metadata, dict)
                else ''
            )
            if (
                normalization_mode
                in GUARDED_ROUTE_RECONFIGURATION_ACTION_BY_MODE
            ):
                proof_reason = _clearance_route_switch_proof_reason(
                    metadata,
                    side=side,
                    expanded=expanded,
                )
                if proof_reason:
                    return _safety_decision(
                        accepted=False,
                        original_action=command,
                        reason=proof_reason,
                    )
            if (
                guarded_occupancy_reasons
                or normalization_mode in ROUTE_NORMALIZATION_ACTION_BY_MODE
            ):
                proof_reason = _guarded_route_normalization_proof_reason(
                    command,
                    rails=rails,
                    side=side,
                    expanded=expanded,
                    changed_switches=changed_switches,
                )
                if proof_reason:
                    reason_parts = [*guarded_occupancy_reasons, proof_reason]
                    return _safety_decision(
                        accepted=False,
                        original_action=command,
                        reason='; '.join(reason_parts),
                    )
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
        rail = _rail_snapshot(rails, side)
        reason = (
            _explicit_state_quality_reason(rail, f'{side} trusted safety state')
            or _sensor_dropout_reason(rail, side)
        )
        if reason:
            return _safety_decision(accepted=False, original_action=command, reason=reason)
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
            rail = _rail_snapshot(rails, side)
            reason = (
                _explicit_state_quality_reason(rail, f'{side} trusted safety state')
                or _sensor_dropout_reason(rail, side)
                or _unknown_localization_reason(rails, side, shuttle_name)
                or _obstacle_appearance_reason(rail, side)
                or _target_slot_occupied_reason(
                    rails,
                    slot_sensor_by_side,
                    side,
                    command,
                    shuttle_name,
                )
            )
            if reason:
                return _safety_decision(accepted=False, original_action=command, reason=reason)
            metadata = command.get('closed_loop_executive')
            clearance_mode = (
                _clean_token(metadata.get('mode'))
                if isinstance(metadata, dict)
                else ''
            )
            if clearance_mode in {
                'plansys2_supervised_interior_clearance',
                'plansys2_supervised_interior_advance',
            }:
                route_reason = _clearance_motion_route_proof_reason(
                    metadata,
                    rails=rails,
                    side=side,
                )
                if route_reason:
                    return _safety_decision(
                        accepted=False,
                        original_action=command,
                        reason=route_reason,
                    )
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
        rail = _rail_snapshot(rails, side)
        reason = _explicit_state_quality_reason(rail, f'{side} trusted safety state')
        if moving:
            reason = reason or _sensor_dropout_reason(rail, side) or _obstacle_appearance_reason(rail, side)
        if reason:
            return _safety_decision(accepted=False, original_action=command, reason=reason)
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
        self.max_recovery_retries = max(int(self.defaults.get('max_recovery_retries', 2) or 2), 0)
        self.safety_recovery: dict[str, Any] = {
            'phase': 'idle',
            'retry_count': 0,
            'reason': '',
            'next_step': '',
            'model_input_exposure': 'excluded',
        }
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
            'reached_target_slot': str(msg.reached_target_slot or ''),
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
                if self._handle_recoverable_safety_rejection(decision):
                    self._publish_status()
                    return
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
        if isinstance(command, dict) and 'action_vector' in command:
            decision = _removed_action_vector_decision(command)
            self._record_safety_decision(decision)
            return decision
        if _looks_like_numeric_vector(command):
            decision = _removed_action_vector_decision(command)
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

    def _safety_decode_command(self, command: Any) -> dict[str, Any]:
        if isinstance(command, dict) and 'action_vector' in command:
            return _removed_action_vector_decision(command)
        if _looks_like_numeric_vector(command):
            return _removed_action_vector_decision(command)
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
            if any(word in reason_text for word in ('stale', 'conflicting', 'trusted safety state')):
                self.safety_metrics['trusted_state_rejection_count'] = (
                    int(self.safety_metrics.get('trusted_state_rejection_count') or 0) + 1
                )
            if 'unknown localization' in reason_text:
                self.safety_metrics['unknown_localization_rejection_count'] = (
                    int(self.safety_metrics.get('unknown_localization_rejection_count') or 0) + 1
                )
            if 'obstacle' in reason_text:
                self.safety_metrics['obstacle_stop_count'] = (
                    int(self.safety_metrics.get('obstacle_stop_count') or 0) + 1
                )
            if 'sensor dropout' in reason_text:
                self.safety_metrics['sensor_dropout_count'] = (
                    int(self.safety_metrics.get('sensor_dropout_count') or 0) + 1
                )
            if 'timeout' in reason_text:
                self.safety_metrics['timeout_rejection_count'] = (
                    int(self.safety_metrics.get('timeout_rejection_count') or 0) + 1
                )
            if 'target slot' in reason_text and 'occupied' in reason_text:
                self.safety_metrics['occupied_target_rejection_count'] = (
                    int(self.safety_metrics.get('occupied_target_rejection_count') or 0) + 1
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

    def _handle_recoverable_safety_rejection(self, decision: dict[str, Any]) -> bool:
        reason = str(decision.get('reason') or 'unknown safety rejection')
        if not _is_recoverable_safety_reason(reason):
            return False
        previous = self.safety_recovery if isinstance(self.safety_recovery, dict) else {}
        retry_count = int(previous.get('retry_count') or 0) + 1
        if retry_count > self.max_recovery_retries:
            self.safety_metrics['fail_safe_abort_count'] = (
                int(self.safety_metrics.get('fail_safe_abort_count') or 0) + 1
            )
            self.emergency_stop = True
            self._stop_all(
                close_stoppers=True,
                reason=f'fail-safe abort after bounded recovery retries: {reason}',
            )
            self.safety_recovery = {
                'phase': 'fail_safe_abort',
                'retry_count': retry_count,
                'max_retries': self.max_recovery_retries,
                'reason': reason,
                'next_step': 'manual_intervention_required',
                'model_input_exposure': 'excluded',
            }
            return True

        self.safety_metrics['safety_recovery_count'] = (
            int(self.safety_metrics.get('safety_recovery_count') or 0) + 1
        )
        self._stop_all(
            close_stoppers=False,
            reason=f'safe recovery stop before reobserve/replan: {reason}',
        )
        self.safety_recovery = {
            'phase': 'safe_stop_reobserve_replan',
            'retry_count': retry_count,
            'max_retries': self.max_recovery_retries,
            'reason': reason,
            'next_step': 'reacquire_observations_then_request_new_plan',
            'model_input_exposure': 'excluded',
        }
        return True

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
            target_slot=_slot_number(command.get('target_slot') or ''),
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
        return {
            side: {
                slot: SLOT_SENSOR_BY_SIDE_AND_SLOT[(side, slot)]
                for slot in ('1', '2', '3', '4')
            }
            for side in SIDES
        }

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
        target_slot: str = '',
        speed: float | None = None,
        task_id: str = '',
    ) -> None:
        msg = ShuttleCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = _clean_token(name)
        msg.command = command.upper()
        msg.start_slot = start_slot
        # ShuttleCommand.speed is an ON travel-speed setting. Sending the
        # retained/default value with OFF is physically harmless but makes
        # logs look like a non-zero stop command and invites the same semantic
        # confusion as the historical regression fixed at the state boundary.
        msg.speed = _shuttle_command_speed(
            msg.command,
            speed,
            self._default_speed(),
        )
        msg.target_slot = str(target_slot)
        self.shuttle_command_pubs[side].publish(msg)
        payload = {
            'shuttle': msg.name,
            'command': msg.command,
            'start_slot': msg.start_slot,
            'target_slot': msg.target_slot,
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
                'recovery': dict(getattr(self, 'safety_recovery', {
                    'phase': 'idle',
                    'model_input_exposure': 'excluded',
                })),
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
