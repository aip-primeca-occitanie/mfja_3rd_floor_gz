#!/usr/bin/env python3
"""Translate Room 315 PDDL plan steps into structured primitive commands.

This module is an expert/data-generation bridge. It does not call PlanSys, does
not execute ROS commands, and does not define learned-model inputs. Symbolic PDDL
state may be used here to produce scenario events, but model_input remains owned
by the dataset recorder and stays limited to deployable language,
overhead_images, last_command, and observable_state.
"""

import json
import re
import shlex
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import PRIMITIVE_IDS
from room_315_multi_shuttle import REASON_IDS
from room_315_multi_shuttle import SIDES
from room_315_multi_shuttle import SIDE_IDS
from room_315_multi_shuttle import STOPPER_VALUE_IDS
from room_315_multi_shuttle import SWITCH_VALUE_IDS
from room_315_multi_shuttle import TARGET_IDS
from room_315_multi_shuttle import WAIT_CONDITION_IDS
from room_315_multi_shuttle import device_map as _device_map
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_multi_shuttle import safe_int as _safe_int

SYMBOLIC_ACTION_PRIMITIVE_MAP = {
    'prepare_switches': 'SET_SWITCHES',
    'open_stoppers': 'SET_STOPPERS',
    'set_stoppers': 'SET_STOPPERS',
    'move_shuttle': 'SHUTTLE_ON',
    'move_shuttle_to_slot': 'SHUTTLE_ON',
    'prepare_topology_route': 'SET_SWITCHES',
    'move_shuttle_from_segment_to_slot': 'SHUTTLE_ON',
    'begin_route_clearance': 'SET_SWITCHES',
    'relocate_blocker_to_interior': 'SHUTTLE_ON',
    'finish_route_clearance': 'SET_SWITCHES',
    'stop_shuttle': 'STOP_NOW',
    'finish_task': 'DONE',
    'finish_candidate_task': 'DONE',
    'inspect_state': 'DONE',
    'wait_for_clearance': 'STOP_NOW',
}

@dataclass(frozen=True)
class PddlPlanStep:
    """One symbolic step from a PDDL planner output."""

    name: str
    args: tuple[str, ...]
    kwargs: dict[str, str]
    raw: str = ''

    @classmethod
    def from_text(cls, text: str) -> 'PddlPlanStep':
        return parse_plan_step(text)


@dataclass(frozen=True)
class TranslatedPlanStep:
    """PDDL step translated into a primitive command and event target."""

    pddl_step: PddlPlanStep
    command: dict[str, Any]
    event_action: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            'pddl_step': {
                'name': self.pddl_step.name,
                'args': list(self.pddl_step.args),
                'kwargs': dict(self.pddl_step.kwargs),
                'raw': self.pddl_step.raw,
            },
            'command': self.command,
            'event_action': self.event_action,
        }


def parse_plan_step(text: str) -> PddlPlanStep:
    """Parse a simple PDDL plan step line.

    Supports plain forms such as ``move_shuttle right right_shuttle ...`` and
    common planner output forms such as ``0: (move_shuttle ...) [1.0]``.
    """

    raw = str(text or '').strip()
    cleaned = raw.split(';', 1)[0].strip()
    cleaned = re.sub(r'^\s*\d+(?:\.\d+)?\s*:\s*', '', cleaned)
    cleaned = re.sub(r'\s*\[[^\]]*\]\s*$', '', cleaned).strip()
    if cleaned.startswith('(') and cleaned.endswith(')'):
        cleaned = cleaned[1:-1].strip()
    if not cleaned:
        raise ValueError('empty PDDL plan step')

    tokens = shlex.split(cleaned)
    if not tokens:
        raise ValueError('empty PDDL plan step')

    args: list[str] = []
    kwargs: dict[str, str] = {}
    for token in tokens[1:]:
        if '=' in token:
            key, value = token.split('=', 1)
            kwargs[key.strip().lower()] = value.strip()
        else:
            args.append(_clean_symbol(token))
    return PddlPlanStep(
        name=_clean_symbol(tokens[0]).lower(),
        args=tuple(args),
        kwargs=kwargs,
        raw=raw,
    )


def parse_plan_text(text: str) -> list[PddlPlanStep]:
    steps = []
    for line in str(text or '').splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith(';'):
            continue
        steps.append(parse_plan_step(stripped))
    return steps


def translate_plan(steps: list[str | PddlPlanStep] | tuple[str | PddlPlanStep, ...]) -> list[TranslatedPlanStep]:
    return [translate_step(step) for step in steps]


def translate_step(step: str | PddlPlanStep) -> TranslatedPlanStep:
    parsed = parse_plan_step(step) if isinstance(step, str) else step
    if parsed.name == 'prepare_switches':
        command, event_action = _translate_prepare_switches(parsed)
    elif parsed.name == 'open_stoppers':
        command, event_action = _translate_open_stoppers(parsed)
    elif parsed.name == 'set_stoppers':
        command, event_action = _translate_set_stoppers(parsed)
    elif parsed.name in {'move_shuttle', 'move_shuttle_to_slot'}:
        command, event_action = _translate_move_shuttle(parsed)
    elif parsed.name == 'prepare_topology_route':
        command, event_action = _translate_topology_route(parsed)
    elif parsed.name == 'move_shuttle_from_segment_to_slot':
        command, event_action = _translate_segment_origin_move(parsed)
    elif parsed.name == 'begin_route_clearance':
        command, event_action = _translate_clearance_mode(parsed, begin=True)
    elif parsed.name == 'relocate_blocker_to_interior':
        command, event_action = _translate_interior_clearance(parsed)
    elif parsed.name == 'finish_route_clearance':
        command, event_action = _translate_clearance_mode(parsed, begin=False)
    elif parsed.name == 'stop_shuttle':
        command, event_action = _translate_stop_shuttle(parsed)
    elif parsed.name in {'finish_task', 'finish_candidate_task'}:
        command, event_action = _translate_finish_task(parsed)
    elif parsed.name == 'inspect_state':
        command, event_action = _translate_inspect_state(parsed)
    elif parsed.name == 'wait_for_clearance':
        command, event_action = _translate_wait_for_clearance(parsed)
    else:
        raise ValueError(f'unsupported Room 315 PDDL action {parsed.name!r}')
    return TranslatedPlanStep(
        pddl_step=parsed,
        command=command,
        event_action=event_action,
    )


def _translate_prepare_switches(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side = _side_from_args(step.args)
    state = _normalize_switch_value(step.kwargs.get('state') or step.kwargs.get('switch_state') or 'EXTERIOR')
    switch_name = _switch_target_from_step(step)
    switches = {switch_name: state} if switch_name else {'ALL': state}
    command = {
        'action': 'switches',
        'side': side,
        'switches': switches,
    }
    event_action = _blank_event_action(
        primitive='SET_SWITCHES',
        side=side,
        wait_condition='switch_state_match',
        target_id=switch_name or 'ALL_SWITCHES',
        reason='switch_update',
    )
    event_action['switch_mask'] = {
        name: 1 if not switch_name or name == switch_name else 0
        for name in DEVICE_NAMES
    }
    event_action['switch_values'] = {
        name: state if not switch_name or name == switch_name else 'UNCHANGED'
        for name in DEVICE_NAMES
    }
    return command, event_action


def _switch_target_from_step(step: PddlPlanStep) -> str:
    raw = (
        step.kwargs.get('switch')
        or step.kwargs.get('switch_name')
        or step.kwargs.get('target_switch')
    )
    text = str(raw or '').strip().upper()
    return text if text in DEVICE_NAMES else ''


def _translate_open_stoppers(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side = _side_from_args(step.args)
    command = {
        'action': 'stoppers',
        'side': side,
        'stoppers': {'ALL': '0'},
    }
    event_action = _blank_event_action(
        primitive='SET_STOPPERS',
        side=side,
        wait_condition='stopper_state_match',
        target_id='ALL_STOPPERS',
        reason='stopper_update',
    )
    event_action['stopper_mask'] = {name: 1 for name in DEVICE_NAMES}
    event_action['stopper_values'] = {name: 'open' for name in DEVICE_NAMES}
    return command, event_action


def _translate_set_stoppers(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side = _side_from_args(step.args)
    stopper = _stopper_from_args(step.args)
    state = _stopper_state_from_step(step)
    command_state = '1' if state == 'closed' else '0'
    if stopper == 'ALL':
        assignments = {'ALL': command_state}
        event_values = {name: state for name in DEVICE_NAMES}
        target_id = 'ALL_STOPPERS'
    elif state == 'closed':
        assignments = {'ALL': '0', stopper: '1'}
        event_values = {name: 'open' for name in DEVICE_NAMES}
        event_values[stopper] = 'closed'
        target_id = stopper
    else:
        assignments = {stopper: '0'}
        event_values = {name: 'UNCHANGED' for name in DEVICE_NAMES}
        event_values[stopper] = 'open'
        target_id = stopper

    command = {
        'action': 'stoppers',
        'side': side,
        'stoppers': assignments,
    }
    event_action = _blank_event_action(
        primitive='SET_STOPPERS',
        side=side,
        wait_condition='stopper_state_match',
        target_id=target_id,
        reason='stopper_update',
    )
    event_action['stopper_mask'] = {
        name: int(event_values[name] != 'UNCHANGED') for name in DEVICE_NAMES
    }
    event_action['stopper_values'] = event_values
    return command, event_action


def _translate_move_shuttle(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side, shuttle = _side_and_shuttle_from_args(step.args)
    speed = _float_kwarg(step.kwargs, 'speed', 'speed_mps', default=0.3)
    command = {
        'action': 'shuttle',
        'side': side,
        'shuttle': shuttle,
        'command': 'ON',
        'speed': speed,
    }
    target_stopper = _stopper_or_empty(
        step.kwargs.get('target_stopper') or step.kwargs.get('stopper_target')
    )
    if target_stopper:
        command['target_stopper'] = target_stopper
    event_action = _blank_event_action(
        primitive='SHUTTLE_ON',
        side=side,
        speed_mps=speed,
        wait_condition='shuttle_command_applied',
        target_id=f'{side}_shuttle',
        reason='shuttle_start',
    )
    event_action.update(_multi_shuttle_fields(side=side, shuttle=shuttle))
    return command, event_action


def _translate_topology_route(
    step: PddlPlanStep,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Preserve the route identity for executive topology expansion."""

    side, shuttle = _side_and_shuttle_from_args(step.args)
    if len(step.args) < 4:
        raise ValueError('prepare_topology_route requires shuttle, side, block, and slot')
    source_block = step.args[2]
    target_slot = step.args[3]
    command = {
        'action': 'topology_route',
        'side': side,
        'shuttle': shuttle,
        'source_block': source_block,
        'target_slot': target_slot,
        'deterministic_macro': 'authoritative_topology_switches_and_open_stoppers',
    }
    event_action = _blank_event_action(
        primitive='SET_SWITCHES',
        side=side,
        wait_condition='switch_state_match',
        target_id='ALL_SWITCHES',
        reason='switch_update',
    )
    shuttle_fields = _multi_shuttle_fields(side=side, shuttle=shuttle)
    # This event configures the complete switch group.  Preserve the shuttle
    # identity as audit metadata without changing the physical event target
    # from ALL_SWITCHES or weakening its route-reservation semantics.
    shuttle_fields.pop('target_id', None)
    shuttle_fields.pop('coordination_mode', None)
    event_action.update(shuttle_fields)
    event_action['coordination_mode'] = 'reservation_based_move'
    return command, event_action


def _translate_segment_origin_move(
    step: PddlPlanStep,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Translate a topology-preserving move without dropping PDDL endpoints."""

    side, shuttle = _side_and_shuttle_from_args(step.args)
    if len(step.args) < 5:
        raise ValueError(
            'move_shuttle_from_segment_to_slot requires shuttle, side, '
            'source block, target station, and target slot'
        )
    speed = _float_kwarg(step.kwargs, 'speed', 'speed_mps', default=0.3)
    command = {
        'action': 'shuttle',
        'side': side,
        'shuttle': shuttle,
        'command': 'ON',
        'speed': speed,
        'source_block': step.args[2],
        'target_station': step.args[3],
        'target_slot': step.args[4],
        'topology_route_move': True,
    }
    event_action = _blank_event_action(
        primitive='SHUTTLE_ON',
        side=side,
        speed_mps=speed,
        wait_condition='target_sensor_active',
        target_id=f'{side}_shuttle',
        reason='shuttle_start',
    )
    event_action.update(_multi_shuttle_fields(side=side, shuttle=shuttle))
    event_action['coordination_mode'] = 'reservation_based_move'
    return command, event_action


def _translate_interior_clearance(
    step: PddlPlanStep,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Translate the symbolic relocation into an executive-owned safe macro."""

    side, blocker = _side_and_shuttle_from_args(step.args)
    selected = next(
        (
            value
            for value in step.args
            if value != blocker
            and normalize_shuttle_ref(value, side=side) is not None
        ),
        '',
    )
    speed = _float_kwarg(step.kwargs, 'speed', 'speed_mps', default=0.3)
    command = {
        'action': 'clearance_relocation',
        'side': side,
        'shuttle': blocker,
        'selected_shuttle': selected,
        'command': 'ON',
        'speed': speed,
        'deterministic_macro': 'supervised_a3_interior_visual_stop',
    }
    event_action = _blank_event_action(
        primitive='SHUTTLE_ON',
        side=side,
        speed_mps=speed,
        wait_condition='accepted_visual_interior_pose_then_controller_stop',
        target_id=f'{side}_interior_clearance',
        reason='route_blocker_relocation',
    )
    event_action.update(_multi_shuttle_fields(side=side, shuttle=blocker))
    return command, event_action


def _translate_clearance_mode(
    step: PddlPlanStep,
    *,
    begin: bool,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Translate PDDL clearance phase boundaries for executive expansion."""

    side = _side_from_args(step.args)
    command = {
        'action': 'switches',
        'side': side,
        'switches': (
            {
                'A1': 'EXTERIOR',
                'A2': 'EXTERIOR',
                'A3': 'INTERIOR',
                'A4': 'INTERIOR',
            }
            if begin
            else {'ALL': 'EXTERIOR'}
        ),
        'deterministic_macro': (
            'hold_interior_route_for_all_blockers'
            if begin
            else 'restore_normal_route_once'
        ),
    }
    event_action = _blank_event_action(
        primitive='SET_SWITCHES',
        side=side,
        wait_condition='switch_state_match',
        target_id='ALL_SWITCHES',
        reason='switch_update',
    )
    event_action['coordination_mode'] = (
        'route_clearance_begin' if begin else 'route_clearance_finish'
    )
    return command, event_action


def _translate_stop_shuttle(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side, shuttle = _side_and_shuttle_from_args(step.args)
    command = {
        'action': 'shuttle',
        'side': side,
        'shuttle': shuttle,
        'command': 'OFF',
    }
    event_action = _blank_event_action(
        primitive='STOP_NOW',
        side=side,
        wait_condition='shuttle_command_applied',
        target_id=f'{side}_shuttle',
        reason='shuttle_stop',
    )
    event_action.update(_multi_shuttle_fields(side=side, shuttle=shuttle))
    return command, event_action


def _multi_shuttle_fields(*, side: str, shuttle: str) -> dict[str, Any]:
    spec = normalize_shuttle_ref(shuttle, side=side)
    if spec is None or shuttle == f'{side}_shuttle':
        shuttle_index = 0
        return {
            'shuttle_id': f'{"R" if side == "right" else "L"}{shuttle_index + 1}',
            'shuttle_index': shuttle_index,
            'target_id': f'{side}_shuttle_{shuttle_index + 1}',
            'coordination_mode': 'guarded_motion',
        }
    return {
        'shuttle_id': spec.short_id,
        'shuttle_index': spec.shuttle_index,
        'target_id': f'{side}_shuttle_{spec.shuttle_index + 1}',
        'coordination_mode': 'guarded_motion',
    }


def _translate_finish_task(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    if not step.args:
        raise ValueError('finish_task needs shuttle argument')
    shuttle = step.args[0]
    side = _infer_side(shuttle)
    station = step.args[1] if len(step.args) > 1 else ''
    command = {
        'action': 'DONE',
        'status': 'success',
        'shuttle': shuttle,
        'station': station,
    }
    event_action = _blank_event_action(
        primitive='DONE',
        side=side,
        wait_condition='terminal',
        target_id='terminal',
        reason='task_succeeded',
    )
    return command, event_action


def _translate_inspect_state(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    target = step.args[0] if step.args else 'room315_system'
    command = {
        'action': 'DONE',
        'status': 'success',
        'inspection_subject': target,
        'deterministic_macro': 'inspect_state',
    }
    event_action = _blank_event_action(
        primitive='DONE',
        side='right',
        wait_condition='terminal',
        target_id='terminal',
        reason='task_succeeded',
    )
    return command, event_action


def _translate_wait_for_clearance(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side, shuttle = _side_and_shuttle_from_args(step.args)
    command = {
        'action': 'shuttle',
        'side': side,
        'shuttle': shuttle,
        'command': 'OFF',
        'deterministic_macro': 'wait_for_clearance',
    }
    if len(step.args) > 1:
        command['clearance_target'] = step.args[-1]
    event_action = _blank_event_action(
        primitive='STOP_NOW',
        side=side,
        wait_condition='block_clearance',
        target_id=f'{side}_shuttle',
        reason='wait_for_block_clearance',
    )
    event_action.update(_multi_shuttle_fields(side=side, shuttle=shuttle))
    event_action['coordination_mode'] = 'wait_for_clearance'
    return command, event_action


def _blank_event_action(
    *,
    primitive: str,
    side: str,
    speed_mps: float = 0.0,
    wait_condition: str = 'none',
    target_id: str = 'none',
    reason: str = 'none',
) -> dict[str, Any]:
    return {
        'primitive': primitive,
        'side': side,
        'shuttle_id': '',
        'shuttle_index': -1,
        'switch_mask': {name: 0 for name in DEVICE_NAMES},
        'switch_values': {name: 'UNCHANGED' for name in DEVICE_NAMES},
        'stopper_mask': {name: 0 for name in DEVICE_NAMES},
        'stopper_values': {name: 'UNCHANGED' for name in DEVICE_NAMES},
        'speed_mps': round(float(speed_mps), 4),
        'wait_condition': wait_condition,
        'target_id': target_id,
        'reason': reason,
        'coordination_mode': 'normal',
    }


def _normalize_event_action(action: dict[str, Any]) -> dict[str, Any]:
    normalized = {
        'primitive': str(action.get('primitive') or 'WAIT').strip().upper(),
        'side': _normalize_side(action.get('side')),
        'shuttle_id': str(action.get('shuttle_id') or '').strip(),
        'shuttle_index': int(action.get('shuttle_index', -1)),
        'switch_mask': _device_map(action.get('switch_mask'), default=0),
        'switch_values': _device_map(action.get('switch_values'), default='UNCHANGED'),
        'stopper_mask': _device_map(action.get('stopper_mask'), default=0),
        'stopper_values': _device_map(action.get('stopper_values'), default='UNCHANGED'),
        'speed_mps': round(float(action.get('speed_mps') or 0.0), 4),
        'wait_condition': str(action.get('wait_condition') or 'none').strip(),
        'target_id': str(action.get('target_id') or 'none').strip(),
        'reason': str(action.get('reason') or 'none').strip(),
        'coordination_mode': str(action.get('coordination_mode') or 'normal').strip(),
    }
    if normalized['primitive'] not in PRIMITIVE_IDS:
        raise ValueError(f'unknown primitive {normalized["primitive"]!r}')
    if normalized['wait_condition'] not in WAIT_CONDITION_IDS:
        raise ValueError(f'unknown wait_condition {normalized["wait_condition"]!r}')
    if normalized['target_id'] not in TARGET_IDS:
        raise ValueError(f'unknown target_id {normalized["target_id"]!r}')
    if normalized['reason'] not in REASON_IDS:
        raise ValueError(f'unknown reason {normalized["reason"]!r}')
    for name in DEVICE_NAMES:
        if normalized['switch_mask'][name]:
            value = _normalize_switch_value(normalized['switch_values'][name])
            if value == 'UNCHANGED':
                raise ValueError(f'switch_mask_{name} selected but value is UNCHANGED')
            normalized['switch_values'][name] = value
        else:
            normalized['switch_values'][name] = 'UNCHANGED'
        if normalized['stopper_mask'][name]:
            value = _normalize_stopper_value(normalized['stopper_values'][name])
            if value == 'UNCHANGED':
                raise ValueError(f'stopper_mask_{name} selected but value is UNCHANGED')
            normalized['stopper_values'][name] = value
        else:
            normalized['stopper_values'][name] = 'UNCHANGED'
    if normalized['primitive'] in {'SHUTTLE_ON', 'STOP_NOW'} and normalized['shuttle_index'] < 0:
        normalized['shuttle_index'] = 0
        normalized['shuttle_id'] = f'{"R" if normalized["side"] == "right" else "L"}1'
    if normalized['primitive'] in {'SHUTTLE_ON', 'STOP_NOW'} and normalized['target_id'] in {
        'right_shuttle',
        'left_shuttle',
        'none',
    }:
        normalized['target_id'] = f'{normalized["side"]}_shuttle_{normalized["shuttle_index"] + 1}'
    return normalized


def _side_and_shuttle_from_args(args: tuple[str, ...]) -> tuple[str, str]:
    if not args:
        raise ValueError('shuttle action needs side and shuttle arguments')
    if _normalize_side_or_empty(args[0]):
        if len(args) < 2:
            raise ValueError('shuttle action needs shuttle argument after side')
        side = _normalize_side(args[0])
        shuttle = args[1]
        return side, shuttle
    shuttle = args[0]
    if len(args) > 1 and _normalize_side_or_empty(args[1]):
        return _normalize_side(args[1]), shuttle
    return _infer_side(shuttle), shuttle


def _side_from_args(args: tuple[str, ...]) -> str:
    for arg in args:
        side = _normalize_side_or_empty(arg)
        if side:
            return side
    for arg in args:
        side = _infer_side(arg, default='')
        if side:
            return side
    raise ValueError('PDDL step needs a right or left rail side')


def _stopper_from_args(args: tuple[str, ...]) -> str:
    for arg in args:
        stopper = _stopper_or_empty(arg)
        if stopper:
            return stopper
    raise ValueError('set_stoppers needs stopper target A1-A4 or ALL')


def _stopper_or_empty(value: Any) -> str:
    text = str(value or '').strip().upper()
    if text in {'ALL', *DEVICE_NAMES}:
        return text
    return ''


def _stopper_state_from_step(step: PddlPlanStep) -> str:
    for key in ('state', 'stopper_state', 'value'):
        if key in step.kwargs:
            return _normalize_stopper_value(step.kwargs[key])
    for arg in step.args:
        if _normalize_side_or_empty(arg) or _stopper_or_empty(arg):
            continue
        try:
            state = _normalize_stopper_value(arg)
        except ValueError:
            continue
        if state != 'UNCHANGED':
            return state
    raise ValueError('set_stoppers needs state open/closed')


def _infer_side(value: Any, default: str = 'right') -> str:
    text = str(value or '').casefold()
    if 'left' in text:
        return 'left'
    if 'right' in text:
        return 'right'
    return default


def _normalize_side(value: Any) -> str:
    side = _normalize_side_or_empty(value)
    if not side:
        raise ValueError(f'invalid rail side {value!r}; expected right or left')
    return side


def _normalize_side_or_empty(value: Any) -> str:
    text = str(value or '').strip().casefold()
    if text in {'right', 'r'}:
        return 'right'
    if text in {'left', 'l'}:
        return 'left'
    return ''


def _normalize_switch_value(value: Any) -> str:
    text = str(value or '').strip().upper()
    if text in {'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if text in {'I', 'INTERIOR'}:
        return 'INTERIOR'
    if text in {'', 'UNCHANGED', 'NONE'}:
        return 'UNCHANGED'
    raise ValueError(f'invalid switch value {value!r}')


def _normalize_stopper_value(value: Any) -> str:
    text = str(value or '').strip().lower()
    if text in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return 'open'
    if text in {'1', 'closed', 'close', 'stop', 'blocked', 'on', 'true'}:
        return 'closed'
    if text in {'', 'unchanged', 'none'}:
        return 'UNCHANGED'
    raise ValueError(f'invalid stopper value {value!r}')


def _float_kwarg(kwargs: dict[str, str], *names: str, default: float) -> float:
    for name in names:
        if name in kwargs:
            return round(float(kwargs[name]), 4)
    return round(float(default), 4)


def _clean_symbol(value: str) -> str:
    return str(value).strip().replace('-', '_')


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def main() -> None:
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description='Translate Room 315 PDDL plan steps to VLA primitive event commands.'
    )
    parser.add_argument(
        'plan_file',
        nargs='?',
        help='Optional text file containing one PDDL plan step per line. Reads stdin when omitted.',
    )
    args = parser.parse_args()
    text = sys.stdin.read() if not args.plan_file else open(args.plan_file, encoding='utf-8').read()
    rows = [row.as_dict() for row in translate_plan(parse_plan_text(text))]
    for row in rows:
        print(_json_dumps(row))


if __name__ == '__main__':
    main()
