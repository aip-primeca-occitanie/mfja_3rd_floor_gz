#!/usr/bin/env python3
"""Translate Room 315 PDDL plan steps into VLA event-level commands.

This module is an expert/data-generation bridge. It does not call PlanSys, does
not execute ROS commands, and does not define learned-model inputs. Symbolic PDDL
state may be used here to produce scenario events, but model_input remains owned
by the dataset recorder and stays limited to language, overhead_images, and
last_command.
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

from room_315_multi_shuttle import ACTION_SCHEMA_VERSION
from room_315_multi_shuttle import ACTION_VECTOR_V3_FIELDS
from room_315_multi_shuttle import EVENT_ACTION_V3_FIELDS
from room_315_multi_shuttle import encode_action_v3
from room_315_multi_shuttle import normalize_shuttle_ref


SIDES = ('right', 'left')
DEVICE_NAMES = ('A1', 'A2', 'A3', 'A4')

PRIMITIVE_IDS = {
    'WAIT': 0,
    'DONE': 1,
    'SET_SWITCHES': 2,
    'SET_STOPPERS': 3,
    'SHUTTLE_ON': 4,
    'STOP_NOW': 5,
    'EMERGENCY_STOP': 6,
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
for _side in SIDES:
    for _index in range(1, 5):
        TARGET_IDS[f'{_side}_shuttle_{_index}'] = len(TARGET_IDS)
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

ACTION_VECTOR_FIELDS = list(ACTION_VECTOR_V3_FIELDS)
EVENT_ACTION_FIELDS = list(EVENT_ACTION_V3_FIELDS)


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
    """PDDL step translated into VLA primitive command and event target."""

    pddl_step: PddlPlanStep
    command: dict[str, Any]
    event_action: dict[str, Any]
    action_vector: list[float]

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
            'action_vector': self.action_vector,
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
    elif parsed.name == 'move_shuttle':
        command, event_action = _translate_move_shuttle(parsed)
    elif parsed.name == 'stop_shuttle':
        command, event_action = _translate_stop_shuttle(parsed)
    elif parsed.name == 'finish_task':
        command, event_action = _translate_finish_task(parsed)
    else:
        raise ValueError(f'unsupported Room 315 PDDL action {parsed.name!r}')
    return TranslatedPlanStep(
        pddl_step=parsed,
        command=command,
        event_action=event_action,
        action_vector=encode_event_action(event_action),
    )


def encode_event_action(action: dict[str, Any]) -> list[float]:
    """Encode a Room 315 event action into the canonical schema-v3 vector."""

    return encode_action_v3(_normalize_event_action(action))


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _translate_prepare_switches(step: PddlPlanStep) -> tuple[dict[str, Any], dict[str, Any]]:
    side = _side_from_args(step.args)
    state = _normalize_switch_value(step.kwargs.get('state') or step.kwargs.get('switch_state') or 'EXTERIOR')
    command = {
        'action': 'switches',
        'side': side,
        'switches': {'ALL': state},
    }
    event_action = _blank_event_action(
        primitive='SET_SWITCHES',
        side=side,
        wait_condition='switch_state_match',
        target_id='ALL_SWITCHES',
        reason='switch_update',
    )
    event_action['switch_mask'] = {name: 1 for name in DEVICE_NAMES}
    event_action['switch_values'] = {name: state for name in DEVICE_NAMES}
    return command, event_action


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
            'action_vector_schema_version': ACTION_SCHEMA_VERSION,
            'shuttle_id': f'{"R" if side == "right" else "L"}{shuttle_index + 1}',
            'shuttle_index': shuttle_index,
            'target_id': f'{side}_shuttle_{shuttle_index + 1}',
            'coordination_mode': 'guarded_motion',
        }
    return {
        'action_vector_schema_version': ACTION_SCHEMA_VERSION,
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
        'action_vector_schema_version': ACTION_SCHEMA_VERSION,
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
        'action_vector_schema_version': ACTION_SCHEMA_VERSION,
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


def _device_map(raw: Any, *, default: Any) -> dict[str, Any]:
    values = {name: default for name in DEVICE_NAMES}
    if isinstance(raw, dict):
        for name in DEVICE_NAMES:
            if name in raw:
                values[name] = raw[name]
    return values


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
