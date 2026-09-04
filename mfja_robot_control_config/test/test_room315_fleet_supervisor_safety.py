#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
SUPERVISOR_PATH = SCRIPT_DIR / 'room_315_rail_safety_supervisor.py'


def _load_module(name, path):
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _rails():
    return {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {'mode': 'STOPPED', 'segment': 'A12E'},
                'room315_right_shuttle_2': {'mode': 'STOPPED', 'segment': 'A23E'},
            },
            'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
            'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
            'active_position_sensors': [],
        },
        'left': {
            'shuttles': {'room315_left_shuttle_1': {'mode': 'STOPPED', 'segment': 'A12E'}},
            'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
            'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
            'active_position_sensors': [],
        },
    }


def _decode(module, command):
    return module._decode_room315_primitive_command(
        command,
        rails=_rails(),
        emergency_stop=False,
        active_tasks={},
        slot_sensor_by_side={'right': {}, 'left': {}},
        default_shuttle_name_by_side={
            'right': 'room315_right_shuttle_1',
            'left': 'room315_left_shuttle_1',
        },
    )


def test_supervisor_rejects_ambiguous_multi_shuttle_motion():
    supervisor = _load_module('room_315_rail_safety_supervisor', SUPERVISOR_PATH)

    decision = _decode(supervisor, {'action': 'shuttle', 'side': 'right', 'command': 'ON'})

    assert decision['accepted'] is False
    assert 'ambiguous' in decision['reason']


def test_supervisor_accepts_explicit_short_shuttle_id():
    supervisor = _load_module('room_315_rail_safety_supervisor', SUPERVISOR_PATH)

    decision = _decode(
        supervisor,
        {'action': 'shuttle', 'side': 'right', 'shuttle_id': 'R2', 'command': 'ON'},
    )

    assert decision['accepted'] is True
    assert decision['corrected_action']['shuttle'] == 'room315_right_shuttle_2'
    assert decision['corrected_action']['command'] == 'ON'


def test_supervisor_allows_global_emergency_stop_even_with_multiple_shuttles():
    supervisor = _load_module('room_315_rail_safety_supervisor', SUPERVISOR_PATH)

    decision = _decode(supervisor, {'action': 'emergency_stop'})

    assert decision['accepted'] is True
    assert decision['corrected_action']['action'] == 'emergency_stop'


def test_supervisor_structured_command_targets_specific_shuttle():
    supervisor = _load_module('room_315_rail_safety_supervisor', SUPERVISOR_PATH)

    decision = _decode(
        supervisor,
        {
            'action': 'shuttle',
            'shuttle_id': 'R2',
            'command': 'ON',
            'speed': 0.25,
        },
    )

    assert decision['accepted'] is True
    assert decision['corrected_action']['shuttle'] == 'room315_right_shuttle_2'
    assert decision['corrected_action']['speed'] == 0.25


def test_supervisor_rejects_removed_numeric_action_vector_payload():
    supervisor = _load_module('room_315_rail_safety_supervisor', SUPERVISOR_PATH)

    decision = supervisor._decode_room315_primitive_command(
        [0.0] * 24,
        rails=_rails(),
        emergency_stop=False,
        active_tasks={},
        slot_sensor_by_side={'right': {}, 'left': {}},
        default_shuttle_name_by_side={
            'right': 'room315_right_shuttle_1',
            'left': 'room315_left_shuttle_1',
        },
    )

    assert decision['accepted'] is False
    assert 'removed action_vector commands are not supported' in decision['reason']


def test_supervisor_rejects_removed_embedded_action_vector_payload():
    supervisor = _load_module('room_315_rail_safety_supervisor', SUPERVISOR_PATH)

    decision = _decode(
        supervisor,
        {
            'action': 'shuttle',
            'shuttle_id': 'R2',
            'command': 'ON',
            'speed': 0.25,
            'action_vector': [0.0] * 24,
        },
    )

    assert decision['accepted'] is False
    assert 'removed action_vector commands are not supported' in decision['reason']
