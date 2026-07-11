#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
MULTI_PATH = SCRIPT_DIR / 'room_315_multi_shuttle.py'
SUPERVISOR_PATH = SCRIPT_DIR / 'room_315_vla_supervisor.py'


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
        'left': {'shuttles': {}, 'switches': {}, 'stoppers': {}, 'active_position_sensors': []},
    }


def test_fleet_state_from_rails_marks_occupied_blocks_by_shuttle_id():
    multi = _load_module('room_315_multi_shuttle', MULTI_PATH)

    state = multi.fleet_safety_state_from_rails(_rails())

    assert state.block_occupancy['right:A12E'] == 'R1'
    assert state.block_occupancy['right:A23E'] == 'R2'
    assert state.shuttle_blocks['R1'] == 'right:A12E'


def test_supervisor_rejects_motion_into_occupied_block_from_runtime_rails():
    supervisor = _load_module('room_315_vla_supervisor', SUPERVISOR_PATH)

    decision = supervisor._decode_room315_vla_action(
        {
            'action': 'shuttle',
            'side': 'right',
            'shuttle_id': 'R2',
            'command': 'ON',
            'next_block': 'A12E',
        },
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
    assert 'occupied by R1' in decision['reason']


def test_supervisor_rejects_motion_into_reserved_block():
    supervisor = _load_module('room_315_vla_supervisor', SUPERVISOR_PATH)

    decision = supervisor._decode_room315_vla_action(
        {
            'action': 'shuttle',
            'side': 'right',
            'shuttle_id': 'R2',
            'command': 'ON',
            'next_block': 'A34E',
        },
        rails=_rails(),
        emergency_stop=False,
        active_tasks={},
        slot_sensor_by_side={'right': {}, 'left': {}},
        default_shuttle_name_by_side={
            'right': 'room315_right_shuttle_1',
            'left': 'room315_left_shuttle_1',
        },
        block_reservations={'right:A34E': 'R1'},
    )

    assert decision['accepted'] is False
    assert 'reserved by R1' in decision['reason']


def test_fleet_validator_detects_deadlock_with_deterministic_tie_breaking():
    multi = _load_module('room_315_multi_shuttle', MULTI_PATH)
    registry = multi.ShuttleRegistry(right_count=2, left_count=0)
    state = multi.FleetSafetyState(
        block_occupancy={
            'right:A12E': 'R1',
            'right:A23E': 'R2',
        },
        block_reservations={
            'right:A12E': 'R2',
            'right:A23E': 'R1',
        },
        station_slot_targets={},
        min_headway_blocks=1,
    )

    ok, reason = multi.validate_fleet_command(
        {
            'action': 'shuttle',
            'side': 'right',
            'shuttle_id': 'R2',
            'command': 'ON',
            'next_block': 'A12E',
        },
        registry=registry,
        fleet_state=state,
    )

    assert ok is False
    assert 'deadlock detected' in reason
    assert 'priority to R1' in reason
