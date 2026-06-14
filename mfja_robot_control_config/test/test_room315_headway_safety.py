#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
MULTI_PATH = SCRIPT_DIR / 'room_315_multi_shuttle.py'


def _load_multi():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', MULTI_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_headway_violation_rejects_multi_shuttle_motion():
    multi = _load_multi()
    registry = multi.ShuttleRegistry(right_count=2, left_count=0)
    state = multi.FleetSafetyState(
        block_occupancy={},
        block_reservations={},
        station_slot_targets={},
        min_headway_blocks=2,
    )

    ok, reason = multi.validate_fleet_command(
        {
            'action': 'shuttle',
            'side': 'right',
            'shuttle_id': 'R2',
            'command': 'ON',
            'next_block': 'A23E',
            'headway_blocks_ahead': 1,
        },
        registry=registry,
        fleet_state=state,
    )

    assert ok is False
    assert 'headway violation' in reason


def test_emergency_stop_ignores_headway_state():
    multi = _load_multi()
    registry = multi.ShuttleRegistry(right_count=2, left_count=0)
    state = multi.FleetSafetyState(
        block_occupancy={'right:A23E': 'R1'},
        block_reservations={'right:A34E': 'R1'},
        station_slot_targets={'right:slot:3': 'R1'},
        min_headway_blocks=3,
    )

    ok, reason = multi.validate_fleet_command(
        {'action': 'emergency_stop'},
        registry=registry,
        fleet_state=state,
    )

    assert ok is True
    assert reason == ''
