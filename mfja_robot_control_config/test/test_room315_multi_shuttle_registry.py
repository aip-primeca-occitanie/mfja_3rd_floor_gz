#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_multi_shuttle.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_shuttle_count_accepts_zero_to_four_and_rejects_more():
    multi = _load_module()

    assert [multi.validate_shuttle_count(value) for value in range(5)] == [0, 1, 2, 3, 4]
    with pytest.raises(ValueError, match='0..4'):
        multi.validate_shuttle_count(5)


def test_stable_identity_mapping_for_each_side():
    multi = _load_module()

    right = multi.shuttle_specs_for_side('right', 4)
    left = multi.shuttle_specs_for_side('left', 4)

    assert [spec.shuttle_id for spec in right] == [
        'right_shuttle_1',
        'right_shuttle_2',
        'right_shuttle_3',
        'right_shuttle_4',
    ]
    assert [spec.short_id for spec in left] == ['L1', 'L2', 'L3', 'L4']
    assert right[1].gazebo_entity_name == 'room315_right_shuttle_2'
    assert left[2].shuttle_index == 2


def test_start_slot_parser_defaults_and_validates_duplicates():
    multi = _load_module()

    assert multi.parse_start_slots('', count=0) == []
    assert multi.parse_start_slots('', count=1) == ['2']
    assert multi.parse_start_slots('', count=4) == ['1', '2', '3', '4']
    assert multi.parse_start_slots('1,3', count=2) == ['1', '3']
    with pytest.raises(ValueError, match='Duplicate|duplicate'):
        multi.parse_start_slots('2,2', count=2)


def test_registry_resolves_short_stable_and_gazebo_names():
    multi = _load_module()
    registry = multi.ShuttleRegistry(right_count=4, left_count=3)

    assert registry.resolve('R2').gazebo_entity_name == 'room315_right_shuttle_2'
    assert registry.resolve('right_shuttle_2').short_id == 'R2'
    assert registry.resolve('room315_left_shuttle_3').short_id == 'L3'
    assert registry.resolve('L4') is None
    assert registry.is_multi_shuttle_side('right') is True
    assert registry.count_on_side('left') == 3


def test_fleet_command_validation_rejects_ambiguous_and_reserved_blocks():
    multi = _load_module()
    registry = multi.ShuttleRegistry(right_count=2, left_count=1)

    ok, reason = multi.validate_fleet_command(
        {'action': 'shuttle', 'side': 'right', 'command': 'ON'},
        registry=registry,
    )
    assert ok is False
    assert 'ambiguous' in reason

    state = multi.FleetSafetyState(
        block_occupancy={'right_block_2': 'R1'},
        block_reservations={'right_block_3': 'R1'},
        station_slot_targets={'right_slot_3': 'R1'},
    )
    ok, reason = multi.validate_fleet_command(
        {
            'action': 'shuttle',
            'side': 'right',
            'shuttle_id': 'R2',
            'command': 'ON',
            'next_block': 'right_block_2',
        },
        registry=registry,
        fleet_state=state,
    )
    assert ok is False
    assert 'occupied by R1' in reason

    ok, reason = multi.validate_fleet_command(
        {
            'action': 'shuttle',
            'side': 'right',
            'shuttle_id': 'R2',
            'command': 'ON',
            'next_block': 'right_block_3',
        },
        registry=registry,
        fleet_state=state,
    )
    assert ok is False
    assert 'reserved by R1' in reason

    ok, reason = multi.validate_fleet_command(
        {'action': 'emergency_stop', 'side': 'right'},
        registry=registry,
        fleet_state=state,
    )
    assert ok is True
    assert reason == ''
