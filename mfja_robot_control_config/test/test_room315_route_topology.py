#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_multi_shuttle.py'
KINEMATICS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_kinematics'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _right_topology(module):
    return module.load_rail_topology(
        KINEMATICS_DIR / 'rail_network_right.yaml',
        KINEMATICS_DIR / 'rail_devices_right.yaml',
        side='right',
    )


def test_right_slot_route_uses_directed_topology_intervals():
    multi = _load_module()
    topology = _right_topology(multi)

    blocks = multi.route_blocks_between_slots(topology, '1', '3')

    assert [block.segment for block in blocks] == ['A12E', 'A2E', 'A23', 'A3E', 'A34E']
    assert blocks[0].start_s_ratio == pytest.approx(0.411866742)
    assert blocks[0].end_s_ratio == pytest.approx(1.0)
    assert blocks[-1].start_s_ratio == pytest.approx(0.0)
    assert blocks[-1].end_s_ratio == pytest.approx(0.447469343)


def test_route_blockers_detect_shuttle_on_intermediate_segment():
    multi = _load_module()
    topology = _right_topology(multi)
    rails = {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {'segment': 'A12E', 's_ratio': 0.411866742},
                'room315_right_shuttle_2': {'segment': 'A23', 's_ratio': 0.5},
            }
        }
    }

    blockers = multi.route_blockers_from_rails(
        rails,
        topology,
        source_slot='1',
        target_slot='3',
        selected_shuttle='R1',
    )

    assert [blocker.shuttle_id for blocker in blockers] == ['R2']
    assert blockers[0].block_id == 'right:A23'
    assert blockers[0].reason == 'route_segment_overlap'


def test_route_blockers_ignore_known_shuttle_behind_source_on_same_segment():
    multi = _load_module()
    topology = _right_topology(multi)
    rails = {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {'segment': 'A12E', 's_ratio': 0.411866742},
                'room315_right_shuttle_2': {'segment': 'A12E', 's_ratio': 0.2},
            }
        }
    }

    blockers = multi.route_blockers_from_rails(
        rails,
        topology,
        source_slot='1',
        target_slot='3',
        selected_shuttle='room315_right_shuttle_1',
    )

    assert blockers == []


def test_route_blockers_detect_known_shuttle_ahead_on_source_segment():
    multi = _load_module()
    topology = _right_topology(multi)
    rails = {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {'segment': 'A12E', 's_ratio': 0.411866742},
                'room315_right_shuttle_2': {'segment': 'A12E', 's_ratio': 0.7},
            }
        }
    }

    blockers = multi.route_blockers_from_rails(
        rails,
        topology,
        source_slot='1',
        target_slot='3',
        selected_shuttle='right_shuttle_1',
    )

    assert [blocker.shuttle_id for blocker in blockers] == ['R2']
    assert blockers[0].block_id == 'right:A12E'


def test_route_blockers_treat_unknown_position_on_route_segment_as_blocking():
    multi = _load_module()
    topology = _right_topology(multi)
    rails = {
        'right': {
            'shuttles': {
                'room315_right_shuttle_1': {'segment': 'A12E', 's_ratio': 0.411866742},
                'room315_right_shuttle_2': {'segment': 'A23'},
            }
        }
    }

    blockers = multi.route_blockers_from_rails(
        rails,
        topology,
        source_slot='1',
        target_slot='3',
        selected_shuttle='R1',
    )

    assert [blocker.shuttle_id for blocker in blockers] == ['R2']
    assert blockers[0].s_ratio is None
    assert blockers[0].reason == 'route_segment_overlap_unknown_position'
