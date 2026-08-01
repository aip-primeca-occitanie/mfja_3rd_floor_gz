#!/usr/bin/env python3

import importlib.util
import math
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


def _left_topology(module):
    return module.load_rail_topology(
        KINEMATICS_DIR / 'rail_network_left.yaml',
        KINEMATICS_DIR / 'rail_devices_left.yaml',
        side='left',
    )


def _trace(route):
    return [
        (block.segment, block.start_s_ratio, block.end_s_ratio)
        for block in route.blocks
    ]


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


def test_route_blockers_use_physical_occupancy_interval_at_route_boundary():
    multi = _load_module()
    topology = _right_topology(multi)
    rails = {
        'right': {
            'shuttles': {
                'R1': {'segment': 'A12E', 's_ratio': 0.411866742},
                'R2': {
                    'segment': 'A12E',
                    'rail_position': {
                        'available': True,
                        's_ratio': 0.32,
                        'segment_length_m': 2.228,
                        'position_uncertainty_m': 0.01,
                    },
                },
            },
        },
    }

    blockers = multi.route_blockers_from_rails(
        rails,
        topology,
        source_slot='1',
        target_slot='3',
        selected_shuttle='R1',
    )

    assert [blocker.shuttle_id for blocker in blockers] == ['R2']
    assert blockers[0].reason == 'route_occupancy_interval_overlap'
    assert blockers[0].s_ratio == pytest.approx(0.32)
    assert blockers[0].occupancy_end_s_ratio > topology.slots['1'].s_ratio


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


def test_position_route_from_right_a34i_to_slot_1_has_exact_static_trace():
    multi = _load_module()

    route = multi.route_plan_from_position_to_slot(
        _right_topology(multi),
        'A34I',
        0.95,
        '1',
    )

    assert route.side == 'right'
    assert route.switch_states == {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'I'}
    assert [entry[0] for entry in _trace(route)] == [
        'A34I',
        'A4I',
        'A14',
        'A1E',
        'A12E',
    ]
    assert route.blocks[0].start_s_ratio == pytest.approx(0.95)
    assert route.blocks[-1].end_s_ratio == pytest.approx(0.411866742)


def test_position_route_left_mirror_uses_left_switch_numbering_and_slots():
    multi = _load_module()

    route = multi.route_plan_from_position_to_slot(
        _left_topology(multi),
        'A12I',
        0.95,
        'slot_1',
    )

    assert route.side == 'left'
    assert route.switch_states == {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'I'}
    assert [entry[0] for entry in _trace(route)] == [
        'A12I',
        'A2I',
        'A23',
        'A3E',
        'A34E',
    ]
    assert route.blocks[0].start_s_ratio == pytest.approx(0.95)
    assert route.blocks[-1].end_s_ratio == pytest.approx(0.428330934)


def test_position_route_on_same_segment_moves_forward_or_wraps_as_required():
    multi = _load_module()
    topology = _right_topology(multi)

    forward = multi.route_plan_from_position_to_slot(topology, 'A12E', 0.2, '1')
    wrapped = multi.route_plan_from_position_to_slot(topology, 'A12E', 0.8, '1')

    assert forward.switch_states == {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'}
    assert _trace(forward) == pytest.approx([
        ('A12E', 0.2, 0.411866742),
    ])
    assert wrapped.switch_states == {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'}
    assert [entry[0] for entry in _trace(wrapped)] == [
        'A12E',
        'A2E',
        'A23',
        'A3E',
        'A34E',
        'A4E',
        'A14',
        'A1E',
        'A12E',
    ]
    assert wrapped.blocks[0].start_s_ratio == pytest.approx(0.8)
    assert wrapped.blocks[-1].end_s_ratio == pytest.approx(0.411866742)


@pytest.mark.parametrize(
    ('segment', 's_ratio'),
    [
        ('FALLING', 0.5),
        ('NOT_A_SEGMENT', 0.5),
        ('A34I', -0.01),
        ('A34I', 1.01),
        ('A34I', float('nan')),
    ],
)
def test_position_route_rejects_invalid_observed_positions(segment, s_ratio):
    multi = _load_module()

    with pytest.raises(ValueError):
        multi.route_plan_from_position_to_slot(
            _right_topology(multi),
            segment,
            s_ratio,
            '1',
        )


def test_position_route_rejects_path_requiring_conflicting_static_switch_states():
    multi = _load_module()
    topology = multi.RailTopology(
        side='right',
        routing_table={
            'START': {
                'type': 'switch_select',
                'switch': 'A1',
                'by_state': {'E': 'FALLING', 'I': 'MIDDLE'},
            },
            'MIDDLE': {
                'type': 'switch_guard',
                'switch': 'A1',
                'by_state': {'E': 'TARGET', 'I': 'FALLING'},
            },
            'TARGET': {'type': 'fixed', 'next_segment': 'START'},
        },
        fixed_transitions={},
        slots={
            '1': multi.RailSlotLocation(slot='1', segment='TARGET', s_ratio=0.5),
        },
    )

    with pytest.raises(ValueError, match='conflicting switch states'):
        multi.route_plan_from_position_to_slot(topology, 'START', 0.1, '1')


@pytest.mark.parametrize('side', ('right', 'left'))
@pytest.mark.parametrize('source_s_ratio', (0.0, 0.5, 1.0))
def test_every_authoritative_position_has_a_deterministic_safe_slot_route(
    side,
    source_s_ratio,
):
    multi = _load_module()
    topology = _right_topology(multi) if side == 'right' else _left_topology(multi)
    segments = sorted(multi._topology_segment_names(topology))

    assert len(segments) == 14
    assert set(segments) == {
        'A1E',
        'A1I',
        'A2E',
        'A2I',
        'A3E',
        'A3I',
        'A4E',
        'A4I',
        'A12E',
        'A12I',
        'A14',
        'A23',
        'A34E',
        'A34I',
    }

    for source_segment in segments:
        for target_slot, target in sorted(topology.slots.items()):
            route = multi.route_plan_from_position_to_slot(
                topology,
                source_segment,
                source_s_ratio,
                target_slot,
            )
            repeated = multi.route_plan_from_position_to_slot(
                topology,
                source_segment,
                source_s_ratio,
                target_slot,
            )

            assert route == repeated
            assert route.side == side
            assert 1 <= len(route.blocks) <= 32
            assert route.blocks[0].segment == source_segment
            assert route.blocks[0].start_s_ratio == pytest.approx(source_s_ratio)
            assert route.blocks[-1].segment == target.segment
            assert route.blocks[-1].end_s_ratio == pytest.approx(target.s_ratio)
            assert route.switch_states.keys() == set(multi.DEVICE_NAMES)
            assert set(route.switch_states.values()) <= {'E', 'I'}

            for block in route.blocks:
                assert block.side == side
                assert block.segment != 'FALLING'
                assert block.segment in segments
                assert math.isfinite(block.start_s_ratio)
                assert math.isfinite(block.end_s_ratio)
                assert 0.0 <= block.start_s_ratio <= block.end_s_ratio <= 1.0

            for current, successor in zip(route.blocks, route.blocks[1:]):
                assert current.end_s_ratio == pytest.approx(1.0)
                assert successor.start_s_ratio == pytest.approx(0.0)
                assert multi._next_static_route_segment(
                    topology,
                    current.segment,
                    route.switch_states,
                ) == successor.segment
