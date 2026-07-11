#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
PROVIDER_PATH = SCRIPT_DIR / 'room_315_observed_state_provider.py'


def _load_provider():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_observed_state_provider', PROVIDER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _fact(state, subject, predicate):
    matches = [
        fact
        for fact in state.fused_planner_state
        if fact.subject == subject and fact.predicate == predicate
    ]
    assert len(matches) == 1
    return matches[0]


def _room315_status_4x4():
    rails = {}
    for side, suffix in (('right', 'R'), ('left', 'L')):
        shuttles = {}
        active_position_sensors = []
        payloads = {}
        for index in range(1, 5):
            entity = f'room315_{side}_shuttle_{index}'
            short_id = f'{suffix}{index}'
            segment = ('A1E', 'A12E', 'A23', 'A34E')[index - 1]
            shuttles[entity] = {
                'mode': 'STOPPED',
                'segment': segment,
                's': 0.2 + index * 0.1,
                'x': index,
                'y': index + 1,
                'z': 0.3,
                'yaw': 0.0,
                'speed': 0.0,
            }
            active_position_sensors.append({
                'name': f'DZI{index}{suffix}',
                'type': 'slot',
                'shuttle': entity,
                'segment': segment,
                's_ratio': 0.5,
            })
            payloads[entity] = {
                'shuttle_id': f'{side}_shuttle_{index}',
                'short_id': short_id,
                'side': side,
                'shuttle_index': index - 1,
                'entity_name': entity,
                'loaded': index % 2 == 1,
                'payload_type': 'box' if index % 2 == 1 else 'none',
                'model_input_exposure': 'excluded',
            }
        rails[side] = {
            'shuttles': shuttles,
            'switches': {'A1': 'E', 'A2': 'I', 'A3': 'EXTERIOR', 'A4': 'INTERIOR'},
            'stoppers': {'A1': '0', 'A2': '1', 'A3': 'open', 'A4': 'closed'},
            'payloads': payloads,
            'active_position_sensors': active_position_sensors,
            'active_sensors': [],
            'obstacles': {'test_marker': True},
        }
    return {'rails': rails, 'timestamp': 100.0}


def test_oracle_provider_represents_4_plus_4_fleet_and_labels_truth_oracle_only():
    provider = _load_provider()
    state = provider.OracleObservedStateProvider(
        _room315_status_4x4(),
        stale_after_s=5.0,
    ).observe(timestamp=101.0)

    assert state.visual_model_inputs == []
    present = [fact for fact in state.fused_planner_state if fact.predicate == 'present']
    assert len(present) == 8
    assert {fact.source for fact in present} == {'oracle'}
    assert all(fact.status == 'known' for fact in present)
    assert all(fact.metadata['oracle_only'] is True for fact in state.fused_planner_state)
    assert all(fact.metadata['model_input_exposure'] == 'excluded' for fact in state.fused_planner_state)

    assert _fact(state, 'room315_right_shuttle_3', 'location_slot').value == 'right:slot:3'
    assert _fact(state, 'room315_left_shuttle_4', 'location_block').value == 'left:A34E'
    assert _fact(state, 'right:slot:2', 'occupancy').value == {
        'occupied': True,
        'shuttle': 'room315_right_shuttle_2',
        'sensor': 'DZI2R',
    }
    assert _fact(state, 'room315_right_shuttle_1', 'loaded').value is True
    assert _fact(state, 'room315_right_shuttle_2', 'loaded').value is False
    assert _fact(state, 'right:switch:A2', 'state').value == 'INTERIOR'
    assert _fact(state, 'right:stopper:A2', 'state').value == 'closed'
    assert _fact(state, 'right:obstacles', 'present_obstacles').value == ['test_marker']


def test_fused_provider_keeps_visual_inputs_separate_and_does_not_emit_rail_commands():
    provider = _load_provider()
    visual_fact = provider.ObservedFact(
        fact_id='visual-r1-slot',
        subject='room315_right_shuttle_1',
        predicate='location_slot',
        value='right:slot:1',
        source='visual_model',
        timestamp=10.0,
        confidence=0.62,
        status='known',
    )
    trusted = {
        'rails': {
            'right': {
                'switches': {'A1': 'E'},
                'stoppers': {'A1': '0'},
                'payloads': {},
                'active_position_sensors': [],
                'active_sensors': [],
            },
            'left': {
                'switches': {},
                'stoppers': {},
            },
        },
        'timestamp': 10.0,
    }

    state = provider.FusedObservedStateProvider(
        trusted,
        visual_facts=[visual_fact],
        stale_after_s=5.0,
    ).observe(timestamp=10.5)

    assert state.visual_model_inputs == [visual_fact]
    assert all(fact.source != 'visual_model' for fact in state.fused_planner_state)
    assert all(fact.source != 'oracle' for fact in state.fused_planner_state)
    assert _fact(state, 'room315_right_shuttle_1', 'location_slot').value == 'right:slot:1'
    assert _fact(state, 'room315_right_shuttle_1', 'location_slot').source == 'state_fuser'


def test_fused_provider_uses_trusted_device_priority_and_marks_contradictions():
    provider = _load_provider()
    visual_fact = provider.ObservedFact(
        fact_id='visual-r1-slot',
        subject='room315_right_shuttle_1',
        predicate='location_slot',
        value='right:slot:3',
        source='visual_model',
        timestamp=20.0,
        confidence=0.9,
        status='known',
    )
    trusted = {
        'rails': {
            'right': {
                'switches': {},
                'stoppers': {},
                'payloads': {
                    'room315_right_shuttle_1': {
                        'loaded': True,
                        'model_input_exposure': 'excluded',
                    }
                },
                'active_position_sensors': [
                    {
                        'name': 'DZI2R',
                        'type': 'slot',
                        'shuttle': 'room315_right_shuttle_1',
                        'segment': 'A12E',
                        's_ratio': 0.5,
                    }
                ],
                'active_sensors': [],
            },
            'left': {'switches': {}, 'stoppers': {}, 'active_position_sensors': []},
        },
        'timestamp': 20.0,
    }

    state = provider.FusedObservedStateProvider(
        trusted,
        visual_facts=[visual_fact],
        stale_after_s=5.0,
    ).observe(timestamp=20.5)

    location = _fact(state, 'room315_right_shuttle_1', 'location_slot')
    assert location.value == 'right:slot:2'
    assert location.status == 'conflicting'
    assert location.metadata['selected_source'] == 'trusted_device'
    assert location.metadata['source_priority'][:2] == ['trusted_device', 'oracle']
    assert {item['source'] for item in location.metadata['conflicts']} == {
        'trusted_device',
        'visual_model',
    }
    assert _fact(state, 'room315_right_shuttle_1', 'loaded').value is True
    assert _fact(state, 'room315_left_shuttle_4', 'loaded').status == 'unknown'


def test_provider_marks_stale_observations_and_preserves_unknown_values():
    provider = _load_provider()
    status = {
        'rails': {
            'right': {
                'shuttles': {
                    'room315_right_shuttle_1': {'mode': 'STOPPED', 'segment': 'A12E'}
                },
                'switches': {'A1': 'E'},
                'stoppers': {'A1': '0'},
                'payloads': {},
                'active_position_sensors': [
                    {'name': 'DZI1R', 'shuttle': 'room315_right_shuttle_1', 'segment': 'A12E'}
                ],
                'active_sensors': [],
            },
            'left': {},
        },
    }

    state = provider.OracleObservedStateProvider(
        status,
        source_timestamps={'oracle': 1.0},
        stale_after_s=2.0,
    ).observe(timestamp=10.0)

    assert _fact(state, 'oracle:snapshot', 'freshness').status == 'stale'
    assert _fact(state, 'room315_right_shuttle_1', 'present').status == 'stale'
    assert _fact(state, 'room315_left_shuttle_4', 'present').status == 'stale'

    fresh_state = provider.FusedObservedStateProvider(
        {'rails': {'right': {}, 'left': {}}},
        stale_after_s=5.0,
    ).observe(timestamp=2.0)
    assert _fact(fresh_state, 'room315_left_shuttle_4', 'present').status == 'unknown'
    assert _fact(fresh_state, 'left:slot:4', 'occupancy').status == 'unknown'
