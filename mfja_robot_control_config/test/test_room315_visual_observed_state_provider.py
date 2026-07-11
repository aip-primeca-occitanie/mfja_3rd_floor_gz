#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
PROVIDER_PATH = SCRIPT_DIR / 'room_315_visual_observed_state_provider.py'
CALIBRATION_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_vla'
    / 'visual_observed_state_calibration.yaml'
)


def _load_provider():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location(
        'room_315_visual_observed_state_provider',
        PROVIDER_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _camera_info(width=220, height=100):
    return {
        'width': width,
        'height': height,
        'k': [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0],
        'frame_id': 'overhead_right_rgbd_optical_frame',
    }


def _streams(width=220, height=100, depth_m=2.0):
    depth = [[depth_m for _ in range(width)] for _ in range(height)]
    rgb = [[[0, 0, 0] for _ in range(width)] for _ in range(height)]
    return {
        'overhead_right_rgbd': {
            'rgb': rgb,
            'depth': depth,
            'camera_info': _camera_info(width=width, height=height),
            'timestamp': 10.0,
        }
    }


def _fact(state, subject, predicate):
    matches = [
        fact
        for fact in state.fused_planner_state
        if fact.subject == subject and fact.predicate == predicate
    ]
    assert len(matches) == 1
    return matches[0]


def _visual_facts(state, subject, predicate):
    return [
        fact
        for fact in state.visual_model_inputs
        if fact.subject == subject and fact.predicate == predicate
    ]


def test_pixel_depth_to_rail_mapping_uses_camera_info_and_calibration():
    provider = _load_provider()
    calibration = provider.load_visual_calibration(CALIBRATION_PATH)
    point = provider.pixel_depth_to_room_point(
        (100.0, 50.0),
        2.0,
        _camera_info(),
        calibration['cameras']['overhead_right_rgbd'],
    )

    assert point == pytest.approx((1.0, 0.0, 2.0))
    projection = provider.rail_projection_from_room_point(
        point,
        calibration,
        side_hint='right',
    )

    assert projection.status == 'known'
    assert projection.side == 'right'
    assert projection.segment == 'A12E'
    assert projection.s_ratio == pytest.approx(0.5)
    assert projection.slot == '2'
    assert projection.block_id == 'right:A12E'
    assert projection.slot_id == 'right:slot:2'

    left_point = provider.pixel_depth_to_room_point(
        (100.0, 50.0),
        2.0,
        _camera_info(),
        calibration['cameras']['overhead_left_rgbd'],
    )
    left_projection = provider.rail_projection_from_room_point(
        left_point,
        calibration,
        side_hint='left',
    )
    assert left_point == pytest.approx((1.0, 1.0, 2.0))
    assert left_projection.side == 'left'
    assert left_projection.segment == 'A12E'
    assert left_projection.slot_id == 'left:slot:2'

    far_projection = provider.rail_projection_from_room_point(
        (1.0, 0.6, 2.0),
        calibration,
        side_hint='right',
    )
    assert far_projection.status == 'unknown'
    assert far_projection.reason == 'point_too_far_from_calibrated_rail'


def test_visual_provider_outputs_visual_facts_and_fuses_trusted_stoppers():
    provider = _load_provider()
    model_output = {
        'schema_version': 1,
        'timestamp': 10.0,
        'detections': [
            {
                'kind': 'shuttle',
                'id': 'det-r2',
                'camera': 'overhead_right_rgbd',
                'bbox': [95.0, 45.0, 10.0, 10.0],
                'identity': 'R2',
                'identity_confidence': 0.97,
                'loaded_state': 'loaded',
                'loaded_confidence': 0.93,
                'confidence': 0.94,
                'timestamp': 10.0,
            }
        ],
        'switches': [
            {
                'id': 'sw-a2',
                'camera': 'overhead_right_rgbd',
                'bbox': [94.0, 44.0, 12.0, 12.0],
                'side': 'right',
                'name': 'A2',
                'state': 'INTERIOR',
                'confidence': 0.91,
                'timestamp': 10.0,
            }
        ],
        'obstacles': [
            {
                'id': 'obs-box',
                'camera': 'overhead_right_rgbd',
                'bbox': [95.0, 45.0, 10.0, 10.0],
                'label': 'box',
                'confidence': 0.88,
                'timestamp': 10.0,
            }
        ],
    }
    trusted_status = {
        'timestamp': 10.0,
        'rails': {
            'right': {
                'switches': {},
                'stoppers': {'A2': '1'},
                'payloads': {},
                'active_position_sensors': [],
                'active_sensors': [],
            },
            'left': {
                'switches': {},
                'stoppers': {},
                'payloads': {},
                'active_position_sensors': [],
                'active_sensors': [],
            },
        },
    }

    state = provider.VisualObservedStateProvider(
        calibration_path=CALIBRATION_PATH,
        compact_model=provider.DeterministicFixtureCompactModel(model_output),
        trusted_status_snapshot=trusted_status,
    ).observe(rgbd_streams=_streams(), timestamp=10.0)

    assert all(fact.source == 'visual_model' for fact in state.visual_model_inputs)
    assert not [
        fact
        for fact in state.visual_model_inputs
        if ':stopper:' in fact.subject or 'stopper' in fact.predicate
    ]

    assert _fact(state, 'room315_right_shuttle_2', 'visual_bbox').value == {
        'camera': 'overhead_right_rgbd',
        'bbox_xywh': [95.0, 45.0, 10.0, 10.0],
    }
    assert _fact(state, 'room315_right_shuttle_2', 'identity').value == 'R2'
    assert _fact(state, 'room315_right_shuttle_2', 'rail_side').value == 'right'
    assert _fact(state, 'room315_right_shuttle_2', 'location_slot').value == 'right:slot:2'
    assert _fact(state, 'room315_right_shuttle_2', 'location_block').value == 'right:A12E'
    assert _fact(state, 'room315_right_shuttle_2', 'loaded').value is True

    switch_state = _fact(state, 'right:switch:A2', 'state')
    assert switch_state.value == 'INTERIOR'
    assert switch_state.metadata['selected_source'] == 'visual_model'

    stopper_state = _fact(state, 'right:stopper:A2', 'state')
    assert stopper_state.value == 'closed'
    assert stopper_state.metadata['selected_source'] == 'trusted_device'

    obstacle_evidence = _fact(state, 'right:obstacles', 'obstacle_evidence')
    assert obstacle_evidence.value[0]['id'] == 'obs-box'
    assert obstacle_evidence.value[0]['segment'] == 'A12E'


def test_low_confidence_stale_and_inconsistent_detections_become_unknown():
    provider = _load_provider()
    scene = {
        'schema_version': 1,
        'detections': [
            {
                'kind': 'shuttle',
                'id': 'r1-near-a1',
                'camera': 'overhead_right_rgbd',
                'bbox': [65.0, 45.0, 10.0, 10.0],
                'identity': 'R1',
                'identity_confidence': 0.96,
                'loaded_state': 'empty',
                'loaded_confidence': 0.9,
                'confidence': 0.94,
                'timestamp': 10.0,
            },
            {
                'kind': 'shuttle',
                'id': 'r1-near-a23',
                'camera': 'overhead_right_rgbd',
                'bbox': [145.0, 45.0, 10.0, 10.0],
                'identity': 'R1',
                'identity_confidence': 0.96,
                'loaded_state': 'empty',
                'loaded_confidence': 0.9,
                'confidence': 0.94,
                'timestamp': 10.0,
            },
            {
                'kind': 'shuttle',
                'id': 'r2-low-confidence',
                'camera': 'overhead_right_rgbd',
                'bbox': [95.0, 45.0, 10.0, 10.0],
                'identity': 'R2',
                'identity_confidence': 0.97,
                'loaded_state': 'loaded',
                'loaded_confidence': 0.95,
                'confidence': 0.2,
                'timestamp': 10.0,
            },
            {
                'kind': 'shuttle',
                'id': 'r3-stale',
                'camera': 'overhead_right_rgbd',
                'bbox': [195.0, 45.0, 10.0, 10.0],
                'identity': 'R3',
                'identity_confidence': 0.97,
                'loaded_state': 'loaded',
                'loaded_confidence': 0.95,
                'confidence': 0.9,
                'timestamp': 1.0,
            },
        ],
    }

    state = provider.VisualObservedStateProvider(
        calibration_path=CALIBRATION_PATH,
    ).observe(rgbd_streams=_streams(width=240), model_output=scene, timestamp=10.0)

    assert {fact.status for fact in _visual_facts(state, 'room315_right_shuttle_1', 'location_slot')} == {
        'unknown'
    }
    assert _fact(state, 'room315_right_shuttle_1', 'location_slot').status == 'unknown'
    assert _fact(state, 'room315_right_shuttle_2', 'present').status == 'unknown'
    assert _visual_facts(state, 'room315_right_shuttle_2', 'present')[0].metadata['reasons'] == [
        'low_confidence'
    ]
    assert _fact(state, 'room315_right_shuttle_3', 'present').status == 'unknown'
    assert 'stale_detection' in _visual_facts(state, 'room315_right_shuttle_3', 'present')[0].metadata['reasons']


def test_strict_json_adapter_rejects_privileged_commands_and_trackers():
    provider = _load_provider()
    adapter = provider.StrictJsonCompactModelAdapter()

    with pytest.raises(provider.VisualObservationError, match='strict JSON'):
        adapter.parse(json.dumps({'schema_version': 1}) + '\nextra prose')
    with pytest.raises(provider.VisualObservationError, match='forbidden key'):
        adapter.parse({
            'schema_version': 1,
            'detections': [
                {
                    'kind': 'shuttle',
                    'id': 'bad',
                    'camera': 'overhead_right_rgbd',
                    'bbox': [0.0, 0.0, 10.0, 10.0],
                    'identity': 'R1',
                    'confidence': 0.9,
                    'gazebo_pose': {'x': 1.0},
                }
            ],
        })
    with pytest.raises(provider.VisualObservationError, match='forbidden key'):
        adapter.parse({
            'schema_version': 1,
            'structured_tracker': {'tracks': []},
        })
    with pytest.raises(provider.VisualObservationError, match='forbidden key'):
        adapter.parse({
            'schema_version': 1,
            'stoppers': [{'name': 'A2', 'state': 'closed'}],
        })
    with pytest.raises(provider.VisualObservationError, match='forbidden key'):
        adapter.parse({
            'schema_version': 1,
            'detections': [
                {
                    'kind': 'shuttle',
                    'id': 'bad',
                    'camera': 'overhead_right_rgbd',
                    'bbox': [0.0, 0.0, 10.0, 10.0],
                    'identity': 'R1',
                    'confidence': 0.9,
                    'action_vector': [0.0] * 24,
                }
            ],
        })
