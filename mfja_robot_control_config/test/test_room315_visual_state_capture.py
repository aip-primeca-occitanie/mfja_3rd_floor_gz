#!/usr/bin/env python3

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import yaml
import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
CONFIG_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_visual_state'
    / 'training_scenarios.yaml'
)
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_scenario_generator as scenario_generator
import room_315_visual_state_capture as capture


def _right_single_scenario():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    scenarios = scenario_generator.generate_scenarios(config, count=20, seed=315)
    return next(
        scenario
        for scenario in scenarios
        if scenario['scene_type'] == 'single'
        and scenario['scene']['rails']['right']['shuttles']
    )


def _matching_snapshot(scenario):
    loaded = (
        scenario['scene']['rails']['right']['shuttles'][0]['loaded_state']
        == 'loaded'
    )
    snapshot = capture.CaptureSnapshot()
    snapshot.images = {
        'left_rail_rgb': object(),
        'right_rail_rgb': object(),
    }
    snapshot.image_stamps = {
        'left_rail_rgb': 10.0,
        'right_rail_rgb': 10.01,
    }
    snapshot.shuttle_updates = {'left', 'right'}
    snapshot.payload_updates = {'left', 'right'}
    snapshot.switch_updates = {'left', 'right'}
    snapshot.shuttles['right'] = {
        'room315_right_shuttle_1': {
            'x': -13.4246,
            'y': -3.4369,
            'z': 0.8393,
            'yaw': -0.0004,
            'segment': 'A12E',
            's': 0.9172,
        }
    }
    snapshot.payloads['right'] = {
        'room315_right_shuttle_1': loaded,
    }
    snapshot.switches = {
        side: dict(scenario['scene']['rails'][side]['switches'])
        for side in ('left', 'right')
    }
    return snapshot


def test_parses_payload_state_without_extra_model_features():
    result = capture._payload_map(
        json.dumps({
            'side': 'right',
            'model_input_exposure': 'excluded',
            'shuttles': [
                {
                    'entity_name': 'room315_right_shuttle_1',
                    'loaded': True,
                },
                {
                    'entity_name': 'room315_right_shuttle_2',
                    'loaded': False,
                },
            ],
        }),
        'right',
    )

    assert result == {
        'room315_right_shuttle_1': True,
        'room315_right_shuttle_2': False,
    }


def test_normalizes_live_switch_state_codes():
    assert capture._switch_state('I') == 'interior'
    assert capture._switch_state('E') == 'exterior'
    assert capture._switch_state('INTERIOR') == 'interior'

    snapshot = capture.CaptureSnapshot()
    node = object.__new__(capture.VisualStateCaptureNode)
    node.snapshot = snapshot
    node._on_switch(
        'right',
        SimpleNamespace(
            switches=[
                SimpleNamespace(name='A1', state='I'),
                SimpleNamespace(name='A2', state='E'),
            ]
        ),
    )
    assert snapshot.switches['right'] == {
        'A1': 'interior',
        'A2': 'exterior',
    }


def test_projects_and_validates_matching_simulator_snapshot():
    scenario = _right_single_scenario()
    snapshot = _matching_snapshot(scenario)
    cameras = capture.load_camera_projections(capture._default_camera_model_path())

    labels = capture.visual_labels_from_snapshot(
        snapshot,
        cameras,
        {
            side: capture.public_rail_segment_lengths(side)
            for side in ('left', 'right')
        },
    )
    capture.validate_snapshot(
        snapshot,
        scenario,
        labels,
        max_camera_skew_seconds=0.15,
    )

    r1 = next(shuttle for shuttle in labels['shuttles'] if shuttle['id'] == 'R1')
    assert r1['location'] == {
        'block': 'A12E',
        'side': 'right',
    }
    assert r1['rail_position']['available'] is True
    assert r1['rail_position']['s_ratio'] > 0.4
    assert len(labels['switches']) == 8


def test_writes_new_capture_layout_and_validation(tmp_path):
    scenario = _right_single_scenario()
    snapshot = _matching_snapshot(scenario)
    cameras = capture.load_camera_projections(capture._default_camera_model_path())
    labels = capture.visual_labels_from_snapshot(
        snapshot,
        cameras,
        {
            side: capture.public_rail_segment_lengths(side)
            for side in ('left', 'right')
        },
    )

    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[:, :, 1] = np.arange(64, dtype=np.uint8)
    encoded_ok, encoded = cv2.imencode('.jpg', frame)
    assert encoded_ok

    result = capture.write_capture(
        tmp_path,
        scenario,
        labels,
        {
            'left_rail_rgb': encoded.tobytes(),
            'right_rail_rgb': encoded.tobytes(),
        },
        camera_skew_seconds=0.01,
    )

    training_row = json.loads(
        (tmp_path / 'meta' / 'training_events.jsonl').read_text(encoding='utf-8')
    )
    validation = json.loads(Path(result['validation']).read_text(encoding='utf-8'))
    episode_event = json.loads(
        (
            tmp_path
            / 'episodes'
            / scenario['scenario_id']
            / 'event.json'
        ).read_text(encoding='utf-8')
    )
    assert set(training_row['model_input']) == {'overhead_images'}
    assert 'task' not in training_row
    assert validation['schema_version'] == 'room315.visual_capture_validation.v1'
    assert validation['capture_complete'] is True
    assert validation['labels_valid'] is True
    assert 'task_success' not in validation
    assert episode_event == training_row
    assert not list(
        (tmp_path / 'episodes' / '.capture_tmp').iterdir()
    )
