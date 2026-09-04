#!/usr/bin/env python3

import importlib.util
import json
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
TRACKER_PATH = SCRIPT_DIR / 'room_315_privileged_shuttle_identity_tracker.py'
IDENTITY_CONFIG_PATH = (
    REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_shuttle_identity' / 'shuttle_identity.yaml'
)


def _load_tracker():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_privileged_shuttle_identity_tracker', TRACKER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_tracker_fuses_multiple_perimeter_markers_for_one_shuttle():
    tracker = _load_tracker()
    config = yaml.safe_load(IDENTITY_CONFIG_PATH.read_text(encoding='utf-8'))
    raw = json.dumps({
        'detections': [
            {'tag_id': 111, 'bbox': [10, 20, 30, 40]},
            {'tag_id': 112, 'bbox': [12, 22, 32, 42]},
        ]
    })

    tracks = tracker.tracks_from_detection_message(raw, config, now_s=123.5)

    assert len(tracks) == 1
    assert tracks[0]['shuttle_id'] == 'right_shuttle_2'
    assert tracks[0]['side'] == 'right'
    assert tracks[0]['shuttle_index'] == 1
    assert tracks[0]['visible_marker_ids'] == [111, 112]
    assert tracks[0]['visible_marker_count'] == 2
    assert tracks[0]['visibility_state'] == 'visible'
    assert tracks[0]['model_input_exposure'] == 'excluded'
    assert tracks[0]['timestamp'] == 123.5


def test_tracker_keeps_single_visible_marker_as_partial_occlusion():
    tracker = _load_tracker()
    config = yaml.safe_load(IDENTITY_CONFIG_PATH.read_text(encoding='utf-8'))

    tracks = tracker.tracks_from_detection_message(
        json.dumps([{'tag_id': 223}]),
        config,
        now_s=55.0,
    )

    assert tracks[0]['shuttle_id'] == 'left_shuttle_3'
    assert tracks[0]['visible_marker_count'] == 1
    assert tracks[0]['visibility_state'] == 'partially_occluded'
    assert tracks[0]['model_input_exposure'] == 'excluded'


def test_track_memory_keeps_briefly_missing_identity_as_lost_privileged_track():
    tracker = _load_tracker()
    memory = tracker.IdentityTrackMemory(max_lost_s=1.0)
    first = [{
        'shuttle_id': 'right_shuttle_2',
        'visible_marker_ids': [111, 112],
        'visible_marker_count': 2,
        'visibility_state': 'visible',
        'confidence': 0.9,
        'model_input_exposure': 'excluded',
    }]

    assert memory.update(first, now_s=10.0)[0]['visibility_state'] == 'visible'
    remembered = memory.update([], now_s=10.5)

    assert remembered[0]['shuttle_id'] == 'right_shuttle_2'
    assert remembered[0]['visibility_state'] == 'lost'
    assert remembered[0]['track_age_since_seen_s'] == 0.5
    assert remembered[0]['model_input_exposure'] == 'excluded'
