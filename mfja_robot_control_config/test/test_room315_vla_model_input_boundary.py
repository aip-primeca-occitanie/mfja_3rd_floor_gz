#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_multi_shuttle.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _observable_state():
    return {
        'right': {
            'sensors': {'DZI2R': 1, 'DZI3R': 0},
            'switches': {'A1': 'EXTERIOR', 'A2': 'INTERIOR'},
            'stoppers': {'A1': 'open', 'A2': 'closed'},
        },
        'left': {
            'sensors': {'DZI2L': 0},
            'switches': {'A1': 'UNKNOWN'},
            'stoppers': {'A1': 'unknown'},
        },
    }


def test_model_input_boundary_allows_real_observable_state_without_pose():
    multi = _load_module()

    clean = {
        'language': 'move R2 to the Staubli station',
        'overhead_images': {
            'right_rail_rgb': 'episodes/e/images/right_rail_rgb/000001.jpg',
            'left_rail_rgb': 'episodes/e/images/left_rail_rgb/000001.jpg',
        },
        'last_command': {
            'primitive': 'SET_STOPPERS',
            'side': 'right',
            'target_id': 'ALL_STOPPERS',
        },
        'observable_state': _observable_state(),
    }

    assert multi.model_input_is_clean(clean) is True


def test_model_input_boundary_allows_payload_words_only_in_language():
    multi = _load_module()

    clean = {
        'language': 'move the loaded shuttle to Staubli and send the empty shuttle to Yaskawa',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
        'observable_state': _observable_state(),
    }

    assert multi.model_input_is_clean(clean) is True


def test_model_input_boundary_rejects_privileged_shortcuts():
    multi = _load_module()

    polluted = {
        'language': 'move R2 to the Staubli station',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
        'observable_state': _observable_state(),
        'target_shuttle_id': 'R2',
    }
    sensor_polluted = {
        'language': 'move the loaded right shuttle',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
        'observable_state': _observable_state(),
        'structured_rail_state': {'right_sensor_DZI2R': 1},
    }
    payload_polluted = {
        'language': 'move the loaded right shuttle',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START', 'payload_state': {'R2': 'loaded'}},
        'observable_state': _observable_state(),
    }
    pose_polluted = {
        'language': 'move the loaded right shuttle',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
        'observable_state': {
            'right': {
                'sensors': {'DZI2R': 1},
                'switches': {'A1': 'EXTERIOR'},
                'stoppers': {'A1': 'open'},
                's': 0.42,
            },
        },
    }

    assert multi.model_input_is_clean(polluted) is False
    assert multi.model_input_is_clean(sensor_polluted) is False
    assert multi.model_input_is_clean(payload_polluted) is False
    assert multi.model_input_is_clean(pose_polluted) is False
