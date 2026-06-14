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


def test_model_input_boundary_allows_only_language_images_and_last_command():
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
    }

    assert multi.model_input_is_clean(clean) is True


def test_model_input_boundary_rejects_privileged_shortcuts():
    multi = _load_module()

    polluted = {
        'language': 'move R2 to the Staubli station',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
        'target_shuttle_id': 'R2',
    }
    sensor_polluted = {
        'language': 'move the loaded right shuttle',
        'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
        'structured_rail_state': {'right_sensor_DZI2R': 1},
    }

    assert multi.model_input_is_clean(polluted) is False
    assert multi.model_input_is_clean(sensor_polluted) is False
