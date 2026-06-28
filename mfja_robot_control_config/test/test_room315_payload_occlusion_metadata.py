#!/usr/bin/env python3

import importlib.util
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
RECORDER_PATH = SCRIPT_DIR / 'room_315_vla_dataset_recorder.py'


def _load_recorder():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_vla_dataset_recorder', RECORDER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_payload_occlusion_fields_are_planning_metadata_not_model_input():
    recorder = _load_recorder()
    source = {
        'planning_metadata': {
            'target_shuttle_id': 'L3',
            'payload_present': True,
            'payload_type': 'wide_payload_inside_keepout',
            'visible_marker_count': 2,
            'identity_occlusion_level': 'partial',
            'expected_visible_ids': [221, 222],
        },
        'model_input': {
            'language': 'move L3 to KUKA even though it is carrying a part',
            'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
            'last_command': {'action': 'START'},
            'observable_state': {},
        },
    }

    metadata = recorder._planning_metadata_from_source(source)
    model_input = source['model_input']

    assert metadata['target_shuttle_id'] == 'L3'
    assert metadata['payload_present'] is True
    assert metadata['visible_marker_count'] == 2
    assert metadata['expected_visible_ids'] == [221, 222]
    assert set(model_input) == {'language', 'overhead_images', 'last_command', 'observable_state'}

    serialized = json.dumps(model_input, sort_keys=True)
    for forbidden in (
        'target_shuttle_id',
        'visible_marker_count',
        'identity_occlusion_level',
        'expected_visible_ids',
        '221',
        '222',
    ):
        assert forbidden not in serialized
