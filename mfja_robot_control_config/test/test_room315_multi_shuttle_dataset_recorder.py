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


def _blank_status():
    return {
        'emergency_stop': False,
        'vision': {'image_frames': 2},
        'rails': {
            'right': {
                'shuttles': {
                    'room315_right_shuttle_2': {'segment': 'A23', 'x': 1.0, 's': 0.2}
                },
                'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
                'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
                'active_position_sensors': [],
            },
            'left': {
                'shuttles': {},
                'switches': {'A1': 'E', 'A2': 'E', 'A3': 'E', 'A4': 'E'},
                'stoppers': {'A1': '0', 'A2': '0', 'A3': '0', 'A4': '0'},
                'active_position_sensors': [],
            },
        },
    }


def test_dataset_recorder_encodes_action_vector_schema_v3_for_target_shuttle():
    recorder = _load_recorder()
    action = {
        'action_vector_schema_version': 3,
        'primitive': 'SHUTTLE_ON',
        'side': 'right',
        'shuttle_id': 'R2',
        'shuttle_index': 1,
        'speed_mps': 0.3,
        'wait_condition': 'shuttle_command_applied',
        'target_id': 'right_shuttle_2',
        'reason': 'target_station_route',
        'coordination_mode': 'guarded_motion',
    }

    encoded = recorder._encode_action(action)
    decoded = recorder._decode_action(encoded)

    assert len(encoded) == len(recorder.ACTION_VECTOR_V3_FIELDS)
    assert decoded['primitive'] == 'SHUTTLE_ON'
    assert decoded['shuttle_id'] == 'R2'
    assert decoded['shuttle_index'] == 1
    assert decoded['target_id'] == 'right_shuttle_2'
    assert decoded['coordination_mode'] == 'guarded_motion'


def test_payload_occlusion_metadata_is_extracted_outside_model_input():
    recorder = _load_recorder()
    metadata = recorder._planning_metadata_from_source({
        'planning_metadata': {
            'planning_source': 'plansys',
            'pddl_goal': 'task_done right_shuttle_2 staubli',
            'target_shuttle_id': 'R2',
            'involved_shuttles': ['R1', 'R2'],
            'reserved_blocks': ['right_block_2'],
            'payload_present': True,
            'payload_type': 'medium_centered_payload',
            'visible_marker_count': 2,
            'identity_occlusion_level': 'partial',
            'expected_visible_ids': [111, 112],
        }
    })

    assert metadata['target_shuttle_id'] == 'R2'
    assert metadata['payload_present'] is True
    assert metadata['visible_marker_count'] == 2
    assert metadata['identity_occlusion_level'] == 'partial'

    model_input = recorder._model_input_from_status(
        _blank_status(),
        language='move R2 to the Staubli station',
        overhead_images={'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        last_command={'action': 'START'},
    )
    serialized = json.dumps(model_input, sort_keys=True)

    assert set(model_input) == {'language', 'overhead_images', 'last_command'}
    for forbidden in (
        'target_shuttle_id',
        'visible_marker_count',
        'expected_visible_ids',
        'pddl_goal',
        'right_block_2',
        'tag_id',
        'A23',
        '"x"',
        '"s"',
    ):
        assert forbidden not in serialized


def test_identity_tracks_attach_to_privileged_eval_not_model_input():
    recorder = _load_recorder()
    tracks = recorder._parse_identity_tracks_payload(json.dumps({
        'tracks': [
            {
                'shuttle_id': 'right_shuttle_2',
                'visible_marker_ids': [111, 112],
                'visible_marker_count': 2,
                'visibility_state': 'partially_occluded',
                'confidence': 0.75,
            }
        ]
    }))
    metadata = recorder._identity_metadata_from_tracks(tracks, target_shuttle_id='R2')
    model_input = recorder._model_input_from_status(
        _blank_status(),
        language='move R2 to the Staubli station',
        overhead_images={'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
        last_command={'action': 'START'},
    )

    assert metadata['correct_target_shuttle'] == 'R2'
    assert metadata['visible_marker_count_for_target'] == 2
    assert metadata['identity_occlusion_level'] == 'partial'
    assert metadata['expected_visible_ids'] == [111, 112]
    assert recorder._validated_model_input(model_input) == model_input

    serialized = json.dumps(model_input, sort_keys=True)
    for forbidden in ('visible_marker_ids', 'tag_id', '111', '112', 'shuttle_identity_tracks'):
        assert forbidden not in serialized
