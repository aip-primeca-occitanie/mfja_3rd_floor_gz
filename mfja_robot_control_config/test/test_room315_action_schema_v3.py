#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_multi_shuttle.py'
ACTION_SPACE_PATH = (
    REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'action_space.yaml'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_action_schema_v3_contains_shuttle_and_coordination_fields():
    multi = _load_module()
    config = yaml.safe_load(ACTION_SPACE_PATH.read_text(encoding='utf-8'))

    assert multi.ACTION_SCHEMA_VERSION == 3
    assert config['schema_version'] == 3
    assert config['action_schema_version'] == 3
    assert config['model_input_schema_version'] == multi.MODEL_INPUT_SCHEMA_VERSION
    assert config['model_input_fields'] == [
        'language',
        'overhead_images',
        'last_command',
        'observable_state',
    ]
    assert config['action_vector_fields'] == multi.ACTION_VECTOR_V3_FIELDS
    assert 'shuttle_index' in multi.ACTION_VECTOR_V3_FIELDS
    assert 'coordination_mode' in multi.ACTION_VECTOR_V3_FIELDS
    assert config['target_ids']['right_shuttle_2'] == multi.TARGET_IDS['right_shuttle_2']


def test_action_schema_v3_roundtrips_specific_shuttle_target():
    multi = _load_module()

    action = {
        'primitive': 'SHUTTLE_ON',
        'side': 'right',
        'shuttle_id': 'R2',
        'speed_mps': 0.35,
        'wait_condition': 'shuttle_command_applied',
        'target_id': 'right_shuttle_2',
        'reason': 'target_station_route',
        'coordination_mode': 'guarded_motion',
    }

    encoded = multi.encode_action_v3(action)
    decoded = multi.decode_action_v3(encoded)

    assert len(encoded) == len(multi.ACTION_VECTOR_V3_FIELDS)
    assert encoded[multi.ACTION_VECTOR_V3_FIELDS.index('shuttle_index')] == 1.0
    assert decoded['primitive'] == 'SHUTTLE_ON'
    assert decoded['shuttle_id'] == 'R2'
    assert decoded['shuttle_index'] == 1
    assert decoded['target_id'] == 'right_shuttle_2'
    assert decoded['coordination_mode'] == 'guarded_motion'


def test_global_emergency_stop_can_use_unassigned_shuttle_index():
    multi = _load_module()

    encoded = multi.encode_action_v3({
        'primitive': 'EMERGENCY_STOP',
        'side': 'left',
        'shuttle_index': -1,
        'reason': 'emergency',
        'coordination_mode': 'emergency',
    })
    decoded = multi.decode_action_v3(encoded)

    assert decoded['primitive'] == 'EMERGENCY_STOP'
    assert decoded['shuttle_index'] == -1
    assert decoded['shuttle_id'] == ''
    assert decoded['coordination_mode'] == 'emergency'
