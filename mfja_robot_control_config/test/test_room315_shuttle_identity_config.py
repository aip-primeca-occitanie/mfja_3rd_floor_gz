#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_multi_shuttle.py'
IDENTITY_CONFIG_PATH = (
    REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'shuttle_identity.yaml'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_multi_shuttle', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_identity_config_maps_all_shuttles_and_unique_tags():
    multi = _load_module()
    config = yaml.safe_load(IDENTITY_CONFIG_PATH.read_text(encoding='utf-8'))

    multi.validate_identity_config(config)

    shuttles = {entry['shuttle_id']: entry for entry in config['shuttles']}
    assert set(shuttles) == {spec.shuttle_id for spec in multi.all_shuttle_specs()}
    assert shuttles['right_shuttle_2']['label_text'] == 'R2'
    assert shuttles['right_shuttle_2']['tag_ids'] == {
        'front_left': 111,
        'front_right': 112,
        'rear_left': 113,
        'rear_right': 114,
    }
    assert shuttles['left_shuttle_3']['label_text'] == 'L3'
    assert shuttles['left_shuttle_3']['tag_ids']['front_left'] == 221


def test_identity_config_declares_payload_keepout_and_privileged_boundary():
    config = yaml.safe_load(IDENTITY_CONFIG_PATH.read_text(encoding='utf-8'))
    notes = ' '.join(config['identity_design']['notes'])

    assert config['identity_design']['payload_zone'] == 'center'
    assert set(config['identity_design']['identity_safe_zones']) == {
        'front_left',
        'front_right',
        'rear_left',
        'rear_right',
    }
    assert 'not model_input' in notes
    for entry in config['shuttles']:
        assert len(entry['tag_ids']) >= 2
        assert 'payload_keepout_zone' in entry
