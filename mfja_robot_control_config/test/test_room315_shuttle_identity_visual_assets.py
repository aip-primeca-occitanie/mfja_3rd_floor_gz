#!/usr/bin/env python3

import re
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = REPO_ROOT / 'mfja_3rd_floor_description' / 'models'
IDENTITY_CONFIG = (
    REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'shuttle_identity.yaml'
)
ROOM_ONLY_WORLD = REPO_ROOT / 'mfja_3rd_floor_description' / 'worlds' / 'room_315_only.world'
FULL_WORLD = REPO_ROOT / 'mfja_3rd_floor_description' / 'worlds' / 'mfja_3rd_floor.world'
KINEMATIC_NODE = (
    REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_kinematic_shuttle_node.py'
)
IDENTITY_COLORS = {
    'R1': '0.85 0.05 0.05',
    'R2': '0.05 0.22 0.85',
    'R3': '0.05 0.60 0.20',
    'R4': '0.95 0.75 0.05',
    'L1': '0.05 0.70 0.80',
    'L2': '0.75 0.05 0.75',
    'L3': '0.90 0.36 0.04',
    'L4': '0.90 0.90 0.90',
}


def _short_id(entry):
    prefix = 'R' if entry['side'] == 'right' else 'L'
    return f'{prefix}{int(entry["shuttle_index"]) + 1}'


def test_every_shuttle_has_physical_perimeter_identity_model():
    config = yaml.safe_load(IDENTITY_CONFIG.read_text(encoding='utf-8'))

    for entry in config['shuttles']:
        short_id = _short_id(entry)
        model_dir = MODELS_DIR / f'room315_shuttle_{short_id}'
        sdf_path = model_dir / 'model.sdf'
        text = sdf_path.read_text(encoding='utf-8')

        assert model_dir.exists()
        assert (model_dir / 'model.config').exists()
        assert f'payload_keepout_center_{short_id}' in text
        assert 'model://room315_shuttle/meshes/shuttle.STL' in text
        assert '<albedo_map>' not in text
        assert '<pbr>' not in text
        assert '_identity.svg' not in text
        assert f'<ambient>{IDENTITY_COLORS[short_id]} 1</ambient>' in text
        assert f'<diffuse>{IDENTITY_COLORS[short_id]} 1</diffuse>' in text

        for role, tag_id in entry['tag_ids'].items():
            assert f'identity_region_{role}_label_{short_id}_tag_{tag_id}' in text


def test_worlds_preload_eight_distinct_identity_models():
    for world_path in (ROOM_ONLY_WORLD, FULL_WORLD):
        world = world_path.read_text(encoding='utf-8')
        for short_id in ('R1', 'R2', 'R3', 'R4', 'L1', 'L2', 'L3', 'L4'):
            assert f'model://room315_shuttle_{short_id}' in world

        assert len(re.findall(r'<name>room315_right_shuttle_[1-4]</name>', world)) == 4
        assert len(re.findall(r'<name>room315_left_shuttle_[1-4]</name>', world)) == 4


def test_dynamic_spawn_uses_per_identity_shuttle_sdf_when_available():
    text = KINEMATIC_NODE.read_text(encoding='utf-8')

    assert '_shuttle_model_sdf_for_entity' in text
    assert 'room315_shuttle_{short_id}' in text
    assert "preloaded_shuttle_count': 4" in text
