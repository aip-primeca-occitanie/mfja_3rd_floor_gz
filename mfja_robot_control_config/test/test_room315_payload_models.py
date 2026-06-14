#!/usr/bin/env python3

from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = REPO_ROOT / 'mfja_3rd_floor_description' / 'models'
PAYLOAD_CONFIG = (
    REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_vla' / 'payload_scenarios.yaml'
)


def test_payload_models_exist_for_configured_variants():
    config = yaml.safe_load(PAYLOAD_CONFIG.read_text(encoding='utf-8'))
    variants = config['variants']

    for name in ('small_box', 'medium_box', 'tall_box', 'wide_box_within_keepout', 'partial_marker_occluder'):
        model_uri = variants[name]['model_uri']
        assert model_uri.startswith('model://room315_vla_payload_')
        model_dir = MODELS_DIR / model_uri.removeprefix('model://')
        assert (model_dir / 'model.config').exists()
        assert (model_dir / 'model.sdf').exists()
        assert '<collide_bitmask>0x0000</collide_bitmask>' in (
            model_dir / 'model.sdf'
        ).read_text(encoding='utf-8')


def test_normal_payloads_preserve_all_perimeter_identity_regions():
    config = yaml.safe_load(PAYLOAD_CONFIG.read_text(encoding='utf-8'))

    for name, variant in config['variants'].items():
        if not variant.get('normal_payload'):
            continue
        assert variant['identity_occlusion_level'] in {'none', 'visible'}
        assert variant['visible_marker_count'] == 4
        assert set(variant['expected_visible_regions']) == {
            'front_left',
            'front_right',
            'rear_left',
            'rear_right',
        }


def test_partial_marker_occluder_is_explicitly_controlled_metadata():
    config = yaml.safe_load(PAYLOAD_CONFIG.read_text(encoding='utf-8'))
    variant = config['variants']['partial_marker_occluder']

    assert variant['normal_payload'] is False
    assert variant['identity_occlusion_level'] == 'partial'
    assert variant['visible_marker_count'] == 3
    assert variant['occluded_regions'] == ['front_left']
    assert 'payload_present' in config['metadata_fields_outside_model_input']
    assert 'target_shuttle_id' in config['metadata_fields_outside_model_input']
