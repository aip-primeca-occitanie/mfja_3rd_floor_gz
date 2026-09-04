#!/usr/bin/env python3

from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
IDENTITY_CONFIG = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_shuttle_identity'
    / 'shuttle_identity.yaml'
)
PAYLOAD_CONFIG = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_payload_cases'
    / 'payload_scenarios.yaml'
)


def test_identity_markers_are_outside_center_payload_keepout_zone():
    identity = yaml.safe_load(IDENTITY_CONFIG.read_text(encoding='utf-8'))

    for shuttle in identity['shuttles']:
        keepout = shuttle['payload_keepout_zone']['center_zone_m']
        x_half = float(keepout['x_half_width'])
        y_half = float(keepout['y_half_width'])
        for role, pose in shuttle['expected_marker_locations_on_body'].items():
            assert abs(float(pose['x'])) > x_half, role
            assert abs(float(pose['y'])) > y_half, role


def test_payload_config_matches_identity_keepout_boundary():
    identity = yaml.safe_load(IDENTITY_CONFIG.read_text(encoding='utf-8'))
    payload = yaml.safe_load(PAYLOAD_CONFIG.read_text(encoding='utf-8'))

    sample_keepout = identity['shuttles'][0]['payload_keepout_zone']['center_zone_m']
    payload_keepout = payload['payload_zone']['center_zone_m']
    assert payload_keepout == sample_keepout
    assert payload['model_input_boundary']['model_input_fields'] == [
        'language',
        'overhead_images',
        'last_command',
        'observable_state',
    ]
    assert (
        payload['model_input_boundary']['payload_and_identity_metadata_exposure']
        == 'outside_model_input'
    )
