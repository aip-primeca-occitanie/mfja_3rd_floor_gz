#!/usr/bin/env python3

import copy
import json
import sys
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
CONFIG_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_visual_state'
    / 'blocker_training_scenarios.yaml'
)
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_dataset_audit as audit
import room_315_visual_scenario_generator as generator
import room_315_visual_state_dataset as visual


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        ''.join(
            json.dumps(row, sort_keys=True, separators=(',', ':')) + '\n'
            for row in rows
        ),
        encoding='utf-8',
    )


def _oracle_label():
    segment_length_m = 2.0
    s_m = 0.5
    return {
        'dataset_mode': 'visual_state',
        'sample_id': 'sample-1',
        'visual_state_labels': {
            'schema_version': 'room315.visual_state.v3',
            'calibration_version': 'room315-overhead-v1',
            'confidence': 1.0,
            'shuttles': [{
                'id': 'R1',
                'presence': True,
                'visually_available': True,
                'bbox': [10.0, 20.0, 30.0, 40.0],
                'location': {'side': 'right', 'block': 'A12E'},
                'rail_position': {
                    'available': True,
                    's_m': s_m,
                    's_ratio': s_m / segment_length_m,
                    'segment_length_m': segment_length_m,
                    'position_uncertainty_m': 0.0,
                },
                'loaded_state': 'loaded',
                'confidence': 1.0,
            }],
            'switches': [],
            'obstacles': [],
        },
    }


def test_scenario_audit_checks_reproducibility_diversity_and_coverage():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    scenarios = generator.generate_scenarios(config)

    report = audit.audit_scenarios(config, scenarios)

    assert report['passed'] is True
    assert all(check['passed'] for check in report['checks'].values())
    assert report['checks']['collision_free_world_geometry']['conflict_pairs'] == 0
    assert report['checks']['all_segment_coverage']['observed_side_segment_pairs'] == 28
    assert report['checks']['balanced_zone_coverage']['counts'] == {
        zone: 20 for zone in generator.POSITION_ZONES
    }


def test_visual_label_audit_enforces_v2_oracle_contract(tmp_path):
    labels_path = tmp_path / 'train_visual_labels.jsonl'
    _write_jsonl(labels_path, [_oracle_label()])

    report = audit.audit_visual_labels([labels_path])

    assert report['passed'] is True
    assert report['checks']['oracle_uncertainty_excluded_from_targets']['passed']

    unsafe = _oracle_label()
    unsafe['visual_state_labels']['shuttles'][0]['blocks_route'] = True
    unsafe_path = tmp_path / 'unsafe_visual_labels.jsonl'
    _write_jsonl(unsafe_path, [unsafe])
    unsafe_report = audit.audit_visual_labels([unsafe_path])
    assert unsafe_report['passed'] is False
    assert not unsafe_report['checks']['no_planning_or_safety_labels']['passed']


def test_visual_label_audit_rejects_world_space_shuttle_collision(tmp_path):
    row = _oracle_label()
    second = copy.deepcopy(row['visual_state_labels']['shuttles'][0])
    second['id'] = 'R2'
    second['rail_position']['s_m'] = 0.6
    second['rail_position']['s_ratio'] = 0.3
    row['visual_state_labels']['shuttles'].append(second)
    labels_path = tmp_path / 'colliding_visual_labels.jsonl'
    _write_jsonl(labels_path, [row])

    report = audit.audit_visual_labels([labels_path])

    assert report['passed'] is False
    collision_check = report['checks']['collision_free_world_geometry']
    assert collision_check['affected_rows'] == 1
    assert collision_check['conflict_pairs'] == 1


def test_manifest_label_consistency_detects_position_drift(tmp_path):
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    scenario = generator.generate_scenarios(config, count=1, seed=315)[0]
    shuttles = []
    for side in generator.SIDES:
        for expected in scenario['scene']['rails'][side]['shuttles']:
            position = expected['start_position']
            segment_length_m = generator._segment_lengths(side)[position['segment']]
            shuttles.append({
                'id': expected['id'],
                'presence': True,
                'visually_available': True,
                'bbox': [10.0, 20.0, 30.0, 40.0],
                'location': {
                    'side': side,
                    'block': position['segment'],
                },
                'rail_position': {
                    'available': True,
                    's_m': position['s_ratio'] * segment_length_m,
                    's_ratio': position['s_ratio'],
                    'segment_length_m': segment_length_m,
                    'position_uncertainty_m': 0.0,
                },
                'loaded_state': expected['loaded_state'],
                'confidence': 1.0,
            })
    row = {
        'episode_id': scenario['scenario_id'],
        'scenario_family': scenario['scenario_family'],
        'visual_state_labels': {
            'schema_version': 'room315.visual_state.v3',
            'calibration_version': 'room315-overhead-v1',
            'confidence': 1.0,
            'shuttles': shuttles,
            'switches': [],
            'obstacles': [],
        },
    }
    labels_path = tmp_path / 'labels.jsonl'
    _write_jsonl(labels_path, [row])

    assert audit.audit_manifest_label_consistency(
        [scenario],
        [labels_path],
    )['passed']

    row['visual_state_labels']['shuttles'][0]['rail_position']['s_ratio'] += 0.1
    row['visual_state_labels']['shuttles'][0]['rail_position']['s_m'] = (
        row['visual_state_labels']['shuttles'][0]['rail_position']['s_ratio']
        * row['visual_state_labels']['shuttles'][0]['rail_position']['segment_length_m']
    )
    _write_jsonl(labels_path, [row])
    report = audit.audit_manifest_label_consistency([scenario], [labels_path])

    assert report['passed'] is False
    assert not report['checks']['captured_positions_match_manifest']['passed']
    assert not report['checks']['scenario_relations_preserved_after_capture']['passed']


def test_ratio_consistency_is_validated_and_uncertainty_is_not_a_target():
    label = _oracle_label()
    normalized = visual.normalize_visual_state_labels(label)
    vectorizer = visual.VisualStateLabelVectorizer.fit([normalized])

    assert not any('position_uncertainty_m' in name for name in vectorizer.names)
    assert all(
        visual.is_model_prediction_target(name.split('==')[0])
        for name in vectorizer.names
    )
    serialized = vectorizer.to_json()
    serialized['numeric_keys'] = [
        'shuttles.0.rail_position.position_uncertainty_m',
    ]
    with pytest.raises(
        visual.VisualStateValidationError,
        match='numeric targets',
    ):
        visual.VisualStateLabelVectorizer.from_json(serialized)

    label['visual_state_labels']['shuttles'][0]['rail_position']['s_ratio'] = 0.9
    with pytest.raises(
        visual.VisualStateValidationError,
        match='inconsistent with s_m / segment_length_m',
    ):
        visual.normalize_visual_state_labels(label)


def test_split_audit_detects_train_test_family_leakage(tmp_path):
    split_rows = {
        'train': [{'scenario_family': 'family-train'}],
        'val': [{'scenario_family': 'family-val'}],
        'test': [{'scenario_family': 'family-test'}],
    }
    splits = {}
    for name, rows in split_rows.items():
        filename = f'{name}.jsonl'
        _write_jsonl(tmp_path / filename, rows)
        splits[name] = {
            'file': filename,
            'families': [rows[0]['scenario_family']],
        }
    (tmp_path / 'split_manifest.json').write_text(
        json.dumps({'splits': splits}),
        encoding='utf-8',
    )

    assert audit.audit_split_families(tmp_path)['passed'] is True

    _write_jsonl(
        tmp_path / 'test.jsonl',
        [{'scenario_family': 'family-train'}],
    )
    report = audit.audit_split_families(tmp_path)
    assert report['passed'] is False
    assert not report['checks']['no_scenario_family_leakage']['passed']
