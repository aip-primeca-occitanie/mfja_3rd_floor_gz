#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_grouped_visual_splits as splits


CAPTURE_ROOT = Path(
    '/home/tiago/'
    'room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729'
)
SPLIT_ROOT = Path(
    '/home/tiago/'
    'room315_arbitrary_subset_visual_splits_v1_seed31520260730'
)


def _json(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def _jsonl(path: Path):
    return [
        json.loads(line)
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]


@pytest.fixture(scope='module')
def source():
    return splits.load_source(CAPTURE_ROOT)


def _package_assignment():
    assignment = {}
    for split in splits.SPLIT_CONFIGURATION_COUNTS:
        for configuration_id in _json(
            SPLIT_ROOT / f'{split}_configuration_ids.json'
        )['configuration_ids']:
            assert configuration_id not in assignment
            assignment[configuration_id] = split
    return assignment


def test_split_package_and_exact_grouped_counts_pass():
    verification = splits.verify_package(SPLIT_ROOT)
    assignment = _package_assignment()

    assert verification['passed']
    assert verification['split_counts'] == {
        'train': 1528,
        'validation': 256,
        'test': 256,
    }
    assert len(assignment) == 255
    for split, expected_configurations in (
        ('train', 191),
        ('validation', 32),
        ('test', 32),
    ):
        rows = _jsonl(SPLIT_ROOT / f'{split}.jsonl')
        grouped = {}
        for row in rows:
            configuration_id = row['traceability_metadata'][
                'presence_configuration_id'
            ]
            grouped.setdefault(configuration_id, []).append(row)
        assert len(grouped) == expected_configurations
        assert {len(group) for group in grouped.values()} == {8}


def test_all_scenarios_are_assigned_once_and_hard_leakage_is_zero():
    all_ids = []
    for split in splits.SPLIT_CONFIGURATION_COUNTS:
        all_ids.extend(
            _json(SPLIT_ROOT / f'{split}_scenario_ids.json')['scenario_ids']
        )
    leakage = _json(SPLIT_ROOT / 'leakage_audit.json')

    assert len(all_ids) == len(set(all_ids)) == 2040
    assert leakage['passed']
    assert all(leakage['checks'].values())
    for field_report in leakage['overlaps'].values():
        assert all(
            value['count'] == 0
            for value in field_report.values()
        )


def test_generation_is_deterministic_and_seed_sensitive(source):
    components = splits.image_hash_components(source)
    retry_ids = {
        row['scenario_id']
        for row in source['objects']['capture_state.json'].get(
            'historical_failures'
        ) or []
    }
    features = splits._group_features(source, retry_ids)
    first, _ = splits.assign_components(
        components,
        features,
        seed=splits.SEED,
    )
    repeated, _ = splits.assign_components(
        components,
        features,
        seed=splits.SEED,
    )
    changed, _ = splits.assign_components(
        components,
        features,
        seed=splits.SEED + 1,
    )

    assert first == repeated == _package_assignment()
    assert changed != first


def test_contract_is_camera_only_and_metadata_never_becomes_a_target():
    audit = _json(SPLIT_ROOT / 'target_contract_audit.json')

    assert audit['passed']
    assert audit['allowed_model_input_fields'] == ['overhead_images']
    assert audit['prediction_target_fields'] == [
        'bbox',
        'loaded_state',
        'location.block',
        'location.side',
        'rail_position.s_m',
        'rail_position.s_ratio',
        'rail_position.segment_length_m',
    ]
    assert audit['checks']['target_identity_metadata_only']
    assert audit['checks']['relation_family_metadata_only']
    assert audit['checks']['serialized_tensors_contain_no_metadata']
    assert audit['opposite_camera_bbox_loss_sum'] == 0.0
    assert audit['vectorizer']['dim'] == 200
    assert audit['vectorizer']['fixed_identity_order'] == list(
        splits.IDENTITIES
    )


def test_source_images_are_still_hash_valid_and_were_not_copied_to_split(source):
    fingerprint = _json(SPLIT_ROOT / 'source_dataset_fingerprint.json')
    image_files = list(SPLIT_ROOT.rglob('*.jpg'))

    assert source['source_image_path_count'] == 4080
    assert fingerprint['source_image_count'] == 4080
    assert fingerprint['source_images_referenced_not_copied'] is True
    assert image_files == []
