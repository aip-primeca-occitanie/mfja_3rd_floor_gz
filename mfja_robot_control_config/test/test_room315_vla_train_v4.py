#!/usr/bin/env python3
"""Unit tests for the isolated Room 315 V4 trainer data contracts."""

from __future__ import annotations

import json
import inspect
import sys
from collections import Counter
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
CONFIG_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_vla'
    / 'visual_state_training_v4.json'
)
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_vla_train_v4 as trainer  # noqa: E402
from room_315_rail_defaults import rail_segment_lengths  # noqa: E402


def _shuttle(
    identity,
    *,
    segment='A12E',
    loaded='empty',
    bbox=(64.0, 48.0, 128.0, 96.0),
    ratio=0.25,
    length=2.0,
    visible=True,
):
    if not visible:
        return {
            'id': identity,
            'presence': False,
            'visually_available': False,
        }
    side = trainer.derive_side(identity)
    return {
        'id': identity,
        'presence': True,
        'visually_available': True,
        'bbox': list(bbox),
        'location': {'side': side, 'block': segment},
        'rail_position': {
            'available': True,
            's_m': ratio * length,
            's_ratio': ratio,
            'segment_length_m': length,
        },
        'loaded_state': loaded,
        'confidence': 1.0,
    }


def _label(sample_id, visible_by_identity=None):
    visible_by_identity = visible_by_identity or {
        'L1': {'segment': 'A12E'},
    }
    shuttles = []
    for identity in trainer.FIXED_IDENTITIES:
        values = visible_by_identity.get(identity)
        shuttles.append(
            _shuttle(identity, visible=False)
            if values is None
            else _shuttle(identity, **values)
        )
    return {
        'sample_id': sample_id,
        'visual_state_labels': {
            'schema_version': 'room315.visual_state.v3',
            'calibration_version': 'synthetic.v1',
            'confidence': 1.0,
            'shuttles': shuttles,
            'switches': [],
            'obstacles': [],
        },
    }


def _write_jsonl(path, rows):
    path.write_text(
        ''.join(json.dumps(row, separators=(',', ':')) + '\n' for row in rows),
        encoding='utf-8',
    )


def _images(dataset_root):
    dataset_root.mkdir(parents=True, exist_ok=True)
    left = dataset_root / 'left.jpg'
    right = dataset_root / 'right.jpg'
    Image.new('RGB', (640, 480), (255, 0, 0)).save(left)
    Image.new('RGB', (640, 480), (0, 0, 255)).save(right)
    return left, right


def _row(sample_id):
    return {
        'sample_id': sample_id,
        'dataset_mode': 'visual_state',
        'model_input': {
            'overhead_images': {
                'left_rail_rgb': 'left.jpg',
                'right_rail_rgb': 'right.jpg',
            },
        },
        'traceability_metadata': {
            'target_identity': 'L1',
            'target_zone': 'ordinary',
            'identity_to_zone': {'L1': 'ordinary'},
            'relation_family': 'no_relation_observation',
        },
    }


def _record(
    sample_id,
    source,
    *,
    identity='L1',
    segment='A12E',
    target_zone='ordinary',
    relation='no_relation_observation',
    image_paths=None,
):
    label = _label(sample_id, {identity: {'segment': segment}})
    normalized = trainer.normalize_visual_state_labels(label)
    row = _row(sample_id)
    row['traceability_metadata'].update({
        'target_identity': identity,
        'target_zone': target_zone,
        'identity_to_zone': {identity: target_zone},
        'relation_family': relation,
    })
    return trainer.PairedRecord(
        sample_id=sample_id,
        source=source,
        role='train',
        dataset_root=Path('.'),
        row=row,
        label=label,
        normalized_label=normalized,
        image_paths=dict(image_paths or {}),
        trace=row['traceability_metadata'],
    )


def _sampling(records_per_source=20):
    return {
        'records_per_source_per_epoch': records_per_source,
        'sample_weight_min': 0.5,
        'sample_weight_max': 3.0,
        'boundary_multiplier': 1.5,
        'switch_multiplier': 2.0,
        'multi_blocker_multiplier': 1.5,
    }


def test_environment_defaults_override_and_unresolved_variables_fail_closed(tmp_path):
    raw = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    raw['environment_defaults']['ROOM315_UNIT_ROOT'] = str(tmp_path / 'default')
    raw['data']['train_sources'][0]['rows'] = '${ROOM315_UNIT_ROOT}/train.jsonl'
    config_path = tmp_path / 'config.json'
    config_path.write_text(json.dumps(raw), encoding='utf-8')

    defaulted = trainer.load_v4_config(config_path, environ={})
    overridden = trainer.load_v4_config(
        config_path,
        environ={'ROOM315_UNIT_ROOT': str(tmp_path / 'override')},
    )
    assert Path(defaulted['data']['train_sources'][0]['rows']) == (
        tmp_path / 'default' / 'train.jsonl'
    )
    assert Path(overridden['data']['train_sources'][0]['rows']) == (
        tmp_path / 'override' / 'train.jsonl'
    )

    raw['data']['train_sources'][0]['rows'] = '${ROOM315_MISSING}/train.jsonl'
    config_path.write_text(json.dumps(raw), encoding='utf-8')
    with pytest.raises(trainer.V4TrainerError, match='unresolved or empty'):
        trainer.load_v4_config(config_path, environ={})


def test_no_test_basename_lock_applies_to_configured_artifacts(tmp_path):
    for name in ('test.jsonl', 'TestingData', 'TEST_visual_labels.jsonl'):
        with pytest.raises(trainer.V4TrainerError, match='basename is forbidden'):
            trainer.assert_not_test_path(tmp_path / name)

    raw = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    raw['data']['validation']['rows'] = str(tmp_path / 'test_validation.jsonl')
    config_path = tmp_path / 'config.json'
    config_path.write_text(json.dumps(raw), encoding='utf-8')
    with pytest.raises(trainer.V4TrainerError, match='basename is forbidden'):
        trainer.load_v4_config(config_path)


def test_structured_target_extraction_uses_fixed_order_actual_sizes_and_masks():
    label = _label(
        'sample-1',
        {
            'L1': {
                'segment': 'A14',
                'loaded': 'loaded',
                'bbox': (64.0, 48.0, 128.0, 96.0),
                'ratio': 0.4,
                'length': 2.5,
            },
            'R1': {
                'segment': 'A34E',
                'bbox': (32.0, 24.0, 64.0, 48.0),
                'ratio': 0.2,
                'length': 3.0,
            },
        },
    )
    targets = trainer.extract_structured_targets(
        label,
        image_sizes={
            'left_rail_rgb': (640, 480),
            'right_rail_rgb': (320, 240),
        },
    )

    assert targets['segment'].shape == (8,)
    assert targets['bbox'].shape == (8, 4)
    assert targets['s_ratio'].shape == (8, 1)
    assert targets['segment'][0] == trainer.SEGMENT_CLASSES.index('A14')
    assert targets['segment'][4] == trainer.SEGMENT_CLASSES.index('A34E')
    assert targets['loaded'][0] == 1
    assert targets['visibility_mask'].tolist() == [
        True, False, False, False, True, False, False, False,
    ]
    np.testing.assert_allclose(targets['bbox'][0], [0.1, 0.1, 0.2, 0.2])
    np.testing.assert_allclose(targets['bbox'][4], [0.1, 0.1, 0.2, 0.2])
    assert targets['s_ratio'][0, 0] == pytest.approx(0.4)
    assert targets['s_m'][0, 0] == pytest.approx(1.0)
    assert targets['segment'][1] == -100
    assert not targets['bbox_mask'][1].any()


def test_pairing_is_by_sample_id_with_expected_counts_and_decoded_images(tmp_path):
    dataset_root = tmp_path / 'dataset'
    _images(dataset_root)
    rows_path = tmp_path / 'train_rows.jsonl'
    labels_path = tmp_path / 'train_labels.jsonl'
    rows = [_row('sample-a'), _row('sample-b')]
    labels = [_label('sample-b'), _label('sample-a')]
    _write_jsonl(rows_path, rows)
    _write_jsonl(labels_path, labels)
    spec = {
        'name': 'synthetic_train',
        'dataset_root': str(dataset_root),
        'rows': str(rows_path),
        'labels': str(labels_path),
        'expected_rows': 2,
    }

    records, report = trainer.load_paired_source(
        spec, role='train', decode_images=True
    )
    assert [record.sample_id for record in records] == ['sample-a', 'sample-b']
    assert [record.label['sample_id'] for record in records] == ['sample-a', 'sample-b']
    assert report['paired_records'] == 2
    assert report['decoded_images'] == 4

    spec['expected_rows'] = 3
    with pytest.raises(trainer.V4TrainerError, match='expected 3'):
        trainer.load_paired_source(spec, role='train')


def test_image_pair_resize_preserves_camera_order_without_spatial_augmentation(tmp_path):
    dataset_root = tmp_path / 'dataset'
    left, right = _images(dataset_root)
    record = _record(
        'image-sample',
        'source-a',
        image_paths={'left_rail_rgb': left, 'right_rail_rgb': right},
    )
    preprocessing = {
        'width': 320,
        'height': 240,
        'normalization_mean': [0.0, 0.0, 0.0],
        'normalization_std': [1.0, 1.0, 1.0],
        'train_augmentations': {
            'horizontal_flip': False,
            'camera_swap': False,
            'brightness': 0.0,
            'contrast': 0.0,
            'saturation': 0.0,
            'gamma_min': 1.0,
            'gamma_max': 1.0,
            'gaussian_blur_probability': 0.0,
            'gaussian_noise_probability': 0.0,
            'gaussian_noise_std': 0.0,
        },
    }
    image, sizes = trainer.load_paired_image_arrays(
        record,
        preprocessing,
        training=True,
        seed=315,
        epoch=1,
    )
    assert image.shape == (6, 240, 320)
    assert sizes == {
        'left_rail_rgb': (640, 480),
        'right_rail_rgb': (640, 480),
    }
    assert image[0].mean() > 0.99
    assert image[1:3].max() < 0.01
    assert image[3:5].max() < 0.01
    assert image[5].mean() > 0.99


def test_weighted_source_balanced_sampler_is_deterministic_per_epoch():
    source_a = [
        _record(f'a-{index}', 'source-a', segment='A12E')
        for index in range(3)
    ] + [
        _record('a-rare', 'source-a', segment='A14', target_zone='boundary')
    ]
    source_b = [
        _record(f'b-{index}', 'source-b', identity='R1', segment='A34E')
        for index in range(4)
    ]
    records = {'source-a': source_a, 'source-b': source_b}
    specs = [
        {'name': 'source-a', 'enabled': True, 'epoch_fraction': 0.5},
        {'name': 'source-b', 'enabled': True, 'epoch_fraction': 0.5},
    ]
    sampling = _sampling(20)
    weights = trainer.compute_sampling_weights(source_a, sampling)
    assert weights['a-rare'] > weights['a-0']

    first = trainer.build_epoch_selection(
        records, specs, sampling, seed=315, epoch=1
    )
    repeated = trainer.build_epoch_selection(
        records, specs, sampling, seed=315, epoch=1
    )
    next_epoch = trainer.build_epoch_selection(
        records, specs, sampling, seed=315, epoch=2
    )
    assert first == repeated
    assert first != next_epoch
    assert Counter(item['source'] for item in first) == {
        'source-a': 20,
        'source-b': 20,
    }
    for source in ('source-a', 'source-b'):
        source_draws = [item for item in first if item['source'] == source]
        assert len({item['draw_index'] for item in source_draws}) == 20
        by_sample = {}
        for item in source_draws:
            by_sample.setdefault(item['sample_id'], []).append(
                item['resample_occurrence']
            )
        for occurrences in by_sample.values():
            assert sorted(occurrences) == list(range(len(occurrences)))

    partial_support = trainer.segment_support_report(source_a, role='train')
    assert partial_support['by_side']['left'][
        trainer.SEGMENT_CLASSES.index('A12E')
    ] == 3
    with pytest.raises(trainer.V4TrainerError, match='zero-support cells'):
        trainer.require_full_segment_support(partial_support)
    with pytest.raises(trainer.V4TrainerError, match='zero-support cells'):
        trainer.compute_train_class_counts(records)

    complete = {
        'complete-source': [
            _record(
                f'{side}-{segment}',
                'complete-source',
                identity='L1' if side == 'left' else 'R1',
                segment=segment,
            )
            for side in trainer.SIDES
            for segment in trainer.SEGMENT_CLASSES
        ],
    }
    counts = trainer.compute_train_class_counts(complete)
    assert counts['provenance'] == 'train_only'
    assert counts['all_side_x_segment_cells_supported'] is True
    assert all(value == 1 for value in counts['by_side']['left'])
    assert all(value == 1 for value in counts['by_side']['right'])
    complete_support = trainer.segment_support_report(
        complete['complete-source'],
        role='train',
    )
    trainer.require_full_segment_support(complete_support)


def test_resampled_occurrence_enters_deterministic_augmentation_seed(tmp_path):
    dataset_root = tmp_path / 'dataset'
    left, right = _images(dataset_root)
    base = _record(
        'repeat-me',
        'source-a',
        image_paths={'left_rail_rgb': left, 'right_rail_rgb': right},
    )
    first = replace(base, draw_index=3, resample_occurrence=0)
    repeated = replace(base, draw_index=4, resample_occurrence=1)
    preprocessing = {
        'width': 320,
        'height': 240,
        'normalization_mean': [0.0, 0.0, 0.0],
        'normalization_std': [1.0, 1.0, 1.0],
        'train_augmentations': {
            'horizontal_flip': False,
            'camera_swap': False,
            'brightness': 0.0,
            'contrast': 0.0,
            'saturation': 0.0,
            'gamma_min': 1.0,
            'gamma_max': 1.0,
            'gaussian_blur_probability': 0.0,
            'gaussian_noise_probability': 1.0,
            'gaussian_noise_std': 0.05,
        },
    }
    first_image, _ = trainer.load_paired_image_arrays(
        first, preprocessing, training=True, seed=315, epoch=1
    )
    first_again, _ = trainer.load_paired_image_arrays(
        first, preprocessing, training=True, seed=315, epoch=1
    )
    repeated_image, _ = trainer.load_paired_image_arrays(
        repeated, preprocessing, training=True, seed=315, epoch=1
    )
    np.testing.assert_array_equal(first_image, first_again)
    assert not np.array_equal(first_image, repeated_image)


def test_public_topology_loader_uses_public_left_names_and_stable_fingerprint():
    config = trainer.load_v4_config(CONFIG_PATH)
    first = trainer.load_public_topology_contract(config)
    second = trainer.load_public_topology_contract(config)
    raw_left = rail_segment_lengths('left')

    assert first == second
    assert len(first['fingerprint_sha256']) == 64
    assert set(first['lengths_by_side']) == {'left', 'right'}
    assert tuple(first['lengths_by_side']['left']) == trainer.SEGMENT_CLASSES
    assert first['lengths_by_side']['left']['A14'] == pytest.approx(raw_left['A23'])
    assert first['lengths_by_side']['left']['A14'] != pytest.approx(raw_left['A14'])

    broken = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    broken['topology_contract']['length_loader'] = (
        'room_315_rail_defaults.rail_segment_lengths'
    )
    with pytest.raises(trainer.V4TrainerError, match='public ROS-label'):
        trainer.load_public_topology_contract(broken)


def test_checkpoint_selection_is_validation_lexicographic():
    base = {
        'planning_selection_score': 0.8,
        'planning_metrics': {
            'worst_side_x_segment_recall': {'recall': 0.6},
        },
        'correct_segment_s_m_p95_m': 0.2,
    }
    better_score = dict(base, planning_selection_score=0.81)
    better_worst_cell = {
        **base,
        'planning_metrics': {
            'worst_side_x_segment_recall': {'recall': 0.61},
        },
    }
    better_position = dict(base, correct_segment_s_m_p95_m=0.1)
    assert trainer.checkpoint_selection_key(better_score) > trainer.checkpoint_selection_key(base)
    assert trainer.checkpoint_selection_key(better_worst_cell) > trainer.checkpoint_selection_key(base)
    assert trainer.checkpoint_selection_key(better_position) > trainer.checkpoint_selection_key(base)

    with pytest.raises(trainer.V4TrainerError, match='finite'):
        trainer.checkpoint_selection_key(dict(base, planning_selection_score=float('nan')))


def test_run_metadata_is_atomically_finalized_after_completed_report(tmp_path):
    path = tmp_path / 'run_metadata.json'
    initial = {
        'schema_version': trainer.RUN_SCHEMA_VERSION,
        'status': 'running',
        'started_unix_s': 10.0,
        'canary_loaded': False,
        'test_loaded': False,
    }
    trainer._write_json_exclusive(path, initial)
    final_report = {
        'status': 'completed',
        'runtime_s': 12.5,
        'epochs_completed': 3,
        'selected_checkpoint': {
            'epoch': 2,
            'filename': 'epoch_002.pt',
            'sha256': 'a' * 64,
        },
        'validation_acceptance_status': 'passed',
        'promotion_status': 'pending_post_selection_canary',
        'canary_loaded': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
    }

    completed = trainer._finalize_run_metadata(path, initial, final_report)

    assert json.loads(path.read_text(encoding='utf-8')) == completed
    assert completed['status'] == 'completed'
    assert completed['runtime_s'] == pytest.approx(12.5)
    assert completed['epochs_completed'] == 3
    assert completed['selected_checkpoint'] == {
        'epoch': 2,
        'filename': 'epoch_002.pt',
        'sha256': 'a' * 64,
    }
    assert completed['validation_acceptance_status'] == 'passed'
    assert completed['promotion_status'] == 'pending_post_selection_canary'
    assert completed['completed_unix_s'] >= initial['started_unix_s']
    assert not list(tmp_path.glob('.run_metadata.json.*.tmp'))

    with pytest.raises(trainer.V4TrainerError, match='unexpectedly'):
        trainer._finalize_run_metadata(path, initial, final_report)


def test_approved_v3_baseline_is_bound_to_exact_validation_fingerprints_and_counts():
    config = trainer.load_v4_config(CONFIG_PATH)
    baseline = config['approved_v3_validation_baseline']
    support = {
        'by_side': {
            'left': [1085] + [0] * 13,
            'right': [1169] + [0] * 13,
        },
    }
    fingerprint = {
        'rows': {'sha256': baseline['validation_rows_sha256']},
        'labels': {'sha256': baseline['validation_labels_sha256']},
        'paired_records': 512,
    }

    verified = trainer.verify_approved_v3_validation_baseline(
        config,
        fingerprint,
        support,
    )
    assert verified['verified'] is True
    assert all(verified['provenance_checks'].values())

    broken = json.loads(json.dumps(fingerprint))
    broken['labels']['sha256'] = '0' * 64
    with pytest.raises(trainer.V4TrainerError, match='provenance mismatch'):
        trainer.verify_approved_v3_validation_baseline(config, broken, support)


def test_external_v3_canary_baseline_is_sha_and_exact_count_bound(tmp_path):
    checkpoint_path = tmp_path / 'v3_best.pt'
    checkpoint_path.write_bytes(b'v3 checkpoint')
    checkpoint_sha = trainer.sha256_file(checkpoint_path)
    prior_path = tmp_path / 'prior_canary.json'
    trainer._write_json_exclusive(prior_path, {'role': 'development_regression'})
    baseline = {
        'schema_version': trainer.V3_CANARY_BASELINE_SCHEMA_VERSION,
        'checkpoint_sha256': checkpoint_sha,
        'checkpoint_path': str(checkpoint_path),
        'canary_rows_sha256': 'b' * 64,
        'canary_labels_sha256': 'c' * 64,
        'sample_count': 2,
        'visible_count': 5,
        'visible_left_count': 3,
        'visible_right_count': 2,
        'loaded_correct_count': 4,
        'loaded_correct_left': 2,
        'loaded_correct_right': 2,
        'loaded_accuracy': 4 / 5,
        'loaded_accuracy_left': 2 / 3,
        'loaded_accuracy_right': 1.0,
        'segment_correct_count': 3,
        'segment_correct_left': 2,
        'segment_correct_right': 1,
        'segment_top1_accuracy': 3 / 5,
        'segment_top1_left': 2 / 3,
        'segment_top1_right': 1 / 2,
        'data_role': 'post_training_development_canary_baseline_only',
        'preprocessing_contract': trainer.V3_PREPROCESSING_CONTRACT,
        'camera_order': 'left_then_right',
        'training_performed': False,
        'validation_reproduction_guard_passed': True,
        'automatic_deployment_approval': False,
        'used_for_selection': False,
        'test_loaded': False,
        'prior_exposure_acknowledged': True,
        'prior_canary_artifact_path': str(prior_path),
        'prior_canary_artifact_sha256': trainer.sha256_file(prior_path),
    }
    path = tmp_path / 'baseline.json'
    trainer._write_json_exclusive(path, baseline)
    digest = trainer.sha256_file(path)
    fingerprint = {
        'paired_records': 2,
        'rows': {'sha256': 'b' * 64},
        'labels': {'sha256': 'c' * 64},
    }
    support = {
        'by_side': {
            'left': [3] + [0] * 13,
            'right': [2] + [0] * 13,
        },
    }

    verified = trainer.load_and_verify_approved_v3_canary_baseline(
        path,
        digest,
        {'initialization': {'checkpoint_sha256': checkpoint_sha}},
        fingerprint,
        support,
    )
    assert verified['verified'] is True
    assert all(verified['provenance_checks'].values())
    assert verified['artifact']['sha256'] == digest

    broken = dict(baseline, loaded_correct_count=3)
    broken_path = tmp_path / 'broken_baseline.json'
    trainer._write_json_exclusive(broken_path, broken)
    with pytest.raises(trainer.V4TrainerError, match='provenance mismatch'):
        trainer.load_and_verify_approved_v3_canary_baseline(
            broken_path,
            trainer.sha256_file(broken_path),
            {'initialization': {'checkpoint_sha256': checkpoint_sha}},
            fingerprint,
            support,
        )


def test_canary_coverage_contract_is_finalization_and_validation_bound(tmp_path):
    training_root = tmp_path / 'training_run'
    training_root.mkdir()
    validation_acceptance = {
        'status': 'passed',
        'per_gate': {
            'scene_presence_density.segment_top1.medium': {'status': 'passed'},
            'scene_presence_density.joint_005.medium': {'status': 'passed'},
        },
    }
    validation_path = training_root / 'validation_acceptance.json'
    trainer._write_json_exclusive(validation_path, validation_acceptance)
    finalization = {
        'schema_version': 'room315.hard_case_visual_v3r1.v1',
        'passed': True,
        'profile': 'canary',
        'scenario_count': 2,
        'image_count': 4,
        'rows_sha256': 'b' * 64,
        'labels_sha256': 'c' * 64,
        'image_hashes': {f'sample:{index}': 'd' * 64 for index in range(4)},
        'image_pair_hashes': {f'sample-{index}': 'e' * 64 for index in range(2)},
        'image_pair_hash_unique': True,
    }
    finalization_path = tmp_path / 'canary_finalization.json'
    trainer._write_json_exclusive(finalization_path, finalization)
    dataset_fingerprint = trainer.hashlib.sha256(
        trainer.canonical_json({
            'rows_sha256': 'b' * 64,
            'labels_sha256': 'c' * 64,
            'sample_count': 2,
            'image_hashes': dict(sorted(finalization['image_hashes'].items())),
        }).encode('utf-8')
    ).hexdigest()
    prior_path = tmp_path / 'prior_canary.json'
    trainer._write_json_exclusive(prior_path, {'role': 'development_regression'})
    historical_references = []
    for index, role in enumerate(
        ('old_replay_superset', 'v3r1_train', 'v3r1_validation')
    ):
        reference_path = tmp_path / f'{role}.json'
        trainer._write_json_exclusive(
            reference_path,
            {'image_hashes': {'historical:image': str(index) * 64}},
        )
        historical_references.append({
            'role': role,
            'path': str(reference_path),
            'sha256': trainer.sha256_file(reference_path),
            'hash_field': 'image_hashes',
        })
    contract = {
        'schema_version': trainer.CANARY_COVERAGE_SCHEMA_VERSION,
        'data_role': 'post_training_development_regression_only',
        'sample_count': 2,
        'canary_rows_sha256': 'b' * 64,
        'canary_labels_sha256': 'c' * 64,
        'canary_dataset_fingerprint_sha256': dataset_fingerprint,
        'required_scene_presence_densities': ['sparse', 'dense'],
        'scene_presence_record_counts': {'sparse': 1, 'medium': 0, 'dense': 1},
        'validation_carries_medium_presence': True,
        'canary_finalization_path': str(finalization_path),
        'canary_finalization_sha256': trainer.sha256_file(finalization_path),
        'validation_acceptance_sha256': trainer.sha256_file(validation_path),
        'prior_exposure_acknowledged': True,
        'prior_canary_artifact_path': str(prior_path),
        'prior_canary_artifact_sha256': trainer.sha256_file(prior_path),
        'historical_disjoint_reference_artifacts': historical_references,
        'used_for_selection': False,
        'test_loaded': False,
    }
    contract_path = tmp_path / 'coverage.json'
    trainer._write_json_exclusive(contract_path, contract)
    fingerprint = {
        'paired_records': 2,
        'rows': {'sha256': 'b' * 64},
        'labels': {'sha256': 'c' * 64},
    }

    verified = trainer.load_and_verify_canary_coverage_contract(
        contract_path,
        trainer.sha256_file(contract_path),
        {'data': {'canary': {'expected_rows': 2}}},
        fingerprint,
        training_root,
    )
    assert verified['verified'] is True
    assert all(verified['provenance_checks'].values())
    assert verified['required_scene_presence_densities'] == ['sparse', 'dense']


def test_canary_attempt_reservation_is_global_one_shot(tmp_path):
    output_root = tmp_path / 'outputs'
    run_root = tmp_path / 'run'
    checkpoint_dir = run_root / 'checkpoints'
    checkpoint_dir.mkdir(parents=True)
    checkpoint = checkpoint_dir / 'epoch_011.pt'
    checkpoint.write_bytes(b'checkpoint')
    trainer._write_json_exclusive(
        run_root / 'final_report.json', {'status': 'completed'}
    )
    baseline_path = tmp_path / 'baseline.json'
    trainer._write_json_exclusive(baseline_path, {'approved': True})
    coverage_path = tmp_path / 'coverage.json'
    trainer._write_json_exclusive(coverage_path, {'approved': True})
    finalization_sha = 'f' * 64
    dataset_fingerprint = 'e' * 64
    checkpoint_sha = trainer.sha256_file(checkpoint)
    key = trainer.hashlib.sha256(
        trainer.canonical_json({
            'checkpoint_sha256': checkpoint_sha,
            'canary_dataset_fingerprint_sha256': dataset_fingerprint,
        }).encode('utf-8')
    ).hexdigest()
    output = output_root / f'canary_v4_{key[:16]}_attempt1'
    coverage = {
        'canary_finalization_sha256': finalization_sha,
        'canary_dataset_fingerprint_sha256': dataset_fingerprint,
        'artifact': trainer.file_fingerprint(coverage_path),
        'prior_exposure_acknowledged': True,
        'prior_canary_artifact_sha256': '1' * 64,
    }

    reservation = trainer._reserve_canary_attempt(
        {'output_root': str(output_root)},
        {'effective_config_sha256': '2' * 64},
        checkpoint,
        checkpoint_sha,
        output,
        baseline_path,
        trainer.sha256_file(baseline_path),
        coverage,
    )
    assert Path(reservation['reservation_path']).is_file()
    assert reservation['attempt_key'] == key

    reserialized_coverage = dict(
        coverage,
        canary_finalization_sha256='0' * 64,
    )
    with pytest.raises(trainer.V4TrainerError, match='immutable artifact'):
        trainer._reserve_canary_attempt(
            {'output_root': str(output_root)},
            {'effective_config_sha256': '2' * 64},
            checkpoint,
            checkpoint_sha,
            output,
            baseline_path,
            trainer.sha256_file(baseline_path),
            reserialized_coverage,
        )


def test_canary_reservation_precedes_any_canary_record_load():
    source = inspect.getsource(trainer.evaluate_canary_v4)
    assert source.index('reservation = _reserve_canary_attempt(') < source.index(
        'canary_records, canary_fingerprint = load_paired_source('
    )


def test_finalization_image_keys_use_episode_id_not_step_sample_id(tmp_path):
    left = tmp_path / 'left.jpg'
    right = tmp_path / 'right.jpg'
    left.write_bytes(b'left image')
    right.write_bytes(b'right image')
    record = _record(
        'episode-1:step:0',
        'canary',
        image_paths={'left_rail_rgb': left, 'right_rail_rgb': right},
    )
    row = dict(record.row, episode_id='episode-1')
    record = replace(record, row=row, role='canary')
    finalization = {
        'image_hashes': {
            'episode-1:left_rail_rgb': trainer.sha256_file(left),
            'episode-1:right_rail_rgb': trainer.sha256_file(right),
        },
    }
    finalization_path = tmp_path / 'finalization.json'
    trainer._write_json_exclusive(finalization_path, finalization)

    report = trainer.verify_images_against_finalization(
        [record], finalization_path
    )
    assert report['matches_frozen_finalization'] is True
    assert report['frozen_image_hash_count'] == 2


def test_disjoint_inputs_are_bound_to_training_time_rows_and_labels(tmp_path):
    training_root = tmp_path / 'run'
    training_root.mkdir()
    train_fingerprint = {
        'paired_records': 2,
        'dataset_root': str(tmp_path / 'train_data'),
        'rows': {'sha256': 'a' * 64, 'bytes': 10},
        'labels': {'sha256': 'b' * 64, 'bytes': 20},
    }
    validation_fingerprint = {
        'paired_records': 1,
        'dataset_root': str(tmp_path / 'validation_data'),
        'rows': {'sha256': 'c' * 64, 'bytes': 30},
        'labels': {'sha256': 'd' * 64, 'bytes': 40},
    }
    trainer._write_json_exclusive(
        training_root / 'input_fingerprints.json',
        {
            'train_sources': {'source': train_fingerprint},
            'validation': validation_fingerprint,
        },
    )

    report = trainer.verify_loaded_sources_against_training_fingerprints(
        training_root,
        {'source': train_fingerprint},
        validation_fingerprint,
    )
    assert report['all_current_rows_and_labels_match_training_time'] is True

    changed = json.loads(json.dumps(validation_fingerprint))
    changed['rows']['sha256'] = 'e' * 64
    with pytest.raises(trainer.V4TrainerError, match='frozen training'):
        trainer.verify_loaded_sources_against_training_fingerprints(
            training_root,
            {'source': train_fingerprint},
            changed,
        )


def test_canary_handoff_requires_completion_ledger_bound_to_final_report(tmp_path):
    completion_path = tmp_path / 'completion.json'
    report_path = tmp_path / 'final_report.json'
    attempt_key = 'a' * 64
    report = {
        'status': 'completed',
        'acceptance_status': 'passed',
        'promotion_status': 'eligible_for_manual_runtime_review',
        'canary_loaded': True,
        'checkpoint_selection_performed': False,
        'canary_used_for_checkpoint_selection': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
        'canary_attempt': {
            'attempt_key': attempt_key,
            'completion_path': str(completion_path),
            'completion_ledger_required_for_trust': True,
        },
    }
    trainer._write_json_exclusive(report_path, report)
    completion = {
        'state': 'completed_immutable',
        'attempt_key': attempt_key,
        'test_loaded': False,
        'automatic_runtime_switch': False,
        'artifacts': {
            'final_report.json': trainer.file_fingerprint(report_path),
        },
    }
    trainer._write_json_exclusive(completion_path, completion)

    trusted = trainer.verify_completed_canary_handoff(report_path)
    assert trusted['trusted'] is True
    assert all(trusted['checks'].values())


def test_loss_wrapper_only_maps_config_into_tested_v4_loss_api():
    config = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    predictions = {
        'segment_logits': object(),
        'loaded_logits': object(),
        'bbox': object(),
        's_ratio': object(),
    }
    targets = {
        'segment': object(),
        'loaded': object(),
        'bbox': object(),
        's_ratio': object(),
        'visibility_mask': object(),
    }

    class FakeTrainingApi:
        def __init__(self):
            self.call = None

        def compute_v4_loss(self, passed_predictions, passed_targets, **kwargs):
            self.call = (passed_predictions, passed_targets, kwargs)
            return {'sentinel': True}

    api = FakeTrainingApi()
    weights = [[1.0] * 14, [2.0] * 14]
    distances = [[0.0] * 14 for _ in range(14)]
    result = trainer.compute_configured_loss(
        predictions,
        targets,
        class_weights=weights,
        topology_distances=distances,
        loss_config=config['loss'],
        torch_module=object(),
        training_api=api,
    )
    assert result == {'sentinel': True}
    assert api.call[0] is predictions
    assert api.call[1] is targets
    kwargs = api.call[2]
    assert kwargs['segment_class_weights'] is weights
    assert kwargs['topology_distance_matrix'] is distances
    assert kwargs['segment_label_smoothing'] == pytest.approx(0.02)
    assert kwargs['loaded_label_smoothing'] == pytest.approx(0.01)
    assert kwargs['s_ratio_beta'] == pytest.approx(0.05)
    assert kwargs['loss_weights'] == {
        'segment': 4.0,
        's_ratio': 2.0,
        'loaded': 1.0,
        'bbox': 1.0,
        'topology': 0.25,
    }


def test_split_disjoint_audit_fails_closed_on_sample_overlap():
    train_record = _record('same-sample', 'train-source')
    validation_record = replace(
        _record('same-sample', 'validation-source'),
        role='validation',
    )
    with pytest.raises(trainer.V4TrainerError, match='split overlap'):
        trainer.split_disjoint_audit(
            {'train-source': [train_record]},
            [validation_record],
        )


def test_camera_counterfactuals_measure_exact_rail_isolation_and_swap_drop(monkeypatch):
    torch = pytest.importorskip('torch')
    import room_315_visual_training_v4 as training_api

    class RailLocalModel(torch.nn.Module):
        def forward(self, image):
            batch = int(image.shape[0])
            left_value = image[:, :3].mean(dim=(1, 2, 3))
            right_value = image[:, 3:].mean(dim=(1, 2, 3))
            segment_logits = torch.zeros(batch, 8, 14, device=image.device)
            for start, values in ((0, left_value), (4, right_value)):
                segment_logits[:, start:start + 4, 0] = values[:, None] * 10.0
                segment_logits[:, start:start + 4, 1] = -values[:, None] * 10.0
            return {
                'segment_logits': segment_logits,
                'loaded_logits': torch.zeros(batch, 8, 2, device=image.device),
                'bbox': torch.tensor(
                    [0.1, 0.1, 0.2, 0.2], device=image.device
                ).reshape(1, 1, 4).expand(batch, 8, 4),
                's_ratio': torch.full((batch, 8, 1), 0.5, device=image.device),
            }

    image = torch.empty(2, 6, 4, 4)
    image[0, :3] = 1.0
    image[0, 3:] = -1.0
    image[1, :3] = 2.0
    image[1, 3:] = -2.0
    target_segment = torch.zeros(2, 8, dtype=torch.long)
    target_segment[:, 4:] = 1
    batch = {
        'image': image,
        'segment': target_segment,
        'loaded': torch.zeros(2, 8, dtype=torch.long),
        'bbox': torch.tensor([0.1, 0.1, 0.2, 0.2]).reshape(1, 1, 4).expand(2, 8, 4),
        's_ratio': torch.full((2, 8, 1), 0.5),
        'visibility_mask': torch.ones(2, 8, dtype=torch.bool),
        'segment_length_m': torch.ones(2, 8, 1),
        's_m': torch.full((2, 8, 1), 0.5),
    }
    monkeypatch.setattr(trainer, 'make_loader', lambda *args, **kwargs: [batch])
    config = {
        'training': {
            'automatic_mixed_precision': False,
            'batch_size': 2,
            'seed': 315,
        },
        'image_preprocessing': {
            'normalization_mean': [0.0, 0.0, 0.0],
            'normalization_std': [1.0, 1.0, 1.0],
        },
    }
    lengths = {
        side: {segment: 1.0 for segment in trainer.SEGMENT_CLASSES}
        for side in trainer.SIDES
    }

    report = trainer.evaluate_camera_counterfactuals(
        RailLocalModel(),
        [object(), object()],
        config,
        torch_module=torch,
        training_api=training_api,
        device=torch.device('cpu'),
        topology_lengths=lengths,
    )

    assert report['maximum_cross_side_output_change'] == 0.0
    assert report['blank_opposite_camera']['maximum_own_side_output_change'] == 0.0
    assert report['shuffle_opposite_camera']['maximum_own_side_output_change'] == 0.0
    assert report['effective_shuffle_batches'] == 1
    assert report['camera_order_swap']['baseline_segment_top1'] == 1.0
    assert report['camera_order_swap']['swapped_segment_top1'] == 0.0
    assert report['camera_order_swap']['segment_top1_drop'] == 1.0
