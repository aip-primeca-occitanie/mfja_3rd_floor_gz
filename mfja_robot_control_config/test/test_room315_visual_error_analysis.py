#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path

import pytest
from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_SCRIPT = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_visual_error_analysis.py'
)


def _load_analysis(name='room_315_visual_error_analysis_test'):
    spec = importlib.util.spec_from_file_location(name, ANALYSIS_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _prediction_record(episode_id, value, *, present=1):
    return {
        'episode_id': episode_id,
        'presence_mask': present,
        'identity_correct': value < 5,
        'loaded_state_correct': value < 5,
        'side_correct': value < 5,
        'block_correct': value < 5,
        'full_location_correct': value < 5,
        'absolute_s_m_error': float(value),
        'absolute_s_ratio_error': float(value) / 10.0,
        'bbox_iou': max(0.0, 1.0 - float(value) / 10.0),
        'top2_predicted_blocks': [{'class': 'A14', 'score': 1.0}],
        'ground_truth_block': 'A14',
    }


def test_prediction_jsonl_and_csv_exports_are_deterministic(tmp_path):
    analysis = _load_analysis('room315_error_analysis_deterministic_export')
    records = [
        {'episode_id': 'episode_b', 'slot': 1, 'scores': {'R2': 0.2, 'L2': 0.8}},
        {'episode_id': 'episode_a', 'slot': 0, 'scores': {'R1': 0.9, 'L1': 0.1}},
    ]
    first_jsonl = tmp_path / 'first.jsonl'
    second_jsonl = tmp_path / 'second.jsonl'
    first_csv = tmp_path / 'first.csv'
    second_csv = tmp_path / 'second.csv'

    analysis.write_jsonl(first_jsonl, records)
    analysis.write_jsonl(second_jsonl, records)
    analysis.write_records_csv(first_csv, records)
    analysis.write_records_csv(second_csv, records)

    assert analysis.sha256_file(first_jsonl) == analysis.sha256_file(second_jsonl)
    assert analysis.sha256_file(first_csv) == analysis.sha256_file(second_csv)
    assert [
        json.loads(line)
        for line in first_jsonl.read_text(encoding='utf-8').splitlines()
    ] == records


def test_missing_shuttle_slot_is_masked_and_has_no_decoded_prediction(tmp_path):
    analysis = _load_analysis('room315_error_analysis_missing_slot')
    for camera in ('left', 'right'):
        path = tmp_path / f'{camera}.jpg'
        Image.new('RGB', (8, 8), color=(0, 0, 0)).save(path)
    row = {
        'episode_id': 'visual_0001_case_hash',
        'scenario_family': 'family',
        'model_input': {
            'overhead_images': {
                'left_rail_rgb': 'left.jpg',
                'right_rail_rgb': 'right.jpg',
            },
        },
    }
    label = {
        'shuttles': [
            {'visually_available_identity': 'L1'},
            {'visually_available_identity': 'L2'},
        ],
    }

    record = analysis.prediction_record(
        split_name='val',
        row=row,
        label=label,
        prediction=analysis.np.zeros(1),
        label_names=[],
        slot=2,
        dataset_root=tmp_path,
    )

    assert record['presence_mask'] == 0
    assert record['ground_truth_identity'] is None
    assert record['predicted_identity'] is None
    assert record['raw_identity_scores'] is None
    assert record['bbox_iou'] is None
    assert record['identity_correct'] is None


def test_confusion_matrix_counts_and_class_metrics(tmp_path):
    analysis = _load_analysis('room315_error_analysis_confusion')
    report = analysis.confusion_report([
        ('loaded', 'loaded'),
        ('loaded', 'empty'),
        ('empty', 'empty'),
        ('empty', 'empty'),
    ])
    loaded = report['classes'].index('loaded')
    empty = report['classes'].index('empty')

    assert report['matrix'][loaded][loaded] == 1
    assert report['matrix'][loaded][empty] == 1
    assert report['matrix'][empty][empty] == 2
    assert report['accuracy'] == pytest.approx(0.75)
    assert report['balanced_accuracy'] == pytest.approx(0.75)
    assert report['per_class']['loaded']['support'] == 2
    assert report['per_class']['empty']['predicted_count'] == 3
    analysis.write_confusion_csvs(tmp_path / 'nested', 'loaded', report)
    assert (tmp_path / 'nested' / 'loaded_confusion.csv').is_file()
    assert (tmp_path / 'nested' / 'loaded_metrics.csv').is_file()
    assert (tmp_path / 'nested' / 'loaded_summary.csv').is_file()


def test_scenario_bootstrap_keeps_shuttles_from_one_scenario_grouped():
    analysis = _load_analysis('room315_error_analysis_bootstrap')
    records = [
        _prediction_record('scenario_a', 0.0),
        _prediction_record('scenario_a', 0.0),
        _prediction_record('scenario_b', 10.0),
    ]
    groups = analysis.scenario_groups(records)
    metric = lambda sample: sum(row['absolute_s_m_error'] for row in sample) / len(sample)

    first = analysis.scenario_bootstrap_ci(
        records,
        metric,
        replicates=200,
        seed=17,
    )
    second = analysis.scenario_bootstrap_ci(
        records,
        metric,
        replicates=200,
        seed=17,
    )

    assert sorted(len(group) for group in groups) == [1, 2]
    assert first == second
    assert first['scenario_count'] == 2
    assert first['point_estimate'] == pytest.approx(10.0 / 3.0)
    assert 'all present shuttles retained together' in first['resampling_unit']


def test_bbox_iou_and_bbox_error_calculations():
    analysis = _load_analysis('room315_error_analysis_bbox')

    assert analysis.bbox_iou_xywh([0, 0, 10, 10], [0, 0, 10, 10]) == 1.0
    assert analysis.bbox_iou_xywh([0, 0, 10, 10], [20, 20, 5, 5]) == 0.0
    assert analysis.bbox_iou_xywh([0, 0, 10, 10], [5, 5, 10, 10]) == pytest.approx(
        25.0 / 175.0
    )
    errors = analysis.bbox_errors([0, 0, 10, 20], [2, 4, 14, 18])
    assert errors['bbox_coordinate_errors'] == [2.0, 4.0, 4.0, -2.0]
    assert errors['bbox_size_error'] == pytest.approx((4.0 ** 2 + 2.0 ** 2) ** 0.5)


def test_identity_semantics_audit_detects_slot_conditioned_schema():
    analysis = _load_analysis('room315_error_analysis_identity')
    names = [
        'shuttles.0.visually_available_identity==l1',
        'shuttles.0.visually_available_identity==r1',
        'shuttles.1.visually_available_identity==l2',
        'shuttles.1.visually_available_identity==r2',
        'shuttles.2.visually_available_identity==l3',
        'shuttles.2.visually_available_identity==r3',
    ]
    labels = {
        'val': [
            {
                'shuttles': [
                    {'visually_available_identity': 'L1'},
                    {'visually_available_identity': 'L2'},
                    {'visually_available_identity': 'L3'},
                ],
            },
            {
                'shuttles': [
                    {'visually_available_identity': 'R1'},
                    {'visually_available_identity': 'R2'},
                ],
            },
        ],
        'test': [],
    }

    audit = analysis.audit_identity_semantics(labels, names)

    assert audit['status'] == 'PASS'
    assert audit['is_true_unconstrained_six_class_metric'] is False
    assert audit['is_slot_conditioned_left_right_metric'] is True
    assert audit['slot_order_depends_on_ground_truth_identity'] is True
    assert audit['data_leakage_demonstrated'] is False
    assert audit['recommended_metric_name'] == 'slot_conditioned_identity_accuracy'


def test_latency_protocol_excludes_all_warmup_calls_from_statistics():
    analysis = _load_analysis('room315_error_analysis_latency')
    inference_calls = []
    synchronized = []
    clock_values = iter([0.0, 0.5, 1.0, 1.1, 2.0, 2.2, 3.0, 3.3])

    report = analysis.latency_protocol(
        lambda: inference_calls.append(len(inference_calls)),
        synchronize=lambda: synchronized.append(len(synchronized)),
        warmup_iterations=20,
        timed_iterations=3,
        clock=lambda: next(clock_values),
    )

    assert len(inference_calls) == 1 + 20 + 3
    assert report['cold_start_latency_seconds'] == pytest.approx(0.5)
    assert report['warmup_iterations_excluded'] == 20
    assert report['steady_state']['sample_count'] == 3
    assert report['steady_state']['mean_seconds'] == pytest.approx(0.2)


def test_frozen_checkpoint_and_existing_pilot_file_snapshot_detects_changes(tmp_path):
    analysis = _load_analysis('room315_error_analysis_immutability')
    pilot = tmp_path / 'pilot'
    package = tmp_path / 'package'
    pilot.mkdir()
    package.mkdir()
    checkpoint = pilot / 'best.pt'
    metrics = pilot / 'metrics.json'
    dataset = package / 'split.jsonl'
    checkpoint.write_bytes(b'frozen-checkpoint')
    metrics.write_text('{"status":"PASS"}\n', encoding='utf-8')
    dataset.write_text('{"sample":1}\n', encoding='utf-8')
    snapshot = analysis.snapshot_existing_files({
        'pilot': pilot,
        'package': package,
    })

    assert analysis.verify_snapshot_unchanged(snapshot)['status'] == 'PASS'
    (pilot / 'new_analysis_file.json').write_text('{}\n', encoding='utf-8')
    assert analysis.verify_snapshot_unchanged(snapshot)['status'] == 'PASS'
    metrics.write_text('{"status":"CHANGED"}\n', encoding='utf-8')
    changed = analysis.verify_snapshot_unchanged(snapshot)
    assert changed['status'] == 'FAIL'
    assert changed['changed_files'][0]['key'] == 'pilot/metrics.json'
    assert analysis.sha256_file(checkpoint) == snapshot['files']['pilot/best.pt']['sha256']
