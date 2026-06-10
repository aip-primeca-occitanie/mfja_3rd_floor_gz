#!/usr/bin/env python3

import csv
import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_baseline_eval.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_vla_baseline_eval', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _action(primitive='SET_SWITCHES', side='right', target_id='A3', reason='switch_update'):
    return {
        'primitive': primitive,
        'side': side,
        'switch_mask': {'A1': 0, 'A2': 0, 'A3': 1, 'A4': 0},
        'switch_values': {
            'A1': 'UNCHANGED',
            'A2': 'UNCHANGED',
            'A3': 'INTERIOR',
            'A4': 'UNCHANGED',
        },
        'stopper_mask': {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0},
        'stopper_values': {
            'A1': 'UNCHANGED',
            'A2': 'UNCHANGED',
            'A3': 'UNCHANGED',
            'A4': 'UNCHANGED',
        },
        'wait_condition': 'switch_state_match',
        'target_id': target_id,
        'reason': reason,
    }


def _row(
    tmp_path,
    *,
    episode_id='episode_000001',
    image_name='right.jpg',
    action=None,
    task='route right shuttle through A3 into the interior branch',
    timestamp=1.0,
    safety_metrics=None,
    event_generation_metrics=None,
    task_status='',
):
    (tmp_path / image_name).write_bytes(f'image-bytes-{image_name}'.encode('utf-8'))
    return {
        'episode_id': episode_id,
        'step_index': 0,
        'timestamp': timestamp,
        'task': task,
        'task_status': task_status,
        'safety_decoder_metrics': safety_metrics or {},
        'model_input': {
            'language': task,
            'last_command': {'action': 'status'},
            'overhead_images': {'right_rail_rgb': image_name},
        },
        'observation.images.right_rail_rgb': image_name,
        'observation.state': [0.0, 1.0, 0.0],
        'action': action or _action(),
        'event_generation_metrics': event_generation_metrics or {},
        'privileged_eval': {
            'raw_shuttle_states': {
                'right': {
                    'room315_right_shuttle_1': {
                        'segment': 'A12E',
                        's': 0.42,
                        'x': 1.0,
                    }
                }
            }
        },
    }


def test_baseline_eval_writes_json_and_csv_metrics(tmp_path):
    baseline = _load_module()
    rows = [
        _row(tmp_path, episode_id='episode_000001', image_name='a.jpg'),
        _row(tmp_path, episode_id='episode_000001', image_name='b.jpg'),
        _row(tmp_path, episode_id='episode_000002', image_name='c.jpg', action=_action('STOP_NOW', 'right', 'right_shuttle')),
    ]
    input_path = tmp_path / 'training_events.jsonl'
    input_path.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )
    output_dir = tmp_path / 'metrics'

    summary = baseline.run_baseline_eval(
        input_path,
        output_dir,
        holdout_fraction=0.0,
        seed=7,
    )

    metrics_json = output_dir / 'baseline_metrics.json'
    metrics_csv = output_dir / 'baseline_metrics.csv'
    baseline_family_csv = output_dir / 'baseline_task_family_metrics.csv'
    task_family_csv = output_dir / 'task_family_metrics.csv'
    assert Path(summary['metrics_json']) == metrics_json
    assert Path(summary['metrics_csv']) == metrics_csv
    assert Path(summary['baseline_task_family_metrics_csv']) == baseline_family_csv
    assert Path(summary['task_family_metrics_csv']) == task_family_csv
    parsed = json.loads(metrics_json.read_text(encoding='utf-8'))
    assert {item['baseline'] for item in parsed['baselines']} == {
        'state_only',
        'vla',
        'oracle',
    }
    assert parsed['split_mode'] == 'train_equals_eval'
    assert parsed['rows'] == 3
    assert next(
        item for item in parsed['baselines'] if item['baseline'] == 'oracle'
    )['exact_action_accuracy'] == 1.0
    assert next(
        item for item in parsed['baselines'] if item['baseline'] == 'oracle'
    )['action_accuracy'] == 1.0
    assert 'baseline_task_family_metrics' in parsed
    assert 'task_family_metrics' in parsed

    with metrics_csv.open('r', encoding='utf-8') as stream:
        csv_rows = list(csv.DictReader(stream))
    assert [row['baseline'] for row in csv_rows] == ['state_only', 'vla', 'oracle']
    assert csv_rows[0]['exact_action_accuracy']
    assert csv_rows[0]['device_accuracy']
    assert baseline_family_csv.exists()
    assert task_family_csv.exists()


def test_state_and_vla_features_exclude_privileged_pose(tmp_path):
    baseline = _load_module()
    row = baseline._event_row(_row(tmp_path), 0)

    state_key = baseline.state_only_feature(row)
    vla_key = baseline.vla_feature(row, tmp_path)

    for key in (state_key, vla_key):
        assert 'A12E' not in key
        assert '"x"' not in key
        assert '"s"' not in key
        assert 'raw_shuttle_states' not in key


def test_vla_feature_changes_with_image_content(tmp_path):
    baseline = _load_module()
    row_a = baseline._event_row(_row(tmp_path, image_name='a.jpg'), 0)
    row_b = baseline._event_row(_row(tmp_path, image_name='b.jpg'), 1)

    assert baseline.state_only_feature(row_a) == baseline.state_only_feature(row_b)
    assert baseline.vla_feature(row_a, tmp_path) != baseline.vla_feature(row_b, tmp_path)


def test_vla_feature_uses_images_not_sensor_state(tmp_path):
    baseline = _load_module()
    row_a = baseline._event_row(_row(tmp_path, image_name='same.jpg'), 0)
    row_b = baseline._event_row(_row(tmp_path, image_name='same.jpg'), 1)
    row_b['observation_state'] = [1.0, 0.0, 1.0]

    assert baseline.state_only_feature(row_a) != baseline.state_only_feature(row_b)
    assert baseline.vla_feature(row_a, tmp_path) == baseline.vla_feature(row_b, tmp_path)


def test_task_family_metrics_include_requested_success_rates(tmp_path):
    baseline = _load_module()
    terminal = _action('DONE', 'right', 'terminal', 'task_succeeded')
    rows = [
        baseline._event_row(_row(
            tmp_path,
            episode_id='episode_visual',
            task='left_slot3_kuka_then_slot2',
            image_name='visual_done.jpg',
            timestamp=1.0,
        ), 0),
        baseline._event_row(_row(
            tmp_path,
            episode_id='episode_visual',
            task='left_slot3_kuka_then_slot2',
            image_name='visual_done.jpg',
            timestamp=4.0,
            action=terminal,
            task_status='succeeded',
            safety_metrics={
                'total_proposed_actions': 10,
                'rejected_actions': 2,
                'illegal_proposal_rate': 0.2,
            },
            event_generation_metrics={
                'event_candidate_count': 10,
                'recorded_event_count': 8,
                'skipped_redundant_event_count': 2,
                'noop_action_count': 2,
            },
        ), 1),
        baseline._event_row(_row(
            tmp_path,
            episode_id='episode_obstacle',
            task='right_obstacle_aware_route',
            image_name='obstacle_done.jpg',
            timestamp=3.0,
            action=terminal,
            task_status='succeeded',
        ), 2),
    ]

    metrics = {
        item['task_family']: item
        for item in baseline.task_family_metrics(rows)
    }

    assert 'unknown_position' not in metrics
    assert 'sensor_dropout' not in metrics
    assert metrics['visual_target']['task_success'] == 1.0
    assert metrics['visual_target']['completion_time'] == 3.0
    assert metrics['visual_target']['command_count'] == 2.0
    assert metrics['visual_target']['illegal_proposal_rate'] == 0.2
    assert metrics['visual_target']['rejected_action_rate'] == 0.2
    assert metrics['visual_target']['skipped_redundant_event_count'] == 2.0
    assert metrics['visual_target']['redundant_action_rate'] == 0.2
    assert metrics['visual_target']['noop_action_rate'] == 0.2
    assert metrics['visual_target']['effective_action_rate'] == 0.8
    assert metrics['visual_target']['visual_target_success'] == 1.0
    assert metrics['obstacle_stop']['obstacle_stop_success'] == 1.0


def test_device_accuracy_checks_switch_and_stopper_masks(tmp_path):
    baseline = _load_module()
    expected = baseline._event_row(_row(tmp_path, action=_action()), 0)
    matching = _action()
    wrong_device = _action()
    wrong_device['switch_mask'] = {'A1': 1, 'A2': 0, 'A3': 0, 'A4': 0}
    wrong_device['switch_values'] = {
        'A1': 'INTERIOR',
        'A2': 'UNCHANGED',
        'A3': 'UNCHANGED',
        'A4': 'UNCHANGED',
    }

    assert baseline._device_accuracy([expected], [matching]) == 1.0
    assert baseline._device_accuracy([expected], [wrong_device]) == 0.0
