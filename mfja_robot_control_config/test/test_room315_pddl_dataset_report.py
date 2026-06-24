#!/usr/bin/env python3

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_pddl_dataset_report.py'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_pddl_dataset_report', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _action(primitive='SHUTTLE_ON', side='right', speed=0.3):
    return {
        'primitive': primitive,
        'side': side,
        'switch_mask': {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0},
        'switch_values': {
            'A1': 'UNCHANGED',
            'A2': 'UNCHANGED',
            'A3': 'UNCHANGED',
            'A4': 'UNCHANGED',
        },
        'stopper_mask': {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0},
        'stopper_values': {
            'A1': 'UNCHANGED',
            'A2': 'UNCHANGED',
            'A3': 'UNCHANGED',
            'A4': 'UNCHANGED',
        },
        'speed_mps': speed,
        'wait_condition': 'shuttle_command_applied',
        'target_id': f'{side}_shuttle',
        'reason': 'shuttle_start',
    }


def _pddl_row(episode_id='episode_000001', step_index=0, goal='right_shuttle at staubli'):
    plan = [
        'prepare_switches right yaskawa staubli',
        'open_stoppers right yaskawa staubli',
        'move_shuttle right right_shuttle yaskawa staubli speed=0.3',
        'stop_shuttle right right_shuttle',
        'finish_task right_shuttle staubli',
    ]
    return {
        'episode_id': episode_id,
        'step_index': step_index,
        'event_type': 'command',
        'task': 'move the right shuttle from Yaskawa to Staubli',
        'planning_source': 'pddl',
        'pddl_problem': 'problem_right_yaskawa_to_staubli.pddl',
        'pddl_goal': goal,
        'symbolic_plan': plan,
        'plan_step_index': step_index,
        'generated_language': 'move the right shuttle from Yaskawa to Staubli',
        'language_template_id': 'move_from_to',
        'model_input': {
            'language': 'move the right shuttle from Yaskawa to Staubli',
            'overhead_images': {},
            'last_command': {'action': 'START'},
        },
        'action': _action('SHUTTLE_ON', 'right', 0.3),
        'safety_decoder_metrics': {
            'total_proposed_actions': 10,
            'rejected_actions': 1,
        },
        'event_generation_metrics': {
            'task_success': True,
        },
    }


def _manual_row(episode_id='episode_manual', side='left'):
    return {
        'episode_id': episode_id,
        'step_index': 0,
        'task': 'move the left shuttle from Yaskawa to KUKA',
        'model_input': {
            'language': 'move the left shuttle from Yaskawa to KUKA',
            'overhead_images': {},
            'last_command': {'action': 'START'},
        },
        'action': _action('STOP_NOW', side, 0.0),
    }


def _write_events(dataset_dir, episode_id, rows, *, approved=True, failure_reason=''):
    event_dir = dataset_dir / 'episodes' / episode_id
    event_dir.mkdir(parents=True)
    event_file = event_dir / 'events.jsonl'
    event_file.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )
    validation = {
        'scenario_id': episode_id,
        'goal_id': episode_id,
        'source': 'pddl_plansys',
        'validation_status': 'approved' if approved else 'failed',
        'approved_for_training': approved,
        'failure_reason': '' if approved else (failure_reason or 'test failure'),
        'task_success': approved,
        'rejected_action_rate': 0.0 if approved else 1.0,
        'wrong_shuttle_command_count': 0,
        'headway_violation_count': 0,
        'block_occupancy_violation_count': 0,
        'block_reservation_rejection_count': 0,
        'deadlock_detected_count': 0,
        'deadlock_avoided_count': 0,
    }
    (event_dir / 'validation.json').write_text(
        json.dumps(validation) + '\n',
        encoding='utf-8',
    )
    return event_file


def test_report_loads_events_jsonl(tmp_path):
    reporter = _load_module()
    dataset = tmp_path / 'dataset'
    _write_events(dataset, 'episode_000001', [_pddl_row(), _pddl_row(step_index=1)])

    rows, dataset_root, event_files = reporter.load_event_rows(dataset)

    assert dataset_root == dataset.resolve()
    assert len(event_files) == 1
    assert len(rows) == 2
    assert rows[0]['episode_id'] == 'episode_000001'


def test_report_detects_pddl_generated_metadata(tmp_path):
    reporter = _load_module()
    dataset = tmp_path / 'dataset'
    _write_events(dataset, 'episode_000001', [_pddl_row()])

    report = reporter.build_dataset_report(dataset)

    assert report['dataset_source'] == 'pddl'
    assert report['dataset_source_distribution'] == {'pddl': 1}
    assert report['episodes'][0]['pddl_goal'] == 'right_shuttle at staubli'
    assert report['episodes'][0]['pddl_problem'] == 'problem_right_yaskawa_to_staubli.pddl'
    assert report['episodes'][0]['symbolic_plan_length'] == 5
    assert report['episodes'][0]['generated_language_template_id'] == 'move_from_to'


def test_report_counts_goals(tmp_path):
    reporter = _load_module()
    dataset = tmp_path / 'dataset'
    _write_events(dataset, 'episode_000001', [_pddl_row(goal='right_shuttle at staubli')])
    _write_events(dataset, 'episode_000002', [
        _pddl_row(
            episode_id='episode_000002',
            goal='left_shuttle at kuka',
        )
    ])

    report = reporter.build_dataset_report(dataset)

    assert report['number_of_episodes'] == 2
    assert report['number_of_events'] == 2
    assert report['goals_covered'] == ['left_shuttle at kuka', 'right_shuttle at staubli']
    assert report['language_variants_count'] == 1
    assert report['average_plan_length'] == 5.0
    assert report['action_primitive_distribution'] == {'SHUTTLE_ON': 2}
    assert report['side_distribution'] == {'right': 2}
    assert report['speed_distribution'] == {'0.3': 2}
    assert report['rejected_action_rate'] == 0.1
    assert report['task_success']['success_rate'] == 1.0


def test_report_separates_approved_and_failed_episodes(tmp_path):
    reporter = _load_module()
    dataset = tmp_path / 'dataset'
    _write_events(dataset, 'episode_approved', [_pddl_row(episode_id='episode_approved')])
    _write_events(
        dataset,
        'episode_failed',
        [_pddl_row(episode_id='episode_failed', goal='left_shuttle at kuka')],
        approved=False,
        failure_reason='arrival timeout',
    )

    report = reporter.build_dataset_report(dataset)

    assert report['total_episodes'] == 2
    assert report['approved_episodes'] == 1
    assert report['failed_episodes'] == 1
    assert report['approval_rate'] == 0.5
    assert report['approved_event_count'] == 1
    assert report['skipped_event_count'] == 1
    assert report['failure_reasons_distribution'] == {'arrival timeout': 1}
    assert report['goals_covered'] == ['right_shuttle at staubli']


def test_report_handles_datasets_without_pddl_metadata(tmp_path):
    reporter = _load_module()
    dataset = tmp_path / 'dataset'
    _write_events(dataset, 'episode_manual', [_manual_row()])

    report = reporter.build_dataset_report(dataset)

    assert report['dataset_source'] == 'manual'
    assert report['dataset_source_distribution'] == {'manual': 1}
    assert report['goals_covered'] == ['move the left shuttle from Yaskawa to KUKA']
    assert report['average_plan_length'] == 0.0
    assert report['generated_language_template_ids'] == []
    assert report['episodes'][0]['pddl_goal'] == ''
    assert report['episodes'][0]['symbolic_plan_length'] == 0


def test_report_exports_json(tmp_path):
    dataset = tmp_path / 'dataset'
    _write_events(dataset, 'episode_000001', [_pddl_row()])
    output = tmp_path / 'report.json'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(dataset),
            '--output',
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    stdout_report = json.loads(result.stdout)
    saved_report = json.loads(output.read_text(encoding='utf-8'))
    assert stdout_report['number_of_events'] == 1
    assert saved_report == stdout_report
