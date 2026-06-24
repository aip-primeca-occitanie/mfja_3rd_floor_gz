#!/usr/bin/env python3

import copy
import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'


def _load_script(name: str):
    path = SCRIPT_DIR / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakePlanSysBackend:
    def plan(self, spec, *, speed):
        return [
            f'prepare_switches {spec.side} {spec.source} {spec.target}',
            f'open_stoppers {spec.side} {spec.source} {spec.target}',
            (
                f'move_shuttle {spec.side} {spec.shuttle} '
                f'{spec.source} {spec.target} speed={float(speed):.4g}'
            ),
            f'stop_shuttle {spec.side} {spec.shuttle}',
            f'finish_task {spec.shuttle} {spec.target}',
        ]


def _scenario():
    generator = _load_script('room_315_pddl_scenario_generator')
    return generator.generate_scenario(
        goal='right_yaskawa_to_staubli',
        planner=FakePlanSysBackend(),
    )


def _approved_validation(scenario=None, **overrides):
    gate = _load_script('room_315_pddl_validation_gate')
    scenario = scenario or _scenario()
    execution_result = {
        'success': True,
        'task_success': True,
        'final_goal_satisfied': True,
        'published_commands': list(scenario['primitive_commands']),
        'executed_command_count': len(scenario['primitive_commands']),
    }
    execution_result.update(overrides.pop('execution_result', {}))
    metrics = {
        'total_proposed_actions': len(scenario['primitive_commands']),
        'rejected_actions': 0,
        'rejected_action_rate': 0.0,
        'wrong_shuttle_command_count': 0,
        'headway_violation_count': 0,
        'block_occupancy_violation_count': 0,
        'block_reservation_rejection_count': 0,
        'deadlock_detected_count': 0,
        'deadlock_avoided_count': 0,
    }
    metrics.update(overrides.pop('supervisor_metrics', {}))
    return gate.build_validation_result(
        scenario,
        execution_result=execution_result,
        supervisor_metrics=metrics,
        recorded_event_count=len(scenario['expected_event_targets']),
        **overrides,
    )


def _action():
    return {
        'primitive': 'DONE',
        'side': 'right',
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
        'speed_mps': 0.0,
        'wait_condition': 'terminal',
        'target_id': 'terminal',
        'reason': 'task_succeeded',
        'coordination_mode': 'normal',
    }


def _write_episode(dataset, episode_id, *, approved=True, validation=None):
    episode_dir = dataset / 'episodes' / episode_id
    episode_dir.mkdir(parents=True)
    row = {
        'episode_id': episode_id,
        'event_index': 0,
        'task': 'move the right shuttle from Yaskawa to Staubli',
        'planning_source': 'pddl',
        'pddl_goal': 'right_shuttle at staubli',
        'symbolic_plan': ['finish_task right_shuttle staubli'],
        'model_input': {
            'language': 'move the right shuttle from Yaskawa to Staubli',
            'overhead_images': {},
            'last_command': {'action': 'START'},
        },
        'action': _action(),
    }
    (episode_dir / 'events.jsonl').write_text(json.dumps(row) + '\n', encoding='utf-8')
    if validation is None:
        validation = {
            'scenario_id': episode_id,
            'goal_id': episode_id,
            'source': 'pddl_plansys',
            'validation_status': 'approved' if approved else 'failed',
            'approved_for_training': approved,
            'failure_reason': '' if approved else 'supervisor rejected command',
            'task_success': approved,
            'final_goal_satisfied': approved,
            'rejected_action_rate': 0.0 if approved else 1.0,
            'wrong_shuttle_command_count': 0,
            'headway_violation_count': 0,
            'block_occupancy_violation_count': 0,
            'block_reservation_rejection_count': 0,
            'deadlock_detected_count': 0,
            'deadlock_avoided_count': 0,
        }
    if validation is not False:
        (episode_dir / 'validation.json').write_text(
            json.dumps(validation) + '\n',
            encoding='utf-8',
        )
    return episode_dir


def test_successful_scenario_is_approved():
    validation = _approved_validation()

    assert validation['validation_status'] == 'approved'
    assert validation['approved_for_training'] is True
    assert validation['failure_reason'] == ''


def test_supervisor_rejection_is_not_approved():
    validation = _approved_validation(
        execution_result={
            'success': False,
            'task_success': False,
            'final_goal_satisfied': False,
            'failure_reason': 'unsafe switch change',
            'supervisor_rejected_count': 1,
        },
        supervisor_metrics={
            'rejected_actions': 1,
            'rejected_action_rate': 0.2,
            'rejected_reasons': {'unsafe switch change': 1},
        },
    )

    assert validation['approved_for_training'] is False
    assert validation['supervisor_rejected_count'] == 1
    assert 'supervisor rejected' in ' '.join(validation['failure_reasons'])


def test_ambiguous_payload_rejection_is_not_approved_for_training():
    validation = _approved_validation(
        execution_result={
            'success': False,
            'task_success': False,
            'final_goal_satisfied': False,
            'failure_reason': 'ambiguous payload command: multiple loaded shuttles match',
            'supervisor_rejected_count': 1,
        },
        supervisor_metrics={
            'rejected_actions': 1,
            'rejected_action_rate': 0.2,
        },
    )

    assert validation['approved_for_training'] is False
    assert 'ambiguous payload command' in ' '.join(validation['failure_reasons'])


def test_arrival_timeout_is_not_approved():
    validation = _approved_validation(
        execution_result={
            'success': False,
            'task_success': False,
            'final_goal_satisfied': False,
            'failure_reason': 'timeout waiting for DZI3R',
            'arrival_timeout': True,
        }
    )

    assert validation['approved_for_training'] is False
    assert validation['arrival_timeout'] is True


def test_unsatisfied_final_goal_is_not_approved():
    validation = _approved_validation(
        execution_result={
            'success': False,
            'task_success': False,
            'final_goal_satisfied': False,
            'failure_reason': 'final goal not satisfied',
        }
    )

    assert validation['approved_for_training'] is False
    assert validation['final_goal_satisfied'] is False


def test_invalid_action_vector_is_not_approved():
    scenario = copy.deepcopy(_scenario())
    scenario['action_vectors'][0] = [0.0] * 22

    validation = _approved_validation(scenario)

    assert validation['approved_for_training'] is False
    assert 'action_vector at index 0 length 22' in validation['failure_reason']


def test_multi_shuttle_movement_without_identity_is_not_approved():
    scenario = copy.deepcopy(_scenario())
    scenario['shuttle_counts'] = {'right': 2}
    scenario['primitive_commands'][2]['shuttle'] = 'right_shuttle'
    scenario['primitive_commands'][2].pop('shuttle_id', None)
    scenario['primitive_commands'][2].pop('shuttle_index', None)
    scenario['action_vectors'][2][2] = -1.0

    validation = _approved_validation(scenario, multi_shuttle_active=True)

    assert validation['approved_for_training'] is False
    assert 'multi-shuttle mode' in ' '.join(validation['failure_reasons'])


def test_extractor_skips_failed_and_unvalidated_episodes_by_default(tmp_path):
    extractor = _load_script('room_315_vla_event_extractor')
    dataset = tmp_path / 'dataset'
    _write_episode(dataset, 'episode_approved', approved=True)
    _write_episode(dataset, 'episode_failed', approved=False)
    _write_episode(dataset, 'episode_unvalidated', validation=False)

    output = dataset / 'meta' / 'training_events.jsonl'
    summary = extractor.extract_event_dataset(dataset, output)
    rows = [json.loads(line) for line in output.read_text(encoding='utf-8').splitlines()]

    assert summary['rows'] == 1
    assert summary['approved_episodes'] == 1
    assert summary['skipped_episodes'] == 2
    assert [row['episode_id'] for row in rows] == ['episode_approved']


def test_extractor_includes_failed_only_with_debug_flag(tmp_path):
    extractor = _load_script('room_315_vla_event_extractor')
    dataset = tmp_path / 'dataset'
    _write_episode(dataset, 'episode_approved', approved=True)
    _write_episode(dataset, 'episode_failed', approved=False)

    output = dataset / 'meta' / 'training_events.jsonl'
    summary = extractor.extract_event_dataset(dataset, output, include_failed=True)
    rows = [json.loads(line) for line in output.read_text(encoding='utf-8').splitlines()]

    assert summary['rows'] == 2
    assert summary['included_unapproved_episodes'] == 1
    assert {row['episode_id'] for row in rows} == {'episode_approved', 'episode_failed'}


def test_privileged_model_input_causes_validation_failure():
    scenario = copy.deepcopy(_scenario())
    scenario['model_input'] = {
        'language': 'move',
        'overhead_images': {},
        'last_command': {'action': 'START'},
        'binary_sensor_bits': {'right': {'DZI3R': 1}},
        'payload_state': {'right_shuttle_2': 'loaded'},
    }

    validation = _approved_validation(scenario)

    assert validation['approved_for_training'] is False
    assert 'privileged field inside model_input' in validation['failure_reason']


def test_dataset_report_separates_approved_and_failed_episodes(tmp_path):
    reporter = _load_script('room_315_pddl_dataset_report')
    dataset = tmp_path / 'dataset'
    _write_episode(dataset, 'episode_approved', approved=True)
    _write_episode(dataset, 'episode_failed', approved=False)

    report = reporter.build_dataset_report(dataset)

    assert report['total_episodes'] == 2
    assert report['approved_episodes'] == 1
    assert report['failed_episodes'] == 1
    assert report['approved_event_count'] == 1
    assert report['skipped_event_count'] == 1
    assert report['failure_reasons_distribution'] == {'supervisor rejected command': 1}
