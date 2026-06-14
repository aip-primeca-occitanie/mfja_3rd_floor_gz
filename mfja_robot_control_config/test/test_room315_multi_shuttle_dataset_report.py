#!/usr/bin/env python3

import importlib.util
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_pddl_dataset_report.py'


def _load_reporter():
    spec = importlib.util.spec_from_file_location('room_315_pddl_dataset_report', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _event(target='R2', chosen='R2', success=True, marker_count=2, payload=True):
    return {
        'episode_id': f'episode_{target}',
        'planning_source': 'pddl',
        'pddl_goal': f'task_done {target} staubli',
        'target_shuttle_id': target,
        'payload_present': payload,
        'visible_marker_count': marker_count,
        'identity_occlusion_level': 'partial' if marker_count < 4 else 'visible',
        'model_input': {
            'language': f'move {target} to the Staubli station',
            'overhead_images': {'right_rail_rgb': 'right.jpg', 'left_rail_rgb': 'left.jpg'},
            'last_command': {'action': 'START'},
        },
        'action': {
            'primitive': 'SHUTTLE_ON',
            'side': 'right',
            'shuttle_id': chosen,
            'target_id': f'right_shuttle_{target[-1]}',
            'speed_mps': 0.3,
        },
        'privileged_eval': {
            'correct_target_shuttle': target,
            'visible_marker_count_for_target': marker_count,
            'payload_present': payload,
            'identity_occlusion_level': 'partial' if marker_count < 4 else 'visible',
        },
        'safety_decoder_metrics': {
            'total_proposed_actions': 10,
            'wrong_shuttle_command_count': 1,
            'headway_violation_count': 1,
            'block_reservation_rejection_count': 1,
            'deadlock_detected_count': 1,
            'deadlock_avoided_count': 3,
            'fleet_throughput_tasks_per_minute': 2.5,
            'average_wait_time_by_shuttle': {'R1': 1.0, 'R2': 3.0},
        },
        'event_generation_metrics': {'task_success': success},
    }


def _write_events(dataset_dir, rows):
    event_dir = dataset_dir / 'episodes' / 'episode_multi'
    event_dir.mkdir(parents=True)
    event_file = event_dir / 'events.jsonl'
    event_file.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
    return event_file


def test_multi_shuttle_report_counts_identity_safety_and_occlusion_metrics(tmp_path):
    reporter = _load_reporter()
    dataset = tmp_path / 'dataset'
    _write_events(dataset, [
        _event('R2', 'R2', success=True, marker_count=2, payload=True),
        _event('R3', 'R2', success=False, marker_count=4, payload=False),
    ])

    report = reporter.build_dataset_report(dataset)

    assert report['dataset_source'] == 'pddl'
    assert report['target_shuttle_distribution'] == {'R2': 1, 'R3': 1}
    assert report['visible_marker_count_distribution'] == {'2': 1, '4': 1}
    assert report['identity_occlusion_level_distribution'] == {'partial': 1, 'visible': 1}
    assert report['shuttle_id_accuracy'] == 0.5
    assert report['identity_grounding_accuracy'] == 0.5
    assert report['target_shuttle_selection_accuracy'] == 0.5
    assert report['wrong_shuttle_command_rate'] == 0.1
    assert report['headway_violation_rate'] == 0.1
    assert report['deadlock_rate'] == 0.1
    assert report['deadlock_avoidance_success_rate'] == 0.75
    assert report['loaded_shuttle_success_rate'] == 1.0
    assert report['unloaded_shuttle_success_rate'] == 0.0
    assert report['partial_occlusion_success_rate'] == 1.0
    assert report['average_wait_time'] == 2.0
    assert report['fleet_throughput_tasks_per_minute'] == 2.5
    assert report['per_side_success_rate'] == {'right': 0.5}
    assert report['per_shuttle_success_rate'] == {'R2': 1.0, 'R3': 0.0}
