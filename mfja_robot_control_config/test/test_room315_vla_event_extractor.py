#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'room_315_vla_event_extractor.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_vla_event_extractor', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _rows(path: Path):
    return [
        json.loads(line)
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]


def _write_validation(episode_dir: Path, *, approved: bool = True, reason: str = ''):
    validation = {
        'scenario_id': episode_dir.name,
        'goal_id': episode_dir.name,
        'source': 'pddl_plansys',
        'validation_status': 'approved' if approved else 'failed',
        'approved_for_training': approved,
        'failure_reason': '' if approved else (reason or 'test failure'),
    }
    (episode_dir / 'validation.json').write_text(
        json.dumps(validation) + '\n',
        encoding='utf-8',
    )


def _action(primitive: str, *, side: str = 'right') -> dict:
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
        'speed_mps': 0.0,
        'wait_condition': 'none',
        'target_id': 'none',
        'reason': 'command_event',
    }


def test_event_extractor_writes_minimal_training_rows_and_ignores_raw_frames(tmp_path):
    extractor = _load_module()
    dataset_dir = tmp_path / 'dataset'
    episode_dir = dataset_dir / 'episodes' / 'episode_000001_test'
    episode_dir.mkdir(parents=True)

    event_row = {
        'episode_id': 'episode_000001_test',
        'event_index': 3,
        'task': 'move right shuttle',
        'image_frame_refs': {
            'right_rail_rgb': 'episodes/episode_000001_test/images/right_rail_rgb/000003.jpg',
            'left_rail_rgb': 'episodes/episode_000001_test/images/left_rail_rgb/000003.jpg',
        },
        'observation.state': [0.0, 1.0, 0.0],
        'action': {
            'primitive': 'SET_SWITCHES',
            'side': 'right',
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
            'target_id': 'A3',
            'reason': 'switch_update',
        },
        'privileged_eval': {
            'raw_shuttle_states': {'right': {'room315_right_shuttle_1': {'segment': 'A12E'}}}
        },
        'auxiliary_targets': {
            'switch_states': {'right': {'A3': 'EXTERIOR'}},
            'model_input_exposure': 'excluded',
        },
    }
    (episode_dir / 'events.jsonl').write_text(json.dumps(event_row) + '\n', encoding='utf-8')
    _write_validation(episode_dir)
    (episode_dir / 'data.jsonl').write_text(
        json.dumps({
            'episode_id': 'episode_000001_test',
            'frame_index': 0,
            'raw_replay_only': True,
            'command': {'action': 'shuttle', 'command': 'ON'},
        }) + '\n',
        encoding='utf-8',
    )

    output = dataset_dir / 'meta' / 'training_events.jsonl'
    summary = extractor.extract_event_dataset(dataset_dir, output)
    rows = _rows(output)

    assert summary['rows'] == 1
    assert summary['source'] == 'episodes/*/events.jsonl'
    assert summary['ignored_source'] == 'episodes/*/data.jsonl'
    assert len(rows) == 1
    assert rows[0] == {
        'episode_id': 'episode_000001_test',
        'step_index': 3,
        'task': 'move right shuttle',
        'model_input_schema_version': 3,
        'model_input': {
            'language': 'move right shuttle',
            'overhead_images': {
                'left_rail_rgb': (
                    'episodes/episode_000001_test/images/left_rail_rgb/000003.jpg'
                ),
                'right_rail_rgb': (
                    'episodes/episode_000001_test/images/right_rail_rgb/000003.jpg'
                ),
            },
            'last_command': {'action': 'START'},
        },
        'action': event_row['action'],
        'auxiliary_targets': event_row['auxiliary_targets'],
    }
    assert 'command' not in rows[0]
    assert 'raw_replay_only' not in rows[0]
    assert 'observation.state' not in rows[0]
    assert 'privileged_eval' not in rows[0]


def test_event_extractor_falls_back_to_next_action_and_step_index(tmp_path):
    extractor = _load_module()
    dataset_dir = tmp_path / 'dataset'
    episode_dir = dataset_dir / 'episodes' / 'episode_000002_test'
    episode_dir.mkdir(parents=True)
    action = {
        'primitive': 'DONE',
        'side': 'left',
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
        'wait_condition': 'terminal',
        'target_id': 'terminal',
        'reason': 'task_succeeded',
    }
    (episode_dir / 'events.jsonl').write_text(
        json.dumps({
            'task': 'terminal event',
            'next_action': action,
            'observation.images.left_rail_rgb': 'left.jpg',
            'observation.state': [1.0],
            'privileged_eval': {},
        }) + '\n',
        encoding='utf-8',
    )
    _write_validation(episode_dir)

    output = dataset_dir / 'flat.jsonl'
    extractor.extract_event_dataset(dataset_dir, output)
    rows = _rows(output)

    assert rows[0]['episode_id'] == 'episode_000002_test'
    assert rows[0]['step_index'] == 0
    assert rows[0]['action'] == action
    assert rows[0]['model_input'] == {
        'language': 'terminal event',
        'overhead_images': {'left_rail_rgb': 'left.jpg'},
        'last_command': {'action': 'START'},
    }


def test_event_extractor_rebuilds_previous_command_and_ignores_leaking_input(tmp_path):
    extractor = _load_module()
    dataset_dir = tmp_path / 'dataset'
    episode_dir = dataset_dir / 'episodes' / 'episode_000003_test'
    episode_dir.mkdir(parents=True)

    actions = [
        _action('STOP_NOW'),
        _action('SET_SWITCHES'),
        _action('SET_STOPPERS'),
        _action('SHUTTLE_ON'),
        _action('STOP_NOW'),
        _action('DONE'),
    ]
    for index, action in enumerate(actions):
        action['target_id'] = 'terminal' if action['primitive'] == 'DONE' else 'right_shuttle'
        action['reason'] = 'task_succeeded' if action['primitive'] == 'DONE' else 'command_event'
        if action['primitive'] == 'SET_SWITCHES':
            action['switch_mask']['A3'] = 1
            action['switch_values']['A3'] = 'INTERIOR'
        if action['primitive'] == 'SET_STOPPERS':
            action['stopper_mask']['A3'] = 1
            action['stopper_values']['A3'] = 'closed'
        if action['primitive'] == 'SHUTTLE_ON':
            action['speed_mps'] = 0.2

    leaking_rows = []
    for index, action in enumerate(actions):
        leaking_rows.append({
            'episode_id': 'episode_000003_test',
            'event_index': index,
            'task': 'regression command history',
            'model_input_schema_version': 3,
            'model_input': {
                'language': 'regression command history',
                'overhead_images': {'right_rail_rgb': f'right_{index}.jpg'},
                'last_command': action,
                'binary_sensor_bits': {'right': {'DZI2R': 1}},
            },
            'observation.state': [1.0, 2.0, 3.0],
            'privileged_eval': {'raw_shuttle_states': {'right': {'s': 0.4}}},
            'auxiliary_targets': {'switch_states': {'right': {'A3': 'EXTERIOR'}}},
            'symbolic_next_action': {'action': action['primitive']},
            'action': action,
            'action_vector': [float(index), float(index + 1)],
        })
    (episode_dir / 'events.jsonl').write_text(
        ''.join(json.dumps(row) + '\n' for row in leaking_rows),
        encoding='utf-8',
    )
    _write_validation(episode_dir, approved=False, reason='privileged model_input leak')
    (episode_dir / 'data.jsonl').write_text(
        json.dumps({'model_input': {'last_command': {'action': 'SHOULD_NOT_APPEAR'}}}) + '\n',
        encoding='utf-8',
    )

    output = dataset_dir / 'meta' / 'training_events.jsonl'
    summary = extractor.extract_event_dataset(dataset_dir, output, include_failed=True)
    rows = _rows(output)

    assert summary['source'] == 'episodes/*/events.jsonl'
    assert summary['ignored_source'] == 'episodes/*/data.jsonl'
    assert [row['model_input']['last_command'] for row in rows] == [
        {'action': 'START'},
        actions[0],
        actions[1],
        actions[2],
        actions[3],
        actions[4],
    ]
    assert [row['action'] for row in rows] == actions
    assert [row['action_vector'] for row in rows] == [
        [float(index), float(index + 1)]
        for index in range(len(actions))
    ]
    for row in rows:
        assert row['model_input']['last_command'] != row['action']
        assert set(row['model_input']) == {'language', 'overhead_images', 'last_command'}
        assert 'binary_sensor_bits' not in row['model_input']
        assert row['auxiliary_targets']['switch_states']['right']['A3'] == 'EXTERIOR'
        assert 'observation.state' not in row
        assert 'privileged_eval' not in row
