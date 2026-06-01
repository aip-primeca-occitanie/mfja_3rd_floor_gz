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
    }
    (episode_dir / 'events.jsonl').write_text(json.dumps(event_row) + '\n', encoding='utf-8')
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
        'observation.images.left_rail_rgb': (
            'episodes/episode_000001_test/images/left_rail_rgb/000003.jpg'
        ),
        'observation.images.right_rail_rgb': (
            'episodes/episode_000001_test/images/right_rail_rgb/000003.jpg'
        ),
        'observation.state': [0.0, 1.0, 0.0],
        'action': event_row['action'],
        'privileged_eval': event_row['privileged_eval'],
    }
    assert 'command' not in rows[0]
    assert 'raw_replay_only' not in rows[0]


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

    output = dataset_dir / 'flat.jsonl'
    extractor.extract_event_dataset(dataset_dir, output)
    rows = _rows(output)

    assert rows[0]['episode_id'] == 'episode_000002_test'
    assert rows[0]['step_index'] == 0
    assert rows[0]['action'] == action
    assert rows[0]['observation.images.left_rail_rgb'] == 'left.jpg'
