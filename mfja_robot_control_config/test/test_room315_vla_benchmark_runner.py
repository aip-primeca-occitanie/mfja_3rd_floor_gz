#!/usr/bin/env python3

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_benchmark_task_sequence_groups_are_ordered_for_round_trip_tasks():
    runner = _load_module(
        'room_315_vla_benchmark_runner',
        SCRIPTS_DIR / 'room_315_vla_benchmark_runner.py',
    )

    assert runner.parse_task_sequence('transport') == [
        'right_yaskawa_to_staubli',
        'right_staubli_to_yaskawa',
        'left_yaskawa_to_kuka',
        'left_kuka_to_yaskawa',
    ]
    assert runner.parse_task_sequence('loop_entry') == [
        'right_enter_interior_loop',
        'left_enter_interior_loop',
    ]
    assert runner.parse_task_sequence('all') == [
        'right_yaskawa_to_staubli',
        'right_staubli_to_yaskawa',
        'left_yaskawa_to_kuka',
        'left_kuka_to_yaskawa',
        'right_enter_interior_loop',
        'left_enter_interior_loop',
    ]


def test_benchmark_task_state_parses_supervisor_status():
    runner = _load_module(
        'room_315_vla_benchmark_runner',
        SCRIPTS_DIR / 'room_315_vla_benchmark_runner.py',
    )

    running_status = {
        'last_result': 'task right_yaskawa_to_staubli started',
        'active_tasks': {
            'id': {
                'template': 'right_yaskawa_to_staubli',
            }
        },
    }
    assert runner.task_state_from_status(running_status, 'right_yaskawa_to_staubli')[0] == 'running'

    success_status = {
        'last_result': (
            'task right_yaskawa_to_staubli completed: right shuttle reached target slot 3'
        ),
        'active_tasks': {},
    }
    assert runner.task_state_from_status(success_status, 'right_yaskawa_to_staubli')[0] == 'succeeded'

    failure_status = {
        'last_result': (
            'command rejected: task left_kuka_to_yaskawa rejected: no left-rail shuttle detected'
        ),
        'active_tasks': {},
    }
    assert runner.task_state_from_status(failure_status, 'left_kuka_to_yaskawa')[0] == 'failed'


def test_benchmark_task_names_match_vla_task_templates():
    runner = _load_module(
        'room_315_vla_benchmark_runner',
        SCRIPTS_DIR / 'room_315_vla_benchmark_runner.py',
    )
    expected = {
        'right_yaskawa_to_staubli',
        'right_staubli_to_yaskawa',
        'left_yaskawa_to_kuka',
        'left_kuka_to_yaskawa',
        'right_enter_interior_loop',
        'left_enter_interior_loop',
    }

    assert set(runner.TASK_GOALS) == expected
    assert set(runner.parse_task_sequence('all')) == expected
    assert not any(task_name.endswith('_demo') for task_name in runner.TASK_GOALS)
