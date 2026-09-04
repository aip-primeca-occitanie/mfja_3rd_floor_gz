#!/usr/bin/env python3

import sys
import signal
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
CONFIG_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_visual_state'
    / 'training_scenarios.yaml'
)
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_scenario_generator as generator
import room_315_visual_scenario_runner as runner


def _scenarios():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    return generator.generate_scenarios(config, count=8, seed=315)


def test_builds_clean_launch_switch_and_capture_commands(tmp_path):
    scenario = _scenarios()[0]
    launch = runner.launch_command(scenario, gui=False)
    switch = runner.switch_command(scenario)
    capture = runner.capture_command(
        scenario,
        manifest=tmp_path / 'manifest.jsonl',
        output_dataset=tmp_path / 'dataset',
        timeout_seconds=30.0,
        max_camera_skew_seconds=0.15,
    )

    assert launch[:5] == [
        'ros2',
        'launch',
        'mfja_3rd_floor_bringup',
        'room_315_only.launch.py',
        'gui:=false',
    ]
    assert 'enable_room315_visual_state_dataset_recorder:=false' in launch
    assert 'robots:=none' in launch
    assert not any("'none'" in argument for argument in launch)
    assert not any(argument.endswith(':=') for argument in launch)
    start_slot_arguments = [
        argument for argument in launch if '_start_slots:=' in argument
    ]
    assert start_slot_arguments
    assert all("'" not in argument for argument in start_slot_arguments)
    assert runner._launch_value('room315_right_start_slots', '3') == '3'
    assert runner._launch_value('room315_left_start_slots', '1,4') == '1,4'
    assert switch[:10] == [
        'ros2',
        'topic',
        'pub',
        '--times',
        '3',
        '--rate',
        '2',
        '--keep-alive',
        '0.5',
        '/mfja/conveyor/switch_cmd',
    ]
    assert '/mfja/conveyor/switch_cmd' in switch
    assert capture[:4] == [
        'ros2',
        'run',
        'mfja_robot_control_config',
        'room_315_visual_state_capture.py',
    ]


def test_selects_requested_range_and_rejects_unknown_ids():
    scenarios = _scenarios()
    selected = runner.select_scenarios(
        scenarios,
        scenario_ids=[],
        start=2,
        limit=3,
    )
    assert selected == scenarios[2:5]

    try:
        runner.select_scenarios(
            scenarios,
            scenario_ids=['missing'],
            start=0,
            limit=None,
        )
    except runner.VisualScenarioRunError as exc:
        assert 'unknown scenario ids' in str(exc)
    else:
        raise AssertionError('unknown scenario id was accepted')


def test_resume_never_accepts_an_incomplete_episode(tmp_path):
    scenario = _scenarios()[0]
    dataset = tmp_path / 'dataset'

    assert runner._episode_exists(dataset, scenario) is False

    incomplete = (
        dataset / 'episodes' / scenario['scenario_id']
    )
    incomplete.mkdir(parents=True)
    (incomplete / 'validation.json').write_text(
        '{}\n',
        encoding='utf-8',
    )

    try:
        runner._episode_exists(dataset, scenario)
    except runner.VisualScenarioRunError as exc:
        assert 'incomplete existing episode' in str(exc)
    else:
        raise AssertionError('incomplete episode was accepted as complete')


def test_cleanup_signals_process_group_after_launch_parent_exits(monkeypatch):
    signals = []

    class ExitedProcess:
        pid = 4242

        @staticmethod
        def poll():
            return 0

    monkeypatch.setattr(
        runner.os,
        'killpg',
        lambda group_id, shutdown_signal: signals.append(
            (group_id, shutdown_signal)
        ),
    )
    monkeypatch.setattr(
        runner,
        '_wait_for_process_group_exit',
        lambda _group_id, _timeout: True,
    )

    runner._terminate_process(ExitedProcess())

    assert signals == [(4242, signal.SIGINT)]
