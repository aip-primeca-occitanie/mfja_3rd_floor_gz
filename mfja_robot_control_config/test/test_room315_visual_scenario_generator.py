#!/usr/bin/env python3

import sys
from collections import Counter
from pathlib import Path

import pytest
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
BLOCKER_CONFIG_PATH = CONFIG_PATH.with_name('blocker_training_scenarios.yaml')
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_scenario_generator as generator


def _config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))


def _blocker_config():
    return yaml.safe_load(BLOCKER_CONFIG_PATH.read_text(encoding='utf-8'))


def test_allocates_exact_balanced_pilot_counts():
    counts = generator.allocate_scene_counts(
        50,
        {
            'empty': 0.08,
            'single': 0.32,
            'multi_same_rail': 0.36,
            'dual_rail': 0.24,
        },
    )

    assert counts == {
        'empty': 4,
        'single': 16,
        'multi_same_rail': 18,
        'dual_rail': 12,
    }


def test_generates_deterministic_unique_visual_only_scenarios():
    first = generator.generate_scenarios(_config(), count=50, seed=315)
    second = generator.generate_scenarios(_config(), count=50, seed=315)

    assert first == second
    assert len(first) == 50
    assert len({row['scenario_id'] for row in first}) == 50
    assert len({row['scenario_family'] for row in first}) == 50
    assert Counter(row['scene_type'] for row in first) == {
        'multi_same_rail': 18,
        'single': 16,
        'dual_rail': 12,
        'empty': 4,
    }
    assert all(row['scene']['obstacles'] == [] for row in first)
    assert all(
        row['capture']['cameras'] == ['left_rail_rgb', 'right_rail_rgb']
        for row in first
    )
    assert all(
        not (generator._walk_keys(row) & generator.LEGACY_KEYS)
        for row in first
    )


def test_default_full_plan_has_320_unique_families():
    first = generator.generate_scenarios(_config())
    second = generator.generate_scenarios(_config())

    assert first == second
    assert len(first) == 320
    assert len({row['scenario_family'] for row in first}) == 320


def test_launch_setup_matches_scene_payloads_and_slots():
    scenarios = generator.generate_scenarios(_config(), count=50, seed=315)

    for scenario in scenarios:
        arguments = scenario['setup']['launch_arguments']
        for side in generator.SIDES:
            shuttles = scenario['scene']['rails'][side]['shuttles']
            assert arguments[f'room315_{side}_shuttle_count'] == len(shuttles)
            assert arguments[f'room315_{side}_start_slots'] == ','.join(
                str(shuttle['start_slot']) for shuttle in shuttles
            )
            assert arguments[f'room315_{side}_loaded_shuttles'] == ','.join(
                shuttle['id']
                for shuttle in shuttles
                if shuttle['loaded_state'] == 'loaded'
            )


def test_generates_continuous_blocker_and_hard_negative_scenarios():
    scenarios = generator.generate_scenarios(
        _blocker_config(),
        count=50,
        seed=315,
    )

    assert len(scenarios) == 50
    assert len({row['scenario_family'] for row in scenarios}) == 50
    assert Counter(row['scene_type'] for row in scenarios) == {
        'blocker_ahead_same_segment': 13,
        'blocker_intermediate_segment': 10,
        'nonblocker_adjacent_branch': 10,
        'nonblocker_behind_same_segment': 10,
        'multi_blocker': 7,
    }
    for scenario in scenarios:
        probe = scenario['planning_probe']
        shuttles = scenario['scene']['rails'][probe['side']]['shuttles']
        arguments = scenario['setup']['launch_arguments']
        assert all('start_position' in shuttle for shuttle in shuttles)
        assert arguments[f'room315_{probe["side"]}_start_positions']
        assert probe['model_input_exposure'] == 'excluded'
        assert probe['expected_route_clear'] == (
            not probe['expected_blocker_ids']
        )
        assert not (generator._walk_keys(scenario) & generator.LEGACY_KEYS)


def test_rejects_legacy_task_field():
    scenario = generator.generate_scenarios(_config(), count=1, seed=315)[0]
    scenario['task'] = 'legacy'

    with pytest.raises(generator.VisualScenarioError, match='legacy fields'):
        generator.validate_scenario(scenario)


def test_rejects_overlapping_continuous_start_positions():
    scenario = generator.generate_scenarios(
        _blocker_config(),
        count=1,
        seed=315,
    )[0]
    side = scenario['planning_probe']['side']
    shuttles = scenario['scene']['rails'][side]['shuttles']
    shuttles[1]['start_position'] = {
        'segment': shuttles[0]['start_position']['segment'],
        's_ratio': shuttles[0]['start_position']['s_ratio'] + 0.05,
    }

    with pytest.raises(generator.VisualScenarioError, match='overlapping shuttle geometry'):
        generator.validate_scenario(scenario)


def test_writes_and_reloads_manifest(tmp_path):
    scenarios = generator.generate_scenarios(_config(), count=8, seed=315)
    summary = generator.scenario_summary(scenarios, config_path=CONFIG_PATH)

    manifest_path, summary_path = generator.write_scenario_plan(
        tmp_path,
        scenarios,
        summary,
    )

    assert generator._read_manifest(manifest_path) == scenarios
    assert summary_path.is_file()
    assert summary['scenarios'] == 8
    assert summary['quality_gate']['legacy_fields_present'] == []
