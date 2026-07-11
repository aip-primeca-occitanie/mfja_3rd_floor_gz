#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_vla_benchmark_suite.py'
)
SCENARIO_GENERATOR_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'scripts'
    / 'room_315_pddl_scenario_generator.py'
)
REGRESSION_CASE_CONFIG = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'room_315_vla'
    / 'payload_training_cases_expanded_160_speed_sweep.yaml'
)


def _load_module():
    spec = importlib.util.spec_from_file_location('room_315_vla_benchmark_suite', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_scenario_generator():
    spec = importlib.util.spec_from_file_location(
        'room_315_pddl_scenario_generator_for_benchmark_suite',
        SCENARIO_GENERATOR_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _regression_case_ids():
    loaded = yaml.safe_load(REGRESSION_CASE_CONFIG.read_text(encoding='utf-8'))
    return {
        str(case['case_id'])
        for case in loaded['cases']
    }


def test_seeded_case_config_retains_160_regression_cases_and_balances_extension():
    suite = _load_module()

    config = suite.generate_seeded_balanced_case_config(
        extension_case_count=100,
        seed=11,
    )
    summary = suite.summarize_case_config(config)
    cases = config['cases']
    extension_cases = [
        case for case in cases
        if case.get('benchmark_subset') == suite.BALANCED_EXTENSION_ID
    ]

    assert summary['case_count'] == 260
    assert summary['regression_subset_retained'] is True
    assert summary['required_family_coverage_complete'] is True
    assert summary['extension_family_balance_max_minus_min'] == 0
    assert _regression_case_ids().issubset({case['case_id'] for case in cases})
    assert len(extension_cases) == 100
    assert {
        case['benchmark_family'] for case in extension_cases
    } == set(suite.REQUIRED_BENCHMARK_FAMILIES)
    assert all(
        not any(key in case for key in ('dataset_dir', 'checkpoint', 'checkpoint_path'))
        for case in extension_cases
    )
    assert all(
        len(set(case['start_slots_by_shuttle'].values())) == 4
        for case in extension_cases
        if isinstance(case.get('start_slots_by_shuttle'), dict)
    )


def test_seeded_cases_cover_required_recovery_and_perception_metadata():
    suite = _load_module()

    config = suite.generate_seeded_balanced_case_config(
        extension_case_count=100,
        seed=5,
    )
    by_family = {
        case['benchmark_family']: case
        for case in config['cases']
        if case.get('benchmark_subset') == suite.BALANCED_EXTENSION_ID
    }

    assert by_family['four_plus_four_fleet']['benchmark_conditions']['fleet_shape'] == '4+4'
    assert by_family['loaded_selection']['selection_policy']
    assert by_family['empty_selection']['payload_condition'] == 'empty'
    assert by_family['blocker_clearance']['blocker_shuttle']
    assert by_family['occupied_target']['benchmark_conditions']['target_initially_occupied'] is True
    assert by_family['unknown_position']['benchmark_conditions']['unknown_position_shuttles']
    assert by_family['sensor_dropout']['benchmark_conditions']['dropout']['streams']
    assert by_family['obstacle']['benchmark_conditions']['obstacles']
    assert by_family['inspection']['goal_type'] == 'inspection'
    assert len(by_family['simultaneous_requests']['simultaneous_requests']) == 2


def test_case_config_cli_writes_yaml_without_generated_artifacts(tmp_path):
    suite = _load_module()
    output = tmp_path / 'seeded_cases.yaml'

    assert suite.main([
        'generate-cases',
        '--extension-case-count',
        '100',
        '--seed',
        '13',
        '--output',
        str(output),
    ]) == 0

    written = yaml.safe_load(output.read_text(encoding='utf-8'))
    assert written['generation']['no_generated_datasets_or_checkpoints'] is True
    assert written['balanced_extension']['case_count'] == 100
    assert not list(tmp_path.glob('*.pt'))
    assert not list(tmp_path.glob('*.ckpt'))
    assert not list(tmp_path.glob('*.safetensors'))


def test_seeded_manifest_is_readable_by_existing_case_loader(tmp_path):
    suite = _load_module()
    generator = _load_scenario_generator()
    config = suite.generate_seeded_balanced_case_config(
        extension_case_count=100,
        seed=17,
    )
    path = tmp_path / 'seeded_cases.yaml'
    suite.write_case_config(config, path)

    extension_case_ids = [
        case['case_id']
        for case in config['cases']
        if case.get('benchmark_subset') == suite.BALANCED_EXTENSION_ID
    ]
    specs = [
        generator.scenario_spec_from_case(case_id, case_config=path)
        for case_id in extension_case_ids
    ]

    assert len(specs) == 100
    assert {spec.payload_condition for spec in specs} == {'loaded', 'empty'}
    assert all(spec.side in {'right', 'left'} for spec in specs)
    assert all(spec.target_slot in {'1', '2', '3', '4'} for spec in specs)


def test_method_comparison_keeps_gazebo_and_real_image_claims_separate(tmp_path):
    suite = _load_module()
    raw_results = [
        {
            'method': 'Oracle+PlanSys2',
            'result_scope': 'gazebo_planning',
            'metrics': {
                'success_rate': 1.0,
                'false_success_rate': 0.0,
                'safety_violation_rate': 0.0,
                'supervisor_rejection_rate': 0.0,
                'mean_replans': 0.2,
                'mean_route_length': 5.0,
                'mean_completion_time_s': 12.0,
                'latency_p50_s': 0.01,
                'latency_p95_s': 0.02,
            },
        },
        {
            'method': 'Frozen Visual+PlanSys2',
            'result_scope': 'gazebo_planning',
            'metrics': {
                'task_success': 0.8,
                'false_success': 0.01,
                'unsafe_rate': 0.0,
                'rejected_action_rate': 0.03,
                'replans': 1.2,
                'route_length': 6.0,
                'completion_time': 18.0,
                'p50_latency_s': 0.04,
                'p95_latency_s': 0.08,
            },
        },
        {
            'method': 'LoRA Visual+PlanSys2',
            'perception_source': 'real_image',
            'metrics': {
                'success': 0.84,
                'false_success_rate': 0.0,
                'safety_failure_rate': 0.0,
                'rejection_rate': 0.02,
                'avg_replans': 0.9,
                'command_count': 5.8,
                'total_cycle_time_s': 16.0,
                'p50_inference_latency_s': 0.03,
                'p95_inference_latency_s': 0.07,
            },
        },
        {
            'method': 'legacy direct-action SmolVLA',
            'result_scope': 'legacy_direct_action_offline',
            'metrics': {
                'success_rate': 0.4,
                'false_success_rate': 0.05,
                'safety_violation_rate': 0.01,
                'supervisor_rejection_rate': 0.12,
                'mean_replans': 0.0,
                'mean_route_length': 5.0,
                'mean_completion_time_s': 11.0,
                'latency_p50_s': 0.23,
                'latency_p95_s': 0.31,
            },
        },
    ]

    report = suite.compare_method_results(raw_results)
    by_method = {row['method']: row for row in report['methods']}

    assert report['missing_methods'] == []
    assert by_method['oracle_plansys2']['gazebo_planning_result'] is True
    assert by_method['frozen_visual_plansys2']['real_image_perception_claim'] is False
    assert by_method['lora_visual_plansys2']['real_image_perception_claim'] is True
    assert by_method['legacy_direct_action_smolvla']['result_scope'] == 'legacy_direct_action_offline'
    assert by_method['frozen_visual_plansys2']['metrics']['success_rate'] == 0.8

    paths = suite.write_comparison_report(report, tmp_path / 'comparison.json')
    parsed = json.loads(Path(paths['json']).read_text(encoding='utf-8'))
    csv_text = Path(paths['csv']).read_text(encoding='utf-8')
    assert parsed['claim_boundary']['real_image_perception_methods'] == [
        'lora_visual_plansys2'
    ]
    assert 'legacy_direct_action_smolvla' in csv_text


def test_method_comparison_does_not_fabricate_missing_results():
    suite = _load_module()

    report = suite.compare_method_results(
        [{
            'method': 'Oracle+PlanSys2',
            'metrics': {'success_rate': 1.0},
        }],
        allow_missing_methods=True,
    )
    by_method = {row['method']: row for row in report['methods']}

    assert by_method['oracle_plansys2']['status'] == 'partial'
    assert 'latency_p95_s' in by_method['oracle_plansys2']['missing_metrics']
    assert by_method['frozen_visual_plansys2']['status'] == 'missing_result'
    assert by_method['frozen_visual_plansys2']['metrics']['success_rate'] is None
