from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
import yaml


RUNNER_PATH = Path(__file__).with_name('run_room315_integrated_campaign_v4.py')
MATRIX_PATH = RUNNER_PATH.with_name('room315_integrated_campaign_v4.yaml')
SPEC = importlib.util.spec_from_file_location(
    'room315_integrated_campaign_runner_under_test', RUNNER_PATH,
)
assert SPEC is not None and SPEC.loader is not None
runner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)


def _case(*, side: str = 'right') -> dict:
    identity = 'R1' if side == 'right' else 'L1'
    other_side = 'left' if side == 'right' else 'right'
    return {
        'id': 'T01',
        'side': side,
        'launch': {
            side: {
                'identities': [identity],
                'start_slots': ['1'],
                'loaded': [identity],
            },
            other_side: {'identities': [], 'start_slots': [], 'loaded': []},
        },
        'expected': {
            'selected_identity': identity,
            'target_slot': '2',
        },
    }


def _observation(
    case: dict,
    location: dict,
    *,
    stamp: tuple[int, int],
    accepted_frame_count: int,
) -> dict:
    selected = case['expected']['selected_identity']
    shuttles = []
    for prefix in ('L', 'R'):
        for index in range(1, 5):
            identity = f'{prefix}{index}'
            present = identity == selected
            shuttles.append({
                'identity': identity,
                'presence_state': 'present' if present else 'absent',
                'visual_facts_valid': present,
                'side': case['side'] if present else '',
                'block': location['segment'] if present else '',
                'bbox_xywh': [4.0, 5.0, 20.0, 10.0] if present else [0.0] * 4,
                's_m': 1.0 if present else 0.0,
                's_ratio': location['s_ratio'] if present else 0.0,
                'segment_length_m': 2.0 if present else 0.0,
                'loaded_state': 'loaded' if present else 'unknown',
            })
    return {
        'header': {
            'stamp': {'sec': stamp[0], 'nanosec': stamp[1]},
            'frame_id': 'room_315',
        },
        'schema_version': runner.VISUAL_SCHEMA,
        'checkpoint_sha256': 'a' * 64,
        'stage': 'fused_observed_state',
        'accepted': True,
        'stale': False,
        'model_ready': True,
        'input_ready': True,
        'presence_ready': True,
        'state_fusion_ready': True,
        'validation_reasons': [],
        'clamped_fields': [],
        'accepted_frame_count': accepted_frame_count,
        'shuttles': shuttles,
    }


def test_initial_and_final_v4_observations_match_authoritative_slot_oracle() -> None:
    case = _case()
    initial_location = runner.authoritative_slot_oracle('right', '1')
    final_location = runner.authoritative_slot_oracle('right', '2')
    initial = _observation(case, initial_location, stamp=(10, 0), accepted_frame_count=7)
    final = _observation(case, final_location, stamp=(12, 5), accepted_frame_count=9)

    initial_check = runner.validate_initial_visual(initial, case, 'a' * 64)
    final_check = runner.validate_final_visual(
        final,
        case,
        'a' * 64,
        initial_payload=initial,
        target_slot='2',
        response={},
    )

    assert initial_check['visual_identity_oracle']['status'] == 'passed'
    assert final_check['visual_identity_oracle']['status'] == 'passed'
    assert final_check['frame_progression']['accepted_frame_count_delta'] == 2


def test_left_slot_oracle_uses_published_segment_name() -> None:
    oracle = runner.authoritative_slot_oracle('left', '1')
    assert oracle['segment'] == 'A12E'
    assert oracle['s_ratio'] == pytest.approx(0.428330934)


def test_visual_oracle_rejects_s_ratio_error_above_012() -> None:
    case = _case()
    location = runner.authoritative_slot_oracle('right', '1')
    payload = _observation(case, location, stamp=(1, 0), accepted_frame_count=1)
    payload['shuttles'][4]['s_ratio'] = location['s_ratio'] + 0.120001

    with pytest.raises(runner.CampaignError, match='s_ratio'):
        runner.validate_initial_visual(payload, case, 'a' * 64)


def test_final_v4_observation_must_be_strictly_newer() -> None:
    case = _case()
    initial_location = runner.authoritative_slot_oracle('right', '1')
    final_location = runner.authoritative_slot_oracle('right', '2')
    initial = _observation(case, initial_location, stamp=(5, 0), accepted_frame_count=3)
    final = _observation(case, final_location, stamp=(5, 0), accepted_frame_count=4)

    with pytest.raises(runner.CampaignError, match='is not newer'):
        runner.validate_final_visual(
            final,
            case,
            'a' * 64,
            initial_payload=initial,
            target_slot='2',
            response={},
        )


def test_exact_source_install_pair_rejects_stale_copy(tmp_path: Path) -> None:
    source = tmp_path / 'source.py'
    installed = tmp_path / 'installed.py'
    source.write_text('version = 4\n', encoding='utf-8')
    installed.write_text('version = 4\n', encoding='utf-8')
    rows = runner.verify_exact_source_install_pairs([
        ('runtime', source, installed),
    ])
    assert rows[0]['status'] == 'matched'

    installed.write_text('version = 3\n', encoding='utf-8')
    with pytest.raises(runner.CampaignError, match='stale installed runtime file'):
        runner.verify_exact_source_install_pairs([
            ('runtime', source, installed),
        ])


def test_current_v4_source_install_parity_gate_passes() -> None:
    result = runner.verify_source_install_parity()
    assert result['status'] == 'passed'
    assert result['pair_count'] == 11
    assert result['generated_visual_shuttle_schema']['status'] == 'matched'
    task_runtime = next(
        row for row in result['pairs']
        if row['role'] == 'task_execution_runtime_yaml'
    )
    assert Path(task_runtime['source_path']).name == 'task_execution_runtime.yaml'
    assert Path(task_runtime['installed_path']).name == 'task_execution_runtime.yaml'
    assert task_runtime['source_sha256'] == runner.TASK_RUNTIME_CONFIG_SHA256


def test_current_runner_and_matrix_bind_final_v4_runtime() -> None:
    payload = yaml.safe_load(MATRIX_PATH.read_text(encoding='utf-8'))
    declarations = {
        row['role']: row
        for row in payload['runtime_artifact_contract']['artifacts']
    }

    assert runner.VISUAL_RUNTIME_BUNDLE.name.endswith(
        'closed_loop_runtime_attempt1'
    )
    assert runner.VISUAL_MANIFEST_SHA256 == (
        '506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da'
    )
    assert runner.SOURCE_QUALIFICATION_MANIFEST_SHA256 == (
        '6f9828219c22599825f5a14e405c8f11ce017984cc0d65821a357240d6529e2a'
    )
    assert runner.TASK_EXECUTION_AUTHORIZATION_SHA256 == (
        '14cedafe28c999786a66934a523db5757e1ccdd7ae34705d5a2df58488fc8df1'
    )
    assert runner.VISUAL_MANUAL_DECISION_SHA256 == (
        'df16e885051000117ca914715ace76a58b7f39ffbb6a7ccd6787f7885d18ffdc'
    )
    assert runner.VISUAL_RUNTIME_CONFIG_SHA256 == (
        '22f12a9f96b3d54e0ab3d0bc05c202024ac6912cb50dd6e29ceb4a0a564d24f8'
    )
    assert runner.TASK_RUNTIME_CONFIG_SHA256 == (
        '08eaedd7d6feed3dd1268ef18bfa2545348f203f1ff0dad3c2e9fb1a9f25b6ca'
    )

    expected = {
        'visual_promotion_manifest': (
            runner.VISUAL_MANIFEST, runner.VISUAL_MANIFEST_SHA256, 9338,
        ),
        'visual_manual_decision': (
            runner.VISUAL_MANUAL_DECISION,
            runner.VISUAL_MANUAL_DECISION_SHA256,
            1907,
        ),
        'visual_runtime_configuration': (
            runner.VISUAL_RUNTIME_CONFIG,
            runner.VISUAL_RUNTIME_CONFIG_SHA256,
            821,
        ),
        'task_execution_runtime_configuration': (
            runner.TASK_RUNTIME_CONFIG,
            runner.TASK_RUNTIME_CONFIG_SHA256,
            3848,
        ),
        'task_execution_authorization': (
            runner.TASK_EXECUTION_AUTHORIZATION,
            runner.TASK_EXECUTION_AUTHORIZATION_SHA256,
            785,
        ),
    }
    for role, (path, sha256, size_bytes) in expected.items():
        declaration = declarations[role]
        assert Path(declaration['path']) == path
        assert declaration['sha256'] == sha256
        assert declaration['size_bytes'] == size_bytes

    visual = runner.visual_command()
    execution = runner.execution_command()
    assert f'runtime_config:={runner.VISUAL_RUNTIME_CONFIG}' in visual
    assert f'v4_promotion_manifest:={runner.VISUAL_MANIFEST}' in visual
    assert (
        f'v4_promotion_manifest_sha256:={runner.VISUAL_MANIFEST_SHA256}'
        in visual
    )
    assert f'runtime_config:={runner.TASK_RUNTIME_CONFIG}' in execution
    assert 'execution_enabled:=true' in execution


def test_single_b01_selection_is_explicit_partial_smoke_evidence() -> None:
    payload = yaml.safe_load(MATRIX_PATH.read_text(encoding='utf-8'))
    selected = runner.select_cases(payload, ['B01'])
    result = {
        'status': 'passed',
        'selected_identity': 'R1',
        'executed_step_count': 1,
        'actuating_step_count': 1,
        'non_actuating_step_count': 0,
        'satisfied_postcondition_count': 1,
        'accepted_supervisor_decision_count': 1,
        'safe_abort_sent': False,
        'controller_mode_at_arrival': 'DISABLED',
        'plan_attempts': 1,
        'replans': 0,
        'unknown_retries': 0,
        'rosbag': {
            'visual_schema_audit': {
                'v3_observation_count': 0,
                'v4_observation_count': 1,
            },
        },
    }
    summary = runner.build_summary(
        payload['campaign_id'],
        len(payload['cases']),
        selected,
        [result],
        '2026-08-12T00:00:00Z',
        'partial',
    )

    assert summary['status'] == 'partial'
    assert summary['evidence_scope'] == 'partial_smoke'
    assert summary['partial_smoke_evidence'] is True
    assert summary['full_declared_campaign'] is False
    assert summary['planned_case_ids'] == ['B01']
    assert summary['qualification_manifest_sha256'] == (
        runner.SOURCE_QUALIFICATION_MANIFEST_SHA256
    )
    assert summary['source_qualification_manifest_sha256'] == (
        runner.SOURCE_QUALIFICATION_MANIFEST_SHA256
    )
    assert summary['runtime_manifest_sha256'] == runner.VISUAL_MANIFEST_SHA256
