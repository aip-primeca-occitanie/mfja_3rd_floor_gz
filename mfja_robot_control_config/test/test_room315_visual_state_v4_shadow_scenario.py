"""Static and helper tests for the one-scenario V4 shadow launch."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = (
    ROOT / 'launch' / 'room_315_visual_state_v4_shadow_scenario.launch.py'
)
DUAL_SHADOW_PATH = ROOT / 'launch' / 'room_315_visual_state_v4_shadow.launch.py'
ARTIFACT_NAMES = {
    'checkpoint': 'checkpoint_epoch_011.pt',
    'training_final_report': 'training_final_report.json',
    'canary_final_report': 'canary_final_report.json',
    'canary_completion_ledger': 'canary_completion_ledger.json',
    'effective_config': 'effective_config.json',
    'validation_acceptance': 'validation_acceptance.json',
    'validation_segment_calibration': 'validation_segment_calibration.json',
    'public_topology_contract': 'public_topology_contract.json',
}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'room315_visual_state_v4_shadow_scenario_launch',
        LAUNCH_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_ros_python_launch_loader_can_import_the_file():
    """Exercise the same loader used by ``ros2 launch``.

    Its module is deliberately not inserted in ``sys.modules``; dataclasses
    declared directly in a launch file therefore fail during import on Jazzy.
    """

    from launch.launch_description_sources.python_launch_file_utilities import (
        get_launch_description_from_python_launch_file,
    )

    description = get_launch_description_from_python_launch_file(
        str(LAUNCH_PATH)
    )
    assert description is not None


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: dict) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _candidate(tmp_path: Path) -> tuple[Path, str, str]:
    candidate = tmp_path / 'candidate'
    candidate.mkdir()
    for name, filename in ARTIFACT_NAMES.items():
        (candidate / filename).write_bytes(f'{name}-bytes'.encode())
    checkpoint_sha = _sha256(candidate / ARTIFACT_NAMES['checkpoint'])
    candidate_id = 'unit-test-v4-shadow-candidate'
    artifacts = {
        name: {'path': filename, 'sha256': _sha256(candidate / filename)}
        for name, filename in ARTIFACT_NAMES.items()
    }
    _write_json(candidate / 'runtime_promotion_manifest.json', {
        'schema_version': 'room315.visual_runtime_promotion.v4.v1',
        'candidate_id': candidate_id,
        'immutable': True,
        'deployment_mode': 'shadow',
        'shadow_execution_authorized': True,
        'automatic_promotion_allowed': False,
        'manual_review_approved': False,
        'manual_runtime_review_status': 'pending',
        'artifacts': artifacts,
    })
    manifest_sha = _sha256(candidate / 'runtime_promotion_manifest.json')
    _write_json(candidate / 'candidate_state.json', {
        'schema_version': 'room315.deployment_candidate_state.v4.v1',
        'candidate_id': candidate_id,
        'deployment_mode': 'shadow',
        'shadow_execution_authorized': True,
        'automatic_promotion_allowed': False,
        'active_runtime_selected': False,
        'checkpoint_filename': ARTIFACT_NAMES['checkpoint'],
        'checkpoint_sha256': checkpoint_sha,
    })
    _write_json(candidate / 'acceptance_scenarios.json', {
        'schema_version': 'room315.runtime_acceptance_scenarios.v1',
        'candidate_id': candidate_id,
        'runtime_candidate': {
            'runtime_generation': 'v4',
            'runtime_mode': 'shadow',
            'automatic_promotion_allowed': False,
        },
        'scenarios': [{
            'scenario_id': 'accept_dense',
            'gazebo_setup': {
                'left_active_identities': ['L1', 'L4'],
                'right_active_identities': ['R2', 'R4'],
                'left_start_positions': ['A12E@0.2', 'A34E@0.7'],
                'right_start_positions': ['A12E@0.3', 'A34E@0.8'],
                'left_loaded_identities': ['L4'],
                'right_loaded_identities': ['R2'],
            },
        }],
    })
    (candidate / 'runtime_ros_parameters.yaml').write_text(
        'room_315_visual_state_inference_node:\n'
        '  ros__parameters:\n'
        '    runtime_generation: v4\n'
        '    runtime_mode: shadow\n',
        encoding='utf-8',
    )
    payloads = sorted(
        path for path in candidate.iterdir() if path.name != 'SHA256SUMS'
    )
    (candidate / 'SHA256SUMS').write_text(
        ''.join(f'{_sha256(path)}  {path.name}\n' for path in payloads),
        encoding='utf-8',
    )
    return candidate, manifest_sha, checkpoint_sha


def test_helper_verifies_candidate_hashes_and_exact_default_scenario(tmp_path):
    launch = _load_launch_module()
    candidate, manifest_sha, checkpoint_sha = _candidate(tmp_path)

    verified = launch._verify_shadow_candidate(
        candidate,
        scenario_id='accept_dense',
        expected_manifest_sha256=manifest_sha,
        expected_checkpoint_sha256=checkpoint_sha,
    )

    assert verified.candidate_id == 'unit-test-v4-shadow-candidate'
    assert verified.promotion_manifest_sha256 == manifest_sha
    assert verified.checkpoint_sha256 == checkpoint_sha
    assert verified.scenario_id == 'accept_dense'
    assert verified.shuttle_launch_arguments == {
        'identity_selection_mode': 'explicit',
        'left_active_identities': 'L1,L4',
        'right_active_identities': 'R2,R4',
        'left_shuttle_count': '2',
        'right_shuttle_count': '2',
        'left_start_positions': 'A12E@0.2,A34E@0.7',
        'right_start_positions': 'A12E@0.3,A34E@0.8',
        'left_loaded_shuttles': 'L4',
        'right_loaded_shuttles': 'R2',
    }


@pytest.mark.parametrize('failure', ('manifest_hash', 'checkpoint_hash', 'scenario'))
def test_helper_fails_closed_on_hash_or_scenario_mismatch(tmp_path, failure):
    launch = _load_launch_module()
    candidate, manifest_sha, checkpoint_sha = _candidate(tmp_path)
    scenario_id = 'accept_dense'
    if failure == 'manifest_hash':
        manifest_sha = '0' * 64
    elif failure == 'checkpoint_hash':
        checkpoint_sha = '1' * 64
    else:
        scenario_id = 'not_in_the_candidate'

    with pytest.raises(RuntimeError, match='SHA-256|scenario'):
        launch._verify_shadow_candidate(
            candidate,
            scenario_id=scenario_id,
            expected_manifest_sha256=manifest_sha,
            expected_checkpoint_sha256=checkpoint_sha,
        )


def test_helper_rejects_candidate_scenario_binding_mismatch(tmp_path):
    launch = _load_launch_module()
    candidate, manifest_sha, checkpoint_sha = _candidate(tmp_path)
    scenarios_path = candidate / 'acceptance_scenarios.json'
    scenarios = json.loads(scenarios_path.read_text(encoding='utf-8'))
    scenarios['candidate_id'] = 'different-candidate'
    _write_json(scenarios_path, scenarios)
    sums = candidate / 'SHA256SUMS'
    payloads = sorted(path for path in candidate.iterdir() if path != sums)
    sums.write_text(
        ''.join(f'{_sha256(path)}  {path.name}\n' for path in payloads),
        encoding='utf-8',
    )

    with pytest.raises(RuntimeError, match='scenario manifest'):
        launch._verify_shadow_candidate(
            candidate,
            scenario_id='accept_dense',
            expected_manifest_sha256=manifest_sha,
            expected_checkpoint_sha256=checkpoint_sha,
        )


def test_helper_refuses_output_reuse_or_candidate_mutation(tmp_path):
    launch = _load_launch_module()
    candidate, _, _ = _candidate(tmp_path)
    existing = tmp_path / 'existing-output'
    existing.mkdir()

    with pytest.raises(RuntimeError, match='reuse'):
        launch._new_shadow_output_root(
            existing,
            candidate_directory=candidate.resolve(),
        )
    with pytest.raises(RuntimeError, match='inside the candidate'):
        launch._new_shadow_output_root(
            candidate / 'new-output',
            candidate_directory=candidate.resolve(),
        )
    assert launch._new_shadow_output_root(
        tmp_path / 'new-output',
        candidate_directory=candidate.resolve(),
    ) == (tmp_path / 'new-output').resolve()


def test_launch_stages_readiness_then_includes_isolated_dual_shadow():
    text = LAUNCH_PATH.read_text(encoding='utf-8')
    dual_text = DUAL_SHADOW_PATH.read_text(encoding='utf-8')

    assert "default_value='accept_dense'" in text
    assert '/launch/room_315_only.launch.py' in text
    assert '/launch/room_315_dual_kinematic_shuttles.launch.py' in text
    assert '/launch/room_315_vla_supervisor.launch.py' in text
    assert '/launch/room_315_visual_state_v4_shadow.launch.py' in text
    assert "_after_success('world', [shuttles, scene_gate])" in text
    assert "_after_success('scene', [cameras, camera_gate])" in text
    assert "_after_success('camera', [shadow_pair])" in text
    assert "'enable_camera_bridge': 'false'" in text
    assert "'enable_dataset_recorder': 'false'" in text
    assert "'start_enabled': 'false'" in text
    assert "'v4_promotion_manifest': str(verified.promotion_manifest)" in text
    assert "'minimum_paired_frames': str(minimum_frames)" in text
    assert "'duration_s': str(duration_s)" in text
    rollback_config = (
        '/config/room_315_vla/visual_state_runtime_v3_rollback.yaml'
    )
    assert rollback_config in text
    assert rollback_config in dual_text
    assert '/config/room_315_vla/visual_state_runtime.yaml' not in text
    assert '/config/room_315_vla/visual_state_runtime.yaml' not in dual_text

    assert "node_name='room_315_visual_state_inference_node'" in dual_text
    assert "runtime_generation='v3'" in dual_text
    assert "runtime_mode='active'" in dual_text
    assert 'target_action=comparator' in dual_text
    assert 'Shutdown(' in dual_text


def test_launch_has_no_execution_or_performance_recorder_path():
    text = LAUNCH_PATH.read_text(encoding='utf-8')

    assert 'room_315_task_execution' not in text
    assert 'room_315_runtime_acceptance_recorder.py' not in text
    assert 'room_315_runtime_acceptance_report.py' not in text
    assert 'plansys2_update_enabled' not in text
    assert 'enable_task_execution' not in text
    assert 'execution_enabled' not in text
    assert 'quality_ground_truth_available' not in text
    assert "if output.exists():" in text
    assert 'refusing to reuse V4 shadow output' in text
