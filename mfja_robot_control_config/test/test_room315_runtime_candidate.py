import json
import stat
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from room_315_runtime_acceptance_report import REQUIRED_COVERAGE
from room_315_runtime_acceptance_report import REQUIRED_RECORD_FIELDS
from room_315_runtime_acceptance_report import build_report
from room_315_runtime_acceptance_report import load_json
from room_315_runtime_acceptance_report import validate_scenario_manifest
from room_315_visual_runtime import ArtifactHashes
from room_315_visual_runtime import ArtifactPaths
from room_315_visual_runtime import FIXED_IDENTITY_ORDER
from room_315_visual_runtime import Room315VisualModelRuntime
from room_315_visual_runtime import VisualRuntimeError
from room_315_visual_runtime import _verify_runtime_configuration
from room_315_visual_runtime import sha256_file
from room_315_visual_runtime import verify_artifacts


CANDIDATE = Path(
    '/home/tiago/room315_visual_runtime_candidate_experiment_a_full_'
    'seed31520260730_epoch24_4cb9cd88'
)
CHECKPOINT_SHA256 = (
    '4cb9cd88b0199bf38bcfb08741e22bcadca54aeb0036d757784531a38cdd6a70'
)
ROLLBACK_SHA256 = (
    '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
)


def _candidate_or_skip() -> Path:
    if not CANDIDATE.is_dir():
        pytest.skip('runtime candidate has not been built on this host')
    return CANDIDATE


def _hashes(path: Path) -> ArtifactHashes:
    return ArtifactHashes(
        checkpoint=sha256_file(path / 'best.pt'),
        target_stats=sha256_file(path / 'target_stats.json'),
        vectorizer=sha256_file(path / 'visual_label_vectorizer.json'),
        training_config=sha256_file(path / 'training_config.json'),
        run_metadata=sha256_file(path / 'run_metadata.json'),
        runtime_configuration=sha256_file(path / 'runtime_configuration.json'),
    )


def test_candidate_hash_manifest_and_read_only_permissions():
    candidate = _candidate_or_skip()
    assert sha256_file(candidate / 'best.pt') == CHECKPOINT_SHA256
    lines = (candidate / 'SHA256SUMS').read_text().splitlines()
    assert lines
    for line in lines:
        digest, filename = line.split('  ', 1)
        assert sha256_file(candidate / filename) == digest
    assert not stat.S_IMODE(candidate.stat().st_mode) & 0o222
    for path in candidate.iterdir():
        assert not stat.S_IMODE(path.stat().st_mode) & 0o222


def test_candidate_artifacts_verify_vectorizer_stats_and_contract():
    candidate = _candidate_or_skip()
    artifacts = verify_artifacts(
        ArtifactPaths(candidate / 'best.pt', candidate),
        _hashes(candidate),
    )
    assert artifacts.expected_checkpoint_epoch == 24
    assert artifacts.vectorizer.dim == 200
    assert tuple(artifacts.vectorizer_json['fixed_identity_order']) == (
        FIXED_IDENTITY_ORDER
    )
    assert artifacts.target_mean.shape == (200,)
    assert artifacts.target_std.shape == (200,)
    assert (artifacts.target_std > 0).all()


@pytest.mark.parametrize(
    ('path', 'value', 'message'),
    [
        (('model_contract', 'output_dimension'), 199, 'output dimension'),
        (('model_contract', 'identity_order'), ['L1'], 'identity order'),
        (('model_contract', 'paired_rgb_input_shape'), ['B', 3, 224, 224], 'input'),
        (('model_contract', 'checkpoint_loading'), 'non_strict', 'strict'),
        (('checkpoint', 'epoch'), -1, 'negative'),
        (('artifact_sha256', 'best.pt'), '0' * 64, 'hash mismatch'),
    ],
)
def test_runtime_contract_mismatches_fail_closed(path, value, message):
    candidate = _candidate_or_skip()
    configuration = load_json(candidate / 'runtime_configuration.json')
    modified = json.loads(json.dumps(configuration))
    modified[path[0]][path[1]] = value
    actual = {
        name: sha256_file(candidate / name)
        for name in (
            'best.pt',
            'target_stats.json',
            'visual_label_vectorizer.json',
            'training_config.json',
            'run_metadata.json',
            'runtime_configuration.json',
        )
    }
    with pytest.raises(VisualRuntimeError, match=message):
        _verify_runtime_configuration(modified, actual_hashes=actual)


def test_candidate_checkpoint_strict_load_without_inference_or_dataset_access():
    pytest.importorskip('torch')
    pytest.importorskip('torchvision')
    candidate = _candidate_or_skip()
    artifacts = verify_artifacts(
        ArtifactPaths(candidate / 'best.pt', candidate),
        _hashes(candidate),
    )
    runtime = Room315VisualModelRuntime(artifacts, device='cpu')
    runtime.load()
    assert runtime.ready
    assert runtime.model.head[-1].out_features == 200


def test_acceptance_scenarios_cover_required_cases_and_report_never_approves():
    candidate = _candidate_or_skip()
    manifest = load_json(candidate / 'acceptance_scenarios.json')
    rows = validate_scenario_manifest(manifest)
    coverage = {
        tag for row in rows for tag in row.get('coverage', [])
    }
    assert coverage >= set(REQUIRED_COVERAGE)
    report = build_report(
        candidate_state=load_json(candidate / 'candidate_state.json'),
        manifest=manifest,
        event_records=[],
    )
    assert report['deployment_state'] == 'candidate'
    assert report['acceptance_status'] == 'not_run'
    assert not report['automatic_deployment_approval']
    assert not report['approval']['approved']
    assert all(
        set(REQUIRED_RECORD_FIELDS).issubset(row)
        for row in report['records']
    )


def test_candidate_selection_is_explicit_and_rollback_remains_default():
    candidate = _candidate_or_skip()
    default_yaml = (
        ROOT / 'config/room_315_vla/visual_state_runtime.yaml'
    ).read_text()
    assert ROLLBACK_SHA256 in default_yaml
    assert CHECKPOINT_SHA256 not in default_yaml
    environment = (candidate / 'activate_candidate.env').read_text()
    assert 'ROOM315_VISUAL_MODEL_PATH' in environment
    assert CHECKPOINT_SHA256 in environment
    node_source = (SCRIPTS / 'room_315_visual_state_inference_node.py').read_text()
    assert 'ROOM315_VISUAL_MODEL_PATH' in node_source
    assert 'raw_model_prediction_topic' in node_source


def test_previous_and_new_checkpoints_match_required_hashes():
    candidate = _candidate_or_skip()
    rollback = Path(
        '/home/tiago/room315_full_training_approved_archive_seed31520260730/'
        'results/run/best.pt'
    )
    assert sha256_file(rollback) == ROLLBACK_SHA256
    assert sha256_file(candidate / 'best.pt') == CHECKPOINT_SHA256


def test_acceptance_launch_is_observation_only_by_default():
    candidate = _candidate_or_skip()
    wrapper = (candidate / 'run_gazebo_runtime_acceptance.sh').read_text()
    launch = (ROOT / 'launch/room_315_runtime_acceptance.launch.py').read_text()
    assert 'enable_task_execution:=false execution_enabled:=false' in wrapper
    assert "default_value='false'" in launch
    assert "'automatic_deployment_approval': False" in (
        SCRIPTS / 'room_315_runtime_acceptance_report.py'
    ).read_text()
