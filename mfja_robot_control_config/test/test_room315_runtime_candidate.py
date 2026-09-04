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
import room_315_build_runtime_candidate as historical_candidate


CANDIDATE = Path(
    '/home/tiago/room315_visual_runtime_candidate_experiment_a_full_'
    'seed31520260730_epoch24_4cb9cd88'
)
CHECKPOINT_SHA256 = (
    '4cb9cd88b0199bf38bcfb08741e22bcadca54aeb0036d757784531a38cdd6a70'
)
V4_CHECKPOINT_SHA256 = (
    '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
)
V4_RUNTIME_MANIFEST_SHA256 = (
    '506cae0511cf1675fdd666103ce7fc0b5980eb5e68d4cbadf0af99d9ee9560da'
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


def test_v4_is_the_only_packaged_deployable_visual_runtime():
    visual_default = (
        ROOT / 'config/room_315_visual_state/visual_state_runtime.yaml'
    ).read_text()
    task_default = (
        ROOT / 'config/room_315_task_execution/task_execution_runtime.yaml'
    ).read_text()
    visual_v3_config = (
        ROOT / 'config/room_315_visual_state/visual_state_runtime_v3_rollback.yaml'
    )
    task_v3_config = (
        ROOT / 'config/room_315_task_execution/task_execution_runtime_v3_rollback.yaml'
    )

    assert 'runtime_generation: v4' in visual_default
    assert 'runtime_mode: active' in visual_default
    assert V4_RUNTIME_MANIFEST_SHA256 in visual_default
    assert CHECKPOINT_SHA256 not in visual_default
    assert 'room315.visual_state.v4' in task_default
    assert V4_CHECKPOINT_SHA256 in task_default
    assert 'execution_enabled: false' in task_default
    assert not visual_v3_config.exists()
    assert not task_v3_config.exists()

    node_source = (SCRIPTS / 'room_315_visual_state_inference_node.py').read_text()
    assert "{'v4'}" in node_source
    assert 'ROOM315_VISUAL_MODEL_PATH' not in node_source
    assert 'raw_model_prediction_topic' in node_source


def test_historical_v3_builder_publishes_no_runtime_entrypoints(
    tmp_path,
    monkeypatch,
):
    source = tmp_path / 'source'
    sidecars = tmp_path / 'sidecars'
    source.mkdir()
    sidecars.mkdir()

    checkpoint_path = source / 'best.pt'
    checkpoint_path.write_bytes(b'historical-v3-checkpoint')
    (source / 'run_metadata.json').write_text('{}\n', encoding='utf-8')
    (source / 'final_report.json').write_text('{}\n', encoding='utf-8')

    vectorizer = {
        'dim': 200,
        'fixed_identity_order': list(historical_candidate.IDENTITIES),
    }
    target_stats = {'fixture': True}
    (sidecars / 'visual_label_vectorizer.json').write_text(
        json.dumps(vectorizer), encoding='utf-8'
    )
    (sidecars / 'target_stats.json').write_text(
        json.dumps(target_stats), encoding='utf-8'
    )
    initialization_checkpoint = sidecars / 'best.pt'
    initialization_checkpoint.write_bytes(b'historical-v3-initialization')

    checkpoint = {
        'epoch': 24,
        'continuation_epoch': 10,
        'label_vectorizer': vectorizer,
        'target_stats': target_stats,
    }
    monkeypatch.setattr(historical_candidate, 'CHECKPOINT', checkpoint_path)
    monkeypatch.setattr(historical_candidate, 'SOURCE_OUTPUT', source)
    monkeypatch.setattr(historical_candidate, 'AUTHORITATIVE_SIDECARS', sidecars)
    monkeypatch.setattr(
        historical_candidate,
        'INITIALIZATION_CHECKPOINT',
        initialization_checkpoint,
    )
    monkeypatch.setattr(
        historical_candidate,
        'INITIALIZATION_CHECKPOINT_SHA256',
        historical_candidate.sha256_file(initialization_checkpoint),
    )
    monkeypatch.setattr(
        historical_candidate,
        'CHECKPOINT_SHA256',
        historical_candidate.sha256_file(checkpoint_path),
    )
    monkeypatch.setattr(
        historical_candidate,
        'validate_and_load_checkpoint',
        lambda: checkpoint,
    )

    output = tmp_path / 'historical-v3-archive'
    try:
        historical_candidate.build(output)

        forbidden_runtime_entrypoints = {
            'activate_candidate.env',
            'runtime_ros_parameters.yaml',
            'room_315_runtime_acceptance_report.py',
            'run_gazebo_runtime_acceptance.sh',
            'generate_acceptance_report.sh',
        }
        assert forbidden_runtime_entrypoints.isdisjoint(
            path.name for path in output.iterdir()
        )
        assert not any(path.suffix in {'.py', '.sh'} for path in output.iterdir())

        runtime_configuration = json.loads(
            (output / 'runtime_configuration.json').read_text(encoding='utf-8')
        )
        assert runtime_configuration['deployment_state'] == (
            historical_candidate.ARCHIVE_DEPLOYMENT_STATE
        )
        assert runtime_configuration['archive_policy'] == {
            'runtime_execution_supported': False,
            'required_runtime_generation': 'v4',
        }
        assert 'selection' not in runtime_configuration

        readme = (output / 'README.md').read_text(encoding='utf-8')
        assert 'historical V3 archive' in readme
        assert 'not runnable' in readme
        assert 'ros2 launch' not in readme
        assert 'run_gazebo_runtime_acceptance.sh' not in readme
    finally:
        if output.exists():
            output.chmod(0o755)
            for path in output.iterdir():
                path.chmod(0o644)
