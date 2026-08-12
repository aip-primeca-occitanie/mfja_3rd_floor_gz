import hashlib
import json
import stat
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

import room_315_runtime_acceptance_campaign_v4 as campaign
import room_315_runtime_acceptance_report as reporter


SCENARIO_ROWS = (
    ('accept_l4_loaded', 'l4_loaded'),
    ('accept_r4_loaded', 'r4_loaded'),
    ('accept_exact_l2_l4_r4', 'exact_l2_l4_r4'),
    ('accept_right_slot3_plus_005', 'right_slot3_deliberate_offset'),
    ('accept_sparse', 'sparse_scene'),
    ('accept_dense', 'dense_scene'),
    ('accept_multi_blocker', 'multi_blocker_scene'),
)


def _write_json(path, value):
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def candidate(tmp_path):
    root = tmp_path / 'candidate'
    root.mkdir()
    checkpoint = root / 'checkpoint_epoch_011.pt'
    checkpoint.write_bytes(b'unit-test-checkpoint')
    state = {
        'schema_version': 'room315.deployment_candidate_state.v4.v1',
        'candidate_id': 'room315-v4-unit-candidate',
        'deployment_mode': 'shadow',
        'shadow_execution_authorized': True,
        'automatic_promotion_allowed': False,
        'active_runtime_selected': False,
        'checkpoint_filename': checkpoint.name,
        'checkpoint_sha256': _sha256(checkpoint),
    }
    scenarios = []
    for index, (scenario_id, coverage) in enumerate(SCENARIO_ROWS):
        identity = 'L1' if index % 2 else 'R1'
        scenarios.append({
            'scenario_id': scenario_id,
            'coverage': [coverage],
            'ground_truth': {
                'present_identities': [identity],
                'shuttles': [{
                    'identity': identity,
                    'side': 'left' if identity.startswith('L') else 'right',
                    'segment': 'A12E',
                    'loaded_state': 'empty',
                    's_ratio': 0.2,
                }],
                'model_prediction_target': False,
            },
        })
    manifest = {
        'schema_version': 'room315.runtime_acceptance_scenarios.v1',
        'candidate_id': state['candidate_id'],
        'runtime_candidate': {
            'runtime_generation': 'v4',
            'runtime_mode': 'shadow',
            'automatic_promotion_allowed': False,
        },
        'scenarios': scenarios,
    }
    _write_json(root / 'candidate_state.json', state)
    _write_json(root / 'acceptance_scenarios.json', manifest)
    (root / 'runtime_ros_parameters.yaml').write_text(
        'room_315_visual_state_inference_node:\n'
        '  ros__parameters:\n'
        '    runtime_generation: v4\n'
        '    runtime_mode: shadow\n'
        '    dry_run_state_fusion: true\n'
        '    plansys2_update_enabled: false\n',
        encoding='utf-8',
    )
    payloads = sorted(path for path in root.iterdir() if path.is_file())
    (root / 'SHA256SUMS').write_text(
        ''.join(f'{_sha256(path)}  {path.name}\n' for path in payloads),
        encoding='utf-8',
    )
    return root


def _complete_event(row):
    observation = {
        'ground_truth_comparison': {
            'passed': True,
            'per_identity': {
                row['ground_truth']['present_identities'][0]: {
                    's_ratio_absolute_error': 0.0,
                },
            },
        },
    }
    event = {
        'schema_version': campaign.EVENT_SCHEMA,
        'record_status': 'complete',
        'failure_reasons': [],
        'scenario_id': row['scenario_id'],
        'coverage': row['coverage'],
        'ground_truth': row['ground_truth'],
        'observation_only': True,
    }
    for field in reporter.REQUIRED_RECORD_FIELDS[1:]:
        event[field] = {'status': 'observed'}
    event['execution_decision'] = {
        'status': 'observed',
        'allowed': False,
        'reason': 'observation_only_acceptance_contract',
    }
    event['reobservation_and_effect_verification'] = {
        'status': 'observed',
        'actuation_performed': False,
        'accepted_reobservations': [observation, observation, observation],
    }
    return event


def _launch_value(command, key):
    prefix = f'{key}:='
    return next(value[len(prefix):] for value in command if value.startswith(prefix))


class FakeLaunchRunner:
    def __init__(self, manifest, *, mutate=None, nonzero=None):
        self.by_id = {row['scenario_id']: row for row in manifest['scenarios']}
        self.mutate = mutate or {}
        self.nonzero = set(nonzero or ())
        self.calls = []

    def __call__(self, command, timeout_s):
        self.calls.append((list(command), timeout_s))
        scenario_id = _launch_value(command, 'scenario_id')
        output = Path(_launch_value(command, 'output_root'))
        assert not output.exists()
        event = _complete_event(self.by_id[scenario_id])
        if scenario_id in self.mutate:
            self.mutate[scenario_id](event)
        event_dir = output / 'events'
        event_dir.mkdir(parents=True)
        _write_json(event_dir / f'{scenario_id}.json', event)
        return campaign.ProcessResult(
            returncode=7 if scenario_id in self.nonzero else 0,
            stdout=f'finished {scenario_id}\n',
        )


class FakeReportRunner:
    def __init__(self):
        self.calls = []

    def __call__(self, command, timeout_s):
        self.calls.append((list(command), timeout_s))
        candidate_dir = Path(command[command.index('--candidate-directory') + 1])
        event_dir = Path(command[command.index('--event-directory') + 1])
        output = Path(command[command.index('--output') + 1])
        state = reporter.load_json(candidate_dir / 'candidate_state.json')
        manifest = reporter.load_json(candidate_dir / 'acceptance_scenarios.json')
        events = [reporter.load_json(path) for path in sorted(event_dir.glob('*.json'))]
        report = reporter.build_report(
            candidate_state=state,
            manifest=manifest,
            event_records=events,
        )
        reporter.atomic_write_new(output, report)
        return campaign.ProcessResult(returncode=0, stdout='report written\n')


def _options(candidate, output, **overrides):
    values = {
        'candidate_directory': candidate,
        'output_root': output,
        'world_readiness_timeout_s': 1.0,
        'scene_readiness_timeout_s': 2.0,
        'camera_readiness_timeout_s': 3.0,
        'runtime_readiness_timeout_s': 4.0,
        'record_duration_s': 5.0,
        'process_timeout_margin_s': 6.0,
        'report_timeout_s': 7.0,
    }
    values.update(overrides)
    return campaign.CampaignOptions(**values)


def _restore_write_permissions(output):
    aggregate = output / 'aggregate'
    for path in [aggregate, *(path for path in aggregate.rglob('*') if path.is_dir())]:
        path.chmod(0o755)


def test_campaign_runs_all_seven_in_manifest_order_and_reports_once(
    candidate, tmp_path,
):
    manifest = json.loads((candidate / 'acceptance_scenarios.json').read_text())
    launches = FakeLaunchRunner(manifest)
    reports = FakeReportRunner()
    output = tmp_path / 'campaign'
    options = _options(candidate, output)

    summary = campaign.run_campaign(
        options,
        launch_runner=launches,
        report_runner=reports,
    )

    assert summary['campaign_complete'] is True
    assert summary['complete_event_count'] == 7
    assert summary['acceptance_status'] == 'complete_pending_human_decision'
    assert summary['observation_only'] is True
    assert summary['task_execution_enabled'] is False
    assert summary['execution_enabled'] is False
    assert len(launches.calls) == 7
    assert len(reports.calls) == 1
    assert [
        _launch_value(command, 'scenario_id')
        for command, _timeout in launches.calls
    ] == [row[0] for row in SCENARIO_ROWS]
    for command, timeout_s in launches.calls:
        assert command[:4] == [
            'ros2', 'launch', 'mfja_robot_control_config',
            'room_315_runtime_acceptance.launch.py',
        ]
        assert 'gui:=false' in command
        assert 'enable_task_execution:=false' in command
        assert 'execution_enabled:=false' in command
        assert 'enable_task_execution:=true' not in command
        assert 'execution_enabled:=true' not in command
        assert 'world_readiness_timeout_s:=1.0' in command
        assert 'scene_readiness_timeout_s:=2.0' in command
        assert 'camera_readiness_timeout_s:=3.0' in command
        assert 'runtime_readiness_timeout_s:=4.0' in command
        assert 'record_duration_s:=5.0' in command
        assert timeout_s == options.scenario_process_timeout_s == 21.0

    aggregate_events = sorted((output / 'aggregate' / 'events').glob('*.json'))
    assert len(aggregate_events) == 7
    report_path = output / 'aggregate' / 'acceptance_report.json'
    report = json.loads(report_path.read_text())
    assert report['complete_scenario_count'] == 7
    assert report['automatic_deployment_approval'] is False
    digest_line = (
        output / 'aggregate' / 'acceptance_report.sha256'
    ).read_text().strip()
    assert digest_line == f'{_sha256(report_path)}  acceptance_report.json'
    assert not stat.S_IMODE(report_path.stat().st_mode) & 0o222
    assert not stat.S_IMODE((output / 'aggregate').stat().st_mode) & 0o222
    _restore_write_permissions(output)


def test_only_successful_complete_exact_events_are_aggregated(candidate, tmp_path):
    manifest = json.loads((candidate / 'acceptance_scenarios.json').read_text())

    def wrong_ground_truth(event):
        event['ground_truth'] = {'tampered': True}

    launches = FakeLaunchRunner(
        manifest,
        mutate={'accept_sparse': wrong_ground_truth},
        nonzero={'accept_dense'},
    )
    reports = FakeReportRunner()
    output = tmp_path / 'partial_campaign'
    summary = campaign.run_campaign(
        _options(candidate, output),
        launch_runner=launches,
        report_runner=reports,
    )

    assert summary['campaign_complete'] is False
    assert summary['complete_event_count'] == 5
    assert summary['acceptance_status'] == 'incomplete'
    aggregate_names = {
        path.name for path in (output / 'aggregate' / 'events').glob('*.json')
    }
    assert 'accept_sparse.json' not in aggregate_names
    assert 'accept_dense.json' not in aggregate_names
    sparse = next(
        row for row in summary['scenario_results']
        if row['scenario_id'] == 'accept_sparse'
    )
    assert 'ground truth does not match' in sparse['rejection_reason']
    dense = next(
        row for row in summary['scenario_results']
        if row['scenario_id'] == 'accept_dense'
    )
    assert dense['rejection_reason'] == 'scenario_process_exit_7'
    assert len(reports.calls) == 1
    _restore_write_permissions(output)


def test_campaign_refuses_any_output_reuse_before_launch(candidate, tmp_path):
    output = tmp_path / 'existing'
    output.mkdir()
    manifest = json.loads((candidate / 'acceptance_scenarios.json').read_text())
    launches = FakeLaunchRunner(manifest)
    with pytest.raises(
        campaign.RuntimeAcceptanceCampaignError,
        match='refusing to reuse campaign output',
    ):
        campaign.run_campaign(
            _options(candidate, output),
            launch_runner=launches,
            report_runner=FakeReportRunner(),
        )
    assert not launches.calls


def test_candidate_bundle_hash_manifest_is_verified_before_launch(
    candidate, tmp_path,
):
    with (candidate / 'acceptance_scenarios.json').open('a', encoding='utf-8') as stream:
        stream.write(' ')
    with pytest.raises(
        campaign.RuntimeAcceptanceCampaignError,
        match='failed SHA-256 verification',
    ):
        campaign.run_campaign(
            _options(candidate, tmp_path / 'campaign'),
            launch_runner=lambda _command, _timeout: pytest.fail('must not launch'),
            report_runner=lambda _command, _timeout: pytest.fail('must not report'),
        )


def test_complete_event_still_rejects_execution_or_wrong_identity(candidate):
    verified = campaign.verify_candidate(candidate)
    row = verified.scenarios[0]
    event = _complete_event(row)
    event['execution_decision']['allowed'] = True
    path = candidate.parent / 'unsafe_event.json'
    _write_json(path, event)
    with pytest.raises(
        campaign.RuntimeAcceptanceCampaignError,
        match='observation-only',
    ):
        campaign.validate_complete_event(path, row)

    event = _complete_event(row)
    event['scenario_id'] = 'another_scenario'
    _write_json(path, event)
    with pytest.raises(
        campaign.RuntimeAcceptanceCampaignError,
        match='scenario ID does not match',
    ):
        campaign.validate_complete_event(path, row)


def test_cli_defaults_headless_and_exposes_only_safe_timing_controls(tmp_path):
    options = campaign.parse_args([
        '--candidate-directory', str(tmp_path / 'candidate'),
        '--output-root', str(tmp_path / 'output'),
        '--world-readiness-timeout-s', '11',
        '--scene-readiness-timeout-s', '12',
        '--camera-readiness-timeout-s', '13',
        '--runtime-readiness-timeout-s', '14',
        '--record-duration-s', '15',
        '--process-timeout-margin-s', '16',
        '--report-timeout-s', '17',
    ])
    assert options.gui is False
    assert options.world_readiness_timeout_s == 11
    assert options.scene_readiness_timeout_s == 12
    assert options.camera_readiness_timeout_s == 13
    assert options.runtime_readiness_timeout_s == 14
    assert options.record_duration_s == 15
    assert options.process_timeout_margin_s == 16
    assert options.report_timeout_s == 17
    with pytest.raises(SystemExit):
        campaign.parse_args([
            '--candidate-directory', str(tmp_path / 'candidate'),
            '--output-root', str(tmp_path / 'output'),
            '--execution-enabled', 'true',
        ])
