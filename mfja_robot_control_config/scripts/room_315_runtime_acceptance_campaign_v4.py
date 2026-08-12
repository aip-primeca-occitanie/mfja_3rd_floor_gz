#!/usr/bin/env python3
"""Run the seven V4 Room 315 runtime-acceptance scenarios safely.

The campaign is intentionally observation-only.  It starts a fresh Gazebo
process for each scenario, accepts only complete and manifest-matching event
records, and invokes the non-approving acceptance reporter once over the
aggregate event directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import signal
import stat
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_runtime_acceptance_report import REQUIRED_RECORD_FIELDS
from room_315_runtime_acceptance_report import validate_scenario_manifest


EXPECTED_SCENARIO_COUNT = 7
EVENT_SCHEMA = 'room315.runtime_acceptance_event.v1'
REPORT_SCHEMA = 'room315.runtime_acceptance_report.v1'
SAFE_SCENARIO_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}')
HEX_SHA256 = re.compile(r'[0-9a-f]{64}')


class RuntimeAcceptanceCampaignError(RuntimeError):
    """Raised when the campaign cannot continue without weakening a guard."""


@dataclass(frozen=True)
class CampaignOptions:
    candidate_directory: Path
    output_root: Path
    gui: bool = False
    world_readiness_timeout_s: float = 45.0
    scene_readiness_timeout_s: float = 45.0
    camera_readiness_timeout_s: float = 45.0
    runtime_readiness_timeout_s: float = 120.0
    record_duration_s: float = 60.0
    process_timeout_margin_s: float = 30.0
    report_timeout_s: float = 30.0

    @property
    def scenario_process_timeout_s(self) -> float:
        return (
            self.world_readiness_timeout_s
            + self.scene_readiness_timeout_s
            + self.camera_readiness_timeout_s
            + self.runtime_readiness_timeout_s
            + self.record_duration_s
            + self.process_timeout_margin_s
        )


@dataclass(frozen=True)
class ProcessResult:
    returncode: int
    stdout: str = ''
    stderr: str = ''
    timed_out: bool = False


@dataclass(frozen=True)
class VerifiedCandidate:
    directory: Path
    state: dict[str, Any]
    manifest: dict[str, Any]
    scenarios: tuple[dict[str, Any], ...]
    state_sha256: str
    manifest_sha256: str


ProcessRunner = Callable[[Sequence[str], float], ProcessResult]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise RuntimeAcceptanceCampaignError(
            f'cannot hash campaign input: {path}'
        ) from exc
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeAcceptanceCampaignError(f'cannot read JSON: {path}') from exc
    if not isinstance(value, dict):
        raise RuntimeAcceptanceCampaignError(f'expected JSON object: {path}')
    return value


def atomic_write_new(path: Path, value: dict[str, Any] | str) -> None:
    if path.exists() or path.is_symlink():
        raise RuntimeAcceptanceCampaignError(f'refusing to overwrite: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.tmp-{os.getpid()}')
    try:
        if isinstance(value, str):
            text = value
        else:
            text = json.dumps(value, indent=2, sort_keys=True) + '\n'
        temporary.write_text(text, encoding='utf-8')
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _validate_sha256sums(candidate: Path) -> None:
    sums_path = candidate / 'SHA256SUMS'
    try:
        lines = sums_path.read_text(encoding='utf-8').splitlines()
    except OSError as exc:
        raise RuntimeAcceptanceCampaignError(
            f'candidate SHA256SUMS is missing: {sums_path}'
        ) from exc
    if not lines:
        raise RuntimeAcceptanceCampaignError('candidate SHA256SUMS is empty')
    declared: dict[str, str] = {}
    for line in lines:
        try:
            digest, name = line.split('  ', 1)
        except ValueError as exc:
            raise RuntimeAcceptanceCampaignError(
                'candidate SHA256SUMS contains a malformed line'
            ) from exc
        if (
            not HEX_SHA256.fullmatch(digest)
            or Path(name).name != name
            or name in {'', '.', '..', 'SHA256SUMS'}
            or name in declared
        ):
            raise RuntimeAcceptanceCampaignError(
                f'candidate SHA256SUMS contains an unsafe entry: {name!r}'
            )
        declared[name] = digest

    payload_names = {
        path.name
        for path in candidate.iterdir()
        if path.name != 'SHA256SUMS' and path.is_file() and not path.is_symlink()
    }
    if set(declared) != payload_names:
        raise RuntimeAcceptanceCampaignError(
            'candidate SHA256SUMS payload set does not match candidate files'
        )
    for name, expected in declared.items():
        path = candidate / name
        if not path.is_file() or path.is_symlink() or sha256_file(path) != expected:
            raise RuntimeAcceptanceCampaignError(
                f'candidate artifact failed SHA-256 verification: {name}'
            )


def verify_candidate(candidate_directory: Path) -> VerifiedCandidate:
    candidate = candidate_directory.expanduser().resolve()
    if not candidate.is_dir():
        raise RuntimeAcceptanceCampaignError(
            f'candidate directory does not exist: {candidate}'
        )
    _validate_sha256sums(candidate)
    state_path = candidate / 'candidate_state.json'
    manifest_path = candidate / 'acceptance_scenarios.json'
    state = load_json(state_path)
    manifest = load_json(manifest_path)
    try:
        scenarios = validate_scenario_manifest(manifest)
    except Exception as exc:
        raise RuntimeAcceptanceCampaignError(
            f'invalid acceptance scenario manifest: {exc}'
        ) from exc
    if len(scenarios) != EXPECTED_SCENARIO_COUNT:
        raise RuntimeAcceptanceCampaignError(
            f'V4 campaign requires exactly {EXPECTED_SCENARIO_COUNT} scenarios'
        )
    for row in scenarios:
        scenario_id = str(row.get('scenario_id') or '')
        if not SAFE_SCENARIO_ID.fullmatch(scenario_id) or scenario_id in {'.', '..'}:
            raise RuntimeAcceptanceCampaignError(
                f'unsafe acceptance scenario ID: {scenario_id!r}'
            )

    candidate_id = str(state.get('candidate_id') or '')
    if not candidate_id or manifest.get('candidate_id') != candidate_id:
        raise RuntimeAcceptanceCampaignError(
            'candidate state and acceptance manifest IDs do not match'
        )
    runtime = manifest.get('runtime_candidate')
    if not isinstance(runtime, dict) or runtime != {
        'runtime_generation': 'v4',
        'runtime_mode': 'shadow',
        'automatic_promotion_allowed': False,
    }:
        raise RuntimeAcceptanceCampaignError(
            'acceptance manifest is not an isolated V4 shadow candidate'
        )
    if (
        state.get('deployment_mode') != 'shadow'
        or state.get('shadow_execution_authorized') is not True
        or state.get('automatic_promotion_allowed') is not False
        or state.get('active_runtime_selected') is not False
    ):
        raise RuntimeAcceptanceCampaignError(
            'candidate state does not authorize observation-only shadow evaluation'
        )
    checkpoint_sha256 = str(state.get('checkpoint_sha256') or '')
    if not HEX_SHA256.fullmatch(checkpoint_sha256):
        raise RuntimeAcceptanceCampaignError(
            'candidate state has an invalid checkpoint SHA-256'
        )
    checkpoint_filename = str(state.get('checkpoint_filename') or '').strip()
    if (
        not checkpoint_filename
        or Path(checkpoint_filename).name != checkpoint_filename
        or checkpoint_filename in {'.', '..'}
    ):
        raise RuntimeAcceptanceCampaignError(
            'candidate state has an unsafe checkpoint filename'
        )
    checkpoint_path = candidate / checkpoint_filename
    if (
        not checkpoint_path.is_file()
        or checkpoint_path.is_symlink()
        or sha256_file(checkpoint_path) != checkpoint_sha256
    ):
        raise RuntimeAcceptanceCampaignError(
            'candidate checkpoint does not match candidate state'
        )
    runtime_config = candidate / 'runtime_ros_parameters.yaml'
    if not runtime_config.is_file() or runtime_config.is_symlink():
        raise RuntimeAcceptanceCampaignError(
            'candidate runtime_ros_parameters.yaml is missing or unsafe'
        )
    return VerifiedCandidate(
        directory=candidate,
        state=state,
        manifest=manifest,
        scenarios=tuple(scenarios),
        state_sha256=sha256_file(state_path),
        manifest_sha256=sha256_file(manifest_path),
    )


def assert_candidate_inputs_unchanged(candidate: VerifiedCandidate) -> None:
    if (
        sha256_file(candidate.directory / 'candidate_state.json')
        != candidate.state_sha256
        or sha256_file(candidate.directory / 'acceptance_scenarios.json')
        != candidate.manifest_sha256
    ):
        raise RuntimeAcceptanceCampaignError(
            'candidate state or acceptance manifest changed during campaign'
        )


def build_scenario_command(
    candidate: VerifiedCandidate,
    scenario: dict[str, Any],
    output_root: Path,
    options: CampaignOptions,
) -> list[str]:
    """Build a launch command with execution guards that cannot be overridden."""
    return [
        'ros2',
        'launch',
        'mfja_robot_control_config',
        'room_315_runtime_acceptance.launch.py',
        f'candidate_directory:={candidate.directory}',
        f'scenario_id:={scenario["scenario_id"]}',
        f'output_root:={output_root}',
        f'gui:={str(options.gui).lower()}',
        f'world_readiness_timeout_s:={float(options.world_readiness_timeout_s)}',
        f'scene_readiness_timeout_s:={float(options.scene_readiness_timeout_s)}',
        f'camera_readiness_timeout_s:={float(options.camera_readiness_timeout_s)}',
        f'runtime_readiness_timeout_s:={float(options.runtime_readiness_timeout_s)}',
        f'record_duration_s:={float(options.record_duration_s)}',
        'enable_task_execution:=false',
        'execution_enabled:=false',
    ]


def build_report_command(
    candidate: VerifiedCandidate,
    event_directory: Path,
    output: Path,
) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT_DIR / 'room_315_runtime_acceptance_report.py'),
        '--candidate-directory',
        str(candidate.directory),
        '--event-directory',
        str(event_directory),
        '--output',
        str(output),
    ]


def _process_group_exists(group_id: int) -> bool:
    proc_root = Path('/proc')
    if proc_root.is_dir():
        try:
            for entry in proc_root.iterdir():
                if not entry.name.isdigit():
                    continue
                fields = (
                    (entry / 'stat').read_text(encoding='utf-8')
                    .rsplit(') ', 1)[1]
                    .split()
                )
                state = fields[0]
                process_group = int(fields[2])
                if process_group == group_id and state != 'Z':
                    return True
            return False
        except (IndexError, OSError, ValueError):
            pass
    try:
        os.killpg(group_id, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _wait_for_process_group_exit(group_id: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _process_group_exists(group_id):
            return True
        time.sleep(0.1)
    return not _process_group_exists(group_id)


def _terminate_process_group(process: subprocess.Popen[str]) -> None:
    """Stop the complete ros2/Gazebo process group after exit or timeout."""
    group_id = process.pid
    for shutdown_signal, wait_s in (
        (signal.SIGINT, 15.0),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(group_id, shutdown_signal)
        except ProcessLookupError:
            return
        if _wait_for_process_group_exit(group_id, wait_s):
            return
    if process.poll() is None:
        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            pass


def run_process(command: Sequence[str], timeout_s: float) -> ProcessResult:
    process = subprocess.Popen(
        list(command),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    timed_out = False
    try:
        stdout, stderr = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        _terminate_process_group(process)
        stdout, stderr = process.communicate()
    finally:
        # ros2 launch can return before every Gazebo descendant exits.  A
        # best-effort group shutdown isolates sequential scenarios.
        _terminate_process_group(process)
    return ProcessResult(
        returncode=124 if timed_out else int(process.returncode or 0),
        stdout=stdout or '',
        stderr=stderr or '',
        timed_out=timed_out,
    )


def _write_process_log(
    path: Path,
    command: Sequence[str],
    result: ProcessResult,
) -> None:
    atomic_write_new(
        path,
        '$ ' + shlex.join(command) + '\n'
        + f'returncode={result.returncode} timed_out={result.timed_out}\n'
        + '\n[stdout]\n' + result.stdout
        + '\n[stderr]\n' + result.stderr,
    )


def validate_complete_event(
    path: Path,
    expected_scenario: dict[str, Any],
) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise RuntimeAcceptanceCampaignError(
            f'acceptance event is missing or unsafe: {path}'
        )
    event = load_json(path)
    scenario_id = str(expected_scenario['scenario_id'])
    if event.get('schema_version') != EVENT_SCHEMA:
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: unsupported acceptance event schema'
        )
    if event.get('record_status') != 'complete':
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: acceptance event is not complete'
        )
    if event.get('scenario_id') != scenario_id:
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: recorded scenario ID does not match'
        )
    if event.get('ground_truth') != expected_scenario.get('ground_truth'):
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: recorded ground truth does not match manifest'
        )
    if event.get('coverage') != list(expected_scenario.get('coverage') or []):
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: recorded coverage does not match manifest'
        )
    missing = [field for field in REQUIRED_RECORD_FIELDS if field not in event]
    if missing:
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: complete event is missing fields {missing}'
        )
    execution = event.get('execution_decision')
    verification = event.get('reobservation_and_effect_verification')
    if (
        event.get('observation_only') is not True
        or not isinstance(execution, dict)
        or execution.get('allowed') is not False
        or not isinstance(verification, dict)
        or verification.get('actuation_performed') is not False
    ):
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: event violates observation-only contract'
        )
    observations = verification.get('accepted_reobservations')
    if not isinstance(observations, list) or len(observations) < 3:
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: fewer than three accepted reobservations'
        )
    if any(
        not isinstance(item, dict)
        or not isinstance(item.get('ground_truth_comparison'), dict)
        or item['ground_truth_comparison'].get('passed') is not True
        for item in observations
    ):
        raise RuntimeAcceptanceCampaignError(
            f'{scenario_id}: a reobservation does not match ground truth'
        )
    return event


def _verify_final_report(
    path: Path,
    candidate: VerifiedCandidate,
    accepted_event_count: int,
) -> dict[str, Any]:
    report = load_json(path)
    if (
        report.get('schema_version') != REPORT_SCHEMA
        or report.get('candidate_id') != candidate.state.get('candidate_id')
        or report.get('checkpoint_sha256')
        != candidate.state.get('checkpoint_sha256')
        or report.get('scenario_count') != EXPECTED_SCENARIO_COUNT
        or report.get('complete_scenario_count') != accepted_event_count
        or report.get('automatic_deployment_approval') is not False
        or (report.get('approval') or {}).get('approved') is not False
    ):
        raise RuntimeAcceptanceCampaignError(
            'aggregate acceptance report failed its non-approval contract'
        )
    expected_status = (
        'complete_pending_human_decision'
        if accepted_event_count == EXPECTED_SCENARIO_COUNT
        else 'not_run' if accepted_event_count == 0
        else 'incomplete'
    )
    if report.get('acceptance_status') != expected_status:
        raise RuntimeAcceptanceCampaignError(
            'aggregate report status does not match collected complete events'
        )
    return report


def _make_aggregate_immutable(aggregate_root: Path) -> None:
    for path in sorted(aggregate_root.rglob('*'), reverse=True):
        if path.is_file():
            path.chmod(stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
        elif path.is_dir():
            path.chmod(
                stat.S_IRUSR | stat.S_IXUSR
                | stat.S_IRGRP | stat.S_IXGRP
                | stat.S_IROTH | stat.S_IXOTH
            )
    aggregate_root.chmod(
        stat.S_IRUSR | stat.S_IXUSR
        | stat.S_IRGRP | stat.S_IXGRP
        | stat.S_IROTH | stat.S_IXOTH
    )


def _prepare_new_output_root(output_root: Path) -> Path:
    requested = output_root.expanduser()
    if requested.is_symlink():
        raise RuntimeAcceptanceCampaignError(
            f'refusing to reuse campaign output: {requested}'
        )
    output = requested.resolve()
    if output.exists() or output.is_symlink():
        raise RuntimeAcceptanceCampaignError(
            f'refusing to reuse campaign output: {output}'
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        output.mkdir()
    except FileExistsError as exc:
        raise RuntimeAcceptanceCampaignError(
            f'refusing to reuse campaign output: {output}'
        ) from exc
    (output / 'scenario_outputs').mkdir()
    (output / 'logs').mkdir()
    (output / 'aggregate' / 'events').mkdir(parents=True)
    return output


def run_campaign(
    options: CampaignOptions,
    *,
    launch_runner: ProcessRunner | None = None,
    report_runner: ProcessRunner | None = None,
) -> dict[str, Any]:
    launch_runner = launch_runner or run_process
    report_runner = report_runner or run_process
    candidate = verify_candidate(options.candidate_directory)
    output = _prepare_new_output_root(options.output_root)
    scenario_results: list[dict[str, Any]] = []
    accepted_event_count = 0

    for index, scenario in enumerate(candidate.scenarios, start=1):
        assert_candidate_inputs_unchanged(candidate)
        scenario_id = str(scenario['scenario_id'])
        scenario_output = output / 'scenario_outputs' / f'{index:02d}_{scenario_id}'
        if scenario_output.exists() or scenario_output.is_symlink():
            raise RuntimeAcceptanceCampaignError(
                f'refusing to reuse scenario output: {scenario_output}'
            )
        command = build_scenario_command(
            candidate, scenario, scenario_output, options
        )
        started = time.monotonic()
        result = launch_runner(command, options.scenario_process_timeout_s)
        elapsed_s = time.monotonic() - started
        _write_process_log(
            output / 'logs' / f'{index:02d}_{scenario_id}.log',
            command,
            result,
        )
        event_path = scenario_output / 'events' / f'{scenario_id}.json'
        accepted = False
        rejection_reason: str | None = None
        if result.returncode != 0:
            rejection_reason = (
                'scenario_process_timeout'
                if result.timed_out
                else f'scenario_process_exit_{result.returncode}'
            )
        else:
            try:
                event = validate_complete_event(event_path, scenario)
                aggregate_event = output / 'aggregate' / 'events' / event_path.name
                atomic_write_new(aggregate_event, event)
                accepted = True
                accepted_event_count += 1
            except RuntimeAcceptanceCampaignError as exc:
                rejection_reason = str(exc)
        scenario_results.append({
            'scenario_id': scenario_id,
            'launch_returncode': result.returncode,
            'timed_out': result.timed_out,
            'elapsed_s': elapsed_s,
            'complete_event_aggregated': accepted,
            'rejection_reason': rejection_reason,
            'scenario_output': str(
                Path('scenario_outputs') / f'{index:02d}_{scenario_id}'
            ),
        })

    assert_candidate_inputs_unchanged(candidate)
    report_path = output / 'aggregate' / 'acceptance_report.json'
    report_command = build_report_command(
        candidate, output / 'aggregate' / 'events', report_path
    )
    report_result = report_runner(report_command, options.report_timeout_s)
    _write_process_log(
        output / 'logs' / 'aggregate_report.log', report_command, report_result
    )
    if report_result.returncode != 0:
        raise RuntimeAcceptanceCampaignError(
            f'aggregate acceptance report failed with code '
            f'{report_result.returncode}'
        )
    report = _verify_final_report(report_path, candidate, accepted_event_count)
    report_sha256 = sha256_file(report_path)
    atomic_write_new(
        output / 'aggregate' / 'acceptance_report.sha256',
        f'{report_sha256}  acceptance_report.json\n',
    )
    campaign_complete = (
        accepted_event_count == EXPECTED_SCENARIO_COUNT
        and report.get('acceptance_status') == 'complete_pending_human_decision'
    )
    summary = {
        'schema_version': 'room315.runtime_acceptance_campaign.v4.v1',
        'candidate_id': candidate.state['candidate_id'],
        'checkpoint_sha256': candidate.state['checkpoint_sha256'],
        'candidate_state_sha256': candidate.state_sha256,
        'acceptance_scenarios_sha256': candidate.manifest_sha256,
        'observation_only': True,
        'task_execution_enabled': False,
        'execution_enabled': False,
        'gui': options.gui,
        'scenario_count': len(candidate.scenarios),
        'complete_event_count': accepted_event_count,
        'campaign_complete': campaign_complete,
        'acceptance_status': report['acceptance_status'],
        'automatic_deployment_approval': False,
        'acceptance_report': 'aggregate/acceptance_report.json',
        'acceptance_report_sha256': report_sha256,
        'scenario_results': scenario_results,
    }
    summary_path = output / 'campaign_summary.json'
    atomic_write_new(summary_path, summary)
    summary_path.chmod(stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
    _make_aggregate_immutable(output / 'aggregate')
    return summary


def positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(f'expected a number, got {value!r}') from exc
    if not parsed > 0.0:
        raise argparse.ArgumentTypeError(f'expected a positive number, got {value!r}')
    return parsed


def parse_args(argv: Sequence[str] | None = None) -> CampaignOptions:
    parser = argparse.ArgumentParser(
        description=(
            'Run all seven V4 Room 315 Gazebo runtime-acceptance scenarios '
            'sequentially without task execution.'
        ),
    )
    parser.add_argument('--candidate-directory', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument(
        '--gui', choices=('true', 'false'), default='false',
        help='Gazebo GUI; false by default.',
    )
    parser.add_argument(
        '--world-readiness-timeout-s', type=positive_float, default=45.0,
    )
    parser.add_argument(
        '--scene-readiness-timeout-s', type=positive_float, default=45.0,
    )
    parser.add_argument(
        '--camera-readiness-timeout-s', type=positive_float, default=45.0,
    )
    parser.add_argument(
        '--runtime-readiness-timeout-s', type=positive_float, default=120.0,
    )
    parser.add_argument(
        '--record-duration-s', type=positive_float, default=60.0,
    )
    parser.add_argument(
        '--process-timeout-margin-s', type=positive_float, default=30.0,
    )
    parser.add_argument(
        '--report-timeout-s', type=positive_float, default=30.0,
    )
    args = parser.parse_args(argv)
    return CampaignOptions(
        candidate_directory=args.candidate_directory,
        output_root=args.output_root,
        gui=args.gui == 'true',
        world_readiness_timeout_s=args.world_readiness_timeout_s,
        scene_readiness_timeout_s=args.scene_readiness_timeout_s,
        camera_readiness_timeout_s=args.camera_readiness_timeout_s,
        runtime_readiness_timeout_s=args.runtime_readiness_timeout_s,
        record_duration_s=args.record_duration_s,
        process_timeout_margin_s=args.process_timeout_margin_s,
        report_timeout_s=args.report_timeout_s,
    )


def main(argv: Sequence[str] | None = None) -> int:
    try:
        options = parse_args(argv)
        summary = run_campaign(options)
    except RuntimeAcceptanceCampaignError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['campaign_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
