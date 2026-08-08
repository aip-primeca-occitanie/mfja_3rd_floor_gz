#!/usr/bin/env python3
"""Run and preserve the predeclared Room 315 language-contract evaluation."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml
import llama_cpp


def find_repository_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / 'mfja_robot_control_config').is_dir():
            return candidate
    raise RuntimeError('could not locate the repository root containing mfja_robot_control_config')


REPOSITORY_ROOT = find_repository_root(Path(__file__).resolve().parent)
SCRIPT_ROOT = REPOSITORY_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from room_315_task_goal_dialogue import TaskGoalDialogueManager
from room_315_task_goal_parsers import ConversationalIntentGatewayParser
from room_315_task_goal_parsers import ParserPipeline
from room_315_task_goal_semantic import LocalSemanticModelConfig
from room_315_task_goal_semantic import build_backend_from_config
from room_315_task_goal_validation import Room315DomainValidator


RUNTIME_SOURCE_FILES = (
    'room_315_task_goal_dialogue.py',
    'room_315_task_goal_fusion.py',
    'room_315_task_goal_parsers.py',
    'room_315_task_goal_schema.py',
    'room_315_task_goal_semantic.py',
    'room_315_task_goal_validation.py',
)


class CountingBackend:
    """Transparent backend proxy used to prove whether a case invoked inference."""

    def __init__(self, delegate: Any) -> None:
        self.delegate = delegate
        self.infer_calls = 0
        self.backend_name = getattr(delegate, 'backend_name', '')

    def health(self) -> Any:
        return self.delegate.health()

    def infer(self, user_text: str, *, confirmed_context: dict[str, Any] | None = None) -> Any:
        self.infer_calls += 1
        return self.delegate.infer(user_text, confirmed_context=confirmed_context)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def resolve_path(raw: str) -> Path:
    candidate = Path(raw).expanduser()
    return candidate if candidate.is_absolute() else REPOSITORY_ROOT / candidate


def require_hash(path: Path, expected: str, label: str) -> str:
    actual = sha256_file(path)
    if actual != expected:
        raise RuntimeError(f'{label} SHA-256 mismatch: expected {expected}, got {actual}')
    return actual


def issue_codes(payload: Any) -> set[str]:
    codes: set[str] = set()
    for issue in payload or ():
        if isinstance(issue, dict) and issue.get('code'):
            codes.add(str(issue['code']))
        elif getattr(issue, 'code', ''):
            codes.add(str(issue.code))
    return codes


def git_output(*args: str) -> str:
    return subprocess.run(
        ['git', *args], cwd=REPOSITORY_ROOT, check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    ).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    protocol_path = resolve_path(args.protocol)
    output_root = resolve_path(args.output)
    if output_root.exists():
        raise RuntimeError(f'output directory already exists: {output_root}')
    output_root.mkdir(parents=True)
    (output_root / 'cases').mkdir()
    (output_root / 'source_snapshot').mkdir()

    protocol = yaml.safe_load(protocol_path.read_text(encoding='utf-8')) or {}
    corpus_spec = protocol['corpus']
    config_spec = protocol['semantic_config']
    model_spec = protocol['model']
    source_spec = protocol['source_identity']

    corpus_path = resolve_path(corpus_spec['path'])
    config_path = resolve_path(config_spec['path'])
    model_path = resolve_path(model_spec['path'])
    source_record_path = resolve_path(source_spec['source_identity_record'])

    identities = {
        'protocol_sha256': sha256_file(protocol_path),
        'corpus_sha256': require_hash(corpus_path, corpus_spec['sha256'], 'corpus'),
        'config_sha256': require_hash(config_path, config_spec['sha256'], 'semantic config'),
        'model_sha256': require_hash(model_path, model_spec['sha256'], 'model'),
        'source_identity_record_sha256': require_hash(
            source_record_path,
            source_spec['source_identity_record_sha256'],
            'source identity record',
        ),
    }
    head = git_output('rev-parse', 'HEAD')
    if head != source_spec['git_head']:
        raise RuntimeError(f'Git HEAD mismatch: expected {source_spec["git_head"]}, got {head}')

    shutil.copy2(protocol_path, output_root / 'protocol.yaml')
    shutil.copy2(corpus_path, output_root / 'corpus.yaml')
    shutil.copy2(config_path, output_root / 'semantic_config.yaml')
    shutil.copy2(source_record_path, output_root / 'current_source_identity.txt')
    shutil.copy2(Path(__file__), output_root / 'runner.py')
    for filename in RUNTIME_SOURCE_FILES:
        shutil.copy2(SCRIPT_ROOT / filename, output_root / 'source_snapshot' / filename)

    corpus = yaml.safe_load(corpus_path.read_text(encoding='utf-8')) or {}
    cases = corpus.get('cases') or []
    if len(cases) != int(corpus_spec['case_count']):
        raise RuntimeError('predeclared case count does not match the corpus')

    config = LocalSemanticModelConfig.from_file(config_path)
    if config.backend != config_spec['required_backend']:
        raise RuntimeError(f'unexpected backend: {config.backend}')
    if bool(config.offline_only) is not bool(config_spec['require_offline']):
        raise RuntimeError('offline-only setting does not match the protocol')
    backend = build_backend_from_config(config)
    health = backend.health()
    health_payload = health.to_dict()
    if config_spec['require_ready'] and not health.ready:
        raise RuntimeError(f'semantic backend is not ready: {health_payload}')
    expected_fingerprint = f'sha256:{model_spec["sha256"]}'
    if health.model_fingerprint != expected_fingerprint:
        raise RuntimeError(
            f'model fingerprint mismatch: expected {expected_fingerprint}, '
            f'got {health.model_fingerprint}'
        )

    counted_backend = CountingBackend(backend)
    pipeline = ParserPipeline(semantic_backend=counted_backend, semantic_config=config)
    semantic_gateway = ConversationalIntentGatewayParser(
        backend=counted_backend,
        config=config,
    )
    validator = Room315DomainValidator()
    rows: list[dict[str, Any]] = []
    evaluation_started_at = datetime.now(timezone.utc).isoformat()
    started = time.monotonic()

    for index, case in enumerate(cases, start=1):
        case_started = time.monotonic()
        expected_status = str(case.get('expected_status') or '')
        expected_constraints = dict(case.get('expected_constraints') or {})
        expected_issues = set(case.get('expected_issue_codes') or ())
        is_control = case.get('expected_dialogue_act') == 'cancel'
        infer_calls_before = counted_backend.infer_calls

        if is_control:
            dialogue = TaskGoalDialogueManager(parser=pipeline, validator=validator)
            result = dialogue.handle(str(case['text']), timestamp=0.0)
            result_payload = result.to_dict()
            actual_status = result.status
            actual_constraints: dict[str, Any] = {}
            actual_issues = issue_codes(result_payload.get('errors')) | issue_codes(
                result_payload.get('clarifications')
            )
            trace: dict[str, Any] = {}
            semantic_requirement_met = counted_backend.infer_calls == infer_calls_before
            raw_payload = {'dialogue': result_payload, 'parse': None, 'validation': None}
        else:
            parsed = semantic_gateway.parse(str(case['text']))
            parse_payload = parsed.to_dict()
            trace = (
                dict(parsed.raw_output.get('trace') or {})
                if isinstance(parsed.raw_output, dict)
                else {}
            )
            if parsed.ok:
                validation = validator.validate(parsed.draft, timestamp=0.0)
                validation_payload = validation.to_dict()
                actual_status = validation.status
                actual_constraints = dict(validation.constraints or {})
                actual_issues = issue_codes(validation.errors) | issue_codes(validation.clarifications)
            else:
                validation_payload = None
                parse_issues = tuple(parsed.issues)
                hard_errors = tuple(
                    issue for issue in parse_issues
                    if not issue.code.startswith('missing_') and 'ambiguous' not in issue.code
                )
                actual_status = 'error' if hard_errors else 'clarification_required'
                actual_constraints = {}
                actual_issues = issue_codes(parse_issues)
            semantic_invocation_proved = bool(
                trace.get('model_ready')
                and trace.get('semantic_model_invoked')
                and trace.get('model_fingerprint') == expected_fingerprint
                and trace.get('model_backend') == config.backend
            )
            if expected_status == 'error':
                fallback_policy_met = (
                    not trace.get('fallback_used')
                    or trace.get('fallback_reason') == 'invalid_semantic_envelope'
                )
            else:
                fallback_policy_met = bool(
                    not trace.get('fallback_used') and not trace.get('fallback_reason')
                )
            semantic_requirement_met = semantic_invocation_proved and fallback_policy_met
            raw_payload = {
                'dialogue': None,
                'parse': parse_payload,
                'validation': validation_payload,
            }

        infer_call_delta = counted_backend.infer_calls - infer_calls_before
        if not is_control:
            semantic_requirement_met = semantic_requirement_met and infer_call_delta == 1

        constraints_match = all(
            actual_constraints.get(field) == expected
            for field, expected in expected_constraints.items()
        )
        issues_match = expected_issues.issubset(actual_issues)
        status_match = actual_status == expected_status
        passed = status_match and constraints_match and issues_match and semantic_requirement_met
        unsafe_auto_resolution = bool(expected_issues and actual_status == 'ok')

        row = {
            'index': index,
            'id': case['id'],
            'text': case['text'],
            'expected_status': expected_status,
            'actual_status': actual_status,
            'expected_constraints': expected_constraints,
            'actual_constraints': actual_constraints,
            'expected_issue_codes': sorted(expected_issues),
            'actual_issue_codes': sorted(actual_issues),
            'status_match': status_match,
            'constraints_match': constraints_match,
            'issues_match': issues_match,
            'semantic_invocation_required': not is_control,
            'semantic_requirement_met': semantic_requirement_met,
            'fallback_used': bool(trace.get('fallback_used')),
            'fallback_reason': str(trace.get('fallback_reason') or ''),
            'backend_infer_call_delta': infer_call_delta,
            'unsafe_auto_resolution': unsafe_auto_resolution,
            'elapsed_s': time.monotonic() - case_started,
            'passed': passed,
            'raw': raw_payload,
        }
        rows.append(row)
        write_json(output_root / 'cases' / f'{index:02d}_{case["id"]}.json', row)

    summary = {
        'schema_version': 1,
        'evaluation_id': protocol['evaluation_id'],
        'started_at_utc': evaluation_started_at,
        'finished_at_utc': datetime.now(timezone.utc).isoformat(),
        'elapsed_s': time.monotonic() - started,
        'case_count': len(rows),
        'passed_count': sum(int(row['passed']) for row in rows),
        'failed_count': sum(int(not row['passed']) for row in rows),
        'semantic_case_count': sum(int(row['semantic_invocation_required']) for row in rows),
        'semantic_requirement_met_count': sum(
            int(row['semantic_invocation_required'] and row['semantic_requirement_met'])
            for row in rows
        ),
        'strict_envelope_without_fallback_count': sum(
            int(row['semantic_invocation_required'] and not row['fallback_used'])
            for row in rows
        ),
        'expected_safe_schema_rejection_count': sum(
            int(
                row['expected_status'] == 'error'
                and row['actual_status'] == 'error'
                and row['fallback_reason'] == 'invalid_semantic_envelope'
            )
            for row in rows
        ),
        'control_case_count': sum(int(not row['semantic_invocation_required']) for row in rows),
        'control_bypass_count': sum(
            int(not row['semantic_invocation_required'] and row['backend_infer_call_delta'] == 0)
            for row in rows
        ),
        'supported_goal_count': sum(int(row['expected_status'] == 'ok') for row in rows),
        'supported_goal_passed_count': sum(
            int(row['expected_status'] == 'ok' and row['passed']) for row in rows
        ),
        'backend_infer_call_count': counted_backend.infer_calls,
        'unsafe_automatic_resolution_count': sum(int(row['unsafe_auto_resolution']) for row in rows),
        'all_acceptance_rules_passed': all(row['passed'] for row in rows),
        'backend_health': health_payload,
        'environment': {
            'python_version': sys.version,
            'platform': platform.platform(),
            'processor': platform.processor(),
            'cpu_count': os.cpu_count(),
            'llama_cpp_python_version': getattr(llama_cpp, '__version__', ''),
            'model_size_bytes': model_path.stat().st_size,
        },
        'identities': identities,
        'git_head': head,
        'recorded_source_tree_sha256': source_spec['source_tree_sha256'],
        'case_results': [
            {
                key: row[key]
                for key in (
                    'id', 'expected_status', 'actual_status', 'status_match',
                    'constraints_match', 'issues_match', 'semantic_requirement_met',
                    'backend_infer_call_delta', 'fallback_used', 'fallback_reason',
                    'unsafe_auto_resolution', 'elapsed_s', 'passed',
                )
            }
            for row in rows
        ],
        'claim_boundary': protocol.get('claim_boundary') or [],
    }
    write_json(output_root / 'summary.json', summary)

    files = sorted(path for path in output_root.rglob('*') if path.is_file())
    manifest = {
        'schema_version': 1,
        'evaluation_id': protocol['evaluation_id'],
        'files': [
            {
                'path': str(path.relative_to(output_root)),
                'size_bytes': path.stat().st_size,
                'sha256': sha256_file(path),
            }
            for path in files
        ],
    }
    write_json(output_root / 'manifest.json', manifest)
    checksum_paths = sorted(path for path in output_root.rglob('*') if path.is_file())
    (output_root / 'SHA256SUMS').write_text(
        ''.join(
            f'{sha256_file(path)}  {path.relative_to(output_root)}\n'
            for path in checksum_paths
        ),
        encoding='utf-8',
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['all_acceptance_rules_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
