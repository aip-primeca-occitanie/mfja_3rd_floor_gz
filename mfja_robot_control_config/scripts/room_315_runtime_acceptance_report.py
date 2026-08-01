#!/usr/bin/env python3
"""Build a non-approving Room 315 runtime acceptance report."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any


REQUIRED_COVERAGE = (
    'l4_loaded',
    'r4_loaded',
    'exact_l2_l4_r4',
    'right_slot3_deliberate_offset',
    'sparse_scene',
    'dense_scene',
    'multi_blocker_scene',
)
REQUIRED_RECORD_FIELDS = (
    'ground_truth',
    'raw_model_prediction',
    'decoded_observed_state',
    'presence_provider_result',
    'fusion_result',
    'validation_result',
    'safety_supervisor_decision',
    'execution_decision',
    'reobservation_and_effect_verification',
)


class AcceptanceReportError(RuntimeError):
    pass


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise AcceptanceReportError(f'cannot read JSON: {path}') from exc
    if not isinstance(value, dict):
        raise AcceptanceReportError(f'expected JSON object: {path}')
    return value


def validate_scenario_manifest(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if manifest.get('schema_version') != 'room315.runtime_acceptance_scenarios.v1':
        raise AcceptanceReportError('unsupported acceptance-scenario schema')
    rows = manifest.get('scenarios')
    if not isinstance(rows, list) or not rows:
        raise AcceptanceReportError('acceptance scenarios must be a non-empty list')
    by_id: dict[str, dict[str, Any]] = {}
    coverage: set[str] = set()
    for row in rows:
        if not isinstance(row, dict):
            raise AcceptanceReportError('each acceptance scenario must be an object')
        scenario_id = str(row.get('scenario_id') or '')
        if not scenario_id or scenario_id in by_id:
            raise AcceptanceReportError('acceptance scenario IDs must be unique')
        if not isinstance(row.get('ground_truth'), dict):
            raise AcceptanceReportError(f'{scenario_id}: ground_truth is required')
        tags = row.get('coverage') or []
        if not isinstance(tags, list):
            raise AcceptanceReportError(f'{scenario_id}: coverage must be a list')
        coverage.update(str(tag) for tag in tags)
        by_id[scenario_id] = row
    missing = set(REQUIRED_COVERAGE) - coverage
    if missing:
        raise AcceptanceReportError(
            f'acceptance manifest is missing coverage: {sorted(missing)}'
        )
    return rows


def empty_record(row: dict[str, Any]) -> dict[str, Any]:
    record: dict[str, Any] = {
        'scenario_id': row['scenario_id'],
        'coverage': list(row.get('coverage') or []),
        'ground_truth': row['ground_truth'],
    }
    for field in REQUIRED_RECORD_FIELDS[1:]:
        record[field] = {'status': 'not_run'}
    return record


def build_report(
    *,
    candidate_state: dict[str, Any],
    manifest: dict[str, Any],
    event_records: list[dict[str, Any]],
) -> dict[str, Any]:
    scenarios = validate_scenario_manifest(manifest)
    expected = {row['scenario_id']: row for row in scenarios}
    events: dict[str, dict[str, Any]] = {}
    for event in event_records:
        scenario_id = str(event.get('scenario_id') or '')
        if scenario_id not in expected:
            raise AcceptanceReportError(f'unknown event scenario: {scenario_id!r}')
        if scenario_id in events:
            raise AcceptanceReportError(f'duplicate event scenario: {scenario_id}')
        missing = [key for key in REQUIRED_RECORD_FIELDS if key not in event]
        if missing:
            raise AcceptanceReportError(
                f'{scenario_id}: event is missing fields {missing}'
            )
        if event['ground_truth'] != expected[scenario_id]['ground_truth']:
            raise AcceptanceReportError(
                f'{scenario_id}: recorded ground truth differs from manifest'
            )
        if event.get('record_status') not in {'complete', 'failed'}:
            raise AcceptanceReportError(
                f'{scenario_id}: record_status must be complete or failed'
            )
        events[scenario_id] = event

    records = [events.get(row['scenario_id'], empty_record(row)) for row in scenarios]
    completed = sum(
        record.get('record_status') == 'complete'
        and all(
            isinstance(record.get(field), dict)
            and record[field].get('status') not in {
                'not_run', 'not_observed', 'invalid_json', 'invalid_type'
            }
            for field in REQUIRED_RECORD_FIELDS[1:]
        )
        for record in records
    )
    failed = sum(record.get('record_status') == 'failed' for record in records)
    return {
        'schema_version': 'room315.runtime_acceptance_report.v1',
        'candidate_id': candidate_state.get('candidate_id'),
        'checkpoint_sha256': candidate_state.get('checkpoint_sha256'),
        'deployment_state': 'candidate',
        'automatic_deployment_approval': False,
        'acceptance_status': (
            'complete_pending_human_decision'
            if completed == len(records)
            else 'not_run' if not events else 'incomplete'
        ),
        'scenario_count': len(records),
        'complete_scenario_count': completed,
        'failed_scenario_count': failed,
        'records': records,
        'approval': {
            'approved': False,
            'approved_by': None,
            'approved_at': None,
            'note': 'This generator never approves a deployment candidate.',
        },
    }


def atomic_write_new(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise AcceptanceReportError(f'refusing to overwrite existing report: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.tmp-{os.getpid()}')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--candidate-directory', type=Path, required=True)
    parser.add_argument('--event-directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()

    candidate = args.candidate_directory.resolve()
    candidate_state = load_json(candidate / 'candidate_state.json')
    manifest = load_json(candidate / 'acceptance_scenarios.json')
    records: list[dict[str, Any]] = []
    if args.event_directory:
        for path in sorted(args.event_directory.resolve().glob('*.json')):
            records.append(load_json(path))
    report = build_report(
        candidate_state=candidate_state,
        manifest=manifest,
        event_records=records,
    )
    atomic_write_new(args.output.resolve(), report)
    print(json.dumps({
        'status': report['acceptance_status'],
        'deployment_state': report['deployment_state'],
        'automatic_deployment_approval': False,
        'output': str(args.output.resolve()),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
