#!/usr/bin/env python3
"""Evaluate the offline Room 315 English task-goal corpus."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from statistics import mean

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_task_goal_builder import build_task_goal
from room_315_task_goal_semantic import default_config_path


DEFAULT_CORPUS = SCRIPT_DIR.parents[0] / 'config' / 'room_315_vla' / 'task_goal_english_benchmark.yaml'


def main() -> int:
    parser = argparse.ArgumentParser(description='Room 315 task-goal English benchmark.')
    parser.add_argument('--corpus', default=str(DEFAULT_CORPUS))
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    corpus_path = Path(args.corpus)
    payload = yaml.safe_load(corpus_path.read_text(encoding='utf-8')) or {}
    cases = payload.get('cases') or []
    rows = []
    latencies = []
    field_total = 0
    field_correct = 0
    exact_correct = 0
    ambiguity_expected = 0
    ambiguity_detected = 0
    fallback_count = 0
    disagreement_count = 0
    unsafe_auto_resolution_count = 0

    for case in cases:
        started = time.monotonic()
        result = build_task_goal(case['text'], timestamp=0.0)
        latency = time.monotonic() - started
        latencies.append(latency)
        got_constraints = result.task_goal.constraints if result.task_goal is not None else {}
        expected_constraints = case.get('expected_constraints') or {}
        exact = result.status == case.get('expected_status') and all(
            got_constraints.get(field) == value for field, value in expected_constraints.items()
        )
        exact_correct += int(exact)
        for field, expected in expected_constraints.items():
            field_total += 1
            field_correct += int(got_constraints.get(field) == expected)

        issue_codes = {issue.code for issue in result.errors + result.clarifications}
        expected_issue_codes = set(case.get('expected_issue_codes') or ())
        if expected_issue_codes:
            ambiguity_expected += int(any('ambiguous' in code or code.startswith('missing_') for code in expected_issue_codes))
            ambiguity_detected += int(bool(expected_issue_codes & issue_codes))
        trace = result.normalized_request.get('parse_trace') or {}
        if trace.get('fallback_reason'):
            fallback_count += 1
        if trace.get('parser_disagreements'):
            disagreement_count += 1
        if result.ok and expected_issue_codes:
            unsafe_auto_resolution_count += 1
        rows.append({
            'id': case['id'],
            'status': result.status,
            'expected_status': case.get('expected_status'),
            'exact': exact,
            'issues': sorted(issue_codes),
            'latency_s': latency,
            'fallback_reason': trace.get('fallback_reason', ''),
            'parser_disagreements': trace.get('parser_disagreements', []),
        })

    report = {
        'schema_version': payload.get('schema_version', 1),
        'benchmark_id': payload.get('benchmark_id', 'room315_task_goal_english'),
        'case_count': len(cases),
        'field_level_accuracy': (field_correct / field_total) if field_total else 0.0,
        'exact_task_goal_accuracy': (exact_correct / len(cases)) if cases else 0.0,
        'ambiguity_detection_recall': (ambiguity_detected / ambiguity_expected) if ambiguity_expected else 1.0,
        'mean_clarification_turns': 0.0,
        'correction_cancellation_accuracy': _correction_cancellation_accuracy(rows, cases),
        'parser_disagreement_rate': (disagreement_count / len(cases)) if cases else 0.0,
        'fallback_rate': (fallback_count / len(cases)) if cases else 0.0,
        'latency_p50_s': _percentile(latencies, 0.50),
        'latency_p95_s': _percentile(latencies, 0.95),
        'unsafe_automatic_resolution_count': unsafe_auto_resolution_count,
        'rows': rows,
        'config_path': str(default_config_path()),
    }
    output = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(output + '\n', encoding='utf-8')
    else:
        print(output)
    return 0 if unsafe_auto_resolution_count == 0 else 1


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * quantile)))
    return ordered[index]


def _correction_cancellation_accuracy(rows: list[dict], cases: list[dict]) -> float:
    selected = [
        (row, case) for row, case in zip(rows, cases)
        if case.get('expected_status') in {'cancelled'} or 'correction' in case.get('id', '')
    ]
    if not selected:
        return 1.0
    return mean(1.0 if row['status'] == case.get('expected_status') else 0.0 for row, case in selected)


if __name__ == '__main__':
    raise SystemExit(main())
