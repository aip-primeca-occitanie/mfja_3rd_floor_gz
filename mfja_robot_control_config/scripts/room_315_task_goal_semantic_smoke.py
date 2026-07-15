#!/usr/bin/env python3
"""Offline smoke checks for Room 315 task-goal semantic understanding."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_task_goal_parsers import ParserPipeline
from room_315_task_goal_semantic import LocalSemanticModelConfig
from room_315_task_goal_semantic import build_backend_from_config
from room_315_task_goal_validation import Room315DomainValidator


def main() -> int:
    parser = argparse.ArgumentParser(description='Room 315 offline task-goal semantic smoke test.')
    parser.add_argument(
        '--config',
        default='',
        help='Versioned task-goal understanding YAML. Defaults to package config.',
    )
    parser.add_argument(
        '--text',
        default='please move the nearest loaded right shuttle to slot 3',
        help='English request to parse.',
    )
    parser.add_argument(
        '--require-real-model',
        action='store_true',
        help='Fail unless the configured local semantic model is loaded and ready.',
    )
    parser.add_argument(
        '--expect-semantic',
        action='store_true',
        help='Fail unless trace proves a ready semantic backend was invoked without fallback.',
    )
    parser.add_argument(
        '--expect-draft-field',
        action='append',
        default=[],
        metavar='FIELD=VALUE',
        help='Require a parsed TaskGoalDraft field value; may be repeated.',
    )
    parser.add_argument(
        '--shadow-mode',
        action='store_true',
        help='Run semantic result in shadow mode; deterministic evidence remains authoritative.',
    )
    args = parser.parse_args()

    try:
        config = LocalSemanticModelConfig.from_file(args.config or None)
        if args.shadow_mode:
            config = LocalSemanticModelConfig(**{**config.__dict__, 'shadow_mode': True})
        backend = build_backend_from_config(config)
        health = backend.health()
        if args.require_real_model and not health.ready:
            print(json.dumps({
                'status': 'error',
                'reason': 'real_local_model_not_ready',
                'health': health.to_dict(),
            }, indent=2, sort_keys=True))
            return 2

        pipeline = ParserPipeline(semantic_backend=backend, semantic_config=config)
        parsed = pipeline.parse(args.text)
        trace = parsed.raw_output.get('trace', {}) if isinstance(parsed.raw_output, dict) else {}
        validation_payload = None
        if parsed.ok:
            validation = Room315DomainValidator().validate(parsed.draft, timestamp=0.0)
            validation_payload = validation.to_dict()
        else:
            validation_payload = {'status': 'parse_error', 'issues': [issue.to_dict() for issue in parsed.issues]}

        semantic_failed = (
            not trace.get('model_ready')
            or not trace.get('semantic_model_invoked')
            or bool(trace.get('fallback_used'))
            or bool(trace.get('fallback_reason'))
        )
        if args.expect_semantic and semantic_failed:
            print(json.dumps({
                'status': 'error',
                'reason': 'semantic_backend_not_used_successfully',
                'health': health.to_dict(),
                'parse': parsed.to_dict(),
                'validation': validation_payload,
            }, indent=2, sort_keys=True))
            return 3

        draft_payload = parsed.draft.to_dict() if parsed.ok and parsed.draft else {}
        expected_failures = []
        for raw_expectation in args.expect_draft_field:
            if '=' not in raw_expectation:
                expected_failures.append({'expectation': raw_expectation, 'reason': 'missing_equals'})
                continue
            field, expected = raw_expectation.split('=', 1)
            actual = draft_payload.get(field)
            if str(actual) != expected:
                expected_failures.append({'field': field, 'expected': expected, 'actual': actual})
        if expected_failures:
            print(json.dumps({
                'status': 'error',
                'reason': 'draft_expectation_failed',
                'failures': expected_failures,
                'health': health.to_dict(),
                'parse': parsed.to_dict(),
                'validation': validation_payload,
            }, indent=2, sort_keys=True))
            return 4

        ok = parsed.ok and validation_payload.get('status') in {'ok', 'clarification_required', 'confirmation_required'}
        print(json.dumps({
            'status': 'ok' if ok else 'error',
            'health': health.to_dict(),
            'parse': parsed.to_dict(),
            'validation': validation_payload,
        }, indent=2, sort_keys=True))
        return 0 if ok else 1
    except Exception as exc:  # pragma: no cover - CLI defensive boundary
        print(json.dumps({'status': 'error', 'reason': str(exc)}, indent=2, sort_keys=True))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
