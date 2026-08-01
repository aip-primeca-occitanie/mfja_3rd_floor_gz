#!/usr/bin/env python3
"""Run the guarded long-duration Room 315 V3R1 capture pipeline."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_text
from room_315_visual_v3r1_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3r1_common import PACKAGE_SCHEMA
from room_315_visual_v3r1_common import SEED
from room_315_visual_v3r1_common import quota_plan_path


STAGES = ('train', 'validation', 'canary', 'split', 'audit')
AUDIT_PREREQUISITES = ('train', 'validation', 'canary', 'split')


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _commands() -> dict[str, list[str]]:
    python = sys.executable
    return {
        'train': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3r1_capture.py'),
            'capture', '--profile', 'train', '--resume', '--keep-going',
        ],
        'validation': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3r1_capture.py'),
            'capture', '--profile', 'validation', '--resume', '--keep-going',
        ],
        'canary': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3r1_capture.py'),
            'capture', '--profile', 'canary', '--resume', '--keep-going',
        ],
        'split': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3r1_splitter.py'),
            '--resume',
        ],
        'audit': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3r1_audit.py'),
            '--mode', 'full',
        ],
    }


def _gate_paths(guard_root: Path) -> dict[str, Path]:
    return {
        'quota_plan': quota_plan_path(guard_root),
        'reuse': guard_root / 'v3_to_v3r1_reuse_audit.json',
        'static': guard_root / 'v3r1_static_audit.json',
        'family_overlap': (
            guard_root / 'static_family_overlap_audit.json'
        ),
        'smoke': guard_root / 'v3r1_position_smoke_report.json',
    }


def run_pipeline(guard_root: Path, *, start_at: str) -> dict[str, Any]:
    gates = _gate_paths(guard_root)
    for name, path in gates.items():
        if not path.is_file():
            raise VisualV3Error(f'missing V3R1 {name} gate: {path}')
        report = json.loads(path.read_text(encoding='utf-8'))
        if report.get('passed') is not True:
            raise VisualV3Error(f'V3R1 {name} gate did not pass')
    if start_at not in STAGES:
        raise VisualV3Error(f'unsupported start stage: {start_at}')
    pid_path = guard_root / 'full_pipeline.pid'
    if pid_path.is_file():
        try:
            existing = int(pid_path.read_text(encoding='utf-8').strip())
        except ValueError as exc:
            raise VisualV3Error('invalid V3R1 pipeline PID file') from exc
        if _pid_alive(existing) and existing != os.getpid():
            raise VisualV3Error(
                f'V3R1 pipeline already running as PID {existing}'
            )
    atomic_text(pid_path, f'{os.getpid()}\n')
    state_path = guard_root / 'full_pipeline_state.json'
    log_path = guard_root / 'full_pipeline.log'
    selected = STAGES[STAGES.index(start_at):]
    previous_elapsed = 0.0
    if start_at == 'audit':
        if not state_path.is_file():
            raise VisualV3Error(
                'audit-only continuation requires existing pipeline state'
            )
        state = json.loads(state_path.read_text(encoding='utf-8'))
        completed = tuple(state.get('completed_stages') or ())
        if completed != AUDIT_PREREQUISITES or any(
            int((state.get('stage_results', {}).get(stage) or {}).get(
                'returncode', -1
            )) != 0
            for stage in AUDIT_PREREQUISITES
        ):
            raise VisualV3Error(
                'audit-only continuation requires successful train, '
                'validation, canary, and split stages'
            )
        previous_elapsed = float(state.get('elapsed_seconds', 0.0))
        state.pop('failure', None)
        state.pop('failed_at', None)
        state['audit_only_resumed_at'] = _utc_now()
    else:
        state = {
            'schema_version': PACKAGE_SCHEMA,
            'seed': SEED,
            'started_at': _utc_now(),
            'selected_stages': list(selected),
            'completed_stages': [],
            'stage_results': {},
        }
    state.update({
        'pid': os.getpid(),
        'updated_at': _utc_now(),
        'status': 'running',
        'current_stage': None,
        'gates': {name: str(path) for name, path in gates.items()},
        'log': str(log_path),
    })
    atomic_json(state_path, state)
    started = time.monotonic()
    commands = _commands()
    with log_path.open('a', encoding='utf-8', buffering=1) as log:
        try:
            for stage in selected:
                state['current_stage'] = stage
                state['updated_at'] = _utc_now()
                atomic_json(state_path, state)
                stage_started = time.monotonic()
                command = commands[stage]
                log.write(f'[{_utc_now()}] $ {" ".join(command)}\n')
                completed = subprocess.run(
                    command,
                    check=False,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                state['stage_results'][stage] = {
                    'returncode': completed.returncode,
                    'elapsed_seconds': round(
                        time.monotonic() - stage_started, 3
                    ),
                }
                if completed.returncode != 0:
                    raise VisualV3Error(
                        f'pipeline stage {stage} failed with '
                        f'code {completed.returncode}'
                    )
                state['completed_stages'].append(stage)
                atomic_json(state_path, state)
            state['status'] = 'completed'
            state['current_stage'] = None
            state['completed_at'] = _utc_now()
            return state
        except BaseException as exc:
            state['status'] = 'failed'
            state['failure'] = f'{type(exc).__name__}: {exc}'
            state['failed_at'] = _utc_now()
            log.write(f'[{_utc_now()}] FAILED: {state["failure"]}\n')
            raise
        finally:
            state['elapsed_seconds'] = round(
                previous_elapsed + time.monotonic() - started, 3
            )
            state['updated_at'] = _utc_now()
            atomic_json(state_path, state)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument('--start-at', choices=STAGES, default='train')
    args = parser.parse_args(argv)
    print(json.dumps(
        run_pipeline(args.guard_root, start_at=args.start_at),
        indent=2,
        sort_keys=True,
    ))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
