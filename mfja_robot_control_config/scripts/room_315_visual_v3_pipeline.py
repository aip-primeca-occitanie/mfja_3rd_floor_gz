#!/usr/bin/env python3
"""Run the guarded long-duration Room 315 V3 capture pipeline."""

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

from room_315_visual_v3_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3_common import PACKAGE_SCHEMA
from room_315_visual_v3_common import SEED
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3_common import atomic_text


STAGES = ('train', 'validation', 'canary', 'split', 'audit')


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
            str(SCRIPT_DIR / 'room_315_visual_v3_capture.py'),
            'capture',
            '--profile',
            'train',
            '--resume',
            '--keep-going',
        ],
        'validation': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3_capture.py'),
            'capture',
            '--profile',
            'validation',
            '--resume',
            '--keep-going',
        ],
        'canary': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3_capture.py'),
            'capture',
            '--profile',
            'canary',
            '--resume',
            '--keep-going',
        ],
        'split': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3_splitter.py'),
            '--resume',
        ],
        'audit': [
            python,
            str(SCRIPT_DIR / 'room_315_visual_v3_audit.py'),
            '--mode',
            'full',
        ],
    }


def run_pipeline(guard_root: Path, *, start_at: str) -> dict[str, Any]:
    smoke_report_path = guard_root / 'smoke' / 'smoke_generation_report.json'
    if not smoke_report_path.is_file():
        raise VisualV3Error(f'missing smoke report: {smoke_report_path}')
    smoke_report = json.loads(smoke_report_path.read_text(encoding='utf-8'))
    if smoke_report.get('passed') is not True:
        raise VisualV3Error('full capture is prohibited until smoke audit passes')
    if start_at not in STAGES:
        raise VisualV3Error(f'unsupported start stage: {start_at}')

    pid_path = guard_root / 'full_pipeline.pid'
    if pid_path.is_file():
        try:
            existing_pid = int(pid_path.read_text(encoding='utf-8').strip())
        except ValueError as exc:
            raise VisualV3Error(f'invalid pipeline PID file: {pid_path}') from exc
        if _pid_alive(existing_pid) and existing_pid != os.getpid():
            raise VisualV3Error(
                f'full V3 pipeline is already running as PID {existing_pid}'
            )
    atomic_text(pid_path, f'{os.getpid()}\n')
    state_path = guard_root / 'full_pipeline_state.json'
    log_path = guard_root / 'full_pipeline.log'
    selected = STAGES[STAGES.index(start_at):]
    state: dict[str, Any] = {
        'schema_version': PACKAGE_SCHEMA,
        'seed': SEED,
        'pid': os.getpid(),
        'started_at': _utc_now(),
        'updated_at': _utc_now(),
        'status': 'running',
        'current_stage': None,
        'selected_stages': list(selected),
        'completed_stages': [],
        'stage_results': {},
        'smoke_report': str(smoke_report_path),
        'smoke_passed': True,
        'log': str(log_path),
    }
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
                    'elapsed_seconds': round(time.monotonic() - stage_started, 3),
                }
                if completed.returncode != 0:
                    raise VisualV3Error(
                        f'pipeline stage {stage} failed with code '
                        f'{completed.returncode}'
                    )
                state['completed_stages'].append(stage)
                state['updated_at'] = _utc_now()
                atomic_json(state_path, state)
            state['status'] = 'passed'
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
            state['elapsed_seconds'] = round(time.monotonic() - started, 3)
            state['updated_at'] = _utc_now()
            atomic_json(state_path, state)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument('--start-at', choices=STAGES, default='train')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = run_pipeline(args.guard_root, start_at=args.start_at)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
