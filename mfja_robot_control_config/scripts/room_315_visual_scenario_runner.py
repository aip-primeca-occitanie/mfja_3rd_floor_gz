#!/usr/bin/env python3
"""Run Room 315 visual scenarios and capture the new image-only dataset."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_scenario_generator import _read_manifest
from room_315_visual_state_dataset import pretty_json


REQUIRED_TOPICS = {
    '/mfja/conveyor/switch_cmd',
    '/room_315/vla/left_rail_rgbd/image',
    '/room_315/vla/right_rail_rgbd/image',
    '/room_315/rails/left/shuttles/state',
    '/room_315/rails/right/shuttles/state',
    '/room_315/rails/left/shuttles/payload_state',
    '/room_315/rails/right/shuttles/payload_state',
    '/room_315/rails/left/switches/state',
    '/room_315/rails/right/switches/state',
}


class VisualScenarioRunError(RuntimeError):
    """Raised when a Gazebo scenario does not capture successfully."""


def _launch_value(name: str, value: Any) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return ''
    if isinstance(value, str):
        if name.endswith('_start_slots'):
            return f"'{value}'"
        return value
    return str(value)


def launch_command(scenario: dict[str, Any], *, gui: bool) -> list[str]:
    setup = scenario['setup']
    command = [
        'ros2',
        'launch',
        setup['launch_package'],
        setup['launch_file'],
        f'gui:={str(bool(gui)).lower()}',
    ]
    command.extend(
        f'{name}:={_launch_value(name, value)}'
        for name, value in sorted(setup['launch_arguments'].items())
        if value not in (None, '')
    )
    return command


def switch_command(scenario: dict[str, Any]) -> list[str]:
    setup = scenario['setup']
    message = json.dumps({'data': setup['switch_command']}, separators=(',', ':'))
    return [
        'ros2',
        'topic',
        'pub',
        '--once',
        setup['switch_topic'],
        'std_msgs/msg/String',
        message,
    ]


def capture_command(
    scenario: dict[str, Any],
    *,
    manifest: Path,
    output_dataset: Path,
    timeout_seconds: float,
    max_camera_skew_seconds: float,
) -> list[str]:
    return [
        'ros2',
        'run',
        'mfja_robot_control_config',
        'room_315_visual_state_capture.py',
        '--scenario-manifest',
        str(manifest.expanduser().resolve()),
        '--scenario-id',
        scenario['scenario_id'],
        '--output-dataset',
        str(output_dataset.expanduser().resolve()),
        '--timeout-seconds',
        f'{timeout_seconds:.6g}',
        '--max-camera-skew-seconds',
        f'{max_camera_skew_seconds:.6g}',
    ]


def _terminate_process(process: subprocess.Popen[Any] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass


def _topic_names() -> set[str]:
    completed = subprocess.run(
        ['ros2', 'topic', 'list'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    if completed.returncode != 0:
        return set()
    return {line.strip() for line in completed.stdout.splitlines() if line.strip()}


def _wait_for_topics(
    process: subprocess.Popen[Any],
    *,
    timeout_seconds: float,
) -> None:
    deadline = time.monotonic() + timeout_seconds
    missing = set(REQUIRED_TOPICS)
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise VisualScenarioRunError(
                f'Gazebo launch exited before readiness with code {return_code}'
            )
        missing = REQUIRED_TOPICS - _topic_names()
        if not missing:
            return
        time.sleep(0.5)
    raise VisualScenarioRunError(
        f'Gazebo topics were not ready after {timeout_seconds:.1f}s: {sorted(missing)}'
    )


def _run_checked(command: list[str], *, timeout_seconds: float) -> subprocess.CompletedProcess:
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_seconds,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise VisualScenarioRunError(
            f'command failed with code {completed.returncode}: {command}\n{detail}'
        )
    return completed


def run_scenario(
    scenario: dict[str, Any],
    *,
    manifest: Path,
    output_dataset: Path,
    log_dir: Path,
    gui: bool,
    readiness_timeout_seconds: float,
    capture_timeout_seconds: float,
    max_camera_skew_seconds: float,
) -> dict[str, Any]:
    scenario_id = scenario['scenario_id']
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{scenario_id}.log'
    launch_process = None
    started_at = time.monotonic()
    with log_path.open('w', encoding='utf-8') as log:
        launch = launch_command(scenario, gui=gui)
        log.write('$ ' + ' '.join(launch) + '\n')
        log.flush()
        try:
            launch_process = subprocess.Popen(
                launch,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            _wait_for_topics(
                launch_process,
                timeout_seconds=readiness_timeout_seconds,
            )
            switch = switch_command(scenario)
            log.write('$ ' + ' '.join(switch) + '\n')
            log.flush()
            _run_checked(switch, timeout_seconds=10.0)
            time.sleep(float(scenario['capture']['settle_seconds']))
            capture = capture_command(
                scenario,
                manifest=manifest,
                output_dataset=output_dataset,
                timeout_seconds=capture_timeout_seconds,
                max_camera_skew_seconds=max_camera_skew_seconds,
            )
            log.write('$ ' + ' '.join(capture) + '\n')
            log.flush()
            completed = _run_checked(
                capture,
                timeout_seconds=capture_timeout_seconds + 10.0,
            )
            log.write(completed.stdout)
        finally:
            _terminate_process(launch_process)
    return {
        'scenario_id': scenario_id,
        'status': 'captured',
        'elapsed_seconds': round(time.monotonic() - started_at, 3),
        'log': str(log_path),
    }


def select_scenarios(
    scenarios: list[dict[str, Any]],
    *,
    scenario_ids: list[str],
    start: int,
    limit: int | None,
) -> list[dict[str, Any]]:
    selected = scenarios
    if scenario_ids:
        requested = set(scenario_ids)
        selected = [row for row in selected if row['scenario_id'] in requested]
        missing = sorted(requested - {row['scenario_id'] for row in selected})
        if missing:
            raise VisualScenarioRunError(f'unknown scenario ids: {missing}')
    if start < 0:
        raise VisualScenarioRunError('--start cannot be negative')
    selected = selected[start:]
    if limit is not None:
        if limit < 1:
            raise VisualScenarioRunError('--limit must be positive')
        selected = selected[:limit]
    if not selected:
        raise VisualScenarioRunError('scenario selection is empty')
    return selected


def _episode_exists(output_dataset: Path, scenario_id: str) -> bool:
    return (output_dataset.expanduser() / 'episodes' / scenario_id).is_dir()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Launch, configure, capture, and stop each new Room 315 '
            'visual-state scenario without the task/PDDL recorder.'
        )
    )
    parser.add_argument('--scenario-manifest', type=Path, required=True)
    parser.add_argument('--output-dataset', type=Path, required=True)
    parser.add_argument('--scenario-id', action='append', default=[])
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--limit', type=int)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--keep-going', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--readiness-timeout-seconds', type=float, default=45.0)
    parser.add_argument('--capture-timeout-seconds', type=float, default=30.0)
    parser.add_argument('--max-camera-skew-seconds', type=float, default=0.15)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    manifest = args.scenario_manifest.expanduser().resolve()
    scenarios = select_scenarios(
        _read_manifest(manifest),
        scenario_ids=args.scenario_id,
        start=args.start,
        limit=args.limit,
    )
    if args.dry_run:
        preview = [
            {
                'scenario_id': scenario['scenario_id'],
                'launch': launch_command(scenario, gui=args.gui),
                'switch': switch_command(scenario),
                'capture': capture_command(
                    scenario,
                    manifest=manifest,
                    output_dataset=args.output_dataset,
                    timeout_seconds=args.capture_timeout_seconds,
                    max_camera_skew_seconds=args.max_camera_skew_seconds,
                ),
            }
            for scenario in scenarios
        ]
        print(pretty_json(preview))
        return 0

    output_dataset = args.output_dataset.expanduser().resolve()
    results = []
    failures = []
    for scenario in scenarios:
        scenario_id = scenario['scenario_id']
        if args.resume and _episode_exists(output_dataset, scenario_id):
            results.append({'scenario_id': scenario_id, 'status': 'already_captured'})
            continue
        try:
            result = run_scenario(
                scenario,
                manifest=manifest,
                output_dataset=output_dataset,
                log_dir=output_dataset / 'logs',
                gui=args.gui,
                readiness_timeout_seconds=args.readiness_timeout_seconds,
                capture_timeout_seconds=args.capture_timeout_seconds,
                max_camera_skew_seconds=args.max_camera_skew_seconds,
            )
            results.append(result)
            print(f'captured {scenario_id}')
        except (OSError, subprocess.TimeoutExpired, VisualScenarioRunError) as exc:
            failure = {'scenario_id': scenario_id, 'error': str(exc)}
            failures.append(failure)
            print(f'failed {scenario_id}: {exc}', file=sys.stderr)
            if not args.keep_going:
                break
    summary = {
        'manifest': str(manifest),
        'output_dataset': str(output_dataset),
        'selected': len(scenarios),
        'captured': sum(row['status'] == 'captured' for row in results),
        'already_captured': sum(
            row['status'] == 'already_captured' for row in results
        ),
        'failures': failures,
        'results': results,
    }
    output_dataset.mkdir(parents=True, exist_ok=True)
    summary_path = output_dataset / 'meta' / 'last_scenario_run.json'
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(pretty_json(summary) + '\n', encoding='utf-8')
    print(pretty_json(summary))
    return 1 if failures else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualScenarioRunError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
