#!/usr/bin/env python3
"""Run Room 315 payload training cases end-to-end."""

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

from room_315_pddl_scenario_generator import DEFAULT_PAYLOAD_TRAINING_CASES_PATH
from room_315_pddl_scenario_generator import load_payload_training_case_config


DEFAULT_DATASET_DIR = Path('~/room315_payload_all_cases')
DEFAULT_RESULTS_DIR = Path('/tmp/room315_payload_case_batch')
ROOM315_PROCESS_PATTERNS = (
    'room_315_only.launch.py',
    'room_315_kinematic_shuttle_node.py',
    'room_315_vla_supervisor.py',
    'room_315_vla_dataset_recorder.py',
    'room315_vla_camera_bridge',
    'room315_world_service_bridge',
    'conveyor_loop_mode_controller.py.*room_315_only',
    'parameter_bridge /clock@rosgraph_msgs/msg/Clock',
    'parameter_bridge.*room_315_only',
    'gz sim.*room_315_only',
)


def _json_dumps(data: Any, *, indent: int | None = None) -> str:
    return json.dumps(data, ensure_ascii=False, indent=indent, sort_keys=True)


def _as_bool_arg(value: str) -> bool:
    text = str(value).strip().casefold()
    if text in {'1', 'true', 'yes', 'on'}:
        return True
    if text in {'0', 'false', 'no', 'off'}:
        return False
    raise argparse.ArgumentTypeError(f'expected true/false, got {value!r}')


def _launch_yaml_string(value: Any) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def _case_items(config: dict[str, Any], requested_ids: list[str]) -> list[dict[str, Any]]:
    cases = config.get('cases', [])
    if not isinstance(cases, list):
        raise ValueError('payload training case config needs a cases list')
    by_id = {
        str(case.get('case_id') or '').strip(): dict(case)
        for case in cases
        if isinstance(case, dict)
    }
    by_id.pop('', None)
    if not requested_ids:
        return [dict(case) for case in cases if isinstance(case, dict) and case.get('case_id')]

    unknown = [case_id for case_id in requested_ids if case_id not in by_id]
    if unknown:
        allowed = ', '.join(sorted(by_id))
        raise ValueError(f'unknown case id(s): {", ".join(unknown)}; allowed: {allowed}')
    return [dict(by_id[case_id]) for case_id in requested_ids]


def _selected_cases(
    cases: list[dict[str, Any]],
    *,
    start_at: int,
    stop_after: int | None,
) -> list[tuple[int, dict[str, Any]]]:
    numbered = list(enumerate(cases, start=1))
    selected = [(number, case) for number, case in numbered if number >= int(start_at)]
    if stop_after is not None:
        selected = [(number, case) for number, case in selected if number <= int(stop_after)]
    return selected


def _case_launch_args(case: dict[str, Any], *, dataset_dir: Path, gui: bool) -> list[str]:
    launch = dict(case.get('launch') or {})
    side = str(case.get('side') or 'right').strip().casefold()
    disable_opposite = _launch_bool(launch.get('disable_opposite_rail'), default=True)
    explicit_right = any(str(key).startswith('right_') for key in launch)
    explicit_left = any(str(key).startswith('left_') for key in launch)
    enable_right = _launch_bool(
        launch.get('enable_right'),
        default=(side == 'right' or explicit_right or not disable_opposite),
    )
    enable_left = _launch_bool(
        launch.get('enable_left'),
        default=(side == 'left' or explicit_left or not disable_opposite),
    )
    right_count = launch.get('right_shuttle_count', 2 if side == 'right' and enable_right else 0)
    right_start_slots = launch.get('right_start_slots', '1,2' if side == 'right' else '')
    right_loaded = launch.get('right_loaded_shuttles', '')
    left_count = launch.get('left_shuttle_count', 2 if side == 'left' and enable_left else 0)
    left_start_slots = launch.get('left_start_slots', '1,2' if side == 'left' else '')
    left_loaded = launch.get('left_loaded_shuttles', '')
    return [
        'ros2',
        'launch',
        'mfja_3rd_floor_bringup',
        'room_315_only.launch.py',
        'robots:=none',
        'start_paused:=false',
        f'gui:={str(bool(gui)).lower()}',
        'enable_room315_kinematic_shuttles:=true',
        'enable_room315_vla:=true',
        'enable_room315_vla_dataset_recorder:=true',
        'room315_enable_payload_visuals:=true',
        f'enable_room315_right_rail:={str(bool(enable_right)).lower()}',
        f'enable_room315_left_rail:={str(bool(enable_left)).lower()}',
        f'room315_right_shuttle_count:={right_count}',
        f'room315_right_start_slots:={_launch_yaml_string(right_start_slots)}',
        f'room315_right_loaded_shuttles:={_launch_yaml_string(right_loaded)}',
        f'room315_left_shuttle_count:={left_count}',
        f'room315_left_start_slots:={_launch_yaml_string(left_start_slots)}',
        f'room315_left_loaded_shuttles:={_launch_yaml_string(left_loaded)}',
        'room315_shuttles_start_enabled:=false',
        'room315_visual_debug_colors:=false',
        'room315_show_device_markers:=false',
        f'room315_vla_dataset_dir:={dataset_dir.expanduser()}',
    ]


def _launch_bool(value: Any, *, default: bool) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    return str(value).strip().casefold() in {'1', 'true', 'yes', 'on'}


def _scenario_generator_cmd(
    *,
    case_id: str,
    case_config: Path,
    mode: str,
    output_path: Path | None = None,
    command_timeout_s: float,
    arrival_timeout_s: float,
    speed: float,
) -> list[str]:
    cmd = [
        'ros2',
        'run',
        'mfja_robot_control_config',
        'room_315_pddl_scenario_generator.py',
        '--case-id',
        case_id,
        '--case-config',
        str(case_config),
        '--planner-backend',
        'plansys',
        '--planner-service',
        '/planner/get_plan',
        '--language-template-id',
        'loaded_shuttle_to_slot',
        '--command-timeout-s',
        f'{float(command_timeout_s):.4g}',
        '--speed',
        f'{float(speed):.4g}',
    ]
    if mode == 'preflight':
        cmd.extend(['--preflight-only', '--ready-line'])
        return cmd
    if mode == 'execute':
        cmd.extend([
            '--arrival-timeout-s',
            f'{float(arrival_timeout_s):.4g}',
            '--quiet',
            '--execute',
        ])
        if output_path is not None:
            cmd.extend(['--output', str(output_path)])
        return cmd
    raise ValueError(f'unknown scenario generator mode {mode!r}')


def _run_text_command(
    cmd: list[str],
    *,
    log_path: Path | None = None,
    timeout_s: float | None = None,
) -> subprocess.CompletedProcess:
    try:
        completed = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        completed = subprocess.CompletedProcess(
            cmd,
            124,
            stdout=exc.stdout if isinstance(exc.stdout, str) else '',
            stderr=(
                (exc.stderr if isinstance(exc.stderr, str) else '')
                + f'\ncommand timed out after {float(timeout_s or 0.0):.1f}s'
            ),
        )
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(
            '$ ' + ' '.join(cmd) + '\n'
            + completed.stdout
            + completed.stderr,
            encoding='utf-8',
        )
    return completed


def _stop_room315_processes() -> None:
    for pattern in ROOM315_PROCESS_PATTERNS:
        subprocess.run(['pkill', '-f', pattern], check=False)
    subprocess.run(['ros2', 'daemon', 'stop'], check=False)
    subprocess.run(['ros2', 'daemon', 'start'], check=False)


def _terminate_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)


def _execution_success(output_path: Path) -> tuple[bool, str]:
    if not output_path.exists():
        return False, 'execute output JSON was not written'
    try:
        scenario = json.loads(output_path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        return False, f'execute output JSON is invalid: {exc}'
    execution = scenario.get('execution')
    if not isinstance(execution, dict):
        return False, 'execute output JSON is missing execution result'
    if bool(execution.get('success', False)):
        return True, ''
    return False, str(execution.get('failure_reason') or 'execution failed')


def _launch_case(
    case: dict[str, Any],
    *,
    case_number: int,
    dataset_dir: Path,
    results_dir: Path,
    gui: bool,
) -> tuple[subprocess.Popen, Path]:
    launch_log = results_dir / f'{case_number:02d}_{case["case_id"]}_launch.log'
    launch_log.parent.mkdir(parents=True, exist_ok=True)
    log_stream = launch_log.open('w', encoding='utf-8')
    cmd = _case_launch_args(case, dataset_dir=dataset_dir, gui=gui)
    log_stream.write('$ ' + ' '.join(cmd) + '\n')
    log_stream.flush()
    process = subprocess.Popen(
        cmd,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    process._room315_log_stream = log_stream  # type: ignore[attr-defined]
    return process, launch_log


def _close_launch_log(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    stream = getattr(process, '_room315_log_stream', None)
    if stream is not None:
        stream.close()


def _launch_log_failure_reason(launch_log: Path) -> str:
    if not launch_log.exists():
        return ''
    text = launch_log.read_text(encoding='utf-8', errors='replace')
    if 'InvalidParameterTypeException' in text:
        for line in reversed(text.splitlines()):
            if 'InvalidParameterTypeException' in line or 'Trying to set parameter' in line:
                return f'Room 315 launch parameter error: {line.strip()}'
        return 'Room 315 launch parameter error'
    if 'room_315_kinematic_shuttle_node.py' in text and 'process has died' in text:
        for line in reversed(text.splitlines()):
            if 'room_315_kinematic_shuttle_node.py' in line and 'process has died' in line:
                return f'kinematic shuttle node crashed: {line.strip()}'
        return 'kinematic shuttle node crashed'
    return ''


def run_cases(args: argparse.Namespace) -> dict[str, Any]:
    config = load_payload_training_case_config(args.case_config)
    case_config_path = Path(config.get('case_config_path') or args.case_config)
    all_cases = _case_items(config, args.case_id)
    selected = _selected_cases(
        all_cases,
        start_at=args.start_at,
        stop_after=args.stop_after,
    )
    dataset_dir = args.dataset_dir.expanduser()
    results_dir = args.results_dir.expanduser()
    results_dir.mkdir(parents=True, exist_ok=True)

    if args.reset_dataset:
        if dataset_dir.exists() and dataset_dir.name.startswith('room315_payload_'):
            import shutil

            shutil.rmtree(dataset_dir)
        dataset_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        'case_config_path': str(case_config_path),
        'dataset_dir': str(dataset_dir),
        'results_dir': str(results_dir),
        'case_count': len(selected),
        'results': [],
    }
    if args.dry_run:
        summary['dry_run'] = True
        for case_number, case in selected:
            summary['results'].append({
                'case_number': case_number,
                'case_id': case['case_id'],
                'title': case.get('title', ''),
                'launch_args': _case_launch_args(case, dataset_dir=dataset_dir, gui=args.gui),
                'success': None,
            })
        return _finalize_summary(summary, args)

    for ordinal, (case_number, case) in enumerate(selected, start=1):
        case_id = str(case.get('case_id') or '').strip()
        case_speed = float(case.get('speed', case.get('speed_mps', args.speed)) or args.speed)
        result: dict[str, Any] = {
            'case_number': case_number,
            'run_index': ordinal,
            'case_id': case_id,
            'title': str(case.get('title') or ''),
            'success': False,
        }
        launch_process: subprocess.Popen | None = None
        print(f'\n=== Case {case_number}: {case_id} ({ordinal}/{len(selected)}) ===', flush=True)
        try:
            print('Stopping old Room 315 processes...', flush=True)
            _stop_room315_processes()
            launch_process, launch_log = _launch_case(
                case,
                case_number=case_number,
                dataset_dir=dataset_dir,
                results_dir=results_dir,
                gui=args.gui,
            )
            result['launch_log'] = str(launch_log)
            print(f'Launching Room 315; log: {launch_log}', flush=True)
            time.sleep(max(float(args.launch_wait_s), 0.0))

            if launch_process.poll() is not None:
                result['failure_reason'] = f'Room 315 launch exited early with code {launch_process.returncode}'
                print(f'FAILED case {case_number}: {result["failure_reason"]}', flush=True)
                summary['results'].append(result)
                continue
            launch_failure = _launch_log_failure_reason(launch_log)
            if launch_failure:
                result['failure_reason'] = launch_failure
                print(f'FAILED case {case_number}: {launch_failure}', flush=True)
                summary['results'].append(result)
                continue

            preflight_log = results_dir / f'{case_number:02d}_{case_id}_preflight.log'
            print('Checking initial shuttle/payload state...', flush=True)
            preflight = _run_text_command(
                _scenario_generator_cmd(
                    case_id=case_id,
                    case_config=case_config_path,
                    mode='preflight',
                    command_timeout_s=args.preflight_timeout_s,
                    arrival_timeout_s=args.arrival_timeout_s,
                    speed=case_speed,
                ),
                log_path=preflight_log,
                timeout_s=max(float(args.preflight_timeout_s) + 30.0, 30.0),
            )
            result['preflight_log'] = str(preflight_log)
            result['preflight_stdout'] = preflight.stdout.strip()
            print(preflight.stdout.strip() or preflight.stderr.strip(), flush=True)
            if preflight.returncode != 0:
                result['failure_reason'] = (
                    preflight.stderr.strip()
                    or preflight.stdout.strip()
                    or f'preflight failed with code {preflight.returncode}'
                )
                print(f'FAILED case {case_number}: {result["failure_reason"]}', flush=True)
                summary['results'].append(result)
                continue

            output_path = results_dir / f'{case_number:02d}_{case_id}_execute.json'
            execute_log = results_dir / f'{case_number:02d}_{case_id}_execute.log'
            print('Executing scenario in Gazebo...', flush=True)
            execute = _run_text_command(
                _scenario_generator_cmd(
                    case_id=case_id,
                    case_config=case_config_path,
                    mode='execute',
                    output_path=output_path,
                    command_timeout_s=args.command_timeout_s,
                    arrival_timeout_s=args.arrival_timeout_s,
                    speed=case_speed,
                ),
                log_path=execute_log,
                timeout_s=max(
                    float(args.arrival_timeout_s) + float(args.command_timeout_s) + 60.0,
                    60.0,
                ),
            )
            result['execute_log'] = str(execute_log)
            result['execute_json'] = str(output_path)
            if execute.returncode != 0:
                result['failure_reason'] = (
                    execute.stderr.strip()
                    or execute.stdout.strip()
                    or f'execute failed with code {execute.returncode}'
                )
                print(f'FAILED case {case_number}: {result["failure_reason"]}', flush=True)
                summary['results'].append(result)
                continue
            success, reason = _execution_success(output_path)
            result['success'] = success
            result['failure_reason'] = reason
            print(
                f'{"OK" if success else "FAILED"} case {case_number}: {case_id}'
                + (f' - {reason}' if reason else ''),
                flush=True,
            )
            summary['results'].append(result)
        finally:
            keep_this_launch = bool(args.keep_last_launch and ordinal == len(selected))
            if not keep_this_launch:
                _terminate_process(launch_process)
                _close_launch_log(launch_process)
                _stop_room315_processes()
            else:
                result['launch_kept_running'] = True

    return _finalize_summary(summary, args)


def _finalize_summary(summary: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    results = list(summary.get('results') or [])
    successes = sum(1 for result in results if result.get('success') is True)
    failures = [result for result in results if result.get('success') is False]
    summary.update({
        'successes': successes,
        'failures': len(failures),
        'failed_cases': [
            {
                'case_number': result.get('case_number'),
                'case_id': result.get('case_id'),
                'failure_reason': result.get('failure_reason', ''),
            }
            for result in failures
        ],
    })
    results_dir = Path(summary['results_dir'])
    summary_path = results_dir / 'payload_case_batch_summary.json'
    summary_path.write_text(_json_dumps(summary, indent=2) + '\n', encoding='utf-8')
    summary['summary_path'] = str(summary_path)

    if not args.dry_run and not args.skip_export:
        export_log = results_dir / 'training_events_export.log'
        export = _run_text_command(
            [
                'ros2',
                'run',
                'mfja_robot_control_config',
                'room_315_vla_event_extractor.py',
                str(Path(summary['dataset_dir'])),
                '--output',
                'meta/training_events.jsonl',
            ],
            log_path=export_log,
        )
        summary['training_events_export_log'] = str(export_log)
        summary['training_events_export_returncode'] = export.returncode
        summary['training_events_jsonl'] = str(
            Path(summary['dataset_dir']) / 'meta' / 'training_events.jsonl'
        )
        if export.returncode != 0:
            summary['export_failure_reason'] = export.stderr.strip() or export.stdout.strip()
        summary_path.write_text(_json_dumps(summary, indent=2) + '\n', encoding='utf-8')

    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Run every Room 315 payload case sequentially and record the dataset.'
    )
    parser.add_argument('--case-config', type=Path, default=DEFAULT_PAYLOAD_TRAINING_CASES_PATH)
    parser.add_argument('--case-id', action='append', default=[], help='Run only this case id. Repeatable.')
    parser.add_argument('--start-at', type=int, default=1, help='First 1-based case number to run.')
    parser.add_argument('--stop-after', type=int, default=None, help='Last 1-based case number to run.')
    parser.add_argument('--dataset-dir', type=Path, default=DEFAULT_DATASET_DIR)
    parser.add_argument('--results-dir', type=Path, default=DEFAULT_RESULTS_DIR)
    parser.add_argument('--reset-dataset', action='store_true')
    parser.add_argument('--gui', type=_as_bool_arg, default=True)
    parser.add_argument('--launch-wait-s', type=float, default=12.0)
    parser.add_argument('--preflight-timeout-s', type=float, default=60.0)
    parser.add_argument('--command-timeout-s', type=float, default=30.0)
    parser.add_argument('--arrival-timeout-s', type=float, default=120.0)
    parser.add_argument('--speed', type=float, default=0.3, help='Default move_shuttle speed for cases without speed.')
    parser.add_argument('--keep-last-launch', action='store_true')
    parser.add_argument('--skip-export', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    summary = run_cases(args)
    print(_json_dumps(summary, indent=2))
    return 1 if summary.get('failures') else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
