#!/usr/bin/env python3
"""Create a human review pack for Room 315 payload training cases."""

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_pddl_scenario_generator import DEFAULT_PAYLOAD_TRAINING_CASES_PATH
from room_315_pddl_scenario_generator import command_payloads_for_execution
from room_315_pddl_scenario_generator import generate_scenario
from room_315_pddl_scenario_generator import load_payload_training_case_config


DEFAULT_REVIEW_DIR = Path('/tmp/room315_payload_case_review')


class ReviewPlanner:
    """Small offline planner used only for review pack generation."""

    def plan(self, spec: Any, *, speed: float) -> list[str]:
        speed_text = f'{float(speed):.4g}'
        return [
            f'prepare_switches {spec.side} {spec.source} {spec.target}',
            f'open_stoppers {spec.side} {spec.source} {spec.target}',
            (
                f'move_shuttle {spec.side} {spec.shuttle} '
                f'{spec.source} {spec.target} speed={speed_text}'
            ),
            f'stop_shuttle {spec.side} {spec.shuttle}',
            f'finish_task {spec.shuttle} {spec.target}',
        ]


def _json_dumps(data: Any, *, indent: int | None = None) -> str:
    return json.dumps(data, ensure_ascii=False, indent=indent, sort_keys=True)


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


def _selected_candidate(scenario: dict[str, Any]) -> dict[str, Any]:
    for candidate in scenario.get('selection_candidates') or []:
        if isinstance(candidate, dict) and bool(candidate.get('selected', False)):
            return dict(candidate)
    return {}


def _assert_model_input_clean(scenario: dict[str, Any], payloads: list[dict[str, Any]]) -> None:
    if 'model_input' in scenario:
        raise RuntimeError(f'{scenario.get("scenario_id", "")}: scenario contains model_input')
    for index, payload in enumerate(payloads):
        if 'model_input' in payload:
            raise RuntimeError(
                f'{scenario.get("scenario_id", "")}: command {index} contains model_input'
            )


def _launch_yaml_string(value: Any) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def _shell_launch_yaml_string(value: Any) -> str:
    return '"' + _launch_yaml_string(value).replace('"', '\\"') + '"'


def _review_record(
    *,
    case: dict[str, Any],
    scenario: dict[str, Any],
    payloads: list[dict[str, Any]],
    scenario_path: Path,
) -> dict[str, Any]:
    blocker = scenario.get('blocker_clearance', {})
    if not isinstance(blocker, dict):
        blocker = {}
    selected = _selected_candidate(scenario)
    move_commands = [
        payload
        for payload in payloads
        if payload.get('action') == 'shuttle' and payload.get('command') == 'ON'
    ]
    return {
        'case_id': str(case.get('case_id') or ''),
        'title': str(case.get('title') or ''),
        'language': str(scenario.get('language') or ''),
        'launch': dict(case.get('launch') or {}),
        'target_shuttle_id': str(scenario.get('target_shuttle_id') or ''),
        'target_slot': str(scenario.get('target_slot') or ''),
        'payload_condition': str(scenario.get('payload_condition') or ''),
        'selected_candidate': selected,
        'blocker_shuttle_id': str(blocker.get('blocker_shuttle_id') or ''),
        'blocker_start_slot': str(blocker.get('blocker_start_slot') or ''),
        'blocker_clear_slot': str(blocker.get('blocker_clear_slot') or ''),
        'blocker_clear_target': str(blocker.get('blocker_clear_target') or ''),
        'blocker_final_slot': str(blocker.get('blocker_final_slot') or ''),
        'blocker_final_target': str(blocker.get('blocker_final_target') or ''),
        'clearance_step_count': int(blocker.get('clearance_step_count') or 0),
        'clearance_steps': list(blocker.get('clearance_steps') or []),
        'symbolic_plan': list(scenario.get('symbolic_plan') or []),
        'move_commands': move_commands,
        'scenario_json': str(scenario_path),
        'model_input_boundary_clean': True,
    }


def build_review_pack(
    *,
    case_config_path: Path,
    review_dir: Path,
    case_ids: list[str],
    language_template_id: str,
    language_seed: int | None,
    speed: float,
) -> dict[str, Any]:
    config = load_payload_training_case_config(case_config_path)
    resolved_case_config_path = Path(config.get('case_config_path') or case_config_path)
    cases = _case_items(config, case_ids)
    review_dir = review_dir.expanduser()
    scenario_dir = review_dir / 'scenarios'
    scenario_dir.mkdir(parents=True, exist_ok=True)

    records = []
    for index, case in enumerate(cases):
        case_id = str(case.get('case_id') or '').strip()
        seed = None if language_seed is None else int(language_seed) + index
        case_speed = float(case.get('speed', case.get('speed_mps', speed)) or speed)
        scenario = generate_scenario(
            case_id=case_id,
            case_config=resolved_case_config_path,
            language_seed=seed,
            language_template_id=language_template_id,
            speed=case_speed,
            planner=ReviewPlanner(),
        )
        payloads = command_payloads_for_execution(scenario)
        _assert_model_input_clean(scenario, payloads)

        review_scenario = deepcopy(scenario)
        review_scenario['review_context'] = {
            'review_required': True,
            'curation_method': 'automatic_accept_successful_batch_runs',
            'case_config_path': str(resolved_case_config_path),
            'note': 'Run the full batch. If a case is visually wrong, report its case number and fix the case definition.',
        }
        scenario_path = scenario_dir / f'{case_id}.json'
        scenario_path.write_text(_json_dumps(review_scenario, indent=2) + '\n', encoding='utf-8')

        records.append(
            _review_record(
                case=case,
                scenario=scenario,
                payloads=payloads,
                scenario_path=scenario_path,
            )
        )

    summary = {
        'case_config_path': str(resolved_case_config_path),
        'review_dir': str(review_dir),
        'scenario_dir': str(scenario_dir),
        'case_count': len(records),
        'review_gate': config.get('review_gate', {}),
        'records': records,
    }
    (review_dir / 'review_summary.json').write_text(
        _json_dumps(summary, indent=2) + '\n',
        encoding='utf-8',
    )
    (review_dir / 'review_index.md').write_text(
        _review_markdown(summary),
        encoding='utf-8',
    )
    return summary


def _review_markdown(summary: dict[str, Any]) -> str:
    lines = [
        '# Room 315 Payload Case Review',
        '',
        f'Config: `{summary["case_config_path"]}`',
        f'Review directory: `{summary["review_dir"]}`',
        '',
        'Batch rule: every successful case run is exported for training.',
        'If a case looks wrong in Gazebo, report its case number and fix the case definition.',
        '',
        'Model input boundary: `language`, `overhead_images`, and `last_command` only.',
        '',
    ]
    for record in summary.get('records') or []:
        launch = record.get('launch') if isinstance(record.get('launch'), dict) else {}
        lines.extend([
            f'## {record["case_id"]}',
            '',
            f'Title: {record["title"]}',
            '',
            f'Language: `{record["language"]}`',
            '',
            'Launch args:',
        ])
        for key, value in launch.items():
            lines.append(f'- `{key}:={value}`')
        lines.extend([
            '',
            f'Selected shuttle: `{record["target_shuttle_id"]}`',
            f'Target slot: `{record["target_slot"]}`',
            f'Payload condition: `{record["payload_condition"]}`',
        ])
        if record.get('blocker_shuttle_id'):
            clear_target = (
                record.get('blocker_clear_slot')
                or record.get('blocker_clear_target')
                or 'unspecified'
            )
            lines.extend([
                (
                    'Blocker: '
                    f'`{record["blocker_shuttle_id"]}` '
                    f'slot `{record["blocker_start_slot"]}` -> '
                    f'clear target `{clear_target}`'
                ),
            ])
        clearance_steps = record.get('clearance_steps') or []
        if clearance_steps:
            lines.append('Clearance steps:')
            for step in clearance_steps:
                clear_target = (
                    step.get('blocker_clear_slot')
                    or step.get('blocker_clear_target')
                    or 'unspecified'
                )
                details = []
                if step.get('blocker_clear_stopper'):
                    details.append(f'stopper `{step["blocker_clear_stopper"]}`')
                if step.get('blocker_clear_sensor'):
                    details.append(f'sensor `{step["blocker_clear_sensor"]}`')
                detail_text = f' ({", ".join(details)})' if details else ''
                lines.append(
                    '- '
                    f'`{step.get("blocker_shuttle_id", "")}` '
                    f'slot `{step.get("blocker_start_slot", "")}` -> '
                    f'clear target `{clear_target}`{detail_text}'
                )
        lines.extend([
            f'Scenario JSON: `{record["scenario_json"]}`',
            '',
            'Plan:',
        ])
        for index, step in enumerate(record.get('symbolic_plan') or []):
            lines.append(f'{index}. `{step}`')
        lines.append('')
    return '\n'.join(lines)


def _launch_command_for_case(case: dict[str, Any]) -> str:
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
    return f"""cd "${{MFJA_WS:-$HOME/mfja_3rd_floor_ros2_ws}}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

pkill -f "room_315_only.launch.py" || true
pkill -f "room_315_kinematic_shuttle_node.py" || true
pkill -f "room_315_vla_supervisor.py" || true
pkill -f "room_315_vla_dataset_recorder.py" || true
pkill -f "room315_vla_camera_bridge" || true
pkill -f "room315_world_service_bridge" || true
pkill -f "conveyor_loop_mode_controller.py.*room_315_only" || true
pkill -f "parameter_bridge /clock@rosgraph_msgs/msg/Clock" || true
pkill -f "parameter_bridge.*room_315_only" || true
pkill -f "gz sim.*room_315_only" || true
ros2 daemon stop || true
ros2 daemon start

ros2 launch mfja_3rd_floor_bringup room_315_only.launch.py \\
  robots:=none \\
  start_paused:=false \\
  gui:=true \\
  enable_room315_kinematic_shuttles:=true \\
  enable_room315_vla:=true \\
  enable_room315_vla_dataset_recorder:=true \\
  room315_enable_payload_visuals:=true \\
  enable_room315_right_rail:={str(bool(enable_right)).lower()} \\
  enable_room315_left_rail:={str(bool(enable_left)).lower()} \\
  room315_right_shuttle_count:={right_count} \\
  room315_right_start_slots:={_shell_launch_yaml_string(right_start_slots)} \\
  room315_right_loaded_shuttles:={_shell_launch_yaml_string(right_loaded)} \\
  room315_left_shuttle_count:={left_count} \\
  room315_left_start_slots:={_shell_launch_yaml_string(left_start_slots)} \\
  room315_left_loaded_shuttles:={_shell_launch_yaml_string(left_loaded)} \\
  room315_shuttles_start_enabled:=false \\
  room315_visual_debug_colors:=false \\
  room315_show_device_markers:=false \\
  room315_vla_dataset_dir:=~/room315_payload_review"""


def _launch_bool(value: Any, *, default: bool) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    return str(value).strip().casefold() in {'1', 'true', 'yes', 'on'}


def _execute_command_for_case(case_id: str, case_config_path: Path) -> str:
    output_path = f'/tmp/{case_id}_execute.json'
    return f"""cd "${{MFJA_WS:-$HOME/mfja_3rd_floor_ros2_ws}}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -e

ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \\
  --case-id {case_id} \\
  --case-config {case_config_path} \\
  --planner-backend plansys \\
  --planner-service /planner/get_plan \\
  --language-template-id loaded_shuttle_to_slot \\
  --command-timeout-s 60 \\
  --preflight-only \\
  --ready-line

ros2 run mfja_robot_control_config room_315_pddl_scenario_generator.py \\
  --case-id {case_id} \\
  --case-config {case_config_path} \\
  --planner-backend plansys \\
  --planner-service /planner/get_plan \\
  --language-template-id loaded_shuttle_to_slot \\
  --command-timeout-s 30 \\
  --arrival-timeout-s 120 \\
  --output {output_path} \\
  --quiet \\
  --execute

python3 -c 'import json; s=json.load(open("{output_path}")); assert s["execution"]["success"] is True; print("OK executed {case_id}")'"""


def _print_summary(summary: dict[str, Any]) -> None:
    print(f'Wrote {summary["case_count"]} review case(s) to {summary["review_dir"]}')
    print(f'Review index: {Path(summary["review_dir"]) / "review_index.md"}')
    for record in summary.get('records') or []:
        blocker = ''
        if record.get('blocker_shuttle_id'):
            clear_target = (
                record.get('blocker_clear_slot')
                or record.get('blocker_clear_target')
                or 'unspecified'
            )
            blocker = (
                f', blocker={record["blocker_shuttle_id"]}:'
                f'{record["blocker_start_slot"]}->{clear_target}'
            )
        print(
            f'- {record["case_id"]}: selected={record["target_shuttle_id"]}, '
            f'target_slot={record["target_slot"]}{blocker}'
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Generate a review pack for Room 315 loaded/empty payload cases.'
    )
    parser.add_argument(
        '--case-config',
        type=Path,
        default=DEFAULT_PAYLOAD_TRAINING_CASES_PATH,
        help='Payload training case YAML.',
    )
    parser.add_argument(
        '--case-id',
        action='append',
        default=[],
        help='Specific case id to review. Repeat for multiple cases. Default: all.',
    )
    parser.add_argument(
        '--review-dir',
        type=Path,
        default=DEFAULT_REVIEW_DIR,
        help='Directory where review_index.md and scenario JSON files are written.',
    )
    parser.add_argument('--language-seed', type=int, default=0)
    parser.add_argument('--language-template-id', default='loaded_shuttle_to_slot')
    parser.add_argument('--speed', type=float, default=0.3)
    parser.add_argument(
        '--print-launch-command',
        action='store_true',
        help='Print the full Room 315 launch command for exactly one case id.',
    )
    parser.add_argument(
        '--print-execute-command',
        action='store_true',
        help='Print the preflight/execute command for exactly one case id.',
    )
    args = parser.parse_args(argv)

    config = load_payload_training_case_config(args.case_config)
    resolved_case_config_path = Path(config.get('case_config_path') or args.case_config)
    selected_cases = _case_items(config, args.case_id)
    if args.print_launch_command or args.print_execute_command:
        if len(selected_cases) != 1:
            raise ValueError('printing a command requires exactly one --case-id')
        case_id = str(selected_cases[0].get('case_id') or '').strip()
        if args.print_launch_command:
            print(_launch_command_for_case(selected_cases[0]))
        if args.print_execute_command:
            print(_execute_command_for_case(case_id, resolved_case_config_path))
        return 0

    summary = build_review_pack(
        case_config_path=resolved_case_config_path,
        review_dir=args.review_dir,
        case_ids=args.case_id,
        language_template_id=args.language_template_id,
        language_seed=args.language_seed,
        speed=args.speed,
    )
    _print_summary(summary)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
