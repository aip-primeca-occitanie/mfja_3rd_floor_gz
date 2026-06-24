#!/usr/bin/env python3

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_pddl_validation_gate import load_validation_result
from room_315_pddl_validation_gate import privileged_model_input_paths
from room_315_pddl_validation_gate import validation_approves_training


MODEL_INPUT_FIELDS = ('language', 'overhead_images', 'last_command')
START_LAST_COMMAND = {'action': 'START'}
PLANNING_METADATA_FIELDS = (
    'planning_source',
    'pddl_domain',
    'pddl_problem',
    'pddl_goal',
    'symbolic_plan',
    'plan_step_index',
    'generated_language',
    'language_template_id',
    'payload_condition',
    'payload_present',
    'payload_type',
)


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _iter_jsonl(path: Path):
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f'{path}:{line_number}: invalid JSONL row: {exc}') from exc
            if isinstance(parsed, dict):
                yield parsed


def _image_fields(row: dict[str, Any]) -> dict[str, Any]:
    images = {
        key: value
        for key, value in row.items()
        if key.startswith('observation.images.')
    }
    if images:
        return images
    refs = row.get('image_frame_refs', {})
    if not isinstance(refs, dict):
        return {}
    return {
        f'observation.images.{camera_name}': image_ref
        for camera_name, image_ref in refs.items()
    }


def _overhead_images(row: dict[str, Any]) -> dict[str, Any]:
    model_input = row.get('model_input')
    if isinstance(model_input, dict) and isinstance(model_input.get('overhead_images'), dict):
        return dict(model_input['overhead_images'])
    return {
        key.removeprefix('observation.images.'): value
        for key, value in _image_fields(row).items()
    }


def _model_input(row: dict[str, Any], previous_command: Any) -> dict[str, Any]:
    model_input = row.get('model_input')
    language = ''
    if isinstance(model_input, dict):
        language = str(model_input.get('language') or '')
    return {
        'language': language or str(row.get('task') or ''),
        'overhead_images': _overhead_images(row),
        'last_command': deepcopy(previous_command),
    }


def _training_row(
    row: dict[str, Any],
    fallback_step_index: int,
    *,
    previous_command: Any,
) -> dict[str, Any]:
    action = row.get('action')
    if not isinstance(action, dict):
        action = row.get('next_action')
    if not isinstance(action, dict):
        raise ValueError(
            f'event row for episode {row.get("episode_id", "")!r} is missing symbolic action'
        )

    training = {
        'episode_id': row.get('episode_id', ''),
        'step_index': int(row.get('step_index', row.get('event_index', fallback_step_index)) or 0),
        'task': row.get('task', ''),
        'model_input_schema_version': int(row.get('model_input_schema_version') or 3),
        'model_input': _model_input(row, previous_command),
        'action': action,
    }
    if row.get('action_vector') is not None:
        training['action_vector'] = row.get('action_vector')
    if isinstance(row.get('auxiliary_targets'), dict):
        training['auxiliary_targets'] = deepcopy(row['auxiliary_targets'])
    for field in PLANNING_METADATA_FIELDS:
        if field in row and row[field] is not None:
            training[field] = deepcopy(row[field])
    return training


def extract_event_dataset(
    dataset_dir: Path,
    output_path: Path,
    *,
    include_failed: bool = False,
    allow_unvalidated: bool = False,
) -> dict[str, Any]:
    dataset_dir = dataset_dir.expanduser().resolve()
    output_path = output_path.expanduser()
    if not output_path.is_absolute():
        output_path = (dataset_dir / output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    include_unapproved = bool(include_failed or allow_unvalidated)
    event_files = sorted((dataset_dir / 'episodes').glob('episode_*/events.jsonl'))
    rows_written = 0
    skipped_rows = 0
    skipped_episodes = 0
    approved_episodes = 0
    included_unapproved_episodes = 0
    skip_reasons: dict[str, int] = {}
    episodes_seen: set[str] = set()
    with output_path.open('w', encoding='utf-8') as output:
        for event_file in event_files:
            episode_dir = event_file.parent
            validation = load_validation_result(episode_dir)
            approved = validation_approves_training(validation)
            event_rows = list(_iter_jsonl(event_file))
            privileged_paths = _privileged_model_input_paths_for_rows(event_rows)
            skip_reason = ''
            if not approved:
                if validation is None:
                    skip_reason = 'unvalidated_episode'
                else:
                    skip_reason = str(
                        validation.get('failure_reason')
                        or validation.get('validation_status')
                        or 'failed_validation'
                    )
            if privileged_paths:
                skip_reason = f'privileged_model_input:{privileged_paths[0]}'

            if skip_reason and not include_unapproved:
                skipped_episodes += 1
                skipped_rows += len(event_rows)
                skip_reasons[skip_reason] = skip_reasons.get(skip_reason, 0) + 1
                continue
            if approved:
                approved_episodes += 1
            else:
                included_unapproved_episodes += 1

            previous_command: Any = deepcopy(START_LAST_COMMAND)
            for fallback_step_index, event_row in enumerate(event_rows):
                training_row = _training_row(
                    event_row,
                    fallback_step_index,
                    previous_command=previous_command,
                )
                if not training_row['episode_id']:
                    training_row['episode_id'] = event_file.parent.name
                output.write(_json_dumps(training_row) + '\n')
                rows_written += 1
                episodes_seen.add(str(training_row['episode_id']))
                previous_command = deepcopy(training_row['action'])

    summary = {
        'dataset_dir': str(dataset_dir),
        'output_path': str(output_path),
        'episodes': len(episodes_seen),
        'event_files': len(event_files),
        'rows': rows_written,
        'approved_episodes': approved_episodes,
        'skipped_episodes': skipped_episodes,
        'skipped_rows': skipped_rows,
        'included_unapproved_episodes': included_unapproved_episodes,
        'skip_reasons': dict(sorted(skip_reasons.items())),
        'include_failed': bool(include_failed),
        'allow_unvalidated': bool(allow_unvalidated),
        'source': 'episodes/*/events.jsonl',
        'ignored_source': 'episodes/*/data.jsonl',
    }
    summary_path = output_path.with_suffix(output_path.suffix + '.summary.json')
    with summary_path.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
    summary['summary_path'] = str(summary_path)
    return summary


def _privileged_model_input_paths_for_rows(rows: list[dict[str, Any]]) -> list[str]:
    paths: list[str] = []
    for index, row in enumerate(rows):
        for path in privileged_model_input_paths(row):
            paths.append(f'row[{index}]{path.removeprefix("$")}')
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Extract Room 315 event-level VLA training rows from events.jsonl only.'
    )
    parser.add_argument(
        'dataset_dir',
        type=Path,
        help='Dataset directory containing episodes/*/events.jsonl.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('meta/training_events.jsonl'),
        help='Output JSONL path. Relative paths are resolved inside dataset_dir.',
    )
    parser.add_argument(
        '--include-failed',
        action='store_true',
        help='Debug only: include failed or unvalidated episodes in the flat export.',
    )
    parser.add_argument(
        '--allow-unvalidated',
        action='store_true',
        help='Debug only: include episodes missing validation.json.',
    )
    args = parser.parse_args()
    summary = extract_event_dataset(
        args.dataset_dir,
        args.output,
        include_failed=args.include_failed,
        allow_unvalidated=args.allow_unvalidated,
    )
    print(_json_dumps(summary))


if __name__ == '__main__':
    main()
