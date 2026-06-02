#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import Any


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


def _training_row(row: dict[str, Any], fallback_step_index: int) -> dict[str, Any]:
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
        'observation.state': row.get('observation.state', []),
        'action': action,
        'privileged_eval': row.get('privileged_eval', {}),
    }
    training.update(_image_fields(row))
    return training


def extract_event_dataset(dataset_dir: Path, output_path: Path) -> dict[str, Any]:
    dataset_dir = dataset_dir.expanduser().resolve()
    output_path = output_path.expanduser()
    if not output_path.is_absolute():
        output_path = (dataset_dir / output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    event_files = sorted((dataset_dir / 'episodes').glob('episode_*/events.jsonl'))
    rows_written = 0
    episodes_seen: set[str] = set()
    with output_path.open('w', encoding='utf-8') as output:
        for event_file in event_files:
            for fallback_step_index, event_row in enumerate(_iter_jsonl(event_file)):
                training_row = _training_row(event_row, fallback_step_index)
                if not training_row['episode_id']:
                    training_row['episode_id'] = event_file.parent.name
                output.write(_json_dumps(training_row) + '\n')
                rows_written += 1
                episodes_seen.add(str(training_row['episode_id']))

    summary = {
        'dataset_dir': str(dataset_dir),
        'output_path': str(output_path),
        'episodes': len(episodes_seen),
        'event_files': len(event_files),
        'rows': rows_written,
        'source': 'episodes/*/events.jsonl',
        'ignored_source': 'episodes/*/data.jsonl',
    }
    summary_path = output_path.with_suffix(output_path.suffix + '.summary.json')
    with summary_path.open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
    summary['summary_path'] = str(summary_path)
    return summary


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
    args = parser.parse_args()
    summary = extract_event_dataset(args.dataset_dir, args.output)
    print(_json_dumps(summary))


if __name__ == '__main__':
    main()
