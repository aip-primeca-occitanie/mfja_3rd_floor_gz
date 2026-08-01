#!/usr/bin/env python3
"""Guarded capture/resume entry point for Room 315 visual V3R1."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_v3_capture import _paths
from room_315_visual_v3_capture import capture_status as _v3_capture_status
from room_315_visual_v3_capture import finalize_profile as _v3_finalize_profile
from room_315_visual_v3_capture import run_capture as _v3_run_capture
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import atomic_json
from room_315_visual_v3r1_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3r1_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_GUARD_ROOT
from room_315_visual_v3r1_common import PACKAGE_SCHEMA


def capture_status(manifest: Path, dataset_root: Path) -> dict:
    result = _v3_capture_status(manifest, dataset_root)
    result['schema_version'] = PACKAGE_SCHEMA
    return result


def finalize_profile(
    profile: str,
    manifest: Path,
    dataset_root: Path,
    output_root: Path,
) -> dict:
    result = _v3_finalize_profile(
        profile,
        manifest,
        dataset_root,
        output_root,
    )
    result['schema_version'] = PACKAGE_SCHEMA
    atomic_json(
        output_root / 'finalized' / f'{profile}_finalization.json',
        result,
    )
    return result


def run_capture(
    profile: str,
    *,
    capture_root: Path,
    canary_root: Path,
    guard_root: Path,
    resume: bool,
    keep_going: bool,
    limit: int | None,
    gui: bool,
    dry_run: bool,
) -> dict:
    result = _v3_run_capture(
        profile,
        capture_root=capture_root,
        canary_root=canary_root,
        guard_root=guard_root,
        resume=resume,
        keep_going=keep_going,
        limit=limit,
        gui=gui,
        dry_run=dry_run,
    )
    result['schema_version'] = PACKAGE_SCHEMA
    if isinstance(result.get('finalization'), dict):
        result['finalization']['schema_version'] = PACKAGE_SCHEMA
        output_root, _, _ = _paths(
            profile,
            capture_root=capture_root,
            canary_root=canary_root,
            guard_root=guard_root,
        )
        atomic_json(
            output_root / 'finalized' / f'{profile}_finalization.json',
            result['finalization'],
        )
    if not dry_run:
        output_root, _, _ = _paths(
            profile,
            capture_root=capture_root,
            canary_root=canary_root,
            guard_root=guard_root,
        )
        atomic_json(output_root / f'{profile}_capture_state.json', result)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('capture', 'status', 'finalize'))
    parser.add_argument(
        '--profile',
        choices=('smoke', 'train', 'validation', 'canary'),
        required=True,
    )
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--guard-root', type=Path, default=DEFAULT_GUARD_ROOT)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--keep-going', action='store_true')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    output_root, manifest, dataset_root = _paths(
        args.profile,
        capture_root=args.capture_root,
        canary_root=args.canary_root,
        guard_root=args.guard_root,
    )
    if args.command == 'status':
        result = capture_status(manifest, dataset_root)
    elif args.command == 'finalize':
        result = finalize_profile(
            args.profile,
            manifest,
            dataset_root,
            output_root,
        )
    else:
        result = run_capture(
            args.profile,
            capture_root=args.capture_root,
            canary_root=args.canary_root,
            guard_root=args.guard_root,
            resume=args.resume,
            keep_going=args.keep_going,
            limit=args.limit,
            gui=args.gui,
            dry_run=args.dry_run,
        )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
