#!/usr/bin/env python3
"""Create the train/validation-only grouped V3R1 split package."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_v3_common import DEFAULT_OLD_TRAIN
from room_315_visual_v3_common import DEFAULT_OLD_TRAIN_LABELS
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_splitter import create_split_package
from room_315_visual_v3r1_common import DEFAULT_CANARY_ROOT
from room_315_visual_v3r1_common import DEFAULT_CAPTURE_ROOT
from room_315_visual_v3r1_common import DEFAULT_SPLIT_ROOT


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument('--canary-root', type=Path, default=DEFAULT_CANARY_ROOT)
    parser.add_argument('--output-root', type=Path, default=DEFAULT_SPLIT_ROOT)
    parser.add_argument('--old-train', type=Path, default=DEFAULT_OLD_TRAIN)
    parser.add_argument(
        '--old-train-labels',
        type=Path,
        default=DEFAULT_OLD_TRAIN_LABELS,
    )
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args(argv)
    result = create_split_package(
        capture_root=args.capture_root,
        canary_root=args.canary_root,
        output_root=args.output_root,
        old_train=args.old_train,
        old_train_labels=args.old_train_labels,
        resume=args.resume,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
