#!/usr/bin/env python3
"""Shared JSONL readers for Room 315 offline tools."""

import json
from collections.abc import Iterator
from pathlib import Path
from typing import Any


def iter_jsonl_objects(
    path: Path,
    *,
    error_type: type[ValueError] = ValueError,
    require_object: bool = False,
) -> Iterator[dict[str, Any]]:
    with path.expanduser().open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError as exc:
                raise error_type(f'{path}:{line_number}: invalid JSONL row: {exc}') from exc
            if isinstance(parsed, dict):
                yield parsed
            elif require_object:
                raise error_type(f'{path}:{line_number}: row must be an object')
