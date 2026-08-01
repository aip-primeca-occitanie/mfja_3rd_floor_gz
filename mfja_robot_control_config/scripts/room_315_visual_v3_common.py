#!/usr/bin/env python3
"""Shared fail-closed utilities for the Room 315 hard-case visual V3 data."""

from __future__ import annotations

import hashlib
import json
import os
import statistics
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any, Callable, Iterable

from PIL import Image

from room_315_visual_fleet import AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_fleet import identity_side


SEED = 31520260730
VISUAL_SCHEMA = 'room315.visual_state.v3'
SCENARIO_SCHEMA = 'room315.visual_capture_scenario.v1'
PACKAGE_SCHEMA = 'room315.hard_case_visual_v3.v1'
IDENTITIES = tuple(FIXED_VISUAL_SHUTTLE_IDENTITIES)
BLOCKS = tuple(AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY)
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
ZONES = ('boundary', 'switch', 'slot', 'merge_conflict', 'buffer', 'ordinary')
RELATIONS = (
    'no_relation_observation',
    'blocker_ahead_same_segment',
    'nonblocker_behind_same_segment',
    'blocker_intermediate_segment',
    'nonblocker_adjacent_branch',
    'multi_blocker',
)
POSITION_RATIOS = (0.05, 0.15, 0.25, 0.40, 0.50, 0.60, 0.75, 0.85, 0.95)
POSITION_BINS = tuple(f'p{round(value * 100):02d}' for value in POSITION_RATIOS)
TARGET_OFFSETS = (-0.15, -0.10, -0.05, -0.02, 0.0, 0.02, 0.05, 0.10, 0.15)
TARGET_OFFSET_BUCKETS = tuple(
    'target'
    if value == 0.0
    else f'{"minus" if value < 0 else "plus"}_{abs(value):.2f}'
    for value in TARGET_OFFSETS
)
OCCLUSION_CLASSES = ('clear', 'partial_risk')
RENDER_BUCKETS = (
    'nominal',
    'light_low',
    'light_high',
    'exposure_low',
    'exposure_high',
    'shadow_soft',
    'noise_low',
    'antialias_variant',
)

FORBIDDEN_TEST_NAMES = {'test.jsonl', 'test_visual_labels.jsonl'}
FORBIDDEN_TEST_HASHES = {
    '2fcf78c0034fe290c39b2816e12076300decf5f7818538357fae072231b9b502',
    '1dc97b0836f40c53810306e9a09874967fa7e1067cd5de315cba0e00570277e3',
}
FORBIDDEN_TEST_PATHS = {
    Path(
        '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/'
        'test.jsonl'
    ).resolve(),
    Path(
        '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/'
        'test_visual_labels.jsonl'
    ).resolve(),
}

DEFAULT_CAPTURE_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3_capture_seed31520260730'
)
DEFAULT_SPLIT_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3_splits_seed31520260730'
)
DEFAULT_CANARY_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3_canary_seed31520260730'
)
DEFAULT_GUARD_ROOT = Path(
    '/home/tiago/room315_hard_case_visual_v3_guard_seed31520260730'
)
DEFAULT_OLD_TRAIN = Path(
    '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/'
    'train.jsonl'
)
DEFAULT_OLD_TRAIN_LABELS = Path(
    '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/'
    'train_visual_labels.jsonl'
)


class VisualV3Error(ValueError):
    """Raised when V3 generation, resume, or audit must fail closed."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def value_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def assert_allowed_input(
    raw_path: Path | str,
    *,
    hasher: Callable[[Path], str] = sha256_file,
    check_hash: bool = True,
) -> Path:
    """Reject the consumed legacy Test before opening it.

    Basename and resolved-path checks intentionally precede the optional hash
    check, ensuring the known forbidden files are never opened by this tool.
    """
    path = Path(raw_path).expanduser()
    resolved = path.resolve(strict=False)
    if path.name.lower() in FORBIDDEN_TEST_NAMES or resolved in FORBIDDEN_TEST_PATHS:
        raise VisualV3Error(f'legacy Test input is prohibited: {resolved}')
    if check_hash and path.is_file():
        fingerprint = hasher(path)
        if fingerprint.lower() in FORBIDDEN_TEST_HASHES:
            raise VisualV3Error(
                f'input content matches a prohibited legacy Test artifact: {resolved}'
            )
    return path


def assert_no_test_role(value: Any, *, context: str = 'configuration') -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).strip().lower() in {
                'test',
                'test_path',
                'test_labels',
                'test_evaluation',
                'final_test',
            }:
                raise VisualV3Error(f'{context} contains a prohibited Test key: {key}')
            assert_no_test_role(child, context=f'{context}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_no_test_role(child, context=f'{context}[{index}]')
    elif isinstance(value, str):
        candidate = Path(value)
        if candidate.name.lower() in FORBIDDEN_TEST_NAMES:
            raise VisualV3Error(f'{context} references a prohibited Test file')


def _atomic_bytes(path: Path, payload: bytes, *, overwrite: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not overwrite and path.exists():
        raise FileExistsError(path)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        suffix='.tmp',
        dir=path.parent,
    )
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if not overwrite and path.exists():
            raise FileExistsError(path)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def atomic_text(path: Path, text: str, *, overwrite: bool = True) -> None:
    _atomic_bytes(path, text.encode('utf-8'), overwrite=overwrite)


def atomic_json(path: Path, value: Any, *, overwrite: bool = True) -> None:
    assert_no_test_role(value)
    atomic_text(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
        overwrite=overwrite,
    )


def atomic_jsonl(
    path: Path,
    rows: Iterable[dict[str, Any]],
    *,
    overwrite: bool = True,
) -> None:
    payload = []
    for row in rows:
        assert_no_test_role(row)
        payload.append(canonical_json(row) + '\n')
    atomic_text(path, ''.join(payload), overwrite=overwrite)


def read_json(path: Path) -> dict[str, Any]:
    assert_allowed_input(path)
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise VisualV3Error(f'expected JSON object: {path}')
    assert_no_test_role(value, context=str(path))
    return value


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    assert_allowed_input(path)
    rows = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise VisualV3Error(f'{path}:{line_number}: invalid JSON') from exc
            if not isinstance(value, dict):
                raise VisualV3Error(f'{path}:{line_number}: expected object')
            assert_no_test_role(value, context=f'{path}:{line_number}')
            rows.append(value)
    return rows


def prepare_output_root(
    root: Path,
    configuration: dict[str, Any],
    *,
    resume: bool,
) -> str:
    """Create a root or verify a same-configuration safe resume."""
    root = root.expanduser().resolve()
    assert_no_test_role(configuration)
    fingerprint = value_sha256(configuration)
    marker = root / 'generation_configuration.json'
    if root.exists() and any(root.iterdir()):
        if not resume:
            raise VisualV3Error(f'refusing to overwrite non-empty output: {root}')
        if not marker.is_file():
            raise VisualV3Error(f'resume root lacks generation configuration: {root}')
        existing = read_json(marker)
        if existing.get('configuration_sha256') != fingerprint:
            raise VisualV3Error(
                'resume configuration mismatch: '
                f'{existing.get("configuration_sha256")} != {fingerprint}'
            )
    else:
        root.mkdir(parents=True, exist_ok=True)
        marker_record = dict(configuration)
        marker_record['configuration_sha256'] = fingerprint
        atomic_json(marker, marker_record, overwrite=False)
    return fingerprint


def stable_int(*parts: Any) -> int:
    return int(value_sha256(list(parts))[:16], 16)


def position_bin(ratio: float) -> str:
    value = float(ratio)
    if not 0.0 <= value <= 1.0:
        raise VisualV3Error(f'position ratio outside [0,1]: {ratio}')
    index = min(
        range(len(POSITION_RATIOS)),
        key=lambda candidate: (
            abs(POSITION_RATIOS[candidate] - value),
            candidate,
        ),
    )
    return POSITION_BINS[index]


def target_offset_bucket(offset: float, *, tolerance: float = 1e-8) -> str:
    value = float(offset)
    for candidate, bucket in zip(TARGET_OFFSETS, TARGET_OFFSET_BUCKETS):
        if abs(value - candidate) <= tolerance:
            return bucket
    raise VisualV3Error(f'unsupported target offset: {offset}')


def side_for_identity(identity: str) -> str:
    if identity not in IDENTITIES:
        raise VisualV3Error(f'unsupported identity: {identity}')
    return identity_side(identity)


def validate_identity_block(identity: str, block: str) -> None:
    side_for_identity(identity)
    if block not in BLOCKS:
        raise VisualV3Error(f'invalid public block for {identity}: {block}')


def family_payload(record: dict[str, Any], *, include_render: bool = True) -> dict[str, Any]:
    active = tuple(record['active_identities'])
    loaded = tuple(record['loaded_identities'])
    blocks = record['identity_to_block']
    bins = record['identity_to_position_bin']
    payload = {
        'active_identities': active,
        'loaded_identities': loaded,
        'identity_to_block': {identity: blocks[identity] for identity in active},
        'identity_to_position_bin': {identity: bins[identity] for identity in active},
        'relation_family': record['relation_family'],
        'target_identity': record.get('target_identity', 'unspecified'),
        'target_zone': record['target_zone'],
        'target_offset_bucket': record.get(
            'target_offset_bucket',
            'not_operational_target',
        ),
        'approach_direction': record.get('approach_direction', 'unspecified'),
        'occlusion_class': record['occlusion_class'],
    }
    if include_render:
        payload['render_bucket'] = record['render_bucket']
    return payload


def configuration_family_id(
    record: dict[str, Any],
    *,
    include_render: bool = True,
) -> str:
    return 'v3_family_' + value_sha256(
        family_payload(record, include_render=include_render)
    )


def image_valid(
    path: Path,
    *,
    expected_size: tuple[int, int] | None = None,
) -> dict[str, Any]:
    try:
        with Image.open(path) as image:
            image.load()
            size = tuple(image.size)
            mode = image.mode
            extrema = image.convert('RGB').getextrema()
    except (OSError, ValueError) as exc:
        raise VisualV3Error(f'image decode failed: {path}: {exc}') from exc
    if expected_size is not None and size != expected_size:
        raise VisualV3Error(f'image size mismatch: {path}: {size} != {expected_size}')
    if size[0] <= 0 or size[1] <= 0:
        raise VisualV3Error(f'image is empty: {path}')
    if all(low == high for low, high in extrema):
        raise VisualV3Error(f'image has no pixel variation: {path}')
    return {
        'path': str(path),
        'size': list(size),
        'mode': mode,
        'sha256': sha256_file(path),
    }


def counter_stats(counts: Counter[Any], expected_keys: Iterable[Any]) -> dict[str, Any]:
    values = [int(counts.get(key, 0)) for key in expected_keys]
    positive = [value for value in values if value > 0]
    minimum = min(values, default=0)
    maximum = max(values, default=0)
    return {
        'minimum': minimum,
        'maximum': maximum,
        'mean': statistics.mean(values) if values else 0.0,
        'median': statistics.median(values) if values else 0.0,
        'imbalance_ratio': (
            maximum / min(positive)
            if positive
            else None
        ),
        'empty_cell_count': sum(value == 0 for value in values),
        'cell_count': len(values),
    }
