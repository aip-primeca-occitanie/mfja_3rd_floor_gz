#!/usr/bin/env python3
"""Fail-closed authoritative fleet and block vocabulary for Room 315 vision.

The visual pipeline must not infer fleet capacity or block classes from a
dataset.  This module reconciles the four repository sources that define the
physical fleet and exposes the one fixed schema order used everywhere else.
"""

from __future__ import annotations

import hashlib
import json
import re
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

from room_315_multi_shuttle import MAX_SHUTTLES_PER_SIDE
from room_315_rail_defaults import LEFT_ENTITY_DEFAULTS
from room_315_rail_defaults import RIGHT_ENTITY_DEFAULTS
from room_315_rail_defaults import default_rail_network_path
from room_315_rail_defaults import public_rail_segment_lengths


FIXED_VISUAL_SHUTTLE_IDENTITIES = (
    'L1', 'L2', 'L3', 'L4',
    'R1', 'R2', 'R3', 'R4',
)
IDENTITY_PATTERN = re.compile(r'^(?P<prefix>[LR])(?P<index>[1-9][0-9]*)$')
WORLD_ENTITY_PATTERN = re.compile(
    r'^room315_(?P<side>left|right)_shuttle_(?P<index>[1-9][0-9]*)$'
)


class VisualFleetError(ValueError):
    """Raised when authoritative Room 315 fleet sources disagree."""


def _source_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_identity_config_path() -> Path:
    source_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'room_315_vla'
        / 'shuttle_identity.yaml'
    )
    if source_path.is_file():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_vla'
            / 'shuttle_identity.yaml'
        )
    except Exception:
        return source_path


def default_world_path() -> Path:
    source_path = (
        _source_root()
        / 'mfja_3rd_floor_description'
        / 'worlds'
        / 'room_315_only.world'
    )
    if source_path.is_file():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_3rd_floor_description'))
            / 'worlds'
            / 'room_315_only.world'
        )
    except Exception:
        return source_path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _identity_sort_key(identity: str) -> tuple[int, int]:
    match = IDENTITY_PATTERN.fullmatch(str(identity))
    if not match:
        raise VisualFleetError(f'invalid visual shuttle identity: {identity!r}')
    return (
        0 if match.group('prefix') == 'L' else 1,
        int(match.group('index')),
    )


def _expected_from_maximum() -> tuple[str, ...]:
    count = int(MAX_SHUTTLES_PER_SIDE)
    return tuple(
        f'{prefix}{index}'
        for prefix in ('L', 'R')
        for index in range(1, count + 1)
    )


def _identities_from_yaml(path: Path) -> tuple[str, ...]:
    loaded = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    entries = loaded.get('shuttles') if isinstance(loaded, dict) else None
    if not isinstance(entries, list):
        raise VisualFleetError(f'{path} must contain a shuttles list')
    identities = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise VisualFleetError(f'{path}: shuttles[{index}] must be an object')
        side = str(entry.get('side') or '').strip().lower()
        shuttle_index = entry.get('shuttle_index')
        label = str(entry.get('label_text') or '').strip().upper()
        if side not in {'left', 'right'}:
            raise VisualFleetError(f'{path}: shuttles[{index}] has invalid side')
        try:
            expected_index = int(shuttle_index) + 1
        except (TypeError, ValueError) as exc:
            raise VisualFleetError(
                f'{path}: shuttles[{index}] has invalid shuttle_index'
            ) from exc
        expected = f'{"L" if side == "left" else "R"}{expected_index}'
        if label != expected:
            raise VisualFleetError(
                f'{path}: shuttles[{index}] label/index mismatch: '
                f'expected {expected}, found {label!r}'
            )
        identities.append(label)
    return tuple(sorted(identities, key=_identity_sort_key))


def _world_inventory(path: Path) -> tuple[tuple[str, ...], dict[str, str]]:
    root = ET.parse(path).getroot()
    by_identity: dict[str, str] = {}
    for element in root.iter('name'):
        entity_name = str(element.text or '').strip()
        match = WORLD_ENTITY_PATTERN.fullmatch(entity_name)
        if not match:
            continue
        prefix = 'L' if match.group('side') == 'left' else 'R'
        identity = f'{prefix}{int(match.group("index"))}'
        if identity in by_identity:
            raise VisualFleetError(
                f'{path} contains duplicate world identity {identity}'
            )
        by_identity[identity] = entity_name
    identities = tuple(sorted(by_identity, key=_identity_sort_key))
    return identities, dict(sorted(by_identity.items(), key=lambda item: _identity_sort_key(item[0])))


def _defaults_inventory() -> tuple[tuple[str, ...], dict[str, str]]:
    identities = []
    entities: dict[str, str] = {}
    for side, defaults in (
        ('left', LEFT_ENTITY_DEFAULTS),
        ('right', RIGHT_ENTITY_DEFAULTS),
    ):
        try:
            count = int(defaults['preloaded_shuttle_count'])
            prefix = str(defaults['entity_name_prefix'])
        except (KeyError, TypeError, ValueError) as exc:
            raise VisualFleetError(
                f'room_315_rail_defaults.py has invalid {side} entity defaults'
            ) from exc
        short_prefix = 'L' if side == 'left' else 'R'
        for index in range(1, count + 1):
            identity = f'{short_prefix}{index}'
            identities.append(identity)
            entities[identity] = f'{prefix}{index}'
    return (
        tuple(sorted(identities, key=_identity_sort_key)),
        dict(sorted(entities.items(), key=lambda item: _identity_sort_key(item[0]))),
    )


@lru_cache(maxsize=4)
def authoritative_visual_fleet(
    identity_config_path: str | Path | None = None,
    world_path: str | Path | None = None,
) -> dict[str, Any]:
    """Return the reconciled eight-shuttle inventory or fail closed."""
    identity_path = Path(
        identity_config_path or default_identity_config_path()
    ).expanduser().resolve()
    resolved_world_path = Path(world_path or default_world_path()).expanduser().resolve()
    if not identity_path.is_file():
        raise VisualFleetError(f'missing shuttle identity configuration: {identity_path}')
    if not resolved_world_path.is_file():
        raise VisualFleetError(f'missing Room 315 world: {resolved_world_path}')

    expected = _expected_from_maximum()
    yaml_identities = _identities_from_yaml(identity_path)
    world_identities, world_entities = _world_inventory(resolved_world_path)
    default_identities, default_entities = _defaults_inventory()
    sources = {
        'MAX_SHUTTLES_PER_SIDE': expected,
        'shuttle_identity.yaml': yaml_identities,
        'world_entity_names': world_identities,
        'room_315_rail_defaults.py': default_identities,
    }
    mismatches = {
        source: list(identities)
        for source, identities in sources.items()
        if identities != FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    entity_mismatches = {
        identity: {
            'world': world_entities.get(identity),
            'rail_defaults': default_entities.get(identity),
        }
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if world_entities.get(identity) != default_entities.get(identity)
    }
    if mismatches or entity_mismatches:
        raise VisualFleetError(
            'Room 315 visual fleet sources do not agree on the fixed eight-shuttle '
            f'inventory; inventories={mismatches}, entities={entity_mismatches}'
        )
    return {
        'schema_order': list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
        'max_shuttles_per_side': int(MAX_SHUTTLES_PER_SIDE),
        'world_entities': world_entities,
        'sources': {
            'maximum_constant': {
                'symbol': 'room_315_multi_shuttle.MAX_SHUTTLES_PER_SIDE',
                'value': int(MAX_SHUTTLES_PER_SIDE),
            },
            'identity_configuration': {
                'path': str(identity_path),
                'sha256': _sha256(identity_path),
            },
            'world': {
                'path': str(resolved_world_path),
                'sha256': _sha256(resolved_world_path),
            },
            'rail_defaults': {
                'symbol': (
                    'room_315_rail_defaults.'
                    'LEFT_ENTITY_DEFAULTS/RIGHT_ENTITY_DEFAULTS'
                ),
                'preloaded_shuttle_count': {
                    'left': int(LEFT_ENTITY_DEFAULTS['preloaded_shuttle_count']),
                    'right': int(RIGHT_ENTITY_DEFAULTS['preloaded_shuttle_count']),
                },
            },
        },
    }


@lru_cache(maxsize=1)
def authoritative_global_block_vocabulary() -> tuple[str, ...]:
    """Return the topology-derived block vocabulary shared by all eight entries."""
    by_side = {
        side: tuple(sorted(str(name).upper() for name in public_rail_segment_lengths(side)))
        for side in ('left', 'right')
    }
    if by_side['left'] != by_side['right']:
        raise VisualFleetError(
            'left/right public Room 315 block vocabularies disagree: '
            f'{json.dumps(by_side, sort_keys=True)}'
        )
    if not by_side['left']:
        raise VisualFleetError('Room 315 public block vocabulary is empty')
    return by_side['left']


def block_vocabulary_metadata() -> dict[str, Any]:
    vocabulary = authoritative_global_block_vocabulary()
    sources = {}
    for side in ('left', 'right'):
        path = (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'room_315_kinematics'
            / f'rail_network_{side}.yaml'
        )
        if not path.is_file():
            path = Path(default_rail_network_path(side))
        sources[side] = {
            'path': str(path.resolve()),
            'sha256': _sha256(path.resolve()),
            'public_segments': list(vocabulary),
        }
    return {
        'vocabulary': list(vocabulary),
        'source': {
            'loader': 'room_315_rail_defaults.public_rail_segment_lengths',
            'rail_networks': sources,
        },
        'dataset_inferred': False,
        'shared_by_every_fixed_entry': True,
    }


def identity_side(identity: str) -> str:
    if identity not in FIXED_VISUAL_SHUTTLE_IDENTITIES:
        raise VisualFleetError(f'identity is not in the fixed visual fleet: {identity!r}')
    return 'left' if identity.startswith('L') else 'right'


def identities_for_side(side: str) -> tuple[str, ...]:
    normalized = str(side).strip().lower()
    if normalized not in {'left', 'right'}:
        raise VisualFleetError(f'unsupported rail side: {side!r}')
    prefix = 'L' if normalized == 'left' else 'R'
    return tuple(
        identity
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if identity.startswith(prefix)
    )


# Import-time reconciliation ensures every consumer fails before generation,
# vectorization, capture, or package construction if the repository drifts.
AUTHORITATIVE_VISUAL_FLEET = authoritative_visual_fleet()
AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY = authoritative_global_block_vocabulary()
