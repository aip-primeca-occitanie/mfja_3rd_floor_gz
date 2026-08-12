#!/usr/bin/env python3
"""Preregister, capture, and seal the untouched Room 315 V4 final Test.

This tool owns data preparation only.  It never imports a model, opens a
checkpoint as model state, performs inference, computes model metrics, tunes a
threshold, or promotes a runtime.  Reference inputs are an exact allowlist of
the V4 development train/validation/canary artifacts.  In particular, no
historical Test path is accepted, enumerated, opened, or hashed.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import random
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_visual_v3_generator as hard_generator
import room_315_visual_v3_quota_planner as hard_quota
from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_fleet import identity_side
from room_315_visual_scenario_generator import validate_scenarios
from room_315_visual_state_dataset import sanitized_visual_state_row
from room_315_visual_state_dataset import validate_visual_state_rows
from room_315_visual_v3_common import BLOCKS
from room_315_visual_v3_common import POSITION_BINS
from room_315_visual_v3_common import POSITION_RATIOS
from room_315_visual_v3_common import RENDER_BUCKETS
from room_315_visual_v3_common import VisualV3Error
from room_315_visual_v3_common import image_valid
from room_315_visual_v3_common import position_bin
from room_315_visual_v3_common import stable_int
from room_315_visual_v3_common import value_sha256


CONFIG_SCHEMA = 'room315.visual_v4.final_test_config.v1'
PREREGISTRATION_SCHEMA = 'room315.visual_v4.final_test_preregistration.v1'
PLAN_LOCK_SCHEMA = 'room315.visual_v4.final_test_plan_lock.v1'
PLAN_SUMMARY_SCHEMA = 'room315.visual_v4.final_test_plan_summary.v1'
FINALIZATION_SCHEMA = 'room315.visual_v4.final_test_finalization.v1'
DISJOINT_AUDIT_SCHEMA = 'room315.visual_v4.final_test_disjoint_audit.v1'
SEAL_SCHEMA = 'room315.visual_v4.final_test_seal.v1'
DATASET_ROLE = 'sealed_final_test_only'
GENERATOR_VERSION = 'room315.visual_v4.final_test_generator.v1'
SEED = 3152026081101
SCENARIO_COUNT = 1024
LATTICE_COUNT = 1008
STRESS_COUNT = 16
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
IDENTITIES = tuple(FIXED_VISUAL_SHUTTLE_IDENTITIES)
SIDES = ('left', 'right')
LOADED_STATES = ('empty', 'loaded')
TARGET_ZONES = (
    'boundary',
    'switch',
    'slot',
    'merge_conflict',
    'buffer',
    'ordinary',
)
IDENTITY_ZONES = TARGET_ZONES + (
    'ahead_region',
    'behind_region',
    'adjacent_branch',
    'intermediate_route',
    'relation_neutral',
)
RELATIONS = (
    'no_relation_observation',
    'blocker_ahead_same_segment',
    'nonblocker_behind_same_segment',
    'blocker_intermediate_segment',
    'nonblocker_adjacent_branch',
    'multi_blocker',
)
NO_RELATION = RELATIONS[0]
MICRO_OFFSETS = (-0.0037, 0.0037)
DEFAULT_CONFIG = (
    SCRIPT_DIR.parent / 'config' / 'room_315_vla' / 'visual_state_final_test_v4.json'
)
DEFAULT_ROOT = Path(
    '/home/tiago/room315_visual_v4_final_test_seed3152026081101'
)
HISTORICAL_TEST_BASENAMES = {'test.jsonl', 'test_visual_labels.jsonl'}
HISTORICAL_TEST_PATHS = {
    Path(
        '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/'
        'test.jsonl'
    ).resolve(),
    Path(
        '/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/'
        'test_visual_labels.jsonl'
    ).resolve(),
}


# This is the complete reference-input authority.  Adding a file is a source
# change and a review event.  The tool never discovers reference files by
# walking a directory, globbing, or following a value supplied at runtime.
REFERENCE_FILE_ALLOWLIST = {
    Path('/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/train.jsonl').resolve():
        'beb6618c5c0bee80e7ec78fa7782e6a2b75c4aabf46e5745a97d6e3871a59095',
    Path('/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730/train_visual_labels.jsonl').resolve():
        '0cebc68d99db5e364d0637336244456be05b96edad5f8f176eb0176c7883e583',
    Path('/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730/train.jsonl').resolve():
        '396e3b83822dcd2ed541025fc033802592a609e288dbb28be555c6d9f586361c',
    Path('/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730/train_visual_labels.jsonl').resolve():
        'ec98fd5a94ed9d29fbb0b33dbed33877d571d263ea4a497be99d088673b71921',
    Path('/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730/validation.jsonl').resolve():
        'a4c90ac7c1043450830f69ad90094e9aacac92ad57f24fcd4439b0b2a14c9fd7',
    Path('/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730/validation_visual_labels.jsonl').resolve():
        'd62310046e9a6737e69d7d0e702f05e1073ae7de8f12b2c64655d510b410e1ab',
    Path('/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730/finalized/canary.jsonl').resolve():
        '28568e8ebf793e0a0a18ad9327f36639b2fd9c27021b9bccc4b318dd48192541',
    Path('/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730/finalized/canary_visual_labels.jsonl').resolve():
        '42d1d6ccab49d4a6bfdb2c2b79d77e404e5f9cb23066a816c1a3851d552b02db',
    Path('/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730/finalized/train_finalization.json').resolve():
        'a24aaf396ab8dc1b8688b1ee6344ce343035964ef7aafab8e8deb3e74f3ae5b1',
    Path('/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730/finalized/validation_finalization.json').resolve():
        '521446d530b92a0544c99b4b7fe8d7195c46276e518307029781c5b057ed939c',
    Path('/home/tiago/room315_hard_case_visual_v3r1_canary_seed31520260730/finalized/canary_finalization.json').resolve():
        '9a8a640062c220a00e6bef426f88823592dc24466267e7355d56f75720064e20',
}
EXPECTED_REFERENCE_NAMES = {
    'old_replay_train',
    'v3r1_train',
    'v3r1_validation',
    'v3r1_canary',
}


class FinalTestError(ValueError):
    """Raised when the final-Test preparation contract must fail closed."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def object_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
        '+00:00', 'Z'
    )


def parse_utc(value: str) -> datetime:
    try:
        parsed = datetime.fromisoformat(str(value).replace('Z', '+00:00'))
    except ValueError as exc:
        raise FinalTestError(f'invalid UTC timestamp: {value!r}') from exc
    if parsed.tzinfo is None:
        raise FinalTestError(f'timestamp has no timezone: {value!r}')
    return parsed.astimezone(timezone.utc)


def _atomic_bytes(path: Path, payload: bytes, *, overwrite: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not overwrite:
        raise FinalTestError(f'refusing to overwrite immutable artifact: {path}')
    descriptor, temporary = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent
    )
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists() and not overwrite:
            raise FinalTestError(f'refusing to overwrite immutable artifact: {path}')
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_json(path: Path, value: Any, *, overwrite: bool = False) -> None:
    _atomic_bytes(
        path,
        (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n').encode(
            'utf-8'
        ),
        overwrite=overwrite,
    )


def atomic_jsonl(
    path: Path,
    rows: Iterable[Mapping[str, Any]],
    *,
    overwrite: bool = False,
) -> None:
    payload = ''.join(canonical_json(dict(row)) + '\n' for row in rows)
    _atomic_bytes(path, payload.encode('utf-8'), overwrite=overwrite)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise FinalTestError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise FinalTestError(f'expected a JSON object: {path}')
    return value


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    try:
        stream = path.open('r', encoding='utf-8')
    except OSError as exc:
        raise FinalTestError(f'cannot open JSONL {path}: {exc}') from exc
    with stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise FinalTestError(f'{path}:{line_number}: invalid JSON') from exc
            if not isinstance(value, dict):
                raise FinalTestError(f'{path}:{line_number}: expected object')
            rows.append(value)
    return rows


def _reject_historical_test_path(path: Path, *, context: str) -> Path:
    """Reject known historical Test paths before any filesystem access."""
    expanded = path.expanduser()
    resolved = expanded.resolve(strict=False)
    if (
        expanded.name.lower() in HISTORICAL_TEST_BASENAMES
        or resolved in HISTORICAL_TEST_PATHS
    ):
        raise FinalTestError(
            f'{context} cannot reference a historical Test artifact: {resolved}'
        )
    return resolved


def _allowed_reference(path: Path, expected_sha256: str) -> Path:
    """Authorize a reference by exact path before any existence/open/hash call."""
    resolved = _reject_historical_test_path(path, context='reference')
    pinned = REFERENCE_FILE_ALLOWLIST.get(resolved)
    if pinned is None:
        raise FinalTestError(
            'reference is not in the explicit train/validation/canary allowlist: '
            f'{resolved}'
        )
    if expected_sha256 != pinned:
        raise FinalTestError(f'reference pin mismatch in configuration: {resolved}')
    if not resolved.is_file():
        raise FinalTestError(f'allowed reference is missing: {resolved}')
    actual = sha256_file(resolved)
    if actual != pinned:
        raise FinalTestError(
            f'allowed reference content changed: {resolved}: {actual} != {pinned}'
        )
    return resolved


def _load_config(path: Path) -> dict[str, Any]:
    config_path = _reject_historical_test_path(path, context='configuration')
    config = read_json(config_path)
    if config.get('schema_version') != CONFIG_SCHEMA:
        raise FinalTestError(f'configuration schema must be {CONFIG_SCHEMA}')
    if config.get('dataset_role') != DATASET_ROLE:
        raise FinalTestError(f'dataset_role must be {DATASET_ROLE}')
    if int(config.get('seed', -1)) != SEED:
        raise FinalTestError(f'final-Test seed must be exactly {SEED}')
    if int(config.get('scenario_count', -1)) != SCENARIO_COUNT:
        raise FinalTestError(f'scenario_count must be exactly {SCENARIO_COUNT}')
    composition = config.get('composition') or {}
    if (
        int(composition.get('lattice_scenarios', -1)) != LATTICE_COUNT
        or int(composition.get('stress_scenarios', -1)) != STRESS_COUNT
    ):
        raise FinalTestError('configuration must preregister 1008 lattice + 16 stress')
    references = config.get('reference_sources')
    if not isinstance(references, list):
        raise FinalTestError('reference_sources must be a list')
    names = {str(item.get('name')) for item in references if isinstance(item, dict)}
    if names != EXPECTED_REFERENCE_NAMES or len(references) != len(names):
        raise FinalTestError(
            'reference_sources must be the exact old-train/V3R1 train, '
            'validation, and canary allowlist'
        )
    for item in references:
        role = str(item.get('role'))
        if role not in {'train', 'validation', 'canary'}:
            raise FinalTestError(f'prohibited reference role: {role!r}')
        for key in ('rows', 'labels'):
            _allowed_reference(Path(str(item[key])), str(item[f'{key}_sha256']))
        if item.get('image_hash_manifest') is not None:
            _allowed_reference(
                Path(str(item['image_hash_manifest'])),
                str(item['image_hash_manifest_sha256']),
            )
    frozen = config.get('frozen_model') or {}
    checkpoint = Path(str(frozen.get('checkpoint') or '')).expanduser().resolve()
    if not checkpoint.is_file():
        raise FinalTestError(f'frozen checkpoint is missing: {checkpoint}')
    if sha256_file(checkpoint) != frozen.get('sha256'):
        raise FinalTestError('frozen checkpoint SHA-256 mismatch')
    parse_utc(str(frozen.get('frozen_at_utc') or ''))
    return config


def _family_digest(value: Any) -> str | None:
    text = str(value or '')
    tail = text.rsplit('_', 1)[-1]
    if len(tail) == 64 and all(character in '0123456789abcdef' for character in tail):
        return tail
    return None


def _present_slots(labels: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    visual = labels.get('visual_state_labels') or labels
    shuttles = visual.get('shuttles') if isinstance(visual, Mapping) else None
    if not isinstance(shuttles, list):
        raise FinalTestError('visual labels are missing fixed shuttle entries')
    return tuple(
        shuttle
        for shuttle in shuttles
        if isinstance(shuttle, dict) and bool(shuttle.get('presence'))
    )


def _label_trajectory_fingerprint(labels: Mapping[str, Any]) -> str:
    positions = []
    for shuttle in _present_slots(labels):
        rail = shuttle.get('rail_position') or {}
        positions.append((
            str(shuttle.get('id')),
            str((shuttle.get('location') or {}).get('block')),
            round(float(rail.get('s_ratio')), 9),
        ))
    return object_sha256(sorted(positions))


def _label_semantic_fingerprint(labels: Mapping[str, Any]) -> str:
    slots = []
    for shuttle in _present_slots(labels):
        rail = shuttle.get('rail_position') or {}
        slots.append((
            str(shuttle.get('id')),
            str(shuttle.get('loaded_state')),
            str((shuttle.get('location') or {}).get('block')),
            round(float(rail.get('s_ratio')), 9),
        ))
    return object_sha256(sorted(slots))


def _scenario_trajectory_fingerprint(scenario: Mapping[str, Any]) -> str:
    return object_sha256(sorted(
        (
            identity,
            str(scenario['identity_to_block'][identity]),
            round(float(scenario['identity_to_s_ratio'][identity]), 9),
        )
        for identity in scenario['active_identities']
    ))


def _scenario_semantic_fingerprint(scenario: Mapping[str, Any]) -> str:
    loaded = set(scenario['loaded_identities'])
    return object_sha256(sorted(
        (
            identity,
            'loaded' if identity in loaded else 'empty',
            str(scenario['identity_to_block'][identity]),
            round(float(scenario['identity_to_s_ratio'][identity]), 9),
        )
        for identity in scenario['active_identities']
    ))


def _pair_content_sha(left_sha256: str, right_sha256: str) -> str:
    return object_sha256({
        'left_sha256': left_sha256,
        'right_sha256': right_sha256,
    })


def contract_pair_sha(
    sample_id: str,
    left_sha256: str,
    right_sha256: str,
) -> str:
    """Canonical pair digest shared with the one-shot evaluator."""
    return object_sha256({
        'sample_id': sample_id,
        'left_sha256': left_sha256,
        'right_sha256': right_sha256,
    })


def _reference_index(config: Mapping[str, Any]) -> dict[str, Any]:
    index: dict[str, Any] = {
        'sample_ids': set(),
        'episode_ids': set(),
        'scenario_ids': set(),
        'scenario_family_digests': set(),
        'configuration_family_digests': set(),
        'configuration_core_family_digests': set(),
        'capture_configuration_fingerprints': set(),
        'geometry_fingerprints': set(),
        'trajectory_fingerprints': set(),
        'semantic_fingerprints': set(),
        'individual_image_sha256': set(),
        'pair_content_sha256': set(),
        'source_counts': {},
    }
    for source in config['reference_sources']:
        rows_path = _allowed_reference(
            Path(source['rows']), source['rows_sha256']
        )
        labels_path = _allowed_reference(
            Path(source['labels']), source['labels_sha256']
        )
        rows = read_jsonl(rows_path)
        labels = read_jsonl(labels_path)
        labels_by_sample = {str(row.get('sample_id')): row for row in labels}
        if len(labels_by_sample) != len(labels):
            raise FinalTestError(f'{source["name"]}: duplicate label sample IDs')
        if {str(row.get('sample_id')) for row in rows} != set(labels_by_sample):
            raise FinalTestError(f'{source["name"]}: row/label sample IDs differ')
        camera_hashes_by_episode: dict[str, dict[str, str]] = defaultdict(dict)
        for row in rows:
            sample_id = str(row.get('sample_id'))
            episode_id = str(row.get('episode_id'))
            trace = row.get('traceability_metadata') or {}
            index['sample_ids'].add(sample_id)
            index['episode_ids'].add(episode_id)
            index['scenario_ids'].add(str(trace.get('scenario_id') or episode_id))
            for source_key, destination in (
                ('configuration_family_id', 'configuration_family_digests'),
                ('configuration_core_family_id', 'configuration_core_family_digests'),
            ):
                digest = _family_digest(trace.get(source_key))
                if digest:
                    index[destination].add(digest)
            digest = _family_digest(row.get('scenario_family'))
            if digest:
                index['scenario_family_digests'].add(digest)
            for source_key, destination in (
                ('capture_configuration_fingerprint', 'capture_configuration_fingerprints'),
                ('geometry_fingerprint', 'geometry_fingerprints'),
            ):
                value = str(trace.get(source_key) or '')
                if value:
                    index[destination].add(value)
            label = labels_by_sample[sample_id]
            index['trajectory_fingerprints'].add(
                _label_trajectory_fingerprint(label)
            )
            index['semantic_fingerprints'].add(
                _label_semantic_fingerprint(label)
            )
            for camera, record in (trace.get('source_images') or {}).items():
                digest = str((record or {}).get('sha256') or '')
                if camera in CAMERAS and len(digest) == 64:
                    camera_hashes_by_episode[episode_id][camera] = digest
                    index['individual_image_sha256'].add(digest)
        manifest_path = source.get('image_hash_manifest')
        if manifest_path is not None:
            allowed = _allowed_reference(
                Path(manifest_path), source['image_hash_manifest_sha256']
            )
            manifest = read_json(allowed)
            for key, digest in (manifest.get('image_hashes') or {}).items():
                text = str(key)
                camera = next((item for item in CAMERAS if text.endswith(f':{item}')), None)
                if camera is None:
                    continue
                episode_id = text[:-(len(camera) + 1)]
                digest = str(digest)
                camera_hashes_by_episode[episode_id][camera] = digest
                index['individual_image_sha256'].add(digest)
        for camera_hashes in camera_hashes_by_episode.values():
            if set(camera_hashes) == set(CAMERAS):
                index['pair_content_sha256'].add(_pair_content_sha(
                    camera_hashes['left_rail_rgb'],
                    camera_hashes['right_rail_rgb'],
                ))
        index['source_counts'][source['name']] = {
            'rows': len(rows),
            'labels': len(labels),
            'individual_image_hashes': sum(
                len(value) for value in camera_hashes_by_episode.values()
            ),
        }
    return index


def _target_zone(side: str, segment: str, base_ratio: float, index: int) -> str:
    if base_ratio in {0.05, 0.95}:
        return 'boundary'
    if (
        segment in {'A1E', 'A1I', 'A2E', 'A2I', 'A3E', 'A3I', 'A4E', 'A4I'}
        and base_ratio in {0.15, 0.85}
    ):
        return 'switch'
    if segment in {'A12E', 'A34E'} and base_ratio in {0.40, 0.60, 0.75}:
        return 'slot'
    if segment in {'A14', 'A23'} and base_ratio in {0.40, 0.50, 0.60}:
        return 'merge_conflict'
    if segment in {'A12I', 'A34I'} and base_ratio in {0.25, 0.40, 0.50, 0.60, 0.75}:
        return 'buffer'
    # Keep ordinary explicitly represented even when later neutral actors use
    # a relation-specific zone label.
    return 'ordinary'


@contextmanager
def _hard_case_seed(seed: int) -> Iterator[None]:
    previous_generator = hard_generator.SEED
    previous_quota = hard_quota.SEED
    hard_generator.SEED = seed
    hard_quota.SEED = seed
    try:
        yield
    finally:
        hard_generator.SEED = previous_generator
        hard_quota.SEED = previous_quota


def _payload_assignment(
    active: tuple[str, ...],
    target: str,
    target_state: str,
    *,
    index: int,
) -> dict[str, str]:
    return hard_quota._payload_assignment(  # noqa: SLF001 - deliberate reuse
        active,
        target,
        target_state,
        profile='final_test',
        index=index,
    )


def _active_subset(
    target: str,
    cardinality: int,
    relation: str,
    *,
    index: int,
) -> tuple[str, ...]:
    return hard_quota._active_subset(  # noqa: SLF001 - deliberate reuse
        target,
        cardinality,
        profile='final_test',
        index=index,
        relation=relation,
    )


def _base_spec(
    *,
    index: int,
    cardinality: int,
    target: str,
    target_state: str,
    segment: str,
    ratio: float,
    zone: str,
    relation: str,
    offset_bucket: str,
    offset: float | None,
    stress: bool,
) -> dict[str, Any]:
    requested_relation = relation
    active = _active_subset(
        target,
        cardinality,
        requested_relation,
        index=index,
    )
    relation = hard_quota._eligible_relation(  # noqa: SLF001
        requested_relation, target, active
    )
    payload = _payload_assignment(
        active,
        target,
        target_state,
        index=index,
    )
    render = RENDER_BUCKETS[index % len(RENDER_BUCKETS)]
    return {
        'profile': 'final_test',
        'generation_index': index,
        'seed': SEED,
        'active_identities': list(active),
        'loaded_identities': [
            identity for identity in active if payload[identity] == 'loaded'
        ],
        'payload_assignment': payload,
        'target_identity': target,
        'target_loaded_state': target_state,
        'target_block': segment,
        'target_s_ratio': round(ratio, 9),
        'target_position_bin': position_bin(ratio),
        'target_zone': zone,
        'target_offset_bucket': offset_bucket,
        'target_offset': offset,
        'target_ratio': round(ratio, 9),
        'operational_target_name': None,
        'operational_target_segment': None,
        'relation_family': relation,
        # This is a requested/pre-materialisation class only.  The hard-case
        # projector replaces it with its calibrated geometric estimate.
        'occlusion_class': 'partial_risk' if stress else 'clear',
        'render_bucket': render,
        'render_parameters': {
            'bucket': render,
            'deterministic_seed': stable_int(SEED, 'final_test', index, 'render')
            % (2**31),
        },
        'approach_direction': ('increasing_s', 'decreasing_s')[index % 2],
        'canary_family': None,
        'matched_pair_id': None,
        'matched_pair_role': None,
        'configuration_variant': None,
        'presence_class': (
            'sparse' if cardinality <= 3 else 'medium' if cardinality == 4 else 'dense'
        ),
        'geometry_seed_index': index + 10_000_000,
        'geometry_seed_key': f'v4-final-test:{SEED}:{index}',
        'position_overrides': {},
        'hard_case_tags': [
            'v4_final_test_stress' if stress else 'v4_final_test_lattice',
            f'presence_{cardinality}',
            f'target_zone_{zone}',
        ],
        'spec_id': (
            f'v4_final_test_{"stress" if stress else "lattice"}_{index + 1:04d}'
        ),
    }


def build_specs(seed: int = SEED) -> list[dict[str, Any]]:
    """Return the preregistered 1008-cell lattice plus 16 stress scenes."""
    if seed != SEED:
        raise FinalTestError(f'final-Test seed is frozen at {SEED}')
    cells = []
    side_identities = {
        side: tuple(identity for identity in IDENTITIES if identity_side(identity) == side)
        for side in SIDES
    }
    for side in SIDES:
        for block_index, segment in enumerate(BLOCKS):
            for ratio_index, base_ratio in enumerate(POSITION_RATIOS):
                for state_index, target_state in enumerate(LOADED_STATES):
                    for replicate, offset in enumerate(MICRO_OFFSETS):
                        target = side_identities[side][
                            (block_index + ratio_index + state_index + replicate) % 4
                        ]
                        cells.append({
                            'side': side,
                            'segment': segment,
                            'base_ratio': base_ratio,
                            'target_state': target_state,
                            'replicate': replicate,
                            'offset': offset,
                            'target': target,
                        })
    if len(cells) != LATTICE_COUNT:
        raise FinalTestError(f'internal lattice count is {len(cells)}, expected 1008')
    random.Random(seed).shuffle(cells)
    specs = []
    with _hard_case_seed(seed):
        for index, cell in enumerate(cells):
            cardinality = index % 8 + 1
            ratio = float(cell['base_ratio']) + float(cell['offset'])
            zone = _target_zone(
                str(cell['side']),
                str(cell['segment']),
                float(cell['base_ratio']),
                index,
            )
            specs.append(_base_spec(
                index=index,
                cardinality=cardinality,
                target=str(cell['target']),
                target_state=str(cell['target_state']),
                segment=str(cell['segment']),
                ratio=ratio,
                zone=zone,
                relation=NO_RELATION,
                offset_bucket=(
                    'micro_minus_0.0037'
                    if float(cell['offset']) < 0
                    else 'micro_plus_0.0037'
                ),
                offset=float(cell['offset']),
                stress=False,
            ))
        stress_relations = (
            NO_RELATION,
            NO_RELATION,
            'blocker_ahead_same_segment',
            'blocker_ahead_same_segment',
            'multi_blocker',
            'multi_blocker',
            'nonblocker_adjacent_branch',
            'nonblocker_adjacent_branch',
            'blocker_intermediate_segment',
            'blocker_intermediate_segment',
            'nonblocker_behind_same_segment',
            'nonblocker_behind_same_segment',
            'multi_blocker',
            'multi_blocker',
            'blocker_ahead_same_segment',
            'blocker_ahead_same_segment',
        )
        for stress_index in range(STRESS_COUNT):
            index = LATTICE_COUNT + stress_index
            cardinality = stress_index // 2 + 1
            target = IDENTITIES[stress_index % len(IDENTITIES)]
            base_ratio = POSITION_RATIOS[(stress_index * 5 + 2) % len(POSITION_RATIOS)]
            ratio = base_ratio + (-0.0073 if stress_index % 2 == 0 else 0.0073)
            zone = TARGET_ZONES[stress_index % len(TARGET_ZONES)]
            specs.append(_base_spec(
                index=index,
                cardinality=cardinality,
                target=target,
                target_state=LOADED_STATES[stress_index % 2],
                segment=BLOCKS[(stress_index * 3 + 1) % len(BLOCKS)],
                ratio=ratio,
                zone=zone,
                relation=stress_relations[stress_index],
                offset_bucket=f'stress_micro_{stress_index + 1:02d}',
                offset=(-0.0073 if stress_index % 2 == 0 else 0.0073),
                stress=True,
            ))
    if len(specs) != SCENARIO_COUNT:
        raise FinalTestError('internal final-Test scenario count mismatch')
    return specs


def _retry_spec(source: Mapping[str, Any], attempt: int) -> dict[str, Any]:
    candidate = copy.deepcopy(dict(source))
    if attempt:
        candidate['family_avoidance_attempt'] = attempt
        candidate['geometry_seed_index'] = (
            int(source['geometry_seed_index']) + attempt * 10007
        )
        candidate['geometry_seed_key'] = (
            f'{source["geometry_seed_key"]}:isolation:{attempt}'
        )
        candidate['approach_direction'] = (
            'increasing_s', 'decreasing_s'
        )[(int(source['generation_index']) + attempt) % 2]
        render = RENDER_BUCKETS[
            (RENDER_BUCKETS.index(str(source['render_bucket'])) + attempt)
            % len(RENDER_BUCKETS)
        ]
        candidate['render_bucket'] = render
        candidate['render_parameters']['bucket'] = render
        candidate['render_parameters']['deterministic_seed'] = stable_int(
            SEED, source['spec_id'], 'isolation-render', attempt
        ) % (2**31)
    else:
        candidate['family_avoidance_attempt'] = 0
    return candidate


def _v4ize_scenario(
    row: dict[str, Any],
    spec: Mapping[str, Any],
    ordinal: int,
) -> dict[str, Any]:
    result = copy.deepcopy(row)
    configuration_digest = _family_digest(row['configuration_family_id'])
    core_digest = _family_digest(row['configuration_core_family_id'])
    if configuration_digest is None or core_digest is None:
        raise FinalTestError('hard-case materializer returned an invalid family ID')
    result.update({
        'manifest_profile_schema': PREREGISTRATION_SCHEMA,
        'generator_version': GENERATOR_VERSION,
        'scenario_id': (
            f'v4_final_test_{ordinal:04d}_{row["geometry_fingerprint"][:12]}'
        ),
        'scenario_family': (
            'v4_final_test_family_capture_'
            + object_sha256({
                'capture_configuration_fingerprint': row[
                    'capture_configuration_fingerprint'
                ],
                'spec_id': spec['spec_id'],
                'seed': SEED,
            })
        ),
        'configuration_family_id': (
            f'v4_final_test_family_{configuration_digest}'
        ),
        'configuration_core_family_id': (
            f'v4_final_test_family_core_{core_digest}'
        ),
        'seed': SEED,
        'dataset_partition': 'final_test',
        'source_profile': 'final_test',
        'imported_from_v3': False,
        'source_scenario_id': None,
        'source_manifest_sha256': None,
        'v4_final_test_role': DATASET_ROLE,
        'inference_exposure': 'never_evaluated_at_plan_time',
        'occlusion_evidence_type': 'static_calibrated_projection_risk_estimate',
        'occlusion_claim_scope': (
            'estimated_geometry_risk_not_measured_pixel_occlusion_percentage'
        ),
    })
    # Preserve the geometric placement zone for the selected target and
    # relation actors, while naming actors that are deliberately irrelevant to
    # the selected route as relation-neutral.  This is metadata only; it does
    # not alter placement, images, labels, or any model input.
    probe = result.get('relation_probe') or {}
    neutral_ids = set(probe.get('relation_neutral_shuttle_ids') or [])
    neutral_ids.update(probe.get('opposite_rail_neutral_shuttle_ids') or [])
    for identity in neutral_ids:
        if identity in result.get('identity_to_zone', {}):
            result['identity_to_zone'][identity] = 'relation_neutral'
    return result


def materialize_plan(
    config: Mapping[str, Any],
    references: Mapping[str, Any],
) -> list[dict[str, Any]]:
    specs = build_specs(int(config['seed']))
    cameras = hard_generator.load_camera_projections(
        hard_generator._default_camera_model_path()  # noqa: SLF001
    )
    identity_contract = hard_generator._identity_visual_contract()  # noqa: SLF001
    local: dict[str, set[str]] = {
        'scenario_ids': set(),
        'scenario_family_digests': set(),
        'configuration_family_digests': set(),
        'configuration_core_family_digests': set(),
        'capture_configuration_fingerprints': set(),
        'geometry_fingerprints': set(),
        'trajectory_fingerprints': set(),
        'semantic_fingerprints': set(),
    }
    rows = []
    with _hard_case_seed(int(config['seed'])):
        for ordinal, source in enumerate(specs, start=1):
            last_reason = 'no attempt made'
            for attempt in range(256):
                candidate = _retry_spec(source, attempt)
                try:
                    raw = hard_generator.materialize_spec(
                        candidate,
                        cameras=cameras,
                        identity_contract=identity_contract,
                    )
                except VisualV3Error as exc:
                    last_reason = str(exc)
                    continue
                row = _v4ize_scenario(raw, candidate, ordinal)
                family_digest = _family_digest(row['configuration_family_id'])
                core_digest = _family_digest(row['configuration_core_family_id'])
                scenario_family_digest = _family_digest(row['scenario_family'])
                trajectory = _scenario_trajectory_fingerprint(row)
                semantic = _scenario_semantic_fingerprint(row)
                comparisons = {
                    'scenario_ids': row['scenario_id'],
                    'scenario_family_digests': scenario_family_digest,
                    'configuration_family_digests': family_digest,
                    'configuration_core_family_digests': core_digest,
                    'capture_configuration_fingerprints': row[
                        'capture_configuration_fingerprint'
                    ],
                    'geometry_fingerprints': row['geometry_fingerprint'],
                    'trajectory_fingerprints': trajectory,
                    'semantic_fingerprints': semantic,
                }
                collision = next(
                    (
                        name
                        for name, value in comparisons.items()
                        if value is None
                        or value in references[name]
                        or value in local[name]
                    ),
                    None,
                )
                if collision:
                    last_reason = f'{collision} collision'
                    continue
                # The 1008 preregistered lattice dimensions may never drift
                # while retrying neutral-actor placement for isolation.
                if ordinal <= LATTICE_COUNT:
                    target = candidate['target_identity']
                    target_loaded = (
                        'loaded' if target in row['loaded_identities'] else 'empty'
                    )
                    if (
                        identity_side(target) != identity_side(source['target_identity'])
                        or row['identity_to_block'][target] != source['target_block']
                        or row['identity_to_position_bin'][target]
                        != source['target_position_bin']
                        or target_loaded != source['target_loaded_state']
                    ):
                        raise FinalTestError(
                            f'{source["spec_id"]}: lattice cell drifted during isolation'
                        )
                for name, value in comparisons.items():
                    local[name].add(str(value))
                rows.append(row)
                break
            else:
                raise FinalTestError(
                    f'{source["spec_id"]}: could not isolate final-Test scene '
                    f'after 256 attempts: {last_reason}'
                )
    validate_scenarios(rows)
    if len(rows) != SCENARIO_COUNT:
        raise FinalTestError('materialized scenario count mismatch')
    return rows


def _presence_density(cardinality: int) -> str:
    if 1 <= cardinality <= 3:
        return 'sparse'
    if cardinality == 4:
        return 'medium'
    if 5 <= cardinality <= 8:
        return 'dense'
    raise FinalTestError(f'invalid presence cardinality: {cardinality}')


def _nested_counter(counter: Counter[tuple[str, str]]) -> dict[str, dict[str, int]]:
    result: dict[str, dict[str, int]] = {}
    first_values = sorted({first for first, _ in counter})
    second_values = sorted({second for _, second in counter})
    for first in first_values:
        result[first] = {
            second: int(counter[(first, second)])
            for second in second_values
            if counter[(first, second)]
        }
    return result


def plan_support_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    cardinality = Counter()
    target_identity = Counter()
    target_side_segment = Counter()
    target_position_bin = Counter()
    target_loaded_state = Counter()
    target_zone = Counter()
    relations = Counter()
    estimated_occlusion = Counter()
    identity_zone = Counter()
    visible_identity = Counter()
    visible_side_segment = Counter()
    for row in rows:
        active = list(row['active_identities'])
        loaded = set(row['loaded_identities'])
        target = str(row['target_identity'])
        cardinality[str(len(active))] += 1
        target_identity[target] += 1
        target_side_segment[(
            identity_side(target), row['identity_to_block'][target]
        )] += 1
        target_position_bin[row['identity_to_position_bin'][target]] += 1
        target_loaded_state['loaded' if target in loaded else 'empty'] += 1
        target_zone[row['target_zone']] += 1
        relations[row['relation_family']] += 1
        estimated_occlusion[row['occlusion_class']] += 1
        for identity in active:
            visible_identity[identity] += 1
            visible_side_segment[(
                identity_side(identity), row['identity_to_block'][identity]
            )] += 1
            identity_zone[(identity, row['identity_to_zone'][identity])] += 1
    return {
        'scenario_count': len(rows),
        'lattice_scenario_count': sum(
            'v4_final_test_lattice' in row.get('hard_case_tags', []) for row in rows
        ),
        'stress_scenario_count': sum(
            'v4_final_test_stress' in row.get('hard_case_tags', []) for row in rows
        ),
        'presence_cardinality_counts': dict(sorted(cardinality.items())),
        'target_by_identity': dict(sorted(target_identity.items())),
        'target_by_side_x_segment': _nested_counter(target_side_segment),
        'target_by_position_bin': dict(sorted(target_position_bin.items())),
        'target_by_loaded_state': dict(sorted(target_loaded_state.items())),
        'target_by_zone': dict(sorted(target_zone.items())),
        'records_by_relation_family': dict(sorted(relations.items())),
        'records_by_estimated_occlusion_class': dict(
            sorted(estimated_occlusion.items())
        ),
        'visible_by_identity': dict(sorted(visible_identity.items())),
        'visible_by_side_x_segment': _nested_counter(visible_side_segment),
        'visible_by_identity_zone': _nested_counter(identity_zone),
        'occlusion_claim_scope': (
            'static calibrated-projection risk estimate; not a measured '
            'pixel-occlusion percentage'
        ),
    }


def _uniqueness_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    fields = {
        'scenario_id': [row['scenario_id'] for row in rows],
        'scenario_family': [row['scenario_family'] for row in rows],
        'configuration_family_id': [row['configuration_family_id'] for row in rows],
        'configuration_core_family_id': [
            row['configuration_core_family_id'] for row in rows
        ],
        'capture_configuration_fingerprint': [
            row['capture_configuration_fingerprint'] for row in rows
        ],
        'geometry_fingerprint': [row['geometry_fingerprint'] for row in rows],
        'trajectory_fingerprint': [
            _scenario_trajectory_fingerprint(row) for row in rows
        ],
        'semantic_fingerprint': [
            _scenario_semantic_fingerprint(row) for row in rows
        ],
    }
    return {
        name: {
            'count': len(values),
            'unique_count': len(set(values)),
            'unique': len(values) == len(set(values)),
        }
        for name, values in fields.items()
    }


def _static_disjoint_audit(
    rows: list[dict[str, Any]],
    references: Mapping[str, Any],
) -> dict[str, Any]:
    candidate_sets = {
        'scenario_ids': {row['scenario_id'] for row in rows},
        'scenario_family_digests': {
            _family_digest(row['scenario_family']) for row in rows
        },
        'configuration_family_digests': {
            _family_digest(row['configuration_family_id']) for row in rows
        },
        'configuration_core_family_digests': {
            _family_digest(row['configuration_core_family_id']) for row in rows
        },
        'capture_configuration_fingerprints': {
            row['capture_configuration_fingerprint'] for row in rows
        },
        'geometry_fingerprints': {row['geometry_fingerprint'] for row in rows},
        'trajectory_fingerprints': {
            _scenario_trajectory_fingerprint(row) for row in rows
        },
        'semantic_fingerprints': {
            _scenario_semantic_fingerprint(row) for row in rows
        },
    }
    overlaps = {
        name: sorted(candidate_sets[name] & references[name])
        for name in candidate_sets
    }
    return {
        'candidate_counts': {
            name: len(values) for name, values in sorted(candidate_sets.items())
        },
        'reference_counts': {
            name: len(references[name]) for name in sorted(candidate_sets)
        },
        'overlap_counts': {
            name: len(values) for name, values in sorted(overlaps.items())
        },
        'overlap_examples': {
            name: values[:5] for name, values in sorted(overlaps.items()) if values
        },
        'passed': not any(overlaps.values()),
    }


def _assert_plan_contract(
    rows: list[dict[str, Any]],
    support: Mapping[str, Any],
    uniqueness: Mapping[str, Any],
    static_disjoint: Mapping[str, Any],
) -> None:
    issues = []
    if len(rows) != SCENARIO_COUNT:
        issues.append(f'scenario count {len(rows)} != {SCENARIO_COUNT}')
    if support['lattice_scenario_count'] != LATTICE_COUNT:
        issues.append('lattice scenario count is not 1008')
    if support['stress_scenario_count'] != STRESS_COUNT:
        issues.append('stress scenario count is not 16')
    expected_cardinality = {str(value): 128 for value in range(1, 9)}
    if support['presence_cardinality_counts'] != expected_cardinality:
        issues.append('presence cardinalities are not exactly 128 each')
    if set(support['target_by_identity']) != set(IDENTITIES):
        issues.append('not all eight identities occur as targets')
    target_identity_counts = list(support['target_by_identity'].values())
    if (
        min(target_identity_counts, default=0) < 127
        or max(target_identity_counts, default=0)
        - min(target_identity_counts, default=0) > 2
    ):
        issues.append('target identity support is not balanced within two scenes')
    for side in SIDES:
        if set(support['target_by_side_x_segment'].get(side, {})) != set(BLOCKS):
            issues.append(f'{side} target coverage does not contain all 14 segments')
        if any(
            count < 36
            for count in support['target_by_side_x_segment'].get(side, {}).values()
        ):
            issues.append(f'{side} has a target segment with fewer than 36 scenes')
    if set(support['target_by_position_bin']) != set(POSITION_BINS):
        issues.append('not all nine target position bins are present')
    if any(count < 112 for count in support['target_by_position_bin'].values()):
        issues.append('a target position bin has fewer than 112 scenes')
    if set(support['target_by_loaded_state']) != set(LOADED_STATES):
        issues.append('target loaded/empty support is incomplete')
    if any(count < 504 for count in support['target_by_loaded_state'].values()):
        issues.append('target loaded/empty support has fewer than 504 scenes')
    if set(support['target_by_zone']) != set(TARGET_ZONES):
        issues.append('six target zones are not all present')
    if set(support['records_by_relation_family']) != set(RELATIONS):
        issues.append('six relation families are not all present')
    if set(support['records_by_estimated_occlusion_class']) != {
        'clear', 'partial_risk'
    }:
        issues.append('estimated clear/partial-risk support is incomplete')
    planned_identity_zones = {
        zone
        for by_zone in support['visible_by_identity_zone'].values()
        for zone in by_zone
    }
    if planned_identity_zones != set(IDENTITY_ZONES):
        issues.append(
            'the static plan does not cover all eleven canonical/relation '
            f'identity zones: {sorted(planned_identity_zones)}'
        )
    for name, result in uniqueness.items():
        if not result['unique']:
            issues.append(f'{name} is not unique')
    if not static_disjoint['passed']:
        issues.append('static train/validation/canary disjoint audit failed')
    if issues:
        raise FinalTestError('final-Test plan contract failed: ' + '; '.join(issues))


def _plan_paths(root: Path) -> dict[str, Path]:
    return {
        'manifest': root / 'scenario_manifest.jsonl',
        'summary': root / 'scenario_summary.json',
        'preregistration': root / 'preregistration.json',
        'lock': root / 'plan_lock.json',
    }


def _git_state() -> dict[str, Any]:
    repository = SCRIPT_DIR.parents[1]
    commit = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    status = subprocess.run(
        ['git', 'status', '--short'],
        cwd=repository,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    return {'commit': commit, 'dirty': bool(status), 'status': status}


def create_plan(
    config_path: Path,
    *,
    output_root: Path | None = None,
) -> dict[str, Any]:
    config_path = config_path.expanduser().resolve()
    config = _load_config(config_path)
    config_sha = sha256_file(config_path)
    root = (
        output_root.expanduser().resolve()
        if output_root is not None
        else Path(config['output_root']).expanduser().resolve()
    )
    if root.exists():
        raise FinalTestError(f'refusing to overwrite final-Test root: {root}')
    root.parent.mkdir(parents=True, exist_ok=True)
    checkpoint_frozen = parse_utc(config['frozen_model']['frozen_at_utc'])
    plan_started = datetime.now(timezone.utc)
    if plan_started <= checkpoint_frozen:
        raise FinalTestError('model freeze must precede final-Test planning')
    checkpoint_mtime = datetime.fromtimestamp(
        Path(config['frozen_model']['checkpoint']).stat().st_mtime,
        tz=timezone.utc,
    )
    if checkpoint_mtime > checkpoint_frozen.replace(microsecond=999999):
        raise FinalTestError(
            'checkpoint file mtime is later than the declared frozen_at_utc'
        )

    references = _reference_index(config)
    rows = materialize_plan(config, references)
    support = plan_support_summary(rows)
    uniqueness = _uniqueness_summary(rows)
    static_disjoint = _static_disjoint_audit(rows, references)
    _assert_plan_contract(rows, support, uniqueness, static_disjoint)

    staging = Path(tempfile.mkdtemp(prefix=f'.{root.name}.plan.', dir=root.parent))
    try:
        paths = _plan_paths(staging)
        atomic_jsonl(paths['manifest'], rows)
        summary = {
            'schema_version': PLAN_SUMMARY_SCHEMA,
            'dataset_role': DATASET_ROLE,
            'generator_version': GENERATOR_VERSION,
            'seed': SEED,
            'scenario_count': len(rows),
            'planned_image_count': len(rows) * len(CAMERAS),
            'support': support,
            'uniqueness': uniqueness,
            'static_reference_disjoint_audit': static_disjoint,
            'reference_source_counts': references['source_counts'],
            'inference_count': 0,
            'inference_status': 'not_run',
        }
        atomic_json(paths['summary'], summary)
        planned_at = utc_now()
        preregistration = {
            'schema_version': PREREGISTRATION_SCHEMA,
            'dataset_role': DATASET_ROLE,
            'purpose': config['purpose'],
            'seed': SEED,
            'preregistered_at_utc': planned_at,
            'model_frozen_before_preregistration': (
                checkpoint_frozen < parse_utc(planned_at)
            ),
            'frozen_model': copy.deepcopy(config['frozen_model']),
            'configuration': {
                'path': str(config_path),
                'sha256': config_sha,
            },
            'scenario_manifest': {
                'path': str(root / 'scenario_manifest.jsonl'),
                'sha256': sha256_file(paths['manifest']),
                'scenario_count': len(rows),
            },
            'coverage_contract': copy.deepcopy(config['composition']),
            'support_summary_sha256': object_sha256(support),
            'static_reference_disjoint_audit': static_disjoint,
            'reference_inputs': [
                {
                    key: value
                    for key, value in item.items()
                    if key in {
                        'name', 'role', 'rows', 'rows_sha256', 'labels',
                        'labels_sha256', 'image_hash_manifest',
                        'image_hash_manifest_sha256',
                    }
                }
                for item in config['reference_sources']
            ],
            'occlusion_claim_scope': config['composition'][
                'occlusion_contract'
            ]['claim_scope'],
            'capture_started': False,
            'inference_count': 0,
            'inference_status': 'not_run',
            'prohibitions': copy.deepcopy(config['prohibitions']),
            'repository_state': _git_state(),
        }
        atomic_json(paths['preregistration'], preregistration)
        lock = {
            'schema_version': PLAN_LOCK_SCHEMA,
            'dataset_role': DATASET_ROLE,
            'locked_at_utc': utc_now(),
            'model_frozen_before_lock': checkpoint_frozen < datetime.now(timezone.utc),
            'seed': SEED,
            'scenario_count': len(rows),
            'configuration': {
                'path': str(config_path),
                'sha256': config_sha,
                'frozen_before_manifest_and_capture': True,
            },
            'frozen_model': copy.deepcopy(config['frozen_model']),
            'artifacts': {
                'scenario_manifest.jsonl': sha256_file(paths['manifest']),
                'scenario_summary.json': sha256_file(paths['summary']),
                'preregistration.json': sha256_file(paths['preregistration']),
            },
            'capture_started': False,
            'inference_count': 0,
            'inference_status': 'not_run',
        }
        atomic_json(paths['lock'], lock)
        for path in paths.values():
            path.chmod(0o444)
        os.replace(staging, root)
    except BaseException:
        if staging.exists():
            shutil.rmtree(staging)
        raise
    return {
        'dataset_role': DATASET_ROLE,
        'output_root': str(root),
        'scenario_count': len(rows),
        'manifest_sha256': lock['artifacts']['scenario_manifest.jsonl'],
        'configuration_sha256': config_sha,
        'inference_status': 'not_run',
        'passed': True,
    }


def _verify_locked_artifacts(root: Path, config_path: Path) -> dict[str, Any]:
    paths = _plan_paths(root)
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise FinalTestError(f'final-Test plan is incomplete: {missing}')
    lock = read_json(paths['lock'])
    if lock.get('schema_version') != PLAN_LOCK_SCHEMA:
        raise FinalTestError('invalid final-Test plan-lock schema')
    if lock.get('dataset_role') != DATASET_ROLE:
        raise FinalTestError('plan-lock dataset role mismatch')
    if int(lock.get('seed', -1)) != SEED or int(lock.get('scenario_count', -1)) != SCENARIO_COUNT:
        raise FinalTestError('plan-lock seed/count mismatch')
    config_path = config_path.expanduser().resolve()
    expected_config = lock.get('configuration') or {}
    if Path(str(expected_config.get('path'))).resolve() != config_path:
        raise FinalTestError('plan-lock configuration path mismatch')
    if sha256_file(config_path) != expected_config.get('sha256'):
        raise FinalTestError('configuration changed after plan lock')
    if not expected_config.get('frozen_before_manifest_and_capture'):
        raise FinalTestError('configuration was not frozen before manifest/capture')
    for filename, expected in (lock.get('artifacts') or {}).items():
        path = root / filename
        if not path.is_file() or sha256_file(path) != expected:
            raise FinalTestError(f'locked artifact changed: {path}')
    preregistration = read_json(paths['preregistration'])
    if (
        preregistration.get('schema_version') != PREREGISTRATION_SCHEMA
        or preregistration.get('dataset_role') != DATASET_ROLE
        or not preregistration.get('model_frozen_before_preregistration')
        or preregistration.get('inference_status') != 'not_run'
        or int(preregistration.get('inference_count', -1)) != 0
    ):
        raise FinalTestError('preregistration freeze/inference contract is invalid')
    config = _load_config(config_path)
    if config['frozen_model'] != lock.get('frozen_model'):
        raise FinalTestError('frozen checkpoint declaration changed after plan lock')
    return {
        'paths': paths,
        'lock': lock,
        'preregistration': preregistration,
        'config': config,
    }


def _assert_v4_prefixes(rows: list[dict[str, Any]]) -> None:
    for row in rows:
        checks = {
            'scenario_id': str(row.get('scenario_id')).startswith('v4_final_test_'),
            'scenario_family': str(row.get('scenario_family')).startswith(
                'v4_final_test_family_'
            ),
            'configuration_family_id': str(
                row.get('configuration_family_id')
            ).startswith('v4_final_test_family_'),
            'configuration_core_family_id': str(
                row.get('configuration_core_family_id')
            ).startswith('v4_final_test_family_'),
            'dataset_partition': row.get('dataset_partition') == 'final_test',
            'source_profile': row.get('source_profile') == 'final_test',
            'imported_from_v3': row.get('imported_from_v3') is False,
        }
        failed = [name for name, passed in checks.items() if not passed]
        if failed:
            raise FinalTestError(
                f'{row.get("scenario_id")}: invalid V4 freshness fields: {failed}'
            )


def verify_plan(
    root: Path,
    config_path: Path,
    *,
    regenerate: bool = True,
) -> dict[str, Any]:
    root = root.expanduser().resolve()
    locked = _verify_locked_artifacts(root, config_path)
    rows = read_jsonl(locked['paths']['manifest'])
    validate_scenarios(rows)
    _assert_v4_prefixes(rows)
    references = _reference_index(locked['config'])
    support = plan_support_summary(rows)
    uniqueness = _uniqueness_summary(rows)
    static_disjoint = _static_disjoint_audit(rows, references)
    _assert_plan_contract(rows, support, uniqueness, static_disjoint)
    deterministic = None
    if regenerate:
        regenerated = materialize_plan(locked['config'], references)
        deterministic = (
            ''.join(canonical_json(row) + '\n' for row in regenerated).encode('utf-8')
            == locked['paths']['manifest'].read_bytes()
        )
        if not deterministic:
            raise FinalTestError('deterministic manifest regeneration differs from lock')
    return {
        'schema_version': PLAN_LOCK_SCHEMA,
        'dataset_role': DATASET_ROLE,
        'root': str(root),
        'scenario_count': len(rows),
        'manifest_sha256': sha256_file(locked['paths']['manifest']),
        'configuration_sha256': sha256_file(config_path.expanduser().resolve()),
        'support': support,
        'uniqueness': uniqueness,
        'static_reference_disjoint_audit': static_disjoint,
        'deterministic_regeneration_matches': deterministic,
        'capture_started_when_preregistered': False,
        'inference_status': 'not_run',
        'passed': True,
    }


def capture_command(root: Path, config_path: Path) -> list[str]:
    verify_plan(root, config_path, regenerate=False)
    root = root.expanduser().resolve()
    return [
        sys.executable,
        str(SCRIPT_DIR / 'room_315_visual_scenario_runner.py'),
        '--scenario-manifest',
        str(root / 'scenario_manifest.jsonl'),
        '--output-dataset',
        str(root / 'dataset'),
        '--readiness-timeout-seconds',
        '45',
        '--capture-timeout-seconds',
        '30',
        '--max-camera-skew-seconds',
        '0.15',
        '--resume',
        '--keep-going',
    ]


def capture_status(root: Path, config_path: Path) -> dict[str, Any]:
    verify_plan(root, config_path, regenerate=False)
    root = root.expanduser().resolve()
    rows = read_jsonl(root / 'scenario_manifest.jsonl')
    complete = []
    incomplete = []
    for row in rows:
        episode = root / 'dataset' / 'episodes' / row['scenario_id']
        required = [
            episode / 'event.json',
            episode / 'validation.json',
            *(
                episode / 'images' / camera / 'frame_000000.jpg'
                for camera in CAMERAS
            ),
        ]
        if all(path.is_file() for path in required):
            complete.append(row['scenario_id'])
        else:
            incomplete.append(row['scenario_id'])
    return {
        'schema_version': 'room315.visual_v4.final_test_capture_status.v1',
        'dataset_role': DATASET_ROLE,
        'scenario_count': len(rows),
        'complete_count': len(complete),
        'incomplete_count': len(incomplete),
        'first_incomplete_scenario_id': incomplete[0] if incomplete else None,
        'capture_complete': not incomplete,
        'inference_status': 'not_run',
    }


def _final_trace(scenario: Mapping[str, Any]) -> dict[str, Any]:
    return {
        'scenario_id': scenario['scenario_id'],
        'spec_id': scenario['spec_id'],
        'generation_index': scenario['generation_index'],
        'dataset_partition': 'final_test',
        'source_profile': 'final_test',
        'configuration_family_id': scenario['configuration_family_id'],
        'configuration_core_family_id': scenario[
            'configuration_core_family_id'
        ],
        'geometry_fingerprint': scenario['geometry_fingerprint'],
        'capture_configuration_fingerprint': scenario[
            'capture_configuration_fingerprint'
        ],
        'active_identities': scenario['active_identities'],
        'loaded_identities': scenario['loaded_identities'],
        'identity_to_block': scenario['identity_to_block'],
        'identity_to_position_bin': scenario['identity_to_position_bin'],
        'identity_to_s_m': scenario['identity_to_s_m'],
        'identity_to_s_ratio': scenario['identity_to_s_ratio'],
        'identity_to_segment_length_m': scenario[
            'identity_to_segment_length_m'
        ],
        'identity_to_zone': scenario['identity_to_zone'],
        'relation_family': scenario['relation_family'],
        'target_identity': scenario['target_identity'],
        'target_zone': scenario['target_zone'],
        'target_offset_bucket': scenario['target_offset_bucket'],
        'target_offset': scenario.get('target_offset'),
        'target_ratio': scenario.get('target_ratio'),
        'presence_class': scenario.get('presence_class'),
        'occlusion_class': scenario['occlusion_class'],
        'occlusion_evidence_type': scenario['occlusion_evidence_type'],
        'occlusion_claim_scope': scenario['occlusion_claim_scope'],
        'render_bucket': scenario['render_variation']['bucket'],
        'hard_case_tags': scenario.get('hard_case_tags', []),
        'imported_from_v3': False,
        'source_scenario_id': None,
        'source_manifest_sha256': None,
        'v4_final_test_role': DATASET_ROLE,
    }


def observed_support_summary(
    model_rows: list[dict[str, Any]],
    label_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    labels_by_sample = {row['sample_id']: row for row in label_rows}
    visible_identity = Counter()
    visible_side_segment = Counter()
    visible_position_bin = Counter()
    records_occlusion = Counter()
    records_density = Counter()
    visible_target_zone = Counter()
    visible_identity_zone = Counter()
    for model_row in model_rows:
        sample_id = model_row['sample_id']
        label_row = labels_by_sample[sample_id]
        trace = model_row['traceability_metadata']
        present = _present_slots(label_row)
        visible = [
            shuttle for shuttle in present if bool(shuttle.get('visually_available'))
        ]
        records_occlusion[str(trace['occlusion_class'])] += 1
        records_density[_presence_density(len(present))] += 1
        target = str(trace['target_identity'])
        visible_ids = {str(shuttle['id']) for shuttle in visible}
        if target in visible_ids:
            visible_target_zone[str(trace['target_zone'])] += 1
        for shuttle in visible:
            identity = str(shuttle['id'])
            side = identity_side(identity)
            block = str((shuttle.get('location') or {}).get('block'))
            ratio = float((shuttle.get('rail_position') or {}).get('s_ratio'))
            zone = str(trace['identity_to_zone'][identity])
            visible_identity[identity] += 1
            visible_side_segment[(side, block)] += 1
            visible_position_bin[position_bin(ratio)] += 1
            visible_identity_zone[(identity, zone)] += 1
    return {
        'visible_by_identity': dict(sorted(visible_identity.items())),
        'visible_by_side_x_segment': _nested_counter(visible_side_segment),
        'visible_by_position_bin': dict(sorted(visible_position_bin.items())),
        'records_by_occlusion_class': dict(sorted(records_occlusion.items())),
        'records_by_presence_density': dict(sorted(records_density.items())),
        'visible_by_target_zone': dict(sorted(visible_target_zone.items())),
        'visible_by_identity_zone': _nested_counter(visible_identity_zone),
        'occlusion_claim_scope': (
            'static calibrated-projection risk estimate; not a measured '
            'pixel-occlusion percentage'
        ),
    }


def _captured_disjoint_audit(
    scenarios: list[dict[str, Any]],
    model_rows: list[dict[str, Any]],
    label_rows: list[dict[str, Any]],
    references: Mapping[str, Any],
    individual_hashes: Mapping[str, str],
    pair_content_hashes: Mapping[str, str],
) -> dict[str, Any]:
    static = _static_disjoint_audit(scenarios, references)
    labels_by_sample = {row['sample_id']: row for row in label_rows}
    candidates = {
        'sample_ids': {str(row['sample_id']) for row in model_rows},
        'episode_ids': {str(row['episode_id']) for row in model_rows},
        'scenario_ids': {
            str(row['traceability_metadata']['scenario_id']) for row in model_rows
        },
        'scenario_family_digests': {
            _family_digest(row['scenario_family']) for row in model_rows
        },
        'configuration_family_digests': {
            _family_digest(row['traceability_metadata']['configuration_family_id'])
            for row in model_rows
        },
        'configuration_core_family_digests': {
            _family_digest(
                row['traceability_metadata']['configuration_core_family_id']
            )
            for row in model_rows
        },
        'capture_configuration_fingerprints': {
            row['traceability_metadata']['capture_configuration_fingerprint']
            for row in model_rows
        },
        'geometry_fingerprints': {
            row['traceability_metadata']['geometry_fingerprint']
            for row in model_rows
        },
        'trajectory_fingerprints': {
            _label_trajectory_fingerprint(labels_by_sample[row['sample_id']])
            for row in model_rows
        },
        'semantic_fingerprints': {
            _label_semantic_fingerprint(labels_by_sample[row['sample_id']])
            for row in model_rows
        },
        'individual_image_sha256': set(individual_hashes.values()),
        'pair_content_sha256': set(pair_content_hashes.values()),
    }
    overlaps = {
        name: sorted(candidates[name] & references[name])
        for name in candidates
    }
    internal_uniqueness = {
        name: {
            'count': (
                len(model_rows) * 2
                if name == 'individual_image_sha256'
                else len(model_rows)
            ),
            'unique_count': len(values),
        }
        for name, values in candidates.items()
    }
    required_unique = {
        name: detail['count'] == detail['unique_count']
        for name, detail in internal_uniqueness.items()
        if name != 'individual_image_sha256'
    }
    passed = (
        static['passed']
        and not any(overlaps.values())
        and all(required_unique.values())
    )
    return {
        'schema_version': DISJOINT_AUDIT_SCHEMA,
        'dataset_role': DATASET_ROLE,
        'reference_scope': [
            'old_replay_train',
            'v3r1_train',
            'v3r1_validation',
            'v3r1_canary',
        ],
        'historical_test_accessed': False,
        'definitions': {
            'trajectory_fingerprint': (
                'sha256(canonical sorted tuples of identity, public segment, '
                'and exact nine-decimal s_ratio)'
            ),
            'semantic_fingerprint': (
                'trajectory tuple plus exact loaded/empty state'
            ),
            'pair_content_sha256': (
                "sha256(canonical_json({'left_sha256': left, "
                "'right_sha256': right}))"
            ),
        },
        'static_plan_audit': static,
        'candidate_counts': {
            name: len(values) for name, values in sorted(candidates.items())
        },
        'reference_counts': {
            name: len(references[name]) for name in sorted(candidates)
        },
        'overlap_counts': {
            name: len(values) for name, values in sorted(overlaps.items())
        },
        'overlap_examples': {
            name: values[:5] for name, values in sorted(overlaps.items()) if values
        },
        'internal_uniqueness': internal_uniqueness,
        'required_internal_uniqueness': required_unique,
        'passed': passed,
    }


def _validate_observed_support(support: Mapping[str, Any]) -> None:
    issues = []
    if set(support['visible_by_identity']) != set(IDENTITIES):
        issues.append('not all eight identities are visible')
    for side in SIDES:
        if set(support['visible_by_side_x_segment'].get(side, {})) != set(BLOCKS):
            issues.append(f'{side} does not have visible support on all 14 segments')
    if set(support['visible_by_position_bin']) != set(POSITION_BINS):
        issues.append('not all nine position bins are visibly supported')
    if set(support['records_by_occlusion_class']) != {'clear', 'partial_risk'}:
        issues.append('clear/partial-risk estimated classes are incomplete')
    if set(support['records_by_presence_density']) != {'sparse', 'medium', 'dense'}:
        issues.append('sparse/medium/dense presence support is incomplete')
    if set(support['visible_by_target_zone']) != set(TARGET_ZONES):
        issues.append('all six target zones are not visibly supported')
    observed_identity_zones = {
        zone
        for by_zone in support['visible_by_identity_zone'].values()
        for zone in by_zone
    }
    if observed_identity_zones != set(IDENTITY_ZONES):
        issues.append(
            'the eleven preregistered canonical/relation identity zones are '
            f'incomplete: {sorted(observed_identity_zones)}'
        )
    if issues:
        raise FinalTestError('captured support contract failed: ' + '; '.join(issues))


def finalize_capture(root: Path, config_path: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    locked = _verify_locked_artifacts(root, config_path)
    plan_verification = verify_plan(root, config_path, regenerate=False)
    status = capture_status(root, config_path)
    if not status['capture_complete']:
        raise FinalTestError(
            f'cannot finalize: {status["incomplete_count"]} scenarios incomplete'
        )
    final_dir = root / 'finalized'
    if final_dir.exists():
        raise FinalTestError(f'refusing to overwrite sealed finalization: {final_dir}')
    scenarios = read_jsonl(root / 'scenario_manifest.jsonl')
    raw_path = root / 'dataset' / 'meta' / 'training_events.jsonl'
    raw_rows = read_jsonl(raw_path)
    raw_by_episode = {str(row.get('episode_id')): row for row in raw_rows}
    expected_ids = {row['scenario_id'] for row in scenarios}
    if len(raw_by_episode) != len(raw_rows) or set(raw_by_episode) != expected_ids:
        raise FinalTestError(
            'captured event IDs must exactly equal the locked scenario IDs'
        )

    model_rows = []
    label_rows = []
    individual_hashes: dict[str, str] = {}
    contract_pair_hashes: dict[str, str] = {}
    pair_content_hashes: dict[str, str] = {}
    capture_pair_hashes: dict[str, str] = {}
    for index, scenario in enumerate(scenarios):
        episode_id = scenario['scenario_id']
        model_row, label_row = sanitized_visual_state_row(
            raw_by_episode[episode_id], index
        )
        trace = _final_trace(scenario)
        model_row['traceability_metadata'] = copy.deepcopy(trace)
        label_row['traceability_metadata'] = copy.deepcopy(trace)
        if (
            model_row['episode_id'] != episode_id
            or not str(model_row['sample_id']).startswith('v4_final_test_')
            or not str(model_row['scenario_family']).startswith(
                'v4_final_test_family_'
            )
        ):
            raise FinalTestError(f'{episode_id}: captured freshness prefix mismatch')
        model_rows.append(model_row)
        label_rows.append(label_row)
        per_camera = {}
        for camera in CAMERAS:
            image_ref = model_row['model_input']['overhead_images'][camera]
            image_path = root / 'dataset' / image_ref
            verification = image_valid(image_path, expected_size=(640, 480))
            digest = verification['sha256']
            per_camera[camera] = digest
            individual_hashes[f'{episode_id}:{camera}'] = digest
        sample_id = model_row['sample_id']
        contract_pair_hashes[sample_id] = contract_pair_sha(
            sample_id,
            per_camera['left_rail_rgb'],
            per_camera['right_rail_rgb'],
        )
        pair_content_hashes[sample_id] = _pair_content_sha(
            per_camera['left_rail_rgb'],
            per_camera['right_rail_rgb'],
        )
        validation_path = (
            root / 'dataset' / 'episodes' / episode_id / 'validation.json'
        )
        validation = read_json(validation_path)
        capture_pair_hashes[episode_id] = str(validation['image_pair_sha256'])

    support = observed_support_summary(model_rows, label_rows)
    _validate_observed_support(support)
    references = _reference_index(locked['config'])
    disjoint = _captured_disjoint_audit(
        scenarios,
        model_rows,
        label_rows,
        references,
        individual_hashes,
        pair_content_hashes,
    )
    if not disjoint['passed']:
        raise FinalTestError('captured train/validation/canary disjoint audit failed')

    staging = Path(tempfile.mkdtemp(prefix='.finalized.', dir=root))
    try:
        rows_path = staging / 'final_test.jsonl'
        labels_path = staging / 'final_test_visual_labels.jsonl'
        audit_path = staging / 'final_test_disjoint_audit.json'
        finalization_path = staging / 'final_test_finalization.json'
        seal_path = staging / 'final_test_seal.json'
        atomic_jsonl(rows_path, model_rows)
        atomic_jsonl(labels_path, label_rows)
        validation = validate_visual_state_rows(model_rows, labels_path)
        if validation.get('issues'):
            raise FinalTestError(
                f'final-Test row/label validation failed: {validation["issues"][:3]}'
            )
        atomic_json(audit_path, disjoint)
        finalized_at = utc_now()
        finalization = {
            'schema_version': FINALIZATION_SCHEMA,
            'dataset_role': DATASET_ROLE,
            'created_at_utc': finalized_at,
            'generation_started_at_utc': locked['preregistration'][
                'preregistered_at_utc'
            ],
            'capture_completed_at_utc': finalized_at,
            'seed': SEED,
            'scenario_count': len(model_rows),
            'image_count': len(individual_hashes),
            'configuration': copy.deepcopy(locked['lock']['configuration']),
            'rows': {
                'path': str(root / 'finalized' / rows_path.name),
                'sha256': sha256_file(rows_path),
            },
            'labels': {
                'path': str(root / 'finalized' / labels_path.name),
                'sha256': sha256_file(labels_path),
            },
            'scenario_manifest': {
                'path': str(root / 'scenario_manifest.jsonl'),
                'sha256': sha256_file(root / 'scenario_manifest.jsonl'),
            },
            'preregistration': {
                'path': str(root / 'preregistration.json'),
                'sha256': sha256_file(root / 'preregistration.json'),
            },
            'plan_lock': {
                'path': str(root / 'plan_lock.json'),
                'sha256': sha256_file(root / 'plan_lock.json'),
            },
            'images': {
                'individual_sha256_by_episode_camera': dict(
                    sorted(individual_hashes.items())
                ),
                'pair_sha256_by_sample_id': dict(
                    sorted(contract_pair_hashes.items())
                ),
                'pair_content_sha256_by_sample_id': dict(
                    sorted(pair_content_hashes.items())
                ),
                'capture_pair_sha256_by_episode': dict(
                    sorted(capture_pair_hashes.items())
                ),
                'individual_unique': (
                    len(individual_hashes) == len(set(individual_hashes.values()))
                ),
                'pair_unique': (
                    len(contract_pair_hashes)
                    == len(set(contract_pair_hashes.values()))
                ),
                'pair_content_unique': (
                    len(pair_content_hashes)
                    == len(set(pair_content_hashes.values()))
                ),
            },
            'support_summary': support,
            'visual_state_validation': validation,
            'disjoint_audit': {
                'path': str(root / 'finalized' / audit_path.name),
                'sha256': sha256_file(audit_path),
                'passed': disjoint['passed'],
                'overlap_counts': disjoint['overlap_counts'],
            },
            'plan_verification': {
                'manifest_sha256': plan_verification['manifest_sha256'],
                'passed': plan_verification['passed'],
            },
            'occlusion_claim_scope': support['occlusion_claim_scope'],
            'historical_test_accessed': False,
            'inference_count': 0,
            'inference_status': 'not_run',
            'passed': (
                len(model_rows) == SCENARIO_COUNT
                and len(individual_hashes) == SCENARIO_COUNT * 2
                and len(contract_pair_hashes)
                == len(set(contract_pair_hashes.values()))
                and disjoint['passed']
                and not validation.get('issues')
            ),
        }
        if not finalization['passed']:
            raise FinalTestError('finalization gates did not all pass')
        atomic_json(finalization_path, finalization)
        seal = {
            'schema_version': SEAL_SCHEMA,
            'dataset_role': DATASET_ROLE,
            'sealed_at_utc': utc_now(),
            'scenario_count': SCENARIO_COUNT,
            'artifacts': {
                rows_path.name: sha256_file(rows_path),
                labels_path.name: sha256_file(labels_path),
                audit_path.name: sha256_file(audit_path),
                finalization_path.name: sha256_file(finalization_path),
                '../scenario_manifest.jsonl': sha256_file(
                    root / 'scenario_manifest.jsonl'
                ),
                '../preregistration.json': sha256_file(
                    root / 'preregistration.json'
                ),
                '../plan_lock.json': sha256_file(root / 'plan_lock.json'),
            },
            'image_hash_map_sha256': object_sha256(finalization['images']),
            'inference_count': 0,
            'inference_status': 'not_run',
            'passed': True,
        }
        atomic_json(seal_path, seal)
        for path in staging.iterdir():
            path.chmod(0o444)
        os.replace(staging, final_dir)
    except BaseException:
        if staging.exists():
            shutil.rmtree(staging)
        raise
    return read_json(final_dir / 'final_test_finalization.json')


def verify_seal(root: Path, config_path: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    verify_plan(root, config_path, regenerate=False)
    final_dir = root / 'finalized'
    seal_path = final_dir / 'final_test_seal.json'
    finalization_path = final_dir / 'final_test_finalization.json'
    if not seal_path.is_file() or not finalization_path.is_file():
        raise FinalTestError('sealed final-Test finalization is missing')
    seal = read_json(seal_path)
    if (
        seal.get('schema_version') != SEAL_SCHEMA
        or seal.get('dataset_role') != DATASET_ROLE
        or seal.get('inference_status') != 'not_run'
        or int(seal.get('inference_count', -1)) != 0
    ):
        raise FinalTestError('final-Test seal contract is invalid')
    for relative, expected in (seal.get('artifacts') or {}).items():
        path = (final_dir / relative).resolve()
        if not path.is_file() or sha256_file(path) != expected:
            raise FinalTestError(f'sealed artifact changed: {path}')
    finalization = read_json(finalization_path)
    if (
        finalization.get('schema_version') != FINALIZATION_SCHEMA
        or finalization.get('dataset_role') != DATASET_ROLE
        or finalization.get('inference_status') != 'not_run'
        or not finalization.get('passed')
    ):
        raise FinalTestError('finalization contract is invalid')
    if object_sha256(finalization['images']) != seal['image_hash_map_sha256']:
        raise FinalTestError('sealed image hash map changed')
    return {
        'schema_version': SEAL_SCHEMA,
        'dataset_role': DATASET_ROLE,
        'root': str(root),
        'scenario_count': finalization['scenario_count'],
        'finalization_sha256': sha256_file(finalization_path),
        'seal_sha256': sha256_file(seal_path),
        'inference_status': 'not_run',
        'passed': True,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config',
        type=Path,
        default=DEFAULT_CONFIG,
        help='Frozen V4 final-Test preregistration configuration.',
    )
    parser.add_argument(
        '--root',
        type=Path,
        default=DEFAULT_ROOT,
        help='Fresh final-Test root; plan refuses to overwrite it.',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('plan')
    verify = subparsers.add_parser('verify-plan')
    verify.add_argument(
        '--skip-deterministic-regeneration',
        action='store_true',
        help='Fast integrity check; the release gate must omit this option.',
    )
    subparsers.add_parser('capture-command')
    subparsers.add_parser('status')
    subparsers.add_parser('finalize')
    subparsers.add_parser('verify-seal')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == 'plan':
        result: Any = create_plan(args.config, output_root=args.root)
    elif args.command == 'verify-plan':
        result = verify_plan(
            args.root,
            args.config,
            regenerate=not args.skip_deterministic_regeneration,
        )
    elif args.command == 'capture-command':
        result = {
            'dataset_role': DATASET_ROLE,
            'command': capture_command(args.root, args.config),
            'note': 'Printed only; no capture or inference was started.',
            'inference_status': 'not_run',
        }
    elif args.command == 'status':
        result = capture_status(args.root, args.config)
    elif args.command == 'finalize':
        result = finalize_capture(args.root, args.config)
    else:
        result = verify_seal(args.root, args.config)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (FinalTestError, OSError, VisualV3Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
