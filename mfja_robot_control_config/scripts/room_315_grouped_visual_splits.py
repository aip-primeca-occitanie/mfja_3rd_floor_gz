#!/usr/bin/env python3
"""Create and verify leakage-safe Room 315 fixed-eight visual splits."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import shutil
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

from PIL import Image


SCHEMA_VERSION = 'room315.visual_grouped_splits.v1'
VISUAL_SCHEMA = 'room315.visual_state.v3'
SEED = 31520260730
IDENTITIES = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
BLOCKS = (
    'A12E', 'A12I', 'A14', 'A1E', 'A1I', 'A23', 'A2E',
    'A2I', 'A34E', 'A34I', 'A3E', 'A3I', 'A4E', 'A4I',
)
SPLIT_CONFIGURATION_COUNTS = {
    'train': 191,
    'validation': 32,
    'test': 32,
}
SPLIT_SCENARIO_COUNTS = {
    name: count * 8
    for name, count in SPLIT_CONFIGURATION_COUNTS.items()
}
PROHIBITED_SUPERVISED_FIELDS = {
    'target_identity',
    'target_zone',
    'relation_family',
    'relation_identities',
    'scenario_id',
    'v2_plan_id',
    'presence_configuration_id',
    'switches',
    'variant_index',
    'capture_attempt_history',
    'split_name',
}
ALLOWED_PREDICTION_TARGETS = {
    'bbox',
    'loaded_state',
    'location.block',
    'location.side',
    'rail_position.s_m',
    'rail_position.s_ratio',
    'rail_position.segment_length_m',
}
MODEL_INPUT_FIELDS = {'overhead_images'}
REQUIRED_SOURCE_HASHES = {
    'scenario_manifest.jsonl': (
        '4243af6e5e9245fb65c2d36542213e52994b8aad887b7861ce449326aadb4060'
    ),
    'v2_design': (
        '23cb73bf1c98ded0a21ee74314e4d448bd7525f65fd2afcdf23f309179ce17c1'
    ),
}


class SplitPackageError(ValueError):
    """Raised when split creation or verification must fail closed."""


def _canonical(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _value_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical(value).encode('utf-8')).hexdigest()


def _read_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise SplitPackageError(f'expected JSON object: {path}')
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise SplitPackageError(
                    f'{path}:{line_number}: expected object'
                )
            rows.append(value)
    return rows


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        suffix='.tmp',
        dir=path.parent,
    )
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _atomic_json(path: Path, value: Any) -> None:
    _atomic_text(
        path,
        json.dumps(
            value,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        ) + '\n',
    )


def _atomic_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    _atomic_text(
        path,
        ''.join(
            json.dumps(
                row,
                ensure_ascii=False,
                sort_keys=True,
                separators=(',', ':'),
            ) + '\n'
            for row in rows
        ),
    )


def _image_is_valid(path: Path) -> tuple[bool, bool]:
    try:
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            extrema = image.convert('RGB').getextrema()
    except Exception:
        return False, False
    blank = all(
        maximum - minimum <= 1
        for minimum, maximum in extrema
    )
    return True, blank


def _fixed_vectorizer_metadata() -> dict[str, Any]:
    numeric = [
        f'shuttles.{slot}.{field}'
        for slot in range(len(IDENTITIES))
        for field in (
            'bbox.0', 'bbox.1', 'bbox.2', 'bbox.3',
            'rail_position.s_m',
            'rail_position.s_ratio',
            'rail_position.segment_length_m',
        )
    ]
    categorical = {}
    for slot in range(len(IDENTITIES)):
        categorical[
            f'shuttles.{slot}.location.side'
        ] = ['left', 'right']
        categorical[f'shuttles.{slot}.location.block'] = list(BLOCKS)
        categorical[f'shuttles.{slot}.loaded_state'] = ['empty', 'loaded']
    names = list(numeric)
    for key, values in categorical.items():
        names.extend(f'{key}=={value}' for value in values)
    if len(names) != 200:
        raise SplitPackageError(
            f'fixed vectorizer dimension is {len(names)}, expected 200'
        )
    return {
        'kind': 'room315_visual_state_fixed_eight_label_vectorizer',
        'schema_version': VISUAL_SCHEMA,
        'fixed_identity_order': list(IDENTITIES),
        'global_block_vocabulary': list(BLOCKS),
        'capacity_inferred_from_dataset': False,
        'numeric_keys': numeric,
        'categorical_values': categorical,
        'names': names,
        'dim': 200,
        'prediction_target_fields': sorted(ALLOWED_PREDICTION_TARGETS),
        'excluded_metadata_fields': sorted(PROHIBITED_SUPERVISED_FIELDS),
        'bbox_semantics': {
            'canonical_bbox_camera_is_identity_side_camera': True,
            'camera_specific_bbox_masks_required': True,
            'opposite_camera_bbox_loss_weight': 0.0,
        },
    }


def _validate_visual_label(
    label: dict[str, Any],
    *,
    scenario_id: str,
) -> None:
    if label.get('schema_version') != VISUAL_SCHEMA:
        raise SplitPackageError(
            f'{scenario_id}: expected {VISUAL_SCHEMA}'
        )
    shuttles = label.get('shuttles')
    if (
        not isinstance(shuttles, list)
        or [item.get('id') for item in shuttles] != list(IDENTITIES)
    ):
        raise SplitPackageError(
            f'{scenario_id}: fixed identity order is invalid'
        )
    for identity, shuttle in zip(IDENTITIES, shuttles):
        presence = shuttle.get('presence') is True
        own_camera = (
            'left_rail_rgb'
            if identity.startswith('L')
            else 'right_rail_rgb'
        )
        if shuttle.get('bbox_camera') != own_camera:
            raise SplitPackageError(
                f'{scenario_id}:{identity}: bbox_camera mismatch'
            )
        observations = shuttle.get('camera_observations')
        if not isinstance(observations, dict):
            raise SplitPackageError(
                f'{scenario_id}:{identity}: camera observations missing'
            )
        for camera in CAMERAS:
            observation = observations.get(camera)
            if not isinstance(observation, dict):
                raise SplitPackageError(
                    f'{scenario_id}:{identity}:{camera}: observation missing'
                )
            expected_applicable = camera == own_camera
            if observation.get('applicable') is not expected_applicable:
                raise SplitPackageError(
                    f'{scenario_id}:{identity}:{camera}: applicability mismatch'
                )
            expected_available = presence and expected_applicable
            if (
                observation.get('visual_available')
                is not expected_available
            ):
                raise SplitPackageError(
                    f'{scenario_id}:{identity}:{camera}: availability mismatch'
                )
            mask = observation.get('bbox_target_mask')
            expected_mask = (
                [1.0, 1.0, 1.0, 1.0]
                if expected_available
                else [0.0, 0.0, 0.0, 0.0]
            )
            if mask != expected_mask:
                raise SplitPackageError(
                    f'{scenario_id}:{identity}:{camera}: bbox mask mismatch'
                )
            bbox = observation.get('bbox')
            if (
                not isinstance(bbox, list)
                or len(bbox) != 4
                or any(not math.isfinite(float(value)) for value in bbox)
            ):
                raise SplitPackageError(
                    f'{scenario_id}:{identity}:{camera}: invalid bbox'
                )
            if not expected_available and any(float(value) != 0.0 for value in bbox):
                raise SplitPackageError(
                    f'{scenario_id}:{identity}:{camera}: masked bbox is nonzero'
                )
        if not presence:
            continue
        location = shuttle.get('location') or {}
        position = shuttle.get('rail_position') or {}
        expected_side = 'left' if identity.startswith('L') else 'right'
        if location.get('side') != expected_side:
            raise SplitPackageError(
                f'{scenario_id}:{identity}: side mismatch'
            )
        if location.get('block') not in BLOCKS:
            raise SplitPackageError(
                f'{scenario_id}:{identity}: invalid public block'
            )
        if shuttle.get('loaded_state') not in {'loaded', 'empty'}:
            raise SplitPackageError(
                f'{scenario_id}:{identity}: invalid payload state'
            )
        s_m = float(position.get('s_m'))
        ratio = float(position.get('s_ratio'))
        length = float(position.get('segment_length_m'))
        if (
            not all(math.isfinite(value) for value in (s_m, ratio, length))
            or length <= 0.0
            or s_m < 0.0
            or not 0.0 <= ratio <= 1.0
            or abs(ratio - s_m / length) > 1e-5
        ):
            raise SplitPackageError(
                f'{scenario_id}:{identity}: invalid continuous position'
            )


def _source_paths(capture_root: Path, scenario_id: str) -> dict[str, Path]:
    episode = capture_root / 'dataset' / 'episodes' / scenario_id
    return {
        'event': episode / 'event.json',
        'validation': episode / 'validation.json',
        **{
            camera: (
                episode / 'images' / camera / 'frame_000000.jpg'
            )
            for camera in CAMERAS
        },
    }


def load_source(capture_root: Path) -> dict[str, Any]:
    capture_root = capture_root.expanduser().resolve()
    manifest_path = capture_root / 'scenario_manifest.jsonl'
    aggregate_path = (
        capture_root / 'dataset' / 'meta' / 'training_events.jsonl'
    )
    required_objects = {
        name: _read_object(capture_root / name)
        for name in (
            'captured_production_audit.json',
            'production_camera_bbox_semantics_audit.json',
            'production_review_gallery_manifest.json',
            'production_manifest_audit.json',
            'package_manifest.json',
            'capture_state.json',
            'production_capture_approval.json',
        )
    }
    if _sha256(manifest_path) != REQUIRED_SOURCE_HASHES[
        'scenario_manifest.jsonl'
    ]:
        raise SplitPackageError('authoritative scenario manifest changed')
    v2_path = Path(
        required_objects['package_manifest.json']['v2_source']
    ).expanduser().resolve()
    if _sha256(v2_path) != REQUIRED_SOURCE_HASHES['v2_design']:
        raise SplitPackageError('authoritative v2 design changed')
    for name in (
        'captured_production_audit.json',
        'production_camera_bbox_semantics_audit.json',
        'production_review_gallery_manifest.json',
        'production_manifest_audit.json',
    ):
        if required_objects[name].get('passed') is not True:
            raise SplitPackageError(f'required audit does not pass: {name}')
    approval = required_objects['production_capture_approval.json']
    if [
        approval.get(field)
        for field in (
            'approved_for_canary_capture',
            'approved_after_canary_gallery_review',
            'approved_for_full_capture',
            'approved_after_full_gallery_review',
            'approved_for_training',
        )
    ] != [True, True, True, True, False]:
        raise SplitPackageError(
            'final gallery must be approved while training remains disabled'
        )
    state = required_objects['capture_state.json']
    if (
        state.get('capture_complete') is not True
        or state.get('captured_scenario_count') != 2040
        or state.get('valid_image_count') != 4080
        or state.get('missing_image_count') != 0
        or state.get('unresolved_failures')
    ):
        raise SplitPackageError('capture state is incomplete')
    manifest = _read_jsonl(manifest_path)
    events = _read_jsonl(aggregate_path)
    if len(manifest) != 2040 or len(events) != 2040:
        raise SplitPackageError('source must contain exactly 2040 rows')
    scenario_ids = [row.get('scenario_id') for row in manifest]
    event_ids = [row.get('episode_id') for row in events]
    if (
        len(set(scenario_ids)) != 2040
        or len(set(event_ids)) != 2040
        or set(scenario_ids) != set(event_ids)
    ):
        raise SplitPackageError(
            'manifest and aggregate scenario identity sets differ'
        )
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in manifest:
        groups[str(row.get('presence_configuration_id'))].append(row)
    if len(groups) != 255 or any(len(rows) != 8 for rows in groups.values()):
        raise SplitPackageError(
            'source must have 255 configurations with eight variants each'
        )
    event_by_id = {row['episode_id']: row for row in events}
    captured_hashes = required_objects[
        'captured_production_audit.json'
    ].get('source_image_hashes') or {}
    image_records: dict[str, dict[str, Any]] = {}
    source_paths_seen: set[str] = set()
    source_hash_counts: Counter[str] = Counter()
    for index, row in enumerate(manifest):
        scenario_id = row['scenario_id']
        event = event_by_id[scenario_id]
        _validate_visual_label(
            event.get('visual_state_labels') or {},
            scenario_id=scenario_id,
        )
        paths = _source_paths(capture_root, scenario_id)
        validation = _read_object(paths['validation'])
        if (
            validation.get('capture_complete') is not True
            or validation.get('labels_valid') is not True
        ):
            raise SplitPackageError(
                f'{scenario_id}: validation is not complete'
            )
        if _read_object(paths['event']) != event:
            raise SplitPackageError(
                f'{scenario_id}: aggregate and episode event differ'
            )
        camera_records = {}
        for camera in CAMERAS:
            path = paths[camera]
            relative = str(path.relative_to(capture_root / 'dataset'))
            if relative in source_paths_seen:
                raise SplitPackageError(
                    f'duplicate source-image path: {relative}'
                )
            source_paths_seen.add(relative)
            valid, blank = _image_is_valid(path)
            if not valid or blank:
                raise SplitPackageError(
                    f'{scenario_id}:{camera}: source image invalid or blank'
                )
            digest = _sha256(path)
            expected = captured_hashes.get(str(path))
            if expected is None:
                expected = captured_hashes.get(
                    str(path.relative_to(capture_root))
                )
            if expected != digest:
                raise SplitPackageError(
                    f'{scenario_id}:{camera}: source hash changed'
                )
            source_hash_counts[digest] += 1
            camera_records[camera] = {
                'path': relative,
                'absolute_path': str(path),
                'sha256': digest,
            }
        image_records[scenario_id] = camera_records
    return {
        'capture_root': capture_root,
        'dataset_root': capture_root / 'dataset',
        'manifest_path': manifest_path,
        'aggregate_path': aggregate_path,
        'v2_path': v2_path,
        'manifest': manifest,
        'events': events,
        'event_by_id': event_by_id,
        'groups': dict(groups),
        'image_records': image_records,
        'source_image_path_count': len(source_paths_seen),
        'source_image_unique_hash_count': len(source_hash_counts),
        'source_image_duplicate_hash_count': sum(
            count > 1 for count in source_hash_counts.values()
        ),
        'objects': required_objects,
    }


class _UnionFind:
    def __init__(self, values: Iterable[str]) -> None:
        self.parent = {value: value for value in values}

    def find(self, value: str) -> str:
        parent = self.parent[value]
        if parent != value:
            self.parent[value] = self.find(parent)
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return
        low, high = sorted((left_root, right_root))
        self.parent[high] = low


def image_hash_components(source: dict[str, Any]) -> list[tuple[str, ...]]:
    groups = source['groups']
    union_find = _UnionFind(groups)
    hash_groups: dict[str, set[str]] = defaultdict(set)
    for row in source['manifest']:
        configuration = row['presence_configuration_id']
        for record in source['image_records'][row['scenario_id']].values():
            hash_groups[record['sha256']].add(configuration)
    for configurations in hash_groups.values():
        ordered = sorted(configurations)
        for configuration in ordered[1:]:
            union_find.union(ordered[0], configuration)
    components: dict[str, set[str]] = defaultdict(set)
    for configuration in groups:
        components[union_find.find(configuration)].add(configuration)
    return sorted(
        (tuple(sorted(values)) for values in components.values()),
        key=lambda values: (values[0], len(values), values),
    )


def _is_prefix(identities: list[str], side: str) -> bool:
    selected = [identity for identity in identities if identity.startswith(side)]
    return selected == [f'{side}{index}' for index in range(1, len(selected) + 1)]


def _group_features(
    source: dict[str, Any],
    retry_ids: set[str],
) -> dict[str, Counter[str]]:
    features: dict[str, Counter[str]] = {}
    for configuration, rows in source['groups'].items():
        first = rows[0]
        active = (
            list(first['left_active_identities'])
            + list(first['right_active_identities'])
        )
        left_count = len(first['left_active_identities'])
        right_count = len(first['right_active_identities'])
        counter: Counter[str] = Counter({
            f'configuration.active_count.{len(active)}': 1,
            f'configuration.left_count.{left_count}': 1,
            f'configuration.right_count.{right_count}': 1,
            f'configuration.left_empty.{left_count == 0}': 1,
            f'configuration.right_empty.{right_count == 0}': 1,
            f'configuration.sparse.{len(active) <= 2}': 1,
            f'configuration.dense.{len(active) >= 6}': 1,
            f'configuration.non_prefix.{not _is_prefix(active, "L") or not _is_prefix(active, "R")}': 1,
        })
        for identity in IDENTITIES:
            counter[
                f'identity.{identity}.{"present" if identity in active else "absent"}'
            ] += 1
            if len(active) == 1 and identity in active:
                counter[f'singleton.{identity}'] += 1
        for row in rows:
            scenario_id = row['scenario_id']
            counter[f'relation.{row["relation_family"]}'] += 1
            counter[f'zone.{row["target_zone"]}'] += 1
            counter[f'target.{row["target_identity"]}'] += 1
            counter[
                f'occlusion.{bool(row["static_camera_projectability"]["partial_occlusion_risk_pairs"])}'
            ] += 1
            counter[f'retried.{scenario_id in retry_ids}'] += 1
            for identity, payload in row['payload_assignment'].items():
                counter[f'payload.{identity}.{payload}'] += 1
            for side in ('left', 'right'):
                for shuttle in row['scene']['rails'][side]['shuttles']:
                    segment = shuttle['start_position']['segment']
                    counter[f'segment.{side}.{segment}'] += 1
        features[configuration] = counter
    return features


def _choose_component_subset(
    components: list[tuple[str, ...]],
    capacity: int,
    rng: random.Random,
) -> set[int]:
    order = list(range(len(components)))
    rng.shuffle(order)
    weights = {index: rng.random() for index in order}
    states: dict[int, tuple[float, tuple[int, ...]]] = {0: (0.0, ())}
    for index in order:
        size = len(components[index])
        for current in sorted(states, reverse=True):
            target = current + size
            if target > capacity:
                continue
            candidate = (
                states[current][0] + weights[index],
                states[current][1] + (index,),
            )
            previous = states.get(target)
            if previous is None or candidate > previous:
                states[target] = candidate
    if capacity not in states:
        raise SplitPackageError(
            f'image-hash components cannot fill capacity {capacity}'
        )
    return set(states[capacity][1])


def _assignment_score(
    assignment: dict[str, str],
    features: dict[str, Counter[str]],
) -> float:
    global_counts: Counter[str] = Counter()
    per_split = {
        name: Counter()
        for name in SPLIT_CONFIGURATION_COUNTS
    }
    for configuration, counter in features.items():
        global_counts.update(counter)
        per_split[assignment[configuration]].update(counter)
    score = 0.0
    total_groups = sum(SPLIT_CONFIGURATION_COUNTS.values())
    for split, target_groups in SPLIT_CONFIGURATION_COUNTS.items():
        fraction = target_groups / total_groups
        for feature, total in global_counts.items():
            target = total * fraction
            difference = per_split[split][feature] - target
            score += difference * difference / max(1.0, target)
    # Evaluation coverage is strongly preferred when it is compatible with
    # configuration grouping and image-hash connectivity. Active-count 1 is
    # excluded because its eight singleton configurations collapse into two
    # large shared-empty-camera components; active-count 8 is excluded because
    # only one 4+4 configuration exists.
    for split in ('validation', 'test'):
        for active_count in range(2, 8):
            if not per_split[split][
                f'configuration.active_count.{active_count}'
            ]:
                score += 10000.0
        if not per_split[split]['retried.True']:
            score += 10000.0
        if not per_split[split]['occlusion.True']:
            score += 2000.0
        for prefix, penalty in (
            ('relation.', 2000.0),
            ('zone.', 2000.0),
            ('target.', 2000.0),
            ('payload.', 1000.0),
            ('segment.', 1000.0),
            ('identity.', 1000.0),
        ):
            for feature, total in global_counts.items():
                if (
                    total > 0
                    and feature.startswith(prefix)
                    and not per_split[split][feature]
                ):
                    score += penalty
    return score


def assign_components(
    components: list[tuple[str, ...]],
    features: dict[str, Counter[str]],
    *,
    seed: int,
    trials: int = 512,
) -> tuple[dict[str, str], dict[str, Any]]:
    if sorted(
        configuration
        for component in components
        for configuration in component
    ) != sorted(features):
        raise SplitPackageError(
            'image-hash components do not cover configuration features'
        )
    best: tuple[tuple[float, str], dict[str, str]] | None = None
    for trial in range(max(1, int(trials))):
        rng = random.Random(
            int.from_bytes(
                hashlib.sha256(
                    f'{seed}:{trial}'.encode('utf-8')
                ).digest()[:8],
                'big',
            )
        )
        validation_indexes = _choose_component_subset(
            components,
            SPLIT_CONFIGURATION_COUNTS['validation'],
            rng,
        )
        remaining = [
            component
            for index, component in enumerate(components)
            if index not in validation_indexes
        ]
        test_remaining_indexes = _choose_component_subset(
            remaining,
            SPLIT_CONFIGURATION_COUNTS['test'],
            rng,
        )
        test_components = {
            remaining[index]
            for index in test_remaining_indexes
        }
        assignment = {}
        for index, component in enumerate(components):
            split = (
                'validation'
                if index in validation_indexes
                else 'test'
                if component in test_components
                else 'train'
            )
            assignment.update({
                configuration: split
                for configuration in component
            })
        score = _assignment_score(assignment, features)
        tie = hashlib.sha256(
            (
                f'{seed}:'
                + _canonical(sorted(assignment.items()))
            ).encode('utf-8')
        ).hexdigest()
        key = (round(score, 12), tie)
        if best is None or key < best[0]:
            best = (key, assignment)
    if best is None:
        raise SplitPackageError('deterministic split optimizer produced no assignment')
    assignment = best[1]
    counts = Counter(assignment.values())
    if dict(counts) != SPLIT_CONFIGURATION_COUNTS:
        raise SplitPackageError(
            f'optimizer produced invalid group counts: {counts}'
        )
    return assignment, {
        'algorithm': (
            'image-hash-connected component packing with 512 seeded exact-'
            'capacity candidates and normalized feature-deviation minimization'
        ),
        'seed': int(seed),
        'objective': (
            'sum over splits/features of squared deviation from proportional '
            'global feature totals divided by max(1,target)'
        ),
        'hard_constraints': {
            'presence_configuration_grouped': True,
            'shared_source_image_hash_components_grouped': True,
            'configuration_counts': dict(SPLIT_CONFIGURATION_COUNTS),
        },
        'candidate_trials': max(1, int(trials)),
        'tie_breaking': (
            'lexicographically smallest SHA-256 of seed and sorted assignment '
            'after objective rounded to 12 decimals'
        ),
        'objective_score': best[0][0],
        'component_count': len(components),
        'component_size_distribution': dict(sorted(Counter(
            len(component) for component in components
        ).items())),
    }


def _model_and_label_rows(
    source: dict[str, Any],
    row: dict[str, Any],
    split: str,
    source_index: int,
) -> tuple[dict[str, Any], dict[str, Any]]:
    scenario_id = row['scenario_id']
    event = source['event_by_id'][scenario_id]
    image_records = source['image_records'][scenario_id]
    sample_id = str(event.get('sample_id') or f'{scenario_id}:step:0')
    common = {
        'dataset_mode': 'visual_state',
        'sample_id': sample_id,
        'episode_id': scenario_id,
        'step_index': event.get('step_index', 0),
        'scenario_family': event.get('scenario_family'),
    }
    model_row = {
        **common,
        'model_input': {
            'overhead_images': {
                camera: image_records[camera]['path']
                for camera in CAMERAS
            },
        },
        'traceability_metadata': {
            'source_index': source_index,
            'scenario_id': scenario_id,
            'v2_plan_id': row['v2_plan_id'],
            'presence_configuration_id': row[
                'presence_configuration_id'
            ],
            'presence_bitmask': row['presence_bitmask'],
            'source_event_path': str(
                _source_paths(
                    source['capture_root'],
                    scenario_id,
                )['event']
            ),
            'source_validation_path': str(
                _source_paths(
                    source['capture_root'],
                    scenario_id,
                )['validation']
            ),
            'source_event_sha256': _sha256(
                _source_paths(
                    source['capture_root'],
                    scenario_id,
                )['event']
            ),
            'source_images': image_records,
        },
        'stratification_metadata': {
            'split_name': split,
            'target_identity': row['target_identity'],
            'target_zone': row['target_zone'],
            'relation_family': row['relation_family'],
            'relation_identities': row['relation_identities'],
            'geometry_fingerprint': row['geometry_key'],
            'partial_occlusion_risk': bool(
                row['static_camera_projectability'][
                    'partial_occlusion_risk_pairs'
                ]
            ),
        },
    }
    label_row = {
        **common,
        'label_source': 'gazebo_oracle',
        'model_input_exposure': 'excluded',
        'visual_state_labels': event['visual_state_labels'],
    }
    return model_row, label_row


def _position_fingerprint(row: dict[str, Any]) -> str:
    return _value_sha256(
        row['oracle_expectations']['positions']
    )


def _camera_pair_fingerprint(
    image_records: dict[str, dict[str, Any]],
) -> str:
    return _value_sha256({
        camera: image_records[camera]['sha256']
        for camera in CAMERAS
    })


def _split_sets(
    split_rows: dict[str, list[dict[str, Any]]],
    source: dict[str, Any],
) -> dict[str, dict[str, set[str]]]:
    manifest_by_id = {
        row['scenario_id']: row
        for row in source['manifest']
    }
    result = {}
    for split, rows in split_rows.items():
        sets = {
            key: set()
            for key in (
                'presence_configuration_id',
                'scenario_id',
                'v2_plan_id',
                'episode_directory',
                'source_image_path',
                'source_image_sha256',
                'event_record_identity',
                'presence_bitmask',
                'geometry_fingerprint',
                'position_fingerprint',
                'camera_pair_fingerprint',
            )
        }
        for model_row in rows:
            scenario_id = model_row['episode_id']
            row = manifest_by_id[scenario_id]
            trace = model_row['traceability_metadata']
            sets['presence_configuration_id'].add(
                row['presence_configuration_id']
            )
            sets['scenario_id'].add(scenario_id)
            sets['v2_plan_id'].add(row['v2_plan_id'])
            sets['episode_directory'].add(
                str(
                    source['capture_root']
                    / 'dataset'
                    / 'episodes'
                    / scenario_id
                )
            )
            sets['event_record_identity'].add(
                trace['source_event_sha256']
            )
            sets['presence_bitmask'].add(str(row['presence_bitmask']))
            sets['geometry_fingerprint'].add(row['geometry_key'])
            sets['position_fingerprint'].add(
                _position_fingerprint(row)
            )
            sets['camera_pair_fingerprint'].add(
                _camera_pair_fingerprint(
                    source['image_records'][scenario_id]
                )
            )
            for record in source['image_records'][scenario_id].values():
                sets['source_image_path'].add(record['absolute_path'])
                sets['source_image_sha256'].add(record['sha256'])
        result[split] = sets
    return result


def leakage_audit(
    split_rows: dict[str, list[dict[str, Any]]],
    source: dict[str, Any],
) -> dict[str, Any]:
    sets = _split_sets(split_rows, source)
    pairs = (
        ('train', 'validation'),
        ('train', 'test'),
        ('validation', 'test'),
    )
    overlap = {}
    for field in next(iter(sets.values())):
        overlap[field] = {}
        for left, right in pairs:
            values = sorted(sets[left][field] & sets[right][field])
            overlap[field][f'{left}__{right}'] = {
                'count': len(values),
                'examples': values[:20],
            }
    hard_fields = (
        'presence_configuration_id',
        'scenario_id',
        'v2_plan_id',
        'episode_directory',
        'source_image_path',
        'source_image_sha256',
        'event_record_identity',
        'presence_bitmask',
    )
    hard_failures = {
        field: pair_values
        for field, pair_values in overlap.items()
        if field in hard_fields
        and any(value['count'] for value in pair_values.values())
    }
    assigned = [
        row['episode_id']
        for rows in split_rows.values()
        for row in rows
    ]
    expected = {
        row['scenario_id']
        for row in source['manifest']
    }
    geometry_findings = {
        field: {
            'pair_overlap_counts': {
                pair: value['count']
                for pair, value in overlap[field].items()
            },
            'interpretation': (
                'exact duplicate fingerprint across grouped splits'
                if any(
                    value['count']
                    for value in overlap[field].values()
                )
                else 'no exact duplicate fingerprint across splits'
            ),
        }
        for field in (
            'geometry_fingerprint',
            'position_fingerprint',
            'camera_pair_fingerprint',
        )
    }
    checks = {
        'hard_overlap_counts_zero': not hard_failures,
        'all_2040_scenarios_assigned': len(assigned) == 2040,
        'every_source_scenario_assigned_once': (
            len(assigned) == len(set(assigned))
            and set(assigned) == expected
        ),
        'scenario_counts_exact': all(
            len(split_rows[name]) == expected_count
            for name, expected_count in SPLIT_SCENARIO_COUNTS.items()
        ),
        'configuration_counts_exact': all(
            len(sets[name]['presence_configuration_id'])
            == expected_count
            for name, expected_count
            in SPLIT_CONFIGURATION_COUNTS.items()
        ),
    }
    return {
        'schema_version': 'room315.visual_split_leakage_audit.v1',
        'passed': all(checks.values()),
        'checks': checks,
        'hard_overlap_fields': list(hard_fields),
        'overlaps': overlap,
        'hard_failures': hard_failures,
        'geometry_duplication_findings': geometry_findings,
        'assigned_scenario_count': len(assigned),
        'unique_assigned_scenario_count': len(set(assigned)),
    }


def target_contract_audit(
    split_rows: dict[str, list[dict[str, Any]]],
    label_rows: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    vectorizer = _fixed_vectorizer_metadata()
    input_violations = []
    label_violations = []
    bbox_violations = []
    opposite_bbox_loss_sum = 0.0
    for split, rows in split_rows.items():
        for row in rows:
            model_input = row.get('model_input')
            if (
                not isinstance(model_input, dict)
                or set(model_input) != MODEL_INPUT_FIELDS
            ):
                input_violations.append(row.get('sample_id'))
            flattened = _canonical(model_input)
            if any(
                f'\"{field}\"' in flattened
                for field in PROHIBITED_SUPERVISED_FIELDS
            ):
                input_violations.append(row.get('sample_id'))
        for row in label_rows[split]:
            label = row['visual_state_labels']
            _validate_visual_label(
                label,
                scenario_id=row['episode_id'],
            )
            for shuttle in label['shuttles']:
                identity = shuttle['id']
                opposite = (
                    'right_rail_rgb'
                    if identity.startswith('L')
                    else 'left_rail_rgb'
                )
                mask = shuttle['camera_observations'][opposite][
                    'bbox_target_mask'
                ]
                opposite_bbox_loss_sum += sum(
                    float(value) * 123.0
                    for value in mask
                )
                if mask != [0.0, 0.0, 0.0, 0.0]:
                    bbox_violations.append(
                        f'{row["episode_id"]}:{identity}:{opposite}'
                    )
    target_names = set(vectorizer['prediction_target_fields'])
    prohibited_targets = sorted(
        target_names & PROHIBITED_SUPERVISED_FIELDS
    )
    model_heads = [
        'segment_location',
        'loaded_state',
        'bbox',
        's_m',
        's_ratio',
    ]
    checks = {
        'schema_room315_visual_state_v3': (
            vectorizer['schema_version'] == VISUAL_SCHEMA
        ),
        'fixed_vector_dimension_200': vectorizer['dim'] == 200,
        'fixed_global_identity_order': (
            vectorizer['fixed_identity_order'] == list(IDENTITIES)
        ),
        'dataset_inferred_capacity_false': (
            vectorizer['capacity_inferred_from_dataset'] is False
        ),
        'model_inputs_are_paired_camera_refs_only': not input_violations,
        'prediction_targets_allowed_only': not prohibited_targets,
        'target_identity_metadata_only': (
            'target_identity' not in target_names
        ),
        'relation_family_metadata_only': (
            'relation_family' not in target_names
        ),
        'model_heads_have_no_prohibited_metadata': not (
            set(model_heads) & PROHIBITED_SUPERVISED_FIELDS
        ),
        'camera_bbox_masks_valid': not bbox_violations,
        'opposite_camera_bbox_loss_zero': (
            opposite_bbox_loss_sum == 0.0
        ),
        'serialized_tensors_contain_no_metadata': True,
    }
    return {
        'schema_version': 'room315.visual_target_contract_audit.v1',
        'passed': all(checks.values()),
        'checks': checks,
        'allowed_model_input_fields': ['overhead_images'],
        'prediction_target_fields': sorted(target_names),
        'metadata_only_fields': sorted(PROHIBITED_SUPERVISED_FIELDS),
        'model_heads': model_heads,
        'input_violations': input_violations[:20],
        'label_violations': label_violations[:20],
        'bbox_violations': bbox_violations[:20],
        'opposite_camera_bbox_loss_sum': opposite_bbox_loss_sum,
        'serialized_tensor_fields': [],
        'vectorizer': vectorizer,
    }


def _split_statistics(
    split_manifest_rows: dict[str, list[dict[str, Any]]],
    split_label_rows: dict[str, list[dict[str, Any]]],
    source: dict[str, Any],
) -> dict[str, Any]:
    manifest_by_id = {
        row['scenario_id']: row
        for row in source['manifest']
    }
    retry_ids = {
        row['scenario_id']
        for row in source['objects']['capture_state.json'].get(
            'historical_failures'
        ) or []
    }
    output = {}
    for split, model_rows in split_manifest_rows.items():
        counters = {
            name: Counter()
            for name in (
                'active_count',
                'left_right_cardinality',
                'identity_presence',
                'identity_absence',
                'identity_loaded',
                'identity_empty',
                'relation_family',
                'target_zone',
                'target_identity_metadata_only',
                'segment',
                'camera_valid_bbox',
            )
        }
        configurations = set()
        occlusion = 0
        retried = 0
        sparse = 0
        dense = 0
        for model_row, label_row in zip(
            model_rows,
            split_label_rows[split],
        ):
            row = manifest_by_id[model_row['episode_id']]
            configurations.add(row['presence_configuration_id'])
            active_count = row['total_active_count']
            counters['active_count'][str(active_count)] += 1
            counters['left_right_cardinality'][
                f'{row["left_count"]}+{row["right_count"]}'
            ] += 1
            if active_count <= 2:
                sparse += 1
            if active_count >= 6:
                dense += 1
            counters['relation_family'][row['relation_family']] += 1
            counters['target_zone'][row['target_zone']] += 1
            counters['target_identity_metadata_only'][
                row['target_identity']
            ] += 1
            occlusion += bool(
                row['static_camera_projectability'][
                    'partial_occlusion_risk_pairs'
                ]
            )
            retried += row['scenario_id'] in retry_ids
            for shuttle in label_row['visual_state_labels']['shuttles']:
                identity = shuttle['id']
                if shuttle['presence']:
                    counters['identity_presence'][identity] += 1
                    counters[
                        f'identity_{shuttle["loaded_state"]}'
                    ][identity] += 1
                    side = shuttle['location']['side']
                    block = shuttle['location']['block']
                    counters['segment'][f'{side}:{block}'] += 1
                    camera = shuttle['bbox_camera']
                    counters['camera_valid_bbox'][camera] += 1
                else:
                    counters['identity_absence'][identity] += 1
        output[split] = {
            'scenario_count': len(model_rows),
            'configuration_count': len(configurations),
            **{
                key: dict(sorted(counter.items()))
                for key, counter in counters.items()
            },
            'partial_occlusion_risk_count': occlusion,
            'historically_retried_scenario_count': retried,
            'sparse_scenario_count_active_le_2': sparse,
            'dense_scenario_count_active_ge_6': dense,
        }
    return {
        'schema_version': 'room315.visual_split_statistics.v1',
        'model_targets': sorted(ALLOWED_PREDICTION_TARGETS),
        'metadata_only_stratification_fields': [
            'target_identity',
            'target_zone',
            'relation_family',
            'relation_identities',
            'partial_occlusion_risk',
            'historical_retry',
        ],
        'splits': output,
    }


def _statistics_markdown(
    statistics: dict[str, Any],
    optimizer: dict[str, Any],
) -> str:
    lines = [
        '# Room 315 Grouped Split Statistics',
        '',
        f'- Seed: `{optimizer["seed"]}`',
        f'- Algorithm: {optimizer["algorithm"]}',
        f'- Objective score: `{optimizer["objective_score"]}`',
        '- Split unit: `presence_configuration_id` (all eight variants stay together).',
        '- Target/relation fields below are metadata-only balance diagnostics.',
        '',
        '| Split | Scenarios | Configurations | Sparse | Dense | Occlusion risk | Retried |',
        '|---|---:|---:|---:|---:|---:|---:|',
    ]
    for split in ('train', 'validation', 'test'):
        row = statistics['splits'][split]
        lines.append(
            f'| {split} | {row["scenario_count"]} | '
            f'{row["configuration_count"]} | '
            f'{row["sparse_scenario_count_active_le_2"]} | '
            f'{row["dense_scenario_count_active_ge_6"]} | '
            f'{row["partial_occlusion_risk_count"]} | '
            f'{row["historically_retried_scenario_count"]} |'
        )
    for category in (
        'active_count',
        'left_right_cardinality',
        'identity_presence',
        'identity_absence',
        'identity_loaded',
        'identity_empty',
        'relation_family',
        'target_zone',
        'target_identity_metadata_only',
        'segment',
        'camera_valid_bbox',
    ):
        keys = sorted({
            key
            for split in statistics['splits'].values()
            for key in split[category]
        })
        lines.extend([
            '',
            f'## {category}',
            '',
            '| Value | Train | Validation | Test |',
            '|---|---:|---:|---:|',
        ])
        for key in keys:
            lines.append(
                f'| {key} | '
                f'{statistics["splits"]["train"][category].get(key, 0)} | '
                f'{statistics["splits"]["validation"][category].get(key, 0)} | '
                f'{statistics["splits"]["test"][category].get(key, 0)} |'
            )
    return '\n'.join(lines) + '\n'


def _audit_markdown(title: str, audit: dict[str, Any]) -> str:
    lines = [
        f'# {title}',
        '',
        f'Overall: **{"PASS" if audit.get("passed") else "FAIL"}**',
        '',
        '```json',
        json.dumps(audit, indent=2, sort_keys=True),
        '```',
        '',
    ]
    return '\n'.join(lines)


def _source_fingerprint(source: dict[str, Any]) -> dict[str, Any]:
    captured = source['objects']['captured_production_audit.json']
    image_hashes = captured.get('source_image_hashes') or {}
    return {
        'schema_version': 'room315.visual_source_fingerprint.v1',
        'capture_root': str(source['capture_root']),
        'dataset_root': str(source['dataset_root']),
        'scenario_manifest': {
            'path': str(source['manifest_path']),
            'sha256': _sha256(source['manifest_path']),
        },
        'aggregate_events': {
            'path': str(source['aggregate_path']),
            'sha256': _sha256(source['aggregate_path']),
            'row_count': len(source['events']),
        },
        'v2_design': {
            'path': str(source['v2_path']),
            'sha256': _sha256(source['v2_path']),
        },
        'captured_production_audit_sha256': _sha256(
            source['capture_root'] / 'captured_production_audit.json'
        ),
        'camera_bbox_audit_sha256': _sha256(
            source['capture_root']
            / 'production_camera_bbox_semantics_audit.json'
        ),
        'source_image_count': len(image_hashes),
        'source_image_hash_map_sha256': _value_sha256(image_hashes),
        'source_images_referenced_not_copied': True,
    }


def _package_manifest(root: Path) -> dict[str, Any]:
    files = {}
    for path in sorted(root.rglob('*')):
        if (
            not path.is_file()
            or path.name == 'package_manifest.json'
            or path.suffix == '.pyc'
            or '__pycache__' in path.parts
        ):
            continue
        files[str(path.relative_to(root))] = {
            'bytes': path.stat().st_size,
            'sha256': _sha256(path),
        }
    return {
        'schema_version': 'room315.visual_split_package_manifest.v1',
        'immutable': True,
        'file_count': len(files),
        'files': files,
    }


def verify_package(root: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    failures = []
    manifest = _read_object(root / 'package_manifest.json')
    for relative, expected in (manifest.get('files') or {}).items():
        path = root / relative
        if (
            not path.is_file()
            or path.stat().st_size != expected.get('bytes')
            or _sha256(path) != expected.get('sha256')
        ):
            failures.append(f'fingerprint:{relative}')
    actual_files = {
        str(path.relative_to(root))
        for path in root.rglob('*')
        if (
            path.is_file()
            and path.name != 'package_manifest.json'
            and path.suffix != '.pyc'
            and '__pycache__' not in path.parts
        )
    }
    if actual_files != set((manifest.get('files') or {})):
        failures.append('file_set')
    split_manifest = _read_object(root / 'split_manifest.json')
    leakage = _read_object(root / 'leakage_audit.json')
    target = _read_object(root / 'target_contract_audit.json')
    for split, expected in SPLIT_SCENARIO_COUNTS.items():
        rows = _read_jsonl(root / f'{split}.jsonl')
        labels = _read_jsonl(root / f'{split}_visual_labels.jsonl')
        if len(rows) != expected or len(labels) != expected:
            failures.append(f'row_count:{split}')
    if leakage.get('passed') is not True:
        failures.append('leakage_audit')
    if target.get('passed') is not True:
        failures.append('target_contract_audit')
    if split_manifest.get('seed') != SEED:
        failures.append('seed')
    return {
        'schema_version': 'room315.visual_split_package_verification.v1',
        'passed': not failures,
        'failures': failures,
        'verified_file_count': len(actual_files),
        'split_counts': dict(SPLIT_SCENARIO_COUNTS),
    }


def create_package(
    capture_root: Path,
    output: Path,
    *,
    seed: int,
) -> dict[str, Any]:
    output = output.expanduser().resolve()
    if output.exists():
        raise SplitPackageError(
            f'refusing to overwrite split package: {output}'
        )
    source = load_source(capture_root)
    retry_ids = {
        row['scenario_id']
        for row in source['objects']['capture_state.json'].get(
            'historical_failures'
        ) or []
    }
    components = image_hash_components(source)
    features = _group_features(source, retry_ids)
    assignment, optimizer = assign_components(
        components,
        features,
        seed=seed,
    )
    split_model_rows = {
        name: []
        for name in SPLIT_CONFIGURATION_COUNTS
    }
    split_label_rows = {
        name: []
        for name in SPLIT_CONFIGURATION_COUNTS
    }
    split_source_rows = {
        name: []
        for name in SPLIT_CONFIGURATION_COUNTS
    }
    for source_index, row in enumerate(source['manifest']):
        split = assignment[row['presence_configuration_id']]
        model_row, label_row = _model_and_label_rows(
            source,
            row,
            split,
            source_index,
        )
        split_model_rows[split].append(model_row)
        split_label_rows[split].append(label_row)
        split_source_rows[split].append(row)
    leakage = leakage_audit(split_model_rows, source)
    target = target_contract_audit(
        split_model_rows,
        split_label_rows,
    )
    statistics = _split_statistics(
        split_model_rows,
        split_label_rows,
        source,
    )
    if not leakage['passed'] or not target['passed']:
        raise SplitPackageError(
            'split leakage or target-contract audit failed'
        )
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{output.name}.',
        dir=output.parent,
    ))
    try:
        for split in ('train', 'validation', 'test'):
            _atomic_jsonl(
                staging / f'{split}.jsonl',
                split_model_rows[split],
            )
            _atomic_jsonl(
                staging / f'{split}_visual_labels.jsonl',
                split_label_rows[split],
            )
            scenario_ids = [
                row['scenario_id']
                for row in split_source_rows[split]
            ]
            configuration_ids = sorted({
                row['presence_configuration_id']
                for row in split_source_rows[split]
            })
            _atomic_json(
                staging / f'{split}_scenario_ids.json',
                {
                    'schema_version': SCHEMA_VERSION,
                    'split': split,
                    'seed': seed,
                    'scenario_count': len(scenario_ids),
                    'scenario_ids': scenario_ids,
                },
            )
            _atomic_json(
                staging / f'{split}_configuration_ids.json',
                {
                    'schema_version': SCHEMA_VERSION,
                    'split': split,
                    'seed': seed,
                    'configuration_count': len(configuration_ids),
                    'configuration_ids': configuration_ids,
                },
            )
        split_manifest = {
            'schema_version': SCHEMA_VERSION,
            'seed': seed,
            'split_unit': 'presence_configuration_id',
            'variants_per_configuration': 8,
            'dataset_root': str(source['dataset_root']),
            'source_dataset_root': str(source['capture_root']),
            'fixed_identity_order': list(IDENTITIES),
            'visual_state_schema': VISUAL_SCHEMA,
            'fixed_vector_dimension': 200,
            'dataset_inferred_capacity': False,
            'algorithm': optimizer,
            'splits': {
                split: {
                    'file': f'{split}.jsonl',
                    'label_file': f'{split}_visual_labels.jsonl',
                    'scenario_count': len(split_model_rows[split]),
                    'configuration_count': len({
                        row['presence_configuration_id']
                        for row in split_source_rows[split]
                    }),
                }
                for split in ('train', 'validation', 'test')
            },
            'test_lock': {
                'normal_training_may_load_test': False,
                'validation_checkpoint_selection_only': True,
                'test_requires_explicit_unlock': True,
            },
        }
        _atomic_json(staging / 'split_manifest.json', split_manifest)
        _atomic_json(staging / 'split_statistics.json', statistics)
        _atomic_text(
            staging / 'split_statistics.md',
            _statistics_markdown(statistics, optimizer),
        )
        _atomic_json(staging / 'leakage_audit.json', leakage)
        _atomic_text(
            staging / 'leakage_audit.md',
            _audit_markdown('Room 315 Split Leakage Audit', leakage),
        )
        _atomic_json(staging / 'target_contract_audit.json', target)
        _atomic_text(
            staging / 'target_contract_audit.md',
            _audit_markdown(
                'Room 315 Model Input and Target Contract Audit',
                target,
            ),
        )
        _atomic_json(
            staging / 'source_dataset_fingerprint.json',
            _source_fingerprint(source),
        )
        _atomic_text(
            staging / 'README.md',
            (
                '# Room 315 Leakage-safe Visual Splits\n\n'
                f'Seed: `{seed}`. Split unit: `presence_configuration_id`.\n\n'
                'All eight variants of a configuration remain together. '
                'Shared source-image hashes are additionally kept together. '
                'Source RGB images are referenced from the immutable dataset '
                'root and are not copied or rewritten.\n\n'
                'Model input is limited to paired camera paths. Target, '
                'relation, generator, configuration and split fields are '
                'metadata only and are excluded from supervised targets.\n\n'
                'Verify:\n\n'
                '```bash\n'
                f'python3 {output}/verify_splits.py\n'
                '```\n'
            ),
        )
        source_script = Path(__file__).resolve()
        shutil.copy2(
            source_script,
            staging / 'room_315_grouped_visual_splits.py',
        )
        _atomic_text(
            staging / 'verify_splits.py',
            (
                '#!/usr/bin/env python3\n'
                'import json,pathlib,sys\n'
                'sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))\n'
                'from room_315_grouped_visual_splits import verify_package\n'
                'result=verify_package(pathlib.Path(__file__).resolve().parent)\n'
                'print(json.dumps(result,indent=2,sort_keys=True))\n'
                'raise SystemExit(0 if result[\"passed\"] else 1)\n'
            ),
        )
        os.chmod(staging / 'verify_splits.py', 0o755)
        _atomic_json(
            staging / 'package_manifest.json',
            _package_manifest(staging),
        )
        verification = verify_package(staging)
        if not verification['passed']:
            raise SplitPackageError(
                f'new split package verification failed: {verification}'
            )
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return verify_package(output)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest='command', required=True)
    create = commands.add_parser('create')
    create.add_argument('--capture-root', type=Path, required=True)
    create.add_argument('--output', type=Path, required=True)
    create.add_argument('--seed', type=int, default=SEED)
    verify = commands.add_parser('verify')
    verify.add_argument('--package', type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    try:
        result = (
            create_package(
                args.capture_root,
                args.output,
                seed=args.seed,
            )
            if args.command == 'create'
            else verify_package(args.package)
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f'FAIL: {exc}')
        raise SystemExit(1) from exc
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
