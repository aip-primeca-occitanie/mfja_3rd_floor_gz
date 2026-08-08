#!/usr/bin/env python3
"""Build a curated Room 315 visual-state dataset from approved Gazebo episodes.

The source recorder keeps oracle state under ``privileged_eval``.  This exporter
projects that state into the recorded overhead cameras, writes explicit visual
labels, removes exact duplicate image pairs, and materialises only retained
images into a separate dataset root.
"""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_json_io import iter_jsonl_objects
from room_315_pddl_validation_gate import load_validation_result
from room_315_pddl_validation_gate import validation_approves_training
from room_315_rail_defaults import LEFT_CALIBRATION_DEFAULTS
from room_315_rail_defaults import RIGHT_CALIBRATION_DEFAULTS
from room_315_rail_defaults import apply_rail_pose_calibration
from room_315_rail_defaults import internal_rail_segment_name_to_public
from room_315_rail_defaults import public_rail_segment_lengths
from room_315_shuttle_geometry import SHUTTLE_COLLISION_CENTER_X_M
from room_315_shuttle_geometry import SHUTTLE_COLLISION_SIZE_M
from room_315_visual_state_dataset import DATASET_MODE_VISUAL_STATE
from room_315_visual_state_dataset import VISUAL_STATE_SCHEMA_VERSION
from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_state_dataset import pretty_json
from room_315_visual_state_dataset import rows_fingerprint
from room_315_visual_state_dataset import visual_state_class_balance
from room_315_visual_state_dataset import write_jsonl


REQUIRED_CAMERAS = ('left_rail_rgb', 'right_rail_rgb')
CALIBRATION_VERSION = 'room315.gazebo_pinhole.v1'
SHUTTLE_NAME_PATTERN = re.compile(
    r'(?:room315_)?(?P<side>right|left)_shuttle_?(?P<index>[1-4])$',
    re.IGNORECASE,
)


class VisualLabelExportError(ValueError):
    """Raised when oracle data cannot safely produce a visual-state label."""


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class CameraProjection:
    side: str
    sensor_name: str
    width: int
    height: int
    horizontal_fov: float
    near_m: float
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]

    @property
    def focal_px(self) -> float:
        return self.width / (2.0 * math.tan(self.horizontal_fov / 2.0))

    def project(self, point: tuple[float, float, float]) -> tuple[float, float] | None:
        delta = tuple(point[index] - self.position[index] for index in range(3))
        camera = _matvec(_transpose(self.rotation), delta)
        forward = camera[0]
        if forward <= self.near_m:
            return None
        focal = self.focal_px
        return (
            self.width / 2.0 - focal * camera[1] / forward,
            self.height / 2.0 - focal * camera[2] / forward,
        )


@dataclass
class ExportCandidate:
    row: dict[str, Any]
    image_refs: dict[str, str]
    image_pair_fingerprint: str = ''
    label_fingerprint: str = ''


def _default_camera_model_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / 'mfja_3rd_floor_description'
        / 'models'
        / 'room315_vla_overhead_devices'
        / 'model.sdf'
    )


def _pose(raw: str | None) -> Pose3:
    values = [float(value) for value in str(raw or '').split()]
    values.extend([0.0] * (6 - len(values)))
    return Pose3(*values[:6])


def _rotation_matrix(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matmul(left, right):
    return tuple(
        tuple(
            sum(left[row][inner] * right[inner][column] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _matvec(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _compose_pose(parent: Pose3, child: Pose3) -> tuple[tuple[float, float, float], tuple]:
    parent_rotation = _rotation_matrix(parent.roll, parent.pitch, parent.yaw)
    child_rotation = _rotation_matrix(child.roll, child.pitch, child.yaw)
    child_offset = _matvec(parent_rotation, (child.x, child.y, child.z))
    position = (
        parent.x + child_offset[0],
        parent.y + child_offset[1],
        parent.z + child_offset[2],
    )
    return position, _matmul(parent_rotation, child_rotation)


def load_camera_projections(path: Path) -> dict[str, CameraProjection]:
    path = path.expanduser().resolve()
    root = ET.parse(path).getroot()
    projections: dict[str, CameraProjection] = {}
    for link in root.findall('.//link'):
        link_pose = _pose(link.findtext('pose'))
        for sensor in link.findall('sensor'):
            name = str(sensor.get('name') or '')
            side = 'right' if 'right' in name else 'left' if 'left' in name else ''
            if not side or sensor.get('type') != 'rgbd_camera':
                continue
            camera = sensor.find('camera')
            if camera is None:
                continue
            image = camera.find('image')
            clip = camera.find('clip')
            if image is None or clip is None:
                raise VisualLabelExportError(f'{name} is missing camera image/clip configuration')
            position, rotation = _compose_pose(link_pose, _pose(sensor.findtext('pose')))
            projections[side] = CameraProjection(
                side=side,
                sensor_name=name,
                width=int(image.findtext('width', default='0')),
                height=int(image.findtext('height', default='0')),
                horizontal_fov=float(camera.findtext('horizontal_fov', default='0')),
                near_m=float(clip.findtext('near', default='0.01')),
                position=position,
                rotation=rotation,
            )
    if set(projections) != {'left', 'right'}:
        raise VisualLabelExportError(f'{path} must define left and right RGB-D cameras')
    return projections


def _rail_calibration(side: str) -> dict[str, float]:
    if side == 'right':
        return RIGHT_CALIBRATION_DEFAULTS
    if side == 'left':
        return LEFT_CALIBRATION_DEFAULTS
    raise VisualLabelExportError(f'unsupported rail side: {side!r}')


def rail_pose_to_gazebo(side: str, state: dict[str, Any]) -> tuple[float, float, float, float]:
    return apply_rail_pose_calibration(
        float(state['x']),
        float(state['y']),
        float(state['z']),
        float(state['yaw']),
        _rail_calibration(side),
    )


def shuttle_bbox(
    camera: CameraProjection,
    gazebo_pose: tuple[float, float, float, float],
    *,
    margin_px: float = 4.0,
) -> list[float] | None:
    center_x, center_y, center_z, yaw = gazebo_pose
    half_x, half_y, half_z = (
        dimension / 2.0 for dimension in SHUTTLE_COLLISION_SIZE_M
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    projected = []
    for local_x in (
        SHUTTLE_COLLISION_CENTER_X_M - half_x,
        SHUTTLE_COLLISION_CENTER_X_M + half_x,
    ):
        for local_y in (-half_y, half_y):
            for local_z in (-half_z, half_z):
                world = (
                    center_x + cos_yaw * local_x - sin_yaw * local_y,
                    center_y + sin_yaw * local_x + cos_yaw * local_y,
                    center_z + local_z,
                )
                pixel = camera.project(world)
                if pixel is not None:
                    projected.append(pixel)
    if not projected:
        return None
    x_min = max(0.0, min(pixel[0] for pixel in projected) - margin_px)
    y_min = max(0.0, min(pixel[1] for pixel in projected) - margin_px)
    x_max = min(float(camera.width), max(pixel[0] for pixel in projected) + margin_px)
    y_max = min(float(camera.height), max(pixel[1] for pixel in projected) + margin_px)
    if x_max <= x_min or y_max <= y_min:
        return None
    return [
        round(x_min, 3),
        round(y_min, 3),
        round(x_max - x_min, 3),
        round(y_max - y_min, 3),
    ]


def _short_shuttle_id(entity_name: str, side_hint: str) -> tuple[str, str]:
    match = SHUTTLE_NAME_PATTERN.fullmatch(str(entity_name).strip())
    if not match:
        raise VisualLabelExportError(f'unsupported shuttle entity name: {entity_name!r}')
    side = match.group('side').lower()
    if side != side_hint:
        raise VisualLabelExportError(
            f'shuttle {entity_name!r} appears under conflicting side {side_hint!r}'
        )
    prefix = 'R' if side == 'right' else 'L'
    return side, f'{prefix}{match.group("index")}'


def _known_switches(privileged: dict[str, Any]) -> list[dict[str, Any]]:
    expert = privileged.get('expert_sensor_state')
    raw_states = expert.get('switch_states') if isinstance(expert, dict) else {}
    switches = []
    if not isinstance(raw_states, dict):
        return switches
    for side in ('right', 'left'):
        side_states = raw_states.get(side)
        if not isinstance(side_states, dict):
            continue
        for name, raw_state in sorted(side_states.items()):
            state = str(raw_state or '').strip().lower()
            if state not in {'interior', 'exterior'}:
                continue
            switches.append({
                'id': f'{side}:{str(name).upper()}',
                'state': state,
                'confidence': 1.0,
            })
    return switches


def _active_obstacles(privileged: dict[str, Any]) -> list[str]:
    status = privileged.get('supervisor_status')
    if not isinstance(status, dict):
        return []
    found: list[str] = []
    top_level = status.get('obstacles')
    if isinstance(top_level, dict):
        found.extend(str(name) for name, present in top_level.items() if bool(present))
    elif isinstance(top_level, list):
        found.extend(str(name) for name in top_level if name)
    rails = status.get('rails')
    if isinstance(rails, dict):
        for side, rail in rails.items():
            if not isinstance(rail, dict):
                continue
            raw = rail.get('obstacles') or rail.get('present_obstacles')
            if isinstance(raw, dict):
                found.extend(f'{side}:{name}' for name, present in raw.items() if bool(present))
            elif isinstance(raw, list):
                found.extend(f'{side}:{name}' for name in raw if name)
    return sorted(set(found))


def visual_labels_from_event(
    event: dict[str, Any],
    cameras: dict[str, CameraProjection],
) -> dict[str, Any]:
    privileged = event.get('privileged_eval')
    if not isinstance(privileged, dict):
        raise VisualLabelExportError('event is missing privileged_eval')
    obstacles = _active_obstacles(privileged)
    if obstacles:
        raise VisualLabelExportError(
            f'active obstacles require segmentation-backed bounding boxes: {obstacles}'
        )
    raw_states = privileged.get('raw_shuttle_states')
    if not isinstance(raw_states, dict):
        raise VisualLabelExportError('privileged_eval.raw_shuttle_states is missing')
    payload_state = privileged.get('payload_state')
    payload_by_shuttle = (
        payload_state.get('by_shuttle')
        if isinstance(payload_state, dict)
        and isinstance(payload_state.get('by_shuttle'), dict)
        else {}
    )
    segment_lengths_by_side = {
        side: public_rail_segment_lengths(side)
        for side in ('right', 'left')
    }
    shuttles = []
    for side in ('right', 'left'):
        side_states = raw_states.get(side)
        if not isinstance(side_states, dict):
            continue
        for entity_name, state in sorted(side_states.items()):
            if not isinstance(state, dict):
                continue
            _, short_id = _short_shuttle_id(entity_name, side)
            if float(state.get('z') or 0.0) <= -5.0:
                # Disabled preloaded entities are parked below the world.
                # They are absent and must not become visible examples.
                continue
            bbox = shuttle_bbox(cameras[side], rail_pose_to_gazebo(side, state))
            payload = payload_by_shuttle.get(entity_name)
            loaded_state = (
                'loaded'
                if isinstance(payload, dict) and payload.get('loaded') is True
                else 'empty'
                if isinstance(payload, dict) and payload.get('loaded') is False
                else 'unknown'
            )
            segment = str(state.get('segment') or '').strip().upper()
            segment = internal_rail_segment_name_to_public(side, segment)
            location = {'side': side}
            if segment:
                location['block'] = segment
            segment_length_m = float(
                segment_lengths_by_side[side].get(segment, 0.0)
            )
            try:
                s_m = float(state.get('s'))
            except (TypeError, ValueError):
                s_m = 0.0
            position_available = segment_length_m > 0.0 and state.get('s') not in (None, '')
            shuttles.append({
                'id': short_id,
                'presence': True,
                'visually_available': bbox is not None,
                'bbox': bbox or [0.0, 0.0, 0.0, 0.0],
                'location': location,
                'rail_position': {
                    'available': position_available,
                    's_m': round(s_m, 6) if position_available else 0.0,
                    's_ratio': (
                        round(max(0.0, min(1.0, s_m / segment_length_m)), 6)
                        if position_available
                        else 0.0
                    ),
                    'segment_length_m': (
                        round(segment_length_m, 6) if position_available else 0.0
                    ),
                    'position_uncertainty_m': 0.0,
                },
                'loaded_state': loaded_state,
                'confidence': 1.0,
            })
    return normalize_visual_state_labels({
        'visual_state_labels': {
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'calibration_version': CALIBRATION_VERSION,
            'confidence': 1.0,
            'shuttles': shuttles,
            'switches': _known_switches(privileged),
            'obstacles': [],
        }
    })


def _image_refs(event: dict[str, Any]) -> dict[str, str]:
    model_input = event.get('model_input')
    if isinstance(model_input, dict) and isinstance(model_input.get('overhead_images'), dict):
        refs = model_input['overhead_images']
    elif isinstance(event.get('image_frame_refs'), dict):
        refs = event['image_frame_refs']
    else:
        refs = {
            key.removeprefix('observation.images.'): value
            for key, value in event.items()
            if key.startswith('observation.images.')
        }
    return {
        camera: str(refs.get(camera) or '').strip()
        for camera in REQUIRED_CAMERAS
    }


def _scenario_family(validation: dict[str, Any], event: dict[str, Any]) -> str:
    raw = str(
        event.get('scenario_family')
        or validation.get('scenario_id')
        or validation.get('goal_id')
        or ''
    ).strip()
    if raw.startswith('room315-'):
        raw = raw.removeprefix('room315-')
    return raw.rsplit('_speed', 1)[0]


def _event_row(
    event: dict[str, Any],
    *,
    validation: dict[str, Any],
    episode_id: str,
    fallback_index: int,
    cameras: dict[str, CameraProjection],
) -> tuple[dict[str, Any], dict[str, str]]:
    step_index = int(event.get('step_index', event.get('event_index', fallback_index)) or 0)
    refs = _image_refs(event)
    missing = [camera for camera, ref in refs.items() if not ref]
    if missing:
        raise VisualLabelExportError(f'event is missing camera references: {missing}')
    family = _scenario_family(validation, event)
    if not family:
        raise VisualLabelExportError('event is missing scenario family metadata')
    sample_id = f'{episode_id}:step:{step_index}'
    return {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'sample_id': sample_id,
        'episode_id': episode_id,
        'step_index': step_index,
        'scenario_family': family,
        'model_input': {
            'overhead_images': refs,
        },
        'visual_state_labels': visual_labels_from_event(event, cameras),
        'oracle_label_provenance': {
            'source': 'gazebo_privileged_eval_projected_to_recorded_camera',
            'calibration_version': CALIBRATION_VERSION,
            'model_input_exposure': 'excluded_after_split',
        },
    }, refs


def _source_image_path(source_root: Path, ref: str) -> Path:
    path = Path(ref).expanduser()
    if path.is_absolute():
        return path.resolve()
    if '..' in path.parts:
        raise VisualLabelExportError(f'image reference escapes dataset root: {ref!r}')
    return (source_root / path).resolve()


def _file_digest(path: Path, cache: dict[Path, str]) -> str:
    if path not in cache:
        cache[path] = hashlib.sha256(path.read_bytes()).hexdigest()
    return cache[path]


def _annotate_fingerprints(
    candidates: list[ExportCandidate],
    source_root: Path,
) -> None:
    digest_cache: dict[Path, str] = {}
    for candidate in candidates:
        image_digests = []
        for camera in REQUIRED_CAMERAS:
            path = _source_image_path(source_root, candidate.image_refs[camera])
            if not path.is_file():
                raise VisualLabelExportError(f'missing image: {path}')
            image_digests.append(f'{camera}:{_file_digest(path, digest_cache)}')
        candidate.image_pair_fingerprint = hashlib.sha256(
            '|'.join(image_digests).encode('utf-8')
        ).hexdigest()
        candidate.label_fingerprint = hashlib.sha256(
            json.dumps(
                candidate.row['visual_state_labels'],
                ensure_ascii=False,
                sort_keys=True,
                separators=(',', ':'),
            ).encode('utf-8')
        ).hexdigest()


def curate_exact_image_pairs(
    candidates: list[ExportCandidate],
) -> tuple[list[ExportCandidate], dict[str, Any]]:
    groups: dict[str, list[ExportCandidate]] = defaultdict(list)
    for candidate in candidates:
        groups[candidate.image_pair_fingerprint].append(candidate)
    retained = []
    duplicate_rows_removed = 0
    conflicting_rows_removed = 0
    conflicting_groups = 0
    conflict_examples = []
    for fingerprint, group in sorted(groups.items()):
        label_fingerprints = {candidate.label_fingerprint for candidate in group}
        if len(label_fingerprints) > 1:
            conflicting_groups += 1
            conflicting_rows_removed += len(group)
            if len(conflict_examples) < 20:
                conflict_examples.append({
                    'image_pair_fingerprint': fingerprint,
                    'samples': [candidate.row['sample_id'] for candidate in group[:8]],
                    'label_variants': len(label_fingerprints),
                })
            continue
        retained.append(min(group, key=lambda candidate: candidate.row['sample_id']))
        duplicate_rows_removed += len(group) - 1
    retained.sort(key=lambda candidate: candidate.row['sample_id'])
    return retained, {
        'source_candidates': len(candidates),
        'unique_image_pair_groups': len(groups),
        'retained_rows': len(retained),
        'duplicate_rows_removed': duplicate_rows_removed,
        'conflicting_groups_removed': conflicting_groups,
        'conflicting_rows_removed': conflicting_rows_removed,
        'conflict_examples': conflict_examples,
        'cross_split_exact_pair_policy': 'one retained row per exact image pair',
    }


def _materialize_file(source: Path, destination: Path) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, destination)
        return 'hardlink'
    except OSError:
        shutil.copy2(source, destination)
        return 'copy'


def _materialize_candidates(
    candidates: list[ExportCandidate],
    source_root: Path,
    output_root: Path,
) -> dict[str, Any]:
    methods = Counter()
    materialized: set[Path] = set()
    episode_families: dict[str, str] = {}
    for candidate in candidates:
        episode_id = str(candidate.row['episode_id'])
        episode_families[episode_id] = str(candidate.row['scenario_family'])
        for ref in candidate.image_refs.values():
            relative = Path(ref)
            if relative.is_absolute() or '..' in relative.parts:
                raise VisualLabelExportError(
                    f'cannot materialise non-relative image reference: {ref!r}'
                )
            destination = output_root / relative
            if destination in materialized:
                continue
            methods[_materialize_file(_source_image_path(source_root, ref), destination)] += 1
            materialized.add(destination)
    for episode_id, scenario_family in sorted(episode_families.items()):
        source = source_root / 'episodes' / episode_id / 'validation.json'
        destination = output_root / 'episodes' / episode_id / 'validation.json'
        if not source.is_file():
            raise VisualLabelExportError(f'missing validation file: {source}')
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            pretty_json({
                'schema_version': 'room315.visual_capture_validation.v1',
                'episode_id': episode_id,
                'scenario_family': scenario_family,
                'validation_status': 'approved',
                'approved_for_training': True,
                'capture_complete': True,
                'labels_valid': True,
                'required_cameras': list(REQUIRED_CAMERAS),
                'label_source': (
                    'gazebo_privileged_eval_projected_to_recorded_camera'
                ),
            }) + '\n',
            encoding='utf-8',
        )
    return {
        'images': len(materialized),
        'episodes': len(episode_families),
        'visual_capture_validations_generated': len(episode_families),
        'methods': dict(sorted(methods.items())),
    }


def export_visual_dataset(
    source_root: Path,
    output_root: Path,
    *,
    camera_model_sdf: Path | None = None,
) -> dict[str, Any]:
    source_root = source_root.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    if not (source_root / 'episodes').is_dir():
        raise FileNotFoundError(f'source dataset has no episodes directory: {source_root}')
    if output_root.exists() and any(output_root.iterdir()):
        raise FileExistsError(f'output dataset is not empty: {output_root}')
    output_root.mkdir(parents=True, exist_ok=True)
    cameras = load_camera_projections(camera_model_sdf or _default_camera_model_path())
    candidates: list[ExportCandidate] = []
    skipped = Counter()
    approved_episodes = 0
    for episode_dir in sorted((source_root / 'episodes').glob('episode_*')):
        validation = load_validation_result(episode_dir)
        if not validation_approves_training(validation):
            skipped['unapproved_episode'] += 1
            continue
        event_file = episode_dir / 'events.jsonl'
        if not event_file.is_file():
            skipped['missing_events_file'] += 1
            continue
        approved_episodes += 1
        for fallback_index, event in enumerate(
            iter_jsonl_objects(
                event_file,
                error_type=VisualLabelExportError,
                require_object=True,
            )
        ):
            try:
                row, refs = _event_row(
                    event,
                    validation=validation or {},
                    episode_id=episode_dir.name,
                    fallback_index=fallback_index,
                    cameras=cameras,
                )
            except VisualLabelExportError as exc:
                skipped[str(exc)] += 1
                continue
            candidates.append(
                ExportCandidate(
                    row=row,
                    image_refs=refs,
                )
            )
    if not candidates:
        raise VisualLabelExportError('no approved visual-state candidates were produced')
    _annotate_fingerprints(candidates, source_root)
    retained, curation = curate_exact_image_pairs(candidates)
    if not retained:
        raise VisualLabelExportError('exact-image curation removed every candidate')
    materialization = _materialize_candidates(retained, source_root, output_root)
    rows = [candidate.row for candidate in retained]
    output_file = output_root / 'meta' / 'training_events.jsonl'
    write_jsonl(output_file, rows)
    labels = [row['visual_state_labels'] for row in rows]
    summary = {
        'tool': 'room_315_visual_label_exporter',
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'schema_version': VISUAL_STATE_SCHEMA_VERSION,
        'calibration_version': CALIBRATION_VERSION,
        'source_dataset': str(source_root),
        'output_dataset': str(output_root),
        'training_events': str(output_file),
        'approved_source_episodes': approved_episodes,
        'retained_episodes': len({row['episode_id'] for row in rows}),
        'retained_families': len({row['scenario_family'] for row in rows}),
        'rows': len(rows),
        'row_fingerprint': rows_fingerprint(rows),
        'class_balance': visual_state_class_balance(labels),
        'curation': curation,
        'materialization': materialization,
        'skipped': dict(sorted(skipped.items())),
        'camera_projections': {
            side: {
                **asdict(camera),
                'focal_px': round(camera.focal_px, 6),
            }
            for side, camera in sorted(cameras.items())
        },
        'quality_gate': {
            'approved_episodes_only': True,
            'oracle_outside_model_input': True,
            'required_camera_refs': list(REQUIRED_CAMERAS),
            'missing_images': 0,
            'exact_duplicate_pairs_retained': 0,
            'conflicting_exact_pairs_retained': 0,
            'active_obstacles_without_bbox_retained': 0,
        },
    }
    summary_path = output_root / 'meta' / 'visual_label_export_summary.json'
    summary_path.write_text(pretty_json(summary) + '\n', encoding='utf-8')
    (output_root / 'meta' / 'info.json').write_text(
        pretty_json({
            'format': 'room315_visual_state_oracle_v1',
            'dataset_mode': DATASET_MODE_VISUAL_STATE,
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'calibration_version': CALIBRATION_VERSION,
            'source_dataset': str(source_root),
            'training_events': 'meta/training_events.jsonl',
            'model_input': 'overhead_images only',
            'oracle_labels': 'inline before split; physically separated by splitter',
        }) + '\n',
        encoding='utf-8',
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Project approved Room 315 Gazebo oracle state into camera-space '
            'visual labels and remove exact duplicate/conflicting image pairs.'
        )
    )
    parser.add_argument('source_dataset', type=Path)
    parser.add_argument('output_dataset', type=Path)
    parser.add_argument(
        '--camera-model-sdf',
        type=Path,
        default=_default_camera_model_path(),
    )
    args = parser.parse_args(argv)
    summary = export_visual_dataset(
        args.source_dataset,
        args.output_dataset,
        camera_model_sdf=args.camera_model_sdf,
    )
    print(pretty_json(summary))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, VisualLabelExportError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
