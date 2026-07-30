#!/usr/bin/env python3
"""Shared helpers for the Room 315 visual-state dataset mode.

Model inputs are camera references only; oracle labels are loaded from a
physically separate JSONL file or from an explicit oracle label field before
split-time sanitisation.
"""

import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_json_io import iter_jsonl_objects
from room_315_visual_fleet import AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
from room_315_visual_fleet import AUTHORITATIVE_VISUAL_FLEET
from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_fleet import block_vocabulary_metadata
from room_315_visual_fleet import identity_side

DATASET_MODE_VISUAL_STATE = 'visual_state'
DATASET_MODES = (DATASET_MODE_VISUAL_STATE,)
VISUAL_STATE_SCHEMA_VERSION = 'room315.visual_state.v3'
SUPPORTED_VISUAL_STATE_SCHEMA_VERSIONS = {
    'room315.visual_state.v1',
    'room315.visual_state.v2',
    VISUAL_STATE_SCHEMA_VERSION,
}
VISUAL_LABEL_SUFFIX = '_visual_labels.jsonl'
VISUAL_MODEL_INPUT_KEYS = {'overhead_images'}
IMAGE_KEYS = ('left_rail_rgb', 'right_rail_rgb')
CAMERA_SIDE = {
    'left_rail_rgb': 'left',
    'right_rail_rgb': 'right',
}
RAIL_POSITION_RATIO_TOLERANCE = 1e-5
MODEL_TARGET_SHUTTLE_FIELDS = {
    'location.side',
    'location.block',
    'rail_position.s_m',
    'rail_position.s_ratio',
    'rail_position.segment_length_m',
    'loaded_state',
}
FIXED_VISUAL_NUMERIC_FIELDS = (
    'bbox.0',
    'bbox.1',
    'bbox.2',
    'bbox.3',
    'rail_position.s_m',
    'rail_position.s_ratio',
    'rail_position.segment_length_m',
)
FIXED_VISUAL_CATEGORICAL_FIELDS = {
    'location.side': ('left', 'right'),
    'location.block': AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY,
    'loaded_state': ('empty', 'loaded'),
}
MODEL_TARGET_EXCLUDED_ORACLE_FIELDS = {
    'rail_position.position_uncertainty_m',
}
SHUTTLE_LABEL_FIELDS = {
    'bbox',
    'bbox_camera',
    'camera_observations',
    'location',
    'visually_available_identity',
    'visible_identity',
    'identity_available',
    'loaded_state',
    'loaded',
    'payload_state',
    'rail_position',
    's_m',
    's_ratio',
    'segment_length_m',
    'position_uncertainty_m',
}
LABEL_LEAKAGE_KEYS = {
    'visual_state_labels',
    'oracle_visual_state',
    'oracle_labels',
    'privileged_eval',
    'bbox',
    'bounding_box',
    'bbox_camera',
    'camera_observations',
    'location',
    'loaded_state',
    'payload_state',
    'rail_position',
    's_m',
    's_ratio',
    'segment_length_m',
    'position_uncertainty_m',
    'visually_available_identity',
    'visible_identity',
    'obstacles',
    'switch_states',
    'calibration_version',
}


class VisualStateValidationError(ValueError):
    """Raised when a visual-state row or label violates the schema."""


def pretty_json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)


def iter_jsonl(path: Path) -> list[dict[str, Any]]:
    return list(
        iter_jsonl_objects(
            path,
            error_type=VisualStateValidationError,
            require_object=True,
        )
    )


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')


def rows_fingerprint(rows: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        payload = json.dumps(row, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        digest.update(payload.encode('utf-8'))
        digest.update(b'\n')
    return digest.hexdigest()


def file_fingerprint(path: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    size = 0
    lines = 0
    with path.expanduser().open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
            size += len(chunk)
            lines += chunk.count(b'\n')
    return {
        'path': str(path),
        'sha256': digest.hexdigest(),
        'bytes': size,
        'newline_count': lines,
    }


def resolve_image_path(dataset_root: Path, image_ref: str) -> Path:
    path = Path(str(image_ref)).expanduser()
    return path if path.is_absolute() else dataset_root / path


def missing_image_error(
    row: dict[str, Any],
    image_key: str,
    reason: str,
    ref: str = '',
) -> str:
    episode_id = str(row.get('episode_id') or '')
    detail = f' ({ref})' if ref else ''
    return f'episode {episode_id!r} image {image_key!r} is {reason}{detail}'


def safe_int(raw: Any, fallback: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return fallback


def sample_id_for_row(row: dict[str, Any], row_index: int | None = None) -> str:
    sample_id = str(row.get('sample_id') or '').strip()
    if sample_id:
        return sample_id
    episode_id = str(row.get('episode_id') or '').strip()
    step = row.get('step_index', row.get('event_index', row_index))
    if episode_id:
        return f'{episode_id}:step:{step}'
    if row_index is None:
        raise VisualStateValidationError('visual-state row is missing sample_id and episode_id')
    return f'row:{row_index}'


def visual_label_path_for_split(split_file: Path) -> Path:
    split_file = split_file.expanduser()
    return split_file.with_name(f'{split_file.stem}{VISUAL_LABEL_SUFFIX}')


def _finite_float(raw: Any, *, context: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise VisualStateValidationError(f'{context} must be a finite number') from exc
    if not math.isfinite(value):
        raise VisualStateValidationError(f'{context} must be finite')
    return value


def _confidence(raw: Any, *, context: str) -> float:
    if raw is None:
        raise VisualStateValidationError(f'{context} is missing confidence')
    value = _finite_float(raw, context=f'{context}.confidence')
    if value < 0.0 or value > 1.0:
        raise VisualStateValidationError(f'{context}.confidence must be in [0, 1]')
    return round(value, 6)


def _bbox(raw: Any, *, context: str) -> list[float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        raise VisualStateValidationError(f'{context}.bbox must contain four numbers')
    return [round(_finite_float(value, context=f'{context}.bbox'), 6) for value in raw]


def _clean_text(raw: Any, fallback: str = 'unknown') -> str:
    text = str(raw if raw is not None else '').strip()
    return text if text else fallback


def _loaded_state(raw: Any) -> str:
    if isinstance(raw, bool):
        return 'loaded' if raw else 'empty'
    text = _clean_text(raw).strip().lower()
    if text in {'loaded', 'empty', 'unknown'}:
        return text
    if text in {'true', 'present', 'with_payload', 'payload_present'}:
        return 'loaded'
    if text in {'false', 'absent', 'without_payload', 'unloaded', 'no_payload'}:
        return 'empty'
    raise VisualStateValidationError(f'unsupported loaded_state: {raw!r}')


def _switch_state(raw: Any) -> str:
    text = _clean_text(raw).strip().upper()
    if text in {'INTERIOR', 'EXTERIOR', 'UNKNOWN'}:
        return text.lower()
    if text in {'0', 'EXT'}:
        return 'exterior'
    if text in {'1', 'INT'}:
        return 'interior'
    raise VisualStateValidationError(f'unsupported switch state: {raw!r}')


def _location(raw: Any) -> dict[str, Any]:
    if isinstance(raw, str):
        return {'entity': _clean_text(raw)}
    if not isinstance(raw, dict):
        raise VisualStateValidationError('location must be an object or entity string')
    allowed = ('side', 'slot', 'block', 'station', 'entity', 'status')
    location = {
        key: _clean_text(raw.get(key))
        for key in allowed
        if raw.get(key) not in (None, '')
    }
    if not location:
        raise VisualStateValidationError('location must include side/slot/block/station/entity/status')
    return dict(sorted(location.items()))


def _optional_nonnegative_float(
    raw: Any,
    *,
    context: str,
    fallback: float = 0.0,
) -> float:
    if raw in (None, ''):
        return float(fallback)
    value = _finite_float(raw, context=context)
    if value < 0.0:
        raise VisualStateValidationError(f'{context} must be non-negative')
    return round(value, 6)


def _rail_position(raw_shuttle: dict[str, Any], *, context: str) -> dict[str, Any]:
    raw = raw_shuttle.get('rail_position')
    if raw is None:
        raw = {
            key: raw_shuttle.get(key)
            for key in (
                'available',
                's_m',
                's_ratio',
                'segment_length_m',
                'position_uncertainty_m',
            )
            if key in raw_shuttle
        }
    if not isinstance(raw, dict):
        raise VisualStateValidationError(f'{context}.rail_position must be an object')
    available = bool(
        raw.get(
            'available',
            raw.get('s_ratio') not in (None, '') and raw.get('s_m') not in (None, ''),
        )
    )
    s_ratio = _optional_nonnegative_float(
        raw.get('s_ratio'),
        context=f'{context}.rail_position.s_ratio',
    )
    if s_ratio > 1.0:
        raise VisualStateValidationError(
            f'{context}.rail_position.s_ratio must be in [0, 1]'
        )
    s_m = _optional_nonnegative_float(
        raw.get('s_m'),
        context=f'{context}.rail_position.s_m',
    )
    segment_length_m = _optional_nonnegative_float(
        raw.get('segment_length_m'),
        context=f'{context}.rail_position.segment_length_m',
    )
    uncertainty_m = _optional_nonnegative_float(
        raw.get('position_uncertainty_m', raw.get('uncertainty_m')),
        context=f'{context}.rail_position.position_uncertainty_m',
    )
    if available:
        if segment_length_m <= 0.0:
            raise VisualStateValidationError(
                f'{context}.rail_position.segment_length_m must be positive when available'
            )
        if s_m > segment_length_m + 1e-6:
            raise VisualStateValidationError(
                f'{context}.rail_position.s_m exceeds segment_length_m'
            )
        expected_ratio = s_m / segment_length_m
        if abs(s_ratio - expected_ratio) > RAIL_POSITION_RATIO_TOLERANCE:
            raise VisualStateValidationError(
                f'{context}.rail_position.s_ratio is inconsistent with '
                's_m / segment_length_m'
            )
    return {
        'available': available,
        's_m': s_m,
        's_ratio': s_ratio,
        'segment_length_m': segment_length_m,
        'position_uncertainty_m': uncertainty_m,
    }


def _raw_label_payload(row: dict[str, Any]) -> dict[str, Any]:
    for key in ('visual_state_labels', 'oracle_visual_state', 'labels'):
        value = row.get(key)
        if isinstance(value, dict):
            return value
    privileged_eval = row.get('privileged_eval')
    if isinstance(privileged_eval, dict):
        for key in ('visual_state_labels', 'oracle_visual_state', 'labels'):
            value = privileged_eval.get(key)
            if isinstance(value, dict):
                return value
    if any(key in row for key in ('shuttles', 'switches', 'obstacles')):
        return row
    raise VisualStateValidationError('row is missing visual_state_labels/oracle_visual_state')


def _iter_entities(raw: Any, *, entity_name: str) -> list[dict[str, Any]]:
    if raw is None:
        return []
    if isinstance(raw, dict):
        entities = []
        for entity_id, value in raw.items():
            if isinstance(value, dict):
                item = dict(value)
            else:
                item = {'state': value}
            item.setdefault('id', entity_id)
            entities.append(item)
        return entities
    if isinstance(raw, list):
        if not all(isinstance(item, dict) for item in raw):
            raise VisualStateValidationError(f'{entity_name} entries must be objects')
        return list(raw)
    raise VisualStateValidationError(f'{entity_name} must be a list or mapping')


def _explicit_bool(raw: Any, *, context: str, default: bool) -> bool:
    if raw is None:
        return default
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, (int, float)) and raw in {0, 1}:
        return bool(raw)
    text = str(raw).strip().lower()
    if text in {'true', 'yes', '1'}:
        return True
    if text in {'false', 'no', '0'}:
        return False
    raise VisualStateValidationError(f'{context} must be boolean')


def canonical_camera_for_identity(identity: str) -> str:
    """Return the only overhead camera that can contain this identity."""
    side = identity_side(str(identity).strip().upper())
    return f'{side}_rail_rgb'


def valid_bbox(value: Any) -> bool:
    """Return whether a value is a finite positive XYWH bounding box."""
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return False
    try:
        bbox = [float(item) for item in value]
    except (TypeError, ValueError):
        return False
    return (
        all(math.isfinite(item) for item in bbox)
        and bbox[2] > 0.0
        and bbox[3] > 0.0
    )


def _zero_bbox(value: Any) -> bool:
    if value is None:
        return True
    try:
        bbox = _bbox(value, context='masked camera bbox')
    except VisualStateValidationError:
        return False
    return all(abs(item) <= 1e-12 for item in bbox)


def _normalize_camera_observations(
    shuttle: dict[str, Any],
    *,
    identity: str,
    present: bool,
    canonical_visible: bool,
    canonical_bbox: list[float],
    context: str,
) -> dict[str, dict[str, Any]]:
    canonical_camera = canonical_camera_for_identity(identity)
    declared_camera = shuttle.get('bbox_camera')
    if (
        declared_camera is not None
        and str(declared_camera).strip() != canonical_camera
    ):
        raise VisualStateValidationError(
            f'{context}.bbox_camera must be {canonical_camera!r}'
        )
    raw_observations = shuttle.get('camera_observations')
    if raw_observations is not None and not isinstance(
        raw_observations,
        dict,
    ):
        raise VisualStateValidationError(
            f'{context}.camera_observations must be an object'
        )
    unknown = (
        set(raw_observations or {})
        - set(IMAGE_KEYS)
    )
    if unknown:
        raise VisualStateValidationError(
            f'{context}.camera_observations has unknown cameras: '
            f'{sorted(unknown)}'
        )

    normalized = {}
    for camera in IMAGE_KEYS:
        applicable = camera == canonical_camera
        raw = (
            (raw_observations or {}).get(camera)
            if raw_observations is not None
            else None
        )
        if raw is not None and not isinstance(raw, dict):
            raise VisualStateValidationError(
                f'{context}.camera_observations.{camera} must be an object'
            )
        raw = raw or {}
        declared_applicable = _explicit_bool(
            raw.get('applicable'),
            context=(
                f'{context}.camera_observations.{camera}.applicable'
            ),
            default=applicable,
        )
        if declared_applicable != applicable:
            raise VisualStateValidationError(
                f'{context}.camera_observations.{camera}.applicable '
                f'must be {applicable}'
            )
        visual_available = _explicit_bool(
            raw.get(
                'visual_available',
                raw.get('visually_available'),
            ),
            context=(
                f'{context}.camera_observations.{camera}.visual_available'
            ),
            default=(
                present and canonical_visible and applicable
                if raw_observations is None
                else False
            ),
        )
        raw_bbox = raw.get('bbox')
        if visual_available:
            if not present or not applicable:
                raise VisualStateValidationError(
                    f'{context}.camera_observations.{camera} cannot be '
                    'available for an absent or opposite-rail identity'
                )
            bbox = _bbox(
                raw_bbox if raw_bbox is not None else canonical_bbox,
                context=(
                    f'{context}.camera_observations.{camera}'
                ),
            )
            if not valid_bbox(bbox):
                raise VisualStateValidationError(
                    f'{context}.camera_observations.{camera}.bbox must '
                    'be positive when visible'
                )
        else:
            if raw_bbox is not None and not _zero_bbox(raw_bbox):
                raise VisualStateValidationError(
                    f'{context}.camera_observations.{camera}.bbox must '
                    'be fully masked when unavailable'
                )
            bbox = [0.0, 0.0, 0.0, 0.0]

        expected_visible = (
            present and canonical_visible and applicable
        )
        if visual_available != expected_visible:
            raise VisualStateValidationError(
                f'{context}.camera_observations.{camera}.visual_available '
                f'must be {expected_visible}'
            )
        if visual_available and any(
            abs(float(first) - float(second)) > 1e-6
            for first, second in zip(bbox, canonical_bbox)
        ):
            raise VisualStateValidationError(
                f'{context}.camera_observations.{camera}.bbox does not '
                'match the canonical own-camera bbox'
            )
        expected_bbox_mask = (
            [1.0, 1.0, 1.0, 1.0]
            if visual_available
            else [0.0, 0.0, 0.0, 0.0]
        )
        declared_bbox_mask = raw.get('bbox_target_mask')
        if declared_bbox_mask is not None:
            if (
                not isinstance(declared_bbox_mask, (list, tuple))
                or len(declared_bbox_mask) != 4
            ):
                raise VisualStateValidationError(
                    f'{context}.camera_observations.{camera}.'
                    'bbox_target_mask must contain four values'
                )
            try:
                parsed_bbox_mask = [
                    float(value) for value in declared_bbox_mask
                ]
            except (TypeError, ValueError) as exc:
                raise VisualStateValidationError(
                    f'{context}.camera_observations.{camera}.'
                    'bbox_target_mask must be numeric'
                ) from exc
            if parsed_bbox_mask != expected_bbox_mask:
                raise VisualStateValidationError(
                    f'{context}.camera_observations.{camera}.'
                    f'bbox_target_mask must be {expected_bbox_mask}'
                )
        normalized[camera] = {
            'applicable': applicable,
            'visual_available': visual_available,
            'bbox': bbox,
            'bbox_target_mask': expected_bbox_mask,
        }
    return normalized


def camera_observation_for_shuttle(
    shuttle: dict[str, Any],
    camera: str,
) -> dict[str, Any]:
    """Return normalized per-camera availability for a fixed shuttle slot."""
    if camera not in CAMERA_SIDE:
        raise VisualStateValidationError(
            f'unsupported Room 315 camera: {camera!r}'
        )
    identity = str(shuttle.get('id') or '').strip().upper()
    observations = shuttle.get('camera_observations')
    if isinstance(observations, dict) and isinstance(
        observations.get(camera),
        dict,
    ):
        return dict(observations[camera])
    applicable = canonical_camera_for_identity(identity) == camera
    visible = bool(
        shuttle.get('presence')
        and shuttle.get('visually_available')
        and applicable
        and valid_bbox(shuttle.get('bbox'))
    )
    return {
        'applicable': applicable,
        'visual_available': visible,
        'bbox': (
            list(shuttle['bbox'])
            if visible
            else [0.0, 0.0, 0.0, 0.0]
        ),
        'bbox_target_mask': (
            [1.0, 1.0, 1.0, 1.0]
            if visible
            else [0.0, 0.0, 0.0, 0.0]
        ),
    }


def _empty_fixed_shuttle(identity: str) -> dict[str, Any]:
    canonical_camera = canonical_camera_for_identity(identity)
    return {
        'id': identity,
        'presence': False,
        'visually_available': False,
        'bbox': [0.0, 0.0, 0.0, 0.0],
        'bbox_camera': canonical_camera,
        'camera_observations': {
            camera: {
                'applicable': camera == canonical_camera,
                'visual_available': False,
                'bbox': [0.0, 0.0, 0.0, 0.0],
                'bbox_target_mask': [0.0, 0.0, 0.0, 0.0],
            }
            for camera in IMAGE_KEYS
        },
        'location': {
            'side': identity_side(identity),
            'block': 'unknown',
        },
        'rail_position': {
            'available': False,
            's_m': 0.0,
            's_ratio': 0.0,
            'segment_length_m': 0.0,
            'position_uncertainty_m': 0.0,
        },
        'loaded_state': 'unknown',
        'confidence': 0.0,
    }


def _normalize_fixed_shuttles(raw_shuttles: Any, *, context: str) -> list[dict[str, Any]]:
    by_identity: dict[str, dict[str, Any]] = {}
    for index, shuttle in enumerate(
        _iter_entities(raw_shuttles, entity_name='shuttles')
    ):
        item_context = f'{context}.shuttles[{index}]'
        identity = _clean_text(
            shuttle.get('id')
            or shuttle.get('shuttle_id')
            or shuttle.get('identity')
            or shuttle.get('visually_available_identity')
        ).upper()
        if identity not in FIXED_VISUAL_SHUTTLE_IDENTITIES:
            raise VisualStateValidationError(
                f'{item_context}.id is not in the authoritative fixed fleet: '
                f'{identity!r}'
            )
        if identity in by_identity:
            raise VisualStateValidationError(
                f'{context} contains duplicate fixed shuttle entry {identity}'
            )
        present = _explicit_bool(
            shuttle.get('presence', shuttle.get('present')),
            context=f'{item_context}.presence',
            default=True,
        )
        bbox_raw = shuttle.get('bbox', shuttle.get('bounding_box'))
        visually_available = _explicit_bool(
            shuttle.get(
                'visually_available',
                shuttle.get('visual_availability', bbox_raw is not None),
            ),
            context=f'{item_context}.visually_available',
            default=bbox_raw is not None,
        )
        if visually_available and not present:
            raise VisualStateValidationError(
                f'{item_context} cannot be visually available while absent'
            )
        if not present:
            empty = _empty_fixed_shuttle(identity)
            empty['camera_observations'] = _normalize_camera_observations(
                shuttle,
                identity=identity,
                present=False,
                canonical_visible=False,
                canonical_bbox=[0.0, 0.0, 0.0, 0.0],
                context=item_context,
            )
            by_identity[identity] = empty
            continue
        expected_side = identity_side(identity)
        location = _location(
            shuttle.get('location', {'side': expected_side, 'block': 'unknown'})
        )
        side = str(location.get('side') or '').strip().lower()
        if side != expected_side:
            raise VisualStateValidationError(
                f'{item_context}.location.side must be {expected_side!r}'
            )
        block = str(location.get('block') or 'unknown').strip().upper()
        rail_position = _rail_position(shuttle, context=item_context)
        loaded_state = _loaded_state(
            shuttle.get(
                'loaded_state',
                shuttle.get('payload_state', shuttle.get('loaded')),
            )
        )
        if visually_available:
            bbox = _bbox(bbox_raw, context=item_context)
            if bbox[2] <= 0.0 or bbox[3] <= 0.0:
                raise VisualStateValidationError(
                    f'{item_context}.bbox width and height must be positive when visible'
                )
            if block not in AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY:
                raise VisualStateValidationError(
                    f'{item_context}.location.block is not representable in the '
                    f'authoritative global vocabulary: {block!r}'
                )
            if not rail_position['available']:
                raise VisualStateValidationError(
                    f'{item_context}.rail_position must be available when visible'
                )
            if loaded_state not in {'loaded', 'empty'}:
                raise VisualStateValidationError(
                    f'{item_context}.loaded_state must be loaded or empty when visible'
                )
        else:
            bbox = [0.0, 0.0, 0.0, 0.0]
        camera_observations = _normalize_camera_observations(
            shuttle,
            identity=identity,
            present=True,
            canonical_visible=visually_available,
            canonical_bbox=bbox,
            context=item_context,
        )
        by_identity[identity] = {
            'id': identity,
            'presence': True,
            'visually_available': visually_available,
            'bbox': bbox,
            'bbox_camera': canonical_camera_for_identity(identity),
            'camera_observations': camera_observations,
            'location': {
                'side': expected_side,
                'block': block,
            },
            'rail_position': rail_position,
            'loaded_state': loaded_state,
            'confidence': _confidence(
                shuttle.get('confidence', 1.0 if visually_available else 0.0),
                context=item_context,
            ),
        }
    return [
        by_identity.get(identity, _empty_fixed_shuttle(identity))
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    ]


def normalize_visual_state_labels(row_or_label: dict[str, Any], *, context: str = 'row') -> dict[str, Any]:
    raw = _raw_label_payload(row_or_label)
    schema_version = _clean_text(raw.get('schema_version'), VISUAL_STATE_SCHEMA_VERSION)
    if schema_version not in SUPPORTED_VISUAL_STATE_SCHEMA_VERSIONS:
        raise VisualStateValidationError(
            f'{context}.schema_version is unsupported: {schema_version!r}'
        )
    calibration_version = _clean_text(raw.get('calibration_version'))
    confidence = _confidence(raw.get('confidence'), context=context)

    if schema_version == VISUAL_STATE_SCHEMA_VERSION:
        shuttles = _normalize_fixed_shuttles(
            raw.get('shuttles'),
            context=context,
        )
    else:
        shuttles = []
        for index, shuttle in enumerate(
            _iter_entities(raw.get('shuttles'), entity_name='shuttles')
        ):
            item_context = f'{context}.shuttles[{index}]'
            identity = _clean_text(
                shuttle.get('visually_available_identity')
                or shuttle.get('visible_identity')
                or shuttle.get('identity')
                or shuttle.get('id')
            )
            shuttles.append({
                'id': _clean_text(
                    shuttle.get('id') or shuttle.get('shuttle_id') or identity
                ),
                'visually_available_identity': identity,
                'identity_available': bool(
                    shuttle.get('identity_available', identity != 'unknown')
                ),
                'bbox': _bbox(
                    shuttle.get('bbox') or shuttle.get('bounding_box'),
                    context=item_context,
                ),
                'location': _location(shuttle.get('location')),
                'rail_position': _rail_position(shuttle, context=item_context),
                'loaded_state': _loaded_state(
                    shuttle.get(
                        'loaded_state',
                        shuttle.get('payload_state', shuttle.get('loaded')),
                    )
                ),
                'confidence': _confidence(
                    shuttle.get('confidence'),
                    context=item_context,
                ),
            })

    switches = []
    raw_switches = raw.get('switches', raw.get('switch_states'))
    for index, switch in enumerate(_iter_entities(raw_switches, entity_name='switches')):
        item_context = f'{context}.switches[{index}]'
        switches.append({
            'id': _clean_text(switch.get('id') or switch.get('switch_id')),
            'state': _switch_state(switch.get('state', switch.get('position'))),
            'confidence': _confidence(switch.get('confidence'), context=item_context),
        })

    obstacles = []
    for index, obstacle in enumerate(_iter_entities(raw.get('obstacles'), entity_name='obstacles')):
        item_context = f'{context}.obstacles[{index}]'
        obstacles.append({
            'id': _clean_text(obstacle.get('id') or obstacle.get('obstacle_id') or f'obstacle_{index}'),
            'bbox': _bbox(obstacle.get('bbox') or obstacle.get('bounding_box'), context=item_context),
            'location': _location(obstacle.get('location', {'status': 'unknown'})),
            'confidence': _confidence(obstacle.get('confidence'), context=item_context),
        })

    if schema_version != VISUAL_STATE_SCHEMA_VERSION:
        shuttles.sort(
            key=lambda item: (item['id'], item['visually_available_identity'])
        )
    switches.sort(key=lambda item: item['id'])
    obstacles.sort(key=lambda item: item['id'])
    return {
        'schema_version': schema_version,
        'calibration_version': calibration_version,
        'confidence': confidence,
        'shuttles': shuttles,
        'switches': switches,
        'obstacles': obstacles,
    }


def _walk_keys(value: Any) -> set[str]:
    keys: set[str] = set()
    if isinstance(value, dict):
        for key, child in value.items():
            keys.add(str(key))
            keys.update(_walk_keys(child))
    elif isinstance(value, list):
        for child in value:
            keys.update(_walk_keys(child))
    return keys


def validate_visual_model_input(row: dict[str, Any], *, context: str = 'row') -> dict[str, Any]:
    model_input = row.get('model_input')
    if not isinstance(model_input, dict):
        raise VisualStateValidationError(f'{context} is missing model_input')
    unexpected = sorted(set(model_input) - VISUAL_MODEL_INPUT_KEYS)
    if unexpected:
        raise VisualStateValidationError(
            f'{context} visual_state model_input has undeclared fields: {unexpected}'
        )
    leaked = sorted(_walk_keys(model_input) & LABEL_LEAKAGE_KEYS)
    if leaked:
        raise VisualStateValidationError(
            f'{context} visual_state model_input contains oracle/label leakage keys: {leaked}'
        )
    overhead_images = model_input.get('overhead_images')
    if not isinstance(overhead_images, dict):
        raise VisualStateValidationError(f'{context} model_input.overhead_images must be an object')
    return {'overhead_images': dict(overhead_images)}


def visual_model_input_image_refs(row: dict[str, Any]) -> dict[str, str]:
    model_input = validate_visual_model_input(row)
    return {
        str(key): str(value)
        for key, value in model_input['overhead_images'].items()
        if value
    }


def raw_visual_image_refs(row: dict[str, Any]) -> dict[str, str]:
    model_input = row.get('model_input')
    if not isinstance(model_input, dict):
        return {}
    overhead_images = model_input.get('overhead_images')
    if not isinstance(overhead_images, dict):
        return {}
    return {
        str(key): str(value)
        for key, value in overhead_images.items()
        if value
    }


def image_integrity_report(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    split_name: str = '',
    operation: str = 'conversion',
    allow_blank_images: bool = False,
    dataset_mode: str = DATASET_MODE_VISUAL_STATE,
) -> dict[str, Any]:
    if dataset_mode not in DATASET_MODES:
        raise ValueError(f'unknown dataset mode: {dataset_mode}')
    per_camera = {
        camera: {
            'referenced_rows': 0,
            'missing_ref_rows': 0,
            'existing_files': 0,
            'missing_files': 0,
            'unreadable_files': 0,
            'blank_substitutions': 0,
        }
        for camera in IMAGE_KEYS
    }
    complete_rows = 0
    problems: list[dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        refs = visual_model_input_image_refs(row)
        row_complete = True
        for camera in IMAGE_KEYS:
            ref = refs.get(camera, '')
            stats = per_camera[camera]
            if not ref:
                stats['missing_ref_rows'] += 1
                row_complete = False
                if allow_blank_images:
                    stats['blank_substitutions'] += 1
                reason = 'missing_ref'
            else:
                stats['referenced_rows'] += 1
                image_path = resolve_image_path(dataset_root, ref)
                if not image_path.exists():
                    stats['missing_files'] += 1
                    row_complete = False
                    if allow_blank_images:
                        stats['blank_substitutions'] += 1
                    reason = 'missing_file'
                else:
                    try:
                        with Image.open(image_path) as image:
                            image.verify()
                        stats['existing_files'] += 1
                        continue
                    except Exception:
                        stats['unreadable_files'] += 1
                        row_complete = False
                        if allow_blank_images:
                            stats['blank_substitutions'] += 1
                        reason = 'unreadable_file'
            if len(problems) < 20:
                problem = {
                    'row_index': row_index,
                    'episode_id': str(row.get('episode_id') or ''),
                    'camera': camera,
                    'reason': reason,
                }
                if ref:
                    problem['ref'] = ref
                problems.append(problem)
        if row_complete:
            complete_rows += 1
    problem_count = sum(
        stats['missing_ref_rows'] + stats['missing_files'] + stats['unreadable_files']
        for stats in per_camera.values()
    )
    if problem_count and not allow_blank_images:
        prefix = f'{split_name} ' if split_name else ''
        first = problems[0] if problems else {}
        raise FileNotFoundError(
            f'{prefix}image integrity check failed before {operation}: {first}'
        )
    total = len(rows)
    return {
        'required_cameras': list(IMAGE_KEYS),
        'total_rows': total,
        'complete_rows': complete_rows,
        'complete_row_rate': round(complete_rows / max(1, total), 6),
        'allow_blank_images': bool(allow_blank_images),
        'debug_blank_image_mode': bool(allow_blank_images),
        'per_camera': per_camera,
        'problem_examples': problems,
    }


def scenario_family_from_row(row: dict[str, Any]) -> str:
    value = str(row.get('scenario_family') or '').strip()
    if value:
        return value
    raise VisualStateValidationError('visual-state row is missing scenario family')


def sanitized_visual_state_row(row: dict[str, Any], row_index: int) -> tuple[dict[str, Any], dict[str, Any]]:
    sample_id = sample_id_for_row(row, row_index)
    labels = normalize_visual_state_labels(row, context=f'row {row_index}')
    model_row = {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'sample_id': sample_id,
        'episode_id': str(row.get('episode_id') or ''),
        'step_index': row.get('step_index', row.get('event_index', row_index)),
        'scenario_family': scenario_family_from_row(row),
        'model_input': {
            'overhead_images': dict(
                (
                    (row.get('model_input') or {}).get('overhead_images')
                    if isinstance(row.get('model_input'), dict)
                    else {}
                )
                or {}
            ),
        },
    }
    validate_visual_model_input(model_row, context=f'row {row_index}')
    label_row = {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'sample_id': sample_id,
        'episode_id': model_row['episode_id'],
        'step_index': model_row['step_index'],
        'scenario_family': model_row['scenario_family'],
        'label_source': 'oracle',
        'model_input_exposure': 'excluded',
        'visual_state_labels': labels,
    }
    return model_row, label_row


def load_visual_labels_for_rows(
    rows: list[dict[str, Any]],
    labels_path: Path | None = None,
) -> list[dict[str, Any]]:
    if labels_path is not None:
        label_rows = iter_jsonl(labels_path)
        labels_by_sample: dict[str, dict[str, Any]] = {}
        for index, label_row in enumerate(label_rows):
            sample_id = sample_id_for_row(label_row, index)
            if sample_id in labels_by_sample:
                raise VisualStateValidationError(f'duplicate visual label sample_id: {sample_id}')
            labels_by_sample[sample_id] = label_row
        labels: list[dict[str, Any]] = []
        missing: list[str] = []
        for index, row in enumerate(rows):
            sample_id = sample_id_for_row(row, index)
            label_row = labels_by_sample.get(sample_id)
            if label_row is None:
                missing.append(sample_id)
                continue
            labels.append(normalize_visual_state_labels(label_row, context=f'label {sample_id}'))
        if missing:
            raise VisualStateValidationError(f'missing visual labels for samples: {missing[:5]}')
        return labels
    return [
        normalize_visual_state_labels(row, context=f'row {index}')
        for index, row in enumerate(rows)
    ]


def validate_visual_state_rows(
    rows: list[dict[str, Any]],
    labels_path: Path | None = None,
) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        try:
            validate_visual_model_input(row, context=f'row {index}')
        except VisualStateValidationError as exc:
            if len(issues) < 20:
                issues.append({
                    'row_index': index,
                    'sample_id': str(row.get('sample_id') or ''),
                    'reason': str(exc),
                })
    if issues:
        raise VisualStateValidationError(f'visual_state model_input integrity failed: {issues[0]}')
    labels = load_visual_labels_for_rows(rows, labels_path)
    return {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'rows_checked': len(rows),
        'labels_checked': len(labels),
        'allowed_model_input_fields': sorted(VISUAL_MODEL_INPUT_KEYS),
        'production_feature_source': 'model_input.overhead_images only',
        'oracle_label_source': 'separate_jsonl' if labels_path is not None else 'explicit_oracle_label_field',
        'oracle_labels_physically_separate': labels_path is not None,
        'label_schema_version': VISUAL_STATE_SCHEMA_VERSION,
        'row_level_metadata_used_as_features': [],
    }


def visual_state_class_balance(labels: list[dict[str, Any]]) -> dict[str, Any]:
    loaded = Counter()
    identities = Counter()
    switch_states = Counter()
    obstacle_counts = Counter()
    schema_versions = Counter()
    calibration_versions = Counter()
    position_available = Counter()
    for label in labels:
        schema_versions[str(label.get('schema_version') or '')] += 1
        calibration_versions[str(label.get('calibration_version') or '')] += 1
        obstacle_counts[str(len(label.get('obstacles') or []))] += 1
        for shuttle in label.get('shuttles') or []:
            if not shuttle.get('presence'):
                continue
            loaded[str(shuttle.get('loaded_state') or 'unknown')] += 1
            identities[str(shuttle.get('id') or 'unknown')] += 1
            position = shuttle.get('rail_position') or {}
            position_available[str(bool(position.get('available'))).lower()] += 1
        for switch in label.get('switches') or []:
            switch_states[str(switch.get('state') or 'unknown')] += 1
    return {
        'labels': len(labels),
        'loaded_state': dict(sorted(loaded.items())),
        'fixed_identity_presence': dict(sorted(identities.items())),
        'visible_switch_state': dict(sorted(switch_states.items())),
        'continuous_position_available': dict(sorted(position_available.items())),
        'obstacle_count': dict(sorted(obstacle_counts.items())),
        'schema_version': dict(sorted(schema_versions.items())),
        'calibration_version': dict(sorted(calibration_versions.items())),
    }


def _flatten(prefix: str, value: Any, output: dict[str, Any]) -> None:
    if isinstance(value, dict):
        for key in sorted(value):
            _flatten(f'{prefix}.{key}' if prefix else str(key), value[key], output)
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _flatten(f'{prefix}.{index}' if prefix else str(index), child, output)
        return
    output[prefix] = value


def _to_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _label_values(label: dict[str, Any]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    _flatten('', normalize_visual_state_labels(label), values)
    return values


def is_model_prediction_target(key: str) -> bool:
    """Return whether a flattened oracle-label field is a model target."""
    parts = str(key).split('.')
    if len(parts) < 3 or parts[0] != 'shuttles' or not parts[1].isdigit():
        return False
    field = '.'.join(parts[2:])
    if field.startswith('bbox.') and field[5:].isdigit():
        return True
    return field in MODEL_TARGET_SHUTTLE_FIELDS


class VisualStateLabelVectorizer:
    """Fixed eight-entry vectorizer; capacity is never fit from dataset rows."""

    KIND = 'room315_visual_state_fixed_eight_label_vectorizer'

    def __init__(self) -> None:
        self.numeric_keys = [
            f'shuttles.{slot}.{field}'
            for slot in range(len(FIXED_VISUAL_SHUTTLE_IDENTITIES))
            for field in FIXED_VISUAL_NUMERIC_FIELDS
        ]
        self.categorical_values = {
            f'shuttles.{slot}.{field}': list(values)
            for slot in range(len(FIXED_VISUAL_SHUTTLE_IDENTITIES))
            for field, values in FIXED_VISUAL_CATEGORICAL_FIELDS.items()
        }

    @classmethod
    def fit(cls, labels: list[dict[str, Any]]) -> 'VisualStateLabelVectorizer':
        if not labels:
            raise VisualStateValidationError(
                'cannot initialize the fixed visual vectorizer from empty labels'
            )
        for label in labels:
            normalized = normalize_visual_state_labels(label)
            if normalized['schema_version'] != VISUAL_STATE_SCHEMA_VERSION:
                raise VisualStateValidationError(
                    'dataset-fitted shuttle capacity is forbidden; training requires '
                    f'{VISUAL_STATE_SCHEMA_VERSION} fixed-eight labels'
                )
            identities = tuple(item['id'] for item in normalized['shuttles'])
            if identities != FIXED_VISUAL_SHUTTLE_IDENTITIES:
                raise VisualStateValidationError(
                    'fixed visual label identity order is invalid: '
                    f'{identities}'
                )
        return cls()

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> 'VisualStateLabelVectorizer':
        if data.get('kind') != cls.KIND:
            raise VisualStateValidationError('visual label vectorizer JSON has unexpected kind')
        vectorizer = cls()
        if data.get('fixed_identity_order') != list(FIXED_VISUAL_SHUTTLE_IDENTITIES):
            raise VisualStateValidationError(
                'visual label vectorizer fixed identity order does not match '
                'the authoritative fleet'
            )
        if data.get('global_block_vocabulary') != list(
            AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
        ):
            raise VisualStateValidationError(
                'visual label vectorizer block vocabulary does not match '
                'the authoritative topology'
            )
        if data.get('numeric_keys') != vectorizer.numeric_keys:
            raise VisualStateValidationError(
                'visual label vectorizer numeric targets are not the fixed schema'
            )
        if data.get('categorical_values') != vectorizer.categorical_values:
            raise VisualStateValidationError(
                'visual label vectorizer categorical targets are not the fixed schema'
            )
        if int(data.get('dim', -1)) != vectorizer.dim:
            raise VisualStateValidationError(
                'visual label vectorizer dimension does not match the fixed schema'
            )
        return vectorizer

    @property
    def names(self) -> list[str]:
        names = list(self.numeric_keys)
        for key, values in self.categorical_values.items():
            names.extend(f'{key}=={value}' for value in values)
        return names

    @property
    def dim(self) -> int:
        return len(self.names)

    def transform(self, label: dict[str, Any]) -> list[float]:
        normalized = normalize_visual_state_labels(label)
        if normalized['schema_version'] != VISUAL_STATE_SCHEMA_VERSION:
            raise VisualStateValidationError(
                f'fixed vectorizer requires {VISUAL_STATE_SCHEMA_VERSION}'
            )
        values: dict[str, Any] = {}
        _flatten('', normalized, values)
        vector: list[float] = []
        for key in self.numeric_keys:
            vector.append(_to_float(values.get(key)) or 0.0)
        for key, allowed_values in self.categorical_values.items():
            raw = str(values.get(key, '')).strip().lower()
            vector.extend(
                1.0 if raw == str(allowed).strip().lower() else 0.0
                for allowed in allowed_values
            )
        return vector

    def target_mask(
        self,
        label: dict[str, Any],
        camera: str | None = None,
    ) -> list[float]:
        """Mask unavailable slots globally or for one explicit camera view."""
        normalized = normalize_visual_state_labels(label)
        if camera is not None and camera not in CAMERA_SIDE:
            raise VisualStateValidationError(
                f'unsupported Room 315 camera: {camera!r}'
            )
        mask: list[float] = []
        for name in self.names:
            parts = name.split('.')
            shuttle_index = (
                int(parts[1])
                if len(parts) >= 3 and parts[0] == 'shuttles' and parts[1].isdigit()
                else None
            )
            available = (
                shuttle_index is not None
                and normalized['shuttles'][shuttle_index]['presence']
                and normalized['shuttles'][shuttle_index][
                    'visually_available'
                ]
            )
            if available and camera is not None:
                available = bool(
                    normalized['shuttles'][shuttle_index][
                        'camera_observations'
                    ][camera]['visual_available']
                )
            mask.append(
                1.0
                if available
                else 0.0
            )
        return mask

    def camera_target_masks(
        self,
        label: dict[str, Any],
    ) -> dict[str, list[float]]:
        """Return fail-closed masks for camera-specific loss computation."""
        return {
            camera: self.target_mask(label, camera=camera)
            for camera in IMAGE_KEYS
        }

    def validate_target(self, label: dict[str, Any]) -> None:
        """Fail if a visible fixed entry lacks exactly one categorical target."""
        normalized = normalize_visual_state_labels(label)
        vector = self.transform(normalized)
        mask = self.target_mask(normalized)
        for slot, shuttle in enumerate(normalized['shuttles']):
            visible = shuttle['presence'] and shuttle['visually_available']
            for field, allowed in FIXED_VISUAL_CATEGORICAL_FIELDS.items():
                base = f'shuttles.{slot}.{field}'
                indexes = [
                    self.names.index(f'{base}=={value}')
                    for value in allowed
                ]
                one_count = sum(
                    float(vector[index]) == 1.0
                    for index in indexes
                )
                if visible and (
                    one_count != 1
                    or not all(mask[index] == 1.0 for index in indexes)
                ):
                    raise VisualStateValidationError(
                        f'visible fixed entry {shuttle["id"]} must have exactly '
                        f'one {field} target'
                    )
                if not visible and any(mask[index] != 0.0 for index in indexes):
                    raise VisualStateValidationError(
                        f'unavailable fixed entry {shuttle["id"]} is not fully masked'
                    )

    def to_json(self) -> dict[str, Any]:
        return {
            'kind': self.KIND,
            'dataset_mode': DATASET_MODE_VISUAL_STATE,
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'fixed_identity_order': list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
            'authoritative_fleet': AUTHORITATIVE_VISUAL_FLEET,
            'global_block_vocabulary': list(
                AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
            ),
            'block_vocabulary_metadata': block_vocabulary_metadata(),
            'capacity_source': 'authoritative_repository_configuration',
            'capacity_inferred_from_dataset': False,
            'numeric_keys': self.numeric_keys,
            'categorical_values': self.categorical_values,
            'names': self.names,
            'dim': self.dim,
            'output_semantics': 'visual_state_labels_not_rail_commands',
            'prediction_target_fields': sorted(
                MODEL_TARGET_SHUTTLE_FIELDS | {'bbox'}
            ),
            'excluded_oracle_fields': sorted(
                MODEL_TARGET_EXCLUDED_ORACLE_FIELDS
            ),
            'target_mask': {
                'kind': 'fixed_identity_presence_and_visual_availability',
                'absent_fixed_entries_masked': True,
                'not_visually_available_entries_masked': True,
                'camera_specific_masks_supported': True,
                'opposite_rail_entries_masked_per_camera': True,
            },
            'bbox_semantics': {
                'kind': 'canonical_own_camera_bbox_with_per_camera_masks',
                'camera_by_identity_side': {
                    'left': 'left_rail_rgb',
                    'right': 'right_rail_rgb',
                },
                'global_pair_loss_uses_each_identity_once': True,
                'camera_specific_loss_requires_camera_target_masks': True,
            },
            'identity_prediction': {
                'supported': False,
                'reason': 'identity is defined by the fixed entry index',
                'classification_metric_reported': False,
            },
        }


def visual_target_stats(
    labels: list[dict[str, Any]],
    vectorizer: VisualStateLabelVectorizer,
    masks: list[list[float]] | None = None,
) -> tuple[list[float], list[float]]:
    if not labels:
        raise VisualStateValidationError('cannot compute visual target stats from empty labels')
    vectors = [vectorizer.transform(label) for label in labels]
    effective_masks = masks or [vectorizer.target_mask(label) for label in labels]
    if len(effective_masks) != len(vectors):
        raise VisualStateValidationError('visual target masks must match label count')
    columns: list[list[float]] = [[] for _ in range(vectorizer.dim)]
    for row_index, (vector, mask) in enumerate(zip(vectors, effective_masks)):
        if len(mask) != vectorizer.dim:
            raise VisualStateValidationError(
                f'visual target mask {row_index} has dimension {len(mask)}, '
                f'expected {vectorizer.dim}'
            )
        for index, (value, available) in enumerate(zip(vector, mask)):
            if float(available) > 0.0:
                columns[index].append(float(value))
    mean = [
        sum(column) / len(column) if column else 0.0
        for column in columns
    ]
    std = []
    for column, avg in zip(columns, mean):
        if not column:
            std.append(1.0)
            continue
        variance = sum((value - avg) ** 2 for value in column) / len(column)
        sigma = math.sqrt(variance)
        std.append(sigma if sigma >= 1e-6 else 1.0)
    return mean, std


def _target_available(record: dict[str, Any], index: int) -> bool:
    mask = record.get('target_mask')
    if not isinstance(mask, (list, tuple)):
        return True
    return index < len(mask) and float(mask[index]) > 0.0


def _mae(records: list[dict[str, Any]], indexes: list[int]) -> float | None:
    if not records or not indexes:
        return None
    total = 0.0
    count = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for index in indexes:
            if not _target_available(record, index):
                continue
            total += abs(float(pred[index]) - float(true[index]))
            count += 1
    return round(total / count, 6) if count else None


def _absolute_error_distribution(
    records: list[dict[str, Any]],
    indexes: list[int],
) -> dict[str, float | None]:
    errors = []
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        errors.extend(
            abs(float(pred[index]) - float(true[index]))
            for index in indexes
            if _target_available(record, index)
        )
    if not errors:
        return {'mean': None, 'p50': None, 'p95': None}
    ordered = sorted(errors)

    def percentile(fraction: float) -> float:
        position = (len(ordered) - 1) * fraction
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return ordered[lower]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    return {
        'mean': round(sum(ordered) / len(ordered), 6),
        'p50': round(percentile(0.50), 6),
        'p95': round(percentile(0.95), 6),
    }


def _binary_accuracy(records: list[dict[str, Any]], indexes: list[int]) -> float | None:
    if not records or not indexes:
        return None
    correct = 0
    total = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for index in indexes:
            if not _target_available(record, index):
                continue
            correct += int((float(pred[index]) >= 0.5) == (float(true[index]) >= 0.5))
            total += 1
    return round(correct / total, 6) if total else None


def _distribution(records: list[dict[str, Any]], key: str) -> dict[str, float | None]:
    values = []
    for record in records:
        try:
            value = float(record.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    if not values:
        return {'p50': None, 'p95': None, 'mean': None}
    ordered = sorted(values)

    def percentile(percent: float) -> float:
        if len(ordered) == 1:
            return ordered[0]
        position = (len(ordered) - 1) * percent
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return ordered[int(position)]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    return {
        'p50': round(percentile(0.50), 6),
        'p95': round(percentile(0.95), 6),
        'mean': round(sum(ordered) / len(ordered), 6),
    }


def _confidence_calibration(
    records: list[dict[str, Any]],
    indexes: list[int],
) -> dict[str, Any]:
    if not records or not indexes:
        return {
            'samples': 0,
            'confidence_fields': 0,
            'mean_abs_calibration_error': None,
            'overconfident_rate': None,
            'underconfident_rate': None,
        }
    total_error = 0.0
    overconfident = 0
    underconfident = 0
    count = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for index in indexes:
            if not _target_available(record, index):
                continue
            predicted_confidence = max(0.0, min(1.0, float(pred[index])))
            target_confidence = max(0.0, min(1.0, float(true[index])))
            total_error += abs(predicted_confidence - target_confidence)
            overconfident += int(predicted_confidence > target_confidence)
            underconfident += int(predicted_confidence < target_confidence)
            count += 1
    return {
        'samples': len(records),
        'confidence_fields': len(indexes),
        'mean_abs_calibration_error': round(total_error / max(1, count), 6),
        'overconfident_rate': round(overconfident / max(1, count), 6),
        'underconfident_rate': round(underconfident / max(1, count), 6),
    }


def _categorical_group_accuracy(
    records: list[dict[str, Any]],
    names: list[str],
    token: str,
) -> float | None:
    groups: dict[str, list[int]] = {}
    for index, name in enumerate(names):
        if token in name and '==' in name:
            groups.setdefault(name.split('==', 1)[0], []).append(index)
    if not records or not groups:
        return None
    correct = 0
    total = 0
    for record in records:
        true = record['true_raw']
        pred = record['pred_raw']
        for indexes in groups.values():
            if not any(_target_available(record, index) for index in indexes):
                continue
            true_index = max(indexes, key=lambda idx: float(true[idx]))
            pred_index = max(indexes, key=lambda idx: float(pred[idx]))
            correct += int(true_index == pred_index)
            total += 1
    return round(correct / total, 6) if total else None


def _categorical_groups(
    names: list[str],
    token: str,
) -> dict[str, list[int]]:
    groups: dict[str, list[int]] = {}
    for index, name in enumerate(names):
        if token in name and '==' in name:
            groups.setdefault(name.split('==', 1)[0], []).append(index)
    return groups


def _categorical_group_results(
    record: dict[str, Any],
    groups: dict[str, list[int]],
    *,
    top_k: int = 1,
) -> dict[str, bool]:
    true = record['true_raw']
    pred = record['pred_raw']
    results = {}
    for base, indexes in groups.items():
        if not any(_target_available(record, index) for index in indexes):
            continue
        active_targets = [
            index
            for index in indexes
            if float(true[index]) == 1.0
        ]
        if len(active_targets) != 1:
            # A malformed all-zero or multi-hot present target is never
            # interpreted as a correct argmax classification.
            results[base] = False
            continue
        ranked = sorted(
            indexes,
            key=lambda index: float(pred[index]),
            reverse=True,
        )
        results[base] = active_targets[0] in ranked[:max(1, top_k)]
    return results


def _fixed_location_metrics(
    records: list[dict[str, Any]],
    label_names: list[str],
) -> dict[str, float | None]:
    side_groups = _categorical_groups(label_names, '.location.side')
    block_groups = _categorical_groups(label_names, '.location.block')
    side_correct = 0
    block_correct = 0
    full_correct = 0
    top2_block_correct = 0
    total = 0
    for record in records:
        side_results = _categorical_group_results(record, side_groups)
        block_results = _categorical_group_results(record, block_groups)
        top2_results = _categorical_group_results(
            record,
            block_groups,
            top_k=2,
        )
        for slot in range(len(FIXED_VISUAL_SHUTTLE_IDENTITIES)):
            side_base = f'shuttles.{slot}.location.side'
            block_base = f'shuttles.{slot}.location.block'
            if side_base not in side_results or block_base not in block_results:
                continue
            side_ok = side_results[side_base]
            block_ok = block_results[block_base]
            side_correct += int(side_ok)
            block_correct += int(block_ok)
            full_correct += int(side_ok and block_ok)
            top2_block_correct += int(top2_results[block_base])
            total += 1

    def accuracy(correct: int) -> float | None:
        return round(correct / total, 6) if total else None

    return {
        'side_accuracy': accuracy(side_correct),
        'block_accuracy': accuracy(block_correct),
        'full_location_accuracy': accuracy(full_correct),
        'top2_block_accuracy': accuracy(top2_block_correct),
    }


def visual_state_metrics(records: list[dict[str, Any]], label_names: list[str]) -> dict[str, Any]:
    if not records:
        raise VisualStateValidationError('cannot summarise empty visual-state predictions')
    all_indexes = list(range(len(label_names)))
    bbox_indexes = [idx for idx, name in enumerate(label_names) if '.bbox.' in name]
    confidence_indexes = [idx for idx, name in enumerate(label_names) if name.endswith('.confidence') or name == 'confidence']
    s_m_indexes = [
        idx
        for idx, name in enumerate(label_names)
        if name.endswith('.rail_position.s_m')
    ]
    s_ratio_indexes = [
        idx
        for idx, name in enumerate(label_names)
        if name.endswith('.rail_position.s_ratio')
    ]
    obstacle_presence = [
        idx
        for idx, name in enumerate(label_names)
        if name.startswith('obstacles.') and name.endswith('.confidence')
    ]
    bbox_mae = _mae(records, bbox_indexes)
    s_m_mae = _mae(records, s_m_indexes)
    s_ratio_mae = _mae(records, s_ratio_indexes)
    confidence_mae = _mae(records, confidence_indexes)
    loaded_state_accuracy = _categorical_group_accuracy(
        records,
        label_names,
        '.loaded_state',
    )
    location_metrics = _fixed_location_metrics(records, label_names)
    obstacle_presence_accuracy = _binary_accuracy(records, obstacle_presence)
    return {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'samples': len(records),
        'label_mae': _mae(records, all_indexes),
        'bbox_mae': bbox_mae,
        's_m_mae': s_m_mae,
        's_ratio_mae': s_ratio_mae,
        'confidence_mae': confidence_mae,
        'identity_classification_supported': False,
        'identity_semantics': (
            'identity is defined by the fixed L1-L4,R1-R4 entry index'
        ),
        'loaded_state_accuracy': loaded_state_accuracy,
        'switch_state_accuracy': _categorical_group_accuracy(records, label_names, '.state'),
        **location_metrics,
        'obstacle_presence_accuracy': obstacle_presence_accuracy,
        'localization_metrics': {
            'bbox_mae': bbox_mae,
            **location_metrics,
            's_m_error': _absolute_error_distribution(records, s_m_indexes),
            's_ratio_error': _absolute_error_distribution(records, s_ratio_indexes),
        },
        'loaded_state_metrics': {
            'accuracy': loaded_state_accuracy,
        },
        'obstacle_metrics': {
            'presence_accuracy': obstacle_presence_accuracy,
        },
        'confidence_calibration': _confidence_calibration(records, confidence_indexes),
        'inference_latency_seconds': _distribution(records, 'inference_latency_seconds'),
        'cycle_time_seconds': _distribution(records, 'cycle_time_seconds'),
    }
