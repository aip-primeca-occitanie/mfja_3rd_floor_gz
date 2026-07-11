#!/usr/bin/env python3
"""Visual ObservedState provider for Room 315.

The core in this module is deliberately provider-agnostic: a model runner only
has to return strict JSON detections, while the provider owns calibration,
RGB-D/CameraInfo checks, and conversion into validated ObservedFacts.
"""

from __future__ import annotations

import copy
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_contracts import ObservedFact
from room_315_contracts import ObservedState
from room_315_multi_shuttle import DEVICE_NAMES
from room_315_multi_shuttle import SIDES
from room_315_multi_shuttle import all_shuttle_specs
from room_315_multi_shuttle import normalize_fleet_block_id
from room_315_multi_shuttle import normalize_shuttle_ref
from room_315_observed_state_provider import DEFAULT_SOURCE_PRIORITY
from room_315_observed_state_provider import FusedObservedStateProvider
from room_315_observed_state_provider import ObservedStateProvider


DEFAULT_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'room_315_vla'
    / 'visual_observed_state_calibration.yaml'
)
VISUAL_SCHEMA_VERSION = 1
SHUTTLE_PREDICATES = (
    'visual_bbox',
    'identity',
    'rail_side',
    'rail_position',
    'location_block',
    'location_slot',
    'loaded',
    'present',
)
COMPACT_FORBIDDEN_KEY_TOKENS = (
    'action_vector',
    'command',
    'gazebo',
    'oracle',
    'pddl',
    'plan_step',
    'primitive',
    'privileged',
    'rail_command',
    'stopper',
    'structured_tracker',
    'tracker',
)
COMPACT_TOP_LEVEL_KEYS = frozenset({
    'schema_version',
    'timestamp',
    'calibration_version',
    'model',
    'detections',
    'switches',
    'obstacles',
})
COMPACT_SHUTTLE_KEYS = frozenset({
    'kind',
    'id',
    'camera',
    'bbox',
    'identity',
    'identity_confidence',
    'side',
    'rail_side',
    'confidence',
    'timestamp',
    'loaded_state',
    'loaded_confidence',
    'label',
})
COMPACT_SWITCH_KEYS = frozenset({
    'id',
    'camera',
    'bbox',
    'side',
    'rail_side',
    'name',
    'state',
    'confidence',
    'timestamp',
})
COMPACT_OBSTACLE_KEYS = frozenset({
    'id',
    'camera',
    'bbox',
    'side',
    'rail_side',
    'label',
    'confidence',
    'timestamp',
})


class VisualObservationError(ValueError):
    """Raised when visual detections or calibration cannot be trusted."""


@dataclass(frozen=True)
class RailProjection:
    side: str | None
    segment: str | None
    s_ratio: float | None
    slot: str | None
    point_m: tuple[float, float, float] | None
    distance_m: float | None
    status: str
    reason: str = ''

    @property
    def block_id(self) -> str | None:
        if not self.side or not self.segment:
            return None
        return normalize_fleet_block_id(self.segment, side=self.side)

    @property
    def slot_id(self) -> str | None:
        if not self.side or not self.slot:
            return None
        return f'{self.side}:slot:{self.slot}'


@dataclass
class PreparedDetection:
    raw: dict[str, Any]
    detection_id: str
    camera_name: str
    bbox: list[float]
    timestamp: float
    confidence: float
    subject: str
    identity_value: str | None
    side: str | None
    loaded_value: bool | None
    loaded_known: bool
    projection: RailProjection
    status: str
    reasons: list[str]


class StrictJsonCompactModelAdapter:
    """Parse and validate compact model output.

    The adapter accepts a single JSON object and rejects extra prose, PDDL,
    action-like fields, stopper fields, Gazebo truth, and structured tracker
    data. It returns normal Python dicts for the provider to calibrate.
    """

    def parse(self, raw_output: str | dict[str, Any]) -> dict[str, Any]:
        if isinstance(raw_output, str):
            try:
                payload = json.loads(raw_output)
            except json.JSONDecodeError as exc:
                raise VisualObservationError('compact model output must be strict JSON') from exc
        elif isinstance(raw_output, dict):
            payload = copy.deepcopy(raw_output)
        else:
            raise VisualObservationError('compact model output must be a JSON object')
        if not isinstance(payload, dict):
            raise VisualObservationError('compact model output must be a JSON object')
        _reject_compact_privileged_keys(payload)
        unexpected = sorted(set(payload) - COMPACT_TOP_LEVEL_KEYS)
        if unexpected:
            raise VisualObservationError(f'compact model output has unsupported keys: {unexpected}')
        if int(payload.get('schema_version', VISUAL_SCHEMA_VERSION)) != VISUAL_SCHEMA_VERSION:
            raise VisualObservationError(f'visual compact schema must be {VISUAL_SCHEMA_VERSION}')

        scene = {
            'schema_version': VISUAL_SCHEMA_VERSION,
            'timestamp': _optional_float(payload.get('timestamp')),
            'calibration_version': str(payload.get('calibration_version') or ''),
            'model': copy.deepcopy(payload.get('model') or {}),
            'detections': [
                _normalise_compact_item(item, COMPACT_SHUTTLE_KEYS, context='detections')
                for item in _require_list(payload.get('detections', []), 'detections')
            ],
            'switches': [
                _normalise_compact_item(item, COMPACT_SWITCH_KEYS, context='switches')
                for item in _require_list(payload.get('switches', []), 'switches')
            ],
            'obstacles': [
                _normalise_compact_item(item, COMPACT_OBSTACLE_KEYS, context='obstacles')
                for item in _require_list(payload.get('obstacles', []), 'obstacles')
            ],
        }
        for index, item in enumerate(scene['detections']):
            if str(item.get('kind') or 'shuttle').strip().casefold() != 'shuttle':
                raise VisualObservationError(f'detections[{index}] kind must be shuttle')
            item['bbox'] = _normalise_bbox(item.get('bbox'), f'detections[{index}].bbox')
        for index, item in enumerate(scene['switches']):
            item['bbox'] = _normalise_bbox(item.get('bbox'), f'switches[{index}].bbox')
        for index, item in enumerate(scene['obstacles']):
            item['bbox'] = _normalise_bbox(item.get('bbox'), f'obstacles[{index}].bbox')
        return scene


class DeterministicFixtureCompactModel:
    """Small deterministic compact-model fixture for provider tests."""

    def __init__(self, payload: dict[str, Any] | str) -> None:
        self.payload = payload

    def infer(self, rgbd_streams: dict[str, Any]) -> str | dict[str, Any]:
        if not isinstance(rgbd_streams, dict):
            raise VisualObservationError('fixture model requires a stream mapping')
        return copy.deepcopy(self.payload)


class VisualObservedStateProvider(ObservedStateProvider):
    """Build fused ObservedState from visual RGB-D detections and devices."""

    def __init__(
        self,
        *,
        calibration_path: Path | str | None = None,
        calibration: dict[str, Any] | None = None,
        compact_model: Any | None = None,
        compact_adapter: StrictJsonCompactModelAdapter | None = None,
        trusted_status_snapshot: dict[str, Any] | None = None,
        source_timestamps: dict[str, float] | None = None,
        stale_after_s: float | None = None,
        min_confidence: float | None = None,
        source_priority: tuple[str, ...] = DEFAULT_SOURCE_PRIORITY,
        state_id: str = 'room315-visual-observed-state',
    ) -> None:
        self.calibration = load_visual_calibration(calibration or calibration_path or DEFAULT_CALIBRATION_PATH)
        thresholds = self.calibration.get('thresholds', {})
        self.stale_after_s = float(
            stale_after_s
            if stale_after_s is not None
            else thresholds.get('stale_after_s', 1.0)
        )
        self.min_confidence = float(
            min_confidence
            if min_confidence is not None
            else thresholds.get('min_detection_confidence', 0.6)
        )
        self.min_identity_confidence = float(thresholds.get('min_identity_confidence', 0.65))
        self.min_loaded_confidence = float(thresholds.get('min_loaded_confidence', 0.65))
        self.compact_model = compact_model
        self.compact_adapter = compact_adapter or StrictJsonCompactModelAdapter()
        self.trusted_status_snapshot = copy.deepcopy(trusted_status_snapshot or {})
        self.source_timestamps = dict(source_timestamps or {})
        self.source_priority = tuple(source_priority)
        self.state_id = state_id

    def observe(
        self,
        *,
        timestamp: float | None = None,
        rgbd_streams: dict[str, Any] | None = None,
        model_output: str | dict[str, Any] | None = None,
        trusted_status_snapshot: dict[str, Any] | None = None,
    ) -> ObservedState:
        observed_at = _timestamp_or_zero(timestamp)
        streams = _normalise_streams(rgbd_streams or {})
        raw_output = model_output
        if raw_output is None:
            raw_output = self._run_compact_model(streams)
        scene = self.compact_adapter.parse(raw_output or {'schema_version': VISUAL_SCHEMA_VERSION})
        visual_facts = self.visual_facts_from_scene(scene, streams, observed_at=observed_at)
        trusted = (
            copy.deepcopy(self.trusted_status_snapshot)
            if trusted_status_snapshot is None
            else copy.deepcopy(trusted_status_snapshot)
        )
        return FusedObservedStateProvider(
            trusted,
            visual_facts=visual_facts,
            stale_after_s=self.stale_after_s,
            source_timestamps=self.source_timestamps,
            source_priority=self.source_priority,
            state_id=self.state_id,
        ).observe(timestamp=observed_at)

    def visual_facts_from_scene(
        self,
        scene: dict[str, Any],
        rgbd_streams: dict[str, dict[str, Any]],
        *,
        observed_at: float,
    ) -> list[ObservedFact]:
        prepared = [
            self._prepare_shuttle_detection(item, index, rgbd_streams, observed_at=observed_at)
            for index, item in enumerate(scene.get('detections') or [])
        ]
        inconsistent_subjects = _inconsistent_shuttle_subjects(prepared)
        facts: list[ObservedFact] = []
        for detection in prepared:
            if detection.subject in inconsistent_subjects:
                detection.status = 'unknown'
                detection.reasons.append('inconsistent_duplicate_detection')
            facts.extend(self._facts_for_shuttle_detection(detection))
        facts.extend(self._switch_facts(scene.get('switches') or [], rgbd_streams, observed_at=observed_at))
        facts.extend(self._obstacle_facts(scene.get('obstacles') or [], rgbd_streams, observed_at=observed_at))
        return facts

    def _run_compact_model(self, rgbd_streams: dict[str, dict[str, Any]]) -> str | dict[str, Any]:
        if self.compact_model is None:
            return {'schema_version': VISUAL_SCHEMA_VERSION}
        if hasattr(self.compact_model, 'infer'):
            return self.compact_model.infer(rgbd_streams)
        if callable(self.compact_model):
            return self.compact_model(rgbd_streams)
        raise VisualObservationError('compact_model must be callable or expose infer(rgbd_streams)')

    def _prepare_shuttle_detection(
        self,
        item: dict[str, Any],
        index: int,
        rgbd_streams: dict[str, dict[str, Any]],
        *,
        observed_at: float,
    ) -> PreparedDetection:
        detection_id = str(item.get('id') or f'shuttle-{index}')
        camera_name = str(item.get('camera') or '').strip()
        timestamp = _timestamp_or_zero(
            item.get('timestamp')
            if item.get('timestamp') is not None
            else observed_at
        )
        confidence = _clamp01(item.get('confidence'), default=0.0)
        reasons: list[str] = []
        status = 'known'
        if confidence < self.min_confidence:
            status = 'unknown'
            reasons.append('low_confidence')
        if _is_stale(timestamp, observed_at, self.stale_after_s):
            status = 'unknown'
            reasons.append('stale_detection')

        identity_confidence = _clamp01(item.get('identity_confidence'), default=0.0)
        spec = normalize_shuttle_ref(item.get('identity'), side=item.get('side') or item.get('rail_side'))
        if spec is None or identity_confidence < self.min_identity_confidence:
            status = 'unknown'
            reasons.append('unknown_identity')
            subject = f'visual_detection:{_slug(detection_id)}'
            identity_value = None
        else:
            subject = spec.gazebo_entity_name
            identity_value = spec.short_id

        side = _side_from_detection(item, self.calibration, camera_name)
        if spec is not None and side and spec.side != side:
            status = 'unknown'
            reasons.append('identity_side_inconsistent_with_camera')
        if spec is not None:
            side = spec.side

        bbox = _normalise_bbox(item.get('bbox'), f'detections[{index}].bbox')
        projection = self._project_detection(
            camera_name,
            bbox,
            rgbd_streams,
            side_hint=side,
            reasons=reasons,
        )
        if projection.status != 'known':
            status = 'unknown'
            reasons.append(projection.reason or 'unprojectable_detection')

        loaded_value, loaded_known, loaded_reason = _normalise_loaded_state(
            item.get('loaded_state'),
            confidence=_clamp01(item.get('loaded_confidence'), default=confidence),
            min_confidence=self.min_loaded_confidence,
        )
        if loaded_reason:
            reasons.append(loaded_reason)

        return PreparedDetection(
            raw=copy.deepcopy(item),
            detection_id=detection_id,
            camera_name=camera_name,
            bbox=bbox,
            timestamp=timestamp,
            confidence=confidence,
            subject=subject,
            identity_value=identity_value,
            side=side,
            loaded_value=loaded_value,
            loaded_known=loaded_known and status == 'known',
            projection=projection,
            status=status,
            reasons=_dedupe(reasons),
        )

    def _project_detection(
        self,
        camera_name: str,
        bbox: list[float],
        rgbd_streams: dict[str, dict[str, Any]],
        *,
        side_hint: str | None,
        reasons: list[str],
    ) -> RailProjection:
        if not camera_name:
            return _unknown_projection('missing_camera')
        camera_calibration = _camera_calibration(self.calibration, camera_name)
        if camera_calibration is None:
            return _unknown_projection('unknown_camera')
        stream = rgbd_streams.get(camera_name)
        if not isinstance(stream, dict):
            return _unknown_projection('missing_rgbd_stream')
        camera_info = stream.get('camera_info')
        depth = stream.get('depth')
        if camera_info is None:
            return _unknown_projection('missing_camera_info')
        if depth is None:
            return _unknown_projection('missing_depth_stream')
        center = _bbox_center(bbox)
        depth_m = depth_at_pixel(depth, center)
        if depth_m is None:
            return _unknown_projection('missing_or_invalid_depth')
        depth_scale = float(camera_calibration.get('depth_scale', 1.0) or 1.0)
        try:
            point_m = pixel_depth_to_room_point(
                center,
                depth_m * depth_scale,
                camera_info,
                camera_calibration,
            )
        except VisualObservationError as exc:
            reasons.append(str(exc))
            return _unknown_projection('camera_projection_failed')
        return rail_projection_from_room_point(
            point_m,
            self.calibration,
            side_hint=side_hint or camera_calibration.get('rail_side'),
        )

    def _facts_for_shuttle_detection(self, detection: PreparedDetection) -> list[ObservedFact]:
        base_meta = {
            'detector': 'compact_visual_model',
            'detection_id': detection.detection_id,
            'camera': detection.camera_name,
            'bbox_xywh': [round(value, 6) for value in detection.bbox],
            'detection_timestamp': round(detection.timestamp, 6),
            'reasons': list(detection.reasons),
        }
        if detection.projection.point_m is not None:
            base_meta['calibrated_point_m'] = [round(value, 6) for value in detection.projection.point_m]
        if detection.projection.distance_m is not None:
            base_meta['rail_distance_m'] = round(detection.projection.distance_m, 6)
        if detection.projection.s_ratio is not None:
            base_meta['s_ratio'] = round(detection.projection.s_ratio, 6)
        if detection.projection.segment:
            base_meta['segment'] = detection.projection.segment
        if detection.projection.slot:
            base_meta['slot'] = detection.projection.slot
        status = detection.status
        known = status == 'known'
        confidence = detection.confidence if known else min(detection.confidence, self.min_confidence)
        loaded_known = detection.loaded_known and known
        values = {
            'visual_bbox': {
                'camera': detection.camera_name,
                'bbox_xywh': [round(value, 6) for value in detection.bbox],
            } if known else None,
            'identity': detection.identity_value if known else None,
            'rail_side': detection.side if known else None,
            'rail_position': {
                'side': detection.projection.side,
                'segment': detection.projection.segment,
                's_ratio': round(detection.projection.s_ratio, 6)
                if detection.projection.s_ratio is not None else None,
                'slot': detection.projection.slot,
            } if known else None,
            'location_block': detection.projection.block_id if known else None,
            'location_slot': detection.projection.slot_id if known and detection.projection.slot_id else None,
            'loaded': detection.loaded_value if loaded_known else None,
            'present': True if known and detection.identity_value else None,
        }
        facts: list[ObservedFact] = []
        for predicate in SHUTTLE_PREDICATES:
            value = values[predicate]
            predicate_known = value is not None
            fact_status = 'known' if predicate_known and known else 'unknown'
            facts.append(_visual_fact(
                subject=detection.subject,
                predicate=predicate,
                value=value,
                timestamp=detection.timestamp,
                confidence=confidence if predicate_known else 0.0,
                status=fact_status,
                metadata=base_meta,
            ))
        return facts

    def _switch_facts(
        self,
        raw_switches: list[dict[str, Any]],
        rgbd_streams: dict[str, dict[str, Any]],
        *,
        observed_at: float,
    ) -> list[ObservedFact]:
        grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
        for item in raw_switches:
            side = _side_from_detection(item, self.calibration, str(item.get('camera') or ''))
            name = _device_name(item.get('name'))
            if side and name:
                grouped.setdefault((side, name), []).append(item)
        facts: list[ObservedFact] = []
        for (side, name), items in sorted(grouped.items()):
            states = []
            timestamps = []
            confidences = []
            bboxes = []
            reasons: list[str] = []
            for item in items:
                timestamp = _timestamp_or_zero(
                    item.get('timestamp')
                    if item.get('timestamp') is not None
                    else observed_at
                )
                confidence = _clamp01(item.get('confidence'), default=0.0)
                state = _normalise_switch_state(item.get('state'))
                if confidence < self.min_confidence:
                    reasons.append('low_confidence')
                    state = None
                if _is_stale(timestamp, observed_at, self.stale_after_s):
                    reasons.append('stale_detection')
                    state = None
                if state is not None:
                    states.append(state)
                timestamps.append(timestamp)
                confidences.append(confidence)
                bboxes.append(_normalise_bbox(item.get('bbox'), 'switch.bbox'))
            status = 'known'
            value = states[0] if states else None
            if not states:
                status = 'unknown'
            elif len(set(states)) > 1:
                status = 'unknown'
                value = None
                reasons.append('inconsistent_switch_state')
            timestamp = max(timestamps) if timestamps else observed_at
            facts.append(_visual_fact(
                subject=f'{side}:switch:{name}',
                predicate='state',
                value=value,
                timestamp=timestamp,
                confidence=min(confidences) if value is not None else 0.0,
                status=status,
                metadata={
                    'detector': 'compact_visual_model',
                    'visible_device': True,
                    'device_kind': 'switch',
                    'side': side,
                    'device': name,
                    'bbox_xywh': [[round(value, 6) for value in bbox] for bbox in bboxes],
                    'reasons': _dedupe(reasons),
                },
            ))
        return facts

    def _obstacle_facts(
        self,
        raw_obstacles: list[dict[str, Any]],
        rgbd_streams: dict[str, dict[str, Any]],
        *,
        observed_at: float,
    ) -> list[ObservedFact]:
        evidence_by_side: dict[str, list[dict[str, Any]]] = {side: [] for side in SIDES}
        unknown_by_side: dict[str, list[str]] = {side: [] for side in SIDES}
        for index, item in enumerate(raw_obstacles):
            camera_name = str(item.get('camera') or '').strip()
            side = _side_from_detection(item, self.calibration, camera_name)
            timestamp = _timestamp_or_zero(
                item.get('timestamp')
                if item.get('timestamp') is not None
                else observed_at
            )
            confidence = _clamp01(item.get('confidence'), default=0.0)
            reasons: list[str] = []
            if confidence < self.min_confidence:
                reasons.append('low_confidence')
            if _is_stale(timestamp, observed_at, self.stale_after_s):
                reasons.append('stale_detection')
            bbox = _normalise_bbox(item.get('bbox'), 'obstacle.bbox')
            projection = self._project_detection(camera_name, bbox, rgbd_streams, side_hint=side, reasons=reasons)
            side = projection.side or side
            if side not in SIDES:
                unknown_by_side['right'].append('unknown_side')
                continue
            if projection.status != 'known':
                reasons.append(projection.reason or 'unprojectable_obstacle')
            if reasons:
                unknown_by_side[side].extend(reasons)
                continue
            obstacle_id = str(item.get('id') or item.get('label') or f'obstacle-{index}')
            evidence_by_side[side].append({
                'id': obstacle_id,
                'label': str(item.get('label') or obstacle_id),
                'camera': camera_name,
                'bbox_xywh': [round(value, 6) for value in bbox],
                'segment': projection.segment,
                'slot': projection.slot,
                's_ratio': round(projection.s_ratio, 6) if projection.s_ratio is not None else None,
                'confidence': round(confidence, 6),
                'timestamp': round(timestamp, 6),
            })

        facts: list[ObservedFact] = []
        for side in SIDES:
            evidence = sorted(evidence_by_side[side], key=lambda item: item['id'])
            reasons = _dedupe(unknown_by_side[side])
            if evidence:
                facts.append(_visual_fact(
                    subject=f'{side}:obstacles',
                    predicate='present_obstacles',
                    value=[item['id'] for item in evidence],
                    timestamp=observed_at,
                    confidence=min(float(item['confidence']) for item in evidence),
                    status='known',
                    metadata={'detector': 'compact_visual_model', 'side': side, 'reasons': reasons},
                ))
                facts.append(_visual_fact(
                    subject=f'{side}:obstacles',
                    predicate='obstacle_evidence',
                    value=evidence,
                    timestamp=observed_at,
                    confidence=min(float(item['confidence']) for item in evidence),
                    status='known',
                    metadata={'detector': 'compact_visual_model', 'side': side, 'reasons': reasons},
                ))
            elif reasons:
                facts.append(_visual_fact(
                    subject=f'{side}:obstacles',
                    predicate='obstacle_evidence',
                    value=None,
                    timestamp=observed_at,
                    confidence=0.0,
                    status='unknown',
                    metadata={'detector': 'compact_visual_model', 'side': side, 'reasons': reasons},
                ))
        return facts


def load_visual_calibration(path_or_config: Path | str | dict[str, Any]) -> dict[str, Any]:
    if isinstance(path_or_config, dict):
        config = copy.deepcopy(path_or_config)
    else:
        config = yaml.safe_load(Path(path_or_config).expanduser().read_text(encoding='utf-8')) or {}
    if not isinstance(config, dict):
        raise VisualObservationError('visual calibration must be a YAML mapping')
    if int(config.get('schema_version', 0)) != VISUAL_SCHEMA_VERSION:
        raise VisualObservationError(f'visual calibration schema must be {VISUAL_SCHEMA_VERSION}')
    cameras = config.get('cameras')
    if not isinstance(cameras, dict) or not cameras:
        raise VisualObservationError('visual calibration needs cameras')
    for name, camera in cameras.items():
        if not isinstance(camera, dict):
            raise VisualObservationError(f'camera {name} must be a mapping')
        _matrix4(camera.get('room_from_camera'), f'camera {name} room_from_camera')
        if camera.get('rail_side') is not None and str(camera.get('rail_side')) not in SIDES:
            raise VisualObservationError(f'camera {name} rail_side must be right or left')
    rail_geometry = config.get('rail_geometry')
    if not isinstance(rail_geometry, dict):
        raise VisualObservationError('visual calibration needs rail_geometry')
    for side in SIDES:
        side_geometry = rail_geometry.get(side)
        if not isinstance(side_geometry, dict):
            raise VisualObservationError(f'rail_geometry.{side} must be a mapping')
        for collection in ('segments', 'slots'):
            if not isinstance(side_geometry.get(collection), dict):
                raise VisualObservationError(f'rail_geometry.{side}.{collection} must be a mapping')
    return config


def pixel_depth_to_room_point(
    pixel_uv: tuple[float, float],
    depth_m: float,
    camera_info: Any,
    camera_calibration: dict[str, Any],
) -> tuple[float, float, float]:
    k = _camera_intrinsics(camera_info)
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if fx == 0.0 or fy == 0.0:
        raise VisualObservationError('CameraInfo intrinsics must have non-zero focal lengths')
    depth = _finite_positive(depth_m, 'depth_m')
    u, v = float(pixel_uv[0]), float(pixel_uv[1])
    point_camera = (
        (u - cx) * depth / fx,
        (v - cy) * depth / fy,
        depth,
        1.0,
    )
    matrix = _matrix4(camera_calibration.get('room_from_camera'), 'room_from_camera')
    transformed = [
        sum(matrix[row][col] * point_camera[col] for col in range(4))
        for row in range(4)
    ]
    w = transformed[3] if transformed[3] else 1.0
    return (
        transformed[0] / w,
        transformed[1] / w,
        transformed[2] / w,
    )


def rail_projection_from_room_point(
    point_m: tuple[float, float, float],
    calibration: dict[str, Any],
    *,
    side_hint: str | None = None,
) -> RailProjection:
    sides = [side_hint] if side_hint in SIDES else list(SIDES)
    best: tuple[float, str, str, float] | None = None
    for side in sides:
        side_geometry = calibration['rail_geometry'][side]
        for segment, entry in side_geometry['segments'].items():
            start = _point3(entry.get('start_m'), f'{side}.{segment}.start_m')
            end = _point3(entry.get('end_m'), f'{side}.{segment}.end_m')
            distance, s_ratio = _distance_to_segment(point_m, start, end)
            if best is None or distance < best[0]:
                best = (distance, side, str(segment), s_ratio)
    if best is None:
        return _unknown_projection('no_rail_segments')
    distance, side, segment, s_ratio = best
    segment_config = calibration['rail_geometry'][side]['segments'][segment]
    max_distance = float(segment_config.get('max_distance_m', 0.25))
    if distance > max_distance:
        return RailProjection(
            side=side,
            segment=None,
            s_ratio=None,
            slot=None,
            point_m=point_m,
            distance_m=distance,
            status='unknown',
            reason='point_too_far_from_calibrated_rail',
        )
    slot = _closest_slot(point_m, calibration['rail_geometry'][side].get('slots') or {})
    return RailProjection(
        side=side,
        segment=segment,
        s_ratio=s_ratio,
        slot=slot,
        point_m=point_m,
        distance_m=distance,
        status='known',
    )


def depth_at_pixel(depth_image: Any, pixel_uv: tuple[float, float]) -> float | None:
    width, height = _image_shape(depth_image)
    if width <= 0 or height <= 0:
        return None
    u = int(round(float(pixel_uv[0])))
    v = int(round(float(pixel_uv[1])))
    if u < 0 or v < 0 or u >= width or v >= height:
        return None
    try:
        raw_value = depth_image[v][u]
    except Exception:
        try:
            raw_value = depth_image[v, u]
        except Exception:
            return None
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value <= 0.0:
        return None
    return value


def _visual_fact(
    *,
    subject: str,
    predicate: str,
    value: Any,
    timestamp: float,
    confidence: float,
    status: str,
    metadata: dict[str, Any],
) -> ObservedFact:
    return ObservedFact(
        fact_id=f'visual-{_slug(subject)}-{_slug(predicate)}',
        subject=subject,
        predicate=predicate,
        value=copy.deepcopy(value),
        source='visual_model',
        timestamp=timestamp,
        confidence=_clamp01(confidence, default=0.0),
        status=status,
        metadata=copy.deepcopy(metadata),
    )


def _reject_compact_privileged_keys(value: Any, path: str = '$') -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            key_text = str(key).casefold()
            if any(token in key_text for token in COMPACT_FORBIDDEN_KEY_TOKENS):
                raise VisualObservationError(f'compact visual output contains forbidden key {path}.{key}')
            _reject_compact_privileged_keys(child, f'{path}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_compact_privileged_keys(child, f'{path}[{index}]')


def _normalise_compact_item(item: Any, allowed_keys: frozenset[str], *, context: str) -> dict[str, Any]:
    if not isinstance(item, dict):
        raise VisualObservationError(f'{context} items must be objects')
    unexpected = sorted(set(item) - allowed_keys)
    if unexpected:
        raise VisualObservationError(f'{context} item has unsupported keys: {unexpected}')
    return copy.deepcopy(item)


def _require_list(value: Any, name: str) -> list[Any]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise VisualObservationError(f'{name} must be a list')
    return value


def _normalise_streams(streams: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if not isinstance(streams, dict):
        raise VisualObservationError('rgbd_streams must be a mapping')
    result: dict[str, dict[str, Any]] = {}
    for name, stream in streams.items():
        if not isinstance(stream, dict):
            raise VisualObservationError(f'rgbd stream {name!r} must be a mapping')
        result[str(name)] = stream
    return result


def _camera_calibration(calibration: dict[str, Any], camera_name: str) -> dict[str, Any] | None:
    cameras = calibration.get('cameras')
    if not isinstance(cameras, dict):
        return None
    camera = cameras.get(camera_name)
    return camera if isinstance(camera, dict) else None


def _camera_intrinsics(camera_info: Any) -> list[float]:
    raw = _attr_or_key(camera_info, 'k')
    if raw is None:
        raw = _attr_or_key(camera_info, 'K')
    if raw is None:
        raw = _attr_or_key(camera_info, 'intrinsic_matrix')
    if raw is None:
        raise VisualObservationError('CameraInfo must provide k/K intrinsics')
    values = [float(value) for value in list(raw)]
    if len(values) != 9:
        raise VisualObservationError('CameraInfo intrinsics must have 9 values')
    if not all(math.isfinite(value) for value in values):
        raise VisualObservationError('CameraInfo intrinsics must be finite')
    return values


def _attr_or_key(value: Any, name: str) -> Any:
    if isinstance(value, dict):
        return value.get(name)
    if hasattr(value, name):
        return getattr(value, name)
    return None


def _normalise_bbox(value: Any, context: str) -> list[float]:
    if not isinstance(value, list) and not isinstance(value, tuple):
        raise VisualObservationError(f'{context} must be [x, y, width, height]')
    if len(value) != 4:
        raise VisualObservationError(f'{context} must have four values')
    bbox = [float(item) for item in value]
    if not all(math.isfinite(item) for item in bbox):
        raise VisualObservationError(f'{context} values must be finite')
    if bbox[2] <= 0.0 or bbox[3] <= 0.0:
        raise VisualObservationError(f'{context} width and height must be positive')
    return bbox


def _bbox_center(bbox: list[float]) -> tuple[float, float]:
    return (bbox[0] + bbox[2] / 2.0, bbox[1] + bbox[3] / 2.0)


def _image_shape(image: Any) -> tuple[int, int]:
    shape = getattr(image, 'shape', None)
    if shape is not None and len(shape) >= 2:
        return int(shape[1]), int(shape[0])
    if isinstance(image, list) and image and isinstance(image[0], list):
        return len(image[0]), len(image)
    return 0, 0


def _matrix4(value: Any, context: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 4:
        raise VisualObservationError(f'{context} must be a 4x4 matrix')
    rows: list[list[float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 4:
            raise VisualObservationError(f'{context} must be a 4x4 matrix')
        parsed = [float(item) for item in row]
        if not all(math.isfinite(item) for item in parsed):
            raise VisualObservationError(f'{context} values must be finite')
        rows.append(parsed)
    return rows


def _point3(value: Any, context: str) -> tuple[float, float, float]:
    if not isinstance(value, list) and not isinstance(value, tuple):
        raise VisualObservationError(f'{context} must be [x, y, z]')
    if len(value) != 3:
        raise VisualObservationError(f'{context} must have three values')
    point = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in point):
        raise VisualObservationError(f'{context} values must be finite')
    return point


def _finite_positive(value: Any, context: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise VisualObservationError(f'{context} must be numeric') from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise VisualObservationError(f'{context} must be finite and positive')
    return parsed


def _distance_to_segment(
    point: tuple[float, float, float],
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, float]:
    vector = tuple(end[index] - start[index] for index in range(3))
    length_sq = sum(value * value for value in vector)
    if length_sq <= 0.0:
        distance = math.sqrt(sum((point[index] - start[index]) ** 2 for index in range(3)))
        return distance, 0.0
    raw_ratio = sum((point[index] - start[index]) * vector[index] for index in range(3)) / length_sq
    ratio = max(0.0, min(1.0, raw_ratio))
    closest = tuple(start[index] + vector[index] * ratio for index in range(3))
    distance = math.sqrt(sum((point[index] - closest[index]) ** 2 for index in range(3)))
    return distance, ratio


def _closest_slot(point_m: tuple[float, float, float], slots: dict[str, Any]) -> str | None:
    best: tuple[float, str] | None = None
    for slot, entry in slots.items():
        if not isinstance(entry, dict):
            continue
        center = _point3(entry.get('center_m'), f'slot {slot} center_m')
        radius = float(entry.get('radius_m', 0.2))
        distance = math.sqrt(sum((point_m[index] - center[index]) ** 2 for index in range(3)))
        if distance <= radius and (best is None or distance < best[0]):
            best = (distance, str(slot))
    return best[1] if best is not None else None


def _unknown_projection(reason: str) -> RailProjection:
    return RailProjection(
        side=None,
        segment=None,
        s_ratio=None,
        slot=None,
        point_m=None,
        distance_m=None,
        status='unknown',
        reason=reason,
    )


def _inconsistent_shuttle_subjects(detections: list[PreparedDetection]) -> set[str]:
    grouped: dict[str, list[PreparedDetection]] = {}
    for detection in detections:
        if detection.identity_value:
            grouped.setdefault(detection.subject, []).append(detection)
    inconsistent: set[str] = set()
    for subject, items in grouped.items():
        known_locations = {
            (
                item.projection.side,
                item.projection.segment,
                round(item.projection.s_ratio or -1.0, 2),
                item.projection.slot,
            )
            for item in items
            if item.status == 'known'
        }
        loaded_values = {
            item.loaded_value
            for item in items
            if item.status == 'known' and item.loaded_known
        }
        if len(known_locations) > 1 or len(loaded_values) > 1:
            inconsistent.add(subject)
    return inconsistent


def _normalise_loaded_state(
    value: Any,
    *,
    confidence: float,
    min_confidence: float,
) -> tuple[bool | None, bool, str]:
    text = str(value or '').strip().casefold()
    if text in {'loaded', 'load', 'with_payload', 'occupied', 'full'} and confidence >= min_confidence:
        return True, True, ''
    if text in {'empty', 'unloaded', 'without_payload', 'none', 'no_payload'} and confidence >= min_confidence:
        return False, True, ''
    if text in {'loaded', 'empty', 'load', 'unloaded', 'with_payload', 'without_payload'}:
        return None, False, 'low_loaded_confidence'
    return None, False, 'unknown_loaded_state'


def _side_from_detection(item: dict[str, Any], calibration: dict[str, Any], camera_name: str) -> str | None:
    raw = item.get('side') if item.get('side') is not None else item.get('rail_side')
    if raw is not None:
        text = str(raw).strip().casefold()
        if text in SIDES:
            return text
    camera = _camera_calibration(calibration, camera_name)
    if camera is not None and camera.get('rail_side') in SIDES:
        return str(camera.get('rail_side'))
    return None


def _device_name(value: Any) -> str | None:
    text = str(value or '').strip().upper()
    return text if text in DEVICE_NAMES else None


def _normalise_switch_state(value: Any) -> str | None:
    text = str(value or '').strip().casefold()
    if text in {'e', 'exterior', 'external'}:
        return 'EXTERIOR'
    if text in {'i', 'interior', 'internal'}:
        return 'INTERIOR'
    return None


def _timestamp_or_zero(value: Any) -> float:
    if value is None:
        return 0.0
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(parsed) or parsed < 0.0:
        return 0.0
    return parsed


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _is_stale(source_timestamp: float, observed_at: float, stale_after_s: float) -> bool:
    return observed_at - source_timestamp > max(0.0, stale_after_s)


def _clamp01(value: Any, *, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        parsed = default
    if not math.isfinite(parsed):
        parsed = default
    return max(0.0, min(1.0, parsed))


def _dedupe(items: list[str]) -> list[str]:
    return sorted({str(item) for item in items if str(item)})


def _slug(value: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '-', str(value)).strip('-') or 'fact'


def main() -> int:
    adapter = StrictJsonCompactModelAdapter()
    scene = adapter.parse(sys.stdin.read())
    print(json.dumps(scene, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
