#!/usr/bin/env python3
"""Pure deterministic validation and stabilization for Room 315 predictions."""

from __future__ import annotations

import math
import sys
from collections import Counter
from collections import deque
from dataclasses import dataclass
from dataclasses import replace
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_presence_provider import PRESENCE_ABSENT
from room_315_presence_provider import PRESENCE_PRESENT
from room_315_presence_provider import PRESENCE_UNKNOWN
from room_315_presence_provider import PresenceSnapshot
from room_315_visual_fleet import AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY
from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_fleet import identity_side
from room_315_visual_runtime import DecodedShuttlePrediction
from room_315_visual_runtime import DecodedVisualPrediction


class PredictionValidationError(ValueError):
    """Raised for invalid validator configuration."""


@dataclass(frozen=True)
class ValidationConfig:
    stale_image_timeout_s: float = 1.0
    maximum_timestamp_difference_s: float = 0.1
    s_ratio_tolerance: float = 0.02
    s_m_tolerance_m: float = 0.02
    position_consistency_tolerance_m: float = 0.08
    reconcile_position_consistency: bool = False
    max_position_reconciliation_error_m: float = 0.40

    def __post_init__(self) -> None:
        for field_name in (
            'stale_image_timeout_s',
            'maximum_timestamp_difference_s',
            's_ratio_tolerance',
            's_m_tolerance_m',
            'position_consistency_tolerance_m',
            'max_position_reconciliation_error_m',
        ):
            value = float(getattr(self, field_name))
            if not math.isfinite(value) or value < 0.0:
                raise PredictionValidationError(
                    f'{field_name} must be finite and non-negative'
                )
        if (
            self.max_position_reconciliation_error_m
            < self.position_consistency_tolerance_m
        ):
            raise PredictionValidationError(
                'max_position_reconciliation_error_m must be greater than '
                'or equal to position_consistency_tolerance_m'
            )


@dataclass(frozen=True)
class ValidationResult:
    accepted: bool
    reasons: tuple[str, ...]
    clamped_fields: tuple[str, ...]
    topology_consistent: bool
    timestamp_consistent: bool
    artifact_healthy: bool
    input_healthy: bool
    presence_ready: bool
    prediction: DecodedVisualPrediction | None


def validate_prediction(
    prediction: DecodedVisualPrediction | None,
    presence: PresenceSnapshot,
    *,
    now_s: float,
    config: ValidationConfig | None = None,
    artifact_healthy: bool = True,
    input_healthy: bool = True,
) -> ValidationResult:
    """Validate a synchronized fixed-slot prediction and fail closed."""

    cfg = config or ValidationConfig()
    reasons: list[str] = []
    clamped: list[str] = []
    topology_consistent = True
    timestamp_consistent = True

    now = _finite(now_s, 'now_s')
    if not artifact_healthy:
        reasons.append('artifact_not_healthy')
    if not input_healthy:
        reasons.append('input_not_healthy')
    if not presence.ready:
        reasons.extend(presence.reasons or ('presence_registry_not_ready',))
    presence_by_id = presence.by_identity()
    if tuple(presence_by_id) != tuple(FIXED_VISUAL_SHUTTLE_IDENTITIES):
        reasons.append('presence_identity_order_invalid')
    if any(
        entry.state == PRESENCE_UNKNOWN
        for entry in presence.entries
    ):
        reasons.append('presence_contains_unknown_slot')

    if prediction is None:
        reasons.append('prediction_unavailable')
        return _result(
            reasons,
            clamped,
            topology_consistent=False,
            timestamp_consistent=False,
            artifact_healthy=artifact_healthy,
            input_healthy=input_healthy,
            presence_ready=presence.ready,
            prediction=None,
        )

    expected_present = tuple(
        identity
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if presence_by_id.get(identity)
        and presence_by_id[identity].state == PRESENCE_PRESENT
    )
    expected_absent = tuple(
        identity
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if presence_by_id.get(identity)
        and presence_by_id[identity].state == PRESENCE_ABSENT
    )
    if prediction.active_identities != expected_present:
        reasons.append('active_identity_set_does_not_match_presence_registry')
    if prediction.absent_identities != expected_absent:
        reasons.append('absent_identity_set_does_not_match_presence_registry')
    predicted_ids = tuple(item.identity for item in prediction.shuttles)
    if predicted_ids != expected_present:
        reasons.append('decoded_slot_order_does_not_match_active_identity_order')

    if now < prediction.timestamp_s:
        reasons.append('prediction_timestamp_is_in_the_future')
        timestamp_consistent = False
    elif now - prediction.timestamp_s > cfg.stale_image_timeout_s:
        reasons.append('prediction_is_stale')
        timestamp_consistent = False
    for label, stamp in (
        ('left', prediction.left_image_stamp_s),
        ('right', prediction.right_image_stamp_s),
    ):
        if now < stamp:
            reasons.append(f'{label}_image_timestamp_is_in_the_future')
            timestamp_consistent = False
        elif now - stamp > cfg.stale_image_timeout_s:
            reasons.append(f'{label}_image_is_stale')
            timestamp_consistent = False
    if (
        abs(prediction.left_image_stamp_s - prediction.right_image_stamp_s)
        > cfg.maximum_timestamp_difference_s
    ):
        reasons.append('paired_image_timestamp_skew_exceeded')
        timestamp_consistent = False

    validated: list[DecodedShuttlePrediction] = []
    seen: set[str] = set()
    vocabulary = set(AUTHORITATIVE_GLOBAL_BLOCK_VOCABULARY)
    for shuttle in prediction.shuttles:
        prefix = shuttle.identity
        if shuttle.identity in seen:
            reasons.append(f'duplicate_decoded_identity:{prefix}')
            continue
        seen.add(shuttle.identity)
        if shuttle.identity not in FIXED_VISUAL_SHUTTLE_IDENTITIES:
            reasons.append(f'unknown_decoded_identity:{prefix}')
            continue
        expected_side = identity_side(shuttle.identity)
        if shuttle.side not in {'left', 'right'}:
            reasons.append(f'invalid_side:{prefix}:{shuttle.side}')
            topology_consistent = False
        elif shuttle.side != expected_side:
            reasons.append(
                f'identity_side_conflict:{prefix}:{shuttle.side}!={expected_side}'
            )
            topology_consistent = False
        if shuttle.block not in vocabulary:
            reasons.append(f'invalid_block:{prefix}:{shuttle.block}')
            topology_consistent = False
        if shuttle.loaded_state not in {'loaded', 'empty'}:
            reasons.append(f'invalid_loaded_state:{prefix}:{shuttle.loaded_state}')

        bbox = tuple(float(value) for value in shuttle.bbox_xywh)
        if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
            reasons.append(f'invalid_bbox_nonfinite:{prefix}')
        else:
            width, height = (
                prediction.left_image_size
                if expected_side == 'left'
                else prediction.right_image_size
            )
            x, y, box_w, box_h = bbox
            if box_w <= 0.0 or box_h <= 0.0:
                reasons.append(f'invalid_bbox_extent:{prefix}')
            elif x + box_w <= 0.0 or y + box_h <= 0.0 or x >= width or y >= height:
                reasons.append(f'bbox_outside_image:{prefix}')

        s_ratio = float(shuttle.s_ratio)
        s_m = float(shuttle.s_m)
        length = float(shuttle.segment_length_m)
        if not all(math.isfinite(value) for value in (s_ratio, s_m, length)):
            reasons.append(f'nonfinite_rail_position:{prefix}')
            validated.append(shuttle)
            continue
        if length <= 0.0:
            reasons.append(f'invalid_segment_length:{prefix}')
        if s_ratio < -cfg.s_ratio_tolerance or s_ratio > 1.0 + cfg.s_ratio_tolerance:
            reasons.append(f's_ratio_out_of_range:{prefix}')
        elif s_ratio < 0.0 or s_ratio > 1.0:
            s_ratio = min(1.0, max(0.0, s_ratio))
            clamped.append(f'{prefix}.s_ratio')
        if s_m < -cfg.s_m_tolerance_m:
            reasons.append(f's_m_negative:{prefix}')
        elif s_m < 0.0:
            s_m = 0.0
            clamped.append(f'{prefix}.s_m')
        if length > 0.0:
            if s_m > length + cfg.s_m_tolerance_m:
                reasons.append(f's_m_exceeds_segment_length:{prefix}')
            expected_s_m = s_ratio * length
            consistency_error_m = abs(s_m - expected_s_m)
            if consistency_error_m > cfg.position_consistency_tolerance_m:
                projected_ratio = s_m / length
                can_reconcile = (
                    cfg.reconcile_position_consistency
                    and consistency_error_m
                    <= cfg.max_position_reconciliation_error_m
                    and 0.0 <= projected_ratio <= 1.0
                )
                if can_reconcile:
                    # s_ratio is redundant. This bounded projection uses only
                    # learned visual outputs (s_m and segment_length_m); it
                    # never reads controller/Gazebo pose or oracle labels.
                    s_ratio = projected_ratio
                    clamped.append(
                        f'{prefix}.s_ratio_consistency_projection'
                    )
                else:
                    reasons.append(f's_m_s_ratio_inconsistent:{prefix}')
        validated.append(replace(shuttle, s_m=s_m, s_ratio=s_ratio))

    updated = replace(prediction, shuttles=tuple(validated))
    return _result(
        reasons,
        clamped,
        topology_consistent=topology_consistent,
        timestamp_consistent=timestamp_consistent,
        artifact_healthy=artifact_healthy,
        input_healthy=input_healthy,
        presence_ready=presence.ready,
        prediction=updated,
    )


class DeterministicTemporalStabilizer:
    """Optional categorical majority and numeric EMA; disabled by default."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        majority_window: int = 3,
        ema_alpha: float = 0.5,
    ) -> None:
        if int(majority_window) <= 0:
            raise PredictionValidationError('majority_window must be positive')
        if not math.isfinite(float(ema_alpha)) or not 0.0 < float(ema_alpha) <= 1.0:
            raise PredictionValidationError('ema_alpha must be in (0,1]')
        self.enabled = bool(enabled)
        self.majority_window = int(majority_window)
        self.ema_alpha = float(ema_alpha)
        self.reset()

    def reset(self) -> None:
        self._categories: dict[str, deque[tuple[str, str, str]]] = {}
        self._numeric: dict[str, tuple[tuple[float, ...], float, float, float]] = {}

    def apply(
        self,
        result: ValidationResult,
    ) -> tuple[ValidationResult, bool]:
        if not result.accepted or result.prediction is None:
            self.reset()
            return result, False
        if not self.enabled:
            return result, False

        active = set(result.prediction.active_identities)
        self._categories = {
            key: value
            for key, value in self._categories.items()
            if key in active
        }
        self._numeric = {
            key: value
            for key, value in self._numeric.items()
            if key in active
        }
        stabilized: list[DecodedShuttlePrediction] = []
        for shuttle in result.prediction.shuttles:
            categories = self._categories.setdefault(
                shuttle.identity,
                deque(maxlen=self.majority_window),
            )
            categories.append((shuttle.side, shuttle.block, shuttle.loaded_state))
            side = _majority([item[0] for item in categories])
            block = _majority([item[1] for item in categories])
            loaded = _majority([item[2] for item in categories])

            current = (
                tuple(shuttle.bbox_xywh),
                shuttle.s_m,
                shuttle.s_ratio,
                shuttle.segment_length_m,
            )
            previous = self._numeric.get(shuttle.identity)
            if previous is None:
                numeric = current
            else:
                alpha = self.ema_alpha
                numeric = (
                    tuple(
                        alpha * now + (1.0 - alpha) * old
                        for now, old in zip(current[0], previous[0])
                    ),
                    alpha * current[1] + (1.0 - alpha) * previous[1],
                    alpha * current[2] + (1.0 - alpha) * previous[2],
                    alpha * current[3] + (1.0 - alpha) * previous[3],
                )
            self._numeric[shuttle.identity] = numeric
            stabilized.append(DecodedShuttlePrediction(
                identity=shuttle.identity,
                side=side,
                block=block,
                bbox_xywh=numeric[0],
                s_m=numeric[1],
                s_ratio=numeric[2],
                segment_length_m=numeric[3],
                loaded_state=loaded,
            ))
        return replace(
            result,
            prediction=replace(
                result.prediction,
                shuttles=tuple(stabilized),
            ),
        ), True


def _result(
    reasons: list[str],
    clamped: list[str],
    *,
    topology_consistent: bool,
    timestamp_consistent: bool,
    artifact_healthy: bool,
    input_healthy: bool,
    presence_ready: bool,
    prediction: DecodedVisualPrediction | None,
) -> ValidationResult:
    unique_reasons = tuple(dict.fromkeys(reasons))
    return ValidationResult(
        accepted=not unique_reasons,
        reasons=unique_reasons,
        clamped_fields=tuple(dict.fromkeys(clamped)),
        topology_consistent=topology_consistent,
        timestamp_consistent=timestamp_consistent,
        artifact_healthy=artifact_healthy,
        input_healthy=input_healthy,
        presence_ready=presence_ready,
        prediction=prediction,
    )


def _majority(values: list[str]) -> str:
    counts = Counter(values)
    maximum = max(counts.values())
    # Deterministic newest-value tie break.
    for value in reversed(values):
        if counts[value] == maximum:
            return value
    raise AssertionError('majority input unexpectedly empty')


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PredictionValidationError(f'{name} must be numeric') from exc
    if not math.isfinite(result) or result < 0.0:
        raise PredictionValidationError(f'{name} must be finite and non-negative')
    return result
