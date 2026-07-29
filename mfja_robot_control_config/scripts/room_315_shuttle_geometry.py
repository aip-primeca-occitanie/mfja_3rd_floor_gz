#!/usr/bin/env python3
"""Shared world-space geometry checks for Room 315 rail shuttles."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Iterable

from room_315_kinematic_shuttle import CUBIC_HERMITE_PATH_BACKEND, RailNetwork
from room_315_rail_defaults import (
    LEFT_CALIBRATION_DEFAULTS,
    LEFT_PUBLIC_SEGMENT_NAME_MAP,
    RIGHT_CALIBRATION_DEFAULTS,
    apply_rail_pose_calibration,
    default_rail_network_path,
)


# These values mirror the collision box in the Room 315 shuttle SDF models.
SHUTTLE_COLLISION_CENTER_X_M = -0.078417048
SHUTTLE_COLLISION_SIZE_M = (0.36, 0.22, 0.17)

# Require a visible physical gap, not merely non-penetrating collision boxes.
MIN_SHUTTLE_FOOTPRINT_CLEARANCE_M = 0.04


class ShuttleGeometryError(ValueError):
    """Raised when a shuttle position cannot be mapped to world geometry."""


@dataclass(frozen=True)
class ShuttleWorldPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class ShuttlePosition:
    shuttle_id: str
    side: str
    segment: str
    s_ratio: float


@lru_cache(maxsize=2)
def _rail_network(side: str) -> RailNetwork:
    normalized = _normalized_side(side)
    return RailNetwork.from_yaml(
        default_rail_network_path(normalized),
        path_backend=CUBIC_HERMITE_PATH_BACKEND,
    )


def _normalized_side(side: str) -> str:
    normalized = str(side or '').strip().lower()
    if normalized not in {'right', 'left'}:
        raise ShuttleGeometryError(f'unsupported rail side: {side!r}')
    return normalized


def _canonical_segment(side: str, public_segment: str) -> str:
    normalized_side = _normalized_side(side)
    segment = str(public_segment or '').strip().upper()
    if normalized_side == 'left':
        public_to_canonical = {
            public_name: canonical_name
            for canonical_name, public_name in LEFT_PUBLIC_SEGMENT_NAME_MAP.items()
        }
        segment = public_to_canonical.get(segment, segment)
    network = _rail_network(normalized_side)
    if segment not in network.segments:
        raise ShuttleGeometryError(
            f'unknown {normalized_side} rail segment: {public_segment!r}'
        )
    return segment


def shuttle_world_pose(
    side: str,
    public_segment: str,
    s_ratio: float,
) -> ShuttleWorldPose:
    """Resolve a public segment ratio to the calibrated Gazebo model pose."""
    normalized_side = _normalized_side(side)
    try:
        ratio = float(s_ratio)
    except (TypeError, ValueError) as exc:
        raise ShuttleGeometryError(f'invalid s_ratio: {s_ratio!r}') from exc
    if not 0.0 <= ratio <= 1.0:
        raise ShuttleGeometryError(f's_ratio must be in [0, 1]: {ratio}')
    network = _rail_network(normalized_side)
    segment_name = _canonical_segment(normalized_side, public_segment)
    segment = network.segments[segment_name]
    point, yaw = segment.sample(ratio * segment.length)
    calibration = (
        RIGHT_CALIBRATION_DEFAULTS
        if normalized_side == 'right'
        else LEFT_CALIBRATION_DEFAULTS
    )
    x, y, z, calibrated_yaw = apply_rail_pose_calibration(
        point.x,
        point.y,
        point.z,
        yaw,
        calibration,
    )
    return ShuttleWorldPose(x=x, y=y, z=z, yaw=calibrated_yaw)


def shuttle_collision_center(pose: ShuttleWorldPose) -> tuple[float, float]:
    return (
        pose.x + SHUTTLE_COLLISION_CENTER_X_M * math.cos(pose.yaw),
        pose.y + SHUTTLE_COLLISION_CENTER_X_M * math.sin(pose.yaw),
    )


def _axes(pose: ShuttleWorldPose) -> tuple[tuple[float, float], ...]:
    return (
        (math.cos(pose.yaw), math.sin(pose.yaw)),
        (-math.sin(pose.yaw), math.cos(pose.yaw)),
    )


def _projection_radius(
    axis: tuple[float, float],
    pose: ShuttleWorldPose,
    half_length_m: float,
    half_width_m: float,
) -> float:
    longitudinal, lateral = _axes(pose)
    return (
        half_length_m
        * abs(axis[0] * longitudinal[0] + axis[1] * longitudinal[1])
        + half_width_m
        * abs(axis[0] * lateral[0] + axis[1] * lateral[1])
    )


def shuttle_footprints_conflict(
    first: ShuttleWorldPose,
    second: ShuttleWorldPose,
    *,
    clearance_m: float = MIN_SHUTTLE_FOOTPRINT_CLEARANCE_M,
) -> bool:
    """Return whether two oriented collision boxes overlap within the clearance."""
    if clearance_m < 0.0:
        raise ShuttleGeometryError('clearance_m cannot be negative')
    half_length_m = (SHUTTLE_COLLISION_SIZE_M[0] + clearance_m) / 2.0
    half_width_m = (SHUTTLE_COLLISION_SIZE_M[1] + clearance_m) / 2.0
    first_center = shuttle_collision_center(first)
    second_center = shuttle_collision_center(second)
    delta = (
        second_center[0] - first_center[0],
        second_center[1] - first_center[1],
    )
    for axis in _axes(first) + _axes(second):
        center_projection = abs(delta[0] * axis[0] + delta[1] * axis[1])
        combined_radius = _projection_radius(
            axis,
            first,
            half_length_m,
            half_width_m,
        ) + _projection_radius(
            axis,
            second,
            half_length_m,
            half_width_m,
        )
        if center_projection >= combined_radius - 1e-9:
            return False
    return True


def shuttle_position_conflicts(
    positions: Iterable[ShuttlePosition],
    *,
    clearance_m: float = MIN_SHUTTLE_FOOTPRINT_CLEARANCE_M,
) -> list[dict[str, Any]]:
    """Return all physically conflicting shuttle pairs in world coordinates."""
    resolved = [
        (
            position,
            shuttle_world_pose(
                position.side,
                position.segment,
                position.s_ratio,
            ),
        )
        for position in positions
    ]
    conflicts = []
    for (first, first_pose), (second, second_pose) in itertools.combinations(resolved, 2):
        if not shuttle_footprints_conflict(
            first_pose,
            second_pose,
            clearance_m=clearance_m,
        ):
            continue
        first_center = shuttle_collision_center(first_pose)
        second_center = shuttle_collision_center(second_pose)
        conflicts.append({
            'first_id': first.shuttle_id,
            'second_id': second.shuttle_id,
            'first_side': first.side,
            'second_side': second.side,
            'first_segment': first.segment,
            'second_segment': second.segment,
            'first_s_ratio': round(float(first.s_ratio), 6),
            'second_s_ratio': round(float(second.s_ratio), 6),
            'center_distance_m': round(
                math.hypot(
                    first_center[0] - second_center[0],
                    first_center[1] - second_center[1],
                ),
                6,
            ),
            'required_footprint_clearance_m': round(float(clearance_m), 6),
        })
    return conflicts
