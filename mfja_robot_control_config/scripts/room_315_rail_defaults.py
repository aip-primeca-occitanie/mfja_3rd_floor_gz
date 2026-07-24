#!/usr/bin/env python3
"""Static Room 315 rail defaults shared by the kinematic runtime."""

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class AllowedStartPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


ALLOWED_START_POSES = {
    '1': AllowedStartPose(-15.43, -3.86, 0.84, 0.0, 0.0, 3.14),
    '2': AllowedStartPose(-14.95, -3.86, 0.84, 0.0, 0.0, 3.14),
    '3': AllowedStartPose(-14.77, -5.54, 0.84, 0.0, 0.0, 0.0),
    '4': AllowedStartPose(-15.24, -5.54, 0.84, 0.0, 0.0, 0.0),
}

RIGHT_TOPIC_DEFAULTS = {
    'pose_topic': '/room_315/rails/right/shuttles/pose_cmd',
    'pose_topic_prefix': '/room_315/rails/right/shuttles',
    'shuttle_state_topic': '/room_315/rails/right/shuttles/state',
    'add_shuttle_service': '/room_315/rails/right/shuttles/add',
    'shuttle_control_command_topic': '/room_315/rails/right/shuttles/command',
    'switch_command_topic': '/room_315/rails/right/switches/command',
    'switch_state_topic': '/room_315/rails/right/switches/state',
    'stopper_command_topic': '/room_315/rails/right/stoppers/command',
    'stopper_state_topic': '/room_315/rails/right/stoppers/state',
    'sensor_feedback_topic': '/room_315/rails/right/sensors/feedback',
    'pose_offset_command_topic': '/room_315/rails/right/shuttles/pose_offset_command',
    'payload_state_topic': '/room_315/rails/right/shuttles/payload_state',
    'payload_command_topic': '/room_315/rails/right/shuttles/payload_command',
}

LEFT_TOPIC_DEFAULTS = {
    'pose_topic': '/room_315/rails/left/shuttles/pose_cmd',
    'pose_topic_prefix': '/room_315/rails/left/shuttles',
    'shuttle_state_topic': '/room_315/rails/left/shuttles/state',
    'add_shuttle_service': '/room_315/rails/left/shuttles/add',
    'shuttle_control_command_topic': '/room_315/rails/left/shuttles/command',
    'switch_command_topic': '/room_315/rails/left/switches/command',
    'switch_state_topic': '/room_315/rails/left/switches/state',
    'stopper_command_topic': '/room_315/rails/left/stoppers/command',
    'stopper_state_topic': '/room_315/rails/left/stoppers/state',
    'sensor_feedback_topic': '/room_315/rails/left/sensors/feedback',
    'pose_offset_command_topic': '/room_315/rails/left/shuttles/pose_offset_command',
    'payload_state_topic': '/room_315/rails/left/shuttles/payload_state',
    'payload_command_topic': '/room_315/rails/left/shuttles/payload_command',
}

RIGHT_ENTITY_DEFAULTS = {
    'preloaded_shuttle_count': 4,
    'gazebo_entity_name': 'room315_right_shuttle_1',
    'entity_name_prefix': 'room315_right_shuttle_',
}

LEFT_ENTITY_DEFAULTS = {
    'preloaded_shuttle_count': 4,
    'gazebo_entity_name': 'room315_left_shuttle_1',
    'entity_name_prefix': 'room315_left_shuttle_',
}

RIGHT_CALIBRATION_DEFAULTS = {
    'pose_transform_a': -0.893249246800,
    'pose_transform_b': 0.005839516878,
    'pose_transform_tx': -26.921427375871,
    'pose_transform_c': 0.001889497475,
    'pose_transform_d': 1.308619216904,
    'pose_transform_ty': 0.666926143808,
    'pose_transform_z_offset': 0.0,
    'pose_transform_yaw_offset': 0.0,
    'pose_scale_x': 1.0,
    'pose_scale_y': 1.0,
    'pose_scale_origin_x': -15.855195431322,
    'pose_scale_origin_y': -4.525523413467,
    'pose_rotation_deg': 0.0,
    'pose_rotation_origin_x': -15.855195431322,
    'pose_rotation_origin_y': -4.525523413467,
    'pose_offset_x': 0.0,
    'pose_offset_y': 0.0,
    'pose_offset_z': 0.0,
}

LEFT_CALIBRATION_DEFAULTS = {
    'pose_transform_a': -0.8938584503560025,
    'pose_transform_b': 0.005001975618640809,
    'pose_transform_tx': -22.47198317328330,
    'pose_transform_c': 0.001348127530438647,
    'pose_transform_d': 1.255463611604302,
    'pose_transform_ty': 0.4431777232193935,
    'pose_transform_z_offset': 0.0,
    'pose_transform_yaw_offset': 0.0,
    'pose_scale_x': 0.98,
    'pose_scale_y': 1.041,
    'pose_scale_origin_x': -10.6365565,
    'pose_scale_origin_y': -4.6995835,
    'pose_rotation_deg': 180.0,
    'pose_rotation_origin_x': -10.6365565,
    'pose_rotation_origin_y': -4.6995835,
    'pose_offset_x': 0.14,
    'pose_offset_y': 0.0,
    'pose_offset_z': 0.0,
}


def _calibration_value(calibration: Any, name: str) -> float:
    raw = calibration[name] if isinstance(calibration, Mapping) else getattr(calibration, name)
    return float(raw)


def apply_rail_point_calibration(
    x: float,
    y: float,
    z: float,
    calibration: Any,
) -> tuple[float, float, float]:
    """Transform one rail-coordinate point into the calibrated Gazebo frame."""
    base_x = (
        _calibration_value(calibration, 'pose_transform_a') * x
        + _calibration_value(calibration, 'pose_transform_b') * y
        + _calibration_value(calibration, 'pose_transform_tx')
    )
    base_y = (
        _calibration_value(calibration, 'pose_transform_c') * x
        + _calibration_value(calibration, 'pose_transform_d') * y
        + _calibration_value(calibration, 'pose_transform_ty')
    )
    scale_origin_x = _calibration_value(calibration, 'pose_scale_origin_x')
    scale_origin_y = _calibration_value(calibration, 'pose_scale_origin_y')
    scaled_x = scale_origin_x + (
        base_x - scale_origin_x
    ) * _calibration_value(calibration, 'pose_scale_x')
    scaled_y = scale_origin_y + (
        base_y - scale_origin_y
    ) * _calibration_value(calibration, 'pose_scale_y')

    rotation = math.radians(_calibration_value(calibration, 'pose_rotation_deg'))
    rotation_origin_x = _calibration_value(calibration, 'pose_rotation_origin_x')
    rotation_origin_y = _calibration_value(calibration, 'pose_rotation_origin_y')
    dx = scaled_x - rotation_origin_x
    dy = scaled_y - rotation_origin_y
    cos_rotation = math.cos(rotation)
    sin_rotation = math.sin(rotation)
    return (
        rotation_origin_x
        + cos_rotation * dx
        - sin_rotation * dy
        + _calibration_value(calibration, 'pose_offset_x'),
        rotation_origin_y
        + sin_rotation * dx
        + cos_rotation * dy
        + _calibration_value(calibration, 'pose_offset_y'),
        z
        + _calibration_value(calibration, 'pose_transform_z_offset')
        + _calibration_value(calibration, 'pose_offset_z'),
    )


def apply_rail_pose_calibration(
    x: float,
    y: float,
    z: float,
    yaw: float,
    calibration: Any,
) -> tuple[float, float, float, float]:
    """Transform a rail-coordinate pose into the calibrated Gazebo frame."""
    gazebo_x, gazebo_y, gazebo_z = apply_rail_point_calibration(
        x,
        y,
        z,
        calibration,
    )
    direction_x = math.cos(yaw)
    direction_y = math.sin(yaw)
    transformed_x = (
        _calibration_value(calibration, 'pose_transform_a') * direction_x
        + _calibration_value(calibration, 'pose_transform_b') * direction_y
    ) * _calibration_value(calibration, 'pose_scale_x')
    transformed_y = (
        _calibration_value(calibration, 'pose_transform_c') * direction_x
        + _calibration_value(calibration, 'pose_transform_d') * direction_y
    ) * _calibration_value(calibration, 'pose_scale_y')
    gazebo_yaw = (
        math.atan2(transformed_y, transformed_x)
        + math.radians(_calibration_value(calibration, 'pose_rotation_deg'))
        + _calibration_value(calibration, 'pose_transform_yaw_offset')
    )
    return gazebo_x, gazebo_y, gazebo_z, gazebo_yaw


RAIL_SENSOR_TYPE = 'sensor'
MARKER_VISUAL_DEFAULT = 'default'
MARKER_VISUAL_INACTIVE = 'inactive'
MARKER_VISUAL_ACTIVE = 'active'
SHUTTLE_VISUAL_NORMAL = 'normal'
SHUTTLE_VISUAL_FALLING = 'falling'
SHUTTLE_VISUAL_REFRESH_RETRY_INTERVAL_S = 0.5

VISUAL_SWITCH_SELECTOR_MAP = {
    'A1R': ('A1', 'right'),
    'A2R': ('A2', 'right'),
    'A3R': ('A3', 'right'),
    'A4R': ('A4', 'right'),
    'A1L': ('A1', 'left'),
    'A2L': ('A2', 'left'),
    'A3L': ('A3', 'left'),
    'A4L': ('A4', 'left'),
    'A1_DROIT_SWITCH': ('A1', 'right'),
    'A2_DROIT_SWITCH': ('A2', 'right'),
    'A3_DROIT_SWITCH': ('A3', 'right'),
    'A4_DROIT_SWITCH': ('A4', 'right'),
    'A1_GAUCHE_SWITCH': ('A1', 'left'),
    'A2_GAUCHE_SWITCH': ('A2', 'left'),
    'A3_GAUCHE_SWITCH': ('A3', 'left'),
    'A4_GAUCHE_SWITCH': ('A4', 'left'),
}
RIGHT_VISUAL_SWITCH_SELECTOR_MAP = {
    selector_name: station
    for selector_name, (station, side) in VISUAL_SWITCH_SELECTOR_MAP.items()
    if side == 'right'
}
LEFT_VISUAL_SWITCH_SELECTOR_MAP = {
    selector_name: station
    for selector_name, (station, side) in VISUAL_SWITCH_SELECTOR_MAP.items()
    if side == 'left'
}
VISUAL_SWITCH_SELECTOR_MAP_BY_SIDE = {
    'right': RIGHT_VISUAL_SWITCH_SELECTOR_MAP,
    'left': LEFT_VISUAL_SWITCH_SELECTOR_MAP,
}
VISUAL_GROUP_SELECTOR_BY_SIDE = {
    'right': 'RIGHT',
    'left': 'LEFT',
}
VISUAL_SELECTOR_SUFFIX_BY_SIDE = {
    'right': 'R',
    'left': 'L',
}

PUBLIC_SWITCH_ORDER = ('A1', 'A2', 'A3', 'A4')
STOPPER_PASS_STATE = '0'
STOPPER_STOP_STATE = '1'
SWITCH_INTERIOR_STATE = 'I'
SWITCH_EXTERIOR_STATE = 'E'

LEFT_PUBLIC_SEGMENT_NAME_MAP = {
    'A1E': 'A3E',
    'A1I': 'A3I',
    'A2E': 'A4E',
    'A2I': 'A4I',
    'A3E': 'A1E',
    'A3I': 'A1I',
    'A4E': 'A2E',
    'A4I': 'A2I',
    'A12E': 'A34E',
    'A12I': 'A34I',
    'A14': 'A23',
    'A23': 'A14',
    'A34E': 'A12E',
    'A34I': 'A12I',
}

DEVICE_MARKER_STYLES = {
    'position_sensor': {
        'shape': 'sphere',
        'radius': 0.04,
        'active_radius_scale': 1.35,
        'length': 0.0,
        'z_offset_m': 0.10,
        'rgba': (0.05, 0.45, 1.0, 0.85),
        'rgba_by_state': {
            MARKER_VISUAL_INACTIVE: (0.05, 0.45, 1.0, 0.85),
            MARKER_VISUAL_ACTIVE: (0.0, 0.85, 0.18, 0.95),
        },
    },
    'stopper': {
        'shape': 'cylinder',
        'radius': 0.045,
        'active_radius_scale': 1.25,
        'length': 0.09,
        'z_offset_m': 0.0,
        'rgba': (1.0, 0.72, 0.08, 0.9),
        'rgba_by_state': {
            MARKER_VISUAL_INACTIVE: (1.0, 0.72, 0.08, 0.9),
            MARKER_VISUAL_ACTIVE: (1.0, 0.02, 0.02, 0.95),
        },
    },
}
