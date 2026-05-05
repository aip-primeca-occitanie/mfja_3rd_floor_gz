#!/usr/bin/env python3

import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.msg import EntityFactory
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import String

from room_315_kinematic_shuttle import (
    CUBIC_HERMITE_PATH_BACKEND,
    FALLING,
    KinematicShuttleCore,
    MOVING,
    RailNetwork,
    ShuttlePose,
    ShuttleState,
    WAITING,
)


def _default_network_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_kinematics'
            / 'rail_network_right.yaml'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[2]
            / 'mfja_robot_control_config'
            / 'config'
            / 'room_315_kinematics'
            / 'rail_network_right.yaml'
        )


def _default_left_network_path() -> Path:
    return _default_network_path().with_name('rail_network_left.yaml')


def _default_shuttle_model_sdf_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_3rd_floor_description'))
            / 'models'
            / 'room315_shuttle'
            / 'model.sdf'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[2]
            / 'mfja_3rd_floor_description'
            / 'models'
            / 'room315_shuttle'
            / 'model.sdf'
        )


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


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
    'pose_topic': '/room_315_right/shuttle/pose_cmd',
    'pose_topic_prefix': '/room_315_right/shuttles',
    'state_topic': '/room_315_right/shuttle/state',
    'add_shuttle_command_topic': '/room_315_right/shuttle/add_cmd',
    'shuttle_control_command_topic': '/room_315_right/shuttle/control_cmd',
    'switch_command_topic': '/room_315_right/switch_states',
    'stopper_command_topic': '/room_315_right/stopper_states',
    'sensor_state_topic': '/room_315_right/sensors/switch_approach',
    'position_sensor_state_topic': '/room_315_right/sensors/position',
    'pose_offset_command_topic': '/room_315_right/shuttle/pose_offset_cmd',
}

LEFT_TOPIC_DEFAULTS = {
    'pose_topic': '/room_315_left/shuttle/pose_cmd',
    'pose_topic_prefix': '/room_315_left/shuttles',
    'state_topic': '/room_315_left/shuttle/state',
    'add_shuttle_command_topic': '/room_315_left/shuttle/add_cmd',
    'shuttle_control_command_topic': '/room_315_left/shuttle/control_cmd',
    'switch_command_topic': '/room_315_left/switch_states',
    'stopper_command_topic': '/room_315_left/stopper_states',
    'sensor_state_topic': '/room_315_left/sensors/switch_approach',
    'position_sensor_state_topic': '/room_315_left/sensors/position',
    'pose_offset_command_topic': '/room_315_left/shuttle/pose_offset_cmd',
}

RIGHT_ENTITY_DEFAULTS = {
    'preloaded_shuttle_count': 4,
    'gazebo_entity_name': 'room315_right_shuttle_1',
    'entity_name_prefix': 'room315_right_shuttle_',
}

LEFT_ENTITY_DEFAULTS = {
    'preloaded_shuttle_count': 1,
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

LEFT_PUBLIC_SEGMENT_NAME_MAP = {
    'A1G': 'A3G',
    'A1S': 'A3S',
    'A2G': 'A4G',
    'A2S': 'A4S',
    'A3G': 'A1G',
    'A3S': 'A1S',
    'A4G': 'A2G',
    'A4S': 'A2S',
    'A12E': 'A34E',
    'A12I': 'A34I',
    'A14': 'A23',
    'A23': 'A14',
    'A34E': 'A12E',
    'A34I': 'A12I',
}


def _canonical_switch_name(name: str) -> str:
    return str(name).strip().upper()


def _canonical_segment_name(name: str) -> str:
    return str(name).strip().upper()


def _canonical_sensor_name(name: str) -> str:
    return str(name).strip().upper()


def _normalize_rail_side(raw_value: str) -> str:
    side = str(raw_value).strip().lower()
    if side in {'right', 'r', 'droit'}:
        return 'right'
    if side in {'left', 'l', 'gauche'}:
        return 'left'
    raise ValueError(
        f'Unsupported rail_side={raw_value!r}; use right or left.'
    )


def _dedupe_aliases(values: tuple[str, ...]) -> tuple[str, ...]:
    ordered_values: list[str] = []
    seen_values: set[str] = set()
    for raw_value in values:
        value = str(raw_value).strip().upper()
        if not value or value in seen_values:
            continue
        seen_values.add(value)
        ordered_values.append(value)
    return tuple(ordered_values)


def _ordered_switch_states(switch_states: Dict[str, str]) -> Dict[str, str]:
    ordered: Dict[str, str] = {}
    for switch_name in PUBLIC_SWITCH_ORDER:
        if switch_name in switch_states:
            ordered[switch_name] = switch_states[switch_name]

    for switch_name, state in switch_states.items():
        canonical_name = _canonical_switch_name(switch_name)
        if canonical_name not in ordered:
            ordered[canonical_name] = state
    return ordered


@dataclass(frozen=True)
class StopPoint:
    segment: str
    stop_s: float
    sensor_distance_m: float


@dataclass(frozen=True)
class StopperConfig:
    name: str
    before_switch: str
    default_state: str
    stop_points: tuple[StopPoint, ...]


@dataclass(frozen=True)
class PositionSensorPoint:
    segment: str
    sensor_s: float
    sensor_distance_m: float


@dataclass(frozen=True)
class PositionSensorConfig:
    name: str
    sensor_kind: str
    points: tuple[PositionSensorPoint, ...]
    switch_name: str | None = None
    branch_state: str | None = None
    loop_side: str | None = None
    index_zone: str | None = None
    start_slot: str | None = None
    aliases: tuple[str, ...] = ()


@dataclass
class ManagedShuttle:
    entity_name: str
    start_slot: str
    start_pose: AllowedStartPose
    start_snap_distance_m: float
    core: KinematicShuttleCore
    pose_publisher: object
    pending_set_pose: object | None = None
    pending_spawn: object | None = None
    last_gazebo_set_pose_time: object | None = None
    gazebo_spawned: bool = False
    deployed: bool = True
    blocked_by: str | None = None
    collision_distance_m: float | None = None
    enabled: bool = True
    stopped_by: str | None = None
    stopper_distance_m: float | None = None
    set_pose_warning_logged: bool = False
    spawn_failure_logged: bool = False


class Room315KinematicShuttleNode(Node):
    def __init__(self) -> None:
        super().__init__('room_315_kinematic_shuttle')

        self.declare_parameter('network_yaml', str(_default_network_path()))
        self.declare_parameter('rail_side', 'right')
        self.declare_parameter('path_backend', CUBIC_HERMITE_PATH_BACKEND)
        self.declare_parameter('arc_length_samples_per_edge', 16)
        self.declare_parameter('shuttle_count', 1)
        self.declare_parameter('start_enabled', True)
        self.declare_parameter('start_slot', 2)
        self.declare_parameter('start_slots', '')
        self.declare_parameter('start_snap_tolerance_m', 0.25)
        self.declare_parameter('initial_segment', 'A23')
        self.declare_parameter('initial_s', 0.0)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('enable_collision_avoidance', True)
        self.declare_parameter('shuttle_collision_distance_m', 0.33)
        self.declare_parameter('collision_search_iterations', 12)
        self.declare_parameter('pose_topic', '/room_315_right/shuttle/pose_cmd')
        self.declare_parameter('pose_topic_prefix', '/room_315_right/shuttles')
        self.declare_parameter('state_topic', '/room_315_right/shuttle/state')
        self.declare_parameter('add_shuttle_command_topic', '/room_315_right/shuttle/add_cmd')
        self.declare_parameter('shuttle_control_command_topic', '/room_315_right/shuttle/control_cmd')
        self.declare_parameter('switch_command_topic', '/room_315_right/switch_states')
        self.declare_parameter('stopper_command_topic', '/room_315_right/stopper_states')
        self.declare_parameter('sensor_state_topic', '/room_315_right/sensors/switch_approach')
        self.declare_parameter('position_sensor_state_topic', '/room_315_right/sensors/position')
        self.declare_parameter('pose_offset_command_topic', '/room_315_right/shuttle/pose_offset_cmd')
        self.declare_parameter('visual_switch_command_topic', '/mfja/conveyor/switch_cmd')
        self.declare_parameter('visual_switch_state_topic', '/mfja/conveyor/switch_states')
        self.declare_parameter('sync_from_visual_switch_states', True)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('enable_gazebo_set_pose', False)
        self.declare_parameter('gazebo_world_name', 'room_315_only')
        self.declare_parameter('gazebo_set_pose_service', '')
        self.declare_parameter('enable_gazebo_spawn', True)
        self.declare_parameter('gazebo_spawn_service', '')
        self.declare_parameter('enable_gazebo_delete', True)
        self.declare_parameter('gazebo_delete_service', '')
        self.declare_parameter('shuttle_model_sdf', str(_default_shuttle_model_sdf_path()))
        self.declare_parameter('preloaded_shuttle_count', 4)
        self.declare_parameter('reject_occupied_start_slots', True)
        self.declare_parameter('start_slot_occupancy_radius_m', 0.33)
        self.declare_parameter('gazebo_entity_name', 'room315_right_shuttle_1')
        self.declare_parameter('gazebo_entity_names', '')
        self.declare_parameter('entity_name_prefix', 'room315_right_shuttle_')
        self.declare_parameter('gazebo_set_pose_rate_hz', 10.0)
        self.declare_parameter('publish_visual_switch_commands', True)
        self.declare_parameter('enable_gazebo_pose_transform', True)
        self.declare_parameter('pose_transform_a', -0.893249246800)
        self.declare_parameter('pose_transform_b', 0.005839516878)
        self.declare_parameter('pose_transform_tx', -26.921427375871)
        self.declare_parameter('pose_transform_c', 0.001889497475)
        self.declare_parameter('pose_transform_d', 1.308619216904)
        self.declare_parameter('pose_transform_ty', 0.666926143808)
        self.declare_parameter('pose_transform_z_offset', 0.0)
        self.declare_parameter('pose_transform_yaw_offset', 0.0)
        self.declare_parameter('pose_scale_x', 1.0)
        self.declare_parameter('pose_scale_y', 1.0)
        self.declare_parameter('pose_scale_origin_x', -15.855195431322)
        self.declare_parameter('pose_scale_origin_y', -4.525523413467)
        self.declare_parameter('pose_rotation_deg', 0.0)
        self.declare_parameter('pose_rotation_origin_x', -15.855195431322)
        self.declare_parameter('pose_rotation_origin_y', -4.525523413467)
        self.declare_parameter('pose_offset_x', 0.0)
        self.declare_parameter('pose_offset_y', 0.0)
        self.declare_parameter('pose_offset_z', 0.0)

        self.rail_side = _normalize_rail_side(
            str(self.get_parameter('rail_side').value)
        )
        self.active_visual_switch_selector_map = VISUAL_SWITCH_SELECTOR_MAP_BY_SIDE[
            self.rail_side
        ]
        self.active_visual_group_selector = VISUAL_GROUP_SELECTOR_BY_SIDE[self.rail_side]
        self.active_visual_selector_suffix = VISUAL_SELECTOR_SUFFIX_BY_SIDE[
            self.rail_side
        ]
        network_path = self._side_default_path(
            Path(str(self.get_parameter('network_yaml').value))
        )
        path_backend = str(self.get_parameter('path_backend').value)
        arc_length_samples_per_edge = int(
            self.get_parameter('arc_length_samples_per_edge').value
        )
        shuttle_count = int(self.get_parameter('shuttle_count').value)
        start_enabled = bool(self.get_parameter('start_enabled').value)
        start_slot = self.get_parameter('start_slot').value
        start_slots = str(self.get_parameter('start_slots').value)
        start_snap_tolerance_m = float(
            self.get_parameter('start_snap_tolerance_m').value
        )
        initial_segment = str(self.get_parameter('initial_segment').value)
        initial_s = float(self.get_parameter('initial_s').value)
        speed = float(self.get_parameter('speed').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.enable_collision_avoidance = bool(
            self.get_parameter('enable_collision_avoidance').value
        )
        self.shuttle_collision_distance_m = float(
            self.get_parameter('shuttle_collision_distance_m').value
        )
        self.collision_search_iterations = int(
            self.get_parameter('collision_search_iterations').value
        )
        pose_topic = self._side_default_string(
            str(self.get_parameter('pose_topic').value),
            'pose_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        self.pose_topic_prefix = self._side_default_string(
            str(self.get_parameter('pose_topic_prefix').value).rstrip('/'),
            'pose_topic_prefix',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        ).rstrip('/')
        state_topic = self._side_default_string(
            str(self.get_parameter('state_topic').value),
            'state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        add_shuttle_command_topic = self._side_default_string(
            str(self.get_parameter('add_shuttle_command_topic').value),
            'add_shuttle_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        shuttle_control_command_topic = self._side_default_string(
            str(self.get_parameter('shuttle_control_command_topic').value),
            'shuttle_control_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        switch_command_topic = self._side_default_string(
            str(self.get_parameter('switch_command_topic').value),
            'switch_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        stopper_command_topic = self._side_default_string(
            str(self.get_parameter('stopper_command_topic').value),
            'stopper_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        sensor_state_topic = self._side_default_string(
            str(self.get_parameter('sensor_state_topic').value),
            'sensor_state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        position_sensor_state_topic = self._side_default_string(
            str(self.get_parameter('position_sensor_state_topic').value),
            'position_sensor_state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        pose_offset_command_topic = self._side_default_string(
            str(self.get_parameter('pose_offset_command_topic').value),
            'pose_offset_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        visual_switch_command_topic = str(
            self.get_parameter('visual_switch_command_topic').value
        )
        visual_switch_state_topic = str(
            self.get_parameter('visual_switch_state_topic').value
        )
        self.sync_from_visual_switch_states = bool(
            self.get_parameter('sync_from_visual_switch_states').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.enable_gazebo_set_pose = bool(self.get_parameter('enable_gazebo_set_pose').value)
        self.gazebo_world_name = str(self.get_parameter('gazebo_world_name').value)
        gazebo_set_pose_service = self._resolve_world_service(
            raw_service=str(self.get_parameter('gazebo_set_pose_service').value),
            suffix='set_pose',
        )
        self.enable_gazebo_spawn = bool(self.get_parameter('enable_gazebo_spawn').value)
        gazebo_spawn_service = self._resolve_world_service(
            raw_service=str(self.get_parameter('gazebo_spawn_service').value),
            suffix='create',
        )
        self.enable_gazebo_delete = bool(
            self.get_parameter('enable_gazebo_delete').value
        )
        gazebo_delete_service = self._resolve_world_service(
            raw_service=str(self.get_parameter('gazebo_delete_service').value),
            suffix='remove',
        )
        self.shuttle_model_sdf = Path(str(self.get_parameter('shuttle_model_sdf').value))
        self.preloaded_shuttle_count = int(
            self._side_default_numeric(
                int(self.get_parameter('preloaded_shuttle_count').value),
                'preloaded_shuttle_count',
                right_defaults=RIGHT_ENTITY_DEFAULTS,
                left_defaults=LEFT_ENTITY_DEFAULTS,
            )
        )
        self.reject_occupied_start_slots = bool(
            self.get_parameter('reject_occupied_start_slots').value
        )
        self.start_slot_occupancy_radius_m = float(
            self.get_parameter('start_slot_occupancy_radius_m').value
        )
        self.gazebo_entity_name = self._side_default_string(
            str(self.get_parameter('gazebo_entity_name').value),
            'gazebo_entity_name',
            right_defaults=RIGHT_ENTITY_DEFAULTS,
            left_defaults=LEFT_ENTITY_DEFAULTS,
        )
        gazebo_entity_names = str(self.get_parameter('gazebo_entity_names').value)
        self.entity_name_prefix = self._normalize_entity_name_prefix(
            self._side_default_string(
                str(self.get_parameter('entity_name_prefix').value),
                'entity_name_prefix',
                right_defaults=RIGHT_ENTITY_DEFAULTS,
                left_defaults=LEFT_ENTITY_DEFAULTS,
            )
        )
        self.preloaded_entity_pattern = re.compile(
            rf'^{re.escape(self.entity_name_prefix)}(\d+)$'
        )
        gazebo_set_pose_rate_hz = float(self.get_parameter('gazebo_set_pose_rate_hz').value)
        self.gazebo_set_pose_period = 1.0 / max(gazebo_set_pose_rate_hz, 1.0)
        self.publish_visual_switch_commands = bool(
            self.get_parameter('publish_visual_switch_commands').value
        )
        self.enable_gazebo_pose_transform = bool(
            self.get_parameter('enable_gazebo_pose_transform').value
        )
        self.pose_transform_a = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_a').value),
                'pose_transform_a',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_b = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_b').value),
                'pose_transform_b',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_tx = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_tx').value),
                'pose_transform_tx',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_c = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_c').value),
                'pose_transform_c',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_d = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_d').value),
                'pose_transform_d',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_ty = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_ty').value),
                'pose_transform_ty',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_z_offset = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_z_offset').value),
                'pose_transform_z_offset',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_transform_yaw_offset = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_transform_yaw_offset').value),
                'pose_transform_yaw_offset',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_scale_x = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_scale_x').value),
                'pose_scale_x',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_scale_y = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_scale_y').value),
                'pose_scale_y',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_scale_origin_x = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_scale_origin_x').value),
                'pose_scale_origin_x',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_scale_origin_y = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_scale_origin_y').value),
                'pose_scale_origin_y',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_rotation_deg = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_rotation_deg').value),
                'pose_rotation_deg',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_rotation_origin_x = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_rotation_origin_x').value),
                'pose_rotation_origin_x',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_rotation_origin_y = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_rotation_origin_y').value),
                'pose_rotation_origin_y',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_offset_x = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_offset_x').value),
                'pose_offset_x',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_offset_y = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_offset_y').value),
                'pose_offset_y',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_offset_z = float(
            self._side_default_numeric(
                float(self.get_parameter('pose_offset_z').value),
                'pose_offset_z',
                right_defaults=RIGHT_CALIBRATION_DEFAULTS,
                left_defaults=LEFT_CALIBRATION_DEFAULTS,
            )
        )
        self.pose_calibration_defaults = {
            'pose_scale_x': self.pose_scale_x,
            'pose_scale_y': self.pose_scale_y,
            'pose_scale_origin_x': self.pose_scale_origin_x,
            'pose_scale_origin_y': self.pose_scale_origin_y,
            'pose_rotation_deg': self.pose_rotation_deg,
            'pose_rotation_origin_x': self.pose_rotation_origin_x,
            'pose_rotation_origin_y': self.pose_rotation_origin_y,
            'pose_offset_x': self.pose_offset_x,
            'pose_offset_y': self.pose_offset_y,
            'pose_offset_z': self.pose_offset_z,
        }
        self.start_snap_tolerance_m = start_snap_tolerance_m
        self.default_shuttle_speed = speed
        self.spawn_warning_logged = False
        self.deleted_preloaded_entity_names: set[str] = set()
        self.deleting_entity_names: set[str] = set()

        self.network = RailNetwork.from_yaml(
            network_path,
            path_backend=path_backend,
            arc_length_samples_per_edge=arc_length_samples_per_edge,
        )
        self.allowed_start_poses = self._load_allowed_start_poses()
        self.switch_states: Dict[str, str] = self.network.default_switch_states()
        self.stopper_configs = self._load_stopper_configs()
        self.position_sensor_configs = self._load_position_sensor_configs()
        self.stopper_states: Dict[str, str] = {
            name: config.default_state
            for name, config in self.stopper_configs.items()
        }
        shuttle_specs = self._resolve_shuttle_specs(
            shuttle_count=shuttle_count,
            raw_start_slot=start_slot,
            raw_start_slots=start_slots,
            default_entity_name=self.gazebo_entity_name,
            raw_entity_names=gazebo_entity_names,
        )
        self.shuttles: list[ManagedShuttle] = []
        for shuttle_index, (entity_name, slot) in enumerate(shuttle_specs):
            self.shuttles.append(
                self._create_managed_shuttle(
                    entity_name=entity_name,
                    slot=slot,
                    speed=speed,
                    enabled=start_enabled,
                    deployed=start_enabled,
                    pose_topic_override=pose_topic if shuttle_index == 0 else None,
                )
            )

        self.state_publisher = self.create_publisher(String, state_topic, 10)
        self.sensor_state_publisher = self.create_publisher(String, sensor_state_topic, 10)
        self.position_sensor_state_publisher = self.create_publisher(
            String,
            position_sensor_state_topic,
            10,
        )
        self.visual_switch_publisher = self.create_publisher(
            String,
            visual_switch_command_topic,
            10,
        )
        self.switch_subscription = self.create_subscription(
            String,
            switch_command_topic,
            self._on_switch_command,
            10,
        )
        self.stopper_subscription = self.create_subscription(
            String,
            stopper_command_topic,
            self._on_stopper_command,
            10,
        )
        self.add_shuttle_subscription = self.create_subscription(
            String,
            add_shuttle_command_topic,
            self._on_add_shuttle_command,
            10,
        )
        self.shuttle_control_subscription = self.create_subscription(
            String,
            shuttle_control_command_topic,
            self._on_shuttle_control_command,
            10,
        )
        self.visual_switch_state_subscription = None
        if self.sync_from_visual_switch_states:
            visual_state_qos = QoSProfile(depth=1)
            visual_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            visual_state_qos.reliability = ReliabilityPolicy.RELIABLE
            self.visual_switch_state_subscription = self.create_subscription(
                String,
                visual_switch_state_topic,
                self._on_visual_switch_state,
                visual_state_qos,
            )
        self.pose_offset_subscription = self.create_subscription(
            String,
            pose_offset_command_topic,
            self._on_pose_offset_command,
            10,
        )
        self.set_pose_client = None
        if self.enable_gazebo_set_pose:
            self.set_pose_client = self.create_client(SetEntityPose, gazebo_set_pose_service)
        self.spawn_client = None
        if self.enable_gazebo_spawn:
            self.spawn_client = self.create_client(SpawnEntity, gazebo_spawn_service)
        self.delete_client = None
        if self.enable_gazebo_delete:
            self.delete_client = self.create_client(DeleteEntity, gazebo_delete_service)

        for shuttle in self.shuttles:
            self._request_spawn_if_needed(shuttle)

        self.last_tick = self.get_clock().now()
        timer_period = 1.0 / max(update_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            'Room 315 kinematic shuttle started with '
            f'rail_side={self.rail_side}, network={network_path}, path_backend={path_backend}, '
            f'pose_topic={pose_topic}, '
            f'gazebo_world={self.gazebo_world_name}, '
            f'add_shuttle_topic={add_shuttle_command_topic}, '
            f'shuttle_control_topic={shuttle_control_command_topic}, '
            f'switch_topic={switch_command_topic}, '
            f'stopper_topic={stopper_command_topic}, '
            f'sensor_topic={sensor_state_topic}, '
            f'position_sensor_topic={position_sensor_state_topic}, '
            f'offset_topic={pose_offset_command_topic}, '
            f'visual_switch_topic={visual_switch_command_topic}, '
            f'visual_switch_state_topic={visual_switch_state_topic}, '
            f'entity_prefix={self.entity_name_prefix}, '
            f'spawn_service={gazebo_spawn_service}, '
            f'delete_service={gazebo_delete_service}, '
            f'shuttles={self._shuttle_summary()}'
        )

    @staticmethod
    def _split_list_parameter(raw_value: str) -> list[str]:
        return [
            token.strip()
            for token in re.split(r'[\s,;]+', raw_value)
            if token.strip()
        ]

    def _resolve_world_service(self, raw_service: str, suffix: str) -> str:
        service = raw_service.strip()
        if service:
            return service

        world_name = self.gazebo_world_name.strip().strip('/')
        if not world_name:
            raise ValueError(
                'gazebo_world_name cannot be empty when Gazebo service names are auto-derived.'
            )
        return f'/world/{world_name}/{suffix}'

    @staticmethod
    def _normalize_entity_name_prefix(raw_value: str) -> str:
        prefix = str(raw_value).strip()
        if prefix:
            return prefix
        raise ValueError('entity_name_prefix must not be empty.')

    def _side_default_path(self, configured_path: Path) -> Path:
        if self.rail_side != 'left':
            return configured_path

        right_default = _default_network_path().resolve()
        if configured_path.resolve() == right_default:
            return _default_left_network_path()
        return configured_path

    def _side_default_string(
        self,
        configured_value: str,
        key: str,
        *,
        right_defaults: Dict[str, str],
        left_defaults: Dict[str, str],
    ) -> str:
        if self.rail_side == 'left' and configured_value == right_defaults[key]:
            return left_defaults[key]
        return configured_value

    def _side_default_numeric(
        self,
        configured_value,
        key: str,
        *,
        right_defaults: Dict[str, float] | Dict[str, int],
        left_defaults: Dict[str, float] | Dict[str, int],
    ):
        if self.rail_side == 'left' and configured_value == right_defaults[key]:
            return left_defaults[key]
        return configured_value

    def _load_allowed_start_poses(self) -> Dict[str, AllowedStartPose]:
        raw_slots = self.network.config.get('start_slots') or {}
        if not raw_slots:
            return dict(ALLOWED_START_POSES)

        allowed: Dict[str, AllowedStartPose] = {}
        for raw_slot, raw_config in raw_slots.items():
            slot = str(raw_slot).strip()
            pose_values = raw_config.get('pose') if isinstance(raw_config, dict) else raw_config
            if pose_values is None or len(pose_values) != 6:
                raise ValueError(
                    f'start_slots.{slot} must define pose: [x, y, z, roll, pitch, yaw].'
                )
            allowed[slot] = AllowedStartPose(*[float(value) for value in pose_values])

        if not allowed:
            raise ValueError('rail_network_right.yaml start_slots must not be empty.')
        return allowed

    def _load_stopper_configs(self) -> Dict[str, StopperConfig]:
        configs: Dict[str, StopperConfig] = {}
        raw_configs = self.network.config.get('stoppers', {}) or {}
        for raw_name, raw_config in raw_configs.items():
            internal_name = str(raw_name).strip().upper()
            name = _canonical_switch_name(internal_name)
            before_switch = _canonical_switch_name(
                str(raw_config.get('before_switch', internal_name)).strip().upper()
            )
            default_state = self._normalize_stopper_state(
                str(raw_config.get('default_state', '0'))
            )
            default_sensor_distance_m = float(raw_config.get('sensor_distance_m', 0.25))
            default_stop_offset_m = float(raw_config.get('stop_offset_m', 0.08))
            stop_points: list[StopPoint] = []

            raw_stop_points = raw_config.get('stop_points')
            if raw_stop_points is None:
                raw_stop_points = [
                    {'segment': segment_name}
                    for segment_name in raw_config.get('segments', [])
                ]

            for raw_stop_point in raw_stop_points:
                segment_name = str(raw_stop_point['segment']).strip()
                if segment_name not in self.network.segments:
                    raise ValueError(
                        f'Stopper {name} references unknown segment {segment_name!r}.'
                    )

                segment = self.network.segments[segment_name]
                if 's' in raw_stop_point:
                    stop_s = float(raw_stop_point['s'])
                else:
                    stop_offset_m = float(
                        raw_stop_point.get('stop_offset_m', default_stop_offset_m)
                    )
                    stop_s = segment.length - stop_offset_m
                stop_s = max(0.0, min(stop_s, segment.length))
                sensor_distance_m = float(
                    raw_stop_point.get('sensor_distance_m', default_sensor_distance_m)
                )
                stop_points.append(
                    StopPoint(
                        segment=segment_name,
                        stop_s=stop_s,
                        sensor_distance_m=max(0.0, sensor_distance_m),
                    )
                )

            if not stop_points:
                raise ValueError(f'Stopper {name} must define at least one stop point.')
            configs[name] = StopperConfig(
                name=name,
                before_switch=before_switch,
                default_state=default_state,
                stop_points=tuple(stop_points),
            )
        return configs

    @staticmethod
    def _normalize_position_sensor_branch(
        raw_branch: str | None,
    ) -> tuple[str | None, str | None]:
        if raw_branch is None:
            return None, None

        branch = str(raw_branch).strip().upper()
        if branch in {'G', 'E', 'EXTERIOR', 'GRAND', 'GRANDE', 'BIG', 'LARGE'}:
            return 'G', 'EXTERIOR'
        if branch in {'S', 'I', 'INTERIOR', 'P', 'PETIT', 'PETITE', 'SMALL'}:
            return 'S', 'INTERIOR'
        raise ValueError(
            f'Unknown position sensor branch {raw_branch!r}; use G/S or EXTERIOR/INTERIOR.'
        )

    def _position_sensor_points_from_config(
        self,
        sensor_name: str,
        raw_config: dict,
    ) -> tuple[PositionSensorPoint, ...]:
        raw_points = raw_config.get('points')
        if raw_points is None:
            raw_points = [raw_config]

        default_sensor_distance_m = float(raw_config.get('sensor_distance_m', 0.08))
        points: list[PositionSensorPoint] = []
        for raw_point in raw_points:
            sensor_distance_m = float(
                raw_point.get('sensor_distance_m', default_sensor_distance_m)
            )
            if 'slot' in raw_point:
                slot = self._normalize_start_slot(raw_point['slot'])
                start_pose = self.allowed_start_poses[slot]
                segment_name, sensor_s, _distance_m = self._closest_network_position(start_pose)
            else:
                segment_name = str(raw_point['segment']).strip()
                if segment_name not in self.network.segments:
                    raise ValueError(
                        f'Position sensor {sensor_name} references unknown segment '
                        f'{segment_name!r}.'
                    )

                segment = self.network.segments[segment_name]
                if 's' in raw_point:
                    sensor_s = float(raw_point['s'])
                else:
                    offset_m = float(raw_point.get('offset_m', 0.0))
                    reference = str(raw_point.get('reference', 'start')).strip().lower()
                    if reference in {'start', 'begin', 'from_start'}:
                        sensor_s = offset_m
                    elif reference in {'end', 'finish', 'from_end', 'before_end'}:
                        sensor_s = segment.length - offset_m
                    else:
                        raise ValueError(
                            f'Position sensor {sensor_name} uses unsupported reference '
                            f'{reference!r}; use start or end.'
                        )
                sensor_s = max(0.0, min(sensor_s, segment.length))

            points.append(
                PositionSensorPoint(
                    segment=segment_name,
                    sensor_s=sensor_s,
                    sensor_distance_m=max(0.0, sensor_distance_m),
                )
            )

        if not points:
            raise ValueError(f'Position sensor {sensor_name} must define at least one point.')
        return tuple(points)

    def _load_position_sensor_configs(self) -> Dict[str, PositionSensorConfig]:
        configs: Dict[str, PositionSensorConfig] = {}
        raw_configs = self.network.config.get('position_sensors', {}) or {}
        for raw_name, raw_config in raw_configs.items():
            if not isinstance(raw_config, dict):
                raise ValueError(
                    f'position_sensors.{raw_name} must be a mapping, got {type(raw_config)!r}.'
                )

            internal_name = str(raw_name).strip().upper()
            name = _canonical_sensor_name(internal_name)
            branch_state, normalized_loop_side = self._normalize_position_sensor_branch(
                raw_config.get('branch')
            )
            explicit_loop_side = raw_config.get('loop_side')
            if explicit_loop_side is not None:
                normalized_loop_side = str(explicit_loop_side).strip().upper()
            switch_name = raw_config.get('switch')
            index_zone = raw_config.get('index_zone')
            start_slot = raw_config.get('slot')
            configured_aliases = tuple(
                str(alias).strip().upper()
                for alias in raw_config.get('aliases', [])
                if str(alias).strip()
            )
            public_aliases: list[str] = []
            for alias in configured_aliases:
                public_alias = _canonical_sensor_name(alias)
                if public_alias != name:
                    public_aliases.append(public_alias)
            aliases = _dedupe_aliases(tuple(public_aliases))
            configs[name] = PositionSensorConfig(
                name=name,
                sensor_kind=str(raw_config.get('kind', 'position')).strip().lower(),
                points=self._position_sensor_points_from_config(name, raw_config),
                switch_name=(
                    _canonical_switch_name(str(switch_name).strip().upper())
                    if switch_name is not None
                    else None
                ),
                branch_state=branch_state,
                loop_side=normalized_loop_side,
                index_zone=str(index_zone).strip() if index_zone is not None else None,
                start_slot=(
                    self._normalize_start_slot(start_slot)
                    if start_slot is not None
                    else None
                ),
                aliases=aliases,
            )
        return configs

    def _resolve_shuttle_specs(
        self,
        shuttle_count: int,
        raw_start_slot,
        raw_start_slots: str,
        default_entity_name: str,
        raw_entity_names: str,
    ) -> list[tuple[str, str]]:
        if shuttle_count < 1:
            raise ValueError('shuttle_count must be at least 1.')

        start_slots = self._split_list_parameter(raw_start_slots)
        if not start_slots:
            start_slots = (
                [str(raw_start_slot)]
                if shuttle_count == 1
                else sorted(self.allowed_start_poses)[:shuttle_count]
            )
        start_slots = [self._normalize_start_slot(slot) for slot in start_slots]
        if len(start_slots) < shuttle_count:
            if self.reject_occupied_start_slots:
                raise ValueError(
                    f'shuttle_count={shuttle_count} requires {shuttle_count} explicit '
                    f'unique start slots, but only {len(start_slots)} were provided.'
                )
            start_slots = [
                start_slots[index % len(start_slots)]
                for index in range(shuttle_count)
            ]
        elif len(start_slots) > shuttle_count:
            raise ValueError(
                f'shuttle_count={shuttle_count} but start_slots has '
                f'{len(start_slots)} value(s): {start_slots}.'
            )
        if self.reject_occupied_start_slots and len(set(start_slots)) != len(start_slots):
            raise ValueError(
                f'Duplicate start slots are not allowed at startup: {start_slots}.'
            )

        entity_names = self._split_list_parameter(raw_entity_names)
        if not entity_names:
            entity_names = (
                [default_entity_name]
                if shuttle_count == 1
                else [
                    self._auto_entity_name(index)
                    for index in range(1, shuttle_count + 1)
                ]
            )
        if len(entity_names) != shuttle_count:
            raise ValueError(
                f'shuttle_count={shuttle_count} but gazebo_entity_names has '
                f'{len(entity_names)} value(s): {entity_names}.'
            )
        if len(set(entity_names)) != len(entity_names):
            raise ValueError(
                f'Duplicate gazebo entity names are not allowed: {entity_names}.'
            )

        return list(zip(entity_names, start_slots))

    @staticmethod
    def _topic_safe_name(entity_name: str) -> str:
        return re.sub(r'[^A-Za-z0-9_]+', '_', entity_name).strip('_') or 'shuttle'

    def _shuttle_summary(self) -> str:
        if not self.shuttles:
            return 'none'
        return ', '.join(
            f'{shuttle.entity_name}:slot{shuttle.start_slot}:'
            f'{self._public_segment_name(shuttle.core.state.current_segment)}@{shuttle.core.state.s:.3f}:'
            f'snap={shuttle.start_snap_distance_m:.3f}m'
            for shuttle in self.shuttles
        )

    def _create_managed_shuttle(
        self,
        entity_name: str,
        slot,
        speed: float,
        enabled: bool = True,
        deployed: bool = True,
        pose_topic_override: str | None = None,
    ) -> ManagedShuttle:
        (
            resolved_slot,
            start_pose,
            start_snap_distance_m,
            initial_segment,
            initial_s,
        ) = self._resolve_allowed_start_slot(slot, self.start_snap_tolerance_m)
        pose_topic = (
            pose_topic_override
            if pose_topic_override is not None
            else f'{self.pose_topic_prefix}/{self._topic_safe_name(entity_name)}/pose_cmd'
        )
        return ManagedShuttle(
            entity_name=entity_name,
            start_slot=resolved_slot,
            start_pose=start_pose,
            start_snap_distance_m=start_snap_distance_m,
            core=KinematicShuttleCore(
                network=self.network,
                initial_state=ShuttleState(
                    current_segment=initial_segment,
                    s=initial_s,
                    speed=speed,
                    mode=MOVING if enabled and deployed and speed > 0.0 else WAITING,
                ),
            ),
            pose_publisher=self.create_publisher(PoseStamped, pose_topic, 10),
            last_gazebo_set_pose_time=self.get_clock().now(),
            gazebo_spawned=self._is_preloaded_shuttle_entity(entity_name),
            deployed=deployed,
            enabled=enabled,
            stopped_by=None if deployed else 'NOT_DEPLOYED',
            stopper_distance_m=None if deployed else 0.0,
        )

    def _on_add_shuttle_command(self, message: String) -> None:
        try:
            entity_name, slot, speed = self._parse_add_shuttle_command(message.data)
            shuttle = self._create_managed_shuttle(
                entity_name=entity_name,
                slot=slot,
                speed=speed,
            )
        except (RuntimeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f'Failed to add shuttle: {error}')
            return

        self.shuttles.append(shuttle)
        self._request_spawn_if_needed(shuttle)
        self.get_logger().info(
            f'Added shuttle {shuttle.entity_name} at slot {shuttle.start_slot}; '
            f'shuttles={self._shuttle_summary()}'
        )

    def _parse_add_shuttle_command(self, raw_command: str) -> tuple[str, str, float]:
        command = raw_command.strip()
        if not command:
            raise ValueError('Empty add shuttle command')

        if command.startswith('{'):
            payload = json.loads(command)
            assignments = [(str(key), str(value)) for key, value in payload.items()]
        elif '=' not in command and ':' not in command:
            assignments = [('slot', command)]
        else:
            assignments = []
            for token in re.split(r'[\s,;]+', command.replace(':', '=')):
                if not token:
                    continue
                if '=' not in token:
                    raise ValueError(
                        f'Add shuttle command must look like slot=3, got {token!r}'
                    )
                key, raw_value = token.split('=', 1)
                assignments.append((key, raw_value))

        slot = ''
        entity_name = ''
        speed = self.default_shuttle_speed
        for raw_key, raw_value in assignments:
            key = raw_key.strip().lower()
            value = raw_value.strip()
            if key in {'slot', 'start_slot', 'start'}:
                slot = self._normalize_start_slot(value)
            elif key in {'entity', 'entity_name', 'gazebo_entity_name', 'name'}:
                entity_name = value
            elif key == 'speed':
                speed = float(value)
            else:
                raise ValueError(
                    f'Unknown add shuttle key {raw_key!r}; use slot, entity, or speed.'
                )

        if not slot:
            slot = self._next_unused_start_slot()
        if self.reject_occupied_start_slots:
            occupied_by = self._start_slot_occupancy_blocker(slot)
            if occupied_by is not None:
                entity_name_at_slot, distance_m = occupied_by
                raise ValueError(
                    f'start slot {slot} is occupied by {entity_name_at_slot} '
                    f'at distance {distance_m:.3f} m; add command rejected.'
                )

        if not entity_name:
            entity_name = self._next_unused_entity_name()
        if (
            any(shuttle.entity_name == entity_name for shuttle in self.shuttles)
            or entity_name in self.deleting_entity_names
        ):
            raise ValueError(
                f'Gazebo entity {entity_name!r} is already controlled by this node.'
            )

        return entity_name, slot, speed

    def _next_unused_start_slot(self) -> str:
        slots = sorted(self.allowed_start_poses)
        if not self.reject_occupied_start_slots:
            return slots[len(self.shuttles) % len(slots)]

        for slot in slots:
            if self._start_slot_occupancy_blocker(slot) is None:
                return slot
        raise ValueError('All allowed start slots are currently occupied.')

    def _start_slot_occupancy_blocker(
        self,
        slot: str,
        ignore_entity_name: str | None = None,
    ) -> tuple[str, float] | None:
        start_pose = self.allowed_start_poses[slot]
        blockers = []
        for shuttle in self.shuttles:
            if ignore_entity_name is not None and shuttle.entity_name == ignore_entity_name:
                continue
            pose = self._to_gazebo_pose(shuttle.core.pose())
            distance_m = math.hypot(start_pose.x - pose.x, start_pose.y - pose.y)
            if distance_m < self.start_slot_occupancy_radius_m:
                blockers.append((shuttle.entity_name, distance_m))
        if not blockers:
            return None
        return sorted(blockers, key=lambda item: item[1])[0]

    def _next_unused_entity_name(self) -> str:
        used_entities = {shuttle.entity_name for shuttle in self.shuttles}
        used_entities.update(self.deleting_entity_names)
        index = 1
        while True:
            entity_name = self._auto_entity_name(index)
            if entity_name not in used_entities:
                return entity_name
            index += 1

    def _auto_entity_name(self, index: int) -> str:
        return f'{self.entity_name_prefix}{index}'

    def _request_spawn_if_needed(self, shuttle: ManagedShuttle) -> None:
        if not shuttle.deployed:
            return
        if not self.enable_gazebo_spawn:
            return
        if self.spawn_client is None:
            return
        if shuttle.gazebo_spawned:
            return
        if self._is_preloaded_shuttle_entity(shuttle.entity_name):
            shuttle.gazebo_spawned = True
            return
        if shuttle.pending_spawn is not None:
            return

        if not self.spawn_client.service_is_ready():
            if not self.spawn_warning_logged:
                self.get_logger().warn(
                    'Gazebo spawn service is not ready yet. New shuttles beyond '
                    f'preloaded_shuttle_count={self.preloaded_shuttle_count} will be '
                    'controlled only after the spawn service becomes available.'
                )
                self.spawn_warning_logged = True
            return

        request = SpawnEntity.Request()
        request.entity_factory = self._make_spawn_entity_factory(shuttle)
        shuttle.pending_spawn = self.spawn_client.call_async(request)
        self.get_logger().info(f'Requested Gazebo spawn for {shuttle.entity_name}')

    def _is_preloaded_shuttle_entity(self, entity_name: str) -> bool:
        match = self.preloaded_entity_pattern.match(entity_name)
        return bool(
            match
            and int(match.group(1)) <= self.preloaded_shuttle_count
            and entity_name not in self.deleted_preloaded_entity_names
            and entity_name not in self.deleting_entity_names
        )

    def _make_spawn_entity_factory(self, shuttle: ManagedShuttle) -> EntityFactory:
        pose = self._to_gazebo_pose(shuttle.core.pose())
        factory = EntityFactory()
        factory.name = shuttle.entity_name
        factory.allow_renaming = False
        factory.sdf_filename = str(self.shuttle_model_sdf)
        factory.relative_to = 'world'
        factory.pose.position.x = pose.x
        factory.pose.position.y = pose.y
        factory.pose.position.z = pose.z
        qx, qy, qz, qw = _yaw_to_quaternion(pose.yaw)
        factory.pose.orientation.x = qx
        factory.pose.orientation.y = qy
        factory.pose.orientation.z = qz
        factory.pose.orientation.w = qw
        return factory

    def _spawn_ready_for_motion(self, shuttle: ManagedShuttle) -> bool:
        if not shuttle.deployed:
            return False
        if shuttle.pending_spawn is None:
            needs_spawn = self.enable_gazebo_spawn and not shuttle.gazebo_spawned
            self._request_spawn_if_needed(shuttle)
            if needs_spawn and shuttle.pending_spawn is None:
                return False
            return not needs_spawn

        if not shuttle.pending_spawn.done():
            return False

        try:
            response = shuttle.pending_spawn.result()
        except Exception as error:
            if not shuttle.spawn_failure_logged:
                self.get_logger().error(
                    f'Gazebo spawn request for {shuttle.entity_name} failed: {error}'
                )
                shuttle.spawn_failure_logged = True
            return False

        if not response.success:
            if not shuttle.spawn_failure_logged:
                self.get_logger().error(
                    f'Gazebo spawn service rejected {shuttle.entity_name}.'
                )
                shuttle.spawn_failure_logged = True
            return False

        shuttle.pending_spawn = None
        shuttle.gazebo_spawned = True
        self.deleted_preloaded_entity_names.discard(shuttle.entity_name)
        self.get_logger().info(f'Gazebo spawned {shuttle.entity_name}')
        return True

    def _resolve_allowed_start_slot(
        self,
        raw_slot: str,
        tolerance_m: float,
    ) -> tuple[str, AllowedStartPose, float, str, float]:
        slot = self._normalize_start_slot(raw_slot)
        start_pose = self.allowed_start_poses[slot]
        segment_name, s, distance_m = self._closest_network_position(start_pose)
        if distance_m > tolerance_m:
            allowed = ', '.join(sorted(self.allowed_start_poses))
            raise RuntimeError(
                f'start_slot={slot} is {distance_m:.3f} m away from the current '
                f'rail network, which is above start_snap_tolerance_m={tolerance_m:.3f}. '
                f'Allowed start slots are: {allowed}. Correct the slot pose or add the '
                'missing rail segment; the shuttle will not silently auto-correct.'
            )
        return slot, start_pose, distance_m, segment_name, s

    def _normalize_start_slot(self, raw_slot) -> str:
        slot = str(raw_slot).strip().lower().replace('-', '_')
        slot = re.sub(r'^(slot|start|start_slot)_?', '', slot)
        if slot in self.allowed_start_poses:
            return slot
        allowed = ', '.join(sorted(self.allowed_start_poses))
        raise ValueError(
            f'Unsupported start_slot={raw_slot!r}. Use one of: {allowed}.'
        )

    def _closest_network_position(
        self,
        start_pose: AllowedStartPose,
    ) -> tuple[str, float, float]:
        best_segment = ''
        best_s = 0.0
        best_distance = math.inf

        for segment_name, segment in self.network.segments.items():
            for index, (previous, current) in enumerate(
                zip(segment.points, segment.points[1:])
            ):
                p0 = self._to_gazebo_point(previous.x, previous.y, previous.z)
                p1 = self._to_gazebo_point(current.x, current.y, current.z)
                vx = p1[0] - p0[0]
                vy = p1[1] - p0[1]
                vz = p1[2] - p0[2]
                edge_length_sq = vx * vx + vy * vy + vz * vz
                if edge_length_sq <= 1e-12:
                    continue

                wx = start_pose.x - p0[0]
                wy = start_pose.y - p0[1]
                wz = start_pose.z - p0[2]
                ratio = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / edge_length_sq))
                projected = (
                    p0[0] + ratio * vx,
                    p0[1] + ratio * vy,
                    p0[2] + ratio * vz,
                )
                distance = math.dist(
                    (start_pose.x, start_pose.y, start_pose.z),
                    projected,
                )
                if distance < best_distance:
                    best_distance = distance
                    best_segment = segment_name
                    previous_s = segment.arc_lengths[index]
                    current_s = segment.arc_lengths[index + 1]
                    best_s = previous_s + ratio * (current_s - previous_s)

        if not best_segment:
            raise RuntimeError('Could not snap allowed start pose to the rail network.')
        return best_segment, best_s, best_distance

    def _to_gazebo_point(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        if not self.enable_gazebo_pose_transform:
            return x, y, z

        base_x = self.pose_transform_a * x + self.pose_transform_b * y + self.pose_transform_tx
        base_y = self.pose_transform_c * x + self.pose_transform_d * y + self.pose_transform_ty
        scaled_x = (
            self.pose_scale_origin_x
            + (base_x - self.pose_scale_origin_x) * self.pose_scale_x
        )
        scaled_y = (
            self.pose_scale_origin_y
            + (base_y - self.pose_scale_origin_y) * self.pose_scale_y
        )
        rotated_x, rotated_y = self._apply_planar_rotation(scaled_x, scaled_y)
        return (
            rotated_x + self.pose_offset_x,
            rotated_y + self.pose_offset_y,
            z + self.pose_transform_z_offset + self.pose_offset_z,
        )

    def _pose_rotation_rad(self) -> float:
        return math.radians(self.pose_rotation_deg)

    def _apply_planar_rotation(self, x: float, y: float) -> tuple[float, float]:
        rotation_rad = self._pose_rotation_rad()
        if abs(rotation_rad) <= 1e-12:
            return x, y

        dx = x - self.pose_rotation_origin_x
        dy = y - self.pose_rotation_origin_y
        cos_theta = math.cos(rotation_rad)
        sin_theta = math.sin(rotation_rad)
        return (
            self.pose_rotation_origin_x + cos_theta * dx - sin_theta * dy,
            self.pose_rotation_origin_y + sin_theta * dx + cos_theta * dy,
        )

    def _on_pose_offset_command(self, message: String) -> None:
        try:
            target_offsets = self._parse_pose_offset_command(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        parameters = [
            Parameter(name, Parameter.Type.DOUBLE, value)
            for name, value in target_offsets.items()
        ]
        results = self.set_parameters(parameters)
        if not all(result.successful for result in results):
            reasons = ', '.join(result.reason for result in results if result.reason)
            self.get_logger().error(f'Failed to update pose offsets: {reasons}')
            return

        self.get_logger().info(
            'Updated pose calibration: '
            f'scale_x={self.pose_scale_x:.6f}, '
            f'scale_y={self.pose_scale_y:.6f}, '
            f'rotation_deg={self.pose_rotation_deg:.3f}, '
            f'offset_x={self.pose_offset_x:.4f}, '
            f'offset_y={self.pose_offset_y:.4f}, '
            f'offset_z={self.pose_offset_z:.4f}'
        )

    def _parse_pose_offset_command(self, raw_command: str) -> Dict[str, float]:
        command = raw_command.strip()
        if not command:
            raise ValueError('Empty pose offset command')

        if command.lower() in {'reset', 'zero', '0'}:
            return dict(self.pose_calibration_defaults)

        if command.startswith('{'):
            payload = json.loads(command)
            assignments = [(str(key), str(value)) for key, value in payload.items()]
            reset_requested = False
        else:
            assignments = []
            reset_requested = False
            for token in re.split(r'[\s,;]+', command.replace(':', '=')):
                if not token:
                    continue
                if token.strip().lower() in {'reset', 'zero', '0'}:
                    reset_requested = True
                    continue
                if '=' not in token:
                    raise ValueError(
                        f'Pose offset command must look like x=0.1 or dx=-0.01, got {token!r}'
                    )
                key, raw_value = token.split('=', 1)
                assignments.append((key, raw_value))

        default_calibration = self.pose_calibration_defaults
        next_x = (
            default_calibration['pose_offset_x']
            if reset_requested else self.pose_offset_x
        )
        next_y = (
            default_calibration['pose_offset_y']
            if reset_requested else self.pose_offset_y
        )
        next_z = (
            default_calibration['pose_offset_z']
            if reset_requested else self.pose_offset_z
        )
        next_scale_x = (
            default_calibration['pose_scale_x']
            if reset_requested else self.pose_scale_x
        )
        next_scale_y = (
            default_calibration['pose_scale_y']
            if reset_requested else self.pose_scale_y
        )
        next_origin_x = (
            default_calibration['pose_scale_origin_x']
            if reset_requested else self.pose_scale_origin_x
        )
        next_origin_y = (
            default_calibration['pose_scale_origin_y']
            if reset_requested else self.pose_scale_origin_y
        )
        next_rotation_deg = (
            default_calibration['pose_rotation_deg']
            if reset_requested else self.pose_rotation_deg
        )
        next_rotation_origin_x = (
            default_calibration['pose_rotation_origin_x']
            if reset_requested else self.pose_rotation_origin_x
        )
        next_rotation_origin_y = (
            default_calibration['pose_rotation_origin_y']
            if reset_requested else self.pose_rotation_origin_y
        )
        for raw_key, raw_value in assignments:
            key = raw_key.strip().lower()
            value = float(raw_value)
            if key in {'x', 'offset_x', 'pose_offset_x'}:
                next_x = value
            elif key in {'y', 'offset_y', 'pose_offset_y'}:
                next_y = value
            elif key in {'z', 'offset_z', 'pose_offset_z'}:
                next_z = value
            elif key in {'dx', 'add_x'}:
                next_x += value
            elif key in {'dy', 'add_y'}:
                next_y += value
            elif key in {'dz', 'add_z'}:
                next_z += value
            elif key in {'sx', 'scale_x', 'pose_scale_x'}:
                next_scale_x = value
            elif key in {'sy', 'scale_y', 'pose_scale_y'}:
                next_scale_y = value
            elif key in {'dsx', 'add_scale_x'}:
                next_scale_x += value
            elif key in {'dsy', 'add_scale_y'}:
                next_scale_y += value
            elif key in {'origin_x', 'scale_origin_x', 'pose_scale_origin_x'}:
                next_origin_x = value
            elif key in {'origin_y', 'scale_origin_y', 'pose_scale_origin_y'}:
                next_origin_y = value
            elif key in {
                'rot',
                'rotation',
                'rotation_deg',
                'rot_deg',
                'deg',
                'pose_rotation_deg',
            }:
                next_rotation_deg = value
            elif key in {'drot', 'drotation', 'drotation_deg', 'drot_deg', 'ddeg'}:
                next_rotation_deg += value
            elif key in {'rotation_rad', 'rot_rad', 'rad', 'pose_rotation_rad'}:
                next_rotation_deg = math.degrees(value)
            elif key in {'drotation_rad', 'drot_rad', 'drad'}:
                next_rotation_deg += math.degrees(value)
            elif key in {
                'rot_origin_x',
                'rotation_origin_x',
                'pose_rotation_origin_x',
            }:
                next_rotation_origin_x = value
            elif key in {
                'rot_origin_y',
                'rotation_origin_y',
                'pose_rotation_origin_y',
            }:
                next_rotation_origin_y = value
            else:
                raise ValueError(
                    f'Unknown pose calibration key {raw_key!r}; use x/y/z for offsets, '
                    'dx/dy/dz for incremental offsets, sx/sy for scale, dsx/dsy '
                    'for incremental scale, and rot_deg/deg or rot_rad for planar '
                    'rotation.'
                )

        return {
            'pose_scale_x': next_scale_x,
            'pose_scale_y': next_scale_y,
            'pose_scale_origin_x': next_origin_x,
            'pose_scale_origin_y': next_origin_y,
            'pose_rotation_deg': next_rotation_deg,
            'pose_rotation_origin_x': next_rotation_origin_x,
            'pose_rotation_origin_y': next_rotation_origin_y,
            'pose_offset_x': next_x,
            'pose_offset_y': next_y,
            'pose_offset_z': next_z,
        }

    def _on_stopper_command(self, message: String) -> None:
        try:
            updates = self._parse_stopper_command(message.data)
        except (ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(str(error))
            return

        self.stopper_states.update(updates)
        self.get_logger().info(f'Updated stopper states: {self.stopper_states}')

    def _parse_stopper_command(self, raw_command: str) -> Dict[str, str]:
        assignments = self._parse_assignments(raw_command, 'Stopper')
        updates: Dict[str, str] = {}
        for raw_selector, raw_state in assignments:
            selector = raw_selector.strip().upper()
            state = self._normalize_stopper_state(raw_state)
            if selector == 'ALL':
                for stopper_name in self.stopper_configs:
                    updates[stopper_name] = state
                continue
            if selector not in self.stopper_configs:
                allowed = ', '.join(['ALL', *sorted(self.stopper_configs)])
                raise ValueError(
                    f'Unknown stopper selector {selector!r}; use one of: {allowed}.'
                )
            updates[selector] = state
        return updates

    @staticmethod
    def _normalize_stopper_state(raw_state: str) -> str:
        state = str(raw_state).strip().upper()
        if state in {'1', 'ON', 'STOP', 'STOPPED', 'CLOSED', 'BLOCK', 'BLOCKED', 'TRUE'}:
            return '1'
        if state in {'0', 'OFF', 'OPEN', 'RELEASE', 'UNSTOP', 'UNBLOCK', 'FALSE'}:
            return '0'
        raise ValueError(
            f'Unknown stopper state {raw_state!r}; use 1/STOP/CLOSED or 0/OPEN/RELEASE.'
        )

    def _on_shuttle_control_command(self, message: String) -> None:
        try:
            updates = self._parse_shuttle_control_command(message.data)
        except (ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(str(error))
            return

        applied_updates: Dict[str, str] = {}
        for entity_name, action in updates.items():
            shuttle = self._find_shuttle(entity_name)
            if shuttle is None:
                self.get_logger().error(
                    f'Unknown shuttle {entity_name!r}; command ignored.'
                )
                continue
            try:
                self._apply_shuttle_action(shuttle, action)
                applied_updates[entity_name] = action
            except ValueError as error:
                self.get_logger().error(str(error))

        if applied_updates:
            self.get_logger().info(
                f'Applied shuttle commands: {applied_updates}; '
                f'shuttles={self._shuttle_summary()}'
            )

    def _parse_shuttle_control_command(self, raw_command: str) -> Dict[str, str]:
        assignments = self._parse_assignments(raw_command, 'Shuttle control')
        keyed_payload = {key.strip().lower(): value for key, value in assignments}
        if {'entity', 'entity_name', 'name', 'shuttle'} & set(keyed_payload):
            entity_name = (
                keyed_payload.get('entity')
                or keyed_payload.get('entity_name')
                or keyed_payload.get('name')
                or keyed_payload.get('shuttle')
            )
            raw_action = (
                keyed_payload.get('action')
                or keyed_payload.get('command')
                or keyed_payload.get('enabled')
                or keyed_payload.get('state')
                or keyed_payload.get('mode')
                or keyed_payload.get('power')
            )
            if entity_name is None or raw_action is None:
                raise ValueError(
                    'Shuttle control command with entity=... must also include '
                    'enabled=ON/OFF or action=RESET/REMOVE.'
                )
            return {entity_name: self._normalize_shuttle_action(raw_action)}

        updates: Dict[str, str] = {}
        for raw_selector, raw_action in assignments:
            selector = raw_selector.strip()
            action = self._normalize_shuttle_action(raw_action)
            if selector.upper() == 'ALL':
                for shuttle in self.shuttles:
                    updates[shuttle.entity_name] = action
            else:
                updates[selector] = action
        return updates

    @staticmethod
    def _normalize_enabled_state(raw_state: str) -> bool:
        state = str(raw_state).strip().upper()
        if state in {'1', 'ON', 'ENABLE', 'ENABLED', 'START', 'RUN', 'TRUE'}:
            return True
        if state in {'0', 'OFF', 'DISABLE', 'DISABLED', 'STOP', 'PAUSE', 'FALSE'}:
            return False
        raise ValueError(
            f'Unknown shuttle control state {raw_state!r}; use ON/OFF or ENABLE/DISABLE.'
        )

    @classmethod
    def _normalize_shuttle_action(cls, raw_state: str) -> str:
        state = str(raw_state).strip().upper()
        if state in {'RESET', 'RESPAWN', 'RECOVER', 'RESTART', 'HOME'}:
            return 'RESET'
        if state in {'REMOVE', 'DELETE', 'ERASE', 'DROP'}:
            return 'REMOVE'
        return 'ENABLE' if cls._normalize_enabled_state(raw_state) else 'DISABLE'

    def _apply_shuttle_action(self, shuttle: ManagedShuttle, action: str) -> None:
        if action == 'ENABLE':
            self._set_shuttle_enabled(shuttle, True)
            return
        if action == 'DISABLE':
            self._set_shuttle_enabled(shuttle, False)
            return
        if action == 'RESET':
            self._reset_shuttle(shuttle)
            return
        if action == 'REMOVE':
            self._remove_shuttle(shuttle)
            return
        raise ValueError(f'Unsupported shuttle action {action!r}.')

    def _set_shuttle_enabled(self, shuttle: ManagedShuttle, enabled: bool) -> None:
        shuttle.enabled = enabled
        shuttle.blocked_by = None
        shuttle.collision_distance_m = None
        if not enabled:
            shuttle.core.state.mode = WAITING
            shuttle.stopped_by = 'NOT_DEPLOYED' if not shuttle.deployed else 'DISABLED'
            shuttle.stopper_distance_m = 0.0
            return

        if not shuttle.deployed:
            shuttle.deployed = True
            shuttle.pending_set_pose = None
            shuttle.last_gazebo_set_pose_time = None
            shuttle.stopped_by = None
            shuttle.stopper_distance_m = None

        if shuttle.stopped_by in {'DISABLED', 'NOT_DEPLOYED'}:
            shuttle.stopped_by = None
            shuttle.stopper_distance_m = None
        if shuttle.core.state.mode in {WAITING, FALLING} and shuttle.core.state.speed > 0.0:
            shuttle.core.state.mode = MOVING

    def _reset_shuttle(self, shuttle: ManagedShuttle) -> None:
        if shuttle.pending_spawn is not None and not shuttle.pending_spawn.done():
            raise ValueError(
                f'Cannot reset {shuttle.entity_name} while its Gazebo spawn request is still in flight.'
            )

        occupied_by = self._start_slot_occupancy_blocker(
            shuttle.start_slot,
            ignore_entity_name=shuttle.entity_name,
        )
        if occupied_by is not None:
            blocker_name, distance_m = occupied_by
            raise ValueError(
                f'Cannot reset {shuttle.entity_name} to slot {shuttle.start_slot}: '
                f'occupied by {blocker_name} at distance {distance_m:.3f} m.'
            )

        (
            resolved_slot,
            start_pose,
            start_snap_distance_m,
            initial_segment,
            initial_s,
        ) = self._resolve_allowed_start_slot(shuttle.start_slot, self.start_snap_tolerance_m)
        shuttle.start_slot = resolved_slot
        shuttle.start_pose = start_pose
        shuttle.start_snap_distance_m = start_snap_distance_m
        shuttle.core.state = ShuttleState(
            current_segment=initial_segment,
            s=initial_s,
            speed=shuttle.core.state.speed,
            mode=MOVING if shuttle.enabled and shuttle.core.state.speed > 0.0 else WAITING,
        )
        shuttle.blocked_by = None
        shuttle.collision_distance_m = None
        shuttle.pending_set_pose = None
        shuttle.last_gazebo_set_pose_time = None
        if shuttle.enabled:
            shuttle.stopped_by = None
            shuttle.stopper_distance_m = None
        else:
            shuttle.stopped_by = 'NOT_DEPLOYED' if not shuttle.deployed else 'DISABLED'
            shuttle.stopper_distance_m = 0.0

        self.get_logger().info(
            f'Reset shuttle {shuttle.entity_name} to slot {shuttle.start_slot} '
            f'({initial_segment}@{initial_s:.3f}).'
        )

    def _remove_shuttle(self, shuttle: ManagedShuttle) -> None:
        if shuttle.pending_spawn is not None and not shuttle.pending_spawn.done():
            raise ValueError(
                f'Cannot remove {shuttle.entity_name} while its Gazebo spawn request is still in flight.'
            )

        if self._find_shuttle(shuttle.entity_name) is None:
            return

        should_delete_entity = shuttle.gazebo_spawned or self._is_preloaded_shuttle_entity(
            shuttle.entity_name
        )
        if not should_delete_entity:
            self.shuttles = [
                managed for managed in self.shuttles
                if managed.entity_name != shuttle.entity_name
            ]
            self.get_logger().info(
                f'Removed shuttle {shuttle.entity_name} from node state.'
            )
            return

        if self.delete_client is None or not self.enable_gazebo_delete:
            raise ValueError(
                f'Cannot remove {shuttle.entity_name} from Gazebo because delete support is disabled.'
            )
        if not self.delete_client.service_is_ready():
            raise ValueError(
                f'Cannot remove {shuttle.entity_name} because the Gazebo delete service is not ready.'
            )

        request = DeleteEntity.Request()
        request.entity.name = shuttle.entity_name
        request.entity.type = Entity.MODEL
        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda result, entity_name=shuttle.entity_name: self._on_delete_entity_result(
                entity_name,
                result,
            )
        )
        self.deleting_entity_names.add(shuttle.entity_name)
        self.shuttles = [
            managed for managed in self.shuttles
            if managed.entity_name != shuttle.entity_name
        ]
        self.get_logger().info(f'Requested Gazebo removal for {shuttle.entity_name}.')

    def _on_delete_entity_result(self, entity_name: str, future) -> None:
        self.deleting_entity_names.discard(entity_name)
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Gazebo delete request for {entity_name} failed: {error}'
            )
            return

        if not response.success:
            self.get_logger().error(
                f'Gazebo delete service rejected {entity_name}; the model may still exist in the simulation.'
            )
            return

        match = self.preloaded_entity_pattern.match(entity_name)
        if match and int(match.group(1)) <= self.preloaded_shuttle_count:
            self.deleted_preloaded_entity_names.add(entity_name)
        self.get_logger().info(f'Gazebo removed {entity_name}.')

    def _find_shuttle(self, entity_name: str) -> ManagedShuttle | None:
        for shuttle in self.shuttles:
            if shuttle.entity_name == entity_name:
                return shuttle
        return None

    @staticmethod
    def _parse_assignments(raw_command: str, command_name: str) -> list[tuple[str, str]]:
        command = raw_command.strip()
        if not command:
            raise ValueError(f'Empty {command_name.lower()} command')

        if command.startswith('{'):
            payload = json.loads(command)
            if not isinstance(payload, dict):
                raise ValueError(f'{command_name} JSON command must be an object.')
            return [(str(key), str(value)) for key, value in payload.items()]

        assignments = []
        for token in re.split(r'[\s,;]+', command.replace(':', '=')):
            if not token:
                continue
            if '=' not in token:
                raise ValueError(
                    f'{command_name} command must look like NAME=VALUE, got {token!r}'
                )
            key, value = token.split('=', 1)
            assignments.append((key, value))
        return assignments

    def _on_switch_command(self, message: String) -> None:
        try:
            updates, visual_command = self._parse_switch_command(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        if updates:
            self.switch_states.update(updates)
            self.get_logger().info(
                f'Updated route switch states: {self._public_switch_state_map(self.switch_states)}'
            )

        if self.publish_visual_switch_commands and visual_command:
            visual_message = String()
            visual_message.data = visual_command
            self.visual_switch_publisher.publish(visual_message)
            self.get_logger().info(f'Published visual switch command: {visual_command}')

    def _on_visual_switch_state(self, message: String) -> None:
        updates = self._parse_visual_switch_state_summary(message.data)
        if not updates:
            return

        changed = {
            switch_name: state
            for switch_name, state in updates.items()
            if self.switch_states.get(switch_name) != state
        }
        self.switch_states.update(updates)
        if changed:
            self.get_logger().info(
                'Synced route switch states from visual controller: '
                f'{_ordered_switch_states(self.switch_states)}'
            )

    def _parse_visual_switch_state_summary(self, raw_summary: str) -> Dict[str, str]:
        candidates: Dict[str, Dict[str, str]] = {}
        for token in re.split(r'[,\n;]+', raw_summary):
            token = token.strip()
            if not token or '=' not in token:
                continue

            raw_name, raw_state = token.split('=', 1)
            station, side = self._station_from_visual_switch_name(raw_name.strip())
            if station is None:
                continue

            try:
                state = self._normalize_commanded_switch_state(raw_state)
            except ValueError:
                continue

            candidates.setdefault(station, {})[side] = state

        updates: Dict[str, str] = {}
        for station, states_by_side in candidates.items():
            if self.rail_side in states_by_side:
                updates[station] = states_by_side[self.rail_side]
            elif 'station' in states_by_side:
                updates[station] = states_by_side['station']
            else:
                other_side = 'left' if self.rail_side == 'right' else 'right'
                if other_side in states_by_side:
                    updates[station] = states_by_side[other_side]
        return updates

    @staticmethod
    def _station_from_visual_switch_name(raw_name: str) -> tuple[str | None, str]:
        name = raw_name.strip().upper()
        if name in {'ALL', 'RIGHT', 'LEFT'}:
            return None, 'group'

        station_match = re.match(r'^(A[1-4])$', name)
        if station_match:
            station = station_match.group(1)
            return station, 'station'

        mapped_selector = RIGHT_VISUAL_SWITCH_SELECTOR_MAP.get(name)
        if mapped_selector is not None:
            return mapped_selector, 'right'

        mapped_selector = LEFT_VISUAL_SWITCH_SELECTOR_MAP.get(name)
        if mapped_selector is not None:
            return mapped_selector, 'left'

        return None, 'unknown'

    def _parse_switch_command(self, raw_command: str) -> tuple[Dict[str, str], str]:
        stripped = raw_command.strip()
        if not stripped:
            raise ValueError('Empty switch command')

        if stripped.startswith('{'):
            payload = json.loads(stripped)
            assignments = [(str(switch_name), str(state)) for switch_name, state in payload.items()]
        else:
            assignments = []
            for token in re.split(r'[\s,;]+', stripped.replace(':', '=')):
                if not token:
                    continue
                if '=' not in token:
                    raise ValueError(f'Switch command must look like A1=G, got {token!r}')
                selector, raw_state = token.split('=', 1)
                assignments.append((selector, raw_state))

        updates: Dict[str, str] = {}
        visual_entries = []
        for selector, raw_state in assignments:
            selector_name = selector.strip().upper()
            state = self._normalize_commanded_switch_state(raw_state)
            logic_targets = self._logic_targets_for_selector(selector_name)
            visual_selector = self._visual_selector_for_selector(selector_name)

            if not logic_targets and visual_selector is None:
                raise ValueError(
                    f'Unknown switch selector {selector_name!r}; use A1..A4, A1R/A1L, '
                    'RIGHT, LEFT, or ALL.'
                )

            for switch_name in logic_targets:
                updates[switch_name] = state

            if visual_selector is not None:
                visual_entries.append(
                    f'{visual_selector}={self._visual_mode_for_state(state)}'
                )

        return updates, ', '.join(visual_entries)

    def _normalize_commanded_switch_state(self, raw_state: str) -> str:
        state = raw_state.strip().upper()
        if state in {'G', 'E', 'GRAND', 'GRAND_BOUCLE', 'BIG', 'LARGE', 'EXTERIOR'}:
            return 'G'
        if state in {'S', 'I', 'PETIT', 'PETIT_BOUCLE', 'SMALL', 'INTERIOR'}:
            return 'S'
        return self.network.normalized_switch_state(state)

    def _logic_targets_for_selector(self, selector_name: str) -> list[str]:
        if selector_name == 'ALL':
            return sorted(self.network.switches)
        if selector_name == self.active_visual_group_selector:
            return sorted(self.network.switches)
        if selector_name in {'RIGHT', 'LEFT'}:
            return []

        station_match = re.match(r'^(A[1-4])$', selector_name)
        if station_match:
            station = station_match.group(1)
            return [station]

        mapped_selector = self.active_visual_switch_selector_map.get(selector_name)
        if mapped_selector is not None:
            return [mapped_selector]

        return []

    def _visual_selector_for_selector(self, selector_name: str) -> str | None:
        if selector_name == 'ALL':
            return self.active_visual_group_selector
        if selector_name in {'RIGHT', 'LEFT'}:
            return selector_name if selector_name == self.active_visual_group_selector else None

        station_match = re.match(r'^(A[1-4])$', selector_name)
        if station_match:
            return f'{station_match.group(1)}{self.active_visual_selector_suffix}'

        if selector_name in self.active_visual_switch_selector_map:
            return selector_name

        return None

    @staticmethod
    def _visual_mode_for_state(state: str) -> str:
        return 'GRAND_BOUCLE' if state == 'G' else 'PETIT_BOUCLE'

    def _on_parameter_update(self, parameters) -> SetParametersResult:
        numeric_parameters = {
            'pose_transform_a',
            'pose_transform_b',
            'pose_transform_tx',
            'pose_transform_c',
            'pose_transform_d',
            'pose_transform_ty',
            'pose_transform_z_offset',
            'pose_transform_yaw_offset',
            'pose_scale_x',
            'pose_scale_y',
            'pose_scale_origin_x',
            'pose_scale_origin_y',
            'pose_rotation_deg',
            'pose_rotation_origin_x',
            'pose_rotation_origin_y',
            'pose_offset_x',
            'pose_offset_y',
            'pose_offset_z',
            'gazebo_set_pose_rate_hz',
            'shuttle_collision_distance_m',
            'collision_search_iterations',
            'start_slot_occupancy_radius_m',
        }
        boolean_parameters = {
            'enable_collision_avoidance',
            'enable_gazebo_pose_transform',
            'publish_visual_switch_commands',
            'reject_occupied_start_slots',
        }

        try:
            for parameter in parameters:
                if parameter.name in numeric_parameters:
                    if parameter.name == 'gazebo_set_pose_rate_hz':
                        rate = float(parameter.value)
                        self.gazebo_set_pose_period = 1.0 / max(rate, 1.0)
                    elif parameter.name == 'collision_search_iterations':
                        self.collision_search_iterations = max(1, int(parameter.value))
                    else:
                        setattr(self, parameter.name, float(parameter.value))
                elif parameter.name in boolean_parameters:
                    setattr(self, parameter.name, bool(parameter.value))
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_tick).nanoseconds / 1e9)
        self.last_tick = now

        if not self.shuttles:
            self._publish_state([], [])
            self._publish_sensor_state()
            self._publish_position_sensor_state()
            return

        raw_poses = []
        gazebo_poses = []
        occupied_poses = {
            shuttle.entity_name: self._to_gazebo_pose(shuttle.core.pose())
            for shuttle in self.shuttles
            if shuttle.deployed
        }
        for shuttle in self.shuttles:
            if not shuttle.deployed:
                shuttle.blocked_by = None
                shuttle.collision_distance_m = None
                shuttle.stopped_by = 'NOT_DEPLOYED'
                shuttle.stopper_distance_m = 0.0
                raw_poses.append(shuttle.core.pose())
                gazebo_poses.append(self._hidden_gazebo_pose(shuttle))
                continue

            if not self._spawn_ready_for_motion(shuttle):
                pose = shuttle.core.pose()
                gazebo_pose = self._to_gazebo_pose(pose)
                occupied_poses[shuttle.entity_name] = gazebo_pose
                raw_poses.append(pose)
                gazebo_poses.append(gazebo_pose)
                continue

            pose = self._step_with_motion_guards(
                shuttle=shuttle,
                dt=dt,
                occupied_poses=occupied_poses,
            )
            gazebo_pose = self._to_gazebo_pose(pose)
            pose_message = self._publish_pose(shuttle, gazebo_pose)
            self._send_gazebo_pose(shuttle, pose_message)
            occupied_poses[shuttle.entity_name] = gazebo_pose
            raw_poses.append(pose)
            gazebo_poses.append(gazebo_pose)

            if pose.mode == FALLING:
                self.get_logger().error(
                    f'Shuttle {shuttle.entity_name} entered FALLING mode at '
                    f'segment={self._public_segment_name(pose.current_segment)}, s={pose.s:.3f}'
                )

        self._publish_state(raw_poses, gazebo_poses)
        self._publish_sensor_state()
        self._publish_position_sensor_state()

    @staticmethod
    def _hidden_gazebo_pose(shuttle: ManagedShuttle) -> ShuttlePose:
        return ShuttlePose(
            x=0.0,
            y=0.0,
            z=-10.0,
            yaw=0.0,
            current_segment=shuttle.core.state.current_segment,
            s=shuttle.core.state.s,
            mode=WAITING,
        )

    def _step_with_motion_guards(
        self,
        shuttle: ManagedShuttle,
        dt: float,
        occupied_poses: Dict[str, ShuttlePose],
    ) -> ShuttlePose:
        if not shuttle.enabled:
            shuttle.core.state.mode = WAITING
            shuttle.blocked_by = None
            shuttle.collision_distance_m = None
            shuttle.stopped_by = 'DISABLED'
            shuttle.stopper_distance_m = 0.0
            return shuttle.core.pose()

        shuttle.stopped_by = None
        shuttle.stopper_distance_m = None
        active_stop = self._active_stopper_ahead(shuttle)
        effective_dt = dt
        stop_reached = False
        if active_stop is not None and shuttle.core.state.speed > 0.0:
            stopper_name, stop_point, distance_m = active_stop
            if distance_m <= 1e-6:
                shuttle.core.state.s = stop_point.stop_s
                shuttle.core.state.mode = WAITING
                shuttle.blocked_by = None
                shuttle.collision_distance_m = None
                shuttle.stopped_by = stopper_name
                shuttle.stopper_distance_m = 0.0
                return shuttle.core.pose()

            time_to_stop = distance_m / shuttle.core.state.speed
            if time_to_stop <= dt:
                effective_dt = time_to_stop
                stop_reached = True

        pose = self._step_with_collision_avoidance(
            shuttle=shuttle,
            dt=effective_dt,
            occupied_poses=occupied_poses,
        )
        if shuttle.blocked_by is not None or pose.mode == FALLING:
            return pose

        if stop_reached and active_stop is not None:
            stopper_name, stop_point, _distance_m = active_stop
            if shuttle.core.state.current_segment == stop_point.segment:
                shuttle.core.state.s = stop_point.stop_s
            shuttle.core.state.mode = WAITING
            shuttle.stopped_by = stopper_name
            shuttle.stopper_distance_m = 0.0
            return shuttle.core.pose()

        shuttle.stopped_by = None
        shuttle.stopper_distance_m = None
        return pose

    def _active_stopper_ahead(
        self,
        shuttle: ManagedShuttle,
    ) -> tuple[str, StopPoint, float] | None:
        state = shuttle.core.state
        candidates: list[tuple[str, StopPoint, float]] = []
        for stopper_name, stopper_config in self.stopper_configs.items():
            if self.stopper_states.get(stopper_name, '0') != '1':
                continue
            for stop_point in stopper_config.stop_points:
                if stop_point.segment != state.current_segment:
                    continue
                distance_m = stop_point.stop_s - state.s
                if distance_m >= -1e-6:
                    candidates.append((stopper_name, stop_point, max(0.0, distance_m)))
        if not candidates:
            return None
        return sorted(candidates, key=lambda item: item[2])[0]

    def _step_with_collision_avoidance(
        self,
        shuttle: ManagedShuttle,
        dt: float,
        occupied_poses: Dict[str, ShuttlePose],
    ) -> ShuttlePose:
        if not self.enable_collision_avoidance or dt <= 0.0:
            shuttle.blocked_by = None
            shuttle.collision_distance_m = None
            return shuttle.core.step(dt, switch_states=self.switch_states)

        start_state = self._snapshot_shuttle_state(shuttle)
        start_pose = shuttle.core.pose()
        start_gazebo_pose = self._to_gazebo_pose(start_pose)
        blockers = self._collision_blockers(shuttle.entity_name, start_gazebo_pose, occupied_poses)
        if blockers:
            blocker_name, distance = blockers[0]
            self._restore_shuttle_state(shuttle, start_state)
            shuttle.core.state.mode = WAITING
            shuttle.blocked_by = blocker_name
            shuttle.collision_distance_m = distance
            return shuttle.core.pose()

        self._restore_shuttle_state(shuttle, start_state)
        proposed_pose = shuttle.core.step(dt, switch_states=self.switch_states)
        proposed_gazebo_pose = self._to_gazebo_pose(proposed_pose)
        blockers = self._collision_blockers(
            shuttle.entity_name,
            proposed_gazebo_pose,
            occupied_poses,
        )
        if not blockers:
            shuttle.blocked_by = None
            shuttle.collision_distance_m = None
            return proposed_pose

        high = dt
        low = 0.0
        blocker_name, distance = blockers[0]
        for _ in range(max(1, self.collision_search_iterations)):
            mid = 0.5 * (low + high)
            self._restore_shuttle_state(shuttle, start_state)
            mid_pose = shuttle.core.step(mid, switch_states=self.switch_states)
            mid_gazebo_pose = self._to_gazebo_pose(mid_pose)
            mid_blockers = self._collision_blockers(
                shuttle.entity_name,
                mid_gazebo_pose,
                occupied_poses,
            )
            if mid_blockers:
                high = mid
                blocker_name, distance = mid_blockers[0]
            else:
                low = mid

        self._restore_shuttle_state(shuttle, start_state)
        shuttle.core.step(low, switch_states=self.switch_states)
        shuttle.core.state.mode = WAITING
        shuttle.blocked_by = blocker_name
        shuttle.collision_distance_m = distance
        return shuttle.core.pose()

    @staticmethod
    def _snapshot_shuttle_state(shuttle: ManagedShuttle) -> ShuttleState:
        state = shuttle.core.state
        return ShuttleState(
            current_segment=state.current_segment,
            s=state.s,
            speed=state.speed,
            mode=state.mode,
        )

    @staticmethod
    def _restore_shuttle_state(shuttle: ManagedShuttle, state: ShuttleState) -> None:
        shuttle.core.state = ShuttleState(
            current_segment=state.current_segment,
            s=state.s,
            speed=state.speed,
            mode=state.mode,
        )

    def _collision_blockers(
        self,
        entity_name: str,
        pose: ShuttlePose,
        occupied_poses: Dict[str, ShuttlePose],
    ) -> list[tuple[str, float]]:
        blockers = []
        for other_name, other_pose in occupied_poses.items():
            if other_name == entity_name:
                continue
            distance = math.hypot(pose.x - other_pose.x, pose.y - other_pose.y)
            if distance < self.shuttle_collision_distance_m:
                blockers.append((other_name, distance))
        return sorted(blockers, key=lambda item: item[1])

    def _to_gazebo_pose(self, pose: ShuttlePose) -> ShuttlePose:
        if not self.enable_gazebo_pose_transform:
            return pose

        base_x = (
            self.pose_transform_a * pose.x
            + self.pose_transform_b * pose.y
            + self.pose_transform_tx
        )
        base_y = (
            self.pose_transform_c * pose.x
            + self.pose_transform_d * pose.y
            + self.pose_transform_ty
        )
        x = (
            self.pose_scale_origin_x
            + (base_x - self.pose_scale_origin_x) * self.pose_scale_x
        )
        y = (
            self.pose_scale_origin_y
            + (base_y - self.pose_scale_origin_y) * self.pose_scale_y
        )
        x, y = self._apply_planar_rotation(x, y)
        x += self.pose_offset_x
        y += self.pose_offset_y

        raw_direction_x = math.cos(pose.yaw)
        raw_direction_y = math.sin(pose.yaw)
        transformed_direction_x = (
            self.pose_transform_a * raw_direction_x
            + self.pose_transform_b * raw_direction_y
        )
        transformed_direction_y = (
            self.pose_transform_c * raw_direction_x
            + self.pose_transform_d * raw_direction_y
        )
        yaw = math.atan2(
            transformed_direction_y * self.pose_scale_y,
            transformed_direction_x * self.pose_scale_x,
        )
        yaw += self._pose_rotation_rad()
        yaw += self.pose_transform_yaw_offset

        return ShuttlePose(
            x=x,
            y=y,
            z=pose.z + self.pose_transform_z_offset + self.pose_offset_z,
            yaw=yaw,
            current_segment=pose.current_segment,
            s=pose.s,
            mode=pose.mode,
        )

    def _publish_pose(self, shuttle: ManagedShuttle, pose: ShuttlePose) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.position.z = pose.z
        qx, qy, qz, qw = _yaw_to_quaternion(pose.yaw)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        shuttle.pose_publisher.publish(message)
        return message

    def _send_gazebo_pose(
        self,
        shuttle: ManagedShuttle,
        pose_message: PoseStamped,
    ) -> None:
        if not self.enable_gazebo_set_pose or self.set_pose_client is None:
            return
        now = self.get_clock().now()
        last_sent = shuttle.last_gazebo_set_pose_time
        if last_sent is not None:
            elapsed = (now - last_sent).nanoseconds / 1e9
            if elapsed < self.gazebo_set_pose_period:
                return
        if shuttle.pending_set_pose is not None and not shuttle.pending_set_pose.done():
            return
        if not self.set_pose_client.service_is_ready():
            if not shuttle.set_pose_warning_logged:
                self.get_logger().warn(
                    'Gazebo set_pose service is not ready yet; still publishing pose topic.'
                )
                shuttle.set_pose_warning_logged = True
            return

        request = SetEntityPose.Request()
        request.entity.name = shuttle.entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose_message.pose
        shuttle.pending_set_pose = self.set_pose_client.call_async(request)
        shuttle.last_gazebo_set_pose_time = now

    def _sensor_events(self) -> list[dict]:
        events = []
        for shuttle in self.shuttles:
            if not shuttle.deployed:
                continue
            state = shuttle.core.state
            for stopper_name, stopper_config in self.stopper_configs.items():
                for stop_point in stopper_config.stop_points:
                    if stop_point.segment != state.current_segment:
                        continue
                    distance_m = stop_point.stop_s - state.s
                    if 0.0 <= distance_m <= stop_point.sensor_distance_m:
                        events.append(
                            {
                                'sensor': self._public_sensor_name(f'{stopper_name}_APPROACH'),
                                'stopper': self._public_switch_name(stopper_name),
                                'before_switch': self._public_switch_name(
                                    stopper_config.before_switch
                                ),
                                'entity_name': shuttle.entity_name,
                                'segment': self._public_segment_name(state.current_segment),
                                'distance_m': distance_m,
                                'stopper_state': self.stopper_states.get(stopper_name, '0'),
                                'workflow': 'sensor -> stop shuttle -> move switch -> unstop shuttle',
                            }
                        )
        return events

    def _position_sensor_events(self) -> list[dict]:
        events = []
        for shuttle in self.shuttles:
            if not shuttle.deployed:
                continue
            state = shuttle.core.state
            for sensor_name, sensor_config in self.position_sensor_configs.items():
                for point in sensor_config.points:
                    if point.segment != state.current_segment:
                        continue
                    distance_m = abs(point.sensor_s - state.s)
                    if distance_m > point.sensor_distance_m:
                        continue

                    event = {
                        'sensor': self._public_sensor_name(sensor_name),
                        'sensor_kind': sensor_config.sensor_kind,
                        'entity_name': shuttle.entity_name,
                        'segment': self._public_segment_name(state.current_segment),
                        'distance_m': distance_m,
                    }
                    if sensor_config.switch_name is not None:
                        event['switch'] = self._public_switch_name(sensor_config.switch_name)
                    if sensor_config.branch_state is not None:
                        event['branch_state'] = sensor_config.branch_state
                    if sensor_config.loop_side is not None:
                        event['loop_side'] = sensor_config.loop_side
                    if sensor_config.index_zone is not None:
                        event['index_zone'] = sensor_config.index_zone
                    if sensor_config.start_slot is not None:
                        event['start_slot'] = sensor_config.start_slot
                    if sensor_config.aliases:
                        event['aliases'] = [
                            self._public_sensor_name(alias) for alias in sensor_config.aliases
                        ]
                    events.append(event)
                    break
        return events

    def _publish_sensor_state(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                'sensors': self._sensor_events(),
                'stopper_states': self._public_switch_state_map(self.stopper_states),
            },
            sort_keys=True,
        )
        self.sensor_state_publisher.publish(message)

    def _publish_position_sensor_state(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                'sensors': self._position_sensor_events(),
            },
            sort_keys=True,
        )
        self.position_sensor_state_publisher.publish(message)

    def _public_switch_name(self, name: str) -> str:
        return _canonical_switch_name(name)

    def _public_segment_name(self, name: str) -> str:
        canonical_name = _canonical_segment_name(name)
        if self.rail_side != 'left':
            return canonical_name
        return LEFT_PUBLIC_SEGMENT_NAME_MAP.get(canonical_name, canonical_name)

    def _public_sensor_name(self, name: str) -> str:
        return _canonical_sensor_name(name)

    def _public_switch_state_map(self, raw_states: Dict[str, str]) -> Dict[str, str]:
        return _ordered_switch_states(raw_states)

    def _publish_state(
        self,
        raw_poses: list[ShuttlePose],
        gazebo_poses: list[ShuttlePose],
    ) -> None:
        shuttles_payload = []
        for shuttle, pose, gazebo_pose in zip(self.shuttles, raw_poses, gazebo_poses):
            pose_payload = asdict(pose)
            pose_payload['current_segment'] = self._public_segment_name(pose.current_segment)
            gazebo_pose_payload = asdict(gazebo_pose)
            gazebo_pose_payload['current_segment'] = self._public_segment_name(
                gazebo_pose.current_segment
            )
            shuttles_payload.append(
                {
                    **pose_payload,
                    'entity_name': shuttle.entity_name,
                    'blocked_by': shuttle.blocked_by,
                    'collision_distance_m': shuttle.collision_distance_m,
                    'enabled': shuttle.enabled,
                    'stopped_by': shuttle.stopped_by,
                    'stopper_distance_m': shuttle.stopper_distance_m,
                    'gazebo_spawned': shuttle.gazebo_spawned,
                    'deployed': shuttle.deployed,
                    'gazebo_pose': gazebo_pose_payload,
                    'start_slot': shuttle.start_slot,
                    'start_snap_distance_m': shuttle.start_snap_distance_m,
                    'speed': shuttle.core.state.speed,
                }
            )

        primary_payload = {
            'current_segment': None,
            'entity_name': None,
            'gazebo_pose': None,
            'mode': None,
            's': None,
            'speed': None,
            'start_slot': None,
            'start_snap_distance_m': None,
            'x': None,
            'y': None,
            'yaw': None,
            'z': None,
        }
        if self.shuttles and raw_poses and gazebo_poses:
            first_shuttle = self.shuttles[0]
            first_pose = raw_poses[0]
            first_gazebo_pose = gazebo_poses[0]
            first_pose_payload = asdict(first_pose)
            first_pose_payload['current_segment'] = self._public_segment_name(
                first_pose.current_segment
            )
            first_gazebo_pose_payload = asdict(first_gazebo_pose)
            first_gazebo_pose_payload['current_segment'] = self._public_segment_name(
                first_gazebo_pose.current_segment
            )
            primary_payload.update(first_pose_payload)
            primary_payload.update(
                {
                    'entity_name': first_shuttle.entity_name,
                    'gazebo_pose': first_gazebo_pose_payload,
                    'speed': first_shuttle.core.state.speed,
                    'start_slot': first_shuttle.start_slot,
                    'start_snap_distance_m': first_shuttle.start_snap_distance_m,
                }
            )

        message = String()
        message.data = json.dumps(
            {
                **primary_payload,
                'pose_offset': {
                    'x': self.pose_offset_x,
                    'y': self.pose_offset_y,
                    'z': self.pose_offset_z,
                },
                'pose_scale': {
                    'x': self.pose_scale_x,
                    'y': self.pose_scale_y,
                    'origin_x': self.pose_scale_origin_x,
                    'origin_y': self.pose_scale_origin_y,
                },
                'pose_rotation': {
                    'deg': self.pose_rotation_deg,
                    'origin_x': self.pose_rotation_origin_x,
                    'origin_y': self.pose_rotation_origin_y,
                },
                'shuttle_count': len(self.shuttles),
                'collision_avoidance': {
                    'enabled': self.enable_collision_avoidance,
                    'distance_m': self.shuttle_collision_distance_m,
                },
                'shuttles': shuttles_payload,
                'switch_states': self._public_switch_state_map(self.switch_states),
                'stopper_states': self._public_switch_state_map(self.stopper_states),
                'sensor_events': self._sensor_events(),
                'position_sensor_events': self._position_sensor_events(),
            },
            sort_keys=True,
        )
        self.state_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = Room315KinematicShuttleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
