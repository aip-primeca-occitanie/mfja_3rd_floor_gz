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
            / 'rail_network.yaml'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[2]
            / 'mfja_robot_control_config'
            / 'config'
            / 'room_315_kinematics'
            / 'rail_network.yaml'
        )


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
    '1': AllowedStartPose(-14.95, -3.86, 0.84, 0.0, 0.0, 3.14),
    '2': AllowedStartPose(-15.43, -3.86, 0.84, 0.0, 0.0, 3.14),
    '3': AllowedStartPose(-15.24, -5.54, 0.84, 0.0, 0.0, 0.0),
    '4': AllowedStartPose(-14.77, -5.54, 0.84, 0.0, 0.0, 0.0),
}


VISUAL_SWITCH_SELECTOR_MAP = {
    'A1R': ('A3', 'right'),
    'A2R': ('A4', 'right'),
    'A3R': ('A1', 'right'),
    'A4R': ('A2', 'right'),
    'A1L': ('A4', 'left'),
    'A2L': ('A3', 'left'),
    'A3L': ('A2', 'left'),
    'A4L': ('A1', 'left'),
    'A1_DROIT_SWITCH': ('A3', 'right'),
    'A2_DROIT_SWITCH': ('A4', 'right'),
    'A3_DROIT_SWITCH': ('A1', 'right'),
    'A4_DROIT_SWITCH': ('A2', 'right'),
    'A1_GAUCHE_SWITCH': ('A4', 'left'),
    'A2_GAUCHE_SWITCH': ('A3', 'left'),
    'A3_GAUCHE_SWITCH': ('A2', 'left'),
    'A4_GAUCHE_SWITCH': ('A1', 'left'),
}
RIGHT_VISUAL_SWITCH_SELECTOR_MAP = {
    selector_name: station
    for selector_name, (station, side) in VISUAL_SWITCH_SELECTOR_MAP.items()
    if side == 'right'
}
LEFT_VISUAL_SWITCH_SELECTORS = {
    selector_name
    for selector_name, (_station, side) in VISUAL_SWITCH_SELECTOR_MAP.items()
    if side == 'left'
}

LOGICAL_SWITCH_SELECTOR_MAP = {
    'A1': 'A3',
    'A2': 'A4',
    'A3': 'A1',
    'A4': 'A2',
}
PUBLIC_SWITCH_ORDER = ('A1', 'A2', 'A3', 'A4')
INTERNAL_TO_PUBLIC_SWITCH_MAP = {
    internal_name: public_name
    for public_name, internal_name in LOGICAL_SWITCH_SELECTOR_MAP.items()
}


def _public_switch_name(name: str) -> str:
    normalized_name = str(name).strip().upper()
    return INTERNAL_TO_PUBLIC_SWITCH_MAP.get(normalized_name, normalized_name)


def _public_switch_states(switch_states: Dict[str, str]) -> Dict[str, str]:
    ordered: Dict[str, str] = {}
    for switch_name in PUBLIC_SWITCH_ORDER:
        internal_name = LOGICAL_SWITCH_SELECTOR_MAP[switch_name]
        if internal_name in switch_states:
            ordered[switch_name] = switch_states[internal_name]

    for internal_name, state in switch_states.items():
        public_name = _public_switch_name(internal_name)
        if public_name not in ordered:
            ordered[public_name] = state
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
        self.declare_parameter('path_backend', CUBIC_HERMITE_PATH_BACKEND)
        self.declare_parameter('arc_length_samples_per_edge', 16)
        self.declare_parameter('shuttle_count', 1)
        self.declare_parameter('start_slot', 2)
        self.declare_parameter('start_slots', '')
        self.declare_parameter('start_snap_tolerance_m', 0.25)
        self.declare_parameter('initial_segment', 'A14')
        self.declare_parameter('initial_s', 0.0)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('enable_collision_avoidance', True)
        self.declare_parameter('shuttle_collision_distance_m', 0.33)
        self.declare_parameter('collision_search_iterations', 12)
        self.declare_parameter('pose_topic', '/room_315/shuttle/pose_cmd')
        self.declare_parameter('pose_topic_prefix', '/room_315/shuttles')
        self.declare_parameter('state_topic', '/room_315/shuttle/state')
        self.declare_parameter('add_shuttle_command_topic', '/room_315/shuttle/add_cmd')
        self.declare_parameter('shuttle_control_command_topic', '/room_315/shuttle/control_cmd')
        self.declare_parameter('switch_command_topic', '/room_315/switch_states')
        self.declare_parameter('stopper_command_topic', '/room_315/stopper_states')
        self.declare_parameter('sensor_state_topic', '/room_315/sensors/switch_approach')
        self.declare_parameter('pose_offset_command_topic', '/room_315/shuttle/pose_offset_cmd')
        self.declare_parameter('visual_switch_command_topic', '/mfja/conveyor/switch_cmd')
        self.declare_parameter('visual_switch_state_topic', '/mfja/conveyor/switch_states')
        self.declare_parameter('sync_from_visual_switch_states', True)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('enable_gazebo_set_pose', False)
        self.declare_parameter('gazebo_world_name', 'room_315_only')
        self.declare_parameter('gazebo_set_pose_service', '')
        self.declare_parameter('enable_gazebo_spawn', True)
        self.declare_parameter('gazebo_spawn_service', '')
        self.declare_parameter('shuttle_model_sdf', str(_default_shuttle_model_sdf_path()))
        self.declare_parameter('preloaded_shuttle_count', 4)
        self.declare_parameter('reject_occupied_start_slots', True)
        self.declare_parameter('start_slot_occupancy_radius_m', 0.33)
        self.declare_parameter('gazebo_entity_name', 'room315_shuttle_1')
        self.declare_parameter('gazebo_entity_names', '')
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
        self.declare_parameter('pose_offset_x', 0.0)
        self.declare_parameter('pose_offset_y', 0.0)
        self.declare_parameter('pose_offset_z', 0.0)

        network_path = Path(str(self.get_parameter('network_yaml').value))
        path_backend = str(self.get_parameter('path_backend').value)
        arc_length_samples_per_edge = int(
            self.get_parameter('arc_length_samples_per_edge').value
        )
        shuttle_count = int(self.get_parameter('shuttle_count').value)
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
        pose_topic = str(self.get_parameter('pose_topic').value)
        self.pose_topic_prefix = str(
            self.get_parameter('pose_topic_prefix').value
        ).rstrip('/')
        state_topic = str(self.get_parameter('state_topic').value)
        add_shuttle_command_topic = str(
            self.get_parameter('add_shuttle_command_topic').value
        )
        shuttle_control_command_topic = str(
            self.get_parameter('shuttle_control_command_topic').value
        )
        switch_command_topic = str(self.get_parameter('switch_command_topic').value)
        stopper_command_topic = str(self.get_parameter('stopper_command_topic').value)
        sensor_state_topic = str(self.get_parameter('sensor_state_topic').value)
        pose_offset_command_topic = str(
            self.get_parameter('pose_offset_command_topic').value
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
        self.shuttle_model_sdf = Path(str(self.get_parameter('shuttle_model_sdf').value))
        self.preloaded_shuttle_count = int(self.get_parameter('preloaded_shuttle_count').value)
        self.reject_occupied_start_slots = bool(
            self.get_parameter('reject_occupied_start_slots').value
        )
        self.start_slot_occupancy_radius_m = float(
            self.get_parameter('start_slot_occupancy_radius_m').value
        )
        self.gazebo_entity_name = str(self.get_parameter('gazebo_entity_name').value)
        gazebo_entity_names = str(self.get_parameter('gazebo_entity_names').value)
        gazebo_set_pose_rate_hz = float(self.get_parameter('gazebo_set_pose_rate_hz').value)
        self.gazebo_set_pose_period = 1.0 / max(gazebo_set_pose_rate_hz, 1.0)
        self.publish_visual_switch_commands = bool(
            self.get_parameter('publish_visual_switch_commands').value
        )
        self.enable_gazebo_pose_transform = bool(
            self.get_parameter('enable_gazebo_pose_transform').value
        )
        self.pose_transform_a = float(self.get_parameter('pose_transform_a').value)
        self.pose_transform_b = float(self.get_parameter('pose_transform_b').value)
        self.pose_transform_tx = float(self.get_parameter('pose_transform_tx').value)
        self.pose_transform_c = float(self.get_parameter('pose_transform_c').value)
        self.pose_transform_d = float(self.get_parameter('pose_transform_d').value)
        self.pose_transform_ty = float(self.get_parameter('pose_transform_ty').value)
        self.pose_transform_z_offset = float(
            self.get_parameter('pose_transform_z_offset').value
        )
        self.pose_transform_yaw_offset = float(
            self.get_parameter('pose_transform_yaw_offset').value
        )
        self.pose_scale_x = float(self.get_parameter('pose_scale_x').value)
        self.pose_scale_y = float(self.get_parameter('pose_scale_y').value)
        self.pose_scale_origin_x = float(self.get_parameter('pose_scale_origin_x').value)
        self.pose_scale_origin_y = float(self.get_parameter('pose_scale_origin_y').value)
        self.pose_offset_x = float(self.get_parameter('pose_offset_x').value)
        self.pose_offset_y = float(self.get_parameter('pose_offset_y').value)
        self.pose_offset_z = float(self.get_parameter('pose_offset_z').value)
        self.start_snap_tolerance_m = start_snap_tolerance_m
        self.default_shuttle_speed = speed
        self.spawn_warning_logged = False

        self.network = RailNetwork.from_yaml(
            network_path,
            path_backend=path_backend,
            arc_length_samples_per_edge=arc_length_samples_per_edge,
        )
        self.allowed_start_poses = self._load_allowed_start_poses()
        self.switch_states: Dict[str, str] = self.network.default_switch_states()
        self.stopper_configs = self._load_stopper_configs()
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
                    pose_topic_override=pose_topic if shuttle_index == 0 else None,
                )
            )

        self.core = self.shuttles[0].core
        self.start_slot = self.shuttles[0].start_slot
        self.start_pose = self.shuttles[0].start_pose
        self.start_snap_distance_m = self.shuttles[0].start_snap_distance_m

        self.state_publisher = self.create_publisher(String, state_topic, 10)
        self.sensor_state_publisher = self.create_publisher(String, sensor_state_topic, 10)
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

        for shuttle in self.shuttles:
            self._request_spawn_if_needed(shuttle)

        self.last_tick = self.get_clock().now()
        timer_period = 1.0 / max(update_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            'Room 315 kinematic shuttle started with '
            f'network={network_path}, path_backend={path_backend}, '
            f'pose_topic={pose_topic}, '
            f'gazebo_world={self.gazebo_world_name}, '
            f'add_shuttle_topic={add_shuttle_command_topic}, '
            f'shuttle_control_topic={shuttle_control_command_topic}, '
            f'switch_topic={switch_command_topic}, '
            f'stopper_topic={stopper_command_topic}, '
            f'sensor_topic={sensor_state_topic}, '
            f'offset_topic={pose_offset_command_topic}, '
            f'visual_switch_topic={visual_switch_command_topic}, '
            f'visual_switch_state_topic={visual_switch_state_topic}, '
            f'spawn_service={gazebo_spawn_service}, '
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
            raise ValueError('rail_network.yaml start_slots must not be empty.')
        return allowed

    def _load_stopper_configs(self) -> Dict[str, StopperConfig]:
        configs: Dict[str, StopperConfig] = {}
        raw_configs = self.network.config.get('stoppers', {}) or {}
        for raw_name, raw_config in raw_configs.items():
            internal_name = str(raw_name).strip().upper()
            name = _public_switch_name(internal_name)
            before_switch = _public_switch_name(
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
                else [f'room315_shuttle_{index}' for index in range(1, shuttle_count + 1)]
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
        return ', '.join(
            f'{shuttle.entity_name}:slot{shuttle.start_slot}:'
            f'{shuttle.core.state.current_segment}@{shuttle.core.state.s:.3f}:'
            f'snap={shuttle.start_snap_distance_m:.3f}m'
            for shuttle in self.shuttles
        )

    def _create_managed_shuttle(
        self,
        entity_name: str,
        slot,
        speed: float,
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
                    mode=MOVING,
                ),
            ),
            pose_publisher=self.create_publisher(PoseStamped, pose_topic, 10),
            last_gazebo_set_pose_time=self.get_clock().now(),
            gazebo_spawned=(
                not self.enable_gazebo_spawn
                or self._is_preloaded_shuttle_entity(entity_name)
            ),
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
        if any(shuttle.entity_name == entity_name for shuttle in self.shuttles):
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

    def _start_slot_occupancy_blocker(self, slot: str) -> tuple[str, float] | None:
        start_pose = self.allowed_start_poses[slot]
        blockers = []
        for shuttle in self.shuttles:
            pose = self._to_gazebo_pose(shuttle.core.pose())
            distance_m = math.hypot(start_pose.x - pose.x, start_pose.y - pose.y)
            if distance_m < self.start_slot_occupancy_radius_m:
                blockers.append((shuttle.entity_name, distance_m))
        if not blockers:
            return None
        return sorted(blockers, key=lambda item: item[1])[0]

    def _next_unused_entity_name(self) -> str:
        used_entities = {shuttle.entity_name for shuttle in self.shuttles}
        index = 1
        while True:
            entity_name = f'room315_shuttle_{index}'
            if entity_name not in used_entities:
                return entity_name
            index += 1

    def _request_spawn_if_needed(self, shuttle: ManagedShuttle) -> None:
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
        match = re.match(r'^room315_shuttle_(\d+)$', entity_name)
        return bool(match and int(match.group(1)) <= self.preloaded_shuttle_count)

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
        return (
            self.pose_scale_origin_x
            + (base_x - self.pose_scale_origin_x) * self.pose_scale_x
            + self.pose_offset_x,
            self.pose_scale_origin_y
            + (base_y - self.pose_scale_origin_y) * self.pose_scale_y
            + self.pose_offset_y,
            z + self.pose_transform_z_offset + self.pose_offset_z,
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
            f'offset_x={self.pose_offset_x:.4f}, '
            f'offset_y={self.pose_offset_y:.4f}, '
            f'offset_z={self.pose_offset_z:.4f}'
        )

    def _parse_pose_offset_command(self, raw_command: str) -> Dict[str, float]:
        command = raw_command.strip()
        if not command:
            raise ValueError('Empty pose offset command')

        if command.lower() in {'reset', 'zero', '0'}:
            return {
                'pose_scale_x': 1.0,
                'pose_scale_y': 1.0,
                'pose_offset_x': 0.0,
                'pose_offset_y': 0.0,
                'pose_offset_z': 0.0,
            }

        if command.startswith('{'):
            payload = json.loads(command)
            assignments = [(str(key), str(value)) for key, value in payload.items()]
        else:
            assignments = []
            for token in re.split(r'[\s,;]+', command.replace(':', '=')):
                if not token:
                    continue
                if '=' not in token:
                    raise ValueError(
                        f'Pose offset command must look like x=0.1 or dx=-0.01, got {token!r}'
                    )
                key, raw_value = token.split('=', 1)
                assignments.append((key, raw_value))

        next_x = self.pose_offset_x
        next_y = self.pose_offset_y
        next_z = self.pose_offset_z
        next_scale_x = self.pose_scale_x
        next_scale_y = self.pose_scale_y
        next_origin_x = self.pose_scale_origin_x
        next_origin_y = self.pose_scale_origin_y
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
            else:
                raise ValueError(
                    f'Unknown pose calibration key {raw_key!r}; use x/y/z for offsets, '
                    'dx/dy/dz for incremental offsets, sx/sy for scale, or dsx/dsy '
                    'for incremental scale.'
                )

        return {
            'pose_scale_x': next_scale_x,
            'pose_scale_y': next_scale_y,
            'pose_scale_origin_x': next_origin_x,
            'pose_scale_origin_y': next_origin_y,
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

        for entity_name, enabled in updates.items():
            shuttle = self._find_shuttle(entity_name)
            if shuttle is None:
                self.get_logger().error(
                    f'Unknown shuttle {entity_name!r}; command ignored.'
                )
                continue
            shuttle.enabled = enabled
            if not enabled:
                shuttle.core.state.mode = WAITING
                shuttle.stopped_by = 'DISABLED'
                shuttle.stopper_distance_m = 0.0
            else:
                if shuttle.stopped_by == 'DISABLED':
                    shuttle.stopped_by = None
                    shuttle.stopper_distance_m = None
                if shuttle.core.state.mode == WAITING and shuttle.core.state.speed > 0.0:
                    shuttle.core.state.mode = MOVING
        self.get_logger().info(
            'Updated shuttle enable states: '
            f'{ {shuttle.entity_name: shuttle.enabled for shuttle in self.shuttles} }'
        )

    def _parse_shuttle_control_command(self, raw_command: str) -> Dict[str, bool]:
        assignments = self._parse_assignments(raw_command, 'Shuttle control')
        keyed_payload = {key.strip().lower(): value for key, value in assignments}
        if {'entity', 'entity_name', 'name', 'shuttle'} & set(keyed_payload):
            entity_name = (
                keyed_payload.get('entity')
                or keyed_payload.get('entity_name')
                or keyed_payload.get('name')
                or keyed_payload.get('shuttle')
            )
            raw_state = (
                keyed_payload.get('enabled')
                or keyed_payload.get('state')
                or keyed_payload.get('mode')
                or keyed_payload.get('power')
            )
            if entity_name is None or raw_state is None:
                raise ValueError(
                    'Shuttle control command with entity=... must also include enabled=ON/OFF.'
                )
            return {entity_name: self._normalize_enabled_state(raw_state)}

        updates: Dict[str, bool] = {}
        for raw_selector, raw_state in assignments:
            selector = raw_selector.strip()
            enabled = self._normalize_enabled_state(raw_state)
            if selector.upper() == 'ALL':
                for shuttle in self.shuttles:
                    updates[shuttle.entity_name] = enabled
            else:
                updates[selector] = enabled
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
                f'Updated route switch states: {_public_switch_states(self.switch_states)}'
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
                f'{_public_switch_states(self.switch_states)}'
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
            # The calibrated shuttle path currently follows the droit/right rail set.
            if 'right' in states_by_side:
                updates[station] = states_by_side['right']
            elif 'station' in states_by_side:
                updates[station] = states_by_side['station']
            elif 'left' in states_by_side:
                updates[station] = states_by_side['left']
        return updates

    @staticmethod
    def _station_from_visual_switch_name(raw_name: str) -> tuple[str | None, str]:
        name = raw_name.strip().upper()
        if name in {'ALL', 'RIGHT', 'LEFT'}:
            return None, 'group'

        station_match = re.match(r'^(A[1-4])$', name)
        if station_match:
            station = station_match.group(1)
            return LOGICAL_SWITCH_SELECTOR_MAP.get(station, station), 'station'

        mapped_selector = RIGHT_VISUAL_SWITCH_SELECTOR_MAP.get(name)
        if mapped_selector is not None:
            return mapped_selector, 'right'

        if name in LEFT_VISUAL_SWITCH_SELECTORS:
            return None, 'left'

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
        if state in {'G', 'GRAND', 'GRAND_BOUCLE', 'BIG', 'LARGE'}:
            return 'G'
        if state in {'S', 'PETIT', 'PETIT_BOUCLE', 'SMALL'}:
            return 'S'
        return self.network.normalized_switch_state(state)

    def _logic_targets_for_selector(self, selector_name: str) -> list[str]:
        if selector_name in {'ALL', 'RIGHT'}:
            return sorted(self.network.switches)
        if selector_name == 'LEFT':
            return []

        station_match = re.match(r'^(A[1-4])$', selector_name)
        if station_match:
            station = station_match.group(1)
            return [LOGICAL_SWITCH_SELECTOR_MAP.get(station, station)]

        mapped_selector = RIGHT_VISUAL_SWITCH_SELECTOR_MAP.get(selector_name)
        if mapped_selector is not None:
            return [mapped_selector]

        if selector_name in LEFT_VISUAL_SWITCH_SELECTORS:
            return []

        return []

    def _visual_selector_for_selector(self, selector_name: str) -> str | None:
        if selector_name in {'ALL', 'RIGHT', 'LEFT'}:
            return selector_name

        station_match = re.match(r'^(A[1-4])$', selector_name)
        if station_match:
            return selector_name

        if selector_name in VISUAL_SWITCH_SELECTOR_MAP:
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

        raw_poses = []
        gazebo_poses = []
        occupied_poses = {
            shuttle.entity_name: self._to_gazebo_pose(shuttle.core.pose())
            for shuttle in self.shuttles
        }
        for shuttle in self.shuttles:
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
                    f'segment={pose.current_segment}, s={pose.s:.3f}'
                )

        self._publish_state(raw_poses, gazebo_poses)
        self._publish_sensor_state()

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
            + self.pose_offset_x
        )
        y = (
            self.pose_scale_origin_y
            + (base_y - self.pose_scale_origin_y) * self.pose_scale_y
            + self.pose_offset_y
        )

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
        last_sent = shuttle.last_gazebo_set_pose_time or now
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
            state = shuttle.core.state
            for stopper_name, stopper_config in self.stopper_configs.items():
                for stop_point in stopper_config.stop_points:
                    if stop_point.segment != state.current_segment:
                        continue
                    distance_m = stop_point.stop_s - state.s
                    if 0.0 <= distance_m <= stop_point.sensor_distance_m:
                        events.append(
                            {
                                'sensor': f'{stopper_name}_APPROACH',
                                'stopper': stopper_name,
                                'before_switch': stopper_config.before_switch,
                                'entity_name': shuttle.entity_name,
                                'segment': state.current_segment,
                                'distance_m': distance_m,
                                'stopper_state': self.stopper_states.get(stopper_name, '0'),
                                'workflow': 'sensor -> stop shuttle -> move switch -> unstop shuttle',
                            }
                        )
        return events

    def _publish_sensor_state(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                'sensors': self._sensor_events(),
                'stopper_states': self.stopper_states,
            },
            sort_keys=True,
        )
        self.sensor_state_publisher.publish(message)

    def _publish_state(
        self,
        raw_poses: list[ShuttlePose],
        gazebo_poses: list[ShuttlePose],
    ) -> None:
        shuttles_payload = []
        for shuttle, pose, gazebo_pose in zip(self.shuttles, raw_poses, gazebo_poses):
            shuttles_payload.append(
                {
                    **asdict(pose),
                    'entity_name': shuttle.entity_name,
                    'blocked_by': shuttle.blocked_by,
                    'collision_distance_m': shuttle.collision_distance_m,
                    'enabled': shuttle.enabled,
                    'stopped_by': shuttle.stopped_by,
                    'stopper_distance_m': shuttle.stopper_distance_m,
                    'gazebo_pose': asdict(gazebo_pose),
                    'start_slot': shuttle.start_slot,
                    'start_snap_distance_m': shuttle.start_snap_distance_m,
                    'speed': shuttle.core.state.speed,
                }
            )

        first_pose = raw_poses[0]
        first_gazebo_pose = gazebo_poses[0]
        message = String()
        message.data = json.dumps(
            {
                **asdict(first_pose),
                'entity_name': self.shuttles[0].entity_name,
                'gazebo_pose': asdict(first_gazebo_pose),
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
                'speed': self.core.state.speed,
                'start_slot': self.start_slot,
                'start_snap_distance_m': self.start_snap_distance_m,
                'shuttle_count': len(self.shuttles),
                'collision_avoidance': {
                    'enabled': self.enable_collision_avoidance,
                    'distance_m': self.shuttle_collision_distance_m,
                },
                'shuttles': shuttles_payload,
                'switch_states': _public_switch_states(self.switch_states),
                'stopper_states': self.stopper_states,
                'sensor_events': self._sensor_events(),
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
