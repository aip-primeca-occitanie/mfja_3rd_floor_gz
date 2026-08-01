#!/usr/bin/env python3

import json
import math
import re
import time
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

from mfja_rail_interfaces.msg import NamedState
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import SensorReading
from mfja_rail_interfaces.msg import ShuttleCommand as RailShuttleCommand
from mfja_rail_interfaces.msg import ShuttleState as RailShuttleState
from mfja_rail_interfaces.msg import StopperCommand
from mfja_rail_interfaces.msg import StopperState as RailStopperState
from mfja_rail_interfaces.msg import SwitchCommand
from mfja_rail_interfaces.msg import SwitchState as RailSwitchState
from mfja_rail_interfaces.srv import AddShuttle
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
from room_315_rail_defaults import (
    ALLOWED_START_POSES,
    DEVICE_MARKER_STYLES,
    LEFT_CALIBRATION_DEFAULTS,
    LEFT_ENTITY_DEFAULTS,
    LEFT_PUBLIC_SEGMENT_NAME_MAP,
    LEFT_TOPIC_DEFAULTS,
    LEFT_VISUAL_SWITCH_SELECTOR_MAP,
    MARKER_VISUAL_ACTIVE,
    MARKER_VISUAL_DEFAULT,
    MARKER_VISUAL_INACTIVE,
    PUBLIC_SWITCH_ORDER,
    RAIL_SENSOR_TYPE,
    RIGHT_CALIBRATION_DEFAULTS,
    RIGHT_ENTITY_DEFAULTS,
    RIGHT_TOPIC_DEFAULTS,
    RIGHT_VISUAL_SWITCH_SELECTOR_MAP,
    SHUTTLE_VISUAL_FALLING,
    SHUTTLE_VISUAL_NORMAL,
    SHUTTLE_VISUAL_REFRESH_RETRY_INTERVAL_S,
    STOPPER_PASS_STATE,
    STOPPER_STOP_STATE,
    SWITCH_EXTERIOR_STATE,
    SWITCH_INTERIOR_STATE,
    VISUAL_GROUP_SELECTOR_BY_SIDE,
    VISUAL_SELECTOR_SUFFIX_BY_SIDE,
    VISUAL_SWITCH_SELECTOR_MAP,
    VISUAL_SWITCH_SELECTOR_MAP_BY_SIDE,
    AllowedStartPose,
    apply_rail_point_calibration,
    apply_rail_pose_calibration,
)
from room_315_rail_devices import (
    PositionSensorConfig,
    PositionSensorPoint,
    RailDevice,
    RailDeviceSet,
    StopPoint,
    StopperConfig,
    canonical_segment_name as _canonical_segment_name,
    canonical_sensor_name as _canonical_sensor_name,
    canonical_switch_name as _canonical_switch_name,
    load_rail_devices,
    normalize_rail_side as _normalize_rail_side,
    ordered_switch_states as _ordered_switch_states,
)
from room_315_multi_shuttle import shuttle_specs_for_identities


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


def _default_right_devices_path() -> Path:
    return _default_network_path().with_name('rail_devices_right.yaml')


def _default_left_devices_path() -> Path:
    return _default_network_path().with_name('rail_devices_left.yaml')


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


def _default_payload_model_sdf_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_3rd_floor_description'))
            / 'models'
            / 'room315_vla_payload_small_box'
            / 'model.sdf'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[2]
            / 'mfja_3rd_floor_description'
            / 'models'
            / 'room315_vla_payload_small_box'
            / 'model.sdf'
        )


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


@dataclass
class ManagedShuttle:
    entity_name: str
    start_slot: str
    start_pose: AllowedStartPose
    start_snap_distance_m: float
    start_segment: str
    start_s: float
    core: KinematicShuttleCore
    pose_publisher: object
    previous_sensor_segment: str | None = None
    previous_sensor_s: float | None = None
    pending_set_pose: object | None = None
    pending_set_pose_wall_time: float | None = None
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
    set_pose_timeout_warning_logged: bool = False
    spawn_failure_logged: bool = False
    visual_state: str = SHUTTLE_VISUAL_NORMAL
    spawned_visual_state: str | None = None
    visual_refresh_in_progress: bool = False
    pending_visual_delete: object | None = None
    pending_visual_spawn: object | None = None
    pending_visual_spawn_state: str | None = None
    next_visual_refresh_time: float = 0.0
    visual_refresh_failure_logged: bool = False
    sensor_markers_armed: bool = False
    payload_loaded: bool = False
    payload_type: str = 'none'
    payload_entity_name: str = ''
    payload_gazebo_spawned: bool = False
    pending_payload_spawn: object | None = None
    pending_payload_delete: object | None = None
    pending_payload_set_pose: object | None = None
    pending_payload_set_pose_wall_time: float | None = None
    last_payload_gazebo_set_pose_time: object | None = None
    payload_spawn_failure_logged: bool = False
    payload_delete_warning_logged: bool = False
    payload_set_pose_warning_logged: bool = False
    payload_set_pose_timeout_warning_logged: bool = False
    motion_target_slot: str = ''
    motion_target_segment: str = ''
    motion_target_s: float | None = None
    reached_target_slot: str = ''


@dataclass
class DeviceMarker:
    entity_name: str
    device_type: str
    device_name: str
    segment: str
    pose: ShuttlePose
    sdf: str
    sensor_s: float | None = None
    sensor_radius_m: float | None = None
    active_overlay: bool = False
    visible_pose: ShuttlePose | None = None
    hidden_pose: ShuttlePose | None = None
    desired_visible: bool = True
    visible: bool = True
    visual_state: str = MARKER_VISUAL_DEFAULT
    pending_spawn_visual_state: str | None = None
    spawned_visual_state: str | None = None
    pending_spawn: object | None = None
    pending_delete: object | None = None
    pending_pose: object | None = None
    pending_visible: bool | None = None
    spawned: bool = False
    spawn_failure_logged: bool = False
    spawn_attempts: int = 0
    next_spawn_attempt_time: float = 0.0
    last_spawn_success_time: float | None = None
    visual_hold_until_s: float = 0.0


@dataclass
class PendingDiscreteStateUpdate:
    target_state: str
    apply_at_s: float
    source: str


class Room315KinematicShuttleNode(Node):
    def __init__(self) -> None:
        super().__init__('room_315_kinematic_shuttle')

        self.declare_parameter('network_yaml', str(_default_network_path()))
        self.declare_parameter('devices_yaml', '')
        self.declare_parameter('rail_side', 'right')
        self.declare_parameter('path_backend', CUBIC_HERMITE_PATH_BACKEND)
        self.declare_parameter('arc_length_samples_per_edge', 16)
        self.declare_parameter('shuttle_count', 1)
        self.declare_parameter('identity_selection_mode', 'count_prefix')
        self.declare_parameter('active_identities', '')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('start_slot', 2)
        self.declare_parameter('start_slots', '')
        self.declare_parameter('start_positions', '')
        self.declare_parameter('start_snap_tolerance_m', 0.25)
        self.declare_parameter('initial_segment', 'A23')
        self.declare_parameter('initial_s', 0.0)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('falling_stop_offset_m', 0.0)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('sensor_publish_rate_hz', 10.0)
        self.declare_parameter('sync_sensor_feedback_to_motion_tick', True)
        self.declare_parameter('enable_collision_avoidance', True)
        self.declare_parameter('shuttle_collision_distance_m', 0.33)
        self.declare_parameter('collision_search_iterations', 12)
        self.declare_parameter('pose_topic', RIGHT_TOPIC_DEFAULTS['pose_topic'])
        self.declare_parameter('pose_topic_prefix', RIGHT_TOPIC_DEFAULTS['pose_topic_prefix'])
        self.declare_parameter('shuttle_state_topic', RIGHT_TOPIC_DEFAULTS['shuttle_state_topic'])
        self.declare_parameter(
            'add_shuttle_service',
            RIGHT_TOPIC_DEFAULTS['add_shuttle_service'],
        )
        self.declare_parameter(
            'shuttle_control_command_topic',
            RIGHT_TOPIC_DEFAULTS['shuttle_control_command_topic'],
        )
        self.declare_parameter(
            'switch_command_topic',
            RIGHT_TOPIC_DEFAULTS['switch_command_topic'],
        )
        self.declare_parameter('switch_state_topic', RIGHT_TOPIC_DEFAULTS['switch_state_topic'])
        self.declare_parameter(
            'stopper_command_topic',
            RIGHT_TOPIC_DEFAULTS['stopper_command_topic'],
        )
        self.declare_parameter('stopper_state_topic', RIGHT_TOPIC_DEFAULTS['stopper_state_topic'])
        self.declare_parameter(
            'sensor_feedback_topic',
            RIGHT_TOPIC_DEFAULTS['sensor_feedback_topic'],
        )
        self.declare_parameter(
            'pose_offset_command_topic',
            RIGHT_TOPIC_DEFAULTS['pose_offset_command_topic'],
        )
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
        self.declare_parameter('show_device_markers', True)
        self.declare_parameter('enable_device_markers', True)
        self.declare_parameter('device_marker_scale', 1.0)
        self.declare_parameter('device_marker_z_offset_m', 0.0)
        self.declare_parameter('device_marker_spawn_interval_s', 0.05)
        self.declare_parameter('device_marker_retry_interval_s', 0.5)
        self.declare_parameter('device_marker_dynamic_refresh', False)
        self.declare_parameter('device_marker_refresh_grace_period_s', 0.5)
        self.declare_parameter('device_marker_max_spawn_attempts', 8)
        self.declare_parameter('sensor_marker_visual_hold_s', 0.35)
        self.declare_parameter('visual_debug_colors', True)
        self.declare_parameter('shuttle_model_sdf', str(_default_shuttle_model_sdf_path()))
        self.declare_parameter('enable_payload_visuals', True)
        self.declare_parameter('payload_model_sdf', str(_default_payload_model_sdf_path()))
        self.declare_parameter('payload_type', 'box')
        self.declare_parameter('loaded_shuttles', '')
        self.declare_parameter('payload_pose_x_offset_m', -0.08)
        self.declare_parameter('payload_pose_z_offset_m', 0.0)
        self.declare_parameter('payload_state_topic', RIGHT_TOPIC_DEFAULTS['payload_state_topic'])
        self.declare_parameter('payload_command_topic', RIGHT_TOPIC_DEFAULTS['payload_command_topic'])
        self.declare_parameter('preloaded_shuttle_count', 4)
        self.declare_parameter('reject_occupied_start_slots', True)
        self.declare_parameter('start_slot_occupancy_radius_m', 0.33)
        self.declare_parameter('gazebo_entity_name', 'room315_right_shuttle_1')
        self.declare_parameter('gazebo_entity_names', '')
        self.declare_parameter('entity_name_prefix', 'room315_right_shuttle_')
        self.declare_parameter('gazebo_set_pose_rate_hz', 30.0)
        self.declare_parameter('gazebo_set_pose_timeout_s', 0.5)
        self.declare_parameter('publish_visual_switch_commands', True)
        self.declare_parameter('switch_motion_delay_s', 0.3)
        self.declare_parameter('stopper_motion_delay_s', 0.1)
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
        devices_path = self._devices_path(
            str(self.get_parameter('devices_yaml').value),
            network_path,
        )
        path_backend = str(self.get_parameter('path_backend').value)
        arc_length_samples_per_edge = int(
            self.get_parameter('arc_length_samples_per_edge').value
        )
        shuttle_count = int(self.get_parameter('shuttle_count').value)
        identity_selection_mode = str(
            self.get_parameter('identity_selection_mode').value
        )
        active_identities = str(
            self.get_parameter('active_identities').value
        )
        start_enabled = bool(self.get_parameter('start_enabled').value)
        start_slot = self.get_parameter('start_slot').value
        start_slots = str(self.get_parameter('start_slots').value)
        start_positions = str(self.get_parameter('start_positions').value)
        start_snap_tolerance_m = float(
            self.get_parameter('start_snap_tolerance_m').value
        )
        initial_segment = str(self.get_parameter('initial_segment').value)
        initial_s = float(self.get_parameter('initial_s').value)
        speed = float(self.get_parameter('speed').value)
        self.falling_stop_offset_m = max(
            0.0,
            float(self.get_parameter('falling_stop_offset_m').value),
        )
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        sensor_publish_rate_hz = float(self.get_parameter('sensor_publish_rate_hz').value)
        if sensor_publish_rate_hz <= 0.0:
            raise ValueError('sensor_publish_rate_hz must be greater than 0.0.')
        self.sensor_publish_rate_hz = sensor_publish_rate_hz
        self.sync_sensor_feedback_to_motion_tick = bool(
            self.get_parameter('sync_sensor_feedback_to_motion_tick').value
        )
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
        shuttle_state_topic = self._side_default_string(
            str(self.get_parameter('shuttle_state_topic').value),
            'shuttle_state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        add_shuttle_service = self._side_default_string(
            str(self.get_parameter('add_shuttle_service').value),
            'add_shuttle_service',
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
        switch_state_topic = self._side_default_string(
            str(self.get_parameter('switch_state_topic').value),
            'switch_state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        stopper_command_topic = self._side_default_string(
            str(self.get_parameter('stopper_command_topic').value),
            'stopper_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        stopper_state_topic = self._side_default_string(
            str(self.get_parameter('stopper_state_topic').value),
            'stopper_state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        sensor_feedback_topic = self._side_default_string(
            str(self.get_parameter('sensor_feedback_topic').value),
            'sensor_feedback_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        pose_offset_command_topic = self._side_default_string(
            str(self.get_parameter('pose_offset_command_topic').value),
            'pose_offset_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        self._ensure_command_state_topics_are_distinct(
            'switch',
            switch_command_topic,
            switch_state_topic,
        )
        self._ensure_command_state_topics_are_distinct(
            'stopper',
            stopper_command_topic,
            stopper_state_topic,
        )
        self._ensure_command_state_topics_are_distinct(
            'shuttle',
            shuttle_control_command_topic,
            shuttle_state_topic,
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
        self.enable_device_markers = (
            bool(self.get_parameter('enable_device_markers').value)
            and bool(self.get_parameter('show_device_markers').value)
        )
        self.device_marker_scale = max(
            0.05,
            float(self.get_parameter('device_marker_scale').value),
        )
        self.device_marker_z_offset_m = float(
            self.get_parameter('device_marker_z_offset_m').value
        )
        self.device_marker_spawn_interval_s = max(
            0.0,
            float(self.get_parameter('device_marker_spawn_interval_s').value),
        )
        self.device_marker_retry_interval_s = max(
            0.05,
            float(self.get_parameter('device_marker_retry_interval_s').value),
        )
        self.device_marker_dynamic_refresh = bool(
            self.get_parameter('device_marker_dynamic_refresh').value
        )
        self.device_marker_refresh_grace_period_s = max(
            0.0,
            float(self.get_parameter('device_marker_refresh_grace_period_s').value),
        )
        self.device_marker_max_spawn_attempts = max(
            0,
            int(self.get_parameter('device_marker_max_spawn_attempts').value),
        )
        self.sensor_marker_visual_hold_s = max(
            0.0,
            float(self.get_parameter('sensor_marker_visual_hold_s').value),
        )
        self.visual_debug_colors = bool(self.get_parameter('visual_debug_colors').value)
        self.shuttle_model_sdf = Path(str(self.get_parameter('shuttle_model_sdf').value))
        self.enable_payload_visuals = bool(self.get_parameter('enable_payload_visuals').value)
        self.payload_model_sdf = Path(str(self.get_parameter('payload_model_sdf').value))
        self.payload_type = (
            str(self.get_parameter('payload_type').value).strip() or 'box'
        )
        self.initial_loaded_shuttle_refs = {
            token.casefold()
            for token in self._split_list_parameter(
                str(self.get_parameter('loaded_shuttles').value)
            )
        }
        self.payload_pose_x_offset_m = float(
            self.get_parameter('payload_pose_x_offset_m').value
        )
        self.payload_pose_z_offset_m = float(
            self.get_parameter('payload_pose_z_offset_m').value
        )
        payload_state_topic = self._side_default_string(
            str(self.get_parameter('payload_state_topic').value),
            'payload_state_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
        payload_command_topic = self._side_default_string(
            str(self.get_parameter('payload_command_topic').value),
            'payload_command_topic',
            right_defaults=RIGHT_TOPIC_DEFAULTS,
            left_defaults=LEFT_TOPIC_DEFAULTS,
        )
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
        self.gazebo_set_pose_timeout_s = max(
            self.gazebo_set_pose_period,
            float(self.get_parameter('gazebo_set_pose_timeout_s').value),
        )
        self.publish_visual_switch_commands = bool(
            self.get_parameter('publish_visual_switch_commands').value
        )
        self.switch_motion_delay_s = max(
            0.0,
            float(self.get_parameter('switch_motion_delay_s').value),
        )
        self.stopper_motion_delay_s = max(
            0.0,
            float(self.get_parameter('stopper_motion_delay_s').value),
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
        self.device_marker_spawn_warning_logged = False
        self.next_device_marker_spawn_time = 0.0
        self.deleted_preloaded_entity_names: set[str] = set()
        self.deleting_entity_names: set[str] = set()

        self.network = RailNetwork.from_yaml(
            network_path,
            path_backend=path_backend,
            arc_length_samples_per_edge=arc_length_samples_per_edge,
        )
        self.rail_devices = load_rail_devices(devices_path, self.network)
        self.switch_states: Dict[str, str] = self.network.default_switch_states()
        self.allowed_start_poses = self._load_allowed_start_poses()
        self.stopper_configs = self._load_stopper_configs()
        self.position_sensor_configs = self._load_position_sensor_configs()
        self.stopper_states: Dict[str, str] = {
            name: config.default_state
            for name, config in self.stopper_configs.items()
        }
        self.device_markers = (
            self._make_device_markers()
            if self.enable_device_markers
            else []
        )
        self.shuttle_state_topic = shuttle_state_topic
        self.add_shuttle_service = add_shuttle_service
        self.shuttle_control_command_topic = shuttle_control_command_topic
        self.payload_state_topic = payload_state_topic
        self.payload_command_topic = payload_command_topic
        self.switch_command_topic = switch_command_topic
        self.switch_state_topic = switch_state_topic
        self.stopper_command_topic = stopper_command_topic
        self.stopper_state_topic = stopper_state_topic
        self.pose_offset_command_topic = pose_offset_command_topic
        self.pending_switch_state_updates: Dict[str, PendingDiscreteStateUpdate] = {}
        self.pending_stopper_state_updates: Dict[str, PendingDiscreteStateUpdate] = {}
        shuttle_specs = self._resolve_shuttle_specs(
            shuttle_count=shuttle_count,
            identity_selection_mode=identity_selection_mode,
            raw_active_identities=active_identities,
            raw_start_slot=start_slot,
            raw_start_slots=start_slots,
            default_entity_name=self.gazebo_entity_name,
            raw_entity_names=gazebo_entity_names,
        )
        position_overrides = self._resolve_start_position_overrides(
            start_positions,
            shuttle_count=len(shuttle_specs),
        )
        self.shuttles: list[ManagedShuttle] = []
        for shuttle_index, (entity_name, slot) in enumerate(shuttle_specs):
            self.shuttles.append(
                self._create_managed_shuttle(
                    entity_name=entity_name,
                    slot=slot,
                    speed=speed,
                    enabled=start_enabled,
                    deployed=True,
                    start_position=position_overrides[shuttle_index],
                    pose_topic_override=pose_topic if shuttle_index == 0 else None,
                )
            )

        self.state_publisher = self.create_publisher(
            RailShuttleState,
            shuttle_state_topic,
            10,
        )
        self.switch_state_publisher = self.create_publisher(
            RailSwitchState,
            switch_state_topic,
            10,
        )
        self.stopper_state_publisher = self.create_publisher(
            RailStopperState,
            stopper_state_topic,
            10,
        )
        self.sensor_feedback_publisher = self.create_publisher(
            SensorFeedback,
            sensor_feedback_topic,
            10,
        )
        self.visual_switch_publisher = self.create_publisher(
            String,
            visual_switch_command_topic,
            10,
        )
        self.payload_state_publisher = self.create_publisher(
            String,
            payload_state_topic,
            10,
        )
        self.switch_subscription = self.create_subscription(
            SwitchCommand,
            switch_command_topic,
            self._on_switch_command,
            10,
        )
        self.stopper_subscription = self.create_subscription(
            StopperCommand,
            stopper_command_topic,
            self._on_stopper_command,
            10,
        )
        self.add_shuttle_service_server = self.create_service(
            AddShuttle,
            add_shuttle_service,
            self._on_add_shuttle_request,
        )
        self.shuttle_control_subscription = self.create_subscription(
            RailShuttleCommand,
            shuttle_control_command_topic,
            self._on_shuttle_control_command,
            10,
        )
        self.payload_command_subscription = self.create_subscription(
            String,
            payload_command_topic,
            self._on_payload_command,
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
        if self.enable_gazebo_spawn or self.enable_device_markers:
            self.spawn_client = self.create_client(SpawnEntity, gazebo_spawn_service)
        self.delete_client = None
        if self.enable_gazebo_delete:
            self.delete_client = self.create_client(DeleteEntity, gazebo_delete_service)

        self._update_device_markers()

        for shuttle in self.shuttles:
            self._request_spawn_if_needed(shuttle)

        self.last_tick = self.get_clock().now()
        timer_period = 1.0 / max(update_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)
        self.sensor_timer = None
        self._reset_sensor_timer()
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            'Room 315 kinematic shuttle started with '
            f'rail_side={self.rail_side}, network={network_path}, path_backend={path_backend}, '
            f'devices={self.rail_devices.path}, '
            f'pose_topic={pose_topic}, '
            f'gazebo_world={self.gazebo_world_name}, '
            f'add_shuttle_service={add_shuttle_service}, '
            f'shuttle_control_topic={shuttle_control_command_topic}, '
            f'shuttle_state_topic={shuttle_state_topic}, '
            f'payload_state_topic={payload_state_topic}, '
            f'payload_command_topic={payload_command_topic}, '
            f'switch_command_topic={switch_command_topic}, '
            f'switch_state_topic={switch_state_topic}, '
            f'stopper_command_topic={stopper_command_topic}, '
            f'stopper_state_topic={stopper_state_topic}, '
            f'sensor_feedback_topic={sensor_feedback_topic}, '
            f'offset_topic={pose_offset_command_topic}, '
            f'visual_switch_topic={visual_switch_command_topic}, '
            f'visual_switch_state_topic={visual_switch_state_topic}, '
            f'switch_motion_delay_s={self.switch_motion_delay_s:.3f}, '
            f'stopper_motion_delay_s={self.stopper_motion_delay_s:.3f}, '
            f'sensor_publish_rate_hz={sensor_publish_rate_hz:.3f}, '
            f'sync_sensor_feedback_to_motion_tick={self.sync_sensor_feedback_to_motion_tick}, '
            f'entity_prefix={self.entity_name_prefix}, '
            f'spawn_service={gazebo_spawn_service}, '
            f'delete_service={gazebo_delete_service}, '
            f'payload_visuals={self.enable_payload_visuals}, '
            f'loaded_shuttles={sorted(self.initial_loaded_shuttle_refs)}, '
            f'device_markers={len(self.device_markers)}, '
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

    @staticmethod
    def _ensure_command_state_topics_are_distinct(
        device_name: str,
        command_topic: str,
        state_topic: str,
    ) -> None:
        if command_topic == state_topic:
            raise ValueError(
                f'{device_name} command topic and state topic must be different '
                f'in Phase 3, got {command_topic!r}.'
            )

    def _side_default_path(self, configured_path: Path) -> Path:
        if self.rail_side != 'left':
            return configured_path

        right_default = _default_network_path().resolve()
        if configured_path.resolve() == right_default:
            return _default_left_network_path()
        return configured_path

    def _devices_path(self, raw_value: str, network_path: Path) -> Path:
        configured_value = raw_value.strip()
        if not configured_value:
            return (
                _default_left_devices_path()
                if self.rail_side == 'left'
                else _default_right_devices_path()
            )

        configured_path = Path(configured_value)
        if configured_path.is_absolute():
            return configured_path
        return network_path.parent / configured_path

    def _side_default(
        self,
        configured_value,
        key: str,
        *,
        right_defaults: Dict[str, object],
        left_defaults: Dict[str, object],
    ):
        if self.rail_side == 'left' and configured_value == right_defaults[key]:
            return left_defaults[key]
        return configured_value

    _side_default_string = _side_default
    _side_default_numeric = _side_default

    def _load_allowed_start_poses(self) -> Dict[str, AllowedStartPose]:
        if self.rail_devices.slots:
            allowed: Dict[str, AllowedStartPose] = {}
            for slot, device in self.rail_devices.slots.items():
                gazebo_pose = self._to_gazebo_pose(
                    ShuttlePose(
                        x=device.x,
                        y=device.y,
                        z=device.z,
                        yaw=device.yaw,
                        current_segment=device.segment,
                        s=device.s,
                        mode=WAITING,
                    )
                )
                allowed[slot] = AllowedStartPose(
                    x=gazebo_pose.x,
                    y=gazebo_pose.y,
                    z=gazebo_pose.z,
                    roll=0.0,
                    pitch=0.0,
                    yaw=gazebo_pose.yaw,
                )
            return allowed

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
            raise ValueError('rail device slots must not be empty.')
        return allowed

    def _load_stopper_configs(self) -> Dict[str, StopperConfig]:
        return self._load_stopper_configs_from_devices()

    def _linked_position_sensor_lookup(self) -> Dict[str, tuple[str, tuple[RailDevice, ...]]]:
        sensors_by_stopper: Dict[str, tuple[str, tuple[RailDevice, ...]]] = {}
        for sensor_name, sensor_devices in self.rail_devices.position_sensors.items():
            if not sensor_devices:
                continue
            metadata = sensor_devices[0].metadata or {}
            raw_stopper = metadata.get('stopper')
            if raw_stopper is None:
                continue
            stopper_name = _canonical_switch_name(str(raw_stopper))
            if not stopper_name:
                raise ValueError(
                    f'position_sensors.{sensor_name}.stopper must name a stopper.'
                )
            if stopper_name in sensors_by_stopper:
                raise ValueError(
                    f'Multiple position sensors are linked to stopper {stopper_name}; '
                    'use one stopper-linked position_sensors entry per stopper.'
                )
            for device in sensor_devices:
                device_stopper = _canonical_switch_name(
                    str((device.metadata or {}).get('stopper', raw_stopper))
                )
                if device_stopper != stopper_name:
                    raise ValueError(
                        f'position_sensors.{sensor_name} points must link to the '
                        'same stopper.'
                    )
            sensors_by_stopper[stopper_name] = (sensor_name, sensor_devices)
        return sensors_by_stopper

    def _load_stopper_configs_from_devices(self) -> Dict[str, StopperConfig]:
        configs: Dict[str, StopperConfig] = {}
        sensors_by_stopper = self._linked_position_sensor_lookup()
        for stopper_name, stopper_devices in self.rail_devices.stoppers.items():
            if not stopper_devices:
                raise ValueError(f'Stopper {stopper_name} must define at least one point.')

            first_device = stopper_devices[0]
            metadata = first_device.metadata or {}
            before_switch = _canonical_switch_name(
                str(metadata.get('before_switch', stopper_name))
            )
            default_state = self._normalize_stopper_state(
                str(first_device.default_state)
            )
            linked_sensor = sensors_by_stopper.get(stopper_name)
            if linked_sensor is None:
                raise ValueError(
                    f'Stopper {stopper_name} must have a matching position_sensors '
                    'entry with stopper and before_stopper_m.'
                )
            sensor_name, sensor_devices = linked_sensor
            if len(sensor_devices) != len(stopper_devices):
                raise ValueError(
                    f'position_sensors.{sensor_name} has {len(sensor_devices)} '
                    f'point(s), but stoppers.{stopper_name} has '
                    f'{len(stopper_devices)} point(s).'
                )
            stop_points: list[StopPoint] = []
            for index, stopper_device in enumerate(stopper_devices):
                sensor_device = sensor_devices[index]
                if sensor_device.segment != stopper_device.segment:
                    raise ValueError(
                        f'position_sensors.{sensor_name} point {index} must be on '
                        f'the same segment as stoppers.{stopper_name} point {index}.'
                    )
                if sensor_device.s > stopper_device.s + 1e-6:
                    raise ValueError(
                        f'position_sensors.{sensor_name} point {index} must be before '
                        f'stoppers.{stopper_name} point {index}.'
                    )
                stop_points.append(
                    StopPoint(
                        segment=stopper_device.segment,
                        stop_s=stopper_device.s,
                        trigger_s=sensor_device.s,
                    )
                )

            configs[stopper_name] = StopperConfig(
                name=stopper_name,
                before_switch=before_switch,
                default_state=default_state,
                stop_points=tuple(stop_points),
            )
        unused_stoppers = sorted(
            set(sensors_by_stopper) - set(self.rail_devices.stoppers)
        )
        if unused_stoppers:
            raise ValueError(
                'position_sensors entries reference unknown stopper(s): '
                f'{unused_stoppers}.'
            )
        return configs

    @staticmethod
    def _normalize_position_sensor_branch(
        raw_branch: str | None,
    ) -> tuple[str | None, str | None]:
        if raw_branch is None:
            return None, None

        branch = str(raw_branch).strip().upper()
        if branch in {'E', 'EXTERIOR'}:
            return SWITCH_EXTERIOR_STATE, 'EXTERIOR'
        if branch in {'I', 'INTERIOR'}:
            return SWITCH_INTERIOR_STATE, 'INTERIOR'
        raise ValueError(
            f'Unknown position sensor branch {raw_branch!r}; use E/I or EXTERIOR/INTERIOR.'
        )

    def _validate_position_sensor_metadata(
        self,
        sensor_name: str,
        metadata: dict,
        *,
        start_slot,
    ) -> None:
        """Validate descriptive YAML labels without making them drive feedback."""
        self._normalize_position_sensor_branch(metadata.get('branch'))

        switch_name = metadata.get('switch')
        if switch_name is not None:
            _canonical_switch_name(str(switch_name).strip().upper())

        index_zone = metadata.get('index_zone')
        if index_zone is not None:
            str(index_zone).strip()

        linked_stopper = metadata.get('stopper')
        if linked_stopper is not None:
            _canonical_switch_name(str(linked_stopper))

        before_stopper_m = metadata.get('before_stopper_m')
        if before_stopper_m is not None and float(before_stopper_m) < 0.0:
            raise ValueError(
                f'Position sensor {sensor_name} before_stopper_m must be greater '
                'than or equal to 0.0.'
            )

        if start_slot is not None:
            self._normalize_start_slot(start_slot)

        for alias in metadata.get('aliases', []):
            public_alias = _canonical_sensor_name(alias)
            if public_alias != sensor_name:
                str(public_alias).strip()

    def _position_sensor_points_from_config(
        self,
        sensor_name: str,
        raw_config: dict,
    ) -> tuple[PositionSensorPoint, ...]:
        raw_points = raw_config.get('points')
        if raw_points is None:
            raw_points = [raw_config]

        points: list[PositionSensorPoint] = []
        for raw_point in raw_points:
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

            radius_value = raw_point.get('radius_m', raw_config.get('radius_m'))
            if radius_value is None:
                raise ValueError(f'Position sensor {sensor_name} must define radius_m.')
            radius_m = float(radius_value)
            if radius_m < 0.0:
                raise ValueError(
                    f'Position sensor {sensor_name} radius_m must be greater '
                    'than or equal to 0.0.'
                )
            points.append(
                PositionSensorPoint(
                    segment=segment_name,
                    sensor_s=sensor_s,
                    radius_m=radius_m,
                )
            )

        if not points:
            raise ValueError(f'Position sensor {sensor_name} must define at least one point.')
        return tuple(points)

    def _load_position_sensor_configs(self) -> Dict[str, PositionSensorConfig]:
        if self.rail_devices.position_sensors:
            return self._load_position_sensor_configs_from_devices()

        configs: Dict[str, PositionSensorConfig] = {}
        raw_configs = self.network.config.get('position_sensors', {}) or {}
        for raw_name, raw_config in raw_configs.items():
            if not isinstance(raw_config, dict):
                raise ValueError(
                    f'position_sensors.{raw_name} must be a mapping, got {type(raw_config)!r}.'
                )

            internal_name = str(raw_name).strip().upper()
            name = _canonical_sensor_name(internal_name)
            self._validate_position_sensor_metadata(
                name,
                raw_config,
                start_slot=raw_config.get('slot'),
            )
            configs[name] = PositionSensorConfig(
                name=name,
                points=self._position_sensor_points_from_config(name, raw_config),
            )
        return configs

    def _load_position_sensor_configs_from_devices(self) -> Dict[str, PositionSensorConfig]:
        configs: Dict[str, PositionSensorConfig] = {}
        for sensor_name, sensor_devices in self.rail_devices.position_sensors.items():
            if not sensor_devices:
                raise ValueError(
                    f'Position sensor {sensor_name} must define at least one point.'
                )

            first_device = sensor_devices[0]
            metadata = first_device.metadata or {}
            self._validate_position_sensor_metadata(
                sensor_name,
                metadata,
                start_slot=metadata.get('start_slot', metadata.get('slot')),
            )

            points = tuple(
                PositionSensorPoint(
                    segment=device.segment,
                    sensor_s=device.s,
                    radius_m=device.radius_m,
                )
                for device in sensor_devices
            )
            configs[sensor_name] = PositionSensorConfig(
                name=sensor_name,
                points=points,
            )
        return configs

    def _resolve_shuttle_specs(
        self,
        shuttle_count: int,
        identity_selection_mode: str,
        raw_active_identities: str,
        raw_start_slot,
        raw_start_slots: str,
        default_entity_name: str,
        raw_entity_names: str,
    ) -> list[tuple[str, str]]:
        if shuttle_count < 0:
            raise ValueError('shuttle_count must be greater than or equal to 0.')
        if shuttle_count > 4:
            raise ValueError('Room 315 supports at most 4 shuttles per rail side.')
        identity_selection_mode = identity_selection_mode.strip().casefold()
        if identity_selection_mode not in {'count_prefix', 'explicit'}:
            raise ValueError(
                'identity_selection_mode must be count_prefix or explicit.'
            )

        explicit_entity_names: list[str] | None = None
        if identity_selection_mode == 'explicit':
            identity_specs = shuttle_specs_for_identities(
                self.rail_side,
                raw_active_identities,
            )
            if len(identity_specs) != shuttle_count:
                raise ValueError(
                    f'explicit {self.rail_side} active_identities resolves to '
                    f'{len(identity_specs)} shuttle(s), but shuttle_count='
                    f'{shuttle_count}.'
                )
            explicit_entity_names = [
                spec.gazebo_entity_name
                for spec in identity_specs
            ]
        elif str(raw_active_identities or '').strip():
            raise ValueError(
                'active_identities was supplied while identity_selection_mode '
                'is count_prefix; set identity_selection_mode=explicit.'
            )
        if shuttle_count == 0:
            return []

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
        if explicit_entity_names is not None:
            if entity_names and entity_names != explicit_entity_names:
                raise ValueError(
                    'gazebo_entity_names does not exactly match the entities '
                    'resolved from active_identities.'
                )
            entity_names = explicit_entity_names
        elif not entity_names:
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

    def _resolve_start_position_overrides(
        self,
        raw_positions: str,
        *,
        shuttle_count: int,
    ) -> list[tuple[str, float] | None]:
        tokens = self._split_list_parameter(raw_positions)
        if not tokens:
            return [None] * shuttle_count
        if len(tokens) != shuttle_count:
            raise ValueError(
                f'shuttle_count={shuttle_count} but start_positions has '
                f'{len(tokens)} value(s): {tokens}.'
            )
        resolved: list[tuple[str, float] | None] = []
        for token in tokens:
            if '@' not in token:
                raise ValueError(
                    f'Invalid start position {token!r}; use SEGMENT@S_RATIO.'
                )
            requested_segment, ratio_text = token.split('@', 1)
            requested_segment = requested_segment.strip().upper()
            segment_name = (
                LEFT_PUBLIC_SEGMENT_NAME_MAP.get(
                    requested_segment,
                    requested_segment,
                )
                if self.rail_side == 'left'
                else requested_segment
            )
            if segment_name not in self.network.segments:
                raise ValueError(
                    f'Unknown start position segment {requested_segment!r}.'
                )
            try:
                s_ratio = float(ratio_text)
            except ValueError as exc:
                raise ValueError(
                    f'Invalid start position ratio in {token!r}.'
                ) from exc
            if not 0.0 <= s_ratio <= 1.0:
                raise ValueError(
                    f'Start position ratio must be in [0, 1]: {token!r}.'
                )
            resolved.append((
                segment_name,
                s_ratio * self.network.segments[segment_name].length,
            ))
        return resolved

    def _create_managed_shuttle(
        self,
        entity_name: str,
        slot,
        speed: float,
        enabled: bool = True,
        deployed: bool = True,
        payload_loaded: bool | None = None,
        start_position: tuple[str, float] | None = None,
        pose_topic_override: str | None = None,
    ) -> ManagedShuttle:
        (
            resolved_slot,
            start_pose,
            start_snap_distance_m,
            initial_segment,
            initial_s,
        ) = self._resolve_allowed_start_slot(slot, self.start_snap_tolerance_m)
        if start_position is not None:
            initial_segment, initial_s = start_position
            segment = self.network.segments.get(initial_segment)
            if segment is None:
                raise ValueError(f'Unknown initial segment: {initial_segment!r}')
            initial_s = max(0.0, min(float(initial_s), segment.length))
            start_snap_distance_m = 0.0
        pose_topic = (
            pose_topic_override
            if pose_topic_override is not None
            else f'{self.pose_topic_prefix}/{self._topic_safe_name(entity_name)}/pose_cmd'
        )
        gazebo_spawned = self._is_preloaded_shuttle_entity(entity_name)
        loaded = (
            self._initial_payload_loaded_for_entity(entity_name)
            if payload_loaded is None
            else bool(payload_loaded)
        )
        if not deployed:
            initial_stopped_by = 'NOT_DEPLOYED'
            initial_stopper_distance_m = 0.0
        elif not enabled:
            initial_stopped_by = 'DISABLED'
            initial_stopper_distance_m = 0.0
        else:
            initial_stopped_by = None
            initial_stopper_distance_m = None

        return ManagedShuttle(
            entity_name=entity_name,
            start_slot=resolved_slot,
            start_pose=start_pose,
            start_snap_distance_m=start_snap_distance_m,
            start_segment=initial_segment,
            start_s=initial_s,
            previous_sensor_segment=initial_segment,
            previous_sensor_s=initial_s,
            core=KinematicShuttleCore(
                network=self.network,
                initial_state=ShuttleState(
                    current_segment=initial_segment,
                    s=initial_s,
                    speed=speed,
                    mode=MOVING if enabled and deployed and speed > 0.0 else WAITING,
                ),
                falling_stop_offset_m=self.falling_stop_offset_m,
            ),
            pose_publisher=self.create_publisher(PoseStamped, pose_topic, 10),
            last_gazebo_set_pose_time=self.get_clock().now(),
            gazebo_spawned=gazebo_spawned,
            deployed=deployed,
            enabled=enabled,
            stopped_by=initial_stopped_by,
            stopper_distance_m=initial_stopper_distance_m,
            payload_loaded=loaded,
            payload_type=self.payload_type if loaded else 'none',
            payload_entity_name=self._payload_entity_name_for_shuttle(entity_name),
            spawned_visual_state=(
                SHUTTLE_VISUAL_NORMAL
                if gazebo_spawned
                else None
            ),
        )

    def _on_add_shuttle_request(
        self,
        request: AddShuttle.Request,
        response: AddShuttle.Response,
    ) -> AddShuttle.Response:
        try:
            entity_name, slot, speed, enabled = self._parse_add_shuttle_request(request)
            shuttle = self._create_managed_shuttle(
                entity_name=entity_name,
                slot=slot,
                speed=speed,
                enabled=enabled,
            )
        except (RuntimeError, ValueError) as error:
            response.success = False
            response.message = str(error)
            response.name = ''
            self.get_logger().error(f'Failed to add shuttle: {error}')
            return response

        self._finish_add_shuttle(shuttle)
        response.success = True
        response.message = f'Added shuttle {shuttle.entity_name} at slot {shuttle.start_slot}'
        response.name = shuttle.entity_name
        return response

    def _finish_add_shuttle(self, shuttle: ManagedShuttle) -> None:
        self.shuttles.append(shuttle)
        self._request_spawn_if_needed(shuttle)
        self.get_logger().info(
            f'Added shuttle {shuttle.entity_name} at slot {shuttle.start_slot}; '
            f'enabled={shuttle.enabled}; '
            f'shuttles={self._shuttle_summary()}'
        )

    def _parse_add_shuttle_request(
        self,
        request: AddShuttle.Request,
    ) -> tuple[str, str, float, bool]:
        slot = (
            self._normalize_start_slot(request.start_slot)
            if request.start_slot.strip()
            else ''
        )
        speed = (
            float(request.speed)
            if request.speed > 0.0
            else self.default_shuttle_speed
        )
        return self._resolve_add_shuttle_request(
            entity_name=request.name.strip(),
            slot=slot,
            speed=speed,
            enabled=bool(request.start_enabled),
        )

    def _resolve_add_shuttle_request(
        self,
        *,
        entity_name: str,
        slot: str,
        speed: float,
        enabled: bool,
    ) -> tuple[str, str, float, bool]:
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

        return entity_name, slot, speed, enabled

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

    @staticmethod
    def _payload_entity_name_for_shuttle(entity_name: str) -> str:
        return f'{entity_name}_payload'

    def _initial_payload_loaded_for_entity(self, entity_name: str) -> bool:
        refs = getattr(self, 'initial_loaded_shuttle_refs', set())
        if not refs or 'none' in refs:
            return False
        if 'all' in refs:
            return True
        return bool(refs & self._payload_reference_tokens(entity_name))

    def _payload_reference_tokens(self, entity_name: str) -> set[str]:
        lowered = str(entity_name or '').strip().casefold()
        tokens = {lowered}
        match = re.fullmatch(r'room315_(right|left)_shuttle_([1-4])', lowered)
        if match:
            side, index = match.groups()
            short_id = f'{"r" if side == "right" else "l"}{index}'
            tokens.update({
                f'{side}_shuttle_{index}',
                f'{side}{index}',
                short_id,
            })
            if side == self.rail_side:
                tokens.add(index)
        return tokens

    @staticmethod
    def _normalize_payload_loaded(raw_value) -> bool:
        if isinstance(raw_value, bool):
            return raw_value
        value = str(raw_value or '').strip().casefold()
        if value in {'1', 'true', 'yes', 'on', 'load', 'loaded', 'box', 'payload'}:
            return True
        if value in {'0', 'false', 'no', 'off', 'empty', 'none', 'unload', 'unloaded'}:
            return False
        raise ValueError(
            f'Unknown payload state {raw_value!r}; use loaded/empty or true/false.'
        )

    def _payload_state_payload(self) -> dict:
        shuttles = []
        for shuttle in self.shuttles:
            shuttle_id, short_id, index = self._public_shuttle_ids(shuttle.entity_name)
            loaded = bool(shuttle.payload_loaded)
            shuttles.append({
                'shuttle_id': shuttle_id,
                'short_id': short_id,
                'side': self.rail_side,
                'shuttle_index': index,
                'entity_name': shuttle.entity_name,
                'payload_entity_name': shuttle.payload_entity_name,
                'loaded': loaded,
                'payload_type': shuttle.payload_type if loaded else 'none',
                'model_input_exposure': 'excluded',
            })
        return {
            'side': self.rail_side,
            'payload_model': str(self.payload_model_sdf),
            'payload_state_topic': self.payload_state_topic,
            'model_input_exposure': 'excluded',
            'shuttles': shuttles,
        }

    def _publish_payload_state(self) -> None:
        if not hasattr(self, 'payload_state_publisher'):
            return
        message = String()
        message.data = json.dumps(self._payload_state_payload(), sort_keys=True)
        self.payload_state_publisher.publish(message)

    def _public_shuttle_ids(self, entity_name: str) -> tuple[str, str, int]:
        match = re.fullmatch(r'room315_(right|left)_shuttle_([1-4])', entity_name)
        if not match:
            return entity_name, '', -1
        side, index_text = match.groups()
        index = int(index_text) - 1
        short_id = f'{"R" if side == "right" else "L"}{index_text}'
        return f'{side}_shuttle_{index_text}', short_id, index

    def _on_payload_command(self, message: String) -> None:
        raw = str(message.data or '').strip()
        if not raw:
            return
        try:
            selector, loaded, payload_type = self._parse_payload_command(raw)
            targets = self._resolve_payload_command_targets(selector)
            for shuttle in targets:
                self._set_shuttle_payload_loaded(
                    shuttle,
                    loaded,
                    payload_type=payload_type,
                )
            self._publish_payload_state()
        except ValueError as error:
            self.get_logger().error(f'Payload command rejected: {error}')

    def _parse_payload_command(self, raw: str) -> tuple[str, bool, str | None]:
        if raw.startswith('{'):
            parsed = json.loads(raw)
            if not isinstance(parsed, dict):
                raise ValueError('payload command JSON must be an object')
            selector = str(
                parsed.get('shuttle')
                or parsed.get('shuttle_id')
                or parsed.get('name')
                or parsed.get('entity_name')
                or ''
            ).strip()
            state_value = (
                parsed.get('loaded')
                if 'loaded' in parsed
                else parsed.get('state', parsed.get('payload_state', parsed.get('payload_type')))
            )
            loaded = self._normalize_payload_loaded(state_value)
            payload_type = str(parsed.get('payload_type') or '').strip() or None
            return selector, loaded, payload_type

        tokens = raw.split()
        if not tokens:
            raise ValueError('empty payload command')
        selector = ''
        state_token = tokens[-1]
        if len(tokens) > 1:
            selector = tokens[0]
        elif len(self.shuttles) == 1:
            selector = self.shuttles[0].entity_name
        loaded = self._normalize_payload_loaded(state_token)
        return selector, loaded, None

    def _resolve_payload_command_targets(self, selector: str) -> list[ManagedShuttle]:
        token = str(selector or '').strip().casefold()
        if not token:
            if len(self.shuttles) == 1:
                return [self.shuttles[0]]
            raise ValueError('payload command must name a shuttle or use ALL')
        if token == 'all':
            return list(self.shuttles)
        matches = [
            shuttle
            for shuttle in self.shuttles
            if token in self._payload_reference_tokens(shuttle.entity_name)
        ]
        if not matches:
            raise ValueError(f'unknown payload shuttle selector {selector!r}')
        return matches

    def _set_shuttle_payload_loaded(
        self,
        shuttle: ManagedShuttle,
        loaded: bool,
        *,
        payload_type: str | None = None,
    ) -> None:
        shuttle.payload_loaded = bool(loaded)
        shuttle.payload_type = (
            (payload_type or self.payload_type or 'box')
            if shuttle.payload_loaded
            else 'none'
        )
        if shuttle.payload_loaded:
            shuttle.payload_delete_warning_logged = False
            self._request_payload_spawn_if_needed(shuttle)
        else:
            self._request_payload_delete_if_needed(shuttle)
        state = 'loaded' if shuttle.payload_loaded else 'empty'
        self.get_logger().info(f'Shuttle {shuttle.entity_name} payload state is now {state}.')

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
            shuttle.spawned_visual_state = SHUTTLE_VISUAL_NORMAL
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
        factory.sdf_filename = str(self._shuttle_model_sdf_for_entity(shuttle.entity_name))
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

    def _make_payload_spawn_entity_factory(
        self,
        shuttle: ManagedShuttle,
        pose: ShuttlePose | None = None,
    ) -> EntityFactory:
        gazebo_pose = pose if pose is not None else self._to_gazebo_pose(shuttle.core.pose())
        payload_pose = self._payload_pose_from_shuttle_pose(gazebo_pose)
        factory = EntityFactory()
        factory.name = shuttle.payload_entity_name
        factory.allow_renaming = False
        factory.sdf_filename = str(self.payload_model_sdf)
        factory.relative_to = 'world'
        factory.pose.position.x = payload_pose.x
        factory.pose.position.y = payload_pose.y
        factory.pose.position.z = payload_pose.z
        qx, qy, qz, qw = _yaw_to_quaternion(payload_pose.yaw)
        factory.pose.orientation.x = qx
        factory.pose.orientation.y = qy
        factory.pose.orientation.z = qz
        factory.pose.orientation.w = qw
        return factory

    def _payload_pose_from_shuttle_pose(self, pose: ShuttlePose) -> ShuttlePose:
        local_x_offset = self.payload_pose_x_offset_m
        world_x_offset = local_x_offset * math.cos(pose.yaw)
        world_y_offset = local_x_offset * math.sin(pose.yaw)
        return ShuttlePose(
            x=pose.x + world_x_offset,
            y=pose.y + world_y_offset,
            z=pose.z + self.payload_pose_z_offset_m,
            yaw=pose.yaw,
            current_segment=pose.current_segment,
            s=pose.s,
            mode=pose.mode,
        )

    def _shuttle_model_sdf_for_entity(self, entity_name: str) -> Path:
        """Return the per-identity shuttle model when available.

        The deployable VLA policy should learn shuttle identity from physical
        RGB-visible perimeter labels. Dynamic Gazebo spawns therefore use the
        R1..R4/L1..L4 shuttle models when the entity name follows the Room 315
        convention, falling back to the configured generic model for custom
        names.
        """

        match = re.fullmatch(r'room315_(right|left)_shuttle_([1-4])', entity_name)
        if not match:
            return self.shuttle_model_sdf
        side, index = match.groups()
        short_id = f'{"R" if side == "right" else "L"}{index}'
        candidate = self.shuttle_model_sdf.parents[1] / f'room315_shuttle_{short_id}' / 'model.sdf'
        return candidate if candidate.exists() else self.shuttle_model_sdf

    def _desired_shuttle_visual_state(self, shuttle: ManagedShuttle) -> str:
        if not self.visual_debug_colors:
            return SHUTTLE_VISUAL_NORMAL
        return (
            SHUTTLE_VISUAL_FALLING
            if shuttle.core.state.mode == FALLING
            else SHUTTLE_VISUAL_NORMAL
        )

    @staticmethod
    def _shuttle_visual_rgba(visual_state: str) -> tuple[float, float, float, float]:
        if visual_state == SHUTTLE_VISUAL_FALLING:
            return 1.0, 0.02, 0.02, 1.0
        return 0.01, 0.01, 0.01, 1.0

    def _shuttle_visual_sdf(self, entity_name: str, visual_state: str) -> str:
        red, green, blue, alpha = self._shuttle_visual_rgba(visual_state)
        return (
            '<sdf version="1.9">'
            f'<model name="{entity_name}">'
            '<static>false</static>'
            '<link name="room315_shuttle_link">'
            '<kinematic>true</kinematic>'
            '<gravity>false</gravity>'
            '<pose>0 0 0 0 0 0</pose>'
            '<inertial>'
            '<mass>0.1</mass>'
            '<inertia>'
            '<ixx>0.001</ixx>'
            '<ixy>0.0</ixy>'
            '<ixz>0.0</ixz>'
            '<iyy>0.001</iyy>'
            '<iyz>0.0</iyz>'
            '<izz>0.001</izz>'
            '</inertia>'
            '</inertial>'
            '<visual name="room315_shuttle_visual">'
            '<pose>-0.25 -0.100000024 -0.078499973 0 0 0</pose>'
            '<geometry>'
            '<mesh>'
            '<uri>model://room315_shuttle/meshes/shuttle.STL</uri>'
            '</mesh>'
            '</geometry>'
            '<material>'
            f'<ambient>{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}</ambient>'
            f'<diffuse>{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}</diffuse>'
            '<specular>0.20 0.20 0.20 1</specular>'
            '<emissive>0 0 0 1</emissive>'
            '</material>'
            '</visual>'
            '<collision name="room315_shuttle_robot_collision">'
            '<pose>-0.078417048 0 0 0 0 0</pose>'
            '<geometry>'
            '<box>'
            '<size>0.36 0.22 0.17</size>'
            '</box>'
            '</geometry>'
            '<surface>'
            '<contact>'
            '<collide_bitmask>0x0000</collide_bitmask>'
            '</contact>'
            '</surface>'
            '</collision>'
            '</link>'
            '<plugin filename="gz-sim-pose-publisher-system" '
            'name="gz::sim::systems::PosePublisher">'
            '<publish_model_pose>true</publish_model_pose>'
            '<publish_link_pose>true</publish_link_pose>'
            '<publish_collision_pose>false</publish_collision_pose>'
            '<publish_visual_pose>false</publish_visual_pose>'
            '<publish_nested_model_pose>false</publish_nested_model_pose>'
            '<update_frequency>30</update_frequency>'
            '</plugin>'
            '</model>'
            '</sdf>'
        )

    def _make_shuttle_visual_factory(
        self,
        shuttle: ManagedShuttle,
        visual_state: str,
    ) -> EntityFactory:
        pose = self._to_gazebo_pose(shuttle.core.pose())
        factory = EntityFactory()
        factory.name = shuttle.entity_name
        factory.allow_renaming = False
        factory.sdf = self._shuttle_visual_sdf(shuttle.entity_name, visual_state)
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

    def _update_shuttle_visuals(self) -> None:
        for shuttle in self.shuttles:
            self._process_shuttle_visual_futures(shuttle)
            self._request_shuttle_visual_refresh_if_needed(shuttle)

    def _process_shuttle_visual_futures(self, shuttle: ManagedShuttle) -> None:
        if (
            shuttle.pending_visual_delete is not None
            and shuttle.pending_visual_delete.done()
        ):
            try:
                response = shuttle.pending_visual_delete.result()
            except Exception as error:
                shuttle.pending_visual_delete = None
                shuttle.visual_refresh_in_progress = False
                shuttle.next_visual_refresh_time = (
                    time.monotonic() + SHUTTLE_VISUAL_REFRESH_RETRY_INTERVAL_S
                )
                if not shuttle.visual_refresh_failure_logged:
                    self.get_logger().warn(
                        f'Gazebo shuttle visual delete request for '
                        f'{shuttle.entity_name} failed: {error}'
                    )
                    shuttle.visual_refresh_failure_logged = True
                return

            shuttle.pending_visual_delete = None
            if not response.success:
                shuttle.visual_refresh_in_progress = False
                shuttle.next_visual_refresh_time = (
                    time.monotonic() + SHUTTLE_VISUAL_REFRESH_RETRY_INTERVAL_S
                )
                if not shuttle.visual_refresh_failure_logged:
                    self.get_logger().warn(
                        f'Gazebo shuttle visual delete service rejected '
                        f'{shuttle.entity_name}.'
                    )
                    shuttle.visual_refresh_failure_logged = True
                return

            shuttle.gazebo_spawned = False
            shuttle.spawned_visual_state = None

        if (
            shuttle.pending_visual_spawn is not None
            and shuttle.pending_visual_spawn.done()
        ):
            requested_state = shuttle.pending_visual_spawn_state
            try:
                response = shuttle.pending_visual_spawn.result()
            except Exception as error:
                shuttle.pending_visual_spawn = None
                shuttle.pending_visual_spawn_state = None
                shuttle.next_visual_refresh_time = (
                    time.monotonic() + SHUTTLE_VISUAL_REFRESH_RETRY_INTERVAL_S
                )
                if not shuttle.visual_refresh_failure_logged:
                    self.get_logger().warn(
                        f'Gazebo shuttle visual spawn request for '
                        f'{shuttle.entity_name} failed: {error}'
                    )
                    shuttle.visual_refresh_failure_logged = True
                return

            shuttle.pending_visual_spawn = None
            shuttle.pending_visual_spawn_state = None
            if not response.success:
                shuttle.next_visual_refresh_time = (
                    time.monotonic() + SHUTTLE_VISUAL_REFRESH_RETRY_INTERVAL_S
                )
                if not shuttle.visual_refresh_failure_logged:
                    self.get_logger().warn(
                        f'Gazebo shuttle visual spawn service rejected '
                        f'{shuttle.entity_name}.'
                    )
                    shuttle.visual_refresh_failure_logged = True
                return

            shuttle.gazebo_spawned = True
            shuttle.spawned_visual_state = requested_state
            shuttle.visual_refresh_in_progress = False
            shuttle.visual_refresh_failure_logged = False
            self.deleted_preloaded_entity_names.discard(shuttle.entity_name)

    def _request_shuttle_visual_refresh_if_needed(
        self,
        shuttle: ManagedShuttle,
    ) -> None:
        if not shuttle.deployed:
            return

        desired_state = self._desired_shuttle_visual_state(shuttle)
        shuttle.visual_state = desired_state

        if (
            shuttle.pending_visual_delete is not None
            or shuttle.pending_visual_spawn is not None
        ):
            return
        if time.monotonic() < shuttle.next_visual_refresh_time:
            return

        if shuttle.visual_refresh_in_progress and not shuttle.gazebo_spawned:
            self._request_shuttle_visual_spawn(shuttle, desired_state)
            return

        if not shuttle.gazebo_spawned or shuttle.spawned_visual_state == desired_state:
            return

        if self.delete_client is None or not self.delete_client.service_is_ready():
            return

        request = DeleteEntity.Request()
        request.entity.name = shuttle.entity_name
        request.entity.type = Entity.MODEL
        shuttle.pending_visual_delete = self.delete_client.call_async(request)
        shuttle.visual_refresh_in_progress = True
        self.get_logger().info(
            f'Refreshing shuttle visual {shuttle.entity_name}: {desired_state}.'
        )

    def _request_shuttle_visual_spawn(
        self,
        shuttle: ManagedShuttle,
        visual_state: str,
    ) -> None:
        if self.spawn_client is None or not self.spawn_client.service_is_ready():
            return

        request = SpawnEntity.Request()
        request.entity_factory = self._make_shuttle_visual_factory(
            shuttle,
            visual_state,
        )
        shuttle.pending_visual_spawn = self.spawn_client.call_async(request)
        shuttle.pending_visual_spawn_state = visual_state

    def _make_device_markers(self) -> list[DeviceMarker]:
        markers: list[DeviceMarker] = []
        position_sensor_base_visuals: list[tuple[str, ShuttlePose]] = []
        marker_specs = [
            ('position_sensors', 'position_sensor', self.rail_devices.position_sensors),
            ('stoppers', 'stopper', self.rail_devices.stoppers),
        ]
        for category, marker_type, grouped_devices in marker_specs:
            for public_name, raw_devices in grouped_devices.items():
                devices = (
                    (raw_devices,)
                    if isinstance(raw_devices, RailDevice)
                    else tuple(raw_devices)
                )
                for index, device in enumerate(devices):
                    entity_name = self._device_marker_entity_name(
                        category=category,
                        marker_type=marker_type,
                        public_name=public_name,
                        device=device,
                        duplicate_suffix=device.segment if len(devices) > 1 else '',
                    )
                    raw_pose = ShuttlePose(
                        x=device.x,
                        y=device.y,
                        z=device.z,
                        yaw=device.yaw,
                        current_segment=device.segment,
                        s=device.s,
                        mode=WAITING,
                    )
                    gazebo_pose = self._to_gazebo_pose(raw_pose)
                    style = DEVICE_MARKER_STYLES[marker_type]
                    visual_state = self._initial_device_marker_visual_state(
                        marker_type,
                        device.name,
                    )
                    marker_pose = ShuttlePose(
                        x=gazebo_pose.x,
                        y=gazebo_pose.y,
                        z=(
                            gazebo_pose.z
                            + self.device_marker_z_offset_m
                            + float(style.get('z_offset_m', 0.0))
                        ),
                        yaw=gazebo_pose.yaw,
                        current_segment=gazebo_pose.current_segment,
                        s=gazebo_pose.s,
                        mode=gazebo_pose.mode,
                    )
                    if marker_type == 'position_sensor':
                        position_sensor_base_visuals.append((entity_name, marker_pose))
                        hidden_pose = ShuttlePose(
                            x=marker_pose.x,
                            y=marker_pose.y,
                            z=-10.0,
                            yaw=marker_pose.yaw,
                            current_segment=marker_pose.current_segment,
                            s=marker_pose.s,
                            mode=marker_pose.mode,
                        )
                        active_entity_name = f'{entity_name}_active'
                        markers.append(
                            DeviceMarker(
                                entity_name=active_entity_name,
                                device_type=marker_type,
                                device_name=device.name,
                                segment=device.segment,
                                pose=hidden_pose,
                                sdf=self._device_marker_sdf(
                                    active_entity_name,
                                    marker_type,
                                    MARKER_VISUAL_ACTIVE,
                                ),
                                sensor_s=device.s,
                                sensor_radius_m=device.radius_m,
                                active_overlay=True,
                                visible_pose=marker_pose,
                                hidden_pose=hidden_pose,
                                desired_visible=False,
                                visible=False,
                                visual_state=MARKER_VISUAL_ACTIVE,
                            )
                        )
                        continue
                    if marker_type == 'stopper':
                        markers.append(
                            DeviceMarker(
                                entity_name=entity_name,
                                device_type='stopper_base',
                                device_name=device.name,
                                segment=device.segment,
                                pose=marker_pose,
                                sdf=self._device_marker_sdf(
                                    entity_name,
                                    marker_type,
                                    MARKER_VISUAL_INACTIVE,
                                ),
                                visual_state=MARKER_VISUAL_INACTIVE,
                            )
                        )
                        hidden_pose = ShuttlePose(
                            x=marker_pose.x,
                            y=marker_pose.y,
                            z=-10.0,
                            yaw=marker_pose.yaw,
                            current_segment=marker_pose.current_segment,
                            s=marker_pose.s,
                            mode=marker_pose.mode,
                        )
                        active_pose = ShuttlePose(
                            x=marker_pose.x,
                            y=marker_pose.y,
                            z=marker_pose.z + (0.015 * self.device_marker_scale),
                            yaw=marker_pose.yaw,
                            current_segment=marker_pose.current_segment,
                            s=marker_pose.s,
                            mode=marker_pose.mode,
                        )
                        active_entity_name = f'{entity_name}_active'
                        default_closed = (
                            self.stopper_states.get(device.name, STOPPER_PASS_STATE)
                            == STOPPER_STOP_STATE
                        )
                        markers.append(
                            DeviceMarker(
                                entity_name=active_entity_name,
                                device_type=marker_type,
                                device_name=device.name,
                                segment=device.segment,
                                pose=hidden_pose,
                                sdf=self._device_marker_sdf(
                                    active_entity_name,
                                    marker_type,
                                    MARKER_VISUAL_ACTIVE,
                                ),
                                active_overlay=True,
                                visible_pose=active_pose,
                                hidden_pose=hidden_pose,
                                desired_visible=default_closed,
                                visible=False,
                                visual_state=MARKER_VISUAL_ACTIVE,
                            )
                        )
                        continue
                    markers.append(
                        DeviceMarker(
                            entity_name=entity_name,
                            device_type=marker_type,
                            device_name=device.name,
                            segment=device.segment,
                            pose=marker_pose,
                            sdf=self._device_marker_sdf(
                                entity_name,
                                marker_type,
                                visual_state,
                            ),
                            sensor_s=(
                                device.s
                                if marker_type == 'position_sensor'
                                else None
                            ),
                            sensor_radius_m=(
                                device.radius_m
                                if marker_type == 'position_sensor'
                                else None
                            ),
                            visual_state=visual_state,
                        )
                    )
        if position_sensor_base_visuals:
            entity_name = f'marker_{self.rail_side}_position_sensors'
            markers.insert(
                0,
                DeviceMarker(
                    entity_name=entity_name,
                    device_type='position_sensor_base',
                    device_name='position_sensors',
                    segment='',
                    pose=ShuttlePose(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        yaw=0.0,
                        current_segment='',
                        s=0.0,
                        mode=WAITING,
                    ),
                    sdf=self._position_sensor_base_marker_sdf(
                        entity_name,
                        position_sensor_base_visuals,
                    ),
                    visual_state=MARKER_VISUAL_INACTIVE,
                ),
            )
        return markers

    def _initial_device_marker_visual_state(
        self,
        marker_type: str,
        device_name: str,
    ) -> str:
        if marker_type == 'position_sensor':
            return MARKER_VISUAL_INACTIVE
        if marker_type == 'stopper':
            stopper_name = _canonical_switch_name(device_name)
            return (
                MARKER_VISUAL_ACTIVE
                if (
                    self.stopper_states.get(stopper_name, STOPPER_PASS_STATE)
                    == STOPPER_STOP_STATE
                )
                else MARKER_VISUAL_INACTIVE
            )
        return MARKER_VISUAL_DEFAULT

    def _device_marker_entity_name(
        self,
        *,
        category: str,
        marker_type: str,
        public_name: str,
        device: RailDevice | None,
        duplicate_suffix: str,
    ) -> str:
        if marker_type == 'position_sensor':
            raw_name = f'marker_{self.rail_side}_{device.name}'
        elif marker_type == 'stopper':
            raw_name = f'marker_{self.rail_side}_stopper_{public_name}'
        else:
            device_name = device.name if device is not None else public_name
            raw_name = f'marker_{self.rail_side}_{marker_type}_{device_name}'

        if duplicate_suffix:
            raw_name = f'{raw_name}_{duplicate_suffix}'
        return re.sub(r'[^A-Za-z0-9_]+', '_', raw_name).strip('_')

    def _device_marker_sdf(
        self,
        entity_name: str,
        marker_type: str,
        visual_state: str = MARKER_VISUAL_DEFAULT,
    ) -> str:
        style = DEVICE_MARKER_STYLES[marker_type]
        radius = style['radius'] * self.device_marker_scale
        if visual_state == MARKER_VISUAL_ACTIVE:
            radius *= float(style.get('active_radius_scale', 1.0))
        length = style['length'] * self.device_marker_scale
        red, green, blue, alpha = self._device_marker_rgba(marker_type, visual_state)
        if style['shape'] == 'cylinder':
            geometry = (
                '<cylinder>'
                f'<radius>{radius:.6f}</radius>'
                f'<length>{length:.6f}</length>'
                '</cylinder>'
            )
        else:
            geometry = f'<sphere><radius>{radius:.6f}</radius></sphere>'

        material = (
            '<material>'
            f'<ambient>{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}</ambient>'
            f'<diffuse>{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}</diffuse>'
            '</material>'
        )
        return (
            '<sdf version="1.9">'
            f'<model name="{entity_name}">'
            '<static>true</static>'
            '<link name="link">'
            '<visual name="visual">'
            '<cast_shadows>false</cast_shadows>'
            f'<geometry>{geometry}</geometry>'
            f'{material}'
            '</visual>'
            '</link>'
            '</model>'
            '</sdf>'
        )

    def _position_sensor_base_marker_sdf(
        self,
        entity_name: str,
        sensor_visuals: list[tuple[str, ShuttlePose]],
    ) -> str:
        style = DEVICE_MARKER_STYLES['position_sensor']
        radius = style['radius'] * self.device_marker_scale
        red, green, blue, alpha = self._device_marker_rgba(
            'position_sensor',
            MARKER_VISUAL_INACTIVE,
        )
        material = (
            '<material>'
            f'<ambient>{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}</ambient>'
            f'<diffuse>{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}</diffuse>'
            '</material>'
        )
        visuals = []
        for raw_name, pose in sensor_visuals:
            visual_name = re.sub(r'[^A-Za-z0-9_]+', '_', raw_name).strip('_')
            visuals.append(
                f'<visual name="{visual_name}">'
                '<cast_shadows>false</cast_shadows>'
                f'<pose>{pose.x:.6f} {pose.y:.6f} {pose.z:.6f} 0 0 0</pose>'
                '<geometry>'
                '<sphere>'
                f'<radius>{radius:.6f}</radius>'
                '</sphere>'
                '</geometry>'
                f'{material}'
                '</visual>'
            )
        return (
            '<sdf version="1.9">'
            f'<model name="{entity_name}">'
            '<static>true</static>'
            '<link name="link">'
            f'{"".join(visuals)}'
            '</link>'
            '</model>'
            '</sdf>'
        )

    @staticmethod
    def _device_marker_rgba(
        marker_type: str,
        visual_state: str,
    ) -> tuple[float, float, float, float]:
        style = DEVICE_MARKER_STYLES[marker_type]
        state_colors = style.get('rgba_by_state', {})
        return state_colors.get(visual_state, style['rgba'])

    def _make_device_marker_factory(self, marker: DeviceMarker) -> EntityFactory:
        factory = EntityFactory()
        factory.name = marker.entity_name
        factory.allow_renaming = False
        factory.sdf = marker.sdf
        factory.relative_to = 'world'
        factory.pose.position.x = marker.pose.x
        factory.pose.position.y = marker.pose.y
        factory.pose.position.z = marker.pose.z
        qx, qy, qz, qw = _yaw_to_quaternion(marker.pose.yaw)
        factory.pose.orientation.x = qx
        factory.pose.orientation.y = qy
        factory.pose.orientation.z = qz
        factory.pose.orientation.w = qw
        return factory

    def _update_device_markers(self) -> None:
        if not self.enable_device_markers or not self.device_markers:
            return
        self._process_device_marker_futures()
        self._request_device_marker_pose_updates()
        self._request_device_marker_refreshes()
        self._request_device_marker_spawns()

    def _process_device_marker_futures(self) -> None:
        now = time.monotonic()
        for marker in self.device_markers:
            if marker.pending_pose is not None and marker.pending_pose.done():
                try:
                    response = marker.pending_pose.result()
                    if response.success and marker.pending_visible is not None:
                        marker.visible = marker.pending_visible
                    elif not response.success:
                        self.get_logger().warn(
                            f'Gazebo marker pose service rejected {marker.entity_name}.'
                        )
                except Exception as error:
                    self.get_logger().warn(
                        f'Gazebo marker pose request for {marker.entity_name} failed: {error}'
                    )
                marker.pending_pose = None
                marker.pending_visible = None

            if marker.pending_delete is not None and marker.pending_delete.done():
                try:
                    response = marker.pending_delete.result()
                    if not response.success:
                        self.get_logger().warn(
                            f'Gazebo marker delete service rejected {marker.entity_name}; '
                            'the refreshed marker spawn may be delayed until the world restarts.'
                        )
                    else:
                        marker.spawn_attempts = 0
                        marker.spawn_failure_logged = False
                        marker.spawned_visual_state = None
                except Exception as error:
                    self.get_logger().warn(
                        f'Gazebo marker delete request for {marker.entity_name} failed: {error}'
                    )
                marker.pending_delete = None
                marker.next_spawn_attempt_time = now

            if marker.pending_spawn is None or not marker.pending_spawn.done():
                continue

            try:
                response = marker.pending_spawn.result()
            except Exception as error:
                marker.pending_spawn = None
                marker.pending_spawn_visual_state = None
                marker.next_spawn_attempt_time = now + self.device_marker_retry_interval_s
                if self._marker_spawn_attempts_exhausted(marker):
                    if not marker.spawn_failure_logged:
                        self.get_logger().warn(
                            f'Gazebo marker spawn request for {marker.entity_name} '
                            f'failed after {marker.spawn_attempts} attempt(s): {error}; '
                            'will keep retrying.'
                        )
                        marker.spawn_failure_logged = True
                continue

            marker.pending_spawn = None
            if response.success:
                marker.spawned = True
                marker.spawned_visual_state = marker.pending_spawn_visual_state
                marker.pending_spawn_visual_state = None
                marker.last_spawn_success_time = now
                if marker.active_overlay:
                    self._request_device_marker_pose_update(marker)
                continue
            marker.pending_spawn_visual_state = None

            marker.next_spawn_attempt_time = now + self.device_marker_retry_interval_s

            if self._marker_spawn_attempts_exhausted(marker):
                if not marker.spawn_failure_logged:
                    self.get_logger().warn(
                        f'Gazebo marker spawn service rejected {marker.entity_name} '
                        f'after {marker.spawn_attempts} attempt(s); will keep retrying.'
                    )
                    marker.spawn_failure_logged = True

    def _request_device_marker_delete(self, marker: DeviceMarker, *, reason: str) -> bool:
        if marker.active_overlay or marker.device_type == 'position_sensor':
            return False
        if self.delete_client is None or not self.delete_client.service_is_ready():
            return False
        request = DeleteEntity.Request()
        request.entity.name = marker.entity_name
        request.entity.type = Entity.MODEL
        marker.spawned = False
        marker.pending_delete = self.delete_client.call_async(request)
        self.get_logger().info(
            f'Refreshing device marker {marker.entity_name}: {reason}.'
        )
        return True

    def _request_device_marker_pose_updates(self) -> None:
        for marker in self.device_markers:
            self._request_device_marker_pose_update(marker)

    def _request_device_marker_pose_update(self, marker: DeviceMarker) -> bool:
        if (
            not marker.active_overlay
            or not marker.spawned
            or marker.pending_pose is not None
            or marker.desired_visible == marker.visible
        ):
            return False
        if self.set_pose_client is None or not self.set_pose_client.service_is_ready():
            return False

        target_pose = marker.visible_pose if marker.desired_visible else marker.hidden_pose
        if target_pose is None:
            return False

        request = SetEntityPose.Request()
        request.entity.name = marker.entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = target_pose.x
        request.pose.position.y = target_pose.y
        request.pose.position.z = target_pose.z
        qx, qy, qz, qw = _yaw_to_quaternion(target_pose.yaw)
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        marker.pose = target_pose
        marker.pending_pose = self.set_pose_client.call_async(request)
        marker.pending_visible = marker.desired_visible
        return True

    def _request_device_marker_refreshes(self) -> None:
        if not self.device_marker_dynamic_refresh:
            return
        if self.delete_client is None or not self.delete_client.service_is_ready():
            return

        now = time.monotonic()
        for marker in self.device_markers:
            if marker.device_type == 'position_sensor':
                continue
            if (
                not marker.spawned
                or marker.spawned_visual_state == marker.visual_state
                or marker.pending_spawn is not None
                or marker.pending_delete is not None
            ):
                continue
            if (
                marker.last_spawn_success_time is not None
                and now - marker.last_spawn_success_time
                < self.device_marker_refresh_grace_period_s
            ):
                continue

            self._request_device_marker_delete(
                marker,
                reason=f'{marker.device_type} visual state {marker.visual_state}',
            )

    def _marker_spawn_attempts_exhausted(self, marker: DeviceMarker) -> bool:
        return (
            self.device_marker_max_spawn_attempts > 0
            and marker.spawn_attempts >= self.device_marker_max_spawn_attempts
        )

    def _request_device_marker_spawns(self) -> None:
        if self.spawn_client is None:
            return
        if not self.spawn_client.service_is_ready():
            if not self.device_marker_spawn_warning_logged:
                self.get_logger().warn(
                    'Gazebo create service is not ready yet; device markers will be spawned later.'
                )
                self.device_marker_spawn_warning_logged = True
            return

        now = time.monotonic()
        if now < self.next_device_marker_spawn_time:
            return

        for marker in self.device_markers:
            if (
                marker.spawned
                or marker.pending_spawn is not None
                or marker.pending_delete is not None
                or now < marker.next_spawn_attempt_time
            ):
                continue

            request = SpawnEntity.Request()
            request.entity_factory = self._make_device_marker_factory(marker)
            marker.pending_spawn = self.spawn_client.call_async(request)
            marker.pending_spawn_visual_state = marker.visual_state
            marker.spawn_attempts += 1
            self.next_device_marker_spawn_time = (
                now + self.device_marker_spawn_interval_s
            )
            return

    def _set_device_marker_visual_state(
        self,
        marker: DeviceMarker,
        visual_state: str,
    ) -> bool:
        if marker.visual_state == visual_state:
            return False

        marker.visual_state = visual_state
        marker.sdf = self._device_marker_sdf(
            marker.entity_name,
            marker.device_type,
            visual_state,
        )
        marker.spawn_failure_logged = False
        if marker.pending_spawn is None:
            marker.spawn_attempts = 0
        marker.next_spawn_attempt_time = time.monotonic()
        return True

    def _set_device_marker_visibility(
        self,
        marker: DeviceMarker,
        visible: bool,
    ) -> bool:
        if marker.desired_visible == visible:
            return False
        marker.desired_visible = visible
        return True

    def _update_sensor_marker_states(self) -> None:
        if not self.enable_device_markers or not self.device_markers:
            return

        now_s = (
            self._state_update_time_s()
            if hasattr(self, '_state_update_time_s')
            else time.monotonic()
        )
        visual_hold_s = max(0.0, float(getattr(self, 'sensor_marker_visual_hold_s', 0.0)))
        changed = False
        for marker in self.device_markers:
            if marker.device_type != 'position_sensor' or not marker.active_overlay:
                continue

            active_shuttle = None
            if marker.sensor_s is not None and marker.sensor_radius_m is not None:
                active_shuttle = self._shuttle_on_sensor_for_marker(
                    marker.segment,
                    marker.sensor_s,
                    marker.sensor_radius_m,
                )
            visible = active_shuttle is not None
            if visible:
                marker.visual_hold_until_s = max(
                    marker.visual_hold_until_s,
                    now_s + visual_hold_s,
                )
            elif marker.visual_hold_until_s > now_s:
                visible = True
            changed = (
                self._set_device_marker_visibility(
                    marker,
                    visible,
                )
                or changed
            )

        if changed:
            self._request_device_marker_pose_updates()

    @staticmethod
    def _sensor_markers_should_arm(shuttle: ManagedShuttle) -> bool:
        state = shuttle.core.state
        if state.mode != MOVING:
            return False
        return (
            state.current_segment != shuttle.start_segment
            or abs(state.s - shuttle.start_s) > 0.03
        )

    def _shuttle_on_sensor_for_marker(
        self,
        segment: str,
        sensor_s: float,
        radius_m: float,
    ) -> ManagedShuttle | None:
        shuttle = self._shuttle_on_sensor(segment, sensor_s, radius_m)
        if shuttle is None:
            return self._shuttle_crossed_sensor_since_last_tick(
                segment,
                sensor_s,
                radius_m,
            )
        if shuttle.sensor_markers_armed:
            return shuttle
        if self._sensor_markers_should_arm(shuttle):
            shuttle.sensor_markers_armed = True
            return shuttle
        return None

    def _shuttle_crossed_sensor_since_last_tick(
        self,
        segment: str,
        sensor_s: float,
        radius_m: float,
    ) -> ManagedShuttle | None:
        sensor_start = sensor_s - radius_m
        sensor_end = sensor_s + radius_m
        for shuttle in self.shuttles:
            if not shuttle.deployed:
                continue
            previous_segment = getattr(shuttle, 'previous_sensor_segment', None)
            previous_s = getattr(shuttle, 'previous_sensor_s', None)
            if previous_segment is None or previous_s is None:
                continue

            state = shuttle.core.state
            if (
                previous_segment == state.current_segment
                and abs(state.s - previous_s) <= 1e-9
            ):
                continue

            interval_start = None
            interval_end = None
            if previous_segment == segment and state.current_segment == segment:
                interval_start = min(previous_s, state.s)
                interval_end = max(previous_s, state.s)
            elif previous_segment == segment:
                interval_start = previous_s
                interval_end = self.network.segments[segment].length
            elif state.current_segment == segment:
                interval_start = 0.0
                interval_end = state.s

            if interval_start is None or interval_end is None:
                continue
            if interval_end < sensor_start or interval_start > sensor_end:
                continue

            if shuttle.sensor_markers_armed:
                return shuttle
            if self._sensor_markers_should_arm(shuttle):
                shuttle.sensor_markers_armed = True
                return shuttle
        return None

    def _update_stopper_marker_states(self) -> None:
        if not self.enable_device_markers or not self.device_markers:
            return

        pose_changed = False
        refresh_changed = False
        for marker in self.device_markers:
            if marker.device_type != 'stopper':
                continue

            stopper_name = _canonical_switch_name(marker.device_name)
            closed = (
                self.stopper_states.get(stopper_name, STOPPER_PASS_STATE)
                == STOPPER_STOP_STATE
            )
            if marker.active_overlay:
                pose_changed = (
                    self._set_device_marker_visibility(marker, closed)
                    or pose_changed
                )
                continue

            visual_state = (
                MARKER_VISUAL_ACTIVE
                if closed
                else MARKER_VISUAL_INACTIVE
            )
            refresh_changed = (
                self._set_device_marker_visual_state(marker, visual_state)
                or refresh_changed
            )

        if pose_changed:
            self._request_device_marker_pose_updates()
        if refresh_changed:
            self._request_device_marker_refreshes()

    def _spawn_ready_for_motion(self, shuttle: ManagedShuttle) -> bool:
        self._process_shuttle_visual_futures(shuttle)
        if shuttle.visual_refresh_in_progress:
            self._request_shuttle_visual_refresh_if_needed(shuttle)
            return False
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
        shuttle.spawned_visual_state = self._desired_shuttle_visual_state(shuttle)
        self.deleted_preloaded_entity_names.discard(shuttle.entity_name)
        self.get_logger().info(f'Gazebo spawned {shuttle.entity_name}')
        return True

    def _update_payload_visual(
        self,
        shuttle: ManagedShuttle,
        gazebo_pose: ShuttlePose | None = None,
    ) -> None:
        self._process_payload_futures(shuttle)
        if not self.enable_payload_visuals:
            return
        if not shuttle.deployed or not shuttle.payload_loaded:
            self._request_payload_delete_if_needed(shuttle)
            return
        self._request_payload_spawn_if_needed(shuttle, gazebo_pose)
        if shuttle.payload_gazebo_spawned and gazebo_pose is not None:
            self._send_payload_gazebo_pose(shuttle, gazebo_pose)

    def _process_payload_futures(self, shuttle: ManagedShuttle) -> None:
        if shuttle.pending_payload_spawn is not None and shuttle.pending_payload_spawn.done():
            try:
                response = shuttle.pending_payload_spawn.result()
            except Exception as error:
                if not shuttle.payload_spawn_failure_logged:
                    self.get_logger().error(
                        f'Gazebo payload spawn request for {shuttle.payload_entity_name} '
                        f'failed: {error}'
                    )
                    shuttle.payload_spawn_failure_logged = True
                shuttle.pending_payload_spawn = None
            else:
                shuttle.pending_payload_spawn = None
                if bool(getattr(response, 'success', False)):
                    shuttle.payload_gazebo_spawned = True
                    shuttle.payload_spawn_failure_logged = False
                    self.get_logger().info(f'Gazebo spawned {shuttle.payload_entity_name}')
                elif not shuttle.payload_spawn_failure_logged:
                    self.get_logger().error(
                        f'Gazebo spawn service rejected {shuttle.payload_entity_name}.'
                    )
                    shuttle.payload_spawn_failure_logged = True

        if shuttle.pending_payload_delete is not None and shuttle.pending_payload_delete.done():
            try:
                response = shuttle.pending_payload_delete.result()
            except Exception as error:
                self.get_logger().error(
                    f'Gazebo payload delete request for {shuttle.payload_entity_name} '
                    f'failed: {error}'
                )
            else:
                if bool(getattr(response, 'success', False)):
                    shuttle.payload_gazebo_spawned = False
                    shuttle.pending_payload_set_pose = None
                    self.get_logger().info(f'Gazebo removed {shuttle.payload_entity_name}')
                else:
                    self.get_logger().error(
                        f'Gazebo delete service rejected {shuttle.payload_entity_name}; '
                        'the payload may still be visible.'
                    )
            shuttle.pending_payload_delete = None

        if (
            shuttle.pending_payload_set_pose is not None
            and shuttle.pending_payload_set_pose.done()
        ):
            shuttle.pending_payload_set_pose = None
            shuttle.pending_payload_set_pose_wall_time = None
            shuttle.payload_set_pose_timeout_warning_logged = False

    def _request_payload_spawn_if_needed(
        self,
        shuttle: ManagedShuttle,
        gazebo_pose: ShuttlePose | None = None,
    ) -> None:
        if not self.enable_payload_visuals or not shuttle.payload_loaded or not shuttle.deployed:
            return
        if self.spawn_client is None or not self.enable_gazebo_spawn:
            return
        if shuttle.payload_gazebo_spawned or shuttle.pending_payload_spawn is not None:
            return
        if shuttle.pending_payload_delete is not None:
            return
        if not self.spawn_client.service_is_ready():
            if not shuttle.payload_spawn_failure_logged:
                self.get_logger().warn(
                    'Gazebo spawn service is not ready yet. Loaded shuttle '
                    f'{shuttle.entity_name} will publish payload state, but the '
                    'payload box is not visible yet.'
                )
                shuttle.payload_spawn_failure_logged = True
            return

        request = SpawnEntity.Request()
        request.entity_factory = self._make_payload_spawn_entity_factory(
            shuttle,
            gazebo_pose,
        )
        shuttle.pending_payload_spawn = self.spawn_client.call_async(request)
        self.get_logger().info(f'Requested Gazebo payload spawn for {shuttle.entity_name}')

    def _request_payload_delete_if_needed(self, shuttle: ManagedShuttle) -> None:
        if shuttle.pending_payload_spawn is not None:
            return
        if shuttle.pending_payload_delete is not None:
            return
        if not shuttle.payload_gazebo_spawned:
            return
        if self.delete_client is None or not self.enable_gazebo_delete:
            if not shuttle.payload_delete_warning_logged:
                self.get_logger().warn(
                    f'Cannot remove payload {shuttle.payload_entity_name}: '
                    'Gazebo delete support is disabled.'
                )
                shuttle.payload_delete_warning_logged = True
            return
        if not self.delete_client.service_is_ready():
            if not shuttle.payload_delete_warning_logged:
                self.get_logger().warn(
                    f'Cannot remove payload {shuttle.payload_entity_name}: '
                    'Gazebo delete service is not ready.'
                )
                shuttle.payload_delete_warning_logged = True
            return

        request = DeleteEntity.Request()
        request.entity.name = shuttle.payload_entity_name
        request.entity.type = Entity.MODEL
        shuttle.pending_payload_delete = self.delete_client.call_async(request)
        self.get_logger().info(f'Requested Gazebo payload removal for {shuttle.entity_name}.')

    def _send_payload_gazebo_pose(
        self,
        shuttle: ManagedShuttle,
        shuttle_pose: ShuttlePose,
    ) -> None:
        if not self.enable_gazebo_set_pose or self.set_pose_client is None:
            return
        if not shuttle.payload_gazebo_spawned:
            return
        now = self.get_clock().now()
        last_sent = shuttle.last_payload_gazebo_set_pose_time
        if last_sent is not None:
            elapsed = (now - last_sent).nanoseconds / 1e9
            if elapsed < self.gazebo_set_pose_period:
                return
        if shuttle.pending_payload_set_pose is not None:
            if shuttle.pending_payload_set_pose.done():
                shuttle.pending_payload_set_pose = None
                shuttle.pending_payload_set_pose_wall_time = None
                shuttle.payload_set_pose_timeout_warning_logged = False
            else:
                pending_since = shuttle.pending_payload_set_pose_wall_time
                pending_age_s = (
                    time.monotonic() - pending_since
                    if pending_since is not None
                    else 0.0
                )
                if pending_age_s < self.gazebo_set_pose_timeout_s:
                    return
                if not shuttle.payload_set_pose_timeout_warning_logged:
                    self.get_logger().warn(
                        f'Gazebo set_pose request for {shuttle.payload_entity_name} '
                        f'was pending for {pending_age_s:.2f}s; sending newest pose.'
                    )
                    shuttle.payload_set_pose_timeout_warning_logged = True
                shuttle.pending_payload_set_pose = None
                shuttle.pending_payload_set_pose_wall_time = None
        if not self.set_pose_client.service_is_ready():
            if not shuttle.payload_set_pose_warning_logged:
                self.get_logger().warn(
                    'Gazebo set_pose service is not ready yet; still publishing payload state.'
                )
                shuttle.payload_set_pose_warning_logged = True
            return

        payload_pose = self._payload_pose_from_shuttle_pose(shuttle_pose)
        request = SetEntityPose.Request()
        request.entity.name = shuttle.payload_entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = payload_pose.x
        request.pose.position.y = payload_pose.y
        request.pose.position.z = payload_pose.z
        qx, qy, qz, qw = _yaw_to_quaternion(payload_pose.yaw)
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw
        shuttle.pending_payload_set_pose = self.set_pose_client.call_async(request)
        shuttle.pending_payload_set_pose_wall_time = time.monotonic()
        shuttle.last_payload_gazebo_set_pose_time = now

    def _resolve_allowed_start_slot(
        self,
        raw_slot: str,
        tolerance_m: float,
    ) -> tuple[str, AllowedStartPose, float, str, float]:
        slot = self._normalize_start_slot(raw_slot)
        start_pose = self.allowed_start_poses[slot]
        slot_device = self.rail_devices.slots.get(slot)
        if slot_device is not None:
            return slot, start_pose, 0.0, slot_device.segment, slot_device.s

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

        return apply_rail_point_calibration(x, y, z, self)

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

    def _on_stopper_command(
        self,
        message: StopperCommand,
    ) -> None:
        try:
            updates = self._stopper_updates_from_named_states(message.stoppers)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self._handle_stopper_updates(updates, source='typed command')

    def _handle_stopper_updates(
        self,
        updates: Dict[str, str],
        *,
        source: str,
    ) -> None:
        self._schedule_stopper_state_updates(
            updates,
            source=source,
        )
        self._publish_stopper_state()

    def _schedule_switch_state_updates(
        self,
        updates: Dict[str, str],
        *,
        source: str,
    ) -> Dict[str, str]:
        return self._schedule_discrete_state_updates(
            actual_states=self.switch_states,
            pending_updates=self.pending_switch_state_updates,
            updates=updates,
            delay_s=self.switch_motion_delay_s,
            label='switch',
            source=source,
        )

    def _schedule_stopper_state_updates(
        self,
        updates: Dict[str, str],
        *,
        source: str,
    ) -> Dict[str, str]:
        return self._schedule_discrete_state_updates(
            actual_states=self.stopper_states,
            pending_updates=self.pending_stopper_state_updates,
            updates=updates,
            delay_s=self.stopper_motion_delay_s,
            label='stopper',
            source=source,
        )

    def _schedule_discrete_state_updates(
        self,
        *,
        actual_states: Dict[str, str],
        pending_updates: Dict[str, PendingDiscreteStateUpdate],
        updates: Dict[str, str],
        delay_s: float,
        label: str,
        source: str,
    ) -> Dict[str, str]:
        if not updates:
            return {}

        now_s = self._state_update_time_s()
        apply_at_s = now_s + max(0.0, delay_s)
        immediate_updates: Dict[str, str] = {}
        scheduled_updates: Dict[str, str] = {}
        cancelled_updates: Dict[str, str] = {}

        for name, target_state in updates.items():
            current_state = actual_states.get(name)
            pending_update = pending_updates.get(name)
            if delay_s <= 0.0:
                pending_updates.pop(name, None)
                if current_state != target_state:
                    immediate_updates[name] = target_state
                continue

            if current_state == target_state:
                if pending_update is not None:
                    pending_updates.pop(name, None)
                    cancelled_updates[name] = target_state
                continue

            if pending_update is not None and pending_update.target_state == target_state:
                continue

            pending_updates[name] = PendingDiscreteStateUpdate(
                target_state=target_state,
                apply_at_s=apply_at_s,
                source=source,
            )
            scheduled_updates[name] = target_state

        if immediate_updates:
            actual_states.update(immediate_updates)
            self.get_logger().info(
                f'Applied {label} state updates immediately from {source}: '
                f'{self._public_switch_state_map(immediate_updates)}'
            )

        if scheduled_updates:
            self.get_logger().info(
                f'Scheduled {label} state updates from {source} after '
                f'{delay_s:.3f}s: {self._public_switch_state_map(scheduled_updates)}'
            )

        if cancelled_updates:
            self.get_logger().info(
                f'Cancelled pending {label} state updates because the requested '
                f'state is already actual: {self._public_switch_state_map(cancelled_updates)}'
            )

        return immediate_updates

    def _stopper_updates_from_named_states(
        self,
        named_states,
    ) -> Dict[str, str]:
        return self._stopper_updates_from_assignments(
            [(named_state.name, named_state.state) for named_state in named_states]
        )

    def _stopper_updates_from_assignments(
        self,
        assignments: list[tuple[str, str]],
    ) -> Dict[str, str]:
        updates: Dict[str, str] = {}
        for raw_selector, raw_state in assignments:
            selector = raw_selector.strip().upper()
            if selector == 'ALL':
                state = self._normalize_stopper_state(raw_state)
                for stopper_name in self.stopper_configs:
                    updates[stopper_name] = state

        for raw_selector, raw_state in assignments:
            selector = raw_selector.strip().upper()
            if selector != 'ALL':
                state = self._normalize_stopper_state(raw_state)
                if selector not in self.stopper_configs:
                    allowed = ', '.join(['ALL', *sorted(self.stopper_configs)])
                    raise ValueError(f'Unknown stopper selector {selector!r}; use {allowed}.')
                updates[selector] = state
        return updates

    @staticmethod
    def _normalize_stopper_state(raw_state: str) -> str:
        state = str(raw_state).strip().upper()
        if state in {'1', 'ON', 'STOP', 'STOPPED', 'CLOSED', 'BLOCK', 'BLOCKED', 'TRUE'}:
            return STOPPER_STOP_STATE
        if state in {'0', 'OFF', 'OPEN', 'PASS', 'RELEASE', 'UNSTOP', 'UNBLOCK', 'FALSE'}:
            return STOPPER_PASS_STATE
        raise ValueError(
            f'Unknown stopper state {raw_state!r}; use 1/STOP/CLOSED or 0/PASS/OPEN.'
        )

    def _on_shuttle_control_command(self, message: RailShuttleCommand) -> None:
        try:
            updates = self._parse_shuttle_control_typed_command(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self._apply_shuttle_control_updates(updates)

    def _apply_shuttle_control_updates(
        self,
        updates: Dict[str, tuple[str, float | None, str]],
    ) -> None:
        applied_updates: Dict[str, str] = {}
        for entity_name, (action, speed, target_slot) in updates.items():
            shuttle = self._find_shuttle(entity_name)
            if shuttle is None:
                self.get_logger().warn(
                    f'Unknown shuttle {entity_name!r}; command ignored.'
                )
                continue
            try:
                self._apply_shuttle_action(
                    shuttle,
                    action,
                    speed,
                    target_slot=target_slot,
                )
                applied_updates[entity_name] = action
                if speed is not None:
                    applied_updates[entity_name] += f'@{speed:.3f}m/s'
                if target_slot:
                    applied_updates[entity_name] += f'->slot{target_slot}'
            except ValueError as error:
                self.get_logger().error(str(error))

        if applied_updates:
            self.get_logger().info(
                f'Applied shuttle commands: {applied_updates}; '
                f'shuttles={self._shuttle_summary()}'
            )

    def _parse_shuttle_control_typed_command(
        self,
        command: RailShuttleCommand,
    ) -> Dict[str, tuple[str, float | None, str]]:
        raw_selector = command.name.strip()
        raw_action = command.command.strip()
        if not raw_action:
            raise ValueError(
                'Typed shuttle control command must set command=ON/OFF/RESET/REMOVE.'
            )
        action = self._normalize_shuttle_action(raw_action)
        speed = float(command.speed) if command.speed > 0.0 else None
        raw_target_slot = str(command.target_slot or '').strip()
        target_slot = (
            self._normalize_start_slot(raw_target_slot)
            if raw_target_slot
            else ''
        )
        if target_slot and action != 'ENABLE':
            raise ValueError('target_slot is valid only with an ON command.')
        update = (action, speed, target_slot)
        if raw_selector.upper() == 'ALL':
            return {shuttle.entity_name: update for shuttle in self.shuttles}
        if not raw_selector:
            if len(self.shuttles) == 1:
                return {self.shuttles[0].entity_name: update}
            raise ValueError(
                'Typed shuttle control command must set name, or use name=ALL.'
        )
        return {raw_selector: update}

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
        if state in {'REMOVE', 'DELETE', 'REMOVE_SHUTTLE', 'DELETE_SHUTTLE', 'ERASE', 'DROP'}:
            return 'REMOVE'
        return 'ENABLE' if cls._normalize_enabled_state(raw_state) else 'DISABLE'

    def _apply_shuttle_action(
        self,
        shuttle: ManagedShuttle,
        action: str,
        speed: float | None = None,
        *,
        target_slot: str = '',
    ) -> None:
        if action == 'ENABLE':
            self._set_shuttle_enabled(
                shuttle,
                True,
                speed,
                target_slot=target_slot,
            )
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

    def _set_shuttle_enabled(
        self,
        shuttle: ManagedShuttle,
        enabled: bool,
        speed: float | None = None,
        *,
        target_slot: str = '',
    ) -> None:
        target_device = None
        if enabled and target_slot:
            target_device = self.rail_devices.slots.get(str(target_slot))
            if target_device is None:
                raise ValueError(
                    f'Unknown target_slot {target_slot!r}; use slot 1, 2, 3, or 4.'
                )
        shuttle.enabled = enabled
        shuttle.blocked_by = None
        shuttle.collision_distance_m = None
        if not enabled:
            shuttle.motion_target_slot = ''
            shuttle.motion_target_segment = ''
            shuttle.motion_target_s = None
            shuttle.core.state.mode = WAITING
            shuttle.stopped_by = 'NOT_DEPLOYED' if not shuttle.deployed else 'DISABLED'
            shuttle.stopper_distance_m = 0.0
            return

        shuttle.reached_target_slot = ''
        shuttle.motion_target_slot = ''
        shuttle.motion_target_segment = ''
        shuttle.motion_target_s = None
        if target_device is not None:
            shuttle.motion_target_slot = str(target_slot)
            shuttle.motion_target_segment = str(target_device.segment)
            shuttle.motion_target_s = float(target_device.s)

        if not shuttle.deployed:
            shuttle.deployed = True
            shuttle.pending_set_pose = None
            shuttle.pending_set_pose_wall_time = None
            shuttle.last_gazebo_set_pose_time = None
            shuttle.set_pose_timeout_warning_logged = False
            shuttle.stopped_by = None
            shuttle.stopper_distance_m = None

        if shuttle.stopped_by in {'DISABLED', 'NOT_DEPLOYED'}:
            shuttle.stopped_by = None
            shuttle.stopper_distance_m = None
        if speed is not None:
            shuttle.core.state.speed = speed
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
        shuttle.start_segment = initial_segment
        shuttle.start_s = initial_s
        shuttle.sensor_markers_armed = False
        shuttle.motion_target_slot = ''
        shuttle.motion_target_segment = ''
        shuttle.motion_target_s = None
        shuttle.reached_target_slot = ''
        shuttle.previous_sensor_segment = initial_segment
        shuttle.previous_sensor_s = initial_s
        shuttle.core.state = ShuttleState(
            current_segment=initial_segment,
            s=initial_s,
            speed=shuttle.core.state.speed,
            mode=MOVING if shuttle.enabled and shuttle.core.state.speed > 0.0 else WAITING,
        )
        shuttle.blocked_by = None
        shuttle.collision_distance_m = None
        shuttle.pending_set_pose = None
        shuttle.pending_set_pose_wall_time = None
        shuttle.last_gazebo_set_pose_time = None
        shuttle.set_pose_timeout_warning_logged = False
        shuttle.pending_payload_set_pose = None
        shuttle.pending_payload_set_pose_wall_time = None
        shuttle.last_payload_gazebo_set_pose_time = None
        shuttle.payload_set_pose_timeout_warning_logged = False
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
        if (
            shuttle.pending_payload_spawn is not None
            and not shuttle.pending_payload_spawn.done()
        ):
            raise ValueError(
                f'Cannot remove {shuttle.entity_name} while its payload spawn request is still in flight.'
            )

        if self._find_shuttle(shuttle.entity_name) is None:
            return

        if shuttle.payload_gazebo_spawned:
            self._request_payload_delete_if_needed(shuttle)

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

    def _on_switch_command(
        self,
        message: SwitchCommand,
    ) -> None:
        try:
            updates = self._switch_updates_from_named_states(message.switches)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self._handle_switch_updates(updates, source='typed command')

    def _handle_switch_updates(
        self,
        updates: Dict[str, str],
        *,
        source: str,
    ) -> None:
        if not updates:
            return

        immediate_updates = self._schedule_switch_state_updates(
            updates,
            source=source,
            )
        if immediate_updates:
            self._publish_visual_switch_actual_updates(
                immediate_updates,
                source=source,
            )
        self._publish_switch_state()

    def _on_visual_switch_state(self, message: String) -> None:
        updates = self._parse_visual_switch_state_summary(message.data)
        if not updates:
            return

        changed = {}
        for switch_name, state in updates.items():
            pending_update = self.pending_switch_state_updates.get(switch_name)
            if (
                self.switch_states.get(switch_name) != state
                and (
                    pending_update is None
                    or pending_update.target_state != state
                )
            ):
                changed[switch_name] = state
        if changed:
            immediate_updates = self._schedule_switch_state_updates(
                changed,
                source='visual state sync',
            )
            self._publish_switch_state()
            self.get_logger().info(
                'Received visual switch state sync request: '
                f'{_ordered_switch_states(changed)}'
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

    def _switch_updates_from_named_states(
        self,
        named_states,
    ) -> Dict[str, str]:
        updates, _visual_command = self._switch_updates_from_assignments(
            [(named_state.name, named_state.state) for named_state in named_states]
        )
        return updates

    def _switch_updates_from_assignments(
        self,
        assignments: list[tuple[str, str]],
    ) -> tuple[Dict[str, str], str]:
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

            if selector_name in ('ALL', 'RIGHT', 'LEFT', self.active_visual_group_selector):
                for switch_name in logic_targets:
                    updates[switch_name] = state

            if visual_selector:
                visual_entries.append((visual_selector, state))

        for selector, raw_state in assignments:
            selector_name = selector.strip().upper()
            if selector_name not in ('ALL', 'RIGHT', 'LEFT', self.active_visual_group_selector):
                state = self._normalize_commanded_switch_state(raw_state)
                logic_targets = self._logic_targets_for_selector(selector_name)
                for switch_name in logic_targets:
                    updates[switch_name] = state

        visual_command_parts = [
            f'{selector}={self._visual_mode_for_state(state)}'
            for selector, state in visual_entries
        ]

        return updates, ', '.join(visual_command_parts)

    def _normalize_commanded_switch_state(self, raw_state: str) -> str:
        state = raw_state.strip().upper()
        if state in {'E', 'EXTERIOR'}:
            return SWITCH_EXTERIOR_STATE
        if state in {'I', 'INTERIOR'}:
            return SWITCH_INTERIOR_STATE
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
        return 'EXTERIOR' if state == SWITCH_EXTERIOR_STATE else 'INTERIOR'

    def _apply_due_pending_state_updates(self) -> None:
        due_switch_updates = self._pop_due_discrete_state_updates(
            self.pending_switch_state_updates
        )
        switch_updates = {
            name: pending_update.target_state
            for name, pending_update in due_switch_updates.items()
        }
        if switch_updates:
            self.switch_states.update(switch_updates)
            visual_updates = {
                name: pending_update.target_state
                for name, pending_update in due_switch_updates.items()
                if pending_update.source != 'visual state sync'
            }
            self._publish_visual_switch_actual_updates(
                visual_updates,
                source='motion delay',
            )
            self.get_logger().info(
                'Applied actual switch states after motion delay: '
                f'{self._public_switch_state_map(self.switch_states)}'
            )

        due_stopper_updates = self._pop_due_discrete_state_updates(
            self.pending_stopper_state_updates
        )
        stopper_updates = {
            name: pending_update.target_state
            for name, pending_update in due_stopper_updates.items()
        }
        if stopper_updates:
            self.stopper_states.update(stopper_updates)
            self.get_logger().info(
                'Applied actual stopper states after motion delay: '
                f'{self._public_switch_state_map(self.stopper_states)}'
            )

    def _publish_visual_switch_actual_updates(
        self,
        updates: Dict[str, str],
        *,
        source: str,
    ) -> None:
        if not self.publish_visual_switch_commands or not updates:
            return

        visual_entries = []
        for switch_name, state in self._public_switch_state_map(updates).items():
            visual_selector = self._visual_selector_for_selector(switch_name)
            if visual_selector is None:
                continue
            visual_entries.append(
                f'{visual_selector}={self._visual_mode_for_state(state)}'
            )

        if not visual_entries:
            return

        visual_command = ', '.join(visual_entries)
        visual_message = String()
        visual_message.data = visual_command
        self.visual_switch_publisher.publish(visual_message)
        self.get_logger().info(
            f'Published visual switch command from actual state ({source}): '
            f'{visual_command}'
        )

    def _state_update_time_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _pop_due_discrete_state_updates(
        self,
        pending_updates: Dict[str, PendingDiscreteStateUpdate],
    ) -> Dict[str, PendingDiscreteStateUpdate]:
        now_s = self._state_update_time_s()
        due_updates: Dict[str, PendingDiscreteStateUpdate] = {}
        for name, pending_update in list(pending_updates.items()):
            if pending_update.apply_at_s > now_s:
                continue
            due_updates[name] = pending_update
            pending_updates.pop(name, None)
        return due_updates

    def _fill_header(self, message) -> None:
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

    def _named_states_from_map(self, raw_states: Dict[str, str]) -> list[NamedState]:
        return [
            NamedState(name=name, state=state)
            for name, state in self._public_switch_state_map(raw_states).items()
        ]

    def _publish_switch_state(self) -> None:
        message = RailSwitchState()
        self._fill_header(message)
        message.switches = self._named_states_from_map(self.switch_states)
        self.switch_state_publisher.publish(message)

    def _publish_stopper_state(self) -> None:
        message = RailStopperState()
        self._fill_header(message)
        message.stoppers = self._named_states_from_map(self.stopper_states)
        self._update_stopper_marker_states()
        self.stopper_state_publisher.publish(message)

    def _reset_sensor_timer(self) -> None:
        if self.sensor_timer is not None:
            self.destroy_timer(self.sensor_timer)
            self.sensor_timer = None
        if self.sync_sensor_feedback_to_motion_tick:
            return
        self.sensor_timer = self.create_timer(
            1.0 / self.sensor_publish_rate_hz,
            self._publish_all_sensor_feedback,
        )

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
            'gazebo_set_pose_timeout_s',
            'shuttle_collision_distance_m',
            'collision_search_iterations',
            'start_slot_occupancy_radius_m',
            'switch_motion_delay_s',
            'stopper_motion_delay_s',
            'sensor_publish_rate_hz',
            'device_marker_refresh_grace_period_s',
            'sensor_marker_visual_hold_s',
        }
        boolean_parameters = {
            'enable_collision_avoidance',
            'enable_gazebo_pose_transform',
            'device_marker_dynamic_refresh',
            'publish_visual_switch_commands',
            'reject_occupied_start_slots',
            'show_device_markers',
            'sync_sensor_feedback_to_motion_tick',
        }

        try:
            for parameter in parameters:
                if parameter.name in numeric_parameters:
                    if parameter.name == 'gazebo_set_pose_rate_hz':
                        rate = float(parameter.value)
                        self.gazebo_set_pose_period = 1.0 / max(rate, 1.0)
                        self.gazebo_set_pose_timeout_s = max(
                            self.gazebo_set_pose_period,
                            getattr(self, 'gazebo_set_pose_timeout_s', 0.5),
                        )
                    elif parameter.name == 'gazebo_set_pose_timeout_s':
                        timeout_s = float(parameter.value)
                        if timeout_s <= 0.0:
                            raise ValueError(
                                'gazebo_set_pose_timeout_s must be greater than 0.0.'
                            )
                        self.gazebo_set_pose_timeout_s = max(
                            self.gazebo_set_pose_period,
                            timeout_s,
                        )
                    elif parameter.name == 'collision_search_iterations':
                        self.collision_search_iterations = max(1, int(parameter.value))
                    elif parameter.name in {
                        'switch_motion_delay_s',
                        'stopper_motion_delay_s',
                    }:
                        value = float(parameter.value)
                        if value < 0.0:
                            raise ValueError(
                                f'{parameter.name} must be greater than or equal to 0.0.'
                            )
                        setattr(self, parameter.name, value)
                    elif parameter.name == 'sensor_publish_rate_hz':
                        rate = float(parameter.value)
                        if rate <= 0.0:
                            raise ValueError('sensor_publish_rate_hz must be greater than 0.0.')
                        self.sensor_publish_rate_hz = rate
                        self._reset_sensor_timer()
                    elif parameter.name == 'sensor_marker_visual_hold_s':
                        hold_s = float(parameter.value)
                        if hold_s < 0.0:
                            raise ValueError(
                                'sensor_marker_visual_hold_s must be greater than '
                                'or equal to 0.0.'
                            )
                        self.sensor_marker_visual_hold_s = hold_s
                    else:
                        setattr(self, parameter.name, float(parameter.value))
                elif parameter.name in boolean_parameters:
                    if parameter.name == 'show_device_markers':
                        self.enable_device_markers = bool(parameter.value)
                    elif parameter.name == 'sync_sensor_feedback_to_motion_tick':
                        self.sync_sensor_feedback_to_motion_tick = bool(
                            parameter.value
                        )
                        self._reset_sensor_timer()
                    else:
                        setattr(self, parameter.name, bool(parameter.value))
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_tick).nanoseconds / 1e9)
        self.last_tick = now
        self._update_device_markers()
        self._apply_due_pending_state_updates()
        self._update_shuttle_visuals()

        if not self.shuttles:
            self._publish_state([], [])
            self._publish_payload_state()
            self._publish_switch_state()
            self._publish_stopper_state()
            if self.sync_sensor_feedback_to_motion_tick:
                self._publish_all_sensor_feedback()
            return

        raw_poses = []
        gazebo_poses = []
        occupied_poses = {
            shuttle.entity_name: self._to_gazebo_pose(shuttle.core.pose())
            for shuttle in self.shuttles
            if shuttle.deployed
        }
        for shuttle in self.shuttles:
            self._capture_sensor_sweep_start(shuttle)
            if not shuttle.deployed:
                shuttle.blocked_by = None
                shuttle.collision_distance_m = None
                shuttle.stopped_by = 'NOT_DEPLOYED'
                shuttle.stopper_distance_m = 0.0
                raw_poses.append(shuttle.core.pose())
                hidden_pose = self._hidden_gazebo_pose(shuttle)
                gazebo_poses.append(hidden_pose)
                self._update_payload_visual(shuttle, hidden_pose)
                continue

            if not self._spawn_ready_for_motion(shuttle):
                pose = shuttle.core.pose()
                gazebo_pose = self._to_gazebo_pose(pose)
                occupied_poses[shuttle.entity_name] = gazebo_pose
                raw_poses.append(pose)
                gazebo_poses.append(gazebo_pose)
                self._update_payload_visual(shuttle, gazebo_pose)
                continue

            pose = self._step_with_motion_guards(
                shuttle=shuttle,
                dt=dt,
                occupied_poses=occupied_poses,
            )
            gazebo_pose = self._to_gazebo_pose(pose)
            pose_message = self._publish_pose(shuttle, gazebo_pose)
            self._send_gazebo_pose(shuttle, pose_message)
            self._update_payload_visual(shuttle, gazebo_pose)
            occupied_poses[shuttle.entity_name] = gazebo_pose
            raw_poses.append(pose)
            gazebo_poses.append(gazebo_pose)

            if pose.mode == FALLING:
                self.get_logger().error(
                    f'Shuttle {shuttle.entity_name} entered FALLING mode at '
                    f'segment={self._public_segment_name(pose.current_segment)}, s={pose.s:.3f}'
                )

        self._publish_state(raw_poses, gazebo_poses)
        self._publish_payload_state()
        self._publish_switch_state()
        self._publish_stopper_state()
        if self.sync_sensor_feedback_to_motion_tick:
            self._publish_all_sensor_feedback()
        self._update_shuttle_visuals()

    @staticmethod
    def _capture_sensor_sweep_start(shuttle: ManagedShuttle) -> None:
        state = shuttle.core.state
        shuttle.previous_sensor_segment = state.current_segment
        shuttle.previous_sensor_s = state.s

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
        active_target = self._active_motion_target_ahead(shuttle)
        effective_dt = dt
        stop_reached = False
        target_reached = False
        if active_stop is not None and shuttle.core.state.speed > 0.0:
            stopper_name, stop_point, target_s, distance_m = active_stop
            if distance_m <= 1e-6:
                shuttle.core.state.s = target_s
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

        if active_target is not None and shuttle.core.state.speed > 0.0:
            target_slot, target_segment, target_s, distance_m = active_target
            time_to_target = distance_m / shuttle.core.state.speed
            if time_to_target <= effective_dt:
                effective_dt = time_to_target
                stop_reached = False
                target_reached = True

        pose = self._step_with_collision_avoidance(
            shuttle=shuttle,
            dt=effective_dt,
            occupied_poses=occupied_poses,
        )
        if shuttle.blocked_by is not None or pose.mode == FALLING:
            return pose

        if target_reached and active_target is not None:
            target_slot, target_segment, target_s, _distance_m = active_target
            if shuttle.core.state.current_segment == target_segment:
                shuttle.core.state.s = target_s
            shuttle.core.state.mode = WAITING
            shuttle.enabled = False
            shuttle.reached_target_slot = target_slot
            shuttle.stopped_by = f'TARGET_SLOT_{target_slot}'
            shuttle.stopper_distance_m = 0.0
            return shuttle.core.pose()

        if stop_reached and active_stop is not None:
            stopper_name, stop_point, target_s, _distance_m = active_stop
            if shuttle.core.state.current_segment == stop_point.segment:
                shuttle.core.state.s = target_s
            shuttle.core.state.mode = WAITING
            shuttle.stopped_by = stopper_name
            shuttle.stopper_distance_m = 0.0
            return shuttle.core.pose()

        shuttle.stopped_by = None
        shuttle.stopper_distance_m = None
        return pose

    @staticmethod
    def _active_motion_target_ahead(
        shuttle: ManagedShuttle,
    ) -> tuple[str, str, float, float] | None:
        """Return the exact commanded slot setpoint when it is ahead.

        This is low-level trajectory control using the controller's internal
        state. It does not publish or substitute a visual localization fact.
        """

        if (
            not shuttle.motion_target_slot
            or not shuttle.motion_target_segment
            or shuttle.motion_target_s is None
        ):
            return None
        state = shuttle.core.state
        if state.current_segment != shuttle.motion_target_segment:
            return None
        distance_m = float(shuttle.motion_target_s) - float(state.s)
        if distance_m < -1e-9:
            return None
        return (
            shuttle.motion_target_slot,
            shuttle.motion_target_segment,
            float(shuttle.motion_target_s),
            max(distance_m, 0.0),
        )

    def _active_stopper_ahead(
        self,
        shuttle: ManagedShuttle,
    ) -> tuple[str, StopPoint, float, float] | None:
        state = shuttle.core.state
        candidates: list[tuple[str, StopPoint, float, float]] = []
        for stopper_name, stopper_config in self.stopper_configs.items():
            if self.stopper_states.get(stopper_name, STOPPER_PASS_STATE) != STOPPER_STOP_STATE:
                continue
            for stop_point in stopper_config.stop_points:
                if stop_point.segment != state.current_segment:
                    continue
                physical_distance_m = stop_point.stop_s - state.s
                if physical_distance_m < -1e-6:
                    continue

                target_s = stop_point.trigger_s
                distance_m = target_s - state.s
                if distance_m < -1e-6:
                    target_s = state.s
                    distance_m = 0.0
                candidates.append((stopper_name, stop_point, target_s, distance_m))
        if not candidates:
            return None
        return sorted(candidates, key=lambda item: item[3])[0]

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

        x, y, z, yaw = apply_rail_pose_calibration(
            pose.x,
            pose.y,
            pose.z,
            pose.yaw,
            self,
        )

        return ShuttlePose(
            x=x,
            y=y,
            z=z,
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
        if shuttle.pending_set_pose is not None:
            if shuttle.pending_set_pose.done():
                shuttle.pending_set_pose = None
                shuttle.pending_set_pose_wall_time = None
                shuttle.set_pose_timeout_warning_logged = False
            else:
                pending_since = shuttle.pending_set_pose_wall_time
                pending_age_s = (
                    time.monotonic() - pending_since
                    if pending_since is not None
                    else 0.0
                )
                if pending_age_s < self.gazebo_set_pose_timeout_s:
                    return
                if not shuttle.set_pose_timeout_warning_logged:
                    self.get_logger().warn(
                        f'Gazebo set_pose request for {shuttle.entity_name} '
                        f'was pending for {pending_age_s:.2f}s; sending newest pose.'
                    )
                    shuttle.set_pose_timeout_warning_logged = True
                shuttle.pending_set_pose = None
                shuttle.pending_set_pose_wall_time = None
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
        shuttle.pending_set_pose_wall_time = time.monotonic()
        shuttle.last_gazebo_set_pose_time = now

    def _shuttle_on_sensor(
        self,
        segment: str,
        sensor_s: float,
        radius_m: float,
    ) -> ManagedShuttle | None:
        """Rail sensors are binary occupancy sensors; YAML radius defines detection."""
        for shuttle in self.shuttles:
            if not shuttle.deployed:
                continue
            state = shuttle.core.state
            if state.current_segment != segment:
                continue
            if abs(state.s - sensor_s) <= radius_m:
                return shuttle
        return None

    def _position_sensor_readings(self) -> list[SensorReading]:
        readings = []
        for sensor_name, sensor_config in self.position_sensor_configs.items():
            active_shuttle = None
            active_point = None
            fallback_point = sensor_config.points[0]
            for point in sensor_config.points:
                # Sensor feedback is continuous binary occupancy. Marker
                # arming exists only to suppress startup visualization and
                # must never suppress the physical sensor message itself.
                active_shuttle = self._shuttle_on_sensor(
                    point.segment,
                    point.sensor_s,
                    point.radius_m,
                )
                if active_shuttle is None:
                    active_shuttle = self._shuttle_crossed_sensor_since_last_tick(
                        point.segment,
                        point.sensor_s,
                        point.radius_m,
                    )
                if active_shuttle is not None:
                    active_point = point
                    break
            point = active_point or fallback_point
            segment_length = self.network.segments[point.segment].length
            readings.append(
                SensorReading(
                    name=self._public_sensor_name(sensor_name),
                    sensor_type=RAIL_SENSOR_TYPE,
                    active=1 if active_shuttle is not None else 0,
                    shuttle_name='' if active_shuttle is None else active_shuttle.entity_name,
                    segment=self._public_segment_name(point.segment),
                    s=point.sensor_s,
                    s_ratio=(
                        point.sensor_s / segment_length
                        if segment_length > 0.0
                        else 0.0
                    ),
                )
            )
        return readings

    def _publish_sensor_feedback(self) -> None:
        message = SensorFeedback()
        self._fill_header(message)
        message.readings = self._position_sensor_readings()
        self._update_sensor_marker_states()
        self.sensor_feedback_publisher.publish(message)

    def _publish_all_sensor_feedback(self) -> None:
        self._publish_sensor_feedback()

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
        ordered_states = _ordered_switch_states(raw_states)
        return {
            name: state
            for name, state in ordered_states.items()
        }

    def _publish_state(
        self,
        raw_poses: list[ShuttlePose],
        gazebo_poses: list[ShuttlePose],
    ) -> None:
        if not self.shuttles:
            self.state_publisher.publish(self._make_shuttle_state_message({}))
            return

        published_count = 0
        for shuttle, raw_pose, gazebo_pose in zip(self.shuttles, raw_poses, gazebo_poses):
            pose_payload = asdict(raw_pose)
            pose_payload['current_segment'] = self._public_segment_name(
                raw_pose.current_segment
            )
            gazebo_pose_payload = asdict(gazebo_pose)
            gazebo_pose_payload['current_segment'] = self._public_segment_name(
                gazebo_pose.current_segment
            )
            pose_payload.update(
                {
                    'entity_name': shuttle.entity_name,
                    'gazebo_pose': gazebo_pose_payload,
                    'speed': shuttle.core.state.speed,
                    'start_slot': shuttle.start_slot,
                    'start_snap_distance_m': shuttle.start_snap_distance_m,
                    'reached_target_slot': shuttle.reached_target_slot,
                }
            )
            self.state_publisher.publish(self._make_shuttle_state_message(pose_payload))
            published_count += 1

        if published_count >= len(self.shuttles):
            return

        for shuttle in self.shuttles[published_count:]:
            fallback_pose = shuttle.core.pose()
            fallback_payload = {
                'current_segment': self._public_segment_name(fallback_pose.current_segment),
                'entity_name': shuttle.entity_name,
                'gazebo_pose': None,
                'mode': fallback_pose.mode,
                's': fallback_pose.s,
                'speed': shuttle.core.state.speed,
                'start_slot': shuttle.start_slot,
                'start_snap_distance_m': shuttle.start_snap_distance_m,
                'reached_target_slot': shuttle.reached_target_slot,
                'x': fallback_pose.x,
                'y': fallback_pose.y,
                'yaw': fallback_pose.yaw,
                'z': fallback_pose.z,
            }
            self.state_publisher.publish(self._make_shuttle_state_message(fallback_payload))

    def _make_shuttle_state_message(self, payload: dict) -> RailShuttleState:
        message = RailShuttleState()
        self._fill_header(message)
        message.name = str(payload.get('entity_name') or '')
        message.mode = str(payload.get('mode') or '')
        message.current_segment = str(payload.get('current_segment') or '')
        message.s = float(payload.get('s') or 0.0)
        message.x = float(payload.get('x') or 0.0)
        message.y = float(payload.get('y') or 0.0)
        message.z = float(payload.get('z') or 0.0)
        message.yaw = float(payload.get('yaw') or 0.0)
        message.speed = float(payload.get('speed') or 0.0)
        message.reached_target_slot = str(
            payload.get('reached_target_slot') or ''
        )
        return message


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
