#!/usr/bin/env python3

import base64
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_multi_shuttle import ACTION_SCHEMA_VERSION
from room_315_multi_shuttle import ACTION_VECTOR_V3_FIELDS
from room_315_multi_shuttle import COORDINATION_MODE_IDS
from room_315_multi_shuttle import REASON_IDS
from room_315_multi_shuttle import SIDE_IDS
from room_315_multi_shuttle import TARGET_IDS
from room_315_multi_shuttle import model_input_is_clean
from room_315_multi_shuttle import shuttle_specs_for_side


ALLOWED_ACTIONS = (
    'route_template',
    'route_shuttle',
    'switches',
    'stoppers',
    'shuttle',
    'add_shuttle',
    'stop_all',
    'emergency_stop',
    'clear_emergency_stop',
    'status',
)
MODEL_OUTPUT_ACTIONS = (
    'switches',
    'stoppers',
    'shuttle',
    'stop_all',
    'emergency_stop',
    'clear_emergency_stop',
    'status',
)
EVENT_ACTION_VECTOR_FIELDS = tuple(ACTION_VECTOR_V3_FIELDS)
EVENT_PRIMITIVE_IDS = {
    'WAIT': 0,
    'DONE': 1,
    'SET_SWITCHES': 2,
    'SET_STOPPERS': 3,
    'SHUTTLE_ON': 4,
    'STOP_NOW': 5,
    'EMERGENCY_STOP': 6,
}
SIDES = ('right', 'left')
DEVICE_NAMES = ('A1', 'A2', 'A3', 'A4')
SENSOR_IDS_BY_SIDE = {
    'right': (
        'DZI1R',
        'DZI2R',
        'DZI3R',
        'DZI4R',
        'DA1R',
        'DA1ER',
        'DA1IR',
        'DA2R',
        'DA2ER',
        'DA2IR',
        'DA3R',
        'DA3ER',
        'DA3IR',
        'DA4R',
        'DA4ER',
        'DA4IR',
        'A1_STOPPER_SENSOR',
        'A2_STOPPER_SENSOR',
        'A3_STOPPER_SENSOR',
        'A4_STOPPER_SENSOR',
    ),
    'left': (
        'DZI1L',
        'DZI2L',
        'DZI3L',
        'DZI4L',
        'DA1L',
        'DA1EL',
        'DA1IL',
        'DA2L',
        'DA2EL',
        'DA2IL',
        'DA3L',
        'DA3EL',
        'DA3IL',
        'DA4L',
        'DA4EL',
        'DA4IL',
        'A1_STOPPER_SENSOR',
        'A2_STOPPER_SENSOR',
        'A3_STOPPER_SENSOR',
        'A4_STOPPER_SENSOR',
    ),
}
MODEL_INPUT_SCHEMA_VERSION = 3
MODEL_INPUT_FIELDS = (
    'language',
    'overhead_images',
    'last_command',
    'observable_state',
)
OVERHEAD_IMAGE_NAMES = {'right_rail_rgb', 'left_rail_rgb'}


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _rails_from_status(status: dict[str, Any]) -> dict[str, Any]:
    rails = status.get('rails', {}) if isinstance(status.get('rails'), dict) else {}
    return rails


def _rail_from_status(status: dict[str, Any], side: str) -> dict[str, Any]:
    rail = _rails_from_status(status).get(side, {})
    return rail if isinstance(rail, dict) else {}


def _normalize_switch_state(raw: Any) -> str:
    state = str(raw or '').strip().upper()
    if state in {'E', 'EXTERIOR'}:
        return 'EXTERIOR'
    if state in {'I', 'INTERIOR'}:
        return 'INTERIOR'
    return 'UNKNOWN'


def _normalize_stopper_state(raw: Any) -> str:
    state = str(raw or '').strip().lower()
    if state in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return 'open'
    if state in {'1', 'closed', 'close', 'stop', 'blocked', 'on', 'true'}:
        return 'closed'
    return 'unknown'


def _active_sensor_ids_from_readings(raw_readings: Any) -> set[str]:
    sensor_ids: set[str] = set()
    if isinstance(raw_readings, dict):
        iterable = raw_readings.values()
    elif isinstance(raw_readings, list):
        iterable = raw_readings
    else:
        return sensor_ids
    for reading in iterable:
        if isinstance(reading, dict):
            name = str(reading.get('name') or reading.get('sensor') or '').strip()
        else:
            name = str(reading or '').strip()
        if name:
            sensor_ids.add(name.upper())
    return sensor_ids


def _active_sensor_ids(rail: dict[str, Any]) -> set[str]:
    return (
        _active_sensor_ids_from_readings(rail.get('active_sensors'))
        | _active_sensor_ids_from_readings(rail.get('active_position_sensors'))
    )


def _binary_sensor_bits(status: dict[str, Any]) -> dict[str, dict[str, int]]:
    bits: dict[str, dict[str, int]] = {}
    for side in SIDES:
        active_ids = _active_sensor_ids(_rail_from_status(status, side))
        bits[side] = {
            sensor_name: 1 if sensor_name in active_ids else 0
            for sensor_name in SENSOR_IDS_BY_SIDE[side]
        }
    return bits


def _switch_states(status: dict[str, Any]) -> dict[str, dict[str, str]]:
    states: dict[str, dict[str, str]] = {}
    for side in SIDES:
        rail = _rail_from_status(status, side)
        switches = rail.get('switches', {}) if isinstance(rail.get('switches'), dict) else {}
        states[side] = {
            name: _normalize_switch_state(switches.get(name))
            for name in DEVICE_NAMES
        }
    return states


def _stopper_states(status: dict[str, Any]) -> dict[str, dict[str, str]]:
    states: dict[str, dict[str, str]] = {}
    for side in SIDES:
        rail = _rail_from_status(status, side)
        stoppers = rail.get('stoppers', {}) if isinstance(rail.get('stoppers'), dict) else {}
        states[side] = {
            name: _normalize_stopper_state(stoppers.get(name))
            for name in DEVICE_NAMES
        }
    return states


def _observable_state_from_status(status: dict[str, Any]) -> dict[str, dict[str, dict[str, Any]]]:
    sensor_bits = _binary_sensor_bits(status)
    switch_states = _switch_states(status)
    stopper_states = _stopper_states(status)
    return {
        side: {
            'sensors': dict(sensor_bits.get(side, {})),
            'switches': dict(switch_states.get(side, {})),
            'stoppers': dict(stopper_states.get(side, {})),
        }
        for side in SIDES
    }


def _overhead_images(image_refs: dict[str, Any]) -> dict[str, Any]:
    return {
        camera_name: image_ref
        for camera_name, image_ref in image_refs.items()
        if camera_name in OVERHEAD_IMAGE_NAMES
    }


def _normalize_action_name(command: dict[str, Any]) -> str:
    action = str(command.get('action') or command.get('intent') or command.get('type') or 'status')
    action = action.lower().strip()
    if action in {'route', 'start_route'}:
        return 'route_shuttle'
    if action in {'template', 'task'}:
        return 'route_template'
    if action in {'switch'}:
        return 'switches'
    if action in {'stopper'}:
        return 'stoppers'
    if action in {'shuttle_command'}:
        return 'shuttle'
    if action in {'spawn_shuttle'}:
        return 'add_shuttle'
    if action in {'all_off'}:
        return 'stop_all'
    if action in {'estop'}:
        return 'emergency_stop'
    if action in {'clear_estop', 'reset_estop'}:
        return 'clear_emergency_stop'
    return action if action in ALLOWED_ACTIONS else 'status'


def _normalized_assignment_map(raw: Any, *, value_kind: str) -> dict[str, str]:
    if not isinstance(raw, dict):
        return {}
    normalized: dict[str, str] = {}
    for name, state in raw.items():
        device_name = str(name).strip().upper()
        if not device_name:
            continue
        if value_kind == 'switch':
            normalized[device_name] = _normalize_switch_state(state)
        elif value_kind == 'stopper':
            normalized[device_name] = _normalize_stopper_state(state)
        else:
            normalized[device_name] = str(state).strip()
    return normalized


def _normalize_last_command(command: Any) -> dict[str, Any]:
    if not isinstance(command, dict):
        return {'action': 'status'}
    action = _normalize_action_name(command)
    normalized: dict[str, Any] = {'action': action}
    if command.get('side') is not None:
        side = str(command.get('side') or '').strip().lower()
        if side in SIDES:
            normalized['side'] = side
    if action == 'route_template' and command.get('template') is not None:
        normalized['template'] = str(command.get('template') or '').strip()
    if action == 'route_shuttle':
        if command.get('loop') is not None:
            normalized['loop'] = str(command.get('loop') or '').strip().lower()
        if command.get('start') is not None:
            normalized['start'] = bool(command.get('start'))
        if command.get('start_slot') is not None:
            normalized['start_slot'] = str(command.get('start_slot') or '').strip()
    if action == 'switches':
        normalized['switches'] = _normalized_assignment_map(
            command.get('switches'),
            value_kind='switch',
        )
    if action == 'stoppers':
        normalized['stoppers'] = _normalized_assignment_map(
            command.get('stoppers'),
            value_kind='stopper',
        )
    if action == 'shuttle':
        normalized['command'] = str(command.get('command') or '').strip().upper()
        if command.get('shuttle') is not None:
            normalized['shuttle'] = str(command.get('shuttle') or '').strip()
        if command.get('start_slot') is not None:
            normalized['start_slot'] = str(command.get('start_slot') or '').strip()
    if action == 'stop_all' and command.get('close_stoppers') is not None:
        normalized['close_stoppers'] = bool(command.get('close_stoppers'))
    if command.get('speed') is not None and action in {'route_shuttle', 'shuttle'}:
        try:
            normalized['speed'] = float(command.get('speed') or 0.0)
        except (TypeError, ValueError):
            normalized['speed'] = 0.0
    return normalized


def _shuttle_command_state(status: dict[str, Any]) -> dict[str, dict[str, str]]:
    state = {
        side: {
            'last_command': 'UNKNOWN',
            'last_shuttle': '',
        }
        for side in SIDES
    }
    primitive = status.get('last_primitive_command')
    if not isinstance(primitive, dict):
        return state
    if str(primitive.get('action') or '').strip().lower() != 'shuttle':
        return state
    side = str(primitive.get('side') or '').strip().lower()
    if side not in SIDES:
        return state
    state[side] = {
        'last_command': str(primitive.get('command') or '').strip().upper() or 'UNKNOWN',
        'last_shuttle': str(primitive.get('shuttle') or '').strip(),
    }
    return state


def _time_since_last_sensor_event(
    sensor_event_times: dict[str, float | None] | None,
    now_s: float | None,
) -> dict[str, float | None]:
    now_value = time.monotonic() if now_s is None else float(now_s)
    times = sensor_event_times or {}
    elapsed: dict[str, float | None] = {}
    side_values = []
    for side in SIDES:
        event_time = times.get(side)
        if event_time is None:
            elapsed[side] = None
            continue
        value = round(max(now_value - float(event_time), 0.0), 3)
        elapsed[side] = value
        side_values.append(value)
    elapsed['any'] = None if not side_values else min(side_values)
    return elapsed


def _model_input_from_status(
    status: dict[str, Any],
    *,
    language: str,
    overhead_images: dict[str, Any],
    last_command: Any,
    sensor_event_times: dict[str, float | None] | None = None,
    now_s: float | None = None,
) -> dict[str, Any]:
    _ = sensor_event_times, now_s
    model_input = {
        'language': str(language or ''),
        'overhead_images': _overhead_images(overhead_images),
        'last_command': _normalize_last_command(last_command),
        'observable_state': _observable_state_from_status(status),
    }
    if not model_input_is_clean(model_input):
        raise ValueError(
            'Room 315 VLA model_input boundary violation: only language, overhead_images, '
            'last_command, and observable_state may be sent to the model'
        )
    return model_input



def _extract_response_text(payload: dict[str, Any]) -> str:
    output_text = payload.get('output_text')
    if isinstance(output_text, str) and output_text.strip():
        return output_text.strip()

    chunks: list[str] = []
    for item in payload.get('output', []):
        if not isinstance(item, dict):
            continue
        for content in item.get('content', []):
            if not isinstance(content, dict):
                continue
            if content.get('type') in {'output_text', 'text'} and isinstance(content.get('text'), str):
                chunks.append(content['text'])
    text = ''.join(chunks).strip()
    if text:
        return text
    raise ValueError('model response did not contain output text')


def _action_vector_schema_version(raw: Any) -> int | None:
    if not isinstance(raw, list):
        return None
    if len(raw) == len(EVENT_ACTION_VECTOR_FIELDS):
        version = ACTION_SCHEMA_VERSION
    else:
        return None
    try:
        [float(value) for value in raw]
    except (TypeError, ValueError):
        return None
    return version


def _is_action_vector(raw: Any) -> bool:
    return _action_vector_schema_version(raw) is not None


def _multi_shuttle_active(status: dict[str, Any]) -> bool:
    rails = _rails_from_status(status)
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        shuttles = rail.get('shuttles', {})
        if isinstance(shuttles, dict) and len(shuttles) > 1:
            return True
    return False


def _preferred_action_schema_version(status: dict[str, Any]) -> int:
    _ = status
    return ACTION_SCHEMA_VERSION


def _shuttle_index_mapping() -> dict[str, dict[str, int]]:
    return {
        side: {spec.short_id: spec.shuttle_index for spec in shuttle_specs_for_side(side, 4)}
        for side in SIDES
    }


def _forbidden_output_keys() -> set[str]:
    return {
        'model_input',
        'pddl_goal',
        'pddl_problem',
        'pddl_domain',
        'symbolic_plan',
        'plan_step_index',
        'structured_rail_state',
        'privileged_eval',
        'shuttle_tracks',
        'shuttle_identity_tracks',
        'fiducial_detections',
        'apriltag_detections',
        'target_shuttle_id',
        'rail_occupancy',
        'block_reservations',
        'gazebo_pose',
        'gazebo_poses',
        'loaded',
        'payload',
        'payload_condition',
        'payload_present',
        'payload_state',
        'payload_type',
    }


def _contains_forbidden_output_key(raw: Any) -> str:
    if isinstance(raw, dict):
        for key, value in raw.items():
            key_text = str(key).strip()
            if key_text in _forbidden_output_keys():
                return key_text
            nested = _contains_forbidden_output_key(value)
            if nested:
                return nested
    elif isinstance(raw, list):
        for item in raw:
            nested = _contains_forbidden_output_key(item)
            if nested:
                return nested
    return ''


def _movement_vector_missing_shuttle_index(action_vector: list[float]) -> bool:
    if _action_vector_schema_version(action_vector) != ACTION_SCHEMA_VERSION:
        return False
    primitive_id = int(round(float(action_vector[0])))
    if primitive_id not in {
        EVENT_PRIMITIVE_IDS['SHUTTLE_ON'],
        EVENT_PRIMITIVE_IDS['STOP_NOW'],
    }:
        return False
    return int(round(float(action_vector[2]))) < 0


def _multi_shuttle_json_command_is_ambiguous(command: dict[str, Any]) -> bool:
    action = str(command.get('action') or '').strip()
    if action != 'shuttle':
        return False
    command_name = str(command.get('command') or '').strip().upper()
    if command_name not in {'ON', 'OFF', 'RESET', 'REMOVE'}:
        return False
    return not any(
        command.get(key) is not None
        for key in ('shuttle', 'shuttle_id', 'name', 'shuttle_index')
    )


def _validate_provider_action_vector(
    raw_vector: Any,
    *,
    declared_schema_version: Any = None,
    multi_shuttle_active: bool = False,
) -> dict[str, Any]:
    version = _action_vector_schema_version(raw_vector)
    if version is None:
        raise ValueError('VLA provider returned an invalid action_vector length or value')
    if declared_schema_version is not None:
        try:
            declared = int(declared_schema_version)
        except (TypeError, ValueError):
            raise ValueError('action_vector_schema_version must be 3') from None
        if declared != ACTION_SCHEMA_VERSION:
            raise ValueError('action_vector_schema_version must be 3')
        if declared != version:
            raise ValueError(
                f'action_vector_schema_version={declared} does not match vector length schema {version}'
            )
    values = [float(value) for value in raw_vector]
    if _movement_vector_missing_shuttle_index(values):
        raise ValueError('schema-v3 movement action_vector requires shuttle_index')
    payload = {
        'action_vector': values,
        'action_vector_schema_version': ACTION_SCHEMA_VERSION,
    }
    return payload


def _parse_command_payload(raw: Any, *, multi_shuttle_active: bool = False) -> dict[str, Any]:
    forbidden = _contains_forbidden_output_key(raw)
    if forbidden:
        raise ValueError(f'VLA provider output included forbidden privileged field {forbidden!r}')
    if isinstance(raw, str):
        parsed = json.loads(raw)
    else:
        parsed = raw
    forbidden = _contains_forbidden_output_key(parsed)
    if forbidden:
        raise ValueError(f'VLA provider output included forbidden privileged field {forbidden!r}')
    if (
        isinstance(parsed, dict)
        and 'action' not in parsed
        and isinstance(parsed.get('command'), (dict, str))
    ):
        parsed = parsed['command']
        if isinstance(parsed, str):
            parsed = json.loads(parsed)
        forbidden = _contains_forbidden_output_key(parsed)
        if forbidden:
            raise ValueError(f'VLA provider output included forbidden privileged field {forbidden!r}')
    if _is_action_vector(parsed):
        return _validate_provider_action_vector(
            parsed,
            multi_shuttle_active=multi_shuttle_active,
        )
    if isinstance(parsed, list):
        raise ValueError('VLA provider returned an invalid action_vector length or value')
    if not isinstance(parsed, dict):
        raise ValueError('VLA provider must return a JSON object command or action_vector')
    if _is_action_vector(parsed.get('action_vector')):
        return _validate_provider_action_vector(
            parsed['action_vector'],
            declared_schema_version=parsed.get('action_vector_schema_version'),
            multi_shuttle_active=multi_shuttle_active,
        )
    action = str(parsed.get('action', '')).strip()
    if action not in MODEL_OUTPUT_ACTIONS:
        raise ValueError(f'unsupported VLA action {action!r}; route_template is not a model output')
    if _multi_shuttle_json_command_is_ambiguous(parsed):
        raise ValueError('schema-v3 primitive shuttle command requires shuttle_id/shuttle/name')
    return parsed


class Room315RealVlaAgent(Node):
    def __init__(self) -> None:
        super().__init__('room_315_real_vla_agent')

        self.declare_parameter('provider', 'http')
        self.declare_parameter('user_goal_topic', '/room_315/vla/user_goal')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('right_image_topic', '/room_315/vla/right_rail_rgbd/image')
        self.declare_parameter('left_image_topic', '/room_315/vla/left_rail_rgbd/image')
        self.declare_parameter('status_topic', '/room_315/vla/status')
        self.declare_parameter('command_topic', '/room_315/vla/command')
        self.declare_parameter('agent_status_topic', '/room_315/vla/agent_status')
        self.declare_parameter('decision_period_s', 0.5)
        self.declare_parameter('request_timeout_s', 20.0)
        self.declare_parameter('image_jpeg_quality', 80)
        self.declare_parameter('image_max_width', 960)
        self.declare_parameter('http_endpoint', os.getenv('ROOM315_VLA_HTTP_ENDPOINT', ''))

        self.provider = str(self.get_parameter('provider').value).strip().lower() or 'http'
        if self.provider != 'http':
            raise ValueError('provider must be http')

        self.bridge = CvBridge()
        self.latest_images: dict[str, Image] = {}
        self.latest_image_times: dict[str, float] = {}
        self.latest_status: dict[str, Any] = {}
        self.pending_goal: str | None = None
        self.pending_goal_time: float | None = None
        self.busy = False
        self.last_command: dict[str, Any] | None = None
        self.last_error = ''
        self.last_sensor_signature_by_side: dict[str, str] = {side: '' for side in SIDES}
        self.last_sensor_event_time_by_side: dict[str, float | None] = {
            side: None for side in SIDES
        }

        self.command_pub = self.create_publisher(
            String,
            str(self.get_parameter('command_topic').value),
            10,
        )
        self.agent_status_pub = self.create_publisher(
            String,
            str(self.get_parameter('agent_status_topic').value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('user_goal_topic').value),
            self._on_goal,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_status,
            10,
        )
        self._subscribe_image('legacy_primary_rgb', str(self.get_parameter('image_topic').value))
        self._subscribe_image('right_rail_rgb', str(self.get_parameter('right_image_topic').value))
        self._subscribe_image('left_rail_rgb', str(self.get_parameter('left_image_topic').value))

        period_s = max(float(self.get_parameter('decision_period_s').value), 0.1)
        self.create_timer(period_s, self._tick)
        self._publish_agent_status('ready')
        self.get_logger().info(
            f'Room 315 real VLA agent ready. provider={self.provider}, '
            f'goal_topic={self.get_parameter("user_goal_topic").value}'
        )

    def _on_goal(self, msg: String) -> None:
        goal = msg.data.strip()
        if not goal:
            return
        self.pending_goal = goal
        self.pending_goal_time = time.monotonic()
        self.last_error = ''
        self._publish_agent_status('goal_received')

    def _on_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring non-JSON VLA supervisor status.')
            return
        if isinstance(parsed, dict):
            self.latest_status = parsed
            self._update_sensor_event_tracking(parsed)

    def _update_sensor_event_tracking(self, status: dict[str, Any]) -> None:
        now = time.monotonic()
        sensor_bits = _binary_sensor_bits(status)
        for side in SIDES:
            signature = _json_dumps(sensor_bits.get(side, {}))
            if signature == self.last_sensor_signature_by_side.get(side):
                continue
            self.last_sensor_signature_by_side[side] = signature
            self.last_sensor_event_time_by_side[side] = now

    def _subscribe_image(self, camera_name: str, topic: str) -> None:
        topic = topic.strip()
        if not topic:
            return
        self.create_subscription(
            Image,
            topic,
            lambda msg, name=camera_name: self._on_image(name, msg),
            10,
        )

    def _on_image(self, camera_name: str, msg: Image) -> None:
        self.latest_images[camera_name] = msg
        self.latest_image_times[camera_name] = time.monotonic()

    def _tick(self) -> None:
        if self.busy or not self.pending_goal:
            return
        if not self.latest_images:
            self._publish_agent_status('waiting_for_image')
            return

        goal = self.pending_goal
        self.pending_goal = None
        self.busy = True
        self._publish_agent_status('planning')
        try:
            command = self._plan(goal)
            self._publish_command(command)
            self.last_command = command
            self.last_error = ''
            self._publish_agent_status('published')
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().error(f'VLA planning failed: {exc}')
            self._publish_agent_status('error')
        finally:
            self.busy = False

    def _plan(self, goal: str) -> dict[str, Any]:
        images_b64 = self._latest_images_as_jpeg_b64()
        return self._http_plan(goal, images_b64)

    def _latest_images_as_jpeg_b64(self) -> dict[str, str]:
        if not self.latest_images:
            raise RuntimeError('no VLA camera image has arrived yet')
        return {
            name: self._image_as_jpeg_b64(msg)
            for name, msg in self.latest_images.items()
        }

    def _image_as_jpeg_b64(self, msg: Image) -> str:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        max_width = int(self.get_parameter('image_max_width').value)
        if max_width > 0 and frame.shape[1] > max_width:
            scale = max_width / float(frame.shape[1])
            frame = cv2.resize(
                frame,
                (max_width, max(1, int(frame.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )
        quality = int(self.get_parameter('image_jpeg_quality').value)
        quality = min(max(quality, 1), 100)
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('failed to encode latest VLA image as JPEG')
        return base64.b64encode(encoded.tobytes()).decode('ascii')

    def _http_plan(self, goal: str, images_b64: dict[str, str]) -> dict[str, Any]:
        endpoint = str(self.get_parameter('http_endpoint').value).strip()
        if not endpoint:
            raise RuntimeError('http_endpoint is empty')
        model_input = _model_input_from_status(
            self.latest_status,
            language=goal,
            overhead_images=images_b64,
            last_command=self.last_command or {'action': 'status'},
            sensor_event_times=self.last_sensor_event_time_by_side,
            now_s=time.monotonic(),
        )
        action_schema_version = _preferred_action_schema_version(self.latest_status)
        response = self._post_json(
            endpoint,
            {
                'model_input_schema_version': MODEL_INPUT_SCHEMA_VERSION,
                'model_input': model_input,
                'model_input_fields': MODEL_INPUT_FIELDS,
                'allowed_actions': MODEL_OUTPUT_ACTIONS,
                'preferred_model_output': 'action_vector',
                'action_vector_schema_version': action_schema_version,
                'allowed_output_formats': ('action_vector', 'json_command'),
                'event_action_vector_fields': EVENT_ACTION_VECTOR_FIELDS,
                'event_primitive_ids': EVENT_PRIMITIVE_IDS,
                'side_ids': SIDE_IDS,
                'shuttle_index_mapping': _shuttle_index_mapping(),
                'target_ids': TARGET_IDS,
                'reason_ids': REASON_IDS,
                'coordination_mode_ids': COORDINATION_MODE_IDS,
                'multi_shuttle_active': _multi_shuttle_active(self.latest_status),
                'contract_note': (
                    'Return schema-v3 action_vector. Movement actions must include '
                    'shuttle_index. Do not return route_template or privileged fields.'
                ),
            },
            headers={},
        )
        return _parse_command_payload(
            response,
            multi_shuttle_active=_multi_shuttle_active(self.latest_status),
        )

    def _post_json(self, url: str, payload: dict[str, Any], headers: dict[str, str]) -> dict[str, Any]:
        data = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            url,
            data=data,
            method='POST',
            headers={
                'Content-Type': 'application/json',
                **headers,
            },
        )
        timeout_s = float(self.get_parameter('request_timeout_s').value)
        try:
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                body = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError(f'HTTP {exc.code} from VLA provider: {detail}') from exc
        parsed = json.loads(body)
        if not isinstance(parsed, dict):
            raise RuntimeError('VLA provider returned non-object JSON')
        return parsed

    def _publish_command(self, command: dict[str, Any]) -> None:
        msg = String()
        msg.data = _json_dumps(command)
        self.command_pub.publish(msg)
        self.get_logger().info(f'Published VLA command: {msg.data}')

    def _publish_agent_status(self, state: str) -> None:
        msg = String()
        now = time.monotonic()
        image_ages = {
            name: round(now - timestamp, 3)
            for name, timestamp in self.latest_image_times.items()
        }
        msg.data = _json_dumps(
            {
                'state': state,
                'provider': self.provider,
                'pending_goal': self.pending_goal,
                'pending_goal_age_s': (
                    None
                    if self.pending_goal_time is None
                    else round(time.monotonic() - self.pending_goal_time, 3)
                ),
                'last_command': self.last_command,
                'last_error': self.last_error,
                'latest_image_ages_s': image_ages,
                'has_supervisor_status': bool(self.latest_status),
                'action_vector_schema_version': _preferred_action_schema_version(
                    self.latest_status,
                ),
                'event_action_vector_fields': EVENT_ACTION_VECTOR_FIELDS,
                'multi_shuttle_active': _multi_shuttle_active(self.latest_status),
            }
        )
        self.agent_status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Room315RealVlaAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
