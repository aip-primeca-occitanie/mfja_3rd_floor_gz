#!/usr/bin/env python3

import json
import re
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String

from mfja_rail_interfaces.msg import NamedState
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleCommand
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import StopperCommand
from mfja_rail_interfaces.msg import StopperState
from mfja_rail_interfaces.msg import SwitchCommand
from mfja_rail_interfaces.msg import SwitchState
from mfja_rail_interfaces.srv import AddShuttle


SIDES = ('right', 'left')
SWITCHES = ('A1', 'A2', 'A3', 'A4')
DEFAULT_SLOT_SENSOR_BY_SIDE = {
    'right': {
        '1': 'DZI1R',
        '2': 'DZI2R',
        '3': 'DZI3R',
        '4': 'DZI4R',
    },
    'left': {
        '1': 'DZI1L',
        '2': 'DZI2L',
        '3': 'DZI3L',
        '4': 'DZI4L',
    },
}



def _default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_vla'
            / 'vla_supervisor.yaml'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'room_315_vla'
            / 'vla_supervisor.yaml'
        )


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open('r', encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f'{path} must contain a YAML mapping.')
    return loaded


def _clean_token(value: Any) -> str:
    return str(value).strip()


def _normalize_side(raw: Any, default: str = 'right') -> str:
    value = _clean_token(raw).lower()
    if value in {'right', 'r', 'droit', 'droite'}:
        return 'right'
    if value in {'left', 'l', 'gauche'}:
        return 'left'
    return default


def _normalize_loop(raw: Any | None) -> str | None:
    if raw is None:
        return None
    value = _clean_token(raw).lower()
    if value in {'g', 'e', 'exterior', 'external', 'grand', 'grand_boucle', 'big'}:
        return 'exterior'
    if value in {'s', 'i', 'interior', 'internal', 'petit', 'petit_boucle', 'small'}:
        return 'interior'
    return None


def _switch_state_for_loop(loop: str | None) -> str | None:
    if loop == 'exterior':
        return 'EXTERIOR'
    if loop == 'interior':
        return 'INTERIOR'
    return None


def _normalize_stopper_state(raw: Any) -> str:
    value = _clean_token(raw).lower()
    if value in {'0', 'open', 'opened', 'release', 'released', 'off', 'false'}:
        return '0'
    if value in {'1', 'close', 'closed', 'stop', 'blocked', 'on', 'true'}:
        return '1'
    return _clean_token(raw)


def _named_states(assignments: dict[str, Any], state_normalizer=lambda value: _clean_token(value)):
    states = []
    for name, raw_state in assignments.items():
        item = NamedState()
        item.name = _clean_token(name).upper()
        item.state = state_normalizer(raw_state)
        states.append(item)
    return states


class Room315VlaSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('room_315_vla_supervisor')

        self.declare_parameter('config_path', str(_default_config_path()))
        self.declare_parameter('command_topic', '/room_315/vla/command')
        self.declare_parameter('status_topic', '/room_315/vla/status')
        self.declare_parameter('emergency_stop_topic', '/room_315/vla/emergency_stop')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('right_image_topic', '/room_315/vla/right_rail_rgbd/image')
        self.declare_parameter('left_image_topic', '/room_315/vla/left_rail_rgbd/image')
        self.declare_parameter('right_camera_info_topic', '/room_315/vla/right_rail_rgbd/camera_info')
        self.declare_parameter('left_camera_info_topic', '/room_315/vla/left_rail_rgbd/camera_info')
        self.declare_parameter('publish_status_period_s', 1.0)

        raw_config_path = str(self.get_parameter('config_path').value).strip()
        config_path = Path(raw_config_path) if raw_config_path else _default_config_path()
        self.config = _load_yaml(config_path)
        self.defaults = self.config.get('defaults', {})

        self.slot_sensor_by_side = self._slot_sensor_map_from_config()

        self.emergency_stop = False
        self.last_command = ''
        self.last_result = 'initialized'
        self.last_image_time: float | None = None
        self.last_camera_info_time: float | None = None
        self.image_frame_count = 0
        self.camera_info_frame_id = ''
        self.camera_vision: dict[str, dict[str, Any]] = {}

        self.rails: dict[str, dict[str, Any]] = {
            side: {
                'shuttles': {},
                'switches': {},
                'stoppers': {},
                'active_sensors': [],
                'active_position_sensors': [],
            }
            for side in SIDES
        }

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            str(self.get_parameter('command_topic').value),
            self._on_command,
            10,
        )
        self.estop_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            self._on_emergency_stop,
            10,
        )
        self.image_subs = []
        self.camera_info_subs = []
        self._subscribe_image_topic('legacy_primary', str(self.get_parameter('image_topic').value))
        self._subscribe_image_topic('right_rail_rgb', str(self.get_parameter('right_image_topic').value))
        self._subscribe_image_topic('left_rail_rgb', str(self.get_parameter('left_image_topic').value))
        self._subscribe_camera_info_topic(
            'legacy_primary',
            str(self.get_parameter('camera_info_topic').value),
        )
        self._subscribe_camera_info_topic(
            'right_rail_rgb',
            str(self.get_parameter('right_camera_info_topic').value),
        )
        self._subscribe_camera_info_topic(
            'left_rail_rgb',
            str(self.get_parameter('left_camera_info_topic').value),
        )

        self.shuttle_command_pubs: dict[str, Any] = {}
        self.shuttle_add_clients: dict[str, Any] = {}
        self.switch_pubs: dict[str, Any] = {}
        self.stopper_pubs: dict[str, Any] = {}

        for side in SIDES:
            prefix = f'/room_315/rails/{side}'
            self.shuttle_command_pubs[side] = self.create_publisher(
                ShuttleCommand,
                f'{prefix}/shuttles/command',
                10,
            )
            self.shuttle_add_clients[side] = self.create_client(
                AddShuttle,
                f'{prefix}/shuttles/add',
            )
            self.switch_pubs[side] = self.create_publisher(
                SwitchCommand,
                f'{prefix}/switches/command',
                10,
            )
            self.stopper_pubs[side] = self.create_publisher(
                StopperCommand,
                f'{prefix}/stoppers/command',
                10,
            )

            self.create_subscription(
                ShuttleState,
                f'{prefix}/shuttles/state',
                lambda msg, rail_side=side: self._on_shuttle_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                SwitchState,
                f'{prefix}/switches/state',
                lambda msg, rail_side=side: self._on_switch_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                StopperState,
                f'{prefix}/stoppers/state',
                lambda msg, rail_side=side: self._on_stopper_state(rail_side, msg),
                10,
            )
            self.create_subscription(
                SensorFeedback,
                f'{prefix}/sensors/feedback',
                lambda msg, rail_side=side: self._on_sensor_feedback(rail_side, msg, 'active_sensors'),
                10,
            )
            self.create_subscription(
                SensorFeedback,
                f'{prefix}/sensors/position_feedback',
                lambda msg, rail_side=side: self._on_sensor_feedback(
                    rail_side,
                    msg,
                    'active_position_sensors',
                ),
                10,
            )

        period_s = max(float(self.get_parameter('publish_status_period_s').value), 0.1)
        self.create_timer(period_s, self._on_status_timer)
        self.get_logger().info(
            f'Room 315 VLA supervisor ready. Command topic: '
            f'{self.get_parameter("command_topic").value}'
        )

    def _subscribe_image_topic(self, camera_name: str, topic: str) -> None:
        topic = topic.strip()
        if not topic:
            return
        self.image_subs.append(
            self.create_subscription(
                Image,
                topic,
                lambda msg, name=camera_name: self._on_image(name, msg),
                10,
            )
        )

    def _subscribe_camera_info_topic(self, camera_name: str, topic: str) -> None:
        topic = topic.strip()
        if not topic:
            return
        self.camera_info_subs.append(
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, name=camera_name: self._on_camera_info(name, msg),
                10,
            )
        )

    def _on_image(self, camera_name: str, _msg: Image) -> None:
        now = time.monotonic()
        self.last_image_time = now
        self.image_frame_count += 1
        camera = self.camera_vision.setdefault(camera_name, {'image_frames': 0})
        camera['image_frames'] = int(camera.get('image_frames', 0)) + 1
        camera['last_image_time'] = now

    def _on_camera_info(self, camera_name: str, msg: CameraInfo) -> None:
        now = time.monotonic()
        self.last_camera_info_time = now
        self.camera_info_frame_id = msg.header.frame_id
        camera = self.camera_vision.setdefault(camera_name, {'image_frames': 0})
        camera['last_camera_info_time'] = now
        camera['camera_info_frame_id'] = msg.header.frame_id

    def _on_shuttle_state(self, side: str, msg: ShuttleState) -> None:
        self.rails[side]['shuttles'][msg.name] = {
            'mode': msg.mode,
            'segment': msg.current_segment,
            's': round(float(msg.s), 4),
            'x': round(float(msg.x), 4),
            'y': round(float(msg.y), 4),
            'z': round(float(msg.z), 4),
            'yaw': round(float(msg.yaw), 4),
            'speed': round(float(msg.speed), 4),
        }

    def _on_switch_state(self, side: str, msg: SwitchState) -> None:
        self.rails[side]['switches'] = {
            item.name: item.state
            for item in msg.switches
        }

    def _on_stopper_state(self, side: str, msg: StopperState) -> None:
        self.rails[side]['stoppers'] = {
            item.name: item.state
            for item in msg.stoppers
        }

    def _on_sensor_feedback(self, side: str, msg: SensorFeedback, key: str) -> None:
        active = []
        for reading in msg.readings:
            if reading.active:
                item = {
                    'name': reading.name,
                    'type': reading.sensor_type,
                    'shuttle': reading.shuttle_name,
                    'segment': reading.segment,
                    's': round(float(reading.s), 4),
                    's_ratio': round(float(reading.s_ratio), 4),
                }
                distance_m = getattr(reading, 'distance_m', None)
                if distance_m is not None:
                    item['distance_m'] = round(float(distance_m), 4)
                active.append(item)
        self.rails[side][key] = active


    def _on_emergency_stop(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self._stop_all(close_stoppers=True, reason='external emergency stop')
        else:
            self._set_result('emergency stop cleared')
        self._publish_status()

    def _on_command(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return
        self.last_command = raw
        try:
            command = self._parse_command(raw)
            self._execute(command)
        except Exception as exc:
            self._set_result(f'command rejected: {exc}')
            self.get_logger().warning(self.last_result)
        self._publish_status()

    def _parse_command(self, raw: str) -> dict[str, Any] | list[Any]:
        if raw.startswith('{') or raw.startswith('['):
            return json.loads(raw)
        return self._parse_text_command(raw)

    def _parse_text_command(self, raw: str) -> dict[str, Any]:
        text = raw.casefold()
        side = self._infer_side(text)
        slot = self._infer_slot(text)
        loop = self._infer_loop(text)

        if 'clear' in text and ('emergency' in text or 'estop' in text):
            return {'action': 'clear_emergency_stop'}
        if 'stop all' in text or 'all off' in text:
            return {'action': 'stop_all'}
        if 'emergency stop' in text or 'estop' in text:
            return {'action': 'emergency_stop'}



        switch_names = self._infer_switch_names(text)
        if 'switch' in text or switch_names:
            state = _switch_state_for_loop(loop)
            if state is None:
                raise ValueError('switch command needs exterior/interior state')
            names = switch_names or ['ALL']
            return {
                'action': 'switches',
                'side': side,
                'switches': {name: state for name in names},
            }

        if 'stopper' in text or 'stoppers' in text:
            names = switch_names or ['ALL']
            if 'close' in text or 'block' in text:
                state = '1'
            elif 'open' in text or 'release' in text:
                state = '0'
            else:
                raise ValueError('stopper command needs open/close state')
            return {
                'action': 'stoppers',
                'side': side,
                'stoppers': {name: state for name in names},
            }

        if (
            'start' in text
            or 'run' in text
            or ' on' in f' {text} '
            or slot is not None
        ):
            return {
                'action': 'route_shuttle',
                'side': side,
                'start_slot': slot,
                'loop': loop or 'exterior',
                'start': True,
            }

        return {'action': 'status'}

    def _infer_side(self, text: str) -> str:
        if any(token in text for token in ('left', 'gauche')):
            return 'left'
        if any(token in text for token in ('right', 'droit', 'droite')):
            return 'right'
        return 'right'

    def _infer_loop(self, text: str) -> str | None:
        if any(token in text for token in ('interior', 'internal', 'petit', 'small')):
            return 'interior'
        if any(token in text for token in ('exterior', 'external', 'grand', 'large')):
            return 'exterior'
        return None

    def _infer_slot(self, text: str) -> str | None:
        match = re.search(r'(?:slot|start_slot|from)\s*_?\s*([1-4])', text)
        if match:
            return match.group(1)
        match = re.search(r'\b([1-4])\b', text)
        if match and 'slot' in text:
            return match.group(1)
        return None

    def _infer_switch_names(self, text: str) -> list[str]:
        names = sorted({match.group(0).upper() for match in re.finditer(r'\bA[1-4]\b', text, re.IGNORECASE)})
        if 'all' in text:
            return ['ALL']
        return names

    def _execute(self, command: dict[str, Any] | list[Any]) -> None:
        if isinstance(command, list):
            for item in command:
                if not isinstance(item, dict):
                    raise ValueError('list commands must contain objects')
                self._execute(item)
            return

        action = str(command.get('action') or command.get('intent') or command.get('type') or 'status')
        action = action.lower()

        if action in {'status', 'snapshot'}:
            self._set_result('status requested')
            return
        if action in {'clear_emergency_stop', 'clear_estop', 'reset_estop'}:
            self.emergency_stop = False
            self._set_result('emergency stop cleared')
            return
        if action in {'emergency_stop', 'estop'}:
            self.emergency_stop = True
            self._stop_all(close_stoppers=True, reason='commanded emergency stop')
            return
        if action in {'stop_all', 'all_off'}:
            self._stop_all(close_stoppers=bool(command.get('close_stoppers', False)), reason='stop all command')
            return

        if self.emergency_stop:
            raise RuntimeError('emergency stop is active; clear it before motion commands')


        if action in {'route_shuttle', 'route', 'start_route'}:
            self._execute_route(command)
            return
        if action in {'add_shuttle', 'spawn_shuttle'}:
            self._execute_add_shuttle(command)
            return
        if action in {'shuttle', 'shuttle_command'}:
            self._execute_shuttle_command(command)
            return
        if action in {'switches', 'switch'}:
            self._execute_switches(command)
            return
        if action in {'stoppers', 'stopper'}:
            self._execute_stoppers(command)
            return
        raise ValueError(f'unknown VLA action {action!r}')

    def _execute_route(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        loop = _normalize_loop(command.get('loop'))
        switches = self._switch_assignments_from_command(command, loop)
        stoppers = dict(command.get('stoppers') or {'ALL': '0'})
        start_slot = command.get('start_slot')
        start = bool(command.get('start', command.get('start_after_prepare', True)))
        stop_before = bool(command.get('stop_before_prepare', True))

        if stop_before:
            self._publish_shuttle_command(side, command.get('shuttle') or self._default_shuttle_name(side), 'OFF')
        if stoppers:
            self._publish_stoppers(side, stoppers)
        if switches:
            self._publish_switches(side, switches)
        if start_slot:
            self._request_add_shuttle(
                side,
                command.get('shuttle') or self._default_shuttle_name(side),
                start_slot=str(start_slot),
                speed=float(command.get('speed', self._default_speed())),
                start_enabled=start,
            )
        elif start:
            self._publish_shuttle_command(
                side,
                command.get('shuttle') or self._default_shuttle_name(side),
                'ON',
            )

        self._set_result(f'route prepared on {side}: loop={loop or "unchanged"} start={start}')



    def _execute_add_shuttle(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        moving = bool(command.get('moving', command.get('start', False)))
        self._request_add_shuttle(
            side,
            command.get('shuttle') or command.get('name') or self._default_shuttle_name(side),
            start_slot=str(command.get('start_slot', '2')),
            speed=float(command.get('speed', self._default_speed())),
            start_enabled=moving,
        )
        self._set_result(f'added shuttle on {side}')

    def _execute_shuttle_command(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        shuttle_command = str(command.get('command', 'ON')).upper()
        if shuttle_command in {'ADD_MOVING', 'ADD_STOPPED'}:
            self._request_add_shuttle(
                side,
                command.get('shuttle') or command.get('name') or self._default_shuttle_name(side),
                start_slot=str(command.get('start_slot', '2')),
                speed=float(command.get('speed', self._default_speed())),
                start_enabled=shuttle_command == 'ADD_MOVING',
            )
            self._set_result(f'add shuttle request sent on {side}')
            return
        self._publish_shuttle_command(
            side,
            command.get('shuttle') or command.get('name') or self._default_shuttle_name(side),
            shuttle_command,
            start_slot=str(command.get('start_slot', '')),
            speed=float(command.get('speed', self._default_speed())),
        )
        self._set_result(f'shuttle command sent on {side}')

    def _execute_switches(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        loop = _normalize_loop(command.get('loop'))
        switches = self._switch_assignments_from_command(command, loop)
        if not switches:
            raise ValueError('switch action needs "switches" or "loop"')
        self._publish_switches(side, switches)
        self._set_result(f'switches commanded on {side}: {switches}')

    def _execute_stoppers(self, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        stoppers = dict(command.get('stoppers') or {})
        if not stoppers and 'name' in command and 'state' in command:
            stoppers = {str(command['name']): command['state']}
        if not stoppers:
            raise ValueError('stopper action needs "stoppers"')
        self._publish_stoppers(side, stoppers)
        self._set_result(f'stoppers commanded on {side}: {stoppers}')

    def _slot_sensor_map_from_config(self) -> dict[str, dict[str, str]]:
        mapping = {
            side: dict(DEFAULT_SLOT_SENSOR_BY_SIDE[side])
            for side in SIDES
        }
        configured = self.defaults.get('slot_sensor_by_side')
        if not isinstance(configured, dict):
            return mapping
        for side in SIDES:
            side_mapping = configured.get(side)
            if not isinstance(side_mapping, dict):
                continue
            for slot, sensor_name in side_mapping.items():
                slot_name = str(slot).strip()
                if slot_name in {'1', '2', '3', '4'}:
                    mapping[side][slot_name] = str(sensor_name).strip()
        return mapping

    def _normalize_slots(self, raw_slots: Any) -> list[str]:
        if raw_slots is None:
            return []
        if isinstance(raw_slots, str):
            values = re.split(r'[\s,]+', raw_slots.strip())
        else:
            try:
                values = list(raw_slots)
            except TypeError:
                values = [raw_slots]
        slots = []
        for value in values:
            slot = str(value).strip()
            if slot.startswith('slot_'):
                slot = slot.rsplit('_', 1)[-1]
            if slot in {'1', '2', '3', '4'} and slot not in slots:
                slots.append(slot)
        return slots

    def _normalize_segments(self, raw_segments: Any) -> list[str]:
        if raw_segments is None:
            return []
        if isinstance(raw_segments, str):
            values = re.split(r'[\s,]+', raw_segments.strip())
        else:
            try:
                values = list(raw_segments)
            except TypeError:
                values = [raw_segments]
        segments = []
        for value in values:
            segment = str(value).strip().upper()
            if segment and segment not in segments:
                segments.append(segment)
        return segments

    def _normalize_names(self, raw_names: Any) -> list[str]:
        if raw_names is None:
            return []
        if isinstance(raw_names, str):
            values = re.split(r'[\s,]+', raw_names.strip())
        else:
            try:
                values = list(raw_names)
            except TypeError:
                values = [raw_names]
        names = []
        for value in values:
            name = str(value).strip()
            if name and name not in names:
                names.append(name)
        return names

    def _slot_sensor_name(self, side: str, slot: str) -> str:
        return self.slot_sensor_by_side.get(side, {}).get(str(slot), '')

    def _sensor_names_for_slots(self, side: str, slots: list[str]) -> list[str]:
        return [
            sensor_name
            for sensor_name in (self._slot_sensor_name(side, slot) for slot in slots)
            if sensor_name
        ]

    def _find_shuttle_in_slots(self, side: str, slots: list[str]) -> tuple[str, str, str]:
        for slot in slots:
            sensor_name = self._slot_sensor_name(side, slot)
            if not sensor_name:
                continue
            reading = self._active_sensor_reading(side, sensor_name)
            if not reading:
                continue
            shuttle_name = str(reading.get('shuttle') or '').strip()
            if shuttle_name:
                return shuttle_name, slot, sensor_name
        return '', '', ''

    def _active_sensor_reading(
        self,
        side: str,
        sensor_name: str,
        shuttle_name: str = '',
    ) -> dict[str, Any] | None:
        wanted = sensor_name.strip().casefold()
        for key in ('active_position_sensors', 'active_sensors'):
            for reading in self.rails.get(side, {}).get(key, []) or []:
                if str(reading.get('name') or '').casefold() != wanted:
                    continue
                reading_shuttle = str(reading.get('shuttle') or '').strip()
                if shuttle_name and reading_shuttle != shuttle_name:
                    continue
                return reading
        return None

    def _on_status_timer(self) -> None:
        self._publish_status()



    def _switch_assignments_from_command(
        self,
        command: dict[str, Any],
        loop: str | None,
    ) -> dict[str, Any]:
        switches = dict(command.get('switches') or {})
        if not switches and 'name' in command and 'state' in command:
            switches = {str(command['name']): command['state']}
        loop_state = _switch_state_for_loop(loop)
        if not switches and loop_state:
            switches = {'ALL': loop_state}
        return switches

    def _publish_switches(self, side: str, assignments: dict[str, Any]) -> None:
        msg = SwitchCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.switches = _named_states(assignments)
        self.switch_pubs[side].publish(msg)

    def _publish_stoppers(self, side: str, assignments: dict[str, Any]) -> None:
        msg = StopperCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stoppers = _named_states(assignments, _normalize_stopper_state)
        self.stopper_pubs[side].publish(msg)

    def _publish_shuttle_command(
        self,
        side: str,
        name: Any,
        command: str,
        *,
        start_slot: str = '',
        speed: float | None = None,
    ) -> None:
        msg = ShuttleCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = _clean_token(name)
        msg.command = command.upper()
        msg.start_slot = start_slot
        msg.speed = float(self._default_speed() if speed is None else speed)
        self.shuttle_command_pubs[side].publish(msg)

    def _request_add_shuttle(
        self,
        side: str,
        name: Any,
        *,
        start_slot: str,
        speed: float,
        start_enabled: bool,
    ) -> None:
        client = self.shuttle_add_clients[side]
        if not client.service_is_ready():
            self.get_logger().warning(
                f'AddShuttle service for {side} rail is not ready yet; request queued by ROS.'
            )
        request = AddShuttle.Request()
        request.name = _clean_token(name)
        request.start_slot = _clean_token(start_slot)
        request.speed = float(speed)
        request.start_enabled = bool(start_enabled)
        future = client.call_async(request)
        future.add_done_callback(
            lambda result, rail_side=side: self._on_add_shuttle_response(rail_side, result)
        )

    def _on_add_shuttle_response(self, side: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'AddShuttle service call failed on {side}: {exc}')
            return
        if response.success:
            self.get_logger().info(f'AddShuttle succeeded on {side}: {response.message}')
        else:
            self.get_logger().error(f'AddShuttle rejected on {side}: {response.message}')

    def _stop_all(self, close_stoppers: bool, reason: str) -> None:
        for side in SIDES:
            self._publish_shuttle_command(side, 'ALL', 'OFF')
            if close_stoppers:
                self._publish_stoppers(side, {'ALL': '1'})
        self._set_result(f'{reason}: all shuttles OFF')

    def _default_speed(self) -> float:
        return float(self.defaults.get('speed', 0.2))

    def _default_shuttle_name(self, side: str) -> str:
        names = self.defaults.get('shuttle_name_by_side', {})
        if isinstance(names, dict) and side in names:
            return str(names[side])
        return f'room315_{side}_shuttle_1'

    def _set_result(self, result: str) -> None:
        self.last_result = result
        self.get_logger().info(result)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._snapshot(), sort_keys=True)
        self.status_pub.publish(msg)



    def _snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        image_age = None if self.last_image_time is None else round(now - self.last_image_time, 3)
        camera_info_age = (
            None
            if self.last_camera_info_time is None
            else round(now - self.last_camera_info_time, 3)
        )
        cameras = {}
        for camera_name, camera in self.camera_vision.items():
            last_image_time = camera.get('last_image_time')
            last_camera_info_time = camera.get('last_camera_info_time')
            cameras[camera_name] = {
                'image_frames': int(camera.get('image_frames', 0)),
                'last_image_age_s': (
                    None if last_image_time is None else round(now - float(last_image_time), 3)
                ),
                'last_camera_info_age_s': (
                    None
                    if last_camera_info_time is None
                    else round(now - float(last_camera_info_time), 3)
                ),
                'camera_info_frame_id': str(camera.get('camera_info_frame_id', '')),
            }
        return {
            'emergency_stop': self.emergency_stop,
            'last_command': self.last_command,
            'last_result': self.last_result,

            'vision': {
                'image_frames': self.image_frame_count,
                'last_image_age_s': image_age,
                'last_camera_info_age_s': camera_info_age,
                'camera_info_frame_id': self.camera_info_frame_id,
                'cameras': cameras,
            },
            'rails': self.rails,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Room315VlaSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
