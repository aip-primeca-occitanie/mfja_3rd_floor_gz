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
RIGHT_YASKAWA_TOKENS = ('yaskawa', 'hc10dt')
LEFT_YASKAWA_TOKENS = ('yaskawa', 'hc10')
STAUBLI_TOKENS = ('staubli', 'stäubli', 'tx2')
KUKA_TOKENS = ('kuka', 'kr6')
ARABIC_STOP_WORDS = ('وقف', 'اوقف', 'إيقاف', 'ايقاف', 'طوارئ')
ARABIC_START_WORDS = ('شغل', 'ابدأ', 'أبدأ', 'تحرك')
ARABIC_OPEN_WORDS = ('افتح', 'أفتح')
ARABIC_CLOSE_WORDS = ('اغلق', 'أغلق', 'سكر')


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
    if value in {'right', 'r', 'droit', 'droite', 'يمين'}:
        return 'right'
    if value in {'left', 'l', 'gauche', 'يسار'}:
        return 'left'
    return default


def _normalize_loop(raw: Any | None) -> str | None:
    if raw is None:
        return None
    value = _clean_token(raw).lower()
    if value in {'g', 'e', 'exterior', 'external', 'grand', 'grand_boucle', 'big', 'خارجي', 'كبير'}:
        return 'exterior'
    if value in {'s', 'i', 'interior', 'internal', 'petit', 'petit_boucle', 'small', 'داخلي', 'صغير'}:
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
    if value in {'0', 'open', 'opened', 'release', 'released', 'off', 'false', 'افتح', 'أفتح'}:
        return '0'
    if value in {'1', 'close', 'closed', 'stop', 'blocked', 'on', 'true', 'اغلق', 'أغلق', 'سكر'}:
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
        self.route_templates = self.config.get('route_templates', {})
        self.task_aliases = self.config.get('task_aliases', {})
        self.slot_sensor_by_side = self._slot_sensor_map_from_config()

        self.emergency_stop = False
        self.last_command = ''
        self.last_result = 'initialized'
        self.last_image_time: float | None = None
        self.last_camera_info_time: float | None = None
        self.image_frame_count = 0
        self.camera_info_frame_id = ''
        self.camera_vision: dict[str, dict[str, Any]] = {}
        self.active_tasks: dict[str, dict[str, Any]] = {}
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
        if key == 'active_position_sensors' and self.active_tasks:
            self._update_active_tasks()

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
        if 'stop all' in text or 'all off' in text or 'وقف الكل' in text:
            return {'action': 'stop_all'}
        if any(word in text for word in ARABIC_STOP_WORDS) or 'emergency stop' in text or 'estop' in text:
            return {'action': 'emergency_stop'}

        task_template = self._infer_task_template(text, side, loop)
        if task_template:
            return {
                'action': 'route_template',
                'template': task_template,
            }

        for alias, template_name in self.task_aliases.items():
            if str(alias).casefold() in text:
                return {
                    'action': 'route_template',
                    'template': template_name,
                }

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

        if 'stopper' in text or 'stoppers' in text or 'ستوبر' in text:
            names = switch_names or ['ALL']
            if any(word in text for word in ARABIC_CLOSE_WORDS) or 'close' in text or 'block' in text:
                state = '1'
            elif any(word in text for word in ARABIC_OPEN_WORDS) or 'open' in text or 'release' in text:
                state = '0'
            else:
                raise ValueError('stopper command needs open/close state')
            return {
                'action': 'stoppers',
                'side': side,
                'stoppers': {name: state for name in names},
            }

        if (
            any(word in text for word in ARABIC_START_WORDS)
            or 'start' in text
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

    def _infer_task_template(self, text: str, side: str, loop: str | None) -> str | None:
        if loop == 'interior' and any(token in text for token in ('loop', 'circulate', 'دور', 'دائرة')):
            if side == 'left':
                return 'left_enter_interior_loop'
            return 'right_enter_interior_loop'

        yaskawa_to_staubli = self._appears_before(text, RIGHT_YASKAWA_TOKENS, STAUBLI_TOKENS)
        staubli_to_yaskawa = self._appears_before(text, STAUBLI_TOKENS, RIGHT_YASKAWA_TOKENS)
        if self._has_any(text, STAUBLI_TOKENS) and self._has_any(text, RIGHT_YASKAWA_TOKENS):
            if yaskawa_to_staubli or ('to staubli' in text or 'to stäubli' in text or 'to tx2' in text):
                return 'right_yaskawa_to_staubli'
            if staubli_to_yaskawa or 'to yaskawa' in text or 'to hc10dt' in text:
                return 'right_staubli_to_yaskawa'

        yaskawa_to_kuka = self._appears_before(text, LEFT_YASKAWA_TOKENS, KUKA_TOKENS)
        kuka_to_yaskawa = self._appears_before(text, KUKA_TOKENS, LEFT_YASKAWA_TOKENS)
        if self._has_any(text, KUKA_TOKENS) and self._has_any(text, LEFT_YASKAWA_TOKENS):
            if yaskawa_to_kuka or 'to kuka' in text or 'to kr6' in text:
                return 'left_yaskawa_to_kuka'
            if kuka_to_yaskawa or 'to yaskawa' in text or 'to hc10' in text:
                return 'left_kuka_to_yaskawa'

        return None

    def _has_any(self, text: str, tokens: tuple[str, ...]) -> bool:
        return any(token in text for token in tokens)

    def _appears_before(
        self,
        text: str,
        first_tokens: tuple[str, ...],
        second_tokens: tuple[str, ...],
    ) -> bool:
        first_positions = [text.find(token) for token in first_tokens if token in text]
        second_positions = [text.find(token) for token in second_tokens if token in text]
        if not first_positions or not second_positions:
            return False
        return min(first_positions) < min(second_positions)

    def _infer_side(self, text: str) -> str:
        if any(token in text for token in ('left', 'gauche', 'يسار')):
            return 'left'
        if any(token in text for token in ('right', 'droit', 'droite', 'يمين')):
            return 'right'
        return 'right'

    def _infer_loop(self, text: str) -> str | None:
        if any(token in text for token in ('interior', 'internal', 'petit', 'small', 'داخلي', 'صغير')):
            return 'interior'
        if any(token in text for token in ('exterior', 'external', 'grand', 'large', 'خارجي', 'كبير')):
            return 'exterior'
        return None

    def _infer_slot(self, text: str) -> str | None:
        match = re.search(r'(?:slot|start_slot|from)\s*_?\s*([1-4])', text)
        if match:
            return match.group(1)
        match = re.search(r'\b([1-4])\b', text)
        if match and any(token in text for token in ('slot', 'فتحة', 'خانه', 'خانة')):
            return match.group(1)
        return None

    def _infer_switch_names(self, text: str) -> list[str]:
        names = sorted({match.group(0).upper() for match in re.finditer(r'\bA[1-4]\b', text, re.IGNORECASE)})
        if 'all' in text or 'كل' in text:
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

        if action in {'route_template', 'template'}:
            template_name = str(command.get('template') or command.get('name') or '')
            self._execute_route_template(template_name, command)
            return
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

    def _execute_route_template(self, template_name: str, overrides: dict[str, Any]) -> None:
        if not template_name:
            raise ValueError('route_template needs "template"')
        template = self.route_templates.get(template_name)
        if not isinstance(template, dict):
            raise ValueError(f'unknown route template {template_name!r}')
        override_items = {
            key: value
            for key, value in overrides.items()
            if key not in {'action', 'intent', 'type', 'template', 'name'} and value is not None
        }
        command = {**template, **override_items}
        template_type = str(command.get('type') or 'route').lower()
        if template_type in {'transport', 'transport_task'}:
            self._execute_transport_task(template_name, command)
            return
        if template_type in {'loop_entry', 'interior_loop_entry'}:
            self._execute_loop_entry_task(template_name, command)
            return
        command['action'] = 'route_shuttle'
        self._execute_route(command)
        self._set_result(f'route template {template_name} executed')

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

    def _execute_transport_task(self, template_name: str, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        source_slots = self._normalize_slots(command.get('source_slots'))
        target_slots = self._normalize_slots(command.get('target_slots'))
        if not source_slots:
            raise ValueError(f'transport task {template_name} needs source_slots')
        if not target_slots:
            raise ValueError(f'transport task {template_name} needs target_slots')

        shuttle_name, source_slot, source_sensor = self._find_shuttle_in_slots(side, source_slots)
        if not shuttle_name:
            sensors = ', '.join(self._sensor_names_for_slots(side, source_slots))
            slots = ' or '.join(source_slots)
            raise ValueError(
                f'task {template_name} rejected: no {side}-rail shuttle detected in '
                f'source slots {slots}; expected one of sensors {sensors}'
            )

        loop = _normalize_loop(command.get('loop')) or 'exterior'
        switches = self._switch_assignments_from_command(command, loop)
        stoppers = dict(command.get('stoppers') or {'ALL': '0'})

        self._publish_shuttle_command(side, shuttle_name, 'OFF')
        if stoppers:
            self._publish_stoppers(side, stoppers)
        if switches:
            self._publish_switches(side, switches)
        self._publish_shuttle_command(
            side,
            shuttle_name,
            'ON',
            speed=float(command.get('speed', self._default_speed())),
        )

        task_id = self._new_task_id(template_name, side, shuttle_name)
        self.active_tasks[task_id] = {
            'type': 'transport',
            'template': template_name,
            'side': side,
            'shuttle': shuttle_name,
            'source_slot': source_slot,
            'source_sensor': source_sensor,
            'target_slots': target_slots,
            'target_sensors': self._sensor_names_for_slots(side, target_slots),
            'timeout_s': float(command.get('completion_timeout_s', 0.0) or 0.0),
            'started_at': time.monotonic(),
        }
        self._set_result(
            f'task {template_name} started: {side} shuttle {shuttle_name} from '
            f'slot {source_slot} toward target slots {" or ".join(target_slots)}'
        )

    def _execute_loop_entry_task(self, template_name: str, command: dict[str, Any]) -> None:
        side = _normalize_side(command.get('side', 'right'))
        source_slots = self._normalize_slots(command.get('source_slots'))
        if not source_slots:
            raise ValueError(f'loop-entry task {template_name} needs source_slots')
        shuttle_name, source_slot, source_sensor = self._find_shuttle_in_slots(side, source_slots)
        if not shuttle_name:
            sensors = ', '.join(self._sensor_names_for_slots(side, source_slots))
            slots = ' or '.join(source_slots)
            raise ValueError(
                f'task {template_name} rejected: no {side}-rail shuttle detected in '
                f'source slots {slots}; expected one of sensors {sensors}'
            )

        trigger_sensor = str(command.get('trigger_sensor') or '').strip()
        trigger_sensors = self._normalize_names(command.get('trigger_sensors'))
        if trigger_sensor and trigger_sensor not in trigger_sensors:
            trigger_sensors.insert(0, trigger_sensor)
        if not trigger_sensors:
            raise ValueError(f'loop-entry task {template_name} needs trigger_sensor')
        timeout_s = float(command.get('timeout_s', 12.0) or 12.0)
        initial_switches = dict(command.get('initial_switches') or {})
        final_switches = dict(command.get('final_switches') or {})
        stoppers = dict(command.get('stoppers') or {'ALL': '0'})
        final_resume_delay_s = float(command.get('final_resume_delay_s', 0.6) or 0.6)
        keepalive_duration_s = float(command.get('keepalive_duration_s', 3.0) or 3.0)
        keepalive_period_s = float(command.get('keepalive_period_s', 0.5) or 0.5)

        self._publish_shuttle_command(side, shuttle_name, 'OFF')
        if stoppers:
            self._publish_stoppers(side, stoppers)
        if initial_switches:
            self._publish_switches(side, initial_switches)
        self._publish_shuttle_command(
            side,
            shuttle_name,
            'ON',
            speed=float(command.get('speed', self._default_speed())),
        )

        task_id = self._new_task_id(template_name, side, shuttle_name)
        self.active_tasks[task_id] = {
            'type': 'loop_entry',
            'template': template_name,
            'side': side,
            'shuttle': shuttle_name,
            'source_slot': source_slot,
            'source_sensor': source_sensor,
            'trigger_sensor': trigger_sensors[0],
            'trigger_sensors': trigger_sensors,
            'trigger_segments': self._normalize_segments(command.get('trigger_segments')),
            'final_switches': final_switches,
            'stoppers': stoppers,
            'timeout_s': timeout_s,
            'phase': 'waiting_for_trigger',
            'final_resume_delay_s': max(final_resume_delay_s, 0.0),
            'keepalive_duration_s': max(keepalive_duration_s, 0.0),
            'keepalive_period_s': max(keepalive_period_s, 0.1),
            'started_at': time.monotonic(),
        }
        self._set_result(
            f'task {template_name} started: waiting for {trigger_sensor} before '
            f'commanding final interior switches'
        )

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

    def _first_triggered_slot(
        self,
        side: str,
        slots: list[str],
        shuttle_name: str,
    ) -> tuple[str, str] | None:
        for slot in slots:
            sensor_name = self._slot_sensor_name(side, slot)
            if sensor_name and self._active_sensor_reading(side, sensor_name, shuttle_name):
                return slot, sensor_name
        return None

    def _loop_entry_trigger_reason(
        self,
        side: str,
        shuttle_name: str,
        trigger_sensors: list[str],
        trigger_segments: list[str],
    ) -> str:
        for trigger_sensor in trigger_sensors:
            if self._active_sensor_reading(side, trigger_sensor, shuttle_name):
                return trigger_sensor
        if trigger_segments:
            shuttle = self.rails.get(side, {}).get('shuttles', {}).get(shuttle_name, {})
            segment = str(shuttle.get('segment') or '').upper()
            if segment in trigger_segments:
                return f'segment {segment}'
        return ''

    def _new_task_id(self, template_name: str, side: str, shuttle_name: str) -> str:
        started_ms = int(time.monotonic() * 1000)
        return f'{template_name}:{side}:{shuttle_name}:{started_ms}'

    def _on_status_timer(self) -> None:
        self._update_active_tasks()
        self._publish_status()

    def _update_active_tasks(self) -> None:
        if not self.active_tasks:
            return
        now = time.monotonic()
        completed_task_ids = []
        for task_id, task in list(self.active_tasks.items()):
            task_type = str(task.get('type') or '')
            side = str(task.get('side') or 'right')
            shuttle_name = str(task.get('shuttle') or '')
            template_name = str(task.get('template') or task_id)
            started_at = float(task.get('started_at') or now)
            timeout_s = float(task.get('timeout_s') or 0.0)

            if task_type == 'transport':
                hit = self._first_triggered_slot(
                    side,
                    list(task.get('target_slots') or []),
                    shuttle_name,
                )
                if hit:
                    target_slot, target_sensor = hit
                    self._publish_shuttle_command(side, shuttle_name, 'OFF')
                    self._set_result(
                        f'task {template_name} completed: {side} shuttle {shuttle_name} '
                        f'reached target slot {target_slot} via {target_sensor}'
                    )
                    completed_task_ids.append(task_id)
                    continue
                if timeout_s > 0.0 and now - started_at > timeout_s:
                    self._publish_shuttle_command(side, shuttle_name, 'OFF')
                    self._set_result(
                        f'task {template_name} failed: target slots '
                        f'{" or ".join(task.get("target_slots") or [])} were not reached '
                        f'within {timeout_s:.1f}s; shuttle {shuttle_name} stopped'
                    )
                    completed_task_ids.append(task_id)
                    continue

            if task_type == 'loop_entry':
                phase = str(task.get('phase') or 'waiting_for_trigger')
                if phase == 'circulating':
                    keepalive_until = float(task.get('keepalive_until') or started_at)
                    next_keepalive_at = float(task.get('next_keepalive_at') or now)
                    if now <= keepalive_until and now >= next_keepalive_at:
                        self._publish_shuttle_command(side, shuttle_name, 'ON')
                        stoppers = dict(task.get('stoppers') or {})
                        if stoppers:
                            self._publish_stoppers(side, stoppers)
                        task['next_keepalive_at'] = now + float(
                            task.get('keepalive_period_s') or 0.5
                        )
                    if now > keepalive_until:
                        self._set_result(
                            f'task {template_name} completed: {side} shuttle {shuttle_name} '
                            f'is circulating on the interior loop'
                        )
                        completed_task_ids.append(task_id)
                    continue

                trigger_sensor = str(task.get('trigger_sensor') or '')
                trigger_reason = self._loop_entry_trigger_reason(
                    side,
                    shuttle_name,
                    list(task.get('trigger_sensors') or [trigger_sensor]),
                    list(task.get('trigger_segments') or []),
                )
                if trigger_reason:
                    final_switches = dict(task.get('final_switches') or {})
                    if final_switches:
                        self._publish_switches(side, final_switches)
                    stoppers = dict(task.get('stoppers') or {})
                    if stoppers:
                        self._publish_stoppers(side, stoppers)
                    task['phase'] = 'circulating'
                    task['triggered_at'] = now
                    task['next_keepalive_at'] = now + float(
                        task.get('final_resume_delay_s') or 0.6
                    )
                    task['keepalive_until'] = now + float(
                        task.get('keepalive_duration_s') or 3.0
                    )
                    self._set_result(
                        f'task {template_name}: {trigger_reason} triggered; final switches '
                        f'commanded {final_switches}; keeping {shuttle_name} enabled for '
                        f'interior circulation'
                    )
                    continue
                if timeout_s > 0.0 and now - started_at > timeout_s:
                    self._publish_shuttle_command(side, shuttle_name, 'OFF')
                    trigger_names = list(task.get('trigger_sensors') or [trigger_sensor])
                    trigger_targets = ' or '.join([
                        *trigger_names,
                        *[f'segment {segment}' for segment in task.get('trigger_segments') or []],
                    ])
                    self._set_result(
                        f'task {template_name} failed: {trigger_targets} was not triggered '
                        f'within {timeout_s:.1f}s; shuttle {shuttle_name} stopped'
                    )
                    completed_task_ids.append(task_id)
                    continue

        for task_id in completed_task_ids:
            self.active_tasks.pop(task_id, None)

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
        self.active_tasks.clear()
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

    def _active_task_snapshot(self) -> dict[str, dict[str, Any]]:
        now = time.monotonic()
        snapshot = {}
        for task_id, task in self.active_tasks.items():
            item = {key: value for key, value in task.items() if key != 'started_at'}
            started_at = task.get('started_at')
            item['age_s'] = None if started_at is None else round(now - float(started_at), 3)
            snapshot[task_id] = item
        return snapshot

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
            'active_tasks': self._active_task_snapshot(),
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
