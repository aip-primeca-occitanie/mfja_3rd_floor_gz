#!/usr/bin/env python3

import json
import os
import time
from datetime import datetime
from datetime import timezone
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


SIDES = ('right', 'left')
DEVICE_NAMES = ('A1', 'A2', 'A3', 'A4')


ACTION_IDS = {
    'status': 0,
    'route_shuttle': 2,
    'switches': 3,
    'stoppers': 4,
    'shuttle': 5,
    'add_shuttle': 6,
    'stop_all': 7,
    'emergency_stop': 8,
    'clear_emergency_stop': 9,
}

SIDE_IDS = {'none': 0, 'right': 1, 'left': 2}
LOOP_IDS = {'none': 0, 'exterior': 1, 'interior': 2}
SHUTTLE_COMMAND_IDS = {'none': 0, 'OFF': 1, 'ON': 2, 'ADD_STOPPED': 3, 'ADD_MOVING': 4}
SWITCH_ALL_STATE_IDS = {'none': 0, 'EXTERIOR': 1, 'INTERIOR': 2}
STOPPER_ALL_STATE_IDS = {'none': 0, 'open': 1, 'closed': 2}

ACTION_VECTOR_FIELDS = [
    'action_id',
    'side_id',
    'slot_id',
    'loop_id',
    'shuttle_command_id',
    'switch_all_state_id',
    'stopper_all_state_id',
    'start',
    'close_stoppers',
    'speed_mps',
]

OBSERVATION_STATE_FIELDS = [
    'emergency_stop',
    'vision_image_frames',
    'right_shuttle_count',
    'right_active_sensors',
    'right_active_position_sensors',
    'left_shuttle_count',
    'left_active_sensors',
    'left_active_position_sensors',
    *[f'right_switch_{name}' for name in DEVICE_NAMES],
    *[f'left_switch_{name}' for name in DEVICE_NAMES],
    *[f'right_stopper_{name}' for name in DEVICE_NAMES],
    *[f'left_stopper_{name}' for name in DEVICE_NAMES],
]


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _safe_filename(text: str, fallback: str) -> str:
    cleaned = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in text.strip())
    cleaned = '_'.join(part for part in cleaned.split('_') if part)
    return cleaned[:80] or fallback


def _parse_json_or_text(raw: str) -> Any:
    text = raw.strip()
    if not text:
        return {}
    if text[0] in '{[':
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {'text_command': text}
    return {'text_command': text}


def _as_command_dict(command: Any) -> dict[str, Any]:
    if isinstance(command, list) and command and isinstance(command[0], dict):
        return command[0]
    if isinstance(command, dict):
        return command
    return {'action': 'status'}


def _normalize_action_name(command: dict[str, Any]) -> str:
    action = str(command.get('action') or command.get('intent') or command.get('type') or 'status')
    action = action.lower().strip()
    if action in {'route', 'start_route'}:
        return 'route_shuttle'
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
    return action if action in ACTION_IDS else 'status'


def _numeric_slot(raw: Any) -> float:
    try:
        slot = int(str(raw).strip())
    except (TypeError, ValueError):
        return 0.0
    return float(slot if 1 <= slot <= 4 else 0)


def _switch_all_state(command: dict[str, Any]) -> str:
    switches = command.get('switches')
    if isinstance(switches, dict):
        state = switches.get('ALL') or switches.get('all')
        if state:
            return str(state).upper()
    loop = str(command.get('loop') or '').lower()
    if loop == 'exterior':
        return 'EXTERIOR'
    if loop == 'interior':
        return 'INTERIOR'
    return 'none'


def _stopper_all_state(command: dict[str, Any]) -> str:
    stoppers = command.get('stoppers')
    state = None
    if isinstance(stoppers, dict):
        state = stoppers.get('ALL') or stoppers.get('all')
    if state is None:
        return 'none'
    state_text = str(state).strip().lower()
    if state_text in {'0', 'open', 'opened', 'release', 'released'}:
        return 'open'
    if state_text in {'1', 'close', 'closed', 'stop', 'blocked'}:
        return 'closed'
    return 'none'


def _encode_action(command: Any) -> list[float]:
    command_dict = _as_command_dict(command)
    action = _normalize_action_name(command_dict)
    side = str(command_dict.get('side') or 'none').lower()
    loop = str(command_dict.get('loop') or 'none').lower()
    shuttle_command = str(command_dict.get('command') or 'none').upper()
    switch_state = _switch_all_state(command_dict)
    stopper_state = _stopper_all_state(command_dict)
    default_start = action in {'route_shuttle'}
    start = command_dict.get('start', command_dict.get('start_after_prepare', default_start))
    return [
        float(ACTION_IDS.get(action, 0)),
        float(SIDE_IDS.get(side, 0)),
        _numeric_slot(command_dict.get('start_slot')),
        float(LOOP_IDS.get(loop, 0)),
        float(SHUTTLE_COMMAND_IDS.get(shuttle_command, 0)),
        float(SWITCH_ALL_STATE_IDS.get(switch_state, 0)),
        float(STOPPER_ALL_STATE_IDS.get(stopper_state, 0)),
        1.0 if _as_bool(start) else 0.0,
        1.0 if _as_bool(command_dict.get('close_stoppers', False)) else 0.0,
        float(command_dict.get('speed', 0.0) or 0.0),
    ]


def _switch_state_value(raw: Any) -> float:
    state = str(raw or '').upper()
    if state == 'EXTERIOR':
        return 1.0
    if state == 'INTERIOR':
        return 2.0
    return 0.0


def _stopper_state_value(raw: Any) -> float:
    state = str(raw or '').strip().lower()
    if state in {'0', 'open', 'opened'}:
        return 1.0
    if state in {'1', 'closed', 'close'}:
        return 2.0
    return 0.0


def _encode_state(status: dict[str, Any]) -> list[float]:
    rails = status.get('rails', {}) if isinstance(status.get('rails'), dict) else {}
    vision = status.get('vision', {}) if isinstance(status.get('vision'), dict) else {}
    values = [
        1.0 if status.get('emergency_stop') else 0.0,
        float(vision.get('image_frames', 0) or 0),
    ]
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        values.extend([
            float(len(rail.get('shuttles', {}) or {})),
            float(len(rail.get('active_sensors', []) or [])),
            float(len(rail.get('active_position_sensors', []) or [])),
        ])
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        switches = rail.get('switches', {}) if isinstance(rail.get('switches'), dict) else {}
        values.extend(_switch_state_value(switches.get(name)) for name in DEVICE_NAMES)
    for side in SIDES:
        rail = rails.get(side, {}) if isinstance(rails.get(side), dict) else {}
        stoppers = rail.get('stoppers', {}) if isinstance(rail.get('stoppers'), dict) else {}
        values.extend(_stopper_state_value(stoppers.get(name)) for name in DEVICE_NAMES)
    return values


class Room315VlaDatasetRecorder(Node):
    def __init__(self) -> None:
        super().__init__('room_315_vla_dataset_recorder')

        self.declare_parameter('dataset_dir', '~/.ros/room315_vla_datasets/smolvla_demo')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('right_image_topic', '/room_315/vla/right_rail_rgbd/image')
        self.declare_parameter('left_image_topic', '/room_315/vla/left_rail_rgbd/image')
        self.declare_parameter('status_topic', '/room_315/vla/status')
        self.declare_parameter('goal_topic', '/room_315/vla/user_goal')
        self.declare_parameter('command_topic', '/room_315/vla/command')
        self.declare_parameter('control_topic', '/room_315/vla/episode_control')
        self.declare_parameter('recorder_status_topic', '/room_315/vla/dataset_status')
        self.declare_parameter('sample_period_s', 0.2)
        self.declare_parameter('auto_start_on_goal', True)
        self.declare_parameter('image_max_width', 640)
        self.declare_parameter('image_jpeg_quality', 85)
        self.declare_parameter('camera_name', 'legacy_primary_rgb')

        raw_dataset_dir = str(self.get_parameter('dataset_dir').value)
        self.dataset_dir = Path(os.path.expandvars(raw_dataset_dir)).expanduser()
        self.meta_dir = self.dataset_dir / 'meta'
        self.episodes_dir = self.dataset_dir / 'episodes'
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        self.episodes_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        primary_camera_name = str(self.get_parameter('camera_name').value)
        self.camera_topics = {}
        for camera_name, topic in {
            primary_camera_name: str(self.get_parameter('image_topic').value),
            'right_rail_rgb': str(self.get_parameter('right_image_topic').value),
            'left_rail_rgb': str(self.get_parameter('left_image_topic').value),
        }.items():
            topic = topic.strip()
            if topic:
                self.camera_topics[camera_name] = topic
        self.latest_images: dict[str, Image] = {}
        self.latest_status: dict[str, Any] = {}
        self.latest_goal = 'unspecified Room 315 rail task'
        self.latest_task_index = -1
        self.latest_command: Any = {'action': 'status'}

        self.active = False
        self.episode_index = self._next_episode_index()
        self.episode_dir: Path | None = None
        self.image_dirs: dict[str, Path] = {}
        self.data_stream = None
        self.frame_index = 0
        self.started_at = ''
        self.last_error = ''

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('recorder_status_topic').value),
            10,
        )
        for camera_name, topic in self.camera_topics.items():
            self._subscribe_image(camera_name, topic)
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('goal_topic').value),
            self._on_goal,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('command_topic').value),
            self._on_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('control_topic').value),
            self._on_control,
            10,
        )

        self._write_dataset_info()
        sample_period_s = max(float(self.get_parameter('sample_period_s').value), 0.05)
        self.create_timer(sample_period_s, self._record_sample)
        self.create_timer(1.0, lambda: self._publish_status('idle' if not self.active else 'recording'))
        self._publish_status('ready')
        self.get_logger().info(f'Room 315 VLA dataset recorder ready: {self.dataset_dir}')

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

    def _on_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.latest_status = parsed

    def _on_goal(self, msg: String) -> None:
        goal = msg.data.strip()
        if not goal:
            return
        self.latest_goal = goal
        self.latest_task_index = self._task_index(goal)
        if _as_bool(self.get_parameter('auto_start_on_goal').value) and not self.active:
            self._start_episode(goal)
        self._publish_status('goal_received')

    def _on_command(self, msg: String) -> None:
        self.latest_command = _parse_json_or_text(msg.data)
        self._publish_status('command_received')

    def _on_control(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return
        lowered = command.lower()
        if lowered.startswith('start'):
            goal = command[5:].strip() or self.latest_goal
            self.latest_goal = goal
            self.latest_task_index = self._task_index(goal)
            self._start_episode(goal)
        elif lowered.startswith('stop'):
            success = None
            if 'success' in lowered:
                success = True
            elif 'failure' in lowered or 'fail' in lowered:
                success = False
            self._stop_episode(success=success, discarded=False)
        elif lowered.startswith('discard'):
            self._stop_episode(success=None, discarded=True)
        else:
            self.last_error = f'unknown episode control command: {command}'
            self.get_logger().warning(self.last_error)
            self._publish_status('error')

    def _start_episode(self, goal: str) -> None:
        if self.active:
            self._stop_episode(success=None, discarded=False)
        self.latest_goal = goal.strip() or 'unspecified Room 315 rail task'
        self.latest_task_index = self._task_index(self.latest_goal)
        suffix = _safe_filename(goal, 'room315_task')
        episode_name = f'episode_{self.episode_index:06d}_{suffix}'
        self.episode_dir = self.episodes_dir / episode_name
        self.image_dirs = {}
        for camera_name in self.camera_topics:
            image_dir = self.episode_dir / 'images' / camera_name
            image_dir.mkdir(parents=True, exist_ok=True)
            self.image_dirs[camera_name] = image_dir
        self.data_stream = (self.episode_dir / 'data.jsonl').open('w', encoding='utf-8')
        self.frame_index = 0
        self.started_at = _utc_now()
        self.active = True
        self.last_error = ''
        self.get_logger().info(f'Started VLA dataset episode {self.episode_index}: {goal}')
        self._publish_status('started')

    def _stop_episode(self, *, success: bool | None, discarded: bool) -> None:
        if not self.active:
            return
        if self.data_stream is not None:
            self.data_stream.close()
            self.data_stream = None
        summary = {
            'episode_index': self.episode_index,
            'task': self.latest_goal,
            'task_index': self.latest_task_index,
            'started_at': self.started_at,
            'ended_at': _utc_now(),
            'frames': self.frame_index,
            'success': success,
            'discarded': discarded,
            'format': 'room315_vla_lerobot_ready_jsonl_v1',
        }
        if self.episode_dir is not None:
            with (self.episode_dir / 'episode.json').open('w', encoding='utf-8') as stream:
                json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
        self.get_logger().info(
            f'Stopped VLA dataset episode {self.episode_index}: frames={self.frame_index}'
        )
        self.episode_index += 1
        self.active = False
        self.episode_dir = None
        self.image_dirs = {}
        self._publish_status('discarded' if discarded else 'stopped')

    def _record_sample(self) -> None:
        if not self.active or not self.latest_images or self.data_stream is None:
            return
        try:
            image_relpaths = {
                camera_name: self._write_image(camera_name, image)
                for camera_name, image in self.latest_images.items()
                if camera_name in self.image_dirs
            }
            row = {
                'episode_index': self.episode_index,
                'frame_index': self.frame_index,
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'task': self.latest_goal,
                'task_index': self.latest_task_index,
                'observation.state': _encode_state(self.latest_status),
                'observation.state_schema': OBSERVATION_STATE_FIELDS,
                'action': _encode_action(self.latest_command),
                'action_schema': ACTION_VECTOR_FIELDS,
                'command': self.latest_command,
                'supervisor_status': self.latest_status,
            }
            for camera_name, image_relpath in image_relpaths.items():
                row[f'observation.images.{camera_name}'] = image_relpath
            self.data_stream.write(_json_dumps(row) + '\n')
            self.data_stream.flush()
            self.frame_index += 1
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().error(f'Failed to record VLA dataset sample: {exc}')
            self._publish_status('error')

    def _write_image(self, camera_name: str, image: Image) -> str:
        if camera_name not in self.image_dirs or self.episode_dir is None:
            raise RuntimeError('episode image directory is not ready')
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        max_width = int(self.get_parameter('image_max_width').value)
        if max_width > 0 and frame.shape[1] > max_width:
            scale = max_width / float(frame.shape[1])
            frame = cv2.resize(
                frame,
                (max_width, max(1, int(frame.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )
        filename = f'{self.frame_index:06d}.jpg'
        image_path = self.image_dirs[camera_name] / filename
        quality = min(max(int(self.get_parameter('image_jpeg_quality').value), 1), 100)
        ok = cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError(f'failed to write image {image_path}')
        return str(image_path.relative_to(self.dataset_dir))

    def _write_dataset_info(self) -> None:
        info = {
            'format': 'room315_vla_lerobot_ready_jsonl_v1',
            'created_or_updated_at': _utc_now(),
            'description': (
                'Room 315 VLA demonstrations for adapting open-source policies '
                'such as SmolVLA/LeRobot to discrete industrial rail-cell control.'
            ),
            'fps': round(1.0 / max(float(self.get_parameter('sample_period_s').value), 0.05), 3),
            'image_features': {
                f'observation.images.{camera_name}': {
                    'encoding': 'jpg',
                    'source_topic': topic,
                }
                for camera_name, topic in self.camera_topics.items()
            },
            'state_features': OBSERVATION_STATE_FIELDS,
            'action_features': ACTION_VECTOR_FIELDS,
            'topics': {
                'goal': str(self.get_parameter('goal_topic').value),
                'command': str(self.get_parameter('command_topic').value),
                'status': str(self.get_parameter('status_topic').value),
                'control': str(self.get_parameter('control_topic').value),
                'right_image': str(self.get_parameter('right_image_topic').value),
                'left_image': str(self.get_parameter('left_image_topic').value),
            },
        }
        with (self.meta_dir / 'info.json').open('w', encoding='utf-8') as stream:
            json.dump(info, stream, ensure_ascii=False, indent=2, sort_keys=True)

    def _task_index(self, task: str) -> int:
        tasks_path = self.meta_dir / 'tasks.jsonl'
        task = task.strip() or 'unspecified Room 315 rail task'
        tasks: dict[str, int] = {}
        if tasks_path.exists():
            with tasks_path.open('r', encoding='utf-8') as stream:
                for line in stream:
                    try:
                        item = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if isinstance(item, dict) and isinstance(item.get('task'), str):
                        tasks[item['task']] = int(item.get('task_index', len(tasks)))
        if task in tasks:
            return tasks[task]
        task_index = len(tasks)
        with tasks_path.open('a', encoding='utf-8') as stream:
            stream.write(_json_dumps({'task_index': task_index, 'task': task}) + '\n')
        return task_index

    def _next_episode_index(self) -> int:
        max_index = -1
        for path in self.episodes_dir.glob('episode_*'):
            parts = path.name.split('_')
            if len(parts) >= 2 and parts[1].isdigit():
                max_index = max(max_index, int(parts[1]))
        return max_index + 1

    def _publish_status(self, state: str) -> None:
        msg = String()
        msg.data = _json_dumps({
            'state': state,
            'active': self.active,
            'dataset_dir': str(self.dataset_dir),
            'episode_index': self.episode_index,
            'frame_index': self.frame_index,
            'task': self.latest_goal,
            'task_index': self.latest_task_index,
            'available_cameras': sorted(self.latest_images),
            'last_error': self.last_error,
        })
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop_episode(success=None, discarded=False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Room315VlaDatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
