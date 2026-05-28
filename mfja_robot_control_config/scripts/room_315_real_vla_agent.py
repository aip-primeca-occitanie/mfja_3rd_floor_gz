#!/usr/bin/env python3

import base64
import json
import os
import re
import time
import urllib.error
import urllib.request
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


ALLOWED_ACTIONS = (
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

def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)



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


def _parse_command_payload(raw: Any) -> dict[str, Any]:
    if isinstance(raw, str):
        parsed = json.loads(raw)
    else:
        parsed = raw
    if isinstance(parsed, dict) and isinstance(parsed.get('command'), (dict, str)):
        parsed = parsed['command']
        if isinstance(parsed, str):
            parsed = json.loads(parsed)
    if not isinstance(parsed, dict):
        raise ValueError('VLA provider must return a JSON object command')
    action = str(parsed.get('action', '')).strip()
    if action not in ALLOWED_ACTIONS:
        raise ValueError(f'unsupported VLA action {action!r}')
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
        response = self._post_json(
            endpoint,
            {
                'goal': goal,
                'supervisor_status': self.latest_status,
                'image_jpeg_b64': next(iter(images_b64.values())),
                'images_jpeg_b64': images_b64,
                'allowed_actions': ALLOWED_ACTIONS,
            },
            headers={},
        )
        return _parse_command_payload(response)

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
