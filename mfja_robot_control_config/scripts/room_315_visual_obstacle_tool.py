#!/usr/bin/env python3
"""Move the Room 315 visual obstacle markers in Gazebo."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


ENTITY_BY_SIDE = {
    'right': 'room315_visual_right_obstacle_marker',
    'left': 'room315_visual_left_obstacle_marker',
}
DEFAULT_POSE_FILE = '~/.ros/room315_visual_obstacles.json'


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def build_pose_request(entity: str, x: float, y: float, z: float, yaw: float) -> str:
    qx, qy, qz, qw = _quaternion_from_yaw(yaw)
    return (
        f'name: "{entity}" '
        f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ x: {qx:.9f} y: {qy:.9f} z: {qz:.9f} w: {qw:.9f} }}'
    )


def build_set_pose_request(entity: str, x: float, y: float, z: float, yaw: float):
    qx, qy, qz, qw = _quaternion_from_yaw(yaw)
    request = SetEntityPose.Request()
    request.entity.name = entity
    request.entity.type = Entity.MODEL
    request.pose.position.x = float(x)
    request.pose.position.y = float(y)
    request.pose.position.z = float(z)
    request.pose.orientation.x = qx
    request.pose.orientation.y = qy
    request.pose.orientation.z = qz
    request.pose.orientation.w = qw
    return request


def _pose_file_path(raw_path: str) -> Path:
    return Path(raw_path).expanduser()


def _write_pose_file(path: Path, side: str, x: float, y: float, z: float, yaw: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {}
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding='utf-8'))
            if isinstance(loaded, dict):
                data = loaded
        except (OSError, json.JSONDecodeError):
            data = {}
    data[side] = {
        'x': float(x),
        'y': float(y),
        'z': float(z),
        'yaw': float(yaw),
        'updated_at_unix_s': round(time.time(), 3),
    }
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _pose_from_args(args: argparse.Namespace) -> tuple[float, float, float, float]:
    return args.x, args.y, args.z, args.yaw


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Move a Room 315 visual obstacle marker to explicit coordinates.',
    )
    parser.add_argument('--world', default='room_315_only', help='Gazebo world name.')
    parser.add_argument('--side', choices=sorted(ENTITY_BY_SIDE), required=True)
    parser.add_argument('--x', type=float, required=True, help='Obstacle x position.')
    parser.add_argument('--y', type=float, required=True, help='Obstacle y position.')
    parser.add_argument('--z', type=float, default=0.856, help='Obstacle z position.')
    parser.add_argument('--yaw', type=float, default=0.0, help='Obstacle yaw in radians.')
    parser.add_argument('--timeout-s', type=float, default=5.0)
    parser.add_argument(
        '--pose-file',
        default=DEFAULT_POSE_FILE,
        help='Pose cache path for reusable obstacle marker positions.',
    )
    parser.add_argument(
        '--no-pose-file',
        action='store_true',
        help='Move Gazebo only; do not update the obstacle pose cache.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the ROS set_pose request without calling the service.',
    )
    return parser


def _call_set_pose(service: str, request, timeout_s: float) -> bool:
    rclpy.init(args=None)
    node = rclpy.create_node('room_315_visual_obstacle_tool')
    try:
        client = node.create_client(SetEntityPose, service)
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            print(
                f'Service {service} is not available. '
                'Make sure Gazebo is running from room_315_only.launch.py.',
                file=sys.stderr,
            )
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=float(timeout_s))
        if not future.done():
            print(f'Service call to {service} timed out.', file=sys.stderr)
            return False
        response = future.result()
        if response is None:
            print(f'Service call to {service} returned no response.', file=sys.stderr)
            return False
        if not bool(response.success):
            print(
                f'Service {service} rejected the pose. '
                'Check that the obstacle entity exists in the running world.',
                file=sys.stderr,
            )
            return False
        return True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    entity = ENTITY_BY_SIDE[args.side]
    x, y, z, yaw = _pose_from_args(args)
    service = f'/world/{args.world}/set_pose'

    print(build_pose_request(entity, x, y, z, yaw))
    if args.dry_run:
        print(f'ROS service: {service} ({SetEntityPose.__name__})')
        if not args.no_pose_file:
            print(f'Pose cache: {_pose_file_path(args.pose_file)}')
        return 0
    request = build_set_pose_request(entity, x, y, z, yaw)
    if not _call_set_pose(service, request, args.timeout_s):
        return 1
    if not args.no_pose_file:
        path = _pose_file_path(args.pose_file)
        _write_pose_file(path, args.side, x, y, z, yaw)
        print(f'Updated pose cache: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
