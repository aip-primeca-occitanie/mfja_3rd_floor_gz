#!/usr/bin/env python3

import argparse
import os
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


MODEL_PRESETS: Dict[str, Dict[str, List[float]]] = {
    'kuka_kr6r900sixx': {
        'joint_names': ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6'],
        'positions': [0.8, -1.0, 1.2, 0.0, 0.4, 0.0],
    },
    'staubli_tx2_60l': {
        'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        'positions': [0.0, 0.3, -0.5, 0.0, 0.6, 0.0],
    },
    'yaskawa_hc10': {
        'joint_names': ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'],
        'positions': [0.0, -0.6, 0.8, 0.0, 0.5, 0.0],
    },
    'yaskawa_hc10dt': {
        'joint_names': ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'],
        'positions': [0.0, -0.5, 0.7, 0.0, 0.4, 0.0],
    },
    'tiago': {
        'joint_names': [
            'torso_lift_joint',
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint',
            'head_1_joint',
            'head_2_joint',
        ],
        'positions': [0.10, 0.3, -0.5, -0.4, 1.0, 0.2, -0.2, 0.1, 0.2, -0.2],
    },
}


def _resolve_robot_config(path_hint: str) -> str:
    if os.path.isabs(path_hint):
        return path_hint
    pkg_path = get_package_share_directory('mfja_3rd_floor_gz')
    return os.path.join(pkg_path, path_hint)


def _load_enabled_robots(config_path: str):
    with open(config_path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return [robot for robot in data.get('robots', []) if robot.get('enabled', True)]


def _parse_goal_overrides(goal_entries: List[str]) -> Dict[str, List[float]]:
    overrides = {}
    for entry in goal_entries:
        if '=' not in entry:
            raise RuntimeError(
                f'Invalid --goal "{entry}". Expected format: robot_name=v1,v2,v3'
            )
        robot_name, values = entry.split('=', 1)
        robot_name = robot_name.strip()
        positions = [float(value.strip()) for value in values.split(',') if value.strip()]
        if not robot_name or not positions:
            raise RuntimeError(
                f'Invalid --goal "{entry}". Expected format: robot_name=v1,v2,v3'
            )
        overrides[robot_name] = positions
    return overrides


def _build_robot_targets(enabled_robots, goal_overrides: Dict[str, List[float]]):
    targets = []
    enabled_by_name = {str(robot['name']): robot for robot in enabled_robots}
    enabled_names = set(enabled_by_name)

    for robot_name in goal_overrides:
        if robot_name not in enabled_names:
            raise RuntimeError(
                f'Goal override provided for "{robot_name}", but it is not enabled in robots.yaml.'
            )

    if goal_overrides:
        robots_to_command = [enabled_by_name[robot_name] for robot_name in goal_overrides]
    else:
        robots_to_command = enabled_robots

    for robot in robots_to_command:
        robot_name = str(robot['name'])
        model_name = str(robot.get('model', ''))
        preset = MODEL_PRESETS.get(model_name)
        if preset is None:
            continue

        positions = goal_overrides.get(robot_name, preset['positions'])
        if len(positions) != len(preset['joint_names']):
            raise RuntimeError(
                f'Robot "{robot_name}" expects {len(preset["joint_names"])} values '
                f'but received {len(positions)}.'
            )

        targets.append({
            'name': robot_name,
            'model': model_name,
            'joint_names': list(preset['joint_names']),
            'positions': list(positions),
        })

    return targets


def _print_joint_reference(config_path: str):
    enabled_robots = _load_enabled_robots(config_path)
    print(f'Enabled robots from: {config_path}')
    for robot in enabled_robots:
        robot_name = str(robot['name'])
        model_name = str(robot.get('model', ''))
        preset = MODEL_PRESETS.get(model_name)
        if preset is None:
            print(f'- {robot_name}: model "{model_name}" has no preset in multi_robot_sync_demo.py')
            continue
        print(f'- {robot_name} ({model_name})')
        print(f'  joints: {", ".join(preset["joint_names"])}')
        print(f'  default: {preset["positions"]}')


class MultiRobotSyncDemo(Node):
    def __init__(self, args):
        super().__init__('multi_robot_sync_demo')
        self.args = args
        self.start_time = self.get_clock().now()
        self.started = False
        self.active_base_motion = False
        self.shutdown_deadline = None
        self.base_stop_time = None

        config_path = _resolve_robot_config(args.robot_config)
        enabled_robots = _load_enabled_robots(config_path)
        goal_overrides = _parse_goal_overrides(args.goal)
        self.get_logger().info(f'Loaded robot config: {config_path}')

        self.robot_publishers = {}
        self.robot_targets = _build_robot_targets(enabled_robots, goal_overrides)
        self.tiago_base_publisher = None

        for robot in self.robot_targets:
            robot_name = robot['name']
            topic = f'/{robot_name}/joint_trajectory'
            publisher = self.create_publisher(JointTrajectory, topic, 10)
            self.robot_publishers[robot_name] = publisher

            if robot['model'] == 'tiago' and args.tiago_base_duration > 0.0:
                self.tiago_base_publisher = self.create_publisher(
                    Twist, f'/{robot_name}/cmd_vel', 10
                )

        if not self.robot_targets:
            raise RuntimeError('No enabled robots with known sync presets were found.')

        self.check_timer = self.create_timer(0.25, self._check_and_start)
        self.base_timer = self.create_timer(0.05, self._handle_tiago_base)
        self.finish_timer = self.create_timer(0.2, self._handle_shutdown)

    def _all_subscribers_ready(self) -> bool:
        for robot in self.robot_targets:
            publisher = self.robot_publishers[robot['name']]
            if publisher.get_subscription_count() < 1:
                return False
        if self.tiago_base_publisher is not None and self.tiago_base_publisher.get_subscription_count() < 1:
            return False
        return True

    def _check_and_start(self):
        if self.started:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self._all_subscribers_ready():
            self.get_logger().info('All command topics have subscribers. Publishing synchronized motion.')
            self._publish_joint_trajectories()
            self.started = True
            self.check_timer.cancel()
            if self.tiago_base_publisher is not None:
                self.active_base_motion = True
                self.base_stop_time = self.get_clock().now() + Duration(
                    seconds=self.args.tiago_base_duration
                )
                self.shutdown_deadline = self.base_stop_time + Duration(seconds=0.5)
            else:
                self.shutdown_deadline = self.get_clock().now() + Duration(seconds=1.0)
            return

        if elapsed >= self.args.wait_timeout:
            missing = []
            for robot in self.robot_targets:
                publisher = self.robot_publishers[robot['name']]
                if publisher.get_subscription_count() < 1:
                    missing.append(f'/{robot["name"]}/joint_trajectory')
            if self.tiago_base_publisher is not None and self.tiago_base_publisher.get_subscription_count() < 1:
                missing.append('TIAGo /cmd_vel')
            raise RuntimeError(
                'Timed out waiting for subscribers on: ' + ', '.join(missing)
            )

    def _publish_joint_trajectories(self):
        stamp = self.get_clock().now().to_msg()
        seconds = int(self.args.trajectory_duration)
        nanoseconds = int((self.args.trajectory_duration - seconds) * 1e9)

        for robot in self.robot_targets:
            msg = JointTrajectory()
            msg.header.stamp = stamp
            msg.joint_names = list(robot['joint_names'])

            point = JointTrajectoryPoint()
            point.positions = list(robot['positions'])
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            msg.points = [point]

            self.robot_publishers[robot['name']].publish(msg)
            self.get_logger().info(
                f'Published synchronized trajectory to {robot["name"]} ({robot["model"]}).'
            )

    def _handle_tiago_base(self):
        if not self.active_base_motion or self.tiago_base_publisher is None:
            return

        now = self.get_clock().now()
        if now >= self.base_stop_time:
            stop = Twist()
            self.tiago_base_publisher.publish(stop)
            self.active_base_motion = False
            self.get_logger().info('Published TIAGo base stop command.')
            return

        twist = Twist()
        twist.linear.x = self.args.tiago_base_linear
        twist.angular.z = self.args.tiago_base_angular
        self.tiago_base_publisher.publish(twist)

    def _handle_shutdown(self):
        if self.shutdown_deadline is None:
            return
        if self.get_clock().now() >= self.shutdown_deadline:
            self.get_logger().info('Multi-robot synchronized demo finished.')
            self.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Publish synchronized trajectories to enabled robots from config/robots.yaml. '
            'If --goal is omitted, all enabled robots with presets receive their default targets. '
            'If one or more --goal arguments are provided, only those named robots are commanded.'
        )
    )
    parser.add_argument(
        '--robot-config',
        default='config/robots.yaml',
        help='Absolute or package-relative path to the robot YAML file.',
    )
    parser.add_argument(
        '--trajectory-duration',
        type=float,
        default=4.0,
        help='Trajectory time_from_start in seconds for all arm commands.',
    )
    parser.add_argument(
        '--goal',
        action='append',
        default=[],
        help=(
            'Command one robot from the command line. Format: robot_name=v1,v2,v3. '
            'When one or more --goal arguments are present, only the listed robots move.'
        ),
    )
    parser.add_argument(
        '--list-joints',
        action='store_true',
        help='Print enabled robots with expected joint names and default positions, then exit.',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=20.0,
        help='Maximum time to wait for topic subscribers before failing.',
    )
    parser.add_argument(
        '--tiago-base-linear',
        type=float,
        default=0.0,
        help='Linear x speed for TIAGo base. 0 disables base motion unless angular is non-zero.',
    )
    parser.add_argument(
        '--tiago-base-angular',
        type=float,
        default=0.0,
        help='Angular z speed for TIAGo base.',
    )
    parser.add_argument(
        '--tiago-base-duration',
        type=float,
        default=0.0,
        help='How long to publish TIAGo base cmd_vel in seconds. 0 disables base motion.',
    )
    args, ros_args = parser.parse_known_args()

    config_path = _resolve_robot_config(args.robot_config)
    if args.list_joints:
        _print_joint_reference(config_path)
        return

    rclpy.init(args=ros_args)
    node = MultiRobotSyncDemo(args)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
