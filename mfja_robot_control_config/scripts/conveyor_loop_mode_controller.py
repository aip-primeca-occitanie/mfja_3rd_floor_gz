#!/usr/bin/env python3

import argparse
import math
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass
class SwitchPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


MODE_YAWS: Dict[str, Dict[str, float]] = {
    'petit_boucle': {
        'A1_droit_switch': -1.5877,
        'A2_droit_switch': -1.0587,
        'A3_droit_switch': 2.08919,
        'A4_droit_switch': 2.60106,
        'A1_gauche_switch': -0.540815,
        'A2_gauche_switch': -1.0587,
        'A3_gauche_switch': 2.08919,
        'A4_gauche_switch': 1.55349,
    },
    'grand_boucle': {
        'A1_droit_switch': 0.50666,
        'A2_droit_switch': 3.13,
        'A3_droit_switch': -2.1,
        'A4_droit_switch': 0.50666,
        'A1_gauche_switch': -2.63653,
        'A2_gauche_switch': 1.04,
        'A3_gauche_switch': -0.025,
        'A4_gauche_switch': -2.63653,
    },
}

SWITCH_ORDER = tuple(MODE_YAWS['grand_boucle'].keys())

MODE_ALIASES = {
    'grand_boucle': 'grand_boucle',
    'grand': 'grand_boucle',
    'grande_boucle': 'grand_boucle',
    'large': 'grand_boucle',
    'big': 'grand_boucle',
    '1': 'grand_boucle',
    'petit_boucle': 'petit_boucle',
    'petit': 'petit_boucle',
    'small': 'petit_boucle',
    'mini': 'petit_boucle',
    '0': 'petit_boucle',
}


def _canonical_mode_label(mode: str) -> str:
    return mode.upper()


def _normalize_mode(raw_value: str) -> Optional[str]:
    normalized = re.sub(r'[^a-z0-9]+', '_', raw_value.strip().lower()).strip('_')
    return MODE_ALIASES.get(normalized)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angles_match(first: float, second: float, tolerance: float = 0.03) -> bool:
    return abs(_normalize_angle(first - second)) <= tolerance


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def _resolve_world_file(path_hint: str, world_name: str) -> str:
    if path_hint:
        return path_hint if os.path.isabs(path_hint) else os.path.abspath(path_hint)

    description_pkg = get_package_share_directory('mfja_3rd_floor_description')
    world_file_stem = 'mfja_3rd_floor' if world_name == 'default' else world_name
    return os.path.join(description_pkg, 'worlds', world_file_stem + '.world')


def _parse_pose(pose_text: str) -> SwitchPose:
    values = [float(token) for token in pose_text.split()]
    if len(values) == 3:
        values.extend([0.0, 0.0, 0.0])
    if len(values) != 6:
        raise RuntimeError(
            f'Expected pose with 3 or 6 values, but received: "{pose_text}"'
        )
    return SwitchPose(*values)


def _load_switch_layout(world_file: str) -> Dict[str, SwitchPose]:
    if not os.path.exists(world_file):
        raise RuntimeError(f'World file does not exist: {world_file}')

    tree = ET.parse(world_file)
    root = tree.getroot()
    world_element = root.find('world')
    if world_element is None:
        raise RuntimeError(f'No <world> element found in: {world_file}')

    layout = {}
    for include in world_element.findall('include'):
        name_element = include.find('name')
        pose_element = include.find('pose')
        if name_element is None or pose_element is None:
            continue
        entity_name = (name_element.text or '').strip()
        if entity_name not in SWITCH_ORDER:
            continue
        layout[entity_name] = _parse_pose(pose_element.text or '')

    return layout


def _detect_mode_from_layout(layout: Dict[str, SwitchPose]) -> Optional[str]:
    if any(name not in layout for name in SWITCH_ORDER):
        return None

    for mode_name, target_yaws in MODE_YAWS.items():
        if all(
            _angles_match(layout[name].yaw, target_yaws[name])
            for name in SWITCH_ORDER
        ):
            return mode_name
    return None


class ConveyorLoopModeController(Node):
    def __init__(self, args):
        super().__init__('conveyor_loop_mode_controller')
        self.world_name = args.world
        self.world_file = _resolve_world_file(args.world_file, self.world_name)
        self.partition = args.partition
        self.timeout_ms = args.timeout_ms
        self.retries = args.retries
        self.command_topic = args.command_topic
        self.state_topic = args.state_topic

        self.switch_layout = _load_switch_layout(self.world_file)
        self.managed_switches = [
            switch_name for switch_name in SWITCH_ORDER
            if switch_name in self.switch_layout
        ]
        self.current_mode = _detect_mode_from_layout(self.switch_layout)

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.state_publisher = self.create_publisher(String, self.state_topic, state_qos)
        self.command_subscription = self.create_subscription(
            String, self.command_topic, self._handle_command, 10
        )

        if self.managed_switches:
            self.get_logger().info(
                f'Loaded {len(self.managed_switches)} managed switch poses from {self.world_file}.'
            )
        else:
            self.get_logger().warning(
                'No managed loop switches were found in the selected world file. '
                'Incoming commands will be ignored.'
            )

        missing_switches = [
            switch_name for switch_name in SWITCH_ORDER
            if switch_name not in self.switch_layout
        ]
        if missing_switches:
            self.get_logger().warning(
                'Missing managed switches in world layout: ' + ', '.join(missing_switches)
            )

        if self.current_mode is not None:
            self.get_logger().info(
                'Detected initial loop mode from world file: '
                f'{_canonical_mode_label(self.current_mode)}'
            )
            self._publish_state_label(_canonical_mode_label(self.current_mode))
        else:
            self.get_logger().info(
                'Initial world layout does not exactly match PETIT_BOUCLE or GRAND_BOUCLE. '
                f'Listening on {self.command_topic} for explicit mode commands.'
            )

    def _handle_command(self, msg: String):
        requested_mode = _normalize_mode(msg.data)
        if requested_mode is None:
            self.get_logger().warning(
                f'Unsupported loop mode command "{msg.data}". '
                'Use PETIT_BOUCLE or GRAND_BOUCLE.'
            )
            return

        self._apply_mode(requested_mode, source=f'topic command "{msg.data}"')

    def _apply_mode(self, mode: str, source: str):
        if not self.managed_switches:
            self.get_logger().warning(
                f'Ignoring {source} because no managed switches are available.'
            )
            return

        self.get_logger().info(
            f'Applying {_canonical_mode_label(mode)} to {len(self.managed_switches)} switches from {source}.'
        )

        world_was_paused_by_controller = False
        pause_ok, pause_output = self._set_world_pause(True)
        if pause_ok:
            world_was_paused_by_controller = True
        else:
            pause_details = pause_output or 'no diagnostic output returned by gz service'
            self.get_logger().warning(
                'Failed to pause the Gazebo world before switching modes. '
                f'Continuing anyway: {pause_details}'
            )

        try:
            failures = self._set_switch_poses_parallel(mode)
        finally:
            if world_was_paused_by_controller:
                resume_ok, resume_output = self._set_world_pause(False)
                if not resume_ok:
                    resume_details = (
                        resume_output or 'no diagnostic output returned by gz service'
                    )
                    self.get_logger().error(
                        'Failed to resume the Gazebo world after switching modes: '
                        f'{resume_details}'
                    )

        if failures:
            self.current_mode = None
            self._publish_state_label('UNKNOWN')
            for switch_name, output in failures:
                details = output or 'no diagnostic output returned by gz service'
                self.get_logger().error(
                    f'Failed to apply {_canonical_mode_label(mode)} to {switch_name}: {details}'
                )
            return

        self.current_mode = mode
        self._publish_state_label(_canonical_mode_label(mode))
        self.get_logger().info(f'Applied {_canonical_mode_label(mode)} successfully.')

    def _build_set_pose_command(self, switch_name: str, switch_pose: SwitchPose, target_yaw: float):
        qx, qy, qz, qw = _quaternion_from_rpy(
            switch_pose.roll,
            switch_pose.pitch,
            target_yaw,
        )
        request = (
            f'name: "{switch_name}", '
            f'position: {{x: {switch_pose.x}, y: {switch_pose.y}, z: {switch_pose.z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.timeout_ms),
            '--req', request,
        ]
        return command

    def _set_world_pause(self, paused: bool):
        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.timeout_ms),
            '--req', f'pause: {"true" if paused else "false"}',
        ]
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition

        for _ in range(self.retries):
            completed = subprocess.run(
                command,
                check=False,
                env=environment,
                text=True,
                capture_output=True,
            )
            output = '\n'.join(
                part for part in [completed.stdout, completed.stderr] if part
            ).strip()
            lowered_output = output.lower()
            timed_out = 'timed out' in lowered_output
            returned_false = 'data: false' in lowered_output
            success = completed.returncode == 0 and not timed_out and not returned_false
            if success:
                return True, output

        return False, output

    def _set_switch_poses_parallel(self, mode: str):
        pending = {
            switch_name: (
                self.switch_layout[switch_name],
                MODE_YAWS[mode][switch_name],
            )
            for switch_name in self.managed_switches
        }
        failures = {}
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition

        for attempt in range(1, self.retries + 1):
            if not pending:
                break

            launched = {}
            for switch_name, (switch_pose, target_yaw) in pending.items():
                command = self._build_set_pose_command(
                    switch_name=switch_name,
                    switch_pose=switch_pose,
                    target_yaw=target_yaw,
                )
                launched[switch_name] = (
                    subprocess.Popen(
                        command,
                        env=environment,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                    ),
                    switch_pose,
                    target_yaw,
                )

            next_pending = {}
            failures.clear()

            for switch_name, (process, switch_pose, target_yaw) in launched.items():
                stdout, stderr = process.communicate()
                output = '\n'.join(part for part in [stdout, stderr] if part).strip()
                lowered_output = output.lower()
                timed_out = 'timed out' in lowered_output
                returned_false = 'data: false' in lowered_output
                success = process.returncode == 0 and not timed_out and not returned_false

                if success:
                    self.switch_layout[switch_name] = SwitchPose(
                        x=switch_pose.x,
                        y=switch_pose.y,
                        z=switch_pose.z,
                        roll=switch_pose.roll,
                        pitch=switch_pose.pitch,
                        yaw=target_yaw,
                    )
                    continue

                failures[switch_name] = output
                next_pending[switch_name] = (switch_pose, target_yaw)

            pending = next_pending
            if pending and attempt < self.retries:
                self.get_logger().warning(
                    f'Parallel switch update round {attempt} failed for '
                    f'{len(pending)} switch(es); retrying the remaining requests.'
                )

        return [(switch_name, failures.get(switch_name, '')) for switch_name in pending]

    def _publish_state_label(self, label: str):
        msg = String()
        msg.data = label
        self.state_publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Subscribe to a high-level ROS 2 loop-mode topic and switch all MFJA conveyor '
            'rail blades between PETIT_BOUCLE and GRAND_BOUCLE.'
        )
    )
    parser.add_argument(
        '--world',
        default='default',
        help='Gazebo world entity name used by /world/<name>/set_pose.',
    )
    parser.add_argument(
        '--world-file',
        default='',
        help=(
            'Absolute or relative path to the world file. If omitted, the script resolves '
            'mfja_3rd_floor.world when --world is "default".'
        ),
    )
    parser.add_argument(
        '--partition',
        default=os.environ.get('GZ_PARTITION', 'rail_switch_demo'),
        help='Gazebo transport partition. Defaults to $GZ_PARTITION or rail_switch_demo.',
    )
    parser.add_argument(
        '--command-topic',
        default='/mfja/conveyor/loop_mode_cmd',
        help='ROS 2 std_msgs/String topic used to request PETIT_BOUCLE or GRAND_BOUCLE.',
    )
    parser.add_argument(
        '--state-topic',
        default='/mfja/conveyor/loop_mode',
        help='ROS 2 std_msgs/String topic that republishes the last known applied mode.',
    )
    parser.add_argument(
        '--timeout-ms',
        type=int,
        default=1500,
        help='Timeout passed to each gz set_pose request.',
    )
    parser.add_argument(
        '--retries',
        type=int,
        default=3,
        help='How many times to retry each per-switch set_pose request.',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ConveyorLoopModeController(args)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
