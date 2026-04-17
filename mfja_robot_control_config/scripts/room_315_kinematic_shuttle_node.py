#!/usr/bin/env python3

import json
import math
import re
from dataclasses import asdict
from pathlib import Path
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String

from room_315_kinematic_shuttle import (
    FALLING,
    KinematicShuttleCore,
    MOVING,
    RailNetwork,
    ShuttlePose,
    ShuttleState,
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


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class Room315KinematicShuttleNode(Node):
    def __init__(self) -> None:
        super().__init__('room_315_kinematic_shuttle')

        self.declare_parameter('network_yaml', str(_default_network_path()))
        self.declare_parameter('initial_segment', 'A14')
        self.declare_parameter('initial_s', 0.0)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('pose_topic', '/room_315/shuttle/pose_cmd')
        self.declare_parameter('state_topic', '/room_315/shuttle/state')
        self.declare_parameter('switch_command_topic', '/room_315/switch_states')
        self.declare_parameter('pose_offset_command_topic', '/room_315/shuttle/pose_offset_cmd')
        self.declare_parameter('visual_switch_command_topic', '/mfja/conveyor/switch_cmd')
        self.declare_parameter('visual_switch_state_topic', '/mfja/conveyor/switch_states')
        self.declare_parameter('sync_from_visual_switch_states', True)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('enable_gazebo_set_pose', False)
        self.declare_parameter('gazebo_set_pose_service', '/world/room_315_only/set_pose')
        self.declare_parameter('gazebo_entity_name', 'room315_shuttle_1')
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
        initial_segment = str(self.get_parameter('initial_segment').value)
        initial_s = float(self.get_parameter('initial_s').value)
        speed = float(self.get_parameter('speed').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        switch_command_topic = str(self.get_parameter('switch_command_topic').value)
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
        gazebo_set_pose_service = str(self.get_parameter('gazebo_set_pose_service').value)
        self.gazebo_entity_name = str(self.get_parameter('gazebo_entity_name').value)
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

        self.network = RailNetwork.from_yaml(network_path)
        self.switch_states: Dict[str, str] = self.network.default_switch_states()
        self.core = KinematicShuttleCore(
            network=self.network,
            initial_state=ShuttleState(
                current_segment=initial_segment,
                s=initial_s,
                speed=speed,
                mode=MOVING,
            ),
        )

        self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.state_publisher = self.create_publisher(String, state_topic, 10)
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
        self.pending_set_pose = None
        self.set_pose_warning_logged = False
        self.last_gazebo_set_pose_time = self.get_clock().now()
        if self.enable_gazebo_set_pose:
            self.set_pose_client = self.create_client(SetEntityPose, gazebo_set_pose_service)

        self.last_tick = self.get_clock().now()
        timer_period = 1.0 / max(update_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            'Room 315 kinematic shuttle started with '
            f'network={network_path}, pose_topic={pose_topic}, '
            f'switch_topic={switch_command_topic}, '
            f'offset_topic={pose_offset_command_topic}, '
            f'visual_switch_topic={visual_switch_command_topic}, '
            f'visual_switch_state_topic={visual_switch_state_topic}'
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

    def _on_switch_command(self, message: String) -> None:
        try:
            updates, visual_command = self._parse_switch_command(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        if updates:
            self.switch_states.update(updates)
            self.get_logger().info(f'Updated route switch states: {self.switch_states}')

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
                f'Synced route switch states from visual controller: {self.switch_states}'
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
            return station_match.group(1), 'station'

        short_match = re.match(r'^(A[1-4])([RL])$', name)
        if short_match:
            return short_match.group(1), 'right' if short_match.group(2) == 'R' else 'left'

        gazebo_match = re.match(r'^(A[1-4])_(DROIT|GAUCHE)_SWITCH$', name)
        if gazebo_match:
            return gazebo_match.group(1), 'right' if gazebo_match.group(2) == 'DROIT' else 'left'

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
        if selector_name in {'ALL', 'RIGHT', 'LEFT'}:
            return sorted(self.network.switches)

        station_match = re.match(r'^(A[1-4])([RL])?$', selector_name)
        if station_match:
            return [station_match.group(1)]

        return []

    def _visual_selector_for_selector(self, selector_name: str) -> str | None:
        if selector_name in {'ALL', 'RIGHT', 'LEFT'}:
            return selector_name

        station_match = re.match(r'^(A[1-4])([RL])?$', selector_name)
        if station_match:
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
        }
        boolean_parameters = {
            'enable_gazebo_pose_transform',
            'publish_visual_switch_commands',
        }

        try:
            for parameter in parameters:
                if parameter.name in numeric_parameters:
                    if parameter.name == 'gazebo_set_pose_rate_hz':
                        rate = float(parameter.value)
                        self.gazebo_set_pose_period = 1.0 / max(rate, 1.0)
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

        pose = self.core.step(dt, switch_states=self.switch_states)
        gazebo_pose = self._to_gazebo_pose(pose)
        pose_message = self._publish_pose(gazebo_pose)
        self._send_gazebo_pose(pose_message)
        self._publish_state(pose, gazebo_pose)

        if pose.mode == FALLING:
            self.get_logger().error(
                'Shuttle entered FALLING mode at '
                f'segment={pose.current_segment}, s={pose.s:.3f}'
            )

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

    def _publish_pose(self, pose: ShuttlePose) -> PoseStamped:
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
        self.pose_publisher.publish(message)
        return message

    def _send_gazebo_pose(self, pose_message: PoseStamped) -> None:
        if not self.enable_gazebo_set_pose or self.set_pose_client is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.last_gazebo_set_pose_time).nanoseconds / 1e9
        if elapsed < self.gazebo_set_pose_period:
            return
        if self.pending_set_pose is not None and not self.pending_set_pose.done():
            return
        if not self.set_pose_client.service_is_ready():
            if not self.set_pose_warning_logged:
                self.get_logger().warn(
                    'Gazebo set_pose service is not ready yet; still publishing pose topic.'
                )
                self.set_pose_warning_logged = True
            return

        request = SetEntityPose.Request()
        request.entity.name = self.gazebo_entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose_message.pose
        self.pending_set_pose = self.set_pose_client.call_async(request)
        self.last_gazebo_set_pose_time = now

    def _publish_state(self, pose: ShuttlePose, gazebo_pose: ShuttlePose) -> None:
        message = String()
        message.data = json.dumps(
            {
                **asdict(pose),
                'gazebo_pose': asdict(gazebo_pose),
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
                'switch_states': self.switch_states,
            },
            sort_keys=True,
        )
        self.state_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = Room315KinematicShuttleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
