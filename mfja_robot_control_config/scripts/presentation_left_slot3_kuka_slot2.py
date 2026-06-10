#!/usr/bin/env python3

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from mfja_rail_interfaces.msg import NamedState
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleCommand
from mfja_rail_interfaces.msg import StopperCommand
from mfja_rail_interfaces.msg import SwitchCommand


LEFT_RAIL_PREFIX = '/room_315/rails/left'
LEFT_SLOT_SENSORS = {
    '2': 'DZI2L',
    '3': 'DZI3L',
}
KUKA_JOINT_NAMES = (
    'joint_a1',
    'joint_a2',
    'joint_a3',
    'joint_a4',
    'joint_a5',
    'joint_a6',
)
KUKA_PRESENTATION_POSITIONS_RAD = (
    1.57079632679,
    -0.52359877560,
    1.91986217719,
    0.69813170080,
    -0.03490658504,
    0.0,
)
KUKA_INITIAL_POSITIONS_RAD = (
    0.0,
    -1.57079632679,
    1.91986217719,
    0.0,
    -0.03490658504,
    0.0,
)


class LeftSlot3KukaSlot2Presentation(Node):
    def __init__(self) -> None:
        super().__init__('presentation_left_slot3_kuka_slot2')
        self.declare_parameter('shuttle_name', 'room315_left_shuttle_1')
        self.declare_parameter('shuttle_speed', 0.16)
        self.declare_parameter('slot_timeout_s', 90.0)
        self.declare_parameter('slot3_overshoot_m', 0.15)
        self.declare_parameter('kuka_duration_s', 4.0)
        self.declare_parameter('startup_delay_s', 2.0)

        self.active_sensors: set[str] = set()
        self.sensor_feedback_seen = False

        self.switch_pub = self.create_publisher(
            SwitchCommand,
            f'{LEFT_RAIL_PREFIX}/switches/command',
            10,
        )
        self.stopper_pub = self.create_publisher(
            StopperCommand,
            f'{LEFT_RAIL_PREFIX}/stoppers/command',
            10,
        )
        self.shuttle_pub = self.create_publisher(
            ShuttleCommand,
            f'{LEFT_RAIL_PREFIX}/shuttles/command',
            10,
        )
        self.kuka_pub = self.create_publisher(
            JointTrajectory,
            '/kuka1/joint_trajectory',
            10,
        )
        self.create_subscription(
            SensorFeedback,
            f'{LEFT_RAIL_PREFIX}/sensors/feedback',
            self._on_sensor_feedback,
            10,
        )

    def _on_sensor_feedback(self, message: SensorFeedback) -> None:
        self.active_sensors = {
            reading.name
            for reading in message.readings
            if bool(reading.active)
        }
        self.sensor_feedback_seen = True

    def _spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(float(seconds), 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_until(self, predicate, timeout_s: float, label: str) -> bool:
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and time.monotonic() <= deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warning(f'timeout waiting for {label}')
        return False

    @staticmethod
    def _named_state(name: str, state: str) -> NamedState:
        item = NamedState()
        item.name = name
        item.state = state
        return item

    def _prepare_left_exterior_route(self) -> None:
        switch_msg = SwitchCommand()
        switch_msg.switches = [self._named_state('ALL', 'EXTERIOR')]
        self.switch_pub.publish(switch_msg)

        stopper_msg = StopperCommand()
        stopper_msg.stoppers = [self._named_state('ALL', '0')]
        self.stopper_pub.publish(stopper_msg)
        self._spin_for(1.0)

    def _command_shuttle(self, command: str) -> None:
        message = ShuttleCommand()
        message.name = str(self.get_parameter('shuttle_name').value)
        message.command = command
        if command.upper() == 'ON':
            message.speed = float(self.get_parameter('shuttle_speed').value)
        self.shuttle_pub.publish(message)
        self._spin_for(0.3)

    def _active_slot(self, slot: str) -> bool:
        return LEFT_SLOT_SENSORS[slot] in self.active_sensors

    def _overshoot_after_slot_if_needed(self, slot: str) -> None:
        if slot != '3':
            return
        overshoot_m = max(0.0, float(self.get_parameter('slot3_overshoot_m').value))
        speed_mps = max(0.001, float(self.get_parameter('shuttle_speed').value))
        if overshoot_m <= 0.0:
            return
        duration_s = overshoot_m / speed_mps
        self.get_logger().info(
            f'left shuttle continuing {overshoot_m:.3f} m after slot {slot} '
            f'before stopping ({duration_s:.2f} s at {speed_mps:.3f} m/s)'
        )
        self._spin_for(duration_s)

    def _move_to_slot(self, slot: str) -> bool:
        sensor_name = LEFT_SLOT_SENSORS[slot]
        self.get_logger().info(f'moving left shuttle to slot {slot} ({sensor_name})')
        target_active_at_start = self._active_slot(slot)
        self._prepare_left_exterior_route()
        self._command_shuttle('ON')
        hit = False
        try:
            if target_active_at_start and not self._wait_until(
                lambda: not self._active_slot(slot),
                min(10.0, float(self.get_parameter('slot_timeout_s').value)),
                f'left shuttle to leave slot {slot}',
            ):
                return False
            hit = self._wait_until(
                lambda: self._active_slot(slot),
                float(self.get_parameter('slot_timeout_s').value),
                f'left slot {slot}',
            )
            if hit:
                self._overshoot_after_slot_if_needed(slot)
            return hit
        finally:
            self._command_shuttle('OFF')
            if hit:
                self.get_logger().info(f'left shuttle stopped at slot {slot}')

    def _command_kuka_pose(
        self,
        positions: tuple[float, ...],
        label: str,
    ) -> None:
        duration_s = float(self.get_parameter('kuka_duration_s').value)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(
            round((duration_s - int(duration_s)) * 1_000_000_000)
        )

        message = JointTrajectory()
        message.joint_names = list(KUKA_JOINT_NAMES)
        message.points = [point]
        self.get_logger().info(label)
        self.kuka_pub.publish(message)
        self._spin_for(duration_s + 0.5)

    def _command_kuka(self) -> None:
        self._command_kuka_pose(
            KUKA_PRESENTATION_POSITIONS_RAD,
            'moving KUKA to presentation pose: deg=[90, -30, 110, 40, -2, 0]',
        )

    def _command_kuka_initial(self) -> None:
        self._command_kuka_pose(
            KUKA_INITIAL_POSITIONS_RAD,
            'returning KUKA to initial pose: deg=[0, -90, 110, 0, -2, 0]',
        )

    def run(self) -> bool:
        self._spin_for(float(self.get_parameter('startup_delay_s').value))
        if not self._wait_until(
            lambda: self.sensor_feedback_seen,
            10.0,
            'left rail sensor feedback',
        ):
            return False
        if not self._move_to_slot('3'):
            return False
        self._spin_for(1.0)
        self._command_kuka()
        self._spin_for(1.0)
        self._command_kuka_initial()
        self._spin_for(1.0)
        return self._move_to_slot('2')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = LeftSlot3KukaSlot2Presentation()
    try:
        ok = node.run()
        return 0 if ok else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
