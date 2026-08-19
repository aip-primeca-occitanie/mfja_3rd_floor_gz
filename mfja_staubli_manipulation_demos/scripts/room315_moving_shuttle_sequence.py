#!/usr/bin/env python3
"""Coordinate the simulated moving-shuttle Room 315 manipulation demo."""

import argparse
import re
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from mfja_rail_interfaces.msg import NamedState
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleCommand
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import StopperCommand
from mfja_rail_interfaces.msg import StopperState
from mfja_rail_interfaces.msg import SwitchCommand
from mfja_rail_interfaces.msg import SwitchState
from mfja_rail_interfaces.srv import AddShuttle

from room315_manipulation_sequence import (
    ManipulationCoordinator,
    add_manipulation_arguments,
    finalize_manipulation_args,
    format_pose,
    payload_pose_from_shuttle_pose,
    poses_close,
    run_hpp_cycle,
)


RIGHT_RAIL_PREFIX = "/room_315/rails/right"
DEFAULT_PICKUP_SHUTTLE_NAME = "room315_right_shuttle_1"
DEFAULT_DROP_SHUTTLE_NAME = "room315_right_shuttle_2"
DEFAULT_PICKUP_SENSOR = "DZI3R"
DEFAULT_DROP_SENSOR = "DZI4R"


def topic_safe_name(name):
    return re.sub(r"[^A-Za-z0-9_]+", "_", name)


class MovingShuttleCoordinator(ManipulationCoordinator):
    def __init__(self, args):
        super().__init__(args, "room315_moving_shuttle_sequence")
        self.latest_sensor_feedback = None
        self.latest_shuttle_states = {}
        self.shuttle_state_updates = {}
        self.latest_poses = {}
        self.pose_updates = {}
        self.latest_switch_states = {}
        self.switch_state_updates = 0
        self.latest_stopper_states = {}
        self.stopper_state_updates = 0
        self.last_payload_stream_time = 0.0
        self.pose_subscriptions = []
        self.active_shuttles = set()

        self.switch_publisher = self.create_publisher(
            SwitchCommand, args.switch_command_topic, 10
        )
        self.stopper_publisher = self.create_publisher(
            StopperCommand, args.stopper_command_topic, 10
        )
        self.shuttle_publisher = self.create_publisher(
            ShuttleCommand, args.shuttle_command_topic, 10
        )
        self.create_subscription(
            SensorFeedback, args.sensor_feedback_topic, self._on_sensor_feedback, 10
        )
        self.create_subscription(
            ShuttleState, args.shuttle_state_topic, self._on_shuttle_state, 10
        )
        self.create_subscription(
            SwitchState, args.switch_state_topic, self._on_switch_state, 10
        )
        self.create_subscription(
            StopperState, args.stopper_state_topic, self._on_stopper_state, 10
        )
        self.pose_subscriptions.append(
            self.create_subscription(
                PoseStamped,
                args.pose_topic,
                self._pose_callback(args.pickup_shuttle_name),
                10,
            )
        )
        drop_pose_topic = args.drop_pose_topic or (
            f"{args.pose_topic_prefix}/"
            f"{topic_safe_name(args.drop_shuttle_name)}/pose_cmd"
        )
        if drop_pose_topic != args.pose_topic:
            self.pose_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    drop_pose_topic,
                    self._pose_callback(args.drop_shuttle_name),
                    10,
                )
            )
        self.add_shuttle_client = self.create_client(
            AddShuttle, args.add_shuttle_service
        )

    def _on_sensor_feedback(self, message):
        self.latest_sensor_feedback = message

    def _on_shuttle_state(self, message):
        name = message.name or self.args.pickup_shuttle_name
        self.latest_shuttle_states[name] = message
        self.shuttle_state_updates[name] = (
            self.shuttle_state_updates.get(name, 0) + 1
        )

    def _on_switch_state(self, message):
        self.latest_switch_states = {
            state.name: state.state for state in message.switches
        }
        self.switch_state_updates += 1

    def _on_stopper_state(self, message):
        self.latest_stopper_states = {
            state.name: state.state for state in message.stoppers
        }
        self.stopper_state_updates += 1

    def _pose_callback(self, shuttle_name):
        def callback(message):
            self.latest_poses[shuttle_name] = message
            self.pose_updates[shuttle_name] = self.pose_updates.get(shuttle_name, 0) + 1

        return callback

    def wait_for_publishers(self, timeout):
        publishers = [
            (self.switch_publisher, self.args.switch_command_topic),
            (self.stopper_publisher, self.args.stopper_command_topic),
            (self.shuttle_publisher, self.args.shuttle_command_topic),
        ]
        if self.trajectory_publisher is not None:
            publishers.append(
                (self.trajectory_publisher, self.args.trajectory_topic)
            )
        self.wait_for_subscribers(publishers, timeout)

    def falling_state(self, shuttle_name):
        state = self.latest_shuttle_states.get(shuttle_name)
        if state is not None and state.mode == "FALLING":
            return state
        return None

    def wait_until(
        self,
        predicate,
        timeout,
        label,
        *,
        fail_on_falling=True,
        shuttle_name=None,
    ):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            falling = self.falling_state(shuttle_name) if shuttle_name else None
            if fail_on_falling and falling is not None:
                raise RuntimeError(
                    f"{shuttle_name} entered FALLING on "
                    f"{falling.current_segment}@{falling.s:.3f}"
                )
            if predicate():
                return
        raise RuntimeError(f"timed out waiting for {label}")

    def sensor_reading(self, sensor_name):
        if self.latest_sensor_feedback is None:
            return None
        for reading in self.latest_sensor_feedback.readings:
            if reading.name == sensor_name:
                return reading
        return None

    def sensor_is_active(self, sensor_name, shuttle_name):
        reading = self.sensor_reading(sensor_name)
        if reading is None or reading.active == 0:
            return False
        return reading.shuttle_name in {"", shuttle_name}

    def wait_for_sensor_known(self, sensor_name):
        self.wait_until(
            lambda: self.sensor_reading(sensor_name) is not None,
            self.args.feedback_timeout,
            f"sensor {sensor_name} feedback",
            fail_on_falling=False,
        )

    def wait_for_pose(self, shuttle_name):
        self.wait_until(
            lambda: shuttle_name in self.latest_poses,
            self.args.feedback_timeout,
            f"initial pose for {shuttle_name}",
            fail_on_falling=False,
        )

    def wait_for_sensor_active(
        self, sensor_name, shuttle_name, timeout, *, stream_payload=False
    ):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            falling = self.falling_state(shuttle_name)
            if falling is not None:
                raise RuntimeError(
                    f"{shuttle_name} entered FALLING on "
                    f"{falling.current_segment}@{falling.s:.3f}"
                )
            if stream_payload:
                self.stream_payload_on_shuttle()
            if self.sensor_is_active(sensor_name, shuttle_name):
                return
        raise RuntimeError(f"timed out waiting for {sensor_name} active")

    def wait_for_sensor_inactive(
        self, sensor_name, shuttle_name, timeout, *, stream_payload=False
    ):
        stable_since = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            falling = self.falling_state(shuttle_name)
            if falling is not None:
                raise RuntimeError(
                    f"{shuttle_name} entered FALLING on "
                    f"{falling.current_segment}@{falling.s:.3f}"
                )
            if self.sensor_reading(sensor_name) is None:
                continue
            if stream_payload:
                self.stream_payload_on_shuttle()
            if self.sensor_is_active(sensor_name, shuttle_name):
                stable_since = None
                continue
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            if now - stable_since >= self.args.sensor_inactive_dwell_s:
                return
        raise RuntimeError(f"timed out waiting for {sensor_name} inactive")

    def latest_pose(self, shuttle_name):
        stamped = self.latest_poses.get(shuttle_name)
        return stamped.pose if stamped is not None else None

    def ensure_payload_on_shuttle(self, shuttle_name):
        self.wait_for_pose(shuttle_name)
        pose = payload_pose_from_shuttle_pose(self.latest_pose(shuttle_name))
        self.initialize_payload(pose, shuttle_name)

    def stream_payload_on_shuttle(self):
        shuttle_pose = self.latest_pose(self.args.pickup_shuttle_name)
        if shuttle_pose is None:
            return
        now = time.monotonic()
        period = 1.0 / self.args.shuttle_box_rate
        if now - self.last_payload_stream_time < period:
            return
        if not self.finish_pending_payload_pose():
            return

        self.pending_payload_pose = self.set_payload_pose_async(
            payload_pose_from_shuttle_pose(shuttle_pose)
        )
        self.last_payload_stream_time = now

    def wait_for_stopped_pose(
        self,
        shuttle_name,
        *,
        after_pose_update=None,
        after_state_update=None,
        require_waiting_state=True,
    ):
        if after_pose_update is None:
            after_pose_update = self.pose_updates.get(shuttle_name, 0)
        if after_state_update is None:
            after_state_update = self.shuttle_state_updates.get(shuttle_name, 0)

        stable_since = None
        previous_pose = None
        last_pose_update = after_pose_update
        deadline = time.monotonic() + self.args.stopped_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose_update = self.pose_updates.get(shuttle_name, 0)
            state_update = self.shuttle_state_updates.get(shuttle_name, 0)
            if pose_update <= last_pose_update:
                continue
            if require_waiting_state and state_update <= after_state_update:
                continue

            last_pose_update = pose_update
            pose = self.latest_pose(shuttle_name)
            state = self.latest_shuttle_states.get(shuttle_name)
            if pose is None or (require_waiting_state and state is None):
                continue
            if state is not None and state.mode == "FALLING":
                raise RuntimeError(
                    f"{shuttle_name} entered FALLING on "
                    f"{state.current_segment}@{state.s:.3f}"
                )
            if require_waiting_state and state.mode != "WAITING":
                stable_since = None
                previous_pose = pose
                continue

            now = time.monotonic()
            if previous_pose is None:
                stable_since = now
            elif poses_close(
                previous_pose,
                pose,
                self.args.pose_stable_position_tolerance,
                self.args.pose_stable_yaw_tolerance,
            ):
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = now

            previous_pose = pose
            if (
                stable_since is not None
                and now - stable_since >= self.args.pose_stable_s
            ):
                return pose

        raise RuntimeError(
            f"timed out waiting for stable stopped pose for {shuttle_name}"
        )

    def publish_switch_all_exterior(self):
        message = SwitchCommand()
        message.switches = [NamedState(name="ALL", state="EXTERIOR")]
        self.switch_publisher.publish(message)
        self.spin_sleep(0.1)
        print("right rail switches commanded to ALL=EXTERIOR", flush=True)

    def publish_stoppers_open(self):
        message = StopperCommand()
        message.stoppers = [NamedState(name="ALL", state="0")]
        self.stopper_publisher.publish(message)
        self.spin_sleep(0.1)
        print("right rail stoppers commanded open", flush=True)

    def publish_shuttle_command(self, shuttle_name, command):
        message = ShuttleCommand()
        message.name = shuttle_name
        message.command = command
        if command == "ON":
            self.active_shuttles.add(shuttle_name)
        self.shuttle_publisher.publish(message)
        self.spin_sleep(0.1)
        if command == "OFF":
            self.active_shuttles.discard(shuttle_name)
        print(f"shuttle {shuttle_name} command: {command}", flush=True)

    def stop_active_shuttles(self):
        for shuttle_name in list(self.active_shuttles):
            try:
                self.publish_shuttle_command(shuttle_name, "OFF")
            except Exception as exc:
                self.get_logger().error(
                    f"failed to stop shuttle {shuttle_name}: {exc}"
                )

    def route_is_ready(self):
        return (
            bool(self.latest_switch_states)
            and all(state == "E" for state in self.latest_switch_states.values())
            and bool(self.latest_stopper_states)
            and all(state == "0" for state in self.latest_stopper_states.values())
        )

    def prepare_route(self):
        switch_update = self.switch_state_updates
        stopper_update = self.stopper_state_updates
        self.publish_switch_all_exterior()
        self.publish_stoppers_open()
        self.wait_until(
            lambda: (
                self.switch_state_updates > switch_update
                and self.stopper_state_updates > stopper_update
                and self.route_is_ready()
            ),
            self.args.route_timeout,
            "right rail switches and stoppers to acknowledge the route",
            fail_on_falling=False,
        )
        print("right rail route ready", flush=True)

    def add_drop_shuttle(self):
        request = AddShuttle.Request()
        request.name = self.args.drop_shuttle_name
        request.start_slot = str(self.args.drop_start_slot)
        request.speed = 0.0
        request.start_enabled = False
        response = self.call_service(
            self.add_shuttle_client,
            request,
            f"add shuttle {self.args.drop_shuttle_name}",
            timeout=self.args.add_shuttle_timeout,
        )
        print(response.message, flush=True)
        self.wait_for_pose(self.args.drop_shuttle_name)
        self.wait_for_sensor_known(self.args.drop_sensor)
        self.wait_for_sensor_active(
            self.args.drop_sensor,
            self.args.drop_shuttle_name,
            self.args.feedback_timeout,
        )
        pose = self.wait_for_stopped_pose(
            self.args.drop_shuttle_name,
            require_waiting_state=False,
        )
        print(
            f"drop: stopped shuttle pose {format_pose(pose)} on "
            f"{self.args.drop_sensor}",
            flush=True,
        )
        return pose

    def move_to_pickup_slot(
        self,
        label,
        shuttle_name,
        sensor_name,
        *,
        require_leave_first,
        timeout,
        stream_payload=False,
    ):
        print(f"{label}: moving {shuttle_name} toward {sensor_name}", flush=True)
        self.publish_shuttle_command(shuttle_name, "ON")
        try:
            if require_leave_first or self.sensor_is_active(sensor_name, shuttle_name):
                self.wait_for_sensor_inactive(
                    sensor_name,
                    shuttle_name,
                    timeout,
                    stream_payload=stream_payload,
                )
                print(f"{label}: {shuttle_name} left {sensor_name}", flush=True)
            self.wait_for_sensor_active(
                sensor_name,
                shuttle_name,
                timeout,
                stream_payload=stream_payload,
            )
            print(
                f"{label}: {sensor_name} active, stopping {shuttle_name}",
                flush=True,
            )
            pose_update = self.pose_updates.get(shuttle_name, 0)
            state_update = self.shuttle_state_updates.get(shuttle_name, 0)
            self.publish_shuttle_command(shuttle_name, "OFF")
            pose = self.wait_for_stopped_pose(
                shuttle_name,
                after_pose_update=pose_update,
                after_state_update=state_update,
            )
            if stream_payload:
                if not self.finish_pending_payload_pose(timeout=1.0):
                    raise RuntimeError("set payload pose service call timed out")
                self.set_payload_pose(
                    payload_pose_from_shuttle_pose(pose),
                    timeout=1.0,
                )
            print(f"{label}: stopped shuttle pose {format_pose(pose)}", flush=True)
            return pose
        finally:
            if shuttle_name in self.active_shuttles:
                try:
                    self.publish_shuttle_command(shuttle_name, "OFF")
                except Exception as exc:
                    self.get_logger().error(
                        f"failed to stop shuttle {shuttle_name}: {exc}"
                    )


def parse_args(argv):
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    add_manipulation_arguments(parser)
    parser.add_argument(
        "--pickup-shuttle-name",
        default=DEFAULT_PICKUP_SHUTTLE_NAME,
    )
    parser.add_argument(
        "--drop-shuttle-name",
        default=DEFAULT_DROP_SHUTTLE_NAME,
    )
    parser.add_argument("--drop-start-slot", default="4")
    parser.add_argument("--pickup-sensor", default=DEFAULT_PICKUP_SENSOR)
    parser.add_argument("--drop-sensor", default=DEFAULT_DROP_SENSOR)
    parser.set_defaults(
        switch_command_topic=f"{RIGHT_RAIL_PREFIX}/switches/command",
        stopper_command_topic=f"{RIGHT_RAIL_PREFIX}/stoppers/command",
        shuttle_command_topic=f"{RIGHT_RAIL_PREFIX}/shuttles/command",
        sensor_feedback_topic=f"{RIGHT_RAIL_PREFIX}/sensors/feedback",
        shuttle_state_topic=f"{RIGHT_RAIL_PREFIX}/shuttles/state",
        switch_state_topic=f"{RIGHT_RAIL_PREFIX}/switches/state",
        stopper_state_topic=f"{RIGHT_RAIL_PREFIX}/stoppers/state",
        pose_topic=f"{RIGHT_RAIL_PREFIX}/shuttles/pose_cmd",
        pose_topic_prefix=f"{RIGHT_RAIL_PREFIX}/shuttles",
        drop_pose_topic=None,
        add_shuttle_service=f"{RIGHT_RAIL_PREFIX}/shuttles/add",
        feedback_timeout=10.0,
        add_shuttle_timeout=10.0,
        arrival_timeout=120.0,
        stopped_timeout=15.0,
        route_timeout=5.0,
        shuttle_box_rate=15.0,
        sensor_inactive_dwell_s=0.25,
        pose_stable_s=0.3,
        pose_stable_position_tolerance=0.002,
        pose_stable_yaw_tolerance=0.01,
    )
    return finalize_manipulation_args(parser.parse_args(argv), script_dir)


def run_moving_shuttle_demo(node, args):
    node.wait_for_publishers(args.publisher_timeout)
    node.wait_for_sensor_known(args.pickup_sensor)
    drop_pose = node.add_drop_shuttle()

    node.ensure_payload_on_shuttle(args.pickup_shuttle_name)
    node.prepare_route()

    preposition = node.start_preposition_arm() if args.preposition else None
    pickup_pose = node.move_to_pickup_slot(
        "arrival",
        args.pickup_shuttle_name,
        args.pickup_sensor,
        require_leave_first=False,
        timeout=args.arrival_timeout,
        stream_payload=True,
    )
    if preposition is not None:
        node.wait_preposition_arm(preposition)

    run_hpp_cycle(
        args,
        pickup_pose,
        direction="shuttle-to-shuttle",
        destination_shuttle_pose=drop_pose,
    )


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    args.hpp_script = args.hpp_script.resolve()
    if not args.hpp_script.exists():
        raise RuntimeError(f"HPP wrapper does not exist: {args.hpp_script}")
    rclpy.init()
    node = MovingShuttleCoordinator(args)
    try:
        run_moving_shuttle_demo(node, args)
    finally:
        node.stop_active_shuttles()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("two-shuttle manipulation demo complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
