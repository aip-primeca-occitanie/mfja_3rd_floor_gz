#!/usr/bin/env python3
"""Coordinate the moving-shuttle Room 315 Staubli manipulation demo."""

import argparse
import math
import re
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from mfja_rail_interfaces.msg import NamedState
from mfja_rail_interfaces.msg import SensorFeedback
from mfja_rail_interfaces.msg import ShuttleCommand
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import StopperCommand
from mfja_rail_interfaces.msg import SwitchCommand
from mfja_rail_interfaces.srv import AddShuttle
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.srv import SpawnEntity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


RIGHT_RAIL_PREFIX = "/room_315/rails/right"
DEFAULT_PICKUP_SHUTTLE_NAME = "room315_right_shuttle_1"
DEFAULT_DROP_SHUTTLE_NAME = "room315_right_shuttle_2"
DEFAULT_PICKUP_SENSOR = "DZI3R"
DEFAULT_DROP_SENSOR = "DZI4R"
DEFAULT_NOMINAL_PICKUP_POSE = (
    -15.310,
    -5.536,
    0.839346,
    0.0,
    0.0,
    -0.002,
)
STAUBLI_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
DEFAULT_HPP_START_JOINTS = (
    -1.56136443,
    0.47307870,
    2.04964315,
    -0.00130315,
    -0.32991444,
    0.00524110,
)
BOX_SIZE = (0.07, 0.05, 0.06)
SHUTTLE_CONTACT_Z = 0.085
PAYLOAD_ON_SHUTTLE_Z = SHUTTLE_CONTACT_Z + 0.5 * BOX_SIZE[2]

PAYLOAD_BOX_SDF = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="room315_payload_box">
    <static>true</static>
    <link name="base_link">
      <visual name="payload_visual">
        <geometry>
          <box>
            <size>{BOX_SIZE[0]} {BOX_SIZE[1]} {BOX_SIZE[2]}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.35 0.95 1</ambient>
          <diffuse>0.05 0.35 0.95 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def quaternion_to_rpy(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_distance(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def pose_values(pose):
    roll, pitch, yaw = quaternion_to_rpy(pose.orientation)
    return (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        roll,
        pitch,
        yaw,
    )


def pose_from_values(values):
    x, y, z, roll, pitch, yaw = values
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    return pose


def poses_close(first, second, position_tolerance, yaw_tolerance):
    first_values = pose_values(first)
    second_values = pose_values(second)
    distance = math.dist(first_values[:3], second_values[:3])
    return (
        distance <= position_tolerance
        and yaw_distance(first_values[5], second_values[5]) <= yaw_tolerance
    )


def pose_argument_values(pose):
    return [f"{value:.9f}" for value in pose_values(pose)]


def format_pose(pose):
    x, y, z, roll, pitch, yaw = pose_values(pose)
    return (
        f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
        f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
    )


def payload_pose_from_shuttle_pose(shuttle_pose):
    pose = Pose()
    pose.position.x = shuttle_pose.position.x
    pose.position.y = shuttle_pose.position.y
    pose.position.z = shuttle_pose.position.z + PAYLOAD_ON_SHUTTLE_Z
    pose.orientation = shuttle_pose.orientation
    return pose


def topic_safe_name(name):
    return re.sub(r"[^A-Za-z0-9_]+", "_", name)


def duration_msg(seconds):
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - msg.sec) * 1e9)
    return msg


@dataclass
class HppCycleProcess:
    command: list[str]
    process: subprocess.Popen
    ready_file: Path
    start_file: Path
    abort_file: Path
    tempdir: tempfile.TemporaryDirectory

    def poll(self):
        return self.process.poll()

    def ready(self):
        return self.ready_file.exists()

    def start(self):
        self.start_file.write_text(f"{time.monotonic():.3f}\n")
        return self.process.wait()

    def abort(self, timeout):
        self.abort_file.write_text(f"{time.monotonic():.3f}\n")
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                return self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                return self.process.wait()

    def cleanup(self):
        self.tempdir.cleanup()


@dataclass
class ArmPreposition:
    target: list[float]
    timeout: float
    active: bool


class MovingShuttleCoordinator(Node):
    def __init__(self, args):
        super().__init__("room315_moving_shuttle_sequence")
        self.args = args
        self.latest_sensor_feedback = None
        self.latest_shuttle_states = {}
        self.latest_poses = {}
        self.latest_joint_positions = None
        self.last_payload_stream_time = 0.0
        self.pending_payload_pose = None
        self.pose_subscriptions = []

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, args.trajectory_topic, 10
        )
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
            JointState, args.joint_state_topic, self._on_joint_state, 10
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
            f"{args.pose_topic_prefix}/{topic_safe_name(args.drop_shuttle_name)}/pose_cmd"
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
        service_prefix = f"/world/{args.world_name}"
        self.add_shuttle_client = self.create_client(
            AddShuttle, args.add_shuttle_service
        )
        self.spawn_client = self.create_client(SpawnEntity, f"{service_prefix}/create")
        self.delete_client = self.create_client(DeleteEntity, f"{service_prefix}/remove")
        self.pose_client = self.create_client(SetEntityPose, f"{service_prefix}/set_pose")

    def _on_sensor_feedback(self, message):
        self.latest_sensor_feedback = message

    def _on_shuttle_state(self, message):
        if message.name:
            self.latest_shuttle_states[message.name] = message
        else:
            self.latest_shuttle_states[self.args.pickup_shuttle_name] = message

    def _on_joint_state(self, message):
        positions_by_name = dict(zip(message.name, message.position))
        if all(name in positions_by_name for name in STAUBLI_JOINT_NAMES):
            self.latest_joint_positions = [
                positions_by_name[name] for name in STAUBLI_JOINT_NAMES
            ]

    def _pose_callback(self, shuttle_name):
        def callback(message):
            self.latest_poses[shuttle_name] = message

        return callback

    def spin_sleep(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )

    def wait_for_publishers(self, timeout):
        publishers = [
            (self.trajectory_publisher, self.args.trajectory_topic),
            (self.switch_publisher, self.args.switch_command_topic),
            (self.stopper_publisher, self.args.stopper_command_topic),
            (self.shuttle_publisher, self.args.shuttle_command_topic),
        ]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(publisher.get_subscription_count() > 0 for publisher, _ in publishers):
                return

        missing = [
            topic
            for publisher, topic in publishers
            if publisher.get_subscription_count() == 0
        ]
        if missing:
            self.get_logger().warning(
                "no subscriber discovered yet on: " + ", ".join(missing)
            )

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

    def call_service(self, client, request, label, timeout=3.0, require_success=True):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{label} service is unavailable")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"{label} service call timed out")

        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} service returned no result")
        if require_success and hasattr(result, "success") and not result.success:
            raise RuntimeError(f"{label} service failed: {result}")
        return result

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

    def wait_for_joint_state(self):
        self.wait_until(
            lambda: self.latest_joint_positions is not None,
            self.args.preposition_joint_state_timeout,
            f"joint state on {self.args.joint_state_topic}",
            fail_on_falling=False,
        )
        return list(self.latest_joint_positions)

    def wait_for_arm_configuration(self, target, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            current = self.latest_joint_positions
            if current is None:
                continue
            error = max(abs(a - b) for a, b in zip(current, target))
            if error <= self.args.preposition_tolerance:
                return error
        current = self.latest_joint_positions
        if current is None:
            return float("inf")
        return max(abs(a - b) for a, b in zip(current, target))

    def start_preposition_arm(self):
        target = list(self.args.preposition_q_start)
        current = self.wait_for_joint_state()
        error = max(abs(a - b) for a, b in zip(current, target))
        if error <= self.args.preposition_tolerance:
            print(
                f"Staubli already at HPP start before shuttle motion "
                f"(error {error:.3f} rad)",
                flush=True,
            )
            return ArmPreposition(target, 0.0, False)

        duration = max(
            self.args.min_preposition_duration,
            error / self.args.preposition_joint_speed,
        )
        sample_count = max(
            2, int(duration * self.args.preposition_samples_per_second) + 1
        )
        trajectory = JointTrajectory()
        trajectory.joint_names = list(STAUBLI_JOINT_NAMES)

        hold = JointTrajectoryPoint()
        hold.positions = list(current)
        hold.time_from_start = duration_msg(0.0)
        trajectory.points.append(hold)

        held = JointTrajectoryPoint()
        held.positions = list(current)
        held.time_from_start = duration_msg(self.args.preposition_initial_hold)
        trajectory.points.append(held)

        for index in range(1, sample_count):
            alpha = index / (sample_count - 1)
            point = JointTrajectoryPoint()
            point.positions = [
                (1.0 - alpha) * start + alpha * goal
                for start, goal in zip(current, target)
            ]
            point.time_from_start = duration_msg(
                self.args.preposition_initial_hold + alpha * duration
            )
            trajectory.points.append(point)

        print(
            f"prepositioning Staubli to HPP start over {duration:.1f} s",
            flush=True,
        )
        self.trajectory_publisher.publish(trajectory)
        rclpy.spin_once(self, timeout_sec=0.05)

        timeout = (
            self.args.preposition_initial_hold
            + duration * self.args.preposition_timeout_scale
            + 5.0
        )
        return ArmPreposition(target, timeout, True)

    def wait_preposition_arm(self, preposition):
        if not preposition.active:
            return
        final_error = self.wait_for_arm_configuration(
            preposition.target,
            preposition.timeout,
        )
        if final_error > self.args.preposition_tolerance:
            raise RuntimeError(
                f"Staubli did not reach HPP start after preposition "
                f"(error {final_error:.3f} rad)"
            )
        print(
            f"Staubli prepositioned at HPP start (error {final_error:.3f} rad)",
            flush=True,
        )

    def preposition_arm(self):
        self.wait_preposition_arm(self.start_preposition_arm())

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

    def delete_payload(self):
        request = DeleteEntity.Request()
        request.entity.name = self.args.box_entity_name
        request.entity.type = Entity.MODEL
        try:
            self.call_service(
                self.delete_client,
                request,
                f"delete {self.args.box_entity_name}",
                timeout=2.0,
                require_success=False,
            )
        except RuntimeError as exc:
            self.get_logger().warning(str(exc))

    def spawn_payload(self, pose):
        request = SpawnEntity.Request()
        request.entity_factory.name = self.args.box_entity_name
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf = PAYLOAD_BOX_SDF.replace(
            'model name="room315_payload_box"',
            f'model name="{self.args.box_entity_name}"',
        )
        request.entity_factory.pose = pose
        request.entity_factory.relative_to = "world"
        try:
            self.call_service(
                self.spawn_client,
                request,
                f"spawn {self.args.box_entity_name}",
                timeout=5.0,
            )
            return True
        except RuntimeError as exc:
            self.get_logger().warning(str(exc))
            return False

    def set_payload_pose(self, pose, timeout=1.0):
        request = SetEntityPose.Request()
        request.entity.name = self.args.box_entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose
        self.call_service(
            self.pose_client,
            request,
            f"set pose for {self.args.box_entity_name}",
            timeout=timeout,
        )

    def set_payload_pose_async(self, pose):
        request = SetEntityPose.Request()
        request.entity.name = self.args.box_entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose
        return self.pose_client.call_async(request)

    def latest_pose(self, shuttle_name):
        stamped = self.latest_poses.get(shuttle_name)
        return stamped.pose if stamped is not None else None

    def ensure_payload_on_shuttle(self, shuttle_name):
        self.wait_for_pose(shuttle_name)
        pose = payload_pose_from_shuttle_pose(self.latest_pose(shuttle_name))
        if self.args.replace_box:
            self.delete_payload()
        spawned = self.spawn_payload(pose)
        if not spawned:
            self.get_logger().info(
                f"using existing Gazebo entity {self.args.box_entity_name}"
            )
        self.set_payload_pose(pose)
        print(
            f"payload initialized on {shuttle_name}: {format_pose(pose)}",
            flush=True,
        )

    def stream_payload_on_shuttle(self):
        shuttle_pose = self.latest_pose(self.args.pickup_shuttle_name)
        if shuttle_pose is None:
            return
        now = time.monotonic()
        period = 1.0 / self.args.shuttle_box_rate
        if now - self.last_payload_stream_time < period:
            return
        if self.pending_payload_pose is not None:
            if self.pending_payload_pose.done():
                self.pending_payload_pose = None
            else:
                return

        self.pending_payload_pose = self.set_payload_pose_async(
            payload_pose_from_shuttle_pose(shuttle_pose)
        )
        self.last_payload_stream_time = now

    def wait_for_stopped_pose(self, shuttle_name):
        stable_since = None
        previous_pose = None
        deadline = time.monotonic() + self.args.stopped_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.latest_pose(shuttle_name)
            if pose is None:
                continue
            state = self.latest_shuttle_states.get(shuttle_name)
            if state is not None and state.mode == "FALLING":
                raise RuntimeError(
                    f"{shuttle_name} entered FALLING on "
                    f"{state.current_segment}@{state.s:.3f}"
                )
            if state is not None and state.mode != "WAITING":
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
            if stable_since is not None and now - stable_since >= self.args.pose_stable_s:
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
        self.shuttle_publisher.publish(message)
        self.spin_sleep(0.1)
        print(f"shuttle {shuttle_name} command: {command}", flush=True)

    def prepare_route(self):
        self.publish_switch_all_exterior()
        self.publish_stoppers_open()
        self.spin_sleep(self.args.route_settle_s)

    def add_drop_shuttle(self):
        request = AddShuttle.Request()
        request.name = self.args.drop_shuttle_name
        request.start_slot = str(self.args.drop_start_slot)
        request.speed = self.args.drop_shuttle_speed
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
        pose = self.wait_for_stopped_pose(self.args.drop_shuttle_name)
        print(
            f"drop: stopped shuttle pose {format_pose(pose)} on {self.args.drop_sensor}",
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
        print(f"{label}: {sensor_name} active, stopping {shuttle_name}", flush=True)
        self.publish_shuttle_command(shuttle_name, "OFF")
        pose = self.wait_for_stopped_pose(shuttle_name)
        if stream_payload:
            self.set_payload_pose(payload_pose_from_shuttle_pose(pose), timeout=1.0)
        print(f"{label}: stopped shuttle pose {format_pose(pose)}", flush=True)
        return pose


def hpp_cycle_command(
    args,
    hpp_args,
    direction,
    shuttle_pose,
    replace_box,
    *,
    destination_shuttle_pose=None,
):
    command = [
        str(args.hpp_script),
        *hpp_args,
        "--robot-name",
        args.robot_name,
        "--world-name",
        args.world_name,
        "--box-entity-name",
        args.box_entity_name,
        "--execute",
        "--direction",
        direction,
        "--shuttle-pose",
        *pose_argument_values(shuttle_pose),
    ]
    if destination_shuttle_pose is not None:
        command.extend(
            [
                "--destination-shuttle-pose",
                *pose_argument_values(destination_shuttle_pose),
            ]
        )
    command.extend(["--gripper-output", args.gripper_output])
    if args.gripper_command_topic:
        command.extend(["--gripper-command-topic", args.gripper_command_topic])
    if args.gripper_trajectory_topic:
        command.extend(["--gripper-trajectory-topic", args.gripper_trajectory_topic])
    if replace_box:
        command.append("--replace-box")
    return command


def run_hpp_cycle(
    args,
    hpp_args,
    direction,
    shuttle_pose,
    replace_box,
    *,
    destination_shuttle_pose=None,
):
    command = hpp_cycle_command(
        args,
        hpp_args,
        direction,
        shuttle_pose,
        replace_box,
        destination_shuttle_pose=destination_shuttle_pose,
    )
    print("running HPP cycle: " + " ".join(command), flush=True)
    subprocess.run(command, cwd=args.hpp_script.parent.parent, check=True)


def start_hpp_cycle_preplan(
    args,
    hpp_args,
    direction,
    shuttle_pose,
    *,
    destination_shuttle_pose=None,
):
    tempdir = tempfile.TemporaryDirectory(
        prefix="room315_hpp_preplan_",
        dir=args.preplan_dir,
    )
    tempdir_path = Path(tempdir.name)
    ready_file = tempdir_path / "ready"
    start_file = tempdir_path / "start"
    abort_file = tempdir_path / "abort"
    command = hpp_cycle_command(
        args,
        hpp_args,
        direction,
        shuttle_pose,
        False,
        destination_shuttle_pose=destination_shuttle_pose,
    )
    command.extend(
        [
            "--ready-file",
            str(ready_file),
            "--start-file",
            str(start_file),
            "--abort-file",
            str(abort_file),
        ]
    )
    print("preplanning HPP cycle: " + " ".join(command), flush=True)
    process = subprocess.Popen(command, cwd=args.hpp_script.parent.parent)
    return HppCycleProcess(command, process, ready_file, start_file, abort_file, tempdir)


def use_preplanned_or_run(
    args,
    hpp_args,
    preplan,
    direction,
    actual_shuttle_pose,
    nominal_shuttle_pose,
    *,
    actual_destination_shuttle_pose=None,
    nominal_destination_shuttle_pose=None,
):
    if preplan is not None:
        reuse = poses_close(
            actual_shuttle_pose,
            nominal_shuttle_pose,
            args.preplan_position_tolerance,
            args.preplan_yaw_tolerance,
        )
        if reuse and actual_destination_shuttle_pose is not None:
            reuse = poses_close(
                actual_destination_shuttle_pose,
                nominal_destination_shuttle_pose,
                args.preplan_position_tolerance,
                args.preplan_yaw_tolerance,
            )

        if reuse and preplan.poll() is None:
            print("using preplanned HPP cycle", flush=True)
            result = preplan.start()
            preplan.cleanup()
            if result == 0:
                return
            raise RuntimeError(
                f"preplanned HPP execution exited with {result}; stopping sequence"
            )
        else:
            reason = "pose mismatch" if not reuse else "preplan process exited early"
            print(f"discarding preplanned HPP cycle: {reason}", flush=True)
            preplan.abort(args.preplan_abort_timeout)
            preplan.cleanup()

    run_hpp_cycle(
        args,
        hpp_args,
        direction,
        actual_shuttle_pose,
        replace_box=False,
        destination_shuttle_pose=actual_destination_shuttle_pose,
    )


def parse_args(argv):
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hpp-script",
        type=Path,
        default=script_dir / "room315_hpp_manipulation.sh",
    )
    parser.add_argument("--robot-name", default="staubli1")
    parser.add_argument("--world-name", default="room_315_only")
    parser.add_argument("--box-entity-name", default="room315_payload_box")
    parser.add_argument(
        "--gripper-output",
        choices=["none", "bool", "joint-trajectory"],
        default="joint-trajectory",
    )
    parser.add_argument("--gripper-command-topic", default=None)
    parser.add_argument("--gripper-trajectory-topic", default=None)
    parser.add_argument("--trajectory-topic", default=None)
    parser.add_argument("--joint-state-topic", default=None)
    parser.add_argument(
        "--scenario",
        choices=["two-shuttle", "round-trip"],
        default="two-shuttle",
    )
    parser.add_argument(
        "--shuttle-name",
        "--pickup-shuttle-name",
        dest="pickup_shuttle_name",
        default=DEFAULT_PICKUP_SHUTTLE_NAME,
    )
    parser.add_argument("--drop-shuttle-name", default=DEFAULT_DROP_SHUTTLE_NAME)
    parser.add_argument("--drop-start-slot", default="4")
    parser.add_argument("--drop-shuttle-speed", type=float, default=0.0)
    parser.add_argument("--pickup-sensor", default=DEFAULT_PICKUP_SENSOR)
    parser.add_argument("--drop-sensor", default=DEFAULT_DROP_SENSOR)
    parser.add_argument(
        "--switch-command-topic",
        default=f"{RIGHT_RAIL_PREFIX}/switches/command",
    )
    parser.add_argument(
        "--stopper-command-topic",
        default=f"{RIGHT_RAIL_PREFIX}/stoppers/command",
    )
    parser.add_argument(
        "--shuttle-command-topic",
        default=f"{RIGHT_RAIL_PREFIX}/shuttles/command",
    )
    parser.add_argument(
        "--sensor-feedback-topic",
        default=f"{RIGHT_RAIL_PREFIX}/sensors/feedback",
    )
    parser.add_argument(
        "--shuttle-state-topic",
        default=f"{RIGHT_RAIL_PREFIX}/shuttles/state",
    )
    parser.add_argument(
        "--pose-topic",
        default=f"{RIGHT_RAIL_PREFIX}/shuttles/pose_cmd",
    )
    parser.add_argument(
        "--pose-topic-prefix",
        default=f"{RIGHT_RAIL_PREFIX}/shuttles",
    )
    parser.add_argument("--drop-pose-topic", default=None)
    parser.add_argument(
        "--add-shuttle-service",
        default=f"{RIGHT_RAIL_PREFIX}/shuttles/add",
    )
    parser.add_argument("--replace-box", action="store_true")
    parser.add_argument("--no-early-plan", dest="early_plan", action="store_false")
    parser.set_defaults(early_plan=True)
    parser.add_argument("--skip-preposition", action="store_true")
    parser.add_argument(
        "--preposition-q-start",
        nargs=6,
        type=float,
        default=DEFAULT_HPP_START_JOINTS,
        metavar=tuple(STAUBLI_JOINT_NAMES),
    )
    parser.add_argument("--preposition-joint-speed", type=float, default=0.18)
    parser.add_argument("--min-preposition-duration", type=float, default=6.0)
    parser.add_argument("--preposition-samples-per-second", type=int, default=20)
    parser.add_argument("--preposition-initial-hold", type=float, default=0.3)
    parser.add_argument("--preposition-tolerance", type=float, default=0.07)
    parser.add_argument("--hpp-start-tolerance", type=float, default=0.08)
    parser.add_argument("--preposition-timeout-scale", type=float, default=3.0)
    parser.add_argument("--preposition-joint-state-timeout", type=float, default=10.0)
    parser.add_argument(
        "--nominal-pickup-pose",
        nargs=6,
        type=float,
        default=DEFAULT_NOMINAL_PICKUP_POSE,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
    )
    parser.add_argument("--preplan-position-tolerance", type=float, default=0.06)
    parser.add_argument("--preplan-yaw-tolerance", type=float, default=0.05)
    parser.add_argument("--preplan-abort-timeout", type=float, default=2.0)
    parser.add_argument("--preplan-dir", type=Path, default=Path("/dev/shm"))
    parser.add_argument("--feedback-timeout", type=float, default=10.0)
    parser.add_argument("--publisher-timeout", type=float, default=5.0)
    parser.add_argument("--add-shuttle-timeout", type=float, default=10.0)
    parser.add_argument("--arrival-timeout", type=float, default=120.0)
    parser.add_argument("--return-timeout", type=float, default=120.0)
    parser.add_argument("--stopped-timeout", type=float, default=15.0)
    parser.add_argument("--route-settle-s", type=float, default=0.4)
    parser.add_argument("--shuttle-box-rate", type=float, default=15.0)
    parser.add_argument("--sensor-inactive-dwell-s", type=float, default=0.25)
    parser.add_argument("--pose-stable-s", type=float, default=0.3)
    parser.add_argument("--pose-stable-position-tolerance", type=float, default=0.002)
    parser.add_argument("--pose-stable-yaw-tolerance", type=float, default=0.01)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, hpp_args = parse_args(sys.argv[1:] if argv is None else argv)
    if "--start-tolerance" not in hpp_args:
        hpp_args = ["--start-tolerance", f"{args.hpp_start_tolerance:.3f}", *hpp_args]
    args.hpp_script = args.hpp_script.resolve()
    if not args.hpp_script.exists():
        raise RuntimeError(f"HPP wrapper does not exist: {args.hpp_script}")
    if args.trajectory_topic is None:
        args.trajectory_topic = f"/{args.robot_name}/joint_trajectory"
    if args.joint_state_topic is None:
        args.joint_state_topic = f"/{args.robot_name}/joint_states"
    nominal_pickup_pose = pose_from_values(args.nominal_pickup_pose)
    active_preplans = []

    rclpy.init()
    node = MovingShuttleCoordinator(args)
    try:
        node.wait_for_publishers(args.publisher_timeout)
        node.wait_for_sensor_known(args.pickup_sensor)
        drop_pose = None
        if args.scenario == "two-shuttle":
            drop_pose = node.add_drop_shuttle()

        node.ensure_payload_on_shuttle(args.pickup_shuttle_name)

        first_preplan = None
        if args.early_plan:
            if args.scenario == "two-shuttle":
                first_preplan = start_hpp_cycle_preplan(
                    args,
                    hpp_args,
                    "shuttle-to-shuttle",
                    nominal_pickup_pose,
                    destination_shuttle_pose=drop_pose,
                )
                active_preplans.append(first_preplan)
            else:
                first_preplan = start_hpp_cycle_preplan(
                    args,
                    hpp_args,
                    "shuttle-to-table",
                    nominal_pickup_pose,
                )
                active_preplans.append(first_preplan)

        node.prepare_route()

        preposition = None
        if not args.skip_preposition:
            preposition = node.start_preposition_arm()

        first_pose = node.move_to_pickup_slot(
            "arrival",
            args.pickup_shuttle_name,
            args.pickup_sensor,
            require_leave_first=False,
            timeout=args.arrival_timeout,
            stream_payload=True,
        )
        if preposition is not None:
            node.wait_preposition_arm(preposition)

        if args.scenario == "two-shuttle":
            use_preplanned_or_run(
                args,
                hpp_args,
                first_preplan,
                "shuttle-to-shuttle",
                first_pose,
                nominal_pickup_pose,
                actual_destination_shuttle_pose=drop_pose,
                nominal_destination_shuttle_pose=drop_pose,
            )
            if first_preplan is not None:
                active_preplans.remove(first_preplan)
        else:
            use_preplanned_or_run(
                args,
                hpp_args,
                first_preplan,
                "shuttle-to-table",
                first_pose,
                nominal_pickup_pose,
            )
            if first_preplan is not None:
                active_preplans.remove(first_preplan)

        if args.scenario == "two-shuttle":
            print("two-shuttle manipulation demo complete", flush=True)
            return 0

        second_preplan = None
        if args.early_plan:
            second_preplan = start_hpp_cycle_preplan(
                args,
                hpp_args,
                "table-to-shuttle",
                nominal_pickup_pose,
            )
            active_preplans.append(second_preplan)
        second_pose = node.move_to_pickup_slot(
            "return",
            args.pickup_shuttle_name,
            args.pickup_sensor,
            require_leave_first=True,
            timeout=args.return_timeout,
        )
        use_preplanned_or_run(
            args,
            hpp_args,
            second_preplan,
            "table-to-shuttle",
            second_pose,
            nominal_pickup_pose,
        )
        if second_preplan is not None:
            active_preplans.remove(second_preplan)
    finally:
        for preplan in active_preplans:
            if preplan is not None:
                if preplan.poll() is None:
                    preplan.abort(args.preplan_abort_timeout)
                preplan.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("moving shuttle manipulation demo complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
