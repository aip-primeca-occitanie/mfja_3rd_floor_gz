#!/usr/bin/env python3
"""Run one simulated Room 315 manipulation cycle with fixed support poses."""

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.srv import SpawnEntity


DEFAULT_SHUTTLE_POSE = (-15.240, -5.536, 0.839, 0.0, 0.0, 0.0)
BOX_SIZE = (0.07, 0.05, 0.06)
SHUTTLE_CONTACT_Z = 0.085
PAYLOAD_ON_SHUTTLE_Z = SHUTTLE_CONTACT_Z + 0.5 * BOX_SIZE[2]

PACKAGE_DIR = Path(
    os.environ.get(
        "ROOM315_PACKAGE_DIR",
        Path(__file__).resolve().parents[1],
    )
)
PAYLOAD_BOX_SDF = (PACKAGE_DIR / "models" / "room315_payload_box.sdf").read_text()


def pose_from_values(values):
    x, y, z, roll, pitch, yaw = values
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    return pose


def quaternion_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = (
        math.copysign(math.pi / 2.0, sinp)
        if abs(sinp) >= 1.0
        else math.asin(sinp)
    )

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
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


class ManipulationCoordinator(Node):
    def __init__(self, args, node_name="room315_manipulation_sequence"):
        super().__init__(node_name)
        self.args = args
        self.pending_payload_pose = None

        service_prefix = f"/world/{args.world_name}"
        self.spawn_client = self.create_client(
            SpawnEntity, f"{service_prefix}/create"
        )
        self.delete_client = self.create_client(
            DeleteEntity, f"{service_prefix}/remove"
        )
        self.pose_client = self.create_client(
            SetEntityPose, f"{service_prefix}/set_pose"
        )

    def spin_sleep(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )

    def wait_for_subscribers(self, publishers, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(
                publisher.get_subscription_count() > 0
                for publisher, _ in publishers
            ):
                return

        missing = [
            topic
            for publisher, topic in publishers
            if publisher.get_subscription_count() == 0
        ]
        if missing:
            raise RuntimeError(
                "no subscriber discovered on: "
                + ", ".join(missing)
                + ". Start room315_demo.sh first and wait for Gazebo to finish "
                "loading."
            )

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
        if require_success and not result.success:
            raise RuntimeError(f"{label} service failed: {result}")
        return result

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

    def finish_pending_payload_pose(self, timeout=None):
        if self.pending_payload_pose is None:
            return True
        if timeout is not None:
            rclpy.spin_until_future_complete(
                self,
                self.pending_payload_pose,
                timeout_sec=timeout,
            )
        if not self.pending_payload_pose.done():
            return False

        try:
            result = self.pending_payload_pose.result()
        except Exception as exc:
            raise RuntimeError(f"set payload pose service failed: {exc}") from exc
        self.pending_payload_pose = None
        if result is None:
            raise RuntimeError("set payload pose service returned no result")
        if not result.success:
            raise RuntimeError(f"set payload pose service failed: {result}")
        return True

    def initialize_payload(self, pose, support):
        if self.args.replace_box:
            self.delete_payload()
        if not self.spawn_payload(pose):
            self.get_logger().info(
                f"using existing Gazebo entity {self.args.box_entity_name}"
            )
        self.set_payload_pose(pose)
        print(f"payload initialized on {support}: {format_pose(pose)}", flush=True)


def hpp_cycle_command(
    args,
    shuttle_pose,
    *,
    direction,
    destination_shuttle_pose=None,
):
    command = [
        str(args.hpp_script),
        "--robot-name",
        args.robot_name,
        "--world-name",
        args.world_name,
        "--box-entity-name",
        args.box_entity_name,
        "--payload-output",
        "gazebo",
        "--execute",
        "--direction",
        direction,
        "--shuttle-pose",
        *pose_argument_values(shuttle_pose),
        "--start-tolerance",
        f"{args.start_tolerance:.3f}",
        "--gripper-output",
        "joint-trajectory",
    ]
    if destination_shuttle_pose is not None:
        command.extend(
            [
                "--destination-shuttle-pose",
                *pose_argument_values(destination_shuttle_pose),
            ]
        )
    return command


def run_hpp_cycle(
    args,
    shuttle_pose,
    *,
    direction,
    destination_shuttle_pose=None,
):
    command = hpp_cycle_command(
        args,
        shuttle_pose,
        direction=direction,
        destination_shuttle_pose=destination_shuttle_pose,
    )
    print("running HPP cycle: " + " ".join(command), flush=True)
    subprocess.run(command, cwd=args.hpp_script.parent.parent, check=True)


def add_manipulation_arguments(parser):
    parser.add_argument("--robot-name", default="staubli1")
    parser.add_argument("--world-name", default="room_315_only")
    parser.add_argument("--box-entity-name", default="room315_payload_box")
    parser.add_argument("--replace-box", action="store_true")
    parser.set_defaults(
        start_tolerance=0.08,
    )


def finalize_manipulation_args(args, script_dir):
    args.hpp_script = script_dir / "room315_hpp_manipulation.sh"
    return args


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    add_manipulation_arguments(parser)
    parser.add_argument(
        "--shuttle-pose",
        nargs=6,
        type=float,
        default=DEFAULT_SHUTTLE_POSE,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Fixed pickup shuttle pose in the Gazebo world frame.",
    )
    return finalize_manipulation_args(parser.parse_args(argv), Path(__file__).parent)


def run_fixed_manipulation(node, args):
    shuttle_pose = pose_from_values(args.shuttle_pose)
    node.initialize_payload(
        payload_pose_from_shuttle_pose(shuttle_pose),
        "fixed pickup shuttle",
    )

    run_hpp_cycle(
        args,
        shuttle_pose,
        direction="shuttle-to-table",
    )


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    args.hpp_script = args.hpp_script.resolve()
    if not args.hpp_script.exists():
        raise RuntimeError(f"HPP wrapper does not exist: {args.hpp_script}")

    rclpy.init()
    node = ManipulationCoordinator(args)
    try:
        run_fixed_manipulation(node, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("fixed-support manipulation demo complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
