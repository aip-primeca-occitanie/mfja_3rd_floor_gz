"""Execute the Room 315 manipulation plan through ROS and Gazebo."""

from copy import copy
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from hpp_exec import (
    configs_to_joint_trajectory,
    execute_segments as execute_hpp_segments,
    segments_by_transition,
)
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from room315_problem import (
    box_rank,
    box_world_pose_msg,
    config,
    normalize_box_quaternion,
)


class JointStateTracker:
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic
        self.configuration = None
        self.last_update = None
        self.subscription = node.create_subscription(
            JointState, topic, self.update, 10
        )

    def update(self, message):
        positions = {
            name.split("::")[-1]: value
            for name, value in zip(message.name, message.position)
        }
        try:
            self.configuration = np.array(
                [positions[joint] for joint in config["planning"]["joint_names"]]
            )
            self.last_update = time.monotonic()
        except KeyError:
            return

    def current(self):
        if self.configuration is None:
            return None
        return self.configuration.copy()

    def is_stale(self, timeout):
        return (
            self.last_update is None
            or time.monotonic() - self.last_update > timeout
        )


def duration_msg(seconds):
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - msg.sec) * 1e9)
    return msg


class JointTrajectoryGripperOutput:
    def __init__(self, node, args):
        self.node = node
        self.topic = args.gripper_trajectory_topic or (
            f"/{args.robot_name}/gripper_joint_trajectory"
        )
        self.joints = list(args.gripper_joints)
        self.open_positions = list(args.gripper_open_positions)
        self.close_positions = list(args.gripper_close_positions)
        self.duration = args.gripper_motion_duration
        self.settle_s = args.gripper_settle_s
        if len(self.open_positions) != len(self.joints):
            raise RuntimeError("gripper open positions do not match its joints")
        if len(self.close_positions) != len(self.joints):
            raise RuntimeError("gripper close positions do not match its joints")
        self.publisher = node.create_publisher(JointTrajectory, self.topic, 10)
        wait_for_subscriber(node, self.publisher, self.topic, args.subscriber_timeout)

    def command(self, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration_msg(self.duration)
        trajectory.points.append(point)
        publish_trajectory(self.node, self.publisher, self.topic, trajectory)
        if self.duration + self.settle_s > 0.0:
            sleep_with_spin(self.node, self.duration + self.settle_s)
        return True

    def open(self):
        return self.command(self.open_positions)

    def close(self):
        return self.command(self.close_positions)


class StaubliIOGripperOutput:
    def __init__(self, node, args):
        try:
            from staubli_msgs.msg import IOModule
            from staubli_msgs.msg import ServiceReturnCode
            from staubli_msgs.srv import WriteSingleIO
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "staubli_msgs is required for --gripper-output staubli-io"
            ) from exc

        self.node = node
        self.WriteSingleIO = WriteSingleIO
        self.ServiceReturnCode = ServiceReturnCode
        self.service_name = args.staubli_io_service
        self.module_id = IOModule.VALVE_OUT
        self.pin = 0
        self.timeout = args.staubli_io_timeout
        self.settle_s = args.gripper_settle_s
        self.client = node.create_client(WriteSingleIO, self.service_name)
        if not self.client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(
                f"Staubli IO service {self.service_name} is unavailable"
            )

    def command(self, state, label):
        request = self.WriteSingleIO.Request()
        request.module.id = self.module_id
        request.pin = self.pin
        request.state = state
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=self.timeout
        )
        response = future.result()
        if response is None:
            raise RuntimeError(
                f"Staubli IO gripper {label} timed out on {self.service_name}"
            )
        if response.code.val != self.ServiceReturnCode.SUCCESS:
            raise RuntimeError(
                f"Staubli IO gripper {label} failed on "
                f"{self.service_name} pin {self.pin}: code {response.code.val}"
            )

        if self.settle_s > 0.0:
            sleep_with_spin(self.node, self.settle_s)
        return True

    def open(self):
        return self.command(True, "open")

    def close(self):
        return self.command(False, "close")


class NoGripperOutput:
    def open(self):
        return True

    def close(self):
        return True


def make_gripper_output(node, args):
    if args.gripper_output == "joint-trajectory":
        return JointTrajectoryGripperOutput(node, args)
    if args.gripper_output == "staubli-io":
        return StaubliIOGripperOutput(node, args)
    return NoGripperOutput()


def publish_trajectory(node, publisher, topic, trajectory):
    if publisher.get_subscription_count() == 0:
        raise RuntimeError(f"no subscriber detected on {topic}")
    publisher.publish(trajectory)
    rclpy.spin_once(node, timeout_sec=0.05)


def wait_for_subscriber(node, publisher, topic, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        raise RuntimeError(f"no subscriber detected on {topic}")


def call_service(node, client, request, label, timeout=3.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"{label} service is unavailable")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"{label} service call timed out")

    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} service returned no result")
    if not result.success:
        raise RuntimeError(f"{label} service failed: {result}")
    return result


def make_set_payload_pose_request(entity_name, pose):
    request = SetEntityPose.Request()
    request.entity.name = entity_name
    request.entity.type = Entity.MODEL
    request.pose = pose
    return request


def set_payload_pose(node, pose_client, entity_name, pose, timeout=1.0):
    call_service(
        node,
        pose_client,
        make_set_payload_pose_request(entity_name, pose),
        f"set pose for {entity_name}",
        timeout=timeout,
    )
    return True


def set_payload_pose_async(pose_client, entity_name, pose):
    return pose_client.call_async(make_set_payload_pose_request(entity_name, pose))


def sleep_with_spin(node, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
        )


def interpolate_indexed_config(robot, configs, progress):
    if progress <= 0:
        return configs[0]
    if progress >= len(configs) - 1:
        return configs[-1]

    lower = int(np.floor(progress))
    upper = lower + 1
    alpha = progress - lower
    q = (1.0 - alpha) * configs[lower] + alpha * configs[upper]
    return normalize_box_quaternion(robot, q)


def nearest_arm_progress(current, arm_positions, progress, lookahead):
    if len(arm_positions) < 2:
        return 0.0, float(np.max(np.abs(current - arm_positions[0])))

    first = max(0, int(np.floor(progress)) - 1)
    last = min(len(arm_positions) - 2, int(np.floor(progress)) + lookahead)
    best_progress = progress
    best_error = float("inf")

    for index in range(first, last + 1):
        start = arm_positions[index]
        end = arm_positions[index + 1]
        delta = end - start
        norm2 = float(delta @ delta)
        if norm2 <= 1e-12:
            alpha = 0.0
            closest = start
        else:
            alpha = float(np.clip(((current - start) @ delta) / norm2, 0.0, 1.0))
            closest = start + alpha * delta
        error = float(np.max(np.abs(current - closest)))
        candidate = index + alpha
        if error < best_error:
            best_error = error
            best_progress = candidate

    return max(progress, best_progress), best_error


def payload_pose_changed(robot, previous, current, threshold):
    if previous is None:
        return True
    rank = box_rank(robot)
    return (
        np.max(np.abs(current[rank : rank + 3] - previous[rank : rank + 3]))
        > threshold
        or np.max(np.abs(current[rank + 3 : rank + 7] - previous[rank + 3 : rank + 7]))
        > threshold
    )


def wait_for_arm_configuration(node, tracker, target, timeout, tolerance):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = tracker.current()
        if current is not None:
            error = float(np.max(np.abs(current - target)))
            if error <= tolerance:
                return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


def segment_samples(plan, segment):
    configs = plan.configs[segment.start_index : segment.end_index]
    payload_configs = plan.payload_configs[segment.start_index : segment.end_index]
    times = plan.times[segment.start_index : segment.end_index]
    start_time = times[0]
    return configs, payload_configs, [value - start_time for value in times]


def wait_for_segment_end(node, tracker, plan, segment, name, args):
    configs, _, times = segment_samples(plan, segment)
    timeout = args.execution_timeout_scale * times[-1] + 5.0
    if wait_for_arm_configuration(
        node, tracker, configs[-1][:6], timeout, args.segment_tolerance
    ):
        return True

    current = tracker.current()
    error = (
        float(np.max(np.abs(current - configs[-1][:6])))
        if current is not None
        else float("inf")
    )
    raise RuntimeError(
        f"Staubli did not finish segment {name} within {timeout:.1f} s "
        f"(error {error:.3f} rad)"
    )


def follow_payload(
    node,
    pose_client,
    tracker,
    robot,
    entity_name,
    arm_configs,
    payload_configs,
    times,
    args,
):
    arm_positions = np.asarray([config[:6] for config in arm_configs])
    period = 1.0 / args.box_rate
    start = time.monotonic()
    deadline = start + args.execution_timeout_scale * times[-1] + 30.0
    next_tick = start
    progress = 0.0
    last_payload_config = None
    pending_pose = None
    pending_payload_config = None

    while True:
        now = time.monotonic()
        if tracker.is_stale(args.joint_state_stale_timeout):
            raise RuntimeError(
                f"no fresh joint state on {tracker.topic} for "
                f"{args.joint_state_stale_timeout:.1f} s"
            )

        if pending_pose is not None and pending_pose.done():
            try:
                result = pending_pose.result()
            except Exception as exc:
                raise RuntimeError(
                    f"set pose for {entity_name} failed: {exc}"
                ) from exc
            if result is None:
                raise RuntimeError(
                    f"set pose for {entity_name} returned no result"
                )
            if not result.success:
                raise RuntimeError(
                    f"set pose for {entity_name} failed: {result}"
                )
            last_payload_config = pending_payload_config
            pending_pose = None
            pending_payload_config = None

        current = tracker.current()
        segment_end_error = float("inf")
        if current is not None:
            candidate, error = nearest_arm_progress(
                current,
                arm_positions,
                progress,
                args.payload_sync_lookahead,
            )
            if error <= args.payload_sync_error:
                progress = candidate
            segment_end_error = float(np.max(np.abs(current - arm_positions[-1])))

        if pending_pose is not None and now >= deadline:
            raise RuntimeError(
                f"set pose for {entity_name} did not complete before "
                "the payload sync deadline"
            )

        if segment_end_error <= args.segment_tolerance:
            if pending_pose is None:
                set_payload_pose(
                    node,
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, payload_configs[-1]),
                    timeout=0.5,
                )
                break
        elif progress >= len(arm_configs) - 1:
            if pending_pose is None:
                set_payload_pose(
                    node,
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, payload_configs[-1]),
                    timeout=0.5,
                )
                break
        elif now >= deadline:
            final_snap_start = len(arm_configs) - 1 - args.payload_final_snap_samples
            if progress >= final_snap_start:
                set_payload_pose(
                    node,
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, payload_configs[-1]),
                    timeout=0.5,
                )
                break
            raise RuntimeError(
                f"payload sync timed out at progress {progress:.1f}/"
                f"{len(arm_configs) - 1}"
            )
        else:
            q = interpolate_indexed_config(robot, payload_configs, progress)
            if pending_pose is None and payload_pose_changed(
                robot, last_payload_config, q, args.payload_pose_epsilon
            ):
                pending_pose = set_payload_pose_async(
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, q),
                )
                pending_payload_config = q.copy()
                rclpy.spin_once(node, timeout_sec=0.0)
        next_tick += period
        sleep_with_spin(node, max(0.0, next_tick - time.monotonic()))


def run_segment_actions(actions, segment_index, position):
    for action in actions:
        if not action():
            raise RuntimeError(
                f"execution segment {segment_index} {position}-action failed"
            )


def execute_topic_segments(
    node,
    publisher,
    topic,
    pose_client,
    tracker,
    robot,
    entity_name,
    plan,
    segments,
    args,
):
    for index, (segment, payload_mode) in enumerate(
        zip(segments, plan.payload_modes)
    ):
        run_segment_actions(segment.pre_actions, index, "pre")
        configs, payload_configs, times = segment_samples(plan, segment)
        trajectory = configs_to_joint_trajectory(
            configs, times, config["planning"]["joint_names"]
        )
        publish_trajectory(node, publisher, topic, trajectory)

        if payload_mode == "follow" and pose_client is not None:
            follow_payload(
                node,
                pose_client,
                tracker,
                robot,
                entity_name,
                configs,
                payload_configs,
                times,
                args,
            )
        run_segment_actions(segment.post_actions, index, "post")


def configured_segments(node, tracker, pose_client, robot, plan, gripper, args):
    transitions = config["graph"]["transitions"]
    segments = []
    for segment in plan.segments:
        configured = copy(segment)
        configured.pre_actions = list(segment.pre_actions)
        configured.post_actions = list(segment.post_actions)
        segments.append(configured)

    by_transition = segments_by_transition(segments)
    grasp_segment = by_transition[transitions["grasp"]][0]
    release_segment = by_transition[transitions["detach"]][0]

    if args.gripper_output == "joint-trajectory":
        segments[0].pre_actions.append(gripper.open)
    grasp_segment.pre_actions.append(gripper.close)
    release_segment.pre_actions.append(gripper.open)

    if pose_client is not None:
        grasp_pose = box_world_pose_msg(
            robot, plan.payload_configs[grasp_segment.start_index]
        )
        release_pose = box_world_pose_msg(
            robot, plan.payload_configs[release_segment.start_index]
        )

        def attach_payload():
            return set_payload_pose(
                node, pose_client, args.box_entity_name, grasp_pose
            )

        def detach_payload():
            return set_payload_pose(
                node, pose_client, args.box_entity_name, release_pose
            )

        grasp_segment.pre_actions.append(attach_payload)
        release_segment.pre_actions.append(detach_payload)

    for segment, name in zip(segments, plan.segment_names):

        def wait_for_end(segment=segment, name=name):
            return wait_for_segment_end(
                node, tracker, plan, segment, name, args
            )

        segment.post_actions.append(wait_for_end)
    return segments


def execute_action_segments(plan, segments, controller_topic):
    if not execute_hpp_segments(
        segments,
        plan.configs,
        plan.times,
        config["planning"]["joint_names"],
        controller_topic=controller_topic,
    ):
        raise RuntimeError(
            f"Staubli failed the HPP execution segments on {controller_topic}"
        )


def require_start(node, tracker, args, q_start):
    target = q_start[:6]
    wait_for_arm_configuration(
        node,
        tracker,
        target,
        args.joint_state_timeout,
        args.start_tolerance,
    )
    current = tracker.current()
    if current is None:
        raise RuntimeError(f"could not read {tracker.topic}")

    error = float(np.max(np.abs(current - target)))
    if error > args.start_tolerance:
        raise RuntimeError(
            f"Staubli is {error:.3f} rad from the HPP start. In simulation, "
            "restart the manipulation scene and wait for its controller to reach "
            "the configured startup target. On hardware, pass "
            "--q-start for a measured robot pose validated in the commissioned "
            "model."
        )


def execute_plan(
    robot,
    plan,
    q_source,
    args,
):
    rclpy.init()
    node = Node("room315_pick_place")
    try:
        if args.trajectory_action is None:
            arm_target = args.trajectory_topic or (
                f"/{args.robot_name}/joint_trajectory"
            )
            publisher = node.create_publisher(JointTrajectory, arm_target, 10)
            wait_for_subscriber(node, publisher, arm_target, args.subscriber_timeout)
        else:
            arm_target = args.trajectory_action
            publisher = None
        joint_state_topic = args.joint_state_topic or (
            "/joint_states"
            if args.trajectory_action is not None
            else f"/{args.robot_name}/joint_states"
        )
        tracker = JointStateTracker(node, joint_state_topic)
        gripper = make_gripper_output(node, args)

        pose_client = None
        if args.payload_output == "gazebo":
            service_prefix = f"/world/{args.world_name}"
            pose_client = node.create_client(SetEntityPose, f"{service_prefix}/set_pose")
            set_payload_pose(
                node,
                pose_client,
                args.box_entity_name,
                box_world_pose_msg(robot, q_source),
            )

        require_start(node, tracker, args, q_source)
        segments = configured_segments(
            node,
            tracker,
            pose_client,
            robot,
            plan,
            gripper,
            args,
        )
        if publisher is None:
            execute_action_segments(plan, segments, arm_target)
        else:
            execute_topic_segments(
                node,
                publisher,
                arm_target,
                pose_client,
                tracker,
                robot,
                args.box_entity_name,
                plan,
                segments,
                args,
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
