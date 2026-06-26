"""ROS, Gazebo, gripper, and payload execution helpers for the Room 315 demo."""

import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from hpp_exec import configs_to_joint_trajectory
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from room315_problem import (
    BOX_ENTITY_NAME,
    JOINT_NAMES,
    PAYLOAD_BOX_SDF,
    box_rank,
    box_world_pose_msg,
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
            self.configuration = np.array([positions[joint] for joint in JOINT_NAMES])
            self.last_update = time.monotonic()
        except KeyError:
            return

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.configuration is not None:
                return self.configuration.copy()
        return None

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


class BoolCommandGripperOutput:
    def __init__(self, node, args):
        self.node = node
        self.topic = args.gripper_command_topic or (
            f"/{args.robot_name}/gripper/command"
        )
        self.settle_s = args.gripper_settle_s
        self.publisher = node.create_publisher(Bool, self.topic, 10)
        wait_for_subscriber(node, self.publisher, self.topic, args.subscriber_timeout)

    def command(self, closed):
        message = Bool()
        message.data = closed
        self.publisher.publish(message)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        action = "close" if closed else "open"
        print(f"gripper pre-action {action}: {self.topic}={closed}", flush=True)
        if self.settle_s > 0.0:
            sleep_with_spin(self.node, self.settle_s)

    def open(self):
        self.command(False)

    def close(self):
        self.command(True)

    def destroy(self):
        pass


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
        self.publisher = node.create_publisher(JointTrajectory, self.topic, 10)
        wait_for_subscriber(node, self.publisher, self.topic, args.subscriber_timeout)

        if len(self.open_positions) != len(self.joints):
            raise RuntimeError("--gripper-open-positions must match --gripper-joints")
        if len(self.close_positions) != len(self.joints):
            raise RuntimeError("--gripper-close-positions must match --gripper-joints")

    def command(self, positions, label):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration_msg(self.duration)
        trajectory.points.append(point)
        publish_trajectory(self.node, self.publisher, self.topic, trajectory)
        print(
            f"gripper pre-action {label}: {self.topic} {self.joints} -> {positions}",
            flush=True,
        )
        if self.duration + self.settle_s > 0.0:
            sleep_with_spin(self.node, self.duration + self.settle_s)

    def open(self):
        self.command(self.open_positions, "open")

    def close(self):
        self.command(self.close_positions, "close")

    def destroy(self):
        pass


class StaubliIOGripperOutput:
    def __init__(self, node, args):
        from staubli_msgs.msg import IOModule
        from staubli_msgs.msg import ServiceReturnCode
        from staubli_msgs.srv import WriteSingleIO

        self.node = node
        self.WriteSingleIO = WriteSingleIO
        self.ServiceReturnCode = ServiceReturnCode
        self.service_name = args.staubli_io_service
        self.pin = args.staubli_io_pin
        self.module_id = (
            IOModule.VALVE_OUT
            if args.staubli_io_module_id is None
            else args.staubli_io_module_id
        )
        self.closed_state = not args.staubli_io_inverted
        self.open_state = args.staubli_io_inverted
        self.timeout = args.staubli_io_timeout
        self.settle_s = args.gripper_settle_s
        self.client = node.create_client(WriteSingleIO, self.service_name)

    def command(self, close):
        label = "close" if close else "open"
        state = self.closed_state if close else self.open_state
        if not self.client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(
                f"Staubli IO service {self.service_name} is unavailable"
            )

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

        print(
            f"gripper pre-action {label}: {self.service_name} "
            f"module={self.module_id} pin={self.pin} state={state}",
            flush=True,
        )
        if self.settle_s > 0.0:
            sleep_with_spin(self.node, self.settle_s)

    def open(self):
        self.command(False)

    def close(self):
        self.command(True)

    def destroy(self):
        pass


class NoGripperOutput:
    def open(self):
        pass

    def close(self):
        pass

    def destroy(self):
        pass


def make_gripper_output(node, args):
    if args.gripper_output == "bool":
        return BoolCommandGripperOutput(node, args)
    if args.gripper_output == "joint-trajectory":
        return JointTrajectoryGripperOutput(node, args)
    if args.gripper_output == "staubli-io":
        return StaubliIOGripperOutput(node, args)
    return NoGripperOutput()


def publish_trajectory(node, publisher, topic, trajectory):
    if publisher.get_subscription_count() == 0:
        node.get_logger().warning(f"no subscriber detected on {topic}")
    publisher.publish(trajectory)
    rclpy.spin_once(node, timeout_sec=0.05)


def timed_joint_trajectory(configs, times):
    return configs_to_joint_trajectory(configs, times, JOINT_NAMES)


def wait_for_subscriber(node, publisher, topic, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().warning(f"no subscriber detected on {topic}")


def call_service(node, client, request, label, timeout=3.0, require_success=True):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"{label} service is unavailable")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"{label} service call timed out")

    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} service returned no result")
    if require_success and hasattr(result, "success") and not result.success:
        raise RuntimeError(f"{label} service failed: {result}")
    return result


def delete_payload(node, client, entity_name):
    request = DeleteEntity.Request()
    request.entity.name = entity_name
    request.entity.type = Entity.MODEL
    try:
        call_service(
            node,
            client,
            request,
            f"delete {entity_name}",
            timeout=2.0,
            require_success=False,
        )
    except RuntimeError as exc:
        node.get_logger().warning(str(exc))


def spawn_payload(node, spawn_client, entity_name, pose):
    request = SpawnEntity.Request()
    request.entity_factory.name = entity_name
    request.entity_factory.allow_renaming = False
    request.entity_factory.sdf = PAYLOAD_BOX_SDF.replace(
        f'model name="{BOX_ENTITY_NAME}"',
        f'model name="{entity_name}"',
    )
    request.entity_factory.pose = pose
    request.entity_factory.relative_to = "world"
    try:
        call_service(node, spawn_client, request, f"spawn {entity_name}", timeout=5.0)
        return True
    except RuntimeError as exc:
        node.get_logger().warning(str(exc))
        return False


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


def wait_for_phase_end(node, tracker, phase, args):
    timeout = args.execution_timeout_scale * phase.times[-1] + 5.0
    if wait_for_arm_configuration(
        node, tracker, phase.configs[-1][:6], timeout, args.segment_tolerance
    ):
        return True

    current = tracker.current()
    error = (
        float(np.max(np.abs(current - phase.configs[-1][:6])))
        if current is not None
        else float("inf")
    )
    raise RuntimeError(
        f"Staubli did not finish phase {phase.name} within {timeout:.1f} s "
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
    last_report = start
    pending_pose = None

    while True:
        now = time.monotonic()
        if tracker.is_stale(args.joint_state_stale_timeout):
            raise RuntimeError(
                f"no fresh joint state on {tracker.topic} for "
                f"{args.joint_state_stale_timeout:.1f} s"
            )

        current = tracker.current()
        phase_end_error = float("inf")
        if current is not None:
            candidate, error = nearest_arm_progress(
                current,
                arm_positions,
                progress,
                args.payload_sync_lookahead,
            )
            if error <= args.payload_sync_error:
                progress = candidate
            elif now - last_report >= args.payload_sync_report_period:
                print(
                    f"payload sync waiting: progress={progress:.1f}/"
                    f"{len(arm_configs) - 1}, nearest error={error:.3f} rad",
                    flush=True,
                )
                last_report = now
            phase_end_error = float(np.max(np.abs(current - arm_positions[-1])))

        q = interpolate_indexed_config(robot, payload_configs, progress)
        if payload_pose_changed(robot, last_payload_config, q, args.payload_pose_epsilon):
            if pending_pose is not None and pending_pose.done():
                pending_pose = None
            if pending_pose is None:
                pending_pose = set_payload_pose_async(
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, q),
                )
                rclpy.spin_once(node, timeout_sec=0.0)
                last_payload_config = q

        if phase_end_error <= args.segment_tolerance:
            set_payload_pose(
                node,
                pose_client,
                entity_name,
                box_world_pose_msg(robot, payload_configs[-1]),
                timeout=0.5,
            )
            print(
                f"payload sync final snap: arm reached phase end, "
                f"progress={progress:.1f}/{len(arm_configs) - 1}",
                flush=True,
            )
            break
        if progress >= len(arm_configs) - 1:
            set_payload_pose(
                node,
                pose_client,
                entity_name,
                box_world_pose_msg(robot, payload_configs[-1]),
                timeout=0.5,
            )
            break
        if now >= deadline:
            final_snap_start = len(arm_configs) - 1 - args.payload_final_snap_samples
            if progress >= final_snap_start:
                set_payload_pose(
                    node,
                    pose_client,
                    entity_name,
                    box_world_pose_msg(robot, payload_configs[-1]),
                    timeout=0.5,
                )
                print(
                    f"payload sync final snap: progress={progress:.1f}/"
                    f"{len(arm_configs) - 1}",
                    flush=True,
                )
                break
            raise RuntimeError(
                f"payload sync timed out at progress {progress:.1f}/"
                f"{len(arm_configs) - 1}"
            )
        next_tick += period
        sleep_with_spin(node, max(0.0, next_tick - time.monotonic()))


def set_payload_config(node, pose_client, robot, entity_name, config):
    set_payload_pose(
        node,
        pose_client,
        entity_name,
        box_world_pose_msg(robot, config),
    )


def semantic_grasp(
    node,
    gripper,
    pose_client,
    robot,
    entity_name,
    phase,
):
    gripper.close()
    print("semantic grasp: payload follows gripper TCP pose")
    if pose_client is not None:
        set_payload_config(
            node, pose_client, robot, entity_name, phase.payload_configs[0]
        )
    return True


def semantic_release(
    node,
    gripper,
    pose_client,
    robot,
    entity_name,
    phase,
):
    gripper.open()
    print(f"semantic release: payload fixed in {phase.payload_mode}")
    if pose_client is not None:
        set_payload_config(
            node, pose_client, robot, entity_name, phase.payload_configs[0]
        )
    return True


def execute_phase(
    node,
    publisher,
    topic,
    pose_client,
    tracker,
    robot,
    entity_name,
    phase,
    args,
):
    trajectory = timed_joint_trajectory(phase.configs, phase.times)
    print(
        f"publishing phase {phase.name}: "
        f"{len(phase.configs)} points, {phase.times[-1]:.1f} s",
        flush=True,
    )
    publish_trajectory(node, publisher, topic, trajectory)

    if phase.payload_mode == "follow" and pose_client is not None:
        follow_payload(
            node,
            pose_client,
            tracker,
            robot,
            entity_name,
            phase.configs,
            phase.payload_configs,
            phase.times,
            args,
        )
        wait_for_phase_end(node, tracker, phase, args)
    else:
        wait_for_phase_end(node, tracker, phase, args)


def move_to_start(node, publisher, topic, tracker, args, q_start):
    current = tracker.wait(args.joint_state_timeout)
    if current is None:
        raise RuntimeError(f"could not read {tracker.topic}")

    delta = float(np.max(np.abs(current - q_start[:6])))
    if delta < 0.02:
        return

    duration = max(args.min_start_duration, delta / args.start_joint_speed)
    n_samples = max(3, int(duration * args.start_samples_per_second) + 1)
    start_configs = [
        (1.0 - alpha) * current + alpha * q_start[:6]
        for alpha in np.linspace(0.0, 1.0, n_samples)
    ]
    trajectory = timed_joint_trajectory(
        [current] + start_configs,
        [0.0]
        + np.linspace(args.initial_hold, args.initial_hold + duration, n_samples).tolist(),
    )
    print(f"moving Staubli to the planned start ({duration:.1f} s)")
    publish_trajectory(node, publisher, topic, trajectory)
    timeout = args.execution_timeout_scale * (args.initial_hold + duration) + 5.0
    if not wait_for_arm_configuration(
        node, tracker, q_start[:6], timeout, args.start_tolerance
    ):
        current = tracker.current()
        error = (
            float(np.max(np.abs(current - q_start[:6])))
            if current is not None
            else float("inf")
        )
        raise RuntimeError(
            f"Staubli did not reach the planned start within {timeout:.1f} s "
            f"(error {error:.3f} rad)"
        )


def require_start(node, tracker, args, q_start):
    current = tracker.wait(args.joint_state_timeout)
    if current is None:
        raise RuntimeError(f"could not read {tracker.topic}")

    target = q_start[:6]
    error = float(np.max(np.abs(current - target)))
    if error > args.start_tolerance:
        raise RuntimeError(
            f"Staubli is {error:.3f} rad from the HPP start. Run the moving "
            "demo helper's pre-position step first, or pass --q-start for the "
            "real robot pose. Only use --start-mode move after checking that "
            "pre-position path is clear."
        )

    print(f"Staubli already at the planned start (error {error:.3f} rad)", flush=True)


def snap_to_start(node, publisher, topic, tracker, args, q_start):
    current = tracker.wait(args.joint_state_timeout)
    if current is None:
        raise RuntimeError(f"could not read {tracker.topic}")

    target = q_start[:6]
    delta = float(np.max(np.abs(current - target)))
    if delta < 0.02:
        return

    duration = args.snap_start_duration
    n_samples = max(2, int(duration * args.start_samples_per_second) + 1)
    start_configs = [
        (1.0 - alpha) * current + alpha * target
        for alpha in np.linspace(0.0, 1.0, n_samples)
    ]
    trajectory = timed_joint_trajectory(
        start_configs,
        np.linspace(0.0, duration, n_samples).tolist(),
    )
    print(f"snapping Staubli to the planned start ({duration:.1f} s)", flush=True)
    publish_trajectory(node, publisher, topic, trajectory)
    if not wait_for_arm_configuration(
        node, tracker, target, args.snap_start_timeout, args.start_tolerance
    ):
        current = tracker.current()
        error = (
            float(np.max(np.abs(current - target))) if current is not None else float("inf")
        )
        raise RuntimeError(
            f"Staubli did not settle at the planned start within "
            f"{args.snap_start_timeout:.1f} s (error {error:.3f} rad)"
        )


def execute_plan(
    robot,
    phases,
    q_source,
    args,
):
    rclpy.init()
    node = Node("room315_hpp_manipulation")
    gripper = None
    try:
        trajectory_topic = args.trajectory_topic or (
            f"/{args.robot_name}/joint_trajectory"
        )
        joint_state_topic = args.joint_state_topic or f"/{args.robot_name}/joint_states"
        publisher = node.create_publisher(JointTrajectory, trajectory_topic, 10)
        tracker = JointStateTracker(node, joint_state_topic)
        gripper = make_gripper_output(node, args)
        wait_for_subscriber(node, publisher, trajectory_topic, args.subscriber_timeout)

        pose_client = None
        if args.payload_output == "gazebo":
            service_prefix = f"/world/{args.world_name}"
            spawn_client = node.create_client(SpawnEntity, f"{service_prefix}/create")
            delete_client = node.create_client(DeleteEntity, f"{service_prefix}/remove")
            pose_client = node.create_client(SetEntityPose, f"{service_prefix}/set_pose")

            if args.replace_box:
                delete_payload(node, delete_client, args.box_entity_name)
            spawned = spawn_payload(
                node,
                spawn_client,
                args.box_entity_name,
                box_world_pose_msg(robot, q_source),
            )
            if not spawned:
                node.get_logger().info(
                    f"using existing Gazebo entity {args.box_entity_name}"
                )
            set_payload_pose(
                node,
                pose_client,
                args.box_entity_name,
                box_world_pose_msg(robot, q_source),
            )

        if args.start_mode == "check":
            require_start(node, tracker, args, q_source)
        elif args.start_mode == "move":
            move_to_start(node, publisher, trajectory_topic, tracker, args, q_source)
        else:
            snap_to_start(node, publisher, trajectory_topic, tracker, args, q_source)

        if pose_client is not None:
            set_payload_config(
                node,
                pose_client,
                robot,
                args.box_entity_name,
                q_source,
            )

        execute_phase(
            node,
            publisher,
            trajectory_topic,
            pose_client,
            tracker,
            robot,
            args.box_entity_name,
            phases[0],
            args,
        )
        semantic_grasp(
            node,
            gripper,
            pose_client,
            robot,
            args.box_entity_name,
            phases[1],
        )
        execute_phase(
            node,
            publisher,
            trajectory_topic,
            pose_client,
            tracker,
            robot,
            args.box_entity_name,
            phases[1],
            args,
        )
        semantic_release(
            node,
            gripper,
            pose_client,
            robot,
            args.box_entity_name,
            phases[2],
        )
        execute_phase(
            node,
            publisher,
            trajectory_topic,
            pose_client,
            tracker,
            robot,
            args.box_entity_name,
            phases[2],
            args,
        )
    finally:
        if gripper is not None:
            gripper.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def touch_file(path):
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{time.monotonic():.3f}\n")


def wait_for_execution_start(args):
    touch_file(args.ready_file)
    if args.start_file is None:
        return True

    print(f"HPP plan ready; waiting for execution trigger {args.start_file}", flush=True)
    while True:
        if args.abort_file is not None and args.abort_file.exists():
            print(f"HPP execution aborted by {args.abort_file}", flush=True)
            return False
        if args.start_file.exists():
            print("HPP execution trigger received", flush=True)
            return True
        time.sleep(0.1)
