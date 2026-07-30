#!/usr/bin/env python3
"""ROS graph smoke for the fail-closed Room 315 visual runtime."""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest


os.environ.setdefault('ROS_DOMAIN_ID', '97')

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('message_filters')
from mfja_rail_interfaces.msg import ShuttleState
from mfja_rail_interfaces.msg import VisualStateObservation
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from room_315_visual_state_inference_node import Room315VisualStateInferenceNode


def test_ros_node_rejects_when_model_is_unavailable_and_owns_no_commands():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_sim_time:=false',
        '-p', 'checkpoint_path:=/definitely/missing/room315_best.pt',
        '-p', 'sidecar_directory:=/definitely/missing/room315_sidecars',
        '-p', 'presence_warmup_s:=0.0',
        '-p', 'presence_state_timeout_s:=5.0',
        '-p', 'diagnostic_period_s:=0.1',
    ])
    runtime = Room315VisualStateInferenceNode()
    probe = Node('room315_visual_runtime_integration_probe')
    received: list[VisualStateObservation] = []
    probe.create_subscription(
        VisualStateObservation,
        '/room_315/visual_state/validation',
        received.append,
        10,
    )
    left_presence = probe.create_publisher(
        ShuttleState,
        '/room_315/rails/left/shuttles/state',
        qos_profile_sensor_data,
    )
    right_presence = probe.create_publisher(
        ShuttleState,
        '/room_315/rails/right/shuttles/state',
        qos_profile_sensor_data,
    )
    left_image = probe.create_publisher(
        Image,
        '/room_315/vla/left_rail_rgbd/image',
        qos_profile_sensor_data,
    )
    right_image = probe.create_publisher(
        Image,
        '/room_315/vla/right_rail_rgbd/image',
        qos_profile_sensor_data,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(runtime)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not received:
            stamp = probe.get_clock().now().to_msg()
            left_state = ShuttleState()
            left_state.header.stamp = stamp
            left_state.name = 'room315_left_shuttle_1'
            right_state = ShuttleState()
            right_state.header.stamp = stamp
            right_state.name = 'room315_right_shuttle_1'
            left_presence.publish(left_state)
            right_presence.publish(right_state)

            left_frame = Image()
            left_frame.header.stamp = stamp
            left_frame.height = 1
            left_frame.width = 1
            left_frame.encoding = 'rgb8'
            left_frame.step = 3
            left_frame.data = bytes((0, 0, 0))
            right_frame = Image()
            right_frame.header.stamp = stamp
            right_frame.height = 1
            right_frame.width = 1
            right_frame.encoding = 'rgb8'
            right_frame.step = 3
            right_frame.data = bytes((0, 0, 0))
            left_image.publish(left_frame)
            right_image.publish(right_frame)
            executor.spin_once(timeout_sec=0.05)

        assert received, 'runtime did not publish its fail-closed validation result'
        result = received[-1]
        assert not result.accepted
        assert result.presence_ready
        assert 'model_runtime_not_ready' in result.validation_reasons
        assert all(
            shuttle.presence_state in {'present', 'absent'}
            for shuttle in result.shuttles
        )
        assert not any(shuttle.visual_facts_valid for shuttle in result.shuttles)

        publishers = probe.get_publisher_names_and_types_by_node(
            'room_315_visual_state_inference_node',
            '/',
        )
        published_topics = {topic for topic, _types in publishers}
        forbidden_fragments = (
            '/command',
            '/cmd_vel',
            '/trajectory',
            '/planner',
            '/executor',
        )
        assert not any(
            fragment in topic
            for topic in published_topics
            for fragment in forbidden_fragments
        )
    finally:
        executor.remove_node(probe)
        executor.remove_node(runtime)
        probe.destroy_node()
        runtime.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
