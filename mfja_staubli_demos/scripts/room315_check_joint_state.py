#!/usr/bin/env python3
"""Exercise both installed hpp-exec joint-state APIs on isolated loopback DDS."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from hpp_exec import JointStateReader, read_current_configuration
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main() -> None:
    if os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST":
        raise RuntimeError("set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
    if os.environ.get("ROS_DOMAIN_ID") != "229":
        raise RuntimeError("set ROS_DOMAIN_ID=229 for the isolated self-test")

    config_path = (
        Path(get_package_share_directory("mfja_staubli_demos"))
        / "config"
        / "room315_cartesian_line.yaml"
    )
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    joint_names = config["robot"]["joint_names"]
    valid_configuration = config["robot"]["default_configuration"]
    topic = config["self_test"]["joint_state_topic"]

    rclpy.init()
    publisher_node = Node("mfja_hpp_joint_state_test_publisher")
    publisher = publisher_node.create_publisher(JointState, topic, 10)
    message = JointState()
    message.name = [
        "joint_6",
        "joint_5",
        "joint_4",
        "joint_3",
        "joint_2",
        "joint_1",
    ]
    message.position = [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]

    try:
        consumer = Node("mfja_hpp_ordered_joint_state_test")
        stop = threading.Event()

        def publish_repeatedly() -> None:
            while not stop.wait(0.05):
                publisher.publish(message)

        publishing_thread = threading.Thread(target=publish_repeatedly, daemon=True)
        publishing_thread.start()
        try:
            configuration = read_current_configuration(
                consumer,
                ["joint_1", "joint_2"],
                topic=topic,
                timeout_sec=5.0,
                require_single_publisher=True,
            )
            if configuration is None:
                raise TimeoutError(
                    "read_current_configuration did not receive loopback data"
                )
            np.testing.assert_allclose(configuration, [1.0, 2.0])
        finally:
            stop.set()
            publishing_thread.join(timeout=1.0)
            consumer.destroy_node()

        reader = JointStateReader(topic)
        stop.clear()
        publishing_thread = threading.Thread(target=publish_repeatedly, daemon=True)
        publishing_thread.start()
        try:
            for _ in range(100):
                rclpy.spin_once(reader, timeout_sec=0.05)
                if reader.get_current_configuration() is not None:
                    break
            configuration = reader.get_current_configuration()
            if configuration is None:
                raise TimeoutError("JointStateReader did not receive loopback data")
            np.testing.assert_allclose(configuration, [6, 5, 4, 3, 2, 1])
        finally:
            stop.set()
            publishing_thread.join(timeout=1.0)
            reader.destroy_node()

        stop.clear()
        publishing_thread = threading.Thread(target=publish_repeatedly, daemon=True)
        publishing_thread.start()
        try:
            completed = subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).with_name("room315_read_configuration.py")),
                    "--topic",
                    topic,
                    "--timeout",
                    "5",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=10,
            )
            payload = json.loads(completed.stdout)
            np.testing.assert_allclose(payload["positions"], [1, 2, 3, 4, 5, 6])
        finally:
            stop.set()
            publishing_thread.join(timeout=1.0)

        message.position = list(reversed(valid_configuration))
        stop.clear()
        publishing_thread = threading.Thread(target=publish_repeatedly, daemon=True)
        publishing_thread.start()
        try:
            completed = subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).with_name("room315_export_staubli_line.py")),
                    "--joint-states-topic",
                    topic,
                    "--line",
                    "0",
                    "0",
                    "0.10",
                    "--duration",
                    "5",
                    "--samples",
                    "20",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=20,
            )
            trajectory = json.loads(completed.stdout)
            if trajectory["joint_names"] != list(joint_names):
                raise RuntimeError("live exporter returned unexpected joint names")
            np.testing.assert_allclose(
                trajectory["points"][0]["positions"], valid_configuration
            )
        finally:
            stop.set()
            publishing_thread.join(timeout=1.0)
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

    print(
        "Joint-state DDS self-test: JointStateReader, ordered reader, "
        "MFJA CLI, and live HPP exporter passed"
    )


if __name__ == "__main__":
    main()
