#!/usr/bin/env python3
"""Read and print the Staubli configuration without publishing any command."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from hpp_exec import read_current_configuration
from rclpy.node import Node


def main() -> int:
    config_path = (
        Path(get_package_share_directory("mfja_staubli_demos"))
        / "config"
        / "room315_cartesian_line.yaml"
    )
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    joint_names = config["robot"]["joint_names"]

    if not np.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be finite and positive")

    rclpy.init()
    node = Node("room315_read_configuration")
    try:
        configuration = read_current_configuration(
            node,
            joint_names,
            topic=args.topic,
            timeout_sec=args.timeout,
            require_single_publisher=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if configuration is None:
        raise TimeoutError(
            f"no configuration received from {args.topic}; verify the topic "
            "and ROS_DOMAIN_ID"
        )
    if not np.all(np.isfinite(configuration)):
        raise RuntimeError("the received configuration contains non-finite values")

    print(
        json.dumps(
            {
                "topic": args.topic,
                "joint_names": joint_names,
                "positions": configuration.tolist(),
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
