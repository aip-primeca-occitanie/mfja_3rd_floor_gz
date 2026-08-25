#!/usr/bin/env python3
"""Read and print the Staubli configuration without publishing any command."""

from __future__ import annotations

import argparse
import json

import numpy as np
import rclpy
from hpp_exec import read_current_configuration
from rclpy.node import Node

JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    if not np.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be finite and positive")

    rclpy.init()
    node = Node("room315_read_configuration")
    try:
        configuration = read_current_configuration(
            node,
            JOINT_NAMES,
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
                "joint_names": JOINT_NAMES,
                "positions": configuration.tolist(),
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
