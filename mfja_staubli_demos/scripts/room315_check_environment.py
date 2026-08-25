#!/usr/bin/env python3
"""Report the installed HPP, ROS, and MFJA Python environment."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import hpp_exec
import pyhpp
import rclpy
import staubli_msgs
from ament_index_python.packages import get_package_share_path
from hpp_exec import read_current_configuration, send_trajectory
from pyhpp.manipulation import Device
from staubli_msgs.srv import WriteSingleIO


def module_path(module: object) -> Path:
    return Path(module.__file__).resolve()


def main() -> None:
    if not callable(read_current_configuration) or not callable(send_trajectory):
        raise RuntimeError("hpp-exec ROS APIs are unavailable")
    if WriteSingleIO is None:
        raise RuntimeError("staubli_msgs Python type support is unavailable")

    model = (
        get_package_share_path("mfja_staubli_manipulation_demos")
        / "models"
        / "room315_payload_box.sdf"
    )
    if not model.is_file():
        raise RuntimeError(f"installed payload model is missing: {model}")

    print(f"python={sys.version.split()[0]} ({sys.executable})")
    print(f"ros-distro={os.environ.get('ROS_DISTRO', 'unknown')}")
    print(f"pyhpp={module_path(pyhpp)}")
    print(f"hpp-exec={module_path(hpp_exec)}")
    print(f"rclpy={module_path(rclpy)}")
    print(f"staubli-msgs={module_path(staubli_msgs)}")
    print(f"payload-model={model.resolve()}")
    print("MFJA environment check: HPP and ROS imports share one Python ABI")


if __name__ == "__main__":
    main()
