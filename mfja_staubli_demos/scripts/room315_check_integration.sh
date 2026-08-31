#!/usr/bin/env bash
# Focused, read-only validation of the installed HPP/MFJA environment.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
mfja_root=$(cd -- "$script_dir/../.." && pwd -P)
hpp_exec_source=${HPP_EXEC_SOURCE:-}

python3 "$script_dir/room315_check_environment.py"
python3 "$script_dir/room315_read_configuration.py" --help >/dev/null

required_staubli_packages=(
  industrial_msgs
  industrial_robot_client
  industrial_utils
  simple_message
  staubli_msgs
  staubli_tx2_60l_description
  staubli_val3_driver
  urdf_extention
)
for package in "${required_staubli_packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null
done

obsolete_staubli_packages=(
  motion_control_msgs
  moveit_interface
  robot_middleware
  staubli_support
  staubli_tx2_60l_moveit_config
)
for package in "${obsolete_staubli_packages[@]}"; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Obsolete Staubli package remains visible: $package" >&2
    exit 1
  fi
done

installed_manipulation_executables=$(
  ros2 pkg executables mfja_staubli_manipulation_demos
)
for executable in \
  room315_demo.sh \
  room315_hpp_manipulation.sh \
  room315_manipulation_demo.sh \
  room315_moving_shuttle_demo.sh; do
  if grep -Fq "mfja_staubli_manipulation_demos $executable" \
    <<<"$installed_manipulation_executables"; then
    echo "Legacy shuttle executable remains visible: $executable" >&2
    exit 1
  fi
done

python3 - <<'PY'
import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_path
from launch import LaunchDescription
import yaml


def generate_launch_description(path: Path) -> LaunchDescription:
    namespace = {"__file__": str(path), "__name__": f"static_check_{path.stem}"}
    exec(compile(path.read_text(), str(path), "exec"), namespace)
    description = namespace["generate_launch_description"]()
    if not isinstance(description, LaunchDescription) or not description.entities:
        raise RuntimeError(f"invalid launch description: {path}")
    return description


driver_share = get_package_share_path("staubli_val3_driver")
description_share = get_package_share_path("staubli_tx2_60l_description")
demo_share = get_package_share_path("mfja_staubli_manipulation_demos")
driver_launch = driver_share / "launch" / "robot_interface_streaming.launch.py"
simulation_launch = demo_share / "launch" / "room_315_staubli_pick_place_sim.launch.py"
for path in (
    driver_launch,
    driver_share / "launch" / "robot_state.launch.py",
    driver_share / "launch" / "motion_streaming_interface.launch.py",
    driver_share / "launch" / "io_interface.launch.py",
    driver_share / "launch" / "system_interface.launch.py",
    demo_share / "launch" / "room_315_staubli_hardware.launch.py",
    simulation_launch,
    demo_share / "models" / "room315_payload_box.sdf",
):
    if not path.is_file():
        raise RuntimeError(f"installed launch file is missing: {path}")

if (demo_share / "models" / "room315_pick_support.sdf").exists():
    raise RuntimeError("obsolete fixed-support model remains installed")
for name in (
    "room315_shuttle_manipulation.py",
    "room315_shuttle_deck.urdf",
    "room315_shuttle_deck.srdf",
):
    if (demo_share / "hpp" / name).exists():
        raise RuntimeError(f"obsolete shuttle file remains installed: {name}")

generate_launch_description(driver_launch)
generate_launch_description(
    demo_share / "launch" / "room_315_staubli_hardware.launch.py"
)
generate_launch_description(simulation_launch)

robot_xml = xacro.process_file(
    str(description_share / "urdf" / "tx2_60l.xacro")
).toxml()
for joint_name in ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"):
    if f'name="{joint_name}"' not in robot_xml:
        raise RuntimeError(f"Staubli robot description is missing {joint_name}")

joint_config_path = driver_share / "config" / "tx2_60l_streaming.yaml"
joint_config = yaml.safe_load(joint_config_path.read_text())
expected_joint_names = [f"joint_{index}" for index in range(1, 7)]
for node_name in (
    "robot_state_interface",
    "joint_trajectory_interface",
    "joint_trajectory_action",
):
    configured = joint_config[node_name]["ros__parameters"]["joint_names"]
    if configured != expected_joint_names:
        raise RuntimeError(f"{node_name} has unexpected joint names: {configured}")
velocity_limits = joint_config["joint_trajectory_interface"]["ros__parameters"][
    "joint_velocity_limits"
]
if len(velocity_limits) != len(expected_joint_names):
    raise RuntimeError("Staubli streaming velocity limits are incomplete")

for path in (
    driver_launch,
    demo_share / "launch" / "room_315_staubli_hardware.launch.py",
):
    if "staubli_tx2_60l_moveit_config" in path.read_text():
        raise RuntimeError(f"MoveIt configuration reference remains in {path}")

executables = {
    "industrial_robot_client": (
        "joint_trajectory_action",
        "motion_streaming_interface",
    ),
    "staubli_val3_driver": (
        "staubli_io_interface",
        "staubli_robot_state",
        "staubli_system_interface",
    ),
}
for package, names in executables.items():
    executable_dir = Path(get_package_prefix(package)) / "lib" / package
    for name in names:
        path = executable_dir / name
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f"installed ROS executable is missing: {path}")

print("Staubli MoveIt-free stack: launches, joint config, xacro, and executables resolved")
PY

python3 - <<'PY'
import sys

import coal
sys.modules.setdefault("hppfcl", coal)
import pyhpp_toppra
import pyhpp_viser
import trimesh
import viser

from pyhpp_toppra import Toppra

print(f"Viser planner view: {viser.__version__}")
PY

test_paths=(
  "$mfja_root/mfja_staubli_demos/test/test_hpp_repos.py"
  "$mfja_root/mfja_staubli_demos/test/test_trajectory_export.py"
)
if [[ -n "$hpp_exec_source" && -d "$hpp_exec_source/tests" ]]; then
  test_paths=("$hpp_exec_source/tests" "${test_paths[@]}")
fi
python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  "${test_paths[@]}"

ROS_DOMAIN_ID=229 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  timeout 20s python3 "$script_dir/room315_check_joint_state.py"

ROS_DOMAIN_ID=229 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 "$script_dir/room315_export_staubli_line.py" \
  --line 0 0 0.10 \
  --duration 5 \
  --samples 20 \
  | python3 -c 'import json, sys; payload = json.load(sys.stdin); assert len(payload["points"]) == 21; print("Offline HPP export: 21 six-joint points")'

ros2 run mfja_staubli_manipulation_demos \
  room315_pick_place.sh --help >/dev/null
echo "Installed Room 315 resource lookup: OK"
