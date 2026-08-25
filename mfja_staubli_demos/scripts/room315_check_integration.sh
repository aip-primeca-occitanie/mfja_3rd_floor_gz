#!/usr/bin/env bash
# Focused, read-only validation of the installed HPP/MFJA environment.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd -P)
mfja_root=$(cd -- "$script_dir/../.." && pwd -P)
hpp_exec_source=${HPP_EXEC_SOURCE:-$(dirname "$mfja_root")/nix-hpp/src/hpp-exec}

python3 "$script_dir/room315_check_environment.py"
python3 "$script_dir/room315_read_configuration.py" --help >/dev/null

required_staubli_packages=(
  industrial_msgs
  industrial_robot_client
  industrial_utils
  simple_message
  staubli_msgs
  staubli_tx2_60l_description
  staubli_tx2_60l_moveit_config
  staubli_val3_driver
  urdf_extention
)
for package in "${required_staubli_packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null
done

python3 - <<'PY'
import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_path
from launch import LaunchDescription


def generate_launch_description(path: Path) -> LaunchDescription:
    namespace = {"__file__": str(path), "__name__": f"static_check_{path.stem}"}
    exec(compile(path.read_text(), str(path), "exec"), namespace)
    description = namespace["generate_launch_description"]()
    if not isinstance(description, LaunchDescription) or not description.entities:
        raise RuntimeError(f"invalid launch description: {path}")
    return description


moveit_share = get_package_share_path("staubli_tx2_60l_moveit_config")
driver_share = get_package_share_path("staubli_val3_driver")
moveit_launch = moveit_share / "launch" / "staubli_tx2_60l_planning_execution_real.launch.py"
driver_launch = driver_share / "launch" / "robot_interface_streaming.launch.py"
for path in (
    moveit_launch,
    driver_launch,
    driver_share / "launch" / "robot_state.launch.py",
    driver_share / "launch" / "motion_streaming_interface.launch.py",
    driver_share / "launch" / "io_interface.launch.py",
    driver_share / "launch" / "system_interface.launch.py",
):
    if not path.is_file():
        raise RuntimeError(f"installed launch file is missing: {path}")

generate_launch_description(moveit_launch)
generate_launch_description(driver_launch)

robot_xml = xacro.process_file(
    str(moveit_share / "config" / "staubli_tx2_60l.urdf.xacro")
).toxml()
for joint_name in ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"):
    if f'name="{joint_name}"' not in robot_xml:
        raise RuntimeError(f"MoveIt robot description is missing {joint_name}")

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

print("Staubli real-stack static check: launches, xacro, and executables resolved")
PY

test_paths=("$mfja_root/mfja_staubli_demos/test/test_trajectory_export.py")
if [[ -d "$hpp_exec_source/tests" ]]; then
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
  room315_moving_shuttle_demo.sh --help >/dev/null
echo "Installed Room 315 resource lookup: OK"
