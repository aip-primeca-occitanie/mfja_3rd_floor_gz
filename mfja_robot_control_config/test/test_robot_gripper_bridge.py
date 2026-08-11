#!/usr/bin/env python3

import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MULTI_LAUNCH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'launch'
    / 'multi_robot_sim.launch.py'
)
ISOLATED_LAUNCH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'launch'
    / 'isolated_industrial_robot.launch.py'
)


def _load_launch_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _read_bridge(path: str) -> list[dict[str, str]]:
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _gripper_bridge(config: list[dict[str, str]]) -> dict[str, str]:
    matches = [
        item
        for item in config
        if item['ros_topic_name'].endswith('/gripper/position_command')
    ]
    assert len(matches) == 1
    return matches[0]


def test_multi_robot_bridge_adds_model_scoped_gripper_command(tmp_path, monkeypatch):
    launch = _load_launch_module(MULTI_LAUNCH)
    monkeypatch.setattr(launch.tempfile, 'gettempdir', lambda: str(tmp_path))

    bridge_path = launch._make_bridge_yaml(
        'kuka1',
        'room_315_only',
        'kuka_kr6r900sixx',
    )
    command = _gripper_bridge(_read_bridge(bridge_path))

    assert command == {
        'ros_topic_name': '/kuka1/gripper/position_command',
        'gz_topic_name': '/model/kuka1/gripper/position_command',
        'ros_type_name': 'std_msgs/msg/Float64',
        'gz_type_name': 'gz.msgs.Double',
        'direction': 'ROS_TO_GZ',
    }


def test_multi_robot_bridge_does_not_advertise_gripper_for_mobile_model(
    tmp_path,
    monkeypatch,
):
    launch = _load_launch_module(MULTI_LAUNCH)
    monkeypatch.setattr(launch.tempfile, 'gettempdir', lambda: str(tmp_path))

    bridge_path = launch._make_bridge_yaml('tiago1', 'room_315_only', 'tiago')
    bridge_config = _read_bridge(bridge_path)

    assert not any(
        item['ros_topic_name'].endswith('/gripper/position_command')
        for item in bridge_config
    )


def test_isolated_robot_bridge_adds_model_scoped_gripper_command(
    tmp_path,
    monkeypatch,
):
    launch = _load_launch_module(ISOLATED_LAUNCH)
    monkeypatch.setattr(launch.tempfile, 'gettempdir', lambda: str(tmp_path))

    bridge_path = launch._make_bridge_yaml('staubli1', 'isolated_industrial_robot')
    command = _gripper_bridge(_read_bridge(bridge_path))

    assert command['ros_topic_name'] == '/staubli1/gripper/position_command'
    assert command['gz_topic_name'] == '/model/staubli1/gripper/position_command'
    assert command['ros_type_name'] == 'std_msgs/msg/Float64'
    assert command['gz_type_name'] == 'gz.msgs.Double'
    assert command['direction'] == 'ROS_TO_GZ'
