#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'robot_joint_command.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('robot_joint_command', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_degree_input_converts_only_angular_joints_for_tiago():
    command = _load_module()
    profile = command.resolve_profile('tiago1')

    converted = command.converted_positions(
        profile,
        [0.10, 90.0, -45.0, 0.0, 180.0, 30.0, -30.0, 15.0, 20.0, -20.0],
        'deg',
    )

    assert converted[0] == pytest.approx(0.10)
    assert converted[1:] == pytest.approx(
        [
            math.pi / 2.0,
            -math.pi / 4.0,
            0.0,
            math.pi,
            math.pi / 6.0,
            -math.pi / 6.0,
            math.pi / 12.0,
            math.radians(20.0),
            math.radians(-20.0),
        ]
    )


def test_radian_input_is_passed_through_for_industrial_arm():
    command = _load_module()
    profile = command.resolve_profile('hc10dt')

    positions = [-0.2, -0.5, 0.8, 0.0, 0.5, -0.2]

    assert command.converted_positions(profile, positions, 'rad') == positions


def test_position_count_validation_names_expected_joints():
    command = _load_module()
    profile = command.resolve_profile('kuka')

    with pytest.raises(ValueError, match='expects 6 position values'):
        command.converted_positions(profile, [0.0, 1.0], 'rad')


def test_profile_topics_match_bridge_names():
    command = _load_module()
    profile = command.resolve_profile('kuka')

    assert profile.topic == '/kuka1/joint_trajectory'
    assert profile.joint_state_topic == '/kuka1/joint_states'


def test_unit_aliases_and_shortcuts():
    command = _load_module()

    assert command.normalize_unit('degrees') == 'deg'
    assert command.normalize_unit('radian') == 'rad'
    assert command.normalize_unit(None, degrees=True) == 'deg'
    with pytest.raises(ValueError, match='only one'):
        command.normalize_unit('rad', degrees=True, radians=True)
    with pytest.raises(ValueError, match='do not combine'):
        command.normalize_unit('deg', radians=True)


def test_parser_defaults_publish_a_short_burst():
    command = _load_module()

    args = command.build_parser().parse_args(
        ['staubli', '--positions', '0', '0', '0', '0', '0', '0']
    )

    assert args.times == command.DEFAULT_PUBLISH_TIMES
    assert args.rate == pytest.approx(command.DEFAULT_PUBLISH_RATE_HZ)
    assert args.ready_timeout == pytest.approx(command.DEFAULT_READY_TIMEOUT_SEC)


def test_trajectory_preview_includes_publish_burst():
    command = _load_module()
    profile = command.resolve_profile('hc10')

    preview = command.trajectory_preview(
        profile,
        [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        3.0,
        times=command.DEFAULT_PUBLISH_TIMES,
        rate_hz=command.DEFAULT_PUBLISH_RATE_HZ,
    )

    assert 'Topic: /yaskawa_hc10_1/joint_trajectory' in preview
    assert 'Joint state topic: /yaskawa_hc10_1/joint_states' in preview
    assert 'Publish burst: times=10, rate_hz=10' in preview
