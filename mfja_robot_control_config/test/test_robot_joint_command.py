#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

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
        [0.10, 90.0, -45.0, 0.0, 90.0, 30.0, -30.0, 15.0, 20.0, -20.0],
        'deg',
    )

    assert converted[0] == pytest.approx(0.10)
    assert converted[1:] == pytest.approx(
        [
            math.pi / 2.0,
            -math.pi / 4.0,
            0.0,
            math.pi / 2.0,
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


def test_parser_defaults_to_one_dense_trajectory_message():
    command = _load_module()

    args = command.build_parser().parse_args(
        ['staubli', '--positions', '0', '0', '0', '0', '0', '0']
    )

    assert args.times == 1
    assert args.rate == pytest.approx(command.DEFAULT_TRAJECTORY_RATE_HZ)
    assert args.ready_timeout == pytest.approx(command.DEFAULT_READY_TIMEOUT_SEC)


def test_trajectory_preview_describes_dense_single_publication():
    command = _load_module()
    profile = command.resolve_profile('hc10')

    preview = command.trajectory_preview(
        profile,
        [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        3.0,
        rate_hz=command.DEFAULT_TRAJECTORY_RATE_HZ,
    )

    assert 'Topic: /yaskawa_hc10_1/joint_trajectory' in preview
    assert 'Joint state topic: /yaskawa_hc10_1/joint_states' in preview
    assert 'Smooth trajectory: quintic, points=301, rate_hz=100' in preview
    assert 'Start positions: read live by joint name from joint_states' in preview
    assert 'Publication count: 1' in preview


def test_targets_must_be_finite_and_inside_named_joint_limits():
    command = _load_module()
    kuka = command.resolve_profile('kuka')

    with pytest.raises(ValueError, match='joint_a1 must be finite'):
        command.converted_positions(kuka, [math.nan, 0, 0, 0, 0, 0], 'rad')
    with pytest.raises(ValueError, match=r'joint_a1.*allowed range'):
        command.converted_positions(kuka, [3.1, 0, 0, 0, 0, 0], 'rad')

    tiago_base = command.resolve_profile('tiago_base')
    with pytest.raises(ValueError, match=r'torso_lift_joint.*\[0, 0.35\] m'):
        command.converted_positions(tiago_base, [0.36], 'deg')


def test_joint_state_positions_are_selected_by_name_not_array_order():
    command = _load_module()
    profile = command.resolve_profile('kuka')
    expected = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    names = ['uncontrolled_gripper_joint', *reversed(profile.joint_names)]
    values = [0.01, *reversed(expected)]

    assert command.positions_from_joint_state(profile, names, values) == pytest.approx(
        expected
    )


def test_joint_state_rejects_missing_duplicate_and_nonfinite_values():
    command = _load_module()
    profile = command.resolve_profile('staubli')

    with pytest.raises(ValueError, match='missing joints'):
        command.positions_from_joint_state(profile, profile.joint_names[:-1], [0.0] * 5)
    with pytest.raises(ValueError, match='duplicate names'):
        command.positions_from_joint_state(
            profile,
            [*profile.joint_names, profile.joint_names[0]],
            [0.0] * 7,
        )
    with pytest.raises(ValueError, match='joint_2 must be finite'):
        command.positions_from_joint_state(
            profile,
            profile.joint_names,
            [0.0, math.inf, 0.0, 0.0, 0.0, 0.0],
        )


def test_quintic_trajectory_is_dense_smooth_and_exact_at_boundaries():
    command = _load_module()
    samples = command.sampled_quintic_trajectory([0.0, 1.0], [1.0, -1.0], 1.0, 100.0)

    assert len(samples) == 101
    assert samples[0].time_from_start_sec == 0.0
    assert samples[-1].time_from_start_sec == 1.0
    assert samples[0].positions == (0.0, 1.0)
    assert samples[-1].positions == (1.0, -1.0)
    assert samples[0].velocities == pytest.approx([0.0, 0.0])
    assert samples[-1].velocities == pytest.approx([0.0, 0.0])
    assert samples[0].accelerations == pytest.approx([0.0, 0.0])
    assert samples[-1].accelerations == pytest.approx([0.0, 0.0])
    assert samples[50].positions == pytest.approx([0.5, 0.0])
    assert samples[50].velocities == pytest.approx([1.875, -3.75])
    assert all(
        earlier.positions[0] <= later.positions[0]
        for earlier, later in zip(samples, samples[1:])
    )
    assert max(
        later.time_from_start_sec - earlier.time_from_start_sec
        for earlier, later in zip(samples, samples[1:])
    ) == pytest.approx(0.01)


def test_trajectory_rate_is_limited_to_controller_safe_dense_range():
    command = _load_module()

    with pytest.raises(ValueError, match='between 100 and 200 Hz'):
        command.sampled_quintic_trajectory([0.0], [1.0], 1.0, 99.0)
    with pytest.raises(ValueError, match='between 100 and 200 Hz'):
        command.sampled_quintic_trajectory([0.0], [1.0], 1.0, 201.0)
    with pytest.raises(ValueError, match='duration must be at least 0.005'):
        command.sampled_quintic_trajectory([0.0], [1.0], 0.004, 100.0)

    short_samples = command.sampled_quintic_trajectory([0.0], [1.0], 0.006, 200.0)
    assert len(short_samples) == 2
    assert short_samples[-1].time_from_start_sec == pytest.approx(0.006)


def test_dense_ros_message_has_strict_timestamps_and_zero_final_velocity():
    command = _load_module()
    profile = command.resolve_profile('kuka')
    start = [0.0, -1.57, 1.92, 0.0, -0.03, 0.0]
    target = [0.2, -1.0, 1.5, 0.1, 0.2, -0.1]

    message = command.build_trajectory_message(profile, start, target, 1.0, 100.0)

    assert message.joint_names == list(profile.joint_names)
    assert len(message.points) == 101
    assert message.points[0].positions == pytest.approx(start)
    assert message.points[-1].positions == pytest.approx(target)
    assert message.points[-1].velocities == pytest.approx([0.0] * 6)
    assert message.points[-1].accelerations == pytest.approx([0.0] * 6)
    timestamps_ns = [
        point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
        for point in message.points
    ]
    assert timestamps_ns[0] == 0
    assert timestamps_ns[-1] == 1_000_000_000
    assert all(left < right for left, right in zip(timestamps_ns, timestamps_ns[1:]))


def test_all_profile_limits_match_their_model_sdf():
    command = _load_module()
    model_directories = {
        'kuka_kr6r900sixx': 'kuka_kr6r900sixx',
        'staubli_tx2_60l': 'staubli_tx2_60l',
        'yaskawa_hc10': 'yaskawa_hc10',
        'yaskawa_hc10dt': 'yaskawa_hc10dt',
        'tiago_with_arm': 'tiago',
        'tiago_base': 'tiago_base',
    }

    for profile in command.PROFILES:
        sdf_path = (
            REPO_ROOT
            / 'mfja_3rd_floor_description'
            / 'models'
            / model_directories[profile.model_name]
            / 'model.sdf'
        )
        model = ET.parse(sdf_path).getroot()
        actual_limits = []
        for joint_name in profile.joint_names:
            joint = model.find(f".//joint[@name='{joint_name}']")
            assert joint is not None, (profile.robot_name, joint_name)
            lower = joint.findtext('./axis/limit/lower')
            upper = joint.findtext('./axis/limit/upper')
            assert lower is not None and upper is not None
            actual_limits.append((float(lower), float(upper)))
        for actual, configured in zip(actual_limits, profile.joint_limits, strict=True):
            assert actual == pytest.approx(configured)
