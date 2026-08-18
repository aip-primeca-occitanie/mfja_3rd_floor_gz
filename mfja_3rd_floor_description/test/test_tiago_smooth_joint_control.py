#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_DIRECTORY = REPO_ROOT / 'mfja_3rd_floor_description' / 'models'
CONTROLLER_NAME = 'mfja::sim::systems::SmoothJointTrajectoryController'
CONTROLLER_FILENAME = 'mfja-smooth-joint-trajectory-controller-system'
LEGACY_CONTROLLER_NAME = 'gz::sim::systems::JointTrajectoryController'

EXPECTED_CONTROLLED_JOINTS = {
    'tiago': (
        ('torso_lift_joint', 0.05),
        ('arm_1_joint', 0.0),
        ('arm_2_joint', 0.0),
        ('arm_3_joint', 0.0),
        ('arm_4_joint', 0.0),
        ('arm_5_joint', 0.0),
        ('arm_6_joint', 0.0),
        ('arm_7_joint', 0.0),
        ('head_1_joint', 0.0),
        ('head_2_joint', 0.0),
    ),
    'tiago_base': (
        ('torso_lift_joint', 0.05),
    ),
}

PID_TAGS = (
    'position_p_gain',
    'position_i_gain',
    'position_d_gain',
    'position_i_min',
    'position_i_max',
    'position_cmd_min',
    'position_cmd_max',
)


def _model_and_controller(model_name):
    root = ET.parse(MODEL_DIRECTORY / model_name / 'model.sdf').getroot()
    model = root.find('model')
    assert model is not None

    controllers = model.findall(f"plugin[@name='{CONTROLLER_NAME}']")
    assert len(controllers) == 1
    return model, controllers[0]


@pytest.mark.parametrize('model_name', EXPECTED_CONTROLLED_JOINTS)
def test_tiago_uses_deterministic_smooth_joint_controller(model_name):
    model, controller = _model_and_controller(model_name)

    assert controller.get('filename') == CONTROLLER_FILENAME
    assert model.find(f"plugin[@name='{LEGACY_CONTROLLER_NAME}']") is None

    joint_names = [node.text.strip() for node in controller.findall('joint_name')]
    initial_positions = [
        float(node.text.strip()) for node in controller.findall('initial_position')
    ]
    expected = EXPECTED_CONTROLLED_JOINTS[model_name]

    assert joint_names == [joint_name for joint_name, _position in expected]
    assert initial_positions == [position for _joint_name, position in expected]


@pytest.mark.parametrize('model_name', EXPECTED_CONTROLLED_JOINTS)
def test_tiago_smooth_controller_has_no_force_pid_configuration(model_name):
    _model, controller = _model_and_controller(model_name)

    for tag in PID_TAGS:
        assert controller.find(tag) is None, tag


@pytest.mark.parametrize('model_name', EXPECTED_CONTROLLED_JOINTS)
def test_tiago_controlled_initial_positions_respect_joint_limits(model_name):
    model, _controller = _model_and_controller(model_name)
    joints = {joint.get('name'): joint for joint in model.findall('joint')}

    for joint_name, initial_position in EXPECTED_CONTROLLED_JOINTS[model_name]:
        joint = joints[joint_name]
        lower = float(joint.findtext('axis/limit/lower'))
        upper = float(joint.findtext('axis/limit/upper'))
        assert lower <= initial_position <= upper, joint_name


@pytest.mark.parametrize('model_name', EXPECTED_CONTROLLED_JOINTS)
def test_tiago_smooth_controller_fallback_motion_is_bounded(model_name):
    _model, controller = _model_and_controller(model_name)

    assert float(controller.findtext('default_duration_sec')) >= 3.0
    velocity_scale = float(controller.findtext('max_velocity_scale'))
    assert 0.0 < velocity_scale <= 0.5
