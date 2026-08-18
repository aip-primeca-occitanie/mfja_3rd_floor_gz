#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION_PATH = REPO_ROOT / 'mfja_3rd_floor_description'
INDUSTRIAL_ROBOT_MODELS = (
    'kuka_kr6r900sixx',
    'staubli_tx2_60l',
    'yaskawa_hc10',
    'yaskawa_hc10dt',
)
SMOOTH_CONTROLLER_NAME = (
    'mfja::sim::systems::SmoothJointTrajectoryController'
)
SMOOTH_CONTROLLER_FILENAME = (
    'mfja-smooth-joint-trajectory-controller-system'
)
EXPECTED_ARM_JOINTS = {
    'kuka_kr6r900sixx': (
        'joint_a1',
        'joint_a2',
        'joint_a3',
        'joint_a4',
        'joint_a5',
        'joint_a6',
    ),
    'staubli_tx2_60l': (
        'joint_1',
        'joint_2',
        'joint_3',
        'joint_4',
        'joint_5',
        'joint_6',
    ),
    'yaskawa_hc10': (
        'joint_1_s',
        'joint_2_l',
        'joint_3_u',
        'joint_4_r',
        'joint_5_b',
        'joint_6_t',
    ),
    'yaskawa_hc10dt': (
        'joint_1_s',
        'joint_2_l',
        'joint_3_u',
        'joint_4_r',
        'joint_5_b',
        'joint_6_t',
    ),
}
EXPECTED_DEFAULT_DURATIONS = {
    'kuka_kr6r900sixx': 4.0,
    'staubli_tx2_60l': 3.0,
    'yaskawa_hc10': 3.0,
    'yaskawa_hc10dt': 3.0,
}


def test_industrial_robot_assets_are_collision_free_for_kinematic_debug():
    for model_name in INDUSTRIAL_ROBOT_MODELS:
        sdf_path = DESCRIPTION_PATH / 'models' / model_name / 'model.sdf'
        urdf_path = DESCRIPTION_PATH / 'urdf' / f'{model_name}.urdf'
        sdf_root = ET.parse(sdf_path).getroot()
        urdf_root = ET.parse(urdf_path).getroot()

        assert sdf_root.findall('.//collision') == [], model_name
        assert urdf_root.findall('.//collision') == [], model_name
        assert sdf_root.findall('.//visual'), model_name
        assert urdf_root.findall('.//visual'), model_name
        assert sdf_root.findall('.//joint'), model_name
        assert urdf_root.findall('.//joint'), model_name


def test_industrial_arms_use_deterministic_smooth_position_control():
    for model_name in INDUSTRIAL_ROBOT_MODELS:
        sdf_path = DESCRIPTION_PATH / 'models' / model_name / 'model.sdf'
        root = ET.parse(sdf_path).getroot()

        controllers = root.findall(
            f".//plugin[@name='{SMOOTH_CONTROLLER_NAME}']"
        )
        assert len(controllers) == 1, model_name
        controller = controllers[0]
        assert controller.get('filename') == SMOOTH_CONTROLLER_FILENAME

        # A force PID cannot guarantee an exact visual pose under gravity.
        # These collision-free debug arms therefore use the deterministic
        # position-reset controller exclusively.
        assert not root.findall(
            ".//plugin[@name='gz::sim::systems::JointTrajectoryController']"
        ), model_name
        controller_tags = [child.tag for child in controller]
        assert not any(
            tag.startswith('position_') or tag.startswith('velocity_')
            for tag in controller_tags
        ), model_name

        joint_names = tuple(
            node.text.strip() for node in controller.findall('joint_name')
        )
        initial_positions = tuple(
            float(node.text) for node in controller.findall('initial_position')
        )
        assert joint_names == EXPECTED_ARM_JOINTS[model_name]
        assert len(initial_positions) == len(joint_names)

        # Repeated parameters are positional in Gazebo plugin SDF. Keep every
        # initial position adjacent to the joint it configures.
        expected_pair_tags = ['joint_name', 'initial_position'] * len(joint_names)
        assert controller_tags[:-2] == expected_pair_tags, model_name
        assert controller_tags[-2:] == [
            'default_duration_sec',
            'max_velocity_scale',
        ], model_name

        for joint_name, initial_position in zip(
            joint_names, initial_positions, strict=True
        ):
            joint = root.find(f".//joint[@name='{joint_name}']")
            assert joint is not None, (model_name, joint_name)
            assert joint.get('type') == 'revolute', (model_name, joint_name)
            lower = float(joint.findtext('axis/limit/lower'))
            upper = float(joint.findtext('axis/limit/upper'))
            assert lower <= initial_position <= upper, (model_name, joint_name)

        assert float(controller.findtext('default_duration_sec')) == (
            EXPECTED_DEFAULT_DURATIONS[model_name]
        )
        velocity_scale = float(controller.findtext('max_velocity_scale'))
        assert 0.0 < velocity_scale <= 1.0
