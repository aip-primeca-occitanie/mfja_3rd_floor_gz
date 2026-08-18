#!/usr/bin/env python3

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION_PATH = REPO_ROOT / 'mfja_3rd_floor_description'
JAW_JOINTS = ('gripper_left_jaw_joint', 'gripper_right_jaw_joint')
OPEN_POSITIONS = {
    'kuka_kr6r900sixx': 0.030,
    'staubli_tx2_60l': 0.0025,
    'yaskawa_hc10': 0.040,
    'yaskawa_hc10dt': 0.010,
}
MONOLITHIC_GRIPPER_ASSETS = {
    'kuka_kr6r900sixx': 'schunk_kgg_140_60_011l5_mss_22_01.stl',
    'staubli_tx2_60l': 'schunk_edited.stl',
    'yaskawa_hc10': 'geh6040il_03_b01geh6000il.stl',
    'yaskawa_hc10dt': 'lwr50l_03_00001_a_000.stl',
}
KUKA_JAW_MESH_RELATIVE_PATH = Path(
    'models/kuka_kr6r900sixx/meshes/gripper/jaw_kuka.stl'
)
KUKA_JAW_MESH_SHA256 = (
    '4697308a6a668573f48dd960811bb1891464efb86b199f3baf3823eb33fa7172'
)


def _axis_vector(text: str) -> tuple[float, float, float]:
    values = tuple(float(value) for value in text.split())
    assert len(values) == 3
    return values


def _opposites(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> bool:
    return all(a == pytest.approx(-b) for a, b in zip(first, second, strict=True))


def _assert_vector(text: str, expected: tuple[float, ...]) -> None:
    values = tuple(float(value) for value in text.split())
    assert values == pytest.approx(expected)


def test_kuka_jaws_reuse_unmodified_source_mesh():
    mesh_path = DESCRIPTION_PATH / KUKA_JAW_MESH_RELATIVE_PATH
    assert mesh_path.is_file()
    assert mesh_path.stat().st_size > 0
    assert hashlib.sha256(mesh_path.read_bytes()).hexdigest() == KUKA_JAW_MESH_SHA256

    sdf_path = DESCRIPTION_PATH / 'models/kuka_kr6r900sixx/model.sdf'
    sdf_text = sdf_path.read_text(encoding='utf-8')
    assert sdf_text.count('jaw_kuka.stl') == 2
    sdf_root = ET.fromstring(sdf_text)
    sdf_uri = 'model://kuka_kr6r900sixx/meshes/gripper/jaw_kuka.stl'
    sdf_poses = {
        'gripper_left_jaw': (
            -0.047,
            0.015,
            0.021,
            -1.57079632679,
            0.0,
            3.14159265359,
        ),
        'gripper_right_jaw': (
            0.047,
            -0.015,
            0.021,
            -1.57079632679,
            0.0,
            0.0,
        ),
    }
    for link_name, expected_pose in sdf_poses.items():
        link = sdf_root.find(f".//link[@name='{link_name}']")
        assert link is not None
        visuals = link.findall('visual')
        assert len(visuals) == 1
        assert not link.findall('.//box')

        visual = visuals[0]
        assert visual.findtext('geometry/mesh/uri') == sdf_uri
        _assert_vector(visual.findtext('geometry/mesh/scale'), (1.0, 1.0, 1.0))
        _assert_vector(visual.findtext('pose'), expected_pose)

    urdf_path = DESCRIPTION_PATH / 'urdf/kuka_kr6r900sixx.urdf'
    urdf_text = urdf_path.read_text(encoding='utf-8')
    assert urdf_text.count('jaw_kuka.stl') == 2
    urdf_root = ET.fromstring(urdf_text)
    urdf_filename = (
        'package://mfja_3rd_floor_description/'
        'models/kuka_kr6r900sixx/meshes/gripper/jaw_kuka.stl'
    )
    urdf_origins = {
        'gripper_left_jaw': (
            (-0.047, 0.015, 0.021),
            (-1.57079632679, 0.0, 3.14159265359),
        ),
        'gripper_right_jaw': (
            (0.047, -0.015, 0.021),
            (-1.57079632679, 0.0, 0.0),
        ),
    }
    for link_name, (expected_xyz, expected_rpy) in urdf_origins.items():
        link = urdf_root.find(f".//link[@name='{link_name}']")
        assert link is not None
        visuals = link.findall('visual')
        assert len(visuals) == 1
        assert not link.findall('.//box')

        visual = visuals[0]
        mesh = visual.find('geometry/mesh')
        assert mesh is not None
        assert mesh.get('filename') == urdf_filename
        _assert_vector(mesh.get('scale'), (1.0, 1.0, 1.0))

        origin = visual.find('origin')
        assert origin is not None
        _assert_vector(origin.get('xyz'), expected_xyz)
        _assert_vector(origin.get('rpy'), expected_rpy)


@pytest.mark.parametrize(('model_name', 'open_position'), OPEN_POSITIONS.items())
def test_sdf_exposes_symmetric_commanded_jaws(model_name, open_position):
    sdf_path = DESCRIPTION_PATH / 'models' / model_name / 'model.sdf'
    sdf_text = sdf_path.read_text(encoding='utf-8')
    root = ET.fromstring(sdf_text)

    # The source assemblies contain fixed jaw geometry. Runtime visuals must
    # use local body / jaw geometry so a static jaw cannot remain underneath
    # the articulated links.
    assert MONOLITHIC_GRIPPER_ASSETS[model_name] not in sdf_text

    for link_name in ('gripper_left_jaw', 'gripper_right_jaw'):
        link = root.find(f".//link[@name='{link_name}']")
        assert link is not None, (model_name, link_name)
        assert link.find('visual') is not None, (model_name, link_name)

    joints = {
        name: root.find(f".//joint[@name='{name}']")
        for name in JAW_JOINTS
    }
    assert all(joint is not None for joint in joints.values()), model_name
    assert all(joint.get('type') == 'prismatic' for joint in joints.values())
    assert all(joint.findtext('parent') == 'gripper' for joint in joints.values())
    assert joints[JAW_JOINTS[0]].findtext('child') == 'gripper_left_jaw'
    assert joints[JAW_JOINTS[1]].findtext('child') == 'gripper_right_jaw'

    axes = [
        _axis_vector(joints[name].findtext('axis/xyz'))
        for name in JAW_JOINTS
    ]
    assert _opposites(*axes), model_name
    for joint in joints.values():
        assert float(joint.findtext('axis/limit/lower')) == pytest.approx(0.0)
        assert float(joint.findtext('axis/limit/upper')) == pytest.approx(open_position)
        assert float(joint.findtext('axis/limit/velocity')) > 0.0
        assert float(joint.findtext('axis/limit/effort')) > 0.0

    state_publisher = root.find(
        ".//plugin[@name='gz::sim::systems::JointStatePublisher']"
    )
    assert state_publisher is not None
    state_joints = [node.text.strip() for node in state_publisher.findall('joint_name')]
    assert all(name in state_joints for name in JAW_JOINTS)

    # One local controller consumes one scalar command and applies the same
    # position to both opposite-axis joints. This prevents either jaw from
    # being left behind by independent physics controllers.
    assert not root.findall(
        ".//plugin[@name='gz::sim::systems::JointPositionController']"
    )
    controllers = root.findall(
        ".//plugin[@name='mfja::sim::systems::SymmetricGripperController']"
    )
    assert len(controllers) == 1, model_name
    controller = controllers[0]
    assert controller.get('filename') == (
        'mfja-symmetric-gripper-controller-system'
    )
    assert controller.findtext('left_joint_name') == JAW_JOINTS[0]
    assert controller.findtext('right_joint_name') == JAW_JOINTS[1]
    assert controller.findtext('sub_topic') == 'gripper/position_command'
    assert float(controller.findtext('min_position')) == pytest.approx(0.0)
    assert float(controller.findtext('max_position')) == pytest.approx(open_position)
    assert float(controller.findtext('initial_position')) == pytest.approx(0.0)
    joint_velocities = {
        float(joint.findtext('axis/limit/velocity'))
        for joint in joints.values()
    }
    assert len(joint_velocities) == 1
    assert float(controller.findtext('max_velocity')) == pytest.approx(
        joint_velocities.pop()
    )

    arm_controller = root.find(
        ".//plugin[@name='mfja::sim::systems::SmoothJointTrajectoryController']"
    )
    assert arm_controller is not None
    arm_joint_names = {
        node.text.strip() for node in arm_controller.findall('joint_name')
    }
    assert not arm_joint_names.intersection(JAW_JOINTS)


@pytest.mark.parametrize(('model_name', 'open_position'), OPEN_POSITIONS.items())
def test_urdf_matches_sdf_gripper_kinematics(model_name, open_position):
    urdf_path = DESCRIPTION_PATH / 'urdf' / f'{model_name}.urdf'
    urdf_text = urdf_path.read_text(encoding='utf-8')
    root = ET.fromstring(urdf_text)

    assert MONOLITHIC_GRIPPER_ASSETS[model_name] not in urdf_text

    assert root.find(".//link[@name='gripper_left_jaw']") is not None
    assert root.find(".//link[@name='gripper_right_jaw']") is not None

    joints = [root.find(f".//joint[@name='{name}']") for name in JAW_JOINTS]
    assert all(joint is not None for joint in joints), model_name
    assert all(joint.get('type') == 'prismatic' for joint in joints)
    axes = [_axis_vector(joint.find('axis').get('xyz')) for joint in joints]
    assert _opposites(*axes), model_name

    for joint in joints:
        assert joint.find('parent').get('link') == 'gripper'
        limit = joint.find('limit')
        assert float(limit.get('lower')) == pytest.approx(0.0)
        assert float(limit.get('upper')) == pytest.approx(open_position)
        assert float(limit.get('velocity')) > 0.0
        assert float(limit.get('effort')) > 0.0
