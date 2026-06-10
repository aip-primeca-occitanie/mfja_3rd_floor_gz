#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
KUKA_MODEL_PATH = (
    REPO_ROOT
    / 'mfja_3rd_floor_description'
    / 'models'
    / 'kuka_kr6r900sixx'
    / 'model.sdf'
)
EXPECTED_INITIAL_POSITIONS = (
    ('joint_a1', 0.0),
    ('joint_a2', -1.57079632679),
    ('joint_a3', 1.91986217719),
    ('joint_a4', 0.0),
    ('joint_a5', -0.03490658504),
    ('joint_a6', 0.0),
)


def test_kuka_initial_joint_positions_match_debug_home_pose():
    root = ET.parse(KUKA_MODEL_PATH).getroot()
    controller = root.find(
        ".//plugin[@name='gz::sim::systems::JointTrajectoryController']"
    )

    assert controller is not None

    joint_names = [node.text.strip() for node in controller.findall('joint_name')]
    initial_positions = [
        float(node.text.strip()) for node in controller.findall('initial_position')
    ]

    assert joint_names == [joint for joint, _position in EXPECTED_INITIAL_POSITIONS]
    assert initial_positions == [
        position for _joint, position in EXPECTED_INITIAL_POSITIONS
    ]
