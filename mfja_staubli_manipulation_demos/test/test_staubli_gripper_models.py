import ast
from pathlib import Path
from xml.etree import ElementTree

import pytest


REPOSITORY = Path(__file__).parents[2]
DESCRIPTION = REPOSITORY / "mfja_3rd_floor_description"
DEMO = REPOSITORY / "mfja_staubli_manipulation_demos"
ROBOT_URDF = DESCRIPTION / "urdf" / "staubli_tx2_60l.urdf"
ROBOT_SDF = DESCRIPTION / "models" / "staubli_tx2_60l" / "model.sdf"
DEMO_SDF = DEMO / "models" / "staubli_tx2_60l_gripper" / "model.sdf"
PROBLEM = DEMO / "hpp" / "room315_problem.py"


def floats(text):
    return tuple(float(value) for value in text.split())


def assigned_literal(path, name):
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == name
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} is not assigned in {path}")


def urdf_joint_translation(root, name):
    return floats(root.find(f"./joint[@name='{name}']/origin").attrib["xyz"])


def sdf_joint_translation(root, name):
    return floats(root.find(f".//joint[@name='{name}']/pose").text)[:3]


def add_vectors(*vectors):
    return tuple(sum(values) for values in zip(*vectors))


def test_required_gripper_meshes_resolve():
    canonical = ElementTree.parse(ROBOT_URDF).getroot()
    articulated = ElementTree.parse(DEMO_SDF).getroot()
    uris = {
        element.text
        for element in articulated.findall(".//visual/geometry/mesh/uri")
        if "/meshes/gripper/" in element.text
    }

    assert canonical.find(
        "./link[@name='gripper']/visual/geometry/mesh"
    ).attrib["filename"].endswith("/meshes/gripper/schunk_edited.stl")
    assert {
        uri.rsplit("/", 1)[-1] for uri in uris
    } == {
        "schunk_pgn_plus_p_40_318448.stl",
        "staubli_custom_jaw.stl",
        "staubli_pneumatic_adapter.stl",
    }
    for uri in uris:
        path = (
            DESCRIPTION
            / "models"
            / "staubli_tx2_60l"
            / "meshes"
            / "gripper"
            / uri.rsplit("/", 1)[-1]
        )
        assert path.is_file()


def test_gripper_collision_envelope_exists_in_urdf_and_sdf():
    urdf = ElementTree.parse(ROBOT_URDF).getroot()
    sdf = ElementTree.parse(ROBOT_SDF).getroot()
    urdf_names = {
        collision.attrib["name"]
        for link_name in ("gripper_mount", "gripper")
        for collision in urdf.find(f"./link[@name='{link_name}']").findall(
            "collision"
        )
    }
    sdf_names = {
        collision.attrib["name"]
        for link_name in ("gripper_mount", "gripper")
        for collision in sdf.find(f".//link[@name='{link_name}']").findall(
            "collision"
        )
    }

    expected = {
        "rear_attachment",
        "gripper_body",
        "left_finger_sweep",
        "right_finger_sweep",
    }
    assert urdf_names == expected
    assert sdf_names == expected


def test_provisional_tcp_frame_is_consistent_across_models():
    urdf = ElementTree.parse(ROBOT_URDF).getroot()
    sdf = ElementTree.parse(ROBOT_SDF).getroot()
    demo_sdf = ElementTree.parse(DEMO_SDF).getroot()

    urdf_tcp = add_vectors(
        urdf_joint_translation(urdf, "tool0_gripper_mount_joint"),
        urdf_joint_translation(urdf, "gripper_mount_joint"),
        urdf_joint_translation(urdf, "gripper_tcp_joint"),
    )
    sdf_tcp = add_vectors(
        sdf_joint_translation(sdf, "tool0_gripper_mount_joint"),
        sdf_joint_translation(sdf, "gripper_mount_joint"),
        sdf_joint_translation(sdf, "gripper_tcp_joint"),
    )
    demo_tcp = add_vectors(
        sdf_joint_translation(demo_sdf, "tool0_gripper_base"),
        sdf_joint_translation(demo_sdf, "gripper_base_tcp"),
    )

    assert urdf_tcp == pytest.approx(sdf_tcp)
    assert urdf_tcp == pytest.approx(demo_tcp)


def test_simulated_jaw_commands_match_sdf_limits():
    sdf = ElementTree.parse(DEMO_SDF).getroot()
    joint_names = assigned_literal(PROBLEM, "GAZEBO_GRIPPER_JOINTS")
    open_positions = assigned_literal(PROBLEM, "GAZEBO_GRIPPER_OPEN_POSITIONS")
    close_positions = assigned_literal(
        PROBLEM, "GAZEBO_GRIPPER_CLOSE_POSITIONS"
    )

    for name, open_position, close_position in zip(
        joint_names, open_positions, close_positions
    ):
        limit = sdf.find(f".//joint[@name='{name}']/axis/limit")
        lower = float(limit.find("lower").text)
        upper = float(limit.find("upper").text)

        assert upper - lower == pytest.approx(0.0025)
        assert open_position == pytest.approx(upper)
        assert close_position == pytest.approx(lower)


def test_hpp_uses_canonical_robot_and_shared_cell_descriptions():
    assert assigned_literal(PROBLEM, "ROBOT_URDF") == (
        "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
    )
    assert assigned_literal(PROBLEM, "CELL_URDF") == (
        "package://mfja_3rd_floor_description/urdf/room315_cell.urdf"
    )
