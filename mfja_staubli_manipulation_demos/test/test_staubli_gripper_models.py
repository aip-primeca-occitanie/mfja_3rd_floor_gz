import ast
from pathlib import Path
from xml.etree import ElementTree

import pytest


REPOSITORY = Path(__file__).parents[2]
DESCRIPTION = REPOSITORY / "mfja_3rd_floor_description"
DEMO = REPOSITORY / "mfja_staubli_manipulation_demos"
ROBOT_URDF = DESCRIPTION / "urdf" / "staubli_tx2_60l.urdf"
CELL_URDF = DESCRIPTION / "urdf" / "room315_cell.urdf"
ROOM315_WORLD = DESCRIPTION / "worlds" / "room_315_only.world"
ROBOT_SDF = DESCRIPTION / "models" / "staubli_tx2_60l" / "model.sdf"
DEMO_SDF = DEMO / "models" / "staubli_tx2_60l_gripper" / "model.sdf"
TABLE_URDF = DEMO / "hpp" / "room315_staubli_table_drop_zone.urdf"
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
            value = (
                node.value.args[0]
                if isinstance(node.value, ast.Call)
                else node.value
            )
            return ast.literal_eval(value)
    raise AssertionError(f"{name} is not assigned in {path}")


def controller_initial_positions(plugin):
    positions = {}
    joint_name = None
    for element in plugin:
        if element.tag == "joint_name":
            joint_name = element.text
        elif element.tag == "initial_position" and joint_name is not None:
            positions[joint_name] = float(element.text)
    return positions


def urdf_joint_translation(root, name):
    return floats(root.find(f"./joint[@name='{name}']/origin").attrib["xyz"])


def urdf_joint_pose(root, name):
    origin = root.find(f"./joint[@name='{name}']/origin")
    return floats(origin.attrib["xyz"]) + floats(origin.attrib["rpy"])


def sdf_joint_translation(root, name):
    return floats(root.find(f".//joint[@name='{name}']/pose").text)[:3]


def sdf_include_pose(root, name):
    return floats(
        next(
            include
            for include in root.findall(".//include")
            if include.findtext("name") == name
        ).findtext("pose")
    )


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


def test_simulated_arm_controller_target_matches_hpp_default_configuration():
    sdf = ElementTree.parse(DEMO_SDF).getroot()
    controller = next(
        plugin
        for plugin in sdf.findall(".//plugin")
        if plugin.attrib["filename"]
        == "gz-sim-joint-trajectory-controller-system"
        and plugin.findtext("joint_name") == "joint_1"
    )
    initial_positions = controller_initial_positions(controller)
    joint_names = [f"joint_{index}" for index in range(1, 7)]

    assert tuple(initial_positions[name] for name in joint_names) == pytest.approx(
        assigned_literal(PROBLEM, "DEFAULT_Q_START")
    )


def test_hpp_uses_canonical_robot_and_shared_cell_descriptions():
    assert assigned_literal(PROBLEM, "ROBOT_URDF") == (
        "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
    )
    assert assigned_literal(PROBLEM, "CELL_URDF") == (
        "package://mfja_3rd_floor_description/urdf/room315_cell.urdf"
    )


def test_hpp_room_fixture_visuals_match_collision_meshes_and_world_poses():
    cell = ElementTree.parse(CELL_URDF).getroot()
    world = ElementTree.parse(ROOM315_WORLD).getroot()
    fixtures = {
        "carter_droit": "room315_carter_droit_1",
        "carter_gauche": "room315_carter_gauche_1",
        "cell_static_droit": "room315_cell_static_droit_final_1",
        "cell_static_gauche": "cell_static_gauche_final_1",
        "cell_path_left": "room315_cell_path_left_1",
        "cell_path_right": "room315_cell_path_right_1",
    }

    for link_name, world_name in fixtures.items():
        link = cell.find(f"./link[@name='{link_name}']")
        visual = link.find("visual/geometry/mesh").attrib["filename"]
        collision = link.find("collision/geometry/mesh").attrib["filename"]
        assert visual == collision
        assert urdf_joint_pose(cell, f"{link_name}_joint") == pytest.approx(
            sdf_include_pose(world, world_name)
        )

    shell = cell.find("./link[@name='room_shell']/visual/geometry/mesh")
    assert shell.attrib["filename"].endswith("/room_315/meshes/315_room.stl")
    assert urdf_joint_pose(cell, "room_shell_joint") == pytest.approx(
        sdf_include_pose(world, "room_315_1")
    )


def test_hpp_table_visual_and_collision_match_the_room_table_pose():
    table = ElementTree.parse(TABLE_URDF).getroot()
    world = ElementTree.parse(ROOM315_WORLD).getroot()
    link = table.find("./link[@name='drop_zone_link']")
    table_visual = next(
        visual
        for visual in link.findall("visual")
        if visual.find("geometry/mesh") is not None
    )
    collision = link.find("collision")

    assert table_visual.find("geometry/mesh").attrib["filename"] == (
        collision.find("geometry/mesh").attrib["filename"]
    )
    assert table_visual.find("origin").attrib == collision.find("origin").attrib

    drop_zone_pose = assigned_literal(PROBLEM, "TABLE_DROP_ZONE_POSE")
    mesh_pose = add_vectors(
        drop_zone_pose[:3], floats(collision.find("origin").attrib["xyz"])
    )
    world_pose = sdf_include_pose(world, "room315_staubli_table_1")
    assert mesh_pose == pytest.approx(world_pose[:3])
    assert floats(collision.find("origin").attrib["rpy"]) == pytest.approx(
        world_pose[3:]
    )
