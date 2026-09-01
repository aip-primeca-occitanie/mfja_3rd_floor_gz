from pathlib import Path
from xml.etree import ElementTree


REPOSITORY = Path(__file__).parents[2]
STAUBLI_URDF = (
    REPOSITORY
    / "mfja_3rd_floor_description"
    / "urdf"
    / "staubli_tx2_60l.urdf"
)
STAUBLI_SRDF = (
    REPOSITORY
    / "mfja_3rd_floor_description"
    / "srdf"
    / "staubli_tx2_60l.srdf"
)


def test_staubli_arm_links_have_collision_geometry():
    robot = ElementTree.parse(STAUBLI_URDF).getroot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}

    for name in ["base_link", *(f"link_{index}" for index in range(1, 7))]:
        assert links[name].find("collision/geometry/mesh") is not None


def test_staubli_semantics_reference_shared_robot_links():
    urdf = ElementTree.parse(STAUBLI_URDF).getroot()
    srdf = ElementTree.parse(STAUBLI_SRDF).getroot()
    links = {link.attrib["name"] for link in urdf.findall("link")}

    assert srdf.attrib["name"] == urdf.attrib["name"]
    assert srdf.find("gripper/link").attrib["name"] == "gripper_tcp"
    for pair in srdf.findall("disable_collisions"):
        assert pair.attrib["link1"] in links
        assert pair.attrib["link2"] in links
