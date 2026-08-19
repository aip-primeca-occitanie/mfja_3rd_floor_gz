from pathlib import Path
from xml.etree import ElementTree


REPOSITORY = Path(__file__).parents[2]
STAUBLI_URDF = (
    REPOSITORY
    / "mfja_3rd_floor_description"
    / "urdf"
    / "staubli_tx2_60l.urdf"
)


def test_staubli_arm_links_have_collision_geometry():
    robot = ElementTree.parse(STAUBLI_URDF).getroot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}

    for name in ["base_link", *(f"link_{index}" for index in range(1, 7))]:
        assert links[name].find("collision/geometry/mesh") is not None
