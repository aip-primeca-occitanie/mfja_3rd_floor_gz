#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
SHUTTLE_SDF = (
    REPO_ROOT
    / 'mfja_3rd_floor_description'
    / 'models'
    / 'room315_shuttle'
    / 'model.sdf'
)


def test_room315_shuttle_has_no_gazebo_contact_collisions():
    root = ET.parse(SHUTTLE_SDF).getroot()
    bitmasks = [
        bitmask.text.strip()
        for bitmask in root.findall('.//collision/surface/contact/collide_bitmask')
        if bitmask.text
    ]

    assert bitmasks
    assert set(bitmasks) == {'0x0000'}
