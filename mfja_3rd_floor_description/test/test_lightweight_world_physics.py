#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
WORLD_PATH = REPO_ROOT / 'mfja_3rd_floor_description' / 'worlds'
LIGHTWEIGHT_STEP_SIZE = '0.005'
WORLD_FILES = (
    'mfja_3rd_floor.world',
    'room_315_only.world',
    'isolated_industrial_robot.world',
)


def test_worlds_keep_lightweight_physics_step():
    for world_file in WORLD_FILES:
        world = ET.parse(WORLD_PATH / world_file).getroot()
        max_step_size = world.findtext('.//physics/max_step_size', default='').strip()

        assert max_step_size == LIGHTWEIGHT_STEP_SIZE, world_file
