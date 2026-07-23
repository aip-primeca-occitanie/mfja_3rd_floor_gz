"""Shared helpers for Room 315 launch descriptions."""

import xml.etree.ElementTree as ET


def get_world_entity_name(world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_element = root.find('world')
    if world_element is None:
        raise RuntimeError(f'No <world> element found in: {world_path}')
    return world_element.attrib.get('name', 'default')
