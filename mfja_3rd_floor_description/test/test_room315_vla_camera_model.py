#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = (
    REPO_ROOT
    / 'mfja_3rd_floor_description'
    / 'models'
    / 'room315_vla_overhead_devices'
    / 'model.sdf'
)


def test_vla_rgbd_camera_sensor_visualization_is_disabled():
    root = ET.parse(MODEL_PATH).getroot()
    link_names = {link.get('name') for link in root.findall('.//link')}
    sensors = [
        sensor
        for sensor in root.findall('.//sensor')
        if str(sensor.get('name', '')).startswith('room315_vla_')
    ]

    assert 'vla_status_panel_link' not in link_names
    assert 'vla_estop_pedestal_link' not in link_names
    assert {sensor.get('name') for sensor in sensors} == {
        'room315_vla_right_rail_rgbd',
        'room315_vla_left_rail_rgbd',
    }
    for sensor in sensors:
        visualize = sensor.findtext('visualize', default='false').strip().lower()
        assert visualize == 'false'
