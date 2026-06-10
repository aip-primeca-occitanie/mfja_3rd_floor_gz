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
OBSTACLE_MODEL_PATH = (
    REPO_ROOT
    / 'mfja_3rd_floor_description'
    / 'models'
    / 'room315_vla_removable_obstacle_marker'
    / 'model.sdf'
)
WORLD_PATHS = (
    REPO_ROOT / 'mfja_3rd_floor_description' / 'worlds' / 'room_315_only.world',
    REPO_ROOT / 'mfja_3rd_floor_description' / 'worlds' / 'mfja_3rd_floor.world',
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


def test_vla_station_and_inspection_eval_markers_are_removed():
    root = ET.parse(MODEL_PATH).getroot()
    link_names = {link.get('name') for link in root.findall('.//link')}
    visual_names = {visual.get('name') for visual in root.findall('.//visual')}
    collision_names = {collision.get('name') for collision in root.findall('.//collision')}

    slot_fiducials = {
        'right_slot_1_marker',
        'right_slot_2_marker',
        'right_slot_3_marker',
        'right_slot_4_marker',
        'left_slot_1_marker',
        'left_slot_2_marker',
        'left_slot_3_marker',
        'left_slot_4_marker',
    }
    removed_eval_markers = {
        'right_yaskawa_station_marker',
        'right_staubli_station_marker',
        'left_yaskawa_station_marker',
        'left_kuka_station_marker',
        'right_green_inspection_marker',
        'left_green_inspection_marker',
        'right_station_empty_marker',
        'right_station_occupied_marker',
        'left_station_empty_marker',
        'left_station_occupied_marker',
    }

    assert 'visual_eval_markers_link' not in link_names
    assert slot_fiducials <= visual_names
    assert not slot_fiducials & collision_names
    assert not removed_eval_markers & visual_names
    assert not removed_eval_markers & collision_names


def test_vla_removable_obstacle_marker_is_visual_only_and_in_room315_worlds():
    root = ET.parse(OBSTACLE_MODEL_PATH).getroot()
    visual_names = {visual.get('name') for visual in root.findall('.//visual')}

    assert {
        'obstacle_body_visual',
        'obstacle_top_warning_stripe',
        'obstacle_front_warning_stripe',
        'obstacle_back_warning_stripe',
    } <= visual_names
    assert root.findall('.//collision') == []

    for world_path in WORLD_PATHS:
        world = ET.parse(world_path).getroot()
        includes = {
            include.findtext('name', default='').strip(): include.findtext('uri', default='').strip()
            for include in world.findall('.//include')
        }
        assert includes['room315_vla_right_obstacle_marker'] == (
            'model://room315_vla_removable_obstacle_marker'
        )
        assert includes['room315_vla_left_obstacle_marker'] == (
            'model://room315_vla_removable_obstacle_marker'
        )
        assert 'room315_vla_removable_obstacle_marker' not in includes
