#!/usr/bin/env python3

from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
KINEMATICS_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'config' / 'room_315_kinematics'


def _sensor_by_name(path: Path) -> dict[str, dict]:
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    return {
        sensor['name']: sensor
        for sensor in config['position_sensors']
    }


def test_right_da2er_is_on_exterior_incoming_branch():
    sensors = _sensor_by_name(KINEMATICS_DIR / 'rail_devices_right.yaml')
    da2er = sensors['DA2ER']

    assert da2er['switch'] == 'A2'
    assert da2er['branch'] == 'E'
    assert da2er['segment'] == 'A12E'
    assert da2er['radius_m'] >= 0.08


def test_left_da4l_covers_both_a4_connector_branches():
    sensors = _sensor_by_name(KINEMATICS_DIR / 'rail_devices_left.yaml')
    da4l = sensors['DA4L']

    assert da4l['switch'] == 'A4'
    assert da4l['radius_m'] >= 0.07
    assert [point['segment'] for point in da4l['points']] == ['A2E', 'A2I']
    assert all(point['s_ratio'] > 0.95 for point in da4l['points'])
