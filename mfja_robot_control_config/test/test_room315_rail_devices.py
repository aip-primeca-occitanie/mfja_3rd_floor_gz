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
