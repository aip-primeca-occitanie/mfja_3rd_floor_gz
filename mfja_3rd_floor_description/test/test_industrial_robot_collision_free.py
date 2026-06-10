#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION_PATH = REPO_ROOT / 'mfja_3rd_floor_description'
INDUSTRIAL_ROBOT_MODELS = (
    'kuka_kr6r900sixx',
    'staubli_tx2_60l',
    'yaskawa_hc10',
    'yaskawa_hc10dt',
)


def test_industrial_robot_assets_are_collision_free_for_kinematic_debug():
    for model_name in INDUSTRIAL_ROBOT_MODELS:
        sdf_path = DESCRIPTION_PATH / 'models' / model_name / 'model.sdf'
        urdf_path = DESCRIPTION_PATH / 'urdf' / f'{model_name}.urdf'
        sdf_root = ET.parse(sdf_path).getroot()
        urdf_root = ET.parse(urdf_path).getroot()

        assert sdf_root.findall('.//collision') == [], model_name
        assert urdf_root.findall('.//collision') == [], model_name
        assert sdf_root.findall('.//visual'), model_name
        assert urdf_root.findall('.//visual'), model_name
        assert sdf_root.findall('.//joint'), model_name
        assert urdf_root.findall('.//joint'), model_name
