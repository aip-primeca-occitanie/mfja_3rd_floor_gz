#!/usr/bin/env python3

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MULTI_ROBOT_LAUNCH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'launch'
    / 'multi_robot_sim.launch.py'
)
DESCRIPTION = REPO_ROOT / 'mfja_3rd_floor_description'


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'multi_robot_sim_mobile_frame_test',
        MULTI_ROBOT_LAUNCH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_mobile_sdf_materializer_prefixes_transport_and_uses_urdf_root(
    tmp_path,
    monkeypatch,
):
    launch = _load_launch_module()
    monkeypatch.setattr(launch.tempfile, 'gettempdir', lambda: str(tmp_path))

    for model_name, robot_name in (
        ('tiago', 'tiago1'),
        ('tiago_base', 'tiago_base1'),
    ):
        source = DESCRIPTION / 'models' / model_name / 'model.sdf'
        output = Path(
            launch._materialize_mobile_model_sdf(str(source), robot_name)
        )
        text = output.read_text(encoding='utf-8')

        assert f'<topic>/model/{robot_name}/cmd_vel</topic>' in text
        assert f'<odom_topic>/model/{robot_name}/odom</odom_topic>' in text
        assert f'<tf_topic>/model/{robot_name}/tf</tf_topic>' in text
        assert f'<frame_id>{robot_name}/odom</frame_id>' in text
        assert (
            f'<child_frame_id>{robot_name}/base_link</child_frame_id>'
            in text
        )
        assert '<frame_id>odom</frame_id>' not in text
        assert '<child_frame_id>base_footprint</child_frame_id>' not in text

        urdf = ET.parse(DESCRIPTION / 'urdf' / f'{model_name}.urdf')
        footprint_joint = urdf.find("./joint[@name='base_footprint_joint']")
        assert footprint_joint is not None
        assert footprint_joint.find('parent').attrib['link'] == 'base_link'
        assert footprint_joint.find('child').attrib['link'] == 'base_footprint'


def test_robot_state_publishers_use_per_instance_frame_prefixes():
    text = MULTI_ROBOT_LAUNCH.read_text(encoding='utf-8')

    assert "frame_prefix = f'{robot_name}/'" in text
    assert "frame_prefix = '' if model_name in MOBILE_MODELS" not in text
