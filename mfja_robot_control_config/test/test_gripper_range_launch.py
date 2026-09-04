#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTROL_PATH = REPO_ROOT / 'mfja_robot_control_config'
DESCRIPTION_PATH = REPO_ROOT / 'mfja_3rd_floor_description'
BRINGUP_PATH = REPO_ROOT / 'mfja_3rd_floor_bringup'
UTILITY_PATH = CONTROL_PATH / 'launch' / 'gripper_range_config.py'
MULTI_LAUNCH = CONTROL_PATH / 'launch' / 'multi_robot_sim.launch.py'
ISOLATED_LAUNCH = (
    CONTROL_PATH / 'launch' / 'isolated_industrial_robot.launch.py'
)
JAW_JOINTS = ('gripper_left_jaw_joint', 'gripper_right_jaw_joint')


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _entry(q0=0.004, q100=0.025, open_percentage=80.0, close_percentage=90.0):
    return {
        'position_at_0_percent_m': q0,
        'position_at_100_percent_m': q100,
        'default_open_percentage': open_percentage,
        'default_close_percentage': close_percentage,
    }


def _write_config(path, grippers, schema_version=2):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump({
            'schema_version': schema_version,
            'grippers': grippers,
        }),
        encoding='utf-8',
    )


def test_loader_validates_only_requested_instances_and_returns_plain_dicts(tmp_path):
    utility = _load_module('gripper_range_config', UTILITY_PATH)
    config_path = tmp_path / 'grippers.yaml'
    _write_config(config_path, {
        'staubli1': _entry(q0='0.003', q100='0.021'),
        'not_selected': 'intentionally invalid and ignored',
    })

    loaded = utility.load_gripper_range_config(config_path, ['staubli1'])

    assert loaded == {
        'staubli1': {
            'position_at_0_percent_m': pytest.approx(0.003),
            'position_at_100_percent_m': pytest.approx(0.021),
            'default_open_percentage': pytest.approx(80.0),
            'default_close_percentage': pytest.approx(90.0),
        }
    }
    assert type(loaded) is dict
    assert type(loaded['staubli1']) is dict


def test_loader_requires_schema_v2_and_each_selected_instance(tmp_path):
    utility = _load_module('gripper_range_config_schema', UTILITY_PATH)
    wrong_schema = tmp_path / 'wrong_schema.yaml'
    _write_config(wrong_schema, {'staubli1': _entry()}, schema_version=1)

    with pytest.raises(RuntimeError, match='schema_version: 2'):
        utility.load_gripper_range_config(wrong_schema, ['staubli1'])

    valid = tmp_path / 'valid.yaml'
    _write_config(valid, {'staubli1': _entry()})
    with pytest.raises(RuntimeError, match=r'missing grippers\.kuka1'):
        utility.load_gripper_range_config(valid, ['staubli1', 'kuka1'])


def test_loader_rejects_unknown_root_and_selected_entry_fields(tmp_path):
    utility = _load_module('gripper_range_config_unknown_fields', UTILITY_PATH)
    unknown_root = tmp_path / 'unknown_root.yaml'
    _write_config(unknown_root, {'staubli1': _entry()})
    with unknown_root.open('a', encoding='utf-8') as stream:
        stream.write('unexpected: true\n')

    with pytest.raises(RuntimeError, match='unknown top-level field'):
        utility.load_gripper_range_config(unknown_root, ['staubli1'])

    unknown_entry = tmp_path / 'unknown_entry.yaml'
    entry = _entry()
    entry['unexpected'] = True
    _write_config(unknown_entry, {'staubli1': entry})

    with pytest.raises(RuntimeError, match='unknown field'):
        utility.load_gripper_range_config(unknown_entry, ['staubli1'])


@pytest.mark.parametrize(
    ('updates', 'message'),
    (
        ({'position_at_0_percent_m': -0.001}, 'greater than or equal to 0'),
        ({'position_at_100_percent_m': 0.004}, 'must be greater than'),
        ({'position_at_100_percent_m': float('inf')}, 'finite number'),
        ({'position_at_0_percent_m': True}, 'finite number'),
        ({'default_open_percentage': 101.0}, 'between 0 and 100'),
        ({'default_close_percentage': float('nan')}, 'finite number'),
    ),
)
def test_loader_rejects_invalid_ranges_and_percentages(tmp_path, updates, message):
    utility = _load_module('gripper_range_config_invalid', UTILITY_PATH)
    entry = _entry()
    entry.update(updates)
    config_path = tmp_path / 'invalid.yaml'
    _write_config(config_path, {'staubli1': entry})

    with pytest.raises(RuntimeError, match=message):
        utility.load_gripper_range_config(config_path, ['staubli1'])


def test_empty_selection_does_not_require_a_gripper_file(tmp_path):
    utility = _load_module('gripper_range_config_empty', UTILITY_PATH)

    assert utility.load_gripper_range_config(
        tmp_path / 'does_not_exist.yaml',
        [],
    ) == {}


@pytest.mark.parametrize(
    ('robot_name', 'model_name'),
    (
        ('kuka1', 'kuka_kr6r900sixx'),
        ('staubli1', 'staubli_tx2_60l'),
        ('yaskawa_hc10_1', 'yaskawa_hc10'),
        ('yaskawa_hc10dt_1', 'yaskawa_hc10dt'),
    ),
)
def test_materializer_overrides_both_sdf_joints_controller_and_urdf(
    tmp_path,
    robot_name,
    model_name,
):
    utility = _load_module('gripper_range_materializer', UTILITY_PATH)
    source_sdf = DESCRIPTION_PATH / 'models' / model_name / 'model.sdf'
    source_urdf = DESCRIPTION_PATH / 'urdf' / f'{model_name}.urdf'
    original_sdf = source_sdf.read_bytes()
    original_urdf = source_urdf.read_bytes()
    output_sdf = tmp_path / f'{model_name}_configured.sdf'

    assets = utility.materialize_gripper_assets(
        source_sdf,
        source_urdf,
        robot_name,
        _entry(q0=0.004, q100=0.025),
        output_sdf_path=output_sdf,
    )

    assert Path(assets['sdf_path']) == output_sdf
    sdf_root = ET.parse(output_sdf).getroot()
    urdf_root = ET.fromstring(assets['robot_description'])
    for joint_name in JAW_JOINTS:
        sdf_joint = sdf_root.find(f".//joint[@name='{joint_name}']")
        urdf_joint = urdf_root.find(f".//joint[@name='{joint_name}']")
        assert float(sdf_joint.findtext('axis/limit/lower')) == pytest.approx(0.004)
        assert float(sdf_joint.findtext('axis/limit/upper')) == pytest.approx(0.025)
        assert float(urdf_joint.find('limit').get('lower')) == pytest.approx(0.004)
        assert float(urdf_joint.find('limit').get('upper')) == pytest.approx(0.025)

    controller = sdf_root.find(
        ".//plugin[@name='mfja::sim::systems::SymmetricGripperController']"
    )
    assert float(controller.findtext('min_position')) == pytest.approx(0.004)
    assert float(controller.findtext('max_position')) == pytest.approx(0.025)
    assert float(controller.findtext('initial_position')) == pytest.approx(0.004)
    assert source_sdf.read_bytes() == original_sdf
    assert source_urdf.read_bytes() == original_urdf


def _launch_context(values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _package_share_resolver(control_share):
    def resolve(package_name):
        if package_name == 'mfja_3rd_floor_description':
            return str(DESCRIPTION_PATH)
        if package_name == 'mfja_robot_control_config':
            return str(control_share)
        if package_name == 'ros_gz_sim':
            return str(control_share / 'fake_ros_gz_sim')
        raise LookupError(package_name)

    return resolve


@pytest.mark.parametrize(
    ('module_name', 'launch_path', 'extra_values'),
    (
        (
            'multi_robot_gripper_range_launch',
            MULTI_LAUNCH,
            {
                'robots': 'staubli',
                'enable_room315_visual_obstacles': 'true',
                'pause_during_switch_update': 'false',
                'visual_debug_colors': 'true',
                'initial_loop_mode': 'auto',
            },
        ),
        (
            'isolated_robot_gripper_range_launch',
            ISOLATED_LAUNCH,
            {'robot': 'staubli'},
        ),
    ),
)
def test_launch_setup_loads_relative_config_and_materializes_selected_robot(
    tmp_path,
    monkeypatch,
    module_name,
    launch_path,
    extra_values,
):
    launch = _load_module(module_name, launch_path)
    control_share = tmp_path / 'control_share'
    config_path = control_share / 'config' / 'custom_grippers.yaml'
    _write_config(config_path, {'staubli1': _entry(q0=0.006, q100=0.031)})
    monkeypatch.setattr(
        launch,
        'get_package_share_directory',
        _package_share_resolver(control_share),
    )

    original_materializer = launch._materialize_gripper_assets
    calls = []

    def record_materialization(sdf_path, urdf_path, robot_name, range_config):
        assets = original_materializer(
            sdf_path,
            urdf_path,
            robot_name,
            range_config,
        )
        calls.append((robot_name, range_config, assets))
        return assets

    monkeypatch.setattr(
        launch,
        '_materialize_gripper_assets',
        record_materialization,
    )
    values = {
        'world_name': 'isolated_industrial_robot',
        'gz_partition': 'gripper_range_launch_test',
        'robot_config': str(
            CONTROL_PATH / 'config' / 'robots_room_315_only.yaml'
        ),
        'gripper_config': 'config/custom_grippers.yaml',
        'gui_config': 'config/mfja_light.gui.config',
        'use_sim_time': 'true',
        'gui': 'false',
        'start_paused': 'true',
        **extra_values,
    }

    actions = launch._launch_setup(_launch_context(values))

    assert actions
    assert len(calls) == 1
    robot_name, loaded_range, assets = calls[0]
    assert robot_name == 'staubli1'
    assert loaded_range['position_at_0_percent_m'] == pytest.approx(0.006)
    assert loaded_range['position_at_100_percent_m'] == pytest.approx(0.031)
    materialized_root = ET.parse(assets['sdf_path']).getroot()
    left_joint = materialized_root.find(
        ".//joint[@name='gripper_left_jaw_joint']"
    )
    assert float(left_joint.findtext('axis/limit/lower')) == pytest.approx(0.006)
    assert float(left_joint.findtext('axis/limit/upper')) == pytest.approx(0.031)
    Path(assets['sdf_path']).unlink()


def test_both_launches_declare_the_shared_gripper_config_argument():
    for launch_path in (MULTI_LAUNCH, ISOLATED_LAUNCH):
        text = launch_path.read_text(encoding='utf-8')
        assert "'gripper_config'" in text
        assert "default_value='config/gripper_command_defaults.yaml'" in text
        assert '_load_gripper_range_config' in text
        assert '_materialize_gripper_assets' in text
        assert "['sdf_path']" in text
        assert "['robot_description']" in text


def test_bringup_launches_declare_and_forward_gripper_config():
    single_text = (
        BRINGUP_PATH / 'launch' / 'single_industrial_robot.launch.py'
    ).read_text(encoding='utf-8')
    floor_common_text = (
        BRINGUP_PATH / 'launch' / 'room_315_floor_common.py'
    ).read_text(encoding='utf-8')

    for text in (single_text, floor_common_text):
        assert "DeclareLaunchArgument(\n            'gripper_config'" in text
        assert "'gripper_config': LaunchConfiguration('gripper_config')" in text
