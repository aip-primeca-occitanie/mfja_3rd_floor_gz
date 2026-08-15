#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'mfja_robot_control_config' / 'scripts' / 'robot_gripper_command.py'
DESCRIPTION_PATH = REPO_ROOT / 'mfja_3rd_floor_description'
DEFAULTS_PATH = (
    REPO_ROOT
    / 'mfja_robot_control_config'
    / 'config'
    / 'gripper_command_defaults.yaml'
)
JAW_JOINTS = ('gripper_left_jaw_joint', 'gripper_right_jaw_joint')


def _load_module():
    spec = importlib.util.spec_from_file_location('robot_gripper_command', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _shipped_settings(command, selector):
    profile = command.resolve_profile(selector)
    return command.load_command_defaults(DEFAULTS_PATH)[profile.robot_name]


@pytest.mark.parametrize(
    ('selector', 'robot_name'),
    (
        ('kuka', 'kuka1'),
        ('staubli', 'staubli1'),
        ('hc10', 'yaskawa_hc10_1'),
        ('hc10dt', 'yaskawa_hc10dt_1'),
    ),
)
def test_profiles_match_gripper_topics(selector, robot_name):
    command = _load_module()

    profile = command.resolve_profile(selector)

    assert profile.robot_name == robot_name
    assert profile.topic == f'/{robot_name}/gripper/position_command'
    assert profile.joint_state_topic == f'/{robot_name}/joint_states'


def test_source_sdf_and_urdf_gripper_limits_match():
    command = _load_module()

    for profile in command.PROFILES:
        sdf_root = ET.parse(
            DESCRIPTION_PATH / 'models' / profile.model_name / 'model.sdf'
        ).getroot()
        urdf_root = ET.parse(
            DESCRIPTION_PATH / 'urdf' / f'{profile.model_name}.urdf'
        ).getroot()

        for joint_name in JAW_JOINTS:
            sdf_joint = sdf_root.find(f".//joint[@name='{joint_name}']")
            urdf_joint = urdf_root.find(f".//joint[@name='{joint_name}']")
            assert sdf_joint is not None, (profile.model_name, joint_name)
            assert urdf_joint is not None, (profile.model_name, joint_name)

            sdf_lower = float(sdf_joint.findtext('axis/limit/lower'))
            sdf_upper = float(sdf_joint.findtext('axis/limit/upper'))
            urdf_limit = urdf_joint.find('limit')
            urdf_lower = float(urdf_limit.get('lower'))
            urdf_upper = float(urdf_limit.get('upper'))

            assert urdf_lower == pytest.approx(sdf_lower)
            assert urdf_upper == pytest.approx(sdf_upper)
            assert sdf_lower >= 0.0
            assert sdf_upper > sdf_lower


def test_full_model_and_numeric_aliases_resolve_to_same_profile():
    command = _load_module()

    assert command.resolve_profile('1') is command.resolve_profile('KUKA_KR6R900SIXX')
    assert command.resolve_profile('4') is command.resolve_profile('yaskawa_hc10dt_1')


def test_open_close_and_custom_position_resolution():
    command = _load_module()
    profile = command.resolve_profile('staubli')
    settings = _shipped_settings(command, 'staubli')

    assert command.resolve_target_position(
        profile, 'open', None, defaults=settings
    ) == pytest.approx(settings.position_at_100_percent_m)
    assert command.resolve_target_position(
        profile, 'close', None, defaults=settings
    ) == pytest.approx(settings.position_at_0_percent_m)
    assert command.resolve_target_position(
        profile,
        None,
        settings.position_at_100_percent_m,
        defaults=settings,
    ) == pytest.approx(settings.position_at_100_percent_m)


@pytest.mark.parametrize(
    ('action', 'percentage', 'expected'),
    (
        ('open', 0.0, 0.0),
        ('open', 25.0, 0.0075),
        ('open', 100.0, 0.030),
        ('close', 0.0, 0.030),
        ('close', 25.0, 0.0225),
        ('close', 100.0, 0.0),
    ),
)
def test_explicit_action_percentage_maps_to_absolute_travel(
    action,
    percentage,
    expected,
):
    command = _load_module()
    profile = command.resolve_profile('kuka')
    settings = _shipped_settings(command, 'kuka')

    assert command.resolve_target_position(
        profile,
        action,
        None,
        percentage,
        settings,
    ) == pytest.approx(expected)


@pytest.mark.parametrize(
    'selector',
    ('kuka', 'staubli', 'hc10', 'hc10dt'),
)
@pytest.mark.parametrize('percentage', (0.0, 25.0, 50.0, 100.0))
def test_percentage_equations_hold_for_every_profile(selector, percentage):
    command = _load_module()
    profile = command.resolve_profile(selector)
    settings = _shipped_settings(command, selector)
    q0 = settings.position_at_0_percent_m
    q100 = settings.position_at_100_percent_m
    travel = q100 - q0

    opened = command.resolve_target_position(
        profile,
        'open',
        None,
        percentage,
        settings,
    )
    closed = command.resolve_target_position(
        profile,
        'close',
        None,
        percentage,
        settings,
    )

    assert opened == pytest.approx(q0 + travel * percentage / 100.0)
    assert closed == pytest.approx(q100 - travel * percentage / 100.0)
    assert opened + closed == pytest.approx(q0 + q100)


@pytest.mark.parametrize(
    ('action', 'normalized'),
    (
        ('open', 'open'),
        ('OPEN', 'open'),
        ('close', 'close'),
        ('CLOSE', 'close'),
    ),
)
def test_english_actions_are_case_insensitive(action, normalized):
    command = _load_module()

    assert command.normalize_command(action) == normalized


@pytest.mark.parametrize('action', ('ouvrir', 'open_now', 'shut'))
def test_non_open_close_actions_are_rejected(action):
    command = _load_module()

    with pytest.raises(ValueError, match='open, close'):
        command.normalize_command(action)


def test_command_vocabulary_is_english_only():
    command = _load_module()

    assert command.COMMANDS == ('open', 'close')


def test_bare_action_uses_configurable_per_robot_defaults():
    command = _load_module()
    profile = command.resolve_profile('hc10')
    defaults = command.GripperCommandDefaults(
        position_at_0_percent_m=0.004,
        position_at_100_percent_m=0.044,
        open_percentage=60.0,
        close_percentage=80.0,
    )

    assert command.resolve_target_position(
        profile,
        'open',
        None,
        defaults=defaults,
    ) == pytest.approx(0.028)
    assert command.resolve_target_position(
        profile,
        'close',
        None,
        defaults=defaults,
    ) == pytest.approx(0.012)


@pytest.mark.parametrize(
    ('action', 'percentage', 'expected'),
    (
        ('open', 0.0, 0.005),
        ('open', 25.0, 0.015),
        ('open', 100.0, 0.045),
        ('close', 0.0, 0.045),
        ('close', 25.0, 0.035),
        ('close', 100.0, 0.005),
    ),
)
def test_percentage_mapping_uses_configured_nonzero_endpoints(
    action,
    percentage,
    expected,
):
    command = _load_module()
    profile = command.resolve_profile('staubli')
    settings = command.GripperCommandDefaults(
        position_at_0_percent_m=0.005,
        position_at_100_percent_m=0.045,
    )

    assert command.resolve_target_position(
        profile,
        action,
        None,
        percentage,
        settings,
    ) == pytest.approx(expected)


@pytest.mark.parametrize('percentage', (-1.0, 100.01, float('inf'), float('nan')))
def test_action_percentage_must_be_finite_and_between_zero_and_100(percentage):
    command = _load_module()
    profile = command.resolve_profile('kuka')
    settings = _shipped_settings(command, 'kuka')

    with pytest.raises(ValueError, match='between 0 and 100'):
        command.resolve_target_position(profile, 'open', None, percentage, settings)


def test_percentage_requires_action_and_is_incompatible_with_position():
    command = _load_module()
    profile = command.resolve_profile('hc10')
    settings = _shipped_settings(command, 'hc10')

    with pytest.raises(ValueError, match='requires an open/close command'):
        command.resolve_target_position(profile, None, None, 50.0, settings)
    with pytest.raises(ValueError, match='either open/close or --position'):
        command.resolve_target_position(profile, 'open', 0.01, 50.0, settings)


def test_defaults_file_covers_all_robots_and_uses_full_action_defaults():
    command = _load_module()

    defaults = command.load_command_defaults(DEFAULTS_PATH)

    assert set(defaults) == {profile.robot_name for profile in command.PROFILES}
    for settings in defaults.values():
        assert settings.position_at_0_percent_m >= 0.0
        assert settings.position_at_100_percent_m > settings.position_at_0_percent_m
        assert 0.0 <= settings.open_percentage <= 100.0
        assert 0.0 <= settings.close_percentage <= 100.0


@pytest.mark.parametrize(
    'arguments',
    (
        ('kuka', 'open', '50'),
        ('kuka', 'open'),
        ('kuka', '--position', '0.015'),
    ),
)
def test_every_command_mode_requires_the_configuration_file(arguments, tmp_path, capsys):
    command = _load_module()
    missing_path = tmp_path / 'missing.yaml'

    result = command.main([
        *arguments,
        '--defaults-file',
        str(missing_path),
        '--dry-run',
    ])

    assert result == 2
    assert 'cannot load gripper defaults' in capsys.readouterr().err


def test_defaults_loader_rejects_non_mapping_and_boolean_values(tmp_path):
    command = _load_module()
    non_mapping = tmp_path / 'non_mapping.yaml'
    non_mapping.write_text('schema_version: 2\ngrippers: []\n', encoding='utf-8')

    with pytest.raises(ValueError, match='"grippers" mapping'):
        command.load_command_defaults(non_mapping)

    boolean_value = tmp_path / 'boolean.yaml'
    boolean_value.write_text(
        DEFAULTS_PATH.read_text(encoding='utf-8').replace(
            'default_open_percentage: 100.0',
            'default_open_percentage: true',
            1,
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='between 0 and 100'):
        command.load_command_defaults(boolean_value)


def test_defaults_loader_requires_schema_version_2(tmp_path):
    command = _load_module()
    old_schema = tmp_path / 'old_schema.yaml'
    old_schema.write_text(
        DEFAULTS_PATH.read_text(encoding='utf-8').replace(
            'schema_version: 2',
            'schema_version: 1',
        ),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='schema_version: 2'):
        command.load_command_defaults(old_schema)


def test_defaults_loader_rejects_unknown_schema_fields(tmp_path):
    command = _load_module()
    unknown_root = tmp_path / 'unknown_root.yaml'
    unknown_root.write_text(
        DEFAULTS_PATH.read_text(encoding='utf-8') + 'unexpected: true\n',
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='unknown top-level keys: unexpected'):
        command.load_command_defaults(unknown_root)

    unknown_record = tmp_path / 'unknown_record.yaml'
    unknown_record.write_text(
        DEFAULTS_PATH.read_text(encoding='utf-8').replace(
            '  kuka1:\n',
            '  kuka1:\n    unexpected: true\n',
            1,
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='unknown fields in grippers.kuka1'):
        command.load_command_defaults(unknown_record)


@pytest.mark.parametrize(
    ('field', 'replacement', 'message'),
    (
        ('position_at_0_percent_m: 0.0', 'position_at_0_percent_m: -0.001', 'zero or greater'),
        (
            'position_at_100_percent_m: 0.030',
            'position_at_100_percent_m: 0.0',
            'must be greater',
        ),
        (
            'position_at_100_percent_m: 0.030',
            'position_at_100_percent_m: .inf',
            'finite position',
        ),
    ),
)
def test_defaults_loader_validates_configured_endpoints(
    field,
    replacement,
    message,
    tmp_path,
):
    command = _load_module()
    invalid = tmp_path / 'invalid_endpoint.yaml'
    invalid.write_text(
        DEFAULTS_PATH.read_text(encoding='utf-8').replace(field, replacement, 1),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match=message):
        command.load_command_defaults(invalid)


def test_defaults_loader_requires_every_current_robot_but_allows_future_robots(tmp_path):
    command = _load_module()
    loaded = DEFAULTS_PATH.read_text(encoding='utf-8')
    missing = tmp_path / 'missing_robot.yaml'
    missing.write_text(
        loaded.replace(
            '''\
  yaskawa_hc10dt_1:
    position_at_0_percent_m: 0.0
    position_at_100_percent_m: 0.010
    default_open_percentage: 100.0
    default_close_percentage: 100.0
''',
            '',
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='missing grippers.yaskawa_hc10dt_1'):
        command.load_command_defaults(missing)

    future = tmp_path / 'future_robot.yaml'
    future.write_text(
        loaded
        + '''\
  future_robot:
    position_at_0_percent_m: 0.0
    position_at_100_percent_m: 0.1
    default_open_percentage: 100
    default_close_percentage: 100
''',
        encoding='utf-8',
    )
    configured = command.load_command_defaults(future)
    assert set(configured) == {profile.robot_name for profile in command.PROFILES}


def test_single_robot_command_accepts_a_matching_partial_config(tmp_path, capsys):
    command = _load_module()
    partial = tmp_path / 'staubli_only.yaml'
    partial.write_text(
        '''\
schema_version: 2
grippers:
  staubli1:
    position_at_0_percent_m: 0.003
    position_at_100_percent_m: 0.020
    default_open_percentage: 100
    default_close_percentage: 100
''',
        encoding='utf-8',
    )

    result = command.main([
        'staubli',
        'open',
        '100',
        '--defaults-file',
        str(partial),
        '--dry-run',
    ])

    assert result == 0
    output = capsys.readouterr().out
    assert 'Target position: 0.02 m' in output
    assert 'Allowed range: [0.003, 0.02] m' in output


def test_defaults_file_is_registered_for_install():
    cmake_text = (
        REPO_ROOT / 'mfja_robot_control_config' / 'CMakeLists.txt'
    ).read_text(encoding='utf-8')

    assert 'config/gripper_command_defaults.yaml' in cmake_text


@pytest.mark.parametrize('position', (-0.001, 0.031, float('inf'), float('nan')))
def test_custom_position_must_be_finite_and_inside_configured_range(position):
    command = _load_module()
    profile = command.resolve_profile('kuka')
    settings = _shipped_settings(command, 'kuka')

    with pytest.raises(ValueError):
        command.resolve_target_position(profile, None, position, defaults=settings)


def test_custom_position_accepts_both_configured_endpoints():
    command = _load_module()
    profile = command.resolve_profile('kuka')
    settings = command.GripperCommandDefaults(
        position_at_0_percent_m=0.006,
        position_at_100_percent_m=0.025,
    )

    assert command.resolve_target_position(
        profile, None, 0.006, defaults=settings
    ) == pytest.approx(0.006)
    assert command.resolve_target_position(
        profile, None, 0.025, defaults=settings
    ) == pytest.approx(0.025)
    with pytest.raises(ValueError, match='between 0.006 m and 0.025 m'):
        command.resolve_target_position(profile, None, 0.005, defaults=settings)


def test_command_and_position_are_mutually_exclusive():
    command = _load_module()
    profile = command.resolve_profile('hc10')
    settings = _shipped_settings(command, 'hc10')

    with pytest.raises(ValueError, match='either open/close or --position'):
        command.resolve_target_position(profile, 'open', 0.01, defaults=settings)
    with pytest.raises(ValueError, match='is required'):
        command.resolve_target_position(profile, None, None, defaults=settings)


def test_parser_defaults_publish_a_short_burst():
    command = _load_module()

    args = command.build_parser().parse_args(['hc10dt', 'close'])

    assert args.times == command.DEFAULT_PUBLISH_TIMES
    assert args.rate == pytest.approx(command.DEFAULT_PUBLISH_RATE_HZ)
    assert args.wait_timeout == pytest.approx(command.DEFAULT_WAIT_TIMEOUT_SEC)
    assert args.ready_timeout == pytest.approx(command.DEFAULT_READY_TIMEOUT_SEC)


def test_parser_accepts_percentage_after_english_action():
    command = _load_module()

    parsed = command.build_parser().parse_args(['hc10', 'open', '75'])

    assert parsed.command == 'open'
    assert parsed.percentage == pytest.approx(75.0)


def test_dry_run_resolves_without_ros_import(capsys):
    command = _load_module()

    result = command.main(['hc10', '--position', '0.012', '--dry-run'])

    assert result == 0
    output = capsys.readouterr().out
    assert 'Topic: /yaskawa_hc10_1/gripper/position_command' in output
    assert 'Message type: std_msgs/msg/Float64' in output
    assert 'Target position: 0.012 m' in output
    assert 'Publish burst: times=10, rate_hz=10' in output


def test_dry_run_reports_explicit_percentage(capsys):
    command = _load_module()

    result = command.main(['hc10', 'open', '25', '--dry-run'])

    assert result == 0
    output = capsys.readouterr().out
    assert 'Action: open 25% (explicit)' in output
    assert 'Target position: 0.01 m' in output


def test_explicit_percentage_and_position_use_configured_endpoints(tmp_path, capsys):
    command = _load_module()
    config_path = tmp_path / 'custom_endpoints.yaml'
    config_path.write_text(
        DEFAULTS_PATH.read_text(encoding='utf-8')
        .replace('position_at_0_percent_m: 0.0', 'position_at_0_percent_m: 0.005', 1)
        .replace('position_at_100_percent_m: 0.030', 'position_at_100_percent_m: 0.045', 1),
        encoding='utf-8',
    )

    result = command.main([
        'kuka',
        'open',
        '25',
        '--defaults-file',
        str(config_path),
        '--dry-run',
    ])
    assert result == 0
    output = capsys.readouterr().out
    assert 'Target position: 0.015 m' in output
    assert 'Allowed range: [0.005, 0.045] m' in output

    result = command.main([
        'kuka',
        '--position',
        '0.004',
        '--defaults-file',
        str(config_path),
        '--dry-run',
    ])
    assert result == 2
    assert 'must be between 0.005 m and 0.045 m' in capsys.readouterr().err


def test_dry_run_reads_bare_action_percentage_from_override_file(tmp_path, capsys):
    command = _load_module()
    config_path = tmp_path / 'defaults.yaml'
    config_path.write_text(
        '''\
schema_version: 2
grippers:
  kuka1: {position_at_0_percent_m: 0, position_at_100_percent_m: 0.03, default_open_percentage: 100, default_close_percentage: 100}
  staubli1: {position_at_0_percent_m: 0, position_at_100_percent_m: 0.0025, default_open_percentage: 100, default_close_percentage: 100}
  yaskawa_hc10_1: {position_at_0_percent_m: 0.004, position_at_100_percent_m: 0.044, default_open_percentage: 60, default_close_percentage: 80}
  yaskawa_hc10dt_1: {position_at_0_percent_m: 0, position_at_100_percent_m: 0.01, default_open_percentage: 100, default_close_percentage: 100}
''',
        encoding='utf-8',
    )

    result = command.main([
        'hc10',
        'close',
        '--defaults-file',
        str(config_path),
        '--dry-run',
    ])

    assert result == 0
    output = capsys.readouterr().out
    assert 'Action: close 80% (configured default)' in output
    assert f'Defaults file: {config_path}' in output
    assert 'Target position: 0.012 m' in output
    assert 'Allowed range: [0.004, 0.044] m' in output


def test_list_reports_all_four_robots_without_ros_import(capsys):
    command = _load_module()

    result = command.main(['--list'])

    assert result == 0
    output = capsys.readouterr().out
    assert 'kuka1' in output
    assert 'staubli1' in output
    assert 'yaskawa_hc10_1' in output
    assert 'yaskawa_hc10dt_1' in output
    assert 'position_0%=0 m' in output
    staubli = _shipped_settings(command, 'staubli')
    assert f'position_100%={staubli.position_at_100_percent_m:.10g} m' in output
