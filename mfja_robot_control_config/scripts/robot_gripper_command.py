#!/usr/bin/env python3

"""Publish percentage-based actions or bounded positions to an MFJA gripper."""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import yaml


@dataclass(frozen=True)
class GripperCommandProfile:
    robot_name: str
    model_name: str
    aliases: tuple[str, ...]

    @property
    def topic(self) -> str:
        return f'/{self.robot_name}/gripper/position_command'

    @property
    def joint_state_topic(self) -> str:
        return f'/{self.robot_name}/joint_states'


@dataclass(frozen=True)
class GripperCommandDefaults:
    position_at_0_percent_m: float
    position_at_100_percent_m: float
    open_percentage: float = 100.0
    close_percentage: float = 100.0


PROFILES = (
    GripperCommandProfile(
        robot_name='kuka1',
        model_name='kuka_kr6r900sixx',
        aliases=('1', 'kuka', 'kuka1', 'kuka_kr6r900sixx'),
    ),
    GripperCommandProfile(
        robot_name='staubli1',
        model_name='staubli_tx2_60l',
        aliases=('2', 'staubli', 'staubli1', 'staubli_tx2_60l'),
    ),
    GripperCommandProfile(
        robot_name='yaskawa_hc10_1',
        model_name='yaskawa_hc10',
        aliases=('3', 'hc10', 'yaskawa_hc10', 'yaskawa_hc10_1'),
    ),
    GripperCommandProfile(
        robot_name='yaskawa_hc10dt_1',
        model_name='yaskawa_hc10dt',
        aliases=('4', 'hc10dt', 'yaskawa_hc10dt', 'yaskawa_hc10dt_1'),
    ),
)

DEFAULT_PUBLISH_TIMES = 10
DEFAULT_PUBLISH_RATE_HZ = 10.0
DEFAULT_WAIT_TIMEOUT_SEC = 5.0
DEFAULT_READY_TIMEOUT_SEC = 2.0
DEFAULTS_FILENAME = 'gripper_command_defaults.yaml'
COMMANDS = ('open', 'close')
JAW_JOINT_NAMES = frozenset(
    ('gripper_left_jaw_joint', 'gripper_right_jaw_joint')
)


def default_defaults_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / 'config' / DEFAULTS_FILENAME
    if source_path.is_file():
        return source_path

    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = get_package_share_directory('mfja_robot_control_config')
    except (ImportError, LookupError) as exc:
        raise ValueError(
            f'cannot locate the default {DEFAULTS_FILENAME}; '
            'use --defaults-file PATH'
        ) from exc

    return Path(package_share) / 'config' / DEFAULTS_FILENAME


def _validate_percentage(value: float, label: str = 'percentage') -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be a finite number between 0 and 100')
    try:
        percentage = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f'{label} must be a finite number between 0 and 100'
        ) from exc
    if not math.isfinite(percentage) or not 0.0 <= percentage <= 100.0:
        raise ValueError(f'{label} must be a finite number between 0 and 100')
    return percentage


def _validate_position(value: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{label} must be a finite position in meters')
    try:
        position = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be a finite position in meters') from exc
    if not math.isfinite(position):
        raise ValueError(f'{label} must be a finite position in meters')
    return position


def load_command_defaults(
    path: str | Path,
    robot_names: Sequence[str] | None = None,
) -> dict[str, GripperCommandDefaults]:
    config_path = Path(path).expanduser()
    try:
        loaded = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'cannot load gripper defaults from {config_path}: {exc}') from exc

    if not isinstance(loaded, dict):
        raise ValueError(f'{config_path} must contain a YAML mapping')
    unexpected_root_keys = set(loaded) - {'schema_version', 'grippers'}
    if unexpected_root_keys:
        unexpected = ', '.join(sorted(str(key) for key in unexpected_root_keys))
        raise ValueError(f'{config_path} contains unknown top-level keys: {unexpected}')
    schema_version = loaded.get('schema_version')
    if isinstance(schema_version, bool) or schema_version != 2:
        raise ValueError(f'{config_path} must set schema_version: 2')

    grippers = loaded.get('grippers')
    if not isinstance(grippers, dict):
        raise ValueError(
            f'{config_path} must contain a "grippers" mapping'
        )

    known_robot_names = {profile.robot_name for profile in PROFILES}
    requested_robot_names = (
        [profile.robot_name for profile in PROFILES]
        if robot_names is None
        else list(dict.fromkeys(str(name).strip() for name in robot_names))
    )
    unknown_robot_names = set(requested_robot_names) - known_robot_names
    if unknown_robot_names:
        unknown = ', '.join(sorted(unknown_robot_names))
        raise ValueError(f'unknown gripper robot name(s): {unknown}')

    defaults: dict[str, GripperCommandDefaults] = {}
    required_fields = {
        'position_at_0_percent_m',
        'position_at_100_percent_m',
        'default_open_percentage',
        'default_close_percentage',
    }
    for robot_name in requested_robot_names:
        values = grippers.get(robot_name)
        if not isinstance(values, dict):
            raise ValueError(
                f'{config_path} is missing grippers.{robot_name}'
            )
        unknown_fields = set(values) - required_fields
        if unknown_fields:
            unknown = ', '.join(sorted(str(field) for field in unknown_fields))
            raise ValueError(
                f'{config_path} contains unknown fields in '
                f'grippers.{robot_name}: {unknown}'
            )
        try:
            position_at_0_percent_m = _validate_position(
                values['position_at_0_percent_m'],
                f'grippers.{robot_name}.position_at_0_percent_m',
            )
            position_at_100_percent_m = _validate_position(
                values['position_at_100_percent_m'],
                f'grippers.{robot_name}.position_at_100_percent_m',
            )
            open_percentage = _validate_percentage(
                values['default_open_percentage'],
                f'grippers.{robot_name}.default_open_percentage',
            )
            close_percentage = _validate_percentage(
                values['default_close_percentage'],
                f'grippers.{robot_name}.default_close_percentage',
            )
        except KeyError as exc:
            raise ValueError(
                f'{config_path} is missing grippers.{robot_name}.{exc.args[0]}'
            ) from exc
        if position_at_0_percent_m < 0.0:
            raise ValueError(
                f'grippers.{robot_name}.position_at_0_percent_m '
                'must be zero or greater'
            )
        if position_at_100_percent_m <= position_at_0_percent_m:
            raise ValueError(
                f'grippers.{robot_name}.position_at_100_percent_m '
                'must be greater than position_at_0_percent_m'
            )
        defaults[robot_name] = GripperCommandDefaults(
            position_at_0_percent_m=position_at_0_percent_m,
            position_at_100_percent_m=position_at_100_percent_m,
            open_percentage=open_percentage,
            close_percentage=close_percentage,
        )
    return defaults


def _selector_map() -> dict[str, GripperCommandProfile]:
    selectors: dict[str, GripperCommandProfile] = {}
    for profile in PROFILES:
        for alias in profile.aliases:
            selectors[alias.lower()] = profile
    return selectors


def profile_help(
    defaults: dict[str, GripperCommandDefaults] | None = None,
) -> str:
    rows = []
    for profile in PROFILES:
        selector = profile.aliases[0]
        aliases = ', '.join(profile.aliases[1:])
        row = (
            f'{selector}: {profile.robot_name} ({profile.model_name}) '
            f'aliases={aliases}'
        )
        if defaults is not None:
            configured = defaults[profile.robot_name]
            row += (
                f' position_0%={configured.position_at_0_percent_m:.10g} m'
                f' position_100%={configured.position_at_100_percent_m:.10g} m'
                f' default_open={configured.open_percentage:.10g}%'
                f' default_close={configured.close_percentage:.10g}%'
            )
        rows.append(row)
    return '\n'.join(rows)


def resolve_profile(selector: str) -> GripperCommandProfile:
    normalized = selector.strip().lower()
    try:
        return _selector_map()[normalized]
    except KeyError as exc:
        raise ValueError(
            f'unknown robot selector "{selector}". Available selectors:\n{profile_help()}'
        ) from exc


def normalize_command(command: str) -> str:
    normalized = command.strip().lower()
    if normalized not in COMMANDS:
        available = ', '.join(COMMANDS)
        raise ValueError(f'command must be one of: {available}')
    return normalized


def resolve_command_percentage(
    command: str,
    explicit_percentage: float | None,
    defaults: GripperCommandDefaults | None = None,
) -> tuple[str, float, bool]:
    normalized = normalize_command(command)
    if explicit_percentage is not None:
        return normalized, _validate_percentage(explicit_percentage), False

    if defaults is None:
        raise ValueError('configured gripper defaults are required')
    configured = defaults
    percentage = (
        configured.open_percentage
        if normalized == 'open'
        else configured.close_percentage
    )
    return normalized, _validate_percentage(percentage), True


def resolve_target_position(
    profile: GripperCommandProfile,
    command: str | None,
    explicit_position: float | None,
    percentage: float | None = None,
    defaults: GripperCommandDefaults | None = None,
) -> float:
    if command is not None and explicit_position is not None:
        raise ValueError('use either open/close or --position, not both')
    if percentage is not None and command is None:
        raise ValueError('a percentage requires an open/close command')
    if command is None and explicit_position is None:
        raise ValueError('open, close, or --position is required')
    if defaults is None:
        raise ValueError('configured gripper endpoints are required')

    if explicit_position is not None:
        target = _validate_position(explicit_position, 'position')
    else:
        normalized, resolved_percentage, _uses_default = resolve_command_percentage(
            command,
            percentage,
            defaults,
        )
        position_at_0_percent_m = defaults.position_at_0_percent_m
        position_at_100_percent_m = defaults.position_at_100_percent_m
        travel = position_at_100_percent_m - position_at_0_percent_m
        if normalized == 'open':
            target = position_at_0_percent_m + travel * resolved_percentage / 100.0
        else:
            target = position_at_100_percent_m - travel * resolved_percentage / 100.0
        # Keep exact endpoint commands inside the configured interval despite
        # floating-point subtraction at values such as close 100.
        target = min(position_at_100_percent_m, max(position_at_0_percent_m, target))

    if not math.isfinite(target):
        raise ValueError('position must be a finite number')

    lower = defaults.position_at_0_percent_m
    upper = defaults.position_at_100_percent_m
    if target < lower or target > upper:
        raise ValueError(
            f'position for {profile.robot_name} must be between '
            f'{lower:.10g} m and {upper:.10g} m, got {target:.10g} m'
        )
    return target


def command_preview(
    profile: GripperCommandProfile,
    defaults: GripperCommandDefaults,
    position: float,
    *,
    times: int,
    rate_hz: float,
    command: str | None = None,
    percentage: float | None = None,
    percentage_is_default: bool = False,
    defaults_path: Path | None = None,
) -> str:
    rows = [
        f'Robot: {profile.robot_name} ({profile.model_name})',
        f'Topic: {profile.topic}',
        'Message type: std_msgs/msg/Float64',
    ]
    if defaults_path is not None:
        rows.append(f'Defaults file: {defaults_path}')
    if command is not None and percentage is not None:
        source = 'configured default' if percentage_is_default else 'explicit'
        rows.append(f'Action: {command} {percentage:.10g}% ({source})')
    rows.extend((
        f'Target position: {position:.10g} m',
        (
            'Allowed range: '
            f'[{defaults.position_at_0_percent_m:.10g}, '
            f'{defaults.position_at_100_percent_m:.10g}] m'
        ),
        f'Joint state topic: {profile.joint_state_topic}',
        f'Publish burst: times={times}, rate_hz={rate_hz:.10g}',
    ))
    return '\n'.join(rows)


def _validate_publish_options(
    *,
    times: int,
    rate_hz: float,
    wait_timeout_sec: float,
    ready_timeout_sec: float,
) -> None:
    if times < 1:
        raise ValueError('--times must be at least 1')
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError('--rate must be a finite number greater than zero')
    if not math.isfinite(wait_timeout_sec) or wait_timeout_sec < 0.0:
        raise ValueError('--wait-timeout must be a finite number zero or greater')
    if not math.isfinite(ready_timeout_sec) or ready_timeout_sec < 0.0:
        raise ValueError('--ready-timeout must be a finite number zero or greater')


def publish_position(
    profile: GripperCommandProfile,
    position: float,
    *,
    times: int,
    rate_hz: float,
    wait_timeout_sec: float,
    ready_timeout_sec: float,
) -> None:
    _validate_publish_options(
        times=times,
        rate_hz=rate_hz,
        wait_timeout_sec=wait_timeout_sec,
        ready_timeout_sec=ready_timeout_sec,
    )

    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    rclpy.init(args=None)
    node = rclpy.create_node('mfja_robot_gripper_command')
    publisher = node.create_publisher(Float64, profile.topic, 10)
    try:
        if wait_timeout_sec > 0.0:
            deadline = time.monotonic() + wait_timeout_sec
            while (
                rclpy.ok()
                and publisher.get_subscription_count() == 0
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.1)

        if publisher.get_subscription_count() == 0:
            raise RuntimeError(
                f'no subscribers matched {profile.topic}. '
                'Launch the selected robot and its gripper bridge first.'
            )

        if ready_timeout_sec > 0.0:
            joint_state_seen = False

            def _on_joint_state(message: JointState) -> None:
                nonlocal joint_state_seen
                joint_state_seen = JAW_JOINT_NAMES.issubset(message.name)

            subscription = node.create_subscription(
                JointState,
                profile.joint_state_topic,
                _on_joint_state,
                10,
            )
            deadline = time.monotonic() + ready_timeout_sec
            while rclpy.ok() and not joint_state_seen and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_subscription(subscription)

            if not joint_state_seen:
                raise RuntimeError(
                    'no live gripper joint states received on '
                    f'{profile.joint_state_topic}. '
                    'Gazebo may be paused/stale, on a different GZ partition, '
                    'or the articulated gripper model is not launched.'
                )

        message = Float64()
        message.data = float(position)
        delay = 1.0 / rate_hz
        for index in range(times):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            if index < times - 1:
                time.sleep(delay)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Open, close, or position one of the four MFJA industrial grippers. '
            'Positions are configured per-jaw linear travel in meters.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Available robot selectors:\n' + profile_help(),
    )
    parser.add_argument(
        'robot',
        nargs='?',
        help='Robot selector such as kuka, staubli, hc10, or hc10dt.',
    )
    parser.add_argument(
        'command',
        nargs='?',
        metavar='{open,close}',
        help=(
            'Opening or closing action. With no percentage, the configured '
            'per-robot default is used.'
        ),
    )
    parser.add_argument(
        'percentage',
        nargs='?',
        type=float,
        metavar='PERCENT',
        help=(
            'Optional action percentage from 0 to 100. open 100 is fully open; '
            'close 100 is fully closed within the configured endpoints.'
        ),
    )
    parser.add_argument(
        '-p',
        '--position',
        type=float,
        metavar='METERS',
        help='Command a custom per-jaw position within the configured range.',
    )
    parser.add_argument(
        '--times',
        type=int,
        default=DEFAULT_PUBLISH_TIMES,
        help=(
            f'Number of publications in the command burst. Default: {DEFAULT_PUBLISH_TIMES}.'
        ),
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=DEFAULT_PUBLISH_RATE_HZ,
        help=f'Burst publication rate in Hz. Default: {DEFAULT_PUBLISH_RATE_HZ:.10g}.',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=DEFAULT_WAIT_TIMEOUT_SEC,
        help=(
            'Seconds to wait for a matching gripper command subscription. '
            f'Default: {DEFAULT_WAIT_TIMEOUT_SEC:.10g}.'
        ),
    )
    parser.add_argument(
        '--ready-timeout',
        type=float,
        default=DEFAULT_READY_TIMEOUT_SEC,
        help=(
            'Seconds to wait for live joint_states before publishing. '
            f'Default: {DEFAULT_READY_TIMEOUT_SEC:.10g}. Use 0 to skip this check.'
        ),
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the resolved target without importing ROS or publishing.',
    )
    parser.add_argument(
        '--defaults-file',
        type=Path,
        metavar='PATH',
        help=(
            f'Override the {DEFAULTS_FILENAME} path used for gripper endpoints '
            'and bare open/close defaults.'
        ),
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List supported robot selectors and their position ranges, then exit.',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        defaults_path: Path | None = None
        configured_defaults: dict[str, GripperCommandDefaults] | None = None
        if args.list:
            defaults_path = (
                args.defaults_file.expanduser()
                if args.defaults_file is not None
                else default_defaults_path()
            )
            configured_defaults = load_command_defaults(defaults_path)
            print(f'Defaults file: {defaults_path}')
            print(profile_help(configured_defaults))
            return 0

        if not args.robot:
            parser.error('robot is required unless --list is used')

        profile = resolve_profile(args.robot)
        normalized_command = (
            normalize_command(args.command)
            if args.command is not None
            else None
        )
        defaults_path = (
            args.defaults_file.expanduser()
            if args.defaults_file is not None
            else default_defaults_path()
        )
        configured_defaults = load_command_defaults(
            defaults_path,
            [profile.robot_name],
        )

        selected_defaults = configured_defaults[profile.robot_name]
        resolved_command: str | None = None
        resolved_percentage: float | None = None
        percentage_is_default = False
        if normalized_command is not None:
            (
                resolved_command,
                resolved_percentage,
                percentage_is_default,
            ) = resolve_command_percentage(
                normalized_command,
                args.percentage,
                selected_defaults,
            )

        position = resolve_target_position(
            profile,
            resolved_command,
            args.position,
            resolved_percentage,
            selected_defaults,
        )
        _validate_publish_options(
            times=args.times,
            rate_hz=args.rate,
            wait_timeout_sec=args.wait_timeout,
            ready_timeout_sec=args.ready_timeout,
        )

        if args.dry_run:
            print(
                command_preview(
                    profile,
                    selected_defaults,
                    position,
                    times=args.times,
                    rate_hz=args.rate,
                    command=resolved_command,
                    percentage=resolved_percentage,
                    percentage_is_default=percentage_is_default,
                    defaults_path=defaults_path,
                )
            )
            return 0

        publish_position(
            profile,
            position,
            times=args.times,
            rate_hz=args.rate,
            wait_timeout_sec=args.wait_timeout,
            ready_timeout_sec=args.ready_timeout,
        )
        print(
            command_preview(
                profile,
                selected_defaults,
                position,
                times=args.times,
                rate_hz=args.rate,
                command=resolved_command,
                percentage=resolved_percentage,
                percentage_is_default=percentage_is_default,
                defaults_path=defaults_path,
            )
        )
        return 0
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
