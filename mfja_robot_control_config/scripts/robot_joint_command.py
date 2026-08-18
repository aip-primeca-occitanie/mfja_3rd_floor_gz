#!/usr/bin/env python3

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class JointCommandProfile:
    robot_name: str
    model_name: str
    joint_names: tuple[str, ...]
    angular_joints: tuple[bool, ...]
    joint_limits: tuple[tuple[float, float], ...]
    aliases: tuple[str, ...]
    default_duration_sec: float = 3.0

    @property
    def topic(self) -> str:
        return f'/{self.robot_name}/joint_trajectory'

    @property
    def joint_state_topic(self) -> str:
        return f'/{self.robot_name}/joint_states'


INDUSTRIAL_JOINTS_A = (
    'joint_a1',
    'joint_a2',
    'joint_a3',
    'joint_a4',
    'joint_a5',
    'joint_a6',
)
STAUBLI_JOINTS = (
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
)
YASKAWA_JOINTS = (
    'joint_1_s',
    'joint_2_l',
    'joint_3_u',
    'joint_4_r',
    'joint_5_b',
    'joint_6_t',
)
TIAGO_WITH_ARM_JOINTS = (
    'torso_lift_joint',
    'arm_1_joint',
    'arm_2_joint',
    'arm_3_joint',
    'arm_4_joint',
    'arm_5_joint',
    'arm_6_joint',
    'arm_7_joint',
    'head_1_joint',
    'head_2_joint',
)
TIAGO_BASE_JOINTS = ('torso_lift_joint',)

# These limits mirror the corresponding <axis><limit> values in the model SDFs.
# Angular limits are radians and linear limits are metres.
KUKA_JOINT_LIMITS = (
    (-2.96706, 2.96706),
    (-3.31613, 0.7854),
    (-2.0944, 2.72271),
    (-3.22886, 3.22886),
    (-2.0944, 2.0944),
    (-6.10865, 6.10865),
)
STAUBLI_JOINT_LIMITS = (
    (-3.14159265359, 3.14159265359),
    (-2.22529479629, 2.22529479629),
    (-2.66162710929, 2.66162710929),
    (-4.71238898038, 4.71238898038),
    (-2.11184839491, 2.31256125889),
    (-4.71238898038, 4.71238898038),
)
YASKAWA_JOINT_LIMITS = (
    (-3.14159265359, 3.14159265359),
    (-3.14159265359, 3.14159265359),
    (-0.08726646260, 6.19591884458),
    (-3.14159265359, 3.14159265359),
    (-3.14159265359, 3.14159265359),
    (-3.14159265359, 3.14159265359),
)
TIAGO_WITH_ARM_JOINT_LIMITS = (
    (0.0, 0.35),
    (0.0, 2.74889357189),
    (-1.57079632679, 1.0908307825),
    (-3.53429173529, 1.57079632679),
    (-0.3926990817, 2.35619449019),
    (-2.09439510239, 2.09439510239),
    (-1.41371669412, 1.41371669412),
    (-2.09439510239, 2.09439510239),
    (-1.30899693899, 1.30899693899),
    (-1.0471975512, 0.7853981634),
)
TIAGO_BASE_JOINT_LIMITS = ((0.0, 0.35),)


PROFILES = (
    JointCommandProfile(
        robot_name='kuka1',
        model_name='kuka_kr6r900sixx',
        joint_names=INDUSTRIAL_JOINTS_A,
        angular_joints=(True,) * 6,
        joint_limits=KUKA_JOINT_LIMITS,
        aliases=('1', 'kuka', 'kuka1', 'kuka_kr6r900sixx'),
        default_duration_sec=4.0,
    ),
    JointCommandProfile(
        robot_name='staubli1',
        model_name='staubli_tx2_60l',
        joint_names=STAUBLI_JOINTS,
        angular_joints=(True,) * 6,
        joint_limits=STAUBLI_JOINT_LIMITS,
        aliases=('2', 'staubli', 'staubli1', 'staubli_tx2_60l'),
    ),
    JointCommandProfile(
        robot_name='yaskawa_hc10_1',
        model_name='yaskawa_hc10',
        joint_names=YASKAWA_JOINTS,
        angular_joints=(True,) * 6,
        joint_limits=YASKAWA_JOINT_LIMITS,
        aliases=('3', 'hc10', 'yaskawa_hc10', 'yaskawa_hc10_1'),
    ),
    JointCommandProfile(
        robot_name='yaskawa_hc10dt_1',
        model_name='yaskawa_hc10dt',
        joint_names=YASKAWA_JOINTS,
        angular_joints=(True,) * 6,
        joint_limits=YASKAWA_JOINT_LIMITS,
        aliases=('4', 'hc10dt', 'yaskawa_hc10dt', 'yaskawa_hc10dt_1'),
    ),
    JointCommandProfile(
        robot_name='tiago1',
        model_name='tiago_with_arm',
        joint_names=TIAGO_WITH_ARM_JOINTS,
        angular_joints=(False, True, True, True, True, True, True, True, True, True),
        joint_limits=TIAGO_WITH_ARM_JOINT_LIMITS,
        aliases=('5', 'tiago', 'tiago1', 'tiago_arm', 'tiago_with_arm'),
        default_duration_sec=4.0,
    ),
    JointCommandProfile(
        robot_name='tiago_base1',
        model_name='tiago_base',
        joint_names=TIAGO_BASE_JOINTS,
        angular_joints=(False,),
        joint_limits=TIAGO_BASE_JOINT_LIMITS,
        aliases=('6', 'tiago_base', 'tiago_base1', 'tiago_mobile_base', 'tiago_no_arm'),
    ),
)

UNIT_ALIASES = {
    'rad': 'rad',
    'radian': 'rad',
    'radians': 'rad',
    'deg': 'deg',
    'degree': 'deg',
    'degrees': 'deg',
}
DEFAULT_PUBLISH_TIMES = 1
DEFAULT_TRAJECTORY_RATE_HZ = 100.0
MIN_TRAJECTORY_RATE_HZ = 100.0
MAX_TRAJECTORY_RATE_HZ = 200.0
MIN_TRAJECTORY_DURATION_SEC = 1.0 / MAX_TRAJECTORY_RATE_HZ
DEFAULT_READY_TIMEOUT_SEC = 2.0


@dataclass(frozen=True)
class TrajectorySample:
    time_from_start_sec: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]


def _selector_map() -> dict[str, JointCommandProfile]:
    selectors: dict[str, JointCommandProfile] = {}
    for profile in PROFILES:
        for alias in profile.aliases:
            selectors[alias.lower()] = profile
    return selectors


def profile_help() -> str:
    rows = []
    for profile in PROFILES:
        selector = profile.aliases[0]
        aliases = ', '.join(profile.aliases[1:])
        rows.append(
            f'{selector}: {profile.robot_name} ({profile.model_name}) '
            f'joints={len(profile.joint_names)} aliases={aliases}'
        )
    return '\n'.join(rows)


def resolve_profile(selector: str) -> JointCommandProfile:
    normalized = selector.strip().lower()
    try:
        return _selector_map()[normalized]
    except KeyError as exc:
        raise ValueError(
            f'unknown robot selector "{selector}". Available selectors:\n{profile_help()}'
        ) from exc


def _resolve_unit_alias(unit: str) -> str:
    normalized = unit.strip().lower()
    try:
        return UNIT_ALIASES[normalized]
    except KeyError as exc:
        raise ValueError('unit must be rad/radian/radians or deg/degree/degrees') from exc


def normalize_unit(unit: str | None, *, degrees: bool = False, radians: bool = False) -> str:
    if degrees and radians:
        raise ValueError('use only one of --degrees or --radians')

    explicit_unit = _resolve_unit_alias(unit) if unit is not None else None
    if degrees:
        if explicit_unit not in (None, 'deg'):
            raise ValueError('do not combine --degrees with a radian --unit value')
        return 'deg'
    if radians:
        if explicit_unit not in (None, 'rad'):
            raise ValueError('do not combine --radians with a degree --unit value')
        return 'rad'
    return explicit_unit or 'rad'


def converted_positions(
    profile: JointCommandProfile,
    positions: Sequence[float],
    unit: str,
) -> list[float]:
    if len(positions) != len(profile.joint_names):
        raise ValueError(
            f'{profile.robot_name} expects {len(profile.joint_names)} position values '
            f'for joints {list(profile.joint_names)}, got {len(positions)}'
        )

    if unit not in ('rad', 'deg'):
        raise ValueError('unit must be "rad" or "deg"')

    converted = [
        math.radians(float(position)) if unit == 'deg' and is_angular else float(position)
        for position, is_angular in zip(positions, profile.angular_joints, strict=True)
    ]
    return validate_positions(profile, converted, value_label='target')


def validate_positions(
    profile: JointCommandProfile,
    positions: Sequence[float],
    *,
    value_label: str,
    limit_tolerance: float = 0.0,
) -> list[float]:
    """Validate and normalize positions in controller units and profile order."""
    expected = len(profile.joint_names)
    if len(positions) != expected:
        raise ValueError(
            f'{value_label} for {profile.robot_name} must contain {expected} values, '
            f'got {len(positions)}'
        )
    if len(profile.angular_joints) != expected or len(profile.joint_limits) != expected:
        raise ValueError(f'invalid joint metadata for profile {profile.robot_name}')
    if not math.isfinite(limit_tolerance) or limit_tolerance < 0.0:
        raise ValueError('limit tolerance must be a finite non-negative number')

    validated = []
    for joint_name, position, is_angular, limits in zip(
        profile.joint_names,
        positions,
        profile.angular_joints,
        profile.joint_limits,
        strict=True,
    ):
        value = float(position)
        if not math.isfinite(value):
            raise ValueError(
                f'{value_label} for joint {joint_name} must be finite, got {value!r}'
            )
        lower, upper = limits
        if value < lower - limit_tolerance or value > upper + limit_tolerance:
            unit = 'rad' if is_angular else 'm'
            raise ValueError(
                f'{value_label} for joint {joint_name} is {value:.10g} {unit}; '
                f'allowed range is [{lower:.10g}, {upper:.10g}] {unit}'
            )
        # Joint-state publishers can report tiny solver overshoots at a hard limit.
        validated.append(min(max(value, lower), upper))
    return validated


def positions_from_joint_state(
    profile: JointCommandProfile,
    names: Sequence[str],
    positions: Sequence[float],
) -> list[float]:
    """Return live positions in profile order, independent of JointState ordering."""
    if len(names) != len(positions):
        raise ValueError(
            f'joint state has {len(names)} names but {len(positions)} positions'
        )

    by_name: dict[str, float] = {}
    duplicate_names: set[str] = set()
    for name, position in zip(names, positions, strict=True):
        if name in by_name:
            duplicate_names.add(name)
        by_name[name] = float(position)
    if duplicate_names:
        raise ValueError(f'joint state contains duplicate names: {sorted(duplicate_names)}')

    missing = [joint_name for joint_name in profile.joint_names if joint_name not in by_name]
    if missing:
        raise ValueError(
            f'joint state on {profile.joint_state_topic} is missing joints: {missing}'
        )
    ordered = [by_name[joint_name] for joint_name in profile.joint_names]
    return validate_positions(
        profile,
        ordered,
        value_label='live position',
        limit_tolerance=1e-6,
    )


def duration_to_sec_nanosec(duration_sec: float) -> tuple[int, int]:
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        raise ValueError('--duration must be a finite number greater than zero')

    return nonnegative_seconds_to_sec_nanosec(duration_sec)


def nonnegative_seconds_to_sec_nanosec(value_sec: float) -> tuple[int, int]:
    if not math.isfinite(value_sec) or value_sec < 0.0:
        raise ValueError('trajectory timestamp must be a finite non-negative number')

    sec = int(math.floor(value_sec))
    nanosec = int(round((value_sec - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def validate_trajectory_rate(rate_hz: float) -> float:
    rate = float(rate_hz)
    if not math.isfinite(rate):
        raise ValueError('--rate must be finite')
    if rate < MIN_TRAJECTORY_RATE_HZ or rate > MAX_TRAJECTORY_RATE_HZ:
        raise ValueError(
            f'--rate must be between {MIN_TRAJECTORY_RATE_HZ:.10g} and '
            f'{MAX_TRAJECTORY_RATE_HZ:.10g} Hz'
        )
    return rate


def trajectory_interval_count(duration_sec: float, rate_hz: float) -> int:
    duration_to_sec_nanosec(duration_sec)
    rate = validate_trajectory_rate(rate_hz)
    if duration_sec < MIN_TRAJECTORY_DURATION_SEC:
        raise ValueError(
            f'--duration must be at least {MIN_TRAJECTORY_DURATION_SEC:.10g} seconds '
            'for a 100-200 Hz trajectory'
        )

    minimum_intervals = max(
        1,
        int(math.ceil(duration_sec * MIN_TRAJECTORY_RATE_HZ - 1e-12)),
    )
    maximum_intervals = max(
        1,
        int(math.floor(duration_sec * MAX_TRAJECTORY_RATE_HZ + 1e-12)),
    )
    desired_intervals = max(1, int(round(duration_sec * rate)))
    return min(max(desired_intervals, minimum_intervals), maximum_intervals)


def sampled_quintic_trajectory(
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    duration_sec: float,
    rate_hz: float = DEFAULT_TRAJECTORY_RATE_HZ,
) -> list[TrajectorySample]:
    """Pre-sample a minimum-jerk quintic trajectory for Gazebo's controller."""
    interval_count = trajectory_interval_count(duration_sec, rate_hz)
    if len(start_positions) != len(target_positions):
        raise ValueError('start and target positions must have the same length')
    if not start_positions:
        raise ValueError('trajectory must contain at least one joint')

    start = tuple(float(value) for value in start_positions)
    target = tuple(float(value) for value in target_positions)
    if not all(math.isfinite(value) for value in start + target):
        raise ValueError('start and target positions must be finite')

    deltas = tuple(end - begin for begin, end in zip(start, target, strict=True))
    samples = []
    for index in range(interval_count + 1):
        time_sec = duration_sec * index / interval_count
        phase = index / interval_count
        phase2 = phase * phase
        phase3 = phase2 * phase
        phase4 = phase3 * phase
        phase5 = phase4 * phase
        blend = 10.0 * phase3 - 15.0 * phase4 + 6.0 * phase5
        blend_velocity = (
            30.0 * phase2 - 60.0 * phase3 + 30.0 * phase4
        ) / duration_sec
        blend_acceleration = (
            60.0 * phase - 180.0 * phase2 + 120.0 * phase3
        ) / (duration_sec * duration_sec)

        # Preserve exact boundary values instead of accumulating floating-point error.
        positions_at_time = (
            start
            if index == 0
            else target
            if index == interval_count
            else tuple(
                begin + delta * blend
                for begin, delta in zip(start, deltas, strict=True)
            )
        )
        samples.append(
            TrajectorySample(
                time_from_start_sec=time_sec,
                positions=positions_at_time,
                velocities=tuple(delta * blend_velocity for delta in deltas),
                accelerations=tuple(delta * blend_acceleration for delta in deltas),
            )
        )
    return samples


def build_trajectory_message(
    profile: JointCommandProfile,
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    duration_sec: float,
    rate_hz: float = DEFAULT_TRAJECTORY_RATE_HZ,
):
    """Build one dense ROS JointTrajectory message from a measured start state."""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    start = validate_positions(
        profile,
        start_positions,
        value_label='live position',
        limit_tolerance=1e-6,
    )
    target = validate_positions(profile, target_positions, value_label='target')
    samples = sampled_quintic_trajectory(start, target, duration_sec, rate_hz)

    message = JointTrajectory()
    message.joint_names = list(profile.joint_names)
    for sample in samples:
        point = JointTrajectoryPoint()
        point.positions = list(sample.positions)
        point.velocities = list(sample.velocities)
        point.accelerations = list(sample.accelerations)
        sec, nanosec = nonnegative_seconds_to_sec_nanosec(sample.time_from_start_sec)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        message.points.append(point)
    return message


def trajectory_preview(
    profile: JointCommandProfile,
    positions_rad_or_m: Sequence[float],
    duration_sec: float,
    *,
    rate_hz: float = DEFAULT_TRAJECTORY_RATE_HZ,
) -> str:
    sec, nanosec = duration_to_sec_nanosec(duration_sec)
    rate = validate_trajectory_rate(rate_hz)
    point_count = trajectory_interval_count(duration_sec, rate) + 1
    target = validate_positions(profile, positions_rad_or_m, value_label='target')
    positions = ', '.join(f'{position:.10g}' for position in target)
    joints = ', '.join(f"'{joint_name}'" for joint_name in profile.joint_names)
    lines = [
        f'Topic: {profile.topic}',
        f'Joint state topic: {profile.joint_state_topic}',
        f'Joint names: [{joints}]',
        f'Target positions (rad for angular joints, m for linear joints): [{positions}]',
        f'Duration: sec={sec}, nanosec={nanosec}',
        f'Smooth trajectory: quintic, points={point_count}, rate_hz={rate:.10g}',
        'Start positions: read live by joint name from joint_states',
        'Publication count: 1',
    ]
    return '\n'.join(lines)


def publish_trajectory(
    profile: JointCommandProfile,
    positions_rad_or_m: Sequence[float],
    duration_sec: float,
    *,
    times: int,
    rate_hz: float,
    wait_timeout_sec: float,
    ready_timeout_sec: float,
) -> None:
    if times != 1:
        raise ValueError('--times must be 1; a smooth trajectory is published exactly once')
    trajectory_interval_count(duration_sec, rate_hz)
    if not math.isfinite(wait_timeout_sec) or wait_timeout_sec < 0.0:
        raise ValueError('--wait-timeout must be a finite number zero or greater')
    if not math.isfinite(ready_timeout_sec) or ready_timeout_sec <= 0.0:
        raise ValueError('--ready-timeout must be a finite number greater than zero')
    target_positions = validate_positions(
        profile,
        positions_rad_or_m,
        value_label='target',
    )

    import rclpy
    from rclpy.duration import Duration
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init(args=None)
    node = rclpy.create_node('mfja_robot_joint_command')
    publisher = node.create_publisher(JointTrajectory, profile.topic, 10)
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
                f'Launch the robot bridge/model first, or select robots:=all.'
            )

        current_positions: list[float] | None = None
        joint_state_error: str | None = None

        def _on_joint_state(message: JointState) -> None:
            nonlocal current_positions, joint_state_error
            try:
                current_positions = positions_from_joint_state(
                    profile,
                    message.name,
                    message.position,
                )
                joint_state_error = None
            except ValueError as exc:
                joint_state_error = str(exc)

        subscription = node.create_subscription(
            JointState,
            profile.joint_state_topic,
            _on_joint_state,
            10,
        )
        deadline = time.monotonic() + ready_timeout_sec
        while rclpy.ok() and current_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(subscription)

        if current_positions is None:
            detail = f' Last invalid state: {joint_state_error}' if joint_state_error else ''
            raise RuntimeError(
                f'no complete live joint state received on {profile.joint_state_topic}. '
                'Gazebo may be paused/stale, on a different GZ partition, '
                f'or the robot model is not launched.{detail}'
            )

        message = build_trajectory_message(
            profile,
            current_positions,
            target_positions,
            duration_sec,
            rate_hz,
        )
        publisher.publish(message)
        if not publisher.wait_for_all_acked(Duration(seconds=1.0)):
            raise RuntimeError(
                f'timed out waiting for the bridge to acknowledge {profile.topic}'
            )
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Publish a JointTrajectory command to any configured MFJA robot arm. '
            'Use --unit deg for degree input; angular joints are converted to radians.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Available robot selectors:\n' + profile_help(),
    )
    parser.add_argument(
        'robot',
        nargs='?',
        help='Robot selector such as kuka, staubli, hc10, hc10dt, tiago1, or tiago_base1.',
    )
    parser.add_argument(
        '-p',
        '--positions',
        nargs='+',
        type=float,
        help=(
            'Joint positions in profile order. Values are radians by default, or degrees '
            'with --unit deg/--degrees. Linear joints, such as torso_lift_joint, stay in meters.'
        ),
    )
    parser.add_argument(
        '--unit',
        default=None,
        help='Input unit for angular joints: rad/radian/radians or deg/degree/degrees.',
    )
    parser.add_argument(
        '--degrees',
        '--deg',
        action='store_true',
        help='Shortcut for --unit deg.',
    )
    parser.add_argument(
        '--radians',
        '--rad',
        action='store_true',
        help='Shortcut for --unit rad.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        help='Trajectory time_from_start in seconds. Defaults to the robot profile value.',
    )
    parser.add_argument(
        '--times',
        type=int,
        choices=(1,),
        default=DEFAULT_PUBLISH_TIMES,
        help=(
            'Compatibility option; must be 1. The complete smooth trajectory is published once.'
        ),
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=DEFAULT_TRAJECTORY_RATE_HZ,
        help=(
            'Pre-sampling rate for the smooth trajectory in Hz; must be between '
            f'{MIN_TRAJECTORY_RATE_HZ:.10g} and {MAX_TRAJECTORY_RATE_HZ:.10g}. '
            f'Default: {DEFAULT_TRAJECTORY_RATE_HZ:.10g}.'
        ),
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for a matching subscription before publishing. Default: 5.',
    )
    parser.add_argument(
        '--ready-timeout',
        type=float,
        default=DEFAULT_READY_TIMEOUT_SEC,
        help=(
            'Seconds to wait for a complete live joint state before publishing; must be positive. '
            f'Default: {DEFAULT_READY_TIMEOUT_SEC:.10g}.'
        ),
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the resolved trajectory without publishing.',
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List supported robot selectors and exit.',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list:
        print(profile_help())
        return 0

    if not args.robot:
        parser.error('robot is required unless --list is used')
    if args.positions is None:
        parser.error('--positions is required unless --list is used')

    try:
        profile = resolve_profile(args.robot)
        unit = normalize_unit(args.unit, degrees=args.degrees, radians=args.radians)
        duration_sec = args.duration if args.duration is not None else profile.default_duration_sec
        positions = converted_positions(profile, args.positions, unit)

        if args.dry_run:
            print(
                trajectory_preview(
                    profile,
                    positions,
                    duration_sec,
                    rate_hz=args.rate,
                )
            )
            return 0

        publish_trajectory(
            profile,
            positions,
            duration_sec,
            times=args.times,
            rate_hz=args.rate,
            wait_timeout_sec=args.wait_timeout,
            ready_timeout_sec=args.ready_timeout,
        )
        print(
            trajectory_preview(
                profile,
                positions,
                duration_sec,
                rate_hz=args.rate,
            )
        )
        return 0
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
