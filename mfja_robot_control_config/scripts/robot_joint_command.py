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


PROFILES = (
    JointCommandProfile(
        robot_name='kuka1',
        model_name='kuka_kr6r900sixx',
        joint_names=INDUSTRIAL_JOINTS_A,
        angular_joints=(True,) * 6,
        aliases=('1', 'kuka', 'kuka1', 'kuka_kr6r900sixx'),
        default_duration_sec=4.0,
    ),
    JointCommandProfile(
        robot_name='staubli1',
        model_name='staubli_tx2_60l',
        joint_names=STAUBLI_JOINTS,
        angular_joints=(True,) * 6,
        aliases=('2', 'staubli', 'staubli1', 'staubli_tx2_60l'),
    ),
    JointCommandProfile(
        robot_name='yaskawa_hc10_1',
        model_name='yaskawa_hc10',
        joint_names=YASKAWA_JOINTS,
        angular_joints=(True,) * 6,
        aliases=('3', 'hc10', 'yaskawa_hc10', 'yaskawa_hc10_1'),
    ),
    JointCommandProfile(
        robot_name='yaskawa_hc10dt_1',
        model_name='yaskawa_hc10dt',
        joint_names=YASKAWA_JOINTS,
        angular_joints=(True,) * 6,
        aliases=('4', 'hc10dt', 'yaskawa_hc10dt', 'yaskawa_hc10dt_1'),
    ),
    JointCommandProfile(
        robot_name='tiago1',
        model_name='tiago_with_arm',
        joint_names=TIAGO_WITH_ARM_JOINTS,
        angular_joints=(False, True, True, True, True, True, True, True, True, True),
        aliases=('5', 'tiago', 'tiago1', 'tiago_arm', 'tiago_with_arm'),
        default_duration_sec=4.0,
    ),
    JointCommandProfile(
        robot_name='tiago_base1',
        model_name='tiago_base',
        joint_names=TIAGO_BASE_JOINTS,
        angular_joints=(False,),
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
DEFAULT_PUBLISH_TIMES = 10
DEFAULT_PUBLISH_RATE_HZ = 10.0
DEFAULT_READY_TIMEOUT_SEC = 2.0


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

    if unit == 'rad':
        return [float(position) for position in positions]
    if unit != 'deg':
        raise ValueError('unit must be "rad" or "deg"')

    return [
        math.radians(float(position)) if is_angular else float(position)
        for position, is_angular in zip(positions, profile.angular_joints, strict=True)
    ]


def duration_to_sec_nanosec(duration_sec: float) -> tuple[int, int]:
    if duration_sec <= 0.0:
        raise ValueError('--duration must be greater than zero')

    sec = int(math.floor(duration_sec))
    nanosec = int(round((duration_sec - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def trajectory_preview(
    profile: JointCommandProfile,
    positions_rad_or_m: Sequence[float],
    duration_sec: float,
    *,
    times: int | None = None,
    rate_hz: float | None = None,
) -> str:
    sec, nanosec = duration_to_sec_nanosec(duration_sec)
    positions = ', '.join(f'{position:.10g}' for position in positions_rad_or_m)
    joints = ', '.join(f"'{joint_name}'" for joint_name in profile.joint_names)
    lines = [
        f'Topic: {profile.topic}',
        f'Joint state topic: {profile.joint_state_topic}',
        f'Joint names: [{joints}]',
        f'Published positions (rad for angular joints, m for linear joints): [{positions}]',
        f'Time from start: sec={sec}, nanosec={nanosec}',
    ]
    if times is not None and rate_hz is not None:
        lines.append(f'Publish burst: times={times}, rate_hz={rate_hz:.10g}')
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
    if times < 1:
        raise ValueError('--times must be at least 1')
    if rate_hz <= 0.0:
        raise ValueError('--rate must be greater than zero')
    if wait_timeout_sec < 0.0:
        raise ValueError('--wait-timeout must be zero or greater')
    if ready_timeout_sec < 0.0:
        raise ValueError('--ready-timeout must be zero or greater')

    import rclpy
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

        if ready_timeout_sec > 0.0:
            joint_state_seen = False

            def _on_joint_state(_message: JointState) -> None:
                nonlocal joint_state_seen
                joint_state_seen = True

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
                    f'no live joint states received on {profile.joint_state_topic}. '
                    'Gazebo may be paused/stale, on a different GZ partition, '
                    'or the robot model is not launched.'
                )

        sec, nanosec = duration_to_sec_nanosec(duration_sec)
        point = JointTrajectoryPoint()
        point.positions = list(positions_rad_or_m)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        message = JointTrajectory()
        message.joint_names = list(profile.joint_names)
        message.points = [point]

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
        default=DEFAULT_PUBLISH_TIMES,
        help=(
            f'How many times to publish the trajectory message. Default: {DEFAULT_PUBLISH_TIMES}, '
            'as a short burst so ros_gz/Gazebo controllers do not miss one-shot commands.'
        ),
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=DEFAULT_PUBLISH_RATE_HZ,
        help=(
            'Publish rate in Hz when --times is greater than 1. '
            f'Default: {DEFAULT_PUBLISH_RATE_HZ:.10g}.'
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
            'Seconds to wait for live joint_states before publishing. '
            f'Default: {DEFAULT_READY_TIMEOUT_SEC:.10g}. Use 0 to skip this check.'
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
                    times=args.times,
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
                times=args.times,
                rate_hz=args.rate,
            )
        )
        return 0
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
