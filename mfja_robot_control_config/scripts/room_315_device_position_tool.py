#!/usr/bin/env python3

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from room_315_kinematic_shuttle import CUBIC_HERMITE_PATH_BACKEND, Point3D, RailNetwork


DEVICE_CATEGORIES = {
    'slots',
    'position_sensors',
    'approach_sensors',
    'stoppers',
}

RIGHT_CALIBRATION_DEFAULTS = {
    'pose_transform_a': -0.893249246800,
    'pose_transform_b': 0.005839516878,
    'pose_transform_tx': -26.921427375871,
    'pose_transform_c': 0.001889497475,
    'pose_transform_d': 1.308619216904,
    'pose_transform_ty': 0.666926143808,
    'pose_transform_z_offset': 0.0,
    'pose_scale_x': 1.0,
    'pose_scale_y': 1.0,
    'pose_scale_origin_x': -15.855195431322,
    'pose_scale_origin_y': -4.525523413467,
    'pose_rotation_deg': 0.0,
    'pose_rotation_origin_x': -15.855195431322,
    'pose_rotation_origin_y': -4.525523413467,
    'pose_offset_x': 0.0,
    'pose_offset_y': 0.0,
    'pose_offset_z': 0.0,
}

LEFT_CALIBRATION_DEFAULTS = {
    'pose_transform_a': -0.8938584503560025,
    'pose_transform_b': 0.005001975618640809,
    'pose_transform_tx': -22.47198317328330,
    'pose_transform_c': 0.001348127530438647,
    'pose_transform_d': 1.255463611604302,
    'pose_transform_ty': 0.4431777232193935,
    'pose_transform_z_offset': 0.0,
    'pose_scale_x': 0.98,
    'pose_scale_y': 1.041,
    'pose_scale_origin_x': -10.6365565,
    'pose_scale_origin_y': -4.6995835,
    'pose_rotation_deg': 180.0,
    'pose_rotation_origin_x': -10.6365565,
    'pose_rotation_origin_y': -4.6995835,
    'pose_offset_x': 0.14,
    'pose_offset_y': 0.0,
    'pose_offset_z': 0.0,
}


@dataclass(frozen=True)
class ClosestRailPosition:
    segment: str
    s: float
    s_ratio: float
    distance_m: float
    raw_point: Point3D
    gazebo_point: tuple[float, float, float]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_kinematics'
        )
    except Exception:
        return (
            _repo_root()
            / 'mfja_robot_control_config'
            / 'config'
            / 'room_315_kinematics'
        )


def _calibration_for_side(side: str) -> dict[str, float]:
    return dict(LEFT_CALIBRATION_DEFAULTS if side == 'left' else RIGHT_CALIBRATION_DEFAULTS)


def _apply_planar_rotation(x: float, y: float, calibration: dict[str, float]) -> tuple[float, float]:
    rotation_rad = math.radians(calibration['pose_rotation_deg'])
    if abs(rotation_rad) <= 1e-12:
        return x, y

    dx = x - calibration['pose_rotation_origin_x']
    dy = y - calibration['pose_rotation_origin_y']
    cos_theta = math.cos(rotation_rad)
    sin_theta = math.sin(rotation_rad)
    return (
        calibration['pose_rotation_origin_x'] + cos_theta * dx - sin_theta * dy,
        calibration['pose_rotation_origin_y'] + sin_theta * dx + cos_theta * dy,
    )


def _to_gazebo_point(
    point: Point3D,
    calibration: dict[str, float],
) -> tuple[float, float, float]:
    base_x = (
        calibration['pose_transform_a'] * point.x
        + calibration['pose_transform_b'] * point.y
        + calibration['pose_transform_tx']
    )
    base_y = (
        calibration['pose_transform_c'] * point.x
        + calibration['pose_transform_d'] * point.y
        + calibration['pose_transform_ty']
    )
    scaled_x = (
        calibration['pose_scale_origin_x']
        + (base_x - calibration['pose_scale_origin_x']) * calibration['pose_scale_x']
    )
    scaled_y = (
        calibration['pose_scale_origin_y']
        + (base_y - calibration['pose_scale_origin_y']) * calibration['pose_scale_y']
    )
    rotated_x, rotated_y = _apply_planar_rotation(scaled_x, scaled_y, calibration)
    return (
        rotated_x + calibration['pose_offset_x'],
        rotated_y + calibration['pose_offset_y'],
        point.z + calibration['pose_transform_z_offset'] + calibration['pose_offset_z'],
    )


def _distance_to_target(
    network: RailNetwork,
    segment_name: str,
    s: float,
    target_xyz: tuple[float, float, float],
    calibration: dict[str, float],
) -> tuple[float, Point3D, tuple[float, float, float]]:
    raw_point, _yaw = network.segments[segment_name].sample(s)
    gazebo_point = _to_gazebo_point(raw_point, calibration)
    return math.dist(target_xyz, gazebo_point), raw_point, gazebo_point


def _closest_on_segment(
    network: RailNetwork,
    segment_name: str,
    target_xyz: tuple[float, float, float],
    calibration: dict[str, float],
    sample_step_m: float,
) -> ClosestRailPosition:
    segment = network.segments[segment_name]
    step = max(sample_step_m, 1e-4)
    sample_count = max(2, int(math.ceil(segment.length / step)) + 1)
    best_s = 0.0
    best_distance = math.inf
    best_raw_point = segment.points[0]
    best_gazebo_point = _to_gazebo_point(best_raw_point, calibration)

    for index in range(sample_count):
        s = min(segment.length, index * segment.length / (sample_count - 1))
        distance, raw_point, gazebo_point = _distance_to_target(
            network,
            segment_name,
            s,
            target_xyz,
            calibration,
        )
        if distance < best_distance:
            best_s = s
            best_distance = distance
            best_raw_point = raw_point
            best_gazebo_point = gazebo_point

    low = max(0.0, best_s - step)
    high = min(segment.length, best_s + step)
    for _ in range(40):
        first = low + (high - low) / 3.0
        second = high - (high - low) / 3.0
        first_distance, _first_raw, _first_gazebo = _distance_to_target(
            network,
            segment_name,
            first,
            target_xyz,
            calibration,
        )
        second_distance, _second_raw, _second_gazebo = _distance_to_target(
            network,
            segment_name,
            second,
            target_xyz,
            calibration,
        )
        if first_distance < second_distance:
            high = second
        else:
            low = first

    refined_s = 0.5 * (low + high)
    best_distance, best_raw_point, best_gazebo_point = _distance_to_target(
        network,
        segment_name,
        refined_s,
        target_xyz,
        calibration,
    )
    return ClosestRailPosition(
        segment=segment_name,
        s=refined_s,
        s_ratio=refined_s / segment.length,
        distance_m=best_distance,
        raw_point=best_raw_point,
        gazebo_point=best_gazebo_point,
    )


def closest_rail_position(
    network: RailNetwork,
    target_xyz: tuple[float, float, float],
    calibration: dict[str, float],
    sample_step_m: float,
) -> ClosestRailPosition:
    best: ClosestRailPosition | None = None
    for segment_name in network.segments:
        candidate = _closest_on_segment(
            network,
            segment_name,
            target_xyz,
            calibration,
            sample_step_m,
        )
        if best is None or candidate.distance_m < best.distance_m:
            best = candidate

    if best is None:
        raise RuntimeError('Rail network contains no segments.')
    return best


def _canonical_name(category: str, raw_name: str) -> str:
    name = str(raw_name).strip()
    if category == 'slots':
        lowered = name.lower().replace('-', '_')
        for prefix in ('start_slot_', 'start_', 'slot_'):
            if lowered.startswith(prefix):
                lowered = lowered[len(prefix):]
                break
        return lowered
    return name.upper()


def _device_entries(config: dict[str, Any], category: str) -> list[tuple[str, dict[str, Any]]]:
    raw_category = config.get(category)
    if not isinstance(raw_category, list):
        raise ValueError(f'{category} must be a YAML list for this updater.')

    entries = []
    for index, entry in enumerate(raw_category):
        if not isinstance(entry, dict):
            raise ValueError(f'{category}[{index}] must be a mapping.')
        if 'name' not in entry:
            raise ValueError(f'{category}[{index}] is missing name.')
        entries.append((str(entry['name']), entry))
    return entries


def _find_device_entry(
    config: dict[str, Any],
    category: str,
    name: str,
) -> dict[str, Any]:
    target_name = _canonical_name(category, name)
    for raw_name, entry in _device_entries(config, category):
        if _canonical_name(category, raw_name) == target_name:
            return entry
    raise ValueError(f'Could not find {category}.{name} in devices YAML.')


def _find_device_entry_auto(
    config: dict[str, Any],
    name: str,
) -> tuple[str, dict[str, Any]]:
    matches: list[tuple[str, dict[str, Any]]] = []
    for category in sorted(DEVICE_CATEGORIES):
        target_name = _canonical_name(category, name)
        for raw_name, entry in _device_entries(config, category):
            if _canonical_name(category, raw_name) == target_name:
                matches.append((category, entry))

    if not matches:
        raise ValueError(f'Could not find {name} in devices YAML.')
    if len(matches) > 1:
        categories = ', '.join(category for category, _entry in matches)
        raise ValueError(
            f'Found {name} in multiple device groups: {categories}. '
            'Use --category to choose one explicitly.'
        )
    return matches[0]


def _target_mapping_for_update(
    entry: dict[str, Any],
    point_index: int,
) -> dict[str, Any]:
    if 'points' not in entry:
        if point_index != 0:
            raise ValueError('point-index can only be non-zero when the device has points.')
        return entry

    points = entry['points']
    if not isinstance(points, list) or not points:
        raise ValueError('points must be a non-empty list.')
    if point_index < 0 or point_index >= len(points):
        raise ValueError(f'point-index={point_index} is outside 0..{len(points) - 1}.')
    if not isinstance(points[point_index], dict):
        raise ValueError(f'points[{point_index}] must be a mapping.')
    return points[point_index]


def update_device_yaml(
    devices_path: Path,
    category: str | None,
    name: str,
    point_index: int,
    closest: ClosestRailPosition,
) -> tuple[str, dict[str, Any], dict[str, Any]]:
    with devices_path.open() as handle:
        config = yaml.safe_load(handle) or {}
    if not isinstance(config, dict):
        raise ValueError(f'{devices_path} must contain a YAML mapping.')

    if category is None:
        category, entry = _find_device_entry_auto(config, name)
    else:
        entry = _find_device_entry(config, category, name)
    target = _target_mapping_for_update(entry, point_index)
    previous = {
        'segment': target.get('segment'),
        's_ratio': target.get('s_ratio'),
    }
    target['segment'] = closest.segment
    target['s_ratio'] = round(closest.s_ratio, 9)
    with devices_path.open('w') as handle:
        yaml.safe_dump(
            config,
            handle,
            sort_keys=False,
            default_flow_style=False,
            allow_unicode=False,
        )
    return category, previous, {'segment': target['segment'], 's_ratio': target['s_ratio']}


def parse_args() -> argparse.Namespace:
    config_dir = _default_config_dir()
    parser = argparse.ArgumentParser(
        description=(
            'Convert a Gazebo XYZ point to Room 315 rail segment+s_ratio, '
            'optionally updating a rail_devices_*.yaml entry.'
        )
    )
    parser.add_argument('--side', choices=['right', 'left'], required=True)
    parser.add_argument('--x', type=float, required=True, help='Gazebo X coordinate.')
    parser.add_argument('--y', type=float, required=True, help='Gazebo Y coordinate.')
    parser.add_argument('--z', type=float, required=True, help='Gazebo Z coordinate.')
    parser.add_argument(
        '--category',
        choices=sorted(DEVICE_CATEGORIES),
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument('--name', help='Device name to update, for example DZI1R.')
    parser.add_argument(
        '--point-index',
        type=int,
        default=0,
        help='Index under points: for multi-point devices.',
    )
    parser.add_argument('--network', type=Path)
    parser.add_argument('--devices', type=Path)
    parser.add_argument('--write', action='store_true', help='Update the device YAML file.')
    parser.add_argument(
        '--force',
        action='store_true',
        help='Allow --write even when the Gazebo point is far from the rail.',
    )
    parser.add_argument('--max-distance-m', type=float, default=0.20)
    parser.add_argument('--sample-step-m', type=float, default=0.01)
    parser.add_argument('--path-backend', default=CUBIC_HERMITE_PATH_BACKEND)
    parser.add_argument('--arc-length-samples-per-edge', type=int, default=16)
    parser.add_argument('--pose-offset-x', type=float)
    parser.add_argument('--pose-offset-y', type=float)
    parser.add_argument('--pose-offset-z', type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    network_path = (
        args.network
        if args.network is not None
        else _default_config_dir() / f'rail_network_{args.side}.yaml'
    ).resolve()
    devices_path = (
        args.devices
        if args.devices is not None
        else _default_config_dir() / f'rail_devices_{args.side}.yaml'
    ).resolve()
    calibration = _calibration_for_side(args.side)
    for key, value in (
        ('pose_offset_x', args.pose_offset_x),
        ('pose_offset_y', args.pose_offset_y),
        ('pose_offset_z', args.pose_offset_z),
    ):
        if value is not None:
            calibration[key] = value

    network = RailNetwork.from_yaml(
        network_path,
        path_backend=args.path_backend,
        arc_length_samples_per_edge=args.arc_length_samples_per_edge,
    )
    target_xyz = (args.x, args.y, args.z)
    closest = closest_rail_position(
        network,
        target_xyz,
        calibration,
        sample_step_m=args.sample_step_m,
    )

    print(f'Side: {args.side}')
    print(f'Target Gazebo XYZ: x={args.x:.6f}, y={args.y:.6f}, z={args.z:.6f}')
    print(
        'Closest rail position: '
        f'segment={closest.segment}, '
        f's={closest.s:.6f}, '
        f's_ratio={closest.s_ratio:.9f}, '
        f'distance_m={closest.distance_m:.6f}'
    )
    print(
        'Projected Gazebo XYZ: '
        f'x={closest.gazebo_point[0]:.6f}, '
        f'y={closest.gazebo_point[1]:.6f}, '
        f'z={closest.gazebo_point[2]:.6f}'
    )

    too_far = closest.distance_m > args.max_distance_m
    if too_far:
        print(
            f'WARN: target is {closest.distance_m:.3f} m from the nearest rail, '
            f'above max-distance-m={args.max_distance_m:.3f}.'
        )

    if not args.write:
        print('Dry run only. Add --write with --name to update rail_devices YAML.')
        return 0 if not too_far else 2

    if not args.name:
        print('FAIL: --write requires --name.', file=sys.stderr)
        return 1
    if too_far and not args.force:
        print('FAIL: refusing to write a far target; use --force to override.', file=sys.stderr)
        return 1

    _category, previous, updated = update_device_yaml(
        devices_path=devices_path,
        category=args.category,
        name=args.name,
        point_index=args.point_index,
        closest=closest,
    )
    print(f'Updated: {devices_path}')
    print(
        f'{args.name}: '
        f'{previous["segment"]}@{previous["s_ratio"]} -> '
        f'{updated["segment"]}@{updated["s_ratio"]}'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
