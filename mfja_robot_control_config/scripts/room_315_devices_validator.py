#!/usr/bin/env python3

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict

import yaml

from room_315_kinematic_shuttle import CUBIC_HERMITE_PATH_BACKEND, RailNetwork


DEVICE_CATEGORIES = (
    'slots',
    'position_sensors',
    'approach_sensors',
    'stoppers',
)

STOPPER_STATES = {
    '0',
    '1',
    'OFF',
    'ON',
    'OPEN',
    'CLOSED',
    'RELEASE',
    'STOP',
    'STOPPED',
    'BLOCK',
    'BLOCKED',
    'TRUE',
    'FALSE',
    'UNSTOP',
    'UNBLOCK',
}


@dataclass
class ValidationResult:
    side: str
    network_path: Path
    devices_path: Path
    counts: Dict[str, int] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def status(self) -> str:
        if self.errors:
            return 'FAIL'
        if self.warnings:
            return 'WARN'
        return 'PASS'


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
        pass

    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
    )


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


def _category_entries(config: dict, category: str) -> list[tuple[str, dict]]:
    raw_category = config.get(category)
    if raw_category is None:
        return []

    if isinstance(raw_category, list):
        entries = []
        for index, raw_entry in enumerate(raw_category):
            if not isinstance(raw_entry, dict):
                raise ValueError(
                    f'{category}[{index}] must be a mapping, got {type(raw_entry)!r}.'
                )
            if 'name' not in raw_entry:
                raise ValueError(f'{category}[{index}] is missing required field name.')
            entries.append((str(raw_entry['name']), raw_entry))
        return entries

    if isinstance(raw_category, dict):
        entries = []
        for raw_name, raw_entry in raw_category.items():
            if not isinstance(raw_entry, dict):
                raise ValueError(
                    f'{category}.{raw_name} must be a mapping, got {type(raw_entry)!r}.'
                )
            entries.append((str(raw_name), {'name': raw_name, **raw_entry}))
        return entries

    raise ValueError(f'{category} must be a list or mapping, got {type(raw_category)!r}.')


def _device_points(raw_entry: dict) -> list[dict]:
    if 'points' not in raw_entry:
        return [raw_entry]

    raw_points = raw_entry['points']
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError('points must be a non-empty list.')

    inherited = {
        key: value
        for key, value in raw_entry.items()
        if key not in {'points', 'segment', 's_ratio'}
    }
    points = []
    for raw_point in raw_points:
        if not isinstance(raw_point, dict):
            raise ValueError(f'points entries must be mappings, got {type(raw_point)!r}.')
        points.append({**inherited, **raw_point})
    return points


def _has_field(point: dict, field_name: str) -> bool:
    return field_name in point and point[field_name] is not None and str(point[field_name]) != ''


def _validate_positive_number(value, context: str, errors: list[str]) -> None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        errors.append(f'{context} must be numeric.')
        return

    if number <= 0.0:
        errors.append(f'{context} must be positive, got {number}.')


def _validate_s_ratio(value, context: str, errors: list[str]) -> None:
    try:
        s_ratio = float(value)
    except (TypeError, ValueError):
        errors.append(f'{context}.s_ratio must be numeric.')
        return

    if not 0.0 <= s_ratio <= 1.0:
        errors.append(f'{context}.s_ratio must be in [0.0, 1.0], got {s_ratio}.')


def _validate_stopper_state(value, context: str, errors: list[str]) -> None:
    state = str(value).strip().upper()
    if state not in STOPPER_STATES:
        errors.append(
            f'{context}.default_state={value!r} is invalid; use 0/open or 1/closed.'
        )


def validate_devices(
    *,
    side: str,
    network_path: Path,
    devices_path: Path,
    path_backend: str,
    arc_length_samples_per_edge: int,
) -> ValidationResult:
    result = ValidationResult(
        side=side,
        network_path=network_path,
        devices_path=devices_path,
    )

    try:
        network = RailNetwork.from_yaml(
            network_path,
            path_backend=path_backend,
            arc_length_samples_per_edge=arc_length_samples_per_edge,
        )
    except Exception as error:
        result.errors.append(f'Could not load network YAML {network_path}: {error}')
        return result

    try:
        with devices_path.open() as handle:
            config = yaml.safe_load(handle) or {}
    except Exception as error:
        result.errors.append(f'Could not load devices YAML {devices_path}: {error}')
        return result

    if not isinstance(config, dict):
        result.errors.append(f'{devices_path} must contain a YAML mapping.')
        return result

    declared_side = config.get('rail_side')
    if declared_side is not None and str(declared_side).strip().lower() != side:
        result.warnings.append(
            f'rail_side is {declared_side!r}, but this validation target is {side!r}.'
        )

    stopper_names = set()
    approach_targets = set()

    for category in DEVICE_CATEGORIES:
        try:
            entries = _category_entries(config, category)
        except ValueError as error:
            result.errors.append(str(error))
            result.counts[category] = 0
            continue

        result.counts[category] = len(entries)
        if not entries:
            result.errors.append(f'{category} must be defined and non-empty.')
            continue

        seen_names = set()
        for raw_name, raw_entry in entries:
            name_key = _canonical_name(category, raw_name)
            if not name_key:
                result.errors.append(f'{category} contains an empty device name.')
                continue
            if name_key in seen_names:
                result.errors.append(f'Duplicate {category} name {raw_name!r}.')
                continue
            seen_names.add(name_key)
            if category == 'stoppers':
                stopper_names.add(name_key)

            try:
                points = _device_points(raw_entry)
            except ValueError as error:
                result.errors.append(f'{category}.{raw_name}: {error}')
                continue

            for index, point in enumerate(points):
                context = f'{category}.{raw_name}'
                if len(points) > 1:
                    context += f'.points[{index}]'

                for field_name in ('segment', 's_ratio'):
                    if not _has_field(point, field_name):
                        result.errors.append(f'{context} is missing required field {field_name}.')

                if _has_field(point, 'segment') and str(point['segment']).strip() not in network.segments:
                    result.errors.append(
                        f'{context}.segment={point["segment"]!r} is not in {network_path.name}.'
                    )
                if _has_field(point, 's_ratio'):
                    _validate_s_ratio(point['s_ratio'], context, result.errors)

                if category == 'position_sensors':
                    if not _has_field(point, 'radius_m'):
                        result.errors.append(f'{context} is missing required field radius_m.')
                    else:
                        _validate_positive_number(point['radius_m'], f'{context}.radius_m', result.errors)

                if category == 'approach_sensors':
                    stopper = _canonical_name(
                        'stoppers',
                        str(point.get('stopper', raw_name)).replace('_APPROACH', ''),
                    )
                    approach_targets.add((stopper, str(point.get('segment', '')).strip()))

                if category == 'stoppers':
                    if not _has_field(point, 'default_state'):
                        result.errors.append(f'{context} is missing required field default_state.')
                    else:
                        _validate_stopper_state(point['default_state'], context, result.errors)

                if category != 'position_sensors' and _has_field(point, 'radius_m'):
                    _validate_positive_number(point['radius_m'], f'{context}.radius_m', result.errors)

    for stopper_name in sorted(stopper_names):
        if not any(target_name == stopper_name for target_name, _segment in approach_targets):
            result.warnings.append(
                f'Stopper {stopper_name} has no matching approach_sensors entry.'
            )

    return result


def print_result(result: ValidationResult) -> None:
    print(
        f'{result.side.upper()} {result.status}: '
        f'{result.devices_path} against {result.network_path}'
    )
    if result.counts:
        sensor_count = (
            result.counts.get('position_sensors', 0)
            + result.counts.get('approach_sensors', 0)
        )
        print(
            '  devices: '
            f'slots={result.counts.get("slots", 0)}, '
            f'sensors={sensor_count}, '
            f'stoppers={result.counts.get("stoppers", 0)}'
        )
    for warning in result.warnings:
        print(f'  WARN: {warning}')
    for error in result.errors:
        print(f'  FAIL: {error}')


def parse_args() -> argparse.Namespace:
    config_dir = _default_config_dir()
    parser = argparse.ArgumentParser(
        description='Validate Room 315 rail device YAML files.'
    )
    parser.add_argument(
        '--right-network',
        type=Path,
        default=config_dir / 'rail_network_right.yaml',
    )
    parser.add_argument(
        '--right-devices',
        type=Path,
        default=config_dir / 'rail_devices_right.yaml',
    )
    parser.add_argument(
        '--left-network',
        type=Path,
        default=config_dir / 'rail_network_left.yaml',
    )
    parser.add_argument(
        '--left-devices',
        type=Path,
        default=config_dir / 'rail_devices_left.yaml',
    )
    parser.add_argument(
        '--path-backend',
        default=CUBIC_HERMITE_PATH_BACKEND,
        help='Path backend used while loading rail segment geometry.',
    )
    parser.add_argument(
        '--arc-length-samples-per-edge',
        type=int,
        default=16,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = [
        validate_devices(
            side='right',
            network_path=args.right_network.resolve(),
            devices_path=args.right_devices.resolve(),
            path_backend=args.path_backend,
            arc_length_samples_per_edge=args.arc_length_samples_per_edge,
        ),
        validate_devices(
            side='left',
            network_path=args.left_network.resolve(),
            devices_path=args.left_devices.resolve(),
            path_backend=args.path_backend,
            arc_length_samples_per_edge=args.arc_length_samples_per_edge,
        ),
    ]

    for result in results:
        print_result(result)

    if any(result.errors for result in results):
        print('SUMMARY FAIL: rail device validation failed.')
        return 1
    if any(result.warnings for result in results):
        print('SUMMARY WARN: rail device validation passed with warnings.')
        return 0

    print('SUMMARY PASS: rail device validation passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
