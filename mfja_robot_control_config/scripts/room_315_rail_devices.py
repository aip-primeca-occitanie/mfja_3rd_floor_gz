#!/usr/bin/env python3
"""Rail-device schema and YAML loader for the Room 315 kinematic runtime."""

import re
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_kinematic_shuttle import RailNetwork
from room_315_rail_defaults import PUBLIC_SWITCH_ORDER


def canonical_switch_name(name: str) -> str:
    return str(name).strip().upper()


def canonical_segment_name(name: str) -> str:
    return str(name).strip().upper()


def canonical_sensor_name(name: str) -> str:
    return str(name).strip().upper()


def canonical_slot_name(name: str) -> str:
    slot = str(name).strip().lower().replace('-', '_')
    return re.sub(r'^(slot|start|start_slot)_?', '', slot)


def normalize_rail_side(raw_value: str) -> str:
    side = str(raw_value).strip().lower()
    if side in {'right', 'r', 'droit'}:
        return 'right'
    if side in {'left', 'l', 'gauche'}:
        return 'left'
    raise ValueError(
        f'Unsupported rail_side={raw_value!r}; use right or left.'
    )


def ordered_switch_states(switch_states: dict[str, str]) -> dict[str, str]:
    ordered = {
        switch_name: switch_states[switch_name]
        for switch_name in PUBLIC_SWITCH_ORDER
        if switch_name in switch_states
    }
    for switch_name, state in switch_states.items():
        canonical_name = canonical_switch_name(switch_name)
        if canonical_name not in ordered:
            ordered[canonical_name] = state
    return ordered


@dataclass(frozen=True)
class StopPoint:
    segment: str
    stop_s: float
    trigger_s: float


@dataclass(frozen=True)
class StopperConfig:
    name: str
    before_switch: str
    default_state: str
    stop_points: tuple[StopPoint, ...]


@dataclass(frozen=True)
class PositionSensorPoint:
    segment: str
    sensor_s: float
    radius_m: float


@dataclass(frozen=True)
class PositionSensorConfig:
    name: str
    points: tuple[PositionSensorPoint, ...]


@dataclass(frozen=True)
class RailDevice:
    name: str
    device_type: str
    segment: str
    s_ratio: float
    s: float
    x: float
    y: float
    z: float
    yaw: float
    radius_m: float | None = None
    default_state: str | None = None
    metadata: dict | None = None


@dataclass(frozen=True)
class RailDeviceSet:
    path: Path
    slots: dict[str, RailDevice]
    position_sensors: dict[str, tuple[RailDevice, ...]]
    stoppers: dict[str, tuple[RailDevice, ...]]


def _require_mapping(value, context: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f'{context} must be a mapping, got {type(value)!r}.')
    return value


def _category_entries(config: dict, category: str) -> list[tuple[str, dict]]:
    raw_category = config.get(category, [])
    if raw_category is None:
        return []
    if isinstance(raw_category, list):
        entries = []
        for index, raw_entry in enumerate(raw_category):
            entry = _require_mapping(raw_entry, f'{category}[{index}]')
            if 'name' not in entry:
                raise ValueError(f'{category}[{index}] must define name.')
            entries.append((str(entry['name']), entry))
        return entries
    if isinstance(raw_category, dict):
        return [
            (
                str(raw_name),
                {'name': raw_name, **_require_mapping(raw_entry, f'{category}.{raw_name}')},
            )
            for raw_name, raw_entry in raw_category.items()
        ]
    raise ValueError(f'{category} must be a list or mapping, got {type(raw_category)!r}.')


def _device_name_key(category: str, raw_name: str) -> str:
    if category == 'slots':
        key = canonical_slot_name(raw_name)
        if not key:
            raise ValueError(f'{category} name {raw_name!r} does not resolve to a slot id.')
        return key
    if category == 'position_sensors':
        return canonical_sensor_name(raw_name)
    if category == 'stoppers':
        return canonical_switch_name(raw_name)
    return str(raw_name).strip()


def _device_points(raw_entry: dict, category: str, name: str) -> list[dict]:
    if 'points' not in raw_entry:
        return [raw_entry]
    raw_points = raw_entry['points']
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError(f'{category}.{name}.points must be a non-empty list.')
    inherited = {
        key: value
        for key, value in raw_entry.items()
        if key not in {'points', 'segment', 's_ratio'}
    }
    return [
        {
            **inherited,
            **_require_mapping(raw_point, f'{category}.{name}.points[{index}]'),
        }
        for index, raw_point in enumerate(raw_points)
    ]


def _require_device_fields(point: dict, category: str, name: str, index: int) -> None:
    context = f'{category}.{name}'
    if index > 0:
        context += f'.points[{index}]'
    required = ['segment', 's_ratio']
    if category == 'position_sensors':
        required.append('radius_m')
    elif category == 'stoppers':
        required.append('default_state')
    missing = [field for field in required if field not in point]
    if missing:
        raise ValueError(f'{context} is missing required field(s): {missing}.')


def _rail_device_from_point(
    *,
    name: str,
    device_type: str,
    point: dict,
    rail_network: RailNetwork,
) -> RailDevice:
    segment_name = str(point['segment']).strip()
    if segment_name not in rail_network.segments:
        raise ValueError(
            f'{device_type}.{name} references unknown segment {segment_name!r}.'
        )
    try:
        s_ratio = float(point['s_ratio'])
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{device_type}.{name}.s_ratio must be a number between 0.0 and 1.0.'
        ) from error
    if not 0.0 <= s_ratio <= 1.0:
        raise ValueError(
            f'{device_type}.{name}.s_ratio={s_ratio:.6f} is outside [0.0, 1.0].'
        )

    segment = rail_network.segments[segment_name]
    s = s_ratio * segment.length
    sample_point, yaw = segment.sample(s)
    radius_m = (
        float(point['radius_m'])
        if 'radius_m' in point and point['radius_m'] is not None
        else None
    )
    if device_type == 'position_sensors':
        if radius_m is None:
            raise ValueError(f'{device_type}.{name} must define radius_m.')
        if radius_m < 0.0:
            raise ValueError(
                f'{device_type}.{name}.radius_m must be greater than or equal to 0.0.'
            )
    metadata = {
        key: value
        for key, value in point.items()
        if key not in {
            'name',
            'segment',
            's_ratio',
            'radius_m',
            'default_state',
        }
    }
    return RailDevice(
        name=name,
        device_type=device_type,
        segment=segment_name,
        s_ratio=s_ratio,
        s=s,
        x=sample_point.x,
        y=sample_point.y,
        z=sample_point.z,
        yaw=yaw,
        radius_m=radius_m,
        default_state=(
            str(point['default_state'])
            if 'default_state' in point and point['default_state'] is not None
            else None
        ),
        metadata=metadata,
    )


def _load_grouped_rail_devices(
    config: dict,
    category: str,
    rail_network: RailNetwork,
) -> dict[str, tuple[RailDevice, ...]]:
    devices: dict[str, tuple[RailDevice, ...]] = {}
    for raw_name, raw_entry in _unique_category_entries(config, category):
        name_key = _device_name_key(category, raw_name)
        device_name = str(raw_name).strip() or name_key
        points = []
        for index, point in enumerate(_device_points(raw_entry, category, raw_name)):
            _require_device_fields(point, category, raw_name, index)
            points.append(
                _rail_device_from_point(
                    name=device_name,
                    device_type=category,
                    point=point,
                    rail_network=rail_network,
                )
            )
        devices[name_key] = tuple(points)
    return devices


def _unique_category_entries(
    config: dict,
    category: str,
) -> list[tuple[str, dict]]:
    entries = _category_entries(config, category)
    seen_names: set[str] = set()
    for raw_name, _ in entries:
        name_key = _device_name_key(category, raw_name)
        if name_key in seen_names:
            raise ValueError(f'Duplicate {category} name {raw_name!r}.')
        seen_names.add(name_key)
    return entries


def _load_linked_position_sensor_devices(
    *,
    raw_name: str,
    raw_entry: dict,
    rail_network: RailNetwork,
    stoppers: dict[str, tuple[RailDevice, ...]],
) -> tuple[RailDevice, ...]:
    name_key = _device_name_key('position_sensors', raw_name)
    device_name = str(raw_name).strip() or name_key
    location_fields = [
        field
        for field in ('segment', 's_ratio', 'points', 's', 'offset_m', 'reference', 'slot')
        if field in raw_entry
    ]
    if location_fields:
        raise ValueError(
            f'position_sensors.{raw_name} is linked to a stopper and must not '
            f'define location field(s) {location_fields}; edit the matching '
            'stoppers entry or before_stopper_m instead.'
        )
    stopper_name = canonical_switch_name(str(raw_entry.get('stopper', '')))
    if not stopper_name:
        raise ValueError(
            f'position_sensors.{raw_name} uses stopper linkage but stopper is empty.'
        )
    stopper_devices = stoppers.get(stopper_name)
    if not stopper_devices:
        raise ValueError(
            f'position_sensors.{raw_name} references unknown stopper {stopper_name!r}.'
        )
    if 'before_stopper_m' not in raw_entry:
        raise ValueError(
            f'position_sensors.{raw_name} is linked to stopper {stopper_name} '
            'and must define before_stopper_m.'
        )
    try:
        before_stopper_m = float(raw_entry['before_stopper_m'])
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'position_sensors.{raw_name}.before_stopper_m must be a number.'
        ) from error
    if before_stopper_m < 0.0:
        raise ValueError(
            f'position_sensors.{raw_name}.before_stopper_m must be greater than '
            'or equal to 0.0.'
        )
    if 'radius_m' not in raw_entry:
        raise ValueError(f'position_sensors.{raw_name} must define radius_m.')

    sensor_devices = []
    for index, stopper_device in enumerate(stopper_devices):
        segment = rail_network.segments[stopper_device.segment]
        sensor_s = stopper_device.s - before_stopper_m
        if sensor_s < -1e-6:
            raise ValueError(
                f'position_sensors.{raw_name}.before_stopper_m={before_stopper_m:.6f} '
                f'places point {index} before the start of segment '
                f'{stopper_device.segment!r}.'
            )
        sensor_s = max(0.0, sensor_s)
        point = {
            **raw_entry,
            'segment': stopper_device.segment,
            's_ratio': sensor_s / segment.length if segment.length > 0.0 else 0.0,
        }
        sensor_devices.append(
            _rail_device_from_point(
                name=device_name,
                device_type='position_sensors',
                point=point,
                rail_network=rail_network,
            )
        )
    return tuple(sensor_devices)


def _load_position_sensor_devices(
    config: dict,
    rail_network: RailNetwork,
    stoppers: dict[str, tuple[RailDevice, ...]],
) -> dict[str, tuple[RailDevice, ...]]:
    devices: dict[str, tuple[RailDevice, ...]] = {}
    for raw_name, raw_entry in _unique_category_entries(config, 'position_sensors'):
        name_key = _device_name_key('position_sensors', raw_name)
        device_name = str(raw_name).strip() or name_key
        if 'stopper' in raw_entry:
            devices[name_key] = _load_linked_position_sensor_devices(
                raw_name=raw_name,
                raw_entry=raw_entry,
                rail_network=rail_network,
                stoppers=stoppers,
            )
            continue
        points = []
        for index, point in enumerate(_device_points(raw_entry, 'position_sensors', raw_name)):
            _require_device_fields(point, 'position_sensors', raw_name, index)
            points.append(
                _rail_device_from_point(
                    name=device_name,
                    device_type='position_sensors',
                    point=point,
                    rail_network=rail_network,
                )
            )
        devices[name_key] = tuple(points)
    return devices


def load_rail_devices(path: Path, rail_network: RailNetwork) -> RailDeviceSet:
    path = path.resolve()
    with path.open() as handle:
        config = yaml.safe_load(handle) or {}
    if not isinstance(config, dict):
        raise ValueError(f'{path} must contain a YAML mapping.')

    slots_grouped = _load_grouped_rail_devices(config, 'slots', rail_network)
    stoppers = _load_grouped_rail_devices(config, 'stoppers', rail_network)
    position_sensors = _load_position_sensor_devices(config, rail_network, stoppers)
    missing_categories = [
        category
        for category, devices in (
            ('slots', slots_grouped),
            ('position_sensors', position_sensors),
            ('stoppers', stoppers),
        )
        if not devices
    ]
    if missing_categories:
        raise ValueError(
            f'{path} must define non-empty device categories: {missing_categories}.'
        )
    return RailDeviceSet(
        path=path,
        slots={name: devices[0] for name, devices in slots_grouped.items()},
        position_sensors=position_sensors,
        stoppers=stoppers,
    )


__all__ = [
    'PositionSensorConfig',
    'PositionSensorPoint',
    'RailDevice',
    'RailDeviceSet',
    'StopPoint',
    'StopperConfig',
    'canonical_segment_name',
    'canonical_sensor_name',
    'canonical_slot_name',
    'canonical_switch_name',
    'load_rail_devices',
    'normalize_rail_side',
    'ordered_switch_states',
]
