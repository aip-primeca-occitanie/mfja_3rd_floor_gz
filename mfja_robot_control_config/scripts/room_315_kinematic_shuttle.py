#!/usr/bin/env python3

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import yaml


MOVING = 'MOVING'
WAITING = 'WAITING'
FALLING = 'FALLING'
POLYLINE_PATH_BACKEND = 'polyline'
CUBIC_HERMITE_PATH_BACKEND = 'cubic_hermite'
PATH_BACKENDS = {POLYLINE_PATH_BACKEND, CUBIC_HERMITE_PATH_BACKEND}


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float

    def distance_to(self, other: 'Point3D') -> float:
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))


@dataclass
class ShuttleState:
    current_segment: str
    s: float
    speed: float
    mode: str = MOVING


@dataclass(frozen=True)
class ShuttlePose:
    x: float
    y: float
    z: float
    yaw: float
    current_segment: str
    s: float
    mode: str


class SegmentGeometry:
    def __init__(
        self,
        name: str,
        points: Sequence[Point3D],
        path_backend: str = POLYLINE_PATH_BACKEND,
        arc_length_samples_per_edge: int = 16,
    ) -> None:
        if len(points) < 2:
            raise ValueError(f'{name} must contain at least two points')
        self.name = name
        self.points = list(points)
        self.path_backend = _normalize_path_backend(path_backend)
        self.arc_length_samples_per_edge = max(4, int(arc_length_samples_per_edge))
        self.chord_arc_lengths = self._compute_arc_lengths(self.points)
        self._tangents = self._compute_tangents()
        self._arc_map: List[tuple[float, int, float]] = []
        self.arc_lengths = list(self.chord_arc_lengths)
        self._length = self.chord_arc_lengths[-1]
        if self.path_backend == CUBIC_HERMITE_PATH_BACKEND:
            self.arc_lengths, self._arc_map, self._length = self._build_cubic_arc_map()
        if self.length <= 0.0:
            raise ValueError(f'{name} has zero total length')

    @staticmethod
    def _compute_arc_lengths(points: Sequence[Point3D]) -> List[float]:
        arc_lengths = [0.0]
        total = 0.0
        for previous, current in zip(points, points[1:]):
            total += previous.distance_to(current)
            arc_lengths.append(total)
        return arc_lengths

    @property
    def length(self) -> float:
        return self._length

    def sample(self, s: float) -> tuple[Point3D, float]:
        if self.path_backend == CUBIC_HERMITE_PATH_BACKEND:
            return self._sample_cubic_by_arc_length(s)
        return self._sample_polyline(s)

    def _sample_polyline(self, s: float) -> tuple[Point3D, float]:
        clamped_s = max(0.0, min(s, self.length))
        for index in range(1, len(self.arc_lengths)):
            previous_s = self.arc_lengths[index - 1]
            current_s = self.arc_lengths[index]
            if clamped_s > current_s and index < len(self.arc_lengths) - 1:
                continue

            previous = self.points[index - 1]
            current = self.points[index]
            edge_length = current_s - previous_s
            ratio = 0.0 if edge_length == 0.0 else (clamped_s - previous_s) / edge_length
            x = previous.x + ratio * (current.x - previous.x)
            y = previous.y + ratio * (current.y - previous.y)
            z = previous.z + ratio * (current.z - previous.z)
            yaw = math.atan2(current.y - previous.y, current.x - previous.x)
            return Point3D(x=x, y=y, z=z), yaw

        previous = self.points[-2]
        current = self.points[-1]
        yaw = math.atan2(current.y - previous.y, current.x - previous.x)
        return current, yaw

    def _compute_tangents(self) -> List[Point3D]:
        tangents: List[Point3D] = []
        for index, point in enumerate(self.points):
            if index == 0:
                previous = point
                current = self.points[index + 1]
                previous_s = self.chord_arc_lengths[index]
                current_s = self.chord_arc_lengths[index + 1]
            elif index == len(self.points) - 1:
                previous = self.points[index - 1]
                current = point
                previous_s = self.chord_arc_lengths[index - 1]
                current_s = self.chord_arc_lengths[index]
            else:
                previous = self.points[index - 1]
                current = self.points[index + 1]
                previous_s = self.chord_arc_lengths[index - 1]
                current_s = self.chord_arc_lengths[index + 1]

            ds = max(current_s - previous_s, 1e-12)
            tangents.append(
                Point3D(
                    x=(current.x - previous.x) / ds,
                    y=(current.y - previous.y) / ds,
                    z=(current.z - previous.z) / ds,
                )
            )
        return tangents

    def _build_cubic_arc_map(self) -> tuple[List[float], List[tuple[float, int, float]], float]:
        point_arc_lengths = [0.0]
        arc_map: List[tuple[float, int, float]] = [(0.0, 0, 0.0)]
        total = 0.0

        for index in range(len(self.points) - 1):
            previous_point, _ = self._evaluate_cubic_interval(index, 0.0)
            for sample_index in range(1, self.arc_length_samples_per_edge + 1):
                u = sample_index / self.arc_length_samples_per_edge
                current_point, _ = self._evaluate_cubic_interval(index, u)
                total += previous_point.distance_to(current_point)
                arc_map.append((total, index, u))
                previous_point = current_point
            point_arc_lengths.append(total)

        return point_arc_lengths, arc_map, total

    def _evaluate_cubic_interval(self, index: int, u: float) -> tuple[Point3D, Point3D]:
        p0 = self.points[index]
        p1 = self.points[index + 1]
        m0 = self._tangents[index]
        m1 = self._tangents[index + 1]
        h = max(self.chord_arc_lengths[index + 1] - self.chord_arc_lengths[index], 1e-12)

        u2 = u * u
        u3 = u2 * u
        h00 = 2.0 * u3 - 3.0 * u2 + 1.0
        h10 = u3 - 2.0 * u2 + u
        h01 = -2.0 * u3 + 3.0 * u2
        h11 = u3 - u2

        dh00 = 6.0 * u2 - 6.0 * u
        dh10 = 3.0 * u2 - 4.0 * u + 1.0
        dh01 = -6.0 * u2 + 6.0 * u
        dh11 = 3.0 * u2 - 2.0 * u

        point = Point3D(
            x=h00 * p0.x + h10 * h * m0.x + h01 * p1.x + h11 * h * m1.x,
            y=h00 * p0.y + h10 * h * m0.y + h01 * p1.y + h11 * h * m1.y,
            z=h00 * p0.z + h10 * h * m0.z + h01 * p1.z + h11 * h * m1.z,
        )
        tangent = Point3D(
            x=(dh00 * p0.x + dh01 * p1.x) / h + dh10 * m0.x + dh11 * m1.x,
            y=(dh00 * p0.y + dh01 * p1.y) / h + dh10 * m0.y + dh11 * m1.y,
            z=(dh00 * p0.z + dh01 * p1.z) / h + dh10 * m0.z + dh11 * m1.z,
        )
        return point, tangent

    def _sample_cubic_by_arc_length(self, s: float) -> tuple[Point3D, float]:
        clamped_s = max(0.0, min(s, self.length))
        if clamped_s <= 0.0:
            point, tangent = self._evaluate_cubic_interval(0, 0.0)
            return point, math.atan2(tangent.y, tangent.x)
        if clamped_s >= self.length:
            point, tangent = self._evaluate_cubic_interval(len(self.points) - 2, 1.0)
            return point, math.atan2(tangent.y, tangent.x)

        low = 0
        high = len(self._arc_map) - 1
        while low < high:
            middle = (low + high) // 2
            if self._arc_map[middle][0] < clamped_s:
                low = middle + 1
            else:
                high = middle

        upper = self._arc_map[low]
        lower = self._arc_map[max(0, low - 1)]
        lower_s, lower_interval, lower_u = lower
        upper_s, upper_interval, upper_u = upper
        if upper_s <= lower_s:
            interval = upper_interval
            u = upper_u
        else:
            ratio = (clamped_s - lower_s) / (upper_s - lower_s)
            if lower_interval == upper_interval:
                interval = upper_interval
                u = lower_u + ratio * (upper_u - lower_u)
            else:
                interval = upper_interval
                u = upper_u

        point, tangent = self._evaluate_cubic_interval(interval, u)
        yaw = math.atan2(tangent.y, tangent.x)
        return point, yaw


class RailNetwork:
    def __init__(self, network_path: Path, config: dict, segments: Dict[str, SegmentGeometry]) -> None:
        self.network_path = network_path
        self.config = config
        self.segments = segments
        self.routing_table = config.get('routing_table', {})
        self.switches = config.get('switches', {})
        self.valid_switch_states = set(config.get('switch_state_space', {}).get('values', []))

    @classmethod
    def from_yaml(
        cls,
        network_path: Path,
        path_backend: str = POLYLINE_PATH_BACKEND,
        arc_length_samples_per_edge: int = 16,
    ) -> 'RailNetwork':
        network_path = network_path.resolve()
        with network_path.open() as handle:
            config = yaml.safe_load(handle)

        normalized_backend = _normalize_path_backend(path_backend)
        segments = {
            name: SegmentGeometry(
                name=name,
                points=_read_csv_points(_resolve_path(segment_config['csv'], network_path)),
                path_backend=normalized_backend,
                arc_length_samples_per_edge=arc_length_samples_per_edge,
            )
            for name, segment_config in config['segments'].items()
        }
        return cls(network_path=network_path, config=config, segments=segments)

    def default_switch_states(self) -> Dict[str, str]:
        return {
            switch_name: switch_config.get('states', ['G'])[0]
            for switch_name, switch_config in self.switches.items()
        }

    def normalized_switch_state(self, raw_state: str) -> str:
        state = raw_state.strip().upper()
        if state in {'E', 'EXTERIOR', 'GRAND', 'GRAND_BOUCLE', 'BIG', 'LARGE'}:
            state = 'G'
        elif state in {'I', 'INTERIOR', 'PETIT', 'PETIT_BOUCLE', 'SMALL', 'MINI'}:
            state = 'S'
        if state not in self.valid_switch_states:
            raise ValueError(
                f'Unknown switch state {raw_state!r}; expected one of '
                f'{sorted(self.valid_switch_states)} or their aliases '
                '(for example E / I / EXTERIOR / INTERIOR).'
            )
        return state

    def resolve_successor(
        self,
        current_segment: str,
        switch_states: Dict[str, str],
    ) -> Optional[str]:
        rule = self.routing_table.get(current_segment)
        if rule is None:
            return None

        rule_type = rule.get('type')
        if rule_type == 'fixed':
            next_segment = rule.get('next_segment')
            return None if next_segment == FALLING else next_segment

        switch_name = rule.get('switch')
        switch_state = switch_states.get(switch_name)
        if switch_state is None:
            return None

        normalized_state = self.normalized_switch_state(switch_state)
        next_segment = rule.get('by_state', {}).get(normalized_state, rule.get('on_unknown_state'))
        if next_segment == FALLING:
            return None
        return next_segment


class KinematicShuttleCore:
    def __init__(self, network: RailNetwork, initial_state: ShuttleState) -> None:
        if initial_state.current_segment not in network.segments:
            raise ValueError(f'Unknown initial segment: {initial_state.current_segment}')
        self.network = network
        self.state = initial_state

    def pose(self) -> ShuttlePose:
        segment = self.network.segments[self.state.current_segment]
        point, yaw = segment.sample(self.state.s)
        return ShuttlePose(
            x=point.x,
            y=point.y,
            z=point.z,
            yaw=yaw,
            current_segment=self.state.current_segment,
            s=self.state.s,
            mode=self.state.mode,
        )

    def step(self, dt: float, switch_states: Optional[Dict[str, str]] = None) -> ShuttlePose:
        if dt < 0.0:
            raise ValueError('dt must be non-negative')

        switch_states = switch_states or self.network.default_switch_states()
        if self.state.mode == FALLING:
            return self.pose()

        if self.state.speed <= 0.0:
            self.state.mode = WAITING
            return self.pose()

        if self.state.mode == WAITING:
            self.state.mode = MOVING

        remaining_distance = self.state.speed * dt
        while remaining_distance > 0.0 and self.state.mode == MOVING:
            segment = self.network.segments[self.state.current_segment]
            distance_to_end = max(0.0, segment.length - self.state.s)

            if remaining_distance < distance_to_end:
                self.state.s += remaining_distance
                remaining_distance = 0.0
                break

            remaining_distance -= distance_to_end
            self.state.s = segment.length
            successor = self.network.resolve_successor(self.state.current_segment, switch_states)
            if successor is None or successor not in self.network.segments:
                self.state.mode = FALLING
                break

            self.state.current_segment = successor
            self.state.s = 0.0

        return self.pose()


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_network_path() -> Path:
    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / 'rail_network_right.yaml'
    )


def _resolve_path(raw_path: str, network_path: Path) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path

    repo_relative = _repo_root() / path
    if repo_relative.exists():
        return repo_relative
    return network_path.parent / path


def _normalize_path_backend(raw_backend: str) -> str:
    backend = str(raw_backend).strip().lower().replace('-', '_')
    aliases = {
        'csv': POLYLINE_PATH_BACKEND,
        'linear': POLYLINE_PATH_BACKEND,
        'points': POLYLINE_PATH_BACKEND,
        'continuous': CUBIC_HERMITE_PATH_BACKEND,
        'spline': CUBIC_HERMITE_PATH_BACKEND,
        'hermite': CUBIC_HERMITE_PATH_BACKEND,
        'cubic': CUBIC_HERMITE_PATH_BACKEND,
    }
    backend = aliases.get(backend, backend)
    if backend not in PATH_BACKENDS:
        raise ValueError(
            f'Unknown path_backend {raw_backend!r}; expected one of {sorted(PATH_BACKENDS)}'
        )
    return backend


def _read_csv_points(csv_path: Path) -> List[Point3D]:
    with csv_path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or [])
        required = {'index', 'x', 'y', 'z'}
        missing = required - fieldnames
        if missing:
            raise ValueError(f'{csv_path} is missing required columns: {sorted(missing)}')
        return [
            Point3D(x=float(row['x']), y=float(row['y']), z=float(row['z']))
            for row in reader
        ]


def _parse_switch_states(network: RailNetwork, raw_values: Sequence[str]) -> Dict[str, str]:
    switch_states = network.default_switch_states()
    for raw_value in raw_values:
        for token in raw_value.replace(',', ' ').split():
            if '=' not in token:
                raise ValueError(f'Switch assignment must look like A1=G, got {token!r}')
            switch_name, raw_state = token.split('=', 1)
            switch_name = switch_name.strip().upper()
            if switch_name not in network.switches:
                raise ValueError(f'Unknown switch {switch_name!r}; expected one of {sorted(network.switches)}')
            switch_states[switch_name] = network.normalized_switch_state(raw_state)
    return switch_states


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run the room 315 one-shuttle kinematic core without ROS or Gazebo.'
    )
    parser.add_argument(
        '--network',
        type=Path,
        default=_default_network_path(),
        help='rail_network_right.yaml path.',
    )
    parser.add_argument(
        '--segment',
        default='A23',
        help='Initial segment name.',
    )
    parser.add_argument(
        '--s',
        type=float,
        default=0.0,
        help='Initial arc-length progress on the segment.',
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=0.25,
        help='Shuttle speed in m/s.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=10.0,
        help='Simulation duration in seconds.',
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.05,
        help='Simulation time step in seconds.',
    )
    parser.add_argument(
        '--switch',
        action='append',
        default=[],
        help='Switch assignment such as A1=G. Can be repeated or comma/space separated.',
    )
    parser.add_argument(
        '--path-backend',
        default=CUBIC_HERMITE_PATH_BACKEND,
        choices=sorted(PATH_BACKENDS),
        help='Geometry sampling backend. Use polyline for raw CSV interpolation or cubic_hermite for continuous paths.',
    )
    parser.add_argument(
        '--arc-length-samples-per-edge',
        type=int,
        default=16,
        help='Sub-samples per CSV edge used to arc-length parameterize the continuous path backend.',
    )
    parser.add_argument(
        '--trace-every',
        type=float,
        default=1.0,
        help='Print one pose sample every N seconds. Use 0 for final state only.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    network = RailNetwork.from_yaml(
        args.network,
        path_backend=args.path_backend,
        arc_length_samples_per_edge=args.arc_length_samples_per_edge,
    )
    switch_states = _parse_switch_states(network, args.switch)
    core = KinematicShuttleCore(
        network=network,
        initial_state=ShuttleState(
            current_segment=args.segment,
            s=args.s,
            speed=args.speed,
            mode=MOVING,
        ),
    )

    elapsed = 0.0
    next_trace_time = 0.0
    while elapsed < args.duration:
        pose = core.step(args.dt, switch_states=switch_states)
        elapsed += args.dt
        if args.trace_every > 0.0 and elapsed >= next_trace_time:
            print(json.dumps({'t': elapsed, **asdict(pose)}, sort_keys=True))
            next_trace_time += args.trace_every
        if pose.mode == FALLING:
            break

    final_pose = core.pose()
    print(json.dumps({'final': True, 't': elapsed, **asdict(final_pose)}, sort_keys=True))
    return 0 if final_pose.mode != FALLING else 2


if __name__ == '__main__':
    raise SystemExit(main())
