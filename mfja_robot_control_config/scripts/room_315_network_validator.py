#!/usr/bin/env python3

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml


FALLING_SENTINEL = 'FALLING'


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float

    def distance_to(self, other: 'Point3D') -> float:
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))

    def as_list(self) -> List[float]:
        return [self.x, self.y, self.z]


@dataclass(frozen=True)
class SegmentGeometry:
    name: str
    points: List[Point3D]
    duplicates_removed: int

    @property
    def start(self) -> Point3D:
        return self.points[0]

    @property
    def end(self) -> Point3D:
        return self.points[-1]

    @property
    def length(self) -> float:
        return sum(
            previous.distance_to(current)
            for previous, current in zip(self.points, self.points[1:])
        )


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _package_share_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('mfja_robot_control_config'))
    except Exception:
        return _repo_root() / 'mfja_robot_control_config'


def _default_config_dir() -> Path:
    return _package_share_dir() / 'config' / 'room_315_kinematics'


def _default_network_path() -> Path:
    return _default_config_dir() / 'rail_network_right.yaml'


def _default_report_path() -> Path:
    return _default_config_dir() / 'validation_report.yaml'


def _default_plot_path() -> Path:
    return _default_config_dir() / 'debug_plots' / 'network_validation.png'


def _resolve_path(raw_path: str, network_path: Path) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path

    config_relative = _package_share_dir() / path
    if config_relative.exists():
        return config_relative

    source_relative = _repo_root() / path
    if source_relative.exists():
        return source_relative

    return network_path.parent / path


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


def _remove_consecutive_duplicates(
    points: Sequence[Point3D],
    duplicate_tolerance: float,
) -> tuple[List[Point3D], int]:
    if len(points) < 2:
        raise ValueError('A rail segment must contain at least two points')

    normalized = [points[0]]
    duplicates_removed = 0
    for point in points[1:]:
        if normalized[-1].distance_to(point) <= duplicate_tolerance:
            duplicates_removed += 1
            continue
        normalized.append(point)

    if len(normalized) < 2:
        raise ValueError('A rail segment collapsed to fewer than two distinct points')
    return normalized, duplicates_removed


def _load_network(network_path: Path) -> dict:
    with network_path.open() as handle:
        return yaml.safe_load(handle)


def _load_segment_geometry(
    network: dict,
    network_path: Path,
    duplicate_tolerance: float,
) -> Dict[str, SegmentGeometry]:
    geometries: Dict[str, SegmentGeometry] = {}
    for segment_name, segment_config in network['segments'].items():
        csv_path = _resolve_path(segment_config['csv'], network_path)
        raw_points = _read_csv_points(csv_path)
        points, duplicates_removed = _remove_consecutive_duplicates(
            raw_points,
            duplicate_tolerance=duplicate_tolerance,
        )
        geometries[segment_name] = SegmentGeometry(
            name=segment_name,
            points=points,
            duplicates_removed=duplicates_removed,
        )
    return geometries


def _point_from_xyz(xyz: Sequence[float]) -> Point3D:
    return Point3D(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def _vector_from_last_edge(points: Sequence[Point3D]) -> tuple[float, float, float]:
    for previous, current in zip(reversed(points[:-1]), reversed(points[1:])):
        dx = current.x - previous.x
        dy = current.y - previous.y
        dz = current.z - previous.z
        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            return dx, dy, dz
    raise ValueError('Segment has no non-zero final tangent')


def _vector_from_first_edge(points: Sequence[Point3D]) -> tuple[float, float, float]:
    for previous, current in zip(points, points[1:]):
        dx = current.x - previous.x
        dy = current.y - previous.y
        dz = current.z - previous.z
        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            return dx, dy, dz
    raise ValueError('Segment has no non-zero initial tangent')


def _angle_deg(first: Sequence[float], second: Sequence[float]) -> float:
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm == 0.0 or second_norm == 0.0:
        raise ValueError('Cannot compute angle from a zero-length vector')

    dot = sum(a * b for a, b in zip(first, second)) / (first_norm * second_norm)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def _routing_successors(network: dict) -> List[dict]:
    transitions: List[dict] = []
    for from_segment, rule in network.get('routing_table', {}).items():
        rule_type = rule.get('type')
        if rule_type == 'fixed':
            next_segment = rule.get('next_segment')
            if next_segment and next_segment != FALLING_SENTINEL:
                transitions.append(
                    {
                        'from_segment': from_segment,
                        'to_segment': next_segment,
                        'rule': 'fixed',
                        'switch': None,
                        'state': None,
                    }
                )
            continue

        for state, next_segment in rule.get('by_state', {}).items():
            if next_segment == FALLING_SENTINEL:
                continue
            transitions.append(
                {
                    'from_segment': from_segment,
                    'to_segment': next_segment,
                    'rule': rule_type,
                    'switch': rule.get('switch'),
                    'state': state,
                }
            )
    return transitions


def _endpoint_distances(
    network: dict,
    geometries: Dict[str, SegmentGeometry],
    snap_tolerance: float,
) -> tuple[Dict[str, dict], List[dict]]:
    distances: Dict[str, dict] = {}
    warnings: List[dict] = []
    nodes = network['nodes']

    for segment_name, segment_config in network['segments'].items():
        geometry = geometries[segment_name]
        start_node = segment_config['start_node']
        end_node = segment_config['end_node']
        start_anchor = _point_from_xyz(nodes[start_node]['xyz'])
        end_anchor = _point_from_xyz(nodes[end_node]['xyz'])
        start_distance = geometry.start.distance_to(start_anchor)
        end_distance = geometry.end.distance_to(end_anchor)

        distances[segment_name] = {
            'start_node': start_node,
            'start_distance_m': start_distance,
            'end_node': end_node,
            'end_distance_m': end_distance,
        }

        if start_distance > snap_tolerance:
            warnings.append(
                {
                    'type': 'endpoint_gap',
                    'segment': segment_name,
                    'side': 'start',
                    'node': start_node,
                    'distance_m': start_distance,
                    'threshold_m': snap_tolerance,
                }
            )
        if end_distance > snap_tolerance:
            warnings.append(
                {
                    'type': 'endpoint_gap',
                    'segment': segment_name,
                    'side': 'end',
                    'node': end_node,
                    'distance_m': end_distance,
                    'threshold_m': snap_tolerance,
                }
            )

    return distances, warnings


def _transition_diagnostics(
    network: dict,
    geometries: Dict[str, SegmentGeometry],
    gap_tolerance: float,
    tangent_threshold_deg: float,
) -> tuple[List[dict], List[dict]]:
    diagnostics: List[dict] = []
    warnings: List[dict] = []

    for transition in _routing_successors(network):
        from_segment = transition['from_segment']
        to_segment = transition['to_segment']
        from_geometry = geometries[from_segment]
        to_geometry = geometries[to_segment]

        gap = from_geometry.end.distance_to(to_geometry.start)
        angle = _angle_deg(
            _vector_from_last_edge(from_geometry.points),
            _vector_from_first_edge(to_geometry.points),
        )

        record = {
            **transition,
            'endpoint_gap_m': gap,
            'tangent_mismatch_deg': angle,
        }
        diagnostics.append(record)

        if gap > gap_tolerance:
            warnings.append(
                {
                    'type': 'transition_gap',
                    'from_segment': from_segment,
                    'to_segment': to_segment,
                    'distance_m': gap,
                    'threshold_m': gap_tolerance,
                }
            )
        if angle > tangent_threshold_deg:
            warnings.append(
                {
                    'type': 'tangent_mismatch',
                    'from_segment': from_segment,
                    'to_segment': to_segment,
                    'angle_deg': angle,
                    'threshold_deg': tangent_threshold_deg,
                }
            )

    return diagnostics, warnings


def _segment_lengths(geometries: Dict[str, SegmentGeometry]) -> Dict[str, dict]:
    return {
        name: {
            'length_m': geometry.length,
            'point_count': len(geometry.points),
            'consecutive_duplicates_removed': geometry.duplicates_removed,
        }
        for name, geometry in sorted(geometries.items())
    }


def _draw_network_plot(
    network: dict,
    geometries: Dict[str, SegmentGeometry],
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(16, 10))
    color_map = plt.get_cmap('tab20')

    for index, (segment_name, geometry) in enumerate(sorted(geometries.items())):
        color = color_map(index % 20)
        xs = [point.x for point in geometry.points]
        ys = [point.y for point in geometry.points]
        axis.plot(xs, ys, color=color, linewidth=2.0, alpha=0.9)
        axis.scatter(xs, ys, color=[color], s=12, alpha=0.5)

        if len(xs) >= 2:
            axis.annotate(
                '',
                xy=(xs[-1], ys[-1]),
                xytext=(xs[-2], ys[-2]),
                arrowprops={'arrowstyle': '->', 'color': color, 'lw': 1.6},
            )

        middle_index = len(geometry.points) // 2
        middle = geometry.points[middle_index]
        axis.text(
            middle.x,
            middle.y,
            segment_name,
            fontsize=10,
            weight='bold',
            color=color,
            bbox={'facecolor': 'white', 'alpha': 0.65, 'edgecolor': 'none', 'pad': 1.5},
        )

    for node_name, node_config in sorted(network['nodes'].items()):
        node = _point_from_xyz(node_config['xyz'])
        axis.scatter([node.x], [node.y], color='black', marker='x', s=90, linewidths=2)
        axis.text(
            node.x,
            node.y,
            f' {node_name}',
            fontsize=10,
            color='black',
            bbox={'facecolor': 'white', 'alpha': 0.75, 'edgecolor': 'none', 'pad': 1.5},
        )

    axis.set_title('Room 315 rail network validation plot')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_aspect('equal', adjustable='box')
    axis.grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def _validate_attached_endpoints(network: dict) -> List[dict]:
    warnings: List[dict] = []
    segments = network['segments']
    for node_name, node_config in network['nodes'].items():
        for attached in node_config.get('attached_segment_ends', []):
            segment_name = attached.get('segment')
            side = attached.get('side')
            if segment_name not in segments:
                warnings.append(
                    {
                        'type': 'unknown_attached_segment',
                        'node': node_name,
                        'segment': segment_name,
                    }
                )
            elif side not in {'start', 'end'}:
                warnings.append(
                    {
                        'type': 'invalid_attached_side',
                        'node': node_name,
                        'segment': segment_name,
                        'side': side,
                    }
                )
    return warnings


def _build_report(
    network: dict,
    network_path: Path,
    geometries: Dict[str, SegmentGeometry],
    duplicate_tolerance: float,
    tangent_threshold_deg: float,
) -> dict:
    snap_tolerance = float(network.get('snap_tolerance_m', 0.05))
    endpoint_distances, endpoint_warnings = _endpoint_distances(
        network,
        geometries,
        snap_tolerance=snap_tolerance,
    )
    transition_records, transition_warnings = _transition_diagnostics(
        network,
        geometries,
        gap_tolerance=snap_tolerance,
        tangent_threshold_deg=tangent_threshold_deg,
    )
    topology_warnings = _validate_attached_endpoints(network)
    warnings = endpoint_warnings + transition_warnings + topology_warnings

    return {
        'phase': 3,
        'network_yaml': str(network_path),
        'snap_tolerance_m': snap_tolerance,
        'duplicate_tolerance_m': duplicate_tolerance,
        'tangent_warning_threshold_deg': tangent_threshold_deg,
        'node_count': len(network['nodes']),
        'segment_count': len(network['segments']),
        'switch_count': len(network.get('switches', {})),
        'segment_lengths': _segment_lengths(geometries),
        'endpoint_to_anchor_distances': endpoint_distances,
        'transition_diagnostics': transition_records,
        'warnings': warnings,
        'status': 'PASS' if not warnings else 'WARN',
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Validate the room 315 directed rail network and produce debug artifacts.'
    )
    parser.add_argument(
        '--network',
        type=Path,
        default=_default_network_path(),
        help='rail_network_right.yaml path.',
    )
    parser.add_argument(
        '--report',
        type=Path,
        default=_default_report_path(),
        help='Validation report YAML path.',
    )
    parser.add_argument(
        '--plot',
        type=Path,
        default=_default_plot_path(),
        help='Debug plot PNG path.',
    )
    parser.add_argument(
        '--duplicate-tolerance',
        type=float,
        default=1e-6,
        help='Distance threshold for counting consecutive duplicate CSV points.',
    )
    parser.add_argument(
        '--tangent-threshold-deg',
        type=float,
        default=25.0,
        help='Warning threshold for transition tangent mismatch.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    network_path = args.network.resolve()
    network = _load_network(network_path)
    geometries = _load_segment_geometry(
        network,
        network_path=network_path,
        duplicate_tolerance=args.duplicate_tolerance,
    )
    report = _build_report(
        network,
        network_path=network_path,
        geometries=geometries,
        duplicate_tolerance=args.duplicate_tolerance,
        tangent_threshold_deg=args.tangent_threshold_deg,
    )

    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('w') as handle:
        yaml.safe_dump(report, handle, sort_keys=False)

    _draw_network_plot(network, geometries, args.plot)

    print(f'Validated {report["segment_count"]} segments and {report["node_count"]} nodes.')
    print(f'Report: {args.report}')
    print(f'Plot: {args.plot}')
    print(f'Status: {report["status"]} ({len(report["warnings"])} warnings)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
