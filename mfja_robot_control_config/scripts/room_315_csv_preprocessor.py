#!/usr/bin/env python3

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from typing import Iterable, List, Sequence

import yaml


EXPECTED_SEGMENTS: Sequence[str] = (
    'A1G',
    'A1S',
    'A2G',
    'A2S',
    'A3G',
    'A3S',
    'A4G',
    'A4S',
    'A12E',
    'A12I',
    'A14',
    'A23',
    'A34E',
    'A34I',
)


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
class CsvReadResult:
    points: List[Point3D]
    extra_header_fields: List[str]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_input_dir() -> Path:
    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / 'raw_segments'
    )


def _default_output_dir() -> Path:
    return _repo_root() / 'mfja_robot_control_config' / 'config' / 'room_315_kinematics' / 'normalized_segments'


def _default_summary_path() -> Path:
    return _repo_root() / 'mfja_robot_control_config' / 'config' / 'room_315_kinematics' / 'segment_summary.yaml'


def _format_float(value: float) -> str:
    return f'{value:.9f}'.rstrip('0').rstrip('.')


def _read_points(csv_path: Path) -> CsvReadResult:
    with csv_path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        fieldname_set = set(fieldnames)
        required = {'index', 'x', 'y', 'z'}
        missing = required - fieldname_set
        if missing:
            raise ValueError(f'{csv_path} is missing required columns: {sorted(missing)}')

        extra_header_fields = [
            fieldname
            for fieldname in fieldnames
            if fieldname not in required
        ]

        points = [
            Point3D(
                x=float(row['x']),
                y=float(row['y']),
                z=float(row['z']),
            )
            for row in reader
        ]

    if len(points) < 2:
        raise ValueError(f'{csv_path} must contain at least 2 data points')
    return CsvReadResult(points=points, extra_header_fields=extra_header_fields)


def _remove_consecutive_duplicates(
    points: Sequence[Point3D],
    duplicate_tolerance: float,
) -> List[Point3D]:
    normalized = [points[0]]
    for point in points[1:]:
        if normalized[-1].distance_to(point) <= duplicate_tolerance:
            continue
        normalized.append(point)

    if len(normalized) < 2:
        raise ValueError(
            'A segment collapsed to fewer than 2 distinct points after duplicate removal'
        )
    return normalized


def _accumulate_arc_length(points: Sequence[Point3D]) -> List[float]:
    arc_lengths = [0.0]
    total = 0.0
    for previous, current in zip(points, points[1:]):
        total += previous.distance_to(current)
        arc_lengths.append(total)
    return arc_lengths


def _normalize_vector(dx: float, dy: float, dz: float) -> List[float]:
    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if norm == 0.0:
        raise ValueError('Encountered a zero-length tangent vector after normalization')
    return [dx / norm, dy / norm, dz / norm]


def _compute_edge_vectors(points: Sequence[Point3D]) -> List[List[float]]:
    return [
        _normalize_vector(
            current.x - previous.x,
            current.y - previous.y,
            current.z - previous.z,
        )
        for previous, current in zip(points, points[1:])
    ]


def _signed_heading_delta(first_yaw: float, second_yaw: float) -> float:
    return math.atan2(math.sin(second_yaw - first_yaw), math.cos(second_yaw - first_yaw))


def _edge_turn_diagnostics(points: Sequence[Point3D]) -> dict:
    edge_vectors = _compute_edge_vectors(points)
    if len(edge_vectors) < 2:
        return {
            'max_heading_jump_deg': 0.0,
            'sharp_turn_indices': [],
            'warning_flags': [],
        }

    sharp_turn_indices = []
    max_heading_jump_deg = 0.0
    for index, (first, second) in enumerate(zip(edge_vectors, edge_vectors[1:]), start=1):
        first_yaw = math.atan2(first[1], first[0])
        second_yaw = math.atan2(second[1], second[0])
        heading_jump_deg = abs(math.degrees(_signed_heading_delta(first_yaw, second_yaw)))
        max_heading_jump_deg = max(max_heading_jump_deg, heading_jump_deg)
        if heading_jump_deg >= 135.0:
            sharp_turn_indices.append(index)

    warning_flags = []
    if sharp_turn_indices:
        warning_flags.append('sharp_heading_jump')

    return {
        'max_heading_jump_deg': max_heading_jump_deg,
        'sharp_turn_indices': sharp_turn_indices,
        'warning_flags': warning_flags,
    }


def _compute_tangents(points: Sequence[Point3D]) -> List[List[float]]:
    tangents: List[List[float]] = []
    for index, point in enumerate(points):
        if index == 0:
            reference_a = point
            reference_b = points[index + 1]
        elif index == len(points) - 1:
            reference_a = points[index - 1]
            reference_b = point
        else:
            reference_a = points[index - 1]
            reference_b = points[index + 1]

        tangents.append(
            _normalize_vector(
                reference_b.x - reference_a.x,
                reference_b.y - reference_a.y,
                reference_b.z - reference_a.z,
            )
        )
    return tangents


def _write_normalized_csv(
    csv_path: Path,
    points: Sequence[Point3D],
    arc_lengths: Sequence[float],
    tangents: Sequence[Sequence[float]],
) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['index', 'x', 'y', 'z', 's', 'tx', 'ty', 'tz', 'yaw'])
        for index, (point, arc_length, tangent) in enumerate(zip(points, arc_lengths, tangents)):
            tx, ty, tz = tangent
            yaw = math.atan2(ty, tx)
            writer.writerow(
                [
                    index,
                    _format_float(point.x),
                    _format_float(point.y),
                    _format_float(point.z),
                    _format_float(arc_length),
                    _format_float(tx),
                    _format_float(ty),
                    _format_float(tz),
                    _format_float(yaw),
                ]
            )


def _bbox(points: Iterable[Point3D]) -> dict:
    point_list = list(points)
    return {
        'min_x': min(point.x for point in point_list),
        'max_x': max(point.x for point in point_list),
        'min_y': min(point.y for point in point_list),
        'max_y': max(point.y for point in point_list),
        'min_z': min(point.z for point in point_list),
        'max_z': max(point.z for point in point_list),
    }


def _segment_summary(
    name: str,
    raw_points: Sequence[Point3D],
    normalized_points: Sequence[Point3D],
    arc_lengths: Sequence[float],
    output_csv_path: Path,
    extra_header_fields: Sequence[str],
) -> dict:
    raw_count = len(raw_points)
    normalized_count = len(normalized_points)
    z_values = [point.z for point in normalized_points]
    diagnostics = _edge_turn_diagnostics(normalized_points)
    warning_flags = list(diagnostics['warning_flags'])
    if extra_header_fields:
        warning_flags.append('extra_header_fields')
    return {
        'source_csv': str(Path('raw_segments') / f'{name}.csv'),
        'normalized_csv': str(output_csv_path),
        'raw_point_count': raw_count,
        'normalized_point_count': normalized_count,
        'consecutive_duplicates_removed': raw_count - normalized_count,
        'start_point': normalized_points[0].as_list(),
        'end_point': normalized_points[-1].as_list(),
        'approx_length': arc_lengths[-1],
        'bbox': _bbox(normalized_points),
        'z_stats': {
            'min': min(z_values),
            'max': max(z_values),
            'mean': fmean(z_values),
        },
        'extra_header_fields': list(extra_header_fields),
        'max_heading_jump_deg': diagnostics['max_heading_jump_deg'],
        'sharp_turn_indices': diagnostics['sharp_turn_indices'],
        'warning_flags': warning_flags,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Phase 1 preprocessing for room 315 shuttle rail segments. '
            'This tool only normalizes per-segment geometry and does not infer topology.'
        )
    )
    parser.add_argument(
        '--input-dir',
        type=Path,
        default=_default_input_dir(),
        help='Directory containing raw segment CSV files.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=_default_output_dir(),
        help='Directory where normalized CSV files will be written.',
    )
    parser.add_argument(
        '--summary-path',
        type=Path,
        default=_default_summary_path(),
        help='YAML summary file written after preprocessing.',
    )
    parser.add_argument(
        '--duplicate-tolerance',
        type=float,
        default=1e-6,
        help='Distance threshold for removing consecutive duplicate points.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    summary_path = args.summary_path.resolve()

    if not input_dir.exists():
        raise FileNotFoundError(f'Input directory does not exist: {input_dir}')

    found_segment_names = sorted(path.stem for path in input_dir.glob('*.csv'))
    missing = sorted(set(EXPECTED_SEGMENTS) - set(found_segment_names))
    unexpected = sorted(set(found_segment_names) - set(EXPECTED_SEGMENTS))
    if missing:
        raise ValueError(f'Missing expected segment CSV files: {missing}')
    if unexpected:
        raise ValueError(f'Unexpected segment CSV files found: {unexpected}')

    summary = {
        'phase': 1,
        'description': 'CSV preprocessing and normalization for room 315 shuttle segments',
        'input_dir': str(input_dir),
        'output_dir': str(output_dir),
        'input_stage': 'raw',
        'duplicate_tolerance': args.duplicate_tolerance,
        'segment_order': list(EXPECTED_SEGMENTS),
        'segments': {},
    }

    for segment_name in EXPECTED_SEGMENTS:
        source_csv_path = input_dir / f'{segment_name}.csv'
        read_result = _read_points(source_csv_path)
        raw_points = read_result.points
        normalized_points = _remove_consecutive_duplicates(
            raw_points,
            duplicate_tolerance=args.duplicate_tolerance,
        )
        arc_lengths = _accumulate_arc_length(normalized_points)
        tangents = _compute_tangents(normalized_points)
        output_csv_path = output_dir / f'{segment_name}.csv'
        _write_normalized_csv(output_csv_path, normalized_points, arc_lengths, tangents)
        summary['segments'][segment_name] = _segment_summary(
            name=segment_name,
            raw_points=raw_points,
            normalized_points=normalized_points,
            arc_lengths=arc_lengths,
            output_csv_path=output_csv_path.relative_to(_repo_root()),
            extra_header_fields=read_result.extra_header_fields,
        )

    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open('w') as handle:
        yaml.safe_dump(summary, handle, sort_keys=False)

    print(f'Processed {len(EXPECTED_SEGMENTS)} rail segments.')
    print(f'Normalized CSV output: {output_dir}')
    print(f'Summary file: {summary_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
