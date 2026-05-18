#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


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


def _default_raw_dir() -> Path:
    return _default_config_dir() / 'raw_segments'


def _default_normalized_dir() -> Path:
    return _default_config_dir() / 'normalized_segments'


def _default_output_path() -> Path:
    return _default_config_dir() / 'debug_plots' / 'room_315_segments_overview.png'


def _parse_segment_args(raw_values: Iterable[str]) -> List[str]:
    if not raw_values:
        return list(EXPECTED_SEGMENTS)

    parsed: List[str] = []
    for raw_value in raw_values:
        for token in raw_value.split(','):
            name = token.strip()
            if not name:
                continue
            parsed.append(name)

    invalid = [name for name in parsed if name not in EXPECTED_SEGMENTS]
    if invalid:
        raise ValueError(f'Unknown segment names: {invalid}')
    return parsed


def _read_xy_points(csv_path: Path) -> List[tuple[float, float]]:
    with csv_path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        return [(float(row['x']), float(row['y'])) for row in reader]


def _load_segments(base_dir: Path, segment_names: Sequence[str]) -> Dict[str, List[tuple[float, float]]]:
    segments = {}
    for name in segment_names:
        csv_path = base_dir / f'{name}.csv'
        if not csv_path.exists():
            raise FileNotFoundError(f'Missing CSV file: {csv_path}')
        segments[name] = _read_xy_points(csv_path)
    return segments


def _segment_label_point(points: Sequence[tuple[float, float]]) -> tuple[float, float]:
    return points[len(points) // 2]


def _signed_heading_delta(first_yaw: float, second_yaw: float) -> float:
    return math.atan2(math.sin(second_yaw - first_yaw), math.cos(second_yaw - first_yaw))


def _has_sharp_heading_jump(points: Sequence[tuple[float, float]], threshold_deg: float = 135.0) -> bool:
    if len(points) < 3:
        return False

    headings = []
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        if dx == 0.0 and dy == 0.0:
            continue
        headings.append(math.atan2(dy, dx))

    for first_heading, second_heading in zip(headings, headings[1:]):
        jump_deg = abs(math.degrees(_signed_heading_delta(first_heading, second_heading)))
        if jump_deg >= threshold_deg:
            return True
    return False


def _estimate_tangents(points: Sequence[tuple[float, float]]) -> List[tuple[float, float]]:
    tangents: List[tuple[float, float]] = []
    for index, point in enumerate(points):
        if index == 0:
            tangent = (
                points[index + 1][0] - point[0],
                points[index + 1][1] - point[1],
            )
        elif index == len(points) - 1:
            tangent = (
                point[0] - points[index - 1][0],
                point[1] - points[index - 1][1],
            )
        else:
            tangent = (
                0.5 * (points[index + 1][0] - points[index - 1][0]),
                0.5 * (points[index + 1][1] - points[index - 1][1]),
            )
        tangents.append(tangent)
    return tangents


def _sample_smooth_curve(
    points: Sequence[tuple[float, float]],
    samples_per_edge: int = 16,
) -> List[tuple[float, float]]:
    if len(points) < 2:
        return list(points)

    tangents = _estimate_tangents(points)
    sampled: List[tuple[float, float]] = [points[0]]
    for index in range(len(points) - 1):
        p0 = points[index]
        p1 = points[index + 1]
        m0 = tangents[index]
        m1 = tangents[index + 1]

        for step in range(1, samples_per_edge + 1):
            t = step / samples_per_edge
            h00 = 2.0 * t**3 - 3.0 * t**2 + 1.0
            h10 = t**3 - 2.0 * t**2 + t
            h01 = -2.0 * t**3 + 3.0 * t**2
            h11 = t**3 - t**2
            x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
            y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
            sampled.append((x, y))
    return sampled


def _plot_dataset(
    axis,
    dataset: Dict[str, List[tuple[float, float]]],
    title: str,
    annotate_indices: bool,
    smooth_preview: bool,
) -> None:
    color_map = plt.get_cmap('tab20')
    for index, (name, points) in enumerate(dataset.items()):
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        color = color_map(index % 20)
        has_warning = _has_sharp_heading_jump(points)
        label_text = f'!{name}' if has_warning else name

        if smooth_preview:
            smooth_points = _sample_smooth_curve(points)
            smooth_xs = [point[0] for point in smooth_points]
            smooth_ys = [point[1] for point in smooth_points]
            axis.plot(smooth_xs, smooth_ys, '-', color=color, linewidth=2.0, alpha=0.95)
            axis.plot(xs, ys, '--', color=color, linewidth=0.9, alpha=0.4)
            axis.scatter(xs, ys, color=[color], s=16, alpha=0.55)
        else:
            axis.plot(xs, ys, '-', color=color, linewidth=1.4, alpha=0.85)
            axis.scatter(xs, ys, color=[color], s=18, alpha=0.95)

        start_x, start_y = points[0]
        end_x, end_y = points[-1]
        axis.scatter([start_x], [start_y], color='green', s=55, marker='o', edgecolors='black', linewidths=0.5)
        axis.scatter([end_x], [end_y], color='red', s=55, marker='s', edgecolors='black', linewidths=0.5)

        label_x, label_y = _segment_label_point(points)
        axis.text(label_x, label_y, label_text, fontsize=9, weight='bold', color=color)

        if annotate_indices:
            for point_index, (x, y) in enumerate(points):
                axis.text(x, y, str(point_index), fontsize=6, alpha=0.75)

    axis.set_title(title)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_aspect('equal', adjustable='box')
    axis.grid(True, alpha=0.25)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Render a top-down debug plot of room 315 shuttle rail segment points. '
            'The image is intended for offline visual validation.'
        )
    )
    parser.add_argument(
        '--raw-dir',
        type=Path,
        default=_default_raw_dir(),
        help='Directory containing the raw segment CSV files.',
    )
    parser.add_argument(
        '--normalized-dir',
        type=Path,
        default=_default_normalized_dir(),
        help='Directory containing normalized segment CSV files.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=_default_output_path(),
        help='PNG file path to write.',
    )
    parser.add_argument(
        '--segment',
        action='append',
        default=[],
        help='Segment name to plot. Can be repeated or given as a comma-separated list.',
    )
    parser.add_argument(
        '--annotate-indices',
        action='store_true',
        help='Draw point indices next to every point.',
    )
    parser.add_argument(
        '--left-title',
        default='Room 315 segments - raw CSV points / actual polyline',
        help='Title used for the left subplot.',
    )
    parser.add_argument(
        '--right-title',
        default='Room 315 segments - smooth preview from normalized points',
        help='Title used for the right subplot.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    segment_names = _parse_segment_args(args.segment)
    raw_dir = args.raw_dir.resolve()
    normalized_dir = args.normalized_dir.resolve()
    output_path = args.output.resolve()

    raw_segments = _load_segments(raw_dir, segment_names)
    normalized_segments = _load_segments(normalized_dir, segment_names)

    figure, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    _plot_dataset(
        axes[0],
        raw_segments,
        title=args.left_title,
        annotate_indices=args.annotate_indices,
        smooth_preview=False,
    )
    _plot_dataset(
        axes[1],
        normalized_segments,
        title=args.right_title,
        annotate_indices=args.annotate_indices,
        smooth_preview=True,
    )

    plotted_names = ', '.join(segment_names)
    figure.suptitle(
        'Room 315 rail segment debug plot\n'
        f'Green circle = start, red square = end, !label = sharp local reversal warning\n'
        f'Segments: {plotted_names}',
        fontsize=14,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=220)
    plt.close(figure)

    print(f'Wrote debug plot: {output_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
