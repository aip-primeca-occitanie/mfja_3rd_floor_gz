#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

from room_315_kinematic_shuttle import (
    CUBIC_HERMITE_PATH_BACKEND,
    POLYLINE_PATH_BACKEND,
    Point3D,
    RailNetwork,
)


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


def _default_report_path() -> Path:
    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / 'continuous_path_report.yaml'
    )


def _default_plot_path() -> Path:
    return (
        _repo_root()
        / 'mfja_robot_control_config'
        / 'config'
        / 'room_315_kinematics'
        / 'debug_plots'
        / 'continuous_path_validation.png'
    )


def _point_to_segment_distance(point: Point3D, start: Point3D, end: Point3D) -> float:
    vx = end.x - start.x
    vy = end.y - start.y
    vz = end.z - start.z
    wx = point.x - start.x
    wy = point.y - start.y
    wz = point.z - start.z
    length_sq = vx * vx + vy * vy + vz * vz
    if length_sq <= 1e-18:
        return point.distance_to(start)

    ratio = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / length_sq))
    projection = Point3D(
        x=start.x + ratio * vx,
        y=start.y + ratio * vy,
        z=start.z + ratio * vz,
    )
    return point.distance_to(projection)


def _point_to_polyline_distance(point: Point3D, points: Sequence[Point3D]) -> float:
    return min(
        _point_to_segment_distance(point, start, end)
        for start, end in zip(points, points[1:])
    )


def _signed_heading_delta(first_yaw: float, second_yaw: float) -> float:
    return math.atan2(math.sin(second_yaw - first_yaw), math.cos(second_yaw - first_yaw))


def _sample_count(length: float, sample_step_m: float) -> int:
    return max(20, int(math.ceil(length / max(sample_step_m, 1e-6))) + 1)


def _segment_report(
    segment_name: str,
    polyline_segment,
    continuous_segment,
    sample_step_m: float,
    max_deviation_warning_m: float,
    max_length_delta_warning_m: float,
    max_yaw_step_warning_deg: float,
) -> tuple[dict, list[dict]]:
    sample_count = _sample_count(continuous_segment.length, sample_step_m)
    deviations = []
    yaws = []
    samples = []
    for index in range(sample_count):
        s = continuous_segment.length * index / (sample_count - 1)
        point, yaw = continuous_segment.sample(s)
        samples.append((point, yaw))
        yaws.append(yaw)
        deviations.append(_point_to_polyline_distance(point, polyline_segment.points))

    max_yaw_step_deg = 0.0
    for first_yaw, second_yaw in zip(yaws, yaws[1:]):
        max_yaw_step_deg = max(
            max_yaw_step_deg,
            abs(math.degrees(_signed_heading_delta(first_yaw, second_yaw))),
        )

    start_poly, _ = polyline_segment.sample(0.0)
    end_poly, _ = polyline_segment.sample(polyline_segment.length)
    start_cont, _ = continuous_segment.sample(0.0)
    end_cont, _ = continuous_segment.sample(continuous_segment.length)
    length_delta_m = continuous_segment.length - polyline_segment.length
    length_delta_pct = 100.0 * length_delta_m / max(polyline_segment.length, 1e-12)

    warnings = []
    max_deviation_m = max(deviations)
    mean_deviation_m = sum(deviations) / len(deviations)
    if max_deviation_m > max_deviation_warning_m:
        warnings.append(
            {
                'type': 'continuous_path_deviation',
                'segment': segment_name,
                'max_deviation_m': max_deviation_m,
                'limit_m': max_deviation_warning_m,
            }
        )
    if abs(length_delta_m) > max_length_delta_warning_m:
        warnings.append(
            {
                'type': 'continuous_path_length_delta',
                'segment': segment_name,
                'length_delta_m': length_delta_m,
                'limit_m': max_length_delta_warning_m,
            }
        )
    if max_yaw_step_deg > max_yaw_step_warning_deg:
        warnings.append(
            {
                'type': 'continuous_path_yaw_step',
                'segment': segment_name,
                'max_yaw_step_deg': max_yaw_step_deg,
                'limit_deg': max_yaw_step_warning_deg,
            }
        )

    report = {
        'polyline_length_m': polyline_segment.length,
        'continuous_length_m': continuous_segment.length,
        'length_delta_m': length_delta_m,
        'length_delta_pct': length_delta_pct,
        'max_deviation_from_polyline_m': max_deviation_m,
        'mean_deviation_from_polyline_m': mean_deviation_m,
        'max_yaw_step_deg': max_yaw_step_deg,
        'start_endpoint_delta_m': start_cont.distance_to(start_poly),
        'end_endpoint_delta_m': end_cont.distance_to(end_poly),
        'sample_count': sample_count,
    }
    return report, warnings


def _plot_segments(polyline_network, continuous_network, output_path: Path, sample_step_m: float) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(18, 8), constrained_layout=True)
    axes[0].set_title('Room 315 path input - calibrated CSV polylines')
    axes[1].set_title('Room 315 path backend - continuous cubic Hermite')

    for axis in axes:
        axis.set_aspect('equal', adjustable='box')
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.grid(True, alpha=0.25)

    for segment_name, polyline_segment in polyline_network.segments.items():
        xs = [point.x for point in polyline_segment.points]
        ys = [point.y for point in polyline_segment.points]
        axes[0].plot(xs, ys, '-o', linewidth=1.1, markersize=2.5)
        midpoint = polyline_segment.points[len(polyline_segment.points) // 2]
        axes[0].text(midpoint.x, midpoint.y, segment_name, fontsize=8)

        continuous_segment = continuous_network.segments[segment_name]
        sample_count = _sample_count(continuous_segment.length, sample_step_m)
        sampled_points = [
            continuous_segment.sample(continuous_segment.length * index / (sample_count - 1))[0]
            for index in range(sample_count)
        ]
        axes[1].plot(
            [point.x for point in sampled_points],
            [point.y for point in sampled_points],
            '-',
            linewidth=1.6,
        )
        axes[1].plot(xs, ys, '.', color='black', alpha=0.35, markersize=2)
        midpoint = sampled_points[len(sampled_points) // 2]
        axes[1].text(midpoint.x, midpoint.y, segment_name, fontsize=8)

    fig.suptitle('Continuous path validation: CSV measurement vs parametrized path backend')
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Validate the Room 315 continuous path backend against calibrated CSV geometry.'
    )
    parser.add_argument('--network', type=Path, default=_default_network_path())
    parser.add_argument('--report', type=Path, default=_default_report_path())
    parser.add_argument('--plot', type=Path, default=_default_plot_path())
    parser.add_argument('--sample-step-m', type=float, default=0.01)
    parser.add_argument('--arc-length-samples-per-edge', type=int, default=16)
    parser.add_argument('--max-deviation-warning-m', type=float, default=0.035)
    parser.add_argument('--max-length-delta-warning-m', type=float, default=0.05)
    parser.add_argument('--max-yaw-step-warning-deg', type=float, default=18.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    polyline_network = RailNetwork.from_yaml(args.network, path_backend=POLYLINE_PATH_BACKEND)
    continuous_network = RailNetwork.from_yaml(
        args.network,
        path_backend=CUBIC_HERMITE_PATH_BACKEND,
        arc_length_samples_per_edge=args.arc_length_samples_per_edge,
    )

    segment_reports = {}
    warnings = []
    for segment_name in polyline_network.segments:
        report, segment_warnings = _segment_report(
            segment_name=segment_name,
            polyline_segment=polyline_network.segments[segment_name],
            continuous_segment=continuous_network.segments[segment_name],
            sample_step_m=args.sample_step_m,
            max_deviation_warning_m=args.max_deviation_warning_m,
            max_length_delta_warning_m=args.max_length_delta_warning_m,
            max_yaw_step_warning_deg=args.max_yaw_step_warning_deg,
        )
        segment_reports[segment_name] = report
        warnings.extend(segment_warnings)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    report_data = {
        'path_backend': CUBIC_HERMITE_PATH_BACKEND,
        'reference_backend': POLYLINE_PATH_BACKEND,
        'sample_step_m': args.sample_step_m,
        'arc_length_samples_per_edge': args.arc_length_samples_per_edge,
        'warning_limits': {
            'max_deviation_warning_m': args.max_deviation_warning_m,
            'max_length_delta_warning_m': args.max_length_delta_warning_m,
            'max_yaw_step_warning_deg': args.max_yaw_step_warning_deg,
        },
        'segments': segment_reports,
        'warnings': warnings,
        'status': 'PASS' if not warnings else 'WARN',
    }
    with args.report.open('w') as handle:
        yaml.safe_dump(report_data, handle, sort_keys=False)

    _plot_segments(polyline_network, continuous_network, args.plot, args.sample_step_m)

    print(f'Validated continuous paths for {len(segment_reports)} segments.')
    print(f'Report: {args.report}')
    print(f'Plot: {args.plot}')
    print(f"Status: {report_data['status']} ({len(warnings)} warnings)")
    return 0 if not warnings else 1


if __name__ == '__main__':
    raise SystemExit(main())
