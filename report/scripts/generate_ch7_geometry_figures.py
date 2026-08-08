#!/usr/bin/env python3
"""Generate the current, data-backed geometry figures used in Chapter 7.

The script deliberately imports the runtime rail and device loaders.  The
figures therefore use the same CSV samples, cubic-Hermite evaluator and YAML
device positions as the Room 315 controller instead of maintaining a second
hand-drawn geometry model.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patheffects as path_effects
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_SCRIPTS = REPOSITORY_ROOT / "mfja_robot_control_config" / "scripts"
if str(RUNTIME_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(RUNTIME_SCRIPTS))

from room_315_kinematic_shuttle import (  # noqa: E402
    CUBIC_HERMITE_PATH_BACKEND,
    RailNetwork,
)
from room_315_rail_devices import load_rail_devices  # noqa: E402


CONFIGURATION_DIR = (
    REPOSITORY_ROOT
    / "mfja_robot_control_config"
    / "config"
    / "room_315_kinematics"
)
NETWORK_PATH = CONFIGURATION_DIR / "rail_network_right.yaml"
DEVICES_PATH = CONFIGURATION_DIR / "rail_devices_right.yaml"
FIGURE_DIR = REPOSITORY_ROOT / "report" / "figures"

GEOMETRY_OUTPUT = FIGURE_DIR / "f07_calibrated_rail_geometry_current.png"
DEVICES_OUTPUT = FIGURE_DIR / "f07_right_rail_devices_current.png"

EXTERIOR_SEGMENTS = {"A1E", "A2E", "A3E", "A4E", "A12E", "A34E"}
INTERIOR_SEGMENTS = {"A1I", "A2I", "A3I", "A4I", "A12I", "A34I"}
TRUNK_SEGMENTS = {"A14", "A23"}

EXTERIOR_COLOR = "#f28e2b"
INTERIOR_COLOR = "#edc948"
INTERIOR_LABEL_COLOR = "#aa7600"
TRUNK_COLOR = "#243b53"
RAW_POINT_COLOR = "#171717"
SLOT_COLOR = "#234a9f"
SWITCH_COLOR = "#dfead9"
PATH_BACKGROUND = "#cfd5dc"
DA_MAIN_COLOR = "#2a9d3f"
DA_EXTERIOR_COLOR = "#f58220"
DA_INTERIOR_COLOR = "#e9b72f"
DA_INTERIOR_LABEL_COLOR = "#a96f00"
STOPPER_COLOR = "#3568d4"
STOPPER_SENSOR_COLOR = "#7a4ea3"


def plot_xy(point) -> tuple[float, float]:
    """Use the report orientation: CAD x horizontally and -CAD y vertically."""

    return point.x, -point.y


def sample_segment(segment, sample_count: int = 420) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for index in range(sample_count):
        s = segment.length * index / (sample_count - 1)
        point, _ = segment.sample(s)
        x, y = plot_xy(point)
        xs.append(x)
        ys.append(y)
    return xs, ys


def segment_color(name: str) -> str:
    if name in EXTERIOR_SEGMENTS:
        return EXTERIOR_COLOR
    if name in INTERIOR_SEGMENTS:
        return INTERIOR_COLOR
    return TRUNK_COLOR


def add_direction_arrow(ax, segment, color: str, fraction: float = 0.56) -> None:
    start, _ = segment.sample(segment.length * (fraction - 0.035))
    end, _ = segment.sample(segment.length * (fraction + 0.035))
    arrow_start = plot_xy(start)
    arrow_end = plot_xy(end)
    for arrow_color, linewidth, mutation_scale, zorder in (
        ("white", 4.2, 17, 8),
        (color, 1.9, 15, 9),
    ):
        ax.annotate(
            "",
            xy=arrow_end,
            xytext=arrow_start,
            arrowprops={
                "arrowstyle": "-|>",
                "color": arrow_color,
                "lw": linewidth,
                "mutation_scale": mutation_scale,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=zorder,
        )


def add_callout(
    ax,
    text: str,
    anchor: tuple[float, float],
    label_position: tuple[float, float],
    color: str,
    *,
    font_size: float = 10.5,
    horizontal_alignment: str = "center",
) -> None:
    ax.annotate(
        text,
        xy=anchor,
        xytext=label_position,
        ha=horizontal_alignment,
        va="center",
        fontsize=font_size,
        fontweight="bold",
        color=color,
        bbox={
            "boxstyle": "round,pad=0.18",
            "facecolor": "white",
            "edgecolor": color,
            "linewidth": 0.85,
            "alpha": 0.98,
        },
        arrowprops={
            "arrowstyle": "-",
            "color": color,
            "lw": 0.85,
            "shrinkA": 3,
            "shrinkB": 3,
        },
        zorder=12,
    )


def switch_positions(network: RailNetwork) -> dict[str, tuple[float, float]]:
    positions: dict[str, tuple[float, float]] = {}
    for switch_name, switch_configuration in network.switches.items():
        node_name = switch_configuration["controlled_node"]
        x, y, _ = network.config["nodes"][node_name]["xyz"]
        positions[switch_name] = (float(x), -float(y))
    return positions


def draw_switches(
    ax,
    network: RailNetwork,
    *,
    radius: float,
    font_size: float,
    alpha: float = 0.78,
    show_labels: bool = True,
) -> None:
    for name, (x, y) in switch_positions(network).items():
        ax.add_patch(
            Circle(
                (x, y),
                radius,
                facecolor=SWITCH_COLOR,
                edgecolor="#9fb394",
                linewidth=0.8,
                alpha=alpha,
                zorder=2,
            )
        )
        if not show_labels:
            continue
        label = ax.text(
            x,
            y,
            name,
            ha="center",
            va="center",
            fontsize=font_size,
            fontweight="bold",
            color="#314735",
            zorder=11,
        )
        label.set_path_effects(
            [path_effects.Stroke(linewidth=3.2, foreground="white"), path_effects.Normal()]
        )


def draw_offset_switch_labels(ax, network: RailNetwork) -> None:
    """Keep switch names legible without covering nearby DA markers."""

    label_positions = {
        "A4": (-14.405, 4.205),
        "A3": (-12.545, 4.125),
        "A2": (-12.545, 4.025),
        "A1": (-14.405, 3.975),
    }
    for name, anchor in switch_positions(network).items():
        ax.annotate(
            name,
            xy=anchor,
            xytext=label_positions[name],
            ha="center",
            va="center",
            fontsize=10.5,
            fontweight="bold",
            color="#314735",
            arrowprops={
                "arrowstyle": "-",
                "color": "#78906f",
                "lw": 0.75,
                "shrinkA": 2,
                "shrinkB": 3,
            },
            zorder=12,
        )


def draw_runtime_paths(
    ax,
    network: RailNetwork,
    *,
    colored: bool,
    linewidth: float,
    arrows: bool = False,
) -> None:
    for name, segment in network.segments.items():
        color = segment_color(name) if colored else PATH_BACKGROUND
        xs, ys = sample_segment(segment)
        ax.plot(
            xs,
            ys,
            color=color,
            lw=linewidth,
            solid_capstyle="round",
            zorder=3,
        )
        if arrows:
            add_direction_arrow(ax, segment, color)


def configure_rail_axis(
    ax,
    *,
    x_limits: tuple[float, float] = (-15.12, -12.03),
    y_limits: tuple[float, float] = (3.20, 4.98),
) -> None:
    ax.set_xlim(*x_limits)
    ax.set_ylim(*y_limits)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")


def segment_midpoint(network: RailNetwork, name: str) -> tuple[float, float]:
    point, _ = network.segments[name].sample(network.segments[name].length * 0.50)
    return plot_xy(point)


def segment_label_color(name: str) -> str:
    if name in INTERIOR_SEGMENTS:
        return INTERIOR_LABEL_COLOR
    return segment_color(name)


def generate_geometry_figure(network: RailNetwork, devices) -> None:
    figure, ax = plt.subplots(figsize=(13.2, 8.0), constrained_layout=False)
    figure.patch.set_facecolor("white")

    draw_runtime_paths(ax, network, colored=True, linewidth=4.0, arrows=True)

    raw_point_count = 0
    for segment in network.segments.values():
        raw_x = [point.x for point in segment.points]
        raw_y = [-point.y for point in segment.points]
        raw_point_count += len(raw_x)
        ax.scatter(
            raw_x,
            raw_y,
            s=18,
            facecolor=RAW_POINT_COLOR,
            edgecolor="#555555",
            linewidth=0.35,
            alpha=0.92,
            zorder=7,
        )

    if raw_point_count != 276:
        raise RuntimeError(f"Expected 276 CSV samples, found {raw_point_count}.")
    if set(network.segments) != EXTERIOR_SEGMENTS | INTERIOR_SEGMENTS | TRUNK_SEGMENTS:
        raise RuntimeError("The public 14-segment vocabulary changed; update this figure.")

    draw_switches(ax, network, radius=0.092, font_size=13.0)

    for slot_number, slot in sorted(devices.slots.items(), key=lambda item: int(item[0])):
        x, y = slot.x, -slot.y
        ax.scatter(
            [x],
            [y],
            s=120,
            marker="s",
            facecolor="white",
            edgecolor=SLOT_COLOR,
            linewidth=2.1,
            zorder=9,
        )
        label_y = 4.895 if y > 4.0 else 3.255
        ax.text(
            x,
            label_y,
            f"slot {slot_number}",
            ha="center",
            va="center",
            fontsize=11.2,
            fontweight="bold",
            color=SLOT_COLOR,
            zorder=10,
        )

    label_positions = {
        "A34E": (-13.36, 4.805),
        "A34I": (-13.36, 4.385),
        "A12I": (-13.36, 3.935),
        "A12E": (-13.36, 3.335),
        "A14": (-15.010, 4.075),
        "A23": (-12.165, 4.075),
        "A4E": (-14.430, 4.585),
        "A4I": (-14.015, 4.205),
        "A3E": (-12.185, 4.535),
        "A3I": (-12.650, 4.165),
        "A1E": (-14.430, 3.590),
        "A1I": (-14.015, 3.965),
        "A2E": (-12.185, 3.555),
        "A2I": (-12.675, 4.015),
    }
    for name, label_position in label_positions.items():
        add_callout(
            ax,
            name,
            segment_midpoint(network, name),
            label_position,
            segment_label_color(name),
            font_size=9.8 if len(name) > 3 else 10.2,
        )

    ax.set_title(
        "Current Room 315 right-rail source geometry: points and runtime reconstruction",
        fontsize=16.8,
        fontweight="bold",
        color="#17365d",
        pad=10,
    )

    legend_handles = [
        Line2D([0], [0], color=TRUNK_COLOR, lw=4.0, label="shared trunk"),
        Line2D([0], [0], color=EXTERIOR_COLOR, lw=4.0, label="exterior family"),
        Line2D([0], [0], color=INTERIOR_COLOR, lw=4.0, label="interior family"),
        Line2D(
            [0],
            [0],
            marker="o",
            color="none",
            markerfacecolor=RAW_POINT_COLOR,
            markeredgecolor="#555555",
            markersize=6.5,
            label="ordered CSV sample",
        ),
        Line2D(
            [0],
            [0],
            marker="s",
            color="none",
            markerfacecolor="white",
            markeredgecolor=SLOT_COLOR,
            markeredgewidth=1.7,
            markersize=7.5,
            label="configured slot point",
        ),
    ]
    ax.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.015),
        ncol=5,
        frameon=False,
        fontsize=10.5,
        handlelength=2.5,
        columnspacing=1.6,
    )
    configure_rail_axis(ax)
    figure.subplots_adjust(left=0.025, right=0.975, top=0.91, bottom=0.105)
    figure.savefig(GEOMETRY_OUTPUT, dpi=260, facecolor="white")
    plt.close(figure)


def da_color(name: str) -> str:
    if name.endswith("ER"):
        return DA_EXTERIOR_COLOR
    if name.endswith("IR"):
        return DA_INTERIOR_COLOR
    return DA_MAIN_COLOR


def da_label_color(name: str) -> str:
    if name.endswith("IR"):
        return DA_INTERIOR_LABEL_COLOR
    return da_color(name)


def draw_slot_and_dzi_markers(ax, devices) -> None:
    dzi_by_slot: dict[str, object] = {}
    for name, points in devices.position_sensors.items():
        match = re.fullmatch(r"DZI([1-4])R", name)
        if match:
            dzi_by_slot[match.group(1)] = points[0]

    if set(dzi_by_slot) != set(devices.slots):
        raise RuntimeError("DZI and slot identifiers are no longer one-to-one.")

    for slot_number, slot in sorted(devices.slots.items(), key=lambda item: int(item[0])):
        sensor = dzi_by_slot[slot_number]
        if abs(slot.x - sensor.x) > 1e-8 or abs(slot.y - sensor.y) > 1e-8:
            raise RuntimeError(f"slot {slot_number} no longer coincides with DZI{slot_number}R.")
        x, y = slot.x, -slot.y
        ax.scatter(
            [x],
            [y],
            s=138,
            marker="s",
            facecolor="white",
            edgecolor=SLOT_COLOR,
            linewidth=2.0,
            zorder=8,
        )
        ax.scatter(
            [x],
            [y],
            s=35,
            marker="s",
            facecolor="#d62828",
            edgecolor="#d62828",
            zorder=9,
        )
        label_y = 4.865 if y > 4.0 else 3.545
        ax.text(
            x,
            label_y,
            f"slot {slot_number}\nDZI{slot_number}R",
            ha="center",
            va="center",
            fontsize=9.8,
            fontweight="bold",
            color=SLOT_COLOR,
            linespacing=0.92,
            zorder=10,
        )


def mean_device_position(points) -> tuple[float, float]:
    return (
        sum(point.x for point in points) / len(points),
        -sum(point.y for point in points) / len(points),
    )


def draw_da_sensors(ax, devices) -> None:
    da_label_positions = {
        "DA4R": (-14.720, 4.330),
        "DA4ER": (-14.510, 4.610),
        "DA4IR": (-13.930, 4.485),
        "DA3R": (-12.090, 4.205),
        "DA3ER": (-12.085, 4.505),
        "DA3IR": (-12.855, 4.440),
        "DA1R": (-14.710, 3.835),
        "DA1ER": (-14.505, 3.535),
        "DA1IR": (-13.925, 3.665),
        "DA2R": (-12.085, 3.790),
        "DA2ER": (-12.090, 3.520),
        "DA2IR": (-12.920, 3.680),
    }
    for name, points in devices.position_sensors.items():
        if not name.startswith("DA"):
            continue
        color = da_color(name)
        for point in points:
            ax.scatter(
                [point.x],
                [-point.y],
                s=49,
                marker="o",
                facecolor=color,
                edgecolor="white",
                linewidth=0.9,
                zorder=13,
            )
        if len(points) > 1:
            mean_x, mean_y = mean_device_position(points)
            ax.scatter(
                [mean_x],
                [mean_y],
                s=105,
                marker="o",
                facecolor="none",
                edgecolor=color,
                linewidth=1.9,
                zorder=14,
            )
            label = f"{name} ×2"
        else:
            label = name
        add_callout(
            ax,
            label,
            mean_device_position(points),
            da_label_positions[name],
            da_label_color(name),
            font_size=9.2,
        )


def draw_stopper_pairs(ax, devices) -> None:
    label_positions = {
        ("A4", 0): (-14.545, 4.630),
        ("A4", 1): (-13.885, 4.460),
        ("A3", 0): (-12.085, 4.135),
        ("A1", 0): (-14.675, 3.705),
        ("A2", 0): (-12.090, 3.565),
        ("A2", 1): (-12.925, 3.705),
    }
    branch_names = {
        ("A4", 0): "A4 exterior",
        ("A4", 1): "A4 interior",
        ("A3", 0): "A3",
        ("A1", 0): "A1",
        ("A2", 0): "A2 exterior",
        ("A2", 1): "A2 interior",
    }

    for stopper_name, stopper_points in devices.stoppers.items():
        sensor_name = f"{stopper_name}_STOPPER_SENSOR"
        sensor_points = devices.position_sensors[sensor_name]
        if len(stopper_points) != len(sensor_points):
            raise RuntimeError(f"{stopper_name} stopper/sensor point count changed.")
        for index, (stopper, sensor) in enumerate(zip(stopper_points, sensor_points)):
            stopper_xy = (stopper.x, -stopper.y)
            sensor_xy = (sensor.x, -sensor.y)
            ax.plot(
                [sensor_xy[0], stopper_xy[0]],
                [sensor_xy[1], stopper_xy[1]],
                color=STOPPER_SENSOR_COLOR,
                lw=1.25,
                ls=(0, (2.5, 2.0)),
                zorder=7,
            )
            ax.scatter(
                [sensor_xy[0]],
                [sensor_xy[1]],
                s=52,
                marker="o",
                facecolor="white",
                edgecolor=STOPPER_SENSOR_COLOR,
                linewidth=1.8,
                zorder=9,
            )
            ax.scatter(
                [stopper_xy[0]],
                [stopper_xy[1]],
                s=67,
                marker="D",
                facecolor=STOPPER_COLOR,
                edgecolor="white",
                linewidth=0.9,
                zorder=10,
            )
            midpoint = (
                (sensor_xy[0] + stopper_xy[0]) / 2.0,
                (sensor_xy[1] + stopper_xy[1]) / 2.0,
            )
            add_callout(
                ax,
                branch_names[(stopper_name, index)],
                midpoint,
                label_positions[(stopper_name, index)],
                STOPPER_COLOR,
                font_size=9.2,
            )


def generate_devices_figure(network: RailNetwork, devices) -> None:
    figure, (sensor_ax, stopper_ax) = plt.subplots(
        2,
        1,
        figsize=(13.2, 10.2),
        constrained_layout=False,
    )
    figure.patch.set_facecolor("white")

    for ax in (sensor_ax, stopper_ax):
        draw_runtime_paths(ax, network, colored=False, linewidth=2.8)
        draw_switches(ax, network, radius=0.073, font_size=10.0, alpha=0.70)
        configure_rail_axis(ax, y_limits=(3.20, 4.95))

    # Main DA points are almost coincident with controlled switch nodes.  The
    # top panel therefore keeps the reference halos but moves switch names into
    # nearby whitespace so neither the markers nor their identities are hidden.
    sensor_ax.clear()
    draw_runtime_paths(sensor_ax, network, colored=False, linewidth=2.8)
    draw_switches(
        sensor_ax,
        network,
        radius=0.073,
        font_size=10.0,
        alpha=0.70,
        show_labels=False,
    )
    configure_rail_axis(sensor_ax, y_limits=(3.20, 4.95))

    sensor_ax.set_title(
        "(a) Configured indexing zones and approach/branch detectors",
        fontsize=14.2,
        fontweight="bold",
        color="#17365d",
        pad=6,
    )
    draw_slot_and_dzi_markers(sensor_ax, devices)
    draw_da_sensors(sensor_ax, devices)
    draw_offset_switch_labels(sensor_ax, network)

    stopper_ax.set_title(
        "(b) Stopper points and their derived sensors 0.10 m upstream",
        fontsize=14.2,
        fontweight="bold",
        color="#17365d",
        pad=6,
    )
    draw_stopper_pairs(stopper_ax, devices)

    sensor_legend = [
        Line2D(
            [0], [0], marker="s", color="none", markerfacecolor="#d62828",
            markeredgecolor=SLOT_COLOR, markeredgewidth=2.0, markersize=8.0,
            label="slot point / DZI",
        ),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor=DA_MAIN_COLOR,
            markeredgecolor="white", markersize=7.0, label="DA*R approach",
        ),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor=DA_EXTERIOR_COLOR,
            markeredgecolor="white", markersize=7.0, label="DA*ER exterior",
        ),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor=DA_INTERIOR_COLOR,
            markeredgecolor="white", markersize=7.0, label="DA*IR interior",
        ),
    ]
    sensor_ax.legend(
        handles=sensor_legend,
        loc="lower center",
        bbox_to_anchor=(0.5, -0.045),
        ncol=4,
        frameon=False,
        fontsize=9.6,
        columnspacing=1.4,
    )

    stopper_legend = [
        Line2D(
            [0], [0], marker="D", color="none", markerfacecolor=STOPPER_COLOR,
            markeredgecolor="white", markersize=7.5, label="stopper point",
        ),
        Line2D(
            [0], [0], marker="o", color="none", markerfacecolor="white",
            markeredgecolor=STOPPER_SENSOR_COLOR, markeredgewidth=1.7,
            markersize=7.5, label="linked binary sensor",
        ),
        Line2D(
            [0], [0], color=STOPPER_SENSOR_COLOR, lw=1.3,
            ls=(0, (2.5, 2.0)), label="0.10 m along the same segment",
        ),
    ]
    stopper_ax.legend(
        handles=stopper_legend,
        loc="lower center",
        bbox_to_anchor=(0.5, -0.015),
        ncol=3,
        frameon=False,
        fontsize=9.8,
        columnspacing=1.7,
    )

    figure.suptitle(
        "Current Room 315 right-rail device layout from rail_devices_right.yaml",
        fontsize=16.8,
        fontweight="bold",
        color="#17365d",
        y=0.985,
    )
    figure.subplots_adjust(left=0.025, right=0.975, top=0.935, bottom=0.045, hspace=0.18)
    figure.savefig(DEVICES_OUTPUT, dpi=260, facecolor="white")
    plt.close(figure)


def main() -> int:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    network = RailNetwork.from_yaml(
        NETWORK_PATH,
        path_backend=CUBIC_HERMITE_PATH_BACKEND,
        arc_length_samples_per_edge=16,
    )
    devices = load_rail_devices(DEVICES_PATH, network)
    generate_geometry_figure(network, devices)
    generate_devices_figure(network, devices)
    print(f"generated {GEOMETRY_OUTPUT}")
    print(f"generated {DEVICES_OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
