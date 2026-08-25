import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


MODULE_PATH = Path(__file__).parents[1] / "hpp" / "staubli_trajectory_export.py"
SPEC = importlib.util.spec_from_file_location("staubli_trajectory_export", MODULE_PATH)
trajectory_export = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trajectory_export)

LAUNCHER_PATH = (
    Path(__file__).parents[1] / "scripts" / "room315_export_staubli_line.py"
)
LAUNCHER_SPEC = importlib.util.spec_from_file_location(
    "room315_export_staubli_line", LAUNCHER_PATH
)
trajectory_launcher = importlib.util.module_from_spec(LAUNCHER_SPEC)
LAUNCHER_SPEC.loader.exec_module(trajectory_launcher)


JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
CONFIGS = [
    [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
    [0.5, 0.4, 0.3, 0.2, 0.1, 0.0],
]
TIMES = [0.0, 1.25]


def test_joint_trajectory_json_round_trips_as_payload():
    rendered = trajectory_export.render_joint_trajectory(
        CONFIGS, TIMES, JOINT_NAMES
    )

    assert rendered.startswith("{\n")
    assert "ros2 topic pub" not in rendered
    payload = json.loads(rendered)
    assert payload["joint_names"] == JOINT_NAMES
    assert payload["points"][1]["time_from_start"] == {
        "sec": 1,
        "nanosec": 250_000_000,
    }
    assert all(
        len(point["positions"]) == 6
        and all(math.isfinite(value) for value in point["positions"])
        and point["velocities"] == [0.0] * 6
        and set(point) == {"positions", "velocities", "time_from_start"}
        for point in payload["points"]
    )


def test_launcher_separates_planner_chatter_from_payload():
    rendered = trajectory_export.render_joint_trajectory(
        CONFIGS, TIMES, JOINT_NAMES
    )
    payload, prefix, suffix = trajectory_launcher._extract_payload(
        f"Starting HPP planner...\n{rendered}\n"
    )

    assert payload["joint_names"] == JOINT_NAMES
    assert prefix == "Starting HPP planner...\n"
    assert suffix == "\n"


def test_launcher_main_emits_only_payload(monkeypatch, capsys):
    rendered = trajectory_export.render_joint_trajectory(
        CONFIGS, TIMES, JOINT_NAMES
    )

    def run(arguments, *, stdout, text):
        assert arguments[1:] == ["--print-joint-trajectory", "--samples", "2"]
        assert stdout == trajectory_launcher.subprocess.PIPE
        assert text is True
        return SimpleNamespace(
            returncode=0,
            stdout=f"Starting HPP planner...\n{rendered}\n",
        )

    monkeypatch.setattr(trajectory_launcher.subprocess, "run", run)
    monkeypatch.setattr(sys, "argv", ["exporter", "--samples", "2"])

    assert trajectory_launcher.main() == 0
    captured = capsys.readouterr()
    assert json.loads(captured.out)["joint_names"] == JOINT_NAMES
    assert "Starting HPP planner..." in captured.err


@pytest.mark.parametrize(
    ("configs", "times"),
    [
        ([], []),
        (CONFIGS, [0.0]),
        ([[0.0] * 5], [0.0]),
        ([[0.0] * 5 + [float("nan")]], [0.0]),
        ([[0.0] * 6], [-1.0]),
    ],
)
def test_invalid_trajectory_is_rejected(configs, times):
    with pytest.raises(ValueError):
        trajectory_export.joint_trajectory_payload(configs, times, JOINT_NAMES)
