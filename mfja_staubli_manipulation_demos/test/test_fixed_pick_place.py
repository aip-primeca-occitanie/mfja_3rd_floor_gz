import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "hpp" / "room315_pick_place.py"


def load_pick_place(monkeypatch):
    execution = ModuleType("room315_execution")
    execution.calls = []
    execution.execute_plan = lambda *args: execution.calls.append(args)
    monkeypatch.setitem(sys.modules, "room315_execution", execution)

    profiles = ModuleType("room315_execution_profiles")
    profiles.EXECUTION_PROFILES = {"simulation": object(), "hardware": object()}

    def apply_execution_profile(args):
        if args.execution_profile == "simulation":
            args.trajectory_topic = "/staubli1/joint_trajectory"
            args.trajectory_action = None
            args.joint_state_topic = "/staubli1/joint_states"
            args.payload_output = "gazebo"
            args.gripper_output = "joint-trajectory"
        else:
            args.trajectory_topic = None
            args.trajectory_action = "/controller/follow_joint_trajectory"
            args.joint_state_topic = "/joint_states"
            args.payload_output = "none"
            args.gripper_output = "staubli-io"
        return args

    profiles.apply_execution_profile = apply_execution_profile
    profiles.requires_explicit_measured_start = (
        lambda args, explicit: args.execute
        and args.execution_profile == "hardware"
        and not explicit
    )
    monkeypatch.setitem(sys.modules, "room315_execution_profiles", profiles)

    planning = ModuleType("room315_planning")
    planning.calls = []

    def plan_manipulation(*args, **kwargs):
        planning.calls.append((args, kwargs))
        return ["planned-segment"]

    planning.plan_manipulation = plan_manipulation
    planning.format_plan = lambda _segments: None
    planning.build_execution_plan = lambda *_args: "execution-plan"
    monkeypatch.setitem(sys.modules, "room315_planning", planning)

    problem = ModuleType("room315_problem")
    problem.BOX_ENTITY_NAME = "room315_payload_box"
    problem.DEFAULT_Q_START = np.arange(6, dtype=float)
    problem.GAZEBO_GRIPPER_CLOSE_POSITIONS = [0.0, 0.0]
    problem.GAZEBO_GRIPPER_JOINTS = ["left", "right"]
    problem.GAZEBO_GRIPPER_OPEN_POSITIONS = [0.0025, 0.0025]
    problem.JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
    problem.WORLD_NAME = "room_315_only"
    problem.build_calls = []
    problem.table_offsets = []

    def build_problem():
        problem.build_calls.append(())
        return "robot", "problem", "graph"

    def table_box_world_pose(x_offset=0.0, y_offset=0.0):
        problem.table_offsets.append((x_offset, y_offset))
        return (x_offset, y_offset)

    problem.build_problem = build_problem
    problem.table_box_world_pose = table_box_world_pose
    problem.box_configuration_from_world_pose = lambda q, pose: (tuple(q), pose)
    problem.project_free_configuration = (
        lambda _problem, _graph, _guess, label: f"q-{label}"
    )
    monkeypatch.setitem(sys.modules, "room315_problem", problem)

    spec = importlib.util.spec_from_file_location("room315_pick_place", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_command_plans_in_simulation(monkeypatch):
    pick_place = load_pick_place(monkeypatch)

    args = pick_place.parse_args([])

    assert args.execution_profile == "simulation"
    assert np.array_equal(args.q_start, np.arange(6, dtype=float))
    assert not args.build_only
    assert not args.execute


def test_hardware_execution_uses_measured_start(monkeypatch):
    pick_place = load_pick_place(monkeypatch)
    q_start = [str(index / 10) for index in range(6)]

    args = pick_place.parse_args(
        [
            "--execution-profile",
            "hardware",
            "--execute",
            "--q-start",
            *q_start,
        ]
    )

    assert args.q_start == [float(value) for value in q_start]
    assert args.trajectory_action == "/controller/follow_joint_trajectory"
    assert args.execute


def test_hardware_execution_rejects_implicit_start(monkeypatch, capsys):
    pick_place = load_pick_place(monkeypatch)

    with pytest.raises(SystemExit):
        pick_place.parse_args(
            ["--execution-profile", "hardware", "--execute"]
        )

    assert "explicit measured --q-start" in capsys.readouterr().err


def test_public_help_lists_the_supported_modes(monkeypatch, capsys):
    pick_place = load_pick_place(monkeypatch)

    with pytest.raises(SystemExit) as exc_info:
        pick_place.parse_args(["--help"])

    help_text = capsys.readouterr().out
    assert exc_info.value.code == 0
    assert "shuttle" not in help_text.lower()
    assert "trajectory-topic" not in help_text
    assert "payload-output" not in help_text
    assert "--viser" in help_text


def test_fixed_command_uses_two_table_positions_and_the_full_cell(monkeypatch):
    pick_place = load_pick_place(monkeypatch)

    assert pick_place.main([]) == 0

    problem = sys.modules["room315_problem"]
    planning = sys.modules["room315_planning"]
    execution = sys.modules["room315_execution"]
    assert problem.build_calls == [()]
    assert problem.table_offsets == [(-0.10, 0.0), (0.10, 0.0)]
    assert planning.calls[0][0][3:5] == ("q-pick", "q-place")
    assert planning.calls[0][1]["source_label"] == "pick"
    assert planning.calls[0][1]["destination_label"] == "place"
    assert execution.calls == []


def test_viser_mode_displays_the_planned_path(monkeypatch):
    pick_place = load_pick_place(monkeypatch)
    calls = []
    monkeypatch.setattr(
        pick_place,
        "show_in_viser",
        lambda *args: calls.append(args),
    )

    assert pick_place.main(["--viser"]) == 0

    assert calls == [("robot", "problem", ["planned-segment"], "q-pick")]
    assert sys.modules["room315_execution"].calls == []


def test_viser_server_loads_paths_before_opening_browser(monkeypatch):
    pick_place = load_pick_place(monkeypatch)
    calls = []

    class FakeViewer:
        def __init__(self, robot, problem):
            calls.append(("init", robot, problem))

        def start(self, **kwargs):
            calls.append(("start", kwargs))

        def __call__(self, configuration):
            calls.append(("display", configuration))

        def loadPath(self, path, name):
            calls.append(("path", path, name))

    viewer_module = ModuleType("pyhpp_viser")
    viewer_module.Viewer = FakeViewer
    monkeypatch.setitem(sys.modules, "pyhpp_viser", viewer_module)

    browser = ModuleType("webbrowser")
    browser.open = lambda url: calls.append(("browser", url))
    monkeypatch.setitem(sys.modules, "webbrowser", browser)
    monkeypatch.setattr(
        pick_place.threading,
        "Event",
        lambda: SimpleNamespace(wait=lambda: calls.append(("wait",))),
    )

    pick_place.show_in_viser(
        "robot",
        "problem",
        [SimpleNamespace(path="hpp-path", transition_name="approach")],
        "q-start",
    )

    assert calls == [
        ("init", "robot", "problem"),
        ("start", {"open": False}),
        ("display", "q-start"),
        ("path", "hpp-path", "1: approach"),
        ("browser", "http://localhost:8000"),
        ("wait",),
    ]
