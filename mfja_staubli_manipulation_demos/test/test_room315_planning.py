import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


PLANNING = Path(__file__).parents[1] / "hpp" / "room315_planning.py"
def load_planning_module(monkeypatch):
    hpp_exec = ModuleType("hpp_exec")

    @dataclass
    class Segment:
        start_index: int
        end_index: int
        pre_actions: list = field(default_factory=list)
        post_actions: list = field(default_factory=list)
        start_time: float = 0.0
        end_time: float = 0.0
        transition_name: str = ""

    hpp_exec.Segment = Segment
    monkeypatch.setitem(sys.modules, "hpp_exec", hpp_exec)

    pyhpp = ModuleType("pyhpp")
    pyhpp.__path__ = []
    manipulation = ModuleType("pyhpp.manipulation")
    manipulation.TransitionPlanner = object
    monkeypatch.setitem(sys.modules, "pyhpp", pyhpp)
    monkeypatch.setitem(sys.modules, "pyhpp.manipulation", manipulation)

    problem = ModuleType("room315_problem")
    problem.GRASP_TRANSITION = "pick-23"
    problem.PICK_TRANSITIONS = ["pick-01", "pick-12", "pick-23", "pick-34"]
    problem.RELEASE_TRANSITION = "release-21"
    problem.RELEASE_TRANSITIONS = [
        "release-43",
        "release-32",
        "release-21",
        "release-10",
    ]
    problem.TRANSFER_TRANSITION = "transfer"
    problem.box_rank = lambda _robot: 6
    problem.normalize_box_quaternion = lambda _robot, q: q
    monkeypatch.setitem(sys.modules, "room315_problem", problem)

    spec = importlib.util.spec_from_file_location("room315_planning_test", PLANNING)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakePath:
    def __init__(self, length):
        self._length = length

    def length(self):
        return self._length


def segments(module, lengths):
    names = [
        *module.PICK_TRANSITIONS,
        module.TRANSFER_TRANSITION,
        *module.RELEASE_TRANSITIONS,
    ]
    return [
        module.PlannedSegment(name, FakePath(length))
        for name, length in zip(names, lengths)
    ]


def test_simple_plan_accepts_compact_observed_path(monkeypatch):
    planning = load_planning_module(monkeypatch)
    compact = segments(
        planning,
        [1.540, 0.190, 0.118, 0.0, 1.072, 0.0, 0.088, 0.161, 0.728],
    )

    first, total = planning.validate_simple_plan(compact)

    assert first == pytest.approx(1.540)
    assert total == pytest.approx(3.897)


@pytest.mark.parametrize("first_length", [6.464, 10.810])
def test_simple_plan_rejects_observed_opposite_branch(monkeypatch, first_length):
    planning = load_planning_module(monkeypatch)
    distant = segments(
        planning,
        [first_length, 0.190, 0.118, 0.0, 1.072, 0.0, 0.088, 0.161, 0.728],
    )

    with pytest.raises(RuntimeError, match="first approach path length"):
        planning.validate_simple_plan(distant)


def test_execution_plan_uses_semantic_hpp_exec_segments(monkeypatch):
    planning = load_planning_module(monkeypatch)
    planned = [
        SimpleNamespace(transition_name=name)
        for name in [
            *planning.PICK_TRANSITIONS,
            planning.TRANSFER_TRANSITION,
            *planning.RELEASE_TRANSITIONS,
        ]
    ]
    samples = iter(
        [
            (["a0", "a1"], ["pa0", "pa1"], [0.0, 1.0]),
            (["b0", "b1"], ["pb0", "pb1"], [0.0, 2.0]),
            (["c0", "c1"], ["pc0", "pc1"], [0.0, 1.5]),
        ]
    )
    monkeypatch.setattr(
        planning,
        "sample_execution_segment",
        lambda *_args: next(samples),
    )

    plan = planning.build_execution_plan(
        None,
        None,
        planned,
        None,
        None,
        "pickup",
        "drop",
        None,
    )

    assert [
        (segment.start_index, segment.end_index, segment.transition_name)
        for segment in plan.segments
    ] == [
        (0, 2, planning.PICK_TRANSITIONS[0]),
        (2, 4, planning.GRASP_TRANSITION),
        (4, 6, planning.RELEASE_TRANSITION),
    ]
    assert plan.times == [0.0, 1.0, 1.0, 3.0, 3.0, 4.5]
    assert plan.payload_modes == ["pickup-fixed", "follow", "drop-fixed"]
