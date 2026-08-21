import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest


EXECUTION = (
    Path(__file__).parents[1] / "hpp" / "room315_execution.py"
)


def load_execution_module(monkeypatch):
    hpp_exec = ModuleType("hpp_exec")

    @dataclass
    class Segment:
        start_index: int
        end_index: int
        pre_actions: list = field(default_factory=list)
        post_actions: list = field(default_factory=list)
        transition_name: str = ""

    hpp_exec.configs_to_joint_trajectory = lambda *_args: None
    hpp_exec.execute_segments = lambda *_args, **_kwargs: True
    hpp_exec.Segment = Segment
    hpp_exec.segments_by_transition = lambda segments: {
        name: [segment for segment in segments if segment.transition_name == name]
        for name in {segment.transition_name for segment in segments}
    }
    monkeypatch.setitem(sys.modules, "hpp_exec", hpp_exec)

    room315_problem = ModuleType("room315_problem")
    room315_problem.GRASP_TRANSITION = "grasp"
    room315_problem.JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
    room315_problem.RELEASE_TRANSITION = "release"
    room315_problem.box_rank = lambda *_args: 0
    room315_problem.box_world_pose_msg = lambda *_args: None
    room315_problem.normalize_box_quaternion = lambda *_args: None
    monkeypatch.setitem(sys.modules, "room315_problem", room315_problem)

    staubli_msgs = ModuleType("staubli_msgs")
    staubli_msgs.__path__ = []
    staubli_msg = ModuleType("staubli_msgs.msg")
    staubli_msg.IOModule = SimpleNamespace(VALVE_OUT=2)
    staubli_msg.ServiceReturnCode = SimpleNamespace(SUCCESS=1, FAILURE=-1)
    staubli_srv = ModuleType("staubli_msgs.srv")

    class WriteSingleIO:
        class Request:
            def __init__(self):
                self.module = SimpleNamespace(id=None)
                self.pin = None
                self.state = None

    staubli_srv.WriteSingleIO = WriteSingleIO
    monkeypatch.setitem(sys.modules, "staubli_msgs", staubli_msgs)
    monkeypatch.setitem(sys.modules, "staubli_msgs.msg", staubli_msg)
    monkeypatch.setitem(sys.modules, "staubli_msgs.srv", staubli_srv)

    spec = importlib.util.spec_from_file_location(
        "room315_execution_test", EXECUTION
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module.rclpy,
        "spin_until_future_complete",
        lambda *_args, **_kwargs: None,
    )
    return module


class FakeFuture:
    def __init__(self, code):
        self.code = code

    def result(self):
        return SimpleNamespace(code=SimpleNamespace(val=self.code))


class FakeClient:
    def __init__(self, codes):
        self.codes = iter(codes)
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(next(self.codes))


class FakeNode:
    def __init__(self, codes):
        self.client = FakeClient(codes)

    def create_client(self, _service_type, _service_name):
        return self.client


def gripper_args():
    return SimpleNamespace(
        staubli_io_service="/io_interface/write_single_io",
        staubli_io_timeout=5.0,
        gripper_settle_s=0.0,
    )


def test_staubli_io_open_and_close_requests(monkeypatch):
    execution = load_execution_module(monkeypatch)
    node = FakeNode([1, 1])
    output = execution.StaubliIOGripperOutput(node, gripper_args())

    output.open()
    output.close()
    assert [
        (request.module.id, request.pin, request.state)
        for request in node.client.requests
    ] == [(2, 0, True), (2, 0, False)]


def test_staubli_io_failure_raises(monkeypatch):
    execution = load_execution_module(monkeypatch)
    output = execution.StaubliIOGripperOutput(FakeNode([-1]), gripper_args())

    with pytest.raises(RuntimeError, match=r"code -1"):
        output.open()


def test_action_segments_use_hpp_exec_executor(monkeypatch):
    execution = load_execution_module(monkeypatch)
    calls = []
    configs = [object(), object()]
    times = [0.0, 2.0]
    segments = [object()]
    plan = SimpleNamespace(configs=configs, times=times)
    monkeypatch.setattr(
        execution,
        "execute_hpp_segments",
        lambda *args, **kwargs: calls.append((args, kwargs)) or True,
    )

    execution.execute_action_segments(
        plan,
        segments,
        "/manipulator_controller/joint_trajectory_action",
    )

    assert calls == [
        (
            (segments, configs, times, execution.JOINT_NAMES),
            {
                "controller_topic": (
                    "/manipulator_controller/joint_trajectory_action"
                )
            },
        )
    ]


def test_action_segment_failure_raises(monkeypatch):
    execution = load_execution_module(monkeypatch)
    plan = SimpleNamespace(
        configs=[object(), object()],
        times=[0.0, 2.0],
    )
    monkeypatch.setattr(
        execution, "execute_hpp_segments", lambda *_args, **_kwargs: False
    )

    with pytest.raises(RuntimeError, match="failed the HPP execution segments"):
        execution.execute_action_segments(
            plan,
            [object()],
            "/test/action",
        )


def test_configured_segments_assign_gripper_preactions(monkeypatch):
    execution = load_execution_module(monkeypatch)
    Segment = sys.modules["hpp_exec"].Segment
    planned_segments = [
        Segment(0, 2, transition_name="approach"),
        Segment(2, 4, transition_name="grasp"),
        Segment(4, 6, transition_name="release"),
    ]
    plan = SimpleNamespace(
        segments=planned_segments,
        segment_names=["approach", "transfer", "retreat"],
        payload_configs=[None] * 6,
    )
    calls = []
    gripper = SimpleNamespace(
        open=lambda: calls.append("open") or True,
        close=lambda: calls.append("close") or True,
    )

    configured = execution.configured_segments(
        None,
        None,
        None,
        None,
        plan,
        gripper,
        SimpleNamespace(
            box_entity_name="box",
            gripper_output="joint-trajectory",
        ),
    )

    for segment in configured:
        for action in segment.pre_actions:
            assert action()
    assert calls == ["open", "close", "open"]
    assert all(segment.pre_actions == [] for segment in planned_segments)
    assert all(len(segment.post_actions) == 1 for segment in configured)


def test_topic_transport_consumes_segment_actions_in_order(monkeypatch):
    execution = load_execution_module(monkeypatch)
    Segment = sys.modules["hpp_exec"].Segment
    calls = []
    segment = Segment(
        0,
        2,
        pre_actions=[lambda: calls.append("pre") or True],
        post_actions=[lambda: calls.append("post") or True],
        transition_name="approach",
    )
    plan = SimpleNamespace(
        configs=[np.zeros(6), np.ones(6)],
        payload_configs=[np.zeros(7), np.ones(7)],
        times=[0.0, 1.0],
        segment_names=["approach"],
        payload_modes=["fixed"],
    )
    monkeypatch.setattr(
        execution,
        "configs_to_joint_trajectory",
        lambda *_args: object(),
    )
    monkeypatch.setattr(
        execution,
        "publish_trajectory",
        lambda *_args: calls.append("trajectory"),
    )

    execution.execute_topic_segments(
        None,
        object(),
        "/staubli1/joint_trajectory",
        None,
        None,
        None,
        "box",
        plan,
        [segment],
        SimpleNamespace(),
    )

    assert calls == ["pre", "trajectory", "post"]


def test_topic_transport_preserves_three_segment_boundaries(monkeypatch):
    execution = load_execution_module(monkeypatch)
    Segment = sys.modules["hpp_exec"].Segment
    calls = []
    segments = [
        Segment(
            0,
            2,
            pre_actions=[lambda: calls.append("open") or True],
            post_actions=[lambda: calls.append("wait-approach") or True],
        ),
        Segment(
            2,
            4,
            pre_actions=[lambda: calls.append("close-attach") or True],
            post_actions=[lambda: calls.append("wait-transfer") or True],
        ),
        Segment(
            4,
            6,
            pre_actions=[lambda: calls.append("open-detach") or True],
            post_actions=[lambda: calls.append("wait-retreat") or True],
        ),
    ]
    plan = SimpleNamespace(
        configs=[np.full(6, index) for index in range(6)],
        payload_configs=[np.full(7, index) for index in range(6)],
        times=[0.0, 1.0, 1.0, 3.0, 3.0, 4.5],
        payload_modes=["fixed", "follow", "fixed"],
    )

    def make_trajectory(_configs, times, _joint_names):
        calls.append(("trajectory", times))
        return object()

    monkeypatch.setattr(execution, "configs_to_joint_trajectory", make_trajectory)
    monkeypatch.setattr(execution, "publish_trajectory", lambda *_args: None)
    monkeypatch.setattr(
        execution,
        "follow_payload",
        lambda *_args: calls.append("follow"),
    )

    execution.execute_topic_segments(
        None,
        object(),
        "/staubli1/joint_trajectory",
        object(),
        None,
        None,
        "box",
        plan,
        segments,
        SimpleNamespace(),
    )

    assert calls == [
        "open",
        ("trajectory", [0.0, 1.0]),
        "wait-approach",
        "close-attach",
        ("trajectory", [0.0, 2.0]),
        "follow",
        "wait-transfer",
        "open-detach",
        ("trajectory", [0.0, 1.5]),
        "wait-retreat",
    ]


def test_segment_actions_do_not_render_or_print_callables(monkeypatch, capsys):
    execution = load_execution_module(monkeypatch)
    calls = []

    class Action:
        def __call__(self):
            calls.append("action")
            return True

        def __repr__(self):
            raise AssertionError("segment actions must not be rendered")

    execution.run_segment_actions([Action()], 0, "post")

    assert calls == ["action"]
    assert capsys.readouterr().out == ""


def test_require_start_waits_for_controller_to_settle(monkeypatch):
    execution = load_execution_module(monkeypatch)
    target = np.array([0.0, 0.87, 1.22, 0.0, 0.96, 0.0])

    class SettlingTracker:
        topic = "/staubli1/joint_states"

        def __init__(self):
            self.samples = [np.zeros(6), target]

        def current(self):
            if len(self.samples) > 1:
                return self.samples.pop(0)
            return self.samples[0].copy()

    monkeypatch.setattr(execution.rclpy, "spin_once", lambda *_args, **_kwargs: None)
    execution.require_start(
        None,
        SettlingTracker(),
        SimpleNamespace(joint_state_timeout=0.1, start_tolerance=0.01),
        target,
    )


def test_require_start_rejects_final_configuration_outside_tolerance(monkeypatch):
    execution = load_execution_module(monkeypatch)
    tracker = SimpleNamespace(
        topic="/staubli1/joint_states",
        current=lambda: np.zeros(6),
    )

    with pytest.raises(RuntimeError, match=r"1\.000 rad from the HPP start"):
        execution.require_start(
            None,
            tracker,
            SimpleNamespace(joint_state_timeout=0.0, start_tolerance=0.01),
            np.ones(6),
        )
