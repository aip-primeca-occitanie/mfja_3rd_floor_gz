import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


EXECUTION = (
    Path(__file__).parents[1] / "hpp" / "room315_execution.py"
)


def load_execution_module(monkeypatch):
    hpp_exec = ModuleType("hpp_exec")
    hpp_exec.configs_to_joint_trajectory = lambda *_args: None
    hpp_exec.send_trajectory = lambda *_args, **_kwargs: True
    monkeypatch.setitem(sys.modules, "hpp_exec", hpp_exec)

    room315_problem = ModuleType("room315_problem")
    room315_problem.JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
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


def test_action_phase_uses_hpp_exec_sender(monkeypatch):
    execution = load_execution_module(monkeypatch)
    calls = []
    configs = [object(), object()]
    times = [0.0, 2.0]
    phase = SimpleNamespace(
        name="transfer",
        configs=configs,
        times=times,
        payload_mode="fixed",
    )
    monkeypatch.setattr(
        execution,
        "send_trajectory",
        lambda *args, **kwargs: calls.append((args, kwargs)) or True,
    )
    monkeypatch.setattr(execution, "wait_for_phase_end", lambda *_args: None)

    execution.execute_phase(
        None,
        None,
        "/manipulator_controller/joint_trajectory_action",
        None,
        object(),
        None,
        "box",
        phase,
        SimpleNamespace(),
    )

    assert calls == [
        (
            (configs, times, execution.JOINT_NAMES),
            {
                "controller_topic": (
                    "/manipulator_controller/joint_trajectory_action"
                )
            },
        )
    ]


def test_action_phase_failure_raises(monkeypatch):
    execution = load_execution_module(monkeypatch)
    phase = SimpleNamespace(
        name="transfer",
        configs=[object(), object()],
        times=[0.0, 2.0],
        payload_mode="fixed",
    )
    monkeypatch.setattr(execution, "send_trajectory", lambda *_args, **_kwargs: False)

    with pytest.raises(RuntimeError, match="failed phase transfer"):
        execution.execute_phase(
            None,
            None,
            "/test/action",
            None,
            object(),
            None,
            "box",
            phase,
            SimpleNamespace(),
        )
