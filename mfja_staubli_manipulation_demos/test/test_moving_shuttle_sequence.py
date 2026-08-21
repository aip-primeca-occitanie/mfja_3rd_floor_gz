import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "room315_moving_shuttle_sequence.py"
)
sys.path.insert(0, str(SCRIPT.parent))
MANIPULATION = importlib.import_module("room315_manipulation_sequence")
SPEC = importlib.util.spec_from_file_location("room315_moving_shuttle_sequence", SCRIPT)
DEMO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DEMO)


def pose(x=0.0):
    result = MANIPULATION.Pose()
    result.position.x = x
    result.orientation.w = 1.0
    return result


def test_simulation_defaults():
    args = DEMO.parse_args([])

    assert not hasattr(args, "preposition")
    assert not hasattr(args, "q_start")
    assert args.publisher_timeout == 5.0
    assert args.pickup_stop_position == (-15.240, -5.536, 0.839)


def test_scenario_names_and_slots_are_configurable():
    args = DEMO.parse_args(
        [
            "--pickup-shuttle-name",
            "source",
            "--drop-shuttle-name",
            "destination",
            "--drop-start-slot",
            "3",
            "--pickup-sensor",
            "PICK",
            "--drop-sensor",
            "DROP",
        ]
    )

    assert (
        args.pickup_shuttle_name,
        args.drop_shuttle_name,
        args.drop_start_slot,
        args.pickup_sensor,
        args.drop_sensor,
    ) == ("source", "destination", "3", "PICK", "DROP")


def test_route_requires_all_switches_and_stoppers():
    coordinator = object.__new__(DEMO.MovingShuttleCoordinator)
    coordinator.latest_switch_states = {"SW1": "E", "SW2": "E"}
    coordinator.latest_stopper_states = {"ST1": "0", "ST2": "0"}
    assert coordinator.route_is_ready()

    coordinator.latest_switch_states["SW2"] = "INTERIOR"
    assert not coordinator.route_is_ready()


def test_failed_arrival_still_stops_shuttle():
    coordinator = object.__new__(DEMO.MovingShuttleCoordinator)
    coordinator.active_shuttles = set()
    commands = []

    def publish_command(shuttle_name, command):
        commands.append(command)
        if command == "ON":
            coordinator.active_shuttles.add(shuttle_name)
        else:
            coordinator.active_shuttles.discard(shuttle_name)

    def fail_arrival(*_args, **_kwargs):
        raise RuntimeError("arrival failed")

    coordinator.publish_shuttle_command = publish_command
    coordinator.sensor_is_active = lambda *_: False
    coordinator.wait_for_sensor_active = fail_arrival

    with pytest.raises(RuntimeError, match="arrival failed"):
        coordinator.move_to_pickup_slot(
            "arrival",
            "shuttle",
            "sensor",
            require_leave_first=False,
            timeout=1.0,
        )

    assert commands == ["ON", "OFF"]


def test_arrival_reaches_pickup_position_before_stopping():
    coordinator = object.__new__(DEMO.MovingShuttleCoordinator)
    coordinator.args = type("Args", (), {})()
    coordinator.active_shuttles = set()
    coordinator.pose_updates = {"shuttle": 1}
    coordinator.shuttle_state_updates = {"shuttle": 1}
    events = []

    def publish_command(_shuttle_name, command):
        events.append(command)
        if command == "ON":
            coordinator.active_shuttles.add("shuttle")
        else:
            coordinator.active_shuttles.discard("shuttle")

    coordinator.publish_shuttle_command = publish_command
    coordinator.sensor_is_active = lambda *_args: False
    coordinator.wait_for_sensor_active = lambda *_args, **_kwargs: events.append(
        "sensor"
    )
    coordinator.wait_for_position = lambda *_args, **_kwargs: events.append(
        "position"
    )
    coordinator.wait_for_stopped_pose = lambda *_args, **_kwargs: pose()

    coordinator.move_to_pickup_slot(
        "arrival",
        "shuttle",
        "sensor",
        require_leave_first=False,
        timeout=1.0,
        stop_position=(-15.240, -5.536, 0.839),
    )

    assert events == ["ON", "sensor", "position", "OFF"]


def test_moving_scenario_adds_rail_motion_around_manipulation(monkeypatch):
    args = DEMO.parse_args([])
    events = []
    pickup_pose = pose(1.0)
    drop_pose = pose(2.0)

    class FakeCoordinator:
        def wait_for_publishers(self, _timeout):
            events.append("interfaces")

        def wait_for_sensor_known(self, _sensor):
            events.append("sensor")

        def add_drop_shuttle(self):
            events.append("add drop shuttle")
            return drop_pose

        def ensure_payload_on_shuttle(self, _name):
            events.append("payload")

        def prepare_route(self):
            events.append("route")

        def move_to_pickup_slot(self, *_args, **_kwargs):
            events.append("move shuttle")
            return pickup_pose

    def run_hpp(_args, source, **kwargs):
        assert source is pickup_pose
        assert kwargs == {
            "direction": "shuttle-to-shuttle",
            "destination_shuttle_pose": drop_pose,
        }
        events.append("manipulation")

    monkeypatch.setattr(DEMO, "run_hpp_cycle", run_hpp)
    DEMO.run_moving_shuttle_demo(FakeCoordinator(), args)

    assert events == [
        "interfaces",
        "sensor",
        "add drop shuttle",
        "payload",
        "route",
        "move shuttle",
        "manipulation",
    ]
