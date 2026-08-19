import ast
import importlib.util
import sys
from pathlib import Path
from xml.etree import ElementTree


PACKAGE = Path(__file__).parents[1]
SCRIPT = PACKAGE / "scripts" / "room315_manipulation_sequence.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("room315_manipulation_sequence", SCRIPT)
DEMO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DEMO)


def assigned_value(path, name):
    tree = ast.parse(path.read_text())
    for statement in tree.body:
        if not isinstance(statement, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == name
            for target in statement.targets
        ):
            continue
        value = statement.value
        if isinstance(value, ast.Call):
            value = value.args[0]
        return ast.literal_eval(value)
    raise AssertionError(f"{name} not found in {path}")


def test_fixed_defaults_match_hpp_model():
    problem = PACKAGE / "hpp" / "room315_problem.py"

    assert DEMO.DEFAULT_HPP_START_JOINTS == tuple(
        assigned_value(problem, "DEFAULT_Q_START")
    )
    assert DEMO.DEFAULT_SHUTTLE_POSE == assigned_value(
        problem, "DEFAULT_SHUTTLE_SLOT3_POSE"
    )
    assert DEMO.BOX_SIZE == assigned_value(problem, "BOX_SIZE")

    model = ElementTree.fromstring(DEMO.PAYLOAD_BOX_SDF)
    size = tuple(float(value) for value in model.findtext(".//box/size").split())
    assert size == DEMO.BOX_SIZE


def test_manipulation_runner_has_no_rail_dependency():
    assert "mfja_rail_interfaces" not in SCRIPT.read_text()


def test_fixed_scenario_command_does_not_command_shuttles():
    args = DEMO.parse_args([])
    args.hpp_script = Path("room315_hpp_manipulation.sh")
    command = DEMO.hpp_cycle_command(
        args,
        DEMO.pose_from_values(args.shuttle_pose),
        direction="shuttle-to-table",
    )

    assert command[command.index("--direction") + 1] == "shuttle-to-table"
    assert command[command.index("--payload-output") + 1] == "gazebo"
    assert command[command.index("--gripper-output") + 1] == "joint-trajectory"
    assert command[command.index("--trajectory-topic") + 1] == (
        "/staubli1/joint_trajectory"
    )
    assert "--destination-shuttle-pose" not in command
    assert not any("/room_315/rails/" in value for value in command)


def test_fixed_scenario_order(monkeypatch):
    args = DEMO.parse_args([])
    events = []
    preposition = object()

    class FakeCoordinator:
        def wait_for_arm_publisher(self, _timeout):
            events.append("arm interface")

        def initialize_payload(self, _pose, support):
            assert support == "fixed pickup shuttle"
            events.append("payload")

        def start_preposition_arm(self):
            events.append("start arm")
            return preposition

        def wait_preposition_arm(self, value):
            assert value is preposition
            events.append("wait arm")

    def run_hpp(_args, _pose, **kwargs):
        assert kwargs == {"direction": "shuttle-to-table"}
        events.append("manipulation")

    monkeypatch.setattr(DEMO, "run_hpp_cycle", run_hpp)
    DEMO.run_fixed_manipulation(FakeCoordinator(), args)

    assert events == [
        "arm interface",
        "payload",
        "start arm",
        "wait arm",
        "manipulation",
    ]
