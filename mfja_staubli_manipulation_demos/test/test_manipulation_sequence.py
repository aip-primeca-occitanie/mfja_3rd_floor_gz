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

    assert DEMO.DEFAULT_SHUTTLE_POSE == assigned_value(
        problem, "DEFAULT_SHUTTLE_SLOT3_POSE"
    )
    assert DEMO.BOX_SIZE == assigned_value(problem, "BOX_SIZE")

    model = ElementTree.fromstring(DEMO.PAYLOAD_BOX_SDF)
    size = tuple(float(value) for value in model.findtext(".//box/size").split())
    assert size == DEMO.BOX_SIZE


def test_manipulation_runner_has_no_rail_dependency():
    source = SCRIPT.read_text()
    assert "mfja_rail_interfaces" not in source
    assert "preposition" not in source


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
    assert "--q-start" not in command
    assert "--trajectory-topic" not in command
    assert "--joint-state-topic" not in command
    assert "--destination-shuttle-pose" not in command
    assert not any("/room_315/rails/" in value for value in command)


def test_fixed_scenario_order(monkeypatch):
    args = DEMO.parse_args([])
    events = []

    class FakeCoordinator:
        def initialize_payload(self, _pose, support):
            assert support == "fixed pickup shuttle"
            events.append("payload")

    def run_hpp(_args, _pose, **kwargs):
        assert kwargs == {"direction": "shuttle-to-table"}
        events.append("manipulation")

    monkeypatch.setattr(DEMO, "run_hpp_cycle", run_hpp)
    DEMO.run_fixed_manipulation(FakeCoordinator(), args)

    assert events == ["payload", "manipulation"]
