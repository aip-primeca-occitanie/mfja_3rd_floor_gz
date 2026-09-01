import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


MODULE_PATH = (
    Path(__file__).parents[1] / "hpp" / "room315_execution_profiles.py"
)
CONFIG_PATH = Path(__file__).parents[1] / "hpp" / "room315_config.py"
CONFIG_SPEC = importlib.util.spec_from_file_location("room315_config", CONFIG_PATH)
CONFIG_MODULE = importlib.util.module_from_spec(CONFIG_SPEC)
sys.modules[CONFIG_SPEC.name] = CONFIG_MODULE
CONFIG_SPEC.loader.exec_module(CONFIG_MODULE)
SPEC = importlib.util.spec_from_file_location("room315_execution_profiles", MODULE_PATH)
PROFILES = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PROFILES
SPEC.loader.exec_module(PROFILES)


def profile_args(name, **overrides):
    values = {
        "execution_profile": name,
        "robot_name": "staubli1",
        "trajectory_topic": None,
        "trajectory_action": None,
        "joint_state_topic": None,
        "payload_output": None,
        "gripper_output": None,
        "gripper_trajectory_topic": None,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_simulation_profile_uses_gazebo_routes():
    args = PROFILES.apply_execution_profile(profile_args("simulation"))

    assert args.trajectory_topic == "/staubli1/joint_trajectory"
    assert args.trajectory_action is None
    assert args.joint_state_topic == "/staubli1/joint_states"
    assert args.payload_output == "gazebo"
    assert args.gripper_output == "joint-trajectory"


def test_hardware_profile_uses_val3_routes():
    args = PROFILES.apply_execution_profile(profile_args("hardware"))

    assert args.trajectory_topic is None
    assert args.trajectory_action == (
        "/manipulator_controller/joint_trajectory_action"
    )
    assert args.joint_state_topic == "/joint_states"
    assert args.payload_output == "none"
    assert args.gripper_output == "staubli-io"


def test_hardware_execution_requires_an_explicit_measured_start():
    simulation = PROFILES.apply_execution_profile(profile_args("simulation"))
    hardware = PROFILES.apply_execution_profile(profile_args("hardware"))
    simulation.execute = True
    hardware.execute = True
    assert not PROFILES.requires_explicit_measured_start(simulation, False)
    assert PROFILES.requires_explicit_measured_start(hardware, False)
    assert not PROFILES.requires_explicit_measured_start(hardware, True)
