from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).parents[2]
DESCRIPTION = REPOSITORY / "mfja_3rd_floor_description"
DEMO = REPOSITORY / "mfja_staubli_manipulation_demos"
LAUNCH = (
    REPOSITORY
    / "mfja_robot_control_config"
    / "launch"
    / "multi_robot_sim.launch.py"
)


def load_launch_module():
    spec = spec_from_file_location("multi_robot_sim_launch", LAUNCH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gazebo_model_and_robot_description_can_use_distinct_packages():
    module = load_launch_module()
    package_paths = {
        "mfja_3rd_floor_description": DESCRIPTION,
        "mfja_staubli_manipulation_demos": DEMO,
    }
    module.get_package_share_directory = lambda name: str(package_paths[name])
    config = yaml.safe_load(
        (DEMO / "config" / "robots_room315_gripper.yaml").read_text()
    )
    robot = config["robots"][0]

    model_sdf, urdf_path = module._resolve_robot_assets(
        robot,
        str(DESCRIPTION),
        robot["model"],
    )

    assert Path(model_sdf) == (
        DEMO / "models" / "staubli_tx2_60l_gripper" / "model.sdf"
    )
    assert Path(urdf_path) == (
        DESCRIPTION / "urdf" / "staubli_tx2_60l.urdf"
    )
