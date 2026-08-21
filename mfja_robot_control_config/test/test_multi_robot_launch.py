import ast
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


def assigned_literal(path, name):
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == name
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} is not assigned in {path}")


def robot_pose(path, name):
    config = yaml.safe_load(path.read_text())
    robot = next(robot for robot in config["robots"] if robot["name"] == name)
    return tuple(robot[key] for key in ("x_pose", "y_pose", "z_pose", "yaw"))


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


def test_room315_staubli_pose_matches_gazebo_and_hpp_models():
    expected_gazebo_pose = (-15.251, -6.0, 1.0, 0.0)
    config_paths = [
        REPOSITORY / "mfja_robot_control_config" / "config" / "robots.yaml",
        REPOSITORY
        / "mfja_robot_control_config"
        / "config"
        / "robots_room_315_only.yaml",
        DEMO / "config" / "robots_room315_gripper.yaml",
    ]

    for path in config_paths:
        assert robot_pose(path, "staubli1") == expected_gazebo_pose

    expected_hpp_pose = (-15.251, -6.0, 1.0, 0.0, 0.0, 0.0)
    hpp_paths = [
        REPOSITORY / "mfja_staubli_demos" / "hpp" / "room315_hpp_line.py",
        DEMO / "hpp" / "room315_problem.py",
    ]

    for path in hpp_paths:
        assert assigned_literal(path, "ROOM315_ROBOT_POSE") == expected_hpp_pose
