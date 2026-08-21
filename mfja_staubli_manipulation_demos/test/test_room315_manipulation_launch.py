import ast
from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "room_315_staubli_shuttle_manipulation_demo.launch.py"
)


def launch_argument_default(name):
    tree = ast.parse(LAUNCH.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if (
            not isinstance(node.func, ast.Name)
            or node.func.id != "DeclareLaunchArgument"
        ):
            continue
        if not node.args or ast.literal_eval(node.args[0]) != name:
            continue
        default = next(
            keyword.value
            for keyword in node.keywords
            if keyword.arg == "default_value"
        )
        return ast.literal_eval(default)
    raise AssertionError(f"launch argument {name} was not declared")


def forwarded_launch_arguments():
    tree = ast.parse(LAUNCH.read_text())
    forwarded = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                continue
            if not isinstance(value, ast.Call):
                continue
            if (
                not isinstance(value.func, ast.Name)
                or value.func.id != "LaunchConfiguration"
            ):
                continue
            forwarded[key.value] = ast.literal_eval(value.args[0])
    return forwarded


def test_shuttle_stop_defaults_are_repeatable():
    assert launch_argument_default("shuttle_speed") == "0.1"
    assert launch_argument_default("sensor_publish_rate_hz") == "30.0"
    assert forwarded_launch_arguments()["room315_sensor_publish_rate_hz"] == (
        "sensor_publish_rate_hz"
    )
