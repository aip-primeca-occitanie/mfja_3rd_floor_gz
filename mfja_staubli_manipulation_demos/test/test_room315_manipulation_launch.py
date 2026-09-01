from pathlib import Path

import yaml


HARDWARE_LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "room_315_staubli_hardware.launch.py"
)
PICK_PLACE_LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "room_315_staubli_pick_place_sim.launch.py"
)
PICK_PLACE_CONFIG = yaml.safe_load(
    (
        Path(__file__).parents[1]
        / "config"
        / "room315_pick_place.yaml"
    ).read_text()
)


def test_hardware_bringup_uses_direct_driver_and_explicit_ip():
    source = HARDWARE_LAUNCH.read_text()

    assert "staubli_val3_driver" in source
    assert "staubli_tx2_60l_description" in source
    assert "robot_state_publisher" in source
    assert "staubli_tx2_60l_moveit_config" not in source
    assert 'DeclareLaunchArgument(\n                "robot_ip"' in source
    assert "default_value=\"172." not in source
    assert '"joint_config": joint_config' in source
    assert '"enable_system",\n                default_value="false"' in source


def test_fixed_pick_place_simulation_uses_focused_scene():
    source = PICK_PLACE_LAUNCH.read_text()

    assert "multi_robot_sim.launch.py" in source
    assert "room315_pick_place.yaml" in source
    assert "room315_payload_box.sdf" in source
    assert PICK_PLACE_CONFIG["scene"]["payload_entity_name"] == (
        "room315_payload_box"
    )
    assert "room315_pick_support" not in source
    assert '"enable_conveyor_controller": "false"' in source
    assert "room_315_dual_kinematic_shuttles" not in source
    assert "mfja_rail_interfaces" not in source
