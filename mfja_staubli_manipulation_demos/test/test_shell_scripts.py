import os
import subprocess
from pathlib import Path


SCRIPTS = Path(__file__).parents[1] / "scripts"
REPOSITORY = Path(__file__).parents[2]
HPP_SCRIPTS = REPOSITORY / "mfja_staubli_demos" / "scripts"


def test_room315_shell_scripts_parse():
    scripts = sorted(SCRIPTS.glob("room315_*.sh"))

    subprocess.run(["bash", "-n", *scripts], check=True)


def test_top_level_installer_parses_and_is_executable():
    installer = REPOSITORY / "install.sh"

    subprocess.run(["bash", "-n", installer], check=True)
    assert os.access(installer, os.X_OK)


def test_public_setup_templates_parse():
    templates = (
        REPOSITORY / "hpp_setup.bash.in",
        REPOSITORY / "setup.bash.in",
    )

    subprocess.run(["bash", "-n", *templates], check=True)


def test_generated_setup_defines_its_workspace_root():
    setup = (REPOSITORY / "setup.bash.in").read_text()

    assert 'export MFJA_WORK_DIR="$_MFJA_SETUP_ROOT"' in setup
    assert "${HPP_SETUP:-$_MFJA_SETUP_ROOT/hpp/install/setup.bash}" in setup


def test_installer_uses_only_the_hybrid_hpp_workspace():
    installer = (REPOSITORY / "install.sh").read_text()
    builder = (HPP_SCRIPTS / "room315_build_hpp_underlay.sh").read_text()
    overlay_builder = (HPP_SCRIPTS / "room315_build_overlay.sh").read_text()

    assert 'hpp_devel="$work_dir/hpp"' in installer
    assert 'HPP_SETUP="$hpp_install/setup.bash"' in installer
    assert "${HPP_SETUP:-$devel_root/hpp/install/setup.bash}" in overlay_builder
    assert "$work_dir/hpp_sources" not in installer
    assert "$work_dir/hpp_ws" not in installer
    assert "$devel_root/hpp_sources" not in builder
    assert "$devel_root/hpp_ws" not in builder


def test_installer_requires_the_exact_robotpkg_hpp_packages():
    installer = (REPOSITORY / "install.sh").read_text()

    assert "robotpkg=/opt/openrobots" in installer
    assert "required_hpp_version=9.0.2" in installer
    assert "robotpkg-py312-hpp-python" in installer
    assert "robotpkg-py312-qt5-hpp-gepetto-viewer" in installer
    assert '"install ok installed|$required_hpp_version"' in installer


def test_hpp_setup_orders_local_pip_and_robotpkg_prefixes():
    setup = (REPOSITORY / "hpp_setup.bash.in").read_text()

    assert 'PATH="$INSTALL_HPP_DIR/bin$_HPP_PIP_BIN:$ROBOTPKG/bin:/usr/bin' in setup
    assert (
        'PYTHONPATH="$INSTALL_HPP_DIR/lib/python3.12/site-packages'
        '$_HPP_PIP_PYTHON:$ROBOTPKG/lib/python3.12/site-packages' in setup
    )
    assert 'CMAKE_PREFIX_PATH="$INSTALL_HPP_DIR:$ROBOTPKG:/usr' in setup


def test_setup_check_enforces_module_prefixes():
    check = (SCRIPTS / "room315_check_setup.sh").read_text()

    assert '"hpp-exec": require_prefix(hpp_exec, local_prefix)' in check
    assert '"hpp-toppra": require_prefix(pyhpp_toppra, local_prefix)' in check
    assert '"pyhpp": require_prefix(pyhpp, robotpkg_prefix)' in check
    assert '"pyhpp-viser": require_prefix(pyhpp_viser, robotpkg_prefix)' in check
    assert '"rclpy": require_prefix(rclpy, ros_prefix)' in check


def test_install_guides_use_only_robotpkg_stable_and_exact_hpp_versions():
    readme = (REPOSITORY / "README.md").read_text()

    assert "packages/debian/pub noble robotpkg" in readme
    assert "wip/packages" not in readme
    assert "robotpkg-py312-hpp-python=9.0.2" in readme
    assert "robotpkg-py312-qt5-hpp-gepetto-viewer=9.0.2" in readme
    for ros_package in (
        "ros-jazzy-coal",
        "ros-jazzy-jrl-cmakemodules",
        "ros-jazzy-pinocchio",
        "ros-jazzy-proxsuite",
    ):
        assert ros_package not in readme


def test_public_setup_has_no_developer_machine_paths():
    paths = (
        REPOSITORY / "README.md",
        REPOSITORY / "DETAILED_GUIDE.md",
        REPOSITORY / "install.sh",
        REPOSITORY / "hpp_setup.bash.in",
        REPOSITORY / "setup.bash.in",
        REPOSITORY / "mfja_staubli_demos" / "README.md",
        REPOSITORY / "mfja_staubli_manipulation_demos" / "README.md",
    )

    for path in paths:
        source = path.read_text()
        assert "/home/" not in source, path
        assert "nix-hpp" not in source, path
        assert "172.31.0.1" not in source, path
        assert "magma" not in source.lower(), path


def test_user_facing_scripts_are_executable():
    for name in (
        "room315_check_setup.sh",
        "room315_pick_place.sh",
    ):
        assert os.access(SCRIPTS / name, os.X_OK), name


def test_only_fixed_pick_place_scripts_are_installed():
    setup = (SCRIPTS.parent / "setup.py").read_text()

    assert '"scripts/room315_pick_place.sh"' in setup
