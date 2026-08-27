import os
import subprocess
from pathlib import Path


SCRIPTS = Path(__file__).parents[1] / "scripts"
REPOSITORY = Path(__file__).parents[2]


def test_room315_shell_scripts_parse():
    scripts = sorted(SCRIPTS.glob("room315_*.sh"))

    subprocess.run(["bash", "-n", *scripts], check=True)


def test_top_level_installer_parses_and_is_executable():
    installer = REPOSITORY / "install.sh"

    subprocess.run(["bash", "-n", installer], check=True)
    assert os.access(installer, os.X_OK)


def test_generated_setup_defines_its_workspace_root():
    setup = (REPOSITORY / "setup.bash.in").read_text()

    assert 'export MFJA_WORK_DIR="$_MFJA_SETUP_ROOT"' in setup


def test_public_setup_has_no_developer_machine_paths():
    paths = (
        REPOSITORY / "README.md",
        REPOSITORY / "DETAILED_GUIDE.md",
        REPOSITORY / "install.sh",
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
