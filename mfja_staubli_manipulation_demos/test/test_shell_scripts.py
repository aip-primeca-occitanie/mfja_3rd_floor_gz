import os
import subprocess
from pathlib import Path


SCRIPTS = Path(__file__).parents[1] / "scripts"


def test_room315_shell_scripts_parse():
    scripts = sorted(SCRIPTS.glob("room315_*.sh"))

    subprocess.run(["bash", "-n", *scripts], check=True)


def test_user_facing_scripts_are_executable():
    for name in (
        "room315_check_setup.sh",
        "room315_demo.sh",
        "room315_hpp_manipulation.sh",
        "room315_manipulation_demo.sh",
        "room315_moving_shuttle_demo.sh",
    ):
        assert os.access(SCRIPTS / name, os.X_OK), name
