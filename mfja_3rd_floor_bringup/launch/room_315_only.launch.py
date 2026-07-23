from pathlib import Path
from runpy import run_path


_COMMON = run_path(str(Path(__file__).with_name('room_315_floor_common.py')))


def generate_launch_description():
    return _COMMON['generate_floor_launch_description']('room_315_only')
