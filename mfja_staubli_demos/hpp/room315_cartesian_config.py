"""Load the Room 315 Cartesian-line YAML configuration."""

from pathlib import Path

import yaml


def load_config():
    path = Path(__file__).resolve().parents[1] / "config/room315_cartesian_line.yaml"
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)
