from pathlib import Path
import re

import yaml


MANIFEST = Path(__file__).parents[2] / "hpp_jazzy.repos"
REQUIRED_REPOSITORIES = {
    "example-robot-data",
    "hpp-environments",
    "hpp-util",
    "hpp-pinocchio",
    "hpp-statistics",
    "hpp-constraints",
    "hpp-core",
    "hpp-manipulation",
    "hpp-manipulation-urdf",
    "hpp-python",
    "hpp-gepetto-viewer",
    "hpp-exec",
}


def test_hpp_manifest_is_complete_and_exact():
    repositories = yaml.safe_load(MANIFEST.read_text())["repositories"]

    assert set(repositories) == REQUIRED_REPOSITORIES
    for source in repositories.values():
        assert source["type"] == "git"
        assert source["url"].startswith("https://")
        assert re.fullmatch(r"[0-9a-f]{40}", source["version"])
