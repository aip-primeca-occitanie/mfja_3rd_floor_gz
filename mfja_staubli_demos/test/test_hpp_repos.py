import re
from pathlib import Path

import yaml


MANIFEST = Path(__file__).parents[2] / "hpp_jazzy.repos"
REQUIRED_REPOSITORIES = {
    "toppra": (
        "https://github.com/hungpham2511/toppra.git",
        "2dfb9729d2ba2bbc962d512cc3a73b264a5ab466",
    ),
    "hpp-toppra": (
        "https://github.com/humanoid-path-planner/hpp-toppra.git",
        "83efa3beb861132484364c46d28b4ee642830c10",
    ),
    "hpp-exec": (
        "https://github.com/psardin001/hpp-exec.git",
        "d8e7c5a38e073c919326c6e55a358eee5db6751d",
    ),
}


def test_hpp_manifest_is_complete_and_exact():
    repositories = yaml.safe_load(MANIFEST.read_text())["repositories"]

    assert set(repositories) == set(REQUIRED_REPOSITORIES)
    for name, source in repositories.items():
        expected_url, expected_version = REQUIRED_REPOSITORIES[name]
        assert source["type"] == "git"
        assert source["url"] == expected_url
        assert source["version"] == expected_version
        assert re.fullmatch(r"[0-9a-f]{40}", source["version"])
