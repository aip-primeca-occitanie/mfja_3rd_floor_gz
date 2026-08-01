#!/usr/bin/env python3
"""Build the isolated, balanced local Experiment-A Smoke V2 package."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import stat
import sys
import tempfile
from pathlib import Path
from typing import Any


SEED = 31520260730
ATTEMPT = 2
PACKAGE_NAME = f"room315_experiment_a_local_smoke_v2_package_seed{SEED}_attempt{ATTEMPT}"
DEFAULT_OUTPUT = Path("/home/tiago") / PACKAGE_NAME
DEFAULT_ARCHIVE = Path("/home/tiago") / f"{PACKAGE_NAME}.tar.gz"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def write(path: Path, text: str, *, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if executable:
        path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)


def write_json(path: Path, value: Any) -> None:
    write(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def launcher() -> str:
    return f'''#!/usr/bin/env bash
set -Eeuo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
PYTHON="${{ROOM315_LOCAL_PYTHON:-/home/tiago/room315_local_training/venv/bin/python}}"
GUARD="${{ROOM315_EXPERIMENT_A_SMOKE_V2_GUARD:-/home/tiago/room315_experiment_a_local_smoke_v2_attempt{ATTEMPT}_guard_state.json}}"
OUTPUT=/home/tiago/room315_experiment_a_local_outputs/smoke_v2_seed{SEED}_attempt{ATTEMPT}
[[ -x "$PYTHON" ]] || {{ echo "missing local Torch Python: $PYTHON" >&2; exit 1; }}
[[ -f "$GUARD" ]] || {{ echo "missing Smoke V2 guard: $GUARD" >&2; exit 1; }}
[[ ! -e "$OUTPUT" ]] || {{ echo "refusing to overwrite: $OUTPUT" >&2; exit 1; }}
export ROOM315_EXPERIMENT_A_PACKAGE_ROOT="$PACKAGE_ROOT"
export ROOM315_APPROVED_RUN_ROOT="${{ROOM315_APPROVED_RUN_ROOT:-/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run}}"
export ROOM315_OLD_SPLITS_ROOT="${{ROOM315_OLD_SPLITS_ROOT:-/home/tiago/room315_arbitrary_subset_visual_splits_v1_seed31520260730}}"
export ROOM315_OLD_DATASET_ROOT="${{ROOM315_OLD_DATASET_ROOT:-/home/tiago/room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729/dataset}}"
export ROOM315_V3R1_SPLITS_ROOT="${{ROOM315_V3R1_SPLITS_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_splits_seed31520260730}}"
export ROOM315_V3R1_DATASET_ROOT="${{ROOM315_V3R1_DATASET_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_capture_seed31520260730/dataset}}"
export ROOM315_V3R1_GUARD_ROOT="${{ROOM315_V3R1_GUARD_ROOT:-/home/tiago/room315_hard_case_visual_v3r1_guard_seed31520260730}}"
export PYTHONHASHSEED=1455489658
export CUBLAS_WORKSPACE_CONFIG=:4096:8
"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_verify.py" \
  --config "$PACKAGE_ROOT/config/smoke_v2_training.json" \
  --package-root "$PACKAGE_ROOT" >/dev/null
"$PYTHON" -c 'import json,sys; data=json.load(open(sys.argv[1])); assert data["passed"]' \
  "$PACKAGE_ROOT/smoke_v2_selection_audit.json"
"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_smoke_v2_guard.py" begin \
  --guard "$GUARD" --output "$OUTPUT"
finish() {{ "$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_smoke_v2_guard.py" "$1" --guard "$GUARD"; }}
trap 'finish fail' ERR
"$PYTHON" "$PACKAGE_ROOT/scripts/experiment_a_smoke_v2.py" \
  --config "$PACKAGE_ROOT/config/smoke_v2_training.json" \
  --package-root "$PACKAGE_ROOT" --output "$OUTPUT"
trap - ERR
finish complete
'''


def build(repo: Path, output: Path, archive: Path) -> dict[str, Any]:
    if output.exists() or archive.exists():
        raise RuntimeError("refusing to overwrite Smoke V2 package or archive")
    base_builder = load_module(
        "experiment_a_base_builder",
        repo / "mfja_robot_control_config/scripts/room_315_experiment_a_v3r1_package.py",
    )
    module_root = repo / "mfja_robot_control_config/experiment_a_v3r1"
    sys.path.insert(0, str(module_root))
    smoke_v2 = load_module(
        "experiment_a_smoke_v2", module_root / "experiment_a_smoke_v2.py"
    )
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=output.parent))
    try:
        script_dir = staging / "scripts"
        script_dir.mkdir(parents=True)
        for name in (
            "experiment_a_core.py",
            "experiment_a_train.py",
            "experiment_a_verify.py",
            "experiment_a_smoke_v2.py",
            "experiment_a_smoke_v2_guard.py",
        ):
            shutil.copy2(module_root / name, script_dir / name)
        shutil.copy2(
            repo / "mfja_robot_control_config/scripts/room_315_visual_model.py",
            script_dir / "room_315_visual_model.py",
        )
        config = base_builder.local_config(
            "smoke", "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}"
        )
        config["stage"] = "smoke_v2"
        config["schema_version"] = "room315.experiment_a.local_smoke_v2_config.v1"
        config["training"]["maximum_continuation_epochs"] = 2
        config["training"]["early_stopping_patience"] = 0
        config["smoke_v2_contract"] = {
            "old_train_rows": 128,
            "v3r1_train_rows": 128,
            "validation_rows": 128,
            "baseline_before_training": True,
            "exact_continuation_epochs": 2,
            "canary_access": False,
            "legacy_test_access": False,
        }
        config["data"].pop("v3r1_canary", None)
        write_json(staging / "config/smoke_v2_training.json", config)
        environment = {
            "ROOM315_EXPERIMENT_A_PACKAGE_ROOT": str(staging),
            "ROOM315_APPROVED_RUN_ROOT": str(base_builder.LOCAL_PATHS["approved_run"]),
            "ROOM315_OLD_SPLITS_ROOT": str(base_builder.LOCAL_PATHS["old_splits"]),
            "ROOM315_OLD_DATASET_ROOT": str(base_builder.LOCAL_PATHS["old_images"]),
            "ROOM315_V3R1_SPLITS_ROOT": str(base_builder.LOCAL_PATHS["v3r1_splits"]),
            "ROOM315_V3R1_DATASET_ROOT": str(base_builder.LOCAL_PATHS["v3r1_images"]),
            "ROOM315_V3R1_GUARD_ROOT": str(base_builder.LOCAL_PATHS["guard_root"]),
        }
        old_environment = {key: os.environ.get(key) for key in environment}
        os.environ.update(environment)
        selection, selection_audit = smoke_v2.build_selection(config)
        write_json(staging / "config/smoke_v2_selection.json", selection)
        write_json(staging / "smoke_v2_selection_audit.json", selection_audit)
        v1_manifest = smoke_v2.verify_smoke_v1_immutable()
        write_json(staging / "smoke_v1_frozen_integrity.json", v1_manifest)
        invalid_v2_manifest = smoke_v2.verify_invalid_smoke_v2_attempt1_immutable()
        write_json(
            staging / "invalid_smoke_v2_attempt1_frozen_integrity.json",
            invalid_v2_manifest,
        )
        sys.path.insert(0, str(script_dir))
        verifier = load_module(
            "smoke_v2_verifier", script_dir / "experiment_a_verify.py"
        )
        static = verifier.static_audit(
            staging / "config/smoke_v2_training.json", staging,
            decode_images=False,
        )
        for key, value in old_environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        static.update({
            "smoke_v2_selection_audit": selection_audit,
            "smoke_v1_integrity": {
                "expected": smoke_v2.SMOKE_V1_TREE_SHA256,
                "actual": v1_manifest["tree_sha256"],
                "unchanged": True,
            },
            "invalid_smoke_v2_attempt1_integrity": {
                "expected": smoke_v2.INVALID_SMOKE_V2_ATTEMPT1_TREE_SHA256,
                "actual": invalid_v2_manifest["tree_sha256"],
                "unchanged": True,
                "scientific_status": "invalid_due_to_categorical_vector_order_mismatch",
            },
            "categorical_vectorization": {
                "contract": "vectorizer.names explicit order",
                "semantic_failures": selection_audit["categorical_semantic_failures"],
            },
            "checkpoint_head_strict_verification_deferred_to_runtime_before_baseline": True,
            "training_started": False,
            "full_authorized": False,
            "canary_accessed": False,
            "legacy_test_accessed": False,
        })
        write_json(staging / "smoke_v2_static_package_audit.json", static)
        guard = {
            "schema_version": "room315.experiment_a.local_smoke_v2_guard.v1",
            "approved_checkpoint_sha256": base_builder.APPROVED_SHA,
            "full_stage_authorized": False,
            "legacy_test_authorized": False,
            "canary_authorized": False,
            "smoke_v2": {"state": "unauthorized", "attempts": 0, "output": None},
        }
        write_json(staging / "config/smoke_v2_guard_template.json", guard)
        write(staging / "run_local_experiment_a_smoke_v2.sh", launcher(), executable=True)
        readme = f'''# Room 315 local Experiment-A Smoke V2, attempt {ATTEMPT}\n\nThis isolated package uses 128 old replay plus 128 V3R1 training rows and a balanced 128-scenario V3R1 validation subset. It evaluates the untouched approved epoch-14 checkpoint before creating the optimizer, then runs exactly two continuation epochs and evaluates the identical validation subset after each epoch. Attempt 2 corrects the explicit-name categorical vectorization defect detected in preserved attempt 1.\n\nOutput: `/home/tiago/room315_experiment_a_local_outputs/smoke_v2_seed{SEED}_attempt{ATTEMPT}`\n\nFull, Canary, and legacy Test access are not authorized.\n'''
        write(staging / "README.md", readme)
        write_json(staging / "package_manifest.json", base_builder.manifest(staging))
        sums = [
            f'{item["sha256"]}  {item["path"]}'
            for item in base_builder.package_files(staging, exclude_manifests=False)
        ]
        write(staging / "SHA256SUMS", "\n".join(sums) + "\n")
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    archive_sha = base_builder.deterministic_archive(output, archive)
    write(
        archive.with_suffix(archive.suffix + ".sha256"),
        f"{archive_sha}  {archive.name}\n",
    )
    return {
        "package": str(output),
        "archive": str(archive),
        "archive_sha256": archive_sha,
        "selection_audit_passed": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    args = parser.parse_args()
    print(json.dumps(build(args.repo.resolve(), args.output.resolve(), args.archive.resolve()), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
