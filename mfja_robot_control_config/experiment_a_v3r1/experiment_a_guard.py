#!/usr/bin/env python3
"""Atomic three-stage authorization guard for Experiment A."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any

from experiment_a_core import APPROVED_CHECKPOINT_SHA256, ExperimentAError, read_json


STAGES = (
    "corrected_local_full",
    "corrected_local_canary",
    "corrected_kairos_smoke",
    "corrected_kairos_full",
    "corrected_kairos_canary",
)
CANARY_FULL_STAGE = {
    "corrected_local_canary": "corrected_local_full",
    "corrected_kairos_canary": "corrected_kairos_full",
}


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True); stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try: os.unlink(temporary)
        except FileNotFoundError: pass
        raise


def load(path: Path) -> dict[str, Any]:
    value = read_json(path)
    if value.get("approved_checkpoint_sha256") != APPROVED_CHECKPOINT_SHA256:
        raise ExperimentAError("guard has wrong approved checkpoint hash")
    return value


def authorize(path: Path, stage: str) -> None:
    value = load(path)
    if stage not in value["stages"]:
        raise ExperimentAError(f"stage is unavailable in this guard: {stage}")
    if stage in CANARY_FULL_STAGE:
        full_stage = CANARY_FULL_STAGE[stage]
        if value["stages"].get(full_stage, {}).get("state") != "completed":
            raise ExperimentAError("Canary authorization requires completed Full")
    if value["stages"][stage]["state"] != "unauthorized": raise ExperimentAError(f"cannot authorize {stage} from current state")
    value["stages"][stage]["state"] = "authorized"; atomic_json(path, value)


def begin(path: Path, stage: str, output: Path) -> None:
    value = load(path)
    if stage not in value["stages"]:
        raise ExperimentAError(f"stage is unavailable in this guard: {stage}")
    item = value["stages"][stage]
    if item["state"] != "authorized": raise ExperimentAError(f"{stage} is not explicitly authorized")
    if stage.endswith("_full") and int(item.get("attempts", 0)) != 0: raise ExperimentAError("exactly one full-training attempt is allowed")
    if output.exists(): raise ExperimentAError(f"immutable result path already exists: {output}")
    if stage in CANARY_FULL_STAGE and value["stages"][CANARY_FULL_STAGE[stage]]["state"] != "completed": raise ExperimentAError("Canary requires a completed full run")
    item.update({"state": "running", "attempts": int(item.get("attempts", 0)) + 1, "output": str(output.resolve())}); atomic_json(path, value)


def finish(path: Path, stage: str, status: str) -> None:
    value = load(path); item = value["stages"][stage]
    if item["state"] != "running": raise ExperimentAError(f"{stage} is not running")
    item["state"] = status; atomic_json(path, value)


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("action", choices=("authorize", "begin", "complete", "fail", "status")); parser.add_argument("--guard", type=Path, required=True); parser.add_argument("--stage", choices=STAGES); parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.action == "status": print(json.dumps(load(args.guard), indent=2, sort_keys=True)); return
    if not args.stage: parser.error("--stage is required")
    if args.action == "authorize": authorize(args.guard, args.stage)
    elif args.action == "begin":
        if not args.output: parser.error("--output is required")
        begin(args.guard, args.stage, args.output)
    else: finish(args.guard, args.stage, "completed" if args.action == "complete" else "failed")


if __name__ == "__main__": main()
