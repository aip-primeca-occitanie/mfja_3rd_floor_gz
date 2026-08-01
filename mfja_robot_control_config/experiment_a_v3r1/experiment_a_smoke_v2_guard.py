#!/usr/bin/env python3
"""Single-attempt guard for the isolated local Smoke V2 run."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any

from experiment_a_core import APPROVED_CHECKPOINT_SHA256, ExperimentAError


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("approved_checkpoint_sha256") != APPROVED_CHECKPOINT_SHA256:
        raise ExperimentAError("Smoke V2 guard checkpoint hash mismatch")
    if value.get("full_stage_authorized"):
        raise ExperimentAError("Smoke V2 guard must never authorize Full")
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("authorize", "begin", "complete", "fail", "status"))
    parser.add_argument("--guard", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    value = load(args.guard)
    if args.action == "status":
        print(json.dumps(value, indent=2, sort_keys=True))
        return
    state = value["smoke_v2"]
    if args.action == "authorize":
        if state["state"] != "unauthorized":
            raise ExperimentAError("Smoke V2 is not in an authorizable state")
        state["state"] = "authorized"
    elif args.action == "begin":
        if not args.output:
            parser.error("--output is required")
        if state["state"] != "authorized" or int(state["attempts"]) != 0:
            raise ExperimentAError("Smoke V2 is not authorized for its single attempt")
        if args.output.exists():
            raise ExperimentAError(f"Smoke V2 output already exists: {args.output}")
        state.update({"state": "running", "attempts": 1, "output": str(args.output.resolve())})
    else:
        if state["state"] != "running":
            raise ExperimentAError("Smoke V2 is not running")
        state["state"] = "completed" if args.action == "complete" else "failed"
    atomic_json(args.guard, value)


if __name__ == "__main__":
    main()
