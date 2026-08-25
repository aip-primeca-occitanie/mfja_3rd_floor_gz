#!/usr/bin/env python3
"""Launch the Room 315 Staubli trajectory exporter."""

import json
import os
import subprocess
import sys
from pathlib import Path


def _extract_payload(output):
    decoder = json.JSONDecoder()
    for start, character in enumerate(output):
        if character != "{":
            continue
        try:
            payload, end = decoder.raw_decode(output[start:])
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict) and set(payload) == {"joint_names", "points"}:
            return payload, output[:start], output[start + end :]
    raise ValueError("the HPP exporter did not produce a JointTrajectory payload")


def main():
    runner = Path(__file__).resolve().with_name("room315_hpp_line.sh")
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    completed = subprocess.run(
        [str(runner), "--print-joint-trajectory", *sys.argv[1:]],
        stdout=subprocess.PIPE,
        text=True,
    )
    if completed.returncode != 0:
        sys.stderr.write(completed.stdout)
        return completed.returncode

    try:
        payload, prefix, suffix = _extract_payload(completed.stdout)
    except ValueError as error:
        sys.stderr.write(completed.stdout)
        print(error, file=sys.stderr)
        return 1

    sys.stderr.write(prefix)
    sys.stderr.write(suffix)
    json.dump(payload, sys.stdout, indent=2, allow_nan=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
