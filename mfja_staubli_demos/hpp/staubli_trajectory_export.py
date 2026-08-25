"""Render offline Staubli JointTrajectory artifacts for manual publication."""

import json
import math


def _duration(seconds):
    seconds = float(seconds)
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError("trajectory times must be finite and non-negative")
    total_nanoseconds = round(seconds * 1e9)
    sec, nanosec = divmod(total_nanoseconds, 1_000_000_000)
    return {"sec": sec, "nanosec": nanosec}


def joint_trajectory_payload(configs, times, joint_names):
    if len(configs) != len(times):
        raise ValueError("configs and times must have the same length")
    if not configs:
        raise ValueError("trajectory must contain at least one point")

    points = []
    for config, stamp in zip(configs, times):
        if len(config) < len(joint_names):
            raise ValueError("configuration does not contain every requested joint")
        positions = [float(value) for value in config[: len(joint_names)]]
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("joint positions must be finite")
        points.append(
            {
                "positions": positions,
                # The current driver needs a non-empty velocity vector to run
                # its final-point marker logic. Zeros select its 0.1 fallback.
                "velocities": [0.0] * len(joint_names),
                "time_from_start": _duration(stamp),
            }
        )

    return {
        "joint_names": list(joint_names),
        "points": points,
    }


def render_joint_trajectory(configs, times, joint_names):
    payload = joint_trajectory_payload(configs, times, joint_names)
    return json.dumps(payload, indent=2, allow_nan=False)
