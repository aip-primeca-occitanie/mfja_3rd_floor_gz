#!/usr/bin/env python3
"""Derive the V4 B03 initial/final report figure from immutable campaign data.

The script reads the completed V4 campaign's MCAP and JSON records.  It does
not start ROS nodes or modify the campaign directory.  The historical B03
extractor is imported for its exact timestamp selection, RGB decoding and
DZI/controller certificate checks; this script adds V4-only schema auditing,
the current campaign-summary contract and a distinct report asset.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import platform
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import rosbag2_py
from PIL import Image, ImageDraw, ImageFont, __version__ as pillow_version
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CASE_ID = "B03"
IDENTITY = "R3"
OBSERVATION_TOPIC = "/room_315/visual_state/observed_state"
V4_SCHEMA = "room315.visual_state.v4"
EXPECTED_SUMMARY_SHA256 = (
    "1504f25742d135c32ea879651336ff22a64cdfb2e09870be81ea7fb68b279479"
)
REPORT_IMAGE_NAME = "b03_v4_closed_loop_right_camera_initial_final.png"
FONT_REGULAR = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FONT_BOLD = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f"Expected a JSON object: {path}")
    return value


def load_historical_extractor(repo_root: Path) -> tuple[Any, Path]:
    path = (
        repo_root
        / "report/evidence/room315_campaign_figures_2026-08-07"
        / "extract_b03_initial_final.py"
    )
    spec = importlib.util.spec_from_file_location("room315_historical_b03_extractor", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load historical extractor: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, path


def verify_summary(summary: dict[str, Any], summary_path: Path) -> str:
    actual_sha256 = sha256_file(summary_path)
    if actual_sha256 != EXPECTED_SUMMARY_SHA256:
        raise RuntimeError(
            f"Unexpected V4 campaign summary SHA-256: {actual_sha256}"
        )

    exact_values = {
        "schema_version": "room315.v4_closed_loop_campaign.v1",
        "campaign_id": "room315_integrated_campaign_v4",
        "status": "passed",
        "case_count": 12,
        "declared_case_count": 12,
        "selected_case_count": 12,
        "completed_case_count": 12,
        "passed_case_count": 12,
        "failed_case_count": 0,
        "v4_observation_count": 1784,
        "v3_observation_count": 0,
        "executed_step_count": 24,
        "satisfied_postcondition_count": 24,
        "accepted_supervisor_decision_count": 48,
        "total_replans": 4,
        "safe_abort_count": 0,
        "physical_deployment": False,
        "all_terminal_statuses_succeeded": True,
        "all_final_effects_verified": True,
        "all_controllers_stopped": True,
        "full_declared_campaign": True,
    }
    mismatches = {
        key: {"expected": expected, "actual": summary.get(key)}
        for key, expected in exact_values.items()
        if summary.get(key) != expected
    }
    if mismatches:
        raise RuntimeError(
            "V4 campaign summary does not match the report contract: "
            + json.dumps(mismatches, sort_keys=True)
        )
    return str(summary["checkpoint_sha256"])


def audit_v4_observations(
    bag_dir: Path,
    checkpoint_sha256: str,
    selected_header_ns: dict[str, int],
) -> dict[str, Any]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if OBSERVATION_TOPIC not in topic_types:
        raise RuntimeError(f"Required topic missing from B03 bag: {OBSERVATION_TOPIC}")
    observation_type = get_message(topic_types[OBSERVATION_TOPIC])

    schema_counts: Counter[str] = Counter()
    checkpoint_counts: Counter[str] = Counter()
    accepted_count = 0
    selected: dict[str, dict[str, Any]] = {}
    reverse_selection = {stamp: name for name, stamp in selected_header_ns.items()}

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic != OBSERVATION_TOPIC:
            continue
        message = deserialize_message(serialized, observation_type)
        schema_version = str(message.schema_version)
        checkpoint = str(message.checkpoint_sha256)
        header_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        schema_counts[schema_version] += 1
        checkpoint_counts[checkpoint] += 1
        accepted_count += int(bool(message.accepted))
        if header_ns in reverse_selection:
            name = reverse_selection[header_ns]
            if name in selected:
                raise RuntimeError(
                    f"Multiple V4 observations use the selected {name} header stamp"
                )
            selected[name] = {
                "record_ns": int(record_ns),
                "header_ns": header_ns,
                "schema_version": schema_version,
                "checkpoint_sha256": checkpoint,
                "accepted": bool(message.accepted),
                "accepted_frame_count": int(message.accepted_frame_count),
            }

    expected_schema_counts = {V4_SCHEMA: 80}
    expected_checkpoint_counts = {checkpoint_sha256: 80}
    if dict(schema_counts) != expected_schema_counts:
        raise RuntimeError(
            f"B03 is not V4-only: observed schema counts {dict(schema_counts)}"
        )
    if dict(checkpoint_counts) != expected_checkpoint_counts:
        raise RuntimeError(
            "B03 observation checkpoint counts differ from the campaign checkpoint"
        )
    if accepted_count != 80:
        raise RuntimeError(f"Expected 80 accepted B03 V4 observations, got {accepted_count}")
    if set(selected) != set(selected_header_ns):
        raise RuntimeError("Could not resolve both selected observations in the V4 audit")
    for name, record in selected.items():
        if (
            record["schema_version"] != V4_SCHEMA
            or record["checkpoint_sha256"] != checkpoint_sha256
            or record["accepted"] is not True
        ):
            raise RuntimeError(f"Selected {name} observation is not accepted V4 evidence")

    return {
        "observation_count": sum(schema_counts.values()),
        "accepted_count": accepted_count,
        "schema_counts": dict(schema_counts),
        "checkpoint_counts": dict(checkpoint_counts),
        "v4_observation_count": schema_counts[V4_SCHEMA],
        "v3_observation_count": sum(
            count for schema, count in schema_counts.items() if schema != V4_SCHEMA
        ),
        "selected_observations": selected,
    }


def make_composite(
    initial_image: Image.Image,
    final_image: Image.Image,
    initial_stamp_ns: int,
    final_stamp_ns: int,
) -> Image.Image:
    canvas_width = 2000
    canvas_height = 1030
    panel_width = 890
    panel_height = round(initial_image.height * panel_width / initial_image.width)
    image_y = 230
    left_x = 58
    right_x = canvas_width - 58 - panel_width

    canvas = Image.new("RGB", (canvas_width, canvas_height), "#ffffff")
    draw = ImageDraw.Draw(canvas)
    title_font = ImageFont.truetype(str(FONT_BOLD), 54)
    panel_font = ImageFont.truetype(str(FONT_BOLD), 36)
    stamp_font = ImageFont.truetype(str(FONT_REGULAR), 29)
    note_font = ImageFont.truetype(str(FONT_REGULAR), 28)
    arrow_font = ImageFont.truetype(str(FONT_BOLD), 58)

    title = "V4 campaign case B03 · R3 transport · slot 3 → slot 4"
    title_box = draw.textbbox((0, 0), title, font=title_font)
    draw.text(
        ((canvas_width - (title_box[2] - title_box[0])) / 2, 32),
        title,
        font=title_font,
        fill="#123f73",
    )
    draw.line((58, 113, canvas_width - 58, 113), fill="#123f73", width=4)

    panels = [
        (
            "(a) Initial accepted V4 observation — slot 3",
            f"right-camera stamp: {initial_stamp_ns / 1_000_000_000:.3f} s",
            initial_image,
            left_x,
        ),
        (
            "(b) Final accepted V4 observation — slot 4",
            f"right-camera stamp: {final_stamp_ns / 1_000_000_000:.3f} s",
            final_image,
            right_x,
        ),
    ]
    for heading, stamp, source, x in panels:
        draw.rounded_rectangle(
            (x - 13, 135, x + panel_width + 13, image_y + panel_height + 13),
            radius=13,
            fill="#edf4fa",
            outline="#b7cadb",
            width=3,
        )
        draw.text((x, 143), heading, font=panel_font, fill="#1f2933")
        stamp_box = draw.textbbox((0, 0), stamp, font=stamp_font)
        draw.text(
            (x + panel_width - (stamp_box[2] - stamp_box[0]), 191),
            stamp,
            font=stamp_font,
            fill="#526577",
        )
        resized = source.resize((panel_width, panel_height), Image.Resampling.LANCZOS)
        canvas.paste(resized, (x, image_y))
        draw.rectangle(
            (x, image_y, x + panel_width, image_y + panel_height),
            outline="#6c8094",
            width=3,
        )

    arrow = "→"
    arrow_box = draw.textbbox((0, 0), arrow, font=arrow_font)
    arrow_width = arrow_box[2] - arrow_box[0]
    arrow_height = arrow_box[3] - arrow_box[1]
    arrow_y = image_y + panel_height // 2
    draw.text(
        ((canvas_width - arrow_width) / 2, arrow_y - arrow_height / 2 - arrow_box[1]),
        arrow,
        font=arrow_font,
        fill="#008577",
    )

    note = (
        "Exact V4 observation-bound frames; DZI3R/DZI4R and stopped-controller "
        "records certify the slot transition."
    )
    note_box = draw.textbbox((0, 0), note, font=note_font)
    draw.text(
        ((canvas_width - (note_box[2] - note_box[0])) / 2, 969),
        note,
        font=note_font,
        fill="#526577",
    )
    return canvas


def display_path(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def main() -> int:
    script_path = Path(__file__).resolve()
    default_repo_root = script_path.parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=default_repo_root)
    parser.add_argument(
        "--campaign-root",
        type=Path,
        default=Path("/home/tiago/room315_integrated_campaign_v4_attempt1"),
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    campaign_root = args.campaign_root.resolve()
    case_dir = campaign_root / "cases" / CASE_ID
    output_dir = script_path.parent
    report_image = repo_root / "report/images" / REPORT_IMAGE_NAME
    output_paths = {
        "initial": output_dir / "b03_v4_right_camera_initial.png",
        "final": output_dir / "b03_v4_right_camera_final.png",
        "composite": output_dir / REPORT_IMAGE_NAME,
        "provenance": output_dir / "provenance.json",
        "checksums": output_dir / "SHA256SUMS",
    }
    preexisting = [path for path in [*output_paths.values(), report_image] if path.exists()]
    if preexisting and not args.force:
        raise RuntimeError(
            "Refusing to overwrite derived outputs: "
            + ", ".join(str(path) for path in preexisting)
        )

    historical, historical_script = load_historical_extractor(repo_root)
    summary_path = campaign_root / "summary.json"
    manifest_path = campaign_root / "manifest.json"
    checksum_path = campaign_root / "SHA256SUMS"
    summary = load_json(summary_path)
    checkpoint_sha256 = verify_summary(summary, summary_path)
    checksum_sha256_before = sha256_file(checksum_path)
    verified_file_count = historical.verify_campaign_checksums(campaign_root)

    declaration = load_json(case_dir / "declaration.json")
    task_goal = load_json(case_dir / "task_goal.json")
    final_visual = load_json(case_dir / "final_visual_observation.json")
    slot_evidence = historical.validate_slot_evidence(case_dir, declaration)
    task, initial_observation, final_observation = historical.read_bag_boundaries(
        case_dir / "rosbag", task_goal, final_visual
    )
    if initial_observation["record_ns"] >= task["record_ns"]:
        raise RuntimeError("Selected initial observation is not before the TaskGoal")
    if final_observation["record_ns"] <= task["record_ns"]:
        raise RuntimeError("Selected final observation is not after the TaskGoal")
    if initial_observation["checkpoint_sha256"] != checkpoint_sha256:
        raise RuntimeError("Initial observation uses the wrong V4 checkpoint")

    selected_header_ns = {
        "initial": initial_observation["header_ns"],
        "final": final_observation["header_ns"],
    }
    v4_audit = audit_v4_observations(
        case_dir / "rosbag", checkpoint_sha256, selected_header_ns
    )
    case_summary = load_json(case_dir / "case_summary.json")
    recorded_audit = case_summary["rosbag"]["visual_schema_audit"]
    for key in (
        "observation_count",
        "accepted_count",
        "schema_counts",
        "checkpoint_counts",
        "v4_observation_count",
        "v3_observation_count",
    ):
        if recorded_audit[key] != v4_audit[key]:
            raise RuntimeError(f"Independent B03 V4 audit differs for {key}")

    selected_image_stamps = {
        initial_observation["right_image_ns"]: "initial",
        final_observation["right_image_ns"]: "final",
    }
    if len(selected_image_stamps) != 2:
        raise RuntimeError("Initial and final right-camera stamps are not distinct")
    frames = historical.extract_exact_images(case_dir / "rosbag", selected_image_stamps)

    output_dir.mkdir(parents=True, exist_ok=True)
    report_image.parent.mkdir(parents=True, exist_ok=True)
    frames["initial"]["image"].save(output_paths["initial"], compress_level=9)
    frames["final"]["image"].save(output_paths["final"], compress_level=9)
    composite = make_composite(
        frames["initial"]["image"],
        frames["final"]["image"],
        frames["initial"]["header_ns"],
        frames["final"]["header_ns"],
    )
    composite.save(output_paths["composite"], compress_level=9)
    composite.save(report_image, compress_level=9)
    if sha256_file(output_paths["composite"]) != sha256_file(report_image):
        raise RuntimeError("Report image differs from the evidence composite")

    verified_file_count_after = historical.verify_campaign_checksums(campaign_root)
    checksum_sha256_after = sha256_file(checksum_path)
    if verified_file_count_after != verified_file_count:
        raise RuntimeError("Campaign checksum entry count changed during derivation")
    if checksum_sha256_after != checksum_sha256_before:
        raise RuntimeError("Campaign SHA256SUMS changed during derivation")

    source_files = [
        summary_path,
        manifest_path,
        checksum_path,
        historical_script,
        case_dir / "case_summary.json",
        case_dir / "rosbag/rosbag_0.mcap",
        case_dir / "rosbag/metadata.yaml",
        case_dir / "declaration.json",
        case_dir / "task_goal.json",
        case_dir / "readiness/initial_controller_evidence.json",
        case_dir / "terminal_response.json",
        case_dir / "final_visual_observation.json",
    ]
    sources = {
        display_path(path, repo_root): {
            "sha256": sha256_file(path),
            "bytes": path.stat().st_size,
        }
        for path in source_files
    }
    derived_files = {
        path.name: {"sha256": sha256_file(path), "bytes": path.stat().st_size}
        for key, path in output_paths.items()
        if key in {"initial", "final", "composite"}
    }
    derived_files[display_path(report_image, repo_root)] = {
        "sha256": sha256_file(report_image),
        "bytes": report_image.stat().st_size,
    }

    provenance = {
        "schema_version": "room315.v4_campaign_figure_provenance.v1",
        "derived_at_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_id": summary["campaign_id"],
        "campaign_summary_sha256": EXPECTED_SUMMARY_SHA256,
        "case_id": CASE_ID,
        "selected_identity": IDENTITY,
        "declared_transition": {
            "side": declaration["side"],
            "start_slot": "3",
            "target_slot": "4",
            "utterance": declaration["utterance"],
        },
        "selection_contract": {
            "initial": (
                "Latest accepted V4 observation recorded strictly before the single "
                "TaskGoal; the exact right_image_stamp selects the camera frame."
            ),
            "final": (
                "Accepted V4 observation whose header stamp exactly matches "
                "final_visual_observation.json; its exact right_image_stamp selects "
                "the camera frame."
            ),
            "no_approximate_timestamp_matching": True,
        },
        "timestamps": {
            "task_goal_record_ns": task["record_ns"],
            "initial": {
                "observation_record_ns": initial_observation["record_ns"],
                "observation_header_ns": initial_observation["header_ns"],
                "image_header_ns": frames["initial"]["header_ns"],
            },
            "final": {
                "observation_record_ns": final_observation["record_ns"],
                "observation_header_ns": final_observation["header_ns"],
                "image_header_ns": frames["final"]["header_ns"],
            },
        },
        "selected_visual_observations": {
            "initial": initial_observation,
            "final": final_observation,
        },
        "independent_b03_v4_schema_audit": v4_audit,
        "slot_evidence": slot_evidence,
        "image_contract": {
            "encoding": frames["initial"]["encoding"],
            "width": frames["initial"]["width"],
            "height": frames["initial"]["height"],
            "frame_id": frames["initial"]["frame_id"],
            "raw_pngs_are_direct_rgb_decodes": True,
            "composite_operations": ["resizing", "panel layout", "text labels"],
            "visual_bbox_overlay_used": False,
            "slot_labels_are_not_inferred_from_pixels": True,
        },
        "source_integrity": {
            "campaign_sha256sums_verified": True,
            "campaign_checksum_entry_count": verified_file_count,
            "campaign_checksum_manifest_sha256_before": checksum_sha256_before,
            "campaign_checksum_manifest_sha256_after": checksum_sha256_after,
            "campaign_checksum_contract_unchanged": True,
        },
        "sources": sources,
        "derived_files": derived_files,
        "software": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "pillow": pillow_version,
            "font_regular": {
                "path": str(FONT_REGULAR),
                "sha256": sha256_file(FONT_REGULAR),
            },
            "font_bold": {
                "path": str(FONT_BOLD),
                "sha256": sha256_file(FONT_BOLD),
            },
            "script_sha256": sha256_file(script_path),
            "historical_template_sha256": sha256_file(historical_script),
        },
        "claim_boundary": (
            "The raw PNGs are exact RGB decodes of V4 observation-bound right-camera "
            "frames. The composite only resizes those frames and adds labels. Exact "
            "slot identity and stopped arrival come from the preserved DZI/controller "
            "certificate, not from pixels alone. This is Gazebo evidence; physical "
            "deployment is false."
        ),
    }
    output_paths["provenance"].write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    checksum_files = [
        script_path,
        output_paths["initial"],
        output_paths["final"],
        output_paths["composite"],
        output_paths["provenance"],
    ]
    output_paths["checksums"].write_text(
        "".join(
            f"{sha256_file(path)}  {path.name}\n"
            for path in sorted(checksum_files, key=lambda item: item.name)
        ),
        encoding="utf-8",
    )

    print(
        json.dumps(
            {
                "status": "passed",
                "evidence_directory": str(output_dir),
                "report_image": str(report_image),
                "summary_sha256": EXPECTED_SUMMARY_SHA256,
                "b03_v4_observation_count": v4_audit["v4_observation_count"],
                "b03_v3_observation_count": v4_audit["v3_observation_count"],
                "initial_image_stamp_s": frames["initial"]["header_ns"]
                / 1_000_000_000,
                "final_image_stamp_s": frames["final"]["header_ns"]
                / 1_000_000_000,
                "composite_sha256": sha256_file(output_paths["composite"]),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise
