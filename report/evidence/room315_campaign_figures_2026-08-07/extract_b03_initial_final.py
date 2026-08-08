#!/usr/bin/env python3
"""Derive an evidence-bound initial/final figure for campaign case B03.

The script reads the preserved MCAP bag and existing JSON evidence. It does not
start ROS nodes or modify the source campaign directory. Run it after sourcing
ROS 2 Jazzy and the workspace installation so the custom message types are
available.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import rosbag2_py
from PIL import Image, ImageDraw, ImageFont, __version__ as pillow_version
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


IMAGE_TOPIC = "/room_315/vla/right_rail_rgbd/image"
OBSERVATION_TOPIC = "/room_315/visual_state/observed_state"
TASK_TOPIC = "/room_315/task_goal"
CASE_ID = "B03"
IDENTITY = "R3"
REPORT_IMAGE_NAME = "b03_campaign_right_camera_initial_final.png"
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
        return json.load(stream)


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def json_stamp_to_ns(value: dict[str, int]) -> int:
    return int(value["sec"]) * 1_000_000_000 + int(value["nanosec"])


def format_stamp(ns: int) -> str:
    return f"{ns / 1_000_000_000:.3f} s"


def verify_campaign_checksums(campaign_root: Path) -> int:
    checksum_path = campaign_root / "SHA256SUMS"
    verified = 0
    for line in checksum_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        expected, relative = line.split(maxsplit=1)
        relative = relative.lstrip("*")
        candidate = campaign_root / relative
        actual = sha256_file(candidate)
        if actual != expected:
            raise RuntimeError(
                f"Campaign checksum mismatch for {relative}: {actual} != {expected}"
            )
        verified += 1
    if verified == 0:
        raise RuntimeError("Campaign checksum list is empty")
    return verified


def shuttle_record(message: Any, identity: str) -> Any:
    matches = [item for item in message.shuttles if item.identity == identity]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one {identity} record in visual observation, found {len(matches)}"
        )
    return matches[0]


def read_bag_boundaries(
    bag_dir: Path,
    saved_task_goal: dict[str, Any],
    saved_final_observation: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    for required in (TASK_TOPIC, OBSERVATION_TOPIC, IMAGE_TOPIC):
        if required not in topic_types:
            raise RuntimeError(f"Required topic missing from bag: {required}")

    task_type = get_message(topic_types[TASK_TOPIC])
    observation_type = get_message(topic_types[OBSERVATION_TOPIC])
    tasks: list[dict[str, Any]] = []
    observations: list[dict[str, Any]] = []

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic == TASK_TOPIC:
            message = deserialize_message(serialized, task_type)
            tasks.append(
                {
                    "record_ns": int(record_ns),
                    "payload": json.loads(message.data),
                }
            )
        elif topic == OBSERVATION_TOPIC:
            message = deserialize_message(serialized, observation_type)
            r3 = shuttle_record(message, IDENTITY)
            observations.append(
                {
                    "record_ns": int(record_ns),
                    "header_ns": stamp_to_ns(message.header.stamp),
                    "right_image_ns": stamp_to_ns(message.right_image_stamp),
                    "accepted": bool(message.accepted),
                    "accepted_frame_count": int(message.accepted_frame_count),
                    "checkpoint_sha256": message.checkpoint_sha256,
                    "r3": {
                        "presence_state": r3.presence_state,
                        "visual_facts_valid": bool(r3.visual_facts_valid),
                        "side": r3.side,
                        "block": r3.block,
                        "bbox_xywh": [float(value) for value in r3.bbox_xywh],
                    },
                }
            )

    if len(tasks) != 1:
        raise RuntimeError(f"Expected one TaskGoal in B03 bag, found {len(tasks)}")
    task = tasks[0]
    if task["payload"] != saved_task_goal:
        raise RuntimeError("Bag TaskGoal does not match the preserved task_goal.json")

    accepted_before = [
        item
        for item in observations
        if item["accepted"] and item["record_ns"] < task["record_ns"]
    ]
    if not accepted_before:
        raise RuntimeError("No accepted visual observation precedes the TaskGoal")
    initial = max(accepted_before, key=lambda item: item["record_ns"])

    final_header_ns = json_stamp_to_ns(saved_final_observation["header"]["stamp"])
    final_matches = [
        item for item in observations if item["header_ns"] == final_header_ns
    ]
    if len(final_matches) != 1:
        raise RuntimeError(
            "Could not identify exactly one bag observation matching "
            "final_visual_observation.json"
        )
    final = final_matches[0]
    expected_final_image_ns = json_stamp_to_ns(
        saved_final_observation["right_image_stamp"]
    )
    if not final["accepted"]:
        raise RuntimeError("The matching final observation is not accepted")
    if final["right_image_ns"] != expected_final_image_ns:
        raise RuntimeError("Final observation right-image stamp mismatch")
    if final["checkpoint_sha256"] != saved_final_observation["checkpoint_sha256"]:
        raise RuntimeError("Final observation checkpoint mismatch")
    if final["accepted_frame_count"] != saved_final_observation["accepted_frame_count"]:
        raise RuntimeError("Final observation accepted-frame count mismatch")

    for name, item in (("initial", initial), ("final", final)):
        if item["r3"]["presence_state"] != "present":
            raise RuntimeError(f"R3 is not present in the {name} observation")
        if not item["r3"]["visual_facts_valid"]:
            raise RuntimeError(f"R3 visual facts are invalid in the {name} observation")
        if item["r3"]["side"] != "right":
            raise RuntimeError(f"R3 side is not right in the {name} observation")

    return task, initial, final


def decode_image(message: Any) -> Image.Image:
    if message.encoding not in {"rgb8", "bgr8"}:
        raise RuntimeError(f"Unsupported image encoding: {message.encoding}")
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    expected_row = width * 3
    if step < expected_row:
        raise RuntimeError(f"Invalid image step {step} for width {width}")
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if raw.size != height * step:
        raise RuntimeError(
            f"Image byte count {raw.size} does not equal height*step {height * step}"
        )
    array = raw.reshape(height, step)[:, :expected_row].reshape(height, width, 3)
    if message.encoding == "bgr8":
        array = array[:, :, ::-1]
    return Image.fromarray(array.copy(), mode="RGB")


def extract_exact_images(
    bag_dir: Path, selected: dict[int, str]
) -> dict[str, dict[str, Any]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    image_type = get_message(topic_types[IMAGE_TOPIC])
    found: dict[str, dict[str, Any]] = {}

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic != IMAGE_TOPIC:
            continue
        message = deserialize_message(serialized, image_type)
        header_ns = stamp_to_ns(message.header.stamp)
        if header_ns not in selected:
            continue
        key = selected[header_ns]
        if key in found:
            raise RuntimeError(f"Multiple right-camera images have stamp {header_ns}")
        found[key] = {
            "image": decode_image(message),
            "record_ns": int(record_ns),
            "header_ns": header_ns,
            "frame_id": message.header.frame_id,
            "encoding": message.encoding,
            "width": int(message.width),
            "height": int(message.height),
            "step": int(message.step),
        }

    missing = set(selected.values()) - set(found)
    if missing:
        raise RuntimeError(f"Exact selected image frames not found: {sorted(missing)}")
    return found


def validate_slot_evidence(case_dir: Path, declaration: dict[str, Any]) -> dict[str, Any]:
    if declaration["id"] != CASE_ID:
        raise RuntimeError(f"Expected declaration {CASE_ID}")
    if declaration["expected"]["selected_identity"] != IDENTITY:
        raise RuntimeError(f"Expected selected identity {IDENTITY}")
    if declaration["expected"]["target_slot"] != "4":
        raise RuntimeError("B03 target slot is not 4")

    right_launch = declaration["launch"]["right"]
    if right_launch["identities"] != [IDENTITY] or right_launch["start_slots"] != ["3"]:
        raise RuntimeError("B03 declaration does not place only R3 at right slot 3")

    initial_controller = load_json(case_dir / "readiness/initial_controller_evidence.json")
    if initial_controller["status"] != "passed":
        raise RuntimeError("Initial controller evidence did not pass")
    initial_matches = [
        item
        for item in initial_controller["sides"]["right"]["shuttles"]
        if item["identity"] == IDENTITY
    ]
    if len(initial_matches) != 1:
        raise RuntimeError("Initial controller evidence does not contain exactly one R3")
    initial = initial_matches[0]
    if initial["declared_start_slot"] != "3":
        raise RuntimeError("Initial controller evidence does not declare slot 3")
    if initial["declared_start_sensor"] != "DZI3R":
        raise RuntimeError("Initial controller evidence does not use DZI3R")
    if initial["sensor_reading"]["active"] != 1:
        raise RuntimeError("DZI3R is not active in initial controller evidence")
    if initial["sensor_reading"]["shuttle_name"] != "room315_right_shuttle_3":
        raise RuntimeError("DZI3R does not identify R3 initially")

    terminal = load_json(case_dir / "terminal_response.json")
    if terminal["status"] != "succeeded" or terminal["reason"] != "task_goal_satisfied":
        raise RuntimeError("B03 terminal response is not successful")
    steps = terminal["result"]["executed_steps"]
    if len(steps) != 1:
        raise RuntimeError("B03 terminal response does not contain exactly one step")
    certificate = steps[0]["postcondition"]["details"]["arrival_verification"][
        "verified_slot_arrival_certificate"
    ]
    if certificate["identity"] != IDENTITY:
        raise RuntimeError("Final arrival certificate identity is not R3")
    if certificate["slot"] != "4" or certificate["reached_target_slot"] != "4":
        raise RuntimeError("Final arrival certificate does not establish slot 4")
    if certificate["sensor"] != "DZI4R" or not certificate["sensor_identity_confirmed"]:
        raise RuntimeError("Final arrival certificate does not establish DZI4R identity")
    if certificate["controller_mode"] != "DISABLED" or not certificate[
        "controller_stop_confirmed"
    ]:
        raise RuntimeError("Final arrival certificate does not establish stopped state")

    return {
        "initial": {
            "slot": "3",
            "sensor": "DZI3R",
            "sensor_identity_confirmed": True,
            "controller_mode": initial["state"]["mode"],
        },
        "final": {
            "slot": "4",
            "sensor": "DZI4R",
            "sensor_identity_confirmed": True,
            "controller_mode": certificate["controller_mode"],
            "controller_stop_confirmed": certificate["controller_stop_confirmed"],
            "sensor_confirmation_frames": certificate["sensor_confirmation_frames"],
        },
    }


def make_composite(
    initial_image: Image.Image,
    final_image: Image.Image,
    initial_stamp_ns: int,
    final_stamp_ns: int,
) -> Image.Image:
    canvas_width = 1600
    canvas_height = 810
    panel_width = 720
    panel_height = round(initial_image.height * panel_width / initial_image.width)
    image_y = 184
    left_x = 52
    right_x = canvas_width - 52 - panel_width
    background = "#ffffff"
    dark_blue = "#123f73"
    text_colour = "#1f2933"
    light_blue = "#edf4fa"
    accent = "#008577"

    canvas = Image.new("RGB", (canvas_width, canvas_height), background)
    draw = ImageDraw.Draw(canvas)
    title_font = ImageFont.truetype(str(FONT_BOLD), 42)
    panel_font = ImageFont.truetype(str(FONT_BOLD), 27)
    stamp_font = ImageFont.truetype(str(FONT_REGULAR), 23)
    note_font = ImageFont.truetype(str(FONT_REGULAR), 21)

    title = "Campaign case B03 · R3 transport · slot 3 → slot 4"
    title_box = draw.textbbox((0, 0), title, font=title_font)
    title_width = title_box[2] - title_box[0]
    draw.text(
        ((canvas_width - title_width) / 2, 26),
        title,
        font=title_font,
        fill=dark_blue,
    )
    draw.line((52, 91, canvas_width - 52, 91), fill=dark_blue, width=3)

    panels = [
        (
            "(a) Before request — slot 3",
            f"right-camera stamp: {format_stamp(initial_stamp_ns)}",
            initial_image,
            left_x,
        ),
        (
            "(b) Final accepted observation — slot 4",
            f"right-camera stamp: {format_stamp(final_stamp_ns)}",
            final_image,
            right_x,
        ),
    ]
    for heading, stamp, source, x in panels:
        draw.rounded_rectangle(
            (x - 12, 108, x + panel_width + 12, image_y + panel_height + 12),
            radius=12,
            fill=light_blue,
            outline="#b7cadb",
            width=2,
        )
        draw.text((x, 113), heading, font=panel_font, fill=text_colour)
        stamp_box = draw.textbbox((0, 0), stamp, font=stamp_font)
        stamp_width = stamp_box[2] - stamp_box[0]
        draw.text(
            (x + panel_width - stamp_width, 149),
            stamp,
            font=stamp_font,
            fill="#526577",
        )
        resized = source.resize((panel_width, panel_height), Image.Resampling.LANCZOS)
        canvas.paste(resized, (x, image_y))
        draw.rectangle(
            (x, image_y, x + panel_width, image_y + panel_height),
            outline="#6c8094",
            width=2,
        )

    arrow_y = image_y + panel_height // 2
    arrow_font = ImageFont.truetype(str(FONT_BOLD), 46)
    arrow = "→"
    arrow_box = draw.textbbox((0, 0), arrow, font=arrow_font)
    arrow_width = arrow_box[2] - arrow_box[0]
    arrow_height = arrow_box[3] - arrow_box[1]
    draw.text(
        ((canvas_width - arrow_width) / 2, arrow_y - arrow_height / 2 - arrow_box[1]),
        arrow,
        font=arrow_font,
        fill=accent,
    )

    note = (
        "Exact archived frames. Slot labels are established by identity-bearing "
        "DZI3R/DZI4R and stopped-controller evidence."
    )
    note_box = draw.textbbox((0, 0), note, font=note_font)
    note_width = note_box[2] - note_box[0]
    draw.text(
        ((canvas_width - note_width) / 2, 763),
        note,
        font=note_font,
        fill="#526577",
    )
    return canvas


def relative_to_root(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def main() -> int:
    script_path = Path(__file__).resolve()
    default_repo_root = script_path.parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=default_repo_root)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    campaign_root = repo_root / "report/evidence/room315_integrated_campaign_v2"
    case_dir = campaign_root / "cases" / CASE_ID
    output_dir = script_path.parent
    report_image = repo_root / "report/images" / REPORT_IMAGE_NAME
    output_paths = {
        "initial": output_dir / "b03_right_camera_before_request.png",
        "final": output_dir / "b03_right_camera_final_observation.png",
        "composite": output_dir / REPORT_IMAGE_NAME,
        "provenance": output_dir / "provenance.json",
        "checksums": output_dir / "SHA256SUMS",
    }
    preexisting = [path for path in [*output_paths.values(), report_image] if path.exists()]
    if preexisting and not args.force:
        names = ", ".join(str(path) for path in preexisting)
        raise RuntimeError(f"Refusing to overwrite derived outputs: {names}")

    campaign_checksum_path = campaign_root / "SHA256SUMS"
    campaign_checksum_sha256_before = sha256_file(campaign_checksum_path)
    verified_file_count = verify_campaign_checksums(campaign_root)
    declaration = load_json(case_dir / "declaration.json")
    saved_task_goal = load_json(case_dir / "task_goal.json")
    saved_final = load_json(case_dir / "final_visual_observation.json")
    slot_evidence = validate_slot_evidence(case_dir, declaration)

    task, initial_observation, final_observation = read_bag_boundaries(
        case_dir / "rosbag", saved_task_goal, saved_final
    )
    if initial_observation["right_image_ns"] >= task["record_ns"]:
        raise RuntimeError("Selected initial image is not strictly before the TaskGoal")

    selected = {
        initial_observation["right_image_ns"]: "initial",
        final_observation["right_image_ns"]: "final",
    }
    if len(selected) != 2:
        raise RuntimeError("Initial and final right-camera stamps are not distinct")
    frames = extract_exact_images(case_dir / "rosbag", selected)

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
        raise RuntimeError("Report image copy differs from evidence composite")

    verified_file_count_after = verify_campaign_checksums(campaign_root)
    campaign_checksum_sha256_after = sha256_file(campaign_checksum_path)
    if verified_file_count_after != verified_file_count:
        raise RuntimeError("Campaign checksum entry count changed during derivation")
    if campaign_checksum_sha256_after != campaign_checksum_sha256_before:
        raise RuntimeError("Campaign SHA256SUMS changed during derivation")

    source_files = [
        campaign_root / "summary.json",
        campaign_root / "manifest.json",
        campaign_root / "SHA256SUMS",
        case_dir / "rosbag/rosbag_0.mcap",
        case_dir / "rosbag/metadata.yaml",
        case_dir / "declaration.json",
        case_dir / "task_goal.json",
        case_dir / "readiness/initial_controller_evidence.json",
        case_dir / "terminal_response.json",
        case_dir / "final_visual_observation.json",
    ]
    sources = {
        relative_to_root(path, repo_root): {
            "sha256": sha256_file(path),
            "bytes": path.stat().st_size,
        }
        for path in source_files
    }

    derived_files = {
        path.name: {
            "sha256": sha256_file(path),
            "bytes": path.stat().st_size,
        }
        for key, path in output_paths.items()
        if key in {"initial", "final", "composite"}
    }
    derived_files[relative_to_root(report_image, repo_root)] = {
        "sha256": sha256_file(report_image),
        "bytes": report_image.stat().st_size,
    }

    provenance = {
        "schema_version": "room315.campaign_figure_provenance.v1",
        "derived_at_utc": datetime.now(timezone.utc).isoformat(),
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
                "Latest accepted VisualStateObservation recorded strictly before "
                "the single preserved TaskGoal; the image header stamp exactly "
                "matches that observation's right_image_stamp."
            ),
            "final": (
                "The bag VisualStateObservation whose header stamp exactly matches "
                "final_visual_observation.json; the image header stamp exactly "
                "matches that observation's right_image_stamp."
            ),
            "no_approximate_timestamp_matching": True,
        },
        "topics": {
            "task_goal": TASK_TOPIC,
            "visual_observation": OBSERVATION_TOPIC,
            "right_camera": IMAGE_TOPIC,
        },
        "timestamps": {
            "task_goal_record": {
                "nanoseconds": task["record_ns"],
                "seconds": task["record_ns"] / 1_000_000_000,
            },
            "initial": {
                "image_header_nanoseconds": frames["initial"]["header_ns"],
                "image_header_seconds": frames["initial"]["header_ns"] / 1_000_000_000,
                "image_record_nanoseconds": frames["initial"]["record_ns"],
                "observation_header_nanoseconds": initial_observation["header_ns"],
                "observation_record_nanoseconds": initial_observation["record_ns"],
                "milliseconds_before_task_goal_record": (
                    task["record_ns"] - initial_observation["record_ns"]
                )
                / 1_000_000,
            },
            "final": {
                "image_header_nanoseconds": frames["final"]["header_ns"],
                "image_header_seconds": frames["final"]["header_ns"] / 1_000_000_000,
                "image_record_nanoseconds": frames["final"]["record_ns"],
                "observation_header_nanoseconds": final_observation["header_ns"],
                "observation_record_nanoseconds": final_observation["record_ns"],
            },
        },
        "visual_observation_records": {
            "initial": initial_observation,
            "final": final_observation,
        },
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
            "campaign_checksum_manifest_sha256_before": campaign_checksum_sha256_before,
            "campaign_checksum_manifest_sha256_after": campaign_checksum_sha256_after,
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
        },
        "claim_boundary": (
            "The raw PNGs are exact RGB decodes of the archived right-camera "
            "frames; the composite contains resized copies and presentation text. "
            "Exact slot identity and stopped arrival come from the preserved "
            "DZI/controller evidence, not from the pixels alone. The figure makes "
            "no payload-state or reliability claim."
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
    checksum_lines = [
        f"{sha256_file(path)}  {path.name}" for path in sorted(checksum_files)
    ]
    output_paths["checksums"].write_text(
        "\n".join(checksum_lines) + "\n", encoding="utf-8"
    )

    print(json.dumps({
        "status": "passed",
        "evidence_directory": str(output_dir),
        "report_image": str(report_image),
        "initial_image_stamp_s": frames["initial"]["header_ns"] / 1_000_000_000,
        "task_goal_record_s": task["record_ns"] / 1_000_000_000,
        "final_image_stamp_s": frames["final"]["header_ns"] / 1_000_000_000,
        "composite_sha256": sha256_file(output_paths["composite"]),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise
