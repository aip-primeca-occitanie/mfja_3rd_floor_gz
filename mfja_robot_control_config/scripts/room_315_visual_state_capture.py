#!/usr/bin/env python3
"""Capture one Room 315 scene directly into the visual-state dataset format."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from mfja_rail_interfaces.msg import ShuttleState as RailShuttleState
from mfja_rail_interfaces.msg import SwitchState as RailSwitchState
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_label_exporter import CALIBRATION_VERSION
from room_315_visual_label_exporter import _default_camera_model_path
from room_315_visual_label_exporter import load_camera_projections
from room_315_visual_label_exporter import rail_pose_to_gazebo
from room_315_visual_label_exporter import shuttle_bbox
from room_315_visual_scenario_generator import REQUIRED_CAMERAS
from room_315_visual_scenario_generator import _read_manifest
from room_315_visual_state_dataset import DATASET_MODE_VISUAL_STATE
from room_315_visual_state_dataset import VISUAL_STATE_SCHEMA_VERSION
from room_315_visual_state_dataset import normalize_visual_state_labels
from room_315_visual_state_dataset import pretty_json


CAMERA_TOPICS = {
    'left_rail_rgb': '/room_315/vla/left_rail_rgbd/image',
    'right_rail_rgb': '/room_315/vla/right_rail_rgbd/image',
}
SHUTTLE_TOPICS = {
    'left': '/room_315/rails/left/shuttles/state',
    'right': '/room_315/rails/right/shuttles/state',
}
PAYLOAD_TOPICS = {
    'left': '/room_315/rails/left/shuttles/payload_state',
    'right': '/room_315/rails/right/shuttles/payload_state',
}
SWITCH_TOPICS = {
    'left': '/room_315/rails/left/switches/state',
    'right': '/room_315/rails/right/switches/state',
}
VALIDATION_SCHEMA_VERSION = 'room315.visual_capture_validation.v1'


class VisualCaptureError(ValueError):
    """Raised when a scene cannot be captured safely."""


@dataclass
class CaptureSnapshot:
    images: dict[str, Image] = field(default_factory=dict)
    image_stamps: dict[str, float] = field(default_factory=dict)
    shuttles: dict[str, dict[str, dict[str, Any]]] = field(
        default_factory=lambda: {'left': {}, 'right': {}}
    )
    shuttle_updates: set[str] = field(default_factory=set)
    payloads: dict[str, dict[str, bool]] = field(
        default_factory=lambda: {'left': {}, 'right': {}}
    )
    payload_updates: set[str] = field(default_factory=set)
    switches: dict[str, dict[str, str]] = field(
        default_factory=lambda: {'left': {}, 'right': {}}
    )
    switch_updates: set[str] = field(default_factory=set)


def _message_stamp(message: Image) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _short_id(entity_name: str, side: str) -> str:
    prefix = 'R' if side == 'right' else 'L'
    expected_prefix = f'room315_{side}_shuttle_'
    if not entity_name.startswith(expected_prefix):
        raise VisualCaptureError(
            f'unsupported {side} shuttle entity name: {entity_name!r}'
        )
    index = entity_name.removeprefix(expected_prefix)
    if index not in {'1', '2', '3', '4'}:
        raise VisualCaptureError(f'unsupported shuttle index in {entity_name!r}')
    return f'{prefix}{index}'


def _payload_map(raw: str, side: str) -> dict[str, bool]:
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise VisualCaptureError(f'{side} payload state is invalid JSON') from exc
    if not isinstance(parsed, dict) or not isinstance(parsed.get('shuttles'), list):
        raise VisualCaptureError(f'{side} payload state is missing shuttles')
    result = {}
    for item in parsed['shuttles']:
        if not isinstance(item, dict):
            continue
        entity_name = str(item.get('entity_name') or '').strip()
        if entity_name:
            result[entity_name] = bool(item.get('loaded'))
    return result


def _switch_state(raw: Any) -> str:
    normalized = str(raw or '').strip().lower()
    aliases = {
        'e': 'exterior',
        'exterior': 'exterior',
        'i': 'interior',
        'interior': 'interior',
    }
    return aliases.get(normalized, normalized)


def visual_labels_from_snapshot(
    snapshot: CaptureSnapshot,
    cameras: dict[str, Any],
) -> dict[str, Any]:
    shuttles = []
    for side in ('right', 'left'):
        camera = cameras[side]
        for entity_name, state in sorted(snapshot.shuttles[side].items()):
            bbox = shuttle_bbox(camera, rail_pose_to_gazebo(side, state))
            if bbox is None:
                continue
            loaded = snapshot.payloads[side].get(entity_name)
            short_id = _short_id(entity_name, side)
            location = {'side': side}
            block = str(state.get('segment') or '').strip().upper()
            if block:
                location['block'] = block
            shuttles.append({
                'id': short_id,
                'visually_available_identity': short_id,
                'identity_available': True,
                'bbox': bbox,
                'location': location,
                'loaded_state': (
                    'loaded' if loaded is True else 'empty' if loaded is False else 'unknown'
                ),
                'confidence': 1.0,
            })
    switches = [
        {
            'id': f'{side}:{name.upper()}',
            'state': state.lower(),
            'confidence': 1.0,
        }
        for side in ('right', 'left')
        for name, state in sorted(snapshot.switches[side].items())
        if state in {'interior', 'exterior'}
    ]
    return normalize_visual_state_labels({
        'visual_state_labels': {
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'calibration_version': CALIBRATION_VERSION,
            'confidence': 1.0,
            'shuttles': shuttles,
            'switches': switches,
            'obstacles': [],
        }
    })


def _expected_shuttles(scenario: dict[str, Any]) -> dict[str, dict[str, str]]:
    return {
        side: {
            shuttle['id']: shuttle['loaded_state']
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        }
        for side in ('right', 'left')
    }


def validate_snapshot(
    snapshot: CaptureSnapshot,
    scenario: dict[str, Any],
    labels: dict[str, Any],
    *,
    max_camera_skew_seconds: float,
) -> None:
    missing_images = sorted(set(REQUIRED_CAMERAS) - set(snapshot.images))
    if missing_images:
        raise VisualCaptureError(f'missing camera images: {missing_images}')
    skew = max(snapshot.image_stamps.values()) - min(snapshot.image_stamps.values())
    if skew > max_camera_skew_seconds:
        raise VisualCaptureError(
            f'camera timestamp skew {skew:.4f}s exceeds {max_camera_skew_seconds:.4f}s'
        )
    missing_updates = {
        'shuttles': sorted(set(('right', 'left')) - snapshot.shuttle_updates),
        'payloads': sorted(set(('right', 'left')) - snapshot.payload_updates),
        'switches': sorted(set(('right', 'left')) - snapshot.switch_updates),
    }
    missing_updates = {key: value for key, value in missing_updates.items() if value}
    if missing_updates:
        raise VisualCaptureError(f'missing simulator state updates: {missing_updates}')

    expected = _expected_shuttles(scenario)
    actual = {
        side: {
            _short_id(entity_name, side): (
                'loaded'
                if snapshot.payloads[side].get(entity_name) is True
                else 'empty'
                if snapshot.payloads[side].get(entity_name) is False
                else 'unknown'
            )
            for entity_name in snapshot.shuttles[side]
        }
        for side in ('right', 'left')
    }
    if actual != expected:
        raise VisualCaptureError(
            f'simulator shuttle/payload state does not match scenario; '
            f'expected={expected}, actual={actual}'
        )
    if len(labels['shuttles']) != sum(len(value) for value in expected.values()):
        raise VisualCaptureError(
            'one or more expected shuttles are outside the calibrated camera view'
        )
    expected_switches = {
        f'{side}:{name}': state
        for side in ('right', 'left')
        for name, state in scenario['scene']['rails'][side]['switches'].items()
    }
    actual_switches = {
        item['id']: item['state']
        for item in labels['switches']
    }
    if actual_switches != expected_switches:
        raise VisualCaptureError(
            f'simulator switches do not match scenario; '
            f'expected={expected_switches}, actual={actual_switches}'
        )


class VisualStateCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('room_315_visual_state_capture')
        self.snapshot = CaptureSnapshot()
        self.bridge = CvBridge()
        for camera_name, topic in CAMERA_TOPICS.items():
            self.create_subscription(
                Image,
                topic,
                lambda message, name=camera_name: self._on_image(name, message),
                10,
            )
        for side, topic in SHUTTLE_TOPICS.items():
            self.create_subscription(
                RailShuttleState,
                topic,
                lambda message, rail_side=side: self._on_shuttle(rail_side, message),
                10,
            )
        for side, topic in PAYLOAD_TOPICS.items():
            self.create_subscription(
                String,
                topic,
                lambda message, rail_side=side: self._on_payload(rail_side, message),
                10,
            )
        for side, topic in SWITCH_TOPICS.items():
            self.create_subscription(
                RailSwitchState,
                topic,
                lambda message, rail_side=side: self._on_switch(rail_side, message),
                10,
            )

    def _on_image(self, camera_name: str, message: Image) -> None:
        self.snapshot.images[camera_name] = message
        self.snapshot.image_stamps[camera_name] = _message_stamp(message)

    def _on_shuttle(self, side: str, message: RailShuttleState) -> None:
        self.snapshot.shuttle_updates.add(side)
        entity_name = str(message.name or '').strip()
        if not entity_name:
            self.snapshot.shuttles[side] = {}
            return
        self.snapshot.shuttles[side][entity_name] = {
            'x': float(message.x),
            'y': float(message.y),
            'z': float(message.z),
            'yaw': float(message.yaw),
            'segment': str(message.current_segment or ''),
        }

    def _on_payload(self, side: str, message: String) -> None:
        try:
            self.snapshot.payloads[side] = _payload_map(message.data, side)
            self.snapshot.payload_updates.add(side)
        except VisualCaptureError as exc:
            self.get_logger().error(str(exc))

    def _on_switch(self, side: str, message: RailSwitchState) -> None:
        self.snapshot.switches[side] = {
            str(item.name).upper(): _switch_state(item.state)
            for item in message.switches
        }
        self.snapshot.switch_updates.add(side)


def _scenario_from_manifest(path: Path, scenario_id: str) -> dict[str, Any]:
    matches = [
        row
        for row in _read_manifest(path)
        if row['scenario_id'] == scenario_id
    ]
    if len(matches) != 1:
        raise VisualCaptureError(
            f'expected one scenario_id {scenario_id!r} in {path}, found {len(matches)}'
        )
    return matches[0]


def _image_digest(images: dict[str, bytes]) -> str:
    digest = hashlib.sha256()
    for name in REQUIRED_CAMERAS:
        digest.update(name.encode('utf-8'))
        digest.update(b'\0')
        digest.update(images[name])
        digest.update(b'\0')
    return digest.hexdigest()


def _existing_image_fingerprints(dataset_root: Path) -> set[str]:
    path = dataset_root / 'meta' / 'capture_fingerprints.jsonl'
    if not path.is_file():
        return set()
    fingerprints = set()
    with path.open('r', encoding='utf-8') as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            if not raw_line.strip():
                continue
            try:
                row = json.loads(raw_line)
            except json.JSONDecodeError as exc:
                raise VisualCaptureError(
                    f'{path}:{line_number}: invalid JSON: {exc}'
                ) from exc
            fingerprints.add(str(row.get('image_pair_sha256') or ''))
    return fingerprints


def _encode_images(node: VisualStateCaptureNode) -> dict[str, bytes]:
    encoded = {}
    for camera in REQUIRED_CAMERAS:
        frame = node.bridge.imgmsg_to_cv2(
            node.snapshot.images[camera],
            desired_encoding='bgr8',
        )
        success, payload = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not success:
            raise VisualCaptureError(f'could not JPEG-encode {camera}')
        encoded[camera] = payload.tobytes()
    return encoded


def write_capture(
    dataset_root: Path,
    scenario: dict[str, Any],
    labels: dict[str, Any],
    images: dict[str, bytes],
    *,
    camera_skew_seconds: float,
) -> dict[str, Any]:
    dataset_root = dataset_root.expanduser().resolve()
    episode_id = scenario['scenario_id']
    episode_dir = dataset_root / 'episodes' / episode_id
    if episode_dir.exists():
        raise FileExistsError(f'episode already exists: {episode_dir}')
    image_fingerprint = _image_digest(images)
    if image_fingerprint in _existing_image_fingerprints(dataset_root):
        raise VisualCaptureError(
            f'exact camera pair already exists in dataset: {image_fingerprint}'
        )

    image_refs = {}
    for camera in REQUIRED_CAMERAS:
        relative = Path('episodes') / episode_id / 'images' / camera / 'frame_000000.jpg'
        destination = dataset_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(images[camera])
        image_refs[camera] = relative.as_posix()

    row = {
        'dataset_mode': DATASET_MODE_VISUAL_STATE,
        'sample_id': f'{episode_id}:step:0',
        'episode_id': episode_id,
        'step_index': 0,
        'scenario_family': scenario['scenario_family'],
        'model_input': {
            'overhead_images': image_refs,
        },
        'visual_state_labels': labels,
        'oracle_label_provenance': {
            'source': 'gazebo_state_projected_to_synchronized_overhead_cameras',
            'calibration_version': CALIBRATION_VERSION,
            'model_input_exposure': 'excluded_after_split',
        },
    }
    validation = {
        'schema_version': VALIDATION_SCHEMA_VERSION,
        'scenario_id': scenario['scenario_id'],
        'scenario_family': scenario['scenario_family'],
        'validation_status': 'approved',
        'approved_for_training': True,
        'capture_complete': True,
        'labels_valid': True,
        'required_cameras': list(REQUIRED_CAMERAS),
        'camera_skew_seconds': round(camera_skew_seconds, 6),
        'image_pair_sha256': image_fingerprint,
    }
    (episode_dir / 'validation.json').write_text(
        pretty_json(validation) + '\n',
        encoding='utf-8',
    )
    meta_dir = dataset_root / 'meta'
    meta_dir.mkdir(parents=True, exist_ok=True)
    with (meta_dir / 'training_events.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')
    with (meta_dir / 'capture_fingerprints.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps({
            'sample_id': row['sample_id'],
            'image_pair_sha256': image_fingerprint,
        }, separators=(',', ':')) + '\n')
    return {
        'dataset_root': str(dataset_root),
        'episode_id': episode_id,
        'training_row': str(meta_dir / 'training_events.jsonl'),
        'validation': str(episode_dir / 'validation.json'),
        'images': image_refs,
        'image_pair_sha256': image_fingerprint,
    }


def capture_scenario(
    scenario: dict[str, Any],
    dataset_root: Path,
    *,
    timeout_seconds: float,
    max_camera_skew_seconds: float,
    camera_model_sdf: Path | None = None,
) -> dict[str, Any]:
    cameras = load_camera_projections(camera_model_sdf or _default_camera_model_path())
    rclpy.init(args=None)
    node = VisualStateCaptureNode()
    deadline = time.monotonic() + timeout_seconds
    try:
        labels = None
        last_error = 'waiting for simulator data'
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                labels = visual_labels_from_snapshot(node.snapshot, cameras)
                validate_snapshot(
                    node.snapshot,
                    scenario,
                    labels,
                    max_camera_skew_seconds=max_camera_skew_seconds,
                )
                break
            except (KeyError, VisualCaptureError) as exc:
                last_error = str(exc)
        else:
            raise VisualCaptureError(
                f'capture timed out after {timeout_seconds:.1f}s: {last_error}'
            )
        images = _encode_images(node)
        skew = max(node.snapshot.image_stamps.values()) - min(
            node.snapshot.image_stamps.values()
        )
        return write_capture(
            dataset_root,
            scenario,
            labels,
            images,
            camera_skew_seconds=skew,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Capture one configured Room 315 Gazebo scene directly as paired '
            'overhead images plus separately removable oracle visual labels.'
        )
    )
    parser.add_argument('--scenario-manifest', type=Path, required=True)
    parser.add_argument('--scenario-id', required=True)
    parser.add_argument('--output-dataset', type=Path, required=True)
    parser.add_argument('--timeout-seconds', type=float, default=30.0)
    parser.add_argument('--max-camera-skew-seconds', type=float, default=0.15)
    parser.add_argument('--camera-model-sdf', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    scenario = _scenario_from_manifest(args.scenario_manifest, args.scenario_id)
    result = capture_scenario(
        scenario,
        args.output_dataset,
        timeout_seconds=float(args.timeout_seconds),
        max_camera_skew_seconds=float(args.max_camera_skew_seconds),
        camera_model_sdf=args.camera_model_sdf,
    )
    print(pretty_json(result))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (FileExistsError, OSError, VisualCaptureError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
