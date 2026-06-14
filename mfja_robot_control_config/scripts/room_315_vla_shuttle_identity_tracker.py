#!/usr/bin/env python3
"""Privileged Room 315 shuttle identity tracker.

The tracker fuses perimeter marker detections into shuttle identity tracks for
safety, debugging, dataset metadata, and non-deployable ablations. Its outputs
must not be inserted into model_input.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_multi_shuttle import identity_tracks_from_marker_detections
from room_315_multi_shuttle import load_identity_config
from room_315_multi_shuttle import validate_identity_config


def _default_identity_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mfja_robot_control_config'))
            / 'config'
            / 'room_315_vla'
            / 'shuttle_identity.yaml'
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'room_315_vla'
            / 'shuttle_identity.yaml'
        )


def tracks_from_detection_message(
    raw: str,
    identity_config: dict[str, Any],
    *,
    now_s: float | None = None,
) -> list[dict[str, Any]]:
    """Convert JSON detections into privileged shuttle identity tracks."""

    parsed = json.loads(raw or '{}')
    if isinstance(parsed, list):
        detections = parsed
    elif isinstance(parsed, dict):
        detections = parsed.get('detections', [])
    else:
        detections = []
    if not isinstance(detections, list):
        detections = []
    tracks = identity_tracks_from_marker_detections(detections, identity_config)
    stamp = time.time() if now_s is None else float(now_s)
    for track in tracks:
        track['timestamp'] = round(stamp, 6)
        track.setdefault('source', 'perimeter_marker_tracker')
        track.setdefault('model_input_exposure', 'excluded')
    return tracks


class IdentityTrackMemory:
    """Short-lived privileged track memory for intermittent marker detections."""

    def __init__(self, *, max_lost_s: float = 0.75) -> None:
        self.max_lost_s = max(float(max_lost_s), 0.0)
        self._tracks: dict[str, dict[str, Any]] = {}

    def update(self, tracks: list[dict[str, Any]], *, now_s: float | None = None) -> list[dict[str, Any]]:
        stamp = time.time() if now_s is None else float(now_s)
        current_ids = set()
        for track in tracks:
            shuttle_id = str(track.get('shuttle_id') or '')
            if not shuttle_id:
                continue
            updated = dict(track)
            updated['timestamp'] = round(stamp, 6)
            updated['last_seen_timestamp'] = round(stamp, 6)
            updated.setdefault('model_input_exposure', 'excluded')
            self._tracks[shuttle_id] = updated
            current_ids.add(shuttle_id)

        remembered = list(tracks)
        for shuttle_id, previous in sorted(self._tracks.items()):
            if shuttle_id in current_ids:
                continue
            last_seen = float(previous.get('last_seen_timestamp', previous.get('timestamp', stamp)) or stamp)
            age_s = max(stamp - last_seen, 0.0)
            if age_s > self.max_lost_s:
                continue
            lost = dict(previous)
            lost.update({
                'timestamp': round(stamp, 6),
                'track_age_since_seen_s': round(age_s, 6),
                'visibility_state': 'lost',
                'confidence': max(float(previous.get('confidence') or 0.0) * 0.5, 0.05),
                'model_input_exposure': 'excluded',
            })
            remembered.append(lost)
        return sorted(remembered, key=lambda item: str(item.get('shuttle_id') or ''))


class Room315VlaShuttleIdentityTracker:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__('room_315_vla_shuttle_identity_tracker')
                self.declare_parameter('identity_config_path', str(_default_identity_config_path()))
                self.declare_parameter('detections_topic', '/room_315/vla/fiducial_detections')
                self.declare_parameter('tracks_topic', '/room_315/vla/shuttle_identity_tracks')
                self.declare_parameter('debug_topic', '/room_315/vla/shuttle_identity_debug')
                config_path = Path(str(self.get_parameter('identity_config_path').value)).expanduser()
                self.identity_config = load_identity_config(config_path)
                validate_identity_config(self.identity_config)
                self.track_memory = IdentityTrackMemory()
                self.String = String
                self.tracks_pub = self.create_publisher(
                    String,
                    str(self.get_parameter('tracks_topic').value),
                    10,
                )
                self.debug_pub = self.create_publisher(
                    String,
                    str(self.get_parameter('debug_topic').value),
                    10,
                )
                self.create_subscription(
                    String,
                    str(self.get_parameter('detections_topic').value),
                    self._on_detections,
                    10,
                )

            def _on_detections(self, msg: Any) -> None:
                try:
                    tracks = tracks_from_detection_message(
                        msg.data,
                        self.identity_config,
                        now_s=self.get_clock().now().nanoseconds / 1e9,
                    )
                    tracks = self.track_memory.update(
                        tracks,
                        now_s=self.get_clock().now().nanoseconds / 1e9,
                    )
                    payload = {
                        'tracks': tracks,
                        'model_input_exposure': 'excluded',
                    }
                    out = self.String()
                    out.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
                    self.tracks_pub.publish(out)
                    debug = self.String()
                    debug.data = json.dumps(
                        {
                            'track_count': len(tracks),
                            'low_confidence_tracks': [
                                track['shuttle_id']
                                for track in tracks
                                if float(track.get('confidence') or 0.0) < 0.75
                            ],
                            'model_input_exposure': 'excluded',
                        },
                        ensure_ascii=False,
                        sort_keys=True,
                    )
                    self.debug_pub.publish(debug)
                except Exception as exc:
                    debug = self.String()
                    debug.data = json.dumps({'error': str(exc), 'model_input_exposure': 'excluded'})
                    self.debug_pub.publish(debug)

        self.rclpy = rclpy
        self.node = _Node()

    def spin(self) -> None:
        self.rclpy.spin(self.node)


def main() -> int:
    import rclpy

    rclpy.init()
    tracker = Room315VlaShuttleIdentityTracker()
    try:
        tracker.spin()
    finally:
        tracker.node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
