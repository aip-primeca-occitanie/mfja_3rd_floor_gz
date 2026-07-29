#!/usr/bin/env python3
"""Build and validate the human review gate for Room 315 visual captures."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import os
import shutil
import sys
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_fleet import FIXED_VISUAL_SHUTTLE_IDENTITIES
from room_315_visual_scenario_generator import REQUIRED_CAMERAS
from room_315_visual_state_dataset import VISUAL_STATE_SCHEMA_VERSION
from room_315_visual_state_dataset import normalize_visual_state_labels


GALLERY_SCHEMA_VERSION = 'room315.manual_inspection_gallery.v1'
APPROVAL_SCHEMA_VERSION = 'room315.manual_smoke_approval.v1'
CURRENT_APPROVAL_SCHEMA_VERSION = 'room315.arbitrary_subset_smoke_approval.v1'
WAITING_MESSAGE = 'WAITING_FOR_MANUAL_APPROVAL'
ROLE_COLORS = {
    'target': '#ffd43b',
    'blocker': '#ff4d4f',
    'non_blocker': '#40a9ff',
    'relation_neutral': '#52c41a',
    'opposite_rail_neutral': '#b37feb',
    'active_unassigned': '#ff9c6e',
    'absent': '#8c8c8c',
}
SCENARIO_REVIEW_BOOLEAN_FIELDS = (
    'reviewed',
    'both_camera_images_reviewed',
    'expected_shuttle_count_correct',
    'identity_labels_correct',
    'l4_r4_status_correct',
    'bounding_boxes_correct',
    'payload_states_correct',
    'segments_and_positions_plausible',
    'relation_semantics_correct',
    'neutral_shuttles_non_interfering',
    'no_hidden_or_absent_shuttle_labelled_visible',
    'camera_views_synchronized',
    'scenario_pass',
)
AGGREGATE_REVIEW_BOOLEAN_FIELDS = (
    'all_scenarios_reviewed',
    'all_images_reviewed',
    'l4_visible_and_correctly_labelled',
    'r4_visible_and_correctly_labelled',
    'four_plus_four_scenes_correct',
    'bounding_boxes_acceptable',
    'payload_states_visually_correct',
    'relation_semantics_visually_correct',
    'neutral_shuttles_non_interfering',
    'no_hidden_or_absent_shuttle_labelled_visible',
    'camera_views_synchronized',
    'approved_for_full_manifest_generation',
)
LEGACY_AGGREGATE_REVIEW_ALIASES = {
    'all_scenarios_reviewed': 'all_20_scenarios_reviewed',
    'all_images_reviewed': 'all_40_images_reviewed',
}
ARBITRARY_IDENTITY_RAIL_SCOPE = 'arbitrary_identity_subset'
LEGACY_RAIL_MEMBERSHIP = {
    'left_four': (
        ('L1', 'L2', 'L3', 'L4'),
        (),
    ),
    'right_four': (
        (),
        ('R1', 'R2', 'R3', 'R4'),
    ),
    'dual_four_plus_four': (
        ('L1', 'L2', 'L3', 'L4'),
        ('R1', 'R2', 'R3', 'R4'),
    ),
}
CURRENT_APPROVAL_NAME = 'smoke_manual_approval.json'
LEGACY_APPROVAL_NAME = 'manual_smoke_approval.json'


class ManualReviewError(ValueError):
    """Raised when review artifacts cannot be generated or validated."""


def _read_jsonl_objects(path: Path, *, id_field: str) -> list[dict[str, Any]]:
    path = path.expanduser()
    if not path.is_file():
        raise ManualReviewError(f'JSONL file is missing: {path}')
    rows: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    with path.open('r', encoding='utf-8') as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            if not raw_line.strip():
                continue
            try:
                row = json.loads(raw_line)
            except json.JSONDecodeError as exc:
                raise ManualReviewError(
                    f'{path}:{line_number}: invalid JSON: {exc}'
                ) from exc
            if not isinstance(row, dict):
                raise ManualReviewError(
                    f'{path}:{line_number}: row must be a JSON object'
                )
            row_id = str(row.get(id_field) or '').strip()
            if not row_id:
                raise ManualReviewError(
                    f'{path}:{line_number}: {id_field} is missing or empty'
                )
            if row_id in seen_ids:
                raise ManualReviewError(
                    f'{path}:{line_number}: duplicate {id_field}: {row_id}'
                )
            seen_ids.add(row_id)
            rows.append(row)
    if not rows:
        raise ManualReviewError(f'JSONL file is empty: {path}')
    return rows


def read_scenario_manifest(path: Path) -> list[dict[str, Any]]:
    """Read a non-empty manifest and validate its dynamic scenario-ID set."""
    return _read_jsonl_objects(path, id_field='scenario_id')


def read_captured_events(path: Path) -> list[dict[str, Any]]:
    """Read captured events without silently collapsing duplicate episodes."""
    return _read_jsonl_objects(path, id_field='episode_id')


def validate_manifest_event_ids(
    scenario_ids: list[str],
    event_ids: list[str],
) -> None:
    expected = set(scenario_ids)
    captured = set(event_ids)
    if captured != expected:
        raise ManualReviewError(
            'manifest/oracle scenario mismatch: '
            f'missing={sorted(expected - captured)}, '
            f'unexpected={sorted(captured - expected)}'
        )


def _validate_identity_list(
    value: Any,
    *,
    side: str,
    context: str,
) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise ManualReviewError(f'{context}: {side} identities must be a list')
    expected_prefix = 'L' if side == 'left' else 'R'
    identities: list[str] = []
    for raw_identity in value:
        if not isinstance(raw_identity, str) or not raw_identity.strip():
            raise ManualReviewError(
                f'{context}: {side} identity must be a non-empty string'
            )
        identity = raw_identity.strip()
        if identity not in FIXED_VISUAL_SHUTTLE_IDENTITIES:
            raise ManualReviewError(
                f'{context}: unknown shuttle identity: {identity}'
            )
        if not identity.startswith(expected_prefix):
            raise ManualReviewError(
                f'{context}: wrong-side identity {identity} in {side} list'
            )
        identities.append(identity)
    if len(set(identities)) != len(identities):
        raise ManualReviewError(
            f'{context}: duplicate identities in {side} list: {identities}'
        )
    canonical = tuple(
        identity
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
        if identity in identities
    )
    if tuple(identities) != canonical:
        raise ManualReviewError(
            f'{context}: {side} identities must use fixed global order; '
            f'got {identities}, expected {list(canonical)}'
        )
    return canonical


def _scene_identity_membership(
    scenario: dict[str, Any],
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    scenario_id = str(scenario.get('scenario_id') or '<unknown>')
    rails = scenario.get('scene', {}).get('rails')
    if not isinstance(rails, dict):
        raise ManualReviewError(f'{scenario_id}: scene rails are missing')
    memberships = []
    for side in ('left', 'right'):
        rail = rails.get(side)
        if not isinstance(rail, dict):
            raise ManualReviewError(f'{scenario_id}: {side} rail is missing')
        shuttles = rail.get('shuttles')
        if not isinstance(shuttles, list):
            raise ManualReviewError(
                f'{scenario_id}: {side} shuttle records must be a list'
            )
        raw_identities = []
        for shuttle in shuttles:
            if not isinstance(shuttle, dict):
                raise ManualReviewError(
                    f'{scenario_id}: {side} shuttle record must be an object'
                )
            raw_identities.append(shuttle.get('id'))
        memberships.append(_validate_identity_list(
            raw_identities,
            side=side,
            context=f'{scenario_id} scene',
        ))
    return memberships[0], memberships[1]


def exact_active_membership(
    scenario: dict[str, Any],
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Return exact left/right membership without count or prefix inference."""
    scenario_id = str(scenario.get('scenario_id') or '<unknown>')
    rail_scope = str(scenario.get('rail_scope') or '').strip()
    scene_left, scene_right = _scene_identity_membership(scenario)
    if rail_scope == ARBITRARY_IDENTITY_RAIL_SCOPE:
        explicit = []
        for side, scene_membership in (
            ('left', scene_left),
            ('right', scene_right),
        ):
            field = f'{side}_active_identities'
            if field in scenario:
                membership = _validate_identity_list(
                    scenario[field],
                    side=side,
                    context=f'{scenario_id} manifest',
                )
                if membership != scene_membership:
                    raise ManualReviewError(
                        f'{scenario_id}: {side} prefix substitution or exact '
                        f'membership mismatch: requested={list(membership)}, '
                        f'scene={list(scene_membership)}'
                    )
            else:
                membership = scene_membership
            explicit.append(membership)
        left, right = explicit
    elif rail_scope in LEGACY_RAIL_MEMBERSHIP:
        left, right = LEGACY_RAIL_MEMBERSHIP[rail_scope]
        if (scene_left, scene_right) != (left, right):
            raise ManualReviewError(
                f'{scenario_id}: legacy {rail_scope} membership mismatch: '
                f'scene_left={list(scene_left)}, scene_right={list(scene_right)}'
            )
    else:
        raise ManualReviewError(f'unsupported rail scope: {rail_scope!r}')
    if not left and not right:
        raise ManualReviewError(f'{scenario_id}: all-empty scene is invalid')
    target = str(
        scenario.get('relation_probe', {}).get('target_shuttle_id') or ''
    )
    if not target or target not in left + right:
        raise ManualReviewError(
            f'{scenario_id}: target identity is not active: {target!r}'
        )
    return left, right


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        suffix='.tmp',
        dir=path.parent,
    )
    try:
        with os.fdopen(file_descriptor, 'w', encoding='utf-8') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _atomic_write_json(path: Path, value: Any) -> None:
    _atomic_write_text(
        path,
        json.dumps(value, indent=2, sort_keys=True) + '\n',
    )


def _atomic_save_png(image: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.stem}.',
        suffix='.png.tmp',
        dir=path.parent,
    )
    os.close(file_descriptor)
    try:
        image.save(temporary_name, format='PNG')
        with open(temporary_name, 'rb') as stream:
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _image_status(path: Path) -> tuple[Image.Image, dict[str, Any]]:
    if not path.is_file():
        raise ManualReviewError(f'missing smoke image: {path}')
    try:
        with Image.open(path) as source:
            source.verify()
        with Image.open(path) as source:
            image = source.convert('RGB')
    except (OSError, ValueError) as exc:
        raise ManualReviewError(f'unreadable smoke image {path}: {exc}') from exc
    extrema = image.getextrema()
    blank = all((maximum - minimum) <= 1 for minimum, maximum in extrema)
    if blank:
        raise ManualReviewError(f'blank smoke image: {path}')
    return image, {
        'exists': True,
        'readable': True,
        'blank': False,
        'width': image.width,
        'height': image.height,
        'mode': image.mode,
    }


def _rail_mode(rail_scope: str) -> str:
    mapping = {
        'left_four': 'four-left',
        'right_four': 'four-right',
        'dual_four_plus_four': 'simultaneous 4+4',
        ARBITRARY_IDENTITY_RAIL_SCOPE: 'arbitrary exact identity subset',
    }
    try:
        return mapping[rail_scope]
    except KeyError as exc:
        raise ManualReviewError(f'unsupported rail scope: {rail_scope!r}') from exc


def role_mapping(scenario: dict[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    probe = scenario['relation_probe']
    target = probe['target_shuttle_id']
    left, right = exact_active_membership(scenario)
    active = set(left + right)
    roles = {
        identity: ('active_unassigned' if identity in active else 'absent')
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    annotations = {
        identity: ('active without assigned relation' if identity in active else 'absent')
        for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
    }
    roles[target] = 'target'
    relation_text = []
    for relation in probe['relations']:
        identity = relation['other_shuttle_id']
        if identity not in active:
            raise ManualReviewError(
                f'{scenario["scenario_id"]}: relation identity is not active: '
                f'{identity}'
            )
        name = relation['relation']
        role = (
            'non_blocker'
            if 'nonblocker' in name or 'non_blocker' in name
            else 'blocker'
        )
        roles[identity] = role
        annotations[identity] = name
        relation_text.append(f'{identity}:{name}')
    annotations[target] = (
        'target; ' + ', '.join(relation_text)
        if relation_text
        else 'target'
    )
    for identity in probe['relation_neutral_shuttle_ids']:
        if identity not in active:
            raise ManualReviewError(
                f'{scenario["scenario_id"]}: relation-neutral identity is '
                f'not active: {identity}'
            )
        roles[identity] = 'relation_neutral'
        annotations[identity] = 'relation-neutral; non-interfering placement'
    for identity in probe['opposite_rail_neutral_shuttle_ids']:
        if identity not in active:
            raise ManualReviewError(
                f'{scenario["scenario_id"]}: opposite-rail identity is not '
                f'active: {identity}'
            )
        roles[identity] = 'opposite_rail_neutral'
        annotations[identity] = 'opposite-rail neutral'
    unassigned = sorted(
        identity
        for identity in active
        if roles.get(identity) == 'active_unassigned'
    )
    if unassigned:
        raise ManualReviewError(
            f'{scenario["scenario_id"]}: active identities lack roles: {unassigned}'
        )
    return roles, annotations


def _font(size: int) -> ImageFont.ImageFont:
    candidates = (
        Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'),
        Path('/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf'),
    )
    for path in candidates:
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def _draw_overlay(
    source: Image.Image,
    *,
    camera: str,
    labels: list[dict[str, Any]],
    roles: dict[str, str],
    annotations: dict[str, str],
) -> Image.Image:
    overlay = source.copy()
    draw = ImageDraw.Draw(overlay, mode='RGBA')
    font = _font(13)
    small_font = _font(11)
    camera_side = camera.removesuffix('_rail_rgb')
    visible_count = 0
    for label in labels:
        if not label['presence'] or not label['visually_available']:
            continue
        if label['location']['side'] != camera_side:
            continue
        visible_count += 1
        identity = label['id']
        role = roles[identity]
        color = ROLE_COLORS[role]
        x, y, width, height = (float(value) for value in label['bbox'])
        x2 = x + width
        y2 = y + height
        draw.rectangle((x, y, x2, y2), outline=color, width=4)
        side_marker = '#13c2c2' if identity.startswith('L') else '#f759ab'
        draw.rectangle((x, y, min(x + 11, x2), min(y + 11, y2)), fill=side_marker)
        position = label['rail_position']
        lines = [
            f'{identity} | {role.replace("_", " ")} | {label["loaded_state"]}',
            f'{label["location"]["side"]}:{label["location"]["block"]}  '
            f's={position["s_m"]:.3f}m r={position["s_ratio"]:.3f}',
            annotations[identity],
        ]
        text = '\n'.join(lines)
        text_box = draw.multiline_textbbox((0, 0), text, font=small_font, spacing=2)
        text_width = text_box[2] - text_box[0]
        text_height = text_box[3] - text_box[1]
        label_x = max(0.0, min(x, overlay.width - text_width - 8))
        label_y = y - text_height - 8
        if label_y < 0:
            label_y = min(y2 + 4, overlay.height - text_height - 8)
        draw.rectangle(
            (
                label_x,
                label_y,
                label_x + text_width + 8,
                label_y + text_height + 7,
            ),
            fill=(18, 18, 18, 220),
            outline=color,
            width=2,
        )
        draw.multiline_text(
            (label_x + 4, label_y + 3),
            text,
            fill='white',
            font=small_font,
            spacing=2,
        )
    heading = (
        f'{camera} | visible fixed entries: {visible_count} | '
        'boxes are Gazebo ground truth'
    )
    heading_box = draw.textbbox((0, 0), heading, font=font)
    draw.rectangle(
        (0, 0, heading_box[2] + 14, heading_box[3] + 10),
        fill=(0, 0, 0, 205),
    )
    draw.text((7, 5), heading, fill='white', font=font)
    return overlay


def _approval_template(
    package_root: Path,
    scenario_ids: list[str],
) -> dict[str, Any]:
    scenario_template = {
        field: False
        for field in SCENARIO_REVIEW_BOOLEAN_FIELDS
    }
    scenario_template['notes'] = ''
    value: dict[str, Any] = {
        'schema_version': APPROVAL_SCHEMA_VERSION,
        'reviewer': '',
        'reviewed_at': '',
        'gallery_path': str(package_root / 'manual_inspection_gallery.html'),
        'source_dataset': str(package_root),
        'scenarios': {
            scenario_id: dict(scenario_template)
            for scenario_id in scenario_ids
        },
    }
    value.update({
        field: False
        for field in AGGREGATE_REVIEW_BOOLEAN_FIELDS
    })
    value['approved_for_full_capture'] = False
    return value


def create_approval_template(
    package_root: Path,
    scenario_ids: list[str],
) -> tuple[Path, bool]:
    current_path = package_root / CURRENT_APPROVAL_NAME
    legacy_path = package_root / LEGACY_APPROVAL_NAME
    if current_path.exists() and legacy_path.exists():
        raise ManualReviewError(
            'competing approval files are not allowed: '
            f'{current_path} and {legacy_path}'
        )
    if current_path.exists():
        return current_path, False
    if legacy_path.exists():
        return legacy_path, False
    _atomic_write_json(
        legacy_path,
        _approval_template(package_root, scenario_ids),
    )
    return legacy_path, True


def authoritative_approval_path(package_root: Path) -> Path:
    current_path = package_root / CURRENT_APPROVAL_NAME
    legacy_path = package_root / LEGACY_APPROVAL_NAME
    if current_path.exists() and legacy_path.exists():
        raise ManualReviewError(
            'competing approval files are not allowed: '
            f'{current_path} and {legacy_path}'
        )
    if current_path.exists():
        return current_path
    if legacy_path.exists():
        return legacy_path
    raise ManualReviewError('manual approval file is missing')


def _relative_from_package(path: Path, package_root: Path) -> str:
    try:
        return path.relative_to(package_root).as_posix()
    except ValueError as exc:
        raise ManualReviewError(
            f'gallery image must remain within package root: {path}'
        ) from exc


def build_gallery(package_root: Path) -> dict[str, Any]:
    package_root = package_root.expanduser().resolve()
    manifest_path = package_root / 'scenario_manifest.jsonl'
    events_path = package_root / 'dataset' / 'meta' / 'training_events.jsonl'
    dataset_root = (package_root / 'dataset').resolve()
    scenarios = read_scenario_manifest(manifest_path)
    event_rows = read_captured_events(events_path)
    scenario_ids = [scenario['scenario_id'] for scenario in scenarios]
    events = {
        str(row['episode_id']).strip(): row
        for row in event_rows
    }
    validate_manifest_event_ids(scenario_ids, list(events))

    overlays_root = package_root / 'manual_review_overlays'
    prepared_scenarios: list[dict[str, Any]] = []
    source_hashes_before: dict[str, str] = {}
    source_statuses: dict[str, dict[str, Any]] = {}
    referenced_source_paths: set[Path] = set()

    # Complete all manifest, oracle, and source-image checks before writing
    # a single overlay.
    for number, scenario in enumerate(scenarios, start=1):
        scenario_id = scenario['scenario_id']
        event = events[scenario_id]
        labels = normalize_visual_state_labels(event)
        if labels['schema_version'] != VISUAL_STATE_SCHEMA_VERSION:
            raise ManualReviewError(
                f'{scenario_id}: expected {VISUAL_STATE_SCHEMA_VERSION}, '
                f'got {labels["schema_version"]}'
            )
        identities = tuple(row['id'] for row in labels['shuttles'])
        if identities != FIXED_VISUAL_SHUTTLE_IDENTITIES:
            raise ManualReviewError(
                f'{scenario_id}: fixed identity order mismatch: {identities}'
            )
        left_active, right_active = exact_active_membership(scenario)
        active_membership = left_active + right_active
        oracle_active = tuple(
            label['id']
            for label in labels['shuttles']
            if label['presence']
        )
        if oracle_active != active_membership:
            missing = sorted(set(active_membership) - set(oracle_active))
            unexpected = sorted(set(oracle_active) - set(active_membership))
            raise ManualReviewError(
                f'{scenario_id}: manifest/oracle identity mismatch: '
                f'expected={list(active_membership)}, '
                f'oracle={list(oracle_active)}, missing={missing}, '
                f'unexpected={unexpected}'
            )
        roles, annotations = role_mapping(scenario)
        active_identities = list(active_membership)
        absent_identities = [
            identity
            for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES
            if identity not in active_membership
        ]
        scene_shuttles = {
            shuttle['id']: shuttle
            for side in ('left', 'right')
            for shuttle in scenario['scene']['rails'][side]['shuttles']
        }
        displayed_labels = []
        for label in labels['shuttles']:
            position = label['rail_position']
            scene_position = scene_shuttles.get(
                label['id'],
                {},
            ).get('start_position', {})
            displayed_labels.append({
                'identity': label['id'],
                'presence': bool(label['presence']),
                'visually_available': bool(label['visually_available']),
                'bbox': [float(value) for value in label['bbox']],
                'bbox_camera': (
                    f'{label["location"]["side"]}_rail_rgb'
                    if label['visually_available']
                    else None
                ),
                'side': label['location']['side'],
                'block': label['location']['block'],
                'loaded_state': label['loaded_state'],
                's_m': float(position['s_m']),
                's_ratio': float(position['s_ratio']),
                'segment_length_m': float(position['segment_length_m']),
                'rail_position_available': bool(position['available']),
                'position_zone': str(
                    scene_position.get('position_zone') or 'absent'
                ),
                'role': roles[label['id']],
                'relation_annotation': annotations[label['id']],
            })

        image_refs = event.get('model_input', {}).get('overhead_images', {})
        if not isinstance(image_refs, dict):
            raise ManualReviewError(
                f'{scenario_id}: overhead image references must be an object'
            )
        source_paths: dict[str, Path] = {}
        for camera in REQUIRED_CAMERAS:
            reference = str(image_refs.get(camera) or '')
            if not reference:
                raise ManualReviewError(
                    f'{scenario_id}: missing {camera} image reference'
                )
            source_path = (dataset_root / reference).resolve()
            try:
                source_path.relative_to(dataset_root)
            except ValueError as exc:
                raise ManualReviewError(
                    f'{scenario_id}: camera path escapes dataset root: '
                    f'{reference}'
                ) from exc
            if source_path in referenced_source_paths:
                raise ManualReviewError(
                    f'{scenario_id}: duplicate source-image reference: '
                    f'{source_path}'
                )
            referenced_source_paths.add(source_path)
            source_image, status = _image_status(source_path)
            source_image.close()
            source_hashes_before[str(source_path)] = _sha256(source_path)
            source_statuses[str(source_path)] = status
            source_paths[camera] = source_path
        probe = scenario['relation_probe']
        blocker_identities = [
            identity
            for identity in active_identities
            if roles[identity] == 'blocker'
        ]
        non_blocker_identities = [
            identity
            for identity in active_identities
            if roles[identity] == 'non_blocker'
        ]
        prepared_scenarios.append({
            'scenario_number': number,
            'scenario_id': scenario_id,
            'scenario_family': str(
                scenario.get('scenario_family')
                or scenario.get('scene_type')
                or ''
            ),
            'configuration_id': str(
                scenario.get('presence_configuration_id')
                or scenario.get('presence_bitmask')
                or ''
            ),
            'rail_scope': scenario['rail_scope'],
            'rail_mode': _rail_mode(scenario['rail_scope']),
            'left_active_identities': list(left_active),
            'right_active_identities': list(right_active),
            'left_count': len(left_active),
            'right_count': len(right_active),
            'target_identity': probe['target_shuttle_id'],
            'relation_family': str(
                scenario.get('relation_family')
                or scenario.get('scene_type')
                or ''
            ),
            'active_identities': active_identities,
            'absent_identities': absent_identities,
            'role_mapping': roles,
            'relation_annotations': annotations,
            'relations': probe['relations'],
            'blocker_identities': blocker_identities,
            'non_blocker_identities': non_blocker_identities,
            'relation_neutral_identities': list(
                probe['relation_neutral_shuttle_ids']
            ),
            'opposite_rail_distractor_identities': list(
                probe['opposite_rail_neutral_shuttle_ids']
            ),
            'oracle_record_status': {
                'present': True,
                'schema_version': labels['schema_version'],
                'fixed_identity_order_valid': True,
                'exact_membership_valid': True,
            },
            'labels': displayed_labels,
            '_source_paths': source_paths,
            '_normalized_labels': labels['shuttles'],
        })

    expected_source_count = len(scenarios) * len(REQUIRED_CAMERAS)
    if len(referenced_source_paths) != expected_source_count:
        raise ManualReviewError(
            f'expected {expected_source_count} unique source images, got '
            f'{len(referenced_source_paths)}'
        )

    staging_root = Path(tempfile.mkdtemp(
        prefix='.manual_review_overlays.',
        dir=package_root,
    ))
    gallery_scenarios: list[dict[str, Any]] = []
    overlay_paths: list[Path] = []
    try:
        for prepared in prepared_scenarios:
            scenario_id = prepared['scenario_id']
            camera_entries = {}
            for camera in REQUIRED_CAMERAS:
                source_path = prepared['_source_paths'][camera]
                source_image, unused_status = _image_status(source_path)
                staging_path = (
                    staging_root / scenario_id / f'{camera}_overlay.png'
                )
                overlay = _draw_overlay(
                    source_image,
                    camera=camera,
                    labels=prepared['_normalized_labels'],
                    roles=prepared['role_mapping'],
                    annotations=prepared['relation_annotations'],
                )
                source_image.close()
                _atomic_save_png(overlay, staging_path)
                overlay.close()
                final_overlay_path = (
                    overlays_root / scenario_id / f'{camera}_overlay.png'
                )
                overlay_paths.append(final_overlay_path)
                source_digest = source_hashes_before[str(source_path)]
                camera_entries[camera] = {
                    'source_image_path': str(source_path),
                    'source_image_relative_path': _relative_from_package(
                        source_path,
                        package_root,
                    ),
                    'overlay_path': str(final_overlay_path),
                    'overlay_relative_path': _relative_from_package(
                        final_overlay_path,
                        package_root,
                    ),
                    'source_sha256_before': source_digest,
                    'source_sha256_after': source_digest,
                    'source_modified': False,
                    'overlay_sha256': _sha256(staging_path),
                    'image_status': source_statuses[str(source_path)],
                }
            prepared.pop('_source_paths')
            prepared.pop('_normalized_labels')
            prepared['cameras'] = camera_entries
            gallery_scenarios.append(prepared)

        source_hashes_after = {
            path: _sha256(Path(path))
            for path in source_hashes_before
        }
        modified_sources = sorted(
            path
            for path, digest in source_hashes_before.items()
            if source_hashes_after[path] != digest
        )
        if modified_sources:
            raise ManualReviewError(
                f'source images were modified: {modified_sources}'
            )
        if overlays_root.exists():
            shutil.rmtree(overlays_root)
        os.replace(staging_root, overlays_root)
    except BaseException:
        shutil.rmtree(staging_root, ignore_errors=True)
        raise

    actual_overlay_paths = sorted(
        path
        for path in overlays_root.rglob('*')
        if path.is_file()
    )
    if set(actual_overlay_paths) != set(overlay_paths):
        raise ManualReviewError(
            'overlay output set mismatch: '
            f'expected={len(overlay_paths)}, actual={len(actual_overlay_paths)}'
        )
    source_hashes_after = {
        path: _sha256(Path(path))
        for path in source_hashes_before
    }
    modified_sources = sorted(
        path
        for path, digest in source_hashes_before.items()
        if source_hashes_after[path] != digest
    )
    if modified_sources:
        raise ManualReviewError(
            f'source images were modified: {modified_sources}'
        )
    active_count = Counter(
        len(scenario['active_identities'])
        for scenario in gallery_scenarios
    )
    cardinality = Counter(
        f'{scenario["left_count"]}+{scenario["right_count"]}'
        for scenario in gallery_scenarios
    )
    rail_scopes = Counter(
        scenario['rail_scope']
        for scenario in gallery_scenarios
    )
    relation_families = Counter(
        scenario['relation_family']
        for scenario in gallery_scenarios
    )
    per_identity = {}
    for identity in FIXED_VISUAL_SHUTTLE_IDENTITIES:
        present_labels = [
            label
            for scenario in gallery_scenarios
            for label in scenario['labels']
            if label['identity'] == identity and label['presence']
        ]
        per_identity[identity] = {
            'present': len(present_labels),
            'absent': len(gallery_scenarios) - len(present_labels),
            'loaded': sum(
                label['loaded_state'] == 'loaded'
                for label in present_labels
            ),
            'empty': sum(
                label['loaded_state'] == 'empty'
                for label in present_labels
            ),
        }
    fingerprint_lines = ''.join(
        f'{source_hashes_after[path]}  '
        f'{Path(path).relative_to(dataset_root).as_posix()}\n'
        for path in sorted(source_hashes_after)
    )
    return {
        'schema_version': GALLERY_SCHEMA_VERSION,
        'source_package': str(package_root),
        'source_manifest': str(manifest_path),
        'source_training_events': str(events_path),
        'fixed_identity_order': list(FIXED_VISUAL_SHUTTLE_IDENTITIES),
        'visual_state_schema': VISUAL_STATE_SCHEMA_VERSION,
        'scenario_count': len(gallery_scenarios),
        'image_count': len(source_hashes_after),
        'source_image_count': len(source_hashes_after),
        'overlay_image_count': len(actual_overlay_paths),
        'required_cameras': list(REQUIRED_CAMERAS),
        'source_images_unchanged': True,
        'modified_source_images': [],
        'source_image_tree_sha256': hashlib.sha256(
            fingerprint_lines.encode('utf-8')
        ).hexdigest(),
        'source_image_fingerprints': {
            Path(path).relative_to(dataset_root).as_posix(): digest
            for path, digest in sorted(source_hashes_after.items())
        },
        'exact_subset_validation': True,
        'summary': {
            'total_scenarios': len(gallery_scenarios),
            'total_source_images': len(source_hashes_after),
            'total_overlay_images': len(actual_overlay_paths),
            'scenario_count_by_total_active': dict(sorted(
                active_count.items()
            )),
            'scenario_count_by_left_right_cardinality': dict(sorted(
                cardinality.items()
            )),
            'per_identity': per_identity,
            'rail_scope_counts': dict(sorted(rail_scopes.items())),
            'relation_family_counts': dict(sorted(
                relation_families.items()
            )),
            'exact_subset_validation': True,
        },
        'scenarios': gallery_scenarios,
    }


def _badge(text: str, css_class: str = '') -> str:
    return (
        f'<span class="badge {html.escape(css_class)}">'
        f'{html.escape(text)}</span>'
    )


def _gallery_html(gallery: dict[str, Any]) -> str:
    summary = gallery['summary']

    def counter_table(title: str, values: dict[str, int]) -> str:
        rows = ''.join(
            '<tr>'
            f'<td>{html.escape(str(key))}</td>'
            f'<td>{int(value)}</td>'
            '</tr>'
            for key, value in values.items()
        )
        return (
            '<section class="summary-card">'
            f'<h3>{html.escape(title)}</h3>'
            '<table><thead><tr><th>Value</th><th>Scenarios</th></tr></thead>'
            f'<tbody>{rows}</tbody></table></section>'
        )

    identity_rows = ''.join(
        '<tr>'
        f'<td>{html.escape(identity)}</td>'
        f'<td>{counts["present"]}</td>'
        f'<td>{counts["absent"]}</td>'
        f'<td>{counts["loaded"]}</td>'
        f'<td>{counts["empty"]}</td>'
        '</tr>'
        for identity, counts in summary['per_identity'].items()
    )
    summary_html = (
        '<div class="summary-grid">'
        + counter_table(
            'Total active shuttles',
            summary['scenario_count_by_total_active'],
        )
        + counter_table(
            'Left + right cardinality',
            summary['scenario_count_by_left_right_cardinality'],
        )
        + counter_table('Rail scopes', summary['rail_scope_counts'])
        + counter_table(
            'Relation families',
            summary['relation_family_counts'],
        )
        + '<section class="summary-card identity-summary">'
        '<h3>Identity presence and payload</h3>'
        '<table><thead><tr><th>ID</th><th>Present</th><th>Absent</th>'
        '<th>Loaded</th><th>Empty</th></tr></thead>'
        f'<tbody>{identity_rows}</tbody></table></section>'
        '</div>'
    )
    cards = []
    for scenario in gallery['scenarios']:
        label_rows = []
        for label in scenario['labels']:
            role = label['role']
            state = 'present' if label['presence'] else 'absent'
            visible = 'visible' if label['visually_available'] else 'not visible'
            bbox_text = ', '.join(f'{value:.2f}' for value in label['bbox'])
            label_rows.append(
                '<tr>'
                f'<td class="identity {label["side"]}">{html.escape(label["identity"])}</td>'
                f'<td>{_badge(state, state)} {_badge(visible, visible.replace(" ", "-"))}</td>'
                f'<td>{_badge(role.replace("_", " "), role)}</td>'
                f'<td>{html.escape(label["loaded_state"])}</td>'
                f'<td>{html.escape(label["side"])}</td>'
                f'<td>{html.escape(label["block"])}</td>'
                f'<td>{label["s_m"]:.6f}</td>'
                f'<td>{label["s_ratio"]:.6f}</td>'
                f'<td>{label["segment_length_m"]:.6f}</td>'
                f'<td>{html.escape(label["position_zone"])}</td>'
                f'<td>{html.escape(bbox_text)}</td>'
                f'<td>{html.escape(label["relation_annotation"])}</td>'
                '</tr>'
            )
        camera_cards = []
        for camera in REQUIRED_CAMERAS:
            value = scenario['cameras'][camera]
            source = html.escape(value['source_image_relative_path'], quote=True)
            overlay = html.escape(value['overlay_relative_path'], quote=True)
            status = value['image_status']
            camera_cards.append(
                '<section class="camera-card">'
                f'<h4>{html.escape(camera)}</h4>'
                '<div class="view-pair">'
                '<figure>'
                f'<a href="{source}"><img loading="lazy" src="{source}" '
                f'alt="{html.escape(camera)} original"></a>'
                '<figcaption>Original capture</figcaption>'
                '</figure>'
                '<figure>'
                f'<a href="{overlay}"><img loading="lazy" src="{overlay}" '
                f'alt="{html.escape(camera)} overlay"></a>'
                '<figcaption>Ground-truth overlay</figcaption>'
                '</figure>'
                '</div>'
                f'<p><strong>Source:</strong> <code>{html.escape(value["source_image_path"])}</code></p>'
                f'<p>Readable: <strong>{status["readable"]}</strong>; '
                f'blank: <strong>{status["blank"]}</strong>; '
                f'{status["width"]}×{status["height"]}; '
                f'SHA-256: <code>{value["source_sha256_after"]}</code></p>'
                '</section>'
            )
        searchable = ' '.join([
            scenario['scenario_id'],
            scenario['configuration_id'],
            scenario['rail_scope'],
            scenario['relation_family'],
            *scenario['active_identities'],
        ]).casefold()
        cards.append(
            f'<article class="scenario" id="{html.escape(scenario["scenario_id"])}" '
            f'data-search="{html.escape(searchable, quote=True)}">'
            '<header>'
            f'<h2>#{scenario["scenario_number"]:02d} '
            f'{html.escape(scenario["scenario_id"])}</h2>'
            '<div class="summary">'
            f'{_badge(scenario["scenario_family"], "family")}'
            f'{_badge(scenario["rail_mode"], "rail-mode")}'
            f'{_badge(scenario["relation_family"], "family")}'
            f'{_badge("target " + scenario["target_identity"], "target")}'
            '</div>'
            f'<p><strong>Configuration:</strong> '
            f'{html.escape(scenario["configuration_id"] or "not provided")} · '
            f'<strong>Rail scope:</strong> {html.escape(scenario["rail_scope"])}</p>'
            f'<p><strong>Left active:</strong> '
            f'{html.escape(", ".join(scenario["left_active_identities"]) or "empty")} · '
            f'<strong>Right active:</strong> '
            f'{html.escape(", ".join(scenario["right_active_identities"]) or "empty")}</p>'
            f'<p><strong>Active:</strong> {html.escape(", ".join(scenario["active_identities"]))}'
            f' · <strong>Absent:</strong> '
            f'{html.escape(", ".join(scenario["absent_identities"]) or "none")}</p>'
            f'<p><strong>Blockers:</strong> '
            f'{html.escape(", ".join(scenario["blocker_identities"]) or "none")} · '
            f'<strong>Non-blockers:</strong> '
            f'{html.escape(", ".join(scenario["non_blocker_identities"]) or "none")} · '
            f'<strong>Relation-neutral:</strong> '
            f'{html.escape(", ".join(scenario["relation_neutral_identities"]) or "none")} · '
            f'<strong>Opposite-rail distractors:</strong> '
            f'{html.escape(", ".join(scenario["opposite_rail_distractor_identities"]) or "none")}</p>'
            f'<p><strong>Oracle:</strong> present={scenario["oracle_record_status"]["present"]}, '
            f'schema={html.escape(scenario["oracle_record_status"]["schema_version"])}, '
            'fixed order valid=true, exact membership valid=true</p>'
            '</header>'
            + ''.join(camera_cards)
            + '<div class="table-wrap"><table>'
            '<thead><tr><th>ID</th><th>Presence / visibility</th><th>Role</th>'
            '<th>Payload</th><th>Side</th><th>Block</th><th>s_m</th>'
            '<th>s_ratio</th><th>Length m</th><th>Position zone</th>'
            '<th>BBox x,y,w,h</th>'
            '<th>Relation annotation</th></tr></thead>'
            '<tbody>'
            + ''.join(label_rows)
            + '</tbody></table></div>'
            '</article>'
        )
    legend = ''.join(
        _badge(name.replace('_', ' '), name)
        for name in (
            'target',
            'blocker',
            'non_blocker',
            'relation_neutral',
            'opposite_rail_neutral',
            'absent',
        )
    )
    return f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Room 315 exact-identity smoke manual review</title>
<style>
:root {{ color-scheme: dark; font-family: Inter, system-ui, sans-serif; }}
body {{ margin: 0; background: #101216; color: #eef1f5; }}
main {{ max-width: 1880px; margin: auto; padding: 24px; }}
h1 {{ margin-bottom: 8px; }}
.notice {{ background: #30270b; border: 1px solid #ffd43b; padding: 14px; border-radius: 8px; }}
.filter {{ width: min(780px, 100%); box-sizing: border-box; padding: 12px; margin: 12px 0;
  background: #1a1e24; color: #fff; border: 1px solid #596273; border-radius: 8px; }}
.summary-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
  gap: 14px; margin: 20px 0; }}
.summary-card {{ background: #1a1e24; border: 1px solid #343a46; border-radius: 10px; padding: 12px; }}
.identity-summary {{ grid-column: 1 / -1; }}
.scenario {{ background: #1a1e24; border: 1px solid #343a46; border-radius: 12px; margin: 28px 0; padding: 18px; }}
.scenario header {{ border-bottom: 1px solid #343a46; margin-bottom: 16px; }}
.summary, .legend {{ display: flex; flex-wrap: wrap; gap: 8px; margin: 8px 0; }}
.badge {{ display: inline-block; border: 1px solid #666; border-radius: 999px; padding: 3px 9px; margin: 2px; background: #343a40; }}
.target {{ background: #765d00; border-color: {ROLE_COLORS["target"]}; }}
.blocker {{ background: #721c24; border-color: {ROLE_COLORS["blocker"]}; }}
.non_blocker {{ background: #064b76; border-color: {ROLE_COLORS["non_blocker"]}; }}
.relation_neutral {{ background: #1f5d2e; border-color: {ROLE_COLORS["relation_neutral"]}; }}
.opposite_rail_neutral {{ background: #4a2670; border-color: {ROLE_COLORS["opposite_rail_neutral"]}; }}
.absent, .not-visible {{ background: #353535; color: #bbb; }}
.present, .visible {{ background: #174d2a; }}
.family {{ border-color: #fa8c16; }}
.rail-mode {{ border-color: #13c2c2; }}
.camera-card {{ margin: 18px 0; }}
.view-pair {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 14px; }}
figure {{ margin: 0; }}
img {{ display: block; width: 100%; height: auto; border: 1px solid #48505d; background: #000; }}
figcaption {{ text-align: center; padding: 6px; color: #c5cad3; }}
code {{ overflow-wrap: anywhere; color: #afd7ff; }}
.table-wrap {{ overflow-x: auto; }}
table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
th, td {{ border: 1px solid #3b424e; padding: 7px; text-align: left; white-space: nowrap; }}
th {{ background: #252a32; position: sticky; top: 0; }}
.identity {{ font-weight: 800; font-size: 15px; }}
.identity.left {{ color: #36cfc9; }}
.identity.right {{ color: #ff85c0; }}
@media (max-width: 900px) {{ .view-pair {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body><main>
<h1>Room 315 exact-identity smoke manual review</h1>
<p class="notice"><strong>Human approval required.</strong> This gallery is an
inspection aid. Generation does not approve any image or scenario.</p>
<p>Scenarios: {gallery["scenario_count"]}; source images:
{gallery["source_image_count"]}; overlays: {gallery["overlay_image_count"]};
schema: <code>{html.escape(gallery["visual_state_schema"])}</code>;
source images unchanged: <strong>{gallery["source_images_unchanged"]}</strong>;
exact subset validation: <strong>{gallery["exact_subset_validation"]}</strong>.</p>
<label for="scenario-filter"><strong>Find a scenario, configuration, relation,
or exact subset:</strong></label><br>
<input class="filter" id="scenario-filter" type="search"
placeholder="Examples: L3, L2 L4, R1 R3, presence_010, multi_blocker">
{summary_html}
<div class="legend">{legend}</div>
{''.join(cards)}
<script>
const input = document.getElementById('scenario-filter');
const cards = Array.from(document.querySelectorAll('.scenario'));
input.addEventListener('input', () => {{
  const terms = input.value.toLowerCase().trim().split(/\\s+/).filter(Boolean);
  cards.forEach(card => {{
    const haystack = card.dataset.search || '';
    card.hidden = !terms.every(term => haystack.includes(term));
  }});
}});
</script>
</main></body></html>
'''


def write_gallery(package_root: Path) -> dict[str, Any]:
    package_root = package_root.expanduser().resolve()
    gallery = build_gallery(package_root)
    manifest_path = package_root / 'manual_inspection_gallery_manifest.json'
    gallery_path = package_root / 'manual_inspection_gallery.html'
    _atomic_write_json(manifest_path, gallery)
    _atomic_write_text(gallery_path, _gallery_html(gallery))
    approval_path, approval_created = create_approval_template(
        package_root,
        [row['scenario_id'] for row in gallery['scenarios']],
    )
    return {
        'gallery_path': str(gallery_path),
        'gallery_manifest_path': str(manifest_path),
        'approval_path': str(approval_path),
        'approval_created': approval_created,
        'scenario_count': gallery['scenario_count'],
        'image_count': gallery['image_count'],
        'source_image_count': gallery['source_image_count'],
        'overlay_image_count': gallery['overlay_image_count'],
        'exact_subset_validation': gallery['exact_subset_validation'],
        'source_images_unchanged': gallery['source_images_unchanged'],
    }


def validate_manual_approval(
    package_root: Path,
    approval_path: Path | None = None,
) -> dict[str, Any]:
    package_root = package_root.expanduser().resolve()
    manifest_scenarios = read_scenario_manifest(
        package_root / 'scenario_manifest.jsonl'
    )
    expected_ids = [row['scenario_id'] for row in manifest_scenarios]
    approval_path = (
        approval_path.expanduser().resolve()
        if approval_path is not None
        else authoritative_approval_path(package_root)
    )
    if not approval_path.is_file():
        raise ManualReviewError('manual approval file is missing')
    try:
        approval = json.loads(approval_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManualReviewError('manual approval file is unreadable') from exc
    if not isinstance(approval, dict):
        raise ManualReviewError('manual approval must be an object')
    if approval.get('schema_version') == CURRENT_APPROVAL_SCHEMA_VERSION:
        gallery_path = package_root / 'manual_inspection_gallery_manifest.json'
        if not gallery_path.is_file():
            raise ManualReviewError('manual gallery manifest is missing')
        try:
            gallery = json.loads(gallery_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as exc:
            raise ManualReviewError(
                'manual gallery manifest is unreadable'
            ) from exc
        if not isinstance(gallery, dict):
            raise ManualReviewError('manual gallery manifest must be an object')
        gallery_rows = gallery.get('scenarios')
        if not isinstance(gallery_rows, list):
            raise ManualReviewError('manual gallery scenarios are missing')
        gallery_ids = []
        for row in gallery_rows:
            if not isinstance(row, dict):
                raise ManualReviewError(
                    'manual gallery scenario must be an object'
                )
            scenario_id = str(row.get('scenario_id') or '').strip()
            if not scenario_id:
                raise ManualReviewError(
                    'manual gallery scenario ID is missing'
                )
            gallery_ids.append(scenario_id)
        if len(gallery_ids) != len(set(gallery_ids)):
            raise ManualReviewError(
                'duplicate scenario IDs in manual gallery'
            )
        if set(gallery_ids) != set(expected_ids):
            raise ManualReviewError(
                'manual gallery scenario IDs do not match manifest: '
                f'missing={sorted(set(expected_ids) - set(gallery_ids))}, '
                f'unexpected={sorted(set(gallery_ids) - set(expected_ids))}'
            )
        if gallery.get('source_images_unchanged') is not True:
            raise ManualReviewError(
                'manual gallery source-image integrity is not valid'
            )
        if gallery.get('exact_subset_validation') is not True:
            raise ManualReviewError(
                'manual gallery exact-subset validation is not valid'
            )
        if approval.get('approved_after_gallery_review') is not True:
            raise ManualReviewError(
                'approved_after_gallery_review is not true'
            )
        if approval.get('approved_for_training') is not False:
            raise ManualReviewError(
                'approved_for_training must remain false'
            )
        return {
            'valid': True,
            'approval_path': str(approval_path),
            'approval_sha256': _sha256(approval_path),
            'scenario_count': len(expected_ids),
            'approved_after_gallery_review': True,
            'approved_for_training': False,
        }
    if approval.get('schema_version') != APPROVAL_SCHEMA_VERSION:
        raise ManualReviewError('manual approval schema is invalid')
    if str(approval.get('reviewer') or '').strip() == '':
        raise ManualReviewError('manual reviewer is required')
    if str(approval.get('reviewed_at') or '').strip() == '':
        raise ManualReviewError('manual reviewed_at is required')
    if approval.get('gallery_path') != str(
        package_root / 'manual_inspection_gallery.html'
    ):
        raise ManualReviewError('manual approval gallery path is invalid')
    if approval.get('source_dataset') != str(package_root):
        raise ManualReviewError('manual approval source dataset is invalid')
    scenario_reviews = approval.get('scenarios')
    if not isinstance(scenario_reviews, dict):
        raise ManualReviewError('manual scenario reviews are missing')
    if set(scenario_reviews) != set(expected_ids):
        raise ManualReviewError('manual scenario review IDs are incomplete')
    for scenario_id in expected_ids:
        review = scenario_reviews[scenario_id]
        if not isinstance(review, dict):
            raise ManualReviewError(f'{scenario_id}: review must be an object')
        for field in SCENARIO_REVIEW_BOOLEAN_FIELDS:
            if review.get(field) is not True:
                raise ManualReviewError(f'{scenario_id}: {field} is not true')
        if not isinstance(review.get('notes'), str):
            raise ManualReviewError(f'{scenario_id}: notes must be text')
    for field in AGGREGATE_REVIEW_BOOLEAN_FIELDS:
        legacy_alias = LEGACY_AGGREGATE_REVIEW_ALIASES.get(field)
        if approval.get(field) is not True and (
            legacy_alias is None or approval.get(legacy_alias) is not True
        ):
            raise ManualReviewError(f'{field} is not true')
    if not isinstance(approval.get('approved_for_full_capture'), bool):
        raise ManualReviewError(
            'approved_for_full_capture must be an explicit boolean'
        )
    return {
        'valid': True,
        'approval_path': str(approval_path),
        'approval_sha256': _sha256(approval_path),
        'scenario_count': len(expected_ids),
        'approved_for_full_manifest_generation': True,
        'approved_for_full_capture': approval['approved_for_full_capture'],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    gallery = subparsers.add_parser('gallery')
    gallery.add_argument('--package-root', type=Path, required=True)
    validate = subparsers.add_parser('validate-approval')
    validate.add_argument('--package-root', type=Path, required=True)
    validate.add_argument('--approval', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == 'gallery':
        result = write_gallery(args.package_root)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    try:
        result = validate_manual_approval(args.package_root, args.approval)
    except (ManualReviewError, OSError, ValueError):
        print(WAITING_MESSAGE)
        return 2
    print('MANUAL_APPROVAL_VALID')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ManualReviewError, OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        raise SystemExit(1) from None
