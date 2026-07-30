#!/usr/bin/env python3

import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_arbitrary_subset_capture_v2 as capture


PACKAGE_ROOT = Path(
    '/home/tiago/'
    'room315_arbitrary_subset_visual_2040_capture_v2_seed31520260729'
)


@pytest.fixture(scope='module')
def v2_rows():
    return capture.read_jsonl(capture.V2_PLAN)


@pytest.fixture(scope='module')
def rows():
    return capture.read_jsonl(PACKAGE_ROOT / 'scenario_manifest.jsonl')


def _object(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def _file_map(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        for path in sorted(root.rglob('*'))
        if path.is_file()
    }


def _copy_package(tmp_path: Path) -> Path:
    target = tmp_path / 'capture_package'
    # The package is large. Hard-link immutable payload files, then detach the
    # small set this test intentionally mutates so the authoritative source can
    # never be changed through a test copy.
    shutil.copytree(PACKAGE_ROOT, target, copy_function=os.link)
    for relative in (
        'production_capture_approval.json',
        'capture_state.json',
        'canary_gallery.html',
        'production_review_gallery.html',
        'scenario_manifest.jsonl',
    ):
        destination = target / relative
        detached = destination.with_name(f'.{destination.name}.detached')
        shutil.copy2(destination, detached)
        os.replace(detached, destination)
    return target


def test_executable_manifest_is_exact_v2_semantics(v2_rows, rows):
    assert len(v2_rows) == len(rows) == capture.EXPECTED_SCENARIOS
    assert len({row['scenario_id'] for row in rows}) == 2040
    assert all(
        not capture._semantic_equality_errors(source, executable)
        for source, executable in zip(v2_rows, rows)
    )
    assert all(
        row['left_active_identities']
        == [
            shuttle['id']
            for shuttle in row['scene']['rails']['left']['shuttles']
        ]
        and row['right_active_identities']
        == [
            shuttle['id']
            for shuttle in row['scene']['rails']['right']['shuttles']
        ]
        for row in rows
    )


def test_static_audits_pass_without_prefix_substitution():
    audit = _object(PACKAGE_ROOT / 'production_manifest_audit.json')

    assert audit['passed']
    assert all(audit['checks'].values())
    assert audit['checks']['row_by_row_v2_semantic_equality']
    assert not audit['violations']['semantic_equality']
    assert not audit['violations']['full_validation']
    assert audit['fixed_schema'] == {
        'dataset_inferred_capacity': False,
        'identity_order': list(capture.GLOBAL_IDENTITIES),
        'vectorizer_dimension': 200,
        'visual_state_schema': 'room315.visual_state.v3',
    }


def test_canary_is_exact_manifest_subset_with_complete_coverage(rows):
    canary = _object(
        PACKAGE_ROOT / 'production_canary_scenario_ids.json'
    )
    audit = _object(PACKAGE_ROOT / 'production_canary_audit.json')
    scenario_ids = canary['scenario_ids']

    assert len(scenario_ids) == len(set(scenario_ids)) == 64
    assert set(scenario_ids) < {row['scenario_id'] for row in rows}
    assert audit['passed']
    assert audit['required_token_count'] == 118
    assert audit['covered_token_count'] == 118
    assert not audit['missing_required_tokens']


def test_current_full_capture_state_gallery_approved_training_closed():
    approval = _object(PACKAGE_ROOT / 'production_capture_approval.json')
    state = _object(PACKAGE_ROOT / 'capture_state.json')
    status = capture.status_report(PACKAGE_ROOT)

    assert [
        approval[field] for field in capture.APPROVAL_FIELDS
    ] == [True, True, True, True, False]
    assert state['capture_has_started'] is True
    assert state['capture_complete'] is True
    assert len(state['completed_scenarios']) == capture.EXPECTED_SCENARIOS
    assert status['completed_scenarios'] == capture.EXPECTED_SCENARIOS
    assert status['remaining_scenarios'] == 0
    assert status['valid_images'] == capture.EXPECTED_SCENARIOS * 2
    assert status['missing_images'] == 0
    assert status['canary_completed'] == capture.CANARY_COUNT
    assert status['canary_remaining'] == 0
    assert status['capture_has_started'] is True


def test_package_manifest_and_manifest_gate_pass():
    verification = capture.verify_package(PACKAGE_ROOT)
    valid, code, message = capture.approval_gate(
        PACKAGE_ROOT,
        'manifest',
    )

    assert verification['passed']
    assert verification['verified_static_file_count'] == 20
    assert valid and code == 0 and message == 'MANIFEST_GATE_VALID'


def test_approval_transitions_are_explicit_and_fingerprint_pinned(
    tmp_path,
):
    root = _copy_package(tmp_path)
    approval_path = root / 'production_capture_approval.json'
    approval = _object(approval_path)
    approval.update({
        field: False for field in capture.APPROVAL_FIELDS
    })
    for field in (
        *capture.CANARY_REVIEW_FINGERPRINT_FILES,
        capture.CANARY_REVIEW_OVERLAY_FINGERPRINT_FIELD,
    ):
        approval.pop(field, None)
    approval_path.write_text(
        json.dumps(approval, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    state_path = root / 'capture_state.json'
    state = _object(state_path)
    state['unresolved_failures'] = []
    state_path.write_text(
        json.dumps(state, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    valid, code, _ = capture.approval_gate(root, 'canary')
    assert not valid and code == capture.GATE_EXIT_CODES['canary']

    approval['approved_for_canary_capture'] = True
    approval_path.write_text(
        json.dumps(approval, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    valid, code, _ = capture.approval_gate(root, 'canary-gallery')
    assert not valid and code == capture.GATE_EXIT_CODES['canary-gallery']

    with pytest.raises(
        capture.CapturePackageError,
        match='reviewer and notes',
    ):
        capture.approve_canary_gallery(
            root,
            reviewer='',
            notes='',
        )

    capture.approve_canary_gallery(
        root,
        reviewer='Approval Test',
        notes='Reviewed corrected canary artifacts',
    )
    approval = _object(approval_path)
    assert approval['approved_after_canary_gallery_review'] is True
    assert approval['canary_gallery_reviewer'] == 'Approval Test'
    assert approval['canary_gallery_reviewed_at']
    assert approval['approved_for_training'] is False
    assert all(
        approval.get(field)
        for field in (
            *capture.CANARY_REVIEW_FINGERPRINT_FILES,
            capture.CANARY_REVIEW_OVERLAY_FINGERPRINT_FIELD,
        )
    )
    valid, code, message = capture.approval_gate(
        root,
        'canary-gallery',
    )
    assert valid and code == 0 and message == 'CANARY_GALLERY_GATE_VALID'

    original_html = (root / 'canary_gallery.html').read_bytes()
    (root / 'canary_gallery.html').write_bytes(original_html + b'\n')
    valid, code, message = capture.approval_gate(
        root,
        'canary-gallery',
    )
    assert not valid
    assert code == capture.GATE_EXIT_CODES['fingerprint']
    assert message == 'CANARY_REVIEW_FINGERPRINT_MISMATCH'
    (root / 'canary_gallery.html').write_bytes(original_html)

    valid, code, _ = capture.approval_gate(root, 'full-capture')
    assert not valid and code == capture.GATE_EXIT_CODES['full-capture']

    with pytest.raises(
        capture.CapturePackageError,
        match='reviewer and notes',
    ):
        capture.enable_full_capture_approval(
            root,
            reviewer='',
            notes='',
        )

    capture.enable_full_capture_approval(
        root,
        reviewer='Approval Test',
        notes='Authorized remaining production capture',
    )
    approval = _object(approval_path)
    assert approval['approved_for_full_capture'] is True
    assert approval['full_capture_reviewer'] == 'Approval Test'
    assert approval['full_capture_reviewed_at']
    assert approval['approved_after_full_gallery_review'] is False
    assert approval['approved_for_training'] is False
    valid, code, message = capture.approval_gate(root, 'full-capture')
    assert valid and code == 0 and message == 'FULL_CAPTURE_GATE_VALID'

    state = _object(state_path)
    state['unresolved_failures'] = [{
        'scenario_id': 'retryable_full_capture_scenario',
        'returncode': 1,
    }]
    state_path.write_text(
        json.dumps(state, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    valid, code, message = capture.approval_gate(root, 'full-capture')
    assert valid and code == 0 and message == 'FULL_CAPTURE_GATE_VALID'

    # This transition test models the point immediately after full-capture
    # authorization, before a production audit exists.  The source package may
    # itself already be fully captured.
    full_audit_path = root / 'captured_production_audit.json'
    original_full_audit = full_audit_path.read_bytes()
    full_audit_path.unlink()
    valid, code, _ = capture.approval_gate(root, 'full-gallery')
    assert (
        not valid
        and code == capture.GATE_EXIT_CODES['full_capture_incomplete']
    )
    full_audit_path.write_bytes(original_full_audit)
    state['unresolved_failures'] = []
    state_path.write_text(
        json.dumps(state, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    with pytest.raises(
        capture.CapturePackageError,
        match='reviewer and notes',
    ):
        capture.approve_full_gallery(
            root,
            reviewer='',
            notes='',
        )

    capture.approve_full_gallery(
        root,
        reviewer='Approval Test',
        notes='Reviewed final production gallery',
    )
    approval = _object(approval_path)
    assert approval['approved_after_full_gallery_review'] is True
    assert approval['full_gallery_reviewer'] == 'Approval Test'
    assert approval['full_gallery_reviewed_at']
    assert approval['approved_for_training'] is False
    assert all(
        approval.get(field)
        for field in (
            *capture.FULL_REVIEW_FINGERPRINT_FILES,
            capture.FULL_REVIEW_OVERLAY_FINGERPRINT_FIELD,
        )
    )
    valid, code, message = capture.approval_gate(root, 'full-gallery')
    assert valid and code == 0 and message == 'FULL_GALLERY_GATE_VALID'
    valid, code, message = capture.approval_gate(root, 'training')
    assert not valid
    assert code == capture.GATE_EXIT_CODES['training']
    assert message == 'TRAINING_APPROVAL_REQUIRED'

    gallery_html = root / 'production_review_gallery.html'
    original_gallery_html = gallery_html.read_bytes()
    gallery_html.write_bytes(original_gallery_html + b'\n')
    valid, code, message = capture.approval_gate(root, 'full-gallery')
    assert not valid
    assert code == capture.GATE_EXIT_CODES['fingerprint']
    assert message == 'FULL_REVIEW_FINGERPRINT_MISMATCH'
    gallery_html.write_bytes(original_gallery_html)
    valid, code, _ = capture.approval_gate(root, 'training')
    assert (
        not valid
        and code == capture.GATE_EXIT_CODES['training']
    )


def test_manifest_hash_mismatch_blocks_capture(tmp_path):
    root = _copy_package(tmp_path)
    with (root / 'scenario_manifest.jsonl').open(
        'a',
        encoding='utf-8',
    ) as stream:
        stream.write('\n')

    valid, code, message = capture.approval_gate(root, 'canary')

    assert not valid
    assert code == capture.GATE_EXIT_CODES['fingerprint']
    assert message == 'MANIFEST_GATE_FAILED'


def test_canary_episodes_are_reused_by_full_resume(
    tmp_path,
    monkeypatch,
    rows,
):
    canary_ids = set(capture._canary_ids(PACKAGE_ROOT))
    root = tmp_path / 'canary_only_capture'

    monkeypatch.setattr(
        capture,
        '_episode_validation',
        lambda _root, row: {
            'valid': row['scenario_id'] in canary_ids,
        },
    )
    monkeypatch.setattr(capture, '_manifest_rows', lambda _root: rows)
    pending = capture._pending_ids(root, 'full')

    assert len(pending) == 2040 - 64
    assert not (canary_ids & set(pending))
    assert set(pending) == {
        row['scenario_id']
        for row in rows
        if row['scenario_id'] not in canary_ids
    }


def test_invalid_existing_episode_is_never_overwritten(
    tmp_path,
    monkeypatch,
    rows,
):
    root = tmp_path / 'minimal'
    episode = (
        root
        / 'dataset'
        / 'episodes'
        / rows[0]['scenario_id']
    )
    episode.mkdir(parents=True)
    monkeypatch.setattr(capture, '_manifest_rows', lambda _root: rows[:1])
    monkeypatch.setattr(
        capture,
        '_episode_validation',
        lambda _root, _row: {'valid': False, 'error': 'incomplete'},
    )

    with pytest.raises(
        capture.CapturePackageError,
        match='will not be overwritten',
    ):
        capture._pending_ids(root, 'full')


def test_duplicate_aggregate_rows_are_detected(tmp_path):
    path = tmp_path / 'dataset' / 'meta' / 'training_events.jsonl'
    path.parent.mkdir(parents=True)
    path.write_text(
        '{"episode_id":"duplicate"}\n'
        '{"episode_id":"duplicate"}\n',
        encoding='utf-8',
    )

    rows, duplicate_count = capture._aggregate_rows(tmp_path)

    assert len(rows) == 2
    assert duplicate_count == 1


def test_parallel_capture_lock_is_rejected(tmp_path):
    with capture.CaptureLock(tmp_path):
        with pytest.raises(
            capture.CapturePackageError,
            match='parallel capture',
        ):
            with capture.CaptureLock(tmp_path):
                pass


def test_completed_production_audit_has_full_captured_distributions():
    canary = capture.captured_audit(PACKAGE_ROOT, 'canary')
    production = capture.captured_audit(PACKAGE_ROOT, 'full')

    assert canary['passed']
    assert canary['valid_scenario_count'] == capture.CANARY_COUNT
    assert canary['valid_image_count'] == capture.CANARY_COUNT * 2
    assert production['passed']
    assert all(production['checks'].values())
    assert production['valid_scenario_count'] == capture.EXPECTED_SCENARIOS
    assert production['valid_image_count'] == capture.EXPECTED_SCENARIOS * 2
    distributions = production['captured_distributions']
    assert len(distributions['configuration_variant_count']) == 255
    assert set(distributions['configuration_variant_count'].values()) == {8}
    assert set(
        distributions['configuration_unique_geometry_count'].values()
    ) == {8}
    assert distributions['relation_family'] == (
        capture.EXPECTED_RELATION_TOTALS
    )
    assert distributions['target_zone'] == capture.EXPECTED_ZONE_TOTALS
    for identity in capture.GLOBAL_IDENTITIES:
        assert distributions['identity_presence'][identity] == 1024
        assert distributions['identity_absence'][identity] == 1016
        assert distributions['identity_loaded'][identity] == 512
        assert distributions['identity_empty'][identity] == 512
    assert all(
        distributions['all_active_segments'][
            f'{side}:{segment}'
        ] > 0
        for side in ('left', 'right')
        for segment in capture.valid_public_segments(side)
    )


def test_all_generated_shell_scripts_have_valid_syntax():
    for path in sorted(PACKAGE_ROOT.glob('*.sh')):
        completed = subprocess.run(
            ['bash', '-n', str(path)],
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0, (
            f'{path}: {completed.stderr}'
        )


def test_deterministic_regeneration_is_byte_identical(tmp_path):
    regenerated = tmp_path / 'regenerated'
    capture.prepare_package(
        regenerated,
        declared_root=PACKAGE_ROOT,
    )

    regenerated_manifest = _object(
        regenerated / 'package_manifest.json'
    )
    current_manifest = _object(PACKAGE_ROOT / 'package_manifest.json')
    assert regenerated_manifest['static_files'] == (
        current_manifest['static_files']
    )
    assert hashlib.sha256(
        (regenerated / 'scenario_manifest.jsonl').read_bytes()
    ).hexdigest() == hashlib.sha256(
        (PACKAGE_ROOT / 'scenario_manifest.jsonl').read_bytes()
    ).hexdigest()


def test_protected_packages_remain_byte_identical():
    protected = capture._capture_protected_audit()

    assert protected['passed']
    assert all(
        result['file_count'] == result['expected_file_count']
        and result['tree_sha256'] == result['expected_tree_sha256']
        for result in protected['artifacts'].values()
    )


def test_completed_package_contains_production_results_and_no_split_files():
    files = {
        str(path.relative_to(PACKAGE_ROOT))
        for path in PACKAGE_ROOT.rglob('*')
        if path.is_file()
    }

    assert 'captured_production_audit.json' in files
    assert 'production_review_gallery.html' in files
    assert 'production_review_gallery_manifest.json' in files
    assert _object(
        PACKAGE_ROOT / 'captured_production_audit.json'
    )['passed']
    assert _object(
        PACKAGE_ROOT / 'production_review_gallery_manifest.json'
    )['passed']
    assert not any(
        name.endswith((
            'train.jsonl',
            'validation.jsonl',
            'test.jsonl',
        ))
        for name in files
    )
    assert len([
        path
        for path in (PACKAGE_ROOT / 'dataset' / 'episodes').iterdir()
        if path.is_dir() and not path.name.startswith('.')
    ]) == capture.EXPECTED_SCENARIOS
