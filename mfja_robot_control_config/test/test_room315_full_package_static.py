#!/usr/bin/env python3

import hashlib
import json
import subprocess
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(
    '/home/tiago/room315_eight_shuttle_visual_320_seed31520260727'
)
SMOKE_APPROVAL = Path(
    '/home/tiago/room315_eight_shuttle_visual_smoke_seed31520260727/'
    'manual_smoke_approval.json'
)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.skipif(
    not (PACKAGE_ROOT / 'static_manifest_audit.json').is_file(),
    reason='Phase B static package is not available',
)
def test_full_package_static_audit_is_fixed_eight_and_exactly_balanced():
    audit = json.loads(
        (PACKAGE_ROOT / 'static_manifest_audit.json').read_text(
            encoding='utf-8'
        )
    )
    scenario = audit['scenario_audit']

    assert audit['passed'] is True
    assert scenario['count'] == 320
    assert scenario['distributions']['rail_scope'] == {
        'dual_four_plus_four': 64,
        'left_four': 128,
        'right_four': 128,
    }
    assert scenario['distributions']['shuttle_cardinality'] == {
        '4': 256,
        '8': 64,
    }
    assert scenario['distributions']['identity_occurrences'] == {
        identity: 192
        for identity in (
            'L1', 'L2', 'L3', 'L4',
            'R1', 'R2', 'R3', 'R4',
        )
    }
    representability = scenario['checks'][
        'fixed_schema_and_vectorizer_representability'
    ]
    assert representability['passed'] is True
    assert representability['vectorizer_dimension'] == 200
    assert representability['all_zero_present_block_targets'] == []
    assert representability['multi_hot_present_block_targets'] == []
    assert representability['unrepresentable_block_targets'] == []


@pytest.mark.skipif(
    not (PACKAGE_ROOT / 'package_manifest.json').is_file(),
    reason='Phase B static package is not available',
)
def test_package_manifest_hashes_and_capture_boundary():
    package_manifest = json.loads(
        (PACKAGE_ROOT / 'package_manifest.json').read_text(
            encoding='utf-8'
        )
    )
    post_manifest_repairs = {
        'capture_full.sh',
        'resume_capture.sh',
    }
    observed_repairs = set()
    for relative, expected in package_manifest['files'].items():
        path = PACKAGE_ROOT / relative
        assert path.is_file()
        if (
            path.stat().st_size == expected['bytes']
            and _sha256(path) == expected['sha256']
        ):
            continue

        # The two launch wrappers were repaired after manifest freeze so ROS
        # setup files can be sourced safely under bash nounset.  Their exact
        # pre-repair bytes remain beside them; audit both the frozen provenance
        # and the narrowly defined repair instead of pretending the live file
        # still has the historical hash.
        assert relative in post_manifest_repairs
        observed_repairs.add(relative)
        frozen = path.with_name(f'{path.name}.before_ros_nounset_fix')
        assert frozen.is_file()
        assert frozen.stat().st_size == expected['bytes']
        assert _sha256(frozen) == expected['sha256']
        frozen_text = frozen.read_text(encoding='utf-8')
        expected_repaired_text = frozen_text.replace(
            'set -euo pipefail',
            'set -eo pipefail',
            1,
        )
        for setup in (
            '/opt/ros/jazzy/setup.bash',
            '/home/tiago/mfja_3rd_floor_ros2_ws/install/setup.bash',
        ):
            expected_repaired_text = expected_repaired_text.replace(
                f'source {setup}',
                f'set +u\nsource {setup}\nset -u',
                1,
            )
        assert path.read_text(encoding='utf-8') == expected_repaired_text

    assert observed_repairs == post_manifest_repairs

    snapshot = json.loads(
        (PACKAGE_ROOT / 'manifest_approval_snapshot.json').read_text(
            encoding='utf-8'
        )
    )
    assert snapshot['approved_for_full_manifest_generation'] is True
    assert snapshot['approved_for_full_capture'] is False
    approval = json.loads(SMOKE_APPROVAL.read_text(encoding='utf-8'))
    assert approval['approved_for_full_manifest_generation'] is True
    assert approval['approved_for_full_capture'] is True

    completed = subprocess.run(
        [
            str(PACKAGE_ROOT / 'validate_manual_approval.py'),
            '--require',
            'capture',
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
    assert completed.stdout.splitlines()[0] == 'MANUAL_APPROVAL_VALID'


@pytest.mark.skipif(
    not (PACKAGE_ROOT / 'capture_state.json').is_file(),
    reason='Phase B static package is not available',
)
def test_completed_capture_has_exact_dataset_without_split_leakage():
    state = json.loads(
        (PACKAGE_ROOT / 'capture_state.json').read_text(encoding='utf-8')
    )

    assert state['capture_complete'] is True
    assert state['capture_has_started'] is True
    assert state['captured_scenario_count'] == 320
    assert state['expected_scenario_count'] == 320
    assert state['existing_valid_image_count'] == 640
    assert state['expected_image_count'] == 640
    assert state['aggregate_training_event_count'] == 320
    assert state['remaining_scenario_count'] == 0
    for empty_field in (
        'missing_scenarios',
        'incomplete_episodes',
        'event_index_missing',
        'event_index_unexpected',
        'duplicate_training_event_rows',
    ):
        assert state[empty_field] == []

    captured_audit_path = PACKAGE_ROOT / 'captured_dataset_audit.json'
    captured_audit = json.loads(
        captured_audit_path.read_text(encoding='utf-8')
    )
    assert captured_audit['passed'] is True
    assert captured_audit['capture_status'] == state
    expected_audit_hash = (
        PACKAGE_ROOT / 'captured_dataset_audit.sha256'
    ).read_text(encoding='utf-8').split()[0]
    assert _sha256(captured_audit_path) == expected_audit_hash

    training_events = (
        PACKAGE_ROOT / 'dataset' / 'meta' / 'training_events.jsonl'
    )
    assert training_events.is_file()
    assert len(training_events.read_text(encoding='utf-8').splitlines()) == 320
    assert _sha256(training_events) == captured_audit['fingerprints'][
        'training_events.jsonl'
    ]
    assert not list(
        PACKAGE_ROOT.glob('**/train.jsonl')
    )
    assert not list(
        PACKAGE_ROOT.glob('**/val.jsonl')
    )
    assert not list(
        PACKAGE_ROOT.glob('**/test.jsonl')
    )
