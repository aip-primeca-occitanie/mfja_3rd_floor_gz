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
    for relative, expected in package_manifest['files'].items():
        path = PACKAGE_ROOT / relative
        assert path.is_file()
        assert path.stat().st_size == expected['bytes']
        assert _sha256(path) == expected['sha256']

    approval = json.loads(SMOKE_APPROVAL.read_text(encoding='utf-8'))
    assert approval['approved_for_full_manifest_generation'] is True
    assert approval['approved_for_full_capture'] is False

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
    assert completed.returncode == 3
    assert completed.stdout.strip() == 'WAITING_FOR_FULL_CAPTURE_APPROVAL'


@pytest.mark.skipif(
    not (PACKAGE_ROOT / 'capture_state.json').is_file(),
    reason='Phase B static package is not available',
)
def test_static_phase_has_no_capture_or_dataset_split():
    state = json.loads(
        (PACKAGE_ROOT / 'capture_state.json').read_text(encoding='utf-8')
    )

    assert state['captured_scenario_count'] == 0
    assert state['capture_has_started'] is False
    assert state['existing_valid_image_count'] == 0
    assert not (
        PACKAGE_ROOT / 'dataset' / 'meta' / 'training_events.jsonl'
    ).exists()
    assert not list(
        PACKAGE_ROOT.glob('**/train.jsonl')
    )
    assert not list(
        PACKAGE_ROOT.glob('**/val.jsonl')
    )
    assert not list(
        PACKAGE_ROOT.glob('**/test.jsonl')
    )
