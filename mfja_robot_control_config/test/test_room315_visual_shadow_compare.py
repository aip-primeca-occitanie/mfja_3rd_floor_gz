import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from room_315_visual_shadow_compare import (  # noqa: E402
    ShadowComparisonAccumulator,
    compare_paired_observations,
)
import room_315_promote_runtime_v4 as promotion  # noqa: E402


V3_SHA = '8' * 64
V4_SHA = '4' * 64


def _observation(checkpoint, *, accepted=True, segment='A12E', ratio=0.2):
    rows = []
    for identity in ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4'):
        present = identity in {'L1', 'R4'}
        rows.append({
            'identity': identity,
            'presence_state': 'present' if present else 'absent',
            'visual_facts_valid': present,
            'side': ('left' if identity.startswith('L') else 'right') if present else '',
            'block': segment if present else '',
            'loaded_state': 'loaded' if present else '',
            's_ratio': ratio if present else 0.0,
        })
    return {
        'checkpoint_sha256': checkpoint,
        'accepted': accepted,
        'left_image_stamp_s': 10.0,
        'right_image_stamp_s': 10.01,
        'shuttles': rows,
    }


def test_compare_requires_same_pair_and_reports_differences():
    old = _observation(V3_SHA, segment='A12E', ratio=0.2)
    new = _observation(V4_SHA, segment='A34E', ratio=0.25)
    report = compare_paired_observations(old, new)
    assert report['present_identity_count'] == 2
    assert report['segment_agreement_count'] == 0
    assert report['loaded_agreement_count'] == 2
    assert report['maximum_absolute_s_ratio_difference'] == pytest.approx(0.05)
    assert report['wrong_v4_side_identities'] == []


def test_accumulator_pairs_by_image_stamps_and_passes_without_quality_claim():
    accumulator = ShadowComparisonAccumulator(V3_SHA, V4_SHA, minimum_paired_frames=1)
    accumulator.add('v4', _observation(V4_SHA))
    accumulator.add('v3', _observation(V3_SHA))
    report = accumulator.report()
    assert report['status'] == 'passed'
    assert report['paired_frame_count'] == 1
    assert report['v4_acceptance_coverage'] == 1.0
    assert report['quality_ground_truth_available'] is False
    assert not report['automatic_runtime_switch']
    assert report['control_isolation'] == {
        'dry_run_state_fusion': True,
        'plansys2_update_enabled': False,
        'plansys2_mutation_count': 0,
        'actuation_command_count': 0,
        'comparator_owns_command_publisher': False,
        'evidence_scope': (
            'canonical_shadow_launch_contract_and_runtime_node_ownership'
        ),
    }


def test_accumulator_fails_closed_on_hash_rejection_or_wrong_side():
    accumulator = ShadowComparisonAccumulator(V3_SHA, V4_SHA, minimum_paired_frames=1)
    accumulator.add('v3', _observation('0' * 64))
    assert accumulator.report()['status'] == 'failed'

    accumulator = ShadowComparisonAccumulator(V3_SHA, V4_SHA, minimum_paired_frames=1)
    new = _observation(V4_SHA)
    next(row for row in new['shuttles'] if row['identity'] == 'R4')['side'] = 'left'
    accumulator.add('v3', _observation(V3_SHA))
    accumulator.add('v4', new)
    report = accumulator.report()
    assert report['status'] == 'failed'
    assert report['wrong_v4_side_identities'] == ['R4']


def test_actual_report_contract_is_consumed_by_manual_promotion_verifier():
    """Keep the report producer and its only promotion consumer in lockstep."""

    promoter_v3_sha = promotion.V3_CHECKPOINT_SHA256
    accumulator = ShadowComparisonAccumulator(
        promoter_v3_sha,
        V4_SHA,
        minimum_paired_frames=1,
    )
    accumulator.add('v3', _observation(promoter_v3_sha))
    accumulator.add('v4', _observation(V4_SHA))
    report = accumulator.report()

    assert set(report) == {
        'schema_version',
        'status',
        'role',
        'automatic_runtime_switch',
        'control_isolation',
        'quality_ground_truth_available',
        'expected_checkpoint_sha256',
        'minimum_paired_frames',
        'paired_frame_count',
        'present_slot_comparison_count',
        'v3_accepted_frame_count',
        'v4_accepted_frame_count',
        'v4_acceptance_coverage',
        'segment_agreement_rate',
        'loaded_agreement_rate',
        'maximum_absolute_s_ratio_difference',
        'wrong_v4_side_identities',
        'unpaired_frame_count',
        'errors',
        'interpretation',
    }
    assert report['schema_version'] == promotion.SHADOW_REPORT_SCHEMA
    assert report['status'] == 'passed'
    assert report['role'] == 'observation_only_same_frame_shadow'
    assert report['automatic_runtime_switch'] is False
    assert report['quality_ground_truth_available'] is False
    assert report['expected_checkpoint_sha256'] == {
        'v3': promoter_v3_sha,
        'v4': V4_SHA,
    }
    assert report['minimum_paired_frames'] == 1
    assert report['paired_frame_count'] == 1
    assert report['present_slot_comparison_count'] == 2
    assert report['v3_accepted_frame_count'] == 1
    assert report['v4_accepted_frame_count'] == 1
    assert report['v4_acceptance_coverage'] == 1.0
    assert report['segment_agreement_rate'] == 1.0
    assert report['loaded_agreement_rate'] == 1.0
    assert report['maximum_absolute_s_ratio_difference'] == 0.0
    assert report['wrong_v4_side_identities'] == []
    assert report['unpaired_frame_count'] == 0
    assert report['errors'] == []
    assert report['control_isolation'] == {
        'dry_run_state_fusion': True,
        'plansys2_update_enabled': False,
        'plansys2_mutation_count': 0,
        'actuation_command_count': 0,
        'comparator_owns_command_publisher': False,
        'evidence_scope': (
            'canonical_shadow_launch_contract_and_runtime_node_ownership'
        ),
    }
    assert report['interpretation'] == (
        'Agreement is a migration diagnostic only; V3 is not ground truth. '
        'Scenario-grounded runtime acceptance is evaluated separately.'
    )

    # The promotion verifier binds this report to the candidate through the
    # exact V4 checkpoint.  Candidate ID is intentionally optional in this
    # report schema; the quantitative and fault reports carry that ID.
    promotion._verify_shadow_report(
        report,
        SimpleNamespace(
            checkpoint_sha256=V4_SHA,
            candidate_id='candidate-bound-by-the-other-review-reports',
        ),
    )
