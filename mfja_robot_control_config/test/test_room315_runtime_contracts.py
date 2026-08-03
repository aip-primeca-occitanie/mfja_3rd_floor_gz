#!/usr/bin/env python3

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_runtime_contracts import create_runtime_payload_grounding
from room_315_runtime_contracts import create_visual_payload_confirmation
from room_315_runtime_contracts import normalize_runtime_clearance_certificate
from room_315_runtime_contracts import runtime_payload_grounding_matches
from room_315_runtime_contracts import supervisor_decision_accepted
from room_315_runtime_contracts import supervisor_decision_is_terminal
from room_315_runtime_contracts import visual_payload_confirmation_matches


def _confirmation():
    return create_visual_payload_confirmation(
        selected_shuttle='R4',
        payload_filter='loaded',
        state_ids=['visual-1', 'visual-2', 'visual-3'],
        observations_examined=4,
    )


def _proof():
    return create_runtime_payload_grounding(
        selected_shuttle='room315_right_shuttle_4',
        payload_filter='loaded',
        initial_visual_prediction='loaded',
        source_state_id='visual-3',
        confirmation=_confirmation(),
    )


def _clearance_certificate(*, advanced: bool = False):
    certificate = {
        'identity': 'R1',
        'shuttle': 'right_shuttle_1',
        'side': 'right',
        'target_segment': 'a34i',
        'target_s_m': 0.92 if advanced else 0.49,
        'entry_sensor': 'da3ir',
        'matched_by': (
            'certified_interior_origin_plus_bounded_travel_time'
            if advanced
            else 'interior_entry_sensor_plus_bounded_travel_time'
        ),
        'entry_sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'post_stop_visual_frame_received': True,
        'bounded_commanded_motion_completed': True,
        'clearance_mode_held': True,
        'normal_route_restored': False,
        'model_prediction_replaced': False,
        'controller_position_fields_used_for_localization': False,
    }
    if advanced:
        certificate.update({
            'interior_advance_origin_certified': True,
            'motion_origin_s_m': 0.49,
            'bounded_motion_distance_m': 0.43,
            'origin_clearance_proof': {
                'identity': 'R1',
                'target_s_m': 0.49,
                'entry_sensor_identity_confirmed': True,
                'controller_stop_confirmed': True,
                'bounded_commanded_motion_completed': True,
                'controller_position_fields_used_for_localization': False,
            },
        })
    return certificate


def test_shared_payload_contract_builders_canonicalize_identity_once():
    confirmation = _confirmation()
    proof = _proof()

    assert confirmation['selected_shuttle'] == 'right_shuttle_4'
    assert confirmation['confirmation_frames'] == 3
    assert proof['selected_shuttle'] == 'right_shuttle_4'
    assert visual_payload_confirmation_matches(
        confirmation,
        selected_shuttle='R4',
        payload_filter='loaded',
        source_state_id='visual-3',
    )
    assert runtime_payload_grounding_matches(
        proof,
        selected_shuttle='room315_right_shuttle_4',
        payload_filter='loaded',
    )


@pytest.mark.parametrize(
    ('path', 'value'),
    (
        (('contract',), 'wrong.contract'),
        (('source_state_id',), 'wrong-state'),
        (('controller_payload_state_used',), True),
        (('model_prediction_replaced',), True),
        (('temporal_confirmation', 'state_ids'), ['visual-1'] * 3),
        (('temporal_confirmation', 'controller_payload_state_used'), True),
    ),
)
def test_runtime_payload_contract_rejects_tampered_provenance(path, value):
    proof = copy.deepcopy(_proof())
    target = proof
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    assert not runtime_payload_grounding_matches(
        proof,
        selected_shuttle='R4',
        payload_filter='loaded',
    )


def test_payload_confirmation_rejects_short_or_duplicate_frame_sequences():
    with pytest.raises(ValueError, match='distinct non-empty fresh states'):
        create_visual_payload_confirmation(
            selected_shuttle='R4',
            payload_filter='loaded',
            state_ids=['visual-1', 'visual-2'],
            observations_examined=2,
        )
    with pytest.raises(ValueError, match='distinct non-empty fresh states'):
        create_visual_payload_confirmation(
            selected_shuttle='R4',
            payload_filter='loaded',
            state_ids=['visual-1', 'visual-1', 'visual-2'],
            observations_examined=3,
        )


def test_runtime_payload_contract_rejects_identity_or_filter_mismatch():
    proof = _proof()

    assert not runtime_payload_grounding_matches(
        proof,
        selected_shuttle='R2',
        payload_filter='loaded',
    )
    assert not runtime_payload_grounding_matches(
        proof,
        selected_shuttle='R4',
        payload_filter='empty',
    )


@pytest.mark.parametrize('advanced', (False, True))
def test_shared_clearance_contract_accepts_both_supported_proof_modes(advanced):
    normalized = normalize_runtime_clearance_certificate(
        'room315_right_shuttle_1',
        _clearance_certificate(advanced=advanced),
    )

    assert normalized['identity'] == 'R1'
    assert normalized['shuttle'] == 'right_shuttle_1'
    assert normalized['side'] == 'right'
    assert normalized['target_segment'] == 'A34I'
    assert normalized['entry_sensor'] == 'DA3IR'


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('identity', 'R2'),
        ('side', 'left'),
        ('post_stop_visual_frame_received', False),
        ('bounded_commanded_motion_completed', False),
        ('clearance_mode_held', False),
        ('normal_route_restored', True),
        ('model_prediction_replaced', True),
        ('controller_position_fields_used_for_localization', True),
    ),
)
def test_shared_clearance_contract_rejects_one_field_provenance_mutation(
    field,
    value,
):
    certificate = _clearance_certificate()
    certificate[field] = value

    with pytest.raises(ValueError):
        normalize_runtime_clearance_certificate('R1', certificate)


def test_shared_clearance_contract_rejects_invalid_advance_distance():
    certificate = _clearance_certificate(advanced=True)
    certificate['bounded_motion_distance_m'] = 0.42

    with pytest.raises(ValueError, match='advance origin proof is incomplete'):
        normalize_runtime_clearance_certificate('R1', certificate)


@pytest.mark.parametrize(
    'target_s_m',
    (float('nan'), float('inf'), 0.0, -0.01),
)
def test_shared_clearance_contract_rejects_invalid_target_distance(target_s_m):
    certificate = _clearance_certificate()
    certificate['target_s_m'] = target_s_m

    with pytest.raises(ValueError, match='target_s_m out of bounds'):
        normalize_runtime_clearance_certificate('R1', certificate)


@pytest.mark.parametrize(
    'decision',
    (
        {},
        {'status': 'pending'},
        {'decision': 'unknown'},
        {'reason': 'missing explicit decision'},
    ),
)
def test_shared_supervisor_decision_contract_fails_closed(decision):
    assert not supervisor_decision_is_terminal(decision)
    assert not supervisor_decision_accepted(decision)


@pytest.mark.parametrize(
    ('decision', 'accepted'),
    (
        ({'accepted': True}, True),
        ({'status': 'approved'}, True),
        ({'status': 'rejected'}, False),
        ({'accepted': False, 'status': 'accepted'}, False),
    ),
)
def test_shared_supervisor_decision_contract_requires_explicit_result(
    decision,
    accepted,
):
    assert supervisor_decision_is_terminal(decision)
    assert supervisor_decision_accepted(decision) is accepted
