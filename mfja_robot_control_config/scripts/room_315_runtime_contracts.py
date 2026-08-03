#!/usr/bin/env python3
"""Shared fail-closed contracts for the Room 315 runtime boundary.

This module is deliberately ROS-independent.  It owns small cross-component
contracts that must be interpreted identically by visual grounding, PDDL
generation, the closed-loop executive, and the safety supervisor.
"""

from __future__ import annotations

import copy
import math
from typing import Any

from room_315_multi_shuttle import normalize_shuttle_ref


CONTROLLER_DISABLED_MODE = 'DISABLED'
VISUAL_PAYLOAD_CONFIRMATION_CONTRACT = (
    'room315.visual_payload_confirmation.v1'
)
RUNTIME_PAYLOAD_GROUNDING_CONTRACT = (
    'room315.runtime_payload_grounding.v2'
)
PAYLOAD_SELECTION_FILTERS = frozenset({'loaded', 'empty'})
MIN_PAYLOAD_CONFIRMATION_FRAMES = 3
CLEARANCE_ENTRY_MATCH_METHOD = (
    'interior_entry_sensor_plus_bounded_travel_time'
)
CLEARANCE_ADVANCE_MATCH_METHOD = (
    'certified_interior_origin_plus_bounded_travel_time'
)
CLEARANCE_MATCH_METHODS = frozenset({
    CLEARANCE_ENTRY_MATCH_METHOD,
    CLEARANCE_ADVANCE_MATCH_METHOD,
})
ACCEPTED_SUPERVISOR_STATUSES = frozenset({
    'accepted',
    'approved',
    'executed',
    'ok',
})
REJECTED_SUPERVISOR_STATUSES = frozenset({
    'rejected',
    'failed',
    'blocked',
    'timed_out',
    'timeout',
})


def supervisor_decision_is_terminal(decision: Any) -> bool:
    """Return true only for an explicit terminal supervisor decision."""

    if not isinstance(decision, dict) or not decision:
        return False
    status = str(
        decision.get('status') or decision.get('decision') or ''
    ).strip().casefold()
    return bool(
        isinstance(decision.get('accepted'), bool)
        or status in ACCEPTED_SUPERVISOR_STATUSES
        or status in REJECTED_SUPERVISOR_STATUSES
    )


def supervisor_decision_accepted(decision: Any) -> bool:
    """Interpret supervisor decisions fail-closed with no implicit success."""

    if not supervisor_decision_is_terminal(decision):
        return False
    status = str(
        decision.get('status') or decision.get('decision') or ''
    ).strip().casefold()
    if decision.get('accepted') is False:
        return False
    if status in REJECTED_SUPERVISOR_STATUSES:
        return False
    return bool(
        decision.get('accepted') is True
        or status in ACCEPTED_SUPERVISOR_STATUSES
    )


def normalize_runtime_clearance_certificate(
    raw_identity: Any,
    raw_certificate: Any,
) -> dict[str, Any]:
    """Validate and normalize one persisted interior-clearance proof.

    Topology-specific checks (valid branch, sensor for the selected gate, and
    physical spacing between certificates) remain at their authoritative
    boundaries.  This function owns the cross-component provenance contract
    so a proof accepted by one runtime layer cannot be rejected by another
    merely because their field checks drifted apart.
    """

    if not isinstance(raw_certificate, dict):
        raise ValueError('runtime clearance certificate must be an object')
    certificate = copy.deepcopy(raw_certificate)
    key_spec = normalize_shuttle_ref(raw_identity)
    identity_spec = normalize_shuttle_ref(certificate.get('identity'))
    shuttle_spec = normalize_shuttle_ref(certificate.get('shuttle'))
    if key_spec is None or identity_spec is None or shuttle_spec is None:
        raise ValueError('runtime clearance certificate has an unknown identity')
    if key_spec != identity_spec or key_spec != shuttle_spec:
        raise ValueError(
            f'runtime clearance identity conflict:{key_spec.short_id}'
        )

    side = str(certificate.get('side') or '').strip().casefold()
    if side != key_spec.side:
        raise ValueError(
            f'runtime clearance side conflict:{key_spec.short_id}:{side or "missing"}'
        )
    matched_by = str(certificate.get('matched_by') or '').strip()
    if matched_by not in CLEARANCE_MATCH_METHODS:
        raise ValueError(
            f'runtime clearance lacks bounded-motion proof:{key_spec.short_id}'
        )
    proof_requirements = (
        ('entry_sensor_identity_confirmed', True, 'lacks entry sensor proof'),
        ('controller_stop_confirmed', True, 'lacks stop proof'),
        (
            'post_stop_visual_frame_received',
            True,
            'lacks post-stop visual proof',
        ),
        (
            'bounded_commanded_motion_completed',
            True,
            'lacks completed bounded motion',
        ),
        ('clearance_mode_held', True, 'lacks held-route proof'),
        ('normal_route_restored', False, 'lacks held-route proof'),
        ('model_prediction_replaced', False, 'replaced model prediction'),
        (
            'controller_position_fields_used_for_localization',
            False,
            'used forbidden controller position',
        ),
    )
    for field, expected, message in proof_requirements:
        if certificate.get(field) is not expected:
            raise ValueError(
                f'runtime clearance {message}:{key_spec.short_id}'
            )

    target_segment = str(
        certificate.get('target_segment') or ''
    ).strip().upper()
    entry_sensor = str(
        certificate.get('entry_sensor') or ''
    ).strip().upper()
    try:
        target_s_m = float(certificate['target_s_m'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'runtime clearance target_s_m is invalid:{key_spec.short_id}'
        ) from exc
    if not target_segment or not entry_sensor:
        raise ValueError(
            f'runtime clearance target is incomplete:{key_spec.short_id}'
        )
    if not math.isfinite(target_s_m) or target_s_m <= 0.0:
        raise ValueError(
            f'runtime clearance target_s_m out of bounds:{key_spec.short_id}'
        )

    if matched_by == CLEARANCE_ADVANCE_MATCH_METHOD:
        origin_proof = certificate.get('origin_clearance_proof')
        if not isinstance(origin_proof, dict):
            raise ValueError(
                f'runtime clearance advance origin is missing:{key_spec.short_id}'
            )
        origin_spec = normalize_shuttle_ref(
            origin_proof.get('identity') or origin_proof.get('shuttle')
        )
        try:
            motion_origin_s_m = float(certificate['motion_origin_s_m'])
            bounded_distance_m = float(
                certificate['bounded_motion_distance_m']
            )
            origin_target_s_m = float(origin_proof['target_s_m'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                'runtime clearance advance distance is invalid:'
                f'{key_spec.short_id}'
            ) from exc
        if not all(math.isfinite(value) for value in (
            target_s_m,
            motion_origin_s_m,
            bounded_distance_m,
            origin_target_s_m,
        )):
            raise ValueError(
                'runtime clearance advance distance is non-finite:'
                f'{key_spec.short_id}'
            )
        if (
            certificate.get('interior_advance_origin_certified') is not True
            or origin_spec != key_spec
            or origin_proof.get('entry_sensor_identity_confirmed') is not True
            or origin_proof.get('controller_stop_confirmed') is not True
            or origin_proof.get('bounded_commanded_motion_completed') is not True
            or origin_proof.get(
                'controller_position_fields_used_for_localization'
            ) is not False
            or not math.isclose(
                origin_target_s_m,
                motion_origin_s_m,
                abs_tol=1e-9,
            )
            or target_s_m <= motion_origin_s_m
            or bounded_distance_m <= 0.0
            or not math.isclose(
                bounded_distance_m,
                target_s_m - motion_origin_s_m,
                abs_tol=1e-9,
            )
        ):
            raise ValueError(
                'runtime clearance advance origin proof is incomplete:'
                f'{key_spec.short_id}'
            )

    certificate.update({
        'identity': key_spec.short_id,
        'shuttle': key_spec.shuttle_id,
        'side': key_spec.side,
        'target_segment': target_segment,
        'target_s_m': target_s_m,
        'entry_sensor': entry_sensor,
        'matched_by': matched_by,
    })
    return certificate


def create_visual_payload_confirmation(
    *,
    selected_shuttle: str,
    payload_filter: str,
    state_ids: list[str],
    observations_examined: int,
) -> dict[str, Any]:
    """Create one canonical consecutive-frame payload confirmation."""

    spec = normalize_shuttle_ref(selected_shuttle)
    normalized_filter = str(payload_filter or '').strip().casefold()
    normalized_state_ids = [str(value or '').strip() for value in state_ids]
    if spec is None:
        raise ValueError('payload confirmation has no authoritative identity')
    if normalized_filter not in PAYLOAD_SELECTION_FILTERS:
        raise ValueError(
            f'unsupported payload confirmation filter:{normalized_filter!r}'
        )
    if (
        len(normalized_state_ids) < MIN_PAYLOAD_CONFIRMATION_FRAMES
        or any(not state_id for state_id in normalized_state_ids)
        or len(set(normalized_state_ids)) != len(normalized_state_ids)
    ):
        raise ValueError(
            'payload confirmation requires distinct non-empty fresh states'
        )
    examined = int(observations_examined)
    if examined < len(normalized_state_ids):
        raise ValueError(
            'payload confirmation observations cannot be fewer than frames'
        )
    return {
        'contract': VISUAL_PAYLOAD_CONFIRMATION_CONTRACT,
        'required': True,
        'payload_filter': normalized_filter,
        'selected_shuttle': spec.shuttle_id,
        'confirmation_frames': len(normalized_state_ids),
        'state_ids': normalized_state_ids,
        'observations_examined': examined,
        'source': 'accepted_visual_state_sequence',
        'consecutive_identity_agreement': True,
        'raw_visual_predictions_preserved': True,
        'model_prediction_replaced': False,
        'controller_payload_state_used': False,
    }


def visual_payload_confirmation_matches(
    confirmation: dict[str, Any] | None,
    *,
    selected_shuttle: str,
    payload_filter: str,
    source_state_id: str,
) -> bool:
    """Validate visual confirmation without trusting caller-owned metadata."""

    if not isinstance(confirmation, dict):
        return False
    selected_spec = normalize_shuttle_ref(selected_shuttle)
    confirmation_spec = normalize_shuttle_ref(
        confirmation.get('selected_shuttle')
    )
    normalized_filter = str(payload_filter or '').strip().casefold()
    state_ids = confirmation.get('state_ids')
    if not isinstance(state_ids, list):
        return False
    normalized_state_ids = [str(value or '').strip() for value in state_ids]
    try:
        confirmation_frames = int(confirmation.get('confirmation_frames'))
        observations_examined = int(confirmation.get('observations_examined'))
    except (TypeError, ValueError):
        return False
    return bool(
        selected_spec is not None
        and confirmation_spec == selected_spec
        and normalized_filter in PAYLOAD_SELECTION_FILTERS
        and confirmation.get('contract')
        == VISUAL_PAYLOAD_CONFIRMATION_CONTRACT
        and confirmation.get('required') is True
        and confirmation.get('payload_filter') == normalized_filter
        and confirmation_frames >= MIN_PAYLOAD_CONFIRMATION_FRAMES
        and len(normalized_state_ids) == confirmation_frames
        and all(normalized_state_ids)
        and len(set(normalized_state_ids)) == confirmation_frames
        and observations_examined >= confirmation_frames
        and normalized_state_ids[-1] == str(source_state_id or '').strip()
        and confirmation.get('source')
        == 'accepted_visual_state_sequence'
        and confirmation.get('consecutive_identity_agreement') is True
        and confirmation.get('raw_visual_predictions_preserved') is True
        and confirmation.get('model_prediction_replaced') is False
        and confirmation.get('controller_payload_state_used') is False
    )


def create_runtime_payload_grounding(
    *,
    selected_shuttle: str,
    payload_filter: str,
    initial_visual_prediction: str,
    source_state_id: str,
    confirmation: dict[str, Any],
) -> dict[str, Any]:
    """Create a task-scoped payload proof from validated visual consensus."""

    spec = normalize_shuttle_ref(selected_shuttle)
    normalized_filter = str(payload_filter or '').strip().casefold()
    source_state = str(source_state_id or '').strip()
    prediction = str(initial_visual_prediction or '').strip().casefold()
    if spec is None or prediction != normalized_filter:
        raise ValueError('runtime payload grounding target/filter mismatch')
    if not visual_payload_confirmation_matches(
        confirmation,
        selected_shuttle=spec.shuttle_id,
        payload_filter=normalized_filter,
        source_state_id=source_state,
    ):
        raise ValueError(
            'runtime payload grounding lacks valid multi-frame visual '
            'confirmation'
        )
    return {
        'contract': RUNTIME_PAYLOAD_GROUNDING_CONTRACT,
        'selected_shuttle': spec.shuttle_id,
        'payload_filter': normalized_filter,
        'initial_visual_prediction': prediction,
        'source_state_id': source_state,
        'source': 'accepted_visual_temporal_consensus',
        'selection_time_only': True,
        'temporal_confirmation': dict(confirmation),
        'raw_future_visual_predictions_preserved': True,
        'model_prediction_replaced': False,
        'controller_payload_state_used': False,
    }


def runtime_payload_grounding_matches(
    proof: dict[str, Any] | None,
    *,
    selected_shuttle: str,
    payload_filter: str,
) -> bool:
    """Validate a complete task-scoped runtime payload grounding proof."""

    if not isinstance(proof, dict):
        return False
    selected_spec = normalize_shuttle_ref(selected_shuttle)
    proof_spec = normalize_shuttle_ref(proof.get('selected_shuttle'))
    normalized_filter = str(payload_filter or '').strip().casefold()
    source_state_id = str(proof.get('source_state_id') or '').strip()
    confirmation = proof.get('temporal_confirmation')
    return bool(
        selected_spec is not None
        and proof_spec == selected_spec
        and normalized_filter in PAYLOAD_SELECTION_FILTERS
        and proof.get('contract') == RUNTIME_PAYLOAD_GROUNDING_CONTRACT
        and proof.get('payload_filter') == normalized_filter
        and proof.get('initial_visual_prediction') == normalized_filter
        and source_state_id
        and proof.get('source') == 'accepted_visual_temporal_consensus'
        and proof.get('selection_time_only') is True
        and visual_payload_confirmation_matches(
            confirmation,
            selected_shuttle=selected_spec.shuttle_id,
            payload_filter=normalized_filter,
            source_state_id=source_state_id,
        )
        and proof.get('raw_future_visual_predictions_preserved') is True
        and proof.get('model_prediction_replaced') is False
        and proof.get('controller_payload_state_used') is False
    )
