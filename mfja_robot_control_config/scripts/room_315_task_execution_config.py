#!/usr/bin/env python3
"""Authoritative defaults and validation for Room 315 task execution."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import stat
from pathlib import Path
from typing import Any

from room_315_runtime_contracts import MIN_PAYLOAD_CONFIRMATION_FRAMES


SCRIPT_DIR = Path(__file__).resolve().parent
RUNTIME_PDDL_DOMAIN_PATH = (
    SCRIPT_DIR.parent
    / 'config'
    / 'room_315_vla'
    / 'pddl'
    / 'domain_room315_runtime.pddl'
)
PLANSYS2_RUNTIME_SOLVER_TIMEOUT_S = 15.0
DEFAULT_PLANNER_CLIENT_TIMEOUT_S = 20.0
DEFAULT_PAYLOAD_GROUNDING_CONFIRMATION_FRAMES = 5
DEFAULT_PAYLOAD_GROUNDING_MAX_OBSERVATIONS = 15
DEFAULT_ALLOWED_VISUAL_SCHEMA_VERSION = 'room315.visual_state.v4'
DEFAULT_ALLOWED_VISUAL_CHECKPOINT_SHA256 = (
    '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
)
V4_TASK_EXECUTION_AUTHORIZATION_SCHEMA_VERSION = (
    'room315.deployment_candidate_state.v4.v1'
)
V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION = 'room315.visual_state.v4'
V4_TASK_EXECUTION_CHECKPOINT_SHA256 = (
    '869d64049b0092c37d21a4c8b910dc6b91954527e0e49c5694fa82dce570f40d'
)
V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256 = (
    '6f9828219c22599825f5a14e405c8f11ce017984cc0d65821a357240d6529e2a'
)
V4_TASK_EXECUTION_SOURCE_ACTIVE_MANIFEST_SHA256 = (
    '8cb1674fe50bc8fbd372ade36fcb05d188f519ab36ea6d22dbb23ecfac3286c3'
)
V4_TASK_EXECUTION_AUTHORIZATION_SCOPE = (
    'gazebo_v4_closed_loop_qualification_only'
)
V4_TASK_EXECUTION_RUNTIME_AUTHORIZATION_SCOPE = (
    'gazebo_v4_closed_loop_runtime_only'
)
V4_TASK_EXECUTION_CANDIDATE_ID = (
    'room315_visual_runtime_candidate_v4_seed31520260811_epoch11_869d6404_shadow'
)
V4_TASK_EXECUTION_PROMOTION_SCHEMA_VERSION = (
    'room315.visual_runtime_promotion.v4.v1'
)
V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION = (
    'room315.visual_runtime_v4.manual_decision.v1'
)
V4_TASK_EXECUTION_RUNTIME_GUARDS = {
    'actuation_enabled': True,
    'dry_run_state_fusion': True,
    'plansys2_update_enabled': False,
}
V4_TASK_EXECUTION_CLOSED_LOOP_CAMPAIGN_SCHEMA_VERSION = (
    'room315.v4_closed_loop_campaign.v1'
)
V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION = (
    'room315.v4_closed_loop_fault_campaign.v1'
)
V4_TASK_EXECUTION_FAULT_EVIDENCE_SCHEMA_VERSION = (
    'room315.v4_closed_loop_fault_evidence.v1'
)
V4_TASK_EXECUTION_CLOSED_LOOP_CASE_COUNT = 12
V4_TASK_EXECUTION_FAULT_SCENARIOS = (
    ('F01', 'corrupt_authorization'),
    ('F02', 'wrong_promotion_manifest'),
    ('F03', 'planner_unavailable'),
    ('F04', 'emergency_stop_during_long_move'),
    ('F05', 'right_sensor_feedback_loss'),
)
MAX_TASK_EXECUTION_AUTHORIZATION_BYTES = 64 * 1024
MAX_TASK_EXECUTION_EVIDENCE_BYTES = 4 * 1024 * 1024

_SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')

# ``arrival_confirmation_frames`` is accepted only so an older launch override
# does not fail parameter declaration. Runtime arrival is sensor-confirmed.
TASK_EXECUTION_COMPATIBILITY_PARAMETER_DEFAULTS = {
    'arrival_confirmation_frames': 3,
}

# These remain empty in the node fallback deliberately. A deployment YAML must
# provide all values explicitly, while an execution-disabled node can still
# start without an authorization artifact and cannot be enabled implicitly.
TASK_EXECUTION_AUTHORIZATION_PARAMETER_DEFAULTS = {
    'task_execution_authorization_path': '',
    'task_execution_authorization_sha256': '',
    'task_execution_promotion_manifest_path': '',
}

TASK_EXECUTION_ACTIVE_PARAMETER_DEFAULTS = {
    'use_sim_time': True,
    'execution_enabled': False,
    'task_goal_topic': '/room_315/task_goal',
    'task_status_topic': '/room_315/task_goal/status',
    'accepted_observed_state_topic': '/room_315/visual_state/observed_state',
    'allowed_visual_schema_version': DEFAULT_ALLOWED_VISUAL_SCHEMA_VERSION,
    'allowed_visual_checkpoint_sha256': (
        DEFAULT_ALLOWED_VISUAL_CHECKPOINT_SHA256
    ),
    'supervisor_command_topic': '/room_315/vla/command',
    'supervisor_status_topic': '/room_315/vla/status',
    'left_sensor_feedback_topic': '/room_315/rails/left/sensors/feedback',
    'right_sensor_feedback_topic': '/room_315/rails/right/sensors/feedback',
    'diagnostics_topic': '/diagnostics',
    'planner_service': '/planner/get_plan',
    'planner_domain_path': str(RUNTIME_PDDL_DOMAIN_PATH),
    'planner_timeout_s': DEFAULT_PLANNER_CLIENT_TIMEOUT_S,
    'observation_timeout_s': 1.5,
    'supervisor_status_timeout_s': 1.5,
    'slot_sensor_state_timeout_s': 1.0,
    'observation_wait_s': 2.0,
    'planning_slot_tolerance_ratio': 0.12,
    'target_arrival_tolerance_ratio': 0.05,
    'position_consistency_tolerance_m': 0.08,
    'slot_sensor_confirmation_frames': 2,
    'payload_grounding_confirmation_frames': (
        DEFAULT_PAYLOAD_GROUNDING_CONFIRMATION_FRAMES
    ),
    'payload_grounding_max_observations': (
        DEFAULT_PAYLOAD_GROUNDING_MAX_OBSERVATIONS
    ),
    'controller_stop_timeout_s': 3.0,
    'speed_mps': 0.2,
    'max_steps': 32,
    'max_replans': 8,
    'max_unknown_retries': 3,
    'supervisor_timeout_s': 5.0,
    'effect_timeout_s': 30.0,
    'clearance_effect_timeout_s': 60.0,
    # Final arrival remains sensor-gated, but its deadline must cover the
    # authoritative forward-only route rather than assuming every move is an
    # adjacent-slot move.
    'route_arrival_timeout_scale': 1.25,
    'route_arrival_timeout_margin_s': 5.0,
    'external_obstacles_disabled': True,
    'diagnostic_period_s': 1.0,
}

TASK_EXECUTION_PARAMETER_DEFAULTS = {
    **TASK_EXECUTION_ACTIVE_PARAMETER_DEFAULTS,
    **TASK_EXECUTION_COMPATIBILITY_PARAMETER_DEFAULTS,
    **TASK_EXECUTION_AUTHORIZATION_PARAMETER_DEFAULTS,
}

_POSITIVE_FLOAT_PARAMETERS = frozenset({
    'planner_timeout_s',
    'observation_timeout_s',
    'supervisor_status_timeout_s',
    'slot_sensor_state_timeout_s',
    'observation_wait_s',
    'planning_slot_tolerance_ratio',
    'target_arrival_tolerance_ratio',
    'position_consistency_tolerance_m',
    'controller_stop_timeout_s',
    'speed_mps',
    'supervisor_timeout_s',
    'effect_timeout_s',
    'clearance_effect_timeout_s',
    'route_arrival_timeout_scale',
    'route_arrival_timeout_margin_s',
    'diagnostic_period_s',
})
_NON_NEGATIVE_INTEGER_PARAMETERS = frozenset({
    'max_replans',
    'max_unknown_retries',
})
_POSITIVE_INTEGER_PARAMETERS = frozenset({
    'max_steps',
    'slot_sensor_confirmation_frames',
    'payload_grounding_confirmation_frames',
    'payload_grounding_max_observations',
})


def validate_task_execution_parameters(
    parameters: dict[str, Any],
) -> dict[str, str] | None:
    """Reject inconsistent runtime overrides before subscriptions start."""

    missing = sorted(
        name
        for name in TASK_EXECUTION_ACTIVE_PARAMETER_DEFAULTS
        if name not in parameters
    )
    if missing:
        raise ValueError(f'missing task-execution parameters:{missing}')
    validate_visual_publisher_allowlist(
        schema_version=parameters['allowed_visual_schema_version'],
        checkpoint_sha256=parameters['allowed_visual_checkpoint_sha256'],
    )
    for name in _POSITIVE_FLOAT_PARAMETERS:
        value = float(parameters[name])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
    for name in _NON_NEGATIVE_INTEGER_PARAMETERS:
        if int(parameters[name]) < 0:
            raise ValueError(f'{name} must be non-negative')
    for name in _POSITIVE_INTEGER_PARAMETERS:
        if int(parameters[name]) < 1:
            raise ValueError(f'{name} must be at least one')
    for name in (
        'planning_slot_tolerance_ratio',
        'target_arrival_tolerance_ratio',
    ):
        if float(parameters[name]) > 1.0:
            raise ValueError(f'{name} must be no greater than one')
    if float(parameters['planner_timeout_s']) <= (
        PLANSYS2_RUNTIME_SOLVER_TIMEOUT_S
    ):
        raise ValueError(
            'planner_timeout_s must exceed the PlanSys2 solver timeout '
            f'({PLANSYS2_RUNTIME_SOLVER_TIMEOUT_S:.1f}s)'
        )
    confirmation_frames = int(
        parameters['payload_grounding_confirmation_frames']
    )
    if confirmation_frames < MIN_PAYLOAD_CONFIRMATION_FRAMES:
        raise ValueError(
            'payload_grounding_confirmation_frames must be at least '
            f'{MIN_PAYLOAD_CONFIRMATION_FRAMES}'
        )
    if int(parameters['payload_grounding_max_observations']) < (
        confirmation_frames
    ):
        raise ValueError(
            'payload_grounding_max_observations must be no smaller than '
            'payload_grounding_confirmation_frames'
        )
    if float(parameters['clearance_effect_timeout_s']) < float(
        parameters['effect_timeout_s']
    ):
        raise ValueError(
            'clearance_effect_timeout_s must be no smaller than '
            'effect_timeout_s'
        )
    if float(parameters['route_arrival_timeout_scale']) < 1.0:
        raise ValueError(
            'route_arrival_timeout_scale must be at least one'
        )
    return verify_v4_task_execution_authorization(parameters)


def validate_visual_publisher_allowlist(
    *,
    schema_version: Any,
    checkpoint_sha256: Any,
) -> None:
    """Validate the exact visual publisher provenance admitted to planning."""

    if schema_version != V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION:
        raise ValueError(
            'allowed_visual_schema_version must be '
            f'{V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION}'
        )
    if checkpoint_sha256 != V4_TASK_EXECUTION_CHECKPOINT_SHA256:
        raise ValueError(
            'allowed_visual_checkpoint_sha256 must be the exact authorized '
            f'V4 checkpoint {V4_TASK_EXECUTION_CHECKPOINT_SHA256}'
        )


def verify_v4_task_execution_authorization(
    parameters: dict[str, Any],
) -> dict[str, str] | None:
    """Verify the immutable, hash-bound V4 Gazebo execution decision.

    No artifact is required while execution is disabled.  Enabling execution
    binds the task gateway to one visual schema, checkpoint and active V4
    promotion manifest, and requires a separate immutable manual decision.
    """

    execution_enabled = parameters.get('execution_enabled')
    if not isinstance(execution_enabled, bool):
        raise ValueError('execution_enabled must be a boolean')
    if not execution_enabled:
        return None

    if parameters.get('allowed_visual_schema_version') != (
        V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
    ):
        raise ValueError(
            'execution_enabled requires allowed_visual_schema_version='
            f'{V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION}'
        )
    if parameters.get('allowed_visual_checkpoint_sha256') != (
        V4_TASK_EXECUTION_CHECKPOINT_SHA256
    ):
        raise ValueError(
            'execution_enabled requires the exact authorized V4 checkpoint '
            f'{V4_TASK_EXECUTION_CHECKPOINT_SHA256}'
        )

    raw_path = parameters.get('task_execution_authorization_path', '')
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise ValueError(
            'execution_enabled requires task_execution_authorization_path'
        )
    authorization_path = Path(raw_path).expanduser()
    if not authorization_path.is_absolute():
        raise ValueError(
            'task_execution_authorization_path must be absolute'
        )

    expected_sha256 = parameters.get(
        'task_execution_authorization_sha256',
        '',
    )
    if (
        not isinstance(expected_sha256, str)
        or _SHA256_PATTERN.fullmatch(expected_sha256) is None
    ):
        raise ValueError(
            'execution_enabled requires a lowercase '
            'task_execution_authorization_sha256'
        )

    raw = _read_immutable_regular_file(authorization_path)
    actual_sha256 = hashlib.sha256(raw).hexdigest()
    if actual_sha256 != expected_sha256:
        raise ValueError(
            'task execution authorization SHA-256 mismatch: '
            f'{actual_sha256} != {expected_sha256}'
        )
    authorization = _load_strict_json_object(raw)
    authorization_scope = authorization.get('authorization_scope')
    runtime_authorization = authorization_scope == (
        V4_TASK_EXECUTION_RUNTIME_AUTHORIZATION_SCOPE
    )
    if runtime_authorization:
        expected_state = 'active_closed_loop_runtime'
        expected_promotion_sha256 = authorization.get(
            'promotion_manifest_sha256'
        )
        if (
            not isinstance(expected_promotion_sha256, str)
            or _SHA256_PATTERN.fullmatch(expected_promotion_sha256) is None
        ):
            raise ValueError(
                'task execution authorization field mismatch: '
                'promotion_manifest_sha256'
            )
    else:
        # Retain the frozen qualification contract exactly.  An unsupported
        # scope is deliberately compared with this contract so it fails with
        # the same field-level evidence as before runtime support was added.
        authorization_scope = V4_TASK_EXECUTION_AUTHORIZATION_SCOPE
        expected_state = 'active_closed_loop_qualification'
        expected_promotion_sha256 = (
            V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
        )
    expected_contract: dict[str, object] = {
        'authorization_scope': authorization_scope,
        'automatic_promotion_allowed': False,
        'candidate_id': V4_TASK_EXECUTION_CANDIDATE_ID,
        'checkpoint_filename': 'checkpoint_epoch_011.pt',
        'checkpoint_sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
        'deployment_mode': 'active',
        'manual_review_approved': True,
        'physical_deployment_approved': False,
        'promotion_manifest_sha256': expected_promotion_sha256,
        'runtime_guards': V4_TASK_EXECUTION_RUNTIME_GUARDS,
        'schema_version': V4_TASK_EXECUTION_AUTHORIZATION_SCHEMA_VERSION,
        'state': expected_state,
    }
    _verify_exact_object(
        authorization,
        expected_contract,
        label='task execution authorization',
    )

    raw_manifest_path = parameters.get(
        'task_execution_promotion_manifest_path',
        '',
    )
    if not isinstance(raw_manifest_path, str) or not raw_manifest_path.strip():
        raise ValueError(
            'execution_enabled requires '
            'task_execution_promotion_manifest_path'
        )
    promotion_manifest_path = Path(raw_manifest_path).expanduser()
    if not promotion_manifest_path.is_absolute():
        raise ValueError(
            'task_execution_promotion_manifest_path must be absolute'
        )
    promotion_raw = _read_immutable_regular_file(
        promotion_manifest_path,
        label=(
            'V4 closed-loop runtime promotion manifest'
            if runtime_authorization
            else 'V4 closed-loop qualification promotion manifest'
        ),
    )
    promotion_sha256 = hashlib.sha256(promotion_raw).hexdigest()
    if promotion_sha256 != expected_promotion_sha256:
        raise ValueError(
            'V4 closed-loop promotion manifest SHA-256 '
            f'mismatch: {promotion_sha256} != '
            f'{expected_promotion_sha256}'
        )
    if authorization['promotion_manifest_sha256'] != promotion_sha256:
        raise ValueError(
            'task execution authorization is not bound to the opened '
            'promotion manifest'
        )
    promotion = _load_strict_json_object(
        promotion_raw,
        label=(
            'V4 closed-loop runtime promotion manifest'
            if runtime_authorization
            else 'V4 closed-loop qualification promotion manifest'
        ),
    )
    runtime_evidence: dict[str, str] = {}
    if runtime_authorization:
        runtime_evidence = _verify_runtime_manifest_evidence(
            promotion_manifest_path=promotion_manifest_path,
            promotion=promotion,
        )
        manual_decision_path, manual_decision_sha256 = (
            _verify_runtime_manifest_manual_decision(
                promotion_manifest_path=promotion_manifest_path,
                promotion=promotion,
                evidence=runtime_evidence,
            )
        )
    else:
        _verify_qualification_manifest(promotion)
        manual_decision_path, manual_decision_sha256 = (
            _verify_manifest_manual_decision(
                promotion_manifest_path=promotion_manifest_path,
                promotion=promotion,
            )
        )
    verified = {
        'path': str(authorization_path),
        'sha256': actual_sha256,
        'promotion_manifest_path': str(promotion_manifest_path),
        'visual_state_schema_version': (
            V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
        ),
        'visual_checkpoint_sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
        'promotion_manifest_sha256': promotion_sha256,
        'manual_decision_path': str(manual_decision_path),
        'manual_decision_sha256': manual_decision_sha256,
        'authorization_scope': authorization_scope,
    }
    verified.update(runtime_evidence)
    return verified


def _verify_qualification_manifest(promotion: dict[str, Any]) -> None:
    expected_fields = {
        'schema_version': V4_TASK_EXECUTION_PROMOTION_SCHEMA_VERSION,
        'immutable': True,
        'automatic_promotion_allowed': False,
        'manual_only': True,
        'manual_review_approved': True,
        'manual_runtime_review_status': 'approved',
        'deployment_mode': 'active',
    }
    _verify_required_fields(
        promotion,
        expected_fields,
        label='V4 closed-loop qualification promotion manifest',
    )
    qualification = _required_object(
        promotion,
        'closed_loop_qualification',
        label='V4 closed-loop qualification promotion manifest',
    )
    _verify_required_fields(
        qualification,
        {
            'qualification_only': True,
            'physical_deployment_approved': False,
            'campaign_report_path': None,
            'campaign_report_sha256': None,
        },
        label='closed_loop_qualification',
    )
    eligibility = _required_object(
        promotion,
        'eligibility',
        label='V4 closed-loop qualification promotion manifest',
    )
    _verify_required_fields(
        eligibility,
        {
            'active_runtime_review': True,
            'active_runtime_selected': True,
            'active_transition_requires_new_immutable_manifest': False,
        },
        label='V4 closed-loop qualification eligibility',
    )
    artifacts = _required_object(
        promotion,
        'artifacts',
        label='V4 closed-loop qualification promotion manifest',
    )
    checkpoint = _required_object(
        artifacts,
        'checkpoint',
        label='V4 closed-loop qualification artifacts',
    )
    _verify_required_fields(
        checkpoint,
        {
            'path': 'checkpoint_epoch_011.pt',
            'sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
        },
        label='V4 closed-loop qualification checkpoint',
    )
    model_contract = _required_object(
        promotion,
        'model_contract',
        label='V4 closed-loop qualification promotion manifest',
    )
    _verify_required_fields(
        model_contract,
        {
            'checkpoint_epoch': 11,
            'checkpoint_loading': 'strict',
            'checkpoint_schema_version': (
                'room315.visual_training.v4.checkpoint.v1'
            ),
            'model_kind': 'room315_visual_state_resnet18_split_rails_v4',
            'cross_camera_feature_path': False,
            'side_source': 'fixed_identity_prefix',
        },
        label='V4 closed-loop qualification model contract',
    )


def _verify_runtime_manifest_evidence(
    *,
    promotion_manifest_path: Path,
    promotion: dict[str, Any],
) -> dict[str, str]:
    """Verify the final runtime decision and every copied campaign binding."""

    label = 'V4 closed-loop runtime promotion manifest'
    _verify_required_fields(
        promotion,
        {
            'schema_version': V4_TASK_EXECUTION_PROMOTION_SCHEMA_VERSION,
            'immutable': True,
            'automatic_promotion_allowed': False,
            'manual_only': True,
            'manual_review_approved': True,
            'manual_runtime_review_status': 'approved',
            'deployment_mode': 'active',
            'source_active_manifest_sha256': (
                V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
            ),
        },
        label=label,
    )
    qualification = _required_object(
        promotion,
        'closed_loop_qualification',
        label=label,
    )
    _verify_required_fields(
        qualification,
        {
            'qualification_only': False,
            'physical_deployment_approved': False,
        },
        label='V4 closed-loop runtime qualification evidence',
    )
    eligibility = _required_object(promotion, 'eligibility', label=label)
    _verify_required_fields(
        eligibility,
        {
            'active_runtime_review': True,
            'active_runtime_selected': True,
            'active_transition_requires_new_immutable_manifest': False,
        },
        label='V4 closed-loop runtime eligibility',
    )
    artifacts = _required_object(promotion, 'artifacts', label=label)
    checkpoint = _required_object(
        artifacts,
        'checkpoint',
        label='V4 closed-loop runtime artifacts',
    )
    _verify_required_fields(
        checkpoint,
        {
            'path': 'checkpoint_epoch_011.pt',
            'sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
        },
        label='V4 closed-loop runtime checkpoint',
    )
    model_contract = _required_object(
        promotion,
        'model_contract',
        label=label,
    )
    _verify_required_fields(
        model_contract,
        {
            'checkpoint_epoch': 11,
            'checkpoint_loading': 'strict',
            'checkpoint_schema_version': (
                'room315.visual_training.v4.checkpoint.v1'
            ),
            'model_kind': 'room315_visual_state_resnet18_split_rails_v4',
            'cross_camera_feature_path': False,
            'side_source': 'fixed_identity_prefix',
        },
        label='V4 closed-loop runtime model contract',
    )

    campaign_path, campaign_sha256, campaign = _read_bound_json_artifact(
        promotion_manifest_path=promotion_manifest_path,
        raw_path=qualification.get('campaign_report_path'),
        expected_sha256=qualification.get('campaign_report_sha256'),
        label='V4 closed-loop campaign report',
    )
    _verify_closed_loop_campaign(campaign)

    fault_descriptor = _required_object(
        promotion,
        'closed_loop_fault_campaign',
        label=label,
    )
    expected_scenario_ids = [
        scenario_id
        for scenario_id, _fault in V4_TASK_EXECUTION_FAULT_SCENARIOS
    ]
    _verify_required_fields(
        fault_descriptor,
        {
            'schema_version': (
                V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION
            ),
            'scenario_ids': expected_scenario_ids,
            'passed_scenario_count': len(expected_scenario_ids),
            'failed_scenario_count': 0,
            'physical_deployment': False,
        },
        label='V4 closed-loop fault-campaign binding',
    )
    fault_report_path, fault_report_sha256, fault_report = (
        _read_bound_json_artifact(
            promotion_manifest_path=promotion_manifest_path,
            raw_path=fault_descriptor.get('report_path'),
            expected_sha256=fault_descriptor.get('report_sha256'),
            label='V4 closed-loop fault-campaign report',
        )
    )
    _verify_closed_loop_fault_campaign(fault_report)
    evidence_manifest_path, evidence_manifest_sha256, evidence_manifest = (
        _read_bound_json_artifact(
            promotion_manifest_path=promotion_manifest_path,
            raw_path=fault_descriptor.get('evidence_manifest_path'),
            expected_sha256=fault_descriptor.get(
                'evidence_manifest_sha256'
            ),
            label='V4 closed-loop fault-evidence manifest',
        )
    )
    source_sums_path, source_sums_sha256, source_sums_raw = (
        _read_bound_artifact(
            promotion_manifest_path=promotion_manifest_path,
            raw_path=fault_descriptor.get('source_sha256s_path'),
            expected_sha256=fault_descriptor.get('source_sha256s_sha256'),
            label='V4 closed-loop fault-evidence source SHA256SUMS',
        )
    )
    evidence_rows = _verify_fault_evidence_manifest(
        evidence_manifest,
        report_sha256=fault_report_sha256,
    )
    _verify_fault_evidence_sha256s(
        source_sums_raw,
        evidence_rows=evidence_rows,
        report_sha256=fault_report_sha256,
        manifest_sha256=evidence_manifest_sha256,
    )
    return {
        'closed_loop_campaign_report_path': str(campaign_path),
        'closed_loop_campaign_report_sha256': campaign_sha256,
        'fault_campaign_report_path': str(fault_report_path),
        'fault_campaign_report_sha256': fault_report_sha256,
        'fault_evidence_manifest_path': str(evidence_manifest_path),
        'fault_evidence_manifest_sha256': evidence_manifest_sha256,
        'fault_evidence_sha256s_path': str(source_sums_path),
        'fault_evidence_sha256s_sha256': source_sums_sha256,
    }


def _verify_closed_loop_campaign(campaign: dict[str, Any]) -> None:
    _verify_required_fields(
        campaign,
        {
            'schema_version': (
                V4_TASK_EXECUTION_CLOSED_LOOP_CAMPAIGN_SCHEMA_VERSION
            ),
            'status': 'passed',
            'visual_schema_version': (
                V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
            ),
            'checkpoint_sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
            'qualification_manifest_sha256': (
                V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
            ),
            'case_count': V4_TASK_EXECUTION_CLOSED_LOOP_CASE_COUNT,
            'passed_case_count': V4_TASK_EXECUTION_CLOSED_LOOP_CASE_COUNT,
            'failed_case_count': 0,
            'v3_observation_count': 0,
            'physical_deployment': False,
            'all_terminal_statuses_succeeded': True,
            'all_final_effects_verified': True,
            'all_controllers_stopped': True,
            'safe_abort_count': 0,
        },
        label='V4 closed-loop campaign report',
    )
    _require_positive_json_integer(
        campaign.get('v4_observation_count'),
        'V4 closed-loop campaign report v4_observation_count',
    )


def _verify_closed_loop_fault_campaign(campaign: dict[str, Any]) -> None:
    scenario_count = len(V4_TASK_EXECUTION_FAULT_SCENARIOS)
    _verify_required_fields(
        campaign,
        {
            'schema_version': V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION,
            'status': 'passed',
            'authorization_scope': V4_TASK_EXECUTION_AUTHORIZATION_SCOPE,
            'visual_schema_version': (
                V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
            ),
            'visual_checkpoint_sha256': (
                V4_TASK_EXECUTION_CHECKPOINT_SHA256
            ),
            'qualification_manifest_sha256': (
                V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
            ),
            'declared_scenario_count': scenario_count,
            'selected_scenario_count': scenario_count,
            'completed_scenario_count': scenario_count,
            'passed_scenario_count': scenario_count,
            'failed_scenario_count': 0,
            'v3_observation_count': 0,
            'all_false_success_counts_zero': True,
            'all_controllers_disabled': True,
            'physical_deployment': False,
            'failure_reason': '',
        },
        label='V4 closed-loop fault-campaign report',
    )
    _require_positive_json_integer(
        campaign.get('v4_observation_count'),
        'V4 closed-loop fault-campaign report v4_observation_count',
    )
    results = campaign.get('results')
    if not isinstance(results, list):
        raise ValueError(
            'V4 closed-loop fault-campaign report requires array field: '
            'results'
        )
    observed: list[tuple[str, str]] = []
    for index, result in enumerate(results):
        if not isinstance(result, dict):
            raise ValueError(
                'V4 closed-loop fault-campaign result must be an object: '
                f'{index}'
            )
        observed.append((
            str(result.get('scenario_id') or ''),
            str(result.get('fault') or ''),
        ))
        _verify_required_fields(
            result,
            {
                'status': 'passed',
                'visual_schema_version': (
                    V4_TASK_EXECUTION_VISUAL_SCHEMA_VERSION
                ),
                'checkpoint_sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
                'qualification_manifest_sha256': (
                    V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
                ),
                'false_success_count': 0,
                'v3_observation_count': 0,
                'controller_final_mode': 'DISABLED',
                'physical_deployment': False,
            },
            label=f'V4 closed-loop fault-campaign result {index}',
        )
        _require_positive_json_integer(
            result.get('v4_observation_count'),
            'V4 closed-loop fault-campaign result '
            f'{index} v4_observation_count',
        )
    if tuple(observed) != V4_TASK_EXECUTION_FAULT_SCENARIOS:
        raise ValueError(
            'V4 closed-loop fault campaign must contain the exact ordered '
            'F01--F05 scenarios'
        )


def _verify_fault_evidence_manifest(
    manifest: dict[str, Any],
    *,
    report_sha256: str,
) -> dict[str, str]:
    _verify_required_fields(
        manifest,
        {
            'schema_version': (
                V4_TASK_EXECUTION_FAULT_EVIDENCE_SCHEMA_VERSION
            ),
            'campaign_schema': (
                V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION
            ),
            'campaign_status': 'passed',
            'physical_deployment': False,
        },
        label='V4 closed-loop fault-evidence manifest',
    )
    files = manifest.get('files')
    if not isinstance(files, list):
        raise ValueError(
            'V4 closed-loop fault-evidence manifest requires array field: '
            'files'
        )
    count = manifest.get('file_count_excluding_manifest_and_checksums')
    if type(count) is not int or count != len(files):
        raise ValueError(
            'V4 closed-loop fault-evidence manifest field mismatch: '
            'file_count_excluding_manifest_and_checksums'
        )
    rows: dict[str, str] = {}
    for index, row in enumerate(files):
        if not isinstance(row, dict):
            raise ValueError(
                'V4 closed-loop fault-evidence manifest row must be an '
                f'object: {index}'
            )
        path = _safe_evidence_relative_path(
            row.get('path'),
            label=f'fault-evidence manifest row {index}',
        )
        sha256 = _required_sha256_value(
            row.get('sha256'),
            label=f'fault-evidence manifest row {index}',
        )
        size = row.get('size_bytes')
        if type(size) is not int or size < 0:
            raise ValueError(
                'V4 closed-loop fault-evidence manifest row has an invalid '
                f'size: {index}'
            )
        if path in rows:
            raise ValueError(
                'V4 closed-loop fault-evidence manifest contains duplicate '
                f'path: {path}'
            )
        rows[path] = sha256
    if rows.get('summary.json') != report_sha256:
        raise ValueError(
            'V4 closed-loop fault-evidence manifest does not bind the '
            'fault-campaign report'
        )
    return rows


def _verify_fault_evidence_sha256s(
    raw: bytes,
    *,
    evidence_rows: dict[str, str],
    report_sha256: str,
    manifest_sha256: str,
) -> None:
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise ValueError(
            'V4 closed-loop fault-evidence source SHA256SUMS must be UTF-8'
        ) from exc
    declared: dict[str, str] = {}
    for line in text.splitlines():
        pieces = line.split('  ', 1)
        if len(pieces) != 2:
            raise ValueError(
                'V4 closed-loop fault-evidence source SHA256SUMS is malformed'
            )
        sha256 = _required_sha256_value(
            pieces[0],
            label='fault-evidence source SHA256SUMS row',
        )
        path = _safe_evidence_relative_path(
            pieces[1],
            label='fault-evidence source SHA256SUMS row',
        )
        if path in declared:
            raise ValueError(
                'V4 closed-loop fault-evidence source SHA256SUMS contains '
                f'duplicate path: {path}'
            )
        declared[path] = sha256
    if not declared:
        raise ValueError(
            'V4 closed-loop fault-evidence source SHA256SUMS is empty'
        )
    if declared.get('summary.json') != report_sha256:
        raise ValueError(
            'V4 closed-loop fault-evidence source SHA256SUMS does not bind '
            'the fault-campaign report'
        )
    if declared.get('manifest.json') != manifest_sha256:
        raise ValueError(
            'V4 closed-loop fault-evidence source SHA256SUMS does not bind '
            'the evidence manifest'
        )
    if {
        name: digest
        for name, digest in declared.items()
        if name != 'manifest.json'
    } != evidence_rows:
        raise ValueError(
            'V4 closed-loop fault-evidence manifest/SHA256SUMS payload set '
            'mismatch'
        )


def _verify_manifest_manual_decision(
    *,
    promotion_manifest_path: Path,
    promotion: dict[str, Any],
) -> tuple[Path, str]:
    descriptor = _required_object(
        promotion,
        'manual_decision_record',
        label='V4 closed-loop qualification promotion manifest',
    )
    _verify_required_fields(
        descriptor,
        {'schema_version': V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION},
        label='V4 closed-loop qualification manual-decision descriptor',
    )
    relative_path = descriptor.get('path')
    if (
        not isinstance(relative_path, str)
        or Path(relative_path).is_absolute()
        or Path(relative_path).name != relative_path
    ):
        raise ValueError(
            'manual decision record path must be one relative basename'
        )
    expected_sha256 = descriptor.get('sha256')
    if (
        not isinstance(expected_sha256, str)
        or _SHA256_PATTERN.fullmatch(expected_sha256) is None
    ):
        raise ValueError(
            'manual decision record descriptor requires a lowercase SHA-256'
        )
    manual_path = promotion_manifest_path.parent / relative_path
    manual_raw = _read_immutable_regular_file(
        manual_path,
        label='V4 closed-loop qualification manual decision',
    )
    actual_sha256 = hashlib.sha256(manual_raw).hexdigest()
    if actual_sha256 != expected_sha256:
        raise ValueError(
            'V4 closed-loop qualification manual decision SHA-256 mismatch: '
            f'{actual_sha256} != {expected_sha256}'
        )
    manual = _load_strict_json_object(
        manual_raw,
        label='V4 closed-loop qualification manual decision',
    )
    _verify_required_fields(
        manual,
        {
            'schema_version': (
                V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION
            ),
            'scope': V4_TASK_EXECUTION_AUTHORIZATION_SCOPE,
            'candidate_id': V4_TASK_EXECUTION_CANDIDATE_ID,
            'checkpoint_sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
            'decision': 'approved',
            'qualification_only': True,
            'physical_deployment_approved': False,
            'automatic_promotion': False,
            'source_active_manifest_sha256': (
                V4_TASK_EXECUTION_SOURCE_ACTIVE_MANIFEST_SHA256
            ),
            'runtime_guards': V4_TASK_EXECUTION_RUNTIME_GUARDS,
            'closed_loop_campaign': None,
        },
        label='V4 closed-loop qualification manual decision',
    )
    return manual_path, actual_sha256


def _verify_runtime_manifest_manual_decision(
    *,
    promotion_manifest_path: Path,
    promotion: dict[str, Any],
    evidence: dict[str, str],
) -> tuple[Path, str]:
    label = 'V4 closed-loop runtime promotion manifest'
    descriptor = _required_object(
        promotion,
        'manual_decision_record',
        label=label,
    )
    _verify_required_fields(
        descriptor,
        {'schema_version': V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION},
        label='V4 closed-loop runtime manual-decision descriptor',
    )
    manual_path, manual_sha256, manual_raw = _read_bound_artifact(
        promotion_manifest_path=promotion_manifest_path,
        raw_path=descriptor.get('path'),
        expected_sha256=descriptor.get('sha256'),
        label='V4 closed-loop runtime manual decision',
        max_bytes=MAX_TASK_EXECUTION_AUTHORIZATION_BYTES,
    )
    manual = _load_strict_json_object(
        manual_raw,
        label='V4 closed-loop runtime manual decision',
    )
    _verify_required_fields(
        manual,
        {
            'schema_version': (
                V4_TASK_EXECUTION_MANUAL_DECISION_SCHEMA_VERSION
            ),
            'scope': V4_TASK_EXECUTION_RUNTIME_AUTHORIZATION_SCOPE,
            'candidate_id': V4_TASK_EXECUTION_CANDIDATE_ID,
            'checkpoint_sha256': V4_TASK_EXECUTION_CHECKPOINT_SHA256,
            'decision': 'approved',
            'qualification_only': False,
            'physical_deployment_approved': False,
            'automatic_promotion': False,
            'source_active_manifest_sha256': (
                V4_TASK_EXECUTION_PROMOTION_MANIFEST_SHA256
            ),
            'runtime_guards': V4_TASK_EXECUTION_RUNTIME_GUARDS,
        },
        label='V4 closed-loop runtime manual decision',
    )
    for name in ('reviewer', 'reviewed_at_utc'):
        value = manual.get(name)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                'V4 closed-loop runtime manual decision requires non-empty '
                f'field: {name}'
            )
    campaign = _required_object(
        manual,
        'closed_loop_campaign',
        label='V4 closed-loop runtime manual decision',
    )
    _verify_required_fields(
        campaign,
        {
            'schema_version': (
                V4_TASK_EXECUTION_CLOSED_LOOP_CAMPAIGN_SCHEMA_VERSION
            ),
            'sha256': evidence['closed_loop_campaign_report_sha256'],
            'case_count': V4_TASK_EXECUTION_CLOSED_LOOP_CASE_COUNT,
            'passed_case_count': V4_TASK_EXECUTION_CLOSED_LOOP_CASE_COUNT,
            'failed_case_count': 0,
        },
        label='V4 closed-loop runtime manual campaign decision',
    )
    fault = _required_object(
        manual,
        'closed_loop_fault_campaign',
        label='V4 closed-loop runtime manual decision',
    )
    scenario_ids = [
        scenario_id
        for scenario_id, _fault in V4_TASK_EXECUTION_FAULT_SCENARIOS
    ]
    _verify_required_fields(
        fault,
        {
            'schema_version': V4_TASK_EXECUTION_FAULT_CAMPAIGN_SCHEMA_VERSION,
            'sha256': evidence['fault_campaign_report_sha256'],
            'scenario_ids': scenario_ids,
            'passed_scenario_count': len(scenario_ids),
            'failed_scenario_count': 0,
            'evidence_manifest_sha256': (
                evidence['fault_evidence_manifest_sha256']
            ),
            'evidence_sha256s_sha256': (
                evidence['fault_evidence_sha256s_sha256']
            ),
        },
        label='V4 closed-loop runtime manual fault-campaign decision',
    )
    assertions = _required_object(
        manual,
        'review_assertions',
        label='V4 closed-loop runtime manual decision',
    )
    _verify_required_fields(
        assertions,
        {
            'v4_is_authoritative_visual_observation': True,
            'plansys2_problem_built_from_v4_observed_state': True,
            'optional_problem_expert_predicate_mirror_disabled': True,
            'supervisor_and_dzi_safety_gates_required': True,
        },
        label='V4 closed-loop runtime manual review assertions',
    )
    return manual_path, manual_sha256


def _read_bound_json_artifact(
    *,
    promotion_manifest_path: Path,
    raw_path: Any,
    expected_sha256: Any,
    label: str,
) -> tuple[Path, str, dict[str, Any]]:
    path, sha256, raw = _read_bound_artifact(
        promotion_manifest_path=promotion_manifest_path,
        raw_path=raw_path,
        expected_sha256=expected_sha256,
        label=label,
    )
    return path, sha256, _load_strict_json_object(raw, label=label)


def _read_bound_artifact(
    *,
    promotion_manifest_path: Path,
    raw_path: Any,
    expected_sha256: Any,
    label: str,
    max_bytes: int = MAX_TASK_EXECUTION_EVIDENCE_BYTES,
) -> tuple[Path, str, bytes]:
    if (
        not isinstance(raw_path, str)
        or not raw_path
        or raw_path.strip() != raw_path
        or Path(raw_path).is_absolute()
        or Path(raw_path).name != raw_path
    ):
        raise ValueError(f'{label} path must be one relative basename')
    expected = _required_sha256_value(expected_sha256, label=label)
    path = promotion_manifest_path.parent / raw_path
    raw = _read_immutable_regular_file(
        path,
        label=label,
        max_bytes=max_bytes,
    )
    actual = hashlib.sha256(raw).hexdigest()
    if actual != expected:
        raise ValueError(
            f'{label} SHA-256 mismatch: {actual} != {expected}'
        )
    return path, actual, raw


def _required_sha256_value(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or _SHA256_PATTERN.fullmatch(value) is None:
        raise ValueError(f'{label} requires a lowercase SHA-256')
    return value


def _safe_evidence_relative_path(value: Any, *, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value.strip() != value
        or '\\' in value
    ):
        raise ValueError(f'{label} path is unsafe')
    path = Path(value)
    if (
        path.is_absolute()
        or not path.parts
        or any(part in {'.', '..'} for part in path.parts)
    ):
        raise ValueError(f'{label} path is unsafe')
    return path.as_posix()


def _require_positive_json_integer(value: Any, label: str) -> int:
    if type(value) is not int or value < 1:
        raise ValueError(f'{label} must be a positive JSON integer')
    return value


def _verify_exact_object(
    value: dict[str, Any],
    expected: dict[str, object],
    *,
    label: str,
) -> None:
    if set(value) != set(expected):
        missing = sorted(set(expected) - set(value))
        unexpected = sorted(set(value) - set(expected))
        raise ValueError(
            f'{label} has an incompatible field set: '
            f'missing={missing}, unexpected={unexpected}'
        )
    _verify_required_fields(value, expected, label=label)


def _verify_required_fields(
    value: dict[str, Any],
    expected: dict[str, object],
    *,
    label: str,
) -> None:
    for name, expected_value in expected.items():
        if name not in value:
            raise ValueError(f'{label} is missing field: {name}')
        actual = value[name]
        if type(actual) is not type(expected_value) or actual != expected_value:
            raise ValueError(f'{label} field mismatch: {name}')


def _required_object(
    value: dict[str, Any],
    name: str,
    *,
    label: str,
) -> dict[str, Any]:
    child = value.get(name)
    if not isinstance(child, dict):
        raise ValueError(f'{label} requires object field: {name}')
    return child


def _read_immutable_regular_file(
    path: Path,
    *,
    label: str = 'task execution authorization',
    max_bytes: int = MAX_TASK_EXECUTION_AUTHORIZATION_BYTES,
) -> bytes:
    """Read one non-writable regular file without following a final symlink."""

    flags = os.O_RDONLY | getattr(os, 'O_CLOEXEC', 0)
    flags |= getattr(os, 'O_NOFOLLOW', 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise ValueError(f'{label} cannot be opened: {path}') from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError(
                f'{label} must be a regular file'
            )
        if stat.S_IMODE(metadata.st_mode) & 0o222:
            raise ValueError(
                f'{label} must have no write bits'
            )
        if metadata.st_size <= 0:
            raise ValueError(f'{label} must not be empty')
        if metadata.st_size > max_bytes:
            raise ValueError(f'{label} is too large')
        with os.fdopen(descriptor, 'rb', closefd=False) as stream:
            raw = stream.read(max_bytes + 1)
        if len(raw) != metadata.st_size:
            raise ValueError(
                f'{label} changed while being read'
            )
        return raw
    finally:
        os.close(descriptor)


def _load_strict_json_object(
    raw: bytes,
    *,
    label: str = 'task execution authorization',
) -> dict[str, Any]:
    """Decode authorization JSON while rejecting duplicates and NaN values."""

    def reject_constant(value: str) -> None:
        raise ValueError(
            f'{label} contains invalid JSON value {value}'
        )

    def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for name, item in pairs:
            if name in value:
                raise ValueError(
                    f'{label} contains duplicate key '
                    f'{name}'
                )
            value[name] = item
        return value

    try:
        decoded = raw.decode('utf-8')
        value = json.loads(
            decoded,
            object_pairs_hook=unique_object,
            parse_constant=reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(
            f'{label} must be valid UTF-8 JSON'
        ) from exc
    if not isinstance(value, dict):
        raise ValueError(f'{label} must contain an object')
    return value
