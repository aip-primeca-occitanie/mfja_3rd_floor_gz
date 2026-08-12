import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from room_315_presence_provider import (  # noqa: E402
    PRESENCE_ABSENT,
    PRESENCE_PRESENT,
    PresenceEntry,
    PresenceSnapshot,
)
from room_315_visual_contract_v4 import (  # noqa: E402
    SEGMENT_CLASSES,
    SIDES,
    load_authoritative_public_segment_length_contract,
)
from room_315_visual_model_v4 import (  # noqa: E402
    V4_MODEL_KIND,
    V4_OUTPUT_KEYS,
    V4_SLOT_ORDER,
)
import room_315_visual_runtime_v4 as runtime_v4  # noqa: E402
from room_315_visual_runtime_v4 import (  # noqa: E402
    CHECKPOINT_SCHEMA_VERSION,
    CLOSED_LOOP_QUALIFICATION_SCOPE,
    DRY_RUN_AUTHORIZATION_SCOPE,
    MANUAL_DECISION_SCHEMA_VERSION,
    PROMOTION_SCHEMA_VERSION,
    Room315VisualModelRuntimeV4,
    StructuredVisualOutputV4,
    VisualRuntimeV4Error,
    build_diagnostic_legacy_output_v4,
    decode_active_slots_v4,
    preprocess_paired_rgb_v4,
    verify_v4_runtime_promotion,
)


def _canonical_json(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')


def _topology_json():
    topology = load_authoritative_public_segment_length_contract()
    result = topology.canonical_metadata()
    result.update({
        'segment_name_domain': 'public_ros_label',
        'loader': 'room_315_rail_defaults.public_rail_segment_lengths',
        'lengths_by_side': {
            side: dict(topology.as_mapping()[side])
            for side in SIDES
        },
    })
    return result


def _manifest_fixture(tmp_path, *, segment_threshold=0.8, loaded_threshold=0.8):
    checkpoint_path = tmp_path / 'epoch_011.pt'
    checkpoint_path.write_bytes(b'checkpoint bytes are metadata-only in this fixture')
    topology = _topology_json()
    effective_config = {
        'schema_version': 'room315.visual_training.v4.pilot_config.v1',
        'data': {},
        'data_roles': {'checkpoint_selection': 'validation_only'},
        'model': {
            'kind': V4_MODEL_KIND,
            'head_type': 'spatial_query',
            'hidden_dim': 256,
            'attention_heads': 8,
            'dropout': 0.1,
            'slot_order': list(V4_SLOT_ORDER),
            'cross_camera_feature_path': False,
        },
        'image_preprocessing': {
            'width': 320,
            'height': 240,
            'resize': 'aspect_preserving_bilinear_4_by_3',
            'normalization_mean': [0.485, 0.456, 0.406],
            'normalization_std': [0.229, 0.224, 0.225],
            'train_augmentations': {
                'horizontal_flip': False,
                'camera_swap': False,
            },
        },
        'topology_contract': {
            'segment_name_domain': 'public_ros_label',
            'length_loader': 'room_315_rail_defaults.public_rail_segment_lengths',
            'forbid_internal_rail_segment_lengths': True,
            'verify_length_mapping_fingerprint_at_runtime': True,
        },
    }
    effective_fingerprint = hashlib.sha256(
        _canonical_json(effective_config).encode('utf-8')
    ).hexdigest()
    calibration = {
        'schema_version': 'room315.visual_segment_calibration.v4.v1',
        'fit_scope': 'validation_only',
        'data_role': 'validation',
        'temperature': 0.6007370031399095,
    }
    checkpoint_sha = _sha(checkpoint_path)
    validation_acceptance = {
        'schema_version': 'room315.visual_acceptance.v4.v1',
        'accepted': True,
        'status': 'passed',
        'summary': {'gate_count': 2, 'passed': 2, 'failed': 0, 'pending': 0},
    }
    training_report = {
        'schema_version': 'room315.visual_training.v4.run.v1',
        'status': 'completed',
        'validation_acceptance_status': 'passed',
        'validation_acceptance': validation_acceptance,
        'checkpoint_selection': {'role': 'validation_only'},
        'selected_checkpoint': {
            'epoch': 11,
            'filename': 'epoch_011.pt',
            'sha256': checkpoint_sha,
        },
        'canary_used_for_selection': False,
        'canary_loaded': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
        'effective_config_sha256': effective_fingerprint,
        'validation_segment_calibration': calibration,
        'public_topology_contract': topology,
        'topology_length_mapping_fingerprint_sha256': topology['fingerprint_sha256'],
        'config_file_sha256': '1' * 64,
    }
    attempt_key = '2' * 64
    reservation_sha = '3' * 64
    canary_report = {
        'schema_version': 'room315.visual_training.v4.run.v1',
        'status': 'completed',
        'acceptance_status': 'passed',
        'acceptance': {
            'accepted': True,
            'summary': {'gate_count': 2, 'passed': 2, 'failed': 0, 'pending': 0},
        },
        'promotion_status': 'eligible_for_manual_runtime_review',
        'prior_exposure_acknowledged': True,
        'canary_loaded': True,
        'checkpoint_selection_performed': False,
        'canary_used_for_checkpoint_selection': False,
        'test_loaded': False,
        'automatic_runtime_switch': False,
        'calibration_refit_performed': False,
        'calibration_temperature_source': 'validation_only',
        'segment_calibration': {
            'source_temperature_role': 'validation',
            'temperature': calibration['temperature'],
        },
        'checkpoint': {
            'sha256': checkpoint_sha,
            'source_selected_epoch': 11,
            'validation_selected': True,
        },
        'public_topology_contract': topology,
        'topology_length_mapping_fingerprint_sha256': topology['fingerprint_sha256'],
        'canary_attempt': {
            'attempt_key': attempt_key,
            'one_shot': True,
            'completion_ledger_required_for_trust': True,
            'reservation_sha256': reservation_sha,
        },
    }

    paths = {
        'training_final_report': tmp_path / 'training_final_report.json',
        'validation_acceptance': tmp_path / 'validation_acceptance.json',
        'canary_final_report': tmp_path / 'canary_final_report.json',
        'canary_completion_ledger': tmp_path / 'canary_completion_ledger.json',
        'effective_config': tmp_path / 'effective_config.json',
        'validation_segment_calibration': tmp_path / 'validation_calibration.json',
        'public_topology_contract': tmp_path / 'public_topology_contract.json',
    }
    _write_json(paths['training_final_report'], training_report)
    _write_json(paths['validation_acceptance'], validation_acceptance)
    _write_json(paths['canary_final_report'], canary_report)
    _write_json(paths['effective_config'], effective_config)
    _write_json(paths['validation_segment_calibration'], calibration)
    _write_json(paths['public_topology_contract'], topology)
    ledger = {
        'schema_version': 'room315.visual_v4.canary_attempt.v1',
        'state': 'completed_immutable',
        'attempt_key': attempt_key,
        'artifacts': {
            'final_report.json': {'sha256': _sha(paths['canary_final_report'])},
        },
        'reservation': {'sha256': reservation_sha},
        'test_loaded': False,
        'canary_used_for_checkpoint_selection': False,
        'automatic_runtime_switch': False,
    }
    _write_json(paths['canary_completion_ledger'], ledger)
    artifact_paths = {'checkpoint': checkpoint_path, **paths}
    manifest = {
        'schema_version': PROMOTION_SCHEMA_VERSION,
        'immutable': True,
        'manual_review_approved': False,
        'manual_runtime_review_status': 'pending',
        'shadow_execution_authorized': True,
        'automatic_promotion_allowed': False,
        'deployment_mode': 'shadow',
        'artifacts': {
            name: {'path': str(path), 'sha256': _sha(path)}
            for name, path in artifact_paths.items()
        },
        'model_contract': {
            'checkpoint_schema_version': CHECKPOINT_SCHEMA_VERSION,
            'model_kind': V4_MODEL_KIND,
            'head_type': 'spatial_query',
            'hidden_dim': 256,
            'attention_heads': 8,
            'dropout': 0.1,
            'slot_order': list(V4_SLOT_ORDER),
            'segment_order': list(SEGMENT_CLASSES),
            'output_keys': list(V4_OUTPUT_KEYS),
            'checkpoint_epoch': 11,
            'checkpoint_loading': 'strict',
            'effective_config_fingerprint_sha256': effective_fingerprint,
        },
        'preprocessing_contract': {
            'camera_order': ['left_rail_rgb', 'right_rail_rgb'],
            'channel_order': ['left_rgb', 'right_rgb'],
            'input_shape': ['B', 6, 240, 320],
            'width': 320,
            'height': 240,
            'resize': 'aspect_preserving_bilinear_4_by_3',
            'resampling': 'bilinear',
            'normalization_mean': [0.485, 0.456, 0.406],
            'normalization_std': [0.229, 0.224, 0.225],
            'runtime_augmentations': False,
        },
        'topology_contract': {
            'schema_version': topology['schema_version'],
            'source': topology['source'],
            'segment_name_domain': topology['segment_name_domain'],
            'fingerprint_sha256': topology['fingerprint_sha256'],
            'side_order': topology['side_order'],
            'segment_order': topology['segment_order'],
        },
        'calibration_contract': calibration,
        'acceptance_thresholds': {
            'minimum_segment_confidence': segment_threshold,
            'minimum_loaded_confidence': loaded_threshold,
            'active_frame_policy': 'all_present_slots_must_pass',
        },
    }
    manifest_path = tmp_path / 'promotion_manifest.json'
    _write_json(manifest_path, manifest)
    promotion = verify_v4_runtime_promotion(manifest_path, _sha(manifest_path))
    return promotion, artifact_paths, topology


def _presence(present=('L1', 'R1')):
    return PresenceSnapshot(
        timestamp_s=10.0,
        ready=True,
        entries=tuple(
            PresenceEntry(
                identity=identity,
                side='left' if identity.startswith('L') else 'right',
                state=PRESENCE_PRESENT if identity in present else PRESENCE_ABSENT,
            )
            for identity in V4_SLOT_ORDER
        ),
        reasons=(),
        initialized_sides=('left', 'right'),
        stale_sides=(),
        source='synthetic_registry',
    )


def _activate_fixture(promotion, *, scope, guards):
    manifest_path = promotion.manifest_path
    manifest = json.loads(manifest_path.read_text())
    checkpoint_sha = manifest['artifacts']['checkpoint']['sha256']
    decision_path = manifest_path.parent / 'manual_decision_record.json'
    _write_json(decision_path, {
        'schema_version': MANUAL_DECISION_SCHEMA_VERSION,
        'candidate_id': 'fixture-v4',
        'checkpoint_sha256': checkpoint_sha,
        'decision': 'approved',
        'reviewer': 'fixture reviewer',
        'reviewed_at_utc': '2026-08-12T00:00:00Z',
        'scope': scope,
        'automatic_promotion': False,
        'physical_deployment_approved': False,
        'runtime_guards': guards,
    })
    manifest.update({
        'candidate_id': 'fixture-v4',
        'deployment_mode': 'active',
        'manual_review_approved': True,
        'manual_runtime_review_status': 'approved',
        'shadow_execution_authorized': False,
        'manual_decision_record': {
            'path': str(decision_path),
            'sha256': _sha(decision_path),
            'schema_version': MANUAL_DECISION_SCHEMA_VERSION,
        },
    })
    _write_json(manifest_path, manifest)
    return verify_v4_runtime_promotion(manifest_path, _sha(manifest_path))


def _confident_output(segment_index=3, loaded_index=1):
    segment_logits = np.full((8, 14), -8.0, dtype=np.float32)
    loaded_logits = np.full((8, 2), -8.0, dtype=np.float32)
    segment_logits[:, segment_index] = 8.0
    loaded_logits[:, loaded_index] = 8.0
    return StructuredVisualOutputV4(
        segment_logits=segment_logits,
        loaded_logits=loaded_logits,
        bbox=np.tile(
            np.asarray([0.1, 0.2, 0.3, 0.4], dtype=np.float32),
            (8, 1),
        ),
        s_ratio=np.full((8,), 0.5, dtype=np.float32),
    )


def test_manifest_verification_cross_binds_all_pre_torch_evidence(tmp_path):
    promotion, artifacts, topology = _manifest_fixture(tmp_path)

    assert promotion.deployment_mode == 'shadow'
    assert promotion.model_kind == V4_MODEL_KIND
    assert promotion.checkpoint_sha256 == _sha(artifacts['checkpoint'])
    assert promotion.checkpoint_epoch == 11
    assert promotion.segment_temperature == pytest.approx(0.6007370031399095)
    assert promotion.topology.fingerprint_sha256 == topology['fingerprint_sha256']


def test_manifest_and_artifact_tampering_fail_closed(tmp_path):
    promotion, artifacts, _ = _manifest_fixture(tmp_path)
    manifest_path = promotion.manifest_path
    pinned_manifest_sha = promotion.manifest_sha256
    manifest = json.loads(manifest_path.read_text())
    manifest['deployment_mode'] = 'active'
    _write_json(manifest_path, manifest)
    with pytest.raises(VisualRuntimeV4Error, match='manifest SHA-256 mismatch'):
        verify_v4_runtime_promotion(manifest_path, pinned_manifest_sha)

    # Restore the manifest bytes, then tamper with a separately hashed artifact.
    manifest['deployment_mode'] = 'shadow'
    _write_json(manifest_path, manifest)
    calibration_path = artifacts['validation_segment_calibration']
    calibration_path.write_text('{}\n', encoding='utf-8')
    with pytest.raises(VisualRuntimeV4Error, match='artifact SHA-256 mismatch'):
        verify_v4_runtime_promotion(manifest_path, _sha(manifest_path))


def test_shadow_manifest_cannot_claim_pending_active_execution(tmp_path):
    promotion, _, _ = _manifest_fixture(tmp_path)
    manifest = json.loads(promotion.manifest_path.read_text())
    manifest['deployment_mode'] = 'active'
    _write_json(promotion.manifest_path, manifest)

    with pytest.raises(VisualRuntimeV4Error, match='approved manual runtime review'):
        verify_v4_runtime_promotion(
            promotion.manifest_path,
            _sha(promotion.manifest_path),
        )


@pytest.mark.parametrize(
    ('scope', 'guards'),
    [
        (
            DRY_RUN_AUTHORIZATION_SCOPE,
            {
                'dry_run_state_fusion': True,
                'plansys2_update_enabled': False,
                'actuation_enabled': False,
            },
        ),
        (
            CLOSED_LOOP_QUALIFICATION_SCOPE,
            {
                'dry_run_state_fusion': True,
                'plansys2_update_enabled': False,
                'actuation_enabled': True,
            },
        ),
    ],
)
def test_active_manifest_binds_exact_manual_scope_and_guards(
    tmp_path,
    scope,
    guards,
):
    promotion, _, _ = _manifest_fixture(tmp_path)
    active = _activate_fixture(promotion, scope=scope, guards=guards)

    assert active.authorization_scope == scope
    assert dict(active.runtime_guards) == guards


def test_active_manifest_rejects_guard_scope_mismatch(tmp_path):
    promotion, _, _ = _manifest_fixture(tmp_path)
    with pytest.raises(
        VisualRuntimeV4Error,
        match='runtime guards do not match',
    ):
        _activate_fixture(
            promotion,
            scope=DRY_RUN_AUTHORIZATION_SCOPE,
            guards={
                'dry_run_state_fusion': True,
                'plansys2_update_enabled': False,
                'actuation_enabled': True,
            },
        )


def test_preprocessing_is_fixed_left_right_320x240_bilinear_imagenet():
    left = np.zeros((6, 8, 3), dtype=np.uint8)
    right = np.zeros((6, 8, 3), dtype=np.uint8)
    left[..., 0] = 255
    right[..., 2] = 255

    paired = preprocess_paired_rgb_v4(left, right)

    assert paired.shape == (6, 240, 320)
    assert paired.dtype == np.float32
    expected_left = (np.asarray([1.0, 0.0, 0.0]) - [0.485, 0.456, 0.406]) / [
        0.229, 0.224, 0.225,
    ]
    expected_right = (np.asarray([0.0, 0.0, 1.0]) - [0.485, 0.456, 0.406]) / [
        0.229, 0.224, 0.225,
    ]
    assert paired[:3, 100, 100] == pytest.approx(expected_left, abs=1e-6)
    assert paired[3:, 100, 100] == pytest.approx(expected_right, abs=1e-6)
    with pytest.raises(VisualRuntimeV4Error, match='must be uint8'):
        preprocess_paired_rgb_v4(left.astype(np.float32), right)
    with pytest.raises(VisualRuntimeV4Error, match='4:3 aspect ratio'):
        preprocess_paired_rgb_v4(np.zeros((5, 8, 3), dtype=np.uint8), right)


def test_decode_uses_public_topology_confidence_and_original_pixels(tmp_path):
    promotion, _, _ = _manifest_fixture(tmp_path)
    output = _confident_output(segment_index=3, loaded_index=1)

    diagnostic = build_diagnostic_legacy_output_v4(
        output,
        promotion=promotion,
        presence=_presence(),
        left_image_size=(640, 480),
        right_image_size=(800, 600),
    )
    prediction = decode_active_slots_v4(
        output,
        promotion=promotion,
        presence=_presence(),
        timestamp_s=10.0,
        left_image_stamp_s=9.9,
        right_image_stamp_s=9.9,
        left_image_size=(640, 480),
        right_image_size=(800, 600),
    )

    assert diagnostic.control_input_permitted is False
    assert len(diagnostic.legacy_vector) == 200
    by_acceptance = {slot.identity: slot for slot in diagnostic.acceptance.slots}
    assert by_acceptance['L1'].required and by_acceptance['L1'].accepted
    assert not by_acceptance['L2'].required and not by_acceptance['L2'].accepted
    assert by_acceptance['L2'].reasons == ('presence_absent_not_evaluated',)
    assert prediction.active_identities == ('L1', 'R1')
    assert prediction.absent_identities == ('L2', 'L3', 'L4', 'R2', 'R3', 'R4')
    by_identity = {item.identity: item for item in prediction.shuttles}
    left = by_identity['L1']
    right = by_identity['R1']
    assert left.side == 'left' and right.side == 'right'
    assert left.block == SEGMENT_CLASSES[3]
    assert left.loaded_state == 'loaded'
    assert left.bbox_xywh == pytest.approx((64.0, 96.0, 192.0, 192.0))
    assert right.bbox_xywh == pytest.approx((80.0, 120.0, 240.0, 240.0))
    expected_length = promotion.topology.length_m('left', SEGMENT_CLASSES[3])
    assert left.segment_length_m == pytest.approx(expected_length)
    assert left.s_m == pytest.approx(0.5 * expected_length)
    assert left.segment_confidence >= 0.99
    assert left.loaded_confidence >= 0.99


def test_any_low_confidence_present_slot_rejects_whole_active_frame(tmp_path):
    promotion, _, _ = _manifest_fixture(tmp_path)
    output = _confident_output()
    output.segment_logits[0] = 0.0

    diagnostic = build_diagnostic_legacy_output_v4(
        output,
        promotion=promotion,
        presence=_presence(),
        left_image_size=(640, 480),
        right_image_size=(640, 480),
    )
    assert not diagnostic.acceptance.accepted
    assert diagnostic.acceptance.reasons == (
        'L1:segment_confidence_below_threshold',
    )
    with pytest.raises(VisualRuntimeV4Error, match='V4 active frame rejected.*L1'):
        decode_active_slots_v4(
            output,
            promotion=promotion,
            presence=_presence(),
            timestamp_s=10.0,
            left_image_stamp_s=10.0,
            right_image_stamp_s=10.0,
            left_image_size=(640, 480),
            right_image_size=(640, 480),
        )


def test_absent_low_confidence_slot_does_not_poison_active_frame(tmp_path):
    promotion, _, _ = _manifest_fixture(tmp_path)
    output = _confident_output()
    output.segment_logits[1] = 0.0  # L2 is explicitly absent.
    output.loaded_logits[1] = 0.0

    prediction = decode_active_slots_v4(
        output,
        promotion=promotion,
        presence=_presence(),
        timestamp_s=10.0,
        left_image_stamp_s=10.0,
        right_image_stamp_s=10.0,
        left_image_size=(640, 480),
        right_image_size=(640, 480),
    )

    assert prediction.active_identities == ('L1', 'R1')


@pytest.mark.parametrize(
    'mutation',
    ['nan_logits', 'invalid_bbox', 'invalid_ratio'],
)
def test_nonfinite_and_invalid_structured_outputs_fail_closed(tmp_path, mutation):
    promotion, _, _ = _manifest_fixture(tmp_path)
    output = _confident_output()
    if mutation == 'nan_logits':
        output.segment_logits[0, 0] = np.nan
    elif mutation == 'invalid_bbox':
        output.bbox[0] = [0.8, 0.2, 0.3, 0.4]
    else:
        output.s_ratio[0] = 1.1

    with pytest.raises(VisualRuntimeV4Error):
        build_diagnostic_legacy_output_v4(
            output,
            promotion=promotion,
            presence=_presence(),
            left_image_size=(640, 480),
            right_image_size=(640, 480),
        )


def test_loader_requires_checkpoint_metadata_and_strict_state_load(
    tmp_path,
    monkeypatch,
):
    torch = pytest.importorskip('torch')
    promotion, _, topology = _manifest_fixture(tmp_path)
    checkpoint = {
        'schema_version': CHECKPOINT_SCHEMA_VERSION,
        'model_kind': V4_MODEL_KIND,
        'slot_order': list(V4_SLOT_ORDER),
        'segment_order': list(SEGMENT_CLASSES),
        'epoch': 11,
        'checkpoint_selection_role': 'validation_only',
        'canary_seen': False,
        'test_seen': False,
        'effective_config_sha256': promotion.effective_config_fingerprint_sha256,
        'topology_length_mapping_fingerprint_sha256': topology['fingerprint_sha256'],
        'public_topology_contract': topology,
        'topology_lengths_by_side': topology['lengths_by_side'],
        'model_state_dict': {'weight': torch.ones(1)},
    }

    class FakeModel:
        model_kind = V4_MODEL_KIND
        slot_order = V4_SLOT_ORDER
        head_type = 'spatial_query'

        def __init__(self):
            self.strict_seen = False

        def load_state_dict(self, state, strict=False):
            assert state is checkpoint['model_state_dict']
            self.strict_seen = strict

        def state_dict(self):
            return checkpoint['model_state_dict']

        def to(self, _device):
            return self

        def eval(self):
            return self

    fake_model = FakeModel()
    monkeypatch.setattr(runtime_v4, '_torch_load', lambda *_args: checkpoint)
    monkeypatch.setattr(
        runtime_v4,
        'build_visual_state_model_v4',
        lambda *_args, **_kwargs: fake_model,
    )
    runtime = Room315VisualModelRuntimeV4(promotion, device='cpu')
    runtime.load()

    assert runtime.ready
    assert runtime.device == 'cpu'
    assert fake_model.strict_seen is True

    checkpoint['test_seen'] = True
    blocked = Room315VisualModelRuntimeV4(promotion, device='cpu')
    with pytest.raises(VisualRuntimeV4Error, match='test_unseen'):
        blocked.load()
