#!/usr/bin/env python3
"""Compatibility tests for the side-isolated Room 315 visual V4 contract."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_ROOT = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
EXPERIMENT_ROOT = REPO_ROOT / 'mfja_robot_control_config' / 'experiment_a_v3r1'
for path in (SCRIPT_ROOT, EXPERIMENT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import experiment_a_core as legacy_core  # noqa: E402
import room_315_rail_defaults as rail_defaults  # noqa: E402
import room_315_visual_contract_v4 as contract  # noqa: E402
from room_315_rail_defaults import (  # noqa: E402
    public_rail_segment_lengths,
    rail_segment_lengths,
)
from room_315_visual_state_dataset import VisualStateLabelVectorizer  # noqa: E402


def _segment_lengths():
    return {
        side: {
            segment: 1.0 + index * 0.1 + (0.25 if side == 'right' else 0.0)
            for index, segment in enumerate(contract.SEGMENT_CLASSES)
        }
        for side in contract.SIDES
    }


def _segment_length_contract():
    lengths = _segment_lengths()
    return contract.make_test_public_segment_length_contract(
        lengths_m_by_side=tuple(
            tuple(lengths[side][segment] for segment in contract.SEGMENT_CLASSES)
            for side in contract.SIDES
        ),
    )


def _image_sizes_by_side():
    return {
        'left': (640, 480),
        'right': (640, 480),
    }


def _all_accepted():
    return {
        identity: contract.SlotAcceptance(identity=identity)
        for identity in contract.FIXED_IDENTITIES
    }


def _prediction(
    identity,
    *,
    segment_index=0,
    loaded_index=0,
    s_ratio=0.5,
    use_scores=False,
    bbox_xywh=(10.0, 20.0, 30.0, 40.0),
):
    kwargs = {
        'identity': identity,
        'bbox_xywh': bbox_xywh,
        's_ratio': s_ratio,
    }
    if use_scores:
        segment_scores = [-5.0] * len(contract.SEGMENT_CLASSES)
        segment_scores[segment_index] = 3.0
        loaded_scores = [-2.0] * len(contract.LOADED_CLASSES)
        loaded_scores[loaded_index] = 4.0
        kwargs.update(
            segment_scores=segment_scores,
            loaded_scores=loaded_scores,
        )
    else:
        kwargs.update(
            segment_index=segment_index,
            loaded_index=loaded_index,
        )
    return contract.StructuredSlotPrediction(**kwargs)


def _predictions(**overrides):
    values = []
    for index, identity in enumerate(contract.FIXED_IDENTITIES):
        values.append(overrides.get(identity) or _prediction(
            identity,
            segment_index=index % len(contract.SEGMENT_CLASSES),
            loaded_index=index % len(contract.LOADED_CLASSES),
            s_ratio=(index + 1) / 10.0,
        ))
    return values


def _legacy_label(*, absent=()):
    lengths = _segment_lengths()
    shuttles = []
    for index, identity in enumerate(contract.FIXED_IDENTITIES):
        side = contract.derive_side(identity)
        if identity in absent:
            shuttles.append({
                'id': identity,
                'presence': False,
                'visually_available': False,
                'loaded_state': 'unknown',
                'bbox': [0.0, 0.0, 0.0, 0.0],
                'location': {'side': side, 'block': 'unknown'},
                'rail_position': {
                    'available': False,
                    's_m': 0.0,
                    's_ratio': 0.0,
                    'segment_length_m': 0.0,
                },
            })
            continue
        segment = contract.SEGMENT_CLASSES[index % len(contract.SEGMENT_CLASSES)]
        ratio = (index + 1) / 10.0
        length = lengths[side][segment]
        shuttles.append({
            'id': identity,
            'presence': True,
            'visually_available': True,
            'loaded_state': contract.LOADED_CLASSES[index % 2],
            'bbox': [
                10.0 + index,
                20.0 + index,
                30.0 + index,
                40.0 + index,
            ],
            'location': {'side': side, 'block': segment},
            'rail_position': {
                'available': True,
                's_m': ratio * length,
                's_ratio': ratio,
                'segment_length_m': length,
            },
        })
    return {
        'visual_state_labels': {
            'schema_version': legacy_core.VISUAL_SCHEMA,
            'shuttles': shuttles,
        }
    }


def test_declared_legacy_layout_matches_v3_vectorizer_exactly():
    vectorizer = VisualStateLabelVectorizer()
    assert vectorizer.dim == contract.LEGACY_VECTOR_DIMENSION == 200
    assert tuple(vectorizer.names) == contract.LEGACY_VECTOR_NAMES
    assert contract.validate_legacy_vectorizer(vectorizer) == tuple(vectorizer.names)
    assert contract.validate_legacy_vectorizer(vectorizer.to_json()) == tuple(
        vectorizer.names
    )


def test_synthetic_legacy_labels_decode_and_roundtrip_exact_field_order():
    vectorizer = VisualStateLabelVectorizer().to_json()
    legacy_vector, legacy_mask = legacy_core.vectorize_label(
        _legacy_label(),
        vectorizer,
    )
    targets = contract.decode_legacy_targets(
        legacy_vector,
        legacy_mask,
        legacy_vectorizer=vectorizer,
    )
    predictions = [target.as_prediction() for target in targets]
    adapted = contract.assemble_legacy_200(
        predictions,
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        legacy_vectorizer=vectorizer,
        acceptance_by_identity=_all_accepted(),
        allow_non_authoritative_contract_for_testing=True,
    )
    assert tuple(adapted.legacy_vector) == pytest.approx(legacy_vector)
    assert adapted.acceptance.accepted is True
    assert all(slot.accepted for slot in adapted.acceptance.slots)


def test_side_is_derived_from_identity_and_never_selected_by_model():
    assert contract.derive_side('l1') == 'left'
    assert contract.derive_side('R4') == 'right'
    with pytest.raises(contract.VisualContractV4Error, match='unknown fixed'):
        contract.derive_side('L9')

    output = contract.assemble_legacy_200(
        _predictions(),
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        allow_non_authoritative_contract_for_testing=True,
    )
    names = contract.LEGACY_VECTOR_NAMES
    for slot, identity in enumerate(contract.FIXED_IDENTITIES):
        expected = contract.derive_side(identity)
        for side in contract.SIDES:
            index = names.index(f'shuttles.{slot}.location.side=={side}')
            assert output.legacy_vector[index] == float(side == expected)


def test_argmax_selection_and_s_m_invariant():
    predictions = _predictions(
        L1=_prediction(
            'L1', segment_index=5, loaded_index=1, s_ratio=0.25, use_scores=True
        ),
        R1=_prediction(
            'R1', segment_index=9, loaded_index=0, s_ratio=0.9, use_scores=True
        ),
    )
    output = contract.assemble_legacy_200(
        predictions,
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        allow_non_authoritative_contract_for_testing=True,
    )
    by_identity = {slot.identity: slot for slot in output.shuttles}
    assert by_identity['L1'].segment == contract.SEGMENT_CLASSES[5]
    assert by_identity['L1'].loaded_state == 'loaded'
    assert by_identity['L1'].s_ratio == 0.25
    assert by_identity['R1'].segment == contract.SEGMENT_CLASSES[9]
    assert by_identity['R1'].loaded_state == 'empty'
    assert by_identity['R1'].s_ratio == 0.9
    for slot in output.shuttles:
        assert slot.s_m == pytest.approx(slot.s_ratio * slot.segment_length_m)


@pytest.mark.parametrize('invalid_ratio', [-0.001, 1.001])
def test_adapter_rejects_out_of_range_s_ratio_instead_of_clipping(invalid_ratio):
    with pytest.raises(contract.VisualContractV4Error, match=r's_ratio must be in \[0, 1\]'):
        contract.assemble_legacy_200(
            _predictions(L1=_prediction('L1', s_ratio=invalid_ratio)),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            allow_non_authoritative_contract_for_testing=True,
        )


@pytest.mark.parametrize(
    ('bbox_xywh', 'message'),
    [
        ((-1.0, 0.0, 1.0, 1.0), 'non-negative'),
        ((0.0, -1.0, 1.0, 1.0), 'non-negative'),
        ((0.0, 0.0, 0.0, 1.0), 'must be positive'),
        ((0.0, 0.0, 1.0, 0.0), 'must be positive'),
        ((620.0, 20.0, 30.0, 40.0), 'enclosed by image size'),
        ((10.0, 450.0, 30.0, 40.0), 'enclosed by image size'),
    ],
)
def test_adapter_rejects_invalid_or_out_of_image_bbox(bbox_xywh, message):
    with pytest.raises(contract.VisualContractV4Error, match=message):
        contract.assemble_legacy_200(
            _predictions(L1=_prediction('L1', bbox_xywh=bbox_xywh)),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            allow_non_authoritative_contract_for_testing=True,
        )


def test_adapter_requires_exact_positive_image_sizes_for_both_sides():
    with pytest.raises(contract.VisualContractV4Error, match='vocabulary mismatch'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side={'left': (640, 480)},
            allow_non_authoritative_contract_for_testing=True,
        )
    invalid_sizes = _image_sizes_by_side()
    invalid_sizes['right'] = (0, 480)
    with pytest.raises(contract.VisualContractV4Error, match='positive dimensions'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=invalid_sizes,
            allow_non_authoritative_contract_for_testing=True,
        )


def test_missing_acceptance_is_all_rejected_and_never_implicitly_promoted():
    diagnostic = contract.assemble_legacy_200(
        _predictions(),
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        allow_non_authoritative_contract_for_testing=True,
    )

    assert diagnostic.acceptance.accepted is False
    assert all(not slot.accepted for slot in diagnostic.acceptance.slots)
    assert diagnostic.acceptance.reasons == tuple(
        f'{identity}:missing_acceptance'
        for identity in contract.FIXED_IDENTITIES
    )


def test_confidence_and_acceptance_are_separate_from_legacy_200_values():
    predictions = _predictions()
    plain = contract.assemble_legacy_200(
        predictions,
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        allow_non_authoritative_contract_for_testing=True,
    )
    acceptance = {
        identity: contract.SlotAcceptance(
            identity=identity,
            accepted=identity != 'R4',
            segment_confidence=0.99 if identity != 'R4' else 0.42,
            loaded_confidence=0.95,
            reasons=() if identity != 'R4' else ('low_segment_confidence',),
        )
        for identity in contract.FIXED_IDENTITIES
    }
    gated = contract.assemble_legacy_200(
        predictions,
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        acceptance_by_identity=acceptance,
        allow_non_authoritative_contract_for_testing=True,
    )
    assert gated.legacy_vector == plain.legacy_vector
    assert gated.acceptance.accepted is False
    assert gated.acceptance.reasons == ('R4:low_segment_confidence',)
    assert len(gated.legacy_vector) == 200


def test_presence_aware_acceptance_ignores_absent_slots_without_accepting_them():
    present = {'L2', 'R4'}
    acceptance = {
        identity: contract.SlotAcceptance(
            identity=identity,
            accepted=identity in present,
            required=identity in present,
            segment_confidence=0.99 if identity in present else None,
            loaded_confidence=0.98 if identity in present else None,
            reasons=() if identity in present else ('presence_absent_not_evaluated',),
        )
        for identity in contract.FIXED_IDENTITIES
    }
    output = contract.assemble_legacy_200(
        _predictions(),
        segment_length_contract=_segment_length_contract(),
        image_sizes_by_side=_image_sizes_by_side(),
        acceptance_by_identity=acceptance,
        allow_non_authoritative_contract_for_testing=True,
    )
    assert output.acceptance.accepted is True
    assert output.acceptance.required_identities == ('L2', 'R4')
    assert output.acceptance.reasons == ()
    slots = {slot.identity: slot for slot in output.acceptance.slots}
    assert slots['L2'].accepted and slots['L2'].required
    assert not slots['L1'].accepted and not slots['L1'].required
    assert slots['L1'].reasons == ('presence_absent_not_evaluated',)


def test_non_required_slot_cannot_be_marked_accepted():
    acceptance = _all_accepted()
    acceptance['L1'] = contract.SlotAcceptance(
        identity='L1',
        accepted=True,
        required=False,
    )
    with pytest.raises(contract.VisualContractV4Error, match='cannot be marked accepted'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            acceptance_by_identity=acceptance,
            allow_non_authoritative_contract_for_testing=True,
        )


def test_acceptance_requires_boolean_decisions_and_unique_normalized_keys():
    non_boolean = _all_accepted()
    non_boolean['L1'] = contract.SlotAcceptance(
        identity='L1',
        accepted='false',
    )
    with pytest.raises(contract.VisualContractV4Error, match='must be boolean'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            acceptance_by_identity=non_boolean,
            allow_non_authoritative_contract_for_testing=True,
        )

    duplicate = _all_accepted()
    duplicate[' l1 '] = contract.SlotAcceptance(identity='L1')
    with pytest.raises(contract.VisualContractV4Error, match='duplicate normalized'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            acceptance_by_identity=duplicate,
            allow_non_authoritative_contract_for_testing=True,
        )


def test_legacy_target_helper_preserves_masks_for_absent_slots():
    vectorizer = VisualStateLabelVectorizer().to_json()
    vector, mask = legacy_core.vectorize_label(
        _legacy_label(absent={'R4'}),
        vectorizer,
    )
    targets = contract.decode_legacy_targets(
        vector,
        mask,
        legacy_vectorizer=vectorizer,
    )
    by_identity = {target.identity: target for target in targets}
    assert by_identity['L1'].mask.segment is True
    assert by_identity['L1'].mask.loaded is True
    assert all(by_identity['L1'].mask.bbox_xywh)
    assert by_identity['R4'].segment_index is None
    assert by_identity['R4'].loaded_index is None
    assert by_identity['R4'].s_ratio is None
    assert by_identity['R4'].mask == contract.StructuredTargetMask(
        segment=False,
        loaded=False,
        bbox_xywh=(False, False, False, False),
        s_ratio=False,
    )
    with pytest.raises(contract.VisualContractV4Error, match='masked target'):
        by_identity['R4'].as_prediction()


def test_adapter_rejects_raw_topology_mapping_and_invalid_contract_shape():
    with pytest.raises(contract.VisualContractV4Error, match='raw mappings are forbidden'):
        contract.assemble_legacy_200(
            _predictions(L1=_prediction('L1', segment_index=0)),
            segment_length_contract=_segment_lengths(),
            image_sizes_by_side=_image_sizes_by_side(),
        )

    with pytest.raises(contract.VisualContractV4Error, match='cannot be constructed'):
        contract.PublicSegmentLengthContract(
            lengths_m_by_side=_segment_length_contract().as_matrix(),
        )

    with pytest.raises(contract.VisualContractV4Error, match='must have shape'):
        contract.make_test_public_segment_length_contract(
            lengths_m_by_side=((1.0,) * len(contract.SEGMENT_CLASSES),),
        )
    invalid_rows = [list(row) for row in _segment_length_contract().as_matrix()]
    invalid_rows[0][0] = 0.0
    with pytest.raises(contract.VisualContractV4Error, match='finite and positive'):
        contract.make_test_public_segment_length_contract(
            lengths_m_by_side=tuple(tuple(row) for row in invalid_rows),
        )


def test_production_assembly_requires_loader_authority_and_test_opt_in_is_explicit():
    with pytest.raises(
        contract.VisualContractV4Error,
        match='non-authoritative segment length contract',
    ):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
        )

    authoritative = contract.load_authoritative_public_segment_length_contract()
    promoted = contract.assemble_legacy_200(
        _predictions(),
        segment_length_contract=authoritative,
        image_sizes_by_side=_image_sizes_by_side(),
        acceptance_by_identity=_all_accepted(),
    )
    assert authoritative.authoritative is True
    assert promoted.acceptance.accepted is True


def test_adapter_rejects_invalid_legacy_layout():
    vectorizer = VisualStateLabelVectorizer().to_json()
    vectorizer['names'] = list(vectorizer['names'])
    vectorizer['names'][0], vectorizer['names'][1] = (
        vectorizer['names'][1],
        vectorizer['names'][0],
    )
    with pytest.raises(contract.VisualContractV4Error, match='field ordering'):
        contract.validate_legacy_vectorizer(vectorizer)


def test_adapter_rejects_invalid_or_ambiguous_class_selection():
    with pytest.raises(contract.VisualContractV4Error, match='exactly one'):
        contract.assemble_legacy_200(
            _predictions(
                L1=contract.StructuredSlotPrediction(
                    identity='L1',
                    bbox_xywh=(1.0, 2.0, 3.0, 4.0),
                    s_ratio=0.5,
                    segment_index=0,
                    segment_scores=[0.0] * len(contract.SEGMENT_CLASSES),
                    loaded_index=0,
                )
            ),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            allow_non_authoritative_contract_for_testing=True,
        )
    with pytest.raises(contract.VisualContractV4Error, match='outside its vocabulary'):
        contract.assemble_legacy_200(
            _predictions(L1=_prediction('L1', segment_index=99)),
            segment_length_contract=_segment_length_contract(),
            image_sizes_by_side=_image_sizes_by_side(),
            allow_non_authoritative_contract_for_testing=True,
        )


def test_authoritative_public_length_contract_uses_public_left_vocabulary_only():
    authoritative = contract.load_authoritative_public_segment_length_contract()
    raw_internal_left = rail_segment_lengths('left')
    public_left = public_rail_segment_lengths('left')

    assert authoritative.source == contract.PUBLIC_SEGMENT_LENGTH_SOURCE
    assert authoritative.length_m('left', 'A14') == pytest.approx(public_left['A14'])
    assert authoritative.length_m('left', 'A23') == pytest.approx(public_left['A23'])
    assert authoritative.length_m('left', 'A14') == pytest.approx(
        raw_internal_left['A23']
    )
    assert authoritative.length_m('left', 'A23') == pytest.approx(
        raw_internal_left['A14']
    )
    assert raw_internal_left['A14'] != pytest.approx(raw_internal_left['A23'])

    with pytest.raises(contract.VisualContractV4Error, match='raw mappings are forbidden'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=raw_internal_left,
            image_sizes_by_side=_image_sizes_by_side(),
        )

    raw_internal_by_side = {
        side: rail_segment_lengths(side)
        for side in contract.SIDES
    }
    with pytest.raises(contract.VisualContractV4Error, match='raw mappings are forbidden'):
        contract.assemble_legacy_200(
            _predictions(),
            segment_length_contract=raw_internal_by_side,
            image_sizes_by_side=_image_sizes_by_side(),
        )

    explicitly_wrapped_internal = contract.make_test_public_segment_length_contract(
        lengths_m_by_side=tuple(
            tuple(
                raw_internal_by_side[side][segment]
                for segment in contract.SEGMENT_CLASSES
            )
            for side in contract.SIDES
        ),
    )
    explicitly_wrapped_public = contract.make_test_public_segment_length_contract(
        lengths_m_by_side=authoritative.as_matrix(),
    )
    assert (
        explicitly_wrapped_internal.fingerprint_sha256
        != explicitly_wrapped_public.fingerprint_sha256
    )
    assert explicitly_wrapped_internal.authoritative is False
    assert explicitly_wrapped_internal.source == (
        contract.TEST_FIXTURE_SEGMENT_LENGTH_SOURCE
    )


def test_public_length_contract_views_are_canonical_and_immutable():
    first = contract.load_authoritative_public_segment_length_contract()
    second = contract.load_authoritative_public_segment_length_contract()

    assert first.as_matrix() == tuple(
        tuple(
            public_rail_segment_lengths(side)[segment]
            for segment in contract.SEGMENT_CLASSES
        )
        for side in contract.SIDES
    )
    assert first.as_mapping()['right']['A4I'] == first.length_m('right', 'A4I')
    assert first.fingerprint_sha256 == second.fingerprint_sha256
    assert len(first.fingerprint_sha256) == 64
    assert set(first.fingerprint_sha256) <= set('0123456789abcdef')
    assert first.canonical_metadata()['fingerprint_sha256'] == (
        first.fingerprint_sha256
    )
    with pytest.raises(TypeError):
        first.as_mapping()['left']['A14'] = 99.0
    with pytest.raises(AttributeError):
        first.source = 'caller-controlled'


def test_authoritative_loader_rejects_public_mapping_drift(monkeypatch):
    monkeypatch.setattr(
        rail_defaults,
        'public_rail_segment_name_to_internal',
        lambda side, segment: segment,
    )
    with pytest.raises(contract.VisualContractV4Error, match='mapping mismatch'):
        contract.load_authoritative_public_segment_length_contract()


def test_authoritative_loader_rejects_vocabulary_and_invalid_lengths(monkeypatch):
    original_loader = rail_defaults.public_rail_segment_lengths

    def wrong_vocabulary(side):
        values = dict(original_loader(side))
        del values['A14']
        values['INTERNAL_ONLY'] = 1.0
        return values

    monkeypatch.setattr(
        rail_defaults,
        'public_rail_segment_lengths',
        wrong_vocabulary,
    )
    with pytest.raises(contract.VisualContractV4Error, match='vocabulary mismatch'):
        contract.load_authoritative_public_segment_length_contract()

    def invalid_length(side):
        values = dict(original_loader(side))
        values['A14'] = float('nan')
        return values

    monkeypatch.setattr(
        rail_defaults,
        'public_rail_segment_lengths',
        invalid_length,
    )
    with pytest.raises(contract.VisualContractV4Error, match='must be finite'):
        contract.load_authoritative_public_segment_length_contract()
