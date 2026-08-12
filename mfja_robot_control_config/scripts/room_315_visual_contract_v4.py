#!/usr/bin/env python3
"""Side-isolated Room 315 visual contract with a legacy V3 adapter.

The V4 learned representation contains only visual quantities that are not
already fixed by repository configuration: segment class, loaded state,
bounding box, and normalized position.  This module can assemble those values
into the historical ``room315.visual_state.v3`` 200-value vector without
changing the V3 runtime.

The adapter deliberately keeps confidence and acceptance metadata outside the
legacy vector.  It also derives rail side from the fixed identity and derives
``segment_length_m`` and ``s_m`` from a typed, fingerprinted public-topology
contract.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import InitVar, dataclass, field
from types import MappingProxyType
from typing import Any, Mapping, Sequence


CONTRACT_SCHEMA_VERSION = 'room315.visual_contract.v4'
LEGACY_SCHEMA_VERSION = 'room315.visual_state.v3'
LEGACY_VECTOR_DIMENSION = 200

FIXED_IDENTITIES = (
    'L1', 'L2', 'L3', 'L4',
    'R1', 'R2', 'R3', 'R4',
)
SIDES = ('left', 'right')
SEGMENT_CLASSES = (
    'A12E', 'A12I', 'A14', 'A1E', 'A1I', 'A23', 'A2E', 'A2I',
    'A34E', 'A34I', 'A3E', 'A3I', 'A4E', 'A4I',
)
LOADED_CLASSES = ('empty', 'loaded')
PUBLIC_SEGMENT_LENGTH_CONTRACT_SCHEMA = (
    'room315.public_segment_length_contract.v1'
)
PUBLIC_SEGMENT_LENGTH_SOURCE = (
    'room_315_rail_defaults.public_rail_segment_lengths'
)
TEST_FIXTURE_SEGMENT_LENGTH_SOURCE = 'test_fixture'

_AUTHORITATIVE_CONTRACT_TOKEN = object()
_TEST_FIXTURE_CONTRACT_TOKEN = object()
_EXPECTED_INTERNAL_SEGMENT_BY_PUBLIC = {
    'left': (
        'A34E', 'A34I', 'A23', 'A3E', 'A3I', 'A14', 'A4E', 'A4I',
        'A12E', 'A12I', 'A1E', 'A1I', 'A2E', 'A2I',
    ),
    'right': SEGMENT_CLASSES,
}

# V3 stores all numeric fields for all slots first.  It then stores categorical
# groups per slot in side, block, loaded-state order.  Keep this declaration
# explicit so compatibility cannot depend on incidental dictionary ordering.
LEGACY_NUMERIC_FIELDS = (
    'bbox.0',
    'bbox.1',
    'bbox.2',
    'bbox.3',
    'rail_position.s_m',
    'rail_position.s_ratio',
    'rail_position.segment_length_m',
)
LEGACY_CATEGORICAL_FIELDS = (
    ('location.side', SIDES),
    ('location.block', SEGMENT_CLASSES),
    ('loaded_state', LOADED_CLASSES),
)


class VisualContractV4Error(ValueError):
    """Raised when V4 structured values cannot satisfy the legacy contract."""


@dataclass(frozen=True, slots=True)
class PublicSegmentLengthContract:
    """Immutable public segment lengths in canonical side/class order.

    ``lengths_m_by_side`` is always normalized to two immutable rows.  Row
    order is ``SIDES`` (left, right), and column order is ``SEGMENT_CLASSES``.
    The source and authoritative bit are assigned only by a module-controlled
    construction path rather than by the caller.  Production construction is
    available only through
    :func:`load_authoritative_public_segment_length_contract`; synthetic tests
    have a deliberately non-authoritative factory.  The SHA-256 covers the
    schema, provenance, ordering, vocabulary, and exact hexadecimal
    floating-point values.
    """

    lengths_m_by_side: tuple[tuple[float, ...], tuple[float, ...]]
    _creation_token: InitVar[object | None] = None
    source: str = field(init=False)
    authoritative: bool = field(init=False)
    fingerprint_sha256: str = field(init=False)

    def __post_init__(self, _creation_token: object | None) -> None:
        if _creation_token is _AUTHORITATIVE_CONTRACT_TOKEN:
            source = PUBLIC_SEGMENT_LENGTH_SOURCE
            authoritative = True
        elif _creation_token is _TEST_FIXTURE_CONTRACT_TOKEN:
            source = TEST_FIXTURE_SEGMENT_LENGTH_SOURCE
            authoritative = False
        else:
            raise VisualContractV4Error(
                'PublicSegmentLengthContract cannot be constructed directly; '
                'use load_authoritative_public_segment_length_contract() or '
                'make_test_public_segment_length_contract()'
            )
        try:
            rows = tuple(
                tuple(float(value) for value in row)
                for row in self.lengths_m_by_side
            )
        except (TypeError, ValueError) as exc:
            raise VisualContractV4Error(
                'public segment length matrix must be numeric'
            ) from exc
        expected_shape = (len(SIDES), len(SEGMENT_CLASSES))
        actual_shape = (
            len(rows),
            tuple(len(row) for row in rows),
        )
        if len(rows) != expected_shape[0] or any(
            len(row) != expected_shape[1] for row in rows
        ):
            raise VisualContractV4Error(
                'public segment length matrix must have shape '
                f'[{expected_shape[0]}, {expected_shape[1]}], got {actual_shape}'
            )
        for side, row in zip(SIDES, rows):
            for segment, value in zip(SEGMENT_CLASSES, row):
                if not math.isfinite(value) or value <= 0.0:
                    raise VisualContractV4Error(
                        'public segment length must be finite and positive: '
                        f'{side}.{segment}={value!r}'
                    )
        canonical_payload = {
            'schema_version': PUBLIC_SEGMENT_LENGTH_CONTRACT_SCHEMA,
            'source': source,
            'authoritative': authoritative,
            'side_order': list(SIDES),
            'segment_order': list(SEGMENT_CLASSES),
            'lengths_m_hex': [
                [value.hex() for value in row]
                for row in rows
            ],
        }
        canonical_json = json.dumps(
            canonical_payload,
            ensure_ascii=True,
            sort_keys=True,
            separators=(',', ':'),
        )
        object.__setattr__(self, 'lengths_m_by_side', rows)
        object.__setattr__(self, 'source', source)
        object.__setattr__(self, 'authoritative', authoritative)
        object.__setattr__(
            self,
            'fingerprint_sha256',
            hashlib.sha256(canonical_json.encode('utf-8')).hexdigest(),
        )

    def length_m(self, side: str, segment: str) -> float:
        """Return one public length, rejecting aliases and unknown names."""

        if side not in SIDES:
            raise VisualContractV4Error(
                f'unknown public rail side: {side!r}'
            )
        if segment not in SEGMENT_CLASSES:
            raise VisualContractV4Error(
                f'unknown public segment: {segment!r}'
            )
        return self.lengths_m_by_side[SIDES.index(side)][
            SEGMENT_CLASSES.index(segment)
        ]

    def as_matrix(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Return the immutable trainer matrix in canonical side/class order."""

        return self.lengths_m_by_side

    def as_mapping(self) -> Mapping[str, Mapping[str, float]]:
        """Return a recursively read-only public mapping for runtime use."""

        rows = {
            side: MappingProxyType(dict(zip(SEGMENT_CLASSES, row)))
            for side, row in zip(SIDES, self.lengths_m_by_side)
        }
        return MappingProxyType(rows)

    def canonical_metadata(self) -> dict[str, Any]:
        """Return JSON-safe provenance without exposing a mutable source map."""

        return {
            'schema_version': PUBLIC_SEGMENT_LENGTH_CONTRACT_SCHEMA,
            'source': self.source,
            'authoritative': self.authoritative,
            'fingerprint_sha256': self.fingerprint_sha256,
            'side_order': list(SIDES),
            'segment_order': list(SEGMENT_CLASSES),
            'lengths_m_by_side': [
                list(row) for row in self.lengths_m_by_side
            ],
        }


def make_test_public_segment_length_contract(
    *,
    lengths_m_by_side: Sequence[Sequence[float]],
) -> PublicSegmentLengthContract:
    """Create an explicitly non-authoritative contract for isolated tests.

    Production callers must never promote this value.  The compatibility
    adapter rejects it unless its test-only override is explicitly enabled.
    """

    return PublicSegmentLengthContract(
        lengths_m_by_side=lengths_m_by_side,
        _creation_token=_TEST_FIXTURE_CONTRACT_TOKEN,
    )


def load_authoritative_public_segment_length_contract(
) -> PublicSegmentLengthContract:
    """Load the production contract only through public rail segment names."""

    try:
        from room_315_rail_defaults import (
            public_rail_segment_lengths,
            public_rail_segment_name_to_internal,
        )
    except Exception as exc:
        raise VisualContractV4Error(
            'authoritative public segment length loader is unavailable'
        ) from exc

    ordered_rows: list[tuple[float, ...]] = []
    expected_vocabulary = set(SEGMENT_CLASSES)
    for side in SIDES:
        try:
            internal_by_public = tuple(
                public_rail_segment_name_to_internal(side, segment)
                for segment in SEGMENT_CLASSES
            )
        except Exception as exc:
            raise VisualContractV4Error(
                f'failed to verify public segment-name mapping for {side}'
            ) from exc
        if any(not isinstance(value, str) for value in internal_by_public):
            raise VisualContractV4Error(
                f'public-to-internal segment-name mapping is invalid for {side}'
            )
        if (
            len(set(internal_by_public)) != len(SEGMENT_CLASSES)
            or set(internal_by_public) != expected_vocabulary
        ):
            raise VisualContractV4Error(
                f'public-to-internal segment-name mapping is not one-to-one for {side}'
            )
        if internal_by_public != _EXPECTED_INTERNAL_SEGMENT_BY_PUBLIC[side]:
            raise VisualContractV4Error(
                f'public-to-internal segment-name mapping mismatch for {side}'
            )
        try:
            raw_lengths = public_rail_segment_lengths(side)
        except Exception as exc:
            raise VisualContractV4Error(
                f'failed to load authoritative public segment lengths for {side}'
            ) from exc
        if not isinstance(raw_lengths, Mapping):
            raise VisualContractV4Error(
                f'authoritative public segment lengths for {side} are not a mapping'
            )
        actual_vocabulary = set(raw_lengths)
        if actual_vocabulary != expected_vocabulary:
            missing = sorted(expected_vocabulary - actual_vocabulary)
            unexpected = sorted(
                repr(value)
                for value in actual_vocabulary - expected_vocabulary
            )
            raise VisualContractV4Error(
                f'authoritative public segment vocabulary mismatch for {side}; '
                f'missing={missing}, unexpected={unexpected}'
            )
        ordered_rows.append(tuple(
            _positive_finite_length(
                raw_lengths[segment],
                f'{side}.{segment}',
            )
            for segment in SEGMENT_CLASSES
        ))
    return PublicSegmentLengthContract(
        lengths_m_by_side=(ordered_rows[0], ordered_rows[1]),
        _creation_token=_AUTHORITATIVE_CONTRACT_TOKEN,
    )


def _build_legacy_vector_names() -> tuple[str, ...]:
    numeric = tuple(
        f'shuttles.{slot}.{field}'
        for slot in range(len(FIXED_IDENTITIES))
        for field in LEGACY_NUMERIC_FIELDS
    )
    categorical = tuple(
        f'shuttles.{slot}.{field}=={value}'
        for slot in range(len(FIXED_IDENTITIES))
        for field, vocabulary in LEGACY_CATEGORICAL_FIELDS
        for value in vocabulary
    )
    names = numeric + categorical
    if len(names) != LEGACY_VECTOR_DIMENSION or len(set(names)) != len(names):
        raise AssertionError('the declared V3 compatibility layout is not 200 unique values')
    return names


LEGACY_VECTOR_NAMES = _build_legacy_vector_names()


@dataclass(frozen=True)
class StructuredSlotPrediction:
    """One V4 model slot before deterministic topology enrichment.

    ``bbox_xywh`` is expressed in the legacy image-pixel coordinate space; a
    model that predicts normalized boxes must denormalize them before adapting.
    Exactly one of ``segment_index`` and ``segment_scores`` must be supplied,
    and exactly one of ``loaded_index`` and ``loaded_scores`` must be supplied.
    Scores may be logits or any other finite values because only deterministic
    argmax is performed here.  Confidence calibration belongs to the separate
    acceptance envelope.
    """

    identity: str
    bbox_xywh: Sequence[float]
    s_ratio: float
    segment_index: int | None = None
    segment_scores: Sequence[float] | None = None
    loaded_index: int | None = None
    loaded_scores: Sequence[float] | None = None


@dataclass(frozen=True)
class CanonicalSlotPrediction:
    """A structured slot after side and topology-derived values are applied."""

    identity: str
    side: str
    segment_index: int
    segment: str
    loaded_index: int
    loaded_state: str
    bbox_xywh: tuple[float, float, float, float]
    s_ratio: float
    segment_length_m: float
    s_m: float


@dataclass(frozen=True)
class SlotAcceptance:
    """Calibrated decision metadata that is never encoded in the 200 values."""

    identity: str
    accepted: bool = True
    required: bool = True
    segment_confidence: float | None = None
    loaded_confidence: float | None = None
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class AcceptanceEnvelope:
    schema_version: str
    accepted: bool
    slots: tuple[SlotAcceptance, ...]
    required_identities: tuple[str, ...]
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class LegacyCompatibilityOutput:
    """Raw V3-compatible output plus non-vector V4 metadata."""

    schema_version: str
    legacy_schema_version: str
    legacy_vector: tuple[float, ...]
    shuttles: tuple[CanonicalSlotPrediction, ...]
    acceptance: AcceptanceEnvelope


@dataclass(frozen=True)
class StructuredTargetMask:
    segment: bool
    loaded: bool
    bbox_xywh: tuple[bool, bool, bool, bool]
    s_ratio: bool


@dataclass(frozen=True)
class StructuredSlotTarget:
    """Training target decoded from the frozen legacy label representation."""

    identity: str
    side: str
    segment_index: int | None
    loaded_index: int | None
    bbox_xywh: tuple[float, float, float, float]
    s_ratio: float | None
    mask: StructuredTargetMask

    def as_prediction(self) -> StructuredSlotPrediction:
        """Return a prediction-shaped value when every V4 target is available."""

        if (
            self.segment_index is None
            or self.loaded_index is None
            or self.s_ratio is None
            or not self.mask.segment
            or not self.mask.loaded
            or not self.mask.s_ratio
            or not all(self.mask.bbox_xywh)
        ):
            raise VisualContractV4Error(
                f'cannot make a complete prediction from masked target {self.identity}'
            )
        return StructuredSlotPrediction(
            identity=self.identity,
            segment_index=self.segment_index,
            loaded_index=self.loaded_index,
            bbox_xywh=self.bbox_xywh,
            s_ratio=self.s_ratio,
        )


def derive_side(identity: str) -> str:
    """Derive immutable rail side solely from the fixed shuttle identity."""

    normalized = str(identity).strip().upper()
    if normalized not in FIXED_IDENTITIES:
        raise VisualContractV4Error(f'unknown fixed shuttle identity: {identity!r}')
    return 'left' if normalized.startswith('L') else 'right'


def validate_legacy_vectorizer(legacy_vectorizer: Any) -> tuple[str, ...]:
    """Require the exact historical V3 name order, not merely 200 values."""

    if isinstance(legacy_vectorizer, Mapping):
        schema_version = legacy_vectorizer.get('schema_version')
        dimension = legacy_vectorizer.get('dim')
        names = legacy_vectorizer.get('names')
        if schema_version != LEGACY_SCHEMA_VERSION:
            raise VisualContractV4Error(
                f'legacy vectorizer schema must be {LEGACY_SCHEMA_VERSION}'
            )
        try:
            parsed_dimension = int(dimension)
        except (TypeError, ValueError) as exc:
            raise VisualContractV4Error('legacy vectorizer dimension is invalid') from exc
        if parsed_dimension != LEGACY_VECTOR_DIMENSION:
            raise VisualContractV4Error('legacy vectorizer dimension must be 200')
    else:
        names = getattr(legacy_vectorizer, 'names', None)
        dimension = getattr(legacy_vectorizer, 'dim', None)
        try:
            parsed_dimension = int(dimension)
        except (TypeError, ValueError) as exc:
            raise VisualContractV4Error('legacy vectorizer dimension is invalid') from exc
        if parsed_dimension != LEGACY_VECTOR_DIMENSION:
            raise VisualContractV4Error('legacy vectorizer dimension must be 200')
    if not isinstance(names, (list, tuple)):
        raise VisualContractV4Error('legacy vectorizer names are unavailable')
    parsed_names = tuple(str(name) for name in names)
    if parsed_names != LEGACY_VECTOR_NAMES:
        raise VisualContractV4Error(
            'legacy vectorizer field ordering does not match the frozen V3 contract'
        )
    return parsed_names


def assemble_legacy_200(
    predictions: Sequence[StructuredSlotPrediction],
    *,
    segment_length_contract: PublicSegmentLengthContract,
    image_sizes_by_side: Mapping[str, Sequence[float]],
    legacy_vectorizer: Any | None = None,
    acceptance_by_identity: Mapping[str, SlotAcceptance] | None = None,
    allow_non_authoritative_contract_for_testing: bool = False,
) -> LegacyCompatibilityOutput:
    """Assemble side-isolated V4 predictions into the raw legacy 200-vector.

    A raw mapping is deliberately rejected.  Production callers must pass the
    authoritative contract returned by the production loader so internal
    left-rail names cannot masquerade as public segment names.  The explicit
    non-authoritative override exists only for isolated unit tests.

    Bounding boxes use pixel ``xywh`` and are validated against the required
    ``(width, height)`` for each side.  Missing acceptance metadata produces an
    all-rejected diagnostic envelope; the 200 raw values alone are therefore
    never evidence that a prediction was promoted.  Target-stat normalization,
    when needed, remains outside this contract.
    """

    if not isinstance(segment_length_contract, PublicSegmentLengthContract):
        raise VisualContractV4Error(
            'segment_length_contract must be a '
            'PublicSegmentLengthContract; raw mappings are forbidden'
        )
    if not isinstance(allow_non_authoritative_contract_for_testing, bool):
        raise VisualContractV4Error(
            'allow_non_authoritative_contract_for_testing must be boolean'
        )
    if (
        not segment_length_contract.authoritative
        and not allow_non_authoritative_contract_for_testing
    ):
        raise VisualContractV4Error(
            'a non-authoritative segment length contract is forbidden in '
            'production assembly; the override is test-only'
        )
    image_sizes = _canonical_image_sizes(image_sizes_by_side)
    names = (
        validate_legacy_vectorizer(legacy_vectorizer)
        if legacy_vectorizer is not None
        else LEGACY_VECTOR_NAMES
    )
    ordered_predictions = _predictions_by_fixed_identity(predictions)
    canonical = tuple(
        _canonicalize_slot(
            ordered_predictions[identity],
            segment_length_contract=segment_length_contract,
            image_size=image_sizes[derive_side(identity)],
        )
        for identity in FIXED_IDENTITIES
    )

    values = {name: 0.0 for name in names}
    for slot, shuttle in enumerate(canonical):
        numeric = {
            'bbox.0': shuttle.bbox_xywh[0],
            'bbox.1': shuttle.bbox_xywh[1],
            'bbox.2': shuttle.bbox_xywh[2],
            'bbox.3': shuttle.bbox_xywh[3],
            'rail_position.s_m': shuttle.s_m,
            'rail_position.s_ratio': shuttle.s_ratio,
            'rail_position.segment_length_m': shuttle.segment_length_m,
        }
        for field, value in numeric.items():
            values[f'shuttles.{slot}.{field}'] = value
        for value in SIDES:
            values[f'shuttles.{slot}.location.side=={value}'] = float(
                value == shuttle.side
            )
        for value in SEGMENT_CLASSES:
            values[f'shuttles.{slot}.location.block=={value}'] = float(
                value == shuttle.segment
            )
        for value in LOADED_CLASSES:
            values[f'shuttles.{slot}.loaded_state=={value}'] = float(
                value == shuttle.loaded_state
            )

    vector = tuple(float(values[name]) for name in names)
    if len(vector) != LEGACY_VECTOR_DIMENSION or not all(
        math.isfinite(value) for value in vector
    ):
        raise VisualContractV4Error('assembled legacy vector is invalid')
    acceptance = _acceptance_envelope(acceptance_by_identity)
    return LegacyCompatibilityOutput(
        schema_version=CONTRACT_SCHEMA_VERSION,
        legacy_schema_version=LEGACY_SCHEMA_VERSION,
        legacy_vector=vector,
        shuttles=canonical,
        acceptance=acceptance,
    )


def decode_legacy_targets(
    legacy_vector: Sequence[float],
    target_mask: Sequence[float],
    *,
    legacy_vectorizer: Any | None = None,
) -> tuple[StructuredSlotTarget, ...]:
    """Decode V3 labels/masks into the reduced V4 training targets.

    Side, learned ``s_m``, and learned segment length are deliberately omitted
    from the training target.  A visible legacy side target is nevertheless
    checked against the fixed identity so malformed labels fail closed.
    """

    names = (
        validate_legacy_vectorizer(legacy_vectorizer)
        if legacy_vectorizer is not None
        else LEGACY_VECTOR_NAMES
    )
    vector = _finite_vector(legacy_vector, 'legacy_vector')
    mask = _mask_vector(target_mask)
    if len(vector) != LEGACY_VECTOR_DIMENSION or len(mask) != LEGACY_VECTOR_DIMENSION:
        raise VisualContractV4Error('legacy target and mask must each contain 200 values')
    indexes = {name: index for index, name in enumerate(names)}
    targets: list[StructuredSlotTarget] = []
    for slot, identity in enumerate(FIXED_IDENTITIES):
        bbox_names = tuple(f'shuttles.{slot}.bbox.{index}' for index in range(4))
        side_names = tuple(
            f'shuttles.{slot}.location.side=={value}' for value in SIDES
        )
        segment_names = tuple(
            f'shuttles.{slot}.location.block=={value}'
            for value in SEGMENT_CLASSES
        )
        loaded_names = tuple(
            f'shuttles.{slot}.loaded_state=={value}' for value in LOADED_CLASSES
        )
        side_available = _categorical_mask_available(mask, indexes, side_names, identity)
        segment_available = _categorical_mask_available(
            mask, indexes, segment_names, identity
        )
        loaded_available = _categorical_mask_available(
            mask, indexes, loaded_names, identity
        )
        if side_available:
            side_index = _one_hot_index(vector, indexes, side_names, f'{identity}.side')
            if SIDES[side_index] != derive_side(identity):
                raise VisualContractV4Error(
                    f'legacy side target conflicts with fixed identity {identity}'
                )
        segment_index = (
            _one_hot_index(vector, indexes, segment_names, f'{identity}.segment')
            if segment_available
            else None
        )
        loaded_index = (
            _one_hot_index(vector, indexes, loaded_names, f'{identity}.loaded')
            if loaded_available
            else None
        )
        bbox = tuple(vector[indexes[name]] for name in bbox_names)
        bbox_mask = tuple(bool(mask[indexes[name]]) for name in bbox_names)
        s_ratio_name = f'shuttles.{slot}.rail_position.s_ratio'
        s_ratio_available = bool(mask[indexes[s_ratio_name]])
        targets.append(StructuredSlotTarget(
            identity=identity,
            side=derive_side(identity),
            segment_index=segment_index,
            loaded_index=loaded_index,
            bbox_xywh=(bbox[0], bbox[1], bbox[2], bbox[3]),
            s_ratio=vector[indexes[s_ratio_name]] if s_ratio_available else None,
            mask=StructuredTargetMask(
                segment=segment_available,
                loaded=loaded_available,
                bbox_xywh=(bbox_mask[0], bbox_mask[1], bbox_mask[2], bbox_mask[3]),
                s_ratio=s_ratio_available,
            ),
        ))
    return tuple(targets)


def _predictions_by_fixed_identity(
    predictions: Sequence[StructuredSlotPrediction],
) -> dict[str, StructuredSlotPrediction]:
    try:
        values = tuple(predictions)
    except TypeError as exc:
        raise VisualContractV4Error('predictions must be a sequence') from exc
    by_identity: dict[str, StructuredSlotPrediction] = {}
    for prediction in values:
        if not isinstance(prediction, StructuredSlotPrediction):
            raise VisualContractV4Error(
                'every prediction must be a StructuredSlotPrediction'
            )
        identity = str(prediction.identity).strip().upper()
        derive_side(identity)
        if identity in by_identity:
            raise VisualContractV4Error(f'duplicate structured identity: {identity}')
        by_identity[identity] = prediction
    if set(by_identity) != set(FIXED_IDENTITIES):
        missing = sorted(set(FIXED_IDENTITIES) - set(by_identity))
        unexpected = sorted(set(by_identity) - set(FIXED_IDENTITIES))
        raise VisualContractV4Error(
            f'structured predictions must cover the fixed fleet; '
            f'missing={missing}, unexpected={unexpected}'
        )
    return by_identity


def _canonicalize_slot(
    prediction: StructuredSlotPrediction,
    *,
    segment_length_contract: PublicSegmentLengthContract,
    image_size: tuple[float, float],
) -> CanonicalSlotPrediction:
    identity = str(prediction.identity).strip().upper()
    side = derive_side(identity)
    segment_index = _selected_index(
        prediction.segment_index,
        prediction.segment_scores,
        SEGMENT_CLASSES,
        f'{identity}.segment',
    )
    loaded_index = _selected_index(
        prediction.loaded_index,
        prediction.loaded_scores,
        LOADED_CLASSES,
        f'{identity}.loaded',
    )
    segment = SEGMENT_CLASSES[segment_index]
    loaded_state = LOADED_CLASSES[loaded_index]
    bbox_values = _finite_vector(prediction.bbox_xywh, f'{identity}.bbox_xywh')
    if len(bbox_values) != 4:
        raise VisualContractV4Error(f'{identity}.bbox_xywh must contain four values')
    bbox = _validated_bbox_xywh(
        bbox_values,
        image_size=image_size,
        context=f'{identity}.bbox_xywh',
    )
    raw_ratio = _finite_number(prediction.s_ratio, f'{identity}.s_ratio')
    if not 0.0 <= raw_ratio <= 1.0:
        raise VisualContractV4Error(f'{identity}.s_ratio must be in [0, 1]')
    s_ratio = raw_ratio
    segment_length = segment_length_contract.length_m(side, segment)
    return CanonicalSlotPrediction(
        identity=identity,
        side=side,
        segment_index=segment_index,
        segment=segment,
        loaded_index=loaded_index,
        loaded_state=loaded_state,
        bbox_xywh=bbox,
        s_ratio=s_ratio,
        segment_length_m=segment_length,
        s_m=s_ratio * segment_length,
    )


def _selected_index(
    chosen_index: int | None,
    scores: Sequence[float] | None,
    vocabulary: Sequence[str],
    context: str,
) -> int:
    if (chosen_index is None) == (scores is None):
        raise VisualContractV4Error(
            f'{context} requires exactly one of chosen index or scores'
        )
    if chosen_index is not None:
        if isinstance(chosen_index, bool) or not isinstance(chosen_index, int):
            raise VisualContractV4Error(f'{context} index must be an integer')
        if not 0 <= chosen_index < len(vocabulary):
            raise VisualContractV4Error(f'{context} index is outside its vocabulary')
        return chosen_index
    parsed_scores = _finite_vector(scores, f'{context}_scores')
    if len(parsed_scores) != len(vocabulary):
        raise VisualContractV4Error(
            f'{context} scores must contain {len(vocabulary)} values'
        )
    return max(range(len(parsed_scores)), key=parsed_scores.__getitem__)


def _acceptance_envelope(
    acceptance_by_identity: Mapping[str, SlotAcceptance] | None,
) -> AcceptanceEnvelope:
    if acceptance_by_identity is None:
        slots = tuple(
            SlotAcceptance(
                identity=identity,
                accepted=False,
                reasons=('missing_acceptance',),
            )
            for identity in FIXED_IDENTITIES
        )
    else:
        if not isinstance(acceptance_by_identity, Mapping):
            raise VisualContractV4Error('acceptance_by_identity must be a mapping')
        normalized: dict[str, SlotAcceptance] = {}
        for key, value in acceptance_by_identity.items():
            canonical_key = str(key).strip().upper()
            if canonical_key in normalized:
                raise VisualContractV4Error(
                    f'duplicate normalized acceptance identity: {canonical_key}'
                )
            normalized[canonical_key] = value
        if set(normalized) != set(FIXED_IDENTITIES):
            raise VisualContractV4Error(
                'explicit acceptance metadata must cover every fixed identity'
            )
        parsed: list[SlotAcceptance] = []
        for identity in FIXED_IDENTITIES:
            value = normalized[identity]
            if not isinstance(value, SlotAcceptance):
                raise VisualContractV4Error(
                    f'acceptance metadata for {identity} has invalid type'
                )
            if str(value.identity).strip().upper() != identity:
                raise VisualContractV4Error(
                    f'acceptance identity mismatch for {identity}'
                )
            if not isinstance(value.accepted, bool):
                raise VisualContractV4Error(
                    f'acceptance decision for {identity} must be boolean'
                )
            if not isinstance(value.required, bool):
                raise VisualContractV4Error(
                    f'acceptance required flag for {identity} must be boolean'
                )
            if not value.required and value.accepted:
                raise VisualContractV4Error(
                    f'non-required slot {identity} cannot be marked accepted'
                )
            if value.accepted and value.reasons:
                raise VisualContractV4Error(
                    f'accepted slot {identity} cannot have rejection reasons'
                )
            if not value.accepted and not value.reasons:
                raise VisualContractV4Error(
                    f'rejected slot {identity} requires at least one reason'
                )
            _optional_confidence(value.segment_confidence, f'{identity}.segment_confidence')
            _optional_confidence(value.loaded_confidence, f'{identity}.loaded_confidence')
            parsed.append(value)
        slots = tuple(parsed)
    required_identities = tuple(
        slot.identity for slot in slots if slot.required
    )
    accepted = all(slot.accepted for slot in slots if slot.required)
    reasons = tuple(
        f'{slot.identity}:{reason}'
        for slot in slots
        if slot.required
        for reason in slot.reasons
    )
    return AcceptanceEnvelope(
        schema_version=CONTRACT_SCHEMA_VERSION,
        accepted=accepted,
        slots=slots,
        required_identities=required_identities,
        reasons=reasons,
    )


def _optional_confidence(value: float | None, context: str) -> None:
    if value is None:
        return
    parsed = _finite_number(value, context)
    if not 0.0 <= parsed <= 1.0:
        raise VisualContractV4Error(f'{context} must be in [0, 1]')


def _canonical_image_sizes(
    image_sizes_by_side: Mapping[str, Sequence[float]],
) -> dict[str, tuple[float, float]]:
    if not isinstance(image_sizes_by_side, Mapping):
        raise VisualContractV4Error(
            'image_sizes_by_side must be a mapping with exact left/right keys'
        )
    actual_sides = set(image_sizes_by_side)
    expected_sides = set(SIDES)
    if actual_sides != expected_sides:
        missing = sorted(expected_sides - actual_sides)
        unexpected = sorted(repr(value) for value in actual_sides - expected_sides)
        raise VisualContractV4Error(
            'image_sizes_by_side vocabulary mismatch; '
            f'missing={missing}, unexpected={unexpected}'
        )
    result: dict[str, tuple[float, float]] = {}
    for side in SIDES:
        size = _finite_vector(image_sizes_by_side[side], f'{side}.image_size')
        if len(size) != 2:
            raise VisualContractV4Error(
                f'{side}.image_size must contain (width, height)'
            )
        width, height = size
        if width <= 0.0 or height <= 0.0:
            raise VisualContractV4Error(
                f'{side}.image_size must contain positive dimensions'
            )
        result[side] = (width, height)
    return result


def _validated_bbox_xywh(
    bbox_xywh: Sequence[float],
    *,
    image_size: tuple[float, float],
    context: str,
) -> tuple[float, float, float, float]:
    x, y, width, height = bbox_xywh
    if x < 0.0 or y < 0.0:
        raise VisualContractV4Error(f'{context} x/y must be non-negative')
    if width <= 0.0 or height <= 0.0:
        raise VisualContractV4Error(f'{context} width/height must be positive')
    image_width, image_height = image_size
    if x + width > image_width or y + height > image_height:
        raise VisualContractV4Error(
            f'{context} must be enclosed by image size '
            f'(width={image_width}, height={image_height})'
        )
    return (x, y, width, height)


def _finite_number(value: Any, context: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise VisualContractV4Error(f'{context} must be numeric') from exc
    if not math.isfinite(parsed):
        raise VisualContractV4Error(f'{context} must be finite')
    return parsed


def _positive_finite_length(value: Any, context: str) -> float:
    parsed = _finite_number(value, f'{context}.length_m')
    if parsed <= 0.0:
        raise VisualContractV4Error(
            f'public segment length must be positive: {context}'
        )
    return parsed


def _finite_vector(values: Any, context: str) -> tuple[float, ...]:
    if values is None or isinstance(values, (str, bytes)):
        raise VisualContractV4Error(f'{context} must be a numeric sequence')
    try:
        return tuple(_finite_number(value, context) for value in values)
    except TypeError as exc:
        raise VisualContractV4Error(f'{context} must be a numeric sequence') from exc


def _mask_vector(values: Sequence[float]) -> tuple[int, ...]:
    parsed = _finite_vector(values, 'target_mask')
    result: list[int] = []
    for value in parsed:
        if value not in (0.0, 1.0):
            raise VisualContractV4Error('target_mask values must be exactly zero or one')
        result.append(int(value))
    return tuple(result)


def _categorical_mask_available(
    mask: Sequence[int],
    indexes: Mapping[str, int],
    names: Sequence[str],
    identity: str,
) -> bool:
    values = {int(mask[indexes[name]]) for name in names}
    if len(values) != 1:
        raise VisualContractV4Error(
            f'categorical target mask must cover its complete group: {identity}'
        )
    return values == {1}


def _one_hot_index(
    vector: Sequence[float],
    indexes: Mapping[str, int],
    names: Sequence[str],
    context: str,
) -> int:
    values = [float(vector[indexes[name]]) for name in names]
    active = [index for index, value in enumerate(values) if value == 1.0]
    if len(active) != 1 or any(value not in (0.0, 1.0) for value in values):
        raise VisualContractV4Error(f'{context} target must be exactly one-hot')
    return active[0]


__all__ = (
    'AcceptanceEnvelope',
    'CONTRACT_SCHEMA_VERSION',
    'CanonicalSlotPrediction',
    'FIXED_IDENTITIES',
    'LEGACY_SCHEMA_VERSION',
    'LEGACY_VECTOR_DIMENSION',
    'LEGACY_VECTOR_NAMES',
    'LOADED_CLASSES',
    'LegacyCompatibilityOutput',
    'PUBLIC_SEGMENT_LENGTH_CONTRACT_SCHEMA',
    'PUBLIC_SEGMENT_LENGTH_SOURCE',
    'PublicSegmentLengthContract',
    'SEGMENT_CLASSES',
    'SIDES',
    'SlotAcceptance',
    'StructuredSlotPrediction',
    'StructuredSlotTarget',
    'StructuredTargetMask',
    'TEST_FIXTURE_SEGMENT_LENGTH_SOURCE',
    'VisualContractV4Error',
    'assemble_legacy_200',
    'decode_legacy_targets',
    'derive_side',
    'load_authoritative_public_segment_length_contract',
    'make_test_public_segment_length_contract',
    'validate_legacy_vectorizer',
)
