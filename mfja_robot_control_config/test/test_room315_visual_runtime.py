import hashlib
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from room_315_presence_provider import PRESENCE_ABSENT
from room_315_presence_provider import PRESENCE_PRESENT
from room_315_presence_provider import PRESENCE_UNKNOWN
from room_315_presence_provider import PresenceEntry
from room_315_presence_provider import PresenceSnapshot
from room_315_presence_provider import ShuttleStatePresenceProvider
from room_315_contracts import ObservedFact
import room_315_pddl_scenario_generator as pddl_generator
from room_315_visual_runtime import ArtifactHashes
from room_315_visual_runtime import ArtifactPaths
from room_315_visual_runtime import DecodedShuttlePrediction
from room_315_visual_runtime import DecodedVisualPrediction
from room_315_visual_runtime import FIXED_IDENTITY_ORDER
from room_315_visual_runtime import VisualRuntimeError
from room_315_visual_runtime import denormalize_output
from room_315_visual_runtime import decode_active_slots
from room_315_visual_runtime import preprocess_rgb_image
from room_315_visual_runtime import preprocess_paired_rgb
from room_315_visual_runtime import verify_artifacts
from room_315_visual_runtime_fusion import DeterministicPlanSys2FactGate
from room_315_visual_runtime_fusion import fuse_validated_visual_state
from room_315_visual_runtime_validation import ValidationConfig
from room_315_visual_runtime_validation import ValidationResult
from room_315_visual_runtime_validation import validate_prediction


APPROVED = Path(
    '/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run'
)
APPROVED_HASHES = {
    'best.pt': (
        '8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d'
    ),
    'target_stats.json': '2d48078641842aa2db7a59b9285fc5bbedaaa3a0039fc39986ca230db983b18c',
    'visual_label_vectorizer.json': '637c854556f3331c4e187db4aa7fc70457f01df8877947b9a0e988a543f7113e',
    'training_config.json': '5c45544af7766afff397dafa7c14c0b3b05083f07a93122308ef50c2e8f452eb',
    'run_metadata.json': 'd86c0ebfda3f5b174fc3c06f4ce8a3e083d2048db7b44d20efe951aaa7e5428d',
}


def _provider(timeout=1.0, warmup=0.1):
    return ShuttleStatePresenceProvider(timeout_s=timeout, warmup_s=warmup)


def _observe(provider, side, name, at):
    provider.observe(
        topic_side=side,
        entity_name=name,
        source_stamp_s=at,
        receive_time_s=at,
    )


def _initialize(provider, at=1.0):
    _observe(provider, 'left', 'room315_left_shuttle_1', at)
    _observe(provider, 'right', 'room315_right_shuttle_1', at)


def _ready_snapshot(present=('L1', 'R1'), now=10.0):
    entries = []
    for identity in FIXED_IDENTITY_ORDER:
        side = 'left' if identity.startswith('L') else 'right'
        entries.append(PresenceEntry(
            identity=identity,
            side=side,
            state=PRESENCE_PRESENT if identity in present else PRESENCE_ABSENT,
        ))
    return PresenceSnapshot(
        timestamp_s=now,
        ready=True,
        entries=tuple(entries),
        reasons=(),
        initialized_sides=('left', 'right'),
        stale_sides=(),
        source='test_controller_presence',
    )


def _prediction(
    *,
    now=10.0,
    identities=('L1', 'R1'),
    block='A1E',
    side_override=None,
    bbox=(10.0, 20.0, 30.0, 40.0),
    s_m=0.5,
    s_ratio=0.5,
    segment_length_m=1.0,
):
    shuttles = []
    for identity in identities:
        side = side_override or ('left' if identity.startswith('L') else 'right')
        shuttles.append(DecodedShuttlePrediction(
            identity=identity,
            side=side,
            block=block,
            bbox_xywh=bbox,
            s_m=s_m,
            s_ratio=s_ratio,
            segment_length_m=segment_length_m,
            loaded_state='loaded' if identity == 'L1' else 'empty',
        ))
    return DecodedVisualPrediction(
        timestamp_s=now,
        left_image_stamp_s=now,
        right_image_stamp_s=now,
        left_image_size=(640, 480),
        right_image_size=(640, 480),
        shuttles=tuple(shuttles),
        active_identities=tuple(identities),
        absent_identities=tuple(
            item for item in FIXED_IDENTITY_ORDER if item not in identities
        ),
    )


def _accepted_result(prediction):
    return ValidationResult(
        accepted=True,
        reasons=(),
        clamped_fields=(),
        topology_consistent=True,
        timestamp_consistent=True,
        artifact_healthy=True,
        input_healthy=True,
        presence_ready=True,
        prediction=prediction,
    )


def test_fresh_present_and_explicitly_absent_shuttles():
    provider = _provider()
    _initialize(provider)
    snapshot = provider.snapshot(now_s=1.2)
    assert snapshot.ready
    assert snapshot.by_identity()['L1'].state == PRESENCE_PRESENT
    assert snapshot.by_identity()['R1'].state == PRESENCE_PRESENT
    assert snapshot.by_identity()['L2'].state == PRESENCE_ABSENT


def test_blank_empty_rail_heartbeat_initializes_zero_shuttle_side():
    provider = _provider()
    _observe(provider, 'left', '', 1.0)
    _observe(provider, 'right', 'room315_right_shuttle_3', 1.0)

    snapshot = provider.snapshot(now_s=1.2)

    assert snapshot.ready
    assert snapshot.reasons == ()
    assert snapshot.initialized_sides == ('left', 'right')
    assert snapshot.by_identity()['R3'].state == PRESENCE_PRESENT
    assert all(
        snapshot.by_identity()[identity].state == PRESENCE_ABSENT
        for identity in ('L1', 'L2', 'L3', 'L4')
    )


def test_blank_heartbeats_can_represent_two_fresh_empty_rails():
    provider = _provider()
    _observe(provider, 'left', '', 1.0)
    _observe(provider, 'right', '', 1.0)

    snapshot = provider.snapshot(now_s=1.2)

    assert snapshot.ready
    assert snapshot.reasons == ()
    assert all(entry.state == PRESENCE_ABSENT for entry in snapshot.entries)


def test_blank_heartbeat_does_not_make_a_missing_other_side_ready():
    provider = _provider()
    _observe(provider, 'left', '', 1.0)

    snapshot = provider.snapshot(now_s=1.2)

    assert not snapshot.ready
    assert 'presence_topic_not_initialized:right' in snapshot.reasons
    assert all(entry.state == PRESENCE_UNKNOWN for entry in snapshot.entries)


def test_topic_not_initialized_is_unknown_and_not_ready():
    provider = _provider()
    _observe(provider, 'left', 'room315_left_shuttle_1', 1.0)
    snapshot = provider.snapshot(now_s=1.2)
    assert not snapshot.ready
    assert all(entry.state == PRESENCE_UNKNOWN for entry in snapshot.entries)
    assert 'presence_topic_not_initialized:right' in snapshot.reasons


def test_complete_stale_presence_source_is_unknown():
    provider = _provider(timeout=1.0)
    _initialize(provider)
    snapshot = provider.snapshot(now_s=2.2)
    assert not snapshot.ready
    assert all(entry.state == PRESENCE_UNKNOWN for entry in snapshot.entries)
    assert set(snapshot.stale_sides) == {'left', 'right'}


def test_fresh_receive_with_stale_source_headers_is_unknown():
    provider = _provider(timeout=1.0, warmup=0.0)
    _initialize(provider, at=1.0)
    _observe(provider, 'left', 'room315_left_shuttle_1', 3.0)
    provider.observe(
        topic_side='right',
        entity_name='room315_right_shuttle_1',
        source_stamp_s=1.0,
        receive_time_s=3.0,
    )
    snapshot = provider.snapshot(now_s=3.1)
    assert not snapshot.ready
    assert snapshot.stale_sides == ('right',)
    assert all(entry.state == PRESENCE_UNKNOWN for entry in snapshot.entries)


@pytest.mark.parametrize(
    ('side', 'name', 'token'),
    [
        ('left', 'unknown_cart', 'unknown_presence_entity'),
        ('left', 'room315_right_shuttle_1', 'presence_identity_side_conflict'),
    ],
)
def test_unknown_entity_and_identity_side_conflict_fail_closed(side, name, token):
    provider = _provider()
    _initialize(provider)
    _observe(provider, side, name, 1.1)
    snapshot = provider.snapshot(now_s=1.2)
    assert not snapshot.ready
    assert any(token in reason for reason in snapshot.reasons)


def test_duplicate_identity_aliases_fail_closed():
    provider = _provider()
    _initialize(provider)
    _observe(provider, 'left', 'L1', 1.1)
    snapshot = provider.snapshot(now_s=1.2)
    assert not snapshot.ready
    assert any('duplicate_presence_identity:L1' in item for item in snapshot.reasons)


def test_removed_shuttle_times_out_and_readded_shuttle_becomes_present():
    provider = _provider(timeout=1.0)
    _initialize(provider)
    _observe(provider, 'left', 'room315_left_shuttle_2', 1.0)
    _observe(provider, 'right', 'room315_right_shuttle_1', 1.8)
    _observe(provider, 'left', 'room315_left_shuttle_1', 1.8)
    removed = provider.snapshot(now_s=2.1)
    assert removed.ready
    assert removed.by_identity()['L2'].state == PRESENCE_ABSENT
    _observe(provider, 'left', 'room315_left_shuttle_2', 2.1)
    readded = provider.snapshot(now_s=2.2)
    assert readded.by_identity()['L2'].state == PRESENCE_PRESENT


def test_runtime_preprocessing_matches_training_and_preserves_channel_order():
    training_path = SCRIPTS / 'room_315_visual_state_train_local.py'
    spec = importlib.util.spec_from_file_location('room315_training_preprocess', training_path)
    training = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(training)

    left = np.zeros((13, 17, 3), dtype=np.uint8)
    right = np.zeros((13, 17, 3), dtype=np.uint8)
    left[..., 0] = 255
    right[..., 2] = 255
    runtime = preprocess_paired_rgb(left, right)

    # Exercise the exact training path without reading any dataset split.
    left_path = Path('/tmp/room315_runtime_left.png')
    right_path = Path('/tmp/room315_runtime_right.png')
    Image.fromarray(left).save(left_path)
    Image.fromarray(right).save(right_path)
    row = {
        'model_input': {
            'overhead_images': {
                'left_rail_rgb': left_path.name,
                'right_rail_rgb': right_path.name,
            }
        }
    }
    expected = training.load_paired_images(
        row,
        Path('/tmp'),
        width=224,
        height=224,
        normalization_mean=(0.485, 0.456, 0.406),
        normalization_std=(0.229, 0.224, 0.225),
    )
    assert runtime.shape == (6, 224, 224)
    np.testing.assert_allclose(runtime, expected, rtol=0.0, atol=1e-7)
    assert runtime[0].mean() > runtime[2].mean()
    assert runtime[5].mean() > runtime[3].mean()


@pytest.mark.parametrize(
    'image',
    [
        np.zeros((10, 10), dtype=np.uint8),
        np.zeros((10, 10, 4), dtype=np.uint8),
        np.full((10, 10, 3), 2.0, dtype=np.float32),
        np.full((10, 10, 3), np.nan, dtype=np.float32),
    ],
)
def test_malformed_runtime_images_fail_closed(image):
    with pytest.raises(VisualRuntimeError):
        preprocess_rgb_image(image)


def test_denormalization_and_nonfinite_output_rejection():
    mean = np.arange(200, dtype=np.float32)
    std = np.full(200, 2.0, dtype=np.float32)
    np.testing.assert_array_equal(
        denormalize_output(np.ones(200), mean, std),
        mean + 2.0,
    )
    bad = np.zeros(200)
    bad[9] = np.nan
    with pytest.raises(VisualRuntimeError, match='non-finite'):
        denormalize_output(bad, mean, std)


def test_unknown_presence_is_rejected_before_slot_decoding():
    unknown = _ready_snapshot()
    entries = list(unknown.entries)
    entries[0] = PresenceEntry(
        identity='L1',
        side='left',
        state=PRESENCE_UNKNOWN,
    )
    malformed = PresenceSnapshot(
        **{
            **unknown.__dict__,
            'ready': True,
            'entries': tuple(entries),
        }
    )

    class _NeverUsedVectorizer:
        @property
        def names(self):
            raise AssertionError('unknown presence must gate before vector decoding')

    with pytest.raises(VisualRuntimeError, match='presence is unknown'):
        decode_active_slots(
            np.zeros(200, dtype=np.float32),
            vectorizer=_NeverUsedVectorizer(),
            presence=malformed,
            timestamp_s=10.0,
            left_image_stamp_s=10.0,
            right_image_stamp_s=10.0,
            left_image_size=(640, 480),
            right_image_size=(640, 480),
        )


def test_validation_accepts_valid_fixed_order_and_rejects_side_bbox_position():
    presence = _ready_snapshot()
    accepted = validate_prediction(_prediction(), presence, now_s=10.0)
    assert accepted.accepted

    wrong_side = validate_prediction(
        _prediction(identities=('L1',), side_override='right'),
        _ready_snapshot(present=('L1',)),
        now_s=10.0,
    )
    assert not wrong_side.accepted
    assert any('identity_side_conflict' in reason for reason in wrong_side.reasons)

    bbox = validate_prediction(
        _prediction(bbox=(700.0, 20.0, 30.0, 40.0)),
        presence,
        now_s=10.0,
    )
    assert not bbox.accepted
    assert any('bbox_outside_image' in reason for reason in bbox.reasons)

    position = validate_prediction(
        _prediction(s_m=0.9, s_ratio=0.1),
        presence,
        now_s=10.0,
    )
    assert not position.accepted
    assert any('s_m_s_ratio_inconsistent' in reason for reason in position.reasons)


def test_bounded_position_consistency_projection_uses_learned_outputs_only():
    result = validate_prediction(
        _prediction(s_m=0.6, s_ratio=0.3, segment_length_m=1.0),
        _ready_snapshot(),
        now_s=10.0,
        config=ValidationConfig(
            reconcile_position_consistency=True,
            max_position_reconciliation_error_m=0.4,
        ),
    )

    assert result.accepted
    assert result.prediction is not None
    assert all(
        shuttle.s_ratio == pytest.approx(0.6)
        for shuttle in result.prediction.shuttles
    )
    assert {
        'L1.s_ratio_consistency_projection',
        'R1.s_ratio_consistency_projection',
    }.issubset(result.clamped_fields)


def test_position_consistency_projection_still_fails_above_bound():
    result = validate_prediction(
        _prediction(s_m=0.9, s_ratio=0.1, segment_length_m=1.0),
        _ready_snapshot(),
        now_s=10.0,
        config=ValidationConfig(
            reconcile_position_consistency=True,
            max_position_reconciliation_error_m=0.4,
            position_reconciliation_policy='bounded_s_m',
        ),
    )

    assert not result.accepted
    assert any(
        's_m_s_ratio_inconsistent' in reason
        for reason in result.reasons
    )


def test_canonical_s_m_reconciles_live_r1_disagreement_without_oracle_position():
    prediction = _prediction(
        identities=('R1',),
        block='A12E',
        s_m=1.025031328201294,
        s_ratio=0.2509480118751526,
        segment_length_m=2.4118762016296387,
    )
    result = validate_prediction(
        prediction,
        _ready_snapshot(present=('R1',)),
        now_s=10.0,
        config=ValidationConfig(
            reconcile_position_consistency=True,
            max_position_reconciliation_error_m=0.4,
            position_reconciliation_policy='canonical_s_m',
        ),
    )

    assert result.accepted
    assert result.prediction is not None
    assert prediction.shuttles[0].s_ratio == pytest.approx(0.2509480118751526)
    assert result.prediction.shuttles[0].s_ratio == pytest.approx(
        1.025031328201294 / 2.4118762016296387
    )
    assert result.prediction.shuttles[0].s_m == pytest.approx(
        1.025031328201294
    )
    assert 'R1.s_ratio_consistency_projection' in result.clamped_fields
    assert 'R1.s_ratio_large_disagreement_projected' in result.clamped_fields
    assert not any('s_m_s_ratio_inconsistent' in reason for reason in result.reasons)


def test_canonical_s_m_still_rejects_physical_arclength_outside_segment():
    result = validate_prediction(
        _prediction(s_m=1.2, s_ratio=0.2, segment_length_m=1.0),
        _ready_snapshot(),
        now_s=10.0,
        config=ValidationConfig(
            reconcile_position_consistency=True,
            position_reconciliation_policy='canonical_s_m',
        ),
    )

    assert not result.accepted
    assert any('s_m_exceeds_segment_length' in reason for reason in result.reasons)


def test_unknown_position_reconciliation_policy_fails_configuration():
    with pytest.raises(
        ValueError,
        match='position_reconciliation_policy',
    ):
        ValidationConfig(position_reconciliation_policy='oracle')


def test_stale_and_skewed_inputs_fail_closed():
    presence = _ready_snapshot()
    stale = validate_prediction(_prediction(now=8.0), presence, now_s=10.0)
    assert not stale.accepted
    assert 'prediction_is_stale' in stale.reasons
    pred = _prediction()
    skewed = DecodedVisualPrediction(
        **{
            **pred.__dict__,
            'right_image_stamp_s': 9.0,
        }
    )
    result = validate_prediction(
        skewed,
        presence,
        now_s=10.0,
        config=ValidationConfig(stale_image_timeout_s=2.0),
    )
    assert not result.accepted
    assert 'paired_image_timestamp_skew_exceeded' in result.reasons


def test_absent_slots_never_contribute_visual_facts():
    presence = _ready_snapshot(present=('L1',))
    prediction = _prediction(identities=('L1',))
    fusion = fuse_validated_visual_state(
        _accepted_result(prediction),
        presence,
        checkpoint_sha256='a' * 64,
        schema_version='room315.visual_state.v3',
        stale_after_s=1.0,
        state_id='test-state',
    )
    assert fusion.ready
    state = fusion.observed_state
    assert state is not None
    visual_subjects = {
        fact.subject for fact in state.visual_model_inputs
    }
    assert visual_subjects == {'room315_left_shuttle_1'}
    absent = [
        fact
        for fact in state.fused_planner_state
        if fact.predicate == 'present' and fact.value is False
    ]
    assert len(absent) == 7


def test_pddl_snapshot_does_not_require_or_emit_visual_facts_for_absent_slots():
    def fact(subject, predicate, value, index):
        return ObservedFact(
            fact_id=f'fact-{index}',
            subject=subject,
            predicate=predicate,
            value=value,
            source='state_fuser',
            timestamp=10.0,
            confidence=1.0,
            status='known',
        )

    facts = []
    for index, shuttle in enumerate(pddl_generator.all_shuttle_specs()):
        facts.append(fact(shuttle.gazebo_entity_name, 'present', False, index))
    for side_index, side in enumerate(pddl_generator.SIDES):
        for slot_index, slot in enumerate(('1', '2', '3', '4')):
            facts.append(fact(
                pddl_generator._contract_slot_id(side, slot),
                'occupancy',
                False,
                100 + side_index * 4 + slot_index,
            ))
    index = {(item.subject, item.predicate): item for item in facts}
    fleet = pddl_generator._fleet_snapshot_from_observed_state(index)
    assert not any(fleet['present_by_shuttle'].values())
    assert not any(fleet['loaded_by_shuttle'].values())

    init_facts = pddl_generator._pddl_init_facts(
        fleet=fleet,
        devices={'switches': {}, 'stoppers': {}},
        obstacles={'left': [], 'right': []},
        blocks={'blocks': {}, 'block_occupancy': {}, 'block_reservations': {}},
        goal_data={
            'goal_type': 'inspection',
            'inspection_subject': 'room315',
            'target_slot': '',
            'candidate_shuttles': [],
        },
        route_clearance={
            'clear_pairs': set(),
            'blockers_by_pair': {},
            'ordering_pairs': set(),
        },
    )
    for shuttle in pddl_generator.all_shuttle_specs():
        assert not any(shuttle.shuttle_id in item for item in init_facts)


def test_unknown_presence_blocks_fusion_and_plansys_updates():
    unknown = PresenceSnapshot(
        timestamp_s=10.0,
        ready=False,
        entries=tuple(
            PresenceEntry(
                identity=identity,
                side='left' if identity.startswith('L') else 'right',
                state=PRESENCE_UNKNOWN,
            )
            for identity in FIXED_IDENTITY_ORDER
        ),
        reasons=('presence_source_stale:left',),
        initialized_sides=('left', 'right'),
        stale_sides=('left',),
        source='test',
    )
    validation = validate_prediction(None, unknown, now_s=10.0)
    fusion = fuse_validated_visual_state(
        validation,
        unknown,
        checkpoint_sha256='a' * 64,
        schema_version='room315.visual_state.v3',
        stale_after_s=1.0,
        state_id='blocked',
    )
    assert not fusion.ready
    update = DeterministicPlanSys2FactGate().build_update(
        fusion,
        model_ready=True,
        input_ready=True,
        safety_ready=True,
        enabled=True,
    )
    assert not update.accepted
    assert not update.add_predicates


def test_plansys_gate_maps_only_existing_visual_predicates():
    presence = _ready_snapshot(present=('L1',))
    prediction = _prediction(identities=('L1',))
    fusion = fuse_validated_visual_state(
        _accepted_result(prediction),
        presence,
        checkpoint_sha256='a' * 64,
        schema_version='room315.visual_state.v3',
        stale_after_s=1.0,
        state_id='accepted',
    )
    update = DeterministicPlanSys2FactGate().build_update(
        fusion,
        model_ready=True,
        input_ready=True,
        safety_ready=True,
        enabled=True,
    )
    assert update.accepted
    assert '(shuttle_on_side l1 left)' in update.add_predicates
    assert '(loaded l1)' in update.add_predicates
    assert all('command' not in item for item in update.add_predicates)


def test_approved_artifact_hashes_and_metadata_verify_without_checkpoint_load():
    if not APPROVED.is_dir():
        pytest.skip('approved artifact directory is unavailable')
    for name, expected in APPROVED_HASHES.items():
        digest = hashlib.sha256((APPROVED / name).read_bytes()).hexdigest()
        assert digest == expected
    artifacts = verify_artifacts(
        ArtifactPaths(APPROVED / 'best.pt', APPROVED),
        ArtifactHashes(
            checkpoint=APPROVED_HASHES['best.pt'],
            target_stats=APPROVED_HASHES['target_stats.json'],
            vectorizer=APPROVED_HASHES['visual_label_vectorizer.json'],
            training_config=APPROVED_HASHES['training_config.json'],
            run_metadata=APPROVED_HASHES['run_metadata.json'],
        ),
    )
    assert artifacts.vectorizer.dim == 200
    assert tuple(artifacts.vectorizer_json['fixed_identity_order']) == FIXED_IDENTITY_ORDER


def test_approved_checkpoint_strict_load_and_repeated_cpu_inference_are_deterministic():
    pytest.importorskip('torch')
    pytest.importorskip('torchvision')
    if not APPROVED.is_dir():
        pytest.skip('approved artifact directory is unavailable')
    artifacts = verify_artifacts(
        ArtifactPaths(APPROVED / 'best.pt', APPROVED),
        ArtifactHashes(
            checkpoint=APPROVED_HASHES['best.pt'],
            target_stats=APPROVED_HASHES['target_stats.json'],
            vectorizer=APPROVED_HASHES['visual_label_vectorizer.json'],
            training_config=APPROVED_HASHES['training_config.json'],
            run_metadata=APPROVED_HASHES['run_metadata.json'],
        ),
    )
    from room_315_visual_runtime import Room315VisualModelRuntime

    runtime = Room315VisualModelRuntime(artifacts, device='cpu')
    runtime.load()
    rows, columns = np.indices((96, 128))
    image = np.stack(
        (
            (rows * 3 + columns) % 256,
            (rows + columns * 5) % 256,
            (rows * 7 + columns * 11) % 256,
        ),
        axis=-1,
    ).astype(np.uint8)
    first, _first_timings = runtime.infer(image, image[:, ::-1])
    second, _second_timings = runtime.infer(image, image[:, ::-1])
    np.testing.assert_array_equal(first, second)
    assert first.shape == (200,)
    assert runtime.ready
    assert not runtime.model.training


def test_missing_and_wrong_artifact_hashes_fail_closed(tmp_path):
    with pytest.raises(VisualRuntimeError, match='missing'):
        verify_artifacts(
            ArtifactPaths(tmp_path / 'best.pt', tmp_path),
            ArtifactHashes('a', 'b', 'c', 'd', 'e'),
        )
    if not APPROVED.is_dir():
        pytest.skip('approved artifact directory is unavailable')
    with pytest.raises(VisualRuntimeError, match='mismatch'):
        verify_artifacts(
            ArtifactPaths(APPROVED / 'best.pt', APPROVED),
            ArtifactHashes(
                checkpoint='0' * 64,
                target_stats=APPROVED_HASHES['target_stats.json'],
                vectorizer=APPROVED_HASHES['visual_label_vectorizer.json'],
                training_config=APPROVED_HASHES['training_config.json'],
                run_metadata=APPROVED_HASHES['run_metadata.json'],
            ),
        )


def test_runtime_sources_do_not_open_or_name_locked_test_split():
    runtime_paths = [
        SCRIPTS / 'room_315_presence_provider.py',
        SCRIPTS / 'room_315_visual_model.py',
        SCRIPTS / 'room_315_visual_runtime.py',
        SCRIPTS / 'room_315_visual_runtime_validation.py',
        SCRIPTS / 'room_315_visual_runtime_fusion.py',
        SCRIPTS / 'room_315_visual_state_inference_node.py',
    ]
    for path in runtime_paths:
        text = path.read_text(encoding='utf-8').lower()
        assert 'test.jsonl' not in text
        assert 'test_visual_labels.jsonl' not in text


def test_ros_presence_adapter_reads_no_deterministic_localization_fields():
    path = SCRIPTS / 'room_315_visual_state_inference_node.py'
    text = path.read_text(encoding='utf-8')
    callback = text.split(
        '    def _on_presence(',
        1,
    )[1].split(
        '    def _on_safety_status(',
        1,
    )[0]
    for field in (
        'mode',
        'current_segment',
        's',
        'x',
        'y',
        'z',
        'yaw',
        'speed',
    ):
        assert f'message.{field}' not in callback


def test_cpu_smoke_rejects_any_test_role_path():
    import room_315_visual_runtime_cpu_smoke as cpu_smoke

    for path in (
        Path('/dataset/test.jsonl'),
        Path('/dataset/test_visual_labels.jsonl'),
        Path('/dataset/locked_test/labels.jsonl'),
        Path('/dataset/TestAnything.jsonl'),
    ):
        with pytest.raises(ValueError, match='locked Test split'):
            cpu_smoke._reject_locked_test_path(path)
