#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import hashlib
import io
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
SCRIPT_PATH = SCRIPT_DIR / 'room_315_task_goal_builder.py'


def _load_builder():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location('room_315_task_goal_builder', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _envelope(**patch):
    return json.dumps({
        'contract_type': 'SemanticParseEnvelope',
        'dialogue_act': 'new_goal',
        'draft_patch': patch,
        'evidence': {field: f'evidence for {field}' for field in patch},
        'provenance': {field: 'semantic_inference' for field in patch},
        'confidence': {'overall': 0.82},
    })


def _pipeline(response='{}', *, shadow=False, unhealthy=False, timeout=False):
    builder = _load_builder()
    from room_315_task_goal_semantic import FakeSemanticBackend
    from room_315_task_goal_semantic import LocalSemanticModelConfig

    backend = FakeSemanticBackend({'*': response}, unhealthy=unhealthy, timeout=timeout)
    config = LocalSemanticModelConfig(shadow_mode=shadow)
    return builder.ParserPipeline(semantic_backend=backend, semantic_config=config), backend


def test_semantic_backend_invoked_and_validated_with_provenance():
    builder = _load_builder()
    pipeline, backend = _pipeline(_envelope(
        goal_type='transport',
        payload_filter='loaded',
        side='left',
        target_kind='slot',
        target_slot='2',
    ))

    parsed = pipeline.parse('Could you please put a loaded shuttle on the left slot 2?')
    assert parsed.ok, parsed.to_dict()
    assert backend.calls
    trace = parsed.raw_output['trace']
    assert trace['model_backend'] == 'fake_semantic'
    assert trace['model_fingerprint'] == 'fake-semantic-fingerprint'
    assert trace['model_ready'] is True
    assert trace['semantic_model_invoked'] is True
    assert trace['fallback_used'] is False
    assert trace['field_provenance']['goal_type'] == 'semantic_inference'

    validation = builder.Room315DomainValidator().validate(parsed.draft)
    assert validation.ok, validation.to_dict()


def test_explicit_facts_override_semantic_model_disagreement():
    pipeline, _backend = _pipeline(_envelope(
        goal_type='transport',
        side='left',
        target_kind='slot',
        target_slot='4',
        target_shuttle='L2',
    ))

    parsed = pipeline.parse('move R1 on the right rail to slot 3')
    assert parsed.ok, parsed.to_dict()
    assert parsed.draft.side == 'right'
    assert parsed.draft.target_slot == '3'
    assert parsed.draft.target_shuttle == 'R1'
    disagreements = parsed.raw_output['trace']['parser_disagreements']
    assert {item['field'] for item in disagreements} >= {'side', 'target_slot', 'target_shuttle'}


def test_invalid_semantic_json_falls_back_to_deterministic_evidence():
    pipeline, _backend = _pipeline('{not json')

    parsed = pipeline.parse('move the nearest right shuttle to slot 2')
    assert parsed.ok, parsed.to_dict()
    trace = parsed.raw_output['trace']
    assert trace['fallback_reason'] == 'invalid_semantic_json'
    assert trace['fallback_used'] is True
    assert parsed.draft.selection_strategy == 'nearest'
    assert parsed.draft.side == 'right'
    assert parsed.draft.target_slot == '2'


def test_forbidden_semantic_fields_are_rejected_before_fusion():
    pipeline, _backend = _pipeline(json.dumps({
        'contract_type': 'SemanticParseEnvelope',
        'dialogue_act': 'new_goal',
        'draft_patch': {'goal_type': 'transport'},
        'plan': ['move_shuttle'],
    }))

    parsed = pipeline.parse('move the nearest right shuttle to slot 2')
    assert parsed.ok, parsed.to_dict()
    assert parsed.raw_output['trace']['fallback_reason'] == 'forbidden_semantic_field'


def test_timeout_and_unhealthy_model_do_not_block_explicit_request():
    for kwargs, reason in [({'timeout': True}, 'timeout'), ({'unhealthy': True}, 'fake_unhealthy')]:
        pipeline, _backend = _pipeline(_envelope(goal_type='transport'), **kwargs)
        parsed = pipeline.parse('move the nearest right shuttle to slot 2')
        assert parsed.ok, parsed.to_dict()
        assert parsed.raw_output['trace']['fallback_reason'] == reason


def test_shadow_mode_logs_disagreement_without_affecting_draft():
    pipeline, _backend = _pipeline(_envelope(
        goal_type='transport',
        side='left',
        target_kind='slot',
        target_slot='4',
    ), shadow=True)

    parsed = pipeline.parse('move the nearest right shuttle to slot 2')
    assert parsed.ok, parsed.to_dict()
    assert parsed.draft.side == 'right'
    assert parsed.draft.target_slot == '2'
    trace = parsed.raw_output['trace']
    assert trace['shadow_mode'] is True
    assert trace['parser_disagreements']


def test_conflicting_explicit_facts_require_clarification():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(goal_type='transport'))
    parsed = pipeline.parse('move the right and left shuttle to slot 2')
    assert not parsed.ok
    assert parsed.issues[0].code == 'ambiguous_explicit_conflict'

    result = builder._result_for_schema_issues(parsed.issues, normalized_request={'parser': parsed.parser_name})
    assert result.status == 'clarification_required'


def test_dialogue_confirmation_correction_cancel_and_restart():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(goal_type='transport'))
    manager = builder.TaskGoalDialogueManager(parser=pipeline, max_attempts=3)

    first = manager.handle('move the nearest loaded right shuttle to slot 3', timestamp=1.0)
    assert first.status == 'confirmation_required'
    assert 'Action: Transport' in first.confirmation_prompt
    assert first.task_goal is None

    corrected = manager.handle('actually use the left rail', state=first.state, timestamp=2.0)
    assert corrected.status in {'clarification_required', 'confirmation_required'}
    assert corrected.task_goal is None

    cancelled = manager.handle('cancel', state=corrected.state, timestamp=3.0)
    assert cancelled.status == 'cancelled'
    assert cancelled.task_goal is None

    restarted = manager.handle('restart', state=cancelled.state, timestamp=4.0)
    assert restarted.status == 'clarification_required'
    assert restarted.task_goal is None


def test_dialogue_final_task_goal_requires_explicit_confirmation():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(goal_type='transport'))
    manager = builder.TaskGoalDialogueManager(parser=pipeline)

    first = manager.handle('move the nearest loaded right shuttle to slot 3', timestamp=1.0)
    assert first.status == 'confirmation_required'
    assert first.task_goal is None

    final = manager.handle('yes', state=first.state, timestamp=2.0)
    assert final.ok, final.to_dict()
    assert final.task_goal.constraints['selection_strategy'] == 'nearest'
    assert final.task_goal.constraints['payload_filter'] == 'loaded'


def test_unresolved_reference_does_not_auto_resolve_without_context():
    pipeline, _backend = _pipeline(_envelope(goal_type='transport'))
    parsed = pipeline.parse('move it to slot 2')
    assert not parsed.ok
    assert parsed.issues[0].code == 'missing_reference_context'


def test_grounded_reference_uses_confirmed_dialogue_context():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(goal_type='transport'))
    manager = builder.TaskGoalDialogueManager(parser=pipeline)

    first = manager.handle('move R2 to the right slot 3', timestamp=1.0)
    assert first.status == 'confirmation_required'
    final = manager.handle('yes', state=first.state, timestamp=2.0)
    assert final.ok

    follow_up = manager.handle('move the same shuttle to slot 4', state=final.state, timestamp=3.0)
    assert follow_up.status == 'confirmation_required'
    assert follow_up.draft.target_shuttle in {'R2', 'room315_right_shuttle_2'}
    assert follow_up.draft.target_slot == '4'


def test_room315_intent_model_path_env_overrides_config(tmp_path, monkeypatch):
    builder = _load_builder()
    from room_315_task_goal_semantic import INTENT_MODEL_PATH_ENV
    from room_315_task_goal_semantic import LocalSemanticModelConfig

    configured = tmp_path / 'configured.gguf'
    override = tmp_path / 'override.gguf'
    configured.write_bytes(b'configured')
    override.write_bytes(b'override')
    config_path = tmp_path / 'task_goal_understanding.yaml'
    config_path.write_text(
        '\n'.join([
            'schema_version: 1',
            'local_semantic_model:',
            '  enabled: true',
            '  backend: llama_cpp',
            f'  model_path: "{configured}"',
            'generation: {}',
            'runtime: {}',
        ]),
        encoding='utf-8',
    )

    monkeypatch.setenv(INTENT_MODEL_PATH_ENV, str(override))
    config = LocalSemanticModelConfig.from_file(config_path)
    assert config.model_path == str(override)
    assert builder is not None


def test_llama_cpp_backend_and_sha256_fingerprint(tmp_path):
    _load_builder()
    from room_315_task_goal_semantic import LlamaCppSemanticBackend
    from room_315_task_goal_semantic import LocalSemanticModelConfig
    from room_315_task_goal_semantic import build_backend_from_config
    from room_315_task_goal_semantic import fingerprint_model_path

    model = tmp_path / 'tiny.gguf'
    payload = b'room315-test-model'
    model.write_bytes(payload)
    expected = hashlib.sha256(payload).hexdigest()

    assert fingerprint_model_path(model) == f'sha256:{expected}'
    backend = build_backend_from_config(LocalSemanticModelConfig(
        backend='llama_cpp',
        model_path=str(model),
        model_sha256=expected,
    ))
    assert isinstance(backend, LlamaCppSemanticBackend)


def test_generic_loaded_shuttle_does_not_invent_side_nearest_or_identity():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='right',
        target_kind='slot',
        target_slot='1',
        target_shuttle='room315_right_shuttle_3',
    ))

    parsed = pipeline.parse('send loaded shuttle to slot 1')
    assert parsed.ok, parsed.to_dict()
    assert parsed.draft.side is None
    assert parsed.draft.selection_strategy == 'any'
    assert parsed.draft.payload_filter == 'loaded'
    assert parsed.draft.target_kind == 'slot'
    assert parsed.draft.target_slot == '1'
    assert parsed.draft.target_shuttle is None

    validation = builder.Room315DomainValidator().validate(parsed.draft, require_confirmation=True)
    assert validation.status == 'clarification_required'
    assert [issue.code for issue in validation.clarifications] == ['missing_side']


def test_explicit_inspection_shuttle_overrides_semantic_slot_guess():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(
        goal_type='inspection',
        target_kind='slot',
        target_slot='2',
    ))

    parsed = pipeline.parse('inspect L2')
    assert parsed.ok, parsed.to_dict()
    assert parsed.draft.goal_type == 'inspection'
    assert parsed.draft.target_kind == 'shuttle'
    assert parsed.draft.target_shuttle == 'L2'
    assert parsed.draft.target_slot is None

    validation = builder.Room315DomainValidator().validate(parsed.draft)
    assert validation.ok, validation.to_dict()
    assert validation.task_goal.constraints['target_kind'] == 'shuttle'
    assert validation.task_goal.constraints['target_shuttle'] == 'room315_left_shuttle_2'


def test_generic_inspection_selection_drops_ungrounded_semantic_shuttle_kind():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(
        goal_type='inspection',
        target_kind='slot',
        target_slot='3',
    ))

    parsed = pipeline.parse('inspect the loaded right shuttle')
    assert parsed.ok, parsed.to_dict()
    assert parsed.draft.goal_type == 'inspection'
    assert parsed.draft.target_kind == 'shuttle_selection'
    assert parsed.draft.target_shuttle is None
    assert parsed.draft.side == 'right'
    assert parsed.draft.payload_filter == 'loaded'

    validation = builder.Room315DomainValidator().validate(parsed.draft)
    assert validation.ok, validation.to_dict()
    assert validation.task_goal.constraints['target_kind'] == 'shuttle_selection'
    assert validation.task_goal.constraints['inspection_subject'] == 'right:shuttle_selection:any:loaded'


def test_dialogue_generic_loaded_slot_requires_side_then_confirmation():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='right',
        target_kind='slot',
        target_slot='1',
        target_shuttle='room315_right_shuttle_3',
    ))
    manager = builder.TaskGoalDialogueManager(parser=pipeline)

    first = manager.handle('send loaded shuttle to slot 1', timestamp=1.0)
    assert first.status == 'clarification_required'
    assert first.task_goal is None
    assert first.draft.side is None
    assert first.draft.selection_strategy == 'any'
    assert first.draft.payload_filter == 'loaded'
    assert any(issue.code == 'missing_side' for issue in first.clarifications)

    second = manager.handle('right', state=first.state, timestamp=2.0)
    assert second.status == 'confirmation_required'
    assert second.task_goal is None
    assert 'Action: Transport' in second.confirmation_prompt
    assert 'Rail: Right' in second.confirmation_prompt

    final = manager.handle('yes', state=second.state, timestamp=3.0)
    assert final.ok, final.to_dict()
    assert final.task_goal.constraints['selection_strategy'] == 'any'
    assert final.task_goal.constraints['payload_filter'] == 'loaded'
    assert final.task_goal.constraints['side'] == 'right'


def test_explicit_nearest_right_transport_remains_confirmation_only_until_yes():
    builder = _load_builder()
    pipeline, _backend = _pipeline(_envelope(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='right',
        target_kind='slot',
        target_slot='1',
    ))
    manager = builder.TaskGoalDialogueManager(parser=pipeline)

    first = manager.handle('send the nearest loaded right shuttle to slot 1', timestamp=1.0)
    assert first.status == 'confirmation_required'
    assert first.task_goal is None
    assert first.draft.selection_strategy == 'nearest'
    assert first.draft.side == 'right'
    assert first.draft.target_slot == '1'

    final = manager.handle('yes', state=first.state, timestamp=2.0)
    assert final.ok, final.to_dict()
    assert final.task_goal.constraints['selection_strategy'] == 'nearest'


def test_interactive_cli_multi_turn_uses_dialogue_manager_not_smoke_script():
    _load_builder()
    from room_315_task_goal_cli import run_dialogue_session

    pipeline, _backend = _pipeline(_envelope(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='right',
        target_kind='slot',
        target_slot='1',
        target_shuttle='room315_right_shuttle_3',
    ))
    from room_315_task_goal_dialogue import TaskGoalDialogueManager

    manager = TaskGoalDialogueManager(parser=pipeline)
    input_stream = io.StringIO('send loaded shuttle to slot 1\nright\nyes\nquit\n')
    output_stream = io.StringIO()

    assert run_dialogue_session(manager, input_stream, output_stream, timestamp=4.0) == 0
    output = output_stream.getvalue()
    assert 'Which Room 315 rail side should be used' in output
    assert 'Confirm Room 315 goal:' in output
    assert 'Final validated TaskGoal:' in output
    assert '"selection_strategy": "any"' in output
    assert '"payload_filter": "loaded"' in output
    assert 'room_315_task_goal_semantic_smoke' not in output


def test_interactive_cli_auto_confirm_skips_confirmation_prompt_only():
    _load_builder()
    from room_315_task_goal_cli import run_dialogue_session
    from room_315_task_goal_dialogue import TaskGoalDialogueManager

    pipeline, _backend = _pipeline('{}')
    manager = TaskGoalDialogueManager(parser=pipeline)
    input_stream = io.StringIO('send nearest loaded shuttle to kuka station\nquit\n')
    output_stream = io.StringIO()

    assert run_dialogue_session(manager, input_stream, output_stream, timestamp=4.0, auto_confirm=True) == 0
    output = output_stream.getvalue()
    assert 'Confirm Room 315 goal:' not in output
    assert 'Reply yes to finalize' not in output
    assert 'Final validated TaskGoal:' in output
    assert '"target_station": "kuka"' in output
    assert '"target_slot": "3"' in output


def test_station_goal_uses_first_station_slot_without_reusing_previous_goal():
    builder = _load_builder()
    pipeline, _backend = _pipeline('{}')
    manager = builder.TaskGoalDialogueManager(parser=pipeline)

    first = manager.handle('send loaded shuttle to slot 2', timestamp=1.0)
    assert first.status == 'clarification_required'
    second = manager.handle('left', state=first.state, timestamp=2.0)
    assert second.status == 'confirmation_required'
    previous = manager.handle('yes', state=second.state, timestamp=3.0)
    assert previous.ok, previous.to_dict()
    assert previous.task_goal.constraints['target_slot'] == '2'

    next_goal = manager.handle('send nearest loaded shuttle to kuka station', state=previous.state, timestamp=4.0)
    assert next_goal.status == 'confirmation_required'
    assert next_goal.task_goal is None
    assert 'Rail: Left' in next_goal.confirmation_prompt
    assert 'Destination: Station kuka / Slot 3' in next_goal.confirmation_prompt

    final = manager.handle('yes', state=next_goal.state, timestamp=5.0)
    assert final.ok, final.to_dict()
    constraints = final.task_goal.constraints
    assert constraints['selection_strategy'] == 'nearest'
    assert constraints['payload_filter'] == 'loaded'
    assert constraints['side'] == 'left'
    assert constraints['target_kind'] == 'station'
    assert constraints['target_station'] == 'kuka'
    assert constraints['target_slot'] == '3'


def test_yaskawa_station_requires_side_then_uses_first_slot_on_that_side():
    builder = _load_builder()
    pipeline, _backend = _pipeline('{}')
    manager = builder.TaskGoalDialogueManager(parser=pipeline)

    first = manager.handle('send loaded shuttle to yaskawa station', timestamp=1.0)
    assert first.status == 'clarification_required'
    assert first.task_goal is None
    assert any(issue.code == 'ambiguous_station_side' for issue in first.clarifications)
    assert first.questions.count('Which Room 315 rail side should be used: right or left?') == 1

    second = manager.handle('right', state=first.state, timestamp=2.0)
    assert second.status == 'confirmation_required'
    assert 'Rail: Right' in second.confirmation_prompt
    assert 'Destination: Station yaskawa / Slot 1' in second.confirmation_prompt

    final = manager.handle('yes', state=second.state, timestamp=3.0)
    assert final.ok, final.to_dict()
    constraints = final.task_goal.constraints
    assert constraints['side'] == 'right'
    assert constraints['target_station'] == 'yaskawa'
    assert constraints['target_slot'] == '1'


def test_station_slot_mismatch_fails_closed():
    builder = _load_builder()
    draft = builder.TaskGoalDraft(
        goal_type='transport',
        selection_strategy='nearest',
        payload_filter='loaded',
        side='left',
        target_kind='station',
        target_station='kuka',
        target_slot='2',
    )

    result = builder.Room315DomainValidator().validate(draft, require_confirmation=True)
    assert result.status == 'error'
    assert result.errors[0].code == 'station_slot_mismatch'
