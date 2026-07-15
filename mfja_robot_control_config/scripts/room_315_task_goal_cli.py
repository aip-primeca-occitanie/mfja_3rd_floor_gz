#!/usr/bin/env python3
"""Interactive Room 315 task-goal command interface."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import TextIO

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_task_goal_dialogue import DialogueTurnResult
from room_315_task_goal_dialogue import TaskGoalDialogueManager
from room_315_task_goal_dialogue import TaskGoalDialogueState
from room_315_task_goal_parsers import ParserPipeline
from room_315_task_goal_semantic import LocalSemanticBackend
from room_315_task_goal_semantic import LocalSemanticModelConfig
from room_315_task_goal_semantic import build_backend_from_config
from room_315_task_goal_validation import Room315DomainValidator


EXIT_WORDS = {'quit', 'exit'}


def main() -> int:
    parser = argparse.ArgumentParser(description='Interactive Room 315 natural-language TaskGoal CLI.')
    parser.add_argument('--config', default='', help='Versioned task-goal understanding YAML.')
    parser.add_argument('--allow-unready-model', action='store_true', help='Start even if the local model is unavailable.')
    parser.add_argument(
        '--auto-confirm',
        action='store_true',
        help='Debug/operator shortcut: finalize confirmation-ready goals without asking for yes.',
    )
    parser.add_argument('--timestamp', type=float, default=0.0, help='Timestamp used when finalizing TaskGoal objects.')
    args = parser.parse_args()

    try:
        config = LocalSemanticModelConfig.from_file(args.config or None)
        backend = build_backend_from_config(config)
        health = backend.health()
        if not health.ready and not args.allow_unready_model:
            print(json.dumps({
                'status': 'error',
                'reason': 'local_semantic_model_not_ready',
                'health': health.to_dict(),
                'next_command': 'python3 mfja_robot_control_config/scripts/setup_room315_intent_model.py',
            }, indent=2, sort_keys=True), file=sys.stderr)
            return 2
        manager = make_dialogue_manager(backend=backend, config=config)
        print('Room 315 task-goal CLI ready. Type quit to exit.')
        print(json.dumps({'model_health': health.to_dict()}, sort_keys=True))
        return run_dialogue_session(
            manager,
            sys.stdin,
            sys.stdout,
            timestamp=args.timestamp,
            auto_confirm=args.auto_confirm,
        )
    except KeyboardInterrupt:
        print('\nCancelled.')
        return 130
    except Exception as exc:  # pragma: no cover - defensive CLI boundary
        print(json.dumps({'status': 'error', 'reason': str(exc)}, indent=2, sort_keys=True), file=sys.stderr)
        return 1


def make_dialogue_manager(
    *,
    backend: LocalSemanticBackend,
    config: LocalSemanticModelConfig,
) -> TaskGoalDialogueManager:
    return TaskGoalDialogueManager(
        parser=ParserPipeline(semantic_backend=backend, semantic_config=config),
        validator=Room315DomainValidator(),
    )


def run_dialogue_session(
    manager: TaskGoalDialogueManager,
    input_stream: TextIO,
    output_stream: TextIO,
    *,
    timestamp: float = 0.0,
    auto_confirm: bool = False,
) -> int:
    state: TaskGoalDialogueState | None = None
    interactive = _is_interactive(input_stream, output_stream)
    while True:
        if interactive:
            print('room315> ', end='', file=output_stream, flush=True)
        raw_line = input_stream.readline()
        if raw_line == '':
            return 0
        utterance = raw_line.strip()
        if not utterance:
            continue
        if utterance.casefold() in EXIT_WORDS:
            print('Goodbye.', file=output_stream)
            return 0
        result = manager.handle(utterance, state=state, timestamp=timestamp)
        state = result.state
        if auto_confirm and result.status == 'confirmation_required':
            result = manager.handle('yes', state=state, timestamp=timestamp)
            state = result.state
        _print_turn_result(result, output_stream)


def _is_interactive(input_stream: TextIO, output_stream: TextIO) -> bool:
    return bool(
        getattr(input_stream, 'isatty', lambda: False)()
        and getattr(output_stream, 'isatty', lambda: False)()
    )


def _print_turn_result(result: DialogueTurnResult, output_stream: TextIO) -> None:
    if result.status == 'confirmation_required':
        print(result.confirmation_prompt, file=output_stream)
        return
    if result.status == 'clarification_required':
        for question in result.questions:
            print(question, file=output_stream)
        if not result.questions and result.clarifications:
            for issue in result.clarifications:
                print(issue.message, file=output_stream)
        return
    if result.ok:
        print('Final validated TaskGoal:', file=output_stream)
        print(json.dumps(result.task_goal.to_dict(), indent=2, sort_keys=True), file=output_stream)
        return
    if result.status == 'cancelled':
        for question in result.questions:
            print(question, file=output_stream)
        return
    payload = result.to_dict()
    print(json.dumps(payload, indent=2, sort_keys=True), file=output_stream)


if __name__ == '__main__':
    raise SystemExit(main())
