#!/usr/bin/env python3
"""Interactive Room 315 task-goal command interface."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any
from typing import Callable
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
    parser.add_argument(
        '--publish-topic',
        default='',
        help=(
            'Publish each confirmed TaskGoal as JSON on this std_msgs/String '
            'topic. Empty keeps parse/validation-only behavior.'
        ),
    )
    parser.add_argument(
        '--result-topic',
        default='/room_315/task_goal/status',
        help='Task-execution status topic used with --wait-for-result.',
    )
    parser.add_argument(
        '--wait-for-result',
        action='store_true',
        help='Wait for a terminal execution status after publishing a TaskGoal.',
    )
    parser.add_argument(
        '--publish-timeout-s',
        type=float,
        default=5.0,
        help='Maximum wait for the task-execution subscriber.',
    )
    parser.add_argument(
        '--result-timeout-s',
        type=float,
        default=120.0,
        help='Maximum wait for a terminal task-execution result.',
    )
    args = parser.parse_args()

    ros_client = None
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
        if args.wait_for_result and not args.publish_topic:
            parser.error('--wait-for-result requires --publish-topic')
        if args.publish_topic:
            ros_client = RosTaskGoalClient(
                publish_topic=args.publish_topic,
                result_topic=args.result_topic,
                publish_timeout_s=args.publish_timeout_s,
                wait_for_result=args.wait_for_result,
                result_timeout_s=args.result_timeout_s,
            )
        print('Room 315 task-goal CLI ready. Type quit to exit.')
        print(json.dumps({'model_health': health.to_dict()}, sort_keys=True))
        return run_dialogue_session(
            manager,
            sys.stdin,
            sys.stdout,
            timestamp=args.timestamp,
            auto_confirm=args.auto_confirm,
            on_task_goal=(
                ros_client.publish_and_wait
                if ros_client is not None
                else None
            ),
        )
    except KeyboardInterrupt:
        print('\nCancelled.')
        return 130
    except Exception as exc:  # pragma: no cover - defensive CLI boundary
        print(json.dumps({'status': 'error', 'reason': str(exc)}, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    finally:
        if ros_client is not None:
            ros_client.close()


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
    on_task_goal: Callable[[Any], dict[str, Any] | None] | None = None,
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
        _print_turn_result(
            result,
            output_stream,
            on_task_goal=on_task_goal,
        )


def _is_interactive(input_stream: TextIO, output_stream: TextIO) -> bool:
    return bool(
        getattr(input_stream, 'isatty', lambda: False)()
        and getattr(output_stream, 'isatty', lambda: False)()
    )


def _print_turn_result(
    result: DialogueTurnResult,
    output_stream: TextIO,
    *,
    on_task_goal: Callable[[Any], dict[str, Any] | None] | None = None,
) -> None:
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
        if on_task_goal is not None:
            publication = on_task_goal(result.task_goal)
            if publication is not None:
                print('Task execution response:', file=output_stream)
                print(
                    json.dumps(publication, indent=2, sort_keys=True),
                    file=output_stream,
                )
        return
    if result.status == 'cancelled':
        for question in result.questions:
            print(question, file=output_stream)
        return
    payload = result.to_dict()
    print(json.dumps(payload, indent=2, sort_keys=True), file=output_stream)


class RosTaskGoalClient:
    """Lazy ROS adapter used only when the operator requests publication."""

    TERMINAL_STATUSES = frozenset({
        'succeeded',
        'aborted',
        'failed',
        'rejected',
    })

    def __init__(
        self,
        *,
        publish_topic: str,
        result_topic: str,
        publish_timeout_s: float,
        wait_for_result: bool,
        result_timeout_s: float,
    ) -> None:
        import rclpy
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.String = String
        self.publish_timeout_s = max(float(publish_timeout_s), 0.1)
        self.wait_for_result = bool(wait_for_result)
        self.result_timeout_s = max(float(result_timeout_s), 0.1)
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.node = rclpy.create_node('room_315_task_goal_operator_cli')
        self.publisher = self.node.create_publisher(
            String,
            str(publish_topic),
            10,
        )
        self._statuses: list[dict[str, Any]] = []
        self.node.create_subscription(
            String,
            str(result_topic),
            self._on_status,
            10,
        )

    def _on_status(self, message: Any) -> None:
        try:
            payload = json.loads(message.data or '{}')
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self._statuses.append(payload)
            self._statuses = self._statuses[-100:]

    def publish_and_wait(self, task_goal: Any) -> dict[str, Any]:
        deadline = time.monotonic() + self.publish_timeout_s
        while (
            time.monotonic() <= deadline
            and self.publisher.get_subscription_count() <= 0
        ):
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.publisher.get_subscription_count() <= 0:
            return {
                'status': 'error',
                'reason': 'no_task_execution_subscriber',
                'goal_id': task_goal.goal_id,
            }

        message = self.String()
        message.data = json.dumps(task_goal.to_dict(), sort_keys=True)
        self.publisher.publish(message)
        # Let DDS enqueue the message before returning to the input prompt.
        for _ in range(3):
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if not self.wait_for_result:
            return {
                'status': 'published',
                'goal_id': task_goal.goal_id,
            }

        deadline = time.monotonic() + self.result_timeout_s
        latest: dict[str, Any] = {}
        while time.monotonic() <= deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            matching = [
                item
                for item in self._statuses
                if str(item.get('goal_id') or '') == task_goal.goal_id
            ]
            if not matching:
                continue
            latest = matching[-1]
            if str(latest.get('status') or '') in self.TERMINAL_STATUSES:
                return latest
        return {
            'status': 'timed_out',
            'reason': 'task_execution_result_timeout',
            'goal_id': task_goal.goal_id,
            'latest_status': latest,
        }

    def close(self) -> None:
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and self.rclpy.ok():
                self.rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
