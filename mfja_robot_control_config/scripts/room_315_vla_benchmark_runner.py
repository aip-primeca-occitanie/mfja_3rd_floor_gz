#!/usr/bin/env python3

import json
import os
import time
from datetime import datetime
from datetime import timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


TASK_GOALS = {
    'right_yaskawa_to_staubli': 'move the right shuttle from Yaskawa to Staubli',
    'right_staubli_to_yaskawa': 'move the right shuttle from Staubli to Yaskawa',
    'left_yaskawa_to_kuka': 'move the left shuttle from Yaskawa to KUKA',
    'left_kuka_to_yaskawa': 'move the left shuttle from KUKA to Yaskawa',
    'right_enter_interior_loop': 'make the right shuttle circulate on the interior loop',
    'left_enter_interior_loop': 'make the left shuttle circulate on the interior loop',
}

TASK_GROUPS = {
    'transport': [
        'right_yaskawa_to_staubli',
        'right_staubli_to_yaskawa',
        'left_yaskawa_to_kuka',
        'left_kuka_to_yaskawa',
    ],
    'loop_entry': [
        'right_enter_interior_loop',
        'left_enter_interior_loop',
    ],
}
TASK_GROUPS['all'] = [*TASK_GROUPS['transport'], *TASK_GROUPS['loop_entry']]


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def parse_task_sequence(raw_tasks: str) -> list[str]:
    text = str(raw_tasks or '').strip()
    if not text:
        text = 'all'
    task_names: list[str] = []
    for item in text.replace(';', ',').split(','):
        name = item.strip()
        if not name:
            continue
        if name in TASK_GROUPS:
            for grouped_name in TASK_GROUPS[name]:
                if grouped_name not in task_names:
                    task_names.append(grouped_name)
            continue
        if name not in TASK_GOALS:
            allowed = ', '.join(sorted([*TASK_GOALS, *TASK_GROUPS]))
            raise ValueError(f'unknown VLA benchmark task {name!r}; allowed: {allowed}')
        if name not in task_names:
            task_names.append(name)
    return task_names


def task_state_from_status(status: dict[str, Any], task_name: str) -> tuple[str, str]:
    result = str(status.get('last_result') or '')
    active_tasks = status.get('active_tasks', {})
    if isinstance(active_tasks, dict):
        for task in active_tasks.values():
            if isinstance(task, dict) and task.get('template') == task_name:
                return 'running', _json_dumps(task)

    completed_tasks = status.get('completed_tasks', [])
    if isinstance(completed_tasks, list):
        for task in reversed(completed_tasks):
            if not isinstance(task, dict) or task.get('template') != task_name:
                continue
            task_status = str(task.get('status') or '')
            if task_status == 'succeeded':
                return 'succeeded', _json_dumps(task)
            if task_status == 'failed':
                return 'failed', _json_dumps(task)

    lowered = result.casefold()
    task_marker = f'task {task_name}'.casefold()
    if task_marker not in lowered:
        return 'unknown', result
    if ' completed:' in lowered or ' is circulating ' in lowered:
        return 'succeeded', result
    if ' failed:' in lowered or ' rejected:' in lowered or 'command rejected:' in lowered:
        return 'failed', result
    if ' started:' in lowered or 'waiting for' in lowered:
        return 'running', result
    return 'unknown', result


def safety_metrics_from_status(status: dict[str, Any]) -> dict[str, Any]:
    safety = status.get('safety_decoder', {})
    if isinstance(safety, dict) and isinstance(safety.get('metrics'), dict):
        return dict(safety['metrics'])
    metrics = status.get('safety_decoder_metrics', {})
    return dict(metrics) if isinstance(metrics, dict) else {}


class Room315VlaBenchmarkRunner(Node):
    def __init__(self) -> None:
        super().__init__('room_315_vla_benchmark_runner')

        self.declare_parameter('tasks', 'all')
        self.declare_parameter('goal_topic', '/room_315/vla/user_goal')
        self.declare_parameter('command_topic', '/room_315/vla/command')
        self.declare_parameter('status_topic', '/room_315/vla/status')
        self.declare_parameter('episode_control_topic', '/room_315/vla/episode_control')
        self.declare_parameter('benchmark_status_topic', '/room_315/vla/benchmark_status')
        self.declare_parameter('report_dir', '~/.ros/room315_vla_benchmarks')
        self.declare_parameter('task_timeout_s', 120.0)
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('start_delay_s', 3.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('mark_dataset_episodes', True)

        self.tasks = parse_task_sequence(str(self.get_parameter('tasks').value))
        self.report_dir = Path(
            os.path.expandvars(str(self.get_parameter('report_dir').value))
        ).expanduser()
        self.report_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.report_jsonl = self.report_dir / f'room315_vla_benchmark_{stamp}.jsonl'
        self.report_summary = self.report_dir / f'room315_vla_benchmark_{stamp}_summary.json'
        self.report_stream = self.report_jsonl.open('w', encoding='utf-8')

        self.latest_status: dict[str, Any] = {}
        self.results: list[dict[str, Any]] = []
        self.current_task: dict[str, Any] | None = None
        self.next_task_index = 0
        self.next_dispatch_at = (
            time.monotonic() + max(float(self.get_parameter('start_delay_s').value), 0.0)
        )
        self.completed = False
        self.last_error = ''

        self.goal_pub = self.create_publisher(
            String,
            str(self.get_parameter('goal_topic').value),
            10,
        )
        self.command_pub = self.create_publisher(
            String,
            str(self.get_parameter('command_topic').value),
            10,
        )
        self.episode_control_pub = self.create_publisher(
            String,
            str(self.get_parameter('episode_control_topic').value),
            10,
        )
        self.benchmark_status_pub = self.create_publisher(
            String,
            str(self.get_parameter('benchmark_status_topic').value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_status,
            10,
        )
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status)

        self._publish_status()
        self.get_logger().info(
            f'Room 315 VLA benchmark runner ready: tasks={self.tasks}, '
            f'report={self.report_jsonl}'
        )

    def _on_status(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(parsed, dict):
            self.latest_status = parsed

    def _tick(self) -> None:
        if self.completed:
            return
        if not _as_bool(self.get_parameter('auto_start').value):
            return
        now = time.monotonic()
        if self.current_task is None:
            if now >= self.next_dispatch_at:
                self._start_next_task()
            return
        self._update_current_task(now)

    def _start_next_task(self) -> None:
        if self.next_task_index >= len(self.tasks):
            self._finish_benchmark()
            return
        task_name = self.tasks[self.next_task_index]
        goal = TASK_GOALS[task_name]
        self.current_task = {
            'task_index': self.next_task_index,
            'task': task_name,
            'goal': goal,
            'started_monotonic': time.monotonic(),
            'started_at': _utc_now(),
        }
        self.next_task_index += 1

        msg = String()
        msg.data = _json_dumps({'action': 'route_template', 'template': task_name})
        if _as_bool(self.get_parameter('mark_dataset_episodes').value):
            control = String()
            control.data = f'start {goal}'
            self.episode_control_pub.publish(control)
        self.command_pub.publish(msg)
        self.get_logger().info(f'Benchmark task started: {task_name} -> {msg.data}')
        self._publish_status()

    def _update_current_task(self, now: float) -> None:
        if self.current_task is None:
            return
        task_name = str(self.current_task['task'])
        state, detail = task_state_from_status(self.latest_status, task_name)
        timeout_s = float(self.get_parameter('task_timeout_s').value)
        elapsed_s = now - float(self.current_task['started_monotonic'])

        if state == 'succeeded':
            self._complete_current_task(True, detail, elapsed_s)
            return
        if state == 'failed':
            self._complete_current_task(False, detail, elapsed_s)
            return
        if timeout_s > 0.0 and elapsed_s > timeout_s:
            self._complete_current_task(
                False,
                f'benchmark timeout after {timeout_s:.1f}s waiting for {task_name}',
                elapsed_s,
            )

    def _complete_current_task(self, success: bool, detail: str, elapsed_s: float) -> None:
        if self.current_task is None:
            return
        row = {
            'task_index': int(self.current_task['task_index']),
            'task': self.current_task['task'],
            'goal': self.current_task['goal'],
            'started_at': self.current_task['started_at'],
            'ended_at': _utc_now(),
            'duration_s': round(elapsed_s, 3),
            'success': bool(success),
            'detail': detail,
            'safety_decoder_metrics': safety_metrics_from_status(self.latest_status),
        }
        self.results.append(row)
        self.report_stream.write(_json_dumps(row) + '\n')
        self.report_stream.flush()

        if _as_bool(self.get_parameter('mark_dataset_episodes').value):
            control = String()
            control.data = 'stop success' if success else 'stop failure'
            self.episode_control_pub.publish(control)

        self.get_logger().info(
            f'Benchmark task finished: {row["task"]} success={success} '
            f'duration={row["duration_s"]}s'
        )
        self.current_task = None
        self.next_dispatch_at = time.monotonic() + max(
            float(self.get_parameter('settle_time_s').value),
            0.0,
        )
        self._publish_status()

    def _finish_benchmark(self) -> None:
        self.completed = True
        total = len(self.results)
        successes = sum(1 for row in self.results if row.get('success'))
        summary = {
            'completed_at': _utc_now(),
            'tasks': self.tasks,
            'total': total,
            'successes': successes,
            'failures': total - successes,
            'success_rate': None if total == 0 else round(successes / total, 4),
            'safety_decoder_metrics': safety_metrics_from_status(self.latest_status),
            'report_jsonl': str(self.report_jsonl),
            'results': self.results,
        }
        with self.report_summary.open('w', encoding='utf-8') as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
        self.get_logger().info(
            f'Room 315 VLA benchmark complete: {successes}/{total} succeeded; '
            f'summary={self.report_summary}'
        )
        self._publish_status()

    def _publish_status(self) -> None:
        current = None
        if self.current_task is not None:
            current = {
                key: value
                for key, value in self.current_task.items()
                if key != 'started_monotonic'
            }
            current['age_s'] = round(
                time.monotonic() - float(self.current_task['started_monotonic']),
                3,
            )
        total = len(self.results)
        successes = sum(1 for row in self.results if row.get('success'))
        msg = String()
        msg.data = _json_dumps({
            'state': 'completed' if self.completed else 'running',
            'tasks': self.tasks,
            'next_task_index': self.next_task_index,
            'current_task': current,
            'results_total': total,
            'successes': successes,
            'failures': total - successes,
            'safety_decoder_metrics': safety_metrics_from_status(self.latest_status),
            'report_jsonl': str(self.report_jsonl),
            'report_summary': str(self.report_summary),
            'last_error': self.last_error,
        })
        self.benchmark_status_pub.publish(msg)

    def destroy_node(self) -> bool:
        if not self.report_stream.closed:
            self.report_stream.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Room315VlaBenchmarkRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
