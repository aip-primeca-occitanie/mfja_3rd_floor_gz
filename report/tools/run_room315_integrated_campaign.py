#!/usr/bin/env python3
"""Run and preserve a manifest-bound Room 315 integrated simulation campaign.

Each declared case receives a fresh Gazebo process, visual runtime, PlanSys2
planner and task-execution gateway.  The operator-facing language CLI is driven
in two stages: the request is submitted first, the rendered confirmation is
checked against the declaration, and only then is ``yes`` sent.  The script
refuses to reuse an output directory and stops the campaign at the first
unexpected result.
"""

from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
import platform
import re
import selectors
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


REPO = Path(__file__).resolve().parents[2]
WORKSPACE = REPO.parents[1]
INSTALL_SETUP = WORKSPACE / 'install' / 'setup.bash'
SCRIPTS = REPO / 'mfja_robot_control_config' / 'scripts'
DEFAULT_MATRIX = Path(__file__).with_name('room315_integrated_campaign_v2.yaml')
VISUAL_RUN = Path(
    '/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run'
)
VISUAL_VENV = Path(
    '/home/tiago/room315_local_training/venv/lib/python3.12/site-packages'
)
INTENT_MODEL = Path(
    '/home/tiago/models/room315_intent/qwen2.5-1.5b-instruct-q4_k_m.gguf'
)
INTENT_CONFIG = Path(
    '/home/tiago/models/room315_intent/task_goal_understanding.local.yaml'
)
RUNTIME_LOCK = Path('/tmp/mfja_room315_floor_runtime.lock')
TERMINAL_STATUSES = {'succeeded', 'aborted', 'failed', 'rejected'}
PLANNER_SERVICE = '/planner/get_plan'
PLANNER_SERVICE_TYPE = 'plansys2_msgs/srv/GetPlan'
VISUAL_SCHEMA = 'room315.visual_state.v3'
TERMINAL_SYMBOLIC_ACTIONS = {
    'finish_task', 'finish_candidate_task', 'inspect_state',
}
ACTIVE_LOG_ERROR_PATTERNS = (
    re.compile(r'\[ERROR\]'),
    re.compile(r'\[FATAL\]'),
    re.compile(r'Traceback \(most recent call last\)'),
    re.compile(r'process has died'),
    re.compile(r'segmentation fault', re.IGNORECASE),
    re.compile(r'core dumped', re.IGNORECASE),
)

RECORDED_TOPICS = (
    '/clock',
    '/diagnostics',
    '/room_315/task_goal',
    '/room_315/task_goal/status',
    '/room_315/visual_state/raw_model_prediction',
    '/room_315/visual_state/raw',
    '/room_315/visual_state/validation',
    '/room_315/visual_state/observed_state',
    '/room_315/vla/command',
    '/room_315/vla/status',
    '/room_315/vla/emergency_stop',
    '/room_315/vla/right_rail_rgbd/image',
    '/room_315/vla/right_rail_rgbd/camera_info',
    '/room_315/vla/left_rail_rgbd/image',
    '/room_315/vla/left_rail_rgbd/camera_info',
    '/room_315/rails/right/shuttles/state',
    '/room_315/rails/right/shuttles/command',
    '/room_315/rails/right/shuttles/payload_state',
    '/room_315/rails/right/shuttles/payload_command',
    '/room_315/rails/right/switches/state',
    '/room_315/rails/right/switches/command',
    '/room_315/rails/right/stoppers/state',
    '/room_315/rails/right/stoppers/command',
    '/room_315/rails/right/sensors/feedback',
    '/room_315/rails/left/shuttles/state',
    '/room_315/rails/left/shuttles/command',
    '/room_315/rails/left/shuttles/payload_state',
    '/room_315/rails/left/shuttles/payload_command',
    '/room_315/rails/left/switches/state',
    '/room_315/rails/left/switches/command',
    '/room_315/rails/left/stoppers/state',
    '/room_315/rails/left/stoppers/command',
    '/room_315/rails/left/sensors/feedback',
)


class CampaignError(RuntimeError):
    """A declared campaign condition was not met."""


@dataclass
class ManagedProcess:
    name: str
    command: list[str]
    process: subprocess.Popen[str]
    log_path: Path
    log_stream: Any
    evidence_offset: int = 0


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace('+00:00', 'Z')


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(value, encoding='utf-8')
    temporary.replace(path)


def write_bytes(path: Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_bytes(value)
    temporary.replace(path)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def clean_environment(domain_id: int, ros_log_dir: Path | None = None) -> dict[str, str]:
    env = dict(os.environ)
    env.update({
        'ROS_DOMAIN_ID': str(domain_id),
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS2CLI_NO_DAEMON': '1',
        'HF_HUB_OFFLINE': '1',
        'TRANSFORMERS_OFFLINE': '1',
        'ROOM315_INTENT_MODEL_PATH': str(INTENT_MODEL),
        'ROOM315_TASK_GOAL_LOCAL_CONFIG': str(INTENT_CONFIG),
    })
    env.pop('ROS_LOCALHOST_ONLY', None)
    if ros_log_dir is not None:
        ros_log_dir.mkdir(parents=True, exist_ok=True)
        env['ROS_LOG_DIR'] = str(ros_log_dir)
    return env


def run_capture(
    command: list[str],
    *,
    env: dict[str, str],
    cwd: Path = WORKSPACE,
    timeout_s: float = 30.0,
    check: bool = False,
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=False,
        )
        payload = {
            'command': command,
            'returncode': completed.returncode,
            'stdout': completed.stdout,
            'stderr': completed.stderr,
            'duration_s': time.monotonic() - started,
        }
    except subprocess.TimeoutExpired as exc:
        payload = {
            'command': command,
            'returncode': None,
            'stdout': exc.stdout or '',
            'stderr': exc.stderr or '',
            'duration_s': time.monotonic() - started,
            'timed_out': True,
        }
    if check and payload.get('returncode') != 0:
        raise CampaignError(
            f'command failed: {command!r}: {payload.get("stderr") or payload.get("stdout")}'
        )
    return payload


def start_process(
    name: str,
    command: list[str],
    *,
    env: dict[str, str],
    log_path: Path,
    cwd: Path = WORKSPACE,
) -> ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open('w', encoding='utf-8', buffering=1)
    process = subprocess.Popen(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return ManagedProcess(name, command, process, log_path, stream)


def stop_process(process: ManagedProcess, *, timeout_s: float = 15.0) -> dict[str, Any]:
    result: dict[str, Any] = {
        'name': process.name,
        'pid': process.process.pid,
        'command': process.command,
        'planned_shutdown': True,
        'signals': [],
    }
    if process.process.poll() is None:
        for sent_signal, wait_s in (
            (signal.SIGINT, timeout_s),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 2.0),
        ):
            try:
                os.killpg(process.process.pid, sent_signal)
                result['signals'].append(sent_signal.name)
            except ProcessLookupError:
                break
            try:
                process.process.wait(timeout=wait_s)
                break
            except subprocess.TimeoutExpired:
                continue
    result['returncode'] = process.process.poll()
    process.log_stream.flush()
    process.log_stream.close()
    return result


def require_alive(process: ManagedProcess) -> None:
    returncode = process.process.poll()
    if returncode is not None:
        text = process.log_path.read_text(encoding='utf-8', errors='replace')
        raise CampaignError(
            f'{process.name} exited before readiness with {returncode}:\n{text[-4000:]}'
        )


def mark_evidence_window_start(process: ManagedProcess) -> None:
    """Exclude startup-only diagnostics from the task execution window."""

    process.log_stream.flush()
    process.evidence_offset = process.log_path.stat().st_size


def topic_echo(
    topic: str,
    message_type: str,
    *,
    env: dict[str, str],
    timeout_s: float,
    filter_expression: str = '',
) -> dict[str, Any]:
    command = [
        'ros2', 'topic', 'echo', '--no-daemon', '--full-length',
        topic, message_type, '--once',
    ]
    if filter_expression:
        command.extend(['--filter', filter_expression])
    return run_capture(
        command,
        env=env,
        timeout_s=timeout_s,
    )


def parse_ros_yaml(text: str) -> dict[str, Any]:
    cleaned = text.strip()
    if cleaned.endswith('---'):
        cleaned = cleaned[:-3].rstrip()
    payload = yaml.safe_load(cleaned) if cleaned else None
    if not isinstance(payload, dict):
        raise CampaignError('ROS topic output is not one YAML mapping')
    return payload


def wait_for_planner(env: dict[str, str], timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    lifecycle_attempts: list[dict[str, Any]] = []
    service_attempts: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        lifecycle = run_capture(
            ['ros2', 'lifecycle', 'get', '/planner'],
            env=env,
            timeout_s=5.0,
        )
        lifecycle_attempts.append(lifecycle)
        lifecycle_match = re.fullmatch(
            r'active\s+\[(\d+)\]',
            str(lifecycle.get('stdout') or '').strip(),
            flags=re.IGNORECASE,
        )
        service = run_capture(
            ['ros2', 'service', 'type', PLANNER_SERVICE],
            env=env,
            timeout_s=5.0,
        )
        service_attempts.append(service)
        service_ready = (
            service.get('returncode') == 0
            and str(service.get('stdout') or '').strip() == PLANNER_SERVICE_TYPE
        )
        if lifecycle.get('returncode') == 0 and lifecycle_match and service_ready:
            return {
                'status': 'ready',
                'lifecycle_state': 'active',
                'lifecycle_state_id': int(lifecycle_match.group(1)),
                'planner_service': PLANNER_SERVICE,
                'planner_service_type': PLANNER_SERVICE_TYPE,
                'lifecycle_attempts': lifecycle_attempts,
                'service_attempts': service_attempts,
            }
        time.sleep(0.5)
    raise CampaignError(
        'PlanSys2 planner did not report the exact active lifecycle state '
        f'and expose {PLANNER_SERVICE} as {PLANNER_SERVICE_TYPE}'
    )


def wait_for_task_subscriber(env: dict[str, str], timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    attempts: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        result = run_capture(
            ['ros2', 'topic', 'info', '/room_315/task_goal'],
            env=env,
            timeout_s=5.0,
        )
        attempts.append(result)
        match = re.search(r'Subscription count:\s*(\d+)', result.get('stdout', ''))
        if result.get('returncode') == 0 and match and int(match.group(1)) >= 1:
            return {'status': 'ready', 'attempts': attempts}
        time.sleep(0.5)
    raise CampaignError('task execution subscriber did not become available')


def verify_runtime_lock_free() -> None:
    descriptor = os.open(RUNTIME_LOCK, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.lseek(descriptor, 0, os.SEEK_SET)
            owner = os.read(descriptor, 64).decode('ascii', errors='ignore').strip()
            raise CampaignError(
                f'Room 315 runtime lock is held before case start; owner={owner or "unknown"}'
            ) from exc
        fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        os.close(descriptor)


def verify_empty_ros_graph(env: dict[str, str]) -> dict[str, Any]:
    """Reject a domain that already contains a ROS application graph."""

    nodes = run_capture(
        ['ros2', 'node', 'list', '--no-daemon'], env=env, timeout_s=10.0,
    )
    topics = run_capture(
        ['ros2', 'topic', 'list', '--no-daemon'], env=env, timeout_s=10.0,
    )
    services = run_capture(
        ['ros2', 'service', 'list', '--no-daemon'], env=env, timeout_s=10.0,
    )
    for label, result in (
        ('node', nodes), ('topic', topics), ('service', services),
    ):
        if result.get('returncode') != 0:
            raise CampaignError(
                f'could not inspect pre-existing ROS {label} graph: '
                f'{result.get("stderr") or result.get("stdout")}'
            )
    node_names = [line.strip() for line in nodes['stdout'].splitlines() if line.strip()]
    topic_names = [line.strip() for line in topics['stdout'].splitlines() if line.strip()]
    service_names = [
        line.strip() for line in services['stdout'].splitlines() if line.strip()
    ]
    unexpected_topics = sorted(
        set(topic_names) - {'/parameter_events', '/rosout'}
    )
    if node_names or unexpected_topics or service_names:
        raise CampaignError(
            'ROS domain is not empty before the cold start: '
            f'nodes={node_names}, topics={unexpected_topics}, services={service_names}'
        )
    return {
        'status': 'empty',
        'nodes': node_names,
        'topics': topic_names,
        'services': service_names,
        'commands': {'nodes': nodes, 'topics': topics, 'services': services},
    }


def identity_parts(identity: str) -> tuple[str, str, int]:
    match = re.fullmatch(r'([RL])(\d+)', str(identity).strip().upper())
    if not match:
        raise CampaignError(f'invalid Room 315 shuttle identity: {identity!r}')
    side = 'right' if match.group(1) == 'R' else 'left'
    return match.group(1), side, int(match.group(2))


def controller_entity(identity: str) -> str:
    _, side, index = identity_parts(identity)
    return f'room315_{side}_shuttle_{index}'


def symbolic_entity(identity: str) -> str:
    _, side, index = identity_parts(identity)
    return f'{side}_shuttle_{index}'


def header_stamp(payload: dict[str, Any]) -> tuple[int, int]:
    stamp = (payload.get('header') or {}).get('stamp') or {}
    return int(stamp.get('sec') or 0), int(stamp.get('nanosec') or 0)


def side_launch_arguments(side: str, declaration: dict[str, Any]) -> list[str]:
    prefix = f'room315_{side}'
    identities = [str(value) for value in declaration.get('identities') or []]
    slots = [str(value) for value in declaration.get('start_slots') or []]
    loaded = [str(value) for value in declaration.get('loaded') or []]
    if len(identities) != len(slots):
        raise CampaignError(f'{side} identity/start-slot cardinality mismatch')
    values = [f'{prefix}_shuttle_count:={len(identities)}']
    if identities:
        values.extend([
            f'{prefix}_active_identities:={",".join(identities)}',
            f'{prefix}_start_slots:={",".join(slots)}',
        ])
    if loaded:
        values.append(f'{prefix}_loaded_shuttles:={",".join(loaded)}')
    return values


def floor_command(case: dict[str, Any], speed: float) -> list[str]:
    launch = case['launch']
    return [
        'ros2', 'launch', 'mfja_3rd_floor_bringup', 'room_315_only.launch.py',
        'robots:=none',
        'gui:=false',
        'start_paused:=false',
        'enable_room315_kinematic_shuttles:=true',
        'enable_room315_right_rail:=true',
        'enable_room315_left_rail:=true',
        'enable_room315_vla:=true',
        'enable_room315_vla_camera_bridge:=true',
        'enable_room315_vla_dataset_recorder:=false',
        'enable_room315_vla_obstacles:=false',
        'room315_show_device_markers:=false',
        'room315_visual_debug_colors:=false',
        'room315_enable_payload_visuals:=true',
        'room315_identity_selection_mode:=explicit',
        'room315_shuttles_start_enabled:=false',
        f'room315_shuttle_speed:={speed}',
        *side_launch_arguments('right', launch['right']),
        *side_launch_arguments('left', launch['left']),
    ]


def visual_command() -> list[str]:
    return [
        'ros2', 'launch', 'mfja_robot_control_config',
        'room_315_visual_state_runtime.launch.py',
        'use_sim_time:=true',
        'enable_camera_bridge:=false',
        'device:=cuda',
        f'checkpoint_path:={VISUAL_RUN / "best.pt"}',
        f'sidecar_directory:={VISUAL_RUN}',
        'dry_run_state_fusion:=true',
        'plansys2_update_enabled:=false',
    ]


def execution_command() -> list[str]:
    return [
        'ros2', 'launch', 'mfja_robot_control_config',
        'room_315_task_execution.launch.py',
        'use_sim_time:=true',
        'execution_enabled:=true',
        'enable_plansys2:=true',
        'external_obstacles_disabled:=true',
    ]


def bag_command(case_dir: Path, campaign_id: str, case_id: str) -> list[str]:
    return [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--storage-preset-profile', 'zstd_fast',
        '--use-sim-time',
        '--disable-keyboard-controls',
        '--output', str(case_dir / 'rosbag'),
        '--custom-data', f'campaign_id={campaign_id}', f'case_id={case_id}',
        '--topics', *RECORDED_TOPICS,
    ]


def visual_pythonpath(env: dict[str, str]) -> dict[str, str]:
    result = dict(env)
    current = result.get('PYTHONPATH', '')
    parts = ['/usr/lib/python3/dist-packages', str(VISUAL_VENV)]
    if current:
        parts.append(current)
    result['PYTHONPATH'] = ':'.join(parts)
    return result


def cli_environment(env: dict[str, str]) -> dict[str, str]:
    result = dict(env)
    current = result.get('PYTHONPATH', '')
    parts = ['/usr/lib/python3/dist-packages', str(SCRIPTS)]
    if current:
        parts.append(current)
    result['PYTHONPATH'] = ':'.join(parts)
    return result


def extract_json_after(text: str, marker: str) -> dict[str, Any]:
    start = text.find(marker)
    if start < 0:
        raise CampaignError(f'missing transcript marker: {marker.strip()}')
    start += len(marker)
    payload = text[start:].lstrip()
    try:
        value, _ = json.JSONDecoder().raw_decode(payload)
    except json.JSONDecodeError as exc:
        raise CampaignError(f'cannot parse JSON after {marker.strip()}: {exc}') from exc
    if not isinstance(value, dict):
        raise CampaignError(f'JSON after {marker.strip()} is not an object')
    return value


def run_confirmed_cli(
    case: dict[str, Any],
    *,
    env: dict[str, str],
    timeout_s: float,
) -> tuple[str, dict[str, Any], dict[str, Any]]:
    command = [
        sys.executable, '-u', str(SCRIPTS / 'room_315_task_goal_cli.py'),
        '--config', str(INTENT_CONFIG),
        '--publish-topic', '/room_315/task_goal',
        '--result-topic', '/room_315/task_goal/status',
        '--wait-for-result',
        '--publish-timeout-s', '10',
        '--result-timeout-s', str(timeout_s),
    ]
    process = subprocess.Popen(
        command,
        cwd=REPO,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=False,
        bufsize=0,
        start_new_session=True,
    )
    assert process.stdin is not None
    assert process.stdout is not None

    def send_input(value: str) -> None:
        process.stdin.write(value.encode('utf-8'))
        process.stdin.flush()

    send_input(str(case['utterance']) + '\n')
    stdout_fd = process.stdout.fileno()
    os.set_blocking(stdout_fd, False)
    process.stdin.flush()
    selector = selectors.DefaultSelector()
    selector.register(stdout_fd, selectors.EVENT_READ)
    output = bytearray()
    prompt_checked = False
    deadline = time.monotonic() + timeout_s + 60.0
    try:
        while time.monotonic() < deadline:
            events = selector.select(timeout=0.25)
            if not events and process.poll() is not None:
                break
            for _, _ in events:
                try:
                    chunk = os.read(stdout_fd, 65536)
                except BlockingIOError:
                    chunk = b''
                if chunk:
                    output.extend(chunk)
                if not prompt_checked:
                    rendered = output.decode('utf-8', errors='replace')
                    if 'Reply yes to finalize' not in rendered:
                        continue
                    missing = [
                        fragment
                        for fragment in case['expected'].get('prompt_fragments') or []
                        if fragment not in rendered
                    ]
                    if missing:
                        send_input('cancel\nquit\n')
                        raise CampaignError(
                            f'{case["id"]} confirmation did not match declaration: {missing}'
                        )
                    prompt_checked = True
                    send_input('yes\nquit\n')
            if process.poll() is not None:
                while True:
                    try:
                        remaining = os.read(stdout_fd, 65536)
                    except BlockingIOError:
                        break
                    if not remaining:
                        break
                    output.extend(remaining)
                break
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
            raise CampaignError(f'{case["id"]} language-to-motion CLI timed out')
    except BaseException:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
        raise
    finally:
        selector.close()
        if process.stdin and not process.stdin.closed:
            process.stdin.close()
        if process.stdout and not process.stdout.closed:
            process.stdout.close()
    transcript = output.decode('utf-8', errors='replace')
    if not prompt_checked:
        raise CampaignError(f'{case["id"]} did not reach operator confirmation')
    if process.returncode != 0:
        raise CampaignError(
            f'{case["id"]} CLI exited with {process.returncode}:\n{transcript[-4000:]}'
        )
    task_goal = extract_json_after(transcript, 'Final validated TaskGoal:\n')
    response = extract_json_after(transcript, 'Task execution response:\n')
    return transcript, task_goal, response


def validate_initial_visual(
    payload: dict[str, Any],
    case: dict[str, Any],
    expected_checkpoint_sha256: str,
) -> dict[str, Any]:
    required_true = (
        'accepted', 'model_ready', 'input_ready', 'presence_ready',
        'state_fusion_ready',
    )
    missing = [name for name in required_true if payload.get(name) is not True]
    if (
        missing
        or payload.get('stale') is not False
        or payload.get('stage') != 'fused_observed_state'
        or payload.get('schema_version') != VISUAL_SCHEMA
    ):
        raise CampaignError(
            f'{case["id"]} visual readiness rejected: missing={missing}, '
            f'stage={payload.get("stage")}, schema={payload.get("schema_version")}'
        )
    if payload.get('checkpoint_sha256') != expected_checkpoint_sha256:
        raise CampaignError(
            f'{case["id"]} visual checkpoint hash does not match the bound checkpoint'
        )
    validation_reasons = payload.get('validation_reasons')
    clamped_fields = payload.get('clamped_fields')
    allowed_projection = re.compile(
        r'^[RL][1-4]\.s_ratio_consistency_projection$'
    )
    unexpected_clamps = [
        str(field) for field in (clamped_fields or [])
        if not allowed_projection.fullmatch(str(field))
    ]
    if validation_reasons != [] or not isinstance(clamped_fields, list) or unexpected_clamps:
        raise CampaignError(
            f'{case["id"]} initial visual sample is not clean: '
            f'validation_reasons={validation_reasons}, '
            f'unexpected_clamped_fields={unexpected_clamps}'
        )
    if int(payload.get('accepted_frame_count') or 0) < 1:
        raise CampaignError(f'{case["id"]} visual runtime reports no accepted frame')
    if str((payload.get('header') or {}).get('frame_id') or '') != 'room_315':
        raise CampaignError(f'{case["id"]} visual frame_id is not room_315')
    if header_stamp(payload) <= (0, 0):
        raise CampaignError(f'{case["id"]} visual observation has no simulation timestamp')
    by_identity = {
        str(item.get('identity') or '').upper(): item
        for item in payload.get('shuttles') or []
        if isinstance(item, dict)
    }
    expected_identity_set = {
        *(f'L{index}' for index in range(1, 5)),
        *(f'R{index}' for index in range(1, 5)),
    }
    if set(by_identity) != expected_identity_set:
        raise CampaignError(
            f'{case["id"]} visual identity vector is incomplete: '
            f'{sorted(by_identity)}'
        )
    expected_present = {
        str(identity).upper()
        for side in ('right', 'left')
        for identity in case['launch'][side].get('identities') or []
    }
    actual_present = {
        identity
        for identity, item in by_identity.items()
        if str(item.get('presence_state') or '').casefold() == 'present'
    }
    if actual_present != expected_present:
        raise CampaignError(
            f'{case["id"]} presence mismatch: expected={sorted(expected_present)}, '
            f'actual={sorted(actual_present)}'
        )
    invalid_present = [
        identity for identity in expected_present
        if not bool(by_identity.get(identity, {}).get('visual_facts_valid'))
    ]
    if invalid_present:
        raise CampaignError(
            f'{case["id"]} present identities lack visual facts: {invalid_present}'
        )
    invalid_geometry: list[str] = []
    invalid_side: list[str] = []
    for identity in expected_present:
        item = by_identity[identity]
        _, declared_side, _ = identity_parts(identity)
        if str(item.get('side') or '').casefold() != declared_side:
            invalid_side.append(identity)
        bbox = item.get('bbox_xywh') or []
        values = [
            item.get('s_m'), item.get('s_ratio'), item.get('segment_length_m'),
            *bbox,
        ]
        try:
            numeric = [float(value) for value in values]
        except (TypeError, ValueError):
            numeric = []
        if (
            len(bbox) != 4
            or len(numeric) != 7
            or not all(math.isfinite(value) for value in numeric)
            or numeric[1] < 0.0
            or numeric[1] > 1.0
            or numeric[2] <= 0.0
            or numeric[5] <= 0.0
            or numeric[6] <= 0.0
            or not str(item.get('block') or '')
        ):
            invalid_geometry.append(identity)
    if invalid_side or invalid_geometry:
        raise CampaignError(
            f'{case["id"]} invalid present visual facts: '
            f'side={invalid_side}, geometry={invalid_geometry}'
        )
    payload_contract = case['expected'].get('payload_consensus') or {}
    if payload_contract.get('required') is True:
        expected_loaded = {
            str(identity).upper()
            for side in ('right', 'left')
            for identity in case['launch'][side].get('loaded') or []
        }
        payload_mismatches = {
            identity: {
                'expected': 'loaded' if identity in expected_loaded else 'empty',
                'actual': by_identity[identity].get('loaded_state'),
            }
            for identity in sorted(expected_present)
            if str(by_identity[identity].get('loaded_state') or '').casefold()
            != ('loaded' if identity in expected_loaded else 'empty')
        }
        if payload_mismatches:
            raise CampaignError(
                f'{case["id"]} visual payload state contradicts the declared '
                f'payload-qualified scene: {payload_mismatches}'
            )
    return {
        'expected_present_identities': sorted(expected_present),
        'actual_present_identities': sorted(actual_present),
        'checkpoint_sha256': payload.get('checkpoint_sha256'),
        'accepted_frame_count': payload.get('accepted_frame_count'),
        'validation_reasons': validation_reasons,
        'clamped_fields': clamped_fields,
        'canonical_s_m_projection_count': len(clamped_fields),
        'initial_visual_shuttles': payload.get('shuttles') or [],
    }


def validate_final_visual(
    payload: dict[str, Any],
    case: dict[str, Any],
    expected_checkpoint_sha256: str,
) -> dict[str, Any]:
    required_true = (
        'accepted', 'model_ready', 'input_ready', 'presence_ready',
        'state_fusion_ready',
    )
    missing = [name for name in required_true if payload.get(name) is not True]
    clamped_fields = payload.get('clamped_fields')
    allowed_projection = re.compile(
        r'^[RL][1-4]\.s_ratio_consistency_projection$'
    )
    unexpected_clamps = [
        str(field) for field in (clamped_fields or [])
        if not allowed_projection.fullmatch(str(field))
    ]
    if (
        missing
        or payload.get('stale') is not False
        or payload.get('stage') != 'fused_observed_state'
        or payload.get('schema_version') != VISUAL_SCHEMA
        or payload.get('checkpoint_sha256') != expected_checkpoint_sha256
        or payload.get('validation_reasons') != []
        or not isinstance(clamped_fields, list)
        or unexpected_clamps
    ):
        raise CampaignError(
            f'{case["id"]} final visual observation failed its mandatory envelope: '
            f'missing={missing}, stage={payload.get("stage")}, '
            f'checkpoint={payload.get("checkpoint_sha256")}, '
            f'validation_reasons={payload.get("validation_reasons")}, '
            f'unexpected_clamped_fields={unexpected_clamps}'
        )
    by_identity = {
        str(item.get('identity') or '').upper(): item
        for item in payload.get('shuttles') or []
        if isinstance(item, dict)
    }
    expected_present = {
        str(identity).upper()
        for side in ('right', 'left')
        for identity in case['launch'][side].get('identities') or []
    }
    actual_present = {
        identity for identity, item in by_identity.items()
        if str(item.get('presence_state') or '').casefold() == 'present'
    }
    if actual_present != expected_present:
        raise CampaignError(
            f'{case["id"]} final visual presence mismatch: '
            f'expected={sorted(expected_present)}, actual={sorted(actual_present)}'
        )
    invalid_final = [
        identity for identity in sorted(expected_present)
        if by_identity[identity].get('visual_facts_valid') is not True
        or str(by_identity[identity].get('side') or '').casefold()
        != identity_parts(identity)[1]
    ]
    if invalid_final:
        raise CampaignError(
            f'{case["id"]} final visual facts are invalid for {invalid_final}'
        )
    return {
        'checkpoint_sha256': payload.get('checkpoint_sha256'),
        'accepted_frame_count': int(payload.get('accepted_frame_count') or 0),
        'expected_present_identities': sorted(expected_present),
        'actual_present_identities': sorted(actual_present),
        'clamped_fields': clamped_fields,
        'canonical_s_m_projection_count': len(clamped_fields),
    }


def parse_string_json_topic(result: dict[str, Any], label: str) -> dict[str, Any]:
    if result.get('returncode') != 0:
        raise CampaignError(
            f'{label} topic capture failed: '
            f'{result.get("stderr") or result.get("stdout")}'
        )
    message = parse_ros_yaml(str(result.get('stdout') or ''))
    raw = message.get('data')
    if not isinstance(raw, str):
        raise CampaignError(f'{label} topic did not contain a string payload')
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise CampaignError(f'{label} topic JSON is invalid: {exc}') from exc
    if not isinstance(payload, dict):
        raise CampaignError(f'{label} topic JSON is not an object')
    return payload


def capture_supervisor_snapshot(
    env: dict[str, str], timeout_s: float, label: str,
) -> dict[str, Any]:
    echo = topic_echo(
        '/room_315/vla/status', 'std_msgs/msg/String',
        env=env, timeout_s=timeout_s,
    )
    return {'echo': echo, 'payload': parse_string_json_topic(echo, label)}


def safety_metrics(snapshot: dict[str, Any]) -> dict[str, int]:
    raw = ((snapshot.get('safety_decoder') or {}).get('metrics') or {})
    return {
        name: int(raw.get(name) or 0)
        for name in ('total_proposed_actions', 'accepted_actions', 'rejected_actions')
    }


def supervisor_metrics_delta(
    before: dict[str, Any], after: dict[str, Any], case_id: str,
) -> dict[str, int]:
    initial = safety_metrics(before)
    final = safety_metrics(after)
    delta = {name: final[name] - initial[name] for name in initial}
    if any(value < 0 for value in delta.values()):
        raise CampaignError(f'{case_id} supervisor safety counters moved backwards')
    if (
        delta['total_proposed_actions'] < 1
        or delta['accepted_actions'] != delta['total_proposed_actions']
        or delta['rejected_actions'] != 0
    ):
        raise CampaignError(
            f'{case_id} supervisor metric delta is not an all-accepted execution: {delta}'
        )
    return {'initial': initial, 'final': final, 'delta': delta}


def _reading_for_target(
    payload: dict[str, Any], sensor: str, entity: str,
) -> dict[str, Any]:
    matches = [
        item for item in payload.get('readings') or []
        if isinstance(item, dict)
        and str(item.get('name') or '').upper() == sensor
        and int(item.get('active') or 0) == 1
    ]
    if len(matches) != 1:
        raise CampaignError(
            f'expected one active {sensor} reading, received {len(matches)}'
        )
    reading = matches[0]
    if (
        str(reading.get('shuttle_name') or '') != entity
        or str(reading.get('sensor_type') or '').casefold() != 'sensor'
    ):
        raise CampaignError(
            f'{sensor} did not identify {entity}: {reading}'
        )
    return reading


def capture_initial_controller_evidence(
    case: dict[str, Any],
    *,
    env: dict[str, str],
    timeout_s: float,
    destination: Path,
) -> dict[str, Any]:
    record: dict[str, Any] = {'status': 'failed', 'sides': {}}
    try:
        for side in ('right', 'left'):
            side_launch = case['launch'][side]
            identities = [str(value).upper() for value in side_launch.get('identities') or []]
            slots = [str(value) for value in side_launch.get('start_slots') or []]
            if not identities:
                continue
            suffix = 'R' if side == 'right' else 'L'
            sensor_echo = topic_echo(
                f'/room_315/rails/{side}/sensors/feedback',
                'mfja_rail_interfaces/msg/SensorFeedback',
                env=env, timeout_s=timeout_s,
            )
            if sensor_echo.get('returncode') != 0:
                raise CampaignError(f'{case["id"]} initial {side} sensor capture failed')
            sensor_payload = parse_ros_yaml(sensor_echo['stdout'])
            side_record: dict[str, Any] = {
                'sensor_echo': sensor_echo,
                'sensor_payload': sensor_payload,
                'shuttles': [],
            }
            record['sides'][side] = side_record
            for identity, slot in zip(identities, slots):
                _, identity_side, _ = identity_parts(identity)
                if identity_side != side:
                    raise CampaignError(
                        f'{case["id"]} identity {identity} is declared on {side}'
                    )
                entity = controller_entity(identity)
                sensor = f'DZI{slot}{suffix}'
                reading = _reading_for_target(sensor_payload, sensor, entity)
                state_echo = topic_echo(
                    f'/room_315/rails/{side}/shuttles/state',
                    'mfja_rail_interfaces/msg/ShuttleState',
                    env=env, timeout_s=timeout_s,
                    filter_expression=f'm.name == {entity!r}',
                )
                if state_echo.get('returncode') != 0:
                    raise CampaignError(
                        f'{case["id"]} initial controller state for {identity} was not captured'
                    )
                state = parse_ros_yaml(state_echo['stdout'])
                if (
                    str(state.get('name') or '') != entity
                    or str(state.get('mode') or '').upper() != 'DISABLED'
                    or str(state.get('reached_target_slot') or '')
                ):
                    raise CampaignError(
                        f'{case["id"]} initial controller state mismatch for '
                        f'{identity}: {state}'
                    )
                side_record['shuttles'].append({
                    'identity': identity,
                    'entity': entity,
                    'declared_start_slot': slot,
                    'declared_start_sensor': sensor,
                    'sensor_reading': reading,
                    'state_echo': state_echo,
                    'state': state,
                })
        record['status'] = 'passed'
        return record
    finally:
        write_json(destination, record)


def capture_post_task_controller_evidence(
    case: dict[str, Any],
    *,
    env: dict[str, str],
    target_slot: str,
    timeout_s: float,
    destination: Path,
) -> dict[str, Any]:
    identity = str(case['expected']['selected_identity']).upper()
    _, side, _ = identity_parts(identity)
    entity = controller_entity(identity)
    suffix = 'R' if side == 'right' else 'L'
    sensor = f'DZI{target_slot}{suffix}'
    record: dict[str, Any] = {
        'status': 'failed',
        'identity': identity,
        'entity': entity,
        'target_slot': target_slot,
        'target_sensor': sensor,
        'sensor_samples': [],
    }
    try:
        stamps: list[tuple[int, int]] = []
        for sample_index in range(2):
            echo = topic_echo(
                f'/room_315/rails/{side}/sensors/feedback',
                'mfja_rail_interfaces/msg/SensorFeedback',
                env=env, timeout_s=timeout_s,
            )
            if echo.get('returncode') != 0:
                raise CampaignError(
                    f'{case["id"]} post-task sensor sample {sample_index + 1} failed'
                )
            payload = parse_ros_yaml(echo['stdout'])
            stamp = header_stamp(payload)
            reading = _reading_for_target(payload, sensor, entity)
            record['sensor_samples'].append({
                'sample_index': sample_index + 1,
                'echo': echo,
                'payload': payload,
                'matching_reading': reading,
                'header_stamp': {'sec': stamp[0], 'nanosec': stamp[1]},
            })
            stamps.append(stamp)
        if stamps[0] <= (0, 0) or stamps[1] <= (0, 0) or stamps[0] == stamps[1]:
            raise CampaignError(
                f'{case["id"]} post-task sensor samples are not distinct: {stamps}'
            )
        state_echo = topic_echo(
            f'/room_315/rails/{side}/shuttles/state',
            'mfja_rail_interfaces/msg/ShuttleState',
            env=env, timeout_s=timeout_s,
            filter_expression=f'm.name == {entity!r}',
        )
        record['shuttle_state_echo'] = state_echo
        if state_echo.get('returncode') != 0:
            raise CampaignError(
                f'{case["id"]} target-identity ShuttleState capture failed'
            )
        state = parse_ros_yaml(state_echo['stdout'])
        record['shuttle_state'] = state
        if (
            str(state.get('name') or '') != entity
            or str(state.get('mode') or '').upper() != 'DISABLED'
            or str(state.get('reached_target_slot') or '') != target_slot
        ):
            raise CampaignError(
                f'{case["id"]} independent ShuttleState does not certify '
                f'{entity} stopped at slot {target_slot}: {state}'
            )
        record['status'] = 'passed'
        return record
    finally:
        write_json(destination, record)


def validate_task_goal(task_goal: dict[str, Any], case: dict[str, Any]) -> None:
    constraints = task_goal.get('constraints')
    if not isinstance(constraints, dict):
        raise CampaignError(f'{case["id"]} TaskGoal constraints missing')
    mismatches = {
        field: {'expected': expected, 'actual': constraints.get(field)}
        for field, expected in case['expected']['constraints'].items()
        if constraints.get(field) != expected
    }
    if mismatches:
        raise CampaignError(f'{case["id"]} TaskGoal mismatch: {mismatches}')


def arrival_from_steps(steps: list[dict[str, Any]]) -> dict[str, Any]:
    for step in reversed(steps):
        details = ((step.get('postcondition') or {}).get('details') or {})
        arrival = details.get('arrival_verification')
        if isinstance(arrival, dict):
            return arrival
    return {}


def validate_declared_symbolic_sequence(
    steps: list[dict[str, Any]], case: dict[str, Any],
) -> dict[str, Any]:
    expected = case['expected']
    route_contract = expected.get('route_clearance') or {}
    exact_count = expected.get('exact_executed_step_count')
    if exact_count is not None and len(steps) != int(exact_count):
        raise CampaignError(
            f'{case["id"]} executed {len(steps)} steps; the declaration '
            f'requires exactly {int(exact_count)}'
        )
    prefixes = (
        expected.get('symbolic_action_prefixes')
        or expected.get('expected_symbolic_action_prefixes')
        or route_contract.get('exact_action_names')
        or []
    )
    symbolic_steps = [str(step.get('symbolic_step') or '') for step in steps]
    if prefixes:
        if not isinstance(prefixes, list) or len(prefixes) != len(symbolic_steps):
            raise CampaignError(
                f'{case["id"]} declared symbolic prefix count does not match '
                f'the {len(symbolic_steps)} executed steps'
            )
        mismatches = [
            {'index': index, 'expected_prefix': prefix, 'actual': actual}
            for index, (prefix, actual) in enumerate(zip(prefixes, symbolic_steps))
            if not actual.startswith(str(prefix))
        ]
        if mismatches:
            raise CampaignError(
                f'{case["id"]} symbolic sequence mismatch: {mismatches}'
            )
    exact_symbolic_steps = route_contract.get('exact_symbolic_steps') or []
    if exact_symbolic_steps and symbolic_steps != [
        str(value) for value in exact_symbolic_steps
    ]:
        raise CampaignError(
            f'{case["id"]} executed symbolic trace differs from the '
            f'predeclared trace: actual={symbolic_steps}'
        )
    blocker = (
        expected.get('blocker_identity')
        or expected.get('expected_blocker_identity')
        or expected.get('expected_blocker')
        or route_contract.get('blocker_identity')
        or ''
    )
    if isinstance(blocker, dict):
        blocker = blocker.get('identity') or ''
    if blocker:
        blocker_identity = str(blocker).upper()
        blocker_symbol = symbolic_entity(blocker_identity)
        relocation_steps = [
            step for step in steps
            if str(step.get('symbolic_step') or '').startswith(
                'relocate_blocker_to_interior '
            )
            or str(step.get('symbolic_step') or '').startswith(
                'relocate_segment_blocker_to_interior '
            )
        ]
        if not relocation_steps:
            raise CampaignError(
                f'{case["id"]} declared blocker {blocker_identity} was not relocated'
            )
        first_tokens = str(relocation_steps[0].get('symbolic_step') or '').split()
        if len(first_tokens) < 2 or first_tokens[1] != blocker_symbol:
            raise CampaignError(
                f'{case["id"]} relocated blocker is not {blocker_symbol}: {first_tokens}'
            )
        certificate = (
            ((relocation_steps[0].get('postcondition') or {}).get('details') or {})
            .get('route_clearance_certificate') or {}
        )
        if certificate and str(certificate.get('identity') or '').upper() != blocker_identity:
            raise CampaignError(
                f'{case["id"]} blocker certificate identity mismatch: {certificate}'
            )
    return {
        'exact_executed_step_count': int(exact_count) if exact_count is not None else None,
        'declared_prefixes': [str(value) for value in prefixes],
        'declared_exact_symbolic_steps': [
            str(value) for value in exact_symbolic_steps
        ],
        'declared_blocker_identity': str(blocker).upper() if blocker else '',
        'symbolic_steps': symbolic_steps,
    }


def step_is_actuating(step: dict[str, Any]) -> bool:
    postcondition = step.get('postcondition') or {}
    details = postcondition.get('details') or {}
    action = str(step.get('symbolic_step') or '').split(maxsplit=1)[0]
    if details.get('supervisor_command_published') is False:
        return False
    if postcondition.get('reason') == 'non_actuating_terminal_marker':
        return False
    if action in TERMINAL_SYMBOLIC_ACTIONS:
        return False
    return True


def validate_response(
    response: dict[str, Any],
    case: dict[str, Any],
    supervisor_delta: dict[str, Any],
) -> dict[str, Any]:
    if response.get('status') != 'succeeded' or response.get('reason') != 'task_goal_satisfied':
        raise CampaignError(
            f'{case["id"]} terminal status is not a successful task goal: '
            f'{response.get("status")} / {response.get("reason")}'
        )
    result = response.get('result')
    if not isinstance(result, dict) or result.get('status') != 'succeeded':
        raise CampaignError(f'{case["id"]} internal executive result is not succeeded')
    if result.get('safe_abort_sent') is not False:
        raise CampaignError(f'{case["id"]} sent a safe abort during a success case')
    steps = result.get('executed_steps')
    if not isinstance(steps, list):
        raise CampaignError(f'{case["id"]} has no executed-step list')
    plan_attempts = int(result.get('plan_attempts') or 0)
    if plan_attempts < 1:
        raise CampaignError(f'{case["id"]} succeeded without a PlanSys2 plan attempt')
    minimum = int(case['expected'].get('minimum_executed_steps', 1))
    if len(steps) < minimum:
        raise CampaignError(
            f'{case["id"]} executed {len(steps)} steps; declared minimum is {minimum}'
        )
    bad_steps = [
        step.get('step_index')
        for step in steps
        if step.get('supervisor_status') != 'accepted'
        or (step.get('postcondition') or {}).get('status') != 'satisfied'
    ]
    if bad_steps:
        raise CampaignError(f'{case["id"]} has unaccepted or unsatisfied steps: {bad_steps}')
    arrival = arrival_from_steps(steps)
    if not arrival or arrival.get('arrived') is not True:
        raise CampaignError(f'{case["id"]} has no verified final arrival')
    expected_identity = str(case['expected']['selected_identity']).upper()
    expected_side = str(case['side']).casefold()
    expected_shuttle = controller_entity(expected_identity)
    expected_shuttle_contract = symbolic_entity(expected_identity)
    certificate = arrival.get('verified_slot_arrival_certificate') or {}
    if str(certificate.get('identity') or '').upper() != expected_identity:
        raise CampaignError(
            f'{case["id"]} arrived identity mismatch: {certificate.get("identity")}'
        )
    allowed_slots = {
        str(value)
        for value in (
            case['expected'].get('target_slots')
            or [case['expected'].get('target_slot')]
        )
        if value is not None
    }
    actual_slot = str(arrival.get('target_slot') or '')
    if actual_slot not in allowed_slots:
        raise CampaignError(
            f'{case["id"]} arrived slot {actual_slot!r} not in {sorted(allowed_slots)}'
        )
    suffix = 'R' if case['side'] == 'right' else 'L'
    expected_sensor = f'DZI{actual_slot}{suffix}'
    if arrival.get('target_sensor') != expected_sensor:
        raise CampaignError(
            f'{case["id"]} target sensor mismatch: {arrival.get("target_sensor")}'
        )
    required_arrival = {
        'sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'controller_target_slot_confirmed': True,
        'controller_position_fields_used_for_localization': False,
        'visual_localization_used_for_final_stop': False,
    }
    arrival_mismatch = {
        key: {'expected': expected, 'actual': arrival.get(key)}
        for key, expected in required_arrival.items()
        if arrival.get(key) != expected
    }
    if arrival_mismatch:
        raise CampaignError(f'{case["id"]} arrival proof mismatch: {arrival_mismatch}')
    if int(arrival.get('sensor_confirmation_frames') or 0) < 2:
        raise CampaignError(f'{case["id"]} has fewer than two sensor confirmations')
    if certificate.get('controller_mode') != 'DISABLED':
        raise CampaignError(f'{case["id"]} controller was not disabled at arrival')
    expected_station = str(case['expected'].get('target_station') or '')
    outer_expected = {
        'shuttle': expected_shuttle,
        'shuttle_contract': expected_shuttle_contract,
        'side': expected_side,
        'target_sensor': expected_sensor,
        'target_slot': actual_slot,
        'target_slot_contract': f'{expected_side}:slot:{actual_slot}',
        'target_station': expected_station,
        'matched_by': 'deterministic_slot_sensor',
        'ground_truth_scope': 'final_actuation_verification_only',
        'planner_localization_unchanged': True,
    }
    outer_mismatch = {
        key: {'expected': expected, 'actual': arrival.get(key)}
        for key, expected in outer_expected.items()
        if arrival.get(key) != expected
    }
    if outer_mismatch:
        raise CampaignError(
            f'{case["id"]} outer arrival certificate mismatch: {outer_mismatch}'
        )
    if arrival.get('matched_sensors') != [expected_sensor]:
        raise CampaignError(
            f'{case["id"]} arrival sensor set is not uniquely {expected_sensor}'
        )
    certificate_expected = {
        'identity': expected_identity,
        'shuttle': expected_shuttle,
        'side': expected_side,
        'sensor': expected_sensor,
        'slot': actual_slot,
        'reached_target_slot': actual_slot,
        'controller_mode': 'DISABLED',
        'matched_by': 'deterministic_slot_sensor',
        'proof_mode': 'supervised_command_arrival',
        'sensor_identity_confirmed': True,
        'controller_stop_confirmed': True,
        'controller_target_slot_confirmed': True,
        'controller_position_fields_used_for_localization': False,
        'model_prediction_replaced': False,
    }
    certificate_mismatch = {
        key: {'expected': expected, 'actual': certificate.get(key)}
        for key, expected in certificate_expected.items()
        if certificate.get(key) != expected
    }
    if certificate_mismatch:
        raise CampaignError(
            f'{case["id"]} inner arrival certificate mismatch: {certificate_mismatch}'
        )
    if int(certificate.get('sensor_confirmation_frames') or 0) != int(
        arrival.get('sensor_confirmation_frames') or 0
    ):
        raise CampaignError(
            f'{case["id"]} inner/outer sensor confirmation counts disagree'
        )
    for sequence_field in ('motion_epoch', 'sensor_sequence', 'supervisor_sequence'):
        if int(certificate.get(sequence_field) or 0) < 1:
            raise CampaignError(
                f'{case["id"]} certificate lacks positive {sequence_field}'
            )
    if (
        response.get('controller_position_fields_used_for_localization') is not False
        or response.get('location_source') != 'accepted_visual_state'
    ):
        raise CampaignError(
            f'{case["id"]} response provenance contradicts the arrival certificate'
        )
    symbolic_validation = validate_declared_symbolic_sequence(steps, case)
    actuating_steps = [step for step in steps if step_is_actuating(step)]
    accepted_actuating_steps = [
        step for step in actuating_steps if step.get('supervisor_status') == 'accepted'
    ]
    actual_decisions = int(
        (supervisor_delta.get('delta') or {}).get('accepted_actions') or 0
    )
    if len(accepted_actuating_steps) != len(actuating_steps):
        raise CampaignError(f'{case["id"]} has an unaccepted actuating step')
    if actual_decisions < len(accepted_actuating_steps):
        raise CampaignError(
            f'{case["id"]} supervisor metric delta {actual_decisions} is smaller '
            f'than the {len(accepted_actuating_steps)} accepted actuating steps'
        )
    return {
        'status': 'passed',
        'goal_id': response.get('goal_id'),
        'executed_step_count': len(steps),
        'satisfied_postcondition_count': len(steps),
        'actuating_step_count': len(actuating_steps),
        'accepted_actuating_step_count': len(accepted_actuating_steps),
        'non_actuating_step_count': len(steps) - len(actuating_steps),
        'accepted_supervisor_decision_count': actual_decisions,
        'accepted_supervisor_decision_source': 'safety_decoder_metrics_delta',
        'supervisor_metrics': supervisor_delta,
        'plan_attempts': plan_attempts,
        'replans': int(result.get('replans') or 0),
        'unknown_retries': int(result.get('unknown_retries') or 0),
        'safe_abort_sent': False,
        'selected_identity': expected_identity,
        'target_slot': actual_slot,
        'target_sensor': expected_sensor,
        'sensor_confirmation_frames': int(arrival.get('sensor_confirmation_frames') or 0),
        'controller_mode_at_arrival': certificate.get('controller_mode'),
        'symbolic_steps': [step.get('symbolic_step') for step in steps],
        'primitive_steps': [step.get('primitive') for step in steps],
        'symbolic_declaration_validation': symbolic_validation,
    }


def snapshot_active_logs(
    processes: list[ManagedProcess],
    destination: Path,
) -> list[str]:
    issues: list[str] = []
    destination.mkdir(parents=True, exist_ok=True)
    for process in processes:
        if not process.log_stream.closed:
            process.log_stream.flush()
        with process.log_path.open('rb') as stream:
            stream.seek(process.evidence_offset)
            text = stream.read().decode('utf-8', errors='replace')
        write_text(destination / f'{process.name}.log', text)
        for line_number, line in enumerate(text.splitlines(), start=1):
            if any(pattern.search(line) for pattern in ACTIVE_LOG_ERROR_PATTERNS):
                issues.append(f'{process.name}:{line_number}:{line}')
    return issues


def snapshot_full_logs(
    processes: list[ManagedProcess], destination: Path,
) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for process in processes:
        if not process.log_stream.closed:
            process.log_stream.flush()
        if process.log_path.is_file():
            shutil.copy2(process.log_path, destination / f'{process.name}.log')


PAYLOAD_CONSENSUS_PATTERN = re.compile(
    r'Payload-qualified TaskGoal grounded by fresh visual consensus:\s*'
    r'selected=(?P<selected>\S+)\s+'
    r'filter=(?P<filter>\S+)\s+'
    r'frames=(?P<frames>\d+)/(?P<required_frames>\d+)\s+'
    r'observations=(?P<observations>\d+)\s+'
    r'source_state=(?P<source_state>\S+)'
)


def validate_payload_consensus_log(
    execution: ManagedProcess,
    case: dict[str, Any],
    destination: Path,
) -> dict[str, Any]:
    payload_filter = str(
        (case['expected'].get('constraints') or {}).get('payload_filter') or ''
    )
    if not payload_filter:
        record = {'status': 'not_applicable'}
        write_json(destination, record)
        return record
    record: dict[str, Any] = {'status': 'failed'}
    try:
        if not execution.log_stream.closed:
            execution.log_stream.flush()
        with execution.log_path.open('rb') as stream:
            stream.seek(execution.evidence_offset)
            text = stream.read().decode('utf-8', errors='replace')
        matches = list(PAYLOAD_CONSENSUS_PATTERN.finditer(text))
        record['match_count'] = len(matches)
        if len(matches) != 1:
            raise CampaignError(
                f'{case["id"]} expected one payload-consensus record; '
                f'found {len(matches)}'
            )
        values = matches[0].groupdict()
        record.update({
            'selected': values['selected'],
            'payload_filter': values['filter'],
            'frames': int(values['frames']),
            'required_frames': int(values['required_frames']),
            'observations': int(values['observations']),
            'source_state': values['source_state'],
            'matched_log_line': matches[0].group(0),
        })
        consensus_contract = case['expected'].get('payload_consensus') or {}
        expected_selected = str(
            consensus_contract.get('selected_shuttle')
            or symbolic_entity(case['expected']['selected_identity'])
        )
        minimum_frames = int(
            consensus_contract.get('minimum_consecutive_frames') or 1
        )
        if (
            record['selected'] != expected_selected
            or record['payload_filter'] != payload_filter
            or record['frames'] != record['required_frames']
            or record['required_frames'] < minimum_frames
            or record['observations'] < record['frames']
            or not record['source_state']
        ):
            raise CampaignError(
                f'{case["id"]} payload-consensus record contradicts '
                f'the declaration: {record}'
            )
        record['declared_minimum_consecutive_frames'] = minimum_frames
        record['confirmation_source'] = consensus_contract.get(
            'confirmation_source'
        )
        record['controller_payload_state_used'] = consensus_contract.get(
            'controller_payload_state_used'
        )
        record['status'] = 'passed'
        return record
    except Exception as exc:
        record['failure_reason'] = str(exc)
        raise
    finally:
        write_json(destination, record)


def validate_rosbag(
    case_dir: Path,
    case: dict[str, Any],
    response: dict[str, Any],
    *,
    env: dict[str, str],
) -> dict[str, Any]:
    bag_dir = case_dir / 'rosbag'
    metadata_path = bag_dir / 'metadata.yaml'
    if not metadata_path.is_file():
        raise CampaignError(f'{case["id"]} rosbag metadata.yaml is missing')
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    information = metadata.get('rosbag2_bagfile_information')
    if not isinstance(information, dict):
        raise CampaignError(f'{case["id"]} rosbag metadata root is invalid')
    topic_counts: dict[str, int] = {}
    for entry in information.get('topics_with_message_count') or []:
        if not isinstance(entry, dict):
            continue
        topic = ((entry.get('topic_metadata') or {}).get('name'))
        if topic:
            topic_counts[str(topic)] = int(entry.get('message_count') or 0)
    side = str(case['side'])
    required_topics = {
        '/clock',
        '/room_315/task_goal',
        '/room_315/task_goal/status',
        '/room_315/visual_state/raw_model_prediction',
        '/room_315/visual_state/validation',
        '/room_315/visual_state/observed_state',
        '/room_315/vla/command',
        '/room_315/vla/status',
        '/room_315/vla/right_rail_rgbd/image',
        '/room_315/vla/left_rail_rgbd/image',
        f'/room_315/rails/{side}/shuttles/state',
        f'/room_315/rails/{side}/shuttles/command',
        f'/room_315/rails/{side}/sensors/feedback',
    }
    constraints = case['expected'].get('constraints') or {}
    if constraints.get('payload_filter'):
        required_topics.add(f'/room_315/rails/{side}/shuttles/payload_state')
    symbolic_steps = [
        str(step.get('symbolic_step') or '')
        for step in ((response.get('result') or {}).get('executed_steps') or [])
    ]
    if any('route_clearance' in step for step in symbolic_steps):
        required_topics.update({
            f'/room_315/rails/{side}/switches/command',
            f'/room_315/rails/{side}/stoppers/command',
        })
    missing_or_empty = {
        topic: topic_counts.get(topic, 0)
        for topic in sorted(required_topics)
        if topic_counts.get(topic, 0) < 1
    }
    total_messages = int(information.get('message_count') or 0)
    duration_ns = int((information.get('duration') or {}).get('nanoseconds') or 0)
    relative_files = [
        bag_dir / str(path)
        for path in information.get('relative_file_paths') or []
    ]
    invalid_files = [
        str(path) for path in relative_files
        if not path.is_file() or path.stat().st_size < 1
    ]
    if (
        information.get('storage_identifier') != 'mcap'
        or total_messages < 1
        or duration_ns < 1
        or not relative_files
        or invalid_files
        or missing_or_empty
    ):
        raise CampaignError(
            f'{case["id"]} rosbag is incomplete: total={total_messages}, '
            f'duration_ns={duration_ns}, invalid_files={invalid_files}, '
            f'missing_or_empty={missing_or_empty}'
        )
    info = run_capture(
        ['ros2', 'bag', 'info', str(bag_dir)],
        env=env, timeout_s=60.0,
    )
    write_json(case_dir / 'rosbag_info.json', info)
    write_text(case_dir / 'rosbag_info.txt', str(info.get('stdout') or ''))
    if info.get('returncode') != 0 or 'Messages:' not in str(info.get('stdout') or ''):
        raise CampaignError(f'{case["id"]} ros2 bag info failed')
    validation = {
        'status': 'passed',
        'storage_identifier': information.get('storage_identifier'),
        'message_count': total_messages,
        'duration_ns': duration_ns,
        'required_topic_message_counts': {
            topic: topic_counts[topic] for topic in sorted(required_topics)
        },
        'relative_files': [str(path.relative_to(case_dir)) for path in relative_files],
    }
    write_json(case_dir / 'rosbag_validation.json', validation)
    return validation


def send_stop_all(env: dict[str, str], reason: str) -> dict[str, Any]:
    message = json.dumps({
        'action': 'stop_all',
        'close_stoppers': False,
        'reason': reason,
    }, separators=(',', ':'))
    return run_capture(
        [
            'ros2', 'topic', 'pub', '--once',
            '/room_315/vla/command', 'std_msgs/msg/String',
            f'{{data: {json.dumps(message)}}}',
        ],
        env=env,
        timeout_s=10.0,
    )


def run_case(
    case: dict[str, Any],
    *,
    campaign_id: str,
    case_dir: Path,
    domain_id: int,
    protocol: dict[str, Any],
) -> dict[str, Any]:
    case_dir.mkdir(parents=True, exist_ok=False)
    write_json(case_dir / 'declaration.json', case)
    write_text(
        case_dir / 'operator_input.txt',
        f'{case["utterance"]}\nyes\nquit\n',
    )
    started_utc = utc_now()
    started = time.monotonic()
    temporary_root = Path(tempfile.mkdtemp(prefix=f'room315_{case["id"]}_'))
    base_env = clean_environment(domain_id, temporary_root / 'ros_logs_base')
    processes: list[ManagedProcess] = []
    teardown: list[dict[str, Any]] = []
    bag: ManagedProcess | None = None
    bag_stopped = False
    active_issues: list[str] = []
    finalization_errors: list[str] = []
    result_record: dict[str, Any] = {
        'case_id': case['id'],
        'status': 'failed',
        'started_at_utc': started_utc,
        'ros_domain_id': domain_id,
    }
    try:
        verify_runtime_lock_free()
        preexisting_graph = verify_empty_ros_graph(base_env)
        write_json(
            case_dir / 'readiness' / 'preexisting_graph.json',
            preexisting_graph,
        )
        floor = start_process(
            'floor', floor_command(case, float(protocol['shuttle_speed_mps'])),
            env=clean_environment(domain_id, temporary_root / 'ros_logs_floor'),
            log_path=temporary_root / 'floor.log',
        )
        processes.append(floor)
        world = topic_echo(
            '/clock', 'rosgraph_msgs/msg/Clock',
            env=base_env, timeout_s=float(protocol['readiness_timeout_s']),
        )
        write_json(case_dir / 'readiness' / 'world.json', world)
        require_alive(floor)
        if world.get('returncode') != 0:
            raise CampaignError(f'{case["id"]} world clock did not become ready')

        supervisor = topic_echo(
            '/room_315/vla/status', 'std_msgs/msg/String',
            env=base_env, timeout_s=float(protocol['readiness_timeout_s']),
        )
        write_json(case_dir / 'readiness' / 'supervisor.json', supervisor)
        require_alive(floor)
        if supervisor.get('returncode') != 0:
            raise CampaignError(f'{case["id"]} supervisor status did not become ready')

        initial_controller = capture_initial_controller_evidence(
            case,
            env=base_env,
            timeout_s=float(protocol['readiness_timeout_s']),
            destination=case_dir / 'readiness' / 'initial_controller_evidence.json',
        )
        if initial_controller.get('status') != 'passed':
            raise CampaignError(f'{case["id"]} initial controller evidence failed')

        visual = start_process(
            'visual', visual_command(),
            env=visual_pythonpath(
                clean_environment(domain_id, temporary_root / 'ros_logs_visual')
            ),
            log_path=temporary_root / 'visual.log',
        )
        processes.append(visual)
        visual_echo = topic_echo(
            '/room_315/visual_state/observed_state',
            'mfja_rail_interfaces/msg/VisualStateObservation',
            env=base_env,
            timeout_s=float(protocol['readiness_timeout_s']),
        )
        write_json(case_dir / 'readiness' / 'visual_topic.json', visual_echo)
        require_alive(visual)
        if visual_echo.get('returncode') != 0:
            raise CampaignError(f'{case["id"]} accepted visual state did not become ready')
        visual_payload = parse_ros_yaml(visual_echo['stdout'])
        expected_checkpoint_sha256 = sha256_file(VISUAL_RUN / 'best.pt')
        visual_check = validate_initial_visual(
            visual_payload, case, expected_checkpoint_sha256,
        )
        write_json(case_dir / 'readiness' / 'visual_check.json', visual_check)

        execution = start_process(
            'execution', execution_command(),
            env=clean_environment(domain_id, temporary_root / 'ros_logs_execution'),
            log_path=temporary_root / 'execution.log',
        )
        processes.append(execution)
        planner = wait_for_planner(base_env, float(protocol['readiness_timeout_s']))
        write_json(case_dir / 'readiness' / 'planner.json', planner)
        subscriber = wait_for_task_subscriber(
            base_env, float(protocol['readiness_timeout_s'])
        )
        write_json(case_dir / 'readiness' / 'task_subscriber.json', subscriber)
        require_alive(execution)

        graph = run_capture(
            ['ros2', 'node', 'list'], env=base_env, timeout_s=10.0,
        )
        write_json(case_dir / 'readiness' / 'node_graph.json', graph)

        supervisor_before = capture_supervisor_snapshot(
            base_env, 30.0, f'{case["id"]} pre-task supervisor',
        )
        write_json(
            case_dir / 'readiness' / 'supervisor_pre_task.json',
            supervisor_before,
        )
        initial_metrics = safety_metrics(supervisor_before['payload'])
        if any(initial_metrics.values()):
            raise CampaignError(
                f'{case["id"]} supervisor metrics are not zero at cold start: '
                f'{initial_metrics}'
            )

        for process in processes:
            mark_evidence_window_start(process)

        bag = start_process(
            'rosbag', bag_command(case_dir, campaign_id, case['id']),
            env=clean_environment(domain_id, temporary_root / 'ros_logs_bag'),
            log_path=temporary_root / 'rosbag.log',
        )
        time.sleep(2.0)
        require_alive(bag)

        transcript, task_goal, response = run_confirmed_cli(
            case,
            env=cli_environment(base_env),
            timeout_s=float(protocol['task_result_timeout_s']),
        )
        write_text(case_dir / 'cli_transcript.log', transcript)
        write_json(case_dir / 'task_goal.json', task_goal)
        write_json(case_dir / 'terminal_response.json', response)
        validate_task_goal(task_goal, case)

        supervisor_after = capture_supervisor_snapshot(
            base_env, 30.0, f'{case["id"]} post-task supervisor',
        )
        write_json(case_dir / 'post_task_supervisor.json', supervisor_after)
        metrics_delta = supervisor_metrics_delta(
            supervisor_before['payload'], supervisor_after['payload'], case['id'],
        )
        write_json(case_dir / 'supervisor_metrics_delta.json', metrics_delta)
        validation = validate_response(response, case, metrics_delta)

        post_controller = capture_post_task_controller_evidence(
            case,
            env=base_env,
            target_slot=str(validation['target_slot']),
            timeout_s=30.0,
            destination=case_dir / 'post_task_controller_evidence.json',
        )
        if post_controller.get('status') != 'passed':
            raise CampaignError(f'{case["id"]} post-task controller evidence failed')

        final_visual_echo = topic_echo(
            '/room_315/visual_state/observed_state',
            'mfja_rail_interfaces/msg/VisualStateObservation',
            env=base_env,
            timeout_s=30.0,
        )
        write_json(case_dir / 'final_visual_topic.json', final_visual_echo)
        if final_visual_echo.get('returncode') != 0:
            raise CampaignError(f'{case["id"]} mandatory final visual capture failed')
        final_visual_payload = parse_ros_yaml(final_visual_echo['stdout'])
        write_json(
            case_dir / 'final_visual_observation.json', final_visual_payload,
        )
        final_visual_check = validate_final_visual(
            final_visual_payload, case, expected_checkpoint_sha256,
        )
        write_json(case_dir / 'final_visual_validation.json', final_visual_check)

        bag_stop = stop_process(bag, timeout_s=20.0)
        bag_stopped = True
        teardown.append(bag_stop)
        write_json(case_dir / 'rosbag_stop.json', bag_stop)
        if bag_stop.get('returncode') != 0:
            raise CampaignError(
                f'{case["id"]} rosbag did not stop cleanly: {bag_stop}'
            )
        bag_validation = validate_rosbag(
            case_dir, case, response, env=base_env,
        )

        active_issues = snapshot_active_logs(
            processes + [bag],
            case_dir / 'active_runtime_logs',
        )
        payload_consensus = validate_payload_consensus_log(
            execution, case, case_dir / 'payload_consensus.json',
        )
        if active_issues:
            write_json(case_dir / 'active_runtime_log_issues.json', active_issues)
            raise CampaignError(
                f'{case["id"]} active runtime logs contain errors: {active_issues[:3]}'
            )
        validation['initial_controller_evidence_status'] = initial_controller['status']
        validation['post_task_controller_evidence_status'] = post_controller['status']
        validation['initial_visual_checkpoint_sha256'] = expected_checkpoint_sha256
        validation['final_visual_checkpoint_sha256'] = final_visual_check[
            'checkpoint_sha256'
        ]
        validation['rosbag'] = bag_validation
        validation['payload_consensus'] = payload_consensus
        write_json(case_dir / 'validation.json', validation)
        result_record.update(validation)
        result_record['status'] = 'passed'
    except Exception as exc:  # noqa: BLE001 - preserve a structured failed attempt
        result_record['failure_reason'] = str(exc)
    finally:
        managed = processes + ([bag] if bag is not None else [])
        try:
            active_issues = snapshot_active_logs(
                managed, case_dir / 'active_runtime_logs',
            )
            if active_issues:
                write_json(case_dir / 'active_runtime_log_issues.json', active_issues)
                if result_record.get('status') == 'passed':
                    result_record['status'] = 'failed'
                    result_record['failure_reason'] = (
                        f'active runtime logs contain errors: {active_issues[:3]}'
                    )
        except Exception as exc:  # noqa: BLE001 - evidence preservation must finish
            finalization_errors.append(f'active log snapshot failed: {exc}')
        try:
            stop_all = send_stop_all(base_env, f'campaign_case_{case["id"]}_complete')
            write_json(case_dir / 'stop_all.json', stop_all)
            if stop_all.get('returncode') != 0:
                finalization_errors.append(
                    'stop_all publication did not complete successfully'
                )
        except Exception as exc:  # noqa: BLE001 - continue teardown
            finalization_errors.append(f'stop_all failed: {exc}')
        if bag is not None and not bag_stopped:
            try:
                bag_stop = stop_process(bag, timeout_s=20.0)
                teardown.append(bag_stop)
                write_json(case_dir / 'rosbag_stop.json', bag_stop)
                if result_record.get('status') == 'passed' and bag_stop.get('returncode') != 0:
                    result_record['status'] = 'failed'
                    result_record['failure_reason'] = 'rosbag did not stop cleanly'
            except Exception as exc:  # noqa: BLE001 - continue teardown
                finalization_errors.append(f'rosbag teardown failed: {exc}')
        for process in reversed(processes):
            try:
                teardown.append(stop_process(process))
            except Exception as exc:  # noqa: BLE001 - preserve remaining logs
                finalization_errors.append(f'{process.name} teardown failed: {exc}')
        try:
            snapshot_full_logs(managed, case_dir / 'full_runtime_logs')
            ros_log_destination = case_dir / 'full_ros_logs'
            ros_log_destination.mkdir(parents=True, exist_ok=True)
            for path in sorted(temporary_root.iterdir()):
                if path.is_dir() and path.name.startswith('ros_logs'):
                    shutil.copytree(
                        path, ros_log_destination / path.name,
                        dirs_exist_ok=True,
                    )
        except Exception as exc:  # noqa: BLE001 - record evidence-copy failures
            finalization_errors.append(f'full log preservation failed: {exc}')
        if finalization_errors:
            write_json(case_dir / 'finalization_errors.json', finalization_errors)
            if result_record.get('status') == 'passed':
                result_record['status'] = 'failed'
                result_record['failure_reason'] = '; '.join(finalization_errors)
        write_json(case_dir / 'teardown.json', teardown)
        shutil.rmtree(temporary_root, ignore_errors=True)
        result_record['finished_at_utc'] = utc_now()
        result_record['duration_s'] = time.monotonic() - started
        if result_record.get('status') != 'passed':
            write_json(case_dir / 'failure.json', {
                'case_id': case['id'],
                'failed_at_utc': utc_now(),
                'reason': result_record.get('failure_reason') or 'unknown',
                'finalization_errors': finalization_errors,
            })
        write_json(case_dir / 'case_summary.json', result_record)
        time.sleep(2.0)
    if result_record['status'] != 'passed':
        raise CampaignError(
            f'{case["id"]} failed: {result_record.get("failure_reason", "unknown")}'
        )
    return result_record


def command_text(command: list[str]) -> str:
    return subprocess.list2cmdline(command)


def required_artifacts() -> list[Path]:
    return [
        VISUAL_RUN / 'best.pt',
        VISUAL_RUN / 'target_stats.json',
        VISUAL_RUN / 'visual_label_vectorizer.json',
        VISUAL_RUN / 'training_config.json',
        VISUAL_RUN / 'run_metadata.json',
        INTENT_MODEL,
        INTENT_CONFIG,
        REPO / 'mfja_robot_control_config/config/room_315_vla/visual_state_runtime.yaml',
        REPO / 'mfja_robot_control_config/config/room_315_vla/task_execution_runtime.yaml',
        REPO / 'mfja_robot_control_config/config/room_315_vla/pddl/domain_room315_runtime.pddl',
        REPO / 'mfja_robot_control_config/config/room_315_vla/shuttle_identity.yaml',
        REPO / 'mfja_robot_control_config/config/room_315_kinematics/rail_network_right.yaml',
        REPO / 'mfja_robot_control_config/config/room_315_kinematics/rail_network_left.yaml',
        REPO / 'mfja_robot_control_config/config/room_315_kinematics/rail_devices_right.yaml',
        REPO / 'mfja_robot_control_config/config/room_315_kinematics/rail_devices_left.yaml',
    ]


def verify_runtime_artifact_contract(payload: dict[str, Any]) -> dict[str, Any]:
    contract = payload.get('runtime_artifact_contract')
    if not isinstance(contract, dict):
        raise CampaignError('campaign matrix has no runtime artifact contract')
    declarations = contract.get('artifacts')
    if not isinstance(declarations, list):
        raise CampaignError('runtime artifact contract has no artifact list')
    exact_count = int(contract.get('exact_artifact_count') or 0)
    if (
        contract.get('hash_algorithm') != 'sha256'
        or contract.get('verify_before_first_case') is not True
        or len(declarations) != exact_count
    ):
        raise CampaignError('runtime artifact contract envelope is invalid')

    rows: list[dict[str, Any]] = []
    roles: set[str] = set()
    paths: set[Path] = set()
    for declaration in declarations:
        if not isinstance(declaration, dict):
            raise CampaignError('runtime artifact declaration is not an object')
        role = str(declaration.get('role') or '')
        path = Path(str(declaration.get('path') or '')).expanduser().resolve()
        if not role or role in roles or path in paths:
            raise CampaignError(
                f'duplicate or empty runtime artifact declaration: {role!r} {path}'
            )
        roles.add(role)
        paths.add(path)
        if not path.is_file():
            raise CampaignError(f'bound runtime artifact is missing: {path}')
        actual_size = path.stat().st_size
        actual_sha256 = sha256_file(path)
        expected_size = int(declaration.get('size_bytes') or -1)
        expected_sha256 = str(declaration.get('sha256') or '')
        if actual_size != expected_size or actual_sha256 != expected_sha256:
            raise CampaignError(
                f'bound runtime artifact changed: {role}: size '
                f'{actual_size}/{expected_size}, sha256 '
                f'{actual_sha256}/{expected_sha256}'
            )
        rows.append({
            'role': role,
            'path': str(path),
            'size_bytes': actual_size,
            'sha256': actual_sha256,
        })

    required_paths = {path.expanduser().resolve() for path in required_artifacts()}
    if paths != required_paths:
        raise CampaignError(
            'runtime artifact contract does not exactly cover the runner inputs: '
            f'missing={sorted(str(path) for path in required_paths - paths)}, '
            f'extra={sorted(str(path) for path in paths - required_paths)}'
        )
    return {
        'status': 'passed',
        'hash_algorithm': 'sha256',
        'artifact_count': len(rows),
        'artifacts': rows,
    }


def source_file_manifest() -> dict[str, Any]:
    roots = (
        REPO / 'mfja_3rd_floor_bringup',
        REPO / 'mfja_3rd_floor_description',
        REPO / 'mfja_robot_control_config',
        REPO / 'mfja_rail_interfaces',
    )
    rows: list[dict[str, Any]] = []
    for root in roots:
        for path in sorted(item for item in root.rglob('*') if item.is_file()):
            if '__pycache__' in path.parts or path.suffix in {'.pyc', '.log'}:
                continue
            rows.append({
                'path': str(path.relative_to(REPO)),
                'size': path.stat().st_size,
                'sha256': sha256_file(path),
            })
    canonical = json.dumps(rows, sort_keys=True, separators=(',', ':')).encode()
    return {
        'file_count': len(rows),
        'aggregate_sha256': sha256_bytes(canonical),
        'files': rows,
    }


def campaign_environment(
    matrix_path: Path,
    domain_id: int,
    output_root: Path,
) -> dict[str, Any]:
    env = clean_environment(domain_id)
    git_status = run_capture(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'],
        env=env, cwd=REPO, timeout_s=10.0, check=True,
    )['stdout']
    git_diff = run_capture(
        ['git', 'diff', '--binary', 'HEAD', '--', '.', ':(exclude)report', ':(exclude)report_ar'],
        env=env, cwd=REPO, timeout_s=30.0, check=True,
    )['stdout']
    write_text(output_root / 'source' / 'worktree.patch', git_diff)
    source_manifest = source_file_manifest()
    write_json(output_root / 'source' / 'source_files.json', source_manifest)
    artifacts = []
    for path in required_artifacts():
        if not path.is_file():
            raise CampaignError(f'required campaign artifact is missing: {path}')
        artifacts.append({
            'path': str(path),
            'size': path.stat().st_size,
            'sha256': sha256_file(path),
        })
    commands = {
        'ros2_doctor': run_capture(
            ['ros2', 'doctor', '--report'], env=env, timeout_s=30.0,
        ),
        'gazebo_version': run_capture(
            ['gz', 'sim', '--versions'], env=env, timeout_s=10.0,
        ),
        'gpu': run_capture(
            [
                'nvidia-smi', '--query-gpu=name,driver_version,memory.total',
                '--format=csv,noheader',
            ],
            env=env, timeout_s=10.0,
        ),
    }
    head = run_capture(
        ['git', 'rev-parse', 'HEAD'], env=env, cwd=REPO, timeout_s=10.0, check=True,
    )['stdout'].strip()
    branch = run_capture(
        ['git', 'branch', '--show-current'], env=env, cwd=REPO, timeout_s=10.0, check=True,
    )['stdout'].strip()
    return {
        'captured_at_utc': utc_now(),
        'campaign_matrix': {
            'path': str(matrix_path),
            'sha256': sha256_file(matrix_path),
        },
        'git': {
            'head': head,
            'branch': branch,
            'status_porcelain': git_status.splitlines(),
            'worktree_patch_sha256': sha256_bytes(git_diff.encode()),
            'runtime_source_tree_sha256': source_manifest['aggregate_sha256'],
            'runtime_source_file_count': source_manifest['file_count'],
        },
        'platform': {
            'hostname': platform.node(),
            'system': platform.system(),
            'release': platform.release(),
            'machine': platform.machine(),
            'python': platform.python_version(),
            'ros_distro': os.environ.get('ROS_DISTRO', ''),
            'ros_domain_id': domain_id,
            'ros_automatic_discovery_range': 'LOCALHOST',
        },
        'artifacts': artifacts,
        'diagnostic_commands': commands,
    }


def finalise_artifacts(output_root: Path, summary: dict[str, Any]) -> None:
    write_json(output_root / 'summary.json', summary)
    files = [
        path for path in output_root.rglob('*')
        if path.is_file() and path.name not in {'manifest.json', 'SHA256SUMS'}
    ]
    rows = [
        {
            'path': str(path.relative_to(output_root)),
            'size': path.stat().st_size,
            'sha256': sha256_file(path),
        }
        for path in sorted(files)
    ]
    manifest = {
        'schema_version': 1,
        'campaign_id': summary['campaign_id'],
        'created_at_utc': utc_now(),
        'campaign_status': summary['status'],
        'file_count_excluding_manifest': len(rows),
        'files': rows,
    }
    write_json(output_root / 'manifest.json', manifest)
    checksum_files = [
        path for path in output_root.rglob('*')
        if path.is_file() and path.name != 'SHA256SUMS'
    ]
    lines = [
        f'{sha256_file(path)}  {path.relative_to(output_root)}'
        for path in sorted(checksum_files)
    ]
    write_text(output_root / 'SHA256SUMS', '\n'.join(lines) + '\n')


def validate_matrix(payload: dict[str, Any]) -> None:
    if payload.get('schema_version') != 2:
        raise CampaignError('campaign matrix schema_version must be 2')
    if payload.get('campaign_id') != 'room315_integrated_campaign_v2':
        raise CampaignError('campaign matrix identifier is not the expected campaign')
    protocol = payload.get('protocol')
    if not isinstance(protocol, dict):
        raise CampaignError('campaign matrix has no protocol')
    cases = payload.get('cases')
    if not isinstance(cases, list) or not cases:
        raise CampaignError('campaign matrix has no cases')
    identifiers = [str(case.get('id') or '') for case in cases]
    if any(not value for value in identifiers) or len(set(identifiers)) != len(identifiers):
        raise CampaignError('campaign case identifiers must be non-empty and unique')
    if any(not re.fullmatch(r'[A-Z][0-9]{2}', value) for value in identifiers):
        raise CampaignError('campaign case identifiers must match [A-Z][0-9]{2}')
    if (
        int(protocol.get('predeclared_case_count') or 0) != len(cases)
        or protocol.get('predeclared_case_ids') != identifiers
        or protocol.get('campaign_scope') != 'positive_integrated_demonstrations_only'
        or protocol.get('independent_cold_start_per_case') is not True
        or protocol.get('operator_confirmation_required') is not True
        or protocol.get('reuse_existing_runtime_state') is not False
        or protocol.get('include_previous_campaign_data') is not False
        or protocol.get('fail_campaign_on_unexpected_case_outcome') is not True
    ):
        raise CampaignError('campaign protocol does not match the predeclared scope')
    utterances = [str(case.get('utterance') or '') for case in cases]
    if any(not value for value in utterances) or len(set(utterances)) != len(utterances):
        raise CampaignError('campaign utterances must be non-empty and unique')
    for case in cases:
        if case.get('side') not in {'left', 'right'}:
            raise CampaignError(f'{case.get("id")} has an invalid side')
        expected = case.get('expected')
        if not isinstance(expected, dict) or not expected.get('selected_identity'):
            raise CampaignError(f'{case.get("id")} has no expected selected identity')
        for side in ('left', 'right'):
            if side not in case.get('launch', {}):
                raise CampaignError(f'{case.get("id")} is missing {side} launch data')
            declaration = case['launch'][side]
            identities = [str(value).upper() for value in declaration.get('identities') or []]
            slots = [str(value) for value in declaration.get('start_slots') or []]
            loaded = [str(value).upper() for value in declaration.get('loaded') or []]
            if len(identities) != len(slots) or not set(loaded).issubset(identities):
                raise CampaignError(
                    f'{case.get("id")} has inconsistent {side} launch data'
                )
            if any(slot not in {'1', '2', '3', '4'} for slot in slots):
                raise CampaignError(f'{case.get("id")} has an invalid start slot')
            if any(identity_parts(identity)[1] != side for identity in identities):
                raise CampaignError(
                    f'{case.get("id")} assigns an identity to the wrong rail'
                )


def select_cases(payload: dict[str, Any], selected: list[str]) -> list[dict[str, Any]]:
    cases = list(payload['cases'])
    if not selected:
        return cases
    by_id = {case['id']: case for case in cases}
    unknown = [case_id for case_id in selected if case_id not in by_id]
    if unknown:
        raise CampaignError(f'unknown selected cases: {unknown}')
    return [by_id[case_id] for case_id in selected]


def build_summary(
    campaign_id: str,
    declared_case_count: int,
    selected_cases: list[dict[str, Any]],
    results: list[dict[str, Any]],
    started_at_utc: str,
    status: str,
    failure_reason: str = '',
) -> dict[str, Any]:
    passed = [row for row in results if row.get('status') == 'passed']
    total_steps = sum(int(row.get('executed_step_count') or 0) for row in passed)
    identities = sorted({str(row.get('selected_identity')) for row in passed})
    return {
        'schema_version': 1,
        'campaign_id': campaign_id,
        'status': status,
        'started_at_utc': started_at_utc,
        'finished_at_utc': utc_now(),
        'declared_case_count': declared_case_count,
        'selected_case_count': len(selected_cases),
        'full_declared_campaign': (
            len(selected_cases) == declared_case_count
            and status != 'partial'
        ),
        'planned_case_ids': [case['id'] for case in selected_cases],
        'completed_case_count': len(results),
        'passed_case_count': len(passed),
        'failed_case_count': len(results) - len(passed),
        'unique_utterance_count': len({case['utterance'] for case in selected_cases}),
        'right_rail_case_count': sum(case['side'] == 'right' for case in selected_cases),
        'left_rail_case_count': sum(case['side'] == 'left' for case in selected_cases),
        'semantic_families': sorted({case['family'] for case in selected_cases}),
        'selected_identities': identities,
        'executed_step_count': total_steps,
        'actuating_step_count': sum(
            int(row.get('actuating_step_count') or 0) for row in passed
        ),
        'non_actuating_step_count': sum(
            int(row.get('non_actuating_step_count') or 0) for row in passed
        ),
        'satisfied_postcondition_count': sum(
            int(row.get('satisfied_postcondition_count') or 0) for row in passed
        ),
        'accepted_supervisor_decision_count': sum(
            int(row.get('accepted_supervisor_decision_count') or 0) for row in passed
        ),
        'safe_abort_count': sum(bool(row.get('safe_abort_sent')) for row in results),
        'total_plan_attempts': sum(int(row.get('plan_attempts') or 0) for row in passed),
        'total_replans': sum(int(row.get('replans') or 0) for row in passed),
        'total_unknown_retries': sum(
            int(row.get('unknown_retries') or 0) for row in passed
        ),
        'failure_reason': failure_reason,
        'results': results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, default=DEFAULT_MATRIX)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=57)
    parser.add_argument('--case', action='append', default=[])
    args = parser.parse_args()

    matrix_path = args.matrix.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    if output_root.exists():
        raise CampaignError(f'refusing to reuse campaign output: {output_root}')
    if not INSTALL_SETUP.is_file():
        raise CampaignError(f'workspace install is missing: {INSTALL_SETUP}')
    matrix_bytes = matrix_path.read_bytes()
    payload = yaml.safe_load(matrix_bytes.decode('utf-8')) or {}
    validate_matrix(payload)
    artifact_contract_pre = verify_runtime_artifact_contract(payload)
    selected_cases = select_cases(payload, args.case)
    declared_case_ids = [str(case['id']) for case in payload['cases']]
    selected_case_ids = [str(case['id']) for case in selected_cases]
    if len(selected_case_ids) != len(set(selected_case_ids)):
        raise CampaignError('selected campaign case identifiers must be unique')
    is_full_campaign = selected_case_ids == declared_case_ids
    if args.domain_id < 0 or args.domain_id + len(selected_cases) - 1 > 232:
        raise CampaignError(
            'per-case ROS domain allocation must remain in the inclusive range 0..232'
        )
    case_domains = {
        case['id']: args.domain_id + index
        for index, case in enumerate(selected_cases)
    }
    output_root.mkdir(parents=True, exist_ok=False)
    campaign_matrix_copy = output_root / 'campaign_matrix.yaml'
    write_bytes(campaign_matrix_copy, matrix_bytes)
    shutil.copy2(Path(__file__).resolve(), output_root / 'campaign_runner.py')
    started_at_utc = utc_now()
    environment = campaign_environment(
        campaign_matrix_copy, args.domain_id, output_root,
    )
    environment['platform']['base_ros_domain_id'] = args.domain_id
    environment['platform']['case_ros_domain_ids'] = case_domains
    write_json(output_root / 'environment.json', environment)
    write_json(
        output_root / 'runtime_artifact_contract_pre.json',
        artifact_contract_pre,
    )
    write_json(output_root / 'launch_templates.json', {
        'floor_example': floor_command(
            selected_cases[0], float(payload['protocol']['shuttle_speed_mps'])
        ),
        'visual': visual_command(),
        'execution': execution_command(),
        'recorded_topics': list(RECORDED_TOPICS),
    })

    results: list[dict[str, Any]] = []
    status = 'passed' if is_full_campaign else 'partial'
    failure_reason = ''
    for index, case in enumerate(selected_cases, start=1):
        print(
            json.dumps({
                'event': 'case_started',
                'case_id': case['id'],
                'index': index,
                'total': len(selected_cases),
                'timestamp_utc': utc_now(),
            }),
            flush=True,
        )
        try:
            result = run_case(
                case,
                campaign_id=str(payload['campaign_id']),
                case_dir=output_root / 'cases' / case['id'],
                domain_id=case_domains[case['id']],
                protocol=payload['protocol'],
            )
            results.append(result)
            print(
                json.dumps({
                    'event': 'case_passed',
                    'case_id': case['id'],
                    'executed_steps': result['executed_step_count'],
                    'duration_s': result['duration_s'],
                }),
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001 - finalise a failed campaign
            status = 'failed'
            failure_reason = str(exc)
            case_summary_path = output_root / 'cases' / case['id'] / 'case_summary.json'
            if case_summary_path.is_file():
                results.append(json.loads(case_summary_path.read_text(encoding='utf-8')))
            print(
                json.dumps({
                    'event': 'campaign_failed',
                    'case_id': case['id'],
                    'reason': failure_reason,
                }),
                flush=True,
            )
            break

    try:
        artifact_contract_post = verify_runtime_artifact_contract(payload)
        source_post = source_file_manifest()
        environment_after = {
            'captured_at_utc': utc_now(),
            'runtime_artifact_contract': artifact_contract_post,
            'runtime_source_tree_sha256': source_post['aggregate_sha256'],
            'runtime_source_file_count': source_post['file_count'],
            'matches_pre_campaign_artifacts': (
                artifact_contract_post == artifact_contract_pre
            ),
            'matches_pre_campaign_runtime_source': (
                source_post['aggregate_sha256']
                == environment['git']['runtime_source_tree_sha256']
            ),
        }
        write_json(output_root / 'environment_after.json', environment_after)
        if (
            not environment_after['matches_pre_campaign_artifacts']
            or not environment_after['matches_pre_campaign_runtime_source']
        ):
            raise CampaignError(
                'runtime artifacts or source tree changed during the campaign'
            )
    except Exception as exc:  # noqa: BLE001 - bind post-run state to summary
        status = 'failed'
        suffix = f'post-campaign identity check failed: {exc}'
        failure_reason = f'{failure_reason}; {suffix}'.strip('; ')

    summary = build_summary(
        str(payload['campaign_id']),
        len(payload['cases']),
        selected_cases,
        results,
        started_at_utc,
        status,
        failure_reason,
    )
    finalise_artifacts(output_root, summary)
    print(json.dumps({'event': 'campaign_finished', **summary}), flush=True)
    return (
        0
        if status in {'passed', 'partial'} and len(results) == len(selected_cases)
        else 1
    )


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except CampaignError as exc:
        print(json.dumps({'event': 'fatal', 'reason': str(exc)}), file=sys.stderr)
        raise SystemExit(2)
