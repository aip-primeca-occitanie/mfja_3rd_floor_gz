#!/usr/bin/env python3
"""Install and verify the local Room 315 intent GGUF checkpoint."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path


MODEL_REPO = 'Qwen/Qwen2.5-1.5B-Instruct-GGUF'
MODEL_FILENAME = 'qwen2.5-1.5b-instruct-q4_k_m.gguf'
MODEL_URL = f'https://huggingface.co/{MODEL_REPO}/resolve/main/{MODEL_FILENAME}'
MODEL_SHA256 = '6a1a2eb6d15622bf3c96857206351ba97e1af16c30d7a74ee38970e434e9407e'
DEFAULT_MODEL_DIR = Path('/home/tiago/models/room315_intent')
LLAMA_CPP_PACKAGE = 'llama-cpp-python==0.3.16'


def main() -> int:
    parser = argparse.ArgumentParser(description='Set up the offline Room 315 semantic intent model.')
    parser.add_argument('--model-dir', default=str(DEFAULT_MODEL_DIR), help='Directory outside Git for model weights.')
    parser.add_argument('--skip-dependency-install', action='store_true', help='Do not install llama-cpp-python.')
    parser.add_argument('--no-download', action='store_true', help='Fail if the checkpoint is missing instead of downloading.')
    parser.add_argument('--force-download', action='store_true', help='Download even when a checkpoint file already exists.')
    parser.add_argument('--config-output', default='', help='Local YAML output path; defaults under --model-dir.')
    parser.add_argument('--env-output', default='', help='Shell env output path; defaults under --model-dir.')
    args = parser.parse_args()

    model_dir = Path(args.model_dir).expanduser()
    model_dir.mkdir(parents=True, exist_ok=True)
    model_path = model_dir / MODEL_FILENAME
    config_output = Path(args.config_output).expanduser() if args.config_output else model_dir / 'task_goal_understanding.local.yaml'
    env_output = Path(args.env_output).expanduser() if args.env_output else model_dir / 'room315_intent.env'

    if not args.skip_dependency_install:
        ensure_llama_cpp()

    if args.force_download and model_path.exists():
        model_path.unlink()
    if not model_path.exists():
        if args.no_download:
            print(f'missing checkpoint: {model_path}', file=sys.stderr)
            print(f'next command: {Path(__file__).name}', file=sys.stderr)
            return 2
        download_model(model_path)

    actual = sha256_file(model_path)
    if actual != MODEL_SHA256:
        print(
            f'checksum mismatch for {model_path}: expected {MODEL_SHA256}, got {actual}',
            file=sys.stderr,
        )
        return 3

    write_local_config(config_output, model_path)
    write_env_file(env_output, model_path, config_output)

    print('Room 315 intent model is ready.')
    print(f'model_path={model_path}')
    print(f'sha256={actual}')
    print(f'local_config={config_output}')
    print(f'env_file={env_output}')
    print()
    print('Use these commands:')
    print(f'  source {env_output}')
    print(
        '  PYTHONPATH=mfja_robot_control_config/scripts '
        'python3 mfja_robot_control_config/scripts/room_315_task_goal_semantic_smoke.py '
        '--require-real-model --expect-semantic '
        '--text "Could you send whichever carrier is closest and holding a component '
        'to the third position on the right-hand line?"'
    )
    return 0


def ensure_llama_cpp() -> None:
    if importlib.util.find_spec('llama_cpp') is not None:
        print('llama_cpp already importable.')
        return
    print(f'Installing {LLAMA_CPP_PACKAGE} for the current user...')
    command = [sys.executable, '-m', 'pip', 'install', '--user', LLAMA_CPP_PACKAGE]
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode == 0:
        print(result.stdout, end='')
        return
    if 'externally-managed-environment' not in (result.stderr + result.stdout):
        print(result.stdout, end='')
        print(result.stderr, end='', file=sys.stderr)
        result.check_returncode()
    print('PEP 668 blocked --user pip install; retrying with --break-system-packages for user site.')
    retry = command + ['--break-system-packages']
    subprocess.run(retry, check=True)


def download_model(model_path: Path) -> None:
    print(f'Downloading {MODEL_URL}')
    print(f'Output: {model_path}')
    model_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('wb', dir=str(model_path.parent), delete=False) as tmp:
        tmp_path = Path(tmp.name)
        try:
            with urllib.request.urlopen(MODEL_URL, timeout=60) as response:
                total = int(response.headers.get('Content-Length') or 0)
                downloaded = 0
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    tmp.write(chunk)
                    downloaded += len(chunk)
                    if total:
                        percent = downloaded * 100.0 / total
                        print(f'\r{downloaded}/{total} bytes ({percent:.1f}%)', end='', flush=True)
                if total:
                    print()
            tmp.flush()
            os.fsync(tmp.fileno())
        except Exception:
            tmp_path.unlink(missing_ok=True)
            raise
    tmp_path.replace(model_path)


def sha256_file(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def write_local_config(path: Path, model_path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '\n'.join([
            'schema_version: 1',
            'prompt_schema_version: 1',
            '',
            'local_semantic_model:',
            '  enabled: true',
            '  backend: llama_cpp',
            f'  model_path: "{model_path}"',
            f'  model_sha256: "{MODEL_SHA256}"',
            '  device: cpu',
            '  quantization: q4_k_m',
            '  context_size: 4096',
            '  n_threads: 0',
            '  n_gpu_layers: 0',
            '  chat_format: chatml',
            '',
            'generation:',
            '  temperature: 0.0',
            '  top_p: 1.0',
            '  max_output_tokens: 192',
            '  seed: 315',
            '',
            'runtime:',
            '  timeout_s: 45.0',
            '  retry_count: 1',
            '  offline_only: true',
            '  shadow_mode: false',
            '  deterministic_only: false',
            '  require_real_model_for_smoke: true',
            '',
        ]),
        encoding='utf-8',
    )


def write_env_file(path: Path, model_path: Path, config_path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '\n'.join([
            f'export ROOM315_INTENT_MODEL_PATH="{model_path}"',
            f'export ROOM315_TASK_GOAL_LOCAL_CONFIG="{config_path}"',
            'export HF_HUB_OFFLINE=1',
            'export TRANSFORMERS_OFFLINE=1',
            '',
        ]),
        encoding='utf-8',
    )


if __name__ == '__main__':
    raise SystemExit(main())
