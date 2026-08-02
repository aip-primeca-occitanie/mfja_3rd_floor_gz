#!/usr/bin/env python3
"""Local semantic-model runtime and strict envelope schema for Room 315 goals."""

from __future__ import annotations

import concurrent.futures
import copy
import hashlib
import json
import os
import time
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from pathlib import Path
from typing import Any

import yaml

from room_315_contracts import CONTRACT_SCHEMA_VERSION
from room_315_task_goal_schema import MODEL_DRAFT_FIELDS
from room_315_task_goal_schema import TASK_GOAL_DRAFT_CONTRACT_TYPE
from room_315_task_goal_schema import GoalIssue
from room_315_task_goal_schema import TaskGoalDraft
from room_315_task_goal_schema import blocked_paths


SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE = 'SemanticParseEnvelope'
INTENT_MODEL_PATH_ENV = 'ROOM315_INTENT_MODEL_PATH'
DIALOGUE_ACTS = (
    'new_goal',
    'answer',
    'correction',
    'confirm',
    'reject',
    'cancel',
    'restart',
    'help',
)
FIELD_PROVENANCE = (
    'explicit_text',
    'explicit_correction',
    'confirmed_context',
    'semantic_inference',
    'structured_form',
    'user_edited',
)
DRAFT_PATCH_FIELDS = tuple(
    field for field in MODEL_DRAFT_FIELDS
    if field not in {'schema_version', 'contract_type', 'confidence'}
) + ('confidence',)
ENVELOPE_FIELDS = frozenset({
    'schema_version',
    'contract_type',
    'dialogue_act',
    'draft_patch',
    'evidence',
    'provenance',
    'alternatives',
    'confidence',
})
SEMANTIC_RESPONSE_FORMAT: dict[str, Any] = {
    'type': 'json_object',
    'schema': {
        'type': 'object',
        'additionalProperties': False,
        'properties': {
            'schema_version': {'type': 'integer', 'enum': [CONTRACT_SCHEMA_VERSION]},
            'contract_type': {'type': 'string', 'enum': [SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE]},
            'dialogue_act': {'type': 'string', 'enum': list(DIALOGUE_ACTS)},
            'draft_patch': {
                'type': ['object', 'null'],
                'additionalProperties': False,
                'properties': {
                    'goal_type': {'type': ['string', 'null'], 'enum': ['transport', 'inspection', None]},
                    'selection_strategy': {'type': ['string', 'null'], 'enum': ['nearest', 'explicit', 'any', None]},
                    'payload_filter': {'type': ['string', 'null'], 'enum': ['loaded', 'empty', 'any', None]},
                    'side': {'type': ['string', 'null'], 'enum': ['right', 'left', None]},
                    'target_kind': {
                        'type': ['string', 'null'],
                        'enum': [
                            'station',
                            'slot',
                            'shuttle',
                            'shuttle_selection',
                            'rail',
                            'system',
                            None,
                        ],
                    },
                    'target_station': {'type': ['string', 'null'], 'enum': ['yaskawa', 'staubli', 'kuka', None]},
                    'target_slot': {'type': ['string', 'null'], 'enum': ['1', '2', '3', '4', None]},
                    'target_shuttle': {
                        'type': ['string', 'null'],
                        'enum': [
                            'R1', 'R2', 'R3', 'R4', 'L1', 'L2', 'L3', 'L4',
                            'room315_right_shuttle_1', 'room315_right_shuttle_2',
                            'room315_right_shuttle_3', 'room315_right_shuttle_4',
                            'room315_left_shuttle_1', 'room315_left_shuttle_2',
                            'room315_left_shuttle_3', 'room315_left_shuttle_4',
                            None,
                        ],
                    },
                    'inspection_subject': {'type': ['string', 'null']},
                    'confidence': {'type': ['number', 'null'], 'minimum': 0.0, 'maximum': 1.0},
                },
            },
            'evidence': {'type': 'object', 'additionalProperties': {'type': 'string'}},
            'provenance': {
                'type': 'object',
                'additionalProperties': {'type': 'string', 'enum': list(FIELD_PROVENANCE)},
            },
            'alternatives': {'type': 'array', 'items': {'type': 'object'}},
            'confidence': {
                'type': 'object',
                'additionalProperties': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
            },
        },
        'required': ['contract_type', 'dialogue_act', 'draft_patch'],
    },
}


@dataclass(frozen=True)
class SemanticParseEnvelope:
    """Strict local-model output before deterministic fusion."""

    dialogue_act: str = 'new_goal'
    draft_patch: dict[str, Any] | None = None
    evidence: dict[str, str] = dataclass_field(default_factory=dict)
    provenance: dict[str, str] = dataclass_field(default_factory=dict)
    alternatives: tuple[dict[str, Any], ...] = ()
    confidence: dict[str, float] = dataclass_field(default_factory=dict)
    schema_version: int = CONTRACT_SCHEMA_VERSION
    contract_type: str = SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE

    def __post_init__(self) -> None:
        if self.schema_version != CONTRACT_SCHEMA_VERSION:
            raise ValueError(f'SemanticParseEnvelope schema_version must be {CONTRACT_SCHEMA_VERSION}')
        if self.contract_type != SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE:
            raise ValueError(f'contract_type must be {SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE!r}')
        if self.dialogue_act not in DIALOGUE_ACTS:
            raise ValueError(f'dialogue_act must be one of {DIALOGUE_ACTS}')
        patch = self.draft_patch
        if patch is not None:
            _validate_draft_patch(patch)
        for key, value in self.evidence.items():
            if key not in DRAFT_PATCH_FIELDS:
                raise ValueError(f'evidence field {key!r} is not a TaskGoalDraft field')
            if not isinstance(value, str):
                raise ValueError('evidence values must be strings')
        for key, value in self.provenance.items():
            if key not in DRAFT_PATCH_FIELDS:
                raise ValueError(f'provenance field {key!r} is not a TaskGoalDraft field')
            if value not in FIELD_PROVENANCE:
                raise ValueError(f'provenance values must be one of {FIELD_PROVENANCE}')
        for key, value in self.confidence.items():
            if key not in DRAFT_PATCH_FIELDS and key != 'overall':
                raise ValueError(f'confidence field {key!r} is not supported')
            number = float(value)
            if number < 0.0 or number > 1.0:
                raise ValueError('confidence values must be in [0, 1]')

    def to_dict(self) -> dict[str, Any]:
        return {
            'schema_version': self.schema_version,
            'contract_type': self.contract_type,
            'dialogue_act': self.dialogue_act,
            'draft_patch': copy.deepcopy(self.draft_patch),
            'evidence': copy.deepcopy(self.evidence),
            'provenance': copy.deepcopy(self.provenance),
            'alternatives': [copy.deepcopy(item) for item in self.alternatives],
            'confidence': copy.deepcopy(self.confidence),
        }


@dataclass(frozen=True)
class SemanticEnvelopeResult:
    status: str
    envelope: SemanticParseEnvelope | None = None
    issues: tuple[GoalIssue, ...] = ()
    raw_output: Any = None

    @property
    def ok(self) -> bool:
        return self.status == 'ok' and self.envelope is not None

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'ok': self.ok,
            'envelope': self.envelope.to_dict() if self.envelope else None,
            'issues': [issue.to_dict() for issue in self.issues],
            'raw_output': copy.deepcopy(self.raw_output),
        }


def strict_semantic_envelope_from_json(model_output: str | bytes) -> SemanticEnvelopeResult:
    try:
        payload = json.loads(model_output)
    except json.JSONDecodeError as exc:
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue(
                'invalid_semantic_json',
                f'Model output must be strict SemanticParseEnvelope JSON: {exc.msg}',
                'model_output',
                details={'line': exc.lineno, 'column': exc.colno},
            ),),
        )
    if not isinstance(payload, dict):
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue('invalid_semantic_json_type', 'Semantic envelope must be a JSON object.', 'model_output'),),
            raw_output=payload,
        )
    blocked = blocked_paths(payload)
    if blocked:
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue(
                'forbidden_semantic_field',
                'Semantic model may output envelope fields and TaskGoalDraft patch fields only.',
                blocked[0],
                details={'blocked_paths': blocked[:10]},
            ),),
            raw_output=payload,
        )
    unknown = sorted(set(payload) - ENVELOPE_FIELDS)
    if unknown:
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue(
                'unknown_semantic_field',
                'Semantic envelope contains fields outside the strict schema.',
                unknown[0],
                details={'unknown_fields': unknown},
            ),),
            raw_output=payload,
        )
    if payload.get('contract_type') not in (None, SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE):
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue(
                'unsupported_semantic_contract_type',
                'Semantic envelope contract_type must be SemanticParseEnvelope.',
                'contract_type',
            ),),
            raw_output=payload,
        )
    if 'schema_version' in payload and payload.get('schema_version') != CONTRACT_SCHEMA_VERSION:
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue(
                'unsupported_semantic_schema_version',
                f'Semantic envelope schema_version must be {CONTRACT_SCHEMA_VERSION}.',
                'schema_version',
            ),),
            raw_output=payload,
        )
    try:
        envelope = SemanticParseEnvelope(
            schema_version=int(payload.get('schema_version', CONTRACT_SCHEMA_VERSION)),
            contract_type=str(payload.get('contract_type') or SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE),
            dialogue_act=str(payload.get('dialogue_act') or 'new_goal'),
            draft_patch=copy.deepcopy(payload.get('draft_patch')),
            evidence=copy.deepcopy(payload.get('evidence') or {}),
            provenance=copy.deepcopy(payload.get('provenance') or {}),
            alternatives=tuple(copy.deepcopy(payload.get('alternatives') or ())),
            confidence=copy.deepcopy(payload.get('confidence') or {}),
        )
    except (TypeError, ValueError) as exc:
        return SemanticEnvelopeResult(
            status='error',
            issues=(GoalIssue('invalid_semantic_envelope', str(exc), 'model_output'),),
            raw_output=payload,
        )
    return SemanticEnvelopeResult(status='ok', envelope=envelope, raw_output=payload)


@dataclass(frozen=True)
class LocalSemanticModelConfig:
    schema_version: int = CONTRACT_SCHEMA_VERSION
    enabled: bool = True
    backend: str = 'transformers'
    model_path: str = ''
    model_sha256: str = ''
    device: str = 'cpu'
    quantization: str = 'none'
    context_size: int = 2048
    n_threads: int = 0
    n_gpu_layers: int = 0
    chat_format: str = ''
    timeout_s: float = 4.0
    retry_count: int = 1
    max_output_tokens: int = 256
    temperature: float = 0.0
    top_p: float = 1.0
    seed: int = 315
    prompt_schema_version: int = 1
    shadow_mode: bool = False
    deterministic_only: bool = False
    require_real_model_for_smoke: bool = False
    offline_only: bool = True

    @classmethod
    def from_file(cls, path: str | os.PathLike[str] | None = None) -> 'LocalSemanticModelConfig':
        config_path = Path(path) if path else default_config_path()
        if not config_path.exists():
            return cls(enabled=False, model_path='', backend='transformers')
        payload = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
        if not isinstance(payload, dict):
            raise ValueError('task-goal semantic config must be a mapping')
        if int(payload.get('schema_version', CONTRACT_SCHEMA_VERSION)) != CONTRACT_SCHEMA_VERSION:
            raise ValueError(f'task-goal semantic config schema_version must be {CONTRACT_SCHEMA_VERSION}')
        model = payload.get('local_semantic_model') or {}
        generation = payload.get('generation') or {}
        runtime = payload.get('runtime') or {}
        if not isinstance(model, dict) or not isinstance(generation, dict) or not isinstance(runtime, dict):
            raise ValueError('local_semantic_model, generation, and runtime must be mappings')
        configured_model_path = _model_path_from_config(model.get('model_path') or '')
        return cls(
            schema_version=CONTRACT_SCHEMA_VERSION,
            enabled=bool(model.get('enabled', True)),
            backend=str(model.get('backend', 'transformers')),
            model_path=configured_model_path,
            model_sha256=str(model.get('model_sha256') or ''),
            device=str(model.get('device', 'cpu')),
            quantization=str(model.get('quantization', 'none')),
            context_size=int(model.get('context_size', 2048)),
            n_threads=int(model.get('n_threads', 0)),
            n_gpu_layers=int(model.get('n_gpu_layers', 0)),
            chat_format=str(model.get('chat_format') or ''),
            timeout_s=float(runtime.get('timeout_s', 4.0)),
            retry_count=int(runtime.get('retry_count', 1)),
            max_output_tokens=int(generation.get('max_output_tokens', 256)),
            temperature=float(generation.get('temperature', 0.0)),
            top_p=float(generation.get('top_p', 1.0)),
            seed=int(generation.get('seed', 315)),
            prompt_schema_version=int(payload.get('prompt_schema_version', 1)),
            shadow_mode=bool(runtime.get('shadow_mode', False)),
            deterministic_only=bool(runtime.get('deterministic_only', False)),
            require_real_model_for_smoke=bool(runtime.get('require_real_model_for_smoke', False)),
            offline_only=bool(runtime.get('offline_only', True)),
        )

    def validate_local_model_path(self) -> Path:
        if not self.model_path:
            raise ValueError('local semantic model_path is not configured')
        if '://' in self.model_path:
            raise ValueError('local semantic model_path must be a filesystem path, not a URI')
        path = Path(os.path.expandvars(self.model_path)).expanduser()
        if not path.is_absolute():
            raise ValueError('local semantic model_path must be absolute')
        if not path.exists():
            raise ValueError(f'local semantic model_path does not exist: {path}')
        return path

    def validate_expected_sha256(self, path: Path) -> str:
        fingerprint = fingerprint_model_path(path)
        if self.model_sha256:
            expected = self.model_sha256.lower().removeprefix('sha256:')
            actual = fingerprint.removeprefix('sha256:')
            if actual != expected:
                raise ValueError(
                    f'local semantic model checksum mismatch: expected sha256:{expected}, got {fingerprint}'
                )
        return fingerprint


@dataclass(frozen=True)
class BackendHealth:
    ready: bool
    state: str
    backend: str
    model_path: str = ''
    model_fingerprint: str = ''
    message: str = ''

    def to_dict(self) -> dict[str, Any]:
        return {
            'ready': self.ready,
            'state': self.state,
            'backend': self.backend,
            'model_path': self.model_path,
            'model_fingerprint': self.model_fingerprint,
            'message': self.message,
        }


@dataclass(frozen=True)
class SemanticBackendResult:
    status: str
    text: str = ''
    latency_s: float = 0.0
    model_fingerprint: str = ''
    model_path: str = ''
    model_ready: bool = False
    backend: str = ''
    fallback_reason: str = ''
    attempts: int = 0

    @property
    def ok(self) -> bool:
        return self.status == 'ok'

    def to_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'text': self.text,
            'latency_s': self.latency_s,
            'model_fingerprint': self.model_fingerprint,
            'model_path': self.model_path,
            'model_ready': self.model_ready,
            'backend': self.backend,
            'fallback_reason': self.fallback_reason,
            'attempts': self.attempts,
        }


class LocalSemanticBackend:
    backend_name = 'base'

    def load(self) -> None:
        raise NotImplementedError

    def infer(self, user_text: str, *, confirmed_context: dict[str, Any] | None = None) -> SemanticBackendResult:
        raise NotImplementedError

    def health(self) -> BackendHealth:
        raise NotImplementedError


class TransformersSemanticBackend(LocalSemanticBackend):
    """Concrete local-only Hugging Face backend.

    The backend imports transformers lazily, uses local_files_only=True, and
    refuses non-local model paths. It never downloads model weights.
    """

    backend_name = 'transformers'

    def __init__(self, config: LocalSemanticModelConfig) -> None:
        self.config = config
        self._tokenizer = None
        self._model = None
        self._loaded = False
        self._load_error = ''
        self._fingerprint = ''

    def load(self) -> None:
        if self._loaded:
            return
        if not self.config.enabled:
            self._load_error = 'local semantic model is disabled'
            return
        try:
            model_path = self.config.validate_local_model_path()
            self._fingerprint = self.config.validate_expected_sha256(model_path)
            from transformers import AutoModelForCausalLM  # type: ignore
            from transformers import AutoTokenizer  # type: ignore

            self._tokenizer = AutoTokenizer.from_pretrained(
                str(model_path),
                local_files_only=True,
                trust_remote_code=False,
            )
            self._model = AutoModelForCausalLM.from_pretrained(
                str(model_path),
                local_files_only=True,
                trust_remote_code=False,
            )
            if self.config.device and self.config.device != 'auto':
                self._model.to(self.config.device)
            self._model.eval()
            self._warm_up()
            self._loaded = True
            self._load_error = ''
        except Exception as exc:  # pragma: no cover - depends on optional runtime
            self._loaded = False
            self._load_error = str(exc)

    def infer(self, user_text: str, *, confirmed_context: dict[str, Any] | None = None) -> SemanticBackendResult:
        started = time.monotonic()
        self.load()
        if not self._loaded or self._tokenizer is None or self._model is None:
            return SemanticBackendResult(
                status='unavailable',
                latency_s=time.monotonic() - started,
                backend=self.backend_name,
                model_fingerprint=self._fingerprint,
                model_path=self.config.model_path,
                model_ready=False,
                fallback_reason=self._load_error or 'model_not_loaded',
                attempts=0,
            )
        prompt = build_semantic_prompt(
            user_text,
            confirmed_context=confirmed_context,
            prompt_schema_version=self.config.prompt_schema_version,
        )
        attempts = max(1, int(self.config.retry_count) + 1)
        last_reason = ''
        for attempt in range(1, attempts + 1):
            try:
                text = _run_with_timeout(lambda: self._generate_once(prompt), timeout_s=self.config.timeout_s)
                return SemanticBackendResult(
                    status='ok',
                    text=text,
                    latency_s=time.monotonic() - started,
                    backend=self.backend_name,
                    model_fingerprint=self._fingerprint,
                    model_path=self.config.model_path,
                    model_ready=True,
                    attempts=attempt,
                )
            except concurrent.futures.TimeoutError:
                last_reason = 'timeout'
            except Exception as exc:  # pragma: no cover - optional runtime
                last_reason = str(exc)
        return SemanticBackendResult(
            status='error',
            latency_s=time.monotonic() - started,
            backend=self.backend_name,
            model_fingerprint=self._fingerprint,
            model_path=self.config.model_path,
            model_ready=True,
            fallback_reason=last_reason or 'inference_failed',
            attempts=attempts,
        )

    def health(self) -> BackendHealth:
        self.load()
        if self._loaded:
            return BackendHealth(
                ready=True,
                state='ready',
                backend=self.backend_name,
                model_path=self.config.model_path,
                model_fingerprint=self._fingerprint,
            )
        return BackendHealth(
            ready=False,
            state='unavailable',
            backend=self.backend_name,
            model_path=self.config.model_path,
            model_fingerprint=self._fingerprint,
            message=self._load_error or 'model_not_loaded',
        )

    def _warm_up(self) -> None:
        prompt = build_semantic_prompt('help', confirmed_context=None, prompt_schema_version=self.config.prompt_schema_version)
        _ = self._generate_once(prompt, max_new_tokens=1)

    def _generate_once(self, prompt: str, *, max_new_tokens: int | None = None) -> str:
        tokenizer = self._tokenizer
        model = self._model
        tokens = tokenizer(prompt, return_tensors='pt', truncation=True, max_length=self.config.context_size)
        if self.config.device and self.config.device not in {'auto', 'cpu'}:
            tokens = {key: value.to(self.config.device) for key, value in tokens.items()}
        output = model.generate(
            **tokens,
            max_new_tokens=max_new_tokens or self.config.max_output_tokens,
            do_sample=False,
            temperature=0.0,
            top_p=self.config.top_p,
            pad_token_id=getattr(tokenizer, 'eos_token_id', None),
        )
        decoded = tokenizer.decode(output[0], skip_special_tokens=True)
        return extract_first_json_object(decoded[len(prompt):] or decoded)


class FakeSemanticBackend(LocalSemanticBackend):
    """Deterministic backend for unit tests."""

    backend_name = 'fake_semantic'

    def __init__(
        self,
        responses: dict[str, str] | None = None,
        *,
        unhealthy: bool = False,
        timeout: bool = False,
        fingerprint: str = 'fake-semantic-fingerprint',
    ) -> None:
        self.responses = responses or {}
        self.unhealthy = unhealthy
        self.timeout = timeout
        self.fingerprint = fingerprint
        self.calls: list[str] = []
        self.confirmed_contexts: list[dict[str, Any] | None] = []

    def load(self) -> None:
        return

    def infer(self, user_text: str, *, confirmed_context: dict[str, Any] | None = None) -> SemanticBackendResult:
        self.calls.append(user_text)
        self.confirmed_contexts.append(copy.deepcopy(confirmed_context))
        if self.unhealthy:
            return SemanticBackendResult(
                status='unavailable',
                backend=self.backend_name,
                model_fingerprint=self.fingerprint,
                model_ready=False,
                fallback_reason='fake_unhealthy',
            )
        if self.timeout:
            return SemanticBackendResult(
                status='error',
                backend=self.backend_name,
                model_fingerprint=self.fingerprint,
                model_ready=True,
                fallback_reason='timeout',
                attempts=2,
            )
        text = self.responses.get(user_text, self.responses.get('*', '{}'))
        return SemanticBackendResult(
            status='ok',
            text=text,
            backend=self.backend_name,
            model_fingerprint=self.fingerprint,
            model_ready=True,
            attempts=1,
        )

    def health(self) -> BackendHealth:
        return BackendHealth(
            ready=not self.unhealthy,
            state='ready' if not self.unhealthy else 'unavailable',
            backend=self.backend_name,
            model_fingerprint=self.fingerprint,
            message='fake_unhealthy' if self.unhealthy else '',
        )


class LlamaCppSemanticBackend(LocalSemanticBackend):
    """CPU-first llama.cpp backend for a local GGUF intent model.

    The backend imports llama-cpp-python lazily, accepts only filesystem GGUF
    paths, verifies an optional SHA-256, and never downloads weights at runtime.
    """

    backend_name = 'llama_cpp'

    def __init__(self, config: LocalSemanticModelConfig) -> None:
        self.config = config
        self._llm = None
        self._loaded = False
        self._load_error = ''
        self._fingerprint = ''
        self._resolved_model_path = ''

    def load(self) -> None:
        if self._loaded:
            return
        if not self.config.enabled:
            self._load_error = 'local semantic model is disabled'
            return
        if not self.config.offline_only:
            self._load_error = 'llama.cpp backend requires offline_only=true'
            return
        try:
            model_path = self.config.validate_local_model_path()
            if not model_path.is_file() or model_path.suffix.lower() != '.gguf':
                raise ValueError('llama.cpp backend requires an absolute local .gguf model file')
            self._fingerprint = self.config.validate_expected_sha256(model_path)
            from llama_cpp import Llama  # type: ignore

            kwargs: dict[str, Any] = {
                'model_path': str(model_path),
                'n_ctx': self.config.context_size,
                'n_gpu_layers': self.config.n_gpu_layers,
                'seed': self.config.seed,
                'verbose': False,
            }
            if self.config.n_threads > 0:
                kwargs['n_threads'] = self.config.n_threads
            if self.config.chat_format:
                kwargs['chat_format'] = self.config.chat_format
            self._llm = Llama(**kwargs)
            self._warm_up()
            self._resolved_model_path = str(model_path)
            self._loaded = True
            self._load_error = ''
        except Exception as exc:  # pragma: no cover - depends on optional runtime
            self._loaded = False
            self._load_error = str(exc)

    def infer(self, user_text: str, *, confirmed_context: dict[str, Any] | None = None) -> SemanticBackendResult:
        started = time.monotonic()
        self.load()
        if not self._loaded or self._llm is None:
            return SemanticBackendResult(
                status='unavailable',
                latency_s=time.monotonic() - started,
                backend=self.backend_name,
                model_fingerprint=self._fingerprint,
                model_path=self.config.model_path,
                model_ready=False,
                fallback_reason=self._load_error or 'model_not_loaded',
                attempts=0,
            )
        prompt = build_semantic_prompt(
            user_text,
            confirmed_context=confirmed_context,
            prompt_schema_version=self.config.prompt_schema_version,
        )
        attempts = max(1, int(self.config.retry_count) + 1)
        last_reason = ''
        for attempt in range(1, attempts + 1):
            try:
                text = _run_with_timeout(lambda: self._generate_once(prompt), timeout_s=self.config.timeout_s)
                return SemanticBackendResult(
                    status='ok',
                    text=text,
                    latency_s=time.monotonic() - started,
                    backend=self.backend_name,
                    model_fingerprint=self._fingerprint,
                    model_path=self._resolved_model_path or self.config.model_path,
                    model_ready=True,
                    attempts=attempt,
                )
            except concurrent.futures.TimeoutError:
                last_reason = 'timeout'
            except Exception as exc:  # pragma: no cover - optional runtime
                last_reason = str(exc)
        return SemanticBackendResult(
            status='error',
            latency_s=time.monotonic() - started,
            backend=self.backend_name,
            model_fingerprint=self._fingerprint,
            model_path=self._resolved_model_path or self.config.model_path,
            model_ready=True,
            fallback_reason=last_reason or 'inference_failed',
            attempts=attempts,
        )

    def health(self) -> BackendHealth:
        self.load()
        if self._loaded:
            return BackendHealth(
                ready=True,
                state='ready',
                backend=self.backend_name,
                model_path=self._resolved_model_path or self.config.model_path,
                model_fingerprint=self._fingerprint,
            )
        return BackendHealth(
            ready=False,
            state='unavailable',
            backend=self.backend_name,
            model_path=self.config.model_path,
            model_fingerprint=self._fingerprint,
            message=self._load_error or 'model_not_loaded',
        )

    def _warm_up(self) -> None:
        prompt = build_semantic_prompt('help', confirmed_context=None, prompt_schema_version=self.config.prompt_schema_version)
        _ = self._generate_once(prompt, max_new_tokens=1)

    def _generate_once(self, prompt: str, *, max_new_tokens: int | None = None) -> str:
        llm = self._llm
        if llm is None:
            raise RuntimeError('llama.cpp model is not loaded')
        max_tokens = max_new_tokens or self.config.max_output_tokens
        if hasattr(llm, 'create_chat_completion'):
            response = llm.create_chat_completion(
                messages=[
                    {
                        'role': 'system',
                        'content': (
                            'You are an offline English-only semantic parser for Room 315. '
                            'Return only a strict JSON object. Never return PDDL, plans, actions, '
                            'primitive commands, device commands, safety constraints, or prose.'
                        ),
                    },
                    {'role': 'user', 'content': prompt},
                ],
                max_tokens=max_tokens,
                temperature=0.0,
                top_p=self.config.top_p,
                response_format=SEMANTIC_RESPONSE_FORMAT,
            )
            content = response['choices'][0]['message']['content']
        else:  # pragma: no cover - compatibility with older llama-cpp-python
            response = llm(
                prompt,
                max_tokens=max_tokens,
                temperature=0.0,
                top_p=self.config.top_p,
                echo=False,
            )
            content = response['choices'][0]['text']
        return extract_first_json_object(str(content))


_DEFAULT_BACKEND: LocalSemanticBackend | None = None
_DEFAULT_BACKEND_KEY: tuple[str, float | None] | None = None


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / 'config' / 'room_315_vla' / 'task_goal_understanding.yaml'


def build_backend_from_config(config: LocalSemanticModelConfig) -> LocalSemanticBackend:
    if config.backend == 'transformers':
        return TransformersSemanticBackend(config)
    if config.backend in {'llama_cpp', 'llama-cpp'}:
        return LlamaCppSemanticBackend(config)
    raise ValueError(f'Unsupported local semantic backend {config.backend!r}')


def get_default_semantic_backend(config_path: str | os.PathLike[str] | None = None) -> LocalSemanticBackend:
    global _DEFAULT_BACKEND, _DEFAULT_BACKEND_KEY
    path = Path(config_path) if config_path else default_config_path()
    mtime = path.stat().st_mtime if path.exists() else None
    key = (str(path), mtime)
    if _DEFAULT_BACKEND is not None and _DEFAULT_BACKEND_KEY == key:
        return _DEFAULT_BACKEND
    config = LocalSemanticModelConfig.from_file(path)
    _DEFAULT_BACKEND = build_backend_from_config(config)
    _DEFAULT_BACKEND_KEY = key
    return _DEFAULT_BACKEND


def build_semantic_prompt(
    user_text: str,
    *,
    confirmed_context: dict[str, Any] | None,
    prompt_schema_version: int,
) -> str:
    context = json.dumps(confirmed_context or {}, sort_keys=True, separators=(',', ':'))
    return (
        'Return only one strict JSON object with contract_type SemanticParseEnvelope.\n'
        'Allowed top-level keys: schema_version, contract_type, dialogue_act, '
        'draft_patch, evidence, provenance, alternatives, confidence.\n'
        'Allowed draft_patch keys: goal_type, selection_strategy, payload_filter, side, '
        'target_kind, target_station, target_slot, target_shuttle, inspection_subject, confidence.\n'
        'Use only these exact enum values. goal_type: transport, inspection. '
        'selection_strategy: nearest, explicit, any. payload_filter: loaded, empty, any. '
        'side: right, left. target_kind: station, slot, shuttle, shuttle_selection, rail, system. '
        'target_slot: "1", "2", "3", or "4". target_station: yaskawa, staubli, kuka. '
        'Never invent enum values such as whichever, carrier, holding, component, line, or position.\n'
        'target_shuttle may only be R1, R2, R3, R4, L1, L2, L3, L4, or a canonical '
        'room315_right_shuttle_N / room315_left_shuttle_N identifier. Do not copy descriptive '
        'phrases into target_shuttle. If selection_strategy is nearest or any, omit target_shuttle. '
        'Never choose a numbered shuttle for a nearest/closest request.\n'
        'Never output PDDL, plans, actions, primitive commands, device commands, rail commands, '
        'editable safety constraints, markdown, or explanatory prose.\n'
        'Use null or omit fields when ambiguous or missing; never guess.\n'
        'Vocabulary: closest/nearest/whichever carrier is closest -> selection_strategy nearest; '
        'holding/carrying a component/load/part -> payload_filter loaded; without a component -> empty; '
        'right-hand line -> side right; left-hand line -> side left; third position -> target_kind slot, target_slot 3.\n'
        'For an inspection of Room 315 as a whole, use target_kind system and '
        'omit side, selection_strategy, payload_filter, and target_shuttle.\n'
        'For inspection of an unnamed shuttle, use target_kind shuttle_selection. '
        'Use selection_strategy any with an optional payload_filter. Nearest '
        'shuttle inspection is not supported without an explicit reference, so '
        'leave selection_strategy null and let validation request a safer goal.\n'
        'Set target_station only when the current user_text explicitly names yaskawa, '
        'staubli, or kuka. Never infer a station from a slot, rail side, or confirmed_context.\n'
        'For "whichever carrier is closest and holding a component", output '
        '"selection_strategy":"nearest" and "payload_filter":"loaded"; do not set target_shuttle.\n'
        'Correct indirect-nearest output example: '
        '{"contract_type":"SemanticParseEnvelope","dialogue_act":"new_goal",'
        '"draft_patch":{"goal_type":"transport","selection_strategy":"nearest",'
        '"payload_filter":"loaded","side":"right","target_kind":"slot","target_slot":"3"},'
        '"provenance":{"goal_type":"semantic_inference","selection_strategy":"semantic_inference",'
        '"payload_filter":"semantic_inference","side":"semantic_inference","target_kind":"semantic_inference",'
        '"target_slot":"semantic_inference"},"confidence":{"overall":0.8}}\n'
        'Example output for "send the nearest loaded right carrier to slot 3": '
        '{"contract_type":"SemanticParseEnvelope","dialogue_act":"new_goal",'
        '"draft_patch":{"goal_type":"transport","selection_strategy":"nearest",'
        '"payload_filter":"loaded","side":"right","target_kind":"slot","target_slot":"3"},'
        '"provenance":{"goal_type":"semantic_inference","selection_strategy":"semantic_inference",'
        '"payload_filter":"semantic_inference","side":"semantic_inference","target_kind":"semantic_inference",'
        '"target_slot":"semantic_inference"},"confidence":{"overall":0.8}}\n'
        f'prompt_schema_version={prompt_schema_version}\n'
        f'confirmed_context={context}\n'
        f'user_text={json.dumps(user_text)}\n'
        'json='
    )


def extract_first_json_object(text: str) -> str:
    start = text.find('{')
    if start < 0:
        return text.strip()
    depth = 0
    in_string = False
    escape = False
    for index, char in enumerate(text[start:], start=start):
        if in_string:
            if escape:
                escape = False
            elif char == '\\':
                escape = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == '{':
            depth += 1
        elif char == '}':
            depth -= 1
            if depth == 0:
                return text[start:index + 1].strip()
    return text[start:].strip()


def fingerprint_model_path(path: Path) -> str:
    path = Path(path)
    hasher = hashlib.sha256()
    if path.is_file():
        _hash_file(path, hasher)
    else:
        for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
            hasher.update(str(candidate.relative_to(path)).encode('utf-8'))
            _hash_file(candidate, hasher)
    return f'sha256:{hasher.hexdigest()}'


def _hash_file(path: Path, hasher: 'hashlib._Hash') -> None:
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)


def _model_path_from_config(raw_model_path: Any) -> str:
    override = os.environ.get(INTENT_MODEL_PATH_ENV)
    if override is not None:
        return override
    expanded = os.path.expandvars(str(raw_model_path or '')).strip()
    if '$' in expanded:
        return ''
    return expanded


def _run_with_timeout(func: Any, *, timeout_s: float) -> Any:
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    future = executor.submit(func)
    try:
        return future.result(timeout=timeout_s)
    except Exception:
        future.cancel()
        raise
    finally:
        executor.shutdown(wait=False, cancel_futures=True)


def _validate_draft_patch(patch: dict[str, Any]) -> None:
    if not isinstance(patch, dict):
        raise ValueError('draft_patch must be null or an object')
    unknown = sorted(set(patch) - set(DRAFT_PATCH_FIELDS))
    if unknown:
        raise ValueError(f'unknown draft_patch fields: {unknown}')
    blocked = blocked_paths(patch)
    if blocked:
        raise ValueError(f'forbidden draft_patch fields: {blocked[:10]}')
    TaskGoalDraft.from_dict({
        **patch,
        'source': 'semantic_model',
        'raw': {'semantic_draft_patch': copy.deepcopy(patch)},
    }, strict=False)


__all__ = [
    'BackendHealth',
    'DIALOGUE_ACTS',
    'FIELD_PROVENANCE',
    'FakeSemanticBackend',
    'INTENT_MODEL_PATH_ENV',
    'LlamaCppSemanticBackend',
    'LocalSemanticBackend',
    'LocalSemanticModelConfig',
    'SEMANTIC_PARSE_ENVELOPE_CONTRACT_TYPE',
    'SemanticBackendResult',
    'SemanticEnvelopeResult',
    'SemanticParseEnvelope',
    'TransformersSemanticBackend',
    'build_backend_from_config',
    'build_semantic_prompt',
    'default_config_path',
    'extract_first_json_object',
    'fingerprint_model_path',
    'get_default_semantic_backend',
    'strict_semantic_envelope_from_json',
]
