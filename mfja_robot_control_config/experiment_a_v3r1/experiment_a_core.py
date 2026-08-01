#!/usr/bin/env python3
"""Fail-closed contracts and deterministic sampling for Room 315 Experiment A."""

from __future__ import annotations

import hashlib
import json
import math
import os
import random
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Sequence


SEED = 31520260730
PYTHON_HASH_SEED = 1455489658
VISUAL_SCHEMA = "room315.visual_state.v3"
VECTOR_DIMENSION = 200
IDENTITIES = ("L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4")
GLOBAL_BLOCKS = (
    "A12E", "A12I", "A14", "A1E", "A1I", "A23", "A2E", "A2I",
    "A34E", "A34I", "A3E", "A3I", "A4E", "A4I",
)
CATEGORICAL_FIELDS = {
    "side": ("location.side", ("left", "right")),
    "block": ("location.block", GLOBAL_BLOCKS),
    "loaded_state": ("loaded_state", ("empty", "loaded")),
}
APPROVED_CHECKPOINT_SHA256 = (
    "8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d"
)
OLDER_PILOT_CHECKPOINT_SHA256 = (
    "61acabfeb75ca29e4612e51ccdcf233723d9b22c3600f396d9a5cf50c8487f73"
)
FORBIDDEN_LEGACY_HASHES = frozenset({
    "2fcf78c0034fe290c39b2816e12076300decf5f7818538357fae072231b9b502",
    "1dc97b0836f40c53810306e9a09874967fa7e1067cd5de315cba0e00570277e3",
})
FORBIDDEN_LEGACY_BASENAMES = frozenset({
    "test.jsonl",
    "test_visual_labels.jsonl",
})
LOSS_HEADS = ("segment_location", "loaded_state", "bbox", "s_m", "s_ratio")
CHECKPOINT_SELECTION = "v3r1_validation_total_weighted_loss_only"
CONFIGURED_DEFAULT_PROFILE = "configured_default"
LOCAL_DEFAULT_PROFILE = "default_batch32_amp"
LOCAL_FALLBACK_PROFILE = "fallback_batch16_accum2"


class ExperimentAError(RuntimeError):
    """Raised when an Experiment-A contract fails closed."""


def canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def sha256_file(path: Path) -> str:
    reject_forbidden_artifact(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    value = digest.hexdigest()
    reject_forbidden_artifact(path, digest=value)
    return value


def reject_forbidden_artifact(path: Path | str, *, digest: str | None = None) -> None:
    candidate = Path(path)
    if candidate.name.casefold() in FORBIDDEN_LEGACY_BASENAMES:
        raise ExperimentAError(f"legacy evaluation artifact is forbidden: {candidate.name}")
    if digest and digest.casefold() in FORBIDDEN_LEGACY_HASHES:
        raise ExperimentAError("artifact content matches a forbidden legacy evaluation hash")


def expand_path(value: str | Path) -> Path:
    expanded = Path(os.path.expandvars(str(value))).expanduser()
    reject_forbidden_artifact(expanded)
    return expanded.resolve()


def read_json(path: Path) -> dict[str, Any]:
    reject_forbidden_artifact(path)
    parsed = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(parsed, dict):
        raise ExperimentAError(f"expected JSON object: {path}")
    return parsed


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    reject_forbidden_artifact(path)
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ExperimentAError(f"invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(value, dict):
                raise ExperimentAError(f"expected JSON object at {path}:{line_number}")
            rows.append(value)
    return rows


def row_id(row: dict[str, Any]) -> str:
    value = str(row.get("sample_id") or row.get("episode_id") or "").strip()
    if not value:
        raise ExperimentAError("dataset row is missing sample_id and episode_id")
    return value


def categorical_output_name(identity: str, field: str, value: str) -> str:
    """Return the authoritative vector output name without relying on map order."""
    if identity not in IDENTITIES:
        raise ExperimentAError(f"unknown fixed identity: {identity}")
    if field not in CATEGORICAL_FIELDS:
        raise ExperimentAError(f"unknown categorical field: {field}")
    path, vocabulary = CATEGORICAL_FIELDS[field]
    if value not in vocabulary:
        raise ExperimentAError(
            f"unexpected categorical value for {identity}.{field}: {value}"
        )
    return f"shuttles.{IDENTITIES.index(identity)}.{path}=={value}"


def categorical_output_index(
    vectorizer: dict[str, Any], identity: str, field: str, value: str
) -> int:
    """Resolve one categorical target solely through vectorizer names."""
    name = categorical_output_name(identity, field, value)
    names = vectorizer.get("names")
    if not isinstance(names, list):
        raise ExperimentAError("vectorizer names are unavailable")
    matches = [index for index, candidate in enumerate(names) if candidate == name]
    if len(matches) != 1:
        raise ExperimentAError(
            f"required vectorizer name must exist exactly once: {name}"
        )
    return matches[0]


def decode_categorical_target(
    vector: Sequence[float],
    vectorizer: dict[str, Any],
    identity: str,
    field: str,
) -> str:
    """Decode an exactly one-hot categorical target; predictions use metric argmax."""
    _, vocabulary = CATEGORICAL_FIELDS.get(field, (None, ()))
    if not vocabulary:
        raise ExperimentAError(f"unknown categorical field: {field}")
    values = [
        float(vector[categorical_output_index(vectorizer, identity, field, value)])
        for value in vocabulary
    ]
    if any(value not in (0.0, 1.0) for value in values) or sum(values) != 1.0:
        raise ExperimentAError(
            f"categorical target is not exactly one-hot: {identity}.{field}"
        )
    return vocabulary[values.index(1.0)]


def source_balanced_epoch(
    old_ids: Sequence[str],
    new_ids: Sequence[str],
    *,
    seed: int,
    epoch: int,
    per_source: int | None = None,
) -> list[dict[str, Any]]:
    """Return a deterministic, interleaved 50/50 plan without copying rows."""
    if not old_ids or not new_ids:
        raise ExperimentAError("both old replay and V3R1 sources must be non-empty")
    count = int(per_source or max(len(old_ids), len(new_ids)))
    if count <= 0:
        raise ExperimentAError("per_source must be positive")

    def select(source: str, values: Sequence[str], salt: int) -> list[dict[str, Any]]:
        selected: list[dict[str, Any]] = []
        cycle = 0
        while len(selected) < count:
            indexes = list(range(len(values)))
            random.Random(int(seed) + int(epoch) * 1_000_003 + salt + cycle * 97_409).shuffle(indexes)
            for index in indexes:
                if len(selected) >= count:
                    break
                selected.append({
                    "source": source,
                    "source_index": index,
                    "sample_id": str(values[index]),
                    "resample_cycle": cycle,
                })
            cycle += 1
        return selected

    old = select("old_replay", old_ids, 11_003)
    new = select("v3r1_hard_case", new_ids, 29_011)
    combined = old + new
    random.Random(int(seed) + int(epoch) * 2_000_033 + 73_001).shuffle(combined)
    for sequence_index, item in enumerate(combined):
        item["epoch_sequence_index"] = sequence_index
    return combined


def sampling_report(plan: Sequence[dict[str, Any]]) -> dict[str, Any]:
    source_counts = Counter(str(item["source"]) for item in plan)
    unique_counts = Counter()
    max_multiplicity: dict[str, int] = {}
    for source in source_counts:
        counts = Counter(
            str(item["sample_id"]) for item in plan if item["source"] == source
        )
        unique_counts[source] = len(counts)
        max_multiplicity[source] = max(counts.values(), default=0)
    fingerprint = hashlib.sha256(
        canonical_json([
            (item["source"], item["source_index"], item["sample_id"], item["resample_cycle"])
            for item in plan
        ]).encode("utf-8")
    ).hexdigest()
    total = max(1, len(plan))
    return {
        "selected_rows": len(plan),
        "source_counts": dict(sorted(source_counts.items())),
        "source_fractions": {
            key: source_counts[key] / total for key in sorted(source_counts)
        },
        "unique_rows_by_source": dict(sorted(unique_counts.items())),
        "maximum_row_multiplicity_by_source": max_multiplicity,
        "selection_fingerprint_sha256": fingerprint,
    }


def validate_vectorizer(vectorizer: dict[str, Any]) -> None:
    if vectorizer.get("schema_version") != VISUAL_SCHEMA:
        raise ExperimentAError("approved vectorizer schema mismatch")
    if int(vectorizer.get("dim", -1)) != VECTOR_DIMENSION:
        raise ExperimentAError("approved vectorizer output dimension mismatch")
    if tuple(vectorizer.get("fixed_identity_order") or ()) != IDENTITIES:
        raise ExperimentAError("approved vectorizer identity order mismatch")
    names = vectorizer.get("names")
    if not isinstance(names, list) or len(names) != VECTOR_DIMENSION:
        raise ExperimentAError("approved vectorizer names are incompatible")
    numeric_keys = vectorizer.get("numeric_keys")
    categorical_values = vectorizer.get("categorical_values")
    if not isinstance(numeric_keys, list) or not isinstance(categorical_values, dict):
        raise ExperimentAError("approved vectorizer field definitions are incompatible")
    if names[: len(numeric_keys)] != numeric_keys:
        raise ExperimentAError("numeric vectorizer names do not match numeric keys")
    if len(set(names)) != len(names):
        raise ExperimentAError("vectorizer names contain duplicates")
    expected_categorical: list[str] = []
    for identity in IDENTITIES:
        slot = IDENTITIES.index(identity)
        for field in ("side", "block", "loaded_state"):
            path, vocabulary = CATEGORICAL_FIELDS[field]
            key = f"shuttles.{slot}.{path}"
            allowed = categorical_values.get(key)
            if not isinstance(allowed, list) or tuple(allowed) != tuple(vocabulary):
                raise ExperimentAError(
                    f"unexpected categorical vocabulary for {identity}.{field}"
                )
            expected_categorical.extend(
                categorical_output_name(identity, field, value)
                for value in vocabulary
            )
    if set(names[len(numeric_keys) :]) != set(expected_categorical):
        raise ExperimentAError("categorical vectorizer names do not match vocabulary")
    if len(names[len(numeric_keys) :]) != len(expected_categorical):
        raise ExperimentAError("categorical vectorizer output count is incompatible")


def validate_target_stats(stats: dict[str, Any]) -> None:
    mean = stats.get("mean")
    std = stats.get("std")
    if not isinstance(mean, list) or not isinstance(std, list):
        raise ExperimentAError("target statistics require mean and std arrays")
    if len(mean) != VECTOR_DIMENSION or len(std) != VECTOR_DIMENSION:
        raise ExperimentAError("target statistics dimension mismatch")
    if not all(math.isfinite(float(value)) for value in mean + std):
        raise ExperimentAError("target statistics contain non-finite values")
    if not all(float(value) > 0.0 for value in std):
        raise ExperimentAError("target standard deviations must be positive")


def _label_payload(value: dict[str, Any]) -> dict[str, Any]:
    label = value.get("visual_state_labels", value)
    if not isinstance(label, dict):
        raise ExperimentAError("visual label payload must be an object")
    return label


def validate_label(label_row: dict[str, Any]) -> dict[str, Any]:
    label = _label_payload(label_row)
    if label.get("schema_version") != VISUAL_SCHEMA:
        raise ExperimentAError("visual label schema mismatch")
    shuttles = label.get("shuttles")
    if not isinstance(shuttles, list) or len(shuttles) != len(IDENTITIES):
        raise ExperimentAError("visual label must contain eight fixed shuttle slots")
    if tuple(str(item.get("id") or "") for item in shuttles) != IDENTITIES:
        raise ExperimentAError("visual label fixed identity order mismatch")
    for shuttle in shuttles:
        visible = bool(shuttle.get("presence")) and bool(shuttle.get("visually_available"))
        if not visible:
            continue
        if shuttle.get("loaded_state") not in {"loaded", "empty"}:
            raise ExperimentAError("visible shuttle has invalid loaded_state")
        location = shuttle.get("location") or {}
        if location.get("side") not in {"left", "right"}:
            raise ExperimentAError("visible shuttle has invalid side")
        rail = shuttle.get("rail_position") or {}
        if not bool(rail.get("available")):
            raise ExperimentAError("visible shuttle has unavailable rail position")
    return label


def vectorize_label(
    label_row: dict[str, Any], vectorizer: dict[str, Any]
) -> tuple[list[float], list[float]]:
    """Apply the frozen approved vectorizer without fitting anything."""
    validate_vectorizer(vectorizer)
    label = validate_label(label_row)
    shuttles = label["shuttles"]
    numeric_keys = list(vectorizer["numeric_keys"])

    def value_for(key: str) -> Any:
        parts = key.split(".")
        shuttle = shuttles[int(parts[1])]
        if parts[2] == "bbox":
            return shuttle.get("bbox", [0.0] * 4)[int(parts[3])]
        if parts[2] == "rail_position":
            return (shuttle.get("rail_position") or {}).get(parts[3], 0.0)
        if parts[2] == "location":
            return (shuttle.get("location") or {}).get(parts[3], "")
        return shuttle.get(parts[2], "")

    vector = [0.0] * VECTOR_DIMENSION
    for index, key in enumerate(numeric_keys):
        vector[index] = float(value_for(key) or 0.0)
    for identity, shuttle in zip(IDENTITIES, shuttles):
        location = shuttle.get("location") or {}
        raw_values = {
            "side": str(location.get("side") or "").strip(),
            "block": str(location.get("block") or "").strip(),
            "loaded_state": str(shuttle.get("loaded_state") or "").strip(),
        }
        visible = bool(shuttle.get("presence")) and bool(
            shuttle.get("visually_available")
        )
        for field in ("side", "block", "loaded_state"):
            _, vocabulary = CATEGORICAL_FIELDS[field]
            value = raw_values[field]
            if value in vocabulary:
                index = categorical_output_index(
                    vectorizer, identity, field, value
                )
                vector[index] = 1.0
            elif visible:
                raise ExperimentAError(
                    f"visible target has malformed {identity}.{field}: {value}"
                )
    mask: list[float] = []
    for name in vectorizer["names"]:
        slot = int(str(name).split(".")[1])
        shuttle = shuttles[slot]
        mask.append(1.0 if shuttle.get("presence") and shuttle.get("visually_available") else 0.0)
    if len(vector) != VECTOR_DIMENSION or len(mask) != VECTOR_DIMENSION:
        raise ExperimentAError("vectorized target dimension mismatch")
    validate_categorical_target_semantics(label, vectorizer, vector, mask)
    return vector, mask


def validate_categorical_target_semantics(
    label: dict[str, Any],
    vectorizer: dict[str, Any],
    vector: Sequence[float],
    mask: Sequence[float],
) -> dict[str, int]:
    """Fail closed on malformed present targets and improperly masked absences."""
    present = 0
    absent = 0
    names = vectorizer["names"]
    for slot, (identity, shuttle) in enumerate(zip(IDENTITIES, label["shuttles"])):
        visible = bool(shuttle.get("presence")) and bool(
            shuttle.get("visually_available")
        )
        slot_indexes = [
            index
            for index, name in enumerate(names)
            if name.startswith(f"shuttles.{slot}.")
        ]
        if not visible:
            absent += 1
            if any(float(mask[index]) != 0.0 for index in slot_indexes):
                raise ExperimentAError(f"absent slot is not masked: {identity}")
            continue
        present += 1
        if any(float(mask[index]) != 1.0 for index in slot_indexes):
            raise ExperimentAError(f"present slot is not fully targeted: {identity}")
        location = shuttle["location"]
        expected = {
            "side": str(location["side"]),
            "block": str(location["block"]),
            "loaded_state": str(shuttle["loaded_state"]),
        }
        for field in ("side", "block", "loaded_state"):
            decoded = decode_categorical_target(
                vector, vectorizer, identity, field
            )
            if decoded != expected[field]:
                raise ExperimentAError(
                    f"categorical target semantics mismatch for {identity}.{field}: "
                    f"{decoded} != {expected[field]}"
                )
    return {"present_slots": present, "absent_slots": absent}


def loss_head_indexes(names: Sequence[str]) -> dict[str, list[int]]:
    result = {head: [] for head in LOSS_HEADS}
    for index, name in enumerate(names):
        if ".location." in name or name.endswith(".rail_position.segment_length_m"):
            head = "segment_location"
        elif ".loaded_state==" in name:
            head = "loaded_state"
        elif ".bbox." in name:
            head = "bbox"
        elif name.endswith(".rail_position.s_m"):
            head = "s_m"
        elif name.endswith(".rail_position.s_ratio"):
            head = "s_ratio"
        else:
            raise ExperimentAError(f"unassigned visual loss target: {name}")
        result[head].append(index)
    if any(not result[head] for head in LOSS_HEADS):
        raise ExperimentAError("one or more loss heads have no target indexes")
    return result


def strict_load_approved_checkpoint(
    *, torch_module: Any, model: Any, checkpoint_path: Path
) -> dict[str, Any]:
    digest = sha256_file(checkpoint_path)
    if digest != APPROVED_CHECKPOINT_SHA256:
        raise ExperimentAError("approved continuation checkpoint SHA-256 mismatch")
    checkpoint = torch_module.load(checkpoint_path, map_location="cpu", weights_only=False)
    if int(checkpoint.get("epoch", -1)) != 14:
        raise ExperimentAError("approved continuation checkpoint epoch is not 14")
    state = checkpoint.get("model_state_dict")
    if not isinstance(state, dict):
        raise ExperimentAError("approved checkpoint lacks model_state_dict")
    load_result = model.load_state_dict(state, strict=True)
    missing = list(getattr(load_result, "missing_keys", ()))
    unexpected = list(getattr(load_result, "unexpected_keys", ()))
    if missing or unexpected:
        raise ExperimentAError(
            f"strict checkpoint load reported missing={missing}, unexpected={unexpected}"
        )
    head_keys = sorted(key for key in state if key.startswith("head."))
    if not head_keys:
        raise ExperimentAError("approved checkpoint contains no prediction-head parameters")
    loaded_state = model.state_dict()
    mismatched_parameters = [
        key
        for key in sorted(state)
        if key not in loaded_state
        or not torch_module.equal(loaded_state[key].detach().cpu(), state[key].detach().cpu())
    ]
    if mismatched_parameters:
        raise ExperimentAError(
            "checkpoint parameters were not loaded exactly: "
            f"{mismatched_parameters[:8]}"
        )
    mismatched_head = [
        key
        for key in head_keys
        if key not in loaded_state
        or not torch_module.equal(loaded_state[key].detach().cpu(), state[key].detach().cpu())
    ]
    if mismatched_head:
        raise ExperimentAError(
            f"prediction-head parameters were not loaded exactly: {mismatched_head}"
        )
    checkpoint["_strict_load_verification"] = {
        "strict": True,
        "missing_keys": missing,
        "unexpected_keys": unexpected,
        "all_parameter_key_count": len(state),
        "all_parameter_tensors_equal_checkpoint": True,
        "prediction_head_key_count": len(head_keys),
        "prediction_head_reinitialized": False,
        "all_prediction_head_tensors_equal_checkpoint": True,
    }
    return checkpoint


def ensure_training_roles(config: dict[str, Any]) -> None:
    roles = config.get("data_roles") or {}
    corrected_canary = (
        config.get("stage") == "canary"
        and "corrected" in str(config.get("schema_version") or "")
    )
    expected_selection = (
        "none_canary_evaluation" if corrected_canary else "validation_only"
    )
    if roles.get("checkpoint_selection") != expected_selection:
        raise ExperimentAError(
            f"checkpoint selection role must be {expected_selection}"
        )
    if roles.get("canary") != "post_training_development_regression_only":
        raise ExperimentAError("Canary role is invalid")
    expected_training = (
        () if corrected_canary else ("old_replay", "v3r1_hard_case")
    )
    training = tuple(roles.get("training_sources") or ())
    if training != expected_training:
        raise ExperimentAError("training source roles are invalid")
    serialized = canonical_json(config).casefold()
    if '"test.jsonl"' in serialized or '"test_visual_labels.jsonl"' in serialized:
        raise ExperimentAError("configuration references forbidden legacy evaluation data")


def effective_training_config(
    config: dict[str, Any], execution_profile: str = CONFIGURED_DEFAULT_PROFILE
) -> dict[str, Any]:
    """Resolve an explicitly requested runtime profile without automatic fallback."""
    training = dict(config.get("training") or {})
    profile = str(execution_profile or CONFIGURED_DEFAULT_PROFILE)
    if profile == CONFIGURED_DEFAULT_PROFILE:
        return training
    profiles = config.get("execution_profiles") or {}
    if profile not in profiles:
        raise ExperimentAError(f"unknown execution profile: {profile}")
    selected = dict(profiles[profile])
    if selected.get("automatic_selection"):
        raise ExperimentAError("execution profiles may not be selected automatically")
    training.update({
        key: value
        for key, value in selected.items()
        if key not in {"name", "automatic_selection", "explicit_cli_flag"}
    })
    return training


def assert_disjoint(*named_rows: tuple[str, Iterable[dict[str, Any]]]) -> dict[str, Any]:
    ids: dict[str, set[str]] = {
        name: {row_id(row) for row in rows} for name, rows in named_rows
    }
    overlaps: dict[str, int] = {}
    keys = list(ids)
    for i, left in enumerate(keys):
        for right in keys[i + 1:]:
            overlaps[f"{left}__{right}"] = len(ids[left] & ids[right])
    if any(overlaps.values()):
        raise ExperimentAError(f"dataset source overlap detected: {overlaps}")
    return overlaps
