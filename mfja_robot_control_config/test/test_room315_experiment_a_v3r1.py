#!/usr/bin/env python3
"""Focused tests for the data-only Experiment-A continuation package."""

from __future__ import annotations

import importlib.util
import inspect
import json
import os
import sys
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]
MODULE_ROOT = REPO / "mfja_robot_control_config" / "experiment_a_v3r1"
SCRIPT_ROOT = REPO / "mfja_robot_control_config" / "scripts"
sys.path.insert(0, str(MODULE_ROOT))
sys.path.insert(0, str(SCRIPT_ROOT))

import experiment_a_core as core  # noqa: E402
import experiment_a_guard as experiment_guard  # noqa: E402
import experiment_a_smoke_v2 as smoke_v2  # noqa: E402
import experiment_a_train as experiment_train  # noqa: E402
import experiment_a_verify as experiment_verify  # noqa: E402


def load_builder():
    path = SCRIPT_ROOT / "room_315_experiment_a_v3r1_package.py"
    spec = importlib.util.spec_from_file_location("experiment_a_builder", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_corrected_builder():
    path = SCRIPT_ROOT / "room_315_experiment_a_v3r1_corrected_package.py"
    spec = importlib.util.spec_from_file_location("corrected_experiment_a_builder", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_authoritative_checkpoint_and_incorrect_checkpoint_rejection(monkeypatch, tmp_path):
    assert core.APPROVED_CHECKPOINT_SHA256 == "8a2d865e3d3551ec4284b53aa913d66f24640e23556f2f26b49a165f3ce8d51d"
    assert core.OLDER_PILOT_CHECKPOINT_SHA256 != core.APPROVED_CHECKPOINT_SHA256
    checkpoint = tmp_path / "best.pt"
    checkpoint.write_bytes(b"not approved")
    with pytest.raises(core.ExperimentAError, match="SHA-256 mismatch"):
        core.strict_load_approved_checkpoint(
            torch_module=object(), model=object(), checkpoint_path=checkpoint
        )


def test_strict_checkpoint_loading(monkeypatch, tmp_path):
    checkpoint_path = tmp_path / "best.pt"
    checkpoint_path.write_bytes(b"fixture")
    monkeypatch.setattr(core, "sha256_file", lambda _: core.APPROVED_CHECKPOINT_SHA256)

    class Tensor:
        def __init__(self, value): self.value = value
        def detach(self): return self
        def cpu(self): return self

    class Torch:
        @staticmethod
        def load(path, map_location, weights_only):
            return {"epoch": 14, "model_state_dict": {"head.weight": Tensor(1)}}
        @staticmethod
        def equal(left, right):
            return left.value == right.value

    class Model:
        def __init__(self): self.strict = None; self.state = {}
        def load_state_dict(self, state, strict):
            self.strict = strict
            self.state = dict(state)
            return type("LoadResult", (), {"missing_keys": [], "unexpected_keys": []})()
        def state_dict(self): return self.state

    model = Model()
    report = core.strict_load_approved_checkpoint(
        torch_module=Torch, model=model, checkpoint_path=checkpoint_path
    )["_strict_load_verification"]
    assert model.strict is True
    assert report["all_parameter_key_count"] == 1
    assert report["all_parameter_tensors_equal_checkpoint"] is True
    assert report["prediction_head_key_count"] == 1
    assert report["all_prediction_head_tensors_equal_checkpoint"] is True
    assert report["prediction_head_reinitialized"] is False


def test_source_balanced_sampler_ratio_determinism_order_and_traceability():
    old = [f"old-{index}" for index in range(7)]
    new = [f"new-{index}" for index in range(13)]
    first = core.source_balanced_epoch(old, new, seed=core.SEED, epoch=1)
    repeated = core.source_balanced_epoch(old, new, seed=core.SEED, epoch=1)
    second_epoch = core.source_balanced_epoch(old, new, seed=core.SEED, epoch=2)
    assert first == repeated
    assert first != second_epoch
    report = core.sampling_report(first)
    assert report["source_counts"] == {"old_replay": 13, "v3r1_hard_case": 13}
    assert report["source_fractions"] == {"old_replay": 0.5, "v3r1_hard_case": 0.5}
    assert all({"source", "source_index", "sample_id", "resample_cycle"} <= set(item) for item in first)
    assert report["maximum_row_multiplicity_by_source"]["old_replay"] <= 2


def test_training_roles_exclude_validation_and_canary_from_training_and_selection():
    config = {
        "data_roles": {
            "training_sources": ["old_replay", "v3r1_hard_case"],
            "checkpoint_selection": "validation_only",
            "canary": "post_training_development_regression_only",
        }
    }
    core.ensure_training_roles(config)
    assert "validation" not in config["data_roles"]["training_sources"]
    assert "canary" not in config["data_roles"]["training_sources"]
    assert config["data_roles"]["checkpoint_selection"] == "validation_only"


def test_legacy_evaluation_denylist_and_forbidden_hash_rejection(tmp_path):
    with pytest.raises(core.ExperimentAError, match="forbidden"):
        core.reject_forbidden_artifact(tmp_path / "test.jsonl")
    with pytest.raises(core.ExperimentAError, match="forbidden"):
        core.reject_forbidden_artifact(
            tmp_path / "innocent.bin", digest=next(iter(core.FORBIDDEN_LEGACY_HASHES))
        )


def test_vectorizer_and_target_stats_compatibility_and_output_dimension():
    run = Path("/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run")
    vectorizer = json.loads((run / "visual_label_vectorizer.json").read_text())
    stats = json.loads((run / "target_stats.json").read_text())
    core.validate_vectorizer(vectorizer)
    core.validate_target_stats(stats)
    assert vectorizer["dim"] == core.VECTOR_DIMENSION == 200
    assert tuple(vectorizer["fixed_identity_order"]) == core.IDENTITIES


def test_categorical_vectorization_uses_declared_name_order_not_json_key_order():
    run = Path("/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run")
    vectorizer = json.loads((run / "visual_label_vectorizer.json").read_text())
    shuttles = []
    for identity in core.IDENTITIES:
        shuttles.append({
            "id": identity,
            "presence": False,
            "visually_available": False,
            "loaded_state": "unknown",
            "bbox": [0.0, 0.0, 0.0, 0.0],
            "location": {
                "side": "left" if identity.startswith("L") else "right",
                "block": "unknown",
            },
            "rail_position": {
                "available": False,
                "s_m": 0.0,
                "s_ratio": 0.0,
                "segment_length_m": 0.0,
            },
        })
    shuttles[0].update({
        "presence": True,
        "visually_available": True,
        "loaded_state": "loaded",
        "bbox": [1.0, 2.0, 3.0, 4.0],
        "location": {"side": "left", "block": "A1E"},
        "rail_position": {
            "available": True,
            "s_m": 0.5,
            "s_ratio": 0.25,
            "segment_length_m": 2.0,
        },
    })
    vector, mask = core.vectorize_label({
        "visual_state_labels": {
            "schema_version": core.VISUAL_SCHEMA,
            "shuttles": shuttles,
        }
    }, vectorizer)
    names = vectorizer["names"]
    assert [vector[names.index(f"shuttles.0.location.side=={value}")] for value in ("left", "right")] == [1.0, 0.0]
    assert [vector[names.index(f"shuttles.0.loaded_state=={value}")] for value in ("empty", "loaded")] == [0.0, 1.0]
    assert vector[names.index("shuttles.0.location.block==A1E")] == 1.0
    assert all(mask[index] == 1.0 for index, name in enumerate(names) if name.startswith("shuttles.0."))
    assert all(mask[index] == 0.0 for index, name in enumerate(names) if name.startswith("shuttles.1."))


def test_shared_categorical_name_index_contract_for_all_identities():
    run = Path("/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run")
    vectorizer = json.loads((run / "visual_label_vectorizer.json").read_text())
    for identity in core.IDENTITIES:
        for field, (_, vocabulary) in core.CATEGORICAL_FIELDS.items():
            indexes = []
            for value in vocabulary:
                name = core.categorical_output_name(identity, field, value)
                index = core.categorical_output_index(vectorizer, identity, field, value)
                assert vectorizer["names"][index] == name
                indexes.append(index)
            assert len(indexes) == len(set(indexes))
    with pytest.raises(core.ExperimentAError, match="unexpected categorical value"):
        core.categorical_output_name("L1", "block", "NOT_A_BLOCK")
    duplicate = dict(vectorizer)
    duplicate["names"] = list(vectorizer["names"])
    duplicate["names"][-1] = duplicate["names"][-2]
    with pytest.raises(core.ExperimentAError, match="duplicates"):
        core.validate_vectorizer(duplicate)


def test_corrected_semantic_self_test_rejects_historical_order_bug():
    run = Path("/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run")
    vectorizer = json.loads((run / "visual_label_vectorizer.json").read_text())
    report = experiment_verify.semantic_self_test(vectorizer)
    assert report["passed"] is True
    assert report["cases_checked"] == 32
    assert report["identities_checked"] == list(core.IDENTITIES)
    assert report["historical_broken_encoder_rejected_cases"] == 16
    assert report["historical_broken_encoder_total_cases"] == 32
    assert {
        tuple(sorted(case["expected"].items())) for case in report["cases"]
    } == {
        tuple(sorted({"side": "right", "block": "A34E", "loaded_state": "loaded"}.items())),
        tuple(sorted({"side": "left", "block": "A1E", "loaded_state": "empty"}.items())),
        tuple(sorted({"side": "right", "block": "A23", "loaded_state": "empty"}.items())),
        tuple(sorted({"side": "left", "block": "A12I", "loaded_state": "loaded"}.items())),
    }


def test_decode_fails_for_all_zero_and_multi_hot_present_targets():
    run = Path("/home/tiago/room315_full_training_approved_archive_seed31520260730/results/run")
    vectorizer = json.loads((run / "visual_label_vectorizer.json").read_text())
    vector = [0.0] * core.VECTOR_DIMENSION
    with pytest.raises(core.ExperimentAError, match="not exactly one-hot"):
        core.decode_categorical_target(vector, vectorizer, "L1", "side")
    for value in ("left", "right"):
        vector[core.categorical_output_index(vectorizer, "L1", "side", value)] = 1.0
    with pytest.raises(core.ExperimentAError, match="not exactly one-hot"):
        core.decode_categorical_target(vector, vectorizer, "L1", "side")


def test_smoke_subset_constructor_covers_hard_cases():
    builder = load_builder()
    rows = [
        {"sample_id": "l4", "traceability_metadata": {"active_identities": ["L4"], "loaded_identities": ["L4"]}},
        {"sample_id": "r4", "traceability_metadata": {"active_identities": ["R4"], "loaded_identities": ["R4"]}},
        {"sample_id": "triple", "traceability_metadata": {"active_identities": ["L2", "L4", "R4"], "loaded_identities": []}},
        {"sample_id": "offset", "traceability_metadata": {"active_identities": ["R4"], "loaded_identities": [], "operational_target_name": "right_slot_3", "target_offset": 0.02}},
    ] + [{"sample_id": f"fill-{i}", "traceability_metadata": {}} for i in range(20)]
    required = ("L4_loaded", "R4_loaded", "exact_L2_L4_R4", "right_slot3_deliberate_offset", "hard_payload")
    selected, coverage = builder.choose_smoke(rows, 12, required=required, seed=core.SEED)
    assert len(selected) == len(set(selected)) == 12
    assert all(coverage[item] > 0 for item in required)


def test_sidecar_hash_constants():
    builder = load_builder()
    assert builder.HASHES["target_stats"] == "2d48078641842aa2db7a59b9285fc5bbedaaa3a0039fc39986ca230db983b18c"
    assert builder.HASHES["vectorizer"] == "637c854556f3331c4e187db4aa7fc70457f01df8877947b9a0e988a543f7113e"


def test_package_manifest_reproducibility(tmp_path):
    builder = load_builder()
    root = tmp_path / "package"
    (root / "nested").mkdir(parents=True)
    (root / "a.txt").write_text("a\n")
    (root / "nested" / "b.txt").write_text("b\n")
    first = builder.manifest(root)
    second = builder.manifest(root)
    assert first == second
    (root / "nested" / "b.txt").write_text("changed\n")
    assert builder.manifest(root)["tree_sha256"] != first["tree_sha256"]


def test_kairos_launchers_still_require_gh200():
    builder = load_builder()
    assert "--require-gh200" in builder.inside_container()
    for stage in ("smoke", "full", "canary"):
        launcher = builder.launcher(stage)
        assert "apptainer exec --nv" in launcher
        assert "experiment_a_inside_container.py" in launcher


def test_local_launchers_do_not_claim_gh200_or_require_container():
    builder = load_builder()
    for stage in ("smoke", "full", "canary"):
        launcher = builder.local_launcher(stage)
        assert "GH200" not in launcher
        assert "aarch64" not in launcher
        assert "apptainer" not in launcher
        assert "/home/tiago/room315_local_training/venv/bin/python" in launcher


def test_local_and_kairos_output_roots_are_isolated():
    builder = load_builder()
    local = builder.local_config("full", "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
    assert local["local_isolation"]["output_root"] == "/home/tiago/room315_experiment_a_local_outputs"
    assert local["local_isolation"]["guard_state"] == "/home/tiago/room315_experiment_a_local_guard_state.json"
    assert local["local_isolation"]["kairos_output_root_must_not_be_used"] is True
    assert "ROOM315_EXPERIMENT_A_OUTPUT_ROOT" not in builder.local_launcher("full")


def test_local_smoke_leaves_canary_and_legacy_evaluation_untouched():
    builder = load_builder()
    config = builder.local_config("smoke", "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
    assert config["verification_sources"] == [
        "old_replay", "v3r1_train", "v3r1_validation"
    ]
    assert "v3r1_canary" not in config["verification_sources"]
    assert config["data_roles"]["final_evaluation"] == "not_present_and_not_authorized"
    assert config["training"]["maximum_continuation_epochs"] == 2
    assert config["smoke_selection"].endswith("config/smoke_selection.json")


def test_local_full_preserves_exact_experiment_a_scientific_contract():
    builder = load_builder()
    config = builder.local_config("full", "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
    assert config["artifacts"]["approved_checkpoint"]["sha256"] == core.APPROVED_CHECKPOINT_SHA256
    assert config["model"]["input_shape"] == ["B", 6, 224, 224]
    assert config["model"]["output_dimension"] == 200
    assert tuple(config["model"]["identity_order"]) == core.IDENTITIES
    assert config["model"]["augmentations"] == []
    assert config["optimizer"] == {
        "kind": "AdamW", "learning_rate": 0.0001, "weight_decay": 0.0001
    }
    assert config["training"]["maximum_continuation_epochs"] == 10
    assert config["training"]["early_stopping_patience"] == 3
    assert config["training"]["source_balance"] == {
        "old_replay": 0.5, "v3r1_hard_case": 0.5
    }
    assert config["data_roles"]["checkpoint_selection"] == "validation_only"


def test_local_fallback_is_explicit_and_never_automatic():
    builder = load_builder()
    config = builder.local_config("full", "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
    assert config["automatic_fallback"] is False
    default = core.effective_training_config(config, core.LOCAL_DEFAULT_PROFILE)
    fallback = core.effective_training_config(config, core.LOCAL_FALLBACK_PROFILE)
    assert (default["batch_size"], default["gradient_accumulation_steps"]) == (32, 1)
    assert (fallback["batch_size"], fallback["gradient_accumulation_steps"]) == (16, 2)
    assert config["execution_profiles"][core.LOCAL_FALLBACK_PROFILE]["automatic_selection"] is False
    assert config["execution_profiles"][core.LOCAL_FALLBACK_PROFILE]["explicit_cli_flag"] == "--fallback-batch16-accum2"
    launcher = builder.local_launcher("full")
    assert "--fallback-batch16-accum2" in launcher
    assert "no automatic retry" not in launcher.casefold()  # handled by preflight, not a shell retry loop
    assert "while " not in launcher


def _smoke_v2_selection():
    builder = load_builder()
    config = builder.local_config("smoke", "${ROOM315_EXPERIMENT_A_PACKAGE_ROOT}")
    config["stage"] = "smoke_v2"
    config["data"].pop("v3r1_canary", None)
    environment = {
        "ROOM315_EXPERIMENT_A_PACKAGE_ROOT": "/tmp/not-used-by-selection",
        "ROOM315_APPROVED_RUN_ROOT": str(builder.LOCAL_PATHS["approved_run"]),
        "ROOM315_OLD_SPLITS_ROOT": str(builder.LOCAL_PATHS["old_splits"]),
        "ROOM315_OLD_DATASET_ROOT": str(builder.LOCAL_PATHS["old_images"]),
        "ROOM315_V3R1_SPLITS_ROOT": str(builder.LOCAL_PATHS["v3r1_splits"]),
        "ROOM315_V3R1_DATASET_ROOT": str(builder.LOCAL_PATHS["v3r1_images"]),
        "ROOM315_V3R1_GUARD_ROOT": str(builder.LOCAL_PATHS["guard_root"]),
    }
    previous = {key: os.environ.get(key) for key in environment}
    try:
        os.environ.update(environment)
        return smoke_v2.build_selection(config)
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def test_smoke_v2_balanced_payload_side_and_per_identity_coverage():
    selection, audit = _smoke_v2_selection()
    assert selection["counts"] == {
        "old_train": 128,
        "new_train": 128,
        "total_train": 256,
        "validation": 128,
        "old_replay_regression": 128,
    }
    assert audit["passed"] is True
    assert audit["payload_determined_by_side"] is False
    assert all(value > 0 for value in audit["payload_by_side"].values())
    assert all(
        counts["loaded"] >= 8 and counts["empty"] >= 8
        for counts in audit["payload_by_identity"].values()
    )
    assert audit["target_one_hot_failures"] == 0
    assert audit["masked_absent_failures"] == 0
    assert audit["present_mask_failures"] == 0
    assert selection["legacy_test_used"] is False
    assert selection["canary_used"] is False


def test_smoke_v2_baseline_precedes_training_and_validation_is_identical():
    source = inspect.getsource(smoke_v2.run)
    assert source.index("baseline, baseline_side = evaluate_point") < source.index(
        "optimizer = torch.optim.AdamW"
    )
    fingerprint = smoke_v2.rows_fingerprint(["a", "b"])
    fake = {
        "losses": {
            "total": 1.0,
            "segment_location": 1.0,
            "loaded_state": 1.0,
            "bbox": 1.0,
            "s_m": 1.0,
            "s_ratio": 1.0,
        },
        "loaded_state_accuracy": 1.0,
        "per_identity_loaded_state": {},
        "side_accuracy": 1.0,
        "block_top1_accuracy": 1.0,
        "block_top2_accuracy": 1.0,
        "bbox_mae": 0.0,
        "s_m_mae": 0.0,
        "s_m_median": 0.0,
        "s_m_p95": 0.0,
        "s_ratio_mae": 0.0,
        "s_ratio_median": 0.0,
        "s_ratio_p95": 0.0,
    }
    snapshots = [smoke_v2.metric_snapshot(fake, fingerprint) for _ in range(3)]
    assert {
        point["validation_subset_fingerprint_sha256"] for point in snapshots
    } == {fingerprint}


def test_smoke_v1_is_immutable_and_full_remains_unauthorized():
    assert smoke_v2.verify_smoke_v1_immutable()["tree_sha256"] == (
        smoke_v2.SMOKE_V1_TREE_SHA256
    )
    guard = Path("/home/tiago/room315_experiment_a_local_guard_state.json")
    value = json.loads(guard.read_text())
    assert value["stages"]["full"]["state"] == "unauthorized"
    assert value["stages"]["full"]["attempts"] == 0
    assert value["stages"]["full"]["output"] is None
    assert smoke_v2.verify_invalid_smoke_v2_attempt1_immutable()["tree_sha256"] == (
        smoke_v2.INVALID_SMOKE_V2_ATTEMPT1_TREE_SHA256
    )


def test_full_baseline_and_epochs_use_identical_metric_path():
    source = inspect.getsource(experiment_train.train)
    assert source.index("baseline = evaluate_records") < source.index(
        "optimizer = torch.optim.AdamW"
    )
    assert source.count("evaluate_records(") >= 3
    assert "full_baseline_validation_metrics.json" in source
    assert "final_best_validation_metrics.json" in source
    assert "payload_warning_history.json" in source
    canary_source = inspect.getsource(experiment_train.canary)
    assert canary_source.count("evaluate_records(") == 2
    assert "checkpoint_selection_performed\": False" in canary_source


def test_corrected_package_stale_encoder_detection(tmp_path):
    builder = load_corrected_builder()
    clean = tmp_path / "clean"
    clean.mkdir()
    (clean / "encoder.py").write_text("for name in vectorizer['names']:\n    pass\n")
    assert builder.scan_tree(clean)["passed"] is True
    stale = tmp_path / "stale"
    stale.mkdir()
    (stale / "encoder.py").write_text("for key, values in categorical_values.items():\n    pass\n")
    with pytest.raises(builder.CorrectedPackageError, match="stale encoder"):
        builder.scan_tree(stale)


def test_corrected_guard_templates_start_unauthorized_and_isolated():
    builder = load_corrected_builder()
    local = builder.guard_template(local=True)
    assert set(local["stages"]) == {
        "corrected_local_full", "corrected_local_canary"
    }
    assert all(stage["state"] == "unauthorized" for stage in local["stages"].values())
    assert all(stage["attempts"] == 0 for stage in local["stages"].values())
    assert local["automatic_retry"] is False
    assert local["legacy_evaluation_authorized"] is False
    assert local["output_root"] == str(builder.LOCAL_OUTPUT_ROOT)
    config = builder.corrected_config(load_builder(), "full", local=True)
    assert config["local_isolation"]["output_root"] == str(builder.LOCAL_OUTPUT_ROOT)
    assert config["local_isolation"]["guard_state"] == str(builder.LOCAL_GUARD)


def test_corrected_canary_guard_requires_completed_full(tmp_path):
    builder = load_corrected_builder()
    path = tmp_path / "guard.json"
    path.write_text(json.dumps(builder.guard_template(local=True)))
    with pytest.raises(core.ExperimentAError, match="completed Full"):
        experiment_guard.authorize(path, "corrected_local_canary")
    experiment_guard.authorize(path, "corrected_local_full")
    experiment_guard.begin(
        path, "corrected_local_full", tmp_path / "unused_full_output"
    )
    experiment_guard.finish(path, "corrected_local_full", "completed")
    experiment_guard.authorize(path, "corrected_local_canary")
    value = json.loads(path.read_text())
    assert value["stages"]["corrected_local_canary"]["state"] == "authorized"
    assert value["stages"]["corrected_local_full"]["attempts"] == 1


def test_corrected_full_scientific_contract_and_no_canary_training_access():
    builder = load_corrected_builder()
    config = builder.corrected_config(load_builder(), "full", local=True)
    training = config["training"]
    assert training["old_replay_references_per_epoch"] == 4000
    assert training["v3r1_hard_case_references_per_epoch"] == 4000
    assert training["source_balance"] == {"old_replay": 0.5, "v3r1_hard_case": 0.5}
    assert training["batch_size"] == 32
    assert training["automatic_mixed_precision"] is True
    assert training["maximum_continuation_epochs"] == 10
    assert training["early_stopping_patience"] == 3
    assert config["optimizer"] == {
        "kind": "AdamW", "learning_rate": 0.0001, "weight_decay": 0.0001
    }
    assert config["model"]["augmentations"] == []
    assert config["model"]["output_dimension"] == 200
    assert config["verification_sources"] == [
        "old_replay", "v3r1_train", "v3r1_validation"
    ]
    assert config["data_roles"]["checkpoint_selection"] == "validation_only"

    canary = builder.corrected_config(load_builder(), "canary", local=True)
    assert canary["data_roles"]["checkpoint_selection"] == "none_canary_evaluation"
    assert canary["data_roles"]["training_sources"] == []
    assert canary["training"]["training_enabled"] is False
    assert canary["training"]["maximum_continuation_epochs"] == 0
    assert canary["training"]["early_stopping_patience"] == 0
    assert canary["training"]["checkpoint_selection"] == "none"
    assert canary["training"]["old_replay_references_per_epoch"] == 0
    assert canary["training"]["v3r1_hard_case_references_per_epoch"] == 0
    assert canary["training"]["source_balance"] == {}
    assert canary["canary_contract"] == {
        "training_performed": False,
        "checkpoint_selection_performed": False,
        "same_examples_for_approved_and_candidate": True,
        "automatic_deployment_approval": False,
    }


def test_corrected_launchers_preserve_local_and_gh200_contracts():
    builder = load_corrected_builder()
    local = builder.local_launcher("full")
    assert "apptainer" not in local
    assert "GH200" not in local
    assert str(builder.LOCAL_OUTPUT_ROOT) in local
    assert str(builder.LOCAL_GUARD) in local
    assert "experiment_a_local.py" in local
    assert "No automatic fallback" in local
    kairos = builder.kairos_launcher("full")
    assert "apptainer exec --nv" in kairos
    assert "/work/conteneurs/shared/AI/nemo_25.04.03_arm.sif" in kairos
    inside = builder.inside_container()
    assert "--require-gh200" in inside
    assert "--verify-checkpoint-load" in inside


def test_corrected_frozen_integrity_and_archive_reproducibility(tmp_path):
    builder = load_corrected_builder()
    base = load_builder()
    assert builder.frozen_integrity(base)["passed"] is True
    root = tmp_path / "fixture"
    root.mkdir()
    (root / "a.txt").write_text("same\n")
    first = tmp_path / "first.tar.gz"
    second = tmp_path / "second.tar.gz"
    first_hash = base.deterministic_archive(root, first)
    second_hash = base.deterministic_archive(root, second)
    assert first_hash == second_hash


def test_corrected_builder_disables_package_bytecode_generation():
    builder = load_corrected_builder()
    source = Path(builder.__file__).read_text()
    assert "sys.dont_write_bytecode = True" in source
    assert '"PYTHONDONTWRITEBYTECODE": "1"' in source
