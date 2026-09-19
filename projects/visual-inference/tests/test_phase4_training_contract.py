from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, replace
from pathlib import Path
import sqlite3

import pytest
import torch
import yaml

from student_detector.config import load_phase3_config
from student_detector.phase4_identity import (
    verify_phase4_launch_inputs,
    verify_phase4_quad_continuation_inputs,
)
from student_detector.selection_policy import compare_selection_reports

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "configs/correction_phase4_hbb_p3_v1.yaml"
LOCAL_PILOT_CONFIG_PATH = (
    PROJECT_ROOT / "configs/correction_phase4_hbb_p3_local_rtx3060_pilot_v1.yaml"
)
QUAD_PILOT_CONFIG_PATH = (
    PROJECT_ROOT / "configs/correction_phase4_quad_p3_local_rtx3060_pilot_v1.yaml"
)
QUAD_PILOT_CONTRACT_PATH = (
    PROJECT_ROOT / "configs/benchmarks/correction_phase4_quad_pilot_contract_v1.yaml"
)
QUAD_CONTINUATION_CONFIG_PATH = (
    PROJECT_ROOT / "configs/correction_phase4_quad_p3_local_rtx3060_step10000_v1.yaml"
)
QUAD_CONTINUATION_CONTRACT_PATH = (
    PROJECT_ROOT
    / "configs/benchmarks/correction_phase4_quad_continuation_contract_v1.yaml"
)
CONTRACT_PATH = (
    PROJECT_ROOT / "configs/benchmarks/correction_phase4_training_contract_v1.yaml"
)
ENTRY_ARTIFACTS_PATH = (
    PROJECT_ROOT / "configs/benchmarks/correction_phase4_entry_artifacts_v1.json"
)
PROMOTION_POLICY_PATH = (
    PROJECT_ROOT / "configs/benchmarks/correction_phase4_promotion_v1.yaml"
)
CURRENT_HBB_REPORT_PATH = (
    PROJECT_ROOT
    / "artifacts/phase3/correction_quality_1p5_v1/utility_baseline_step200_ema.json"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_phase4_identity_fixture(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> tuple[object, Path, Path, Path]:
    dataset_id = "phase3-production-bg-policy-v2-2026-08-29"
    workspace = tmp_path / dataset_id
    output = workspace / "output"
    annotations = output / "annotations"
    indexes = output / "indexes"
    annotations.mkdir(parents=True)
    indexes.mkdir()
    artifacts: dict[str, object] = {}
    for split in ("train", "val"):
        manifest = annotations / f"proposals_{split}.json"
        manifest.write_text(f"{split}-manifest", encoding="utf-8")
        manifest_sha256 = _sha256(manifest)
        index = indexes / f"quad_{split}.sqlite"
        with sqlite3.connect(index) as connection:
            connection.executescript(
                """
                CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL);
                CREATE TABLE images (row_index INTEGER PRIMARY KEY);
                INSERT INTO images VALUES (0);
                """
            )
            connection.executemany(
                "INSERT INTO metadata VALUES (?, ?)",
                (
                    ("schema_version", "8:proposal-manifest"),
                    ("source_signature", f"sha256:{manifest_sha256}"),
                ),
            )
        artifacts[f"proposals_{split}.json"] = {
            "bytes": manifest.stat().st_size,
            "sha256": manifest_sha256,
        }
        artifacts[f"quad_{split}.sqlite"] = {
            "bytes": index.stat().st_size,
            "images": 1,
            "schema_version": "8:proposal-manifest",
            "source_signature": f"sha256:{manifest_sha256}",
        }
    dataset_contract_path = workspace / "dataset_contract.json"
    dataset_contract_path.write_text(
        json.dumps(
            {
                "schema_version": "phase3-dataset-contract.v1",
                "dataset_id": dataset_id,
                "artifacts": artifacts,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    dataset_contract_sha256 = _sha256(dataset_contract_path)
    cloud_manifest_path = workspace / "reports/cloud_bundle/dataset-manifest.json"
    cloud_manifest_path.parent.mkdir(parents=True)
    cloud_manifest_path.write_text(
        json.dumps(
            {
                "dataset_id": dataset_id,
                "dataset_contract_sha256": dataset_contract_sha256,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    initialization = tmp_path / "initialization.pt"
    initialization.write_bytes(b"phase4-initialization")
    training_contract_path = tmp_path / "training-contract.yaml"
    training_contract_path.write_text(
        yaml.safe_dump(
            {
                "dataset": {
                    "id": dataset_id,
                    "contract_sha256": dataset_contract_sha256,
                    "manifest_sha256": _sha256(cloud_manifest_path),
                },
                "initialization": {
                    "artifact_sha256": _sha256(initialization),
                    "checkpoint_state": "ema_model",
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("PHASE2_DATASET_ROOT", str(output))
    config = load_phase3_config(LOCAL_PILOT_CONFIG_PATH)
    return config, training_contract_path, initialization, indexes / "quad_val.sqlite"


@pytest.fixture
def contract() -> dict[str, object]:
    return yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))


def test_phase4_recipe_matches_optimizer_contract(
    monkeypatch: pytest.MonkeyPatch, contract: dict[str, object]
) -> None:
    monkeypatch.setenv("PHASE2_DATASET_ROOT", "/phase4/dataset")
    config = load_phase3_config(CONFIG_PATH)
    source = contract["source"]
    optimization = contract["optimization"]

    assert isinstance(source, dict)
    assert isinstance(optimization, dict)
    assert source["config_sha256"] == _sha256(CONFIG_PATH)
    assert config.assignment.strides == (8, 16, 32)
    assert config.neck_type == "lite"
    assert not config.pretrained_backbone
    assert config.data.batch_size * config.schedule.accumulation_steps == 128
    assert (
        config.data.batches_per_epoch
        * config.schedule.epochs
        // config.schedule.accumulation_steps
        == optimization["successful_optimizer_steps"]
        == 20_000
    )
    assert optimization["sampled_images"] == 2_560_000
    assert config.schedule.warmup_steps == 1_000
    assert config.schedule.amp_initial_scale == 8_192
    assert config.schedule.optimizer_step_skip_policy == "error"
    assert config.schedule.validation_interval == 2
    assert config.schedule.checkpoint_every_steps == 500


def test_local_rtx3060_pilot_preserves_effective_optimizer_contract(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PHASE2_DATASET_ROOT", "/phase4/dataset")
    cloud = load_phase3_config(CONFIG_PATH)
    local = load_phase3_config(LOCAL_PILOT_CONFIG_PATH)

    assert local.data.batch_size == 16
    assert local.schedule.accumulation_steps == 8
    assert local.data.batch_size * local.schedule.accumulation_steps == 128
    assert local.data.batches_per_epoch // local.schedule.accumulation_steps == 1_000
    assert local.data.workers == 0
    assert local.schedule == replace(cloud.schedule, accumulation_steps=8)
    assert local.data == replace(
        cloud.data,
        batch_size=16,
        batches_per_epoch=8_000,
        workers=0,
    )


def test_quad_pilot_is_matched_to_local_hbb_pilot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PHASE2_DATASET_ROOT", "/phase4/dataset")
    hbb = load_phase3_config(LOCAL_PILOT_CONFIG_PATH)
    quad = load_phase3_config(QUAD_PILOT_CONFIG_PATH)
    contract = yaml.safe_load(QUAD_PILOT_CONTRACT_PATH.read_text(encoding="utf-8"))

    assert quad.data.batch_size == 16
    assert quad.schedule.accumulation_steps == 8
    assert quad.data.batch_size * quad.schedule.accumulation_steps == 128
    assert quad.data.batches_per_epoch // quad.schedule.accumulation_steps == 1_000
    assert quad.schedule == hbb.schedule
    assert quad.data == hbb.data
    assert contract["optimization"]["successful_optimizer_steps"] == 2_000
    assert contract["optimization"]["sampled_images"] == 256_000
    assert contract["initialization"]["checkpoint_state"] == "ema_model"


def test_quad_continuation_preserves_recipe_and_binds_stage(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PHASE2_DATASET_ROOT", "/phase4/dataset")
    pilot = load_phase3_config(QUAD_PILOT_CONFIG_PATH)
    continuation = load_phase3_config(QUAD_CONTINUATION_CONFIG_PATH)
    contract = yaml.safe_load(
        QUAD_CONTINUATION_CONTRACT_PATH.read_text(encoding="utf-8")
    )

    assert continuation == replace(
        pilot,
        output_dir=PROJECT_ROOT / "artifacts/phase4/local_quad_p3_seed42_step10000_v1",
    )
    assert contract["configuration"]["sha256"] == _sha256(QUAD_CONTINUATION_CONFIG_PATH)
    assert contract["resume"]["source_global_step"] == 5_000
    assert contract["optimization"]["target_global_step"] == 10_000
    assert contract["decision"]["step15000_launch"].startswith("requires_all_gates")


def test_phase4_local_launch_accepts_exact_dataset_identity(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, contract_path, initialization, _ = _write_phase4_identity_fixture(
        tmp_path, monkeypatch
    )

    result = verify_phase4_launch_inputs(
        config,
        training_contract_path=contract_path,
        initialization_path=initialization,
        initialization_state="ema_model",
        initialization_required=True,
    )

    assert result["dataset_id"] == "phase3-production-bg-policy-v2-2026-08-29"
    assert result["index_schema"] == "8:proposal-manifest"


def test_phase4_quad_continuation_accepts_exact_resume_and_new_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, contract_path, _, _ = _write_phase4_identity_fixture(tmp_path, monkeypatch)
    config_path = tmp_path / "configs/continuation.yaml"
    config_path.parent.mkdir()
    config_path.write_text("continuation-config\n", encoding="utf-8")
    resume = tmp_path / "resume.pt"
    resume.write_bytes(b"step-5000-full-state")
    output = tmp_path / "step10000"
    config = replace(config, output_dir=output)
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    contract.update(
        {
            "configuration": {"sha256": _sha256(config_path)},
            "resume": {
                "artifact": "resume.pt",
                "artifact_sha256": _sha256(resume),
                "interrupted_stage_artifact": "step10000/last.pt",
                "interrupted_stage_step_range": [5_001, 9_999],
            },
            "optimization": {
                "target_global_step": 10_000,
                "output_dir": "step10000",
                "initial_output_must_be_absent": True,
            },
        }
    )
    contract_path.write_text(yaml.safe_dump(contract), encoding="utf-8")

    result = verify_phase4_quad_continuation_inputs(
        config,
        config_path=config_path,
        training_contract_path=contract_path,
        resume_path=resume,
        target_global_step=10_000,
    )

    assert result["resume_sha256"] == _sha256(resume)
    assert result["resume_mode"] == "initial"


def test_phase4_quad_continuation_accepts_own_interrupted_checkpoint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, contract_path, _, _ = _write_phase4_identity_fixture(tmp_path, monkeypatch)
    config_path = tmp_path / "configs/continuation.yaml"
    config_path.parent.mkdir()
    config_path.write_text("continuation-config\n", encoding="utf-8")
    output = tmp_path / "step10000"
    output.mkdir()
    config = replace(config, output_dir=output)
    resume = output / "last.pt"
    torch.save(
        {
            "phase": "quad_proposals",
            "architecture": "mobilenetv4_conv_medium_lite_fpn96_quad_dqco",
            "config": asdict(config),
            "global_step": 7_500,
        },
        resume,
    )
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    contract.update(
        {
            "configuration": {"sha256": _sha256(config_path)},
            "resume": {
                "artifact": "source.pt",
                "artifact_sha256": "unused-for-recovery",
                "interrupted_stage_artifact": "step10000/last.pt",
                "interrupted_stage_step_range": [5_001, 9_999],
            },
            "optimization": {
                "target_global_step": 10_000,
                "output_dir": "step10000",
                "initial_output_must_be_absent": True,
            },
        }
    )
    contract_path.write_text(yaml.safe_dump(contract), encoding="utf-8")

    result = verify_phase4_quad_continuation_inputs(
        config,
        config_path=config_path,
        training_contract_path=contract_path,
        resume_path=resume,
        target_global_step=10_000,
    )

    assert result["resume_mode"] == "interrupted_stage"


def test_phase4_local_launch_rejects_stale_schema6_index(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, contract_path, initialization, validation_index = (
        _write_phase4_identity_fixture(tmp_path, monkeypatch)
    )
    with sqlite3.connect(validation_index) as connection:
        connection.execute(
            "UPDATE metadata SET value='6:proposal-manifest' WHERE key='schema_version'"
        )

    with pytest.raises(ValueError, match="val index schema mismatch"):
        verify_phase4_launch_inputs(
            config,
            training_contract_path=contract_path,
            initialization_path=initialization,
            initialization_state="ema_model",
            initialization_required=True,
        )


def test_phase4_local_launch_rejects_wrong_initialization(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, contract_path, initialization, _ = _write_phase4_identity_fixture(
        tmp_path, monkeypatch
    )
    initialization.write_bytes(b"wrong")

    with pytest.raises(ValueError, match="initialization SHA-256 mismatch"):
        verify_phase4_launch_inputs(
            config,
            training_contract_path=contract_path,
            initialization_path=initialization,
            initialization_state="ema_model",
            initialization_required=True,
        )


def test_phase4_contract_binds_published_initialization_and_dataset(
    contract: dict[str, object],
) -> None:
    entry = json.loads(ENTRY_ARTIFACTS_PATH.read_text(encoding="utf-8"))
    initialization = contract["initialization"]
    dataset = contract["dataset"]
    hbb_artifact = next(
        item
        for item in entry["artifacts"]
        if item["role"] == "hbb_weights_initialization"
    )

    assert isinstance(initialization, dict)
    assert isinstance(dataset, dict)
    assert initialization["artifact_s3_key"] == hbb_artifact["s3_key"]
    assert initialization["artifact_sha256"] == hbb_artifact["sha256"]
    assert initialization["checkpoint_state"] == "ema_model"
    assert dataset["id"] == entry["dataset"]["id"]
    assert dataset["manifest_sha256"] == entry["dataset"]["manifest"]["sha256"]


def test_phase4_contract_freezes_selection_and_cloud_ceilings(
    contract: dict[str, object],
) -> None:
    validation = contract["validation"]
    cloud = contract["cloud"]
    pilot = contract["pilot"]

    assert isinstance(validation, dict)
    assert isinstance(cloud, dict)
    assert isinstance(pilot, dict)
    assert validation["states"] == ["raw", "ema"]
    assert validation["frozen_selection"]["score_threshold"] == 0.30
    assert validation["frozen_selection"]["nms_iou_threshold"] == 0.70
    assert validation["frozen_selection"]["minimum_ar100_improvement_pp"] == 0.25
    assert validation["frozen_selection"]["incumbent_geometry"] == "quad"
    assert validation["frozen_selection"]["challenger_geometry"] == "hbb"
    assert validation["frozen_selection"]["promotion_requires_both_policies_to_pass"]
    assert validation["frozen_selection"][
        "incumbent_promotion_policy_sha256"
    ] == _sha256(PROMOTION_POLICY_PATH)
    assert cloud["provider"] == "packet"
    assert cloud["gpu"] == "RTX4090"
    assert cloud["maximum_launch_attempts"] == 3
    assert cloud["maximum_wall_time_hours"] == 12
    assert cloud["maximum_projected_compute_cost_usd"] == 12.00
    assert cloud["hourly_rate_input"] == "PACKET_HOURLY_RATE_USD"
    assert "exact-RTX4090" in cloud["placement"]
    assert pilot["mode"] == "pilot"
    assert pilot["successful_optimizer_steps"] == 2_000
    assert pilot["checkpoint_interval_steps"] == 500
    assert pilot["interruption_and_resume"] == "required"


def test_phase4_promotion_policy_keeps_quad_as_current_incumbent() -> None:
    policy = yaml.safe_load(PROMOTION_POLICY_PATH.read_text(encoding="utf-8"))
    report = json.loads(CURRENT_HBB_REPORT_PATH.read_text(encoding="utf-8"))

    result = compare_selection_reports(report, report, policy)

    assert result["comparison"] == {
        "baseline_geometry": "quad",
        "candidate_geometry": "hbb",
    }
    assert result["status"] == "fail"
    assert result["objective"]["delta_pp"] == pytest.approx(-1.7753969772)
