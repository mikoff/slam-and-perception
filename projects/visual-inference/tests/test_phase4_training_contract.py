from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from student_detector.config import load_phase3_config
from student_detector.selection_policy import compare_selection_reports

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "configs/correction_phase4_hbb_p3_v1.yaml"
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

    assert isinstance(validation, dict)
    assert isinstance(cloud, dict)
    assert validation["states"] == ["raw", "ema"]
    assert validation["frozen_selection"]["score_threshold"] == 0.30
    assert validation["frozen_selection"]["nms_iou_threshold"] == 0.70
    assert validation["frozen_selection"]["minimum_ar100_improvement_pp"] == 0.25
    assert validation["frozen_selection"]["incumbent_geometry"] == "quad"
    assert validation["frozen_selection"]["challenger_geometry"] == "hbb"
    assert validation["frozen_selection"]["promotion_requires_both_policies_to_pass"]
    assert (
        validation["frozen_selection"]["incumbent_promotion_policy_sha256"]
        == _sha256(PROMOTION_POLICY_PATH)
    )
    assert cloud["provider"] == "packet"
    assert cloud["gpu"] == "RTX4090"
    assert cloud["maximum_launch_attempts"] == 3
    assert cloud["maximum_wall_time_hours"] == 12
    assert cloud["maximum_projected_compute_cost_usd"] == 12.00
    assert "exact-RTX4090" in cloud["placement"]


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
