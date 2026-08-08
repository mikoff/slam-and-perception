"""Evaluate a saved checkpoint against the deterministic G6 acceptance gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path
from typing import Any

import torch
from torch.utils.data import DataLoader

from student_detector.checkpoints import selected_checkpoint_state
from student_detector.config import load_phase3_config
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import validate_quad


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_metadata() -> dict[str, Any]:
    root = Path(__file__).resolve().parents[1]
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True, capture_output=True, check=True
    ).stdout.strip()
    status = subprocess.run(
        ["git", "status", "--short"], cwd=root, text=True, capture_output=True, check=True
    ).stdout.splitlines()
    return {"revision": revision, "dirty": bool(status), "status": status}


def _training_trend(log_path: Path) -> dict[str, Any]:
    validations: list[dict[str, Any]] = []
    finite_training = True
    if log_path.exists():
        for line in log_path.read_text(encoding="utf-8").splitlines():
            record = json.loads(line)
            if record.get("kind") == "validation":
                validations.append(record)
            elif record.get("kind") == "train":
                finite_training &= all(
                    math.isfinite(float(record[key]))
                    for key in ("loss", "quality_loss", "corner_loss", "validity_loss")
                    if key in record
                )
    first = validations[0] if validations else {}
    last = validations[-1] if validations else {}
    corner_start = first.get("corner_error/normalized_median")
    corner_end = last.get("corner_error/normalized_median")
    score_start = first.get("matched_score/median")
    score_end = last.get("matched_score/median")
    return {
        "validation_points": len(validations),
        "finite_training_log": finite_training,
        "corner_error_start": corner_start,
        "corner_error_end": corner_end,
        "corner_error_decreased": (
            corner_start is not None and corner_end is not None and corner_end < corner_start
        ),
        "matched_score_start": score_start,
        "matched_score_end": score_end,
        "matched_score_increased": (
            score_start is not None and score_end is not None and score_end > score_start
        ),
    }


@torch.no_grad()
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    config = load_phase3_config(args.config)
    device = torch.device(
        "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    )
    if args.device == "auto" and device.type != "cuda":
        device = torch.device("cpu")
    checkpoint_path = args.checkpoint.resolve()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    model = QuadProposalDetector(pretrained_backbone=False).to(device)
    model.load_state_dict(checkpoint[selected_checkpoint_state(checkpoint)])
    model.eval()

    manifest_path = config.data.quad_val_annotations or config.data.val_annotations
    dataset = QuadProposalDataset(
        manifest_path,
        config.data.image_root,
        config.data.index_dir / "g6_evaluation.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=True,
    )
    loader = DataLoader(
        dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=collate_quad_proposal_samples,
    )
    assigner = QuadAssigner(
        strides=config.assignment.strides,
        top_k=config.quad.top_k,
        gamma=config.quad.gamma,
        scale_sigma=config.quad.scale_sigma,
        eligible_levels=config.quad.eligible_levels,
        scale_measure=config.quad.scale_measure,
    )
    builder = QuadTargetBuilder(
        assigner, weak_negative_weight=config.quad.weak_negative_weight
    )
    decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_proposals=config.inference.pre_nms_top_k,
    )
    metrics = validate_quad(
        model,
        loader,
        builder,
        decoder,
        device,
        include_dense_diagnostics=True,
    )
    trend = _training_trend(checkpoint_path.parent / "quad_metrics.jsonl")
    tier_keys = sorted(
        key for key in metrics if key.startswith("recall/geometry_tier_") and key.endswith("@0.50")
    )
    condition_keys = sorted(
        key for key in metrics if key.startswith("recall/condition_") and key.endswith("@0.50")
    )
    checks = {
        "recall_100_iou_050": metrics.get("recall/100@0.50") == 1.0,
        "recall_100_iou_075": metrics.get("recall/100@0.75", 0.0) >= 0.95,
        "every_geometry_tier_matched": bool(tier_keys)
        and all(metrics[key] > 0 for key in tier_keys),
        "every_object_condition_matched": bool(condition_keys)
        and all(metrics[key] > 0 for key in condition_keys),
        "finite_metrics": all(
            math.isfinite(float(value)) for value in metrics.values()
        ),
        "no_invalid_post_nms_quads": metrics.get("decoder/post_nms_invalid_count") == 0.0,
        "corner_error_decreased": trend["corner_error_decreased"],
        "proposal_quality_increased": trend["matched_score_increased"],
        "finite_training_log": trend["finite_training_log"],
        "checkpoint_reached_recorded_step": checkpoint.get("global_step", 0) > 0,
    }
    report = {
        "gate": "G6",
        "pass": all(checks.values()),
        "checks": checks,
        "metrics": metrics,
        "trend": trend,
        "fixture": {
            "manifest": str(Path(manifest_path)),
            "manifest_sha256": _sha256(Path(manifest_path)),
            "image_count": len(dataset),
            "positive_count": sum(record.positive_count for record in dataset.records),
        },
        "run": {
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": _sha256(checkpoint_path),
            "config": str(args.config.resolve()),
            "config_sha256": _sha256(args.config.resolve()),
            "device": str(device),
            "torch": torch.__version__,
            "checkpoint_global_step": checkpoint.get("global_step"),
            "checkpoint_architecture": checkpoint.get("architecture"),
            "checkpoint_geometry_quality_target": checkpoint.get("config", {})
            .get("quad", {})
            .get("geometry_quality_target"),
            "software": _git_metadata(),
        },
    }
    output = args.output or checkpoint_path.parent / "g6_report.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
