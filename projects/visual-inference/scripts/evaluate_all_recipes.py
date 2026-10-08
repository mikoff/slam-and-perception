#!/usr/bin/env python3
"""Comprehensive evaluation script comparing Baseline, Recipe 1, and Recipe 2 (HBB and Quad)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import torch
from torch.utils.data import DataLoader

from student_detector.assigner import ATSSAssigner
from student_detector.decoder import Detection, InferenceDecoder
from student_detector.evaluation import EvaluationImage, evaluate_proposals
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import validate_quad
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentDataset,
    SAMStudentHBBDataset,
    collate_proposal_samples,
    collate_quad_proposal_samples,
)
from student_detector.targets import TargetBuilder
from student_detector.training import validate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Comprehensive evaluation comparing Baseline, Recipe 1, and Recipe 2."
    )
    parser.add_argument(
        "--checkpoints-dir",
        type=Path,
        default=Path("artifacts/phase2/checkpoints"),
        help="Path to checkpoints directory.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("artifacts/phase2/comprehensive_eval_summary.json"),
        help="Output JSON summary path.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device.",
    )
    return parser.parse_args()


@torch.no_grad()
def validate_quad_wrapped(
    model: QuadProposalDetector,
    loader: DataLoader,
    decoder: QuadInferenceDecoder,
    device: torch.device,
) -> dict[str, float]:
    model.eval()
    evaluated: list[EvaluationImage] = []
    for images, samples in loader:
        images = images.to(device, non_blocking=device.type == "cuda")
        output = model(images)
        detections = decoder(output, (images.shape[-2], images.shape[-1]))
        for sample, det in zip(samples, detections, strict=True):
            if det.quads.numel() > 0:
                x1 = det.quads[:, :, 0].min(dim=1).values
                y1 = det.quads[:, :, 1].min(dim=1).values
                x2 = det.quads[:, :, 0].max(dim=1).values
                y2 = det.quads[:, :, 1].max(dim=1).values
                boxes = torch.stack([x1, y1, x2, y2], dim=-1)
            else:
                boxes = torch.empty((0, 4), device=images.device)
            evaluated.append(
                EvaluationImage(
                    sample.image_id,
                    sample.domain,
                    sample.camera_type,
                    (images.shape[-2], images.shape[-1]),
                    sample.boxes.cpu(),
                    sample.ignore_boxes.cpu(),
                    Detection(boxes.cpu(), det.scores.cpu()),
                )
            )
    return evaluate_proposals(evaluated)


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    print(f"Running comprehensive evaluation on {device}...")

    results: dict[str, Any] = {}

    # Datasets
    val_dataset_hbb = SAMStudentHBBDataset(
        pseudo_labels_path=DEFAULT_PSEUDO_LABELS_PATH,
        image_root=DEFAULT_IMAGE_ROOT,
        split="val",
        val_ratio=0.20,
        input_size=384,
        min_stability=0.80,
    )
    val_loader_hbb = DataLoader(
        val_dataset_hbb,
        batch_size=16,
        shuffle=False,
        num_workers=2,
        collate_fn=collate_proposal_samples,
    )

    val_dataset_quad = SAMStudentDataset(
        pseudo_labels_path=DEFAULT_PSEUDO_LABELS_PATH,
        image_root=DEFAULT_IMAGE_ROOT,
        split="val",
        val_ratio=0.20,
        input_size=384,
        min_stability=0.80,
    )
    val_loader_quad = DataLoader(
        val_dataset_quad,
        batch_size=16,
        shuffle=False,
        num_workers=2,
        collate_fn=collate_quad_proposal_samples,
    )

    # 1. HBB Models Evaluation
    hbb_models = {
        "hbb_baseline": args.checkpoints_dir / "mobilenetv4_p2_hbb_best.pt",
        "hbb_recipe2": args.checkpoints_dir / "mobilenetv4_p2_hbb_recipe2_best.pt",
    }

    for name, path in hbb_models.items():
        if not path.exists():
            print(f"Skipping {name}: checkpoint {path.name} not found.")
            continue
        print(f"\nEvaluating HBB model: {name} ({path.name})...")
        ckpt = torch.load(path, map_location=device)
        strides = tuple(ckpt.get("strides", (4, 8, 16, 32)))
        is_dual = ckpt.get("recipe") == "recipe2" or any(
            k.startswith("aux_head") for k in ckpt["model_state_dict"].keys()
        )
        model = StudentDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        model.load_state_dict(ckpt["model_state_dict"])
        model.eval()

        assigner = ATSSAssigner(strides=strides, prior_sizes=(32, 64, 128, 256), top_k=4, center_radius=1.0)
        tb = TargetBuilder(assigner)

        test_modes = ["hard", "part_aware", "part_aware_soft"]
        if is_dual:
            test_modes.append("nms_free")

        for mode in test_modes:
            print(f"  -> Testing {name} with nms_mode='{mode}'...")
            decoder = InferenceDecoder(
                strides=strides,
                top_k=1000,
                nms_iou_threshold=0.50,
                max_detections=100,
                score_mode="objectness",
                nms_mode=mode,
                soft_sigma=0.5,
                part_scale_ratio=0.4,
            )
            m = validate(model, val_loader_hbb, tb, decoder, device)
            tag = f"{name}_{mode}"
            results[tag] = {
                "model": name,
                "nms_mode": mode,
                "recall_100_50": round(m.get("recall/100@50", 0.0) * 100, 2),
                "recall_100_75": round(m.get("recall/100@75", 0.0) * 100, 2),
                "recall_10_50": round(m.get("recall/10@50", 0.0) * 100, 2),
                "ap_50": round(m.get("ap/50", 0.0) * 100, 2),
                "recall_small": round(m.get("recall/size_small/100@50", 0.0) * 100, 2),
                "recall_medium": round(m.get("recall/size_medium/100@50", 0.0) * 100, 2),
                "recall_large": round(m.get("recall/size_large/100@50", 0.0) * 100, 2),
                "recall_general": round(m.get("recall/general/100@50", 0.0) * 100, 2),
                "recall_automotive": round(m.get("recall/automotive/100@50", 0.0) * 100, 2),
                "recall_fisheye": round(m.get("recall/fisheye/100@50", 0.0) * 100, 2),
            }
            print(
                f"     Recall@100@50: {results[tag]['recall_100_50']}% | "
                f"AP@50: {results[tag]['ap_50']}% | "
                f"Small: {results[tag]['recall_small']}% | "
                f"Fisheye: {results[tag]['recall_fisheye']}%"
            )

    # 2. Quad Models Evaluation
    quad_models = {
        "quad_baseline": args.checkpoints_dir / "mobilenetv4_p2_quad_best.pt",
        "quad_recipe2": args.checkpoints_dir / "mobilenetv4_p2_quad_recipe2_best.pt",
    }

    for name, path in quad_models.items():
        if not path.exists():
            print(f"Skipping {name}: checkpoint {path.name} not found.")
            continue
        print(f"\nEvaluating Quad model: {name} ({path.name})...")
        ckpt = torch.load(path, map_location=device)
        strides = tuple(ckpt.get("strides", (4, 8, 16, 32)))
        is_dual = ckpt.get("recipe") == "recipe2" or any(
            k.startswith("aux_head") for k in ckpt["model_state_dict"].keys()
        )
        model = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        model.load_state_dict(ckpt["model_state_dict"])
        model.eval()

        quad_assigner = QuadAssigner(strides=strides, top_k=4, eligible_levels=1)
        qtb = QuadTargetBuilder(quad_assigner)

        test_configs = [
            ("hard_alpha0.3", "hard", 0.3),
            ("hard_alpha0.0", "hard", 0.0),
            ("part_aware", "part_aware", 0.0),
            ("part_aware_soft", "part_aware_soft", 0.0),
        ]
        if is_dual:
            test_configs.append(("nms_free", "nms_free", 0.0))

        for cfg_name, mode, alpha in test_configs:
            print(f"  -> Testing {name} with mode='{mode}' (centerness_alpha={alpha})...")
            decoder = QuadInferenceDecoder(
                strides=strides,
                pre_nms_top_k=1000,
                nms_iou_threshold=0.50,
                max_proposals=100,
                centerness_alpha=alpha,
                nms_mode=mode,
                soft_sigma=0.5,
                part_scale_ratio=0.4,
            )
            m = validate_quad(model, val_loader_quad, qtb, decoder, device)
            tag = f"{name}_{cfg_name}"
            results[tag] = {
                "model": name,
                "config": cfg_name,
                "nms_mode": mode,
                "centerness_alpha": alpha,
                "recall_100_50": round(m.get("recall/100@0.50", 0.0) * 100, 2),
                "recall_100_75": round(m.get("recall/100@0.75", 0.0) * 100, 2),
                "recall_10_50": round(m.get("recall/10@0.50", 0.0) * 100, 2),
                "ar_100": round(m.get("ar/100", 0.0) * 100, 2),
                "recall_small": round(m.get("recall/size_small/100@0.50", 0.0) * 100, 2),
                "recall_medium": round(m.get("recall/size_medium/100@0.50", 0.0) * 100, 2),
                "recall_large": round(m.get("recall/size_large/100@0.50", 0.0) * 100, 2),
                "recall_general": round(m.get("recall/domain_general/100@0.50", 0.0) * 100, 2),
                "recall_automotive": round(m.get("recall/domain_automotive/100@0.50", 0.0) * 100, 2),
                "recall_fisheye": round(m.get("recall/domain_fisheye/100@0.50", 0.0) * 100, 2),
            }
            print(
                f"     Recall@100@0.50: {results[tag]['recall_100_50']}% | "
                f"AR@100: {results[tag]['ar_100']}% | "
                f"Small: {results[tag]['recall_small']}% | "
                f"Fisheye: {results[tag]['recall_fisheye']}%"
            )

    # 3. Quad Models Wrapped to HBB (evaluated against HBB validation labels)
    for name, path in quad_models.items():
        if not path.exists():
            continue
        print(f"\nEvaluating Quad model WRAPPED to HBB: {name} ({path.name})...")
        ckpt = torch.load(path, map_location=device)
        strides = tuple(ckpt.get("strides", (4, 8, 16, 32)))
        is_dual = ckpt.get("recipe") == "recipe2" or any(
            k.startswith("aux_head") for k in ckpt["model_state_dict"].keys()
        )
        model = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        model.load_state_dict(ckpt["model_state_dict"])
        model.eval()

        wrapped_configs = [
            ("part_aware", "part_aware", 0.0),
        ]
        if is_dual:
            wrapped_configs.append(("nms_free", "nms_free", 0.0))

        for cfg_name, mode, alpha in wrapped_configs:
            print(f"  -> Testing {name} wrapped with mode='{mode}'...")
            decoder = QuadInferenceDecoder(
                strides=strides,
                pre_nms_top_k=1000,
                nms_iou_threshold=0.50,
                max_proposals=100,
                centerness_alpha=alpha,
                nms_mode=mode,
                soft_sigma=0.5,
                part_scale_ratio=0.4,
            )
            m = validate_quad_wrapped(model, val_loader_hbb, decoder, device)
            tag = f"{name}_wrapped_to_hbb_{cfg_name}"
            results[tag] = {
                "model": f"{name}_wrapped_to_hbb",
                "config": cfg_name,
                "nms_mode": mode,
                "recall_100_50": round(m.get("recall/100@50", 0.0) * 100, 2),
                "recall_100_75": round(m.get("recall/100@75", 0.0) * 100, 2),
                "recall_10_50": round(m.get("recall/10@50", 0.0) * 100, 2),
                "ap_50": round(m.get("ap/50", 0.0) * 100, 2),
                "recall_small": round(m.get("recall/size_small/100@50", 0.0) * 100, 2),
                "recall_medium": round(m.get("recall/size_medium/100@50", 0.0) * 100, 2),
                "recall_large": round(m.get("recall/size_large/100@50", 0.0) * 100, 2),
                "recall_general": round(m.get("recall/general/100@50", 0.0) * 100, 2),
                "recall_automotive": round(m.get("recall/automotive/100@50", 0.0) * 100, 2),
                "recall_fisheye": round(m.get("recall/fisheye/100@50", 0.0) * 100, 2),
            }
            print(
                f"     Recall@100@50: {results[tag]['recall_100_50']}% | "
                f"AP@50: {results[tag]['ap_50']}% | "
                f"Strict@75: {results[tag]['recall_100_75']}% | "
                f"Large: {results[tag]['recall_large']}% | "
                f"Fisheye: {results[tag]['recall_fisheye']}%"
            )

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nComprehensive evaluation summary saved to: {args.output_json}")


if __name__ == "__main__":
    main()
