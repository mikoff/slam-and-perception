#!/usr/bin/env python3
"""Phase 2: MobileNetV4 Student Proposal Detector Training (HBB and Quad) on the 800-image cohort."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import time
from typing import Any

import torch
from torch.utils.data import DataLoader

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.assigner import ATSSAssigner
from student_detector.decoder import InferenceDecoder
from student_detector.losses import ProposalLoss
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_losses import QuadProposalLoss
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
from student_detector.task_aligned_assigner import (
    QuadTaskAlignedAssigner,
    TaskAlignedAssigner,
)
from student_detector.training import validate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train MobileNetV4 Student Proposal Detector on 800-image SAM cohort."
    )
    parser.add_argument(
        "--geometry",
        type=str,
        choices=["hbb", "quad"],
        default="hbb",
        help="Proposal geometry: 'hbb' (horizontal bounding boxes) or 'quad' (oriented quads).",
    )
    parser.add_argument(
        "--recipe",
        type=str,
        choices=["p2_baseline", "recipe2"],
        default="p2_baseline",
        help="Training recipe: 'p2_baseline' (ATSS single-head) or 'recipe2' (SOTA Dual-Head TAL with NMS-free primary and GWD).",
    )
    parser.add_argument(
        "--pseudo-labels",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_PATH,
        help="Path to pseudo-labels JSON or SQLite.",
    )
    parser.add_argument(
        "--val-pseudo-labels",
        type=Path,
        default=None,
        help="Optional path to validation pseudo-labels (defaults to --pseudo-labels with split='val').",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to root image directory.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=15,
        help="Number of training epochs (default: 15).",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=32,
        help="Batch size (default: 32).",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=1.5e-3,
        help="Peak learning rate for AdamW (default: 1.5e-3).",
    )
    parser.add_argument(
        "--weight-decay",
        type=float,
        default=1e-4,
        help="Weight decay for AdamW (default: 1e-4).",
    )
    parser.add_argument(
        "--input-size",
        type=int,
        default=384,
        help="Input image resolution (default: 384).",
    )
    parser.add_argument(
        "--min-stability",
        type=float,
        default=0.80,
        help="Minimum stability score for pseudo-labels (default: 0.80).",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="DataLoader worker count (default: 4).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/phase2"),
        help="Output directory for checkpoints and metrics (default: artifacts/phase2).",
    )
    parser.add_argument(
        "--val-interval",
        type=int,
        default=1,
        help="Epoch interval for validation (default: 1).",
    )
    parser.add_argument(
        "--score-mode",
        type=str,
        default="objectness_x_centerness",
        choices=["objectness", "objectness_x_centerness"],
        help="Scoring mode for HBB decoder (default: objectness_x_centerness).",
    )
    parser.add_argument(
        "--centerness-alpha",
        type=float,
        default=1.0,
        help="Exponent for centerness scoring in HBB decoder (default: 1.0).",
    )
    parser.add_argument(
        "--eval-only",
        action="store_true",
        help="Run evaluation only using --checkpoint and exit.",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=None,
        help="Path to checkpoint file for evaluation.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device ('cuda' or 'cpu').",
    )
    return parser.parse_args()


def train() -> None:
    args = parse_args()
    device = torch.device(args.device)
    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Starting MobileNetV4 P2-{args.geometry.upper()} Student Training"
    )
    print(
        f"Device: {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})"
    )
    print(
        f"Configuration: geometry={args.geometry}, epochs={args.epochs}, batch_size={args.batch_size}, lr={args.lr}, input_size={args.input_size}"
    )
    print(f"Pseudo-labels: {args.pseudo_labels}")

    checkpoints_dir = args.output_dir / "checkpoints"
    checkpoints_dir.mkdir(parents=True, exist_ok=True)
    strides = (4, 8, 16, 32)

    # 1. Datasets and Loaders
    val_pseudo_labels = args.val_pseudo_labels or args.pseudo_labels
    if args.geometry == "hbb":
        train_dataset = SAMStudentHBBDataset(
            pseudo_labels_path=args.pseudo_labels,
            image_root=args.image_root,
            split="train",
            val_ratio=0.20,
            input_size=args.input_size,
            min_stability=args.min_stability,
        )
        val_dataset = SAMStudentHBBDataset(
            pseudo_labels_path=val_pseudo_labels,
            image_root=args.image_root,
            split="val",
            val_ratio=0.20,
            input_size=args.input_size,
            min_stability=args.min_stability,
        )
        collate_fn = collate_proposal_samples
    else:
        train_dataset = SAMStudentDataset(
            pseudo_labels_path=args.pseudo_labels,
            image_root=args.image_root,
            split="train",
            val_ratio=0.20,
            input_size=args.input_size,
            min_stability=args.min_stability,
        )
        val_dataset = SAMStudentDataset(
            pseudo_labels_path=val_pseudo_labels,
            image_root=args.image_root,
            split="val",
            val_ratio=0.20,
            input_size=args.input_size,
            min_stability=args.min_stability,
        )
        collate_fn = collate_quad_proposal_samples

    print(
        f"Cohorts: Train={len(train_dataset)} images ({train_dataset.total_gt} GT, {train_dataset.total_novel} SAM novel), "
        f"Val={len(val_dataset)} images ({val_dataset.total_gt} GT, {val_dataset.total_novel} SAM novel)"
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
    )

    # 2. Model & SOTA Assignment/Loss/Decoder Setup
    if args.geometry == "hbb":
        is_recipe2 = args.recipe == "recipe2"
        model = StudentDetector(
            pretrained_backbone=True,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_recipe2,
        ).to(device)

        if is_recipe2:
            # SOTA Recipe 2: Consistent Dual Assignment (1-to-1 primary, 1-to-many TAL auxiliary)
            assigner = TaskAlignedAssigner(
                strides=strides,
                assignment_mode="one_to_one",
            )
            assigner_aux = TaskAlignedAssigner(
                strides=strides,
                top_k=4,
                assignment_mode="one_to_many",
            )
            target_builder = TargetBuilder(assigner)
            target_builder_aux = TargetBuilder(assigner_aux)
            # Evaluate primary head in pure NMS-Free mode
            decoder = InferenceDecoder(
                strides=strides,
                top_k=1000,
                nms_iou_threshold=0.50,
                max_detections=100,
                score_mode="objectness",
                nms_mode="nms_free",
            )
        else:
            # P2 Baseline: Tightened ATSS (top_k=4, center_radius=1.0)
            assigner = ATSSAssigner(
                strides=strides,
                prior_sizes=(32, 64, 128, 256),
                top_k=4,
                center_radius=1.0,
            )
            target_builder = TargetBuilder(assigner)
            target_builder_aux = None
            decoder = InferenceDecoder(
                strides=strides,
                top_k=1000,
                nms_iou_threshold=0.50,
                max_detections=100,
                score_mode=args.score_mode,
                centerness_alpha=args.centerness_alpha,
            )

        criterion = ProposalLoss(
            strides=strides,
            objectness_loss="quality_focal",
            box_loss="ciou",
            box_weight=2.0,
            centerness_weight=1.0,
        )
    else:
        is_recipe2 = args.recipe == "recipe2"
        # Quad Proposal Detector with learnable per-level scales
        model = QuadProposalDetector(
            pretrained_backbone=True,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_recipe2,
        ).to(device)

        if is_recipe2:
            # SOTA Recipe 2: Consistent Dual Assignment (1-to-1 primary, 1-to-many TAL auxiliary)
            assigner = QuadTaskAlignedAssigner(
                strides=strides,
                assignment_mode="one_to_one",
            )
            assigner_aux = QuadTaskAlignedAssigner(
                strides=strides,
                top_k=4,
                assignment_mode="one_to_many",
            )
            target_builder = QuadTargetBuilder(assigner, weak_negative_weight=0.15)
            target_builder_aux = QuadTargetBuilder(
                assigner_aux, weak_negative_weight=0.15
            )
            criterion = QuadProposalLoss(
                strides=strides,
                quality_weight=1.0,
                corner_weight=2.0,
                validity_weight=0.05,
                quality_target_mode="blend",
                quality_blend=0.8,
                geometry_quality_target="exact_iou",
                gwd_weight=1.0,  # SOTA: Active Gaussian Wasserstein Distance
            )
            # Evaluate primary head in pure NMS-Free mode
            decoder = QuadInferenceDecoder(
                strides=strides,
                pre_nms_top_k=1000,
                nms_iou_threshold=0.50,
                max_proposals=100,
                centerness_alpha=0.0,
                nms_mode="nms_free",
            )
        else:
            # P2 Baseline: Tightened Quad ATSS (top_k=4, eligible_levels=1)
            assigner = QuadAssigner(
                strides=strides,
                top_k=4,
                eligible_levels=1,
            )
            target_builder = QuadTargetBuilder(assigner, weak_negative_weight=0.15)
            target_builder_aux = None
            criterion = QuadProposalLoss(
                strides=strides,
                quality_weight=1.0,
                corner_weight=2.0,
                validity_weight=0.05,
                quality_target_mode="blend",
                quality_blend=0.8,
                geometry_quality_target="exact_iou",
            )
            decoder = QuadInferenceDecoder(
                strides=strides,
                pre_nms_top_k=1000,
                nms_iou_threshold=0.50,
                max_proposals=100,
                centerness_alpha=0.0,
            )

    if args.eval_only:
        ckpt_path = args.checkpoint
        if ckpt_path is None:
            suffix = "_recipe2" if args.recipe == "recipe2" else ""
            ckpt_path = checkpoints_dir / f"mobilenetv4_p2_{args.geometry}{suffix}_best.pt"
        if not ckpt_path.exists():
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
        print(f"\n[Eval-Only] Loading checkpoint: {ckpt_path}")
        ckpt = torch.load(ckpt_path, map_location=device)
        model.load_state_dict(ckpt["model_state_dict"])
        model.eval()
        if args.geometry == "hbb":
            metrics = validate(
                model=model,
                loader=val_loader,
                target_builder=target_builder,
                decoder=decoder,
                device=device,
            )
        else:
            metrics = validate_quad(
                model=model,
                loader=val_loader,
                target_builder=target_builder,
                decoder=decoder,
                device=device,
            )
        print("\n================ EVALUATION METRICS ================")
        for k, v in sorted(metrics.items()):
            if isinstance(v, float):
                print(f"  {k:35s}: {v * 100:6.2f}%")
        print("====================================================\n")
        return

    # Differential learning rate: 0.2x for pretrained backbone, 1.0x for neck & head
    opt_params = [
        {"params": model.backbone.parameters(), "lr": args.lr * 0.2},
        {"params": model.fpn.parameters(), "lr": args.lr},
        {"params": model.head.parameters(), "lr": args.lr},
    ]
    if getattr(model, "dual_head", False) and model.aux_head is not None:
        opt_params.append({"params": model.aux_head.parameters(), "lr": args.lr})

    optimizer = torch.optim.AdamW(
        opt_params,
        lr=args.lr,
        weight_decay=args.weight_decay,
    )
    warmup_epochs = 1
    warmup_scheduler = torch.optim.lr_scheduler.LinearLR(
        optimizer, start_factor=0.2, end_factor=1.0, total_iters=warmup_epochs
    )
    cosine_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs - warmup_epochs, 1), eta_min=1e-5
    )
    scheduler = torch.optim.lr_scheduler.SequentialLR(
        optimizer,
        schedulers=[warmup_scheduler, cosine_scheduler],
        milestones=[warmup_epochs],
    )
    scaler = torch.amp.GradScaler(device.type, enabled=(device.type == "cuda"))

    history: list[dict[str, Any]] = []
    best_recall_50 = -1.0
    total_start_time = time.time()

    print(
        f"\n--- Training Loop Starting ({args.geometry.upper()}, Recipe={args.recipe}) ---"
    )
    for epoch in range(1, args.epochs + 1):
        epoch_start_time = time.time()
        model.train()
        train_dataset.set_epoch(epoch)

        epoch_loss = 0.0
        epoch_sub1 = 0.0
        epoch_sub2 = 0.0
        batch_count = 0

        for images, samples in train_loader:
            images = images.to(device, non_blocking=(device.type == "cuda"))
            optimizer.zero_grad()

            with torch.amp.autocast(
                device.type, dtype=torch.float16, enabled=(device.type == "cuda")
            ):
                if getattr(model, "dual_head", False):
                    output_primary, output_aux = model(images)
                    if args.geometry == "hbb":
                        feature_shapes = tuple(
                            (t.shape[-2], t.shape[-1]) for t in output_primary.objectness
                        )
                        targets_pri = target_builder(
                            samples, feature_shapes, device=device
                        )
                        targets_aux = target_builder_aux(
                            samples, feature_shapes, device=device
                        )
                        loss_pri = criterion(output_primary, targets_pri)
                        loss_aux = criterion(output_aux, targets_aux)
                        loss_total = loss_pri.total + loss_aux.total
                        sub1 = loss_pri.objectness.item()
                        sub2 = loss_pri.box_ciou.item()
                    else:
                        feature_shapes = tuple(
                            (t.shape[-2], t.shape[-1]) for t in output_primary.quality
                        )
                        targets_pri = target_builder(
                            samples, feature_shapes, device=device
                        )
                        targets_aux = target_builder_aux(
                            samples, feature_shapes, device=device
                        )
                        loss_pri = criterion(output_primary, targets_pri)
                        loss_aux = criterion(output_aux, targets_aux)
                        loss_total = loss_pri.total + loss_aux.total
                        sub1 = loss_pri.quality.item()
                        sub2 = loss_pri.corner.item()
                else:
                    output = model(images)
                    if args.geometry == "hbb":
                        feature_shapes = tuple(
                            (t.shape[-2], t.shape[-1]) for t in output.objectness
                        )
                        targets = target_builder(samples, feature_shapes, device=device)
                        loss_output = criterion(output, targets)
                        loss_total = loss_output.total
                        sub1 = loss_output.objectness.item()
                        sub2 = loss_output.box_ciou.item()
                    else:
                        feature_shapes = tuple(
                            (t.shape[-2], t.shape[-1]) for t in output.quality
                        )
                        targets = target_builder(samples, feature_shapes, device=device)
                        loss_output = criterion(output, targets)
                        loss_total = loss_output.total
                        sub1 = loss_output.quality.item()
                        sub2 = loss_output.corner.item()

            scaler.scale(loss_total).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            scaler.step(optimizer)
            scaler.update()

            epoch_loss += loss_total.item()
            epoch_sub1 += sub1
            epoch_sub2 += sub2
            batch_count += 1

        scheduler.step()
        avg_loss = epoch_loss / max(batch_count, 1)
        avg_sub1 = epoch_sub1 / max(batch_count, 1)
        avg_sub2 = epoch_sub2 / max(batch_count, 1)
        epoch_duration = time.time() - epoch_start_time

        sub1_name = "obj" if args.geometry == "hbb" else "qual"
        sub2_name = "ciou" if args.geometry == "hbb" else "corn"
        print(
            f"Epoch [{epoch:02d}/{args.epochs:02d}] ({epoch_duration:.1f}s) - "
            f"Loss: {avg_loss:.4f} ({sub1_name}: {avg_sub1:.4f}, {sub2_name}: {avg_sub2:.4f}) - "
            f"LR: {scheduler.get_last_lr()[-1]:.6f}"
        )

        epoch_record: dict[str, Any] = {
            "epoch": epoch,
            "duration_seconds": round(epoch_duration, 2),
            "lr": round(scheduler.get_last_lr()[-1], 6),
            "train_loss": round(avg_loss, 4),
            f"train_{sub1_name}_loss": round(avg_sub1, 4),
            f"train_{sub2_name}_loss": round(avg_sub2, 4),
        }

        # Validation
        if epoch % args.val_interval == 0 or epoch == args.epochs:
            val_start_time = time.time()
            if args.geometry == "hbb":
                val_metrics = validate(
                    model=model,
                    loader=val_loader,
                    target_builder=target_builder,
                    decoder=decoder,
                    device=device,
                )
                rec_10_50 = val_metrics.get("recall/10@50", 0.0)
                rec_100_50 = val_metrics.get("recall/100@50", 0.0)
                rec_100_75 = val_metrics.get("recall/100@75", 0.0)
                ap_50 = val_metrics.get("ap/50", 0.0)

                rec_edge = val_metrics.get("recall/size_edge_small/100@50", 0.0)
                rec_small = val_metrics.get("recall/size_small/100@50", 0.0)
                rec_med = val_metrics.get("recall/size_medium/100@50", 0.0)
                rec_lrg = val_metrics.get("recall/size_large/100@50", 0.0)

                rec_gen = val_metrics.get("recall/general/100@50", 0.0)
                rec_auto = val_metrics.get("recall/automotive/100@50", 0.0)
                rec_fish = val_metrics.get("recall/fisheye/100@50", 0.0)
            else:
                val_metrics = validate_quad(
                    model=model,
                    loader=val_loader,
                    target_builder=target_builder,
                    decoder=decoder,
                    device=device,
                )
                rec_10_50 = val_metrics.get("recall/10@0.50", 0.0)
                rec_100_50 = val_metrics.get("recall/100@0.50", 0.0)
                rec_100_75 = val_metrics.get("recall/100@0.75", 0.0)
                ap_50 = val_metrics.get("ar/100", 0.0)  # Average Recall for Quad

                rec_edge = val_metrics.get("recall/size_tiny/100@0.50", 0.0)
                rec_small = val_metrics.get("recall/size_small/100@0.50", 0.0)
                rec_med = val_metrics.get("recall/size_medium/100@0.50", 0.0)
                rec_lrg = val_metrics.get("recall/size_large/100@0.50", 0.0)

                rec_gen = val_metrics.get("recall/domain_general/100@0.50", 0.0)
                rec_auto = val_metrics.get("recall/domain_automotive/100@0.50", 0.0)
                rec_fish = val_metrics.get("recall/domain_fisheye/100@0.50", 0.0)

            val_duration = time.time() - val_start_time
            print(
                f"  -> Val ({val_duration:.1f}s) | "
                f"Recall@100@50: {rec_100_50 * 100:.1f}% | "
                f"Recall@100@75: {rec_100_75 * 100:.1f}% | "
                f"Recall@10: {rec_10_50 * 100:.1f}% | "
                f"AP/AR@50: {ap_50 * 100:.1f}%"
            )
            print(
                f"     Size: Small={rec_small * 100:.1f}%, Med={rec_med * 100:.1f}%, Lrg={rec_lrg * 100:.1f}%"
            )
            print(
                f"     Domain: General={rec_gen * 100:.1f}%, Auto={rec_auto * 100:.1f}%, Fisheye={rec_fish * 100:.1f}%"
            )

            epoch_record.update(
                {
                    "val_duration_seconds": round(val_duration, 2),
                    "recall_10_50": round(rec_10_50, 4),
                    "recall_100_50": round(rec_100_50, 4),
                    "recall_100_75": round(rec_100_75, 4),
                    "ap_50": round(ap_50, 4),
                    "recall_size_edge_small": round(rec_edge, 4),
                    "recall_size_small": round(rec_small, 4),
                    "recall_size_medium": round(rec_med, 4),
                    "recall_size_large": round(rec_lrg, 4),
                    "recall_domain_general": round(rec_gen, 4),
                    "recall_domain_automotive": round(rec_auto, 4),
                    "recall_domain_fisheye": round(rec_fish, 4),
                }
            )

            if rec_100_50 > best_recall_50:
                best_recall_50 = rec_100_50
                suffix = "_recipe2" if args.recipe == "recipe2" else ""
                best_path = checkpoints_dir / f"mobilenetv4_p2_{args.geometry}{suffix}_best.pt"
                torch.save(
                    {
                        "epoch": epoch,
                        "geometry": args.geometry,
                        "recipe": args.recipe,
                        "model_state_dict": model.state_dict(),
                        "optimizer_state_dict": optimizer.state_dict(),
                        "strides": strides,
                        "recall_100_50": best_recall_50,
                        "ap_50": ap_50,
                    },
                    best_path,
                )
                print(
                    f"     *** New Best Model Saved ({best_recall_50 * 100:.1f}% Recall@100@50) -> {best_path.name} ***"
                )

        history.append(epoch_record)

    suffix = "_recipe2" if args.recipe == "recipe2" else ""
    last_path = checkpoints_dir / f"mobilenetv4_p2_{args.geometry}{suffix}_last.pt"
    torch.save(
        {
            "epoch": args.epochs,
            "geometry": args.geometry,
            "recipe": args.recipe,
            "model_state_dict": model.state_dict(),
            "strides": strides,
            "best_recall_100_50": best_recall_50,
        },
        last_path,
    )

    total_duration = time.time() - total_start_time
    print(
        f"\n--- Training Complete ({args.geometry.upper()}, Recipe={args.recipe}) in {total_duration / 60:.1f} minutes ---"
    )
    print(f"Best Recall@100 (0.50): {best_recall_50 * 100:.1f}%")
    print(f"Checkpoints saved to: {checkpoints_dir}")

    summary_path = (
        args.output_dir / f"training_{args.geometry}_recipe2_summary.json"
        if args.recipe == "recipe2"
        else args.output_dir / f"training_{args.geometry}_summary.json"
    )
    summary = {
        "geometry": args.geometry,
        "recipe": args.recipe,
        "completed_at": datetime.now().isoformat(),
        "total_duration_seconds": round(total_duration, 2),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "input_size": args.input_size,
        "best_recall_100_50": round(best_recall_50, 4),
        "history": history,
    }
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    # Also keep general training_summary.json updated if baseline HBB
    if args.geometry == "hbb" and args.recipe == "p2_baseline":
        with open(args.output_dir / "training_summary.json", "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    train()
