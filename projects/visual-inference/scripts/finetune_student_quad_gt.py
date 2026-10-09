#!/usr/bin/env python3
"""Stage-2 Fine-tuning of MobileNetV4 Quad Proposal Detector on Human Ground-Truth annotations.

Fine-tunes the self-trained student checkpoint exclusively on high-quality human quad annotations
from the balanced 10k cohort, using Min-over-8 Corner Smooth-L1 and Gaussian Wasserstein Distance (GWD)
losses to tighten polygon proposal envelopes and calibrate confidence scores.
"""

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

from student_detector.human_gt_data import (
    DEFAULT_GT_INDEX_PATH,
    HumanGTQuadDataset,
    collate_quad_proposal_samples,
)
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import validate_quad
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_DB,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentDataset,
)
from student_detector.task_aligned_assigner import QuadTaskAlignedAssigner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage-2 Fine-Tuning of MobileNetV4 Quad Student on Human Ground Truth."
    )
    parser.add_argument(
        "--init-checkpoint",
        type=Path,
        default=Path(
            "artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt"
        ),
        help="Path to pre-trained quad student checkpoint to fine-tune.",
    )
    parser.add_argument(
        "--cohort-db",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_DB,
        help="Path to 10k sam_discovery.sqlite database.",
    )
    parser.add_argument(
        "--gt-index",
        type=Path,
        default=DEFAULT_GT_INDEX_PATH,
        help="Path to quad_train.sqlite human GT index.",
    )
    parser.add_argument(
        "--val-pseudo-labels",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_PATH,
        help="Path to validation cohort pseudo-labels (sam_pilot_800.json).",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to dataset output image root.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/finetune_10k_quad"),
        help="Output directory for checkpoints and summaries.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=5,
        help="Number of fine-tuning epochs (default: 5).",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=64,
        help="Batch size (default: 64).",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=3e-4,
        help="Peak fine-tuning learning rate (default: 3e-4).",
    )
    parser.add_argument(
        "--weight-decay",
        type=float,
        default=1e-4,
        help="AdamW weight decay (default: 1e-4).",
    )
    parser.add_argument(
        "--input-size",
        type=int,
        default=384,
        help="Input image resolution (default: 384).",
    )
    parser.add_argument(
        "--quality-weight",
        type=float,
        default=1.0,
        help="Quality Focal Loss weight (default: 1.0).",
    )
    parser.add_argument(
        "--corner-weight",
        type=float,
        default=2.0,
        help="Min-over-8 corner Smooth-L1 loss weight (default: 2.0).",
    )
    parser.add_argument(
        "--gwd-weight",
        type=float,
        default=2.0,
        help="Gaussian Wasserstein Distance (GWD) loss weight (default: 2.0).",
    )
    parser.add_argument(
        "--validity-weight",
        type=float,
        default=0.05,
        help="Quad validity/convexity loss weight (default: 0.05).",
    )
    parser.add_argument(
        "--weak-negative-weight",
        type=float,
        default=0.15,
        help="Background weighting to preserve open-world recall (default: 0.15).",
    )
    parser.add_argument(
        "--centerness-alpha",
        type=float,
        default=0.0,
        help="Exponent for centerness scoring in Quad decoder (default: 0.0).",
    )
    parser.add_argument(
        "--nms-mode",
        type=str,
        default="part_aware",
        choices=["part_aware", "nms_free", "hard", "part_aware_soft"],
        help="Inference NMS mode for validation (default: part_aware).",
    )
    parser.add_argument(
        "--part-scale-ratio",
        type=float,
        default=0.4,
        help="Part-aware NMS scale ratio threshold (default: 0.4).",
    )
    parser.add_argument(
        "--part-max-iou",
        type=float,
        default=0.65,
        help="Part-aware NMS maximum duplicate IoU threshold (default: 0.65).",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="DataLoader worker count (default: 4).",
    )
    parser.add_argument(
        "--max-val-batches",
        type=int,
        default=4,
        help="Max validation batches for intermediate epochs (default: 4, ~256 images for speed; 0 for full).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device ('cuda' or 'cpu').",
    )
    return parser.parse_args()


def extract_quad_metrics(m: dict[str, float]) -> tuple[float, float, float, float]:
    rec10 = m.get("recall/10@0.50", m.get("recall/10@50", 0.0))
    rec100 = m.get("recall/100@0.50", m.get("recall/100@50", 0.0))
    rec75 = m.get("recall/100@0.75", m.get("recall/100@75", 0.0))
    ar100 = m.get("ar/100", m.get("ap/50", rec100))
    return rec10, rec100, rec75, ar100


def finetune_quad() -> None:
    args = parse_args()
    device = torch.device(args.device)
    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Starting Stage-2 Quad Human GT Fine-Tuning"
    )
    print(
        f"Device: {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})"
    )
    print(
        f"Config: epochs={args.epochs}, batch_size={args.batch_size}, lr={args.lr}, "
        f"corner_weight={args.corner_weight}, gwd_weight={args.gwd_weight}, weak_neg={args.weak_negative_weight}"
    )
    print(f"Initial Checkpoint: {args.init_checkpoint}")

    checkpoints_dir = args.output_dir / "checkpoints"
    checkpoints_dir.mkdir(parents=True, exist_ok=True)

    # 1. Inspect Checkpoint for Architecture & Strides
    if not args.init_checkpoint.exists():
        raise FileNotFoundError(f"Initial checkpoint not found: {args.init_checkpoint}")

    print(f"Inspecting weights from {args.init_checkpoint}...")
    ckpt = torch.load(args.init_checkpoint, map_location=device)
    strides = ckpt.get("strides", (4, 8, 16, 32))
    has_aux = any("aux_head" in k for k in ckpt["model_state_dict"].keys())
    print(f"Architecture: Strides={strides}, DualHead={has_aux}")

    # 2. Datasets and Loaders
    print("Loading datasets...")
    # Train split: 90% of cohort images
    train_dataset = HumanGTQuadDataset(
        cohort_db_path=args.cohort_db,
        gt_index_path=args.gt_index,
        image_root=args.image_root,
        split="train",
        val_ratio=0.10,
        input_size=args.input_size,
    )
    # Val split on Human GT: 10% of cohort images (1,000 images)
    val_gt_dataset = HumanGTQuadDataset(
        cohort_db_path=args.cohort_db,
        gt_index_path=args.gt_index,
        image_root=args.image_root,
        split="val",
        val_ratio=0.10,
        input_size=args.input_size,
    )
    # Val split on SAM discovery pilot (open-world retention check)
    val_sam_dataset = SAMStudentDataset(
        pseudo_labels_path=args.val_pseudo_labels,
        image_root=args.image_root,
        split="val",
        val_ratio=0.20,
        input_size=args.input_size,
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_quad_proposal_samples,
        drop_last=True,
    )
    val_gt_loader = DataLoader(
        val_gt_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_quad_proposal_samples,
    )
    val_sam_loader = DataLoader(
        val_sam_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_quad_proposal_samples,
    )
    print(
        f"Train Dataset: {len(train_dataset)} images ({len(train_loader)} batches/epoch) | "
        f"Val GT: {len(val_gt_dataset)} images | Val SAM: {len(val_sam_dataset)} images"
    )

    # 3. Model Initialization
    model = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
        dual_head=has_aux,
    ).to(device)
    model.load_state_dict(ckpt["model_state_dict"])

    # 4. Assigner, Target Builder, Loss & Decoders
    if has_aux:
        assigner = QuadTaskAlignedAssigner(
            strides=strides, assignment_mode="one_to_one"
        )
        assigner_aux = QuadTaskAlignedAssigner(
            strides=strides, top_k=4, assignment_mode="one_to_many"
        )
        target_builder = QuadTargetBuilder(
            assigner, weak_negative_weight=args.weak_negative_weight
        )
        target_builder_aux = QuadTargetBuilder(
            assigner_aux, weak_negative_weight=args.weak_negative_weight
        )
    else:
        assigner = QuadAssigner(
            strides=strides,
            top_k=4,
            eligible_levels=1,
        )
        target_builder = QuadTargetBuilder(
            assigner, weak_negative_weight=args.weak_negative_weight
        )
        target_builder_aux = None

    criterion = QuadProposalLoss(
        strides=strides,
        quality_weight=args.quality_weight,
        corner_weight=args.corner_weight,
        gwd_weight=args.gwd_weight,
        validity_weight=args.validity_weight,
        quality_target_mode="blend",
        quality_blend=0.8,
        geometry_quality_target="exact_iou",
    )
    decoder = QuadInferenceDecoder(
        strides=strides,
        pre_nms_top_k=1000,
        nms_iou_threshold=0.45,
        max_proposals=100,
        centerness_alpha=args.centerness_alpha,
        nms_mode=args.nms_mode,
        part_scale_ratio=args.part_scale_ratio,
        part_max_iou=args.part_max_iou,
    )

    # 5. Optimizer & Differential Learning Rate
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
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=1e-5
    )
    scaler = torch.amp.GradScaler(device.type, enabled=(device.type == "cuda"))

    # 6. Baseline Validation (Epoch 0 - Before Fine-Tuning)
    print("\n[Baseline Evaluation - Before Fine-Tuning]")
    base_start = time.time()
    max_b = args.max_val_batches if args.max_val_batches > 0 else None
    val_base_gt = validate_quad(
        model=model,
        loader=val_gt_loader,
        target_builder=target_builder,
        decoder=decoder,
        device=device,
        max_batches=max_b,
    )
    val_base_sam = validate_quad(
        model=model,
        loader=val_sam_loader,
        target_builder=target_builder,
        decoder=decoder,
        device=device,
    )
    base_gt_rec10, base_gt_rec100, base_gt_rec75, base_gt_ap50 = extract_quad_metrics(
        val_base_gt
    )
    _, base_sam_rec100, _, _ = extract_quad_metrics(val_base_sam)
    print(
        f"Baseline ({time.time() - base_start:.1f}s) | "
        f"GT Recall@100@50: {base_gt_rec100 * 100:.2f}% | "
        f"GT Recall@100@75: {base_gt_rec75 * 100:.2f}% | "
        f"GT AR@100: {base_gt_ap50 * 100:.2f}% | "
        f"SAM Open-World Recall: {base_sam_rec100 * 100:.2f}%",
        flush=True,
    )

    history: list[dict[str, Any]] = [
        {
            "epoch": 0,
            "stage": "baseline_before_finetune",
            "gt_recall_10_50": round(base_gt_rec10, 4),
            "gt_recall_100_50": round(base_gt_rec100, 4),
            "gt_recall_100_75": round(base_gt_rec75, 4),
            "gt_ap_50": round(base_gt_ap50, 4),
            "sam_recall_100_50": round(base_sam_rec100, 4),
        }
    ]

    best_gt_recall = base_gt_rec100
    total_start_time = time.time()

    print(f"\n--- Fine-Tuning Loop Starting ({args.epochs} Epochs) ---")
    for epoch in range(1, args.epochs + 1):
        epoch_start_time = time.time()
        model.train()
        train_dataset.set_epoch(epoch)

        epoch_loss = 0.0
        epoch_qual_loss = 0.0
        epoch_corner_loss = 0.0
        epoch_gwd_loss = 0.0
        batch_count = 0

        for images, samples in train_loader:
            images = images.to(device, non_blocking=(device.type == "cuda"))
            optimizer.zero_grad()

            with torch.amp.autocast(
                device.type, dtype=torch.float16, enabled=(device.type == "cuda")
            ):
                if has_aux:
                    output_pri, output_aux = model(images)
                    feat_shapes = tuple(
                        (t.shape[-2], t.shape[-1]) for t in output_pri.quality
                    )
                    targets_pri = target_builder(
                        samples,
                        feat_shapes,
                        device=device,
                        pred_output=output_pri,
                    )
                    assert target_builder_aux is not None
                    targets_aux = target_builder_aux(
                        samples,
                        feat_shapes,
                        device=device,
                        pred_output=output_aux,
                    )
                    loss_pri = criterion(output_pri, targets_pri)
                    loss_aux = criterion(output_aux, targets_aux)
                    loss_total = loss_pri.total + loss_aux.total
                    qual_val = loss_pri.quality.item()
                    corner_val = loss_pri.corner.item()
                    gwd_val = loss_pri.gwd.item()
                else:
                    output = model(images)
                    feat_shapes = tuple(
                        (t.shape[-2], t.shape[-1]) for t in output.quality
                    )
                    targets = target_builder(
                        samples,
                        feat_shapes,
                        device=device,
                        pred_output=output,
                    )
                    loss_out = criterion(output, targets)
                    loss_total = loss_out.total
                    qual_val = loss_out.quality.item()
                    corner_val = loss_out.corner.item()
                    gwd_val = loss_out.gwd.item()

            scaler.scale(loss_total).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=10.0)
            scaler.step(optimizer)
            scaler.update()

            epoch_loss += loss_total.item()
            epoch_qual_loss += qual_val
            epoch_corner_loss += corner_val
            epoch_gwd_loss += gwd_val
            batch_count += 1

            if batch_count % 30 == 0 or batch_count == len(train_loader):
                cur_lr = scheduler.get_last_lr()[-1]
                print(
                    f"Epoch [{epoch:02d}/{args.epochs:02d}] "
                    f"Batch [{batch_count:03d}/{len(train_loader):03d}] "
                    f"Loss: {loss_total.item():.4f} "
                    f"(Qual: {qual_val:.4f}, Corner: {corner_val:.4f}, GWD: {gwd_val:.4f}) "
                    f"LR: {cur_lr:.6f}",
                    flush=True,
                )

        scheduler.step()
        epoch_duration = time.time() - epoch_start_time
        avg_loss = epoch_loss / max(batch_count, 1)
        avg_qual = epoch_qual_loss / max(batch_count, 1)
        avg_corner = epoch_corner_loss / max(batch_count, 1)
        avg_gwd = epoch_gwd_loss / max(batch_count, 1)

        # Validation pass
        val_start_time = time.time()
        val_max_b = (
            args.max_val_batches
            if (args.max_val_batches > 0 and epoch < args.epochs)
            else None
        )
        val_gt = validate_quad(
            model=model,
            loader=val_gt_loader,
            target_builder=target_builder,
            decoder=decoder,
            device=device,
            max_batches=val_max_b,
        )
        val_sam = validate_quad(
            model=model,
            loader=val_sam_loader,
            target_builder=target_builder,
            decoder=decoder,
            device=device,
        )
        gt_rec10, gt_rec100, gt_rec75, gt_ap50 = extract_quad_metrics(val_gt)
        _, sam_rec100, _, _ = extract_quad_metrics(val_sam)
        val_duration = time.time() - val_start_time

        print(
            f"Epoch {epoch:02d} Complete in {epoch_duration:.1f}s | Train Loss: {avg_loss:.4f} "
            f"(Qual: {avg_qual:.4f}, Corner: {avg_corner:.4f}, GWD: {avg_gwd:.4f}) | "
            f"GT Rec@100@50: {gt_rec100 * 100:.2f}% | GT AR@100: {gt_ap50 * 100:.2f}% | "
            f"SAM Rec@100: {sam_rec100 * 100:.2f}%",
            flush=True,
        )

        epoch_record = {
            "epoch": epoch,
            "duration_seconds": round(epoch_duration, 2),
            "lr": round(scheduler.get_last_lr()[-1], 6),
            "train_loss": round(avg_loss, 4),
            "train_qual_loss": round(avg_qual, 4),
            "train_corner_loss": round(avg_corner, 4),
            "train_gwd_loss": round(avg_gwd, 4),
            "val_duration_seconds": round(val_duration, 2),
            "gt_recall_10_50": round(gt_rec10, 4),
            "gt_recall_100_50": round(gt_rec100, 4),
            "gt_recall_100_75": round(gt_rec75, 4),
            "gt_ap_50": round(gt_ap50, 4),
            "sam_recall_100_50": round(sam_rec100, 4),
        }
        history.append(epoch_record)

        if gt_rec100 > best_gt_recall:
            best_gt_recall = gt_rec100
            best_path = checkpoints_dir / "mobilenetv4_p2_quad_finetune_best.pt"
            torch.save(
                {
                    "epoch": epoch,
                    "geometry": "quad",
                    "model_state_dict": model.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "strides": strides,
                    "recall_100_50": best_gt_recall,
                    "recall_100_75": gt_rec75,
                    "ap_50": gt_ap50,
                },
                best_path,
            )
            print(
                f"     *** New Best Fine-Tuned Model Saved ({best_gt_recall * 100:.2f}% GT Recall@100@50) -> {best_path.name} ***"
            )

    last_path = checkpoints_dir / "mobilenetv4_p2_quad_finetune_last.pt"
    torch.save(
        {
            "epoch": args.epochs,
            "geometry": "quad",
            "model_state_dict": model.state_dict(),
            "strides": strides,
            "best_recall_100_50": best_gt_recall,
        },
        last_path,
    )

    total_duration = time.time() - total_start_time
    print(f"\n--- Fine-Tuning Complete in {total_duration / 60:.1f} minutes ---")
    print(
        f"Baseline GT Recall@100@50: {base_gt_rec100 * 100:.2f}% -> Best Fine-Tuned: {best_gt_recall * 100:.2f}%"
    )
    print(f"Checkpoints saved to: {checkpoints_dir}")

    summary_path = args.output_dir / "training_quad_finetune_summary.json"
    summary = {
        "geometry": "quad",
        "stage": "stage2_human_gt_finetuning",
        "init_checkpoint": str(args.init_checkpoint),
        "completed_at": datetime.now().isoformat(),
        "total_duration_seconds": round(total_duration, 2),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "input_size": args.input_size,
        "baseline_gt_recall_100_50": round(base_gt_rec100, 4),
        "best_gt_recall_100_50": round(best_gt_recall, 4),
        "history": history,
    }
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    finetune_quad()
