#!/usr/bin/env python3
"""Stage-2 Fine-tuning of MobileNetV4 Student Proposal Detector on Human Ground-Truth boxes.

Fine-tunes the 15-epoch self-trained checkpoint exclusively on high-quality human annotations
from the balanced 10k cohort, using joint LTRB (L1) and CIoU losses to tighten proposal envelopes.
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

from student_detector.assigner import ATSSAssigner
from student_detector.decoder import InferenceDecoder
from student_detector.human_gt_data import (
    DEFAULT_GT_INDEX_PATH,
    HumanGTHBBDataset,
)
from student_detector.losses import ProposalLoss
from student_detector.model import StudentDetector
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_DB,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentHBBDataset,
    collate_proposal_samples,
)
from student_detector.targets import TargetBuilder
from student_detector.training import validate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage-2 Fine-Tuning of MobileNetV4 Student on Human Ground Truth."
    )
    parser.add_argument(
        "--init-checkpoint",
        type=Path,
        default=Path("artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt"),
        help="Path to pre-trained student checkpoint to fine-tune.",
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
        default=Path("artifacts/finetune_10k_hbb"),
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
        "--box-weight",
        type=float,
        default=2.0,
        help="CIoU loss weight (default: 2.0).",
    )
    parser.add_argument(
        "--ltrb-weight",
        type=float,
        default=0.5,
        help="Smooth-L1 coordinate regression weight for envelope tightening (default: 0.5).",
    )
    parser.add_argument(
        "--weak-negative-weight",
        type=float,
        default=0.15,
        help="Background weighting to preserve open-world recall (default: 0.15).",
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
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device ('cuda' or 'cpu').",
    )
    return parser.parse_args()


def finetune() -> None:
    args = parse_args()
    device = torch.device(args.device)
    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Starting Stage-2 Human GT Fine-Tuning"
    )
    print(
        f"Device: {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})"
    )
    print(
        f"Config: epochs={args.epochs}, batch_size={args.batch_size}, lr={args.lr}, "
        f"box_weight={args.box_weight}, ltrb_weight={args.ltrb_weight}, weak_neg={args.weak_negative_weight}"
    )
    print(f"Initial Checkpoint: {args.init_checkpoint}")

    checkpoints_dir = args.output_dir / "checkpoints"
    checkpoints_dir.mkdir(parents=True, exist_ok=True)
    strides = (4, 8, 16, 32)

    # 1. Datasets and Loaders
    print("Loading datasets...")
    train_dataset = HumanGTHBBDataset(
        cohort_db_path=args.cohort_db,
        gt_index_path=args.gt_index,
        image_root=args.image_root,
        split="all",
        input_size=args.input_size,
    )
    val_dataset = SAMStudentHBBDataset(
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
        collate_fn=collate_proposal_samples,
        drop_last=True,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_proposal_samples,
    )
    print(
        f"Train Dataset: {len(train_dataset)} images | Val Dataset: {len(val_dataset)} images | "
        f"{len(train_loader)} batches/epoch"
    )

    # 2. Model Initialization & Pre-trained Weights Loading
    model = StudentDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
    ).to(device)

    if not args.init_checkpoint.exists():
        raise FileNotFoundError(f"Initial checkpoint not found: {args.init_checkpoint}")

    print(f"Loading weights from {args.init_checkpoint}...")
    ckpt = torch.load(args.init_checkpoint, map_location=device)
    model.load_state_dict(ckpt["model_state_dict"])

    # 3. Assigner, Target Builder, Loss & Decoders
    assigner = ATSSAssigner(
        strides=strides,
        prior_sizes=(32, 64, 128, 256),
        top_k=4,
        center_radius=1.0,
    )
    target_builder = TargetBuilder(
        assigner, default_background_weight=args.weak_negative_weight
    )
    criterion = ProposalLoss(
        strides=strides,
        objectness_loss="quality_focal",
        box_loss="ciou",
        box_weight=args.box_weight,
        ltrb_weight=args.ltrb_weight,
        centerness_weight=1.0,
        objectness_normalization="num_positives",
    )
    decoder = InferenceDecoder(
        strides=strides,
        top_k=1000,
        nms_iou_threshold=0.45,
        max_detections=100,
        score_mode=args.score_mode,
        centerness_alpha=args.centerness_alpha,
        nms_mode=args.nms_mode,
        part_scale_ratio=args.part_scale_ratio,
        part_max_iou=args.part_max_iou,
    )

    # 4. Optimizer and Schedule
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.lr,
        weight_decay=args.weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=1e-5
    )
    scaler = torch.amp.GradScaler(device.type, enabled=(device.type == "cuda"))

    # 5. Evaluate Baseline (Epoch 0 - Before Fine-Tuning)
    print("\n[Baseline Evaluation - Before Fine-Tuning]")
    base_start = time.time()
    val_base = validate(
        model=model,
        loader=val_loader,
        target_builder=target_builder,
        decoder=decoder,
        device=device,
    )
    base_rec_100_50 = val_base.get("recall/100@50", 0.0)
    base_rec_100_75 = val_base.get("recall/100@75", 0.0)
    base_rec_10 = val_base.get("recall/10@50", 0.0)
    base_ap_50 = val_base.get("ap/50", 0.0)
    base_rec_small = val_base.get("recall/size_small/100@50", 0.0)
    base_rec_med = val_base.get("recall/size_medium/100@50", 0.0)
    base_rec_lrg = val_base.get("recall/size_large/100@50", 0.0)
    base_rec_gen = val_base.get("recall/domain_general/100@50", 0.0)
    base_rec_auto = val_base.get("recall/domain_automotive/100@50", 0.0)
    base_rec_fish = val_base.get("recall/domain_fisheye/100@50", 0.0)
    print(
        f"Baseline ({time.time() - base_start:.1f}s) | "
        f"Recall@100@50: {base_rec_100_50 * 100:.2f}% | "
        f"Recall@100@75: {base_rec_100_75 * 100:.2f}% | "
        f"Recall@10: {base_rec_10 * 100:.2f}% | "
        f"AP@50: {base_ap_50 * 100:.2f}%"
    )
    print(
        f"  Size: Small={base_rec_small * 100:.1f}%, Med={base_rec_med * 100:.1f}%, Lrg={base_rec_lrg * 100:.1f}%"
    )
    print(
        f"  Domain: General={base_rec_gen * 100:.1f}%, Auto={base_rec_auto * 100:.1f}%, Fisheye={base_rec_fish * 100:.1f}%"
    )

    history: list[dict[str, Any]] = [
        {
            "epoch": 0,
            "stage": "baseline_before_finetune",
            "recall_10_50": round(base_rec_10, 4),
            "recall_100_50": round(base_rec_100_50, 4),
            "recall_100_75": round(base_rec_100_75, 4),
            "ap_50": round(base_ap_50, 4),
            "recall_size_small": round(base_rec_small, 4),
            "recall_size_medium": round(base_rec_med, 4),
            "recall_size_large": round(base_rec_lrg, 4),
            "recall_domain_general": round(base_rec_gen, 4),
            "recall_domain_automotive": round(base_rec_auto, 4),
            "recall_domain_fisheye": round(base_rec_fish, 4),
        }
    ]

    best_recall_50 = base_rec_100_50
    total_start_time = time.time()

    print(f"\n--- Fine-Tuning Loop Starting ({args.epochs} Epochs) ---")
    for epoch in range(1, args.epochs + 1):
        epoch_start_time = time.time()
        model.train()
        train_dataset.set_epoch(epoch)

        epoch_loss = 0.0
        epoch_obj_loss = 0.0
        epoch_ciou_loss = 0.0
        epoch_ltrb_loss = 0.0
        batch_count = 0

        for images, samples in train_loader:
            images = images.to(device, non_blocking=(device.type == "cuda"))
            optimizer.zero_grad()

            with torch.amp.autocast(
                device.type, dtype=torch.float16, enabled=(device.type == "cuda")
            ):
                output = model(images)
                feature_shapes = tuple(
                    (t.shape[-2], t.shape[-1]) for t in output.objectness
                )
                targets = target_builder(
                    samples,
                    feature_shapes,
                    device=device,
                    pred_output=output,
                )
                loss_output = criterion(output, targets)
                loss_total = loss_output.total
                obj_val = loss_output.objectness.item()
                ciou_val = loss_output.box_ciou.item()
                ltrb_val = loss_output.box_ltrb.item()

            scaler.scale(loss_total).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=10.0)
            scaler.step(optimizer)
            scaler.update()

            epoch_loss += loss_total.item()
            epoch_obj_loss += obj_val
            epoch_ciou_loss += ciou_val
            epoch_ltrb_loss += ltrb_val
            batch_count += 1

            if batch_count % 30 == 0 or batch_count == len(train_loader):
                cur_lr = scheduler.get_last_lr()[0]
                print(
                    f"Epoch [{epoch:02d}/{args.epochs:02d}] "
                    f"Batch [{batch_count:03d}/{len(train_loader):03d}] "
                    f"Loss: {loss_total.item():.4f} "
                    f"(Obj: {obj_val:.4f}, CIoU: {ciou_val:.4f}, L1: {ltrb_val:.4f}) "
                    f"LR: {cur_lr:.6f}"
                )

        scheduler.step()
        epoch_duration = time.time() - epoch_start_time
        avg_loss = epoch_loss / max(batch_count, 1)
        avg_obj = epoch_obj_loss / max(batch_count, 1)
        avg_ciou = epoch_ciou_loss / max(batch_count, 1)
        avg_ltrb = epoch_ltrb_loss / max(batch_count, 1)

        # Validation
        val_start_time = time.time()
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
        rec_small = val_metrics.get("recall/size_small/100@50", 0.0)
        rec_med = val_metrics.get("recall/size_medium/100@50", 0.0)
        rec_lrg = val_metrics.get("recall/size_large/100@50", 0.0)
        rec_gen = val_metrics.get("recall/domain_general/100@50", 0.0)
        rec_auto = val_metrics.get("recall/domain_automotive/100@50", 0.0)
        rec_fish = val_metrics.get("recall/domain_fisheye/100@50", 0.0)
        val_duration = time.time() - val_start_time

        print(
            f"Epoch {epoch:02d} Complete in {epoch_duration:.1f}s | Train Loss: {avg_loss:.4f} "
            f"(Obj: {avg_obj:.4f}, CIoU: {avg_ciou:.4f}, L1: {avg_ltrb:.4f}) | "
            f"Val Recall@100@50: {rec_100_50 * 100:.2f}% | "
            f"Recall@100@75: {rec_100_75 * 100:.2f}% | "
            f"AP@50: {ap_50 * 100:.2f}%"
        )
        print(
            f"     Size: Small={rec_small * 100:.1f}%, Med={rec_med * 100:.1f}%, Lrg={rec_lrg * 100:.1f}%"
        )
        print(
            f"     Domain: General={rec_gen * 100:.1f}%, Auto={rec_auto * 100:.1f}%, Fisheye={rec_fish * 100:.1f}%"
        )

        epoch_record = {
            "epoch": epoch,
            "duration_seconds": round(epoch_duration, 2),
            "lr": round(scheduler.get_last_lr()[0], 6),
            "train_loss": round(avg_loss, 4),
            "train_obj_loss": round(avg_obj, 4),
            "train_ciou_loss": round(avg_ciou, 4),
            "train_ltrb_loss": round(avg_ltrb, 4),
            "val_duration_seconds": round(val_duration, 2),
            "recall_10_50": round(rec_10_50, 4),
            "recall_100_50": round(rec_100_50, 4),
            "recall_100_75": round(rec_100_75, 4),
            "ap_50": round(ap_50, 4),
            "recall_size_small": round(rec_small, 4),
            "recall_size_medium": round(rec_med, 4),
            "recall_size_large": round(rec_lrg, 4),
            "recall_domain_general": round(rec_gen, 4),
            "recall_domain_automotive": round(rec_auto, 4),
            "recall_domain_fisheye": round(rec_fish, 4),
        }
        history.append(epoch_record)

        if rec_100_50 > best_recall_50:
            best_recall_50 = rec_100_50
            best_path = checkpoints_dir / "mobilenetv4_p2_hbb_finetune_best.pt"
            torch.save(
                {
                    "epoch": epoch,
                    "geometry": "hbb",
                    "model_state_dict": model.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "strides": strides,
                    "recall_100_50": best_recall_50,
                    "recall_100_75": rec_100_75,
                    "ap_50": ap_50,
                },
                best_path,
            )
            print(
                f"     *** New Best Fine-Tuned Model Saved ({best_recall_50 * 100:.2f}% Recall@100@50) -> {best_path.name} ***"
            )

    last_path = checkpoints_dir / "mobilenetv4_p2_hbb_finetune_last.pt"
    torch.save(
        {
            "epoch": args.epochs,
            "geometry": "hbb",
            "model_state_dict": model.state_dict(),
            "strides": strides,
            "best_recall_100_50": best_recall_50,
        },
        last_path,
    )

    total_duration = time.time() - total_start_time
    print(f"\n--- Fine-Tuning Complete in {total_duration / 60:.1f} minutes ---")
    print(
        f"Baseline Recall@100@50: {base_rec_100_50 * 100:.2f}% -> Best Fine-Tuned: {best_recall_50 * 100:.2f}%"
    )
    print(f"Checkpoints saved to: {checkpoints_dir}")

    summary_path = args.output_dir / "training_hbb_finetune_summary.json"
    summary = {
        "geometry": "hbb",
        "stage": "stage2_human_gt_finetuning",
        "init_checkpoint": str(args.init_checkpoint),
        "completed_at": datetime.now().isoformat(),
        "total_duration_seconds": round(total_duration, 2),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "input_size": args.input_size,
        "baseline_recall_100_50": round(base_rec_100_50, 4),
        "best_recall_100_50": round(best_recall_50, 4),
        "history": history,
    }
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    finetune()
