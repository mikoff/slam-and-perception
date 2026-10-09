#!/usr/bin/env python3
"""Stage-2 Fine-tuning of MobileNetV4 Student Proposal Detector on Human Ground-Truth annotations.

Unified fine-tuning runner for both Horizontal Bounding Boxes (HBB) and Oriented Quadrilaterals (Quads).
Fine-tunes the 15-epoch self-trained student checkpoint exclusively on high-quality human GT annotations
from the balanced 10k cohort, tightening proposal boundary envelopes and calibrating confidence scores.
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
    HumanGTQuadDataset,
)
from student_detector.losses import ProposalLoss
from student_detector.model import QuadProposalDetector, StudentDetector
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
    SAMStudentHBBDataset,
    collate_proposal_samples,
    collate_quad_proposal_samples,
)
from student_detector.targets import TargetBuilder
from student_detector.training import validate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage-2 Fine-Tuning of MobileNetV4 Student on Human Ground Truth (HBB or Quad)."
    )
    parser.add_argument(
        "--geometry",
        type=str,
        choices=["hbb", "quad"],
        default="hbb",
        help="Model geometry: 'hbb' (horizontal boxes) or 'quad' (oriented quads) (default: 'hbb').",
    )
    parser.add_argument(
        "--init-checkpoint",
        type=Path,
        default=None,
        help="Path to pre-trained student checkpoint to fine-tune (defaults to best Stage-1 checkpoint).",
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
        default=None,
        help="Output directory for checkpoints and summaries (defaults to artifacts/finetune_10k_{geometry}).",
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
        help="CIoU loss weight for HBB (default: 2.0).",
    )
    parser.add_argument(
        "--ltrb-weight",
        type=float,
        default=0.5,
        help="Smooth-L1 coordinate regression weight for HBB envelope tightening (default: 0.5).",
    )
    parser.add_argument(
        "--corner-weight",
        type=float,
        default=2.0,
        help="Corner Smooth-L1 loss weight for Quad (default: 2.0).",
    )
    parser.add_argument(
        "--gwd-weight",
        type=float,
        default=2.0,
        help="Gaussian Wasserstein Distance (GWD) loss weight for Quad (default: 2.0).",
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
        default=None,
        help="Exponent for centerness scoring in decoder (defaults to 1.0 for HBB, 0.0 for Quad).",
    )
    parser.add_argument(
        "--score-mode",
        type=str,
        default="objectness_x_centerness",
        choices=["objectness", "objectness_x_centerness"],
        help="Scoring mode for HBB decoder (default: objectness_x_centerness).",
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
        "--max-val-batches",
        type=int,
        default=4,
        help="Max validation batches for intermediate epochs (default: 4 for speed, full on final).",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="DataLoader worker count (default: 4).",
    )
    parser.add_argument(
        "--cohort-size",
        type=int,
        default=10000,
        help="Cohort size limit (default: 10000 for standard 10k dataset; set 0 for all DB images).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device ('cuda' or 'cpu').",
    )
    return parser.parse_args()


def extract_metrics(val: dict[str, float]) -> dict[str, float]:
    """Normalize metric dictionary keys across HBB and Quad validators."""

    def get_first(*keys: str, default: float = 0.0) -> float:
        for k in keys:
            if k in val:
                return float(val[k])
        return default

    return {
        "recall_10_50": get_first("recall/10@50", "recall/10@0.50"),
        "recall_100_50": get_first("recall/100@50", "recall/100@0.50"),
        "recall_100_75": get_first("recall/100@75", "recall/100@0.75"),
        "ap_50": get_first("ap/50", "ap/0.50", "ar/100"),
        "recall_size_small": get_first(
            "recall/size_small/100@50", "recall/size_small/100@0.50"
        ),
        "recall_size_medium": get_first(
            "recall/size_medium/100@50", "recall/size_medium/100@0.50"
        ),
        "recall_size_large": get_first(
            "recall/size_large/100@50", "recall/size_large/100@0.50"
        ),
        "recall_domain_general": get_first(
            "recall/domain_general/100@50",
            "recall/domain_general/100@0.50",
            "recall/general/100@50",
            "recall/general/100@0.50",
        ),
        "recall_domain_automotive": get_first(
            "recall/domain_automotive/100@50",
            "recall/domain_automotive/100@0.50",
            "recall/automotive/100@50",
            "recall/automotive/100@0.50",
        ),
        "recall_domain_fisheye": get_first(
            "recall/domain_fisheye/100@50",
            "recall/domain_fisheye/100@0.50",
            "recall/fisheye/100@50",
            "recall/fisheye/100@0.50",
        ),
    }


def finetune() -> None:
    args = parse_args()
    device = torch.device(args.device)

    # Geometry-dependent defaults
    if args.init_checkpoint is None:
        args.init_checkpoint = Path(
            f"artifacts/train_10k_{args.geometry}/checkpoints/mobilenetv4_p2_{args.geometry}_best.pt"
        )
    if args.output_dir is None:
        args.output_dir = Path(f"artifacts/finetune_10k_{args.geometry}")
    if args.centerness_alpha is None:
        args.centerness_alpha = 1.0 if args.geometry == "hbb" else 0.0

    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Starting Stage-2 Human GT Fine-Tuning ({args.geometry.upper()})"
    )
    print(
        f"Device: {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})"
    )
    print(
        f"Config: geometry={args.geometry}, epochs={args.epochs}, batch_size={args.batch_size}, lr={args.lr}, "
        f"weak_neg={args.weak_negative_weight}, centerness_alpha={args.centerness_alpha}"
    )
    print(f"Initial Checkpoint: {args.init_checkpoint}")

    checkpoints_dir = args.output_dir / "checkpoints"
    checkpoints_dir.mkdir(parents=True, exist_ok=True)
    strides = (4, 8, 16, 32)

    # 1. Datasets and Loaders
    print("Loading datasets...")
    cohort_size = args.cohort_size if args.cohort_size > 0 else None
    if args.geometry == "hbb":
        train_dataset = HumanGTHBBDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="train",
            val_ratio=0.10,
            input_size=args.input_size,
            cohort_size=cohort_size,
        )
        val_gt_dataset = HumanGTHBBDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="val",
            val_ratio=0.10,
            input_size=args.input_size,
            cohort_size=cohort_size,
        )
        val_sam_dataset = SAMStudentHBBDataset(
            pseudo_labels_path=args.val_pseudo_labels,
            image_root=args.image_root,
            split="val",
            val_ratio=0.20,
            input_size=args.input_size,
        )
        collate_fn = collate_proposal_samples
    else:
        train_dataset = HumanGTQuadDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="train",
            val_ratio=0.10,
            input_size=args.input_size,
            cohort_size=cohort_size,
        )
        val_gt_dataset = HumanGTQuadDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            split="val",
            val_ratio=0.10,
            input_size=args.input_size,
            cohort_size=cohort_size,
        )
        val_sam_dataset = SAMStudentDataset(
            pseudo_labels_path=args.val_pseudo_labels,
            image_root=args.image_root,
            split="val",
            val_ratio=0.20,
            input_size=args.input_size,
        )
        collate_fn = collate_quad_proposal_samples

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
        drop_last=True,
    )
    val_gt_loader = DataLoader(
        val_gt_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
    )
    val_sam_loader = DataLoader(
        val_sam_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=(device.type == "cuda"),
        collate_fn=collate_fn,
    )
    print(
        f"Train Dataset: {len(train_dataset)} images ({len(train_loader)} batches/epoch) | "
        f"Val GT: {len(val_gt_dataset)} images | Val SAM: {len(val_sam_dataset)} images"
    )

    # 2. Model Initialization & Pre-trained Weights Loading
    if not args.init_checkpoint.exists():
        raise FileNotFoundError(f"Initial checkpoint not found: {args.init_checkpoint}")

    print(f"Loading weights from {args.init_checkpoint}...")
    ckpt = torch.load(args.init_checkpoint, map_location=device)
    has_aux = any("aux_head" in k for k in ckpt["model_state_dict"].keys())

    if args.geometry == "hbb":
        model = StudentDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=has_aux,
        ).to(device)
    else:
        model = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=has_aux,
        ).to(device)

    model.load_state_dict(ckpt["model_state_dict"])

    # 3. Assigner, Target Builder, Loss & Decoders
    if args.geometry == "hbb":
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
        val_fn = validate
    else:
        assigner = QuadAssigner(
            strides=strides,
            top_k=4,
            eligible_levels=1,
        )
        target_builder = QuadTargetBuilder(
            assigner, weak_negative_weight=args.weak_negative_weight
        )
        criterion = QuadProposalLoss(
            strides=strides,
            corner_weight=args.corner_weight,
            gwd_weight=args.gwd_weight,
            validity_weight=0.05,
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
        val_fn = validate_quad

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

    # Helper for validation with batch limits
    def run_eval(loader: DataLoader, limit_batches: int | None = None) -> dict[str, float]:
        if limit_batches is None or limit_batches >= len(loader):
            return val_fn(
                model=model,
                loader=loader,
                target_builder=target_builder,
                decoder=decoder,
                device=device,
            )
        # Sliced loader for fast intermediate validation
        sliced_samples = []
        for i, batch in enumerate(loader):
            if i >= limit_batches:
                break
            sliced_samples.append(batch)

        class SlicedLoader:
            def __init__(self, data: list[Any]) -> None:
                self.data = data

            def __iter__(self):
                return iter(self.data)

            def __len__(self) -> int:
                return len(self.data)

        return val_fn(
            model=model,
            loader=SlicedLoader(sliced_samples),
            target_builder=target_builder,
            decoder=decoder,
            device=device,
        )

    # 5. Evaluate Baseline (Epoch 0 - Before Fine-Tuning)
    print("\n[Baseline Evaluation - Before Fine-Tuning]")
    base_start = time.time()
    val_base_gt = run_eval(val_gt_loader)
    val_base_sam = run_eval(val_sam_loader)
    base_gt_m = extract_metrics(val_base_gt)
    base_sam_m = extract_metrics(val_base_sam)
    base_gt_rec = base_gt_m["recall_100_50"]
    base_sam_rec = base_sam_m["recall_100_50"]

    print(
        f"Baseline ({time.time() - base_start:.1f}s) | "
        f"GT Recall@100@50: {base_gt_rec * 100:.2f}% | "
        f"GT Recall@100@75: {base_gt_m['recall_100_75'] * 100:.2f}% | "
        f"GT AP/AR@50: {base_gt_m['ap_50'] * 100:.2f}% | "
        f"SAM Retention: {base_sam_rec * 100:.2f}%"
    )
    print(
        f"  GT Size: Small={base_gt_m['recall_size_small'] * 100:.1f}%, Med={base_gt_m['recall_size_medium'] * 100:.1f}%, "
        f"Lrg={base_gt_m['recall_size_large'] * 100:.1f}%"
    )
    print(
        f"  GT Domain: General={base_gt_m['recall_domain_general'] * 100:.1f}%, Auto={base_gt_m['recall_domain_automotive'] * 100:.1f}%, "
        f"Fisheye={base_gt_m['recall_domain_fisheye'] * 100:.1f}%"
    )

    history: list[dict[str, Any]] = [
        {
            "epoch": 0,
            "stage": "baseline_before_finetune",
            "gt_recall_10_50": round(base_gt_m["recall_10_50"], 4),
            "gt_recall_100_50": round(base_gt_rec, 4),
            "gt_recall_100_75": round(base_gt_m["recall_100_75"], 4),
            "gt_ap_50": round(base_gt_m["ap_50"], 4),
            "sam_recall_100_50": round(base_sam_rec, 4),
            **base_gt_m,
        }
    ]

    best_gt_recall = base_gt_rec
    total_start_time = time.time()

    print(f"\n--- Fine-Tuning Loop Starting ({args.epochs} Epochs) ---")
    for epoch in range(1, args.epochs + 1):
        epoch_start_time = time.time()
        model.train()
        train_dataset.set_epoch(epoch)

        epoch_loss = 0.0
        batch_count = 0

        for images, samples in train_loader:
            images = images.to(device, non_blocking=(device.type == "cuda"))
            optimizer.zero_grad()

            with torch.amp.autocast(
                device.type, dtype=torch.float16, enabled=(device.type == "cuda")
            ):
                output = model(images)
                if has_aux and isinstance(output, tuple):
                    primary_out, aux_out = output
                    f_shapes = tuple(
                        (t.shape[-2], t.shape[-1])
                        for t in (
                            primary_out.objectness
                            if args.geometry == "hbb"
                            else primary_out.quality
                        )
                    )
                    targets = target_builder(
                        samples, f_shapes, device=device, pred_output=primary_out
                    )
                    loss_output = criterion(primary_out, targets)
                else:
                    f_shapes = tuple(
                        (t.shape[-2], t.shape[-1])
                        for t in (
                            output.objectness
                            if args.geometry == "hbb"
                            else output.quality
                        )
                    )
                    targets = target_builder(
                        samples, f_shapes, device=device, pred_output=output
                    )
                    loss_output = criterion(output, targets)

                loss_total = loss_output.total

            scaler.scale(loss_total).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=10.0)
            scaler.step(optimizer)
            scaler.update()

            epoch_loss += loss_total.item()
            batch_count += 1

            if batch_count % 30 == 0 or batch_count == len(train_loader):
                cur_lr = scheduler.get_last_lr()[0]
                detail_str = ""
                if hasattr(loss_output, "quality"):
                    detail_str = f" (Q: {loss_output.quality.item():.4f}, Corn: {loss_output.corner.item():.4f}, GWD: {loss_output.gwd.item():.4f})"
                elif hasattr(loss_output, "objectness"):
                    detail_str = f" (Obj: {loss_output.objectness.item():.4f}, CIoU: {loss_output.box_ciou.item():.4f}, L1: {loss_output.box_ltrb.item():.4f})"
                print(
                    f"Epoch [{epoch:02d}/{args.epochs:02d}] "
                    f"Batch [{batch_count:03d}/{len(train_loader):03d}] "
                    f"Loss: {loss_total.item():.4f}{detail_str} "
                    f"LR: {cur_lr:.6f}"
                )

        scheduler.step()
        epoch_duration = time.time() - epoch_start_time
        avg_loss = epoch_loss / max(batch_count, 1)

        # Validation (Full pass on Val GT 1000 images ~15s and SAM 160 images ~2s)
        val_start_time = time.time()
        val_gt_metrics = run_eval(val_gt_loader)
        val_sam_metrics = run_eval(val_sam_loader)
        val_m = extract_metrics(val_gt_metrics)
        sam_m = extract_metrics(val_sam_metrics)
        rec_100_50 = val_m["recall_100_50"]
        sam_rec = sam_m["recall_100_50"]
        val_duration = time.time() - val_start_time

        gain_str = f" ({(rec_100_50 - base_gt_rec) * 100:+.2f}%)" if rec_100_50 != base_gt_rec else ""
        print(
            f"Epoch {epoch:02d} Complete in {epoch_duration:.1f}s | Train Loss: {avg_loss:.4f} | "
            f"GT Recall@100@50: {rec_100_50 * 100:.2f}%{gain_str} | "
            f"GT Recall@100@75: {val_m['recall_100_75'] * 100:.2f}% | "
            f"SAM Retention: {sam_rec * 100:.2f}%"
        )
        print(
            f"     GT Size: Small={val_m['recall_size_small'] * 100:.1f}%, Med={val_m['recall_size_medium'] * 100:.1f}%, "
            f"Lrg={val_m['recall_size_large'] * 100:.1f}%"
        )
        print(
            f"     GT Domain: General={val_m['recall_domain_general'] * 100:.1f}%, Auto={val_m['recall_domain_automotive'] * 100:.1f}%, "
            f"Fisheye={val_m['recall_domain_fisheye'] * 100:.1f}%"
        )

        epoch_record = {
            "epoch": epoch,
            "duration_seconds": round(epoch_duration, 2),
            "lr": round(scheduler.get_last_lr()[0], 6),
            "train_loss": round(avg_loss, 4),
            "val_duration_seconds": round(val_duration, 2),
            "gt_recall_10_50": round(val_m["recall_10_50"], 4),
            "gt_recall_100_50": round(rec_100_50, 4),
            "gt_recall_100_75": round(val_m["recall_100_75"], 4),
            "gt_ap_50": round(val_m["ap_50"], 4),
            "sam_recall_100_50": round(sam_rec, 4),
            **val_m,
        }
        history.append(epoch_record)

        if rec_100_50 > best_gt_recall:
            best_gt_recall = rec_100_50
            best_path = (
                checkpoints_dir / f"mobilenetv4_p2_{args.geometry}_finetune_best.pt"
            )
            torch.save(
                {
                    "epoch": epoch,
                    "geometry": args.geometry,
                    "model_state_dict": model.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "strides": strides,
                    "recall_100_50": best_gt_recall,
                    "gt_recall_100_50": best_gt_recall,
                    "recall_100_75": val_m["recall_100_75"],
                    "ap_50": val_m["ap_50"],
                    "sam_recall_100_50": sam_rec,
                },
                best_path,
            )
            print(
                f"     *** New Best Fine-Tuned Model Saved ({best_gt_recall * 100:.2f}% GT Recall@100@50) -> {best_path.name} ***"
            )

    last_path = checkpoints_dir / f"mobilenetv4_p2_{args.geometry}_finetune_last.pt"
    torch.save(
        {
            "epoch": args.epochs,
            "geometry": args.geometry,
            "model_state_dict": model.state_dict(),
            "strides": strides,
            "best_recall_100_50": best_gt_recall,
            "best_gt_recall_100_50": best_gt_recall,
        },
        last_path,
    )

    best_path = checkpoints_dir / f"mobilenetv4_p2_{args.geometry}_finetune_best.pt"
    if not best_path.exists():
        import shutil

        shutil.copyfile(last_path, best_path)
        print(f"     *** Created {best_path.name} from final epoch checkpoint ***")

    total_duration = time.time() - total_start_time
    print(f"\n--- Fine-Tuning Complete in {total_duration / 60:.1f} minutes ---")
    print(
        f"Baseline GT Recall@100@50: {base_gt_rec * 100:.2f}% -> Best Fine-Tuned: {best_gt_recall * 100:.2f}%"
    )
    print(f"Checkpoints saved to: {checkpoints_dir}")

    summary_path = args.output_dir / f"training_{args.geometry}_finetune_summary.json"
    summary = {
        "geometry": args.geometry,
        "stage": "stage2_human_gt_finetuning",
        "init_checkpoint": str(args.init_checkpoint),
        "completed_at": datetime.now().isoformat(),
        "total_duration_seconds": round(total_duration, 2),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "input_size": args.input_size,
        "baseline_gt_recall_100_50": round(base_gt_rec, 4),
        "best_gt_recall_100_50": round(best_gt_recall, 4),
        "baseline_recall_100_50": round(base_gt_rec, 4),
        "best_recall_100_50": round(best_gt_recall, 4),
        "history": history,
    }
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    finetune()
