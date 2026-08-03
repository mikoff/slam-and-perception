"""Train the class-agnostic quadrilateral proposal detector."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from student_detector.config import load_phase3_config
from student_detector.data import DomainMixtureBatchSampler, select_source_mixture_indices
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import train_quad_proposals
from student_detector.training import set_reproducibility_seed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/phase3.yaml"))
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--max-val-batches", type=int)
    parser.add_argument("--epochs", type=int)
    parser.add_argument("--workers", type=int)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--batches-per-epoch", type=int)
    parser.add_argument("--overfit-images", type=int)
    parser.add_argument(
        "--overfit-image-ids",
        help="Comma-separated fixed training image IDs for a reproducible micro-overfit",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--log-interval", type=int, default=10)
    parser.add_argument("--no-pretrained", action="store_true")
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument(
        "--geometry-quality-target",
        choices=("exact_iou", "corner_proxy"),
    )
    parser.add_argument(
        "--gwd-weight",
        type=float,
        help="Override the non-negative GWD auxiliary weight for an ablation",
    )
    parser.add_argument(
        "--corner-smooth-l1-beta",
        type=float,
        help="Override the positive Smooth-L1 transition for corner regression",
    )
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--validation-interval", type=int, default=1)
    parser.add_argument("--force-index", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    overfit_image_ids = (
        tuple(int(value) for value in args.overfit_image_ids.split(",") if value)
        if args.overfit_image_ids
        else ()
    )
    overfit = args.overfit_images is not None or bool(overfit_image_ids)
    config = load_phase3_config(args.config)
    if args.epochs is not None or args.no_amp:
        config = replace(
            config,
            schedule=replace(
                config.schedule,
                epochs=args.epochs if args.epochs is not None else config.schedule.epochs,
                amp=False if args.no_amp else config.schedule.amp,
            ),
        )
    if args.workers is not None or args.batch_size is not None:
        config = replace(config, data=replace(
            config.data,
            workers=args.workers if args.workers is not None else config.data.workers,
            batch_size=args.batch_size if args.batch_size is not None else config.data.batch_size,
        ))
    if args.output_dir is not None:
        config = replace(config, output_dir=args.output_dir.resolve())
    if args.no_pretrained:
        config = replace(config, pretrained_backbone=False)
    if (
        args.geometry_quality_target is not None
        or args.gwd_weight is not None
        or args.corner_smooth_l1_beta is not None
    ):
        if args.gwd_weight is not None and args.gwd_weight < 0:
            raise ValueError("--gwd-weight must be non-negative")
        if args.corner_smooth_l1_beta is not None and args.corner_smooth_l1_beta <= 0:
            raise ValueError("--corner-smooth-l1-beta must be positive")
        config = replace(
            config,
            quad=replace(
                config.quad,
                geometry_quality_target=(
                    args.geometry_quality_target
                    if args.geometry_quality_target is not None
                    else config.quad.geometry_quality_target
                ),
                gwd_weight=(
                    args.gwd_weight
                    if args.gwd_weight is not None
                    else config.quad.gwd_weight
                ),
                corner_smooth_l1_beta=(
                    args.corner_smooth_l1_beta
                    if args.corner_smooth_l1_beta is not None
                    else config.quad.corner_smooth_l1_beta
                ),
            ),
        )
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    # Model construction initializes the proposal head, so seed before creating
    # the dataset, sampler, or model. Seeding only inside the training loop made
    # nominally identical runs start from different heads.
    set_reproducibility_seed(config.schedule.seed)
    train_dataset = QuadProposalDataset(
        config.data.quad_train_annotations or config.data.train_annotations,
        config.data.image_root, config.data.index_dir / "quad_train.sqlite",
        config.data, config.augmentation, training=not overfit,
        seed=config.schedule.seed,
        force_index=args.force_index,
    )
    val_annotations = (
        config.data.quad_train_annotations or config.data.train_annotations
        if overfit
        else config.data.quad_val_annotations or config.data.val_annotations
    )
    val_index = "quad_train.sqlite" if overfit else "quad_val.sqlite"
    val_dataset = QuadProposalDataset(
        val_annotations,
        config.data.image_root, config.data.index_dir / val_index,
        config.data, config.augmentation, training=False, seed=config.schedule.seed,
        force_index=args.force_index,
    )
    if overfit:
        if args.overfit_images is not None and args.overfit_images < 1:
            raise ValueError("--overfit-images must be positive")
        if overfit_image_ids:
            by_id = {record.image_id: record for record in train_dataset.records}
            missing = sorted(set(overfit_image_ids) - set(by_id))
            if missing:
                raise ValueError(f"overfit image IDs are not in the training manifest: {missing}")
            selected = [by_id[image_id] for image_id in overfit_image_ids]
        else:
            ordered_records = sorted(
                train_dataset.records,
                key=lambda record: (record.positive_count, record.row_index),
            )
            selected_in_order = select_source_mixture_indices(
                ordered_records,
                config.data.source_weights,
                args.overfit_images,
                positive_only=True,
            )
            selected = [ordered_records[index] for index in selected_in_order]
        train_dataset.records = list(selected)
        val_dataset.records = list(selected)
        optimizer_steps = math.ceil(
            (args.batches_per_epoch or math.ceil(len(selected) / config.data.batch_size))
            / config.schedule.accumulation_steps
        ) * config.schedule.epochs
        config = replace(
            config,
            schedule=replace(
                config.schedule,
                freeze_backbone_epochs=0,
                warmup_steps=min(
                    config.schedule.warmup_steps,
                    max(1, math.ceil(optimizer_steps * 0.1)),
                ),
            ),
        )
    if overfit:
        repeat_count = args.batches_per_epoch or math.ceil(
            len(train_dataset) / config.data.batch_size
        )
        # Every optimizer batch sees every fixed fixture image exactly once.
        # This avoids domain sampling noise and makes the learnability check
        # independent of production mixture weights.
        batch_sampler = [list(range(len(train_dataset))) for _ in range(repeat_count)]
    else:
        batch_sampler = DomainMixtureBatchSampler(
            train_dataset,
            config.data.batch_size,
            domain_weights=config.data.domain_weights,
            source_weights=config.data.source_weights,
            empty_fraction=config.data.empty_fraction,
            seed=config.schedule.seed,
            batches_per_epoch=args.batches_per_epoch,
        )
    train_loader = DataLoader(
        train_dataset,
        batch_sampler=batch_sampler,
        num_workers=config.data.workers,
        collate_fn=collate_quad_proposal_samples,
        pin_memory=device.type == "cuda",
        persistent_workers=config.data.workers > 0,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=config.data.workers,
        collate_fn=collate_quad_proposal_samples,
        pin_memory=device.type == "cuda",
        persistent_workers=config.data.workers > 0,
    )
    assigner = QuadAssigner(
        strides=config.assignment.strides,
        top_k=config.quad.top_k,
        gamma=config.quad.gamma,
        scale_sigma=config.quad.scale_sigma,
        eligible_levels=config.quad.eligible_levels,
        scale_measure=config.quad.scale_measure,
    )
    model = QuadProposalDetector(pretrained_backbone=config.pretrained_backbone)
    target_builder = QuadTargetBuilder(assigner, weak_negative_weight=config.quad.weak_negative_weight)
    criterion = QuadProposalLoss(
        strides=config.assignment.strides,
        quality_weight=config.quad.quality_weight,
        corner_weight=config.quad.corner_weight,
        corner_smooth_l1_beta=config.quad.corner_smooth_l1_beta,
        gwd_weight=config.quad.gwd_weight,
        validity_weight=config.quad.validity_weight,
        quality_focal_beta=config.quad.quality_focal_beta,
        quality_target_mode=config.quad.quality_target_mode,
        quality_blend=config.quad.quality_blend,
        geometry_quality_target=config.quad.geometry_quality_target,
    )
    result = train_quad_proposals(
        model, train_loader, val_loader, target_builder, criterion, config, device,
        max_steps=args.max_steps, max_val_batches=args.max_val_batches,
        log_interval=args.log_interval,
        use_ema_for_validation=not overfit,
        resume=args.resume,
        validation_interval=args.validation_interval,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
