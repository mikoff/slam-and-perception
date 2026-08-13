"""Train the Phase-3 class-agnostic proposal detector."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from student_detector.assigner import ATSSAssigner
from student_detector.config import load_phase3_config
from student_detector.data import (
    DomainMixtureBatchSampler,
    IndexedCocoProposalDataset,
    collate_proposal_samples,
    select_source_mixture_indices,
    use_file_system_tensor_sharing,
)
from student_detector.losses import ProposalLoss
from student_detector.model import StudentDetector
from student_detector.targets import TargetBuilder
from student_detector.training import train_phase3
from student_detector.training_optimization import set_reproducibility_seed

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("configs/phase3.yaml")
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--max-val-batches", type=int)
    parser.add_argument("--batches-per-epoch", type=int)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--workers", type=int)
    parser.add_argument("--epochs", type=int)
    parser.add_argument(
        "--freeze-backbone-epochs",
        type=int,
        help=(
            "Backbone-freeze duration override; use 0 for representative "
            "unfrozen throughput measurements"
        ),
    )
    parser.add_argument(
        "--box-loss", choices=("ciou", "giou"),
        help="One-variable regression-loss override for controlled ablations",
    )
    parser.add_argument(
        "--objectness-loss",
        choices=("focal", "quality_focal"),
        help="Objectness-loss override for controlled ranking ablations",
    )
    parser.add_argument(
        "--centerness-weight",
        type=float,
        help="Centerness-loss weight override",
    )
    parser.add_argument(
        "--ltrb-weight",
        type=float,
        help="Stride-normalized auxiliary SmoothL1 LTRB-loss weight",
    )
    parser.add_argument(
        "--score-mode",
        choices=("objectness", "objectness_x_centerness"),
        help="Inference ranking override",
    )
    parser.add_argument(
        "--pure-atss",
        action="store_true",
        help="Remove the project-specific ATSS centre-region gate",
    )
    parser.add_argument(
        "--prior-multiplier",
        type=int,
        choices=(4, 8),
        help="Set virtual prior sides to multiplier × feature stride",
    )
    parser.add_argument(
        "--tiny-area",
        type=float,
        help="Post-transform positive-area threshold override",
    )
    parser.add_argument(
        "--tiny-min-side",
        type=float,
        help="Post-transform minimum-side threshold override",
    )
    parser.add_argument(
        "--woodscape-background-weight",
        type=float,
        help=(
            "Weak negative-objectness weight for valid non-ignore "
            "WoodScape locations"
        ),
    )
    parser.add_argument(
        "--log-interval",
        type=int,
        default=50,
        help="Write/print training progress every N batches (0 disables it)",
    )
    parser.add_argument(
        "--validation-interval",
        type=int,
        help=(
            "Validate every N epochs; defaults to 5 for Stage 0 and 1 for "
            "production"
        ),
    )
    parser.add_argument(
        "--overfit-images",
        type=int,
        help="Stage-0 mode: train and validate on one fixed balanced subset",
    )
    parser.add_argument("--no-pretrained", action="store_true")
    parser.add_argument("--force-index", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument(
        "--checkpoint-every-steps",
        type=int,
        help=(
            "Write a resumable mid-epoch last.pt every N optimizer steps; "
            "0 disables step checkpoints"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_phase3_config(args.config)
    if args.batch_size is not None or args.workers is not None:
        config = replace(config, data=replace(
            config.data,
            batch_size=args.batch_size or config.data.batch_size,
            workers=(
                args.workers
                if args.workers is not None
                else config.data.workers
            ),
        ))
    if (
        args.epochs is not None
        or args.freeze_backbone_epochs is not None
        or args.checkpoint_every_steps is not None
    ):
        config = replace(
            config,
            schedule=replace(
                config.schedule,
                epochs=(
                    args.epochs
                    if args.epochs is not None
                    else config.schedule.epochs
                ),
                freeze_backbone_epochs=(
                    args.freeze_backbone_epochs
                    if args.freeze_backbone_epochs is not None
                    else config.schedule.freeze_backbone_epochs
                ),
                checkpoint_every_steps=(
                    args.checkpoint_every_steps
                    if args.checkpoint_every_steps is not None
                    else config.schedule.checkpoint_every_steps
                ),
            ),
        )
    if (
        args.box_loss is not None
        or args.objectness_loss is not None
        or args.ltrb_weight is not None
        or args.centerness_weight is not None
    ):
        config = replace(
            config,
            loss=replace(
                config.loss,
                box_loss=args.box_loss or config.loss.box_loss,
                objectness_loss=(
                    args.objectness_loss or config.loss.objectness_loss
                ),
                ltrb_weight=(
                    args.ltrb_weight
                    if args.ltrb_weight is not None
                    else config.loss.ltrb_weight
                ),
                centerness_weight=(
                    args.centerness_weight
                    if args.centerness_weight is not None
                    else config.loss.centerness_weight
                ),
            ),
        )
    if args.score_mode is not None:
        config = replace(
            config,
            inference=replace(
                config.inference, score_mode=args.score_mode
            ),
        )
    if args.pure_atss or args.prior_multiplier is not None:
        config = replace(
            config,
            assignment=replace(
                config.assignment,
                center_radius=(
                    None if args.pure_atss
                    else config.assignment.center_radius
                ),
                prior_sizes=(
                    tuple(
                        args.prior_multiplier * stride
                        for stride in config.assignment.strides
                    )
                    if args.prior_multiplier is not None
                    else config.assignment.prior_sizes
                ),
            ),
        )
    if args.tiny_area is not None or args.tiny_min_side is not None:
        config = replace(
            config,
            data=replace(
                config.data,
                tiny_area=(
                    args.tiny_area
                    if args.tiny_area is not None
                    else config.data.tiny_area
                ),
                tiny_min_side=(
                    args.tiny_min_side
                    if args.tiny_min_side is not None
                    else config.data.tiny_min_side
                ),
            ),
        )
    if args.woodscape_background_weight is not None:
        background_weights = dict(config.data.background_loss_weights)
        background_weights["woodscape_rgb_fisheye"] = (
            args.woodscape_background_weight
        )
        config = replace(
            config,
            data=replace(
                config.data,
                background_loss_weights=background_weights,
            ),
        )
    if args.output_dir is not None:
        config = replace(config, output_dir=args.output_dir.resolve())
    device = torch.device(
        "cuda" if args.device == "auto" and torch.cuda.is_available()
        else "cpu" if args.device == "auto"
        else args.device
    )
    set_reproducibility_seed(config.schedule.seed)
    use_file_system_tensor_sharing()
    train_dataset = IndexedCocoProposalDataset(
        config.data.train_annotations,
        config.data.image_root,
        config.data.index_dir / "train.sqlite",
        config.data,
        config.augmentation,
        # A diagnostic overfit run must see the same pixels every epoch.
        # Production training keeps the full stochastic augmentation pipeline.
        training=not bool(args.overfit_images),
        seed=config.schedule.seed,
        force_index=args.force_index,
    )
    val_annotations = (
        config.data.train_annotations
        if args.overfit_images
        else config.data.val_annotations
    )
    val_index = "train.sqlite" if args.overfit_images else "val.sqlite"
    val_dataset = IndexedCocoProposalDataset(
        val_annotations,
        config.data.image_root,
        config.data.index_dir / val_index,
        config.data,
        config.augmentation,
        training=False,
        seed=config.schedule.seed,
        force_index=args.force_index,
    )
    if args.overfit_images:
        if args.overfit_images < 1:
            raise ValueError("--overfit-images must be positive")
        selected_indices = select_source_mixture_indices(
            train_dataset.records,
            config.data.source_weights,
            args.overfit_images,
            positive_only=True,
        )
        selected = [
            train_dataset.records[index] for index in selected_indices
        ]
        train_dataset.records = list(selected)
        val_dataset.records = list(selected)
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
        collate_fn=collate_proposal_samples,
        pin_memory=device.type == "cuda",
        multiprocessing_context=("spawn" if config.data.workers > 0 else None),
        worker_init_fn=use_file_system_tensor_sharing,
        # Worker-local dataset copies otherwise retain epoch zero forever and
        # repeat the same deterministic augmentation sequence every epoch.
        persistent_workers=False,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=config.data.workers,
        collate_fn=collate_proposal_samples,
        pin_memory=device.type == "cuda",
        multiprocessing_context=("spawn" if config.data.workers > 0 else None),
        worker_init_fn=use_file_system_tensor_sharing,
        persistent_workers=False,
    )
    if args.overfit_images:
        optimizer_steps_per_epoch = math.ceil(
            len(train_loader) / config.schedule.accumulation_steps
        )
        total_steps = optimizer_steps_per_epoch * config.schedule.epochs
        config = replace(
            config,
            schedule=replace(
                config.schedule,
                warmup_steps=min(
                    config.schedule.warmup_steps,
                    max(1, math.ceil(total_steps * 0.1)),
                ),
            ),
        )
    assigner = ATSSAssigner(
        strides=config.assignment.strides,
        prior_sizes=config.assignment.prior_sizes,
        top_k=config.assignment.top_k,
        center_radius=config.assignment.center_radius,
    )
    criterion = ProposalLoss(
        strides=config.assignment.strides,
        objectness_weight=config.loss.objectness_weight,
        box_weight=config.loss.box_weight,
        ltrb_weight=config.loss.ltrb_weight,
        centerness_weight=config.loss.centerness_weight,
        objectness_loss=config.loss.objectness_loss,
        quality_focal_beta=config.loss.quality_focal_beta,
        box_loss=config.loss.box_loss,
        box_weighting=config.loss.box_weighting,
        focal_alpha=config.loss.focal_alpha,
        focal_gamma=config.loss.focal_gamma,
    )
    model = StudentDetector(
        pretrained_backbone=config.pretrained_backbone
        and not args.no_pretrained
    )
    result = train_phase3(
        model,
        train_loader,
        val_loader,
        TargetBuilder(
            assigner,
            background_loss_weights=config.data.background_loss_weights,
        ),
        criterion,
        config,
        device,
        max_steps=args.max_steps,
        max_val_batches=args.max_val_batches,
        resume=args.resume,
        log_interval=args.log_interval,
        use_ema_for_validation=not bool(args.overfit_images),
        validation_interval=(
            args.validation_interval
            if args.validation_interval is not None
            else 5 if args.overfit_images
            else 1
        ),
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
