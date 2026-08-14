"""Train the class-agnostic quadrilateral proposal detector."""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import subprocess
import sys
import types
from dataclasses import replace
from pathlib import Path

# Compatibility shim: Python 3.13 refactored pathlib to pathlib._local.
# Provide alias when loading 3.13 checkpoints under Python 3.12 environments.
if not hasattr(pathlib, "_local"):
    _local_mod = types.ModuleType("pathlib._local")
    _local_mod.PosixPath = pathlib.PosixPath
    _local_mod.WindowsPath = pathlib.WindowsPath
    _local_mod.Path = pathlib.Path
    sys.modules["pathlib._local"] = _local_mod

import torch
from accelerate import Accelerator
from torch.utils.data import DataLoader

from student_detector.checkpoint_transport import (
    AwsCheckpointStore,
    ResolvedCheckpoint,
    resolve_resume_checkpoint,
)
from student_detector.config import load_phase3_config
from student_detector.data import (
    DomainMixtureBatchSampler,
    select_source_mixture_indices,
    use_file_system_tensor_sharing,
)
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import train_quad_proposals
from student_detector.training_optimization import set_reproducibility_seed

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
    parser.add_argument("--seed", type=int)
    parser.add_argument("--neck-type", choices=("lite", "attn_res"))
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
    parser.add_argument("--resume-mode", choices=("none", "auto"), default="none")
    parser.add_argument("--resume-from-run-id")
    parser.add_argument("--run-id", type=str)
    parser.add_argument("--validation-interval", type=int, default=1)
    parser.add_argument("--force-index", action="store_true")
    parser.add_argument(
        "--wandb-project", type=str, help="Weights & Biases project name"
    )
    parser.add_argument("--wandb-entity", type=str, help="Weights & Biases entity name")
    parser.add_argument("--wandb-run-name", type=str, help="Weights & Biases run name")
    return parser.parse_args()


def _verify_resume_ancestry(parent: ResolvedCheckpoint) -> None:
    parent_commit = parent.contract.get("source_commit", "")
    current_commit = os.getenv("SOURCE_COMMIT", "")
    if not parent_commit or not current_commit:
        raise ValueError("cross-run resume requires both source commits")
    result = subprocess.run(
        ["git", "merge-base", "--is-ancestor", parent_commit, current_commit],
        cwd=Path(__file__).resolve().parents[2],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 1:
        raise ValueError(
            "resume source commit is not an ancestor of the current source commit"
        )
    if result.returncode != 0:
        raise RuntimeError(
            "could not verify resume source ancestry: "
            f"{result.stderr.strip() or f'git exited {result.returncode}'}"
        )


def main() -> None:
    args = parse_args()
    overfit_image_ids = (
        tuple(int(value) for value in args.overfit_image_ids.split(",") if value)
        if args.overfit_image_ids
        else ()
    )
    overfit = args.overfit_images is not None or bool(overfit_image_ids)
    config = load_phase3_config(args.config)

    requested_device = torch.device(
        "cuda"
        if args.device == "auto" and torch.cuda.is_available()
        else "cpu"
        if args.device == "auto"
        else args.device
    )

    if args.epochs is not None or args.no_amp:
        config = replace(
            config,
            schedule=replace(
                config.schedule,
                epochs=args.epochs
                if args.epochs is not None
                else config.schedule.epochs,
                amp=False if args.no_amp else config.schedule.amp,
            ),
        )
    if args.workers is not None or args.batch_size is not None:
        config = replace(
            config,
            data=replace(
                config.data,
                workers=args.workers
                if args.workers is not None
                else config.data.workers,
                batch_size=args.batch_size
                if args.batch_size is not None
                else config.data.batch_size,
            ),
        )
    if args.seed is not None or args.neck_type is not None:
        config = replace(
            config,
            schedule=replace(
                config.schedule,
                seed=args.seed if args.seed is not None else config.schedule.seed,
            ),
            neck_type=args.neck_type or config.neck_type,
        )
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
    accelerator = Accelerator(
        cpu=requested_device.type == "cpu",
        gradient_accumulation_steps=config.schedule.accumulation_steps,
        split_batches=True,
        mixed_precision=(
            "fp16" if config.schedule.amp and requested_device.type == "cuda" else "no"
        ),
        log_with="wandb" if args.wandb_project else None,
    )
    device = accelerator.device
    # Model construction initializes the proposal head, so seed before creating
    # the dataset, sampler, or model. Seeding only inside the training loop made
    # nominally identical runs start from different heads.
    set_reproducibility_seed(config.schedule.seed)
    use_file_system_tensor_sharing()
    with accelerator.main_process_first():
        train_dataset = QuadProposalDataset(
            config.data.quad_train_annotations or config.data.train_annotations,
            config.data.image_root,
            config.data.index_dir / "quad_train.sqlite",
            config.data,
            config.augmentation,
            training=not overfit,
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
            config.data.image_root,
            config.data.index_dir / val_index,
            config.data,
            config.augmentation,
            training=False,
            seed=config.schedule.seed,
            force_index=args.force_index,
        )
    if overfit:
        if args.overfit_images is not None and args.overfit_images < 1:
            raise ValueError("--overfit-images must be positive")
        if overfit_image_ids:
            by_id = {record.image_id: record for record in train_dataset.records}
            missing = sorted(set(overfit_image_ids) - set(by_id))
            if missing:
                raise ValueError(
                    f"overfit image IDs are not in the training manifest: {missing}"
                )
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
        optimizer_steps = (
            math.ceil(
                (
                    args.batches_per_epoch
                    or config.data.batches_per_epoch
                    or math.ceil(len(selected) / config.data.batch_size)
                )
                / config.schedule.accumulation_steps
            )
            * config.schedule.epochs
        )
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
        repeat_count = (
            args.batches_per_epoch
            or config.data.batches_per_epoch
            or math.ceil(len(train_dataset) / config.data.batch_size)
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
            batches_per_epoch=(args.batches_per_epoch or config.data.batches_per_epoch),
        )
    train_loader = DataLoader(
        train_dataset,
        batch_sampler=batch_sampler,
        num_workers=config.data.workers,
        collate_fn=collate_quad_proposal_samples,
        pin_memory=device.type == "cuda",
        multiprocessing_context=("spawn" if config.data.workers > 0 else None),
        worker_init_fn=use_file_system_tensor_sharing,
        persistent_workers=config.data.workers > 0,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=config.data.workers,
        collate_fn=collate_quad_proposal_samples,
        pin_memory=device.type == "cuda",
        multiprocessing_context=("spawn" if config.data.workers > 0 else None),
        worker_init_fn=use_file_system_tensor_sharing,
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
    model = QuadProposalDetector(
        pretrained_backbone=config.pretrained_backbone,
        neck_type=config.neck_type,
    )
    target_builder = QuadTargetBuilder(
        assigner, weak_negative_weight=config.quad.weak_negative_weight
    )
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
    resume = args.resume
    resume_contract: dict[str, str] | None = None
    if args.resume_mode == "auto":
        if not args.run_id:
            raise ValueError("--resume-mode auto requires --run-id")
        bucket = os.getenv("S3_BUCKET")
        if not bucket:
            raise ValueError("--resume-mode auto requires S3_BUCKET")
        expected_contract = {
            "source_commit": os.getenv("SOURCE_COMMIT", ""),
            "dataset_id": os.getenv("DATASET_ID", ""),
            "dataset_manifest_sha256": os.getenv("DATASET_MANIFEST_SHA256", ""),
            "config_path": os.getenv("CONFIG_PATH", ""),
        }
        if args.resume_from_run_id:
            expected_contract["resume_from_run_id"] = args.resume_from_run_id
        resolved = resolve_resume_checkpoint(
            run_id=args.run_id,
            output_dir=config.output_dir,
            store=AwsCheckpointStore(bucket, os.getenv("S3_ENDPOINT_URL") or None),
            expected_contract=expected_contract,
        )
        if resolved is None and args.resume_from_run_id:
            parent_expected = {
                key: expected_contract[key]
                for key in (
                    "dataset_id",
                    "dataset_manifest_sha256",
                    "config_path",
                )
            }
            resolved = resolve_resume_checkpoint(
                run_id=args.resume_from_run_id,
                output_dir=config.output_dir,
                store=AwsCheckpointStore(
                    bucket, os.getenv("S3_ENDPOINT_URL") or None
                ),
                expected_contract=parent_expected,
            )
            if resolved is None:
                raise FileNotFoundError(
                    "resume parent has no checkpoint manifest: "
                    f"{args.resume_from_run_id}"
                )
            _verify_resume_ancestry(resolved)
            print(
                "Cross-run resume: "
                f"parent={resolved.run_id} source={resolved.contract['source_commit']}",
                flush=True,
            )
        if resolved is not None:
            resume = resolved.path
            resume_contract = {"run_id": resolved.run_id, **resolved.contract}
    result = train_quad_proposals(
        model,
        train_loader,
        val_loader,
        target_builder,
        criterion,
        config,
        device,
        max_steps=args.max_steps,
        max_val_batches=args.max_val_batches,
        log_interval=args.log_interval,
        resume=resume,
        resume_contract=resume_contract,
        validation_interval=args.validation_interval,
        wandb_project=args.wandb_project,
        wandb_entity=args.wandb_entity,
        wandb_run_name=args.wandb_run_name,
        run_id=args.run_id,
        accelerator=accelerator,
    )
    if accelerator.is_main_process:
        print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
