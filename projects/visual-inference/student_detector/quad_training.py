"""Training and validation loop for the class-agnostic quad proposal path."""

from __future__ import annotations

import json
import math
import os
import subprocess
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any


import torch
from torch import Tensor

from .config import Phase3Config
from .quad_decoder import QuadDetection, QuadInferenceDecoder, decode_dense_quad_output
from .quad_evaluation import QuadEvaluationImage, evaluate_quad_proposals
from .quad_geometry import quad_validity
from .quad_losses import QuadProposalLoss
from .quad_targets import QuadTargetBuilder
from .training import (
    ExponentialMovingAverage,
    WarmupCosine,
    freeze_backbone_batch_norm,
    set_backbone_trainable,
    set_reproducibility_seed,
)


try:
    from torch.utils.tensorboard import SummaryWriter
except ImportError:
    SummaryWriter = None

try:
    from accelerate import Accelerator
except ImportError:
    Accelerator = None


def _feature_shapes(output: Any) -> tuple[tuple[int, int], ...]:

    return tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.quality)


def _write_jsonl(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, sort_keys=True) + "\n")


@torch.no_grad()
def validate_quad(
    model: torch.nn.Module,
    loader: Any,
    target_builder: QuadTargetBuilder,
    decoder: QuadInferenceDecoder,
    device: torch.device,
    *,
    max_batches: int | None = None,
    include_dense_diagnostics: bool = False,
) -> dict[str, float]:
    model.eval()
    evaluated: list[QuadEvaluationImage] = []
    for batch_index, (images, samples) in enumerate(loader):
        if max_batches is not None and batch_index >= max_batches:
            break
        images = images.to(device, non_blocking=device.type == "cuda")
        output = model(images)
        targets = target_builder(samples, _feature_shapes(output), device=device)
        detections = decoder(
            output,
            (images.shape[-2], images.shape[-1]),
            targets.valid_point_masks,
        )
        dense_quads, dense_scores = decode_dense_quad_output(output, decoder.strides)
        valid_points = torch.cat(
            [mask.reshape(mask.shape[0], -1) for mask in targets.valid_point_masks], dim=1
        )
        for sample_index, (sample, detection) in enumerate(
            zip(samples, detections, strict=True)
        ):
            assigned_mask = targets.positive_mask[sample_index]
            assigned = QuadDetection(
                dense_quads[sample_index, assigned_mask].detach().cpu(),
                dense_scores[sample_index, assigned_mask].detach().cpu(),
            )
            dense_detection = None
            if include_dense_diagnostics:
                candidate_quads = dense_quads[sample_index]
                in_bounds = (
                    (candidate_quads[..., 0] >= 0).all(dim=1)
                    & (candidate_quads[..., 0] <= images.shape[-1]).all(dim=1)
                    & (candidate_quads[..., 1] >= 0).all(dim=1)
                    & (candidate_quads[..., 1] <= images.shape[-2]).all(dim=1)
                )
                keep_dense = (
                    valid_points[sample_index]
                    & in_bounds
                    & quad_validity(candidate_quads)
                )
                dense_detection = QuadDetection(
                    candidate_quads[keep_dense].detach().cpu(),
                    dense_scores[sample_index, keep_dense].detach().cpu(),
                )
            evaluated.append(QuadEvaluationImage(
                image_id=sample.image_id,
                domain=sample.domain,
                camera_type=sample.camera_type,
                image_size=(images.shape[-2], images.shape[-1]),
                ground_truth=sample.quads.cpu(),
                ignore_quads=sample.ignore_quads.cpu(),
                detection=type(detection)(
                    detection.quads.cpu(),
                    detection.scores.cpu(),
                    candidate_count=detection.candidate_count,
                    invalid_candidate_count=detection.invalid_candidate_count,
                ),
                pre_nms_detection=(
                    type(detection)(
                        detection.pre_nms_quads.cpu(), detection.pre_nms_scores.cpu()
                    )
                    if detection.pre_nms_quads is not None
                    and detection.pre_nms_scores is not None
                    else None
                ),
                geometry_tiers=sample.geometry_tiers,
                object_conditions=sample.object_conditions,
                seen_statuses=sample.seen_statuses,
                size_bins=sample.size_bins,
                aspect_bins=sample.aspect_bins,
                radial_bins=sample.radial_bins,
                dense_detection=dense_detection,
                assigned_detection=assigned,
                assigned_gt_indices=targets.matched_gt_indices[sample_index, assigned_mask]
                .detach()
                .cpu(),
                positive_scores=dense_scores[
                    sample_index, targets.positive_mask[sample_index]
                ]
                .detach()
                .cpu(),
                trusted_background_scores=dense_scores[
                    sample_index, targets.trusted_background_mask[sample_index]
                ]
                .detach()
                .cpu(),
                weak_background_scores=dense_scores[
                    sample_index, targets.weak_background_mask[sample_index]
                ]
                .detach()
                .cpu(),
            ))
            
        if (batch_index + 1) % 50 == 0 or batch_index + 1 == len(loader):
            ts = datetime.now().strftime("%H:%M:%S")
            print(f"[{ts}] [Validation] Processed batch {batch_index + 1}/{len(loader)}", flush=True)

    return evaluate_quad_proposals(evaluated)


def _build_optimizer(model: torch.nn.Module, config: Phase3Config) -> torch.optim.Optimizer:
    effective_batch = config.data.batch_size * config.schedule.accumulation_steps
    scaled_lr = config.schedule.head_learning_rate * math.sqrt(
        effective_batch / config.schedule.reference_effective_batch
    )
    return torch.optim.AdamW(
        [
            {
                "params": list(model.backbone.parameters()),
                "lr": scaled_lr * config.schedule.backbone_lr_multiplier,
            },
            {"params": [*model.fpn.parameters(), *model.head.parameters()], "lr": scaled_lr},
        ],
        weight_decay=config.schedule.weight_decay,
    )


def _optimizer_to(optimizer: torch.optim.Optimizer, device: torch.device) -> None:
    for state in optimizer.state.values():
        for key, value in state.items():
            if isinstance(value, Tensor):
                state[key] = value.to(device)


def _save_checkpoint(
    path: Path,
    *,
    model: torch.nn.Module,
    ema: ExponentialMovingAverage,
    optimizer: torch.optim.Optimizer,
    scheduler: WarmupCosine,
    scaler: torch.amp.GradScaler,
    epoch: int,
    batch_in_epoch: int,
    epoch_complete: bool,
    global_step: int,
    best_score: float,
    config: Phase3Config,
    metrics: dict[str, Any],
    criterion: QuadProposalLoss,
) -> None:
    payload = {
        "format_version": 2,
        "phase": "quad_proposals",
        "architecture": "mobilenetv4_conv_medium_lite_fpn96_quad_dqco",
        "model": model.state_dict(),
        "ema_model": ema.module.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "scaler": scaler.state_dict(),
        "epoch": epoch,
        "batch_in_epoch": batch_in_epoch,
        "epoch_complete": epoch_complete,
        "global_step": global_step,
        "ema_decay": ema.decay,
        "ema_ramp_steps": ema.ramp_steps,
        "ema_updates": ema.updates,
        "ema_initialization_weight": ema.initialization_weight,
        "best_score": best_score,
        "config": asdict(config),
        "metrics": metrics,
        "quality_curriculum": {
            "mode": criterion.quality_target_mode,
            "blend": criterion.quality_blend,
            "geometry_quality_target": criterion.geometry_quality_target,
            "corner_smooth_l1_beta": criterion.corner_smooth_l1_beta,
            "gwd_weight": criterion.gwd_weight,
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    torch.save(payload, temporary)
    temporary.replace(path)
    _sync_checkpoint_to_s3(path.parent)


def _sync_checkpoint_to_s3(output_dir: Path) -> None:
    s3_bucket = os.getenv("S3_BUCKET")
    if not s3_bucket:
        return
    run_name = os.getenv("WANDB_RUN_NAME") or output_dir.name
    s3_endpoint = os.getenv("S3_ENDPOINT_URL")
    cmd = [
        "aws", "s3", "sync",
        str(output_dir),
        f"s3://{s3_bucket}/runs/{run_name}",
        "--no-progress",
    ]
    if s3_endpoint:
        cmd.extend(["--endpoint-url", s3_endpoint])
    try:
        subprocess.Popen(cmd)
    except Exception as err:
        sys.stderr.write(f"--> [Warning] S3 sync process launch failed: {err}\n")





def train_quad_proposals(
    model: torch.nn.Module,
    train_loader: Any,
    val_loader: Any,
    target_builder: QuadTargetBuilder,
    criterion: QuadProposalLoss,
    config: Phase3Config,
    device: torch.device,
    *,
    max_steps: int | None = None,
    max_val_batches: int | None = None,
    log_interval: int = 50,
    use_ema_for_validation: bool = True,
    resume: Path | None = None,
    validation_interval: int = 1,
    wandb_project: str | None = None,
    wandb_entity: str | None = None,
    wandb_run_name: str | None = None,
) -> dict[str, Any]:
    """Train the quad detector and write checkpoints plus machine-readable logs."""
    if validation_interval < 1:
        raise ValueError("validation_interval must be positive")
    set_reproducibility_seed(config.schedule.seed)
    output_dir = config.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    model.to(device)
    optimizer = _build_optimizer(model, config)
    steps_per_epoch = max(
        math.ceil(len(train_loader) / config.schedule.accumulation_steps), 1
    )
    scheduler = WarmupCosine(
        optimizer,
        total_steps=steps_per_epoch * config.schedule.epochs,
        warmup_steps=config.schedule.warmup_steps,
        min_ratio=config.schedule.min_lr_ratio,
    )
    scaler = torch.amp.GradScaler(
        "cuda", enabled=config.schedule.amp and device.type == "cuda"
    )
    accelerator = None
    if Accelerator is not None and device.type == "cuda":
        accelerator = Accelerator(
            gradient_accumulation_steps=config.schedule.accumulation_steps,
            mixed_precision="fp16" if config.schedule.amp else "no",
        )
    ema = ExponentialMovingAverage(
        model, config.schedule.ema_decay, config.schedule.ema_ramp_steps
    )
    decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        # Frequent training validation uses the deployment budget. The final
        # gate report separately retains the full 300-proposal diagnostic list.
        max_proposals=config.inference.max_proposals,
    )
    best_score = -1.0
    global_step = 0
    start_epoch = 0
    resume_batch = 0
    metrics: dict[str, Any] = {}
    log_path = output_dir / "quad_metrics.jsonl"
    tb_writer = SummaryWriter(log_dir=str(output_dir / "tensorboard")) if SummaryWriter is not None else None
    wandb_run = None
    if wandb_project is not None:
        try:
            import wandb
            wandb_run = wandb.init(
                project=wandb_project,
                entity=wandb_entity,
                name=wandb_run_name,
                config=asdict(config),
            )
        except Exception:
            wandb_run = None
    if tb_writer is not None:
        dataset_summary = (
            "### Training Run Dataset Audit Summary\n\n"
            f"- **Manifest Path**: `{config.data.quad_train_annotations or config.data.train_annotations}`\n"
            f"- **Training Dataset Samples**: {len(train_loader.dataset)} images\n"
            f"- **Validation Dataset Samples**: {len(val_loader.dataset)} images\n"
            f"- **Batch Size**: {config.data.batch_size}\n"
            f"- **Total Epochs**: {config.schedule.epochs}\n"
            f"- **Input Image Size**: {config.data.input_size}x{config.data.input_size}\n"
            f"- **Centerness Modulation Active**: `True (alpha=0.3)`\n"
        )
        tb_writer.add_text("Dataset/Summary", dataset_summary, global_step=0)
    optimizer.zero_grad(set_to_none=True)


    if resume is not None:
        checkpoint = torch.load(resume, map_location="cpu", weights_only=False)
        model.load_state_dict(checkpoint["model"])
        ema.module.load_state_dict(checkpoint["ema_model"])
        ema.restore_tracking(
            int(checkpoint.get("ema_updates", checkpoint.get("global_step", 0))),
            float(checkpoint.get("ema_initialization_weight", 1.0)),
        )
        optimizer.load_state_dict(checkpoint["optimizer"])
        _optimizer_to(optimizer, device)
        scheduler.load_state_dict(checkpoint["scheduler"])
        if checkpoint.get("scaler"):
            scaler.load_state_dict(checkpoint["scaler"])
        epoch_complete = bool(checkpoint.get("epoch_complete", True))
        start_epoch = int(checkpoint["epoch"]) + int(epoch_complete)
        resume_batch = (
            0 if epoch_complete else int(checkpoint.get("batch_in_epoch", 0))
        )
        global_step = int(checkpoint["global_step"])
        best_score = float(checkpoint.get("best_score", -1.0))
        metrics = dict(checkpoint.get("metrics", {}))
        curriculum = checkpoint.get("quality_curriculum", {})
        criterion.quality_target_mode = str(
            curriculum.get("mode", criterion.quality_target_mode)
        )
        criterion.quality_blend = float(
            curriculum.get("blend", criterion.quality_blend)
        )
    if max_steps is not None and global_step >= max_steps:
        return {"global_step": global_step, "best_score": best_score, "metrics": metrics}
    for epoch in range(start_epoch, config.schedule.epochs):
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] [Train] Initializing epoch {epoch} dataset...", flush=True)
        if hasattr(train_loader.dataset, "set_epoch"):
            train_loader.dataset.set_epoch(epoch)
        if hasattr(train_loader.batch_sampler, "set_epoch"):
            train_loader.batch_sampler.set_epoch(epoch)
        
        print(f"[{ts}] [Train] Entering DataLoader for epoch {epoch}...", flush=True)
        set_backbone_trainable(model, epoch >= config.schedule.freeze_backbone_epochs)
        model.train()
        if config.pretrained_backbone:
            freeze_backbone_batch_norm(model.backbone)
        last_batch_in_epoch = resume_batch if epoch == start_epoch else 0
        for batch_index, (images, samples) in enumerate(train_loader):
            batch_in_epoch = batch_index + 1
            if epoch == start_epoch and batch_in_epoch <= resume_batch:
                continue
            images = images.to(device, non_blocking=device.type == "cuda")
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=scaler.is_enabled(),
            ):
                output = model(images)
            if not all(
                torch.isfinite(tensor).all()
                for tensor in (*output.quality, *output.corner_offsets)
            ):
                quality_finite = [bool(torch.isfinite(tensor).all()) for tensor in output.quality]
                offsets_finite = [
                    bool(torch.isfinite(tensor).all()) for tensor in output.corner_offsets
                ]
                raise FloatingPointError(
                    "non-finite quad model output: "
                    f"quality={quality_finite}, corner_offsets={offsets_finite}"
                )
            # Target assignment contains float32-only geometric eigensolvers
            # and is independent of model gradients; keep it outside AMP.
            targets = target_builder(samples, _feature_shapes(output), device=device)
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=scaler.is_enabled(),
            ):
                losses = criterion(output, targets)
                loss = losses.total / config.schedule.accumulation_steps
            if not torch.isfinite(losses.total):
                raise FloatingPointError(
                    "non-finite quad loss: "
                    f"quality={float(losses.quality.detach().float().cpu())}, "
                    f"corner={float(losses.corner.detach().float().cpu())}, "
                    f"gwd={float(losses.gwd.detach().float().cpu())}, "
                    f"validity={float(losses.validity.detach().float().cpu())}"
                )
            if accelerator is not None:
                accelerator.backward(loss)
            else:
                scaler.scale(loss).backward()

            if (batch_index + 1) % 50 == 0 or batch_index + 1 == len(train_loader):
                ts = datetime.now().strftime("%H:%M:%S")
                print(
                    f"[{ts}] [Train] Epoch {epoch} | Step {batch_index + 1}/{len(train_loader)} "
                    f"| Loss: {losses.total.item():.4f} (Qual: {losses.quality.item():.4f}, Corner: {losses.corner.item():.4f})",
                    flush=True
                )

            should_step = (
                (batch_index + 1) % config.schedule.accumulation_steps == 0
                or batch_index + 1 == len(train_loader)
            )
            if should_step:
                if accelerator is not None:
                    accelerator.clip_grad_norm_(
                        model.parameters(), config.schedule.gradient_clip_norm
                    )
                    optimizer.step()
                else:
                    scaler.unscale_(optimizer)
                    torch.nn.utils.clip_grad_norm_(
                        model.parameters(), config.schedule.gradient_clip_norm
                    )
                    scaler.step(optimizer)
                    scaler.update()
                optimizer.zero_grad(set_to_none=True)
                scheduler.step()
                ema.update(model)
                global_step += 1
                checkpoint_interval = config.schedule.checkpoint_every_steps
                if checkpoint_interval > 0 and global_step % checkpoint_interval == 0:
                    _save_checkpoint(
                        output_dir / "last.pt",
                        model=model,
                        ema=ema,
                        optimizer=optimizer,
                        scheduler=scheduler,
                        scaler=scaler,
                        epoch=epoch,
                        batch_in_epoch=batch_in_epoch,
                        epoch_complete=False,
                        global_step=global_step,
                        best_score=best_score,
                        config=config,
                        metrics=metrics,
                        criterion=criterion,
                    )
            last_batch_in_epoch = batch_in_epoch
            if log_interval and (
                (batch_index + 1) % log_interval == 0
                or batch_index + 1 == len(train_loader)
            ):
                train_metrics = {
                    "kind": "train",
                    "epoch": epoch,
                    "step": global_step,
                    "loss": float(losses.total.detach().cpu()),
                    "quality_loss": float(losses.quality.detach().cpu()),
                    "quality_positive_loss": float(losses.quality_positive.detach().cpu()),
                    "quality_trusted_background_loss": float(
                        losses.quality_trusted_background.detach().cpu()
                    ),
                    "quality_weak_background_loss": float(
                        losses.quality_weak_background.detach().cpu()
                    ),
                    "weak_negative_weight": targets.weak_negative_weight,
                    "corner_loss": float(losses.corner.detach().cpu()),
                    "gwd_loss": float(losses.gwd.detach().cpu()),
                    "validity_loss": float(losses.validity.detach().cpu()),
                    "positive_count": float(losses.number_positive.cpu()),
                    "quality_target_mean": float(losses.quality_target_mean.cpu()),
                    "fallback_count": float(targets.fallback_count.cpu()),
                    "unrepresentable_count": float(targets.unrepresentable_count.cpu()),
                    "quality_target_mode": criterion.quality_target_mode,
                    "geometry_quality_target": criterion.geometry_quality_target,
                }
                _write_jsonl(log_path, train_metrics)
                if tb_writer is not None:
                    tb_writer.add_scalar("Train/TotalLoss", train_metrics["loss"], global_step)
                    tb_writer.add_scalar("Train/QualityLoss", train_metrics["quality_loss"], global_step)
                    tb_writer.add_scalar("Train/CornerLoss", train_metrics["corner_loss"], global_step)
                    tb_writer.add_scalar("Train/LearningRate", optimizer.param_groups[0]["lr"], global_step)
                if wandb_run is not None:
                    try:
                        wandb_run.log(train_metrics, step=global_step)
                    except Exception:
                        pass

            if max_steps is not None and global_step >= max_steps:
                break
        # The curriculum is intentionally epoch-based and deterministic.
        warmup_epochs = max(1, config.schedule.epochs // 5)
        if epoch + 1 < warmup_epochs:
            criterion.quality_target_mode = "centerness"
            criterion.quality_blend = 0.0
        elif epoch + 1 < 2 * warmup_epochs:
            criterion.quality_target_mode = "blend"
            criterion.quality_blend = min(1.0, (epoch + 1 - warmup_epochs) / warmup_epochs)
        else:
            criterion.quality_target_mode = "iou"
            criterion.quality_blend = 1.0
        should_validate = (
            (epoch + 1) % validation_interval == 0
            or epoch + 1 == config.schedule.epochs
            or (max_steps is not None and global_step >= max_steps)
        )
        if should_validate:
            validation_model = ema.module if use_ema_for_validation else model
            metrics = validate_quad(
                validation_model,
                val_loader,
                target_builder,
                decoder,
                device,
                max_batches=max_val_batches,
                include_dense_diagnostics=False,
            )
        metrics.update({
            "epoch": epoch,
            "step": global_step,
            "quality_target_mode": criterion.quality_target_mode,
            "geometry_quality_target": criterion.geometry_quality_target,
            "corner_smooth_l1_beta": criterion.corner_smooth_l1_beta,
            "gwd_weight": criterion.gwd_weight,
        })
        if should_validate:
            ts = datetime.now().strftime("%H:%M:%S")
            print(f"[{ts}] [Validation] Evaluation math finished. Writing logs...", flush=True)
            _write_jsonl(log_path, {"kind": "validation", **metrics})
            if tb_writer is not None:
                if "ar/100" in metrics:
                    tb_writer.add_scalar("Val/AR100", metrics["ar/100"], epoch)
                if "recall/100@0.50" in metrics:
                    tb_writer.add_scalar("Val/Recall50", metrics["recall/100@0.50"], epoch)
                if "recall/100@0.75" in metrics:
                    tb_writer.add_scalar("Val/Recall75", metrics["recall/100@0.75"], epoch)
                if "matched_iou/median" in metrics:
                    tb_writer.add_scalar("Val/MatchedIoUMedian", metrics["matched_iou/median"], epoch)
            if wandb_run is not None:
                try:
                    wandb_run.log({"kind": "validation", **metrics}, step=global_step)
                except Exception:
                    pass

        score = metrics.get("ar/100", 0.0)
        epoch_complete = last_batch_in_epoch >= len(train_loader)
        is_best = should_validate and score > best_score
        if is_best:
            best_score = score
            
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] [Validation] Saving checkpoint to {output_dir / 'last.pt'}...", flush=True)
        
        _save_checkpoint(
            output_dir / "last.pt", model=model, ema=ema, optimizer=optimizer,
            scheduler=scheduler, scaler=scaler, epoch=epoch,
            batch_in_epoch=last_batch_in_epoch, epoch_complete=epoch_complete,
            global_step=global_step, best_score=best_score, config=config,
            metrics=metrics, criterion=criterion,
        )
        if is_best:
            _save_checkpoint(
                output_dir / "best.pt", model=model, ema=ema, optimizer=optimizer,
                scheduler=scheduler, scaler=scaler, epoch=epoch,
                batch_in_epoch=last_batch_in_epoch, epoch_complete=epoch_complete,
                global_step=global_step, best_score=best_score, config=config,
                metrics=metrics, criterion=criterion,
            )
            
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] [Validation] Checkpoint saved successfully. Epoch {epoch} complete.", flush=True)
        
        if max_steps is not None and global_step >= max_steps:
            break
    if tb_writer is not None:
        tb_writer.close()
    if wandb_run is not None:
        try:
            wandb_run.finish()
        except Exception:
            pass
    return {"global_step": global_step, "best_score": best_score, "metrics": metrics}

