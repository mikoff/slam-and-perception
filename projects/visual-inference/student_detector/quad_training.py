"""Training and validation loop for the class-agnostic quad proposal path."""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import torch
from torch import Tensor

from .config import Phase3Config
from .quad_assigner import QuadAssigner
from .quad_data import QuadProposalSample
from .quad_decoder import QuadInferenceDecoder
from .quad_evaluation import QuadEvaluationImage, evaluate_quad_proposals
from .quad_losses import QuadProposalLoss
from .quad_targets import QuadTargetBuilder
from .training import (
    ExponentialMovingAverage,
    WarmupCosine,
    freeze_backbone_batch_norm,
    set_backbone_trainable,
    set_reproducibility_seed,
)


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
) -> dict[str, float]:
    model.eval()
    evaluated: list[QuadEvaluationImage] = []
    for batch_index, (images, samples) in enumerate(loader):
        if max_batches is not None and batch_index >= max_batches:
            break
        images = images.to(device, non_blocking=device.type == "cuda")
        output = model(images)
        targets = target_builder(samples, _feature_shapes(output), device=device)
        detections = decoder(output, (images.shape[-2], images.shape[-1]), targets.valid_point_masks)
        for sample, detection in zip(samples, detections, strict=True):
            evaluated.append(QuadEvaluationImage(
                image_id=sample.image_id,
                domain=sample.domain,
                camera_type=sample.camera_type,
                image_size=(images.shape[-2], images.shape[-1]),
                ground_truth=sample.quads.cpu(),
                ignore_quads=sample.ignore_quads.cpu(),
                detection=type(detection)(detection.quads.cpu(), detection.scores.cpu()),
                pre_nms_detection=(
                    type(detection)(
                        detection.pre_nms_quads.cpu(), detection.pre_nms_scores.cpu()
                    )
                    if detection.pre_nms_quads is not None
                    and detection.pre_nms_scores is not None
                    else None
                ),
                geometry_tiers=sample.geometry_tiers,
            ))
    return evaluate_quad_proposals(evaluated)


def _build_optimizer(model: torch.nn.Module, config: Phase3Config) -> torch.optim.Optimizer:
    effective_batch = config.data.batch_size * config.schedule.accumulation_steps
    scaled_lr = config.schedule.head_learning_rate * math.sqrt(
        effective_batch / config.schedule.reference_effective_batch
    )
    return torch.optim.AdamW(
        [
            {"params": list(model.backbone.parameters()), "lr": scaled_lr * config.schedule.backbone_lr_multiplier},
            {"params": [*model.fpn.parameters(), *model.head.parameters()], "lr": scaled_lr},
        ],
        weight_decay=config.schedule.weight_decay,
    )


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
) -> dict[str, Any]:
    """Train the quad detector and write checkpoints plus machine-readable logs."""
    set_reproducibility_seed(config.schedule.seed)
    output_dir = config.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    model.to(device)
    optimizer = _build_optimizer(model, config)
    steps_per_epoch = max(math.ceil(len(train_loader) / config.schedule.accumulation_steps), 1)
    scheduler = WarmupCosine(
        optimizer,
        total_steps=steps_per_epoch * config.schedule.epochs,
        warmup_steps=config.schedule.warmup_steps,
        min_ratio=config.schedule.min_lr_ratio,
    )
    scaler = torch.amp.GradScaler("cuda", enabled=config.schedule.amp and device.type == "cuda")
    ema = ExponentialMovingAverage(model, config.schedule.ema_decay)
    decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        # Validation retains the full 300-candidate diagnostic ceiling; metric
        # budgets slice this same ranked post-NMS list at 50, 100, and 300.
        max_proposals=config.inference.pre_nms_top_k,
    )
    best_score = -1.0
    global_step = 0
    log_path = output_dir / "quad_metrics.jsonl"
    optimizer.zero_grad(set_to_none=True)
    for epoch in range(config.schedule.epochs):
        if hasattr(train_loader.dataset, "set_epoch"):
            train_loader.dataset.set_epoch(epoch)
        if hasattr(train_loader.batch_sampler, "set_epoch"):
            train_loader.batch_sampler.set_epoch(epoch)
        set_backbone_trainable(model, epoch >= config.schedule.freeze_backbone_epochs)
        model.train()
        if config.pretrained_backbone:
            freeze_backbone_batch_norm(model.backbone)
        for batch_index, (images, samples) in enumerate(train_loader):
            images = images.to(device, non_blocking=device.type == "cuda")
            with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=scaler.is_enabled()):
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
            with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=scaler.is_enabled()):
                losses = criterion(output, targets)
                loss = losses.total / config.schedule.accumulation_steps
            if not torch.isfinite(losses.total):
                raise FloatingPointError(
                    "non-finite quad loss: "
                    f"quality={float(losses.quality.detach().float().cpu())}, "
                    f"corner={float(losses.corner.detach().float().cpu())}, "
                    f"validity={float(losses.validity.detach().float().cpu())}"
                )
            scaler.scale(loss).backward()
            should_step = (
                (batch_index + 1) % config.schedule.accumulation_steps == 0
                or batch_index + 1 == len(train_loader)
            )
            if should_step:
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), config.schedule.gradient_clip_norm)
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad(set_to_none=True)
                scheduler.step()
                ema.update(model)
                global_step += 1
            if log_interval and (batch_index + 1) % log_interval == 0:
                _write_jsonl(log_path, {
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
                    "validity_loss": float(losses.validity.detach().cpu()),
                    "positive_count": float(losses.number_positive.cpu()),
                    "quality_target_mean": float(losses.quality_target_mean.cpu()),
                    "fallback_count": float(targets.fallback_count.cpu()),
                    "unrepresentable_count": float(targets.unrepresentable_count.cpu()),
                    "quality_target_mode": criterion.quality_target_mode,
                })
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
        validation_model = ema.module if use_ema_for_validation else model
        metrics = validate_quad(
            validation_model, val_loader, target_builder, decoder, device,
            max_batches=max_val_batches,
        )
        metrics.update({"epoch": epoch, "step": global_step, "quality_target_mode": criterion.quality_target_mode})
        _write_jsonl(log_path, {"kind": "validation", **metrics})
        score = metrics.get("ar/100", 0.0)
        payload = {
            "format_version": 1,
            "architecture": "mobilenetv4_conv_medium_lite_fpn96_quad_dqco",
            "model": model.state_dict(),
            "ema_model": ema.module.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": scheduler.state_dict(),
            "epoch": epoch,
            "global_step": global_step,
            "config": asdict(config),
            "metrics": metrics,
            "quality_curriculum": {
                "mode": criterion.quality_target_mode,
                "blend": criterion.quality_blend,
            },
        }
        torch.save(payload, output_dir / "last.pt")
        if score > best_score:
            best_score = score
            torch.save(payload, output_dir / "best.pt")
        if max_steps is not None and global_step >= max_steps:
            break
    return {"global_step": global_step, "best_score": best_score, "metrics": metrics}
