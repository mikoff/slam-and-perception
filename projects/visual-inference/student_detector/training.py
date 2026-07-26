"""Training utilities and staged Phase-3 optimization loop."""

from __future__ import annotations

import copy
import json
import math
import random
import time
from collections.abc import Iterable
from dataclasses import asdict
from pathlib import Path
from typing import Any

import torch
from torch import Tensor, nn

from .config import Phase3Config
from .data import ProposalSample
from .decoder import InferenceDecoder
from .evaluation import EvaluationImage, evaluate_proposals
from .losses import LossOutput, ProposalLoss
from .model import StudentDetector
from .targets import TargetBuilder, point_validity_from_pixel_mask


def set_reproducibility_seed(seed: int) -> None:
    random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def freeze_backbone_batch_norm(backbone: nn.Module) -> None:
    """Freeze running statistics while leaving affine parameters trainable."""
    for module in backbone.modules():
        if isinstance(module, nn.modules.batchnorm._BatchNorm):
            module.eval()


def set_backbone_trainable(model: StudentDetector, trainable: bool) -> None:
    for parameter in model.backbone.parameters():
        parameter.requires_grad_(trainable)
    freeze_backbone_batch_norm(model.backbone)


class ExponentialMovingAverage:
    def __init__(self, model: nn.Module, decay: float) -> None:
        self.decay = decay
        self.module = copy.deepcopy(model).eval()
        self.module.requires_grad_(False)

    @torch.no_grad()
    def update(self, model: nn.Module) -> None:
        source = model.state_dict()
        for name, value in self.module.state_dict().items():
            current = source[name].detach()
            if value.is_floating_point():
                value.lerp_(current, 1 - self.decay)
            else:
                value.copy_(current)


class WarmupCosine:
    def __init__(
        self,
        optimizer: torch.optim.Optimizer,
        *,
        total_steps: int,
        warmup_steps: int,
        min_ratio: float,
    ) -> None:
        self.optimizer = optimizer
        self.total_steps = max(total_steps, 1)
        self.warmup_steps = min(warmup_steps, self.total_steps)
        self.min_ratio = min_ratio
        self.base_lrs = [group["lr"] for group in optimizer.param_groups]
        self.step_number = 0
        self._apply()

    def _ratio(self) -> float:
        if self.warmup_steps and self.step_number < self.warmup_steps:
            return max((self.step_number + 1) / self.warmup_steps, 1e-3)
        progress = (
            (self.step_number - self.warmup_steps)
            / max(self.total_steps - self.warmup_steps, 1)
        )
        progress = min(max(progress, 0.0), 1.0)
        return self.min_ratio + (1 - self.min_ratio) * (
            1 + math.cos(math.pi * progress)
        ) * 0.5

    def _apply(self) -> None:
        ratio = self._ratio()
        for group, base_lr in zip(
            self.optimizer.param_groups, self.base_lrs, strict=True
        ):
            group["lr"] = base_lr * ratio

    def step(self) -> None:
        self.step_number += 1
        self._apply()

    def state_dict(self) -> dict[str, Any]:
        return {"step_number": self.step_number}

    def load_state_dict(self, state: dict[str, Any]) -> None:
        self.step_number = int(state["step_number"])
        self._apply()


def build_optimizer(
    model: StudentDetector,
    config: Phase3Config,
    *,
    world_size: int = 1,
) -> torch.optim.AdamW:
    effective_batch = (
        config.data.batch_size
        * config.schedule.accumulation_steps
        * world_size
    )
    scaled_lr = config.schedule.head_learning_rate * math.sqrt(
        effective_batch / config.schedule.reference_effective_batch
    )
    return torch.optim.AdamW(
        [
            {
                "params": list(model.backbone.parameters()),
                "lr": scaled_lr * config.schedule.backbone_lr_multiplier,
                "name": "backbone",
            },
            {
                "params": [
                    *model.fpn.parameters(),
                    *model.head.parameters(),
                ],
                "lr": scaled_lr,
                "name": "fpn_head",
            },
        ],
        weight_decay=config.schedule.weight_decay,
    )


def save_checkpoint(
    path: Path,
    *,
    model: StudentDetector,
    ema: ExponentialMovingAverage,
    optimizer: torch.optim.Optimizer,
    scheduler: WarmupCosine,
    epoch: int,
    global_step: int,
    config: Phase3Config,
    metrics: dict[str, float],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "format_version": 1,
        "phase": 3,
        "architecture": (
            "mobilenetv4_conv_medium_lite_fpn96_fcos_atss_reg2"
        ),
        "model": model.state_dict(),
        "ema_model": ema.module.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "epoch": epoch,
        "global_step": global_step,
        "config": asdict(config),
        "metrics": metrics,
    }, path)


def _feature_shapes(output) -> tuple[tuple[int, int], ...]:
    return tuple(
        (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
    )


@torch.inference_mode()
def validate(
    model: StudentDetector,
    loader: Iterable[tuple[Tensor, list[ProposalSample]]],
    target_builder: TargetBuilder,
    decoder: InferenceDecoder,
    device: torch.device,
    *,
    max_batches: int | None = None,
) -> dict[str, float]:
    model.eval()
    evaluated: list[EvaluationImage] = []
    for batch_index, (images, samples) in enumerate(loader):
        if max_batches is not None and batch_index >= max_batches:
            break
        images = images.to(device, non_blocking=device.type == "cuda")
        output = model(images)
        shapes = _feature_shapes(output)
        valid_by_sample = [
            point_validity_from_pixel_mask(
                sample.valid_mask.to(device),
                shapes,
                decoder.strides,
            )[1]
            for sample in samples
        ]
        level_masks = tuple(
            torch.stack([
                valid_by_sample[batch][level]
                for batch in range(len(samples))
            ])
            for level in range(len(shapes))
        )
        detections = decoder(
            output, (images.shape[-2], images.shape[-1]), level_masks
        )
        for sample, detection in zip(samples, detections, strict=True):
            evaluated.append(EvaluationImage(
                sample.image_id,
                sample.domain,
                sample.camera_type,
                (images.shape[-2], images.shape[-1]),
                sample.boxes.cpu(),
                sample.ignore_boxes.cpu(),
                type(detection)(
                    detection.boxes.cpu(), detection.scores.cpu()
                ),
            ))
    return evaluate_proposals(evaluated)


def train_phase3(
    model: StudentDetector,
    train_loader,
    val_loader,
    target_builder: TargetBuilder,
    criterion: ProposalLoss,
    config: Phase3Config,
    device: torch.device,
    *,
    max_steps: int | None = None,
    max_val_batches: int | None = None,
    resume: Path | None = None,
    log_interval: int = 50,
    use_ema_for_validation: bool = True,
    validation_interval: int = 1,
) -> dict[str, Any]:
    """Run the staged trainer; max_steps makes CPU smoke jobs intentionally small."""
    if validation_interval < 1:
        raise ValueError("validation_interval must be positive")
    output_dir = config.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    set_reproducibility_seed(config.schedule.seed)
    model.to(device)
    optimizer = build_optimizer(model, config)
    optimizer_steps_per_epoch = math.ceil(
        len(train_loader) / config.schedule.accumulation_steps
    )
    scheduler = WarmupCosine(
        optimizer,
        total_steps=optimizer_steps_per_epoch * config.schedule.epochs,
        warmup_steps=config.schedule.warmup_steps,
        min_ratio=config.schedule.min_lr_ratio,
    )
    use_amp = config.schedule.amp and device.type == "cuda"
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)
    ema = ExponentialMovingAverage(model, config.schedule.ema_decay)
    decoder = InferenceDecoder(
        strides=config.assignment.strides,
        top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_detections=config.inference.max_proposals,
        score_mode=config.inference.score_mode,
    )
    log_path = output_dir / "metrics.jsonl"
    progress_path = output_dir / "progress.jsonl"
    global_step = 0
    start_epoch = 0
    best_score = -1.0
    last_metrics: dict[str, float] = {}
    optimizer.zero_grad(set_to_none=True)
    if resume is not None:
        checkpoint = torch.load(
            resume, map_location=device, weights_only=False
        )
        model.load_state_dict(checkpoint["model"])
        ema.module.load_state_dict(checkpoint["ema_model"])
        optimizer.load_state_dict(checkpoint["optimizer"])
        scheduler.load_state_dict(checkpoint["scheduler"])
        start_epoch = int(checkpoint["epoch"]) + 1
        global_step = int(checkpoint["global_step"])
        best_score = float(
            checkpoint.get("metrics", {}).get(
                "val/selection_score", best_score
            )
        )

    for epoch in range(start_epoch, config.schedule.epochs):
        epoch_started = time.perf_counter()
        interval_started = epoch_started
        interval_images = 0
        if device.type == "cuda":
            torch.cuda.reset_peak_memory_stats(device)
        backbone_trainable = epoch >= config.schedule.freeze_backbone_epochs
        set_backbone_trainable(model, backbone_trainable)
        model.train()
        freeze_backbone_batch_norm(model.backbone)
        if hasattr(train_loader.dataset, "set_epoch"):
            train_loader.dataset.set_epoch(epoch)
        if hasattr(train_loader.batch_sampler, "set_epoch"):
            train_loader.batch_sampler.set_epoch(epoch)
        running: dict[str, float] = defaultdict_float()
        interval_running: dict[str, float] = defaultdict_float()
        batches = 0
        for batch_index, (images, samples) in enumerate(train_loader):
            images = images.to(device, non_blocking=device.type == "cuda")
            with torch.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=use_amp,
            ):
                output = model(images)
                targets = target_builder(
                    samples, _feature_shapes(output), device=device
                )
                losses: LossOutput = criterion(output, targets)
                scaled_loss = (
                    losses.total / config.schedule.accumulation_steps
                )
            scaler.scale(scaled_loss).backward()
            should_step = (
                (batch_index + 1) % config.schedule.accumulation_steps == 0
                or batch_index + 1 == len(train_loader)
            )
            if should_step:
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
            running["loss/total"] += float(losses.total.detach())
            running["loss/objectness"] += float(losses.objectness.detach())
            running["loss/box_ciou"] += float(losses.box_ciou.detach())
            running["loss/box_ltrb"] += float(losses.box_ltrb.detach())
            running["loss/centerness"] += float(losses.centerness.detach())
            running["number_positive"] += float(losses.number_positive)
            running["number_gt"] += float(targets.valid_gt_count)
            running["number_fallback"] += float(targets.fallback_count)
            running["number_unrepresentable"] += float(
                targets.unrepresentable_count
            )
            interval_running["loss/total"] += float(losses.total.detach())
            interval_running["loss/objectness"] += float(
                losses.objectness.detach()
            )
            interval_running["loss/box_ciou"] += float(
                losses.box_ciou.detach()
            )
            interval_running["loss/box_ltrb"] += float(
                losses.box_ltrb.detach()
            )
            interval_running["loss/centerness"] += float(
                losses.centerness.detach()
            )
            interval_running["number_positive"] += float(
                losses.number_positive
            )
            interval_images += images.shape[0]
            for level, count in enumerate(
                targets.positive_counts_per_level, start=3
            ):
                running[f"positive/P{level}"] += float(count)
            batches += 1
            should_log = log_interval > 0 and (
                batches % log_interval == 0
                or batch_index + 1 == len(train_loader)
                or (max_steps is not None and global_step >= max_steps)
            )
            if should_log:
                now = time.perf_counter()
                interval_batches = (
                    batches % log_interval or min(log_interval, batches)
                )
                elapsed = max(now - interval_started, 1e-9)
                progress = {
                    "event": "train_progress",
                    "epoch": epoch,
                    "batch": batches,
                    "batches_per_epoch": len(train_loader),
                    "global_step": global_step,
                    "backbone_trainable": backbone_trainable,
                    "loss/total": (
                        interval_running["loss/total"] / interval_batches
                    ),
                    "loss/objectness": (
                        interval_running["loss/objectness"] / interval_batches
                    ),
                    "loss/box_ciou": (
                        interval_running["loss/box_ciou"] / interval_batches
                    ),
                    "loss/box_ltrb": (
                        interval_running["loss/box_ltrb"] / interval_batches
                    ),
                    "loss/centerness": (
                        interval_running["loss/centerness"]
                        / interval_batches
                    ),
                    "number_positive": (
                        interval_running["number_positive"]
                        / interval_batches
                    ),
                    "lr/backbone": float(optimizer.param_groups[0]["lr"]),
                    "lr/fpn_head": float(optimizer.param_groups[1]["lr"]),
                    "images_per_second": interval_images / elapsed,
                    "epoch_elapsed_seconds": now - epoch_started,
                }
                if device.type == "cuda":
                    progress["cuda/peak_memory_gib"] = (
                        torch.cuda.max_memory_allocated(device) / 1024**3
                    )
                serialized = json.dumps(progress, sort_keys=True)
                print(serialized, flush=True)
                with progress_path.open("a", encoding="utf-8") as stream:
                    stream.write(serialized + "\n")
                interval_started = now
                interval_images = 0
                interval_running = defaultdict_float()
            if max_steps is not None and global_step >= max_steps:
                break

        train_metrics = {
            key: value / max(batches, 1) for key, value in running.items()
        }
        train_metrics["fallback_rate_per_gt"] = (
            running["number_fallback"] / max(running["number_gt"], 1)
        )
        should_validate = (
            (epoch + 1) % validation_interval == 0
            or epoch + 1 == config.schedule.epochs
            or (max_steps is not None and global_step >= max_steps)
        )
        if should_validate:
            validation_model = ema.module if use_ema_for_validation else model
            val_metrics = validate(
                validation_model,
                val_loader,
                target_builder,
                decoder,
                device,
                max_batches=max_val_batches,
            )
        else:
            val_metrics = {}
        last_metrics = {
            **train_metrics,
            **{f"val/{key}": value for key, value in val_metrics.items()},
            "epoch": float(epoch),
            "global_step": float(global_step),
            "lr/backbone": float(optimizer.param_groups[0]["lr"]),
            "lr/fpn_head": float(optimizer.param_groups[1]["lr"]),
        }
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(last_metrics, sort_keys=True) + "\n")
        save_checkpoint(
            output_dir / "last.pt",
            model=model,
            ema=ema,
            optimizer=optimizer,
            scheduler=scheduler,
            epoch=epoch,
            global_step=global_step,
            config=config,
            metrics=last_metrics,
        )
        score = val_metrics.get("selection_score")
        if score is not None and score > best_score:
            best_score = score
            save_checkpoint(
                output_dir / "best.pt",
                model=model,
                ema=ema,
                optimizer=optimizer,
                scheduler=scheduler,
                epoch=epoch,
                global_step=global_step,
                config=config,
                metrics=last_metrics,
            )
        if max_steps is not None and global_step >= max_steps:
            break
    return {
        "global_step": global_step,
        "best_selection_score": best_score,
        "metrics": last_metrics,
    }


def defaultdict_float() -> dict[str, float]:
    """A typed small helper avoiding defaultdict in checkpoint-visible state."""
    from collections import defaultdict

    return defaultdict(float)
