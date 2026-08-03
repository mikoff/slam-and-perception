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
    def __init__(self, model: nn.Module, decay: float, ramp_steps: int = 0) -> None:
        self.decay = decay
        self.ramp_steps = ramp_steps
        self.updates = 0
        self.initialization_weight = 1.0
        self.module = copy.deepcopy(model).eval()
        self.module.requires_grad_(False)

    def current_decay(self) -> float:
        if self.ramp_steps == 0:
            return self.decay
        return self.decay * (1 - math.exp(-self.updates / self.ramp_steps))

    @torch.no_grad()
    def update(self, model: nn.Module) -> None:
        self.updates += 1
        decay = self.current_decay()
        self.initialization_weight *= decay
        source = model.state_dict()
        for name, value in self.module.state_dict().items():
            current = source[name].detach()
            if value.is_floating_point():
                value.lerp_(current, 1 - decay)
            else:
                value.copy_(current)

    def restore_tracking(self, updates: int, initialization_weight: float) -> None:
        self.updates = updates
        self.initialization_weight = initialization_weight


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
    batch_in_epoch: int,
    epoch_complete: bool,
    global_step: int,
    config: Phase3Config,
    metrics: dict[str, float],
) -> None:
    payload = {
        "format_version": 2,
        "phase": 3,
        "architecture": (
            "mobilenetv4_conv_medium_lite_fpn96_fcos_atss_reg2"
        ),
        "model": model.state_dict(),
        "ema_model": ema.module.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "epoch": epoch,
        "batch_in_epoch": batch_in_epoch,
        "epoch_complete": epoch_complete,
        "global_step": global_step,
        "ema_decay": ema.decay,
        "ema_ramp_steps": ema.ramp_steps,
        "ema_updates": ema.updates,
        "ema_initialization_weight": ema.initialization_weight,
        "config": asdict(config),
        "metrics": metrics,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(payload, temporary)
    temporary.replace(path)


def _optimizer_to(
    optimizer: torch.optim.Optimizer, device: torch.device
) -> None:
    """Move optimizer tensor state after loading a CPU checkpoint."""
    for state in optimizer.state.values():
        for key, value in state.items():
            if isinstance(value, Tensor):
                state[key] = value.to(device)


class TensorMetricAccumulator:
    """Accumulate scalar metrics on the active training device."""

    def __init__(self, device: torch.device) -> None:
        self.device = device
        self.values: dict[str, Tensor] = {}

    def add(self, key: str, value: Tensor | float | int) -> None:
        tensor = (
            value.detach().to(device=self.device, dtype=torch.float32)
            if isinstance(value, Tensor)
            else torch.tensor(float(value), device=self.device)
        )
        self.values[key] = self.values.get(key, torch.zeros_like(tensor)) + tensor

    def as_floats(self, divisor: float = 1.0) -> dict[str, float]:
        if not self.values:
            return {}
        keys = list(self.values)
        stacked = torch.stack([self.values[key] for key in keys]).cpu()
        return {
            key: float(value) / divisor
            for key, value in zip(keys, stacked, strict=True)
        }


def _feature_shapes(output) -> tuple[tuple[int, int], ...]:
    return tuple(
        (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
    )


@torch.no_grad()
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
                sample.valid_mask.to(
                    output.objectness[0].device
                ),
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
                    detection.boxes.cpu(),
                    detection.scores.cpu(),
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
    ema = ExponentialMovingAverage(
        model, config.schedule.ema_decay, config.schedule.ema_ramp_steps
    )
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
    resume_batch = 0
    best_score = -1.0
    last_metrics: dict[str, float] = {}
    optimizer.zero_grad(set_to_none=True)
    if resume is not None:
        checkpoint = torch.load(
            resume, map_location="cpu", weights_only=False
        )
        model.load_state_dict(checkpoint["model"])
        ema.module.load_state_dict(checkpoint["ema_model"])
        ema.restore_tracking(
            int(checkpoint.get("ema_updates", checkpoint.get("global_step", 0))),
            float(checkpoint.get("ema_initialization_weight", 1.0)),
        )
        optimizer.load_state_dict(checkpoint["optimizer"])
        _optimizer_to(optimizer, device)
        scheduler.load_state_dict(checkpoint["scheduler"])
        epoch_complete = bool(checkpoint.get("epoch_complete", True))
        start_epoch = int(checkpoint["epoch"]) + int(epoch_complete)
        resume_batch = (
            0 if epoch_complete
            else int(checkpoint.get("batch_in_epoch", 0))
        )
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
        epoch_resume_batch = (
            resume_batch if epoch == start_epoch else 0
        )
        sampler_skips_batches = hasattr(
            train_loader.batch_sampler, "set_start_batch"
        )
        if sampler_skips_batches:
            train_loader.batch_sampler.set_start_batch(epoch_resume_batch)
        running = TensorMetricAccumulator(device)
        interval_running = TensorMetricAccumulator(device)
        batches = 0
        interval_batches = 0
        last_batch_in_epoch = resume_batch if epoch == start_epoch else 0
        enumerate_start = epoch_resume_batch if sampler_skips_batches else 0
        for batch_index, (images, samples) in enumerate(
            train_loader, start=enumerate_start
        ):
            batch_in_epoch = batch_index + 1
            if (
                not sampler_skips_batches
                and epoch == start_epoch
                and batch_in_epoch <= resume_batch
            ):
                continue
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
                batch_in_epoch % config.schedule.accumulation_steps == 0
                or batch_in_epoch == len(train_loader)
            )
            if should_step:
                if use_amp:
                    scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(
                    model.parameters(), config.schedule.gradient_clip_norm
                )
                if use_amp:
                    scaler.step(optimizer)
                    scaler.update()
                else:
                    optimizer.step()
                optimizer.zero_grad(set_to_none=True)
                scheduler.step()
                ema.update(model)
                global_step += 1
            values = {
                "loss/total": losses.total,
                "loss/objectness": losses.objectness,
                "loss/box_ciou": losses.box_ciou,
                "loss/box_ltrb": losses.box_ltrb,
                "loss/centerness": losses.centerness,
                "number_positive": losses.number_positive,
                "number_gt": targets.valid_gt_count,
                "number_fallback": targets.fallback_count,
                "number_unrepresentable": targets.unrepresentable_count,
            }
            for key, value in values.items():
                running.add(key, value)
                if key in {
                    "loss/total",
                    "loss/objectness",
                    "loss/box_ciou",
                    "loss/box_ltrb",
                    "loss/centerness",
                    "number_positive",
                }:
                    interval_running.add(key, value)
            interval_images += images.shape[0]
            for level_index in range(
                targets.positive_counts_per_level.shape[0]
            ):
                running.add(
                    f"positive/P{level_index + 3}",
                    targets.positive_counts_per_level[level_index],
                )
            batches += 1
            interval_batches += 1
            last_batch_in_epoch = batch_in_epoch
            checkpoint_interval = config.schedule.checkpoint_every_steps
            if (
                should_step
                and checkpoint_interval > 0
                and global_step % checkpoint_interval == 0
            ):
                step_metrics = {
                    **last_metrics,
                    "epoch": float(epoch),
                    "global_step": float(global_step),
                }
                save_checkpoint(
                    output_dir / "last.pt",
                    model=model,
                    ema=ema,
                    optimizer=optimizer,
                    scheduler=scheduler,
                    epoch=epoch,
                    batch_in_epoch=batch_in_epoch,
                    epoch_complete=False,
                    global_step=global_step,
                    config=config,
                    metrics=step_metrics,
                )
            should_log = log_interval > 0 and (
                interval_batches >= log_interval
                or batch_in_epoch == len(train_loader)
                or (max_steps is not None and global_step >= max_steps)
            )
            if should_log:
                now = time.perf_counter()
                elapsed = max(now - interval_started, 1e-9)
                interval_metrics = interval_running.as_floats(
                    max(interval_batches, 1)
                )
                progress = {
                    "event": "train_progress",
                    "epoch": epoch,
                    "batch": batch_in_epoch,
                    "batches_per_epoch": len(train_loader),
                    "global_step": global_step,
                    "backbone_trainable": backbone_trainable,
                    **interval_metrics,
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
                interval_batches = 0
                interval_running = TensorMetricAccumulator(device)
            if max_steps is not None and global_step >= max_steps:
                break

        running_totals = running.as_floats()
        train_metrics = {
            key: value / max(batches, 1)
            for key, value in running_totals.items()
        }
        train_metrics["fallback_rate_per_gt"] = (
            running_totals.get("number_fallback", 0.0)
            / max(running_totals.get("number_gt", 0.0), 1)
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
        epoch_complete = last_batch_in_epoch >= len(train_loader)
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(last_metrics, sort_keys=True) + "\n")
        save_checkpoint(
            output_dir / "last.pt",
            model=model,
            ema=ema,
            optimizer=optimizer,
            scheduler=scheduler,
            epoch=epoch,
            batch_in_epoch=last_batch_in_epoch,
            epoch_complete=epoch_complete,
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
                batch_in_epoch=last_batch_in_epoch,
                epoch_complete=epoch_complete,
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
