"""Shared optimization runtime for HBB and quadrilateral proposal tasks."""

from __future__ import annotations

import math
import random
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol

import torch
from torch import Tensor, nn

from .config import Phase3Config
from .training_optimization import (
    ExponentialMovingAverage,
    WarmupCosine,
    build_detector_optimizer,
    freeze_backbone_batch_norm,
    optimizer_to,
    set_backbone_trainable,
    set_reproducibility_seed,
)
from .training_reporting import StandardReporter

try:
    from accelerate import Accelerator
except ImportError:  # pragma: no cover - exercised by the explicit runtime check
    Accelerator = None

@dataclass(frozen=True)
class CheckpointContext:
    epoch: int
    batch_in_epoch: int
    epoch_complete: bool
    global_step: int
    best_scores: dict[str, float]
    metrics: dict[str, Any]


class ProposalTrainingTask(Protocol):
    """Geometry-specific behavior consumed by the shared trainer."""

    checkpoint_phase: int | str
    architecture: str
    validation_states: tuple[str, ...]
    freeze_batch_norm: bool

    def build_targets(
        self, samples: Sequence[object], output: Any, device: torch.device
    ) -> Any: ...

    def compute_loss(self, output: Any, targets: Any) -> Any: ...

    def validate(
        self,
        model: nn.Module,
        loader: Any,
        device: torch.device,
        max_batches: int | None,
    ) -> dict[str, float]: ...

    def batch_metrics(self, losses: Any, targets: Any) -> dict[str, Tensor | float]: ...

    def check_finite(self, output: Any, losses: Any) -> None: ...

    def on_epoch_complete(self, epoch: int, total_epochs: int) -> None: ...

    def checkpoint_extra(self) -> dict[str, Any]: ...

    def restore_checkpoint_extra(self, state: Mapping[str, Any]) -> None: ...

    def load_model_state(self, model: nn.Module, checkpoint: Mapping[str, Any]) -> None: ...

    def selection_score(self, metrics: Mapping[str, float]) -> float: ...

    def best_checkpoint_names(self, state: str) -> tuple[str, ...]: ...

    def finalize_epoch_metrics(
        self,
        training_totals: Mapping[str, float],
        batches: int,
        validation: Mapping[str, Mapping[str, float]],
        previous_metrics: Mapping[str, Any],
        epoch: int,
        global_step: int,
        optimizer: torch.optim.Optimizer,
    ) -> dict[str, Any]: ...


def _rng_state() -> dict[str, Any]:
    state: dict[str, Any] = {
        "python": random.getstate(),
        "torch": torch.get_rng_state(),
    }
    if torch.cuda.is_available():
        state["cuda"] = torch.cuda.get_rng_state_all()
    return state


def _restore_rng_state(state: Mapping[str, Any]) -> None:
    if "python" in state:
        random.setstate(state["python"])
    if "torch" in state:
        torch.set_rng_state(state["torch"])
    if "cuda" in state and torch.cuda.is_available():
        torch.cuda.set_rng_state_all(state["cuda"])


def save_training_checkpoint(
    path: Path,
    *,
    model: nn.Module,
    ema: ExponentialMovingAverage,
    optimizer: torch.optim.Optimizer,
    scheduler: WarmupCosine,
    scaler_state: Mapping[str, Any],
    context: CheckpointContext,
    config: Phase3Config,
    task: ProposalTrainingTask,
    selected_state: str,
    save_function: Any = torch.save,
) -> None:
    payload = {
        "format_version": 2,
        "phase": task.checkpoint_phase,
        "architecture": task.architecture,
        "model": model.state_dict(),
        "ema_model": ema.module.state_dict(),
        "selected_state": selected_state,
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "scaler": dict(scaler_state),
        "epoch": context.epoch,
        "batch_in_epoch": context.batch_in_epoch,
        "epoch_complete": context.epoch_complete,
        "global_step": context.global_step,
        "ema_decay": ema.decay,
        "ema_ramp_steps": ema.ramp_steps,
        "ema_updates": ema.updates,
        "ema_initialization_weight": ema.initialization_weight,
        "best_score": context.best_scores.get(
            "raw", next(iter(context.best_scores.values()), -1.0)
        ),
        "best_scores": context.best_scores,
        "config": asdict(config),
        "metrics": context.metrics,
        "rng_state": _rng_state(),
        **task.checkpoint_extra(),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    save_function(payload, temporary)
    temporary.replace(path)


def _validation_model(
    state: str, base_model: nn.Module, ema: ExponentialMovingAverage
) -> nn.Module:
    if state == "raw":
        return base_model
    if state == "ema":
        return ema.module
    raise ValueError(f"unknown validation state: {state}")


def train_proposals(
    model: nn.Module,
    train_loader: Any,
    val_loader: Any,
    task: ProposalTrainingTask,
    config: Phase3Config,
    device: torch.device,
    *,
    max_steps: int | None = None,
    max_val_batches: int | None = None,
    log_interval: int = 50,
    resume: Path | None = None,
    validation_interval: int = 1,
    accelerator: Any | None = None,
    reporter: Any | None = None,
) -> dict[str, Any]:
    """Run the common optimization lifecycle with task-specific math hooks."""
    if validation_interval < 1:
        raise ValueError("validation_interval must be positive")
    if Accelerator is None:
        raise RuntimeError("Accelerate is required for proposal training")
    if accelerator is None:
        accelerator = Accelerator(
            cpu=device.type == "cpu",
            gradient_accumulation_steps=config.schedule.accumulation_steps,
            mixed_precision=(
                "fp16" if config.schedule.amp and device.type == "cuda" else "no"
            ),
            split_batches=True,
        )
    device = accelerator.device
    set_reproducibility_seed(config.schedule.seed)
    output_dir = config.output_dir
    reporter = reporter or StandardReporter(output_dir)
    if accelerator.is_main_process:
        output_dir.mkdir(parents=True, exist_ok=True)
    accelerator.wait_for_everyone()

    optimizer = build_detector_optimizer(model, config)
    steps_per_epoch = max(
        math.ceil(len(train_loader) / config.schedule.accumulation_steps), 1
    )
    scheduler = WarmupCosine(
        optimizer,
        total_steps=steps_per_epoch * config.schedule.epochs,
        warmup_steps=config.schedule.warmup_steps,
        min_ratio=config.schedule.min_lr_ratio,
    )
    model, optimizer, train_loader = accelerator.prepare(
        model, optimizer, train_loader
    )
    base_model = accelerator.unwrap_model(model)
    ema = ExponentialMovingAverage(
        base_model, config.schedule.ema_decay, config.schedule.ema_ramp_steps
    )
    best_scores = {state: -1.0 for state in task.validation_states}
    global_step = 0
    start_epoch = 0
    resume_batch = 0
    metrics: dict[str, Any] = {}

    if accelerator.is_main_process:
        reporter.on_start(
            config=config,
            model=base_model,
            train_loader=train_loader,
            val_loader=val_loader,
            optimizer_steps=steps_per_epoch * config.schedule.epochs,
            world_size=accelerator.num_processes,
        )
    optimizer.zero_grad(set_to_none=True)

    if resume is not None:
        checkpoint = torch.load(resume, map_location="cpu", weights_only=False)
        task.load_model_state(base_model, checkpoint)
        ema.module.load_state_dict(checkpoint["ema_model"], strict=True)
        ema.restore_tracking(
            int(checkpoint.get("ema_updates", checkpoint.get("global_step", 0))),
            float(checkpoint.get("ema_initialization_weight", 1.0)),
        )
        optimizer.load_state_dict(checkpoint["optimizer"])
        optimizer_to(optimizer, device)
        scheduler.load_state_dict(checkpoint["scheduler"])
        if checkpoint.get("scaler") and accelerator.scaler is not None:
            accelerator.scaler.load_state_dict(checkpoint["scaler"])
        epoch_complete = bool(checkpoint.get("epoch_complete", True))
        start_epoch = int(checkpoint["epoch"]) + int(epoch_complete)
        resume_batch = 0 if epoch_complete else int(
            checkpoint.get("batch_in_epoch", 0)
        )
        global_step = int(checkpoint["global_step"])
        restored_best = checkpoint.get("best_scores")
        if restored_best is not None:
            best_scores.update(
                {key: float(value) for key, value in restored_best.items()}
            )
        else:
            best_scores[task.validation_states[0]] = float(
                checkpoint.get("best_score", -1.0)
            )
        metrics = dict(checkpoint.get("metrics", {}))
        task.restore_checkpoint_extra(checkpoint)
        if checkpoint.get("rng_state"):
            _restore_rng_state(checkpoint["rng_state"])

    if max_steps is not None and global_step >= max_steps:
        reporter.close()
        return {
            "global_step": global_step,
            "best_scores": best_scores,
            "metrics": metrics,
        }

    try:
        for epoch in range(start_epoch, config.schedule.epochs):
            if hasattr(train_loader.dataset, "set_epoch"):
                train_loader.dataset.set_epoch(epoch)
            if hasattr(train_loader.batch_sampler, "set_epoch"):
                train_loader.batch_sampler.set_epoch(epoch)
            set_backbone_trainable(
                base_model, epoch >= config.schedule.freeze_backbone_epochs
            )
            model.train()
            if task.freeze_batch_norm:
                freeze_backbone_batch_norm(base_model.backbone)

            epoch_resume_batch = resume_batch if epoch == start_epoch else 0
            sampler_skips = hasattr(train_loader.batch_sampler, "set_start_batch")
            if sampler_skips:
                train_loader.batch_sampler.set_start_batch(epoch_resume_batch)
            enumerate_start = epoch_resume_batch if sampler_skips else 0
            running: dict[str, Tensor] = {}
            batches = 0
            last_batch_in_epoch = epoch_resume_batch

            for batch_index, (images, samples) in enumerate(
                train_loader, start=enumerate_start
            ):
                batch_in_epoch = batch_index + 1
                if not sampler_skips and batch_in_epoch <= epoch_resume_batch:
                    continue
                with accelerator.accumulate(model):
                    images = images.to(
                        device, non_blocking=device.type == "cuda"
                    )
                    with accelerator.autocast():
                        output = model(images)
                    targets = task.build_targets(samples, output, device)
                    with accelerator.autocast():
                        losses = task.compute_loss(output, targets)
                    task.check_finite(output, losses)
                    accelerator.backward(losses.total)
                    if accelerator.sync_gradients:
                        accelerator.clip_grad_norm_(
                            model.parameters(),
                            config.schedule.gradient_clip_norm,
                        )
                    optimizer.step()
                    optimizer.zero_grad(set_to_none=True)
                    if accelerator.sync_gradients:
                        scheduler.step()
                        ema.update(base_model)
                        global_step += 1

                batch_values = task.batch_metrics(losses, targets)
                for key, value in batch_values.items():
                    tensor = (
                        value.detach().to(device=device, dtype=torch.float32)
                        if isinstance(value, Tensor)
                        else torch.tensor(float(value), device=device)
                    )
                    running[key] = running.get(key, torch.zeros_like(tensor)) + tensor
                batches += 1
                last_batch_in_epoch = batch_in_epoch
                if accelerator.is_main_process and log_interval and (
                    batches % log_interval == 0
                    or batch_in_epoch == len(train_loader)
                ):
                    reporter.on_batch(
                        metrics={
                            key: float(value.detach().cpu())
                            if isinstance(value, Tensor)
                            else float(value)
                            for key, value in batch_values.items()
                        },
                        epoch=epoch,
                        batch=batch_in_epoch,
                        batches_per_epoch=len(train_loader),
                        global_step=global_step,
                        optimizer=optimizer,
                    )

                if accelerator.sync_gradients:
                    checkpoint_interval = config.schedule.checkpoint_every_steps
                    if (
                        accelerator.is_main_process
                        and checkpoint_interval > 0
                        and global_step % checkpoint_interval == 0
                    ):
                        context = CheckpointContext(
                            epoch,
                            batch_in_epoch,
                            False,
                            global_step,
                            best_scores,
                            metrics,
                        )
                        save_training_checkpoint(
                            output_dir / "last.pt",
                            model=base_model,
                            ema=ema,
                            optimizer=optimizer,
                            scheduler=scheduler,
                            scaler_state=(
                                accelerator.scaler.state_dict()
                                if accelerator.scaler is not None
                                else {}
                            ),
                            context=context,
                            config=config,
                            task=task,
                            selected_state="model",
                            save_function=accelerator.save,
                        )
                        reporter.on_checkpoint(output_dir)
                if max_steps is not None and global_step >= max_steps:
                    break

            task.on_epoch_complete(epoch, config.schedule.epochs)
            should_validate = (
                (epoch + 1) % validation_interval == 0
                or epoch + 1 == config.schedule.epochs
                or (max_steps is not None and global_step >= max_steps)
            )
            validation_metrics: dict[str, dict[str, float]] = {}
            if should_validate:
                accelerator.wait_for_everyone()
                if accelerator.is_main_process:
                    for state in task.validation_states:
                        state_metrics = task.validate(
                            _validation_model(state, base_model, ema),
                            val_loader,
                            device,
                            max_val_batches,
                        )
                        validation_metrics[state] = state_metrics
                        reporter.on_validation(
                            state=state,
                            metrics=state_metrics,
                            epoch=epoch,
                            global_step=global_step,
                        )
                accelerator.wait_for_everyone()

            training_totals = {
                key: float(value.cpu()) for key, value in running.items()
            }
            if accelerator.is_main_process:
                metrics = task.finalize_epoch_metrics(
                    training_totals,
                    batches,
                    validation_metrics,
                    metrics,
                    epoch,
                    global_step,
                    optimizer,
                )
                reporter.on_epoch(metrics=metrics)
            epoch_complete = last_batch_in_epoch >= len(train_loader)
            is_best = {state: False for state in task.validation_states}
            if accelerator.is_main_process and should_validate:
                for state, state_metrics in validation_metrics.items():
                    score = task.selection_score(state_metrics)
                    is_best[state] = score > best_scores[state]
                    if is_best[state]:
                        best_scores[state] = score

            if accelerator.is_main_process:
                context = CheckpointContext(
                    epoch,
                    last_batch_in_epoch,
                    epoch_complete,
                    global_step,
                    best_scores,
                    metrics,
                )
                checkpoint_args = {
                    "model": base_model,
                    "ema": ema,
                    "optimizer": optimizer,
                    "scheduler": scheduler,
                    "scaler_state": (
                        accelerator.scaler.state_dict()
                        if accelerator.scaler is not None
                        else {}
                    ),
                    "context": context,
                    "config": config,
                    "task": task,
                    "save_function": accelerator.save,
                }
                save_training_checkpoint(
                    output_dir / "last.pt",
                    selected_state="model",
                    **checkpoint_args,
                )
                for state, selected in is_best.items():
                    if not selected:
                        continue
                    selected_state = "model" if state == "raw" else "ema_model"
                    for name in task.best_checkpoint_names(state):
                        save_training_checkpoint(
                            output_dir / name,
                            selected_state=selected_state,
                            **checkpoint_args,
                        )
                reporter.on_checkpoint(output_dir)
            accelerator.wait_for_everyone()
            if max_steps is not None and global_step >= max_steps:
                break
    finally:
        reporter.close()

    return {
        "global_step": global_step,
        "best_scores": best_scores,
        "metrics": metrics,
    }
