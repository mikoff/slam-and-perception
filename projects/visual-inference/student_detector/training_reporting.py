"""Training metrics and external reporting side effects."""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import asdict
from pathlib import Path
from typing import Any

import torch

from .config import Phase3Config

try:
    from torch.utils.tensorboard import SummaryWriter
except ImportError:
    SummaryWriter = None


def _write_jsonl(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(dict(value), sort_keys=True) + "\n")


class StandardReporter:
    """Write JSONL, TensorBoard, W&B, and checkpoint notifications."""

    def __init__(
        self,
        output_dir: Path,
        *,
        batch_log: str | None = None,
        epoch_log: str | None = None,
        tensorboard: bool = False,
        wandb_project: str | None = None,
        wandb_entity: str | None = None,
        wandb_run_name: str | None = None,
        start_callback: Callable[..., None] | None = None,
        checkpoint_callback: Callable[[Path], None] | None = None,
    ) -> None:
        self.output_dir = output_dir
        self.batch_path = output_dir / batch_log if batch_log else None
        self.epoch_path = output_dir / epoch_log if epoch_log else None
        self.use_tensorboard = tensorboard
        self.wandb_project = wandb_project
        self.wandb_entity = wandb_entity
        self.wandb_run_name = wandb_run_name
        self.start_callback = start_callback
        self.checkpoint_callback = checkpoint_callback
        self.writer: Any | None = None
        self.wandb_run: Any | None = None

    def on_start(self, **context: Any) -> None:
        config: Phase3Config = context["config"]
        if self.use_tensorboard and SummaryWriter is not None:
            self.writer = SummaryWriter(log_dir=str(self.output_dir / "tensorboard"))
            self.writer.add_text(
                "Dataset/Summary",
                "### Training Run Dataset Audit Summary\n\n"
                f"- **Training Dataset Samples**: {len(context['train_loader'].dataset)} images\n"
                f"- **Validation Dataset Samples**: {len(context['val_loader'].dataset)} images\n"
                f"- **Batch Size**: {config.data.batch_size}\n"
                f"- **Total Epochs**: {config.schedule.epochs}\n"
                f"- **Input Image Size**: {config.data.input_size}x{config.data.input_size}\n",
                global_step=0,
            )
        if self.wandb_project is not None:
            try:
                import wandb

                self.wandb_run = wandb.init(
                    project=self.wandb_project,
                    entity=self.wandb_entity,
                    name=self.wandb_run_name,
                    config=asdict(config),
                )
            except Exception:
                self.wandb_run = None
        if self.start_callback is not None:
            self.start_callback(**context)

    def _record(self, record: Mapping[str, Any], *, step: int) -> None:
        if self.batch_path is not None:
            _write_jsonl(self.batch_path, record)
        if self.wandb_run is not None:
            try:
                self.wandb_run.log(dict(record), step=step)
            except Exception:
                pass

    def on_batch(
        self,
        *,
        metrics: Mapping[str, float],
        epoch: int,
        batch: int,
        batches_per_epoch: int,
        global_step: int,
        optimizer: torch.optim.Optimizer,
    ) -> None:
        record = {
            "kind": "train",
            "epoch": epoch,
            "batch": batch,
            "batches_per_epoch": batches_per_epoch,
            "step": global_step,
            **metrics,
            "lr/backbone": float(optimizer.param_groups[0]["lr"]),
            "lr/fpn_head": float(optimizer.param_groups[1]["lr"]),
        }
        self._record(record, step=global_step)
        if self.writer is not None:
            for key, value in metrics.items():
                self.writer.add_scalar(f"Train/{key}", value, global_step)

    def on_validation(
        self,
        *,
        state: str,
        metrics: Mapping[str, float],
        epoch: int,
        global_step: int,
    ) -> None:
        record = {
            "kind": "validation",
            "state": state,
            "epoch": epoch,
            "step": global_step,
            **metrics,
        }
        self._record(record, step=global_step)
        if self.writer is not None:
            for key, value in metrics.items():
                if isinstance(value, (int, float)):
                    self.writer.add_scalar(f"Val/{state}/{key}", value, epoch)

    def on_epoch(self, *, metrics: Mapping[str, Any]) -> None:
        if self.epoch_path is not None:
            _write_jsonl(self.epoch_path, metrics)

    def on_checkpoint(self, output_dir: Path) -> None:
        if self.checkpoint_callback is not None:
            self.checkpoint_callback(output_dir)

    def close(self) -> None:
        if self.writer is not None:
            self.writer.close()
        if self.wandb_run is not None:
            try:
                self.wandb_run.finish()
            except Exception:
                pass
