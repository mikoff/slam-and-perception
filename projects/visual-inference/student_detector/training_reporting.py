"""Structured training metrics, W&B tracking, and durable checkpoint events."""

from __future__ import annotations

import json
import os
import shutil
import sys
from collections.abc import Callable, Mapping
from dataclasses import asdict
from pathlib import Path
from typing import Any

import torch

from .checkpoint_transport import CheckpointUploader, uploader_from_environment
from .config import Phase3Config


def _write_jsonl(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(dict(value), sort_keys=True) + "\n")


class StandardReporter:
    """Write an offline audit log and optional resumable Accelerate/W&B run."""

    def __init__(
        self,
        output_dir: Path,
        *,
        batch_log: str | None = None,
        epoch_log: str | None = None,
        wandb_project: str | None = None,
        wandb_entity: str | None = None,
        wandb_run_name: str | None = None,
        run_id: str | None = None,
        accelerator: Any | None = None,
        start_callback: Callable[..., None] | None = None,
        checkpoint_uploader: CheckpointUploader | None = None,
    ) -> None:
        self.output_dir = output_dir
        self.batch_path = output_dir / batch_log if batch_log else None
        self.epoch_path = output_dir / epoch_log if epoch_log else None
        self.event_path = output_dir / "events.jsonl"
        self.wandb_project = wandb_project
        self.wandb_entity = wandb_entity
        self.wandb_run_name = wandb_run_name
        self.run_id = run_id
        self.accelerator = accelerator
        self.start_callback = start_callback
        self.checkpoint_uploader = checkpoint_uploader or uploader_from_environment(
            output_dir, run_id
        )
        self.tracking_started = False
        self._best_signatures: dict[str, tuple[int, int]] = {}

    def _event(self, kind: str, **values: Any) -> None:
        _write_jsonl(self.event_path, {"kind": kind, **values})

    def on_start(self, **context: Any) -> None:
        config: Phase3Config = context["config"]
        self._event(
            "run_started",
            run_id=self.run_id,
            train_images=len(context["train_loader"].dataset),
            validation_images=len(context["val_loader"].dataset),
            batch_size=config.data.batch_size,
            epochs=config.schedule.epochs,
            world_size=context["world_size"],
        )
        print(
            "Training started: "
            f"run={self.run_id or 'local'} "
            f"train_images={len(context['train_loader'].dataset)} "
            f"validation_images={len(context['val_loader'].dataset)} "
            f"batch_size={config.data.batch_size} epochs={config.schedule.epochs}",
            flush=True,
        )
        if self.wandb_project is not None:
            if self.accelerator is None:
                raise RuntimeError("W&B reporting requires the training Accelerator")
            try:
                self.accelerator.init_trackers(
                    self.wandb_project,
                    config={
                        "training": asdict(config),
                        "run": {
                            "run_id": self.run_id,
                            "source_commit": os.getenv("SOURCE_COMMIT"),
                            "dataset_id": os.getenv("DATASET_ID"),
                            "dataset_manifest_sha256": os.getenv(
                                "DATASET_MANIFEST_SHA256"
                            ),
                            "config_path": os.getenv("CONFIG_PATH"),
                        },
                    },
                    init_kwargs={
                        "wandb": {
                            "id": self.run_id,
                            "resume": "allow",
                            "name": self.wandb_run_name or self.run_id,
                            "entity": self.wandb_entity,
                        }
                    },
                )
                self.tracking_started = True
            except Exception as error:
                self._event("tracking_error", operation="init", error=str(error))
                raise RuntimeError("failed to initialize W&B tracking") from error
            self._log_dataset_artifact()
        if self.start_callback is not None:
            self.start_callback(**context)

    def _log_dataset_artifact(self) -> None:
        """Best-effort W&B link to the immutable S3 dataset manifest."""
        bucket = os.getenv("S3_BUCKET")
        dataset_id = os.getenv("DATASET_ID")
        if not bucket or not dataset_id:
            return
        if endpoint := os.getenv("S3_ENDPOINT_URL"):
            # W&B's boto3 S3 reference handler uses this endpoint variable.
            os.environ.setdefault("AWS_S3_ENDPOINT_URL", endpoint)
        try:
            import wandb

            artifact = wandb.Artifact(f"dataset-{dataset_id}", type="dataset")
            artifact.add_reference(
                f"s3://{bucket}/datasets/{dataset_id}/dataset-manifest.json"
            )
            self.accelerator.get_tracker("wandb", unwrap=True).log_artifact(
                artifact, aliases=["used"]
            )
        except Exception as error:
            self._event(
                "tracking_error", operation="dataset_artifact", error=str(error)
            )
            print(
                f"W&B dataset artifact reference failed: {error}",
                file=sys.stderr,
                flush=True,
            )

    def _record(self, record: Mapping[str, Any], *, step: int) -> None:
        if self.batch_path is not None:
            _write_jsonl(self.batch_path, record)
        if self.tracking_started:
            try:
                self.accelerator.log(dict(record), step=step)
            except Exception as error:
                self._event(
                    "tracking_error", operation="log", error=str(error), step=step
                )
                print(f"W&B logging failed at step {step}: {error}", file=sys.stderr)

    def on_batch(
        self,
        *,
        metrics: Mapping[str, float],
        epoch: int,
        batch: int,
        batches_per_epoch: int,
        global_step: int,
        optimizer: torch.optim.Optimizer,
        timings: Mapping[str, float] | None = None,
    ) -> None:
        record = {
            "train/epoch": epoch,
            "train/batch": batch,
            "train/batches_per_epoch": batches_per_epoch,
            **{f"train/{key}": value for key, value in metrics.items()},
            "train/lr/backbone": float(optimizer.param_groups[0]["lr"]),
            "train/lr/fpn_head": float(optimizer.param_groups[1]["lr"]),
            **{f"timing/{key}": value for key, value in (timings or {}).items()},
        }
        self._record(record, step=global_step)
        print(
            "Training progress: "
            f"epoch={epoch + 1} batch={batch}/{batches_per_epoch} "
            f"step={global_step} loss={float(metrics.get('loss', 0.0)):.6f}",
            flush=True,
        )

    def on_validation(
        self,
        *,
        state: str,
        metrics: Mapping[str, float],
        epoch: int,
        global_step: int,
        duration_seconds: float | None = None,
    ) -> None:
        prefix = f"val_{state}"
        record = {
            f"{prefix}/epoch": epoch,
            **{f"{prefix}/{key}": value for key, value in metrics.items()},
        }
        if duration_seconds is not None:
            record[f"timing/{prefix}_seconds"] = duration_seconds
        self._record(record, step=global_step)
        print(
            f"Validation complete: state={state} epoch={epoch + 1} "
            f"step={global_step} duration={duration_seconds or 0.0:.1f}s",
            flush=True,
        )

    def on_epoch(self, *, metrics: Mapping[str, Any]) -> None:
        if self.epoch_path is not None:
            _write_jsonl(self.epoch_path, metrics)

    def on_checkpoint(
        self,
        output_dir: Path,
        *,
        global_step: int,
        save_duration_seconds: float | None = None,
    ) -> None:
        checkpoint = output_dir / "last.pt"
        self._event(
            "checkpoint_saved",
            step=global_step,
            path=str(checkpoint),
            size=checkpoint.stat().st_size,
            save_duration_seconds=save_duration_seconds,
            disk_free_bytes=shutil.disk_usage(output_dir).free,
            upload_queue_depth=(
                self.checkpoint_uploader.pending
                if self.checkpoint_uploader is not None
                else 0
            ),
        )
        if self.checkpoint_uploader is not None:
            self.checkpoint_uploader.enqueue(checkpoint, global_step)
            for best in sorted(output_dir.glob("best*.pt")):
                stat = best.stat()
                signature = (stat.st_size, stat.st_mtime_ns)
                if self._best_signatures.get(best.name) == signature:
                    continue
                self.checkpoint_uploader.enqueue(
                    best, global_step, logical_name=best.name
                )
                self._best_signatures[best.name] = signature
        print(
            f"Checkpoint saved: step={global_step} path={checkpoint}",
            f"upload_queue={self.checkpoint_uploader.pending if self.checkpoint_uploader else 0}",
            flush=True,
        )

    def close(self) -> None:
        upload_error: BaseException | None = None
        if self.checkpoint_uploader is not None:
            try:
                self.checkpoint_uploader.close()
            except BaseException as error:
                upload_error = error
                self._event("checkpoint_upload_error", error=str(error))
        if self.tracking_started:
            bucket = os.getenv("S3_BUCKET")
            if bucket and self.run_id and self._best_signatures:
                try:
                    import wandb

                    artifact = wandb.Artifact(
                        f"checkpoints-{self.run_id}", type="model"
                    )
                    for name in sorted(self._best_signatures):
                        artifact.add_reference(
                            f"s3://{bucket}/runs/{self.run_id}/checkpoints/aliases/{name}.json",
                            name=name,
                        )
                    self.accelerator.get_tracker("wandb", unwrap=True).log_artifact(
                        artifact, aliases=["latest"]
                    )
                except Exception as error:
                    self._event(
                        "tracking_error", operation="artifact", error=str(error)
                    )
            self.accelerator.end_training()
        self._event("run_closed", upload_ok=upload_error is None)
        if upload_error is not None:
            raise upload_error
