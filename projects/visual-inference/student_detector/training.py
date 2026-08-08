"""HBB proposal task adapter for the shared training runtime."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
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
from .training_reporting import StandardReporter
from .training_runtime import train_proposals


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
        shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
        )
        valid_by_sample = [
            point_validity_from_pixel_mask(
                sample.valid_mask.to(output.objectness[0].device),
                shapes,
                decoder.strides,
            )[1]
            for sample in samples
        ]
        level_masks = tuple(
            torch.stack(
                [valid_by_sample[batch][level] for batch in range(len(samples))]
            )
            for level in range(len(shapes))
        )
        detections = decoder(
            output, (images.shape[-2], images.shape[-1]), level_masks
        )
        for sample, detection in zip(samples, detections, strict=True):
            evaluated.append(
                EvaluationImage(
                    sample.image_id,
                    sample.domain,
                    sample.camera_type,
                    (images.shape[-2], images.shape[-1]),
                    sample.boxes.cpu(),
                    sample.ignore_boxes.cpu(),
                    type(detection)(detection.boxes.cpu(), detection.scores.cpu()),
                )
            )
    return evaluate_proposals(evaluated)


class _HbbTask:
    checkpoint_phase: int | str = 3
    architecture = "mobilenetv4_conv_medium_lite_fpn96_fcos_atss_reg2"
    freeze_batch_norm = True

    def __init__(
        self,
        target_builder: TargetBuilder,
        criterion: ProposalLoss,
        config: Phase3Config,
        *,
        use_ema_for_validation: bool,
    ) -> None:
        self.target_builder = target_builder
        self.criterion = criterion
        self.decoder = InferenceDecoder(
            strides=config.assignment.strides,
            top_k=config.inference.pre_nms_top_k,
            nms_iou_threshold=config.inference.nms_iou_threshold,
            max_detections=config.inference.max_proposals,
            score_mode=config.inference.score_mode,
        )
        self.validation_states = (
            ("ema",) if use_ema_for_validation else ("raw",)
        )

    def build_targets(
        self, samples: Sequence[object], output: Any, device: torch.device
    ) -> Any:
        shapes = tuple(
            (tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness
        )
        return self.target_builder(samples, shapes, device=device)

    def compute_loss(self, output: Any, targets: Any) -> LossOutput:
        return self.criterion(output, targets)

    def validate(
        self,
        model: nn.Module,
        loader: Any,
        device: torch.device,
        max_batches: int | None,
    ) -> dict[str, float]:
        return validate(
            model,  # type: ignore[arg-type]
            loader,
            self.target_builder,
            self.decoder,
            device,
            max_batches=max_batches,
        )

    def batch_metrics(
        self, losses: LossOutput, targets: Any
    ) -> dict[str, Tensor | float]:
        values: dict[str, Tensor | float] = {
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
        for level, count in enumerate(targets.positive_counts_per_level):
            values[f"positive/P{level + 3}"] = count
        return values

    def check_finite(self, output: Any, losses: LossOutput) -> None:
        if not torch.isfinite(losses.total):
            raise FloatingPointError("non-finite HBB proposal loss")

    def on_epoch_complete(self, epoch: int, total_epochs: int) -> None:
        pass

    def checkpoint_extra(self) -> dict[str, Any]:
        return {}

    def restore_checkpoint_extra(self, state: Mapping[str, Any]) -> None:
        pass

    def load_model_state(
        self, model: nn.Module, checkpoint: Mapping[str, Any]
    ) -> None:
        model.load_state_dict(checkpoint["model"], strict=True)

    def selection_score(self, metrics: Mapping[str, float]) -> float:
        return float(metrics.get("selection_score", 0.0))

    def best_checkpoint_names(self, state: str) -> tuple[str, ...]:
        return ("best.pt",)

    def finalize_epoch_metrics(
        self,
        training_totals: Mapping[str, float],
        batches: int,
        validation: Mapping[str, Mapping[str, float]],
        previous_metrics: Mapping[str, Any],
        epoch: int,
        global_step: int,
        optimizer: torch.optim.Optimizer,
    ) -> dict[str, Any]:
        train_metrics = {
            key: value / max(batches, 1)
            for key, value in training_totals.items()
        }
        train_metrics["fallback_rate_per_gt"] = (
            training_totals.get("number_fallback", 0.0)
            / max(training_totals.get("number_gt", 0.0), 1.0)
        )
        state_metrics = validation.get(self.validation_states[0], {})
        return {
            **train_metrics,
            **{f"val/{key}": value for key, value in state_metrics.items()},
            "epoch": float(epoch),
            "global_step": float(global_step),
            "lr/backbone": float(optimizer.param_groups[0]["lr"]),
            "lr/fpn_head": float(optimizer.param_groups[1]["lr"]),
        }


def train_phase3(
    model: StudentDetector,
    train_loader: Any,
    val_loader: Any,
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
    """Train the HBB proposal task through the shared runtime."""
    task = _HbbTask(
        target_builder,
        criterion,
        config,
        use_ema_for_validation=use_ema_for_validation,
    )
    result = train_proposals(
        model,
        train_loader,
        val_loader,
        task,
        config,
        device,
        max_steps=max_steps,
        max_val_batches=max_val_batches,
        resume=resume,
        log_interval=log_interval,
        validation_interval=validation_interval,
        reporter=StandardReporter(
            config.output_dir,
            batch_log="progress.jsonl",
            epoch_log="metrics.jsonl",
        ),
    )
    selected_state = task.validation_states[0]
    return {
        "global_step": result["global_step"],
        "best_selection_score": result["best_scores"][selected_state],
        "metrics": result["metrics"],
    }
__all__ = ["train_phase3", "validate"]
