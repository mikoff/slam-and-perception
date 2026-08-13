"""Quadrilateral task adapter for the shared proposal training runtime."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import torch
from torch import Tensor, nn

from .checkpoints import architecture_id, load_model_state_strict
from .config import Phase3Config
from .provenance import write_run_contract
from .quad_decoder import QuadDetection, QuadInferenceDecoder, decode_dense_quad_output
from .quad_evaluation import QuadEvaluationAccumulator, QuadEvaluationImage
from .quad_geometry import pairwise_quad_iou, quad_validity
from .quad_losses import QuadLossOutput, QuadProposalLoss
from .quad_targets import QuadTargetBuilder
from .training_reporting import StandardReporter
from .training_runtime import train_proposals


def _feature_shapes(output: Any) -> tuple[tuple[int, int], ...]:
    return tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.quality)


def _append_quad_evaluations(
    evaluated: list[QuadEvaluationImage],
    images: Tensor,
    samples: Sequence[Any],
    detections: Sequence[Any],
    dense_quads: Tensor,
    dense_scores: Tensor,
    targets: Any,
    *,
    include_dense_diagnostics: bool,
) -> None:
    valid_points = torch.cat(
        [mask.reshape(mask.shape[0], -1) for mask in targets.valid_point_masks], dim=1
    )
    for sample_index, (sample, detection) in enumerate(
        zip(samples, detections, strict=True)
    ):
        assigned_mask = targets.positive_mask[sample_index]
        assigned = QuadDetection(
            dense_quads[sample_index, assigned_mask].detach(),
            dense_scores[sample_index, assigned_mask].detach(),
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
                valid_points[sample_index] & in_bounds & quad_validity(candidate_quads)
            )
            dense_detection = QuadDetection(
                candidate_quads[keep_dense].detach(),
                dense_scores[sample_index, keep_dense].detach(),
            )
        evaluated.append(
            QuadEvaluationImage(
                image_id=sample.image_id,
                domain=sample.domain,
                camera_type=sample.camera_type,
                image_size=(images.shape[-2], images.shape[-1]),
                ground_truth=sample.quads.to(images.device),
                ignore_quads=sample.ignore_quads.to(images.device),
                detection=type(detection)(
                    detection.quads.detach(),
                    detection.scores.detach(),
                    candidate_count=detection.candidate_count,
                    invalid_candidate_count=detection.invalid_candidate_count,
                ),
                pre_nms_detection=(
                    type(detection)(
                        detection.pre_nms_quads.detach(),
                        detection.pre_nms_scores.detach(),
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
                assigned_gt_indices=targets.matched_gt_indices[
                    sample_index, assigned_mask
                ].detach(),
                positive_scores=dense_scores[
                    sample_index, targets.positive_mask[sample_index]
                ].detach(),
                trusted_background_scores=dense_scores[
                    sample_index, targets.trusted_background_mask[sample_index]
                ].detach(),
                weak_background_scores=dense_scores[
                    sample_index, targets.weak_background_mask[sample_index]
                ].detach(),
            )
        )


@torch.no_grad()
def validate_quad_states(
    models: Mapping[str, nn.Module],
    loader: Any,
    target_builder: QuadTargetBuilder,
    decoder: QuadInferenceDecoder,
    device: torch.device,
    *,
    max_batches: int | None = None,
    include_dense_diagnostics: bool = False,
) -> dict[str, dict[str, float]]:
    """Evaluate multiple weight states in one loader pass on the target device."""
    for model in models.values():
        model.eval()
    accumulators = {state: QuadEvaluationAccumulator() for state in models}
    if device.type == "cuda":
        print("[Validation] Warming exact quadrilateral IoU kernel", flush=True)
        warmup = torch.tensor(
            [[[0.0, 0.0], [8.0, 0.0], [8.0, 8.0], [0.0, 8.0]]],
            device=device,
        )
        pairwise_quad_iou(warmup, warmup)
    for batch_index, (images, samples) in enumerate(loader):
        if max_batches is not None and batch_index >= max_batches:
            break
        images = images.to(device, non_blocking=device.type == "cuda")
        first_state = next(iter(models))
        first_output = models[first_state](images)
        targets = target_builder(samples, _feature_shapes(first_output), device=device)
        for state, model in models.items():
            output = first_output if state == first_state else model(images)
            detections = decoder(
                output,
                (images.shape[-2], images.shape[-1]),
                targets.valid_point_masks,
            )
            dense_quads, dense_scores = decode_dense_quad_output(
                output, decoder.strides
            )
            batch_evaluations: list[QuadEvaluationImage] = []
            _append_quad_evaluations(
                batch_evaluations,
                images,
                samples,
                detections,
                dense_quads,
                dense_scores,
                targets,
                include_dense_diagnostics=include_dense_diagnostics,
            )
            accumulators[state].update(batch_evaluations)
    return {state: accumulator.compute() for state, accumulator in accumulators.items()}


@torch.no_grad()
def validate_quad(
    model: nn.Module,
    loader: Any,
    target_builder: QuadTargetBuilder,
    decoder: QuadInferenceDecoder,
    device: torch.device,
    *,
    max_batches: int | None = None,
    include_dense_diagnostics: bool = False,
) -> dict[str, float]:
    return validate_quad_states(
        {"model": model},
        loader,
        target_builder,
        decoder,
        device,
        max_batches=max_batches,
        include_dense_diagnostics=include_dense_diagnostics,
    )["model"]


class _QuadTask:
    checkpoint_phase: int | str = "quad_proposals"
    validation_states = ("raw", "ema")

    def __init__(
        self,
        target_builder: QuadTargetBuilder,
        criterion: QuadProposalLoss,
        config: Phase3Config,
    ) -> None:
        self.target_builder = target_builder
        self.criterion = criterion
        self.config = config
        self.freeze_batch_norm = config.pretrained_backbone
        self.architecture = architecture_id("quad", config.neck_type)
        self.decoder = QuadInferenceDecoder(
            strides=config.assignment.strides,
            pre_nms_top_k=config.inference.pre_nms_top_k,
            nms_iou_threshold=config.inference.nms_iou_threshold,
            max_proposals=config.inference.max_proposals,
        )

    def build_targets(
        self, samples: Sequence[object], output: Any, device: torch.device
    ) -> Any:
        return self.target_builder(samples, _feature_shapes(output), device=device)

    def compute_loss(self, output: Any, targets: Any) -> QuadLossOutput:
        return self.criterion(output, targets)

    def validate(
        self,
        model: nn.Module,
        loader: Any,
        device: torch.device,
        max_batches: int | None,
    ) -> dict[str, float]:
        metrics = validate_quad(
            model,
            loader,
            self.target_builder,
            self.decoder,
            device,
            max_batches=max_batches,
            include_dense_diagnostics=False,
        )
        metrics.update(
            {
                "quality_target_mode": self.criterion.quality_target_mode,
                "geometry_quality_target": self.criterion.geometry_quality_target,
                "corner_smooth_l1_beta": self.criterion.corner_smooth_l1_beta,
                "gwd_weight": self.criterion.gwd_weight,
            }
        )
        return metrics

    def validate_states(
        self,
        models: Mapping[str, nn.Module],
        loader: Any,
        device: torch.device,
        max_batches: int | None,
    ) -> dict[str, dict[str, float]]:
        results = validate_quad_states(
            models,
            loader,
            self.target_builder,
            self.decoder,
            device,
            max_batches=max_batches,
            include_dense_diagnostics=False,
        )
        for metrics in results.values():
            metrics.update(
                {
                    "quality_target_mode": self.criterion.quality_target_mode,
                    "geometry_quality_target": self.criterion.geometry_quality_target,
                    "corner_smooth_l1_beta": self.criterion.corner_smooth_l1_beta,
                    "gwd_weight": self.criterion.gwd_weight,
                }
            )
        return results

    def batch_metrics(
        self, losses: QuadLossOutput, targets: Any
    ) -> dict[str, Tensor | float]:
        return {
            "loss": losses.total,
            "quality_loss": losses.quality,
            "quality_positive_loss": losses.quality_positive,
            "quality_trusted_background_loss": losses.quality_trusted_background,
            "quality_weak_background_loss": losses.quality_weak_background,
            "weak_negative_weight": targets.weak_negative_weight,
            "corner_loss": losses.corner,
            "gwd_loss": losses.gwd,
            "validity_loss": losses.validity,
            "positive_count": losses.number_positive,
            "quality_target_mean": losses.quality_target_mean,
            "fallback_count": targets.fallback_count,
            "unrepresentable_count": targets.unrepresentable_count,
        }

    def check_finite(self, output: Any, losses: QuadLossOutput) -> None:
        tensors = (*output.quality, *output.corner_offsets)
        if not all(torch.isfinite(tensor).all() for tensor in tensors):
            raise FloatingPointError("non-finite quad model output")
        if not torch.isfinite(losses.total):
            raise FloatingPointError(
                "non-finite quad loss: "
                f"quality={float(losses.quality.detach().float().cpu())}, "
                f"corner={float(losses.corner.detach().float().cpu())}, "
                f"gwd={float(losses.gwd.detach().float().cpu())}, "
                f"validity={float(losses.validity.detach().float().cpu())}"
            )

    def on_epoch_complete(self, epoch: int, total_epochs: int) -> None:
        warmup_epochs = max(1, total_epochs // 5)
        if epoch + 1 < warmup_epochs:
            self.criterion.quality_target_mode = "centerness"
            self.criterion.quality_blend = 0.0
        elif epoch + 1 < 2 * warmup_epochs:
            self.criterion.quality_target_mode = "blend"
            self.criterion.quality_blend = min(
                1.0, (epoch + 1 - warmup_epochs) / warmup_epochs
            )
        else:
            self.criterion.quality_target_mode = "iou"
            self.criterion.quality_blend = 1.0

    def checkpoint_extra(self) -> dict[str, Any]:
        return {
            "quality_curriculum": {
                "mode": self.criterion.quality_target_mode,
                "blend": self.criterion.quality_blend,
                "geometry_quality_target": self.criterion.geometry_quality_target,
                "corner_smooth_l1_beta": self.criterion.corner_smooth_l1_beta,
                "gwd_weight": self.criterion.gwd_weight,
            }
        }

    def restore_checkpoint_extra(self, state: Mapping[str, Any]) -> None:
        curriculum = state.get("quality_curriculum", {})
        self.criterion.quality_target_mode = str(
            curriculum.get("mode", self.criterion.quality_target_mode)
        )
        self.criterion.quality_blend = float(
            curriculum.get("blend", self.criterion.quality_blend)
        )

    def load_model_state(self, model: nn.Module, checkpoint: Mapping[str, Any]) -> None:
        load_model_state_strict(
            model,
            dict(checkpoint),
            kind="quad",
            neck_type=self.config.neck_type,
        )

    def selection_score(self, metrics: Mapping[str, float]) -> float:
        return float(metrics.get("ar/100", 0.0))

    def best_checkpoint_names(self, state: str) -> tuple[str, ...]:
        if state == "raw":
            return ("best_raw.pt", "best.pt")
        return ("best_ema.pt",)

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
        return dict(validation) if validation else dict(previous_metrics)


def _write_quad_run_contract(**context: Any) -> None:
    write_run_contract(
        context["config"].output_dir,
        config=context["config"],
        model=context["model"],
        repository=Path(__file__).resolve().parents[3],
        train_images=len(context["train_loader"].dataset),
        validation_images=len(context["val_loader"].dataset),
        batches_per_epoch=len(context["train_loader"]),
        optimizer_steps=context["optimizer_steps"],
        world_size=context["world_size"],
    )


def train_quad_proposals(
    model: nn.Module,
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
    resume: Path | None = None,
    validation_interval: int = 1,
    wandb_project: str | None = None,
    wandb_entity: str | None = None,
    wandb_run_name: str | None = None,
    run_id: str | None = None,
    accelerator: Any | None = None,
) -> dict[str, Any]:
    """Train the quad proposal task through the shared runtime."""
    task = _QuadTask(target_builder, criterion, config)
    return train_proposals(
        model,
        train_loader,
        val_loader,
        task,
        config,
        device,
        max_steps=max_steps,
        max_val_batches=max_val_batches,
        log_interval=log_interval,
        resume=resume,
        validation_interval=validation_interval,
        accelerator=accelerator,
        reporter=StandardReporter(
            config.output_dir,
            batch_log="quad_metrics.jsonl",
            wandb_project=wandb_project,
            wandb_entity=wandb_entity,
            wandb_run_name=wandb_run_name,
            run_id=run_id,
            accelerator=accelerator,
            start_callback=_write_quad_run_contract,
        ),
    )


__all__ = ["train_quad_proposals", "validate_quad", "validate_quad_states"]
