"""Compare raw and EMA quad geometry at assigned validation locations."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from student_detector.config import load_phase3_config
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_decoder import QuadInferenceDecoder, decode_dense_quad_output
from student_detector.quad_geometry import aligned_quad_iou, quad_area, quad_validity
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import validate_quad


@torch.no_grad()
def _measure(
    state: dict[str, torch.Tensor], loader: DataLoader, config: object, device: torch.device
) -> dict[str, object]:
    model = QuadProposalDetector(pretrained_backbone=False).to(device)
    model.load_state_dict(state)
    model.eval()
    builder = QuadTargetBuilder(QuadAssigner(
        strides=config.assignment.strides,
        top_k=config.quad.top_k,
        gamma=config.quad.gamma,
        scale_sigma=config.quad.scale_sigma,
        eligible_levels=config.quad.eligible_levels,
        scale_measure=config.quad.scale_measure,
    ))
    ious, errors, predicted_scale, target_scale, validity = [], [], [], [], []
    for images, samples in loader:
        images = images.to(device)
        output = model(images)
        shapes = tuple((value.shape[-2], value.shape[-1]) for value in output.quality)
        targets = builder(samples, shapes, device=device)
        predicted, _ = decode_dense_quad_output(output, config.assignment.strides)
        for index, sample in enumerate(samples):
            positive = targets.positive_mask[index]
            if not positive.any():
                continue
            prediction = predicted[index, positive].float()
            gt_indices = targets.matched_gt_indices[index, positive].cpu()
            target = sample.quads[gt_indices].to(device).float()
            valid = quad_validity(prediction)
            validity.append(valid.float().cpu())
            pair_iou = prediction.new_zeros(prediction.shape[0])
            if valid.any():
                pair_iou[valid] = aligned_quad_iou(prediction[valid], target[valid])
            ious.append(pair_iou.cpu())
            center_error = torch.linalg.vector_norm(
                prediction.mean(dim=1) - target.mean(dim=1), dim=1
            ) / torch.sqrt(quad_area(target).clamp(min=1e-7))
            errors.append(center_error.cpu())
            predicted_scale.append(torch.sqrt(quad_area(prediction).abs().clamp(min=0)).cpu())
            target_scale.append(torch.sqrt(quad_area(target).clamp(min=0)).cpu())
    joined = lambda values: torch.cat(values).float()  # noqa: E731
    geometry = {
        "assigned_locations": float(sum(value.numel() for value in ious)),
        "valid_fraction": float(joined(validity).mean()),
        "iou_median": float(joined(ious).median()),
        "iou_mean": float(joined(ious).mean()),
        "center_error_normalized_median": float(joined(errors).median()),
        "predicted_sqrt_area_median": float(joined(predicted_scale).median()),
        "target_sqrt_area_median": float(joined(target_scale).median()),
    }
    decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_proposals=300,
    )
    proposal_metrics = validate_quad(
        model,
        loader,
        builder,
        decoder,
        device,
        include_dense_diagnostics=True,
    )
    return {"assigned_geometry": geometry, "proposal_metrics": proposal_metrics}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    config = load_phase3_config(args.config)
    device = torch.device(args.device)
    manifest = config.data.quad_val_annotations or config.data.val_annotations
    dataset = QuadProposalDataset(
        manifest, config.data.image_root,
        config.data.index_dir / "quad_checkpoint_diagnostic.sqlite",
        config.data, config.augmentation, training=False, force_index=True,
    )
    loader = DataLoader(
        dataset, batch_size=config.data.batch_size, shuffle=False,
        num_workers=config.data.workers, collate_fn=collate_quad_proposal_samples,
    )
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    checkpoint_decay = float(
        checkpoint.get(
            "ema_decay",
            checkpoint.get("config", {}).get("schedule", {}).get(
                "ema_decay", config.schedule.ema_decay
            ),
        )
    )
    report = {
        "checkpoint": str(args.checkpoint.resolve()),
        "global_step": checkpoint.get("global_step"),
        "ema_decay": checkpoint_decay,
        "ema_initialization_weight": checkpoint.get(
            "ema_initialization_weight",
            checkpoint_decay ** int(checkpoint.get("global_step", 0)),
        ),
        "raw_model": _measure(checkpoint["model"], loader, config, device),
        "ema_model": _measure(checkpoint["ema_model"], loader, config, device),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
