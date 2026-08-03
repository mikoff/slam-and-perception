"""Run quad validation and write metric JSON plus HTML prediction overlays."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path

import torch
from PIL import Image, ImageDraw
from torch.utils.data import DataLoader

from student_detector.config import load_phase3_config
from student_detector.data import IMAGENET_MEAN, IMAGENET_STD
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_evaluation import QuadEvaluationImage, evaluate_quad_proposals
from student_detector.quad_geometry import aligned_quad_iou
from student_detector.quad_targets import QuadTargetBuilder


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/phase3.yaml"))
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/phase3/validation_visuals"))
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-images", type=int, default=100)
    parser.add_argument("--score-threshold", type=float, default=0.05)
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--image-ids", help="Optional comma-separated image IDs")
    parser.add_argument("--raw-model", action="store_true", help="Load non-EMA weights")
    return parser.parse_args()


def _image(sample: object) -> Image.Image:
    tensor = sample.image.detach().cpu()
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    pixels = ((tensor * std + mean).clamp(0, 1) * 255).byte().permute(1, 2, 0).numpy()
    return Image.fromarray(pixels, mode="RGB")


def _draw_quad(draw: ImageDraw.ImageDraw, quad: torch.Tensor, color: tuple[int, int, int], width: int) -> None:
    points = [(round(float(point[0])), round(float(point[1]))) for point in quad]
    draw.line(points + [points[0]], fill=color, width=width, joint="curve")


@torch.no_grad()
def main() -> None:
    args = _args()
    config = load_phase3_config(args.config)
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else args.device)
    if args.device == "auto" and device.type != "cuda":
        device = torch.device("cpu")
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = QuadProposalDetector(pretrained_backbone=False).to(device)
    state = checkpoint["model"] if args.raw_model else checkpoint.get("ema_model", checkpoint["model"])
    model.load_state_dict(state)
    model.eval()

    dataset = QuadProposalDataset(
        (
            config.data.quad_train_annotations or config.data.train_annotations
            if args.split == "train"
            else config.data.quad_val_annotations or config.data.val_annotations
        ),
        config.data.image_root,
        config.data.index_dir / f"quad_{args.split}.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    if args.image_ids:
        requested = {int(value) for value in args.image_ids.split(",") if value}
        dataset.records = [record for record in dataset.records if record.image_id in requested]
        missing = requested - {record.image_id for record in dataset.records}
        if missing:
            raise ValueError(f"image IDs are not in the {args.split} manifest: {sorted(missing)}")
    loader = DataLoader(
        dataset,
        batch_size=config.data.batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=collate_quad_proposal_samples,
    )
    assigner = QuadAssigner(
        strides=config.assignment.strides,
        top_k=config.quad.top_k,
        gamma=config.quad.gamma,
        scale_sigma=config.quad.scale_sigma,
        eligible_levels=config.quad.eligible_levels,
        scale_measure=config.quad.scale_measure,
    )
    target_builder = QuadTargetBuilder(assigner)
    decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=config.inference.nms_iou_threshold,
        max_proposals=config.inference.pre_nms_top_k,
    )
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    evaluated: list[QuadEvaluationImage] = []
    cards: list[str] = []
    seen = 0
    instance_rows: list[dict[str, object]] = []
    for images, samples in loader:
        if seen >= args.max_images:
            break
        images_device = images.to(device)
        output = model(images_device)
        shapes = tuple((level.shape[-2], level.shape[-1]) for level in output.quality)
        targets = target_builder(samples, shapes, device=device)
        detections = decoder(output, (images.shape[-2], images.shape[-1]), targets.valid_point_masks)
        for sample, detection in zip(samples, detections, strict=True):
            if seen >= args.max_images:
                break
            keep = detection.scores >= args.score_threshold
            detection_quads = detection.quads[keep].cpu()
            detection_scores = detection.scores[keep].cpu()
            evaluated.append(QuadEvaluationImage(
                image_id=sample.image_id,
                domain=sample.domain,
                camera_type=sample.camera_type,
                image_size=(images.shape[-2], images.shape[-1]),
                ground_truth=sample.quads.cpu(),
                ignore_quads=sample.ignore_quads.cpu(),
                detection=type(detection)(detection_quads, detection_scores),
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
            for gt_index, (quad, tier) in enumerate(
                zip(sample.quads, sample.geometry_tiers, strict=True)
            ):
                if detection_quads.numel():
                    overlaps = aligned_quad_iou(
                        detection_quads,
                        quad.cpu().expand_as(detection_quads),
                    )
                    best_iou, best_index = overlaps.max(dim=0)
                    best_score = detection_scores[best_index]
                else:
                    best_iou = torch.tensor(0.0)
                    best_score = torch.tensor(0.0)
                instance_rows.append({
                    "image_id": sample.image_id,
                    "gt_index": gt_index,
                    "geometry_tier": tier,
                    "best_iou": float(best_iou),
                    "matched_proposal_score": float(best_score),
                })
            canvas = _image(sample)
            draw = ImageDraw.Draw(canvas)
            for quad in sample.quads:
                _draw_quad(draw, quad, (40, 210, 80), 2)  # ground truth
            for quad in sample.ignore_quads:
                _draw_quad(draw, quad, (245, 190, 30), 2)  # ignore
            for quad, score in zip(detection_quads[:100], detection_scores[:100], strict=True):
                _draw_quad(draw, quad, (230, 50, 50), 1)  # proposal
                label = f"{float(score):.2f}"
                x, y = round(float(quad[:, 0].min())), round(float(quad[:, 1].min()))
                draw.text((x, max(0, y - 12)), label, fill=(230, 50, 50), stroke_width=1, stroke_fill="white")
            filename = f"{seen:04d}_{sample.image_id}.jpg"
            canvas.save(output_dir / filename, quality=92)
            cards.append(
                f"<figure><img loading='lazy' src='{html.escape(filename)}' width='384'>"
                f"<figcaption>image_id={sample.image_id}, domain={html.escape(sample.domain)}, "
                f"tiers={html.escape(','.join(sample.geometry_tiers))}</figcaption></figure>"
            )
            seen += 1
    metrics = evaluate_quad_proposals(evaluated)
    metrics.update({"images": seen, "checkpoint": str(args.checkpoint), "device": str(device)})
    metrics["instances"] = instance_rows  # type: ignore[assignment]
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>Quad validation</title>"
        "<style>figure{display:inline-block;vertical-align:top;margin:8px}figcaption{max-width:384px}</style>"
        + "<h1>Quad validation overlays</h1><p>green=GT, yellow=ignore, red=proposal</p>"
        + "<div>" + "".join(cards) + "</div>\n",
        encoding="utf-8",
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
