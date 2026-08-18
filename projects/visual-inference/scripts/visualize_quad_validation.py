"""Run quad validation and write metric JSON plus HTML prediction overlays."""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import torch
from PIL import Image
from torch.utils.data import DataLoader

from student_detector.checkpoints import load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.data import IMAGENET_MEAN, IMAGENET_STD, ImageRecord
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_evaluation import (
    QuadEvaluationImage,
    evaluate_quad_proposals,
)
from student_detector.quad_geometry import aligned_quad_iou
from student_detector.quad_targets import QuadTargetBuilder


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/phase3.yaml"))
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("artifacts/phase3/validation_visuals")
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--batch-size",
        type=int,
        help="Visualization batch size; defaults to the training config value",
    )
    parser.add_argument("--max-images", type=int, default=100)
    parser.add_argument(
        "--images-per-source",
        type=int,
        help=(
            "Deterministically sample this many images from every source dataset; "
            "overrides --max-images"
        ),
    )
    parser.add_argument("--score-threshold", type=float, default=0.4)
    parser.add_argument(
        "--nms-iou-threshold",
        type=float,
        default=0.7,
        help="Audit-only NMS IoU threshold (default: 0.7)",
    )
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--image-ids", help="Optional comma-separated image IDs")
    parser.add_argument("--raw-model", action="store_true", help="Load non-EMA weights")
    return parser.parse_args()


def _checkpoint_state_key(checkpoint: dict[str, object], *, raw_model: bool) -> str:
    """Select raw or EMA weights without assuming both exist."""
    state_key = (
        "model"
        if raw_model
        else ("ema_model" if "ema_model" in checkpoint else "model")
    )
    if state_key not in checkpoint:
        requested = "raw" if raw_model else "EMA or raw"
        raise ValueError(f"checkpoint does not contain requested {requested} weights")
    return state_key


def _select_records_per_source(
    records: list[ImageRecord], per_source: int
) -> tuple[list[ImageRecord], dict[str, int]]:
    """Select evenly spaced records per source, interleaved for visual review."""
    grouped: dict[str, list[ImageRecord]] = defaultdict(list)
    for record in records:
        grouped[record.source_dataset].append(record)
    insufficient = {
        source: len(source_records)
        for source, source_records in grouped.items()
        if len(source_records) < per_source
    }
    if insufficient:
        raise ValueError(
            f"not enough records for --images-per-source={per_source}: {insufficient}"
        )
    sources = sorted(grouped)
    selected_by_source: dict[str, list[ImageRecord]] = {}
    for source in sources:
        source_records = grouped[source]
        if per_source == 1:
            indices = [len(source_records) // 2]
        else:
            indices = [
                round(index * (len(source_records) - 1) / (per_source - 1))
                for index in range(per_source)
            ]
        selected_by_source[source] = [source_records[index] for index in indices]
    selected = [
        selected_by_source[source][index]
        for index in range(per_source)
        for source in sources
    ]
    return selected, {source: per_source for source in sources}


def _image(sample: object) -> Image.Image:
    tensor = sample.image.detach().cpu()
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    pixels = ((tensor * std + mean).clamp(0, 1) * 255).byte().permute(1, 2, 0).numpy()
    return Image.fromarray(pixels, mode="RGB")


def _quad_points(quad: torch.Tensor) -> list[list[float]]:
    return [[round(float(value), 3) for value in point] for point in quad]


def _html_document(
    records: list[dict[str, object]], *, initial_threshold: float
) -> str:
    serialized = json.dumps(records, ensure_ascii=True, separators=(",", ":")).replace(
        "</", "<\\/"
    )
    threshold = json.dumps(initial_threshold)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quad validation</title>
<style>
  :root {{ color-scheme: light dark; font-family: system-ui, sans-serif; }}
  body {{ margin: 16px; }}
  .legend {{ margin-bottom: 14px; }}
  .legend span {{ white-space: nowrap; margin-right: 14px; }}
  .swatch {{ display: inline-block; width: 12px; height: 12px; margin-right: 4px; }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(400px, 1fr)); gap: 14px; }}
  figure {{ margin: 0; padding: 8px; border: 1px solid #8887; border-radius: 6px; }}
  canvas {{ display: block; width: 384px; max-width: 100%; height: auto; background: #777; }}
  .controls {{ display: grid; gap: 6px; margin-top: 8px; max-width: 384px; }}
  .threshold-row {{ display: grid; grid-template-columns: auto 1fr 3.2em; gap: 8px; align-items: center; }}
  .toggles {{ display: flex; flex-wrap: wrap; gap: 10px; }}
  figcaption {{ margin-top: 6px; max-width: 384px; font-size: 0.9rem; }}
</style>
</head>
<body>
<h1>Quadrilateral validation</h1>
<p class="legend">
  <span><i class="swatch" style="background:#28d250"></i>validation ground truth</span>
  <span><i class="swatch" style="background:#f5be1e"></i>ignored/excluded supervision</span>
  <span><i class="swatch" style="background:#e63232"></i>ranked proposal</span>
</p>
<p>Red does not mean incorrect. Use the score slider and layer controls below each image.</p>
<main class="grid" id="grid"></main>
<script>
const records = {serialized};
const initialThreshold = {threshold};

function drawQuad(ctx, quad, stroke, width) {{
  if (!quad.length) return;
  ctx.beginPath();
  ctx.moveTo(quad[0][0], quad[0][1]);
  for (let index = 1; index < quad.length; index += 1) {{
    ctx.lineTo(quad[index][0], quad[index][1]);
  }}
  ctx.closePath();
  ctx.strokeStyle = stroke;
  ctx.lineWidth = width;
  ctx.lineJoin = "round";
  ctx.stroke();
}}

function scoreLabel(ctx, proposal) {{
  const label = proposal.score.toFixed(2);
  const x = Math.min(...proposal.quad.map(point => point[0]));
  const y = Math.max(11, Math.min(...proposal.quad.map(point => point[1])) - 3);
  ctx.font = "10px system-ui, sans-serif";
  const width = ctx.measureText(label).width + 4;
  ctx.fillStyle = "rgba(255,255,255,0.78)";
  ctx.fillRect(x, y - 10, width, 12);
  ctx.fillStyle = "rgba(190,20,20,0.95)";
  ctx.fillText(label, x + 2, y);
}}

function buildViewer(record) {{
  const figure = document.createElement("figure");
  figure.innerHTML = `
    <canvas width="384" height="384"></canvas>
    <div class="controls">
      <label class="threshold-row">
        <span>Proposal score</span>
        <input class="threshold" type="range" min="0" max="1" step="0.01">
        <output class="threshold-value"></output>
      </label>
      <div class="toggles">
        <label><input class="show-proposals" type="checkbox" checked> proposals</label>
        <label><input class="show-ground-truth" type="checkbox" checked> validation GT</label>
        <label><input class="show-ignore" type="checkbox" checked> ignored</label>
        <label><input class="show-scores" type="checkbox"> scores</label>
      </div>
    </div>
    <figcaption></figcaption>`;
  const canvas = figure.querySelector("canvas");
  const ctx = canvas.getContext("2d");
  const slider = figure.querySelector(".threshold");
  const thresholdValue = figure.querySelector(".threshold-value");
  const caption = figure.querySelector("figcaption");
  slider.value = initialThreshold;
  const image = new Image();

  function redraw() {{
    if (!image.complete || !image.naturalWidth) return;
    const cutoff = Number(slider.value);
    thresholdValue.value = cutoff.toFixed(2);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
    let shown = 0;
    if (figure.querySelector(".show-proposals").checked) {{
      for (const proposal of record.proposals) {{
        if (proposal.score < cutoff) continue;
        drawQuad(ctx, proposal.quad, "rgba(230,50,50,0.52)", 1.25);
        if (figure.querySelector(".show-scores").checked) scoreLabel(ctx, proposal);
        shown += 1;
      }}
    }}
    if (figure.querySelector(".show-ignore").checked) {{
      for (const quad of record.ignored) {{
        drawQuad(ctx, quad, "rgba(245,190,30,0.78)", 2);
      }}
    }}
    if (figure.querySelector(".show-ground-truth").checked) {{
      for (const quad of record.groundTruth) {{
        drawQuad(ctx, quad, "rgba(40,210,80,0.95)", 2.5);
      }}
    }}
    caption.textContent = `image_id=${{record.imageId}}, source=${{record.sourceDataset}}, `
      + `domain=${{record.domain}}, `
      + `proposals=${{shown}}/${{record.proposals.length}}, `
      + `GT=${{record.groundTruth.length}}, ignored=${{record.ignored.length}}`;
  }}

  slider.addEventListener("input", redraw);
  for (const control of figure.querySelectorAll("input[type=checkbox]")) {{
    control.addEventListener("change", redraw);
  }}
  image.addEventListener("load", redraw);
  image.src = record.image;
  return figure;
}}

const grid = document.getElementById("grid");
for (const record of records) grid.appendChild(buildViewer(record));
</script>
</body>
</html>
"""


@torch.no_grad()
def main() -> None:
    args = _args()
    config = load_phase3_config(args.config)
    device = torch.device(
        "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    )
    if args.device == "auto" and device.type != "cuda":
        device = torch.device("cpu")
    if args.batch_size is not None and args.batch_size < 1:
        raise ValueError("batch size must be positive")
    if args.max_images < 1:
        raise ValueError("max images must be positive")
    if args.images_per_source is not None and args.images_per_source < 1:
        raise ValueError("images per source must be positive")
    if args.images_per_source is not None and args.image_ids:
        raise ValueError("--images-per-source and --image-ids are mutually exclusive")
    if not 0 <= args.score_threshold <= 1:
        raise ValueError("score threshold must be between 0 and 1")
    if args.nms_iou_threshold is not None and not 0 <= args.nms_iou_threshold <= 1:
        raise ValueError("NMS IoU threshold must be between 0 and 1")
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type=config.neck_type,
    ).to(device)
    state_key = _checkpoint_state_key(checkpoint, raw_model=args.raw_model)
    load_model_state_strict(
        model,
        checkpoint,
        kind="quad",
        neck_type=config.neck_type,
        state_key=state_key,
    )
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
        dataset.records = [
            record for record in dataset.records if record.image_id in requested
        ]
        missing = requested - {record.image_id for record in dataset.records}
        if missing:
            raise ValueError(
                f"image IDs are not in the {args.split} manifest: {sorted(missing)}"
            )
    requested_source_counts: dict[str, int] | None = None
    if args.images_per_source is not None:
        dataset.records, requested_source_counts = _select_records_per_source(
            dataset.records, args.images_per_source
        )
    image_limit = (
        len(dataset.records) if args.images_per_source is not None else args.max_images
    )
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size or config.data.batch_size,
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
        nms_iou_threshold=(
            args.nms_iou_threshold
            if args.nms_iou_threshold is not None
            else config.inference.nms_iou_threshold
        ),
        max_proposals=config.inference.pre_nms_top_k,
    )
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    evaluated: list[QuadEvaluationImage] = []
    visualization_records: list[dict[str, object]] = []
    seen = 0
    source_counts: Counter[str] = Counter()
    instance_rows: list[dict[str, object]] = []
    for images, samples in loader:
        if seen >= image_limit:
            break
        images_device = images.to(device)
        output = model(images_device)
        shapes = tuple((level.shape[-2], level.shape[-1]) for level in output.quality)
        targets = target_builder(samples, shapes, device=device)
        detections = decoder(
            output, (images.shape[-2], images.shape[-1]), targets.valid_point_masks
        )
        for sample, detection in zip(samples, detections, strict=True):
            if seen >= image_limit:
                break
            detection_quads = detection.quads.cpu()
            detection_scores = detection.scores.cpu()
            evaluated.append(
                QuadEvaluationImage(
                    image_id=sample.image_id,
                    domain=sample.domain,
                    camera_type=sample.camera_type,
                    image_size=(images.shape[-2], images.shape[-1]),
                    ground_truth=sample.quads.cpu(),
                    ignore_quads=sample.ignore_quads.cpu(),
                    detection=type(detection)(detection_quads, detection_scores),
                    pre_nms_detection=(
                        type(detection)(
                            detection.pre_nms_quads.cpu(),
                            detection.pre_nms_scores.cpu(),
                        )
                        if detection.pre_nms_quads is not None
                        and detection.pre_nms_scores is not None
                        else None
                    ),
                    geometry_tiers=sample.geometry_tiers,
                )
            )
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
                instance_rows.append(
                    {
                        "image_id": sample.image_id,
                        "source_dataset": sample.source_dataset,
                        "gt_index": gt_index,
                        "geometry_tier": tier,
                        "best_iou": float(best_iou),
                        "matched_proposal_score": float(best_score),
                    }
                )
            filename = f"{seen:04d}_{sample.image_id}.jpg"
            _image(sample).save(output_dir / filename, quality=92)
            display_count = min(
                config.inference.max_proposals, detection_quads.shape[0]
            )
            visualization_records.append(
                {
                    "image": filename,
                    "imageId": sample.image_id,
                    "sourceDataset": sample.source_dataset,
                    "domain": sample.domain,
                    "groundTruth": [_quad_points(quad) for quad in sample.quads],
                    "ignored": [_quad_points(quad) for quad in sample.ignore_quads],
                    "proposals": [
                        {
                            "quad": _quad_points(quad),
                            "score": round(float(score), 6),
                        }
                        for quad, score in zip(
                            detection_quads[:display_count],
                            detection_scores[:display_count],
                            strict=True,
                        )
                    ],
                }
            )
            source_counts[sample.source_dataset] += 1
            seen += 1
    metrics = evaluate_quad_proposals(evaluated)
    metrics.update(
        {
            "images": seen,
            "checkpoint": str(args.checkpoint),
            "device": str(device),
            "nms_iou_threshold": (
                args.nms_iou_threshold
                if args.nms_iou_threshold is not None
                else config.inference.nms_iou_threshold
            ),
            "source_dataset_counts": dict(sorted(source_counts.items())),
            "requested_source_dataset_counts": requested_source_counts,
        }
    )
    metrics["instances"] = instance_rows  # type: ignore[assignment]
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output_dir / "index.html").write_text(
        _html_document(
            visualization_records,
            initial_threshold=args.score_threshold,
        ),
        encoding="utf-8",
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
