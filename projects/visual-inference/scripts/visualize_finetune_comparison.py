#!/usr/bin/env python3
"""Interactive Before & After Dashboard for Stage-2 Human GT Fine-Tuning Comparison."""

from __future__ import annotations

import argparse
import base64
import io
import json
from pathlib import Path
from typing import Any

from PIL import Image
import torch

from student_detector.decoder import InferenceDecoder
from student_detector.model import StudentDetector
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentHBBDataset,
)
from student_detector.targets import point_validity_from_pixel_mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build Before & After fine-tuning comparison dashboard."
    )
    parser.add_argument(
        "--before-checkpoint",
        type=Path,
        default=Path("artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt"),
        help="Path to pre-finetuning checkpoint (Baseline).",
    )
    parser.add_argument(
        "--after-checkpoint",
        type=Path,
        default=Path(
            "artifacts/finetune_10k_hbb/checkpoints/mobilenetv4_p2_hbb_finetune_last.pt"
        ),
        help="Path to post-finetuning checkpoint (Fine-Tuned).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=Path("artifacts/finetune_10k_hbb/training_hbb_finetune_summary.json"),
        help="Path to fine-tuning summary JSON.",
    )
    parser.add_argument(
        "--output-html",
        type=Path,
        default=Path("artifacts/finetune_10k_hbb/dashboard_finetune_comparison.html"),
        help="Output HTML path.",
    )
    parser.add_argument(
        "--pseudo-labels",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_PATH,
        help="Path to validation pseudo-labels.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to images directory.",
    )
    parser.add_argument(
        "--samples-per-cohort",
        type=int,
        default=15,
        help="Number of images per dataset cohort to embed (default: 15).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Device ('cuda' or 'cpu').",
    )
    return parser.parse_args()


def encode_thumbnail_base64(
    image_path: Path, max_dim: int = 960
) -> tuple[str, int, int]:
    with Image.open(image_path) as img:
        img = img.convert("RGB")
        orig_w, orig_h = img.size
        scale = min(max_dim / max(orig_w, orig_h), 1.0)
        new_w, new_h = int(orig_w * scale), int(orig_h * scale)
        if scale < 1.0:
            thumb = img.resize((new_w, new_h), Image.Resampling.BILINEAR)
        else:
            thumb = img
        buf = io.BytesIO()
        thumb.save(buf, format="JPEG", quality=82)
        encoded = base64.b64encode(buf.getvalue()).decode("utf-8")
        return f"data:image/jpeg;base64,{encoded}", orig_w, orig_h


def generate_comparison_dashboard(
    images_payload: list[dict[str, Any]],
    summary_data: dict[str, Any],
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload_json = json.dumps(images_payload)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MobileNetV4 P2-HBB: Stage-2 Human GT Fine-Tuning Comparison</title>
<style>
  :root {{
    --bg-dark: #0f172a;
    --bg-card: #1e293b;
    --text-primary: #f8fafc;
    --text-secondary: #94a3b8;
    --accent-blue: #38bdf8;
    --accent-green: #34d399;
    --accent-amber: #fbbf24;
    --accent-purple: #c084fc;
    --accent-rose: #f43f5e;
    --border-color: #334155;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    background-color: var(--bg-dark);
    color: var(--text-primary);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    line-height: 1.5;
    padding: 24px;
  }}
  header {{
    margin-bottom: 24px;
    border-bottom: 1px solid var(--border-color);
    padding-bottom: 16px;
  }}
  h1 {{ font-size: 1.8rem; font-weight: 700; color: var(--accent-blue); display: flex; align-items: center; gap: 12px; }}
  .subtitle {{ color: var(--text-secondary); font-size: 0.95rem; margin-top: 4px; }}
  .metrics-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 16px;
    margin-bottom: 24px;
  }}
  .card {{
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 16px;
  }}
  .card .label {{ font-size: 0.8rem; text-transform: uppercase; letter-spacing: 0.05em; color: var(--text-secondary); margin-bottom: 4px; }}
  .card .val-main {{ font-size: 1.6rem; font-weight: 700; color: var(--accent-green); }}
  .card .sub {{ font-size: 0.75rem; color: var(--text-secondary); margin-top: 4px; }}

  .controls {{
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 16px;
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 14px 20px;
    margin-bottom: 24px;
  }}
  .controls label {{ display: flex; align-items: center; gap: 8px; font-size: 0.9rem; color: var(--text-primary); cursor: pointer; }}
  .badge {{ display: inline-block; padding: 3px 10px; border-radius: 4px; font-size: 0.8rem; font-weight: 600; }}
  .badge-gt {{ background: #065f46; color: #a7f3d0; border: 1px solid #10b981; }}
  .badge-before {{ background: #1e3a8a; color: #bfdbfe; border: 1px solid #3b82f6; }}
  .badge-after {{ background: #581c87; color: #e9d5ff; border: 1px solid #a855f7; }}
  .badge-sam {{ background: #374151; color: #9ca3af; border: 1px dashed #6b7280; }}

  .gallery-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(460px, 1fr));
    gap: 20px;
  }}
  .img-card {{
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    overflow: hidden;
  }}
  .img-card-header {{
    padding: 10px 14px;
    background: rgba(15, 23, 42, 0.6);
    display: flex;
    justify-content: space-between;
    font-size: 0.82rem;
    color: var(--text-secondary);
    border-bottom: 1px solid var(--border-color);
  }}
  .canvas-wrapper {{
    position: relative;
    width: 100%;
    aspect-ratio: 16/9;
    background: #020617;
  }}
  canvas {{
    width: 100%;
    height: 100%;
    display: block;
    object-fit: contain;
  }}
</style>
</head>
<body>
<header>
  <h1><span>🔬</span> MobileNetV4 P2-HBB: Stage-2 Human GT Fine-Tuning Comparison</h1>
  <div class="subtitle">
    Before: 15 Epochs SAM Self-Training &bull; After: 5 Epochs High-Quality Human Ground Truth with Joint L1 + CIoU Loss
  </div>
</header>

<section class="metrics-grid">
  <div class="card">
    <div class="label">Pure Human GT Recall@100@50</div>
    <div class="val-main">19.38% <span style="font-size: 1rem; color: var(--accent-green);">(+21% relative)</span></div>
    <div class="sub">Baseline before FT was 15.97%</div>
  </div>

  <div class="card">
    <div class="label">Pure Human GT AP@50</div>
    <div class="val-main">1.87% <span style="font-size: 1rem; color: var(--accent-green);">(+57% relative)</span></div>
    <div class="sub">Baseline before FT was 1.19%</div>
  </div>

  <div class="card">
    <div class="label">Smooth-L1 Coordinate Loss</div>
    <div class="val-main" style="color: var(--accent-blue);">0.1757 <span style="font-size: 1rem; color: var(--accent-blue);">(-50.6%)</span></div>
    <div class="sub">Envelope tightening loss cut in half from 0.3557</div>
  </div>

  <div class="card">
    <div class="label">Fine-Tuning Setup</div>
    <div class="val-main" style="font-size: 1.15rem; color: var(--accent-amber);">10,000 Balanced Images</div>
    <div class="sub">NuImages + BDD100k + WoodScape + COCO</div>
  </div>
</section>

<div class="controls">
  <label>
    <input type="checkbox" id="show-gt" checked>
    <span class="badge badge-gt">Human Ground Truth</span>
  </label>
  <label>
    <input type="checkbox" id="show-before" checked>
    <span class="badge badge-before">Before (SAM 15-ep Baseline)</span>
  </label>
  <label>
    <input type="checkbox" id="show-after" checked>
    <span class="badge badge-after">After (Human GT Fine-Tuned)</span>
  </label>
  <label>
    <input type="checkbox" id="show-sam">
    <span class="badge badge-sam">SAM Discovery Targets</span>
  </label>
  <label style="margin-left: auto;">
    Score Threshold:
    <input type="range" id="score-threshold" min="0.05" max="0.90" step="0.05" value="0.25">
    <span id="score-val" style="font-weight: 700; color: var(--accent-blue); width: 36px;">0.25</span>
  </label>
</div>

<section class="gallery-grid" id="gallery"></section>

<script>
const imagesData = {payload_json};

function renderCards() {{
  const gallery = document.getElementById('gallery');
  gallery.innerHTML = '';
  const showGT = document.getElementById('show-gt').checked;
  const showBefore = document.getElementById('show-before').checked;
  const showAfter = document.getElementById('show-after').checked;
  const showSAM = document.getElementById('show-sam').checked;
  const scoreThresh = parseFloat(document.getElementById('score-threshold').value);

  imagesData.forEach((item, idx) => {{
    const card = document.createElement('div');
    card.className = 'img-card';
    card.innerHTML = `
      <div class="img-card-header">
        <span><strong>#${{item.image_id}}</strong> (${{item.source_dataset}})</span>
        <span>${{item.width}}x${{item.height}} | GT: ${{item.ground_truth.length}} | SAM: ${{item.sam_novel.length}}</span>
      </div>
      <div class="canvas-wrapper">
        <canvas id="canvas-${{idx}}"></canvas>
      </div>
    `;
    gallery.appendChild(card);

    const canvas = card.querySelector('canvas');
    const ctx = canvas.getContext('2d');
    const img = new Image();
    img.src = item.thumbnail_base64;
    img.onload = () => {{
      canvas.width = img.width;
      canvas.height = img.height;
      ctx.drawImage(img, 0, 0);

      const sx = img.width / item.width;
      const sy = img.height / item.height;

      // Draw SAM Discovery Targets (Dashed gray)
      if (showSAM && item.sam_novel) {{
        ctx.strokeStyle = 'rgba(156, 163, 175, 0.7)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        item.sam_novel.forEach(b => {{
          ctx.strokeRect(b[0] * sx, b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy);
        }});
        ctx.setLineDash([]);
      }}

      // Draw Human GT (Green)
      if (showGT && item.ground_truth) {{
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 2.5;
        item.ground_truth.forEach(b => {{
          ctx.strokeRect(b[0] * sx, b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy);
        }});
      }}

      // Draw Before Proposals (Cyan / Blue)
      if (showBefore && item.proposals_before) {{
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2.0;
        item.proposals_before.forEach((b, i) => {{
          const score = item.scores_before[i];
          if (score >= scoreThresh) {{
            ctx.strokeRect(b[0] * sx, b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy);
          }}
        }});
      }}

      // Draw After Proposals (Purple)
      if (showAfter && item.proposals_after) {{
        ctx.strokeStyle = '#c084fc';
        ctx.lineWidth = 2.0;
        item.proposals_after.forEach((b, i) => {{
          const score = item.scores_after[i];
          if (score >= scoreThresh) {{
            ctx.strokeRect(b[0] * sx, b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy);
          }}
        }});
      }}
    }};
  }});
}}

document.getElementById('show-gt').addEventListener('change', renderCards);
document.getElementById('show-before').addEventListener('change', renderCards);
document.getElementById('show-after').addEventListener('change', renderCards);
document.getElementById('show-sam').addEventListener('change', renderCards);
document.getElementById('score-threshold').addEventListener('input', (e) => {{
  document.getElementById('score-val').textContent = parseFloat(e.target.value).toFixed(2);
  renderCards();
}});

window.addEventListener('DOMContentLoaded', renderCards);
</script>
</body>
</html>
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Comparison Dashboard generated -> {output_path}")


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    with open(args.summary_json, encoding="utf-8") as f:
        summary_data = json.load(f)

    with open(args.pseudo_labels, encoding="utf-8") as f:
        val_meta = json.load(f)

    # Map image_id -> raw annotations
    ann_by_img_id: dict[int, dict[str, list[list[float]]]] = {}
    for img_rec in val_meta["images"]:
        iid = img_rec["image_id"]
        gt_boxes = [
            a["bbox"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "human_gt" and a.get("is_positive")
        ]
        sam_boxes = [
            a["bbox"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "sam_discovered" and a.get("is_positive")
        ]
        ann_by_img_id[iid] = {"gt": gt_boxes, "sam": sam_boxes}

    # Load dataset for model inference
    val_dataset = SAMStudentHBBDataset(
        pseudo_labels_path=args.pseudo_labels,
        image_root=args.image_root,
        split="val",
        val_ratio=0.20,
        input_size=384,
    )

    strides = (4, 8, 16, 32)
    decoder = InferenceDecoder(
        strides=strides,
        top_k=1000,
        nms_iou_threshold=0.45,
        max_detections=100,
        score_mode="objectness_x_centerness",
        centerness_alpha=1.0,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
    )

    # Models
    model_before = StudentDetector(
        pretrained_backbone=False, neck_type="lite", strides=strides, fpn_channels=96
    ).to(device)
    ckpt_b = torch.load(args.before_checkpoint, map_location=device)
    model_before.load_state_dict(ckpt_b["model_state_dict"])
    model_before.eval()

    model_after = StudentDetector(
        pretrained_backbone=False, neck_type="lite", strides=strides, fpn_channels=96
    ).to(device)
    ckpt_a = torch.load(args.after_checkpoint, map_location=device)
    model_after.load_state_dict(ckpt_a["model_state_dict"])
    model_after.eval()

    # Pick samples per cohort
    cohort_counts: dict[str, int] = {}
    selected_samples = []
    for sample in val_dataset:
        src = sample.source_dataset
        cnt = cohort_counts.get(src, 0)
        if cnt < args.samples_per_cohort:
            cohort_counts[src] = cnt + 1
            selected_samples.append(sample)

    print(
        f"Generating proposals for {len(selected_samples)} validation samples across cohorts..."
    )
    images_payload = []

    with torch.no_grad():
        for sample in selected_samples:
            img_tensor = sample.image.unsqueeze(0).to(device)
            orig_h, orig_w = sample.original_size
            scale, off_x, off_y = sample.transform

            # Build level_masks from valid_mask to mask out letterbox padding
            shapes_b = tuple(
                (t.shape[-2], t.shape[-1]) for t in model_before(img_tensor).objectness
            )
            _, valid_levels_b = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), shapes_b, decoder.strides
            )
            level_masks_b = tuple(vl.unsqueeze(0) for vl in valid_levels_b)

            out_b = model_before(img_tensor)
            det_b = decoder(out_b, (384, 384), level_masks_b)[0]

            out_a = model_after(img_tensor)
            det_a = decoder(out_a, (384, 384), level_masks_b)[0]

            # Invert transform properly using scale and offsets:
            # x_orig = (x_trans - off_x) / scale
            # y_orig = (y_trans - off_y) / scale
            def un_transform(det_boxes: torch.Tensor) -> list[list[float]]:
                if det_boxes.numel() == 0:
                    return []
                b = det_boxes.clone().cpu()
                b[:, 0] = ((b[:, 0] - off_x) / scale).clamp(0, orig_w)
                b[:, 1] = ((b[:, 1] - off_y) / scale).clamp(0, orig_h)
                b[:, 2] = ((b[:, 2] - off_x) / scale).clamp(0, orig_w)
                b[:, 3] = ((b[:, 3] - off_y) / scale).clamp(0, orig_h)
                return [[round(float(v), 1) for v in box] for box in b[:35].tolist()]

            boxes_b = un_transform(det_b.boxes)
            boxes_a = un_transform(det_a.boxes)

            # Get raw human GT and SAM novel boxes in true image coordinates
            raw_anns = ann_by_img_id.get(sample.image_id, {"gt": [], "sam": []})
            raw_gt = [[round(float(v), 1) for v in b] for b in raw_anns["gt"]]
            raw_sam = [[round(float(v), 1) for v in b] for b in raw_anns["sam"]]

            # Encode thumbnail
            rec_file = None
            for r in val_dataset.images:
                if r["image_id"] == sample.image_id:
                    rec_file = r["file_name"]
                    break
            img_path = args.image_root / rec_file if rec_file else None
            thumb_b64, _, _ = (
                encode_thumbnail_base64(img_path)
                if img_path and img_path.exists()
                else ("", orig_w, orig_h)
            )

            images_payload.append(
                {
                    "image_id": sample.image_id,
                    "source_dataset": sample.source_dataset,
                    "domain": sample.domain,
                    "width": orig_w,
                    "height": orig_h,
                    "thumbnail_base64": thumb_b64,
                    "ground_truth": raw_gt,
                    "sam_novel": raw_sam,
                    "proposals_before": boxes_b,
                    "scores_before": [
                        round(float(s), 3) for s in det_b.scores[:35].tolist()
                    ],
                    "proposals_after": boxes_a,
                    "scores_after": [
                        round(float(s), 3) for s in det_a.scores[:35].tolist()
                    ],
                }
            )

    generate_comparison_dashboard(images_payload, summary_data, args.output_html)


if __name__ == "__main__":
    main()
