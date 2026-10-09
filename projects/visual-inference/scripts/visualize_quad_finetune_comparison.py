#!/usr/bin/env python3
"""Interactive Before & After Dashboard for Stage-2 Quad Human GT Fine-Tuning Comparison."""

from __future__ import annotations

import argparse
import base64
import io
import json
from pathlib import Path
from typing import Any

from PIL import Image
import torch

from student_detector.model import QuadProposalDetector
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentDataset,
)
from student_detector.targets import point_validity_from_pixel_mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build Before & After fine-tuning comparison dashboard for Quads."
    )
    parser.add_argument(
        "--before-checkpoint",
        type=Path,
        default=Path(
            "artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt"
        ),
        help="Path to pre-finetuning checkpoint (Baseline).",
    )
    parser.add_argument(
        "--after-checkpoint",
        type=Path,
        default=Path(
            "artifacts/finetune_10k_quad/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
        ),
        help="Path to post-finetuning checkpoint (Fine-Tuned).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=Path("artifacts/finetune_10k_quad/training_quad_finetune_summary.json"),
        help="Path to fine-tuning summary JSON.",
    )
    parser.add_argument(
        "--output-html",
        type=Path,
        default=Path(
            "artifacts/finetune_10k_quad/dashboard_quad_finetune_comparison.html"
        ),
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


def generate_html(
    images_data: list[dict[str, Any]],
    summary_data: dict[str, Any],
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    baseline_rec = summary_data.get("baseline_gt_recall_100_50", 0.0)
    best_rec = summary_data.get("best_gt_recall_100_50", 0.0)
    history = summary_data.get("history", [])
    epochs_count = summary_data.get("epochs", len(history) - 1)
    duration_min = round(summary_data.get("total_duration_seconds", 0) / 60, 1)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>MobileNetV4 Quad: Stage-2 Human GT Fine-Tuning Comparison</title>
<style>
  :root {{
    --bg-main: #0b0f19;
    --bg-card: #131c2e;
    --border-color: #23324d;
    --text-primary: #e2e8f0;
    --text-secondary: #94a3b8;
    --accent-blue: #38bdf8;
    --accent-green: #34d399;
    --accent-amber: #fbbf24;
    --accent-purple: #c084fc;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    background: var(--bg-main);
    color: var(--text-primary);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    padding: 24px;
    line-height: 1.5;
  }}
  header {{
    margin-bottom: 24px;
    border-bottom: 1px solid var(--border-color);
    padding-bottom: 16px;
  }}
  h1 {{ font-size: 1.8rem; font-weight: 700; color: var(--accent-purple); display: flex; align-items: center; gap: 12px; }}
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
    background: #020617;
  }}
  canvas {{
    width: 100%;
    height: auto;
    display: block;
  }}
</style>
</head>
<body>
<header>
  <h1><span>💎</span> MobileNetV4 Quad: Stage-2 Human GT Fine-Tuning Comparison</h1>
  <div class="subtitle">
    Before: 15 Epochs SAM Pseudo-Labels &bull; After: 5 Epochs Authoritative Human Ground Truth (Min-over-8 Corner L1 + GWD)
  </div>
</header>

<section class="metrics-grid">
  <div class="card">
    <div class="label">Human GT Recall@100@50</div>
    <div class="val-main">{best_rec * 100:.2f}%</div>
    <div class="sub">Baseline: {baseline_rec * 100:.2f}% &rarr; +{(best_rec - baseline_rec) * 100:+.2f}%</div>
  </div>
  <div class="card">
    <div class="label">Training Duration</div>
    <div class="val-main">{duration_min} min</div>
    <div class="sub">{epochs_count} fine-tuning epochs on RTX 3060</div>
  </div>
  <div class="card">
    <div class="label">Cost Function</div>
    <div class="val-main" style="color:var(--accent-purple)">GWD + Min-8 L1</div>
    <div class="sub">Rotation Invariance &amp; Direct Vertex Tightening</div>
  </div>
  <div class="card">
    <div class="label">Confidence Calibration</div>
    <div class="val-main" style="color:var(--accent-blue)">Exact Polygon IoU</div>
    <div class="sub">Continuous QFL (&alpha;=0.0 decoupled centerness)</div>
  </div>
</section>

<div class="controls">
  <label><input type="checkbox" id="show-gt" checked> <span class="badge badge-gt">Human Ground Truth</span></label>
  <label><input type="checkbox" id="show-before" checked> <span class="badge badge-before">Pre-Finetune (SAM Student)</span></label>
  <label><input type="checkbox" id="show-after" checked> <span class="badge badge-after">Post-Finetune (Human GT)</span></label>
  <label><input type="checkbox" id="show-sam"> <span class="badge badge-sam">SAM Discovered</span></label>
  <div style="margin-left: auto; display: flex; align-items: center; gap: 10px;">
    <span style="font-size: 0.85rem; color: var(--text-secondary);">Min Confidence:</span>
    <input type="range" id="score-threshold" min="0.05" max="0.90" step="0.05" value="0.50">
    <span id="score-val" style="font-weight: 700; color: var(--accent-purple); min-width: 32px;">0.50</span>
  </div>
</div>

<div class="gallery-grid" id="gallery"></div>

<script>
const IMAGES = {json.dumps(images_data)};

function drawQuad(ctx, quad, strokeStyle, lineWidth, isDashed = false) {{
  ctx.save();
  ctx.strokeStyle = strokeStyle;
  ctx.lineWidth = lineWidth;
  if (isDashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(quad[0][0], quad[0][1]);
  for (let i = 1; i < 4; i++) {{
    ctx.lineTo(quad[i][0], quad[i][1]);
  }}
  ctx.closePath();
  ctx.stroke();
  ctx.restore();
}}

function renderCards() {{
  const gallery = document.getElementById('gallery');
  gallery.innerHTML = '';
  const showGt = document.getElementById('show-gt').checked;
  const showBefore = document.getElementById('show-before').checked;
  const showAfter = document.getElementById('show-after').checked;
  const showSam = document.getElementById('show-sam').checked;
  const scoreThresh = parseFloat(document.getElementById('score-threshold').value);

  IMAGES.forEach(item => {{
    const card = document.createElement('div');
    card.className = 'img-card';

    const header = document.createElement('div');
    header.className = 'img-card-header';
    header.innerHTML = `<span><strong>${{item.source_dataset}}</strong> #${{item.image_id}}</span><span>${{item.camera_type}}</span>`;

    const wrapper = document.createElement('div');
    wrapper.className = 'canvas-wrapper';

    const canvas = document.createElement('canvas');
    wrapper.appendChild(canvas);
    card.appendChild(header);
    card.appendChild(wrapper);
    gallery.appendChild(card);

    const img = new Image();
    img.src = item.thumbnail;
    img.onload = () => {{
      canvas.width = img.width;
      canvas.height = img.height;
      const ctx = canvas.getContext('2d');
      ctx.drawImage(img, 0, 0);

      const scaleX = img.width / item.width;
      const scaleY = img.height / item.height;

      function rescaleQuad(q) {{
        return q.map(pt => [pt[0] * scaleX, pt[1] * scaleY]);
      }}

      // 1. SAM Discovered (Gray dashed)
      if (showSam && item.sam_quads) {{
        item.sam_quads.forEach(q => drawQuad(ctx, rescaleQuad(q), '#9ca3af', 1.5, true));
      }}

      // 2. Human Ground Truth (Green solid)
      if (showGt && item.gt_quads) {{
        item.gt_quads.forEach(q => drawQuad(ctx, rescaleQuad(q), '#10b981', 2.0));
      }}

      // 3. Before Fine-Tuning (Blue solid)
      if (showBefore && item.preds_before) {{
        item.preds_before.forEach(p => {{
          if (p.score >= scoreThresh) {{
            drawQuad(ctx, rescaleQuad(p.quad), '#3b82f6', 2.0);
          }}
        }});
      }}

      // 4. After Fine-Tuning (Purple solid + label)
      if (showAfter && item.preds_after) {{
        item.preds_after.forEach(p => {{
          if (p.score >= scoreThresh) {{
            const rq = rescaleQuad(p.quad);
            drawQuad(ctx, rq, '#c084fc', 2.5);
            ctx.fillStyle = '#c084fc';
            ctx.font = '11px sans-serif';
            ctx.fillText(p.score.toFixed(2), rq[0][0] + 2, Math.max(12, rq[0][1] - 2));
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
    print(f"Quad Comparison Dashboard generated -> {output_path}")


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    with open(args.summary_json, encoding="utf-8") as f:
        summary_data = json.load(f)

    with open(args.pseudo_labels, encoding="utf-8") as f:
        val_meta = json.load(f)

    # Map image_id -> raw annotations
    ann_by_img_id: dict[int, dict[str, list[list[list[float]]]]] = {}
    for img_rec in val_meta["images"]:
        iid = img_rec["image_id"]
        gt_quads = [
            a["quad"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "human_gt" and a.get("is_positive") and "quad" in a
        ]
        sam_quads = [
            a["quad"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "sam_discovered"
            and a.get("is_positive")
            and "quad" in a
        ]
        ann_by_img_id[iid] = {"gt": gt_quads, "sam": sam_quads}

    val_dataset = SAMStudentDataset(
        pseudo_labels_path=args.pseudo_labels,
        image_root=args.image_root,
        split="val",
        val_ratio=0.20,
        input_size=384,
    )

    ckpt_b = torch.load(args.before_checkpoint, map_location=device)
    strides = ckpt_b.get("strides", (4, 8, 16, 32))
    has_aux_b = any("aux_head" in k for k in ckpt_b["model_state_dict"].keys())

    model_before = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
        dual_head=has_aux_b,
    ).to(device)
    model_before.load_state_dict(ckpt_b["model_state_dict"])
    model_before.eval()

    ckpt_a = torch.load(args.after_checkpoint, map_location=device)
    has_aux_a = any("aux_head" in k for k in ckpt_a["model_state_dict"].keys())
    model_after = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
        dual_head=has_aux_a,
    ).to(device)
    model_after.load_state_dict(ckpt_a["model_state_dict"])
    model_after.eval()

    decoder = QuadInferenceDecoder(
        strides=strides,
        pre_nms_top_k=1000,
        nms_iou_threshold=0.45,
        max_proposals=100,
        centerness_alpha=0.0,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
    )

    cohort_counts: dict[str, int] = {}
    selected_items: list[tuple[Any, dict[str, Any]]] = []
    for idx, sample in enumerate(val_dataset):
        src = sample.source_dataset
        cnt = cohort_counts.get(src, 0)
        if cnt < args.samples_per_cohort:
            cohort_counts[src] = cnt + 1
            rec = val_dataset.images[idx]
            selected_items.append((sample, rec))

    print(
        f"Generating quad proposals for {len(selected_items)} validation samples across cohorts..."
    )
    images_data: list[dict[str, Any]] = []

    with torch.no_grad():
        for sample, rec in selected_items:
            img_tensor = sample.image.unsqueeze(0).to(device)
            orig_h, orig_w = sample.original_size

            # Model Before inference
            out_b = model_before(img_tensor)
            if has_aux_b and isinstance(out_b, tuple):
                out_b = out_b[0]
            shapes_b = tuple((t.shape[-2], t.shape[-1]) for t in out_b.quality)
            _, valid_levels_b = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), shapes_b, decoder.strides
            )
            level_masks_b = tuple(vl.unsqueeze(0) for vl in valid_levels_b)
            dets_b = decoder(out_b, (384, 384), level_masks_b)[0]

            # Model After inference
            out_a = model_after(img_tensor)
            if has_aux_a and isinstance(out_a, tuple):
                out_a = out_a[0]
            shapes_a = tuple((t.shape[-2], t.shape[-1]) for t in out_a.quality)
            _, valid_levels_a = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), shapes_a, decoder.strides
            )
            level_masks_a = tuple(vl.unsqueeze(0) for vl in valid_levels_a)
            dets_a = decoder(out_a, (384, 384), level_masks_a)[0]

            scale, offset_x, offset_y = sample.transform

            def to_orig(quads: torch.Tensor) -> list[list[list[float]]]:
                q = quads.clone()
                q[..., 0] = (q[..., 0] - offset_x) / scale
                q[..., 1] = (q[..., 1] - offset_y) / scale
                q[..., 0] = q[..., 0].clamp(0, orig_w)
                q[..., 1] = q[..., 1].clamp(0, orig_h)
                return q.cpu().tolist()

            preds_b = []
            if dets_b.quads.numel() > 0:
                q_b = to_orig(dets_b.quads)
                s_b = dets_b.scores.cpu().tolist()
                for q, sc in zip(q_b[:40], s_b[:40], strict=False):
                    preds_b.append(
                        {
                            "quad": [
                                [round(float(p[0]), 1), round(float(p[1]), 1)]
                                for p in q
                            ],
                            "score": round(float(sc), 4),
                        }
                    )

            preds_a = []
            if dets_a.quads.numel() > 0:
                q_a = to_orig(dets_a.quads)
                s_a = dets_a.scores.cpu().tolist()
                for q, sc in zip(q_a[:40], s_a[:40], strict=False):
                    preds_a.append(
                        {
                            "quad": [
                                [round(float(p[0]), 1), round(float(p[1]), 1)]
                                for p in q
                            ],
                            "score": round(float(sc), 4),
                        }
                    )

            img_path = args.image_root / rec["file_name"]
            if not img_path.exists():
                continue
            thumb_b64, _, _ = encode_thumbnail_base64(img_path)

            anns = ann_by_img_id.get(sample.image_id, {"gt": [], "sam": []})

            images_data.append(
                {
                    "image_id": sample.image_id,
                    "source_dataset": sample.source_dataset,
                    "camera_type": sample.camera_type,
                    "width": orig_w,
                    "height": orig_h,
                    "thumbnail": thumb_b64,
                    "gt_quads": anns["gt"],
                    "sam_quads": anns["sam"],
                    "preds_before": preds_b,
                    "preds_after": preds_a,
                }
            )

    generate_html(images_data, summary_data, args.output_html)


if __name__ == "__main__":
    main()
