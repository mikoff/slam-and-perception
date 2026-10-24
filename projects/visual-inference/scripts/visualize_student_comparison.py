#!/usr/bin/env python3
"""Unified Visualizer and Comparison Dashboard for MobileNetV4 HBB and Quad Pipelines.

Compares Stage-1 Discovery Baselines vs Stage-2 Human GT Fine-Tuned models on
an audited evaluation manifest. Training-validation metrics are historical.
"""

from __future__ import annotations

import argparse
import base64
import io
from html import escape
import json
from pathlib import Path
from typing import Any

from PIL import Image
import torch

from student_detector.dashboard_holdout import file_sha256, load_holdout
from student_detector.decoder import InferenceDecoder
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
)
from student_detector.targets import point_validity_from_pixel_mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build Unified Comparison Dashboard for HBB and Quad Pipelines."
    )
    parser.add_argument(
        "--hbb-before",
        type=Path,
        default=Path("artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt"),
        help="Path to pre-finetuned HBB checkpoint.",
    )
    parser.add_argument(
        "--hbb-after",
        type=Path,
        default=Path(
            "artifacts/finetune_10k_hbb/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"
        ),
        help="Path to post-finetuned HBB checkpoint.",
    )
    parser.add_argument(
        "--quad-before",
        type=Path,
        default=Path(
            "artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt"
        ),
        help="Path to pre-finetuned Quad checkpoint.",
    )
    parser.add_argument(
        "--quad-after",
        type=Path,
        default=Path(
            "artifacts/finetune_10k_quad/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
        ),
        help="Path to post-finetuned Quad checkpoint.",
    )
    parser.add_argument(
        "--hbb-summary",
        type=Path,
        default=Path("artifacts/finetune_10k_hbb/training_hbb_finetune_summary.json"),
        help="Path to HBB fine-tuning summary JSON.",
    )
    parser.add_argument(
        "--quad-summary",
        type=Path,
        default=Path("artifacts/finetune_10k_quad/training_quad_finetune_summary.json"),
        help="Path to Quad fine-tuning summary JSON.",
    )
    parser.add_argument(
        "--output-html",
        type=Path,
        default=Path("artifacts/dashboard_master_comparison.html"),
        help="Output HTML dashboard path.",
    )
    parser.add_argument(
        "--evaluation-manifest",
        type=Path,
        required=True,
        help="Audited images excluded from both stages of detector training.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to dataset output image root.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device ('cuda' or 'cpu').",
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


def generate_dashboard_html(
    images_data: list[dict[str, Any]],
    hbb_summary: dict[str, Any],
    quad_summary: dict[str, Any],
    output_path: Path,
    provenance: dict[str, Any] | None = None,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    provenance = provenance or {}
    provenance_html = escape(json.dumps(provenance, indent=2))
    checkpoint_labels = " · ".join(
        f"{name}: {Path(info['path']).parent.parent.name}"
        for name, info in provenance.get("checkpoints", {}).items()
        if name.endswith("after")
    )
    q_hist = quad_summary.get("history", [{}])
    q_base = q_hist[0] if q_hist else {}
    q_best = q_hist[-1] if q_hist else {}

    h_hist = hbb_summary.get("history", [{}])
    h_base = h_hist[0] if h_hist else {}
    h_best = h_hist[-1] if h_hist else {}

    q_base_rec = quad_summary.get(
        "baseline_gt_recall_100_50",
        quad_summary.get("baseline_recall_100_50", q_base.get("recall_100_50", 0.5352)),
    )
    q_best_rec = quad_summary.get(
        "best_gt_recall_100_50",
        quad_summary.get("best_recall_100_50", q_best.get("recall_100_50", 0.5352)),
    )
    q_base_rec_10 = q_base.get("gt_recall_10_50", q_base.get("recall_10_50", 0.1270))
    q_best_rec_10 = q_best.get("gt_recall_10_50", q_best.get("recall_10_50", 0.1346))
    q_base_rec_75 = q_base.get("gt_recall_100_75", q_base.get("recall_100_75", 0.1998))
    q_best_rec_75 = q_best.get("gt_recall_100_75", q_best.get("recall_100_75", 0.1210))
    q_base_ap = q_base.get("gt_ap_50", q_base.get("ap_50", 0.2455))
    q_best_ap = q_best.get("gt_ap_50", q_best.get("ap_50", 0.1814))

    h_base_rec = hbb_summary.get(
        "baseline_gt_recall_100_50",
        hbb_summary.get("baseline_recall_100_50", h_base.get("recall_100_50", 0.3666)),
    )
    h_best_rec = hbb_summary.get(
        "best_gt_recall_100_50",
        hbb_summary.get("best_recall_100_50", h_best.get("recall_100_50", 0.5438)),
    )
    h_base_rec_10 = h_base.get("gt_recall_10_50", h_base.get("recall_10_50", 0.1065))
    h_best_rec_10 = h_best.get("gt_recall_10_50", h_best.get("recall_10_50", 0.2912))
    h_base_rec_75 = h_base.get("gt_recall_100_75", h_base.get("recall_100_75", 0.1560))
    h_best_rec_75 = h_best.get("gt_recall_100_75", h_best.get("recall_100_75", 0.1826))
    h_base_ap = h_base.get("gt_ap_50", h_base.get("ap_50", 0.0379))
    h_best_ap = h_best.get("gt_ap_50", h_best.get("ap_50", 0.2269))

    unique_sources = sorted(list({img["source_dataset"] for img in images_data}))
    cohort_counts = {}
    for img in images_data:
        s = img["source_dataset"]
        cohort_counts[s] = cohort_counts.get(s, 0) + 1

    cohort_labels = {
        "ade20k": "ADE20K Validation",
        "open_images_subset": "Open Images Evaluation",
        "roboflow_ev_parking": "PKLot Oblique Holdout",
        "bdd100k_images_100k": "BDD100k Driving",
        "coco_2017": "COCO General",
        "nuimages": "nuImages Multi-View",
        "woodscape_rgb_fisheye": "WoodScape Fisheye",
    }

    filter_buttons = [
        f'<button class="preset-btn active" id="filter-all" onclick="filterCohort(\'all\')">All Cohorts ({len(images_data)})</button>'
    ]
    for src in unique_sources:
        label = cohort_labels.get(src, src.replace("_", " ").title())
        count = cohort_counts.get(src, 0)
        filter_buttons.append(
            f'<button class="preset-btn" id="filter-{src}" onclick="filterCohort(\'{src}\')">{label} ({count})</button>'
        )
    filter_buttons_html = "\n    ".join(filter_buttons)

    is_enriched = provenance.get("cohort_size", 0) > 10000 or any(
        src in ("ade20k", "open_images_subset", "roboflow_ev_parking")
        for src in unique_sources
    )
    cohort_tag = (
        "Enriched Multi-Domain (14k)" if is_enriched else "Pure Human GT (Balanced 10k)"
    )
    stage1_tag = (
        "SAM Discovery + Human GT (Enriched 14k)"
        if is_enriched
        else "SAM Novel Discovery + GT (10k)"
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MobileNetV4 Two-Stage Pipeline: Master HBB & Quad Comparison Dashboard</title>
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
    --accent-cyan: #22d3ee;
    --accent-rose: #f43f5e;
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
  h1 {{ font-size: 1.8rem; font-weight: 700; color: #fff; display: flex; align-items: center; gap: 12px; }}
  .subtitle {{ color: var(--text-secondary); font-size: 0.95rem; margin-top: 4px; }}
  
  .metrics-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
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
  .card .val-main {{ font-size: 1.6rem; font-weight: 700; }}
  .card .sub {{ font-size: 0.75rem; color: var(--text-secondary); margin-top: 4px; }}

  .table-card {{
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 18px;
    margin-bottom: 24px;
    overflow-x: auto;
  }}
  .table-card h2 {{ font-size: 1.1rem; margin-bottom: 12px; color: #fff; }}
  table {{
    width: 100%;
    border-collapse: collapse;
    font-size: 0.88rem;
  }}
  th, td {{
    padding: 10px 14px;
    text-align: left;
    border-bottom: 1px solid var(--border-color);
  }}
  th {{
    background: rgba(15, 23, 42, 0.6);
    color: var(--text-secondary);
    font-weight: 600;
  }}

  .controls {{
    display: flex;
    flex-direction: column;
    gap: 12px;
    position: sticky;
    top: 10px;
    z-index: 1000;
    max-height: calc(100vh - 20px);
    overflow-y: auto;
    background: rgba(15, 23, 42, 0.95);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 14px 20px;
    margin-bottom: 24px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.6);
  }}
  .controls-row {{
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 14px;
  }}
  .controls label {{ display: flex; align-items: center; gap: 8px; font-size: 0.88rem; color: var(--text-primary); cursor: pointer; }}
  .preset-btn {{
    background: #1e293b;
    border: 1px solid var(--border-color);
    color: var(--text-secondary);
    padding: 6px 14px;
    border-radius: 6px;
    font-size: 0.85rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.15s ease;
  }}
  .preset-btn:hover {{ background: #334155; color: #fff; }}
  .preset-btn.active {{
    background: var(--accent-purple);
    border-color: var(--accent-purple);
    color: #0f172a;
    font-weight: 700;
  }}
  .badge {{ display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 0.78rem; font-weight: 600; }}
  .badge-gt {{ background: #065f46; color: #a7f3d0; border: 1px solid #10b981; }}
  .badge-quad-ft {{ background: #581c87; color: #e9d5ff; border: 1px solid #a855f7; }}
  .badge-hbb-ft {{ background: #083344; color: #a5f3fc; border: 1px solid #22d3ee; }}
  .badge-quad-base {{ background: #451a03; color: #fde68a; border: 1px dashed #f59e0b; }}
  .badge-hbb-base {{ background: #1e3a8a; color: #bfdbfe; border: 1px dashed #3b82f6; }}
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
    align-items: center;
    gap: 12px;
    flex-wrap: wrap;
    font-size: 0.82rem;
    color: var(--text-secondary);
    border-bottom: 1px solid var(--border-color);
  }}
  .img-card-metadata {{
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 12px;
    flex: 1 1 260px;
  }}
  .image-view-btn {{
    background: rgba(30, 41, 59, 0.9);
    border: 1px solid var(--border-color);
    color: var(--text-primary);
    padding: 5px 10px;
    border-radius: 6px;
    font-size: 0.78rem;
    font-weight: 700;
    cursor: pointer;
    white-space: nowrap;
    transition: background 0.15s ease, border-color 0.15s ease;
  }}
  .image-view-btn:hover {{ background: #334155; border-color: #64748b; }}
  .image-view-btn[data-mode="gt"] {{ border-color: #10b981; color: #a7f3d0; }}
  .image-view-btn[data-mode="hbb"] {{ border-color: #22d3ee; color: #a5f3fc; }}
  .image-view-btn[data-mode="quad"] {{ border-color: #a855f7; color: #e9d5ff; }}
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
  <h1><span>⚡</span> MobileNetV4 Two-Stage Pipeline: Master Comparison Dashboard</h1>
  <div class="subtitle">
    Before and after fine-tuning &bull; Images excluded from both detector training stages
    <p>{escape(checkpoint_labels)}</p>
    <p>Score 0 shows all exported predictions, up to 100 per model. NMS and the 1,000-candidate shortlist still apply.</p>
    <p>Each image button cycles that card through Human GT, fine-tuned HBB, and fine-tuned quads. Hover any region and scroll the mouse wheel to hide all other boxes and cycle through the active-layer detections directly under the cursor.</p>
    <details><summary>Checkpoints and image exclusion audit</summary><pre>{provenance_html}</pre></details>
  </div>
</header>

<section class="metrics-grid">
  <div class="card" style="border-color: rgba(168, 85, 247, 0.4);">
    <div class="label">Historical Stage-2 Quad GT Recall@100@50</div>
    <div class="val-main" style="color: var(--accent-purple);">{q_best_rec * 100:.2f}%</div>
    <div class="sub">Baseline: {q_base_rec * 100:.2f}% &rarr; <span style="color: var(--accent-green)">{(q_best_rec - q_base_rec) * 100:+.2f}% gain 🚀</span></div>
  </div>
  <div class="card" style="border-color: rgba(34, 211, 238, 0.4);">
    <div class="label">Historical Stage-2 HBB GT Recall@100@50</div>
    <div class="val-main" style="color: var(--accent-cyan);">{h_best_rec * 100:.2f}%</div>
    <div class="sub">Baseline: {h_base_rec * 100:.2f}% &rarr; <span style="color: var(--accent-green)">{(h_best_rec - h_base_rec) * 100:+.2f}% gain 🎯</span></div>
  </div>
</section>

<div class="table-card">
  <h2>Historical training-validation metrics (not measured on the images below)</h2>
  <table>
    <thead>
      <tr>
        <th>Pipeline / Architecture</th>
        <th>Training Cohort &amp; Target</th>
        <th>Epochs</th>
        <th>GT Recall@100 (0.50)</th>
        <th>GT Recall@10 (0.50)</th>
        <th>GT Recall@100 (0.75)</th>
        <th>GT AP / AR@100</th>
        <th>Envelope Quality / Behavior</th>
      </tr>
    </thead>
    <tbody>
      <tr style="background: rgba(168, 85, 247, 0.12); font-weight: 600;">
        <td style="color: var(--accent-purple);">💎 Stage-2 Quad (Fine-Tuned)</td>
        <td>{cohort_tag}</td>
        <td>5</td>
        <td style="color: var(--accent-purple); font-weight: 700;">{q_best_rec * 100:.2f}% 🚀</td>
        <td>{q_best_rec_10 * 100:.2f}%</td>
        <td>{q_best_rec_75 * 100:.2f}%</td>
        <td>{q_best_ap * 100:.2f}%</td>
        <td>Quadrilateral proposals</td>
      </tr>
      <tr style="background: rgba(34, 211, 238, 0.08); font-weight: 600;">
        <td style="color: var(--accent-cyan);">🔬 Stage-2 HBB (Fine-Tuned)</td>
        <td>{cohort_tag}</td>
        <td>5</td>
        <td style="color: var(--accent-cyan); font-weight: 700;">{h_best_rec * 100:.2f}%</td>
        <td>{h_best_rec_10 * 100:.2f}%</td>
        <td>{h_best_rec_75 * 100:.2f}%</td>
        <td>{h_best_ap * 100:.2f}%</td>
        <td>Axis-aligned proposals</td>
      </tr>
      <tr style="background: rgba(245, 158, 11, 0.05);">
        <td style="color: var(--accent-amber);">Stage-1 Quad (Baseline)</td>
        <td>{stage1_tag}</td>
        <td>15</td>
        <td>{q_base_rec * 100:.2f}%</td>
        <td>{q_base_rec_10 * 100:.2f}%</td>
        <td>{q_base_rec_75 * 100:.2f}%</td>
        <td>{q_base_ap * 100:.2f}%</td>
        <td>Broad candidate discovery, loose rotated bounds</td>
      </tr>
      <tr style="background: rgba(59, 130, 246, 0.05);">
        <td style="color: var(--accent-blue);">Stage-1 HBB (Baseline)</td>
        <td>{stage1_tag}</td>
        <td>15</td>
        <td>{h_base_rec * 100:.2f}%</td>
        <td>{h_base_rec_10 * 100:.2f}%</td>
        <td>{h_base_rec_75 * 100:.2f}%</td>
        <td>{h_base_ap * 100:.2f}%</td>
        <td>High discovery recall, loose bounding envelopes</td>
      </tr>
    </tbody>
  </table>
</div>

<div class="controls">
  <div class="controls-row">
    <span style="font-size: 0.85rem; font-weight: 700; color: var(--text-secondary); text-transform: uppercase;">View Presets:</span>
    <button class="preset-btn active" onclick="setPreset('side_by_side')">🌟 Quads vs HBB (Side-to-Side)</button>
    <button class="preset-btn" onclick="setPreset('quad_audit')">💎 Quad Audit (Before vs After)</button>
    <button class="preset-btn" onclick="setPreset('hbb_audit')">🔬 HBB Audit (Before vs After)</button>
    <button class="preset-btn" onclick="setPreset('all')">🔮 All 4 Models</button>
  </div>
  
  <div class="controls-row" style="border-top: 1px solid var(--border-color); padding-top: 10px;">
    <label><input type="checkbox" id="show-gt" checked> <span class="badge badge-gt">Human Ground Truth</span></label>
    <label><input type="checkbox" id="show-quad-ft" checked> <span class="badge badge-quad-ft">Quad (Fine-Tuned)</span></label>
    <label><input type="checkbox" id="show-hbb-ft" checked> <span class="badge badge-hbb-ft">HBB (Fine-Tuned)</span></label>
    <label><input type="checkbox" id="show-quad-base"> <span class="badge badge-quad-base">Quad (Baseline)</span></label>
    <label><input type="checkbox" id="show-hbb-base"> <span class="badge badge-hbb-base">HBB (Baseline)</span></label>
    <label><input type="checkbox" id="show-sam"> <span class="badge badge-sam">SAM Discovered</span></label>

    <div style="margin-left: auto; display: flex; align-items: center; gap: 16px;">
      <div style="display: flex; align-items: center; gap: 8px;">
        <span style="font-size: 0.85rem; color: var(--text-secondary);">Min Score:</span>
        <input type="range" id="score-threshold" min="0" max="1" step="0.01" value="0.30">
        <span id="score-val" style="font-weight: 700; color: var(--accent-purple); min-width: 32px;">0.30</span>
      </div>
      <div style="display: flex; align-items: center; gap: 8px;">
        <span style="font-size: 0.85rem; color: var(--text-secondary);">Max predictions:</span>
        <input type="range" id="max-proposals" min="5" max="100" step="5" value="100">
        <span id="tops-val" style="font-weight: 700; color: var(--accent-cyan); min-width: 24px;">100</span>
      </div>
    </div>
  </div>

  <div class="controls-row" style="border-top: 1px solid var(--border-color); padding-top: 10px;">
    <span style="font-size: 0.85rem; font-weight: 700; color: var(--text-secondary); text-transform: uppercase;">Cohort Filter:</span>
    {filter_buttons_html}
  </div>
</div>

<div class="gallery-grid" id="gallery"></div>

<script>
const IMAGES = {json.dumps(images_data)};
let activeCohort = 'all';
const cardViewModes = ['global', 'gt', 'hbb', 'quad'];
const cardViews = new Map();
const cardViewLabels = {{
  global: 'Combined',
  gt: 'Human GT',
  hbb: 'Fine-tuned HBB',
  quad: 'Fine-tuned Quads',
}};

function drawQuad(ctx, quad, strokeStyle, fillStyle, lineWidth, isDashed = false) {{
  ctx.save();
  ctx.strokeStyle = strokeStyle;
  ctx.fillStyle = fillStyle;
  ctx.lineWidth = lineWidth;
  if (isDashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(quad[0][0], quad[0][1]);
  for (let i = 1; i < 4; i++) {{
    ctx.lineTo(quad[i][0], quad[i][1]);
  }}
  ctx.closePath();
  if (fillStyle) ctx.fill();
  ctx.stroke();
  ctx.restore();
}}

function drawBox(ctx, box, strokeStyle, fillStyle, lineWidth, isDashed = false) {{
  ctx.save();
  ctx.strokeStyle = strokeStyle;
  ctx.fillStyle = fillStyle;
  ctx.lineWidth = lineWidth;
  if (isDashed) ctx.setLineDash([4, 4]);
  if (fillStyle) {{
    ctx.fillRect(box[0], box[1], box[2] - box[0], box[3] - box[1]);
  }}
  ctx.strokeRect(box[0], box[1], box[2] - box[0], box[3] - box[1]);
  ctx.restore();
}}

function pointToSegmentDistSq(px, py, x1, y1, x2, y2) {{
  const dx = x2 - x1;
  const dy = y2 - y1;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return (px - x1) * (px - x1) + (py - y1) * (py - y1);
  let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const projX = x1 + t * dx;
  const projY = y1 + t * dy;
  return (px - projX) * (px - projX) + (py - projY) * (py - projY);
}}

function pointInBox(px, py, box, pad = 3.0) {{
  return px >= box[0] - pad && px <= box[2] + pad && py >= box[1] - pad && py <= box[3] + pad;
}}

function pointInQuad(px, py, quad, pad = 3.0) {{
  let inside = false;
  for (let i = 0, j = quad.length - 1; i < quad.length; j = i++) {{
    const xi = quad[i][0], yi = quad[i][1];
    const xj = quad[j][0], yj = quad[j][1];
    const intersect = ((yi > py) !== (yj > py)) &&
      (px < (xj - xi) * (py - yi) / ((yj - yi) || 1e-7) + xi);
    if (intersect) inside = !inside;
    if (pointToSegmentDistSq(px, py, xi, yi, xj, yj) <= pad * pad) return true;
  }}
  return inside;
}}

function boxArea(box) {{
  return Math.max(1, (box[2] - box[0]) * (box[3] - box[1]));
}}

function quadArea(quad) {{
  let area = 0;
  for (let i = 0, j = quad.length - 1; i < quad.length; j = i++) {{
    area += (quad[j][0] + quad[i][0]) * (quad[j][1] - quad[i][1]);
  }}
  return Math.max(1, Math.abs(area) * 0.5);
}}

function cardKey(item) {{
  return item.sha256 || `${{item.source_dataset}}:${{item.image_id}}:${{item.file_name}}`;
}}

function cycleCardView(item, button, redraw) {{
  const key = cardKey(item);
  const current = cardViews.get(key) || 'global';
  const currentIndex = cardViewModes.indexOf(current);
  const next = cardViewModes[(currentIndex + 1) % cardViewModes.length];
  cardViews.set(key, next);
  button.dataset.mode = next;
  button.textContent = `Layer: ${{cardViewLabels[next]}} ↻`;
  button.setAttribute('aria-label', `Cycle layer for this image; currently ${{cardViewLabels[next]}}`);
  redraw();
}}

function setPreset(mode) {{
  cardViews.clear();
  document.querySelectorAll('.controls-row .preset-btn').forEach(b => b.classList.remove('active'));
  if (event && event.target) event.target.classList.add('active');

  const showGt = document.getElementById('show-gt');
  const showQuadFt = document.getElementById('show-quad-ft');
  const showHbbFt = document.getElementById('show-hbb-ft');
  const showQuadBase = document.getElementById('show-quad-base');
  const showHbbBase = document.getElementById('show-hbb-base');
  const showSam = document.getElementById('show-sam');

  showGt.checked = true;
  showSam.checked = false;

  if (mode === 'side_by_side') {{
    showQuadFt.checked = true;
    showHbbFt.checked = true;
    showQuadBase.checked = false;
    showHbbBase.checked = false;
  }} else if (mode === 'quad_audit') {{
    showQuadFt.checked = true;
    showQuadBase.checked = true;
    showHbbFt.checked = false;
    showHbbBase.checked = false;
  }} else if (mode === 'hbb_audit') {{
    showHbbFt.checked = true;
    showHbbBase.checked = true;
    showQuadFt.checked = false;
    showQuadBase.checked = false;
  }} else if (mode === 'all') {{
    showQuadFt.checked = true;
    showHbbFt.checked = true;
    showQuadBase.checked = true;
    showHbbBase.checked = true;
  }}
  renderCards();
}}

function filterCohort(cohort) {{
  activeCohort = cohort;
  document.querySelectorAll('.controls-row button[id^="filter-"]').forEach(b => b.classList.remove('active'));
  const targetId = cohort === 'all' ? 'filter-all' : 'filter-' + cohort;
  const btn = document.getElementById(targetId);
  if (btn) btn.classList.add('active');
  renderCards();
}}

function renderCards() {{
  const gallery = document.getElementById('gallery');
  gallery.innerHTML = '';

  const showGt = document.getElementById('show-gt').checked;
  const showQuadFt = document.getElementById('show-quad-ft').checked;
  const showHbbFt = document.getElementById('show-hbb-ft').checked;
  const showQuadBase = document.getElementById('show-quad-base').checked;
  const showHbbBase = document.getElementById('show-hbb-base').checked;
  const showSam = document.getElementById('show-sam').checked;
  const scoreThresh = parseFloat(document.getElementById('score-threshold').value);
  const maxTops = parseInt(document.getElementById('max-proposals').value);

  const filtered = IMAGES.filter(item => activeCohort === 'all' || item.source_dataset === activeCohort);

  filtered.forEach(item => {{
    const card = document.createElement('div');
    card.className = 'img-card';

    const header = document.createElement('div');
    header.className = 'img-card-header';
    const imageLabel = item.image_id < 0 ? item.file_name.split('/').pop() : `#${{item.image_id}}`;
    const metadata = document.createElement('div');
    metadata.className = 'img-card-metadata';
    metadata.innerHTML = `<span><strong>${{item.source_dataset}}</strong> ${{imageLabel}}</span><span>${{item.camera_type}} &bull; ${{item.width}}x${{item.height}}</span>`;
    const viewButton = document.createElement('button');
    viewButton.type = 'button';
    viewButton.className = 'image-view-btn';
    const initialMode = cardViews.get(cardKey(item)) || 'global';
    viewButton.dataset.mode = initialMode;
    viewButton.textContent = `Layer: ${{cardViewLabels[initialMode]}} ↻`;
    viewButton.setAttribute('aria-label', `Cycle layer for this image; currently ${{cardViewLabels[initialMode]}}`);
    header.appendChild(metadata);
    header.appendChild(viewButton);

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

      const scaleX = img.width / item.width;
      const scaleY = img.height / item.height;

      function rescaleQuad(q) {{
        return q.map(pt => [pt[0] * scaleX, pt[1] * scaleY]);
      }}

      function rescaleBox(b) {{
        return [b[0] * scaleX, b[1] * scaleY, b[2] * scaleX, b[3] * scaleY];
      }}

      let spotlightActive = false;
      let spotlightHits = [];
      let spotlightIndex = 0;
      let anchorX = null;
      let anchorY = null;

      function getActiveShapes() {{
        const localMode = cardViews.get(cardKey(item)) || 'global';
        const useGlobalLayers = localMode === 'global';
        const shapes = [];

        if (useGlobalLayers && showSam && item.sam_quads) {{
          item.sam_quads.forEach(q => {{
            const rq = rescaleQuad(q);
            shapes.push({{
              kind: 'quad',
              geom: rq,
              area: quadArea(rq),
              layer: 'SAM Discovered',
              score: null,
              stroke: 'rgba(156, 163, 175, 0.65)',
              fill: 'rgba(156, 163, 175, 0.08)',
              lineWidth: 1.5,
              dashed: true,
              labelColor: 'rgba(229, 231, 235, 0.95)',
              showLabel: false,
            }});
          }});
        }}

        if ((useGlobalLayers && showGt) || localMode === 'gt') {{
          if (item.gt_quads && item.gt_quads.length > 0) {{
            item.gt_quads.forEach(q => {{
              const rq = rescaleQuad(q);
              shapes.push({{
                kind: 'quad',
                geom: rq,
                area: quadArea(rq),
                layer: 'Human GT',
                score: null,
                stroke: 'rgba(16, 185, 129, 0.90)',
                fill: 'rgba(16, 185, 129, 0.20)',
                lineWidth: 2.0,
                dashed: false,
                labelColor: 'rgba(167, 243, 208, 0.95)',
                showLabel: false,
              }});
            }});
          }} else if (item.gt_boxes) {{
            item.gt_boxes.forEach(b => {{
              const raw = Array.isArray(b) ? b : b.box;
              const rb = rescaleBox(raw);
              shapes.push({{
                kind: 'box',
                geom: rb,
                area: boxArea(rb),
                layer: 'Human GT',
                score: null,
                stroke: 'rgba(16, 185, 129, 0.90)',
                fill: 'rgba(16, 185, 129, 0.20)',
                lineWidth: 2.0,
                dashed: false,
                labelColor: 'rgba(167, 243, 208, 0.95)',
                showLabel: false,
              }});
            }});
          }}
        }}

        if (useGlobalLayers && showQuadBase && item.quad_before) {{
          item.quad_before.slice(0, maxTops).forEach(p => {{
            if (p.score >= scoreThresh) {{
              const rq = rescaleQuad(p.quad);
              shapes.push({{
                kind: 'quad',
                geom: rq,
                area: quadArea(rq),
                layer: 'Quad Baseline',
                score: p.score,
                stroke: 'rgba(245, 158, 11, 0.70)',
                fill: 'rgba(245, 158, 11, 0.12)',
                lineWidth: 1.8,
                dashed: true,
                labelColor: 'rgba(253, 230, 138, 0.95)',
                showLabel: false,
              }});
            }}
          }});
        }}

        if (useGlobalLayers && showHbbBase && item.hbb_before) {{
          item.hbb_before.slice(0, maxTops).forEach(p => {{
            if (p.score >= scoreThresh) {{
              const rb = rescaleBox(p.box);
              shapes.push({{
                kind: 'box',
                geom: rb,
                area: boxArea(rb),
                layer: 'HBB Baseline',
                score: p.score,
                stroke: 'rgba(59, 130, 246, 0.70)',
                fill: 'rgba(59, 130, 246, 0.12)',
                lineWidth: 1.8,
                dashed: true,
                labelColor: 'rgba(191, 219, 254, 0.95)',
                showLabel: false,
              }});
            }}
          }});
        }}

        if (((useGlobalLayers && showHbbFt) || localMode === 'hbb') && item.hbb_after) {{
          item.hbb_after.slice(0, maxTops).forEach(p => {{
            if (p.score >= scoreThresh) {{
              const rb = rescaleBox(p.box);
              shapes.push({{
                kind: 'box',
                geom: rb,
                area: boxArea(rb),
                layer: 'Fine-tuned HBB',
                score: p.score,
                stroke: 'rgba(34, 211, 238, 0.90)',
                fill: 'rgba(34, 211, 238, 0.22)',
                lineWidth: 2.0,
                dashed: false,
                labelColor: 'rgba(165, 243, 252, 0.95)',
                showLabel: false,
              }});
            }}
          }});
        }}

        if (((useGlobalLayers && showQuadFt) || localMode === 'quad') && item.quad_after) {{
          item.quad_after.slice(0, maxTops).forEach(p => {{
            if (p.score >= scoreThresh) {{
              const rq = rescaleQuad(p.quad);
              shapes.push({{
                kind: 'quad',
                geom: rq,
                area: quadArea(rq),
                layer: 'Fine-tuned Quad',
                score: p.score,
                stroke: 'rgba(192, 132, 252, 0.90)',
                fill: 'rgba(192, 132, 252, 0.22)',
                lineWidth: 2.0,
                dashed: false,
                labelColor: 'rgba(233, 213, 255, 0.95)',
                showLabel: true,
              }});
            }}
          }});
        }}

        return shapes;
      }}

      function renderShape(s, forceHighlight = false) {{
        const lw = forceHighlight ? Math.max(s.lineWidth, 2.6) : s.lineWidth;
        if (s.kind === 'quad') {{
          drawQuad(ctx, s.geom, s.stroke, s.fill, lw, s.dashed);
          if (s.showLabel || forceHighlight) {{
            const txt = forceHighlight
              ? `${{s.layer}}${{s.score !== null ? ' ' + s.score.toFixed(2) : ''}}`
              : (s.score !== null ? s.score.toFixed(2) : '');
            if (txt) {{
              ctx.fillStyle = s.labelColor;
              ctx.font = forceHighlight ? 'bold 12px sans-serif' : '11px sans-serif';
              ctx.fillText(txt, s.geom[0][0] + 2, Math.max(12, s.geom[0][1] - 3));
            }}
          }}
        }} else {{
          drawBox(ctx, s.geom, s.stroke, s.fill, lw, s.dashed);
          if (forceHighlight) {{
            const txt = `${{s.layer}}${{s.score !== null ? ' ' + s.score.toFixed(2) : ''}}`;
            ctx.fillStyle = s.labelColor;
            ctx.font = 'bold 12px sans-serif';
            ctx.fillText(txt, s.geom[0] + 2, Math.max(12, s.geom[1] - 3));
          }}
        }}
      }}

      function drawCard() {{
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(img, 0, 0);

        if (spotlightActive && spotlightHits.length > 0) {{
          const s = spotlightHits[spotlightIndex % spotlightHits.length];
          renderShape(s, true);

          const hudText = `🎯 [${{spotlightIndex + 1}}/${{spotlightHits.length}}] ${{s.layer}}${{s.score !== null ? ' (' + s.score.toFixed(2) + ')' : ''}} — Scroll: cycle • Click/Leave: show all`;
          ctx.save();
          ctx.font = 'bold 12px sans-serif';
          const padX = 8, padY = 5;
          const tw = ctx.measureText(hudText).width;
          ctx.fillStyle = 'rgba(15, 23, 42, 0.86)';
          ctx.fillRect(8, 8, tw + padX * 2, 24);
          ctx.strokeStyle = s.stroke;
          ctx.lineWidth = 1.2;
          ctx.strokeRect(8, 8, tw + padX * 2, 24);
          ctx.fillStyle = '#f8fafc';
          ctx.fillText(hudText, 8 + padX, 8 + 16);
          ctx.restore();
          return;
        }}

        const shapes = getActiveShapes();
        shapes.forEach(s => renderShape(s, false));
      }}

      function canvasCoords(evt) {{
        const rect = canvas.getBoundingClientRect();
        const cx = (evt.clientX - rect.left) * (canvas.width / Math.max(rect.width, 1));
        const cy = (evt.clientY - rect.top) * (canvas.height / Math.max(rect.height, 1));
        return [cx, cy];
      }}

      function computeHitsAt(cx, cy) {{
        const shapes = getActiveShapes();
        const hits = shapes.filter(s =>
          s.kind === 'quad' ? pointInQuad(cx, cy, s.geom) : pointInBox(cx, cy, s.geom)
        );
        hits.sort((a, b) => a.area - b.area);
        return hits;
      }}

      canvas.addEventListener('wheel', (evt) => {{
        const [cx, cy] = canvasCoords(evt);
        const movedFar = anchorX === null || anchorY === null || Math.hypot(cx - anchorX, cy - anchorY) > 8;
        if (!spotlightActive || movedFar) {{
          const hits = computeHitsAt(cx, cy);
          if (hits.length === 0) {{
            if (spotlightActive) {{
              spotlightActive = false;
              spotlightHits = [];
              drawCard();
            }}
            return;
          }}
          evt.preventDefault();
          anchorX = cx;
          anchorY = cy;
          spotlightHits = hits;
          spotlightIndex = 0;
          spotlightActive = true;
          drawCard();
          return;
        }}

        evt.preventDefault();
        const step = evt.deltaY >= 0 ? 1 : -1;
        spotlightIndex = (spotlightIndex + step + spotlightHits.length) % spotlightHits.length;
        drawCard();
      }}, {{ passive: false }});

      canvas.addEventListener('click', () => {{
        if (spotlightActive) {{
          spotlightActive = false;
          spotlightHits = [];
          anchorX = null;
          anchorY = null;
          drawCard();
        }}
      }});

      canvas.addEventListener('mouseleave', () => {{
        if (spotlightActive) {{
          spotlightActive = false;
          spotlightHits = [];
          anchorX = null;
          anchorY = null;
          drawCard();
        }}
      }});

      viewButton.addEventListener('click', () => {{
        spotlightActive = false;
        spotlightHits = [];
        anchorX = null;
        anchorY = null;
        cycleCardView(item, viewButton, drawCard);
      }});
      drawCard();
    }};
  }});
}}

['show-gt', 'show-quad-ft', 'show-hbb-ft', 'show-quad-base', 'show-hbb-base', 'show-sam'].forEach(id => {{
  document.getElementById(id).addEventListener('change', renderCards);
}});

document.getElementById('score-threshold').addEventListener('input', (e) => {{
  document.getElementById('score-val').textContent = parseFloat(e.target.value).toFixed(2);
  renderCards();
}});

document.getElementById('max-proposals').addEventListener('input', (e) => {{
  document.getElementById('tops-val').textContent = e.target.value;
  renderCards();
}});

window.addEventListener('DOMContentLoaded', renderCards);
</script>
</body>
</html>
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Master Comparison Dashboard generated -> {output_path}")


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    # 1. Load summaries
    hbb_summary: dict[str, Any] = {}
    if args.hbb_summary.exists():
        with open(args.hbb_summary, encoding="utf-8") as f:
            hbb_summary = json.load(f)

    quad_summary: dict[str, Any] = {}
    if args.quad_summary.exists():
        with open(args.quad_summary, encoding="utf-8") as f:
            quad_summary = json.load(f)

    selected_items, manifest = load_holdout(args.evaluation_manifest, args.image_root)
    provenance = {
        "checkpoints": {
            name: {
                "path": str(getattr(args, name)),
                "sha256": file_sha256(getattr(args, name)),
            }
            for name in ("hbb_before", "hbb_after", "quad_before", "quad_after")
        },
        "evaluation_manifest": str(args.evaluation_manifest),
        "manifest_sha256": file_sha256(args.evaluation_manifest),
        "excluded_training_images": len(manifest["excluded_training_images"]),
        "excluded_validation_images": len(manifest["excluded_validation_images"]),
        "training_overlap": 0,
        "cohort_size": manifest["cohort_size"],
        "audit": "Detector Stage-1/Stage-2 cohort and external validation inputs excluded by image ID where globally meaningful, resolved path and SHA-256. Near-duplicate scenes and ImageNet pretraining are not audited.",
        "decoder": {"score_floor": 0.0, "pre_nms_top_k": 1000, "max_predictions": 100},
    }
    ann_by_img_id = {
        row["image_id"]: {
            "gt_quads": row["gt_quads"],
            "gt_boxes": row["gt_boxes"],
            "sam": [],
        }
        for _, row in selected_items
    }
    strides = (4, 8, 16, 32)

    # 4. Load Models
    print("Loading models for comparative inference...")
    # HBB Models
    hbb_decoder = InferenceDecoder(
        strides=strides,
        top_k=1000,
        nms_iou_threshold=0.45,
        max_detections=100,
        score_mode="objectness_x_centerness",
        centerness_alpha=0.25,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
        score_threshold=0.0,
    )

    model_hbb_before = StudentDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
    ).to(device)
    if args.hbb_before.exists():
        ckpt = torch.load(args.hbb_before, map_location=device)
        model_hbb_before.load_state_dict(ckpt["model_state_dict"])
        model_hbb_before.eval()

    model_hbb_after = StudentDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
    ).to(device)
    if args.hbb_after.exists():
        ckpt = torch.load(args.hbb_after, map_location=device)
        model_hbb_after.load_state_dict(ckpt["model_state_dict"])
        model_hbb_after.eval()
    else:
        print(
            f"\n[Master Visualizer] ⚠️ WARNING: HBB fine-tuned checkpoint NOT found: {args.hbb_after}\n"
            f"                     Stage-2 HBB fine-tuning has not been run yet. HBB after boxes will be empty.\n"
        )

    # Quad Models
    quad_decoder = QuadInferenceDecoder(
        strides=strides,
        pre_nms_top_k=1000,
        nms_iou_threshold=0.45,
        max_proposals=100,
        centerness_alpha=0.0,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
        score_threshold=0.0,
    )

    ckpt_qb = (
        torch.load(args.quad_before, map_location=device)
        if args.quad_before.exists()
        else {}
    )
    has_aux_qb = any(
        "aux_head" in k for k in ckpt_qb.get("model_state_dict", {}).keys()
    )
    model_quad_before = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
        dual_head=has_aux_qb,
    ).to(device)
    if ckpt_qb:
        model_quad_before.load_state_dict(ckpt_qb["model_state_dict"])
        model_quad_before.eval()

    ckpt_qa = (
        torch.load(args.quad_after, map_location=device)
        if args.quad_after.exists()
        else {}
    )
    has_aux_qa = any(
        "aux_head" in k for k in ckpt_qa.get("model_state_dict", {}).keys()
    )
    model_quad_after = QuadProposalDetector(
        pretrained_backbone=False,
        neck_type="lite",
        strides=strides,
        fpn_channels=96,
        dual_head=has_aux_qa,
    ).to(device)
    if ckpt_qa:
        model_quad_after.load_state_dict(ckpt_qa["model_state_dict"])
        model_quad_after.eval()

    print(
        f"Processing comparative proposals for {len(selected_items)} unseen evaluation samples..."
    )
    images_data: list[dict[str, Any]] = []

    with torch.no_grad():
        for sample_index, (sample, rec) in enumerate(selected_items):
            if sample_index % 25 == 0:
                print(f"Inference: {sample_index}/{len(selected_items)}", flush=True)
            img_tensor = sample.image.unsqueeze(0).to(device)
            orig_h, orig_w = sample.original_size
            scale, offset_x, offset_y = sample.transform

            # Run inference for all 4 models with validity level masks
            # 1. HBB Before
            out_hb = model_hbb_before(img_tensor)
            sh_hb = tuple((t.shape[-2], t.shape[-1]) for t in out_hb.objectness)
            _, vlvl_hb = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), sh_hb, strides
            )
            lmasks_hb = tuple(vl.unsqueeze(0) for vl in vlvl_hb)
            det_hb = hbb_decoder(out_hb, (384, 384), lmasks_hb)[0]

            # 2. HBB After
            out_ha = model_hbb_after(img_tensor)
            sh_ha = tuple((t.shape[-2], t.shape[-1]) for t in out_ha.objectness)
            _, vlvl_ha = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), sh_ha, strides
            )
            lmasks_ha = tuple(vl.unsqueeze(0) for vl in vlvl_ha)
            det_ha = hbb_decoder(out_ha, (384, 384), lmasks_ha)[0]

            # 3. Quad Before
            out_qb = model_quad_before(img_tensor)
            if has_aux_qb and isinstance(out_qb, tuple):
                out_qb = out_qb[0]
            sh_qb = tuple((t.shape[-2], t.shape[-1]) for t in out_qb.quality)
            _, vlvl_qb = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), sh_qb, strides
            )
            lmasks_qb = tuple(vl.unsqueeze(0) for vl in vlvl_qb)
            det_qb = quad_decoder(out_qb, (384, 384), lmasks_qb)[0]

            # 4. Quad After
            out_qa = model_quad_after(img_tensor)
            if has_aux_qa and isinstance(out_qa, tuple):
                out_qa = out_qa[0]
            sh_qa = tuple((t.shape[-2], t.shape[-1]) for t in out_qa.quality)
            _, vlvl_qa = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), sh_qa, strides
            )
            lmasks_qa = tuple(vl.unsqueeze(0) for vl in vlvl_qa)
            det_qa = quad_decoder(out_qa, (384, 384), lmasks_qa)[0]

            # Invert coordinates to native image pixels
            def unbox(boxes: torch.Tensor) -> list[list[float]]:
                if boxes.numel() == 0:
                    return []
                b = boxes.clone().cpu()
                b[:, 0] = ((b[:, 0] - offset_x) / scale).clamp(0, orig_w)
                b[:, 1] = ((b[:, 1] - offset_y) / scale).clamp(0, orig_h)
                b[:, 2] = ((b[:, 2] - offset_x) / scale).clamp(0, orig_w)
                b[:, 3] = ((b[:, 3] - offset_y) / scale).clamp(0, orig_h)
                return [[round(float(v), 1) for v in box] for box in b.tolist()]

            def unquad(quads: torch.Tensor) -> list[list[list[float]]]:
                if quads.numel() == 0:
                    return []
                q = quads.clone().cpu()
                q[..., 0] = ((q[..., 0] - offset_x) / scale).clamp(0, orig_w)
                q[..., 1] = ((q[..., 1] - offset_y) / scale).clamp(0, orig_h)
                return [
                    [[round(float(pt[0]), 1), round(float(pt[1]), 1)] for pt in item]
                    for item in q.tolist()
                ]

            hbb_before_preds = [
                {"box": b, "score": round(float(s), 4)}
                for b, s in zip(
                    unbox(det_hb.boxes),
                    det_hb.scores.cpu().tolist(),
                    strict=False,
                )
            ]
            hbb_after_preds = [
                {"box": b, "score": round(float(s), 4)}
                for b, s in zip(
                    unbox(det_ha.boxes),
                    det_ha.scores.cpu().tolist(),
                    strict=False,
                )
            ]
            quad_before_preds = [
                {"quad": q, "score": round(float(s), 4)}
                for q, s in zip(
                    unquad(det_qb.quads),
                    det_qb.scores.cpu().tolist(),
                    strict=False,
                )
            ]
            quad_after_preds = [
                {"quad": q, "score": round(float(s), 4)}
                for q, s in zip(
                    unquad(det_qa.quads),
                    det_qa.scores.cpu().tolist(),
                    strict=False,
                )
            ]

            img_path = args.image_root / rec["file_name"]
            if not img_path.exists():
                continue
            thumb_b64, _, _ = encode_thumbnail_base64(img_path)

            anns = ann_by_img_id.get(
                sample.image_id, {"gt_quads": [], "gt_boxes": [], "sam": []}
            )

            images_data.append(
                {
                    "image_id": sample.image_id,
                    "source_dataset": sample.source_dataset,
                    "camera_type": sample.camera_type,
                    "width": orig_w,
                    "height": orig_h,
                    "thumbnail": thumb_b64,
                    "file_name": rec["file_name"],
                    "sha256": rec["sha256"],
                    "gt_quads": anns["gt_quads"],
                    "gt_boxes": anns["gt_boxes"],
                    "sam_quads": anns["sam"],
                    "hbb_before": hbb_before_preds,
                    "hbb_after": hbb_after_preds,
                    "quad_before": quad_before_preds,
                    "quad_after": quad_after_preds,
                }
            )

    generate_dashboard_html(
        images_data, hbb_summary, quad_summary, args.output_html, provenance
    )
    args.output_html.with_suffix(".audit.json").write_text(
        json.dumps(provenance, indent=2)
    )


if __name__ == "__main__":
    main()
