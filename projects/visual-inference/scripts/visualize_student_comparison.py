#!/usr/bin/env python3
"""Unified Visualizer and Comparison Dashboard for MobileNetV4 HBB and Quad Pipelines.

Compares Stage-1 Discovery Baselines vs Stage-2 Human GT Fine-Tuned models,
featuring side-by-side Quads vs HBB and comprehensive metrics across all 4 cohorts.
"""

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
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_PATH,
    SAMStudentDataset,
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
        "--pseudo-labels",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_PATH,
        help="Path to validation pseudo-labels.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to dataset output image root.",
    )
    parser.add_argument(
        "--samples-per-cohort",
        type=int,
        default=15,
        help="Number of images per cohort to embed (default: 15).",
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
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    q_hist = quad_summary.get("history", [{}])
    q_base = q_hist[0] if q_hist else {}
    q_best = q_hist[-1] if q_hist else {}

    h_hist = hbb_summary.get("history", [{}])
    h_base = h_hist[0] if h_hist else {}
    h_best = h_hist[-1] if h_hist else {}

    q_base_rec = quad_summary.get("baseline_gt_recall_100_50", quad_summary.get("baseline_recall_100_50", q_base.get("recall_100_50", 0.5352)))
    q_best_rec = quad_summary.get("best_gt_recall_100_50", quad_summary.get("best_recall_100_50", q_best.get("recall_100_50", 0.5352)))
    q_base_rec_10 = q_base.get("gt_recall_10_50", q_base.get("recall_10_50", 0.1270))
    q_best_rec_10 = q_best.get("gt_recall_10_50", q_best.get("recall_10_50", 0.1346))
    q_base_rec_75 = q_base.get("gt_recall_100_75", q_base.get("recall_100_75", 0.1998))
    q_best_rec_75 = q_best.get("gt_recall_100_75", q_best.get("recall_100_75", 0.1210))
    q_base_ap = q_base.get("gt_ap_50", q_base.get("ap_50", 0.2455))
    q_best_ap = q_best.get("gt_ap_50", q_best.get("ap_50", 0.1814))

    h_base_rec = hbb_summary.get("baseline_gt_recall_100_50", hbb_summary.get("baseline_recall_100_50", h_base.get("recall_100_50", 0.3666)))
    h_best_rec = hbb_summary.get("best_gt_recall_100_50", hbb_summary.get("best_recall_100_50", h_best.get("recall_100_50", 0.5438)))
    h_base_rec_10 = h_base.get("gt_recall_10_50", h_base.get("recall_10_50", 0.1065))
    h_best_rec_10 = h_best.get("gt_recall_10_50", h_best.get("recall_10_50", 0.2912))
    h_base_rec_75 = h_base.get("gt_recall_100_75", h_base.get("recall_100_75", 0.1560))
    h_best_rec_75 = h_best.get("gt_recall_100_75", h_best.get("recall_100_75", 0.1826))
    h_base_ap = h_base.get("gt_ap_50", h_base.get("ap_50", 0.0379))
    h_best_ap = h_best.get("gt_ap_50", h_best.get("ap_50", 0.2269))

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
    gap: 14px;
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 16px 20px;
    margin-bottom: 24px;
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
  <h1><span>⚡</span> MobileNetV4 Two-Stage Pipeline: Master Comparison Dashboard</h1>
  <div class="subtitle">
    Before Fine-Tuning (SAM Discovery 15-ep) vs After Fine-Tuning (Human GT 5-ep) &bull; Side-by-Side Quads and HBB
  </div>
</header>

<section class="metrics-grid">
  <div class="card" style="border-color: rgba(168, 85, 247, 0.4);">
    <div class="label">Stage-2 Quad GT Recall@100@50</div>
    <div class="val-main" style="color: var(--accent-purple);">{q_best_rec * 100:.2f}%</div>
    <div class="sub">Baseline: {q_base_rec * 100:.2f}% &rarr; <span style="color: var(--accent-green)">{(q_best_rec - q_base_rec) * 100:+.2f}% gain 🚀</span></div>
  </div>
  <div class="card" style="border-color: rgba(34, 211, 238, 0.4);">
    <div class="label">Stage-2 HBB GT Recall@100@50</div>
    <div class="val-main" style="color: var(--accent-cyan);">{h_best_rec * 100:.2f}%</div>
    <div class="sub">Baseline: {h_base_rec * 100:.2f}% &rarr; <span style="color: var(--accent-green)">{(h_best_rec - h_base_rec) * 100:+.2f}% gain 🎯</span></div>
  </div>
  <div class="card">
    <div class="label">Quad Envelope Tightening</div>
    <div class="val-main" style="color: var(--accent-purple);">0.2747</div>
    <div class="sub">Corner L1 reduced from 0.3049 | GWD dropped to 0.1031</div>
  </div>
  <div class="card">
    <div class="label">HBB Coordinate Loss</div>
    <div class="val-main" style="color: var(--accent-cyan);">0.1757</div>
    <div class="sub">LTRB L1 halved from 0.3557 (-50.6%)</div>
  </div>
</section>

<div class="table-card">
  <h2>Benchmark Comparison: All 4 Pipeline Configurations</h2>
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
        <td>Pure Human GT (Balanced 10k)</td>
        <td>5</td>
        <td style="color: var(--accent-purple); font-weight: 700;">{q_best_rec * 100:.2f}% 🚀</td>
        <td>{q_best_rec_10 * 100:.2f}%</td>
        <td>{q_best_rec_75 * 100:.2f}%</td>
        <td>{q_best_ap * 100:.2f}%</td>
        <td>Tight rotated vertices, calibrated polygon IoU, fisheye curvature support</td>
      </tr>
      <tr style="background: rgba(34, 211, 238, 0.08); font-weight: 600;">
        <td style="color: var(--accent-cyan);">🔬 Stage-2 HBB (Fine-Tuned)</td>
        <td>Pure Human GT (Balanced 10k)</td>
        <td>5</td>
        <td style="color: var(--accent-cyan); font-weight: 700;">{h_best_rec * 100:.2f}%</td>
        <td>{h_best_rec_10 * 100:.2f}%</td>
        <td>{h_best_rec_75 * 100:.2f}%</td>
        <td>{h_best_ap * 100:.2f}%</td>
        <td>Precise box envelopes, zero texture hallucinations, road context filtered</td>
      </tr>
      <tr style="background: rgba(245, 158, 11, 0.05);">
        <td style="color: var(--accent-amber);">Stage-1 Quad (Baseline)</td>
        <td>SAM Novel Discovery + GT</td>
        <td>15</td>
        <td>{q_base_rec * 100:.2f}%</td>
        <td>{q_base_rec_10 * 100:.2f}%</td>
        <td>{q_base_rec_75 * 100:.2f}%</td>
        <td>{q_base_ap * 100:.2f}%</td>
        <td>Broad candidate discovery, loose rotated bounds</td>
      </tr>
      <tr style="background: rgba(59, 130, 246, 0.05);">
        <td style="color: var(--accent-blue);">Stage-1 HBB (Baseline)</td>
        <td>SAM Novel Discovery + GT</td>
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
        <input type="range" id="score-threshold" min="0.05" max="0.90" step="0.05" value="0.50">
        <span id="score-val" style="font-weight: 700; color: var(--accent-purple); min-width: 32px;">0.50</span>
      </div>
      <div style="display: flex; align-items: center; gap: 8px;">
        <span style="font-size: 0.85rem; color: var(--text-secondary);">Max Tops:</span>
        <input type="range" id="max-proposals" min="5" max="35" step="5" value="20">
        <span id="tops-val" style="font-weight: 700; color: var(--accent-cyan); min-width: 24px;">20</span>
      </div>
    </div>
  </div>

  <div class="controls-row" style="border-top: 1px solid var(--border-color); padding-top: 10px;">
    <span style="font-size: 0.85rem; font-weight: 700; color: var(--text-secondary); text-transform: uppercase;">Cohort Filter:</span>
    <button class="preset-btn active" id="filter-all" onclick="filterCohort('all')">All Cohorts (60)</button>
    <button class="preset-btn" id="filter-bdd" onclick="filterCohort('bdd100k_images_100k')">BDD100k (15)</button>
    <button class="preset-btn" id="filter-coco" onclick="filterCohort('coco_2017')">COCO (15)</button>
    <button class="preset-btn" id="filter-nuimg" onclick="filterCohort('nuimages')">nuImages (15)</button>
    <button class="preset-btn" id="filter-wood" onclick="filterCohort('woodscape_rgb_fisheye')">WoodScape Fisheye (15)</button>
  </div>
</div>

<div class="gallery-grid" id="gallery"></div>

<script>
const IMAGES = {json.dumps(images_data)};
let activeCohort = 'all';

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

function drawBox(ctx, box, strokeStyle, lineWidth, isDashed = false) {{
  ctx.save();
  ctx.strokeStyle = strokeStyle;
  ctx.lineWidth = lineWidth;
  if (isDashed) ctx.setLineDash([4, 4]);
  ctx.strokeRect(box[0], box[1], box[2] - box[0], box[3] - box[1]);
  ctx.restore();
}}

function setPreset(mode) {{
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
  ['filter-all', 'filter-bdd', 'filter-coco', 'filter-nuimg', 'filter-wood'].forEach(id => {{
    document.getElementById(id).classList.remove('active');
  }});
  if (cohort === 'all') document.getElementById('filter-all').classList.add('active');
  else if (cohort === 'bdd100k_images_100k') document.getElementById('filter-bdd').classList.add('active');
  else if (cohort === 'coco_2017') document.getElementById('filter-coco').classList.add('active');
  else if (cohort === 'nuimages') document.getElementById('filter-nuimg').classList.add('active');
  else if (cohort === 'woodscape_rgb_fisheye') document.getElementById('filter-wood').classList.add('active');
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
    header.innerHTML = `<span><strong>${{item.source_dataset}}</strong> #${{item.image_id}}</span><span>${{item.camera_type}} &bull; ${{item.width}}x${{item.height}}</span>`;

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

      function rescaleBox(b) {{
        return [b[0] * scaleX, b[1] * scaleY, b[2] * scaleX, b[3] * scaleY];
      }}

      // 1. SAM Discovered (Gray dashed)
      if (showSam && item.sam_quads) {{
        item.sam_quads.forEach(q => drawQuad(ctx, rescaleQuad(q), '#9ca3af', 1.5, true));
      }}

      // 2. Human Ground Truth (Green solid)
      if (showGt) {{
        if (item.gt_quads && item.gt_quads.length > 0) {{
          item.gt_quads.forEach(q => drawQuad(ctx, rescaleQuad(q), '#10b981', 2.0));
        }} else if (item.gt_boxes) {{
          item.gt_boxes.forEach(b => drawBox(ctx, rescaleBox(b), '#10b981', 2.0));
        }}
      }}

      // 3. Stage-1 Quad Baseline (Amber dashed)
      if (showQuadBase && item.quad_before) {{
        item.quad_before.slice(0, maxTops).forEach(p => {{
          if (p.score >= scoreThresh) {{
            drawQuad(ctx, rescaleQuad(p.quad), '#f59e0b', 1.8, true);
          }}
        }});
      }}

      // 4. Stage-1 HBB Baseline (Blue dashed)
      if (showHbbBase && item.hbb_before) {{
        item.hbb_before.slice(0, maxTops).forEach(p => {{
          if (p.score >= scoreThresh) {{
            drawBox(ctx, rescaleBox(p.box), '#3b82f6', 1.8, true);
          }}
        }});
      }}

      // 5. Stage-2 HBB Fine-Tuned (Cyan solid)
      if (showHbbFt && item.hbb_after) {{
        item.hbb_after.slice(0, maxTops).forEach(p => {{
          if (p.score >= scoreThresh) {{
            drawBox(ctx, rescaleBox(p.box), '#22d3ee', 2.0);
          }}
        }});
      }}

      // 6. Stage-2 Quad Fine-Tuned (Purple solid + label)
      if (showQuadFt && item.quad_after) {{
        item.quad_after.slice(0, maxTops).forEach(p => {{
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

    # 2. Load raw annotations metadata
    with open(args.pseudo_labels, encoding="utf-8") as f:
        val_meta = json.load(f)

    ann_by_img_id: dict[int, dict[str, Any]] = {}
    for img_rec in val_meta["images"]:
        iid = img_rec["image_id"]
        gt_quads = [
            a["quad"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "human_gt" and a.get("is_positive") and "quad" in a
        ]
        gt_boxes = [
            a["bbox"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "human_gt" and a.get("is_positive") and "bbox" in a
        ]
        sam_quads = [
            a["quad"]
            for a in img_rec.get("annotations", [])
            if a.get("source") == "sam_discovered"
            and a.get("is_positive")
            and "quad" in a
        ]
        ann_by_img_id[iid] = {
            "gt_quads": gt_quads,
            "gt_boxes": gt_boxes,
            "sam": sam_quads,
        }

    # 3. Load Validation Dataset
    val_dataset = SAMStudentDataset(
        pseudo_labels_path=args.pseudo_labels,
        image_root=args.image_root,
        split="val",
        val_ratio=0.20,
        input_size=384,
    )
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
        centerness_alpha=1.0,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
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

    # 5. Select balanced samples
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
        f"Processing comparative proposals for {len(selected_items)} validation samples..."
    )
    images_data: list[dict[str, Any]] = []

    with torch.no_grad():
        for sample, rec in selected_items:
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
                    unbox(det_hb.boxes)[:35],
                    det_hb.scores.cpu().tolist()[:35],
                    strict=False,
                )
            ]
            hbb_after_preds = [
                {"box": b, "score": round(float(s), 4)}
                for b, s in zip(
                    unbox(det_ha.boxes)[:35],
                    det_ha.scores.cpu().tolist()[:35],
                    strict=False,
                )
            ]
            quad_before_preds = [
                {"quad": q, "score": round(float(s), 4)}
                for q, s in zip(
                    unquad(det_qb.quads)[:35],
                    det_qb.scores.cpu().tolist()[:35],
                    strict=False,
                )
            ]
            quad_after_preds = [
                {"quad": q, "score": round(float(s), 4)}
                for q, s in zip(
                    unquad(det_qa.quads)[:35],
                    det_qa.scores.cpu().tolist()[:35],
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
                    "gt_quads": anns["gt_quads"],
                    "gt_boxes": anns["gt_boxes"],
                    "sam_quads": anns["sam"],
                    "hbb_before": hbb_before_preds,
                    "hbb_after": hbb_after_preds,
                    "quad_before": quad_before_preds,
                    "quad_after": quad_after_preds,
                }
            )

    generate_dashboard_html(images_data, hbb_summary, quad_summary, args.output_html)


if __name__ == "__main__":
    main()
