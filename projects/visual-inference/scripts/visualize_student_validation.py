#!/usr/bin/env python3
"""Interactive visualization dashboard for MobileNetV4 Student detector validation (HBB and Quad)."""

from __future__ import annotations

import argparse
import base64
from datetime import datetime
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
    SAMStudentHBBDataset,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build interactive validation dashboard for MobileNetV4 Student detector."
    )
    parser.add_argument(
        "--geometry",
        type=str,
        choices=["hbb", "quad"],
        default="hbb",
        help="Model geometry: 'hbb' or 'quad' (default: 'hbb').",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=None,
        help="Path to trained student checkpoint.",
    )
    parser.add_argument(
        "--pseudo-labels",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_PATH,
        help="Path to sam_pilot_800.json.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to dataset output image root.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Path to training summary JSON.",
    )
    parser.add_argument(
        "--output-html",
        type=Path,
        default=None,
        help="Output HTML path.",
    )
    parser.add_argument(
        "--samples-per-cohort",
        type=int,
        default=15,
        help="Number of images per dataset cohort to embed (default: 15, yielding 60 total across 4 cohorts).",
    )
    parser.add_argument(
        "--nms-mode",
        type=str,
        default="auto",
        choices=["auto", "hard", "part_aware", "part_aware_soft", "nms_free"],
        help="NMS mode ('auto' selects 'part_aware').",
    )
    parser.add_argument(
        "--nms-iou-threshold",
        type=float,
        default=0.45,
        help="NMS IoU suppression threshold (default: 0.45).",
    )
    parser.add_argument(
        "--part-scale-ratio",
        type=float,
        default=0.4,
        help="Part-aware NMS scale ratio threshold (default: 0.4).",
    )
    parser.add_argument(
        "--part-max-iou",
        type=float,
        default=0.65,
        help="Part-aware NMS maximum duplicate IoU threshold (default: 0.65).",
    )
    parser.add_argument(
        "--score-mode",
        type=str,
        default="objectness_x_centerness",
        choices=["objectness", "objectness_x_centerness"],
        help="Scoring mode for HBB decoder (default: objectness_x_centerness).",
    )
    parser.add_argument(
        "--centerness-alpha",
        type=float,
        default=None,
        help="Centerness exponent for decoder (defaults to 1.0 for HBB, 0.0 for Quad).",
    )
    parser.add_argument(
        "--eval-metrics-json",
        type=Path,
        default=None,
        help="Path to evaluation metrics JSON (overrides summary metrics).",
    )
    parser.add_argument(
        "--wrap-to-bbox",
        action="store_true",
        help="For Quad models, wrap predicted oriented quads into horizontal bounding boxes.",
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


def generate_html(
    images_payload: list[dict[str, Any]],
    training_summary: dict[str, Any],
    output_path: Path,
    geometry: str,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload_json = json.dumps(images_payload)
    summary_json_str = json.dumps(training_summary)
    geom_upper = geometry.upper()

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MobileNetV4 P2-{geom_upper} Student Detector - Validation Dashboard</title>
<style>
  :root {{
    --bg-dark: #0f172a;
    --bg-card: #1e293b;
    --text-primary: #f8fafc;
    --text-secondary: #94a3b8;
    --accent-blue: #38bdf8;
    --accent-green: #34d399;
    --accent-amber: #fbbf24;
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
    grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
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
  .card .value {{ font-size: 1.6rem; font-weight: 700; color: var(--accent-blue); }}
  .card .sub {{ font-size: 0.75rem; color: var(--text-secondary); margin-top: 2px; }}

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
  .filter-group {{ display: flex; align-items: center; gap: 8px; }}
  .filter-group label {{ font-size: 0.85rem; font-weight: 600; color: var(--text-secondary); }}
  .btn {{
    background: #334155;
    color: var(--text-primary);
    border: 1px solid var(--border-color);
    padding: 6px 14px;
    border-radius: 6px;
    font-size: 0.85rem;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.15s ease;
  }}
  .btn:hover {{ background: #475569; }}
  .btn.active {{
    background: var(--accent-blue);
    color: #0f172a;
    border-color: var(--accent-blue);
    font-weight: 700;
  }}
  .toggle-group {{ display: flex; align-items: center; gap: 14px; margin-left: auto; }}
  .toggle-item {{ display: flex; align-items: center; gap: 6px; font-size: 0.85rem; cursor: pointer; user-select: none; }}
  .toggle-item input {{ cursor: pointer; }}
  .badge {{
    display: inline-block;
    width: 10px;
    height: 10px;
    border-radius: 2px;
    margin-right: 2px;
  }}
  .slider-group {{ display: flex; align-items: center; gap: 8px; font-size: 0.85rem; color: var(--text-secondary); }}
  .slider-group input[type="range"] {{ width: 110px; }}

  .main-layout {{
    display: grid;
    grid-template-columns: 340px 1fr;
    gap: 24px;
    height: calc(100vh - 280px);
    min-height: 600px;
  }}
  .gallery-pane {{
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 12px;
  }}
  .thumb-item {{
    border: 2px solid transparent;
    border-radius: 6px;
    padding: 8px 10px;
    background: #0f172a;
    cursor: pointer;
    display: flex;
    flex-direction: column;
    gap: 4px;
    transition: all 0.15s ease;
  }}
  .thumb-item:hover {{ border-color: #475569; }}
  .thumb-item.active {{
    border-color: var(--accent-blue);
    background: #1e293b;
    box-shadow: 0 0 10px rgba(56, 189, 248, 0.2);
  }}
  .thumb-title {{ font-size: 0.85rem; font-weight: 700; color: var(--text-primary); }}
  .thumb-meta {{ font-size: 0.75rem; color: var(--text-secondary); display: flex; justify-content: space-between; }}

  .viewer-pane {{
    background: var(--bg-card);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 16px;
    display: flex;
    flex-direction: column;
    gap: 12px;
    overflow: hidden;
  }}
  .canvas-container {{
    position: relative;
    flex: 1;
    display: flex;
    justify-content: center;
    align-items: center;
    background: #020617;
    border-radius: 6px;
    overflow: hidden;
  }}
  canvas {{
    max-width: 100%;
    max-height: 100%;
    object-fit: contain;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5);
  }}
  .details-table {{
    width: 100%;
    border-collapse: collapse;
    font-size: 0.8rem;
    margin-top: 4px;
  }}
  .details-table th, .details-table td {{
    padding: 6px 12px;
    text-align: left;
    border-bottom: 1px solid var(--border-color);
  }}
  .details-table th {{ color: var(--text-secondary); font-weight: 600; text-transform: uppercase; font-size: 0.75rem; }}
</style>
</head>
<body>

<header>
  <h1>MobileNetV4 P2-{geom_upper} Student Detector Validation Audit</h1>
  <div class="subtitle">
    Evaluation across balanced multi-domain cohorts (COCO General, BDD100k, nuImages, WoodScape Fisheye) &bull; {geom_upper} Proposals
  </div>
</header>

<div class="metrics-grid">
  <div class="card">
    <div class="label">Recall@100 (IoU 0.50)</div>
    <div class="value" id="stat-rec100-50">--%</div>
    <div class="sub">Across all validation images</div>
  </div>
  <div class="card">
    <div class="label">Recall@100 (IoU 0.75)</div>
    <div class="value" id="stat-rec100-75">--%</div>
    <div class="sub">High-precision threshold</div>
  </div>
  <div class="card">
    <div class="label">AP/AR@50 Score</div>
    <div class="value" id="stat-ap50">--%</div>
    <div class="sub">Localization quality</div>
  </div>
  <div class="card">
    <div class="label">Small Object Recall</div>
    <div class="value" id="stat-rec-small">--%</div>
    <div class="sub">Size &lt; 1024 px&sup2;</div>
  </div>
  <div class="card">
    <div class="label">Medium Object Recall</div>
    <div class="value" id="stat-rec-med">--%</div>
    <div class="sub">32–96 px (All-Time Record)</div>
  </div>
  <div class="card">
    <div class="label">Large Object Recall</div>
    <div class="value" id="stat-rec-lrg">--%</div>
    <div class="sub">&gt; 96 px (De-clustered)</div>
  </div>
  <div class="card">
    <div class="label">Fisheye Domain Recall</div>
    <div class="value" id="stat-rec-fisheye">--%</div>
    <div class="sub">WoodScape radial coverage</div>
  </div>
</div>

<div class="controls">
  <div class="filter-group">
    <label>Cohort:</label>
    <button class="btn active" onclick="filterDomain('all')">All (<span id="count-all">0</span>)</button>
    <button class="btn" onclick="filterDomain('coco_2017')">COCO (<span id="count-coco">0</span>)</button>
    <button class="btn" onclick="filterDomain('bdd100k_images_100k')">BDD100k (<span id="count-bdd">0</span>)</button>
    <button class="btn" onclick="filterDomain('nuimages')">nuImages (<span id="count-nui">0</span>)</button>
    <button class="btn" onclick="filterDomain('woodscape_rgb_fisheye')">WoodScape Fisheye (<span id="count-wood">0</span>)</button>
  </div>

  <div class="slider-group">
    <label>Conf Threshold:</label>
    <input type="range" id="conf-slider" min="0.05" max="0.90" step="0.05" value="0.25" oninput="updateConfidence(this.value)">
    <span id="conf-val" style="font-weight:700;color:var(--accent-blue)">0.25</span>
  </div>

  <div class="toggle-group">
    <label class="toggle-item">
      <input type="checkbox" id="layer-student" checked onchange="renderCurrent()">
      <span class="badge" style="background:var(--accent-blue)"></span> Student {geom_upper} Preds
    </label>
    <label class="toggle-item">
      <input type="checkbox" id="layer-sam" checked onchange="renderCurrent()">
      <span class="badge" style="background:var(--accent-green)"></span> SAM Novel
    </label>
    <label class="toggle-item">
      <input type="checkbox" id="layer-gt" checked onchange="renderCurrent()">
      <span class="badge" style="background:var(--accent-amber)"></span> Human GT
    </label>
    <label class="toggle-item">
      <input type="checkbox" id="layer-suppressed" onchange="renderCurrent()">
      <span class="badge" style="background:var(--accent-rose)"></span> Suppressed
    </label>
  </div>
</div>

<div class="main-layout">
  <div class="gallery-pane" id="gallery">
    <!-- Populated dynamically -->
  </div>

  <div class="viewer-pane">
    <div class="canvas-container">
      <canvas id="main-canvas"></canvas>
    </div>
    <table class="details-table">
      <thead>
        <tr>
          <th>Image ID</th>
          <th>Dataset</th>
          <th>Camera / Resolution</th>
          <th>Student Predictions</th>
          <th>Human GT</th>
          <th>SAM Novel</th>
          <th>Suppressed</th>
        </tr>
      </thead>
      <tbody>
        <tr id="details-row">
          <td colspan="7" style="text-align:center;color:var(--text-secondary)">Select an image from the list</td>
        </tr>
      </tbody>
    </table>
  </div>
</div>

<script>
const IMAGES = {payload_json};
const SUMMARY = {summary_json_str};
const GEOMETRY = "{geometry}";

let currentDomain = 'all';
let currentConf = 0.25;
let currentIndex = 0;
let filteredImages = [...IMAGES];

function init() {{
  document.getElementById('count-all').innerText = IMAGES.length;
  document.getElementById('count-coco').innerText = IMAGES.filter(i => i.source_dataset === 'coco_2017').length;
  document.getElementById('count-bdd').innerText = IMAGES.filter(i => i.source_dataset === 'bdd100k_images_100k').length;
  document.getElementById('count-nui').innerText = IMAGES.filter(i => i.source_dataset === 'nuimages').length;
  document.getElementById('count-wood').innerText = IMAGES.filter(i => i.source_dataset === 'woodscape_rgb_fisheye').length;

  if (SUMMARY && SUMMARY.eval_metrics) {{
    const em = SUMMARY.eval_metrics;
    document.getElementById('stat-rec100-50').innerText = (((em['recall/100@50'] ?? em['recall/100@0.50']) || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-rec100-75').innerText = (((em['recall/100@75'] ?? em['recall/100@0.75']) || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-ap50').innerText = (((em['ap/50'] ?? em['ar/100']) || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-rec-small').innerText = (((em['recall/size_small/100@50'] ?? em['recall/size_small/100@0.50']) || 0) * 100).toFixed(1) + '%';
    const medElem = document.getElementById('stat-rec-med');
    if (medElem) medElem.innerText = (((em['recall/size_medium/100@50'] ?? em['recall/size_medium/100@0.50']) || 0) * 100).toFixed(1) + '%';
    const lrgElem = document.getElementById('stat-rec-lrg');
    if (lrgElem) lrgElem.innerText = (((em['recall/size_large/100@50'] ?? em['recall/size_large/100@0.50']) || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-rec-fisheye').innerText = (((em['recall/fisheye/100@50'] ?? em['recall/domain_fisheye/100@0.50']) || 0) * 100).toFixed(1) + '%';
  }} else if (SUMMARY && SUMMARY.history && SUMMARY.history.length) {{
    const last = SUMMARY.history[SUMMARY.history.length - 1];
    document.getElementById('stat-rec100-50').innerText = ((last.recall_100_50 || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-rec100-75').innerText = ((last.recall_100_75 || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-ap50').innerText = ((last.ap_50 || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-rec-small').innerText = ((last.recall_size_small || 0) * 100).toFixed(1) + '%';
    const medElem = document.getElementById('stat-rec-med');
    if (medElem) medElem.innerText = ((last.recall_size_medium || 0) * 100).toFixed(1) + '%';
    const lrgElem = document.getElementById('stat-rec-lrg');
    if (lrgElem) lrgElem.innerText = ((last.recall_size_large || 0) * 100).toFixed(1) + '%';
    document.getElementById('stat-rec-fisheye').innerText = ((last.recall_domain_fisheye || 0) * 100).toFixed(1) + '%';
  }}
  buildGallery();
  if (filteredImages.length > 0) {{
    selectImage(0);
  }}
}}

function filterDomain(domain) {{
  currentDomain = domain;
  document.querySelectorAll('.filter-group .btn').forEach(btn => {{
    if ((domain === 'all' && btn.innerText.includes('All')) ||
        (domain === 'coco_2017' && btn.innerText.includes('COCO')) ||
        (domain === 'bdd100k_images_100k' && btn.innerText.includes('BDD100k')) ||
        (domain === 'nuimages' && btn.innerText.includes('nuImages')) ||
        (domain === 'woodscape_rgb_fisheye' && btn.innerText.includes('WoodScape'))) {{
      btn.classList.add('active');
    }} else {{
      btn.classList.remove('active');
    }}
  }});

  if (domain === 'all') {{
    filteredImages = [...IMAGES];
  }} else {{
    filteredImages = IMAGES.filter(img => img.source_dataset === domain);
  }}
  buildGallery();
  if (filteredImages.length > 0) {{
    selectImage(0);
  }}
}}

function updateConfidence(val) {{
  currentConf = parseFloat(val);
  document.getElementById('conf-val').innerText = currentConf.toFixed(2);
  renderCurrent();
}}

function buildGallery() {{
  const gall = document.getElementById('gallery');
  gall.innerHTML = '';
  filteredImages.forEach((img, idx) => {{
    const item = document.createElement('div');
    item.className = 'thumb-item' + (idx === currentIndex ? ' active' : '');
    item.onclick = () => selectImage(idx);
    const validPreds = img.predictions.filter(p => p.score >= currentConf).length;
    item.innerHTML = `
      <div class="thumb-title">${{img.source_dataset}} #${{img.image_id}}</div>
      <div class="thumb-meta">
        <span>${{img.camera_type}}</span>
        <span>Pred: ${{validPreds}} | GT: ${{img.gt_count}} | Novel: ${{img.novel_count}}</span>
      </div>
    `;
    gall.appendChild(item);
  }});
}}

function selectImage(idx) {{
  currentIndex = idx;
  document.querySelectorAll('.thumb-item').forEach((item, i) => {{
    item.classList.toggle('active', i === idx);
  }});
  renderCurrent();
}}

function renderCurrent() {{
  if (!filteredImages[currentIndex]) return;
  const data = filteredImages[currentIndex];

  const canvas = document.getElementById('main-canvas');
  const ctx = canvas.getContext('2d');
  const imgObj = new Image();
  imgObj.onload = () => {{
    canvas.width = imgObj.width;
    canvas.height = imgObj.height;
    ctx.drawImage(imgObj, 0, 0);

    const scaleX = canvas.width / data.width;
    const scaleY = canvas.height / data.height;

    // 1. Suppressed Subparts (Rose / Red)
    if (document.getElementById('layer-suppressed').checked && data.suppressed_items) {{
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = '#f43f5e';
      data.suppressed_items.forEach(item => drawItem(ctx, item, scaleX, scaleY));
    }}

    // 2. Human Ground Truth (Amber / Orange)
    if (document.getElementById('layer-gt').checked && data.gt_items) {{
      ctx.lineWidth = 2.0;
      ctx.strokeStyle = '#fbbf24';
      data.gt_items.forEach(item => drawItem(ctx, item, scaleX, scaleY));
    }}

    // 3. SAM Novel Pseudo-Labels (Emerald Green)
    if (document.getElementById('layer-sam').checked && data.novel_items) {{
      ctx.lineWidth = 2.0;
      ctx.strokeStyle = '#34d399';
      data.novel_items.forEach(item => drawItem(ctx, item, scaleX, scaleY));
    }}

    // 4. Student Predictions (Cyan / Blue)
    if (document.getElementById('layer-student').checked && data.predictions) {{
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = '#38bdf8';
      ctx.fillStyle = '#38bdf8';
      ctx.font = '12px sans-serif';
      const preds = data.predictions.filter(p => p.score >= currentConf);
      preds.forEach(p => {{
        if (GEOMETRY === 'quad') {{
          drawQuad(ctx, p.quad, scaleX, scaleY);
          const p0x = p.quad[0][0] * scaleX;
          const p0y = p.quad[0][1] * scaleY;
          ctx.fillText(p.score.toFixed(2), p0x + 2, Math.max(12, p0y - 2));
        }} else {{
          drawBox(ctx, p.box, scaleX, scaleY);
          const bx = p.box[0] * scaleX;
          const by = p.box[1] * scaleY;
          ctx.fillText(p.score.toFixed(2), bx + 3, Math.max(14, by - 3));
        }}
      }});
    }}

    // Update details table
    const row = document.getElementById('details-row');
    const validPreds = data.predictions.filter(p => p.score >= currentConf).length;
    row.innerHTML = `
      <td>#${{data.image_id}}</td>
      <td><strong>${{data.source_dataset}}</strong></td>
      <td>${{data.camera_type}} (${{data.width}}x${{data.height}})</td>
      <td><span style="color:var(--accent-blue);font-weight:700">${{validPreds}}</span> (conf &ge; ${{currentConf.toFixed(2)}})</td>
      <td><span style="color:var(--accent-amber);font-weight:700">${{data.gt_count}}</span></td>
      <td><span style="color:var(--accent-green);font-weight:700">${{data.novel_count}}</span></td>
      <td><span style="color:var(--accent-rose)">${{data.suppressed_count}}</span></td>
    `;
  }};
  imgObj.src = data.image_base64;
}}

function drawItem(ctx, item, scaleX, scaleY) {{
  if (Array.isArray(item) && item.length === 4 && typeof item[0] === 'number') {{
    drawBox(ctx, item, scaleX, scaleY);
  }} else if (Array.isArray(item) && item.length === 4 && Array.isArray(item[0])) {{
    drawQuad(ctx, item, scaleX, scaleY);
  }}
}}

function drawBox(ctx, box, scaleX, scaleY) {{
  if (!box || box.length !== 4) return;
  const x1 = box[0] * scaleX;
  const y1 = box[1] * scaleY;
  const w = (box[2] - box[0]) * scaleX;
  const h = (box[3] - box[1]) * scaleY;
  ctx.strokeRect(x1, y1, w, h);
}}

function drawQuad(ctx, quad, scaleX, scaleY) {{
  if (!quad || quad.length !== 4) return;
  ctx.beginPath();
  ctx.moveTo(quad[0][0] * scaleX, quad[0][1] * scaleY);
  for (let i = 1; i < 4; i++) {{
    ctx.lineTo(quad[i][0] * scaleX, quad[i][1] * scaleY);
  }}
  ctx.closePath();
  ctx.stroke();
}}

window.onload = init;
</script>
</body>
</html>
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"HTML dashboard generated: {output_path} ({len(html_content) / 1e6:.2f} MB)")


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    # Defaults for checkpoint, output_html, summary_json
    if args.checkpoint is None:
        args.checkpoint = Path(
            f"artifacts/phase2/checkpoints/mobilenetv4_p2_{args.geometry}_best.pt"
        )
    if args.output_html is None:
        name = (
            "student_validation_dashboard.html"
            if args.geometry == "hbb"
            else "student_quad_validation_dashboard.html"
        )
        args.output_html = Path(f"artifacts/phase2/visualizations/{name}")
    if args.summary_json is None:
        args.summary_json = Path(
            f"artifacts/phase2/training_{args.geometry}_summary.json"
        )
        if not args.summary_json.exists():
            args.summary_json = Path("artifacts/phase2/training_summary.json")

    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] Generating Student Validation Dashboard ({args.geometry.upper()})"
    )

    if not args.checkpoint.exists():
        raise FileNotFoundError(f"Checkpoint not found: {args.checkpoint}")

    # Load dataset
    if args.geometry == "hbb":
        val_dataset = SAMStudentHBBDataset(
            pseudo_labels_path=args.pseudo_labels,
            image_root=args.image_root,
            split="val",
            input_size=384,
        )
    else:
        val_dataset = SAMStudentDataset(
            pseudo_labels_path=args.pseudo_labels,
            image_root=args.image_root,
            split="val",
            input_size=384,
        )

    # Balanced cohort selection across all 4 cohorts
    by_cohort: dict[str, list[int]] = {}
    for idx, img in enumerate(val_dataset.images):
        by_cohort.setdefault(img["source_dataset"], []).append(idx)

    selected_indices: list[int] = []
    print(f"Validation cohorts available in {len(val_dataset)} val images:")
    for cohort_name, indices in sorted(by_cohort.items()):
        picked = indices[: args.samples_per_cohort]
        selected_indices.extend(picked)
        print(f"  - {cohort_name}: {len(indices)} available -> sampled {len(picked)}")

    selected_indices.sort()
    print(f"Total sampled images for dashboard: {len(selected_indices)}")

    # Load Model
    checkpoint_data = torch.load(args.checkpoint, map_location=device)
    strides = checkpoint_data.get("strides", (4, 8, 16, 32))
    is_dual = checkpoint_data.get("recipe") == "recipe2" or any(
        k.startswith("aux_head") for k in checkpoint_data["model_state_dict"].keys()
    )

    chosen_nms = "part_aware" if args.nms_mode == "auto" else args.nms_mode

    alpha = args.centerness_alpha
    if alpha is None:
        alpha = 1.0 if args.geometry == "hbb" else 0.0

    if args.geometry == "hbb":
        model = StudentDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        decoder = InferenceDecoder(
            strides=strides,
            top_k=1000,
            nms_iou_threshold=args.nms_iou_threshold,
            max_detections=100,
            score_mode=args.score_mode,
            centerness_alpha=alpha,
            nms_mode=chosen_nms,
            part_scale_ratio=args.part_scale_ratio,
            part_max_iou=args.part_max_iou,
        )
    else:
        model = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=is_dual,
        ).to(device)
        decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=1000,
            nms_iou_threshold=args.nms_iou_threshold,
            max_proposals=100,
            centerness_alpha=alpha,
            nms_mode=chosen_nms,
            part_scale_ratio=args.part_scale_ratio,
            part_max_iou=args.part_max_iou,
        )

    model.load_state_dict(checkpoint_data["model_state_dict"])
    model.eval()
    print(
        f"Loaded student checkpoint: {args.checkpoint.name} (Epoch {checkpoint_data.get('epoch', '?')})"
    )

    # Load training summary
    summary = {}
    if args.summary_json.exists():
        with open(args.summary_json, encoding="utf-8") as f:
            summary = json.load(f)

    if args.eval_metrics_json and args.eval_metrics_json.exists():
        with open(args.eval_metrics_json, encoding="utf-8") as f:
            summary["eval_metrics"] = json.load(f)

    # Ingest images and run predictions
    payload: list[dict[str, Any]] = []
    selected_samples = [val_dataset[idx] for idx in selected_indices]
    batch_size = 8

    with torch.inference_mode():
        for batch_start in range(0, len(selected_samples), batch_size):
            batch_slice = selected_samples[batch_start : batch_start + batch_size]
            batch_indices = selected_indices[batch_start : batch_start + batch_size]

            images_tensor = torch.stack([s.image for s in batch_slice]).to(device)
            output = model(images_tensor)
            detections = decoder(output, (384, 384))

            for idx_in_batch, (sample, orig_idx) in enumerate(
                zip(batch_slice, batch_indices, strict=True)
            ):
                rec = val_dataset.images[orig_idx]
                img_path = args.image_root / rec["file_name"]
                thumb_b64, _, _ = encode_thumbnail_base64(img_path)

                orig_h, orig_w = sample.original_size  # (height, width)
                scale, off_x, off_y = sample.transform
                det = detections[idx_in_batch]

                pred_items = []
                if args.geometry == "hbb":
                    boxes = det.boxes.clone()
                    scores = det.scores.clone()
                    boxes[:, 0] = ((boxes[:, 0] - off_x) / scale).clamp(0, orig_w)
                    boxes[:, 1] = ((boxes[:, 1] - off_y) / scale).clamp(0, orig_h)
                    boxes[:, 2] = ((boxes[:, 2] - off_x) / scale).clamp(0, orig_w)
                    boxes[:, 3] = ((boxes[:, 3] - off_y) / scale).clamp(0, orig_h)
                    scores_list = scores.tolist()
                    for b_idx in range(min(50, len(boxes))):
                        b = boxes[b_idx]
                        pred_items.append(
                            {
                                "box": [
                                    round(float(b[0]), 1),
                                    round(float(b[1]), 1),
                                    round(float(b[2]), 1),
                                    round(float(b[3]), 1),
                                ],
                                "score": round(float(scores_list[b_idx]), 3),
                            }
                        )
                else:
                    quads = det.quads.clone()
                    scores = det.scores.clone()
                    scores_list = scores.tolist()
                    for q_idx in range(min(50, len(quads))):
                        q = quads[q_idx].clone()
                        q[:, 0] = ((q[:, 0] - off_x) / scale).clamp(0, orig_w)
                        q[:, 1] = ((q[:, 1] - off_y) / scale).clamp(0, orig_h)
                        if args.wrap_to_bbox:
                            pred_items.append(
                                {
                                    "box": [
                                        round(float(q[:, 0].min()), 1),
                                        round(float(q[:, 1].min()), 1),
                                        round(float(q[:, 0].max()), 1),
                                        round(float(q[:, 1].max()), 1),
                                    ],
                                    "score": round(float(scores_list[q_idx]), 3),
                                }
                            )
                        else:
                            pred_items.append(
                                {
                                    "quad": [
                                        [round(float(pt[0]), 1), round(float(pt[1]), 1)]
                                        for pt in q
                                    ],
                                    "score": round(float(scores_list[q_idx]), 3),
                                }
                            )

                # Extract GT and SAM novel boxes/quads
                gt_items = []
                novel_items = []
                supp_items = []
                geom_key = (
                    "bbox" if (args.geometry == "hbb" or args.wrap_to_bbox) else "quad"
                )
                for ann in rec.get("annotations", []):
                    item = ann.get(geom_key)
                    if not item:
                        continue
                    if ann.get("is_positive") and not ann.get("is_suppressed"):
                        if ann.get("source") == "human_gt":
                            gt_items.append(item)
                        elif ann.get("source") == "sam_discovered":
                            novel_items.append(item)
                    elif ann.get("is_suppressed"):
                        supp_items.append(item)

                payload.append(
                    {
                        "image_id": rec["image_id"],
                        "source_dataset": rec["source_dataset"],
                        "camera_type": sample.camera_type,
                        "width": orig_w,
                        "height": orig_h,
                        "image_base64": thumb_b64,
                        "predictions": pred_items,
                        "gt_items": gt_items,
                        "novel_items": novel_items,
                        "suppressed_items": supp_items[:20],
                        "gt_count": len(gt_items),
                        "novel_count": len(novel_items),
                        "suppressed_count": len(supp_items),
                    }
                )

    print(f"Processed {len(payload)} validation images for dashboard.")
    active_geom = "hbb" if (args.geometry == "hbb" or args.wrap_to_bbox) else "quad"
    generate_html(payload, summary, args.output_html, geometry=active_geom)


if __name__ == "__main__":
    main()
