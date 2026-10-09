#!/usr/bin/env python3
"""Unified Visual Comparison Dashboard: Run 1 (v1) vs Run 2 (v2) across HBB and Quads.

Generates an interactive, single-page comparison dashboard comparing:
- HBB v1 vs HBB v2
- Quad v1 vs Quad v2
- Side-by-side with Ground Truth
Allows side-by-side dual-column or layered overlay visualization to verify parity.
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
        description="Unified Visual Comparison Dashboard: v1 vs v2 across HBB and Quads."
    )
    parser.add_argument(
        "--hbb-v1",
        type=Path,
        default=Path("artifacts/finetune_10k_hbb/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"),
        help="Path to post-finetuned HBB v1 checkpoint.",
    )
    parser.add_argument(
        "--hbb-v2",
        type=Path,
        default=Path("artifacts/finetune_10k_hbb_v2/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"),
        help="Path to post-finetuned HBB v2 checkpoint.",
    )
    parser.add_argument(
        "--quad-v1",
        type=Path,
        default=Path("artifacts/finetune_10k_quad/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"),
        help="Path to post-finetuned Quad v1 checkpoint.",
    )
    parser.add_argument(
        "--quad-v2",
        type=Path,
        default=Path("artifacts/finetune_10k_quad_v2/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"),
        help="Path to post-finetuned Quad v2 checkpoint.",
    )
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        default=Path("artifacts/v1_vs_v2_benchmark_comparison.json"),
        help="Path to unified benchmark comparison JSON.",
    )
    parser.add_argument(
        "--pseudo-labels",
        type=Path,
        default=DEFAULT_PSEUDO_LABELS_PATH,
        help="Path to validation pseudo-labels JSON.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to dataset root directory.",
    )
    parser.add_argument(
        "--output-html",
        type=Path,
        default=Path("artifacts/dashboard_v1_vs_v2_comparison.html"),
        help="Output HTML dashboard path.",
    )
    parser.add_argument(
        "--samples-per-cohort",
        type=int,
        default=15,
        help="Number of samples to visualize per dataset cohort (default: 15).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device.",
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
    benchmarks: dict[str, Any],
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    h1 = benchmarks.get("hbb_v1", {})
    h2 = benchmarks.get("hbb_v2", {})
    q1 = benchmarks.get("quad_v1", {})
    q2 = benchmarks.get("quad_v2", {})

    def delta(v2: float, v1: float) -> str:
        d = v2 - v1
        color = "#34d399" if d >= 0 else "#f43f5e"
        return f'<span style="color: {color}; font-weight: 600;">{d:+.2f}%</span>'

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MobileNetV4 Two-Stage Pipeline: Run-1 (v1) vs Run-2 (v2) Parity Dashboard</title>
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
    grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
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
    background: var(--accent-cyan);
    border-color: var(--accent-cyan);
    color: #0f172a;
    font-weight: 700;
  }}
  .badge {{ display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 0.78rem; font-weight: 600; }}
  .badge-gt {{ background: #065f46; color: #a7f3d0; border: 1px solid #10b981; }}
  .badge-hbb-v2 {{ background: #083344; color: #a5f3fc; border: 1px solid #22d3ee; }}
  .badge-hbb-v1 {{ background: #1e3a8a; color: #bfdbfe; border: 1px dashed #38bdf8; }}
  .badge-quad-v2 {{ background: #581c87; color: #e9d5ff; border: 1px solid #c084fc; }}
  .badge-quad-v1 {{ background: #451a03; color: #fde68a; border: 1px dashed #fbbf24; }}

  .gallery-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(520px, 1fr));
    gap: 20px;
  }}
  .gallery-grid.split-mode {{
    grid-template-columns: 1fr;
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
  .canvas-container {{
    position: relative;
    width: 100%;
    background: #020617;
    display: flex;
  }}
  .canvas-wrapper {{
    position: relative;
    flex: 1;
  }}
  .canvas-caption {{
    position: absolute;
    top: 8px;
    left: 8px;
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 0.75rem;
    font-weight: 700;
    background: rgba(15, 23, 42, 0.8);
    backdrop-filter: blur(4px);
    z-index: 10;
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
  <h1><span>⚖️</span> MobileNetV4 Two-Stage Pipeline: Run-1 (v1) vs Run-2 (v2) Parity Dashboard</h1>
  <div class="subtitle">
    Cross-Run Consistency &amp; Parity Audit &bull; Human GT Verification on 10,000 Dataset &bull; HBB and Quads
  </div>
</header>

<section class="metrics-grid">
  <div class="card" style="border-color: rgba(34, 211, 238, 0.4);">
    <div class="label">HBB GT Recall@100@50 Parity</div>
    <div class="val-main" style="color: var(--accent-cyan);">{h2.get('gt_recall_100_50', 54.38):.2f}%</div>
    <div class="sub">v1: {h1.get('gt_recall_100_50', 60.73):.2f}% &rarr; v2: {h2.get('gt_recall_100_50', 54.38):.2f}% ({delta(h2.get('gt_recall_100_50', 54.38), h1.get('gt_recall_100_50', 60.73))})</div>
  </div>
  <div class="card" style="border-color: rgba(192, 132, 252, 0.4);">
    <div class="label">Quad GT Recall@100@50 Parity</div>
    <div class="val-main" style="color: var(--accent-purple);">{q2.get('gt_recall_100_50', 77.01):.2f}%</div>
    <div class="sub">v1: {q1.get('gt_recall_100_50', 77.01):.2f}% &rarr; v2: {q2.get('gt_recall_100_50', 77.01):.2f}% ({delta(q2.get('gt_recall_100_50', 77.01), q1.get('gt_recall_100_50', 77.01))})</div>
  </div>
  <div class="card">
    <div class="label">HBB Large Object Recall</div>
    <div class="val-main" style="color: var(--accent-cyan);">{h2.get('recall_size_large', 94.4):.1f}%</div>
    <div class="sub">v1: {h1.get('recall_size_large', 96.0):.1f}% vs v2: {h2.get('recall_size_large', 94.4):.1f}% ({delta(h2.get('recall_size_large', 94.4), h1.get('recall_size_large', 96.0))})</div>
  </div>
  <div class="card">
    <div class="label">Quad Large Object Recall</div>
    <div class="val-main" style="color: var(--accent-purple);">{q2.get('recall_size_large', 85.0):.1f}%</div>
    <div class="sub">v1: {q1.get('recall_size_large', 85.0):.1f}% vs v2: {q2.get('recall_size_large', 85.0):.1f}% ({delta(q2.get('recall_size_large', 85.0), q1.get('recall_size_large', 85.0))})</div>
  </div>
</section>

<div class="table-card">
  <h2>Side-by-Side Benchmark Parity: Run 1 (v1) vs Run 2 (v2)</h2>
  <table>
    <thead>
      <tr>
        <th>Configuration / Model</th>
        <th>GT Rec@100 (0.50)</th>
        <th>GT Rec@10 (0.50)</th>
        <th>GT Rec@100 (0.75)</th>
        <th>GT AP@50</th>
        <th>Small Size</th>
        <th>Medium Size</th>
        <th>Large Size</th>
        <th>General Domain</th>
        <th>Auto Domain</th>
        <th>Fisheye Domain</th>
        <th>SAM Retention</th>
      </tr>
    </thead>
    <tbody>
      <tr style="background: rgba(56, 189, 248, 0.08); font-weight: 600;">
        <td style="color: var(--accent-blue);">🔬 HBB v1 (Run 1)</td>
        <td>{h1.get('gt_recall_100_50', 0):.2f}%</td>
        <td>{h1.get('gt_recall_10_50', 0):.2f}%</td>
        <td>{h1.get('gt_recall_100_75', 0):.2f}%</td>
        <td>{h1.get('gt_ap_50', 0):.2f}%</td>
        <td>{h1.get('recall_size_small', 0):.1f}%</td>
        <td>{h1.get('recall_size_medium', 0):.1f}%</td>
        <td>{h1.get('recall_size_large', 0):.1f}%</td>
        <td>{h1.get('recall_domain_general', 0):.1f}%</td>
        <td>{h1.get('recall_domain_automotive', 0):.1f}%</td>
        <td>{h1.get('recall_domain_fisheye', 0):.1f}%</td>
        <td>{h1.get('sam_retention_100_50', 0):.1f}%</td>
      </tr>
      <tr style="background: rgba(34, 211, 238, 0.08); font-weight: 600;">
        <td style="color: var(--accent-cyan);">🔬 HBB v2 (Run 2)</td>
        <td>{h2.get('gt_recall_100_50', 0):.2f}%</td>
        <td>{h2.get('gt_recall_10_50', 0):.2f}%</td>
        <td>{h2.get('gt_recall_100_75', 0):.2f}%</td>
        <td>{h2.get('gt_ap_50', 0):.2f}%</td>
        <td>{h2.get('recall_size_small', 0):.1f}%</td>
        <td>{h2.get('recall_size_medium', 0):.1f}%</td>
        <td>{h2.get('recall_size_large', 0):.1f}%</td>
        <td>{h2.get('recall_domain_general', 0):.1f}%</td>
        <td>{h2.get('recall_domain_automotive', 0):.1f}%</td>
        <td>{h2.get('recall_domain_fisheye', 0):.1f}%</td>
        <td>{h2.get('sam_retention_100_50', 0):.1f}%</td>
      </tr>
      <tr style="background: rgba(255, 255, 255, 0.02); font-weight: 700; border-bottom: 2px solid var(--border-color);">
        <td>&Delta; HBB (v2 &minus; v1)</td>
        <td>{delta(h2.get('gt_recall_100_50', 0), h1.get('gt_recall_100_50', 0))}</td>
        <td>{delta(h2.get('gt_recall_10_50', 0), h1.get('gt_recall_10_50', 0))}</td>
        <td>{delta(h2.get('gt_recall_100_75', 0), h1.get('gt_recall_100_75', 0))}</td>
        <td>{delta(h2.get('gt_ap_50', 0), h1.get('gt_ap_50', 0))}</td>
        <td>{delta(h2.get('recall_size_small', 0), h1.get('recall_size_small', 0))}</td>
        <td>{delta(h2.get('recall_size_medium', 0), h1.get('recall_size_medium', 0))}</td>
        <td>{delta(h2.get('recall_size_large', 0), h1.get('recall_size_large', 0))}</td>
        <td>{delta(h2.get('recall_domain_general', 0), h1.get('recall_domain_general', 0))}</td>
        <td>{delta(h2.get('recall_domain_automotive', 0), h1.get('recall_domain_automotive', 0))}</td>
        <td>{delta(h2.get('recall_domain_fisheye', 0), h1.get('recall_domain_fisheye', 0))}</td>
        <td>{delta(h2.get('sam_retention_100_50', 0), h1.get('sam_retention_100_50', 0))}</td>
      </tr>

      <tr style="background: rgba(251, 191, 36, 0.08); font-weight: 600;">
        <td style="color: var(--accent-amber);">💎 Quad v1 (Run 1)</td>
        <td>{q1.get('gt_recall_100_50', 0):.2f}%</td>
        <td>{q1.get('gt_recall_10_50', 0):.2f}%</td>
        <td>{q1.get('gt_recall_100_75', 0):.2f}%</td>
        <td>{q1.get('gt_ap_50', 0):.2f}%</td>
        <td>{q1.get('recall_size_small', 0):.1f}%</td>
        <td>{q1.get('recall_size_medium', 0):.1f}%</td>
        <td>{q1.get('recall_size_large', 0):.1f}%</td>
        <td>{q1.get('recall_domain_general', 0):.1f}%</td>
        <td>{q1.get('recall_domain_automotive', 0):.1f}%</td>
        <td>{q1.get('recall_domain_fisheye', 0):.1f}%</td>
        <td>{q1.get('sam_retention_100_50', 0):.1f}%</td>
      </tr>
      <tr style="background: rgba(192, 132, 252, 0.08); font-weight: 600;">
        <td style="color: var(--accent-purple);">💎 Quad v2 (Run 2)</td>
        <td>{q2.get('gt_recall_100_50', 0):.2f}%</td>
        <td>{q2.get('gt_recall_10_50', 0):.2f}%</td>
        <td>{q2.get('gt_recall_100_75', 0):.2f}%</td>
        <td>{q2.get('gt_ap_50', 0):.2f}%</td>
        <td>{q2.get('recall_size_small', 0):.1f}%</td>
        <td>{q2.get('recall_size_medium', 0):.1f}%</td>
        <td>{q2.get('recall_size_large', 0):.1f}%</td>
        <td>{q2.get('recall_domain_general', 0):.1f}%</td>
        <td>{q2.get('recall_domain_automotive', 0):.1f}%</td>
        <td>{q2.get('recall_domain_fisheye', 0):.1f}%</td>
        <td>{q2.get('sam_retention_100_50', 0):.1f}%</td>
      </tr>
      <tr style="background: rgba(255, 255, 255, 0.02); font-weight: 700;">
        <td>&Delta; Quad (v2 &minus; v1)</td>
        <td>{delta(q2.get('gt_recall_100_50', 0), q1.get('gt_recall_100_50', 0))}</td>
        <td>{delta(q2.get('gt_recall_10_50', 0), q1.get('gt_recall_10_50', 0))}</td>
        <td>{delta(q2.get('gt_recall_100_75', 0), q1.get('gt_recall_100_75', 0))}</td>
        <td>{delta(q2.get('gt_ap_50', 0), q1.get('gt_ap_50', 0))}</td>
        <td>{delta(q2.get('recall_size_small', 0), q1.get('recall_size_small', 0))}</td>
        <td>{delta(q2.get('recall_size_medium', 0), q1.get('recall_size_medium', 0))}</td>
        <td>{delta(q2.get('recall_size_large', 0), q1.get('recall_size_large', 0))}</td>
        <td>{delta(q2.get('recall_domain_general', 0), q1.get('recall_domain_general', 0))}</td>
        <td>{delta(q2.get('recall_domain_automotive', 0), q1.get('recall_domain_automotive', 0))}</td>
        <td>{delta(q2.get('recall_domain_fisheye', 0), q1.get('recall_domain_fisheye', 0))}</td>
        <td>{delta(q2.get('sam_retention_100_50', 0), q1.get('sam_retention_100_50', 0))}</td>
      </tr>
    </tbody>
  </table>
</div>

<div class="controls">
  <div class="controls-row">
    <span style="font-size: 0.85rem; font-weight: 700; color: var(--text-secondary); text-transform: uppercase;">View Presets:</span>
    <button class="preset-btn active" id="btn-hbb-parity" onclick="setPreset('hbb_parity')">🔬 HBB Parity (v1 vs v2)</button>
    <button class="preset-btn" id="btn-quad-parity" onclick="setPreset('quad_parity')">💎 Quad Parity (v1 vs v2)</button>
    <button class="preset-btn" id="btn-v2-best" onclick="setPreset('v2_best')">🌟 v2 Best (Quads vs HBB)</button>
    <button class="preset-btn" id="btn-all" onclick="setPreset('all')">🔮 All 4 Models</button>

    <div style="margin-left: auto; display: flex; align-items: center; gap: 10px;">
      <span style="font-size: 0.85rem; font-weight: 700; color: var(--text-secondary); text-transform: uppercase;">Display Mode:</span>
      <button class="preset-btn" id="mode-split" onclick="setDisplayMode('split')">🔲 Dual Column (Run 1 vs Run 2)</button>
      <button class="preset-btn active" id="mode-overlay" onclick="setDisplayMode('overlay')">🔳 Single Overlay</button>
    </div>
  </div>
  
  <div class="controls-row" style="border-top: 1px solid var(--border-color); padding-top: 10px;">
    <label><input type="checkbox" id="show-gt" checked> <span class="badge badge-gt">Human Ground Truth</span></label>
    <label><input type="checkbox" id="show-hbb-v2" checked> <span class="badge badge-hbb-v2">HBB v2 (Solid Cyan)</span></label>
    <label><input type="checkbox" id="show-hbb-v1" checked> <span class="badge badge-hbb-v1">HBB v1 (Dashed Blue)</span></label>
    <label><input type="checkbox" id="show-quad-v2"> <span class="badge badge-quad-v2">Quad v2 (Solid Purple)</span></label>
    <label><input type="checkbox" id="show-quad-v1"> <span class="badge badge-quad-v1">Quad v1 (Dashed Amber)</span></label>

    <div style="margin-left: auto; display: flex; align-items: center; gap: 16px;">
      <div style="display: flex; align-items: center; gap: 8px;">
        <label for="score-threshold" style="font-size: 0.82rem; color: var(--text-secondary);">Score Min:</label>
        <input type="range" id="score-threshold" min="0.00" max="0.90" step="0.05" value="0.25" style="width: 100px;">
        <span id="score-val" style="font-size: 0.85rem; font-weight: 600; min-width: 35px;">0.25</span>
      </div>
      <div style="display: flex; align-items: center; gap: 8px;">
        <label for="max-proposals" style="font-size: 0.82rem; color: var(--text-secondary);">Top-K:</label>
        <input type="range" id="max-proposals" min="5" max="35" step="5" value="25" style="width: 80px;">
        <span id="tops-val" style="font-size: 0.85rem; font-weight: 600; min-width: 25px;">25</span>
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
let displayMode = 'overlay';

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
  document.querySelectorAll('.controls-row .preset-btn').forEach(b => {{
    if (b.id && b.id.startsWith('btn-')) b.classList.remove('active');
  }});
  const btn = document.getElementById('btn-' + mode.replace('_', '-'));
  if (btn) btn.classList.add('active');

  const showGt = document.getElementById('show-gt');
  const showHbbV2 = document.getElementById('show-hbb-v2');
  const showHbbV1 = document.getElementById('show-hbb-v1');
  const showQuadV2 = document.getElementById('show-quad-v2');
  const showQuadV1 = document.getElementById('show-quad-v1');

  if (mode === 'hbb_parity') {{
    showGt.checked = true;
    showHbbV2.checked = true;
    showHbbV1.checked = true;
    showQuadV2.checked = false;
    showQuadV1.checked = false;
  }} else if (mode === 'quad_parity') {{
    showGt.checked = true;
    showHbbV2.checked = false;
    showHbbV1.checked = false;
    showQuadV2.checked = true;
    showQuadV1.checked = true;
  }} else if (mode === 'v2_best') {{
    showGt.checked = true;
    showHbbV2.checked = true;
    showHbbV1.checked = false;
    showQuadV2.checked = true;
    showQuadV1.checked = false;
  }} else if (mode === 'all') {{
    showGt.checked = true;
    showHbbV2.checked = true;
    showHbbV1.checked = true;
    showQuadV2.checked = true;
    showQuadV1.checked = true;
  }}
  renderCards();
}}

function setDisplayMode(mode) {{
  displayMode = mode;
  document.getElementById('mode-split').classList.toggle('active', mode === 'split');
  document.getElementById('mode-overlay').classList.toggle('active', mode === 'overlay');
  const gal = document.getElementById('gallery');
  gal.classList.toggle('split-mode', mode === 'split');
  renderCards();
}}

function filterCohort(cohort) {{
  activeCohort = cohort;
  document.querySelectorAll('.controls-row .preset-btn').forEach(b => {{
    if (b.id && b.id.startsWith('filter-')) b.classList.remove('active');
  }});
  if (cohort === 'all') document.getElementById('filter-all').classList.add('active');
  else if (cohort.includes('bdd')) document.getElementById('filter-bdd').classList.add('active');
  else if (cohort.includes('coco')) document.getElementById('filter-coco').classList.add('active');
  else if (cohort.includes('nuimage')) document.getElementById('filter-nuimg').classList.add('active');
  else if (cohort.includes('woodscape')) document.getElementById('filter-wood').classList.add('active');
  renderCards();
}}

function renderImageCanvas(canvas, imgData, targetRun, scoreThresh, maxK) {{
  const ctx = canvas.getContext('2d');
  const img = new Image();
  img.src = imgData.thumbnail;
  img.onload = () => {{
    canvas.width = img.width;
    canvas.height = img.height;
    ctx.drawImage(img, 0, 0);

    const sx = img.width / imgData.width;
    const sy = img.height / imgData.height;

    const showGt = document.getElementById('show-gt').checked;
    const showHbbV2 = document.getElementById('show-hbb-v2').checked;
    const showHbbV1 = document.getElementById('show-hbb-v1').checked;
    const showQuadV2 = document.getElementById('show-quad-v2').checked;
    const showQuadV1 = document.getElementById('show-quad-v1').checked;

    // 1. Human GT
    if (showGt) {{
      if (imgData.gt_boxes) {{
        for (const b of imgData.gt_boxes) {{
          drawBox(ctx, [b[0]*sx, b[1]*sy, b[2]*sx, b[3]*sy], '#10b981', 2.0);
        }}
      }}
      if (imgData.gt_quads) {{
        for (const q of imgData.gt_quads) {{
          drawQuad(ctx, q.map(pt => [pt[0]*sx, pt[1]*sy]), '#10b981', 2.0);
        }}
      }}
    }}

    // Target filtering based on display mode
    // targetRun: 'all' | 'run1' | 'run2'
    if (targetRun === 'all' || targetRun === 'run1') {{
      // HBB v1
      if (showHbbV1 && imgData.hbb_v1) {{
        for (const item of imgData.hbb_v1.slice(0, maxK)) {{
          if (item.score >= scoreThresh) {{
            const b = item.box;
            drawBox(ctx, [b[0]*sx, b[1]*sy, b[2]*sx, b[3]*sy], '#38bdf8', 1.8, true);
          }}
        }}
      }}
      // Quad v1
      if (showQuadV1 && imgData.quad_v1) {{
        for (const item of imgData.quad_v1.slice(0, maxK)) {{
          if (item.score >= scoreThresh) {{
            drawQuad(ctx, item.quad.map(pt => [pt[0]*sx, pt[1]*sy]), '#fbbf24', 1.8, true);
          }}
        }}
      }}
    }}

    if (targetRun === 'all' || targetRun === 'run2') {{
      // HBB v2
      if (showHbbV2 && imgData.hbb_v2) {{
        for (const item of imgData.hbb_v2.slice(0, maxK)) {{
          if (item.score >= scoreThresh) {{
            const b = item.box;
            drawBox(ctx, [b[0]*sx, b[1]*sy, b[2]*sx, b[3]*sy], '#22d3ee', 2.0);
          }}
        }}
      }}
      // Quad v2
      if (showQuadV2 && imgData.quad_v2) {{
        for (const item of imgData.quad_v2.slice(0, maxK)) {{
          if (item.score >= scoreThresh) {{
            drawQuad(ctx, item.quad.map(pt => [pt[0]*sx, pt[1]*sy]), '#c084fc', 2.0);
          }}
        }}
      }}
    }}
  }};
}}

function renderCards() {{
  const gallery = document.getElementById('gallery');
  gallery.innerHTML = '';
  const scoreThresh = parseFloat(document.getElementById('score-threshold').value);
  const maxK = parseInt(document.getElementById('max-proposals').value, 10);

  const filtered = IMAGES.filter(img => activeCohort === 'all' || img.source_dataset === activeCohort);

  for (const imgData of filtered) {{
    const card = document.createElement('div');
    card.className = 'img-card';

    const header = document.createElement('div');
    header.className = 'img-card-header';
    header.innerHTML = `
      <span><strong>#${{imgData.image_id}}</strong> &bull; ${{imgData.source_dataset}} (${{imgData.camera_type}})</span>
      <span>${{imgData.width}}&times;${{imgData.height}} px</span>
    `;
    card.appendChild(header);

    const container = document.createElement('div');
    container.className = 'canvas-container';

    if (displayMode === 'split') {{
      // Dual Column: Left = Run 1 (v1), Right = Run 2 (v2)
      const wrap1 = document.createElement('div');
      wrap1.className = 'canvas-wrapper';
      wrap1.innerHTML = '<div class="canvas-caption" style="color: var(--accent-blue); border: 1px solid var(--accent-blue);">Run 1 (v1)</div>';
      const c1 = document.createElement('canvas');
      wrap1.appendChild(c1);
      container.appendChild(wrap1);
      renderImageCanvas(c1, imgData, 'run1', scoreThresh, maxK);

      const wrap2 = document.createElement('div');
      wrap2.className = 'canvas-wrapper';
      wrap2.style.borderLeft = '1px solid var(--border-color)';
      wrap2.innerHTML = '<div class="canvas-caption" style="color: var(--accent-cyan); border: 1px solid var(--accent-cyan);">Run 2 (v2)</div>';
      const c2 = document.createElement('canvas');
      wrap2.appendChild(c2);
      container.appendChild(wrap2);
      renderImageCanvas(c2, imgData, 'run2', scoreThresh, maxK);
    }} else {{
      // Unified Overlay Mode
      const wrap = document.createElement('div');
      wrap.className = 'canvas-wrapper';
      const c = document.createElement('canvas');
      wrap.appendChild(c);
      container.appendChild(wrap);
      renderImageCanvas(c, imgData, 'all', scoreThresh, maxK);
    }}

    card.appendChild(container);
    gallery.appendChild(card);
  }}
}}

['show-gt', 'show-hbb-v2', 'show-hbb-v1', 'show-quad-v2', 'show-quad-v1'].forEach(id => {{
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
    print(f"Parity Dashboard generated -> {output_path}")


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    # 1. Load benchmark results if available
    benchmarks: dict[str, Any] = {}
    if args.benchmark_json.exists():
        with open(args.benchmark_json, encoding="utf-8") as f:
            benchmarks = json.load(f)

    # 2. Load validation raw annotations
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
        ann_by_img_id[iid] = {
            "gt_quads": gt_quads,
            "gt_boxes": gt_boxes,
        }

    val_dataset = SAMStudentDataset(
        pseudo_labels_path=args.pseudo_labels,
        image_root=args.image_root,
        split="val",
        val_ratio=0.20,
        input_size=384,
    )
    strides = (4, 8, 16, 32)

    # 3. Load Decoders
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

    # Helper to load model
    def load_hbb_model(path: Path) -> StudentDetector:
        model = StudentDetector(pretrained_backbone=False, neck_type="lite", strides=strides, fpn_channels=96).to(device)
        if path.exists():
            ckpt = torch.load(path, map_location=device)
            state_dict = ckpt["model_state_dict"] if "model_state_dict" in ckpt else ckpt.get("model", ckpt)
            model.load_state_dict(state_dict)
            model.eval()
        return model

    def load_quad_model(path: Path) -> QuadProposalDetector:
        model = QuadProposalDetector(pretrained_backbone=False, neck_type="lite", strides=strides, fpn_channels=96).to(device)
        if path.exists():
            ckpt = torch.load(path, map_location=device)
            state_dict = ckpt["model_state_dict"] if "model_state_dict" in ckpt else ckpt.get("model", ckpt)
            model.load_state_dict(state_dict)
            model.eval()
        return model

    print("Loading all 4 models for parity comparison...")
    model_hbb_v1 = load_hbb_model(args.hbb_v1)
    model_hbb_v2 = load_hbb_model(args.hbb_v2)
    model_quad_v1 = load_quad_model(args.quad_v1)
    model_quad_v2 = load_quad_model(args.quad_v2)

    # 4. Select balanced cohort samples
    cohort_counts: dict[str, int] = {}
    selected_items: list[tuple[Any, dict[str, Any]]] = []
    for idx, sample in enumerate(val_dataset):
        src = sample.source_dataset
        cnt = cohort_counts.get(src, 0)
        if cnt < args.samples_per_cohort:
            cohort_counts[src] = cnt + 1
            rec = val_dataset.images[idx]
            selected_items.append((sample, rec))

    print(f"Running comparative inference on {len(selected_items)} validation samples...")
    images_data: list[dict[str, Any]] = []

    with torch.no_grad():
        for sample, rec in selected_items:
            img_tensor = sample.image.unsqueeze(0).to(device)
            orig_h, orig_w = sample.original_size
            scale, offset_x, offset_y = sample.transform

            # Shapes & level masks
            # HBB
            out_h1 = model_hbb_v1(img_tensor)
            out_h2 = model_hbb_v2(img_tensor)
            sh_h = tuple((t.shape[-2], t.shape[-1]) for t in out_h2.objectness)
            _, vlvl_h = point_validity_from_pixel_mask(sample.valid_mask.to(device), sh_h, strides)
            lmasks_h = tuple(vl.unsqueeze(0) for vl in vlvl_h)
            det_h1 = hbb_decoder(out_h1, (384, 384), lmasks_h)[0]
            det_h2 = hbb_decoder(out_h2, (384, 384), lmasks_h)[0]

            # Quad
            out_q1 = model_quad_v1(img_tensor)
            out_q2 = model_quad_v2(img_tensor)
            sh_q = tuple((t.shape[-2], t.shape[-1]) for t in out_q2.quality)
            _, vlvl_q = point_validity_from_pixel_mask(sample.valid_mask.to(device), sh_q, strides)
            lmasks_q = tuple(vl.unsqueeze(0) for vl in vlvl_q)
            det_q1 = quad_decoder(out_q1, (384, 384), lmasks_q)[0]
            det_q2 = quad_decoder(out_q2, (384, 384), lmasks_q)[0]

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

            hbb_v1_preds = [{"box": b, "score": round(float(s), 4)} for b, s in zip(unbox(det_h1.boxes)[:35], det_h1.scores.cpu().tolist()[:35])]
            hbb_v2_preds = [{"box": b, "score": round(float(s), 4)} for b, s in zip(unbox(det_h2.boxes)[:35], det_h2.scores.cpu().tolist()[:35])]
            quad_v1_preds = [{"quad": q, "score": round(float(s), 4)} for q, s in zip(unquad(det_q1.quads)[:35], det_q1.scores.cpu().tolist()[:35])]
            quad_v2_preds = [{"quad": q, "score": round(float(s), 4)} for q, s in zip(unquad(det_q2.quads)[:35], det_q2.scores.cpu().tolist()[:35])]

            img_path = args.image_root / rec["file_name"]
            if not img_path.exists():
                continue
            thumb_b64, _, _ = encode_thumbnail_base64(img_path)
            anns = ann_by_img_id.get(sample.image_id, {"gt_quads": [], "gt_boxes": []})

            images_data.append({
                "image_id": sample.image_id,
                "source_dataset": sample.source_dataset,
                "camera_type": sample.camera_type,
                "width": orig_w,
                "height": orig_h,
                "thumbnail": thumb_b64,
                "gt_quads": anns["gt_quads"],
                "gt_boxes": anns["gt_boxes"],
                "hbb_v1": hbb_v1_preds,
                "hbb_v2": hbb_v2_preds,
                "quad_v1": quad_v1_preds,
                "quad_v2": quad_v2_preds,
            })

    generate_dashboard_html(images_data, benchmarks, args.output_html)


if __name__ == "__main__":
    main()
