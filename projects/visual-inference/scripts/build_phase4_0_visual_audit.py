"""Generate a self-contained, interactive HTML visual audit dashboard for Phase 4.0.

Supports switchable inspection of both Quad and BBox (HBB) detector models,
as well as toggling between All Raw Class-Agnostic Proposals and SigLIP Classified Detections.
"""

from __future__ import annotations

import argparse
import base64
from collections import defaultdict
import io
import json
from pathlib import Path
from typing import Any

from PIL import Image
import torch
import torchvision.transforms.functional as TF

from student_detector.checkpoints import load_checkpoint, load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.crop_extractor import CropPolicy, extract_object_crop
from student_detector.data import select_source_mixture_indices
from student_detector.decoder import InferenceDecoder
from student_detector.head import QuadDetectorOutput
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.quad_data import QuadProposalDataset
from student_detector.ontology import is_compatible
from student_detector.proposal_contract import SourceTransform
from student_detector.proposal_evaluation import boxes_to_quads
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou
from student_detector.semantic_evaluation_data import (
    SemanticImageRecord,
    load_semantic_image_records,
    select_stratified_semantic_cohort,
)
from student_detector.siglip import SiglipTeacher


def _resolve_path(p: Path | str) -> Path:
    path = Path(p)
    if path.is_file() or path.is_dir():
        return path.resolve()
    for prefix in (
        Path.cwd(),
        Path("/home/sashamikoff/git/skillup/projects/visual-inference"),
        Path("/home/sashamikoff/git/skillup"),
    ):
        cand = prefix / path
        if cand.is_file() or cand.is_dir():
            return cand.resolve()
    return path.resolve()


def _pil_to_base64_jpeg(img: Image.Image, quality: int = 80) -> str:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality, optimize=True)
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")
    return f"data:image/jpeg;base64,{b64}"


def build_crop_gallery(
    image_records: list[SemanticImageRecord],
    teacher: SiglipTeacher,
    text_bank: torch.Tensor,
    categories: list[str],
    policy: CropPolicy,
    max_crops_per_cat: int = 6,
) -> list[dict[str, Any]]:
    """Extract and classify representative crops for the visual gallery."""
    print("Building visual crop gallery across categories...", flush=True)
    crops_by_cat: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for img_rec in image_records:
        if not img_rec.image_path.is_file():
            continue
        try:
            pil_img = Image.open(img_rec.image_path).convert("RGB")
        except Exception:
            continue

        for gt in img_rec.positive_ground_truths:
            cat = gt.canonical_category
            if not cat or cat not in categories:
                continue
            if len(crops_by_cat[cat]) >= max_crops_per_cat * 2:
                continue

            extracted = extract_object_crop(pil_img, gt.quad, policy)
            if not extracted.is_valid:
                continue

            crops_by_cat[cat].append({
                "image": extracted.image,
                "gt_category": cat,
                "size_bin": gt.size_bin,
                "bbox": list(gt.bbox),
                "file_name": img_rec.file_name,
                "image_id": img_rec.image_id,
            })

    all_crops = []
    metadata = []
    for cat, items in crops_by_cat.items():
        for item in items[:max_crops_per_cat * 2]:
            all_crops.append(item["image"])
            metadata.append(item)

    if not all_crops:
        return []

    print(f"Classifying {len(all_crops)} gallery crops with SigLIP...", flush=True)
    embeddings = teacher.encode_images(all_crops, batch_size=32)
    _, probs = teacher.compute_similarity(embeddings, text_bank)

    gallery: list[dict[str, Any]] = []
    for idx, item in enumerate(metadata):
        p_row = probs[idx]
        sorted_indices = torch.argsort(p_row, descending=True).tolist()
        top1_cat = categories[sorted_indices[0]]
        top3_cats = [
            {"category": categories[i], "probability": round(float(p_row[i].item()), 3)}
            for i in sorted_indices[:3]
        ]
        gt_cat = item["gt_category"]
        is_match = is_compatible(top1_cat, gt_cat)

        crop_b64 = _pil_to_base64_jpeg(item["image"], quality=85)
        gallery.append({
            "image_b64": crop_b64,
            "gt_category": gt_cat,
            "predicted_top1": top1_cat,
            "is_correct": is_match,
            "top3": top3_cats,
            "size_bin": item["size_bin"],
            "image_id": item["image_id"],
        })

    return gallery


def build_dual_model_image_records(
    selected_image_ids: list[int],
    dataset_root: Path,
    quad_detector: QuadProposalDetector,
    hbb_detector: StudentDetector,
    teacher: SiglipTeacher,
    text_bank: torch.Tensor,
    categories: list[str],
    policy: CropPolicy,
    device: torch.device,
    max_images: int = 25,
) -> list[dict[str, Any]]:
    """Build image records with proposals from both Quad and HBB models."""
    print(f"Generating dual-model proposals (Quad + HBB) for {max_images} images...", flush=True)
    semantic_records = load_semantic_image_records(dataset_root, selected_image_ids[:max_images])

    quad_decoder = QuadInferenceDecoder(
        strides=(8, 16, 32),
        pre_nms_top_k=300,
        nms_iou_threshold=0.78,
        max_proposals=100,
    )
    hbb_decoder = InferenceDecoder(
        strides=(8, 16, 32),
        top_k=300,
        nms_iou_threshold=0.70,
        max_detections=100,
    )

    result_records: list[dict[str, Any]] = []

    for idx, img_rec in enumerate(semantic_records):
        if not img_rec.image_path.is_file():
            continue
        try:
            pil_img = Image.open(img_rec.image_path).convert("RGB")
        except Exception:
            continue

        orig_w, orig_h = pil_img.size

        # Model input preparation (resize letterbox to 384x384)
        scale = min(384.0 / orig_w, 384.0 / orig_h)
        scaled_w = int(round(orig_w * scale))
        scaled_h = int(round(orig_h * scale))
        offset_x = (384 - scaled_w) / 2.0
        offset_y = (384 - scaled_h) / 2.0

        resized_img = pil_img.resize((scaled_w, scaled_h), Image.Resampling.BILINEAR)
        canvas = Image.new("RGB", (384, 384), color=(128, 128, 128))
        canvas.paste(resized_img, (int(offset_x), int(offset_y)))

        tensor_img = TF.to_tensor(canvas).unsqueeze(0).to(device)

        # 1. Quad inference
        with torch.no_grad():
            q_out = quad_detector(tensor_img)
            q_out = QuadDetectorOutput(
                quality=tuple(item.float() for item in q_out.quality),
                corner_offsets=tuple(item.float() for item in q_out.corner_offsets),
            )
            q_dets = quad_decoder(q_out, (384, 384))[0]

        # 2. HBB inference
        with torch.no_grad():
            h_out = hbb_detector(tensor_img)
            h_dets = hbb_decoder(h_out, (384, 384))[0]

        source_transform = SourceTransform(
            scale=scale,
            offset_x=offset_x,
            offset_y=offset_y,
            source_size=(orig_h, orig_w),
            model_size=(384, 384),
        )

        q_source_quads = source_transform.model_to_source(q_dets.quads)
        q_scores = q_dets.scores.cpu()

        h_quads_model = boxes_to_quads(h_dets.boxes)
        h_source_quads = source_transform.model_to_source(h_quads_model)
        h_scores = h_dets.scores.cpu()

        # GT objects
        pos_gts = [gt for gt in img_rec.positive_ground_truths if gt.canonical_category in categories]
        gt_tensor = (
            torch.tensor([gt.quad for gt in pos_gts], dtype=torch.float32)
            if pos_gts
            else torch.empty((0, 4, 2))
        )

        # Preview image
        preview_img = pil_img.copy()
        preview_img.thumbnail((640, 640), Image.Resampling.BILINEAR)
        preview_b64 = _pil_to_base64_jpeg(preview_img, quality=70)
        pw, ph = preview_img.size
        sx = pw / orig_w
        sy = ph / orig_h

        # Scaled GT
        scaled_gt = [
            {
                "quad": [[round(p[0] * sx, 1), round(p[1] * sy, 1)] for p in gt.quad],
                "category": gt.canonical_category,
                "size_bin": gt.size_bin,
            }
            for gt in pos_gts
        ]

        def process_model_proposals(source_quads: torch.Tensor, scores: torch.Tensor, max_classify: int = 30) -> list[dict[str, Any]]:
            count = source_quads.shape[0]
            if count == 0:
                return []

            # Match against GT
            if gt_tensor.shape[0] > 0:
                ious = pairwise_quad_iou(gt_tensor, source_quads.cpu())  # [N_gt, N_prop]
                best_ious, best_gt_idxs = ious.max(dim=0)
            else:
                best_ious = torch.zeros(count)
                best_gt_idxs = torch.zeros(count, dtype=torch.long)

            # Classify top proposals with SigLIP
            num_to_classify = min(count, max_classify)
            crops = []
            crop_prop_indices = []
            for p_i in range(num_to_classify):
                quad_pts = source_quads[p_i].tolist()
                extracted = extract_object_crop(pil_img, quad_pts, policy)
                if extracted.is_valid:
                    crops.append(extracted.image)
                    crop_prop_indices.append(p_i)

            p_cats_map: dict[int, tuple[str, float]] = {}
            if crops:
                with torch.no_grad():
                    embeds = teacher.encode_images(crops, batch_size=32)
                    _, p_probs = teacher.compute_similarity(embeds, text_bank)
                for c_i, p_orig_idx in enumerate(crop_prop_indices):
                    row = p_probs[c_i]
                    top_idx = torch.argmax(row).item()
                    top_prob = float(row[top_idx].item())
                    p_cats_map[p_orig_idx] = (categories[top_idx], top_prob)

            proposals_list = []
            for p_i in range(count):
                q_pts = source_quads[p_i].tolist()
                q_score = float(scores[p_i].item())
                biou = round(float(best_ious[p_i].item()), 3)
                gt_cat = str(pos_gts[best_gt_idxs[p_i].item()].canonical_category) if gt_tensor.shape[0] > 0 else ""

                sem_cat, sem_prob = p_cats_map.get(p_i, ("unclassified", 0.0))
                comb_score = round(q_score * sem_prob, 3)
                is_correct = biou >= 0.50 and is_compatible(sem_cat, gt_cat)

                scaled_quad = [[round(p[0] * sx, 1), round(p[1] * sy, 1)] for p in q_pts]

                proposals_list.append({
                    "rank": p_i + 1,
                    "quad": scaled_quad,
                    "proposal_score": round(q_score, 3),
                    "best_iou": biou,
                    "matched_gt_category": gt_cat,
                    "semantic_category": sem_cat,
                    "semantic_prob": round(sem_prob, 3),
                    "combined_score": comb_score,
                    "is_correct_match": is_correct,
                    "is_localized_050": biou >= 0.50,
                })
            return proposals_list

        quad_proposals = process_model_proposals(q_source_quads, q_scores, max_classify=30)
        hbb_proposals = process_model_proposals(h_source_quads, h_scores, max_classify=30)

        result_records.append({
            "image_id": img_rec.image_id,
            "source_dataset": img_rec.source_dataset,
            "file_name": img_rec.file_name,
            "original_width": orig_w,
            "original_height": orig_h,
            "preview_width": pw,
            "preview_height": ph,
            "image_b64": preview_b64,
            "ground_truth": scaled_gt,
            "models": {
                "quad": {
                    "label": "Phase 3 Cloud Quad EMA (Polygon NMS 0.78)",
                    "geometry": "quad",
                    "proposals": quad_proposals,
                },
                "hbb": {
                    "label": "Phase 4 Pilot v3 BBox EMA (Axis-Aligned NMS 0.70)",
                    "geometry": "bbox",
                    "proposals": hbb_proposals,
                },
            },
        })

    return result_records


def generate_html(report_data: dict[str, Any], gallery_data: list[dict[str, Any]], image_records: list[dict[str, Any]]) -> str:
    """Generate self-contained HTML visual audit document."""
    audit_payload = {
        "report": report_data,
        "gallery": gallery_data,
        "images": image_records,
    }
    json_data = json.dumps(audit_payload, separators=(",", ":")).replace("</", "<\\/")

    html_template = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Phase 4.0: Detector + SigLIP Teacher Baselines Visual Audit</title>
  <style>
    :root {{
      --bg: #090e17;
      --card-bg: #111a28;
      --card-border: #1e2d42;
      --text: #f0f6fc;
      --text-muted: #8b9bb4;
      --accent: #38bdf8;
      --accent-subtle: #0369a133;
      --ready: #22c55e;
      --blocked: #ef4444;
      --scarce: #f59e0b;
      --unsupported: #64748b;
      --quad-color: #38bdf8;
      --hbb-color: #ec4899;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }}
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{ background: var(--bg); color: var(--text); padding-bottom: 60px; line-height: 1.5; }}
    header {{
      background: linear-gradient(180deg, #0f172a 0%, var(--bg) 100%);
      padding: 32px 40px 24px;
      border-bottom: 1px solid var(--card-border);
    }}
    .eyebrow {{ color: var(--accent); font-size: 0.8rem; font-weight: 700; text-transform: uppercase; letter-spacing: 0.1em; }}
    h1 {{ font-size: 2.2rem; font-weight: 800; margin: 6px 0 8px; letter-spacing: -0.03em; }}
    .subtitle {{ color: var(--text-muted); font-size: 1rem; max-width: 950px; }}
    .kpi-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 16px;
      margin-top: 24px;
    }}
    .kpi-card {{
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 16px;
    }}
    .kpi-label {{ font-size: 0.75rem; text-transform: uppercase; color: var(--text-muted); font-weight: 600; letter-spacing: 0.05em; }}
    .kpi-value {{ font-size: 1.8rem; font-weight: 800; color: var(--text); margin-top: 4px; }}
    .kpi-sub {{ font-size: 0.8rem; color: var(--text-muted); margin-top: 2px; }}

    nav.tab-nav {{
      display: flex;
      gap: 12px;
      padding: 16px 40px;
      background: #0d1522;
      border-bottom: 1px solid var(--card-border);
      position: sticky;
      top: 0;
      z-index: 50;
      backdrop-filter: blur(8px);
    }}
    .tab-btn {{
      background: transparent;
      border: 1px solid transparent;
      color: var(--text-muted);
      padding: 8px 18px;
      border-radius: 8px;
      font-size: 0.9rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.15s ease;
    }}
    .tab-btn:hover {{ color: var(--text); background: #162338; }}
    .tab-btn.active {{ color: var(--text); background: var(--card-bg); border-color: var(--accent); }}

    main {{ padding: 24px 40px; }}
    .tab-pane {{ display: none; }}
    .tab-pane.active {{ display: block; }}

    /* Table styles */
    .table-container {{
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      overflow-x: auto;
      margin-top: 16px;
    }}
    table {{ width: 100%; border-collapse: collapse; text-align: left; font-size: 0.9rem; }}
    th, td {{ padding: 12px 18px; border-bottom: 1px solid var(--card-border); }}
    th {{ background: #0c1420; color: var(--text-muted); font-size: 0.78rem; text-transform: uppercase; letter-spacing: 0.05em; }}
    tr:last-child td {{ border-bottom: none; }}
    tr:hover td {{ background: #132032; }}

    .badge {{
      display: inline-block;
      padding: 3px 10px;
      border-radius: 999px;
      font-size: 0.75rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    .badge-ready {{ background: #22c55e26; color: #4ade80; border: 1px solid #22c55e55; }}
    .badge-blocked {{ background: #ef444426; color: #f87171; border: 1px solid #ef444455; }}
    .badge-scarce {{ background: #f59e0b26; color: #fbbf24; border: 1px solid #f59e0b55; }}
    .badge-unsupported {{ background: #64748b26; color: #94a3b8; border: 1px solid #64748b55; }}

    /* Switcher toolbar */
    .control-toolbar {{
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
      align-items: center;
      margin-bottom: 20px;
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      padding: 14px 20px;
      border-radius: 12px;
    }}
    .control-group {{
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 0.85rem;
      font-weight: 600;
    }}
    .switch-pill-container {{
      display: flex;
      background: #090e17;
      border: 1px solid var(--card-border);
      border-radius: 8px;
      overflow: hidden;
    }}
    .switch-pill {{
      padding: 6px 14px;
      font-size: 0.82rem;
      font-weight: 700;
      cursor: pointer;
      background: transparent;
      color: var(--text-muted);
      border: none;
      transition: all 0.15s ease;
    }}
    .switch-pill.active {{
      background: #1e324d;
      color: var(--accent);
    }}
    .switch-pill.active.pill-hbb {{
      background: #3b1d32;
      color: var(--hbb-color);
    }}

    select, input[type="range"] {{
      background: #090e17;
      color: var(--text);
      border: 1px solid var(--card-border);
      border-radius: 6px;
      padding: 6px 10px;
      font-size: 0.85rem;
    }}

    /* Gallery Grid */
    .gallery-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
      gap: 16px;
    }}
    .crop-card {{
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 10px;
      overflow: hidden;
      display: flex;
      flex-direction: column;
    }}
    .crop-card img {{
      width: 100%;
      aspect-ratio: 1;
      object-fit: cover;
      background: #000;
    }}
    .crop-info {{
      padding: 12px;
      font-size: 0.82rem;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }}
    .crop-title {{ font-weight: 700; display: flex; justify-content: space-between; align-items: center; }}
    .pred-row {{ color: var(--text-muted); font-size: 0.78rem; display: flex; justify-content: space-between; }}

    /* Canvas visualizer */
    .image-browser-layout {{
      display: grid;
      grid-template-columns: 320px 1fr;
      gap: 20px;
    }}
    .image-list {{
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      max-height: 800px;
      overflow-y: auto;
      padding: 8px;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }}
    .image-list-item {{
      padding: 10px 14px;
      border-radius: 8px;
      cursor: pointer;
      font-size: 0.82rem;
      border: 1px solid transparent;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }}
    .image-list-item:hover {{ background: #18263a; }}
    .image-list-item.selected {{ background: #1e324d; border-color: var(--accent); }}

    .canvas-panel {{
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 20px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }}
    .panels-container {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
    }}
    .view-card {{
      background: #090e17;
      border: 1px solid var(--card-border);
      border-radius: 10px;
      padding: 12px;
    }}
    .view-card h3 {{ font-size: 0.9rem; margin-bottom: 8px; color: var(--text-muted); display: flex; justify-content: space-between; }}
    canvas {{
      width: 100%;
      aspect-ratio: 1;
      display: block;
      background: #000;
      border-radius: 6px;
    }}
    .legend-row {{
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
      font-size: 0.8rem;
      color: var(--text-muted);
      margin-top: 8px;
    }}
    .legend-item {{ display: flex; align-items: center; gap: 6px; }}
    .legend-color {{ width: 12px; height: 12px; border-radius: 3px; display: inline-block; }}
  </style>
</head>
<body>

  <header>
    <div class="eyebrow">Phase 4.0 Semantic Distillation Baseline &middot; Visual Audit</div>
    <h1>Detector + SigLIP Teacher Baselines</h1>
    <p class="subtitle">
      Inspect all class-agnostic proposals from both the <strong>Quad detector</strong> and the <strong>BBox detector</strong>, alongside SigLIP open-vocabulary classifications and GT crop teacher recognition.
    </p>

    <div class="kpi-grid">
      <div class="kpi-card">
        <div class="kpi-label">Conditional Recognition</div>
        <div class="kpi-value" style="color: #4ade80;">{report_data['actual_proposals_baseline']['conditional_recognition_accuracy']:.1%}</div>
        <div class="kpi-sub">On localized proposals (IoU &ge; 0.50)</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">Switchable Models</div>
        <div class="kpi-value" style="font-size: 1.4rem; padding-top: 6px;">Quad vs BBox</div>
        <div class="kpi-sub">Polygon DQCO vs HBB Reg2</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">Winning Crop Policy</div>
        <div class="kpi-value" style="font-size: 1.4rem; padding-top: 6px;">Contextual 15%</div>
        <div class="kpi-sub">31.85% Top-1 vs 25.80% tight</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">Ready Novel Class</div>
        <div class="kpi-value" style="color: #4ade80;">Dog</div>
        <div class="kpi-sub">87.5% Top-1 &middot; 81.2% Recall &middot; 57.8% AP</div>
      </div>
    </div>
  </header>

  <nav class="tab-nav">
    <button class="tab-btn active" data-tab="proposals">Proposal & Detection Visualizer</button>
    <button class="tab-btn" data-tab="readiness">Readiness Decision Gate</button>
    <button class="tab-btn" data-tab="gallery">GT Crop Recognition Gallery</button>
  </nav>

  <main>
    <!-- TAB 1: PROPOSALS VISUALIZER (ACTIVE DEFAULT) -->
    <div class="tab-pane active" id="tab-proposals">
      <div class="control-toolbar">
        <!-- Model Switcher -->
        <div class="control-group">
          <span>Model:</span>
          <div class="switch-pill-container" id="model-pill-group">
            <button class="switch-pill active" data-model="quad">Quad Detector (Polygon)</button>
            <button class="switch-pill pill-hbb" data-model="hbb">BBox Detector (HBB)</button>
          </div>
        </div>

        <!-- Mode Switcher -->
        <div class="control-group">
          <span>View Mode:</span>
          <div class="switch-pill-container" id="mode-pill-group">
            <button class="switch-pill" data-mode="all_proposals">All Proposals (Class-Agnostic)</button>
            <button class="switch-pill active" data-mode="semantic">Classified Detections (SigLIP)</button>
          </div>
        </div>

        <!-- Sliders and controls -->
        <div class="control-group">
          <span id="score-slider-label">Min Quality:</span>
          <input type="range" id="score-slider" min="0.0" max="0.5" step="0.02" value="0.10">
          <span id="score-val" style="color: var(--accent); font-weight: 700;">0.10</span>
        </div>

        <div class="control-group">
          <span>Top K:</span>
          <input type="range" id="topk-slider" min="1" max="100" step="1" value="25">
          <span id="topk-val" style="color: var(--accent); font-weight: 700;">25</span>
        </div>

        <div class="control-group" id="category-filter-group" style="display: none;">
          <span>Category:</span>
          <select id="proposal-cat-filter">
            <option value="all">All Classes</option>
          </select>
        </div>

        <div class="control-group" style="margin-left: auto;">
          <label style="display: flex; align-items: center; gap: 6px; cursor: pointer;">
            <input type="checkbox" id="show-labels-check" checked> Show Labels
          </label>
        </div>
      </div>

      <div class="image-browser-layout">
        <div style="display: flex; flex-direction: column; gap: 10px;">
          <div style="background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 10px; padding: 10px 12px; display: flex; flex-direction: column; gap: 6px;">
            <div style="display: flex; justify-content: space-between; align-items: center;">
              <span style="font-weight: 700; font-size: 0.85rem;">Cohort Images</span>
              <span id="image-count-badge" style="font-size: 0.75rem; color: var(--accent); font-weight: 600;"></span>
            </div>
            <input type="text" id="image-search-input" placeholder="Search ID, dataset (coco, bdd, woodscape)..." style="width: 100%; box-sizing: border-box; background: var(--bg); border: 1px solid var(--card-border); color: var(--text-main); padding: 6px 10px; border-radius: 6px; font-size: 0.8rem;">
          </div>
          <div class="image-list" id="image-list-container" style="max-height: 720px;"></div>
        </div>
        <div class="canvas-panel">
          <div style="display: flex; justify-content: space-between; align-items: center;">
            <div id="selected-image-title" style="font-weight: 700; font-size: 1.05rem;"></div>
            <div id="selected-image-meta" style="color: var(--text-muted); font-size: 0.85rem;"></div>
          </div>

          <div class="panels-container">
            <div class="view-card">
              <h3><span>Ground Truth Objects</span><span id="gt-count-badge" style="color: var(--ready);"></span></h3>
              <canvas id="gt-canvas"></canvas>
            </div>
            <div class="view-card">
              <h3><span id="prop-view-title">Detector Proposals</span><span id="prop-count-badge" style="color: var(--accent);"></span></h3>
              <canvas id="prop-canvas"></canvas>
            </div>
          </div>

          <div class="legend-row" id="legend-all-proposals">
            <div class="legend-item"><span class="legend-color" style="background: #10b981;"></span> Ground Truth (Positive)</div>
            <div class="legend-item"><span class="legend-color" style="background: #22c55e;"></span> High Overlap (IoU &ge; 0.50)</div>
            <div class="legend-item"><span class="legend-color" style="background: #f59e0b;"></span> Partial Overlap (0.10 &le; IoU &lt; 0.50)</div>
            <div class="legend-item"><span class="legend-color" style="background: #38bdf8;"></span> Unmatched Proposal (IoU &lt; 0.10)</div>
          </div>

          <div class="legend-row" id="legend-semantic" style="display: none;">
            <div class="legend-item"><span class="legend-color" style="background: #10b981;"></span> Ground Truth Box</div>
            <div class="legend-item"><span class="legend-color" style="background: #22c55e;"></span> Correct Match (IoU &ge; 0.50 &amp; Class Match)</div>
            <div class="legend-item"><span class="legend-color" style="background: #f59e0b;"></span> Localized but Confused (IoU &ge; 0.50 &amp; Class Mismatch)</div>
            <div class="legend-item"><span class="legend-color" style="background: #38bdf8;"></span> Unmatched / Background Proposal</div>
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 2: READINESS -->
    <div class="tab-pane" id="tab-readiness">
      <h2 style="font-size: 1.3rem; margin-bottom: 6px;">Per-Class Readiness Decision Matrix</h2>
      <p style="color: var(--text-muted); font-size: 0.88rem; margin-bottom: 12px;">
        Evaluates whether each concept meets the quantitative teacher recognition and proposal coverage criteria required for distillation.
      </p>

      <div class="table-container">
        <table>
          <thead>
            <tr>
              <th>Category</th>
              <th>Type</th>
              <th>Instances</th>
              <th>Teacher Top-1</th>
              <th>Proposal Recall @0.50</th>
              <th>End-to-End AP50</th>
              <th>Gate Decision</th>
              <th>Detailed Reason</th>
            </tr>
          </thead>
          <tbody id="readiness-table-body"></tbody>
        </table>
      </div>

      <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-top: 24px;">
        <div class="table-container" style="margin-top: 0; padding: 18px;">
          <h3 style="font-size: 1.05rem; margin-bottom: 12px;">Breakdown by Object Size Tier</h3>
          <table>
            <thead>
              <tr>
                <th>Size Tier</th>
                <th>Instances</th>
                <th>Proposal Recall @0.50</th>
                <th>Teacher Crop Top-1</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td><strong>Tiny</strong> (&lt;16px)</td>
                <td>469</td>
                <td>0.0%</td>
                <td>1.1%</td>
              </tr>
              <tr>
                <td><strong>Small</strong> (16–32px)</td>
                <td>1,098</td>
                <td>0.5%</td>
                <td>8.6%</td>
              </tr>
              <tr>
                <td><strong>Medium</strong> (32–96px)</td>
                <td>1,552</td>
                <td>22.9%</td>
                <td>40.3%</td>
              </tr>
              <tr>
                <td><strong>Large</strong> (&ge;96px)</td>
                <td>653</td>
                <td><strong>88.5%</strong></td>
                <td><strong>72.9%</strong></td>
              </tr>
            </tbody>
          </table>
        </div>

        <div class="table-container" style="margin-top: 0; padding: 18px;">
          <h3 style="font-size: 1.05rem; margin-bottom: 12px;">Crop Policy Benchmark</h3>
          <table>
            <thead>
              <tr>
                <th>Policy</th>
                <th>Mode</th>
                <th>Context Margin</th>
                <th>Evaluated Crops</th>
                <th>Overall Top-1</th>
              </tr>
            </thead>
            <tbody>
              <tr>
                <td><code>tight_source</code></td>
                <td>Axis-Aligned</td>
                <td>0.0</td>
                <td>3,771</td>
                <td>25.80%</td>
              </tr>
              <tr style="background: #0369a122;">
                <td><strong><code>contextual_source</code></strong> (Winner)</td>
                <td>Axis-Aligned</td>
                <td><strong>0.15</strong></td>
                <td>3,771</td>
                <td><strong>31.85%</strong></td>
              </tr>
              <tr>
                <td><code>contextual_resized</code></td>
                <td>Axis-Aligned</td>
                <td>0.15</td>
                <td>3,771</td>
                <td>31.85%</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>

    <!-- TAB 3: CROP RECOGNITION GALLERY -->
    <div class="tab-pane" id="tab-gallery">
      <div class="control-toolbar">
        <label style="font-size: 0.85rem; font-weight: 600;">Category:
          <select id="gallery-cat-filter">
            <option value="all">All Categories</option>
          </select>
        </label>
        <label style="font-size: 0.85rem; font-weight: 600;">Outcome:
          <select id="gallery-outcome-filter">
            <option value="all">All Outcomes</option>
            <option value="correct">Correct Match</option>
            <option value="incorrect">Incorrect / Confused</option>
          </select>
        </label>
        <span id="gallery-count" style="margin-left: auto; color: var(--text-muted); font-size: 0.85rem;"></span>
      </div>

      <div class="gallery-grid" id="crop-gallery-container"></div>
    </div>
  </main>

  <script id="audit-data" type="application/json">{json_data}</script>
  <script>
    const DATA = JSON.parse(document.getElementById('audit-data').textContent);
    const report = DATA.report;
    const gallery = DATA.gallery;
    const images = DATA.images;

    // Tab navigation
    document.querySelectorAll('.tab-btn').forEach(btn => {{
      btn.addEventListener('click', () => {{
        document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
        btn.classList.add('active');
        document.getElementById('tab-' + btn.dataset.tab).classList.add('active');
      }});
    }});

    // Populate Readiness Table
    const tbody = document.getElementById('readiness-table-body');
    Object.entries(report.readiness_decisions).forEach(([cat, dec]) => {{
      const tr = document.createElement('tr');
      const badgeClass = 'badge-' + dec.status.toLowerCase();
      tr.innerHTML = `
        <td><strong><code>${{cat}}</code></strong></td>
        <td><span style="color: var(--text-muted); font-size: 0.8rem; text-transform: uppercase;">${{dec.type}}</span></td>
        <td>${{dec.gt_instances}}</td>
        <td><strong>${{(dec.teacher_top1_accuracy * 100).toFixed(1)}}%</strong></td>
        <td>${{(dec.proposal_recall_iou050 * 100).toFixed(1)}}%</td>
        <td>${{(dec.end_to_end_ap50 * 100).toFixed(1)}}%</td>
        <td><span class="badge ${{badgeClass}}">${{dec.status}}</span></td>
        <td style="color: var(--text-muted); font-size: 0.82rem;">${{dec.reason}}</td>
      `;
      tbody.appendChild(tr);
    }});

    // Gallery Logic
    const catSelect = document.getElementById('gallery-cat-filter');
    const outcomeSelect = document.getElementById('gallery-outcome-filter');
    const galleryContainer = document.getElementById('crop-gallery-container');
    const galleryCount = document.getElementById('gallery-count');

    const uniqueCats = Array.from(new Set(gallery.map(g => g.gt_category))).sort();
    uniqueCats.forEach(c => {{
      const opt = document.createElement('option');
      opt.value = c;
      opt.textContent = c;
      catSelect.appendChild(opt);
    }});

    function renderGallery() {{
      const catFilter = catSelect.value;
      const outcomeFilter = outcomeSelect.value;

      const filtered = gallery.filter(item => {{
        if (catFilter !== 'all' && item.gt_category !== catFilter) return false;
        if (outcomeFilter === 'correct' && !item.is_correct) return false;
        if (outcomeFilter === 'incorrect' && item.is_correct) return false;
        return true;
      }});

      galleryCount.textContent = `Showing ${{filtered.length}} of ${{gallery.length}} crops`;
      galleryContainer.innerHTML = '';

      filtered.forEach(crop => {{
        const card = document.createElement('div');
        card.className = 'crop-card';
        const badge = crop.is_correct
          ? '<span class="badge badge-ready">Match</span>'
          : '<span class="badge badge-blocked">Fail</span>';

        const top3Html = crop.top3.map(t =>
          `<div class="pred-row"><span>${{t.category}}</span><span>${{(t.probability * 100).toFixed(0)}}%</span></div>`
        ).join('');

        card.innerHTML = `
          <img src="${{crop.image_b64}}" loading="lazy" alt="${{crop.gt_category}}">
          <div class="crop-info">
            <div class="crop-title">
              <span><code>${{crop.gt_category}}</code></span>
              ${{badge}}
            </div>
            <div style="font-size: 0.75rem; color: var(--text-muted);">Size: ${{crop.size_bin}} &middot; Img #${{crop.image_id}}</div>
            <div style="margin-top: 4px; padding-top: 4px; border-top: 1px solid var(--card-border);">
              <div style="font-size: 0.72rem; color: var(--text-muted); margin-bottom: 2px;">TOP-3 PREDICTIONS:</div>
              ${{top3Html}}
            </div>
          </div>
        `;
        galleryContainer.appendChild(card);
      }});
    }}

    catSelect.addEventListener('change', renderGallery);
    outcomeSelect.addEventListener('change', renderGallery);
    renderGallery();

    // Populate Category Filter for Proposals
    const propCatFilter = document.getElementById('proposal-cat-filter');
    uniqueCats.forEach(c => {{
      const opt = document.createElement('option');
      opt.value = c;
      opt.textContent = c;
      propCatFilter.appendChild(opt);
    }});

    // Interactive Switchable Proposals Canvas Browser
    let currentImageIndex = 0;
    let selectedModel = 'quad'; // 'quad' or 'hbb'
    let selectedMode = 'semantic'; // 'all_proposals' or 'semantic'

    const imageListContainer = document.getElementById('image-list-container');
    const gtCanvas = document.getElementById('gt-canvas');
    const propCanvas = document.getElementById('prop-canvas');
    const scoreSlider = document.getElementById('score-slider');
    const topkSlider = document.getElementById('topk-slider');
    const showLabelsCheck = document.getElementById('show-labels-check');
    const scoreVal = document.getElementById('score-val');
    const topkVal = document.getElementById('topk-val');
    const scoreSliderLabel = document.getElementById('score-slider-label');
    const catFilterGroup = document.getElementById('category-filter-group');
    const legendAll = document.getElementById('legend-all-proposals');
    const legendSem = document.getElementById('legend-semantic');
    const propViewTitle = document.getElementById('prop-view-title');

    // Model Pill Buttons
    document.querySelectorAll('#model-pill-group .switch-pill').forEach(pill => {{
      pill.addEventListener('click', () => {{
        document.querySelectorAll('#model-pill-group .switch-pill').forEach(p => p.classList.remove('active'));
        pill.classList.add('active');
        selectedModel = pill.dataset.model;
        updateViewConfig();
        renderImagePanels();
      }});
    }});

    // Mode Pill Buttons
    document.querySelectorAll('#mode-pill-group .switch-pill').forEach(pill => {{
      pill.addEventListener('click', () => {{
        document.querySelectorAll('#mode-pill-group .switch-pill').forEach(p => p.classList.remove('active'));
        pill.classList.add('active');
        selectedMode = pill.dataset.mode;
        updateViewConfig();
        renderImagePanels();
      }});
    }});

    function updateViewConfig() {{
      if (selectedMode === 'all_proposals') {{
        scoreSliderLabel.textContent = 'Min Quality:';
        scoreSlider.max = "0.8";
        scoreSlider.step = "0.02";
        if (parseFloat(scoreSlider.value) > 0.8) scoreSlider.value = "0.10";
        topkSlider.max = "100";
        catFilterGroup.style.display = 'none';
        legendAll.style.display = 'flex';
        legendSem.style.display = 'none';
        propViewTitle.textContent = selectedModel === 'quad'
          ? 'Quad Detector Proposals (All)'
          : 'BBox Detector Proposals (All)';
      }} else {{
        scoreSliderLabel.textContent = 'Min Combined:';
        scoreSlider.max = "0.5";
        scoreSlider.step = "0.01";
        topkSlider.max = "50";
        if (parseInt(topkSlider.value) > 50) topkSlider.value = "25";
        catFilterGroup.style.display = 'flex';
        legendAll.style.display = 'none';
        legendSem.style.display = 'flex';
        propViewTitle.textContent = selectedModel === 'quad'
          ? 'Quad + SigLIP Detections'
          : 'BBox + SigLIP Detections';
      }}
      scoreVal.textContent = parseFloat(scoreSlider.value).toFixed(2);
      topkVal.textContent = topkSlider.value;
    }}

    // Filterable Image List
    const imageSearchInput = document.getElementById('image-search-input');
    const imageCountBadge = document.getElementById('image-count-badge');

    function renderImageList(query = '') {{
      imageListContainer.innerHTML = '';
      const q = query.trim().toLowerCase();
      let matchedCount = 0;

      images.forEach((img, idx) => {{
        const matchesQuery = !q || 
          img.image_id.toString().includes(q) || 
          img.source_dataset.toLowerCase().includes(q) ||
          img.file_name.toLowerCase().includes(q);

        if (!matchesQuery) return;
        matchedCount++;

        const item = document.createElement('div');
        item.className = 'image-list-item' + (idx === currentImageIndex ? ' selected' : '');
        item.dataset.index = idx;
        const qCount = img.models.quad ? img.models.quad.proposals.length : 0;
        item.innerHTML = `
          <div>
            <div style="font-weight: 700;">#${{img.image_id}}</div>
            <div style="color: var(--text-muted); font-size: 0.75rem;">${{img.source_dataset}}</div>
          </div>
          <div style="text-align: right; font-size: 0.75rem; color: var(--text-muted);">
            <span>${{img.ground_truth.length}} GT</span> &middot; <span>${{qCount}} Props</span>
          </div>
        `;
        item.addEventListener('click', () => {{
          document.querySelectorAll('.image-list-item').forEach(i => i.classList.remove('selected'));
          item.classList.add('selected');
          currentImageIndex = idx;
          renderImagePanels();
        }});
        imageListContainer.appendChild(item);
      }});

      imageCountBadge.textContent = `${{matchedCount}} / ${{images.length}}`;
    }}

    imageSearchInput.addEventListener('input', (e) => {{
      renderImageList(e.target.value);
    }});

    renderImageList();

    function drawQuad(ctx, pts, strokeColor, fillColor, lineWidth = 2) {{
      if (!pts || pts.length < 4) return;
      ctx.beginPath();
      ctx.moveTo(pts[0][0], pts[0][1]);
      for (let i = 1; i < pts.length; i++) {{
        ctx.lineTo(pts[i][0], pts[i][1]);
      }}
      ctx.closePath();
      if (fillColor) {{
        ctx.fillStyle = fillColor;
        ctx.fill();
      }}
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = lineWidth;
      ctx.stroke();
    }}

    function renderImagePanels() {{
      const record = images[currentImageIndex];
      if (!record) return;

      const modelData = record.models[selectedModel];
      const modelLabel = modelData ? modelData.label : selectedModel;

      document.getElementById('selected-image-title').textContent = `Image #${{record.image_id}} &middot; ${{modelLabel}}`;
      document.getElementById('selected-image-meta').textContent = `${{record.source_dataset}} &middot; ${{record.original_width}}x${{record.original_height}}`;

      const minScore = parseFloat(scoreSlider.value);
      const topK = parseInt(topkSlider.value);
      const showLabels = showLabelsCheck.checked;
      const catFilter = propCatFilter.value;

      const img = new Image();
      img.onload = () => {{
        const w = record.preview_width;
        const h = record.preview_height;

        gtCanvas.width = w;
        gtCanvas.height = h;
        propCanvas.width = w;
        propCanvas.height = h;

        const gtCtx = gtCanvas.getContext('2d');
        const propCtx = propCanvas.getContext('2d');

        gtCtx.drawImage(img, 0, 0, w, h);
        propCtx.drawImage(img, 0, 0, w, h);

        // Draw Ground Truth
        document.getElementById('gt-count-badge').textContent = `${{record.ground_truth.length}} GT Objects`;
        record.ground_truth.forEach(gt => {{
          drawQuad(gtCtx, gt.quad, '#10b981', '#10b98122', 2.5);
          if (showLabels) {{
            const minX = Math.min(...gt.quad.map(p => p[0]));
            const minY = Math.min(...gt.quad.map(p => p[1]));
            gtCtx.font = '11px system-ui';
            const text = gt.category;
            const tw = gtCtx.measureText(text).width + 6;
            gtCtx.fillStyle = '#090e17cc';
            gtCtx.fillRect(minX, Math.max(0, minY - 16), tw, 16);
            gtCtx.fillStyle = '#4ade80';
            gtCtx.fillText(text, minX + 3, Math.max(12, minY - 4));
          }}
        }});

        // Filter proposals based on mode
        const allProps = modelData ? modelData.proposals : [];
        let filteredProps = [];

        if (selectedMode === 'all_proposals') {{
          filteredProps = allProps
            .filter(p => p.proposal_score >= minScore)
            .slice(0, topK);
        }} else {{
          filteredProps = allProps
            .filter(p => {{
              if (p.combined_score < minScore) return false;
              if (catFilter !== 'all' && p.semantic_category !== catFilter) return false;
              return true;
            }})
            .slice(0, topK);
        }}

        document.getElementById('prop-count-badge').textContent = `${{filteredProps.length}} Shown`;

        // Render proposals on propCanvas
        filteredProps.forEach(p => {{
          let color = '#38bdf8';
          let fill = '#38bdf811';

          if (selectedMode === 'all_proposals') {{
            if (p.best_iou >= 0.50) {{
              color = '#22c55e';
              fill = '#22c55e22';
            }} else if (p.best_iou >= 0.10) {{
              color = '#f59e0b';
              fill = '#f59e0b22';
            }} else {{
              color = selectedModel === 'quad' ? '#38bdf8' : '#ec4899';
              fill = color + '11';
            }}
          }} else {{
            // Semantic mode
            if (p.is_correct_match) {{
              color = '#22c55e';
              fill = '#22c55e22';
            }} else if (p.is_localized_050) {{
              color = '#f59e0b';
              fill = '#f59e0b22';
            }} else {{
              color = '#38bdf8';
              fill = '#38bdf811';
            }}
          }}

          drawQuad(propCtx, p.quad, color, fill, 2.0);

          if (showLabels) {{
            const minX = Math.min(...p.quad.map(pt => pt[0]));
            const minY = Math.min(...p.quad.map(pt => pt[1]));
            propCtx.font = '10px ui-monospace, monospace';

            let labelText = '';
            if (selectedMode === 'all_proposals') {{
              labelText = `#${{p.rank}} q:${{p.proposal_score.toFixed(2)}} iou:${{p.best_iou.toFixed(2)}}`;
              if (p.matched_gt_category) labelText += ` (${{p.matched_gt_category}})`;
            }} else {{
              labelText = `#${{p.rank}} ${{p.semantic_category}} (${{p.combined_score.toFixed(2)}})`;
            }}

            const tw = propCtx.measureText(labelText).width + 6;
            propCtx.fillStyle = '#090e17ee';
            propCtx.fillRect(minX, Math.max(0, minY - 15), tw, 15);
            propCtx.fillStyle = color;
            propCtx.fillText(labelText, minX + 3, Math.max(11, minY - 4));
          }}
        }});
      }};
      img.src = record.image_b64;
    }}

    scoreSlider.addEventListener('input', () => {{
      scoreVal.textContent = parseFloat(scoreSlider.value).toFixed(2);
      renderImagePanels();
    }});
    topkSlider.addEventListener('input', () => {{
      topkVal.textContent = topkSlider.value;
      renderImagePanels();
    }});
    showLabelsCheck.addEventListener('change', renderImagePanels);
    propCatFilter.addEventListener('change', renderImagePanels);

    updateViewConfig();
    renderImagePanels();
  </script>
</body>
</html>"""
    return html_template


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, default=Path("artifacts/phase4/baselines/phase4_0_teacher_baseline_report.json"))
    parser.add_argument("--dataset-root", type=Path, default=Path("/home/sashamikoff/git/skillup/data/visual-inference-datasets/output"))
    parser.add_argument("--hbb-checkpoint", type=Path, default=Path("artifacts/phase4/local_hbb_p3_seed42_pilot_v3_correct_dataset/best_ema.pt"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/phase4/baselines/phase4_0_visual_audit.html"))
    parser.add_argument("--max-images", type=int, default=100)
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    report_path = _resolve_path(args.report)
    dataset_root = _resolve_path(args.dataset_root)
    output_path = _resolve_path(args.output)
    hbb_ckpt_path = _resolve_path(args.hbb_checkpoint)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    report_data = json.loads(report_path.read_text(encoding="utf-8"))
    contract = report_data["contract"]

    device = torch.device(args.device if (args.device == "cuda" and torch.cuda.is_available()) else "cpu")
    print(f"Building Phase 4.0 dual-model visual audit using device: {device}")

    # Load teacher
    teacher = SiglipTeacher(
        model_name=contract["teacher"]["model_name"],
        revision=contract["teacher"]["revision"],
        device=device,
    )
    categories = list(contract["vocabulary"]["base_classes"]) + list(contract["vocabulary"]["novel_classes"]) + list(contract["vocabulary"]["distractors"])
    text_bank, _ = teacher.build_text_bank(categories, templates=contract["teacher"]["default_prompts"])

    # Load Quad detector
    quad_detector = QuadProposalDetector(pretrained_backbone=False, neck_type="lite")
    primary_ckpt = _resolve_path(contract["detector"]["primary"]["checkpoint"])
    ckpt = load_checkpoint(primary_ckpt)
    load_model_state_strict(quad_detector, ckpt, kind="quad", neck_type="lite", state_key=contract["detector"]["primary"]["checkpoint_state"])
    quad_detector.to(device).eval()
    print("Loaded Quad detector (Phase 3 Cloud Quad EMA)")

    # Load HBB detector
    hbb_detector = StudentDetector(pretrained_backbone=False, neck_type="lite")
    hbb_ckpt = load_checkpoint(hbb_ckpt_path)
    load_model_state_strict(hbb_detector, hbb_ckpt, kind="hbb", neck_type="lite", state_key="ema_model")
    hbb_detector.to(device).eval()
    print("Loaded HBB detector (Phase 4 Pilot v3 HBB EMA)")

    winning_policy = CropPolicy(
        mode=contract["crop_policies"]["contextual_source"]["mode"],
        context_margin=contract["crop_policies"]["contextual_source"]["context_margin"],
        use_original_resolution=contract["crop_policies"]["contextual_source"]["use_original_resolution"],
    )

    # Load dataset to extract the frozen 800 cohort
    config = load_phase3_config(_resolve_path("configs/correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml"))
    dataset = QuadProposalDataset(
        dataset_root / "annotations/proposals_val.json",
        dataset_root,
        dataset_root / "indexes/quad_val.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=False,
    )
    frozen_800_indices = select_source_mixture_indices(
        dataset.records, config.data.source_weights, 800
    )
    frozen_800_ids = [dataset.records[i].image_id for i in frozen_800_indices]

    # Select stratified rare-class images
    target_cats = list(contract["vocabulary"]["base_classes"]) + list(contract["vocabulary"]["novel_classes"])
    stratified_ids = select_stratified_semantic_cohort(
        dataset_root,
        target_categories=target_cats,
        quota_per_category=10,
        seed=42,
    )

    # Combine: rare classes first, then full 800 frozen cohort
    all_candidate_ids = list(dict.fromkeys(stratified_ids + frozen_800_ids))
    print(f"Total available evaluation cohort candidate images: {len(all_candidate_ids)} (including full 800 frozen cohort)")

    semantic_records = load_semantic_image_records(dataset_root, all_candidate_ids[:args.max_images])

    # 1. Build Crop Gallery
    gallery = build_crop_gallery(
        image_records=semantic_records,
        teacher=teacher,
        text_bank=text_bank,
        categories=categories,
        policy=winning_policy,
        max_crops_per_cat=8,
    )

    # 2. Build Dual-Model Image Records
    image_records = build_dual_model_image_records(
        selected_image_ids=all_candidate_ids,
        dataset_root=dataset_root,
        quad_detector=quad_detector,
        hbb_detector=hbb_detector,
        teacher=teacher,
        text_bank=text_bank,
        categories=categories,
        policy=winning_policy,
        device=device,
        max_images=args.max_images,
    )

    # 3. Generate HTML
    html_content = generate_html(report_data, gallery, image_records)
    output_path.write_text(html_content, encoding="utf-8")
    print(f"Successfully generated dual-model visual audit dashboard: {output_path} ({len(html_content):,} bytes)")


if __name__ == "__main__":
    main()
