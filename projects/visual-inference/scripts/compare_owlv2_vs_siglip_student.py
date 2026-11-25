#!/usr/bin/env python3
"""Compare Old SigLIP-8k Opt-B HBB vs New OWLv2-12k Opt-B HBB (Frozen & Adapted).

Runs stratified 72-image inference (8 pinned core images + 64 unseen holdout images),
computes summary detection/latency stats, writes high-resolution 2x2 PNG panels for the
core diagnostic images, and builds a paginated interactive HTML viewer.
"""

from __future__ import annotations

import argparse
import base64
from collections import Counter, defaultdict
from html import escape
import io
import json
from pathlib import Path
from typing import Any

from PIL import Image
import torch

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

try:
    from scripts.diagnose_teacher_and_hygiene_poc import (
        DATASET_ORDER,
        _load_font,
        _render_quadrant,
        load_diagnostic_images,
        run_student_option_b_inference,
        select_poc_image_ids,
    )
except ModuleNotFoundError:
    from diagnose_teacher_and_hygiene_poc import (  # type: ignore[no-redef]
        DATASET_ORDER,
        _load_font,
        _render_quadrant,
        load_diagnostic_images,
        run_student_option_b_inference,
        select_poc_image_ids,
    )
from student_detector.human_gt_data import DEFAULT_ENRICHED_GT_INDEX_PATH
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT


def summarize_predictions(
    items: list[dict[str, Any]],
    preds_by_id: dict[int, list[dict[str, Any]]],
    *,
    score_thresh: float,
    latency_ms: float,
) -> dict[str, Any]:
    """Summarize detection counts and class diversity over the comparison set."""
    total_boxes = 0
    nonempty_images = 0
    cls_counts: Counter[str] = Counter()
    per_ds_boxes: dict[str, list[int]] = defaultdict(list)

    for item in items:
        img_id = int(item["image_id"])
        src = str(item["source_dataset"])
        kept = [p for p in preds_by_id.get(img_id, []) if float(p["score"]) >= score_thresh]
        total_boxes += len(kept)
        if kept:
            nonempty_images += 1
        per_ds_boxes[src].append(len(kept))
        for p in kept:
            cls_counts[str(p["label"])] += 1

    n_imgs = max(len(items), 1)
    return {
        "score_threshold": score_thresh,
        "latency_ms": latency_ms,
        "images": len(items),
        "nonempty_images": nonempty_images,
        "avg_boxes_per_image": round(total_boxes / n_imgs, 2),
        "unique_classes": len(cls_counts),
        "top_classes": dict(cls_counts.most_common(12)),
        "per_dataset_avg_boxes": {
            ds: round(sum(vals) / max(len(vals), 1), 2) for ds, vals in per_ds_boxes.items()
        },
    }


def render_comparison_panels(
    items: list[dict[str, Any]],
    old_siglip_preds: dict[int, list[dict[str, Any]]],
    owl_frozen_preds: dict[int, list[dict[str, Any]]],
    owl_adapted_preds: dict[int, list[dict[str, Any]]],
    out_dir: Path,
    *,
    siglip_thresh: float,
    owl_thresh: float,
    pane3_label: str = "New OWLv2-12k Frozen",
    pane4_label: str = "New OWLv2-12k Adapted",
) -> list[str]:
    """Render 2x2 PNG panels (Clean GT | Old SigLIP | Pane 3 | Pane 4)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    font_hdr = _load_font(16)
    font_tag = _load_font(13)
    quad_w = 960
    header_h = 34
    saved_paths: list[str] = []

    for item in items:
        if not item.get("is_core"):
            continue
        img_id = int(item["image_id"])
        st = item["hygiene_stats"]
        with Image.open(item["image_path"]) as im:
            rgb = im.convert("RGB")

        gt_draw: list[tuple[list[float], str, tuple[int, int, int], bool]] = []
        for g in item["gt_dropped"]:
            gt_draw.append((g["box"], f"DROP:{g['label']}", (248, 113, 113), True))
        for g in item["gt_demoted"]:
            gt_draw.append((g["box"], f"IGN:{g['raw_label'][:10]}", (251, 146, 60), True))
        for g in item["gt_kept"]:
            gt_draw.append((g["box"], g["label"], (74, 222, 128), False))

        old_raw = old_siglip_preds.get(img_id, [])
        frz_raw = owl_frozen_preds.get(img_id, [])
        adp_raw = owl_adapted_preds.get(img_id, [])

        old_list = [p for p in old_raw if float(p["score"]) >= siglip_thresh][:30]
        frz_list = [p for p in frz_raw if float(p["score"]) >= owl_thresh][:30]
        adp_list = [p for p in adp_raw if float(p["score"]) >= owl_thresh][:30]

        tl = _render_quadrant(
            rgb,
            f"1. Clean GT #{img_id} ({item['source_dataset']}) | Kept={st['kept_positives']}/{st['raw_positives']}",
            (22, 78, 52),
            gt_draw,
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )
        tr = _render_quadrant(
            rgb,
            f"2. Old SigLIP-8k Opt-B (joint>={siglip_thresh:.2f}, n={len(old_list)})",
            (30, 58, 138),
            [(p["box"], f"{p['label']} {int(p['score'] * 100)}%", (96, 165, 250), False) for p in old_list],
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )
        bl = _render_quadrant(
            rgb,
            f"3. {pane3_label} (joint>={owl_thresh:.2f}, n={len(frz_list)})",
            (14, 116, 144),
            [(p["box"], f"{p['label']} {int(p['score'] * 100)}%", (34, 211, 238), False) for p in frz_list],
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )
        br = _render_quadrant(
            rgb,
            f"4. {pane4_label} (joint>={owl_thresh:.2f}, n={len(adp_list)})",
            (107, 33, 168),
            [(p["box"], f"{p['label']} {int(p['score'] * 100)}%", (232, 121, 249), False) for p in adp_list],
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )

        canvas = Image.new("RGB", (quad_w * 2, tl.height * 2), (10, 14, 22))
        canvas.paste(tl, (0, 0))
        canvas.paste(tr, (quad_w, 0))
        canvas.paste(bl, (0, tl.height))
        canvas.paste(br, (quad_w, tl.height))
        out_p = out_dir / f"panel_{img_id}.png"
        canvas.save(out_p)
        saved_paths.append(str(out_p))
    return saved_paths


def build_comparison_html(
    items: list[dict[str, Any]],
    old_siglip_preds: dict[int, list[dict[str, Any]]],
    owl_frozen_preds: dict[int, list[dict[str, Any]]],
    owl_adapted_preds: dict[int, list[dict[str, Any]]],
    summary: dict[str, Any],
    out_path: Path,
    *,
    siglip_thresh: float,
    owl_thresh: float,
    pane3_label: str = "New OWLv2-12k Frozen",
    pane4_label: str = "New OWLv2-12k Adapted",
) -> None:
    """Write paginated interactive HTML comparing Clean GT vs Old SigLIP vs New OWLv2."""
    payload: list[dict[str, Any]] = []
    all_classes: set[str] = set()
    ds_counts: dict[str, int] = defaultdict(int)
    core_count = 0

    for item in items:
        img_id = int(item["image_id"])
        src = str(item["source_dataset"])
        ds_counts[src] += 1
        if item.get("is_core"):
            core_count += 1
        with Image.open(item["image_path"]) as im:
            rgb = im.convert("RGB")
            scale = min(1.0, 800.0 / max(rgb.width, 1))
            if scale < 1.0:
                rgb = rgb.resize((int(rgb.width * scale), int(rgb.height * scale)), Image.Resampling.BILINEAR)
            buf = io.BytesIO()
            rgb.save(buf, format="JPEG", quality=78)
            thumb = "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")

        old_p = old_siglip_preds.get(img_id, [])
        frz_p = owl_frozen_preds.get(img_id, [])
        adp_p = owl_adapted_preds.get(img_id, [])
        for g in item["gt_kept"]:
            all_classes.add(str(g["label"]))
        for p in old_p + frz_p + adp_p:
            all_classes.add(str(p["label"]))

        payload.append(
            {
                "image_id": img_id,
                "is_core": bool(item.get("is_core")),
                "source_dataset": src,
                "camera_type": item["camera_type"],
                "width": item["width"],
                "height": item["height"],
                "hygiene_stats": item["hygiene_stats"],
                "gt_kept": item["gt_kept"],
                "gt_dropped": item["gt_dropped"],
                "gt_demoted": item["gt_demoted"],
                "old_siglip": old_p,
                "owl_frozen": frz_p,
                "owl_adapted": adp_p,
                "thumb": thumb,
            }
        )

    model_rows = ""
    for name, st in summary["models"].items():
        model_rows += (
            f"<tr><td>{escape(name)}</td><td>{st['score_threshold']:.2f}</td>"
            f"<td>{st['latency_ms']:.2f} ms</td><td>{st['nonempty_images']}/{st['images']}</td>"
            f"<td>{st['avg_boxes_per_image']:.2f}</td><td>{st['unique_classes']}</td></tr>"
        )

    ds_chips = (
        f'<button class="chip active" id="ds-all" onclick="setDs(\'all\')">All ({len(payload)})</button>'
        f'<button class="chip" id="ds-core" onclick="setDs(\'core\')">Core ({core_count})</button>'
    )
    for src in DATASET_ORDER:
        if src in ds_counts:
            ds_chips += (
                f'<button class="chip" id="ds-{escape(src)}" onclick="setDs(\'{escape(src)}\')">'
                f"{escape(src)} ({ds_counts[src]})</button>"
            )

    cls_options = '<option value="all">All Classes</option>' + "".join(
        f'<option value="{escape(c)}">{escape(c)}</option>' for c in sorted(all_classes)
    )

    html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Before/After: Old SigLIP-8k vs New OWLv2-12k ({len(payload)} Images)</title>
<style>
body{{font-family:system-ui,sans-serif;background:#0b1020;color:#e2e8f0;margin:0;padding:18px}}
h1{{margin:0 0 8px;font-size:1.3rem}}
table{{border-collapse:collapse;font-size:.82rem;background:#131a2e;margin-bottom:12px}}
th,td{{border:1px solid #26314a;padding:5px 10px;text-align:right}} th:first-child,td:first-child{{text-align:left}}
.controls{{position:sticky;top:0;z-index:10;background:#11182c;padding:10px 12px;border:1px solid #2a3754;border-radius:8px;margin-bottom:14px;display:flex;flex-direction:column;gap:8px}}
.row{{display:flex;gap:12px;flex-wrap:wrap;align-items:center}}
.chip{{background:#1e293b;color:#cbd5e1;border:1px solid #334155;padding:4px 10px;border-radius:999px;cursor:pointer;font-size:.80rem}}
.chip.active{{background:#0284c7;color:#fff;border-color:#38bdf8;font-weight:600}}
select,input[type="text"]{{background:#0f172a;color:#e2e8f0;border:1px solid #334155;border-radius:6px;padding:4px 8px;font-size:.82rem}}
.pager{{display:flex;gap:10px;align-items:center;justify-content:space-between;margin:8px 0}}
.btn{{background:#1e293b;color:#e2e8f0;border:1px solid #334155;padding:5px 12px;border-radius:6px;cursor:pointer;font-size:.82rem}}
.btn:disabled{{opacity:.4;cursor:default}}
.card{{background:#131a2e;border:1px solid #26314a;border-radius:8px;padding:10px;margin-bottom:16px}}
.hdr{{display:flex;justify-content:space-between;margin-bottom:8px;font-size:.9rem}}
.badge{{background:#0369a1;color:#e0f2fe;padding:1px 7px;border-radius:4px;font-size:.75rem;margin-left:8px}}
.grid{{display:grid;grid-template-columns:1fr 1fr;gap:8px}} canvas{{width:100%;height:auto;border-radius:6px;display:block}}
</style></head><body>
<h1>Before / After Student Opt-B HBB (384x384): Old SigLIP-8k vs New OWLv2-12k ({len(payload)} Images)</h1>
<table><tr><th>Model</th><th>Default Joint</th><th>Warm GPU Latency</th><th>Non-Empty</th><th>Avg Boxes/Img</th><th>Unique Classes</th></tr>{model_rows}</table>
<div class="controls">
  <div class="row">{ds_chips}</div>
  <div class="row">
    <label>Class: <select id="cls-filter" onchange="render(true)">{cls_options}</select></label>
    <label>Image ID / Search: <input type="text" id="q-search" placeholder="e.g. 189270" oninput="render(true)" style="width:130px"></label>
    <label><input type="checkbox" id="show-dropped" checked onchange="render(false)"> Show Dropped/Demoted GT</label>
    <label>Old SigLIP &ge; <span id="sv">{siglip_thresh:.2f}</span> <input type="range" id="st" min="0.05" max="0.80" step="0.02" value="{siglip_thresh:.2f}" oninput="sv.textContent=Number(this.value).toFixed(2);render(false)"></label>
    <label>New OWLv2 &ge; <span id="ov">{owl_thresh:.2f}</span> <input type="range" id="ot" min="0.05" max="0.80" step="0.02" value="{owl_thresh:.2f}" oninput="ov.textContent=Number(this.value).toFixed(2);render(false)"></label>
    <label><input type="checkbox" id="show-all" onchange="render(true)"> Show All</label>
  </div>
</div>
<div class="pager"><span id="count-info"></span><div><button class="btn" id="prev-btn" onclick="stepPage(-1)">Prev</button> <span id="page-info" style="margin:0 8px;font-size:.86rem"></span> <button class="btn" id="next-btn" onclick="stepPage(1)">Next</button></div></div>
<div id="gal"></div>
<div class="pager"><span></span><div><button class="btn" id="prev-btn2" onclick="stepPage(-1)">Prev</button> <span id="page-info2" style="margin:0 8px;font-size:.86rem"></span> <button class="btn" id="next-btn2" onclick="stepPage(1)">Next</button></div></div>
<script>
const ITEMS = {json.dumps(payload)};
const PAGE_SIZE = 12;
let activeDs = 'all';
let curPage = 0;

function setDs(ds) {{
  activeDs = ds;
  document.querySelectorAll('.chip').forEach(c => c.classList.remove('active'));
  const el = document.getElementById('ds-' + ds);
  if (el) el.classList.add('active');
  render(true);
}}
function stepPage(delta) {{ curPage += delta; render(false); window.scrollTo(0, 0); }}
function drawPane(canvas, img, title, bg, boxes) {{
  const ctx = canvas.getContext('2d'); const sx = canvas.width / canvas.dataset.w; const sy = canvas.height / canvas.dataset.h;
  ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
  ctx.fillStyle = bg; ctx.fillRect(0, 0, canvas.width, 24);
  ctx.fillStyle = '#fff'; ctx.font = 'bold 12px sans-serif'; ctx.fillText(title, 8, 16);
  for (const b of boxes) {{
    const [x1,y1,x2,y2] = b.box;
    ctx.strokeStyle = b.color; ctx.lineWidth = 2; ctx.setLineDash(b.dash ? [6,4] : []);
    ctx.strokeRect(x1*sx, y1*sy, (x2-x1)*sx, (y2-y1)*sy); ctx.setLineDash([]);
    ctx.font = 'bold 11px sans-serif'; const tw = ctx.measureText(b.txt).width;
    const ty = Math.max(25, y1*sy - 14);
    ctx.fillStyle = 'rgba(10,15,25,0.85)'; ctx.fillRect(x1*sx, ty, tw+6, 14);
    ctx.fillStyle = b.color; ctx.fillText(b.txt, x1*sx+3, ty+11);
  }}
}}
function render(resetPage) {{
  if (resetPage) curPage = 0;
  const showBad = document.getElementById('show-dropped').checked;
  const st = parseFloat(document.getElementById('st').value);
  const ot = parseFloat(document.getElementById('ot').value);
  const clsF = document.getElementById('cls-filter').value;
  const q = document.getElementById('q-search').value.trim().toLowerCase();
  const showAll = document.getElementById('show-all').checked;

  const filtered = ITEMS.filter(it => {{
    if (activeDs === 'core' && !it.is_core) return false;
    if (activeDs !== 'all' && activeDs !== 'core' && it.source_dataset !== activeDs) return false;
    if (q && !String(it.image_id).includes(q) && !it.source_dataset.toLowerCase().includes(q)) return false;
    if (clsF !== 'all') {{
      const hasGt = it.gt_kept.some(g => g.label === clsF);
      const hasOld = it.old_siglip.some(p => p.score >= st && p.label === clsF);
      const hasFrz = it.owl_frozen.some(p => p.score >= ot && p.label === clsF);
      const hasAdp = it.owl_adapted.some(p => p.score >= ot && p.label === clsF);
      if (!hasGt && !hasOld && !hasFrz && !hasAdp) return false;
    }}
    return true;
  }});

  const totalPages = showAll ? 1 : Math.max(1, Math.ceil(filtered.length / PAGE_SIZE));
  if (curPage >= totalPages) curPage = totalPages - 1;
  if (curPage < 0) curPage = 0;
  const slice = showAll ? filtered : filtered.slice(curPage * PAGE_SIZE, (curPage + 1) * PAGE_SIZE);

  document.getElementById('count-info').textContent = `Showing ${{slice.length}} of ${{filtered.length}} filtered images (total ${{ITEMS.length}})`;
  const pTxt = `Page ${{curPage + 1}} / ${{totalPages}}`;
  document.getElementById('page-info').textContent = pTxt;
  document.getElementById('page-info2').textContent = pTxt;
  ['prev-btn','prev-btn2'].forEach(id => document.getElementById(id).disabled = (curPage <= 0 || showAll));
  ['next-btn','next-btn2'].forEach(id => document.getElementById(id).disabled = (curPage >= totalPages - 1 || showAll));

  const gal = document.getElementById('gal'); gal.innerHTML = '';
  for (const it of slice) {{
    const card = document.createElement('div'); card.className = 'card';
    const badge = it.is_core ? '<span class="badge">Core Audited</span>' : '';
    card.innerHTML = `<div class="hdr"><span><strong>${{it.source_dataset}} #${{it.image_id}}</strong>${{badge}}</span><span>${{it.width}}x${{it.height}} | kept=${{it.hygiene_stats.kept_positives}}/${{it.hygiene_stats.raw_positives}}</span></div><div class="grid"></div>`;
    const grid = card.querySelector('.grid');
    const img = new Image(); img.src = it.thumb;
    const panes = [0,1,2,3].map(() => {{ const c = document.createElement('canvas'); grid.appendChild(c); return c; }});
    img.onload = () => {{
      panes.forEach(c => {{ c.width = img.width; c.height = img.height; c.dataset.w = it.width; c.dataset.h = it.height; }});
      const gtBoxes = [];
      if (showBad) {{
        it.gt_dropped.forEach(x => gtBoxes.push({{box:x.box,color:'#f87171',dash:true,txt:'DROP:'+x.label}}));
        it.gt_demoted.forEach(x => gtBoxes.push({{box:x.box,color:'#fb923c',dash:true,txt:'IGN:'+x.raw_label}}));
      }}
      it.gt_kept.forEach(x => gtBoxes.push({{box:x.box,color:'#4ade80',dash:false,txt:x.label}}));
      const oldBoxes = it.old_siglip.filter(p=>p.score>=st).slice(0,30).map(p=>({{box:p.box,color:'#60a5fa',dash:false,txt:`${{p.label}} ${{Math.round(p.score*100)}}%`}}));
      const frzBoxes = it.owl_frozen.filter(p=>p.score>=ot).slice(0,30).map(p=>({{box:p.box,color:'#22d3ee',dash:false,txt:`${{p.label}} ${{Math.round(p.score*100)}}%`}}));
      const adpBoxes = it.owl_adapted.filter(p=>p.score>=ot).slice(0,30).map(p=>({{box:p.box,color:'#e879f9',dash:false,txt:`${{p.label}} ${{Math.round(p.score*100)}}%`}}));
      drawPane(panes[0], img, `1. Clean GT (${{it.hygiene_stats.kept_positives}}/${{it.hygiene_stats.raw_positives}})`, 'rgba(22,78,52,0.9)', gtBoxes);
      drawPane(panes[1], img, `2. Old SigLIP-8k Opt-B (n=${{oldBoxes.length}})`, 'rgba(30,58,138,0.9)', oldBoxes);
      drawPane(panes[2], img, `3. {escape(pane3_label)} (n=${{frzBoxes.length}})`, 'rgba(14,116,144,0.9)', frzBoxes);
      drawPane(panes[3], img, `4. {escape(pane4_label)} (n=${{adpBoxes.length}})`, 'rgba(107,33,168,0.9)', adpBoxes);
    }};
    gal.appendChild(card);
  }}
}}
window.addEventListener('DOMContentLoaded', () => render(true));
</script></body></html>"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare Old SigLIP-8k vs New OWLv2-12k Student HBB.")
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument("--holdout-manifest", type=Path, default=Path("artifacts/dashboard_holdout_8k_8k_full.json"))
    parser.add_argument("--num-images", type=int, default=72)
    parser.add_argument("--siglip-ckpt", type=Path, default=Path("artifacts/sota_8k_hbb/checkpoints/sota_joint_best.pt"))
    parser.add_argument("--siglip-vocab-37", type=Path, default=Path("artifacts/sota_open_vocab/vocabulary_bank_37classes.pt"))
    parser.add_argument(
        "--siglip-vocab-dist", type=Path, default=Path("artifacts/sota_open_vocab/vocabulary_bank_distractors.pt")
    )
    parser.add_argument("--siglip-centroid", type=Path, default=Path("artifacts/sota_open_vocab/visual_centroid.pt"))
    parser.add_argument("--owl-ckpt", type=Path, default=Path("artifacts/owlv2_12k_hbb/checkpoints/owlv2_hbb_best.pt"))
    parser.add_argument(
        "--owl-frozen-vocab-37", type=Path, default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_37classes.pt")
    )
    parser.add_argument(
        "--owl-adapted-vocab-37", type=Path, default=Path("artifacts/owlv2_12k_hbb/owlv2_adapted_vocab_37classes.pt")
    )
    parser.add_argument(
        "--owl-vocab-dist", type=Path, default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt")
    )
    parser.add_argument("--siglip-thresh", type=float, default=0.20)
    parser.add_argument("--owl-thresh", type=float, default=0.25)
    parser.add_argument("--pane3-label", type=str, default="New OWLv2-12k Frozen")
    parser.add_argument("--pane4-label", type=str, default="New OWLv2-12k Adapted")
    parser.add_argument("--html-name", type=str, default="dashboard_owlv2_vs_siglip.html")
    parser.add_argument("--out-dir", type=Path, default=Path("artifacts/owlv2_12k_hbb"))
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    target_ids, core_set = select_poc_image_ids(
        args.gt_index,
        args.holdout_manifest,
        args.image_root,
        num_images=args.num_images,
    )
    items = load_diagnostic_images(args.gt_index, args.image_root, target_ids, core_set)
    print(f"Selected {len(items)} images ({len(core_set)} core pinned).")

    print("[1/3] Running Old SigLIP-8k Opt-B HBB...")
    old_preds, old_ms = run_student_option_b_inference(
        items,
        ckpt_path=args.siglip_ckpt,
        vocab_37_path=args.siglip_vocab_37,
        vocab_dist_path=args.siglip_vocab_dist,
        centroid_path=args.siglip_centroid,
        device=device,
    )

    no_centroid = args.out_dir / "__no_centroid__.pt"
    print(f"[2/3] Running {args.pane3_label}...")
    owl_frz_preds, frz_ms = run_student_option_b_inference(
        items,
        ckpt_path=args.owl_ckpt,
        vocab_37_path=args.owl_frozen_vocab_37,
        vocab_dist_path=args.owl_vocab_dist,
        centroid_path=no_centroid,
        device=device,
    )

    print(f"[3/3] Running {args.pane4_label}...")
    owl_adp_preds, adp_ms = run_student_option_b_inference(
        items,
        ckpt_path=args.owl_ckpt,
        vocab_37_path=args.owl_adapted_vocab_37,
        vocab_dist_path=args.owl_vocab_dist,
        centroid_path=no_centroid,
        device=device,
    )

    summary = {
        "num_images": len(items),
        "core_ids": sorted(core_set),
        "models": {
            "Old SigLIP-8k Opt-B": summarize_predictions(
                items, old_preds, score_thresh=args.siglip_thresh, latency_ms=old_ms
            ),
            args.pane3_label: summarize_predictions(
                items, owl_frz_preds, score_thresh=args.owl_thresh, latency_ms=frz_ms
            ),
            args.pane4_label: summarize_predictions(
                items, owl_adp_preds, score_thresh=args.owl_thresh, latency_ms=adp_ms
            ),
        },
    }

    args.out_dir.mkdir(parents=True, exist_ok=True)
    panels = render_comparison_panels(
        items,
        old_preds,
        owl_frz_preds,
        owl_adp_preds,
        args.out_dir / "panels",
        siglip_thresh=args.siglip_thresh,
        owl_thresh=args.owl_thresh,
        pane3_label=args.pane3_label,
        pane4_label=args.pane4_label,
    )
    html_path = args.out_dir / args.html_name
    build_comparison_html(
        items,
        old_preds,
        owl_frz_preds,
        owl_adp_preds,
        summary,
        html_path,
        siglip_thresh=args.siglip_thresh,
        owl_thresh=args.owl_thresh,
        pane3_label=args.pane3_label,
        pane4_label=args.pane4_label,
    )
    summary_path = args.out_dir / "comparison_72_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"Saved {len(panels)} core panels under {args.out_dir / 'panels'}")
    print(f"HTML: {html_path}")
    print(f"Summary JSON: {summary_path}")


if __name__ == "__main__":
    main()
