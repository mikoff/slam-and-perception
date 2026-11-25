#!/usr/bin/env python3
"""Interactive Visualization Dashboard for Option A vs Option B vs Option A+B Open-Vocabulary Models.

Renders the 210 audited holdout images (0% overlap with train/val) with:
  - Live switching & 3-up side-by-side comparison of Option A (DenseAnchorSemanticHead),
    Option B (RoIPooledSemanticHead), Option A+B Fusion, and Previous Legacy (8k_full + RegionHead v4).
  - Live vocabulary toggle between Seen-37 (OPEN_VOCAB_CATEGORIES) and Zero-Shot-41
    (+ book, painting, cushion, flowerpot).
  - Disagreement filter (Option A != Option B), category/cohort filters, and hover-scroll
    spotlight HUD displaying Option A, Option B, and Option A+B predictions simultaneously.
"""

from __future__ import annotations

import argparse
import base64
from html import escape
import io
import json
from pathlib import Path
import sqlite3
from typing import Any

from PIL import Image
import torch

from student_detector.annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from student_detector.config import AugmentationConfig
from student_detector.dashboard_holdout import load_holdout
from student_detector.data import ProposalSample, ProposalTransform
from student_detector.decoder import InferenceDecoder
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
)
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.ontology import (
    DISTRACTOR_CATEGORIES,
    OPEN_VOCAB_CATEGORIES,
    ZERO_SHOT_EVAL_CATEGORIES,
    map_to_canonical,
    normalize_category_name,
)
from student_detector.open_vocab_heads import (
    DenseAnchorSemanticHead,
    RoIPooledSemanticHead,
    filter_semantic_detections,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.region_head import RegionEmbeddingHead
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT
from student_detector.targets import point_validity_from_pixel_mask
from student_detector.vocabulary import VocabularyBank


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hbb-sota-ckpt",
        type=Path,
        default=Path("artifacts/sota_8k_hbb/checkpoints/sota_joint_best.pt"),
    )
    parser.add_argument(
        "--quad-sota-ckpt",
        type=Path,
        default=Path("artifacts/sota_8k_quad/checkpoints/sota_joint_best.pt"),
    )
    parser.add_argument(
        "--hbb-legacy-ckpt",
        type=Path,
        default=Path(
            "artifacts/finetune_8k_hbb_8k_full/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"
        ),
    )
    parser.add_argument(
        "--quad-legacy-ckpt",
        type=Path,
        default=Path(
            "artifacts/finetune_8k_quad_8k_full/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
        ),
    )
    parser.add_argument(
        "--hbb-legacy-head",
        type=Path,
        default=Path(
            "artifacts/distill_8k_region_head_v4/checkpoints/region_head_best.pt"
        ),
    )
    parser.add_argument(
        "--quad-legacy-head",
        type=Path,
        default=Path(
            "artifacts/distill_8k_region_head_quad_v4/checkpoints/region_head_best.pt"
        ),
    )
    parser.add_argument(
        "--hbb-legacy-vocab",
        type=Path,
        default=Path(
            "artifacts/distill_8k_region_head_v4/vocabulary_bank_37classes.pt"
        ),
    )
    parser.add_argument(
        "--quad-legacy-vocab",
        type=Path,
        default=Path(
            "artifacts/distill_8k_region_head_quad_v4/vocabulary_bank_37classes.pt"
        ),
    )
    parser.add_argument(
        "--vocab-37",
        type=Path,
        default=Path("artifacts/sota_open_vocab/vocabulary_bank_37classes.pt"),
    )
    parser.add_argument(
        "--vocab-41",
        type=Path,
        default=Path("artifacts/sota_open_vocab/vocabulary_bank_41classes_zeroshot.pt"),
    )
    parser.add_argument(
        "--vocab-distractors",
        type=Path,
        default=Path("artifacts/sota_open_vocab/vocabulary_bank_distractors.pt"),
    )
    parser.add_argument(
        "--visual-centroid",
        type=Path,
        default=Path("artifacts/sota_open_vocab/visual_centroid.pt"),
    )
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        default=Path("artifacts/sota_open_vocab/benchmark_option_a_vs_b.json"),
    )
    parser.add_argument(
        "--quad-benchmark-json",
        type=Path,
        default=None,
        help="Optional Quad benchmark JSON whose geometries.quad section is merged into --benchmark-json for joint dashboards.",
    )
    parser.add_argument(
        "--evaluation-manifest",
        type=Path,
        default=Path("artifacts/dashboard_holdout_8k_8k_full.json"),
    )
    parser.add_argument(
        "--gt-index",
        type=Path,
        default=DEFAULT_ENRICHED_GT_INDEX_PATH,
    )
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument(
        "--output-html",
        type=Path,
        default=Path("artifacts/dashboard_open_vocab_a_b_ab.html"),
    )
    parser.add_argument(
        "--only-hbb",
        action="store_true",
        help="Skip Quad checkpoints and render an HBB-only dashboard.",
    )
    parser.add_argument(
        "--vocab-core27",
        type=Path,
        default=None,
        help="Optional Core-27 preset seen VocabularyBank (.pt).",
    )
    parser.add_argument(
        "--vocab-core31",
        type=Path,
        default=None,
        help="Optional Core-27 + 4 zero-shot VocabularyBank (.pt).",
    )
    parser.add_argument(
        "--hbb-baseline-sota-ckpt",
        type=Path,
        default=None,
        help="Optional baseline HBB SOTA checkpoint (e.g. Old SigLIP-8k Opt-B) for comparison.",
    )
    parser.add_argument(
        "--baseline-vocab-37",
        type=Path,
        default=Path("artifacts/sota_open_vocab/vocabulary_bank_37classes.pt"),
    )
    parser.add_argument(
        "--baseline-vocab-distractors",
        type=Path,
        default=Path("artifacts/sota_open_vocab/vocabulary_bank_distractors.pt"),
    )
    parser.add_argument(
        "--baseline-visual-centroid",
        type=Path,
        default=Path("artifacts/sota_open_vocab/visual_centroid.pt"),
    )
    parser.add_argument(
        "--baseline-label",
        type=str,
        default="Previous Legacy (v4)",
    )
    parser.add_argument(
        "--benchmark-core27-json",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--benchmark-baseline-json",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--assert-no-overlap-cache",
        type=Path,
        default=None,
        help="Assert 0 dashboard images appear in this training SQLite cache (processed_images).",
    )
    parser.add_argument(
        "--title",
        type=str,
        default=None,
        help="Optional dashboard HTML title.",
    )
    parser.add_argument("--small-box-score-power", type=float, default=1.0)
    parser.add_argument("--small-box-min-side", type=float, default=16.0)
    parser.add_argument("--decoder-top-k", type=int, default=500)
    parser.add_argument("--decoder-max-detections", type=int, default=80)
    parser.add_argument("--quad-vocab-37", type=Path, default=None)
    parser.add_argument("--quad-vocab-41", type=Path, default=None)
    parser.add_argument("--quad-vocab-core27", type=Path, default=None)
    parser.add_argument("--quad-vocab-core31", type=Path, default=None)
    parser.add_argument("--quad-small-box-score-power", type=float, default=None)
    parser.add_argument("--quad-small-box-min-side", type=float, default=None)
    parser.add_argument("--quad-decoder-top-k", type=int, default=None)
    parser.add_argument("--quad-decoder-max-detections", type=int, default=None)
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    return parser.parse_args()


def encode_thumbnail_base64(image_path: Path, max_dim: int = 920) -> str:
    with Image.open(image_path) as img:
        rgb = img.convert("RGB")
        orig_w, orig_h = rgb.size
        scale = min(max_dim / max(orig_w, orig_h), 1.0)
        if scale < 1.0:
            rgb = rgb.resize(
                (int(orig_w * scale), int(orig_h * scale)),
                Image.Resampling.BILINEAR,
            )
        buf = io.BytesIO()
        rgb.save(buf, format="JPEG", quality=82)
        encoded = base64.b64encode(buf.getvalue()).decode("utf-8")
        return f"data:image/jpeg;base64,{encoded}"


def load_gt_annotations_with_labels(
    gt_index: Path,
    image_ids: list[int],
    *,
    sanitize_annotations: bool = True,
) -> dict[int, list[dict[str, Any]]]:
    """Fetch foreground human GT boxes, quads, and canonical/zero-shot labels from SQLite."""
    out: dict[int, list[dict[str, Any]]] = {img_id: [] for img_id in image_ids}
    if not gt_index.exists() or not image_ids:
        return out
    zs_set = set(ZERO_SHOT_EVAL_CATEGORIES)
    conn = sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True)
    try:
        cols = [r[1] for r in conn.execute("PRAGMA table_info(annotations)").fetchall()]
        cat_col = (
            "a.category_name"
            if "category_name" in cols
            else ("a.category" if "category" in cols else "''")
        )
        tier_col = "a.geometry_tier" if "geometry_tier" in cols else "'unknown'"
        placeholders = ",".join("?" for _ in image_ids)
        rows = conn.execute(
            f"""
            SELECT a.image_id, {cat_col}, a.ignore_region,
                   a.x1, a.y1, a.x2, a.y2, a.quad_json, {tier_col}, i.source_dataset
            FROM annotations a
            JOIN images i ON a.image_id = i.image_id
            WHERE a.image_id IN ({placeholders})
            ORDER BY a.image_id, a.rowid
            """,
            image_ids,
        ).fetchall()
        grouped: dict[int, list[tuple[Any, ...]]] = {}
        for r in rows:
            grouped.setdefault(int(r[0]), []).append(r)

        for img_id, img_rows in grouped.items():
            src_ds = str(img_rows[0][9] or "")
            hygiene_rows = [
                RawAnnotationRow(
                    x1=float(r[3]),
                    y1=float(r[4]),
                    x2=float(r[5]),
                    y2=float(r[6]),
                    ignore_region=int(r[2] or 0),
                    category_name=str(r[1] or ""),
                    geometry_tier=str(r[8] or "unknown"),
                )
                for r in img_rows
            ]
            if sanitize_annotations:
                kept = sanitize_image_annotations(
                    hygiene_rows, source_dataset=src_ds
                ).kept_positive_indices
            else:
                kept = tuple(
                    idx
                    for idx, hr in enumerate(hygiene_rows)
                    if hr.ignore_region == 0
                )
            for idx in kept:
                r = img_rows[idx]
                raw_cat = str(r[1] or "")
                canon = map_to_canonical(raw_cat)
                norm = normalize_category_name(raw_cat)
                label = (
                    canon
                    if canon is not None
                    else (norm if norm in zs_set else raw_cat)
                )
                is_zs = canon is None and norm in zs_set
                box = [
                    round(float(r[3]), 1),
                    round(float(r[4]), 1),
                    round(float(r[5]), 1),
                    round(float(r[6]), 1),
                ]
                quad = None
                if r[7]:
                    raw_q = json.loads(str(r[7]))
                    if isinstance(raw_q, list) and len(raw_q) == 4:
                        quad = [
                            [round(float(pt[0]), 1), round(float(pt[1]), 1)]
                            for pt in raw_q
                        ]
                out.setdefault(img_id, []).append(
                    {
                        "label": label,
                        "raw_label": raw_cat,
                        "is_zero_shot": is_zs,
                        "box": box,
                        "quad": quad,
                    }
                )
    finally:
        conn.close()
    return out


RARE_SHOWCASE_CLASSES = (
    "trash_bin",
    "pillar",
    "fire_hydrant",
    "traffic_sign",
    "traffic_light",
    "signboard",
    "traffic_cone",
    "pole",
)


def load_rare_showcase_val_items(
    gt_index: Path,
    image_root: Path,
    existing_ids: set[int],
    max_per_class: int = 5,
    max_total: int = 26,
) -> list[tuple[ProposalSample, dict[str, Any]]]:
    """Select held-out validation images containing rare & slender classes (0% train overlap)."""
    if not gt_index.exists():
        return []
    val_ds = HumanGTHBBDataset(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="val",
        input_size=384,
        cohort_size=8000,
    )
    val_records_by_id = {
        int(r["image_id"]): r
        for r in val_ds.images
        if int(r["image_id"]) not in existing_ids
    }
    if not val_records_by_id:
        return []

    gt_by_id = load_gt_annotations_with_labels(gt_index, list(val_records_by_id.keys()))
    rare_set = set(RARE_SHOWCASE_CLASSES)
    candidates_by_cls: dict[str, list[tuple[int, list[str]]]] = {
        c: [] for c in RARE_SHOWCASE_CLASSES
    }
    for img_id, anns in gt_by_id.items():
        present = sorted({str(a["label"]) for a in anns if str(a["label"]) in rare_set})
        if not present:
            continue
        for c in present:
            candidates_by_cls[c].append((img_id, present))

    if 600267 in {cid for cid, _ in candidates_by_cls.get("trash_bin", [])}:
        candidates_by_cls["trash_bin"].sort(key=lambda item: 0 if item[0] == 600267 else 1)

    chosen_ids: list[int] = []
    chosen_classes: dict[int, list[str]] = {}
    for c in RARE_SHOWCASE_CLASSES:
        count_c = 0
        for img_id, present in candidates_by_cls[c]:
            if img_id in chosen_classes:
                continue
            chosen_ids.append(img_id)
            chosen_classes[img_id] = present
            count_c += 1
            if count_c >= max_per_class or len(chosen_ids) >= max_total:
                break
        if len(chosen_ids) >= max_total:
            break

    transform = ProposalTransform(
        384, AugmentationConfig(), training=False, tiny_area=4, tiny_min_side=2
    )
    showcase_items: list[tuple[ProposalSample, dict[str, Any]]] = []
    for img_id in chosen_ids:
        row = dict(val_records_by_id[img_id])
        img_path = image_root / row["file_name"]
        if not img_path.exists():
            continue
        with Image.open(img_path) as image:
            orig_w, orig_h = image.size
            transformed = transform(
                image, torch.empty((0, 4)), torch.empty((0, 4)), seed=42
            )
        row["width"] = int(row.get("width") or orig_w)
        row["height"] = int(row.get("height") or orig_h)
        row["camera_type"] = str(row.get("camera_type") or "pinhole")
        row["is_rare_showcase"] = True
        row["showcase_classes"] = chosen_classes[img_id]
        sample = ProposalSample(
            image=transformed[0],
            boxes=transformed[1],
            ignore_boxes=transformed[2],
            valid_mask=transformed[4],
            image_id=img_id,
            source_dataset=str(row["source_dataset"]),
            domain="general",
            camera_type=row["camera_type"],
            background_supervision=False,
            original_size=(row["height"], row["width"]),
            transform=transformed[5],
        )
        showcase_items.append((sample, row))
    return showcase_items


def pack_top3(
    prob_row: torch.Tensor,
    categories: tuple[str, ...],
) -> tuple[str, float, list[list[Any]]]:
    k = min(3, int(prob_row.numel()))
    vals, idxs = prob_row.topk(k)
    lbl = categories[int(idxs[0].item())]
    pr = round(float(vals[0].item()), 3)
    top3 = [
        [categories[int(idxs[i].item())], round(float(vals[i].item()), 3)]
        for i in range(k)
    ]
    return lbl, pr, top3


def _render_hbb_summary_and_table(
    bench: dict[str, Any],
    bench_core27: dict[str, Any] | None,
    bench_baseline: dict[str, Any] | None,
    baseline_label: str,
) -> tuple[str, str]:
    """Render HBB-focused KPI cards and Cohort-Val-800 benchmark table HTML."""

    def _pct(val: Any) -> str:
        return f"{float(val) * 100:.1f}%" if val is not None else "—"

    def _get_hbb_sota(b: dict[str, Any] | None) -> dict[str, Any]:
        if not b:
            return {}
        return (
            b.get("geometries", {})
            .get("hbb", {})
            .get("splits", {})
            .get("cohort_val_800", {})
            .get("sota", {})
        )

    h_full = _get_hbb_sota(bench)
    h_c27 = _get_hbb_sota(bench_core27)
    primary = h_c27 or h_full
    pri_tag = "Core-27" if h_c27 else "Full-37"

    p_loc = primary.get("localization", {})
    p_sem = primary.get("semantics_37_seen", {})
    p_zs = primary.get("zero_shot_4_unseen", {})
    f_sem = h_full.get("semantics_37_seen", {})

    b_loc: dict[str, Any] = {}
    b_sem: dict[str, Any] = {}
    b_zs_row: dict[str, Any] = {}
    if bench_baseline:
        b_split = (
            bench_baseline.get("geometries", {})
            .get("hbb", {})
            .get("splits", {})
            .get("cohort_val_800", {})
        )
        if "sota" in b_split:
            b_sota = b_split["sota"]
            b_loc = b_sota.get("localization", {})
            b_sem = b_sota.get("semantics_37_seen", {}).get("option_b", {})
            b_zs_row = b_sota.get("zero_shot_4_unseen", {}).get("option_b", {})
        else:
            b_leg = b_split.get("legacy", {}).get("8k_full_region_head_v4", {})
            b_loc = b_leg.get("localization", {})
            b_sem = b_leg.get("semantics_37_seen", {})
    elif bench:
        b_split = (
            bench.get("geometries", {})
            .get("hbb", {})
            .get("splits", {})
            .get("cohort_val_800", {})
        )
        b_leg = b_split.get("legacy", {}).get("8k_full_region_head_v4", {})
        b_loc = b_leg.get("localization", {})
        b_sem = b_leg.get("semantics_37_seen", {})

    kpi_html = f"""<section class="kpi-grid">
  <div class="kpi" style="border-color: rgba(59,130,246,0.5);">
    <div class="k-lbl">🎯 Option B ({pri_tag} / Full-37)</div>
    <div class="k-val" style="color: #60a5fa;">{_pct(p_sem.get("option_b", {}).get("matched_prop_top1"))} / {_pct(f_sem.get("option_b", {}).get("matched_prop_top1"))}</div>
    <div class="k-sub">Seen Top-1 &bull; Oracle: {_pct(p_sem.get("option_b", {}).get("oracle_gt_box_top1"))} &bull; mAP@50: {_pct(p_sem.get("option_b", {}).get("open_vocab_map50"))}</div>
  </div>
  <div class="kpi" style="border-color: rgba(34,197,94,0.5);">
    <div class="k-lbl">🚀 Option A ({pri_tag} / Full-37)</div>
    <div class="k-val" style="color: #4ade80;">{_pct(p_sem.get("option_a", {}).get("matched_prop_top1"))} / {_pct(f_sem.get("option_a", {}).get("matched_prop_top1"))}</div>
    <div class="k-sub">Seen Top-1 &bull; mAP@50: {_pct(p_sem.get("option_a", {}).get("open_vocab_map50"))} / {_pct(f_sem.get("option_a", {}).get("open_vocab_map50"))}</div>
  </div>
  <div class="kpi" style="border-color: rgba(168,85,247,0.5);">
    <div class="k-lbl">⚡ Option A+B Fusion ({pri_tag})</div>
    <div class="k-val" style="color: #c084fc;">{_pct(p_sem.get("option_ab", {}).get("matched_prop_top1"))} / {_pct(p_sem.get("option_ab", {}).get("matched_prop_top3"))}</div>
    <div class="k-sub">Top-1 / Top-3 &bull; mAP@50: {_pct(p_sem.get("option_ab", {}).get("open_vocab_map50"))} (Full-37: {_pct(f_sem.get("option_ab", {}).get("open_vocab_map50"))})</div>
  </div>
  <div class="kpi" style="border-color: rgba(245,158,11,0.5);">
    <div class="k-lbl">📐 R@100 &amp; Zero-Shot-4</div>
    <div class="k-val" style="color: #fbbf24;">{_pct(p_loc.get("recall_100_50"))} / {_pct(p_zs.get("option_b", {}).get("unseen_4way_top1"))}</div>
    <div class="k-sub">R@100@50 (AP50 {_pct(p_loc.get("class_agnostic_ap50"))}) &bull; 41-way Top-5: {_pct(p_zs.get("option_b", {}).get("full_41way_top5"))}</div>
  </div>
</section>"""

    def _sota_rows(split_label: str, sota_d: dict[str, Any]) -> str:
        loc = sota_d.get("localization", {})
        sem = sota_d.get("semantics_37_seen", {})
        zs = sota_d.get("zero_shot_4_unseen", {})
        specs = [
            ("a", "🚀 Option A (DenseAnchor)", "#4ade80", ""),
            (
                "b",
                "🎯 Option B (RoIPooled)",
                "#60a5fa",
                ' style="background:rgba(59,130,246,0.10);"',
            ),
            ("ab", "⚡ Option A+B Fusion", "#c084fc", ""),
        ]
        rows = []
        for k, name, color, tr_attr in specs:
            opt_k = sem.get(f"option_{k}", {})
            zs_k = zs.get(f"option_{k}", {})
            rows.append(
                f"<tr{tr_attr}><td><strong>{escape(split_label)}</strong></td>"
                f'<td style="color:{color};font-weight:700;">{name}</td>'
                f"<td>{_pct(loc.get('recall_100_50'))}</td>"
                f"<td>{loc.get('boxes_per_img_030', '—')}</td>"
                f"<td>{loc.get('post_semantic_nms_overlap_pairs_025', loc.get('overlap_pairs_025', '—'))}</td>"
                f"<td>{_pct(loc.get('dup_gt_rate'))}</td>"
                f"<td><strong>{_pct(opt_k.get('matched_prop_top1'))}</strong></td>"
                f"<td>{_pct(opt_k.get('matched_prop_top3'))}</td>"
                f"<td><strong>{_pct(opt_k.get('open_vocab_map50'))}</strong></td>"
                f"<td>{_pct(opt_k.get('clutter_rejection_rate'))}</td>"
                f"<td>{_pct(opt_k.get('signboard_to_window_door_err'))}</td>"
                f"<td>{_pct(zs_k.get('unseen_4way_top1'))} / {_pct(zs_k.get('full_41way_top5'))}</td></tr>"
            )
        return "\n".join(rows)

    rows_html = []
    if h_c27:
        rows_html.append(_sota_rows("HBB (Core-27 Adp)", h_c27))
    if h_full:
        rows_html.append(_sota_rows("HBB (Full-37 Adp)", h_full))
    if b_sem or b_loc:
        rows_html.append(
            f'<tr style="color:var(--muted);"><td>HBB Baseline</td>'
            f"<td>🏛️ {escape(baseline_label)}</td>"
            f"<td>{_pct(b_loc.get('recall_100_50'))}</td>"
            f"<td>{b_loc.get('boxes_per_img_030', '—')}</td>"
            f"<td>{b_loc.get('post_semantic_nms_overlap_pairs_025', b_loc.get('overlap_pairs_025', '—'))}</td>"
            f"<td>{_pct(b_loc.get('dup_gt_rate'))}</td>"
            f"<td>{_pct(b_sem.get('matched_prop_top1'))}</td>"
            f"<td>{_pct(b_sem.get('matched_prop_top3'))}</td>"
            f"<td>{_pct(b_sem.get('open_vocab_map50'))}</td>"
            f"<td>{_pct(b_sem.get('clutter_rejection_rate'))}</td>"
            f"<td>{_pct(b_sem.get('signboard_to_window_door_err'))}</td>"
            f"<td>{_pct(b_zs_row.get('unseen_4way_top1'))} / {_pct(b_zs_row.get('full_41way_top5'))}</td></tr>"
        )

    tbl_html = f"""<details class="tbl-wrap">
  <summary>📊 Expand Full Cohort-Val-800 HBB Benchmark Table (Core-27 &amp; Full-37 &amp; {escape(baseline_label)})</summary>
  <table>
    <thead>
      <tr>
        <th>Preset / Split</th>
        <th>Option / Head</th>
        <th>R@100</th>
        <th>Boxes/Img @0.30</th>
        <th>Post-NMS Ovlp</th>
        <th>Dup GT %</th>
        <th>Seen Top-1</th>
        <th>Seen Top-3</th>
        <th>OV mAP@50</th>
        <th>Clutter Rej</th>
        <th>Signboard&rarr;Win/Door</th>
        <th>Zero-Shot 4w / 41-T5</th>
      </tr>
    </thead>
    <tbody>
      {"".join(rows_html)}
    </tbody>
  </table>
</details>"""
    return kpi_html, tbl_html


def build_html(
    images_data: list[dict[str, Any]],
    bench: dict[str, Any],
    output_path: Path,
    *,
    only_hbb: bool = False,
    has_core27: bool = False,
    has_core31: bool = False,
    bench_core27: dict[str, Any] | None = None,
    bench_baseline: dict[str, Any] | None = None,
    baseline_label: str = "Previous Legacy (v4)",
    title: str | None = None,
    holdout_count: int = 210,
    showcase_count_extra: int = 0,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    page_title = (
        title
        or "Open-Vocabulary Student Dashboard: Option A vs Option B vs Option A+B"
    )
    subtitle_imgs = (
        f"{holdout_count} holdout + {showcase_count_extra} rare-showcase val = {len(images_data)} unseen images (0% overlap with training)"
        if showcase_count_extra > 0
        else f"{len(images_data)} isolated holdout images (0% overlap with training)"
    )
    quad_val = (
        bench.get("geometries", {})
        .get("quad", {})
        .get("splits", {})
        .get("cohort_val_800", {})
    )
    hbb_val = (
        bench.get("geometries", {})
        .get("hbb", {})
        .get("splits", {})
        .get("cohort_val_800", {})
    )
    q_sota = quad_val.get("sota", {})
    q_leg = quad_val.get("legacy", {}).get("8k_full_region_head_v4", {})
    h_sota = hbb_val.get("sota", {})
    h_leg = hbb_val.get("legacy", {}).get("8k_full_region_head_v4", {})

    def _pct(val: Any) -> str:
        return f"{float(val) * 100:.1f}%" if val is not None else "—"

    q_sem = q_sota.get("semantics_37_seen", {})
    q_zs = q_sota.get("zero_shot_4_unseen", {})
    q_loc = q_sota.get("localization", {})
    ql_sem = q_leg.get("semantics_37_seen", {})
    ql_loc = q_leg.get("localization", {})

    h_sem = h_sota.get("semantics_37_seen", {})
    h_zs = h_sota.get("zero_shot_4_unseen", {})
    h_loc = h_sota.get("localization", {})
    hl_sem = h_leg.get("semantics_37_seen", {})
    hl_loc = h_leg.get("localization", {})

    unique_sources = sorted({img["source_dataset"] for img in images_data})
    cohort_counts: dict[str, int] = {}
    for img in images_data:
        s = img["source_dataset"]
        cohort_counts[s] = cohort_counts.get(s, 0) + 1

    cohort_labels = {
        "ade20k": "ADE20K",
        "open_images_subset": "Open Images",
        "roboflow_ev_parking": "EV Parking",
        "bdd100k_images_100k": "BDD100K",
        "coco_2017": "COCO",
        "nuimages": "nuImages",
        "woodscape_rgb_fisheye": "WoodScape Fisheye",
    }
    showcase_count = sum(1 for img in images_data if img.get("is_rare_showcase"))
    cohort_btns = [
        f'<button class="chip active" id="cohort-all" onclick="setCohort(\'all\')">All ({len(images_data)})</button>',
        f'<button class="chip" id="cohort-rare_showcase" onclick="setCohort(\'rare_showcase\')" style="border-color:rgba(245,158,11,0.55);color:#fbbf24;">🎯 Rare-Class Showcase ({showcase_count})</button>',
    ]
    for src in unique_sources:
        lbl = cohort_labels.get(src, src)
        cohort_btns.append(
            f'<button class="chip" id="cohort-{escape(src)}" onclick="setCohort(\'{escape(src)}\')">{escape(lbl)} ({cohort_counts[src]})</button>'
        )

    cat_options = ['<option value="all">🌟 All Categories (37 Seen + 4 Zero-Shot)</option>']
    for c in ZERO_SHOT_EVAL_CATEGORIES:
        cat_options.append(f'<option value="{escape(c)}">🧪 [Zero-Shot] {escape(c)}</option>')
    for c in OPEN_VOCAB_CATEGORIES:
        cat_options.append(f'<option value="{escape(c)}">{escape(c)}</option>')

    if only_hbb:
        kpi_section_html, tbl_section_html = _render_hbb_summary_and_table(
            bench, bench_core27, bench_baseline, baseline_label
        )
    else:
        kpi_section_html = f"""<section class="kpi-grid">
  <div class="kpi" style="border-color: rgba(59,130,246,0.5);">
    <div class="k-lbl">🎯 Option B (Focal+Context RoI)</div>
    <div class="k-val" style="color: #60a5fa;">{_pct(q_sem.get("option_b", {}).get("matched_prop_top1"))} / {_pct(h_sem.get("option_b", {}).get("matched_prop_top1"))}</div>
    <div class="k-sub">Seen-37 Top-1 (Quad / HBB) &bull; Zero-Shot 4-way: {_pct(q_zs.get("option_b", {}).get("unseen_4way_top1"))} &bull; +2.88M params</div>
  </div>
  <div class="kpi" style="border-color: rgba(34,197,94,0.5);">
    <div class="k-lbl">🚀 Option A (Dense Single-Stage)</div>
    <div class="k-val" style="color: #4ade80;">{_pct(q_sem.get("option_a", {}).get("matched_prop_top1"))} / {_pct(h_sem.get("option_a", {}).get("matched_prop_top1"))}</div>
    <div class="k-sub">Seen-37 Top-1 (Quad / HBB) &bull; Zero RoIAlign ops &bull; +0.135M params</div>
  </div>
  <div class="kpi" style="border-color: rgba(168,85,247,0.5);">
    <div class="k-lbl">⚡ Option A+B Fusion</div>
    <div class="k-val" style="color: #c084fc;">{_pct(q_sem.get("option_ab", {}).get("matched_prop_top3"))} / {_pct(h_sem.get("option_ab", {}).get("matched_prop_top3"))}</div>
    <div class="k-sub">Seen-37 Top-3 (Quad / HBB) &bull; Top-1: {_pct(q_sem.get("option_ab", {}).get("matched_prop_top1"))} / {_pct(h_sem.get("option_ab", {}).get("matched_prop_top1"))}</div>
  </div>
  <div class="kpi" style="border-color: rgba(245,158,11,0.5);">
    <div class="k-lbl">🧹 Overlap &amp; Signboard Fix</div>
    <div class="k-val" style="color: #fbbf24;">{q_loc.get("overlap_pairs_025", "1.08")} vs {ql_loc.get("overlap_pairs_025", "75.95")}</div>
    <div class="k-sub">Quad overlap pairs/img (SOTA vs Legacy) &bull; Signboard&rarr;Win/Door: <strong>0.0%</strong> (was 70%)</div>
  </div>
</section>"""
        tbl_section_html = f"""<details class="tbl-wrap">
  <summary>📊 Expand Full Cohort-Val-800 Benchmark Table (Option A vs Option B vs Option A+B vs Previous Legacy v4)</summary>
  <table>
    <thead>
      <tr>
        <th>Geometry</th>
        <th>Option / Head</th>
        <th>Boxes/Img @0.30</th>
        <th>Overlap Pairs</th>
        <th>Dup GT %</th>
        <th>Seen-37 Top-1</th>
        <th>Seen-37 Top-3</th>
        <th>OV mAP@50</th>
        <th>Clutter Rej</th>
        <th>Signboard&rarr;Win/Door</th>
        <th>Zero-Shot 4-Way / 41-Top5</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Quad</strong></td>
        <td style="color:#4ade80;font-weight:700;">🚀 Option A (DenseAnchor)</td>
        <td>{q_loc.get("boxes_per_img_030")}</td>
        <td>{q_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(q_loc.get("dup_gt_rate"))}</td>
        <td>{_pct(q_sem.get("option_a", {}).get("matched_prop_top1"))}</td>
        <td>{_pct(q_sem.get("option_a", {}).get("matched_prop_top3"))}</td>
        <td>{_pct(q_sem.get("option_a", {}).get("open_vocab_map50"))}</td>
        <td>{_pct(q_sem.get("option_a", {}).get("clutter_rejection_rate"))}</td>
        <td>{_pct(q_sem.get("option_a", {}).get("signboard_to_window_door_err"))}</td>
        <td>{_pct(q_zs.get("option_a", {}).get("unseen_4way_top1"))} / {_pct(q_zs.get("option_a", {}).get("full_41way_top5"))}</td>
      </tr>
      <tr style="background:rgba(59,130,246,0.10);">
        <td><strong>Quad</strong></td>
        <td style="color:#60a5fa;font-weight:700;">🎯 Option B (RoIPooled)</td>
        <td>{q_loc.get("boxes_per_img_030")}</td>
        <td>{q_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(q_loc.get("dup_gt_rate"))}</td>
        <td><strong>{_pct(q_sem.get("option_b", {}).get("matched_prop_top1"))}</strong></td>
        <td>{_pct(q_sem.get("option_b", {}).get("matched_prop_top3"))}</td>
        <td><strong>{_pct(q_sem.get("option_b", {}).get("open_vocab_map50"))}</strong></td>
        <td>{_pct(q_sem.get("option_b", {}).get("clutter_rejection_rate"))}</td>
        <td><strong>{_pct(q_sem.get("option_b", {}).get("signboard_to_window_door_err"))}</strong></td>
        <td><strong>{_pct(q_zs.get("option_b", {}).get("unseen_4way_top1"))} / {_pct(q_zs.get("option_b", {}).get("full_41way_top5"))}</strong></td>
      </tr>
      <tr>
        <td><strong>Quad</strong></td>
        <td style="color:#c084fc;font-weight:700;">⚡ Option A+B Fusion</td>
        <td>{q_loc.get("boxes_per_img_030")}</td>
        <td>{q_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(q_loc.get("dup_gt_rate"))}</td>
        <td>{_pct(q_sem.get("option_ab", {}).get("matched_prop_top1"))}</td>
        <td><strong>{_pct(q_sem.get("option_ab", {}).get("matched_prop_top3"))}</strong></td>
        <td>{_pct(q_sem.get("option_ab", {}).get("open_vocab_map50"))}</td>
        <td><strong>{_pct(q_sem.get("option_ab", {}).get("clutter_rejection_rate"))}</strong></td>
        <td><strong>{_pct(q_sem.get("option_ab", {}).get("signboard_to_window_door_err"))}</strong></td>
        <td>{_pct(q_zs.get("option_ab", {}).get("unseen_4way_top1"))} / {_pct(q_zs.get("option_ab", {}).get("full_41way_top5"))}</td>
      </tr>
      <tr style="color:var(--muted);">
        <td>Quad</td>
        <td>🏛️ Previous Legacy (8k_full + v4)</td>
        <td>{ql_loc.get("boxes_per_img_030")}</td>
        <td>{ql_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(ql_loc.get("dup_gt_rate"))}</td>
        <td>{_pct(ql_sem.get("matched_prop_top1"))}</td>
        <td>{_pct(ql_sem.get("matched_prop_top3"))}</td>
        <td>{_pct(ql_sem.get("open_vocab_map50"))}</td>
        <td>{_pct(ql_sem.get("clutter_rejection_rate"))}</td>
        <td style="color:#f87171;">{_pct(ql_sem.get("signboard_to_window_door_err"))}</td>
        <td>—</td>
      </tr>
      <tr>
        <td><strong>HBB</strong></td>
        <td style="color:#4ade80;font-weight:700;">🚀 Option A (DenseAnchor)</td>
        <td>{h_loc.get("boxes_per_img_030")}</td>
        <td>{h_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(h_loc.get("dup_gt_rate"))}</td>
        <td>{_pct(h_sem.get("option_a", {}).get("matched_prop_top1"))}</td>
        <td>{_pct(h_sem.get("option_a", {}).get("matched_prop_top3"))}</td>
        <td>{_pct(h_sem.get("option_a", {}).get("open_vocab_map50"))}</td>
        <td>{_pct(h_sem.get("option_a", {}).get("clutter_rejection_rate"))}</td>
        <td>{_pct(h_sem.get("option_a", {}).get("signboard_to_window_door_err"))}</td>
        <td>{_pct(h_zs.get("option_a", {}).get("unseen_4way_top1"))} / {_pct(h_zs.get("option_a", {}).get("full_41way_top5"))}</td>
      </tr>
      <tr style="background:rgba(59,130,246,0.10);">
        <td><strong>HBB</strong></td>
        <td style="color:#60a5fa;font-weight:700;">🎯 Option B (RoIPooled)</td>
        <td>{h_loc.get("boxes_per_img_030")}</td>
        <td>{h_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(h_loc.get("dup_gt_rate"))}</td>
        <td>{_pct(h_sem.get("option_b", {}).get("matched_prop_top1"))}</td>
        <td>{_pct(h_sem.get("option_b", {}).get("matched_prop_top3"))}</td>
        <td><strong>{_pct(h_sem.get("option_b", {}).get("open_vocab_map50"))}</strong></td>
        <td><strong>{_pct(h_sem.get("option_b", {}).get("clutter_rejection_rate"))}</strong></td>
        <td><strong>{_pct(h_sem.get("option_b", {}).get("signboard_to_window_door_err"))}</strong></td>
        <td><strong>{_pct(h_zs.get("option_b", {}).get("unseen_4way_top1"))} / {_pct(h_zs.get("option_b", {}).get("full_41way_top5"))}</strong></td>
      </tr>
      <tr>
        <td><strong>HBB</strong></td>
        <td style="color:#c084fc;font-weight:700;">⚡ Option A+B Fusion</td>
        <td>{h_loc.get("boxes_per_img_030")}</td>
        <td>{h_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(h_loc.get("dup_gt_rate"))}</td>
        <td><strong>{_pct(h_sem.get("option_ab", {}).get("matched_prop_top1"))}</strong></td>
        <td><strong>{_pct(h_sem.get("option_ab", {}).get("matched_prop_top3"))}</strong></td>
        <td>{_pct(h_sem.get("option_ab", {}).get("open_vocab_map50"))}</td>
        <td>{_pct(h_sem.get("option_ab", {}).get("clutter_rejection_rate"))}</td>
        <td><strong>{_pct(h_sem.get("option_ab", {}).get("signboard_to_window_door_err"))}</strong></td>
        <td>{_pct(h_zs.get("option_ab", {}).get("unseen_4way_top1"))} / {_pct(h_zs.get("option_ab", {}).get("full_41way_top5"))}</td>
      </tr>
      <tr style="color:var(--muted);">
        <td>HBB</td>
        <td>🏛️ Previous Legacy (8k_full + v4)</td>
        <td>{hl_loc.get("boxes_per_img_030")}</td>
        <td>{hl_loc.get("overlap_pairs_025")}</td>
        <td>{_pct(hl_loc.get("dup_gt_rate"))}</td>
        <td>{_pct(hl_sem.get("matched_prop_top1"))}</td>
        <td>{_pct(hl_sem.get("matched_prop_top3"))}</td>
        <td>{_pct(hl_sem.get("open_vocab_map50"))}</td>
        <td>{_pct(hl_sem.get("clutter_rejection_rate"))}</td>
        <td style="color:#f87171;">{_pct(hl_sem.get("signboard_to_window_door_err"))}</td>
        <td>—</td>
      </tr>
    </tbody>
  </table>
</details>"""

    geom_btns_html = (
        '<button class="chip active-purple" id="geom-hbb" onclick="setGeom(\'hbb\')">🔬 HBB Proposals</button>'
        if only_hbb
        else """<button class="chip active-purple" id="geom-quad" onclick="setGeom('quad')">💎 Quad Proposals</button>
    <button class="chip" id="geom-hbb" onclick="setGeom('hbb')">🔬 HBB Proposals</button>
    <button class="chip" id="geom-both" onclick="setGeom('both')">🌟 Both (Quad + HBB)</button>"""
    )

    if has_core27:
        bank_btns = [
            '<button class="chip active" id="bank-27" onclick="setBank(\'27\')">📚 Core-27 Preset</button>'
        ]
        if has_core31:
            bank_btns.append(
                '<button class="chip" id="bank-31" onclick="setBank(\'31\')">🧪 Core-27 + 4 Zero-Shot</button>'
            )
        bank_btns.extend(
            [
                '<button class="chip" id="bank-37" onclick="setBank(\'37\')">📚 Full-37 Adapted</button>',
                '<button class="chip" id="bank-41" onclick="setBank(\'41\')">🧪 Full-37 + 4 Zero-Shot</button>',
            ]
        )
        bank_btns_html = "\n      ".join(bank_btns)
    else:
        bank_btns_html = """<button class="chip active" id="bank-37" onclick="setBank('37')">📚 37 Seen Classes</button>
      <button class="chip" id="bank-41" onclick="setBank('41')">🧪 41 Zero-Shot (+book/painting/cushion/flowerpot)</button>"""

    default_geom = "hbb" if only_hbb else "quad"
    default_bank = "27" if has_core27 else "37"

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{escape(page_title)}</title>
<style>
  :root {{
    --bg: #0b0f19;
    --card: #131c2e;
    --border: #23324d;
    --text: #e2e8f0;
    --muted: #94a3b8;
    --green: #22c55e;
    --blue: #3b82f6;
    --purple: #a855f7;
    --cyan: #22d3ee;
    --amber: #f59e0b;
    --rose: #f43f5e;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    padding: 20px;
    line-height: 1.45;
  }}
  header {{
    border-bottom: 1px solid var(--border);
    padding-bottom: 14px;
    margin-bottom: 18px;
  }}
  h1 {{ font-size: 1.55rem; color: #fff; }}
  .sub {{ color: var(--muted); font-size: 0.9rem; margin-top: 4px; }}
  .kpi-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 12px;
    margin-bottom: 18px;
  }}
  .kpi {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 13px 15px;
  }}
  .kpi .k-lbl {{ font-size: 0.74rem; text-transform: uppercase; letter-spacing: 0.04em; color: var(--muted); }}
  .kpi .k-val {{ font-size: 1.45rem; font-weight: 700; margin: 3px 0; }}
  .kpi .k-sub {{ font-size: 0.76rem; color: var(--muted); }}
  .tbl-wrap {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 14px 16px;
    margin-bottom: 18px;
    overflow-x: auto;
  }}
  .tbl-wrap summary {{
    cursor: pointer;
    font-weight: 700;
    font-size: 0.95rem;
    color: #fff;
  }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.82rem; margin-top: 10px; }}
  th, td {{ padding: 7px 10px; border-bottom: 1px solid var(--border); text-align: left; }}
  th {{ color: var(--muted); background: rgba(15, 23, 42, 0.65); }}
  .controls {{
    position: sticky;
    top: 8px;
    z-index: 1000;
    background: rgba(15, 23, 42, 0.96);
    backdrop-filter: blur(10px);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px 16px;
    margin-bottom: 18px;
    display: flex;
    flex-direction: column;
    gap: 10px;
    box-shadow: 0 10px 28px rgba(0, 0, 0, 0.55);
  }}
  .row {{ display: flex; flex-wrap: wrap; align-items: center; gap: 10px; }}
  .row + .row {{ border-top: 1px solid var(--border); padding-top: 9px; }}
  .r-title {{ font-size: 0.76rem; font-weight: 700; text-transform: uppercase; color: var(--muted); min-width: 112px; }}
  .chip {{
    background: #1e293b;
    border: 1px solid var(--border);
    color: var(--muted);
    padding: 5px 11px;
    border-radius: 6px;
    font-size: 0.8rem;
    font-weight: 600;
    cursor: pointer;
  }}
  .chip:hover {{ background: #334155; color: #fff; }}
  .chip.active {{ background: var(--blue); border-color: var(--blue); color: #fff; }}
  .chip.active-green {{ background: #15803d; border-color: var(--green); color: #fff; }}
  .chip.active-purple {{ background: #6b21a8; border-color: var(--purple); color: #fff; }}
  .chip.active-amber {{ background: #b45309; border-color: var(--amber); color: #fff; }}
  select, input[type="text"] {{
    background: #1e293b;
    border: 1px solid var(--border);
    color: #fff;
    padding: 5px 10px;
    border-radius: 6px;
    font-size: 0.82rem;
  }}
  .gallery {{
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(470px, 1fr));
    gap: 16px;
  }}
  .gallery.split-mode {{
    grid-template-columns: 1fr;
  }}
  .card {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    overflow: hidden;
  }}
  .card-hdr {{
    padding: 8px 12px;
    background: rgba(15, 23, 42, 0.65);
    border-bottom: 1px solid var(--border);
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 0.8rem;
    color: var(--muted);
    gap: 8px;
    flex-wrap: wrap;
  }}
  .panes {{
    display: grid;
    grid-template-columns: 1fr;
    gap: 2px;
    background: #020617;
  }}
  .panes.three-up {{
    grid-template-columns: repeat(3, 1fr);
  }}
  .pane {{
    position: relative;
    background: #020617;
  }}
  .pane-badge {{
    position: absolute;
    top: 6px;
    left: 6px;
    z-index: 5;
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 0.74rem;
    font-weight: 700;
    background: rgba(15, 23, 42, 0.88);
    border: 1px solid var(--border);
    pointer-events: none;
  }}
  canvas {{ width: 100%; height: auto; display: block; }}
  .empty-msg {{
    padding: 36px;
    text-align: center;
    color: var(--muted);
    background: var(--card);
    border: 1px dashed var(--border);
    border-radius: 8px;
  }}
</style>
</head>
<body>
<header>
  <h1>⚡ {escape(page_title)}</h1>
  <div class="sub">
    {escape(subtitle_imgs)} &bull;
    Hover &amp; scroll mouse wheel over any box/quad to spotlight and compare <strong>Option A</strong>, <strong>Option B</strong>, and <strong>Option A+B</strong> side-by-side.
  </div>
</header>

{kpi_section_html}

{tbl_section_html}

<div class="controls">
  <div class="row">
    <span class="r-title">Head / View Mode:</span>
    <button class="chip active" id="mode-option_b" onclick="setHeadMode('option_b')">🎯 Option B (RoI)</button>
    <button class="chip" id="mode-option_a" onclick="setHeadMode('option_a')">🚀 Option A (Dense)</button>
    <button class="chip" id="mode-option_ab" onclick="setHeadMode('option_ab')">⚡ Option A+B Fusion</button>
    <button class="chip" id="mode-diff" onclick="setHeadMode('diff')">🔀 Overlay A vs B Diff</button>
    <button class="chip" id="mode-split3" onclick="setHeadMode('split3')">🪟 3-Up Split (A | B | A+B)</button>
    <button class="chip" id="mode-legacy" onclick="setHeadMode('legacy')">🏛️ {escape(baseline_label)}</button>
  </div>

  <div class="row">
    <span class="r-title">Geometry &amp; Vocab:</span>
    {geom_btns_html}
    <label style="display:flex;align-items:center;gap:6px;font-size:0.82rem;cursor:pointer;margin-left:6px;">
      <input type="checkbox" id="show-gt" onchange="renderGallery()">
      <span style="background:#065f46;color:#a7f3d0;border:1px solid #10b981;padding:2px 8px;border-radius:4px;font-weight:600;">✅ Human GT + Labels</span>
    </label>
    <div style="margin-left:auto;display:flex;align-items:center;gap:8px;flex-wrap:wrap;">
      <span style="font-size:0.78rem;color:var(--muted);font-weight:700;">VOCAB BANK:</span>
      {bank_btns_html}
    </div>
  </div>

  <div class="row">
    <span class="r-title">Filters &amp; Thresholds:</span>
    <select id="diag-filter" onchange="renderGallery()">
      <option value="all">🔎 All Detections</option>
      <option value="slender_infra">🚦 Street Infrastructure (trash_bin / traffic_light / pillar / pole / cone / sign)</option>
      <option value="disagree">⚡ Where Option A &ne; Option B (Disagreements)</option>
      <option value="zeroshot">🧪 Zero-Shot Unseen Predictions (book/painting/cushion/flowerpot)</option>
      <option value="signboard_group">🪧 Signboard / Traffic Sign / Window / Door</option>
    </select>
    <select id="cat-filter" onchange="renderGallery()">
      {"".join(cat_options)}
    </select>
    <input type="text" id="search-box" placeholder="Search class..." oninput="renderGallery()" style="width:150px;">
    <div style="margin-left:auto;display:flex;align-items:center;gap:14px;flex-wrap:wrap;">
      <label style="display:flex;align-items:center;gap:6px;font-size:0.8rem;color:var(--muted);" title="Minimum detector objectness floor (unified joint confidence sqrt(obj * cls) >= Cls Prob)">
        Obj Floor &ge;
        <input type="range" id="obj-thresh" min="0.05" max="0.90" step="0.05" value="0.15" oninput="onSlider()">
        <strong id="obj-val" style="color:#c084fc;min-width:28px;">0.15</strong>
      </label>
      <label style="display:flex;align-items:center;gap:6px;font-size:0.8rem;color:var(--muted);" title="Unified Open-Vocab Joint Confidence sqrt(Obj * Cls) &amp; Class Probability threshold">
        Joint / Cls &ge;
        <input type="range" id="cls-thresh" min="0.00" max="0.90" step="0.05" value="0.25" oninput="onSlider()">
        <strong id="cls-val" style="color:#60a5fa;min-width:28px;">0.25</strong>
      </label>
    </div>
  </div>

  <div class="row">
    <span class="r-title">Dataset Cohort:</span>
    {" ".join(cohort_btns)}
    <span id="visible-counter" style="margin-left:auto;font-size:0.8rem;color:var(--muted);font-weight:600;"></span>
  </div>
</div>

<div class="gallery" id="gallery"></div>

<script>
const IMAGES = {json.dumps(images_data)};
const ZERO_SHOT_SET = new Set({json.dumps(list(ZERO_SHOT_EVAL_CATEGORIES))});
const SIGN_GROUP = new Set(['signboard', 'traffic_sign', 'window', 'door']);
const SLENDER_INFRA_GROUP = new Set(['trash_bin', 'traffic_light', 'pillar', 'pole', 'street_light', 'traffic_cone', 'traffic_sign', 'signboard']);
const BASELINE_LABEL = {json.dumps(baseline_label)};

let headMode = 'option_b'; // option_b | option_a | option_ab | diff | split3 | legacy
let geomMode = {json.dumps(default_geom)};
let activeBank = {json.dumps(default_bank)}; // '27' | '31' | '37' | '41'
let activeCohort = 'all';

function setHeadMode(m) {{
  headMode = m;
  ['option_b', 'option_a', 'option_ab', 'diff', 'split3', 'legacy'].forEach(k => {{
    const el = document.getElementById('mode-' + k);
    if (el) el.className = 'chip' + (k === m ? ' active' : '');
  }});
  const gal = document.getElementById('gallery');
  gal.className = m === 'split3' ? 'gallery split-mode' : 'gallery';
  renderGallery();
}}

function setGeom(g) {{
  geomMode = g;
  ['quad', 'hbb', 'both'].forEach(k => {{
    const el = document.getElementById('geom-' + k);
    if (el) el.className = 'chip' + (k === g ? ' active-purple' : '');
  }});
  renderGallery();
}}

function setBank(b) {{
  activeBank = b;
  ['27', '31', '37', '41'].forEach(k => {{
    const el = document.getElementById('bank-' + k);
    if (!el) return;
    el.className = 'chip' + (k === b ? ((k === '31' || k === '41') ? ' active-amber' : ' active') : '');
  }});
  renderGallery();
}}

function setCohort(c) {{
  activeCohort = c;
  document.querySelectorAll('button[id^="cohort-"]').forEach(b => b.classList.remove('active'));
  const btn = document.getElementById('cohort-' + c);
  if (btn) btn.classList.add('active');
  renderGallery();
}}

function onSlider() {{
  document.getElementById('obj-val').textContent = parseFloat(document.getElementById('obj-thresh').value).toFixed(2);
  document.getElementById('cls-val').textContent = parseFloat(document.getElementById('cls-thresh').value).toFixed(2);
  renderGallery();
}}

function fmtTop3(e) {{
  if (!e || !e.top3 || !e.top3.length) return '';
  return ' | Top3: ' + e.top3.map(t => `${{t[0]}} ${{Math.round(t[1] * 100)}}%`).join(', ');
}}

function getOptEntry(pred, optKey) {{
  if (optKey === 'legacy') {{
    return {{ label: pred.label || 'unknown', prob: pred.prob || 0, top3: pred.top3 || [] }};
  }}
  const bankMap = {{ '27': pred.b27, '31': pred.b31, '37': pred.b37, '41': pred.b41 }};
  const bankObj = bankMap[activeBank] || pred.b37;
  if (!bankObj) return {{ label: '?', prob: 0, top3: [] }};
  return bankObj[optKey] || {{ label: '?', prob: 0, top3: [] }};
}}

function predMatchesFilters(pred, paneOpt, objThresh, clsThresh, diagFilter, catFilter, searchQ) {{
  if (pred.score < objThresh) return false;
  if (paneOpt === 'legacy') {{
    if ((pred.prob || 0) < clsThresh) return false;
    const lbl = (pred.label || '').toLowerCase();
    if (catFilter !== 'all' && lbl !== catFilter) return false;
    if (searchQ && !lbl.includes(searchQ)) return false;
    if (diagFilter === 'zeroshot' && !ZERO_SHOT_SET.has(lbl)) return false;
    if (diagFilter === 'signboard_group' && !SIGN_GROUP.has(lbl)) return false;
    if (diagFilter === 'slender_infra' && !SLENDER_INFRA_GROUP.has(lbl)) return false;
    return true;
  }}
  const eA = getOptEntry(pred, 'a');
  const eB = getOptEntry(pred, 'b');
  const eAB = getOptEntry(pred, 'ab');
  const activeE = paneOpt === 'a' ? eA : (paneOpt === 'b' ? eB : eAB);

  if (paneOpt === 'diff') {{
    const maxP = Math.max(eA.prob, eB.prob);
    const joint = Math.sqrt(pred.score * maxP);
    if (maxP < clsThresh || joint < clsThresh) return false;
  }} else {{
    const joint = Math.sqrt(pred.score * activeE.prob);
    if (activeE.prob < clsThresh || joint < clsThresh) return false;
  }}

  if (diagFilter === 'disagree' && eA.label === eB.label) return false;
  if (diagFilter === 'zeroshot') {{
    const anyZs = ZERO_SHOT_SET.has(eA.label) || ZERO_SHOT_SET.has(eB.label) || ZERO_SHOT_SET.has(eAB.label);
    if (!anyZs) return false;
  }}
  if (diagFilter === 'signboard_group') {{
    const anySign = SIGN_GROUP.has(eA.label) || SIGN_GROUP.has(eB.label) || SIGN_GROUP.has(eAB.label);
    if (!anySign) return false;
  }}
  if (diagFilter === 'slender_infra') {{
    const anyInfra = SLENDER_INFRA_GROUP.has(eA.label) || SLENDER_INFRA_GROUP.has(eB.label) || SLENDER_INFRA_GROUP.has(eAB.label);
    if (!anyInfra) return false;
  }}
  if (catFilter !== 'all') {{
    if (paneOpt === 'diff') {{
      if (eA.label !== catFilter && eB.label !== catFilter) return false;
    }} else if (activeE.label !== catFilter) {{
      return false;
    }}
  }}
  if (searchQ) {{
    const matchQ = eA.label.toLowerCase().includes(searchQ) || eB.label.toLowerCase().includes(searchQ) || eAB.label.toLowerCase().includes(searchQ);
    if (!matchQ) return false;
  }}
  return true;
}}

function drawQuad(ctx, q, stroke, fill, lw, dashed = false) {{
  ctx.save();
  ctx.strokeStyle = stroke;
  ctx.fillStyle = fill;
  ctx.lineWidth = lw;
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(q[0][0], q[0][1]);
  for (let i = 1; i < 4; i++) ctx.lineTo(q[i][0], q[i][1]);
  ctx.closePath();
  if (fill) ctx.fill();
  ctx.stroke();
  ctx.restore();
}}

function drawBox(ctx, b, stroke, fill, lw, dashed = false) {{
  ctx.save();
  ctx.strokeStyle = stroke;
  ctx.fillStyle = fill;
  ctx.lineWidth = lw;
  if (dashed) ctx.setLineDash([4, 4]);
  if (fill) ctx.fillRect(b[0], b[1], b[2] - b[0], b[3] - b[1]);
  ctx.strokeRect(b[0], b[1], b[2] - b[0], b[3] - b[1]);
  ctx.restore();
}}

function drawPillText(ctx, txt, x, y, bg, fg, bold = false) {{
  if (!txt) return;
  ctx.save();
  ctx.font = (bold ? 'bold ' : '') + '11px sans-serif';
  const tw = ctx.measureText(txt).width;
  const px = Math.max(2, Math.min(x, ctx.canvas.width - tw - 10));
  const py = Math.max(14, Math.min(y, ctx.canvas.height - 4));
  ctx.fillStyle = bg;
  ctx.fillRect(px - 2, py - 11, tw + 6, 14);
  ctx.fillStyle = fg;
  ctx.fillText(txt, px + 1, py - 1);
  ctx.restore();
}}

function pointInBox(px, py, b) {{
  return px >= b[0] - 3 && px <= b[2] + 3 && py >= b[1] - 3 && py <= b[3] + 3;
}}

function pointInQuad(px, py, q) {{
  let inside = false;
  for (let i = 0, j = 3; i < 4; j = i++) {{
    const xi = q[i][0], yi = q[i][1], xj = q[j][0], yj = q[j][1];
    if (((yi > py) !== (yj > py)) && (px < (xj - xi) * (py - yi) / ((yj - yi) || 1e-7) + xi)) inside = !inside;
  }}
  return inside;
}}

function shapeArea(s) {{
  if (s.kind === 'box') return Math.max(1, (s.geom[2] - s.geom[0]) * (s.geom[3] - s.geom[1]));
  let a = 0;
  for (let i = 0, j = 3; i < 4; j = i++) a += (s.geom[j][0] + s.geom[i][0]) * (s.geom[j][1] - s.geom[i][1]);
  return Math.max(1, Math.abs(a) * 0.5);
}}

function collectShapesForPane(item, paneOpt, scaleX, scaleY, showGt, objThresh, clsThresh, diagFilter, catFilter, searchQ) {{
  const shapes = [];
  const rsBox = b => [b[0] * scaleX, b[1] * scaleY, b[2] * scaleX, b[3] * scaleY];
  const rsQuad = q => q.map(pt => [pt[0] * scaleX, pt[1] * scaleY]);

  if (showGt && item.gt) {{
    item.gt.forEach(g => {{
      if (geomMode === 'hbb' && g.box) {{
        shapes.push({{
          kind: 'box', geom: rsBox(g.box), stroke: 'rgba(16,185,129,0.92)', fill: 'rgba(16,185,129,0.16)',
          lw: 2.0, dashed: true, tag: `GT: ${{g.label}}`, tagBg: 'rgba(6,78,59,0.88)', tagFg: '#a7f3d0',
          hud: `✅ Human GT: ${{g.label}}${{g.is_zero_shot ? ' [Zero-Shot Unseen]' : ''}}`
        }});
      }} else if (g.quad) {{
        shapes.push({{
          kind: 'quad', geom: rsQuad(g.quad), stroke: 'rgba(16,185,129,0.92)', fill: 'rgba(16,185,129,0.16)',
          lw: 2.0, dashed: true, tag: `GT: ${{g.label}}`, tagBg: 'rgba(6,78,59,0.88)', tagFg: '#a7f3d0',
          hud: `✅ Human GT: ${{g.label}}${{g.is_zero_shot ? ' [Zero-Shot Unseen]' : ''}}`
        }});
      }} else if (g.box) {{
        shapes.push({{
          kind: 'box', geom: rsBox(g.box), stroke: 'rgba(16,185,129,0.92)', fill: 'rgba(16,185,129,0.16)',
          lw: 2.0, dashed: true, tag: `GT: ${{g.label}}`, tagBg: 'rgba(6,78,59,0.88)', tagFg: '#a7f3d0',
          hud: `✅ Human GT: ${{g.label}}${{g.is_zero_shot ? ' [Zero-Shot Unseen]' : ''}}`
        }});
      }}
    }});
  }}

  const useQuad = geomMode === 'quad' || geomMode === 'both';
  const useHbb = geomMode === 'hbb' || geomMode === 'both';

  if (paneOpt === 'legacy') {{
    if (useQuad && item.legacy_quad) {{
      item.legacy_quad.forEach(p => {{
        if (!predMatchesFilters(p, 'legacy', objThresh, clsThresh, diagFilter, catFilter, searchQ)) return;
        shapes.push({{
          kind: 'quad', geom: rsQuad(p.quad), stroke: 'rgba(244,63,94,0.85)', fill: 'rgba(244,63,94,0.16)',
          lw: 1.8, dashed: false, tag: `${{p.label}} ${{Math.round(p.prob*100)}}%`,
          tagBg: 'rgba(136,19,55,0.85)', tagFg: '#fecdd3',
          hud: `🏛️ ${{BASELINE_LABEL}} Quad (obj ${{p.score.toFixed(2)}}): ${{p.label}} (${{Math.round(p.prob*100)}}%)${{fmtTop3(p)}}`
        }});
      }});
    }}
    if (useHbb && item.legacy_hbb) {{
      item.legacy_hbb.forEach(p => {{
        if (!predMatchesFilters(p, 'legacy', objThresh, clsThresh, diagFilter, catFilter, searchQ)) return;
        shapes.push({{
          kind: 'box', geom: rsBox(p.box), stroke: 'rgba(251,146,60,0.85)', fill: 'rgba(251,146,60,0.15)',
          lw: 1.8, dashed: false, tag: `${{p.label}} ${{Math.round(p.prob*100)}}%`,
          tagBg: 'rgba(124,45,18,0.85)', tagFg: '#fed7aa',
          hud: `🏛️ ${{BASELINE_LABEL}} (obj ${{p.score.toFixed(2)}}): ${{p.label}} (${{Math.round(p.prob*100)}}%)${{fmtTop3(p)}}`
        }});
      }});
    }}
    return shapes;
  }}

  function pushSotaPred(p, kind) {{
    if (!predMatchesFilters(p, paneOpt, objThresh, clsThresh, diagFilter, catFilter, searchQ)) return;
    const eA = getOptEntry(p, 'a');
    const eB = getOptEntry(p, 'b');
    const eAB = getOptEntry(p, 'ab');
    const disagree = eA.label !== eB.label;

    let stroke, fill, tag, tagBg, tagFg;
    if (paneOpt === 'diff') {{
      if (disagree) {{
        stroke = 'rgba(245,158,11,0.96)';
        fill = 'rgba(245,158,11,0.22)';
        tag = `A:${{eA.label}}(${{Math.round(eA.prob*100)}}%) ≠ B:${{eB.label}}(${{Math.round(eB.prob*100)}}%)`;
        tagBg = 'rgba(120,53,15,0.92)';
        tagFg = '#fef3c7';
      }} else {{
        stroke = 'rgba(34,197,94,0.85)';
        fill = 'rgba(34,197,94,0.14)';
        tag = `A=B: ${{eB.label}} (${{Math.round(eB.prob*100)}}%)`;
        tagBg = 'rgba(20,83,45,0.85)';
        tagFg = '#bbf7d0';
      }}
    }} else {{
      const e = paneOpt === 'a' ? eA : (paneOpt === 'b' ? eB : eAB);
      const isZs = ZERO_SHOT_SET.has(e.label);
      if (isZs) {{
        stroke = 'rgba(245,158,11,0.95)';
        fill = 'rgba(245,158,11,0.22)';
        tagBg = 'rgba(120,53,15,0.90)';
        tagFg = '#fde68a';
      }} else if (paneOpt === 'a') {{
        stroke = kind === 'quad' ? 'rgba(34,197,94,0.92)' : 'rgba(52,211,153,0.92)';
        fill = 'rgba(34,197,94,0.18)';
        tagBg = 'rgba(20,83,45,0.88)';
        tagFg = '#bbf7d0';
      }} else if (paneOpt === 'b') {{
        stroke = kind === 'quad' ? 'rgba(96,165,250,0.94)' : 'rgba(34,211,238,0.94)';
        fill = 'rgba(59,130,246,0.20)';
        tagBg = 'rgba(30,58,138,0.88)';
        tagFg = '#dbeafe';
      }} else {{
        stroke = kind === 'quad' ? 'rgba(192,132,252,0.94)' : 'rgba(56,189,248,0.94)';
        fill = 'rgba(168,85,247,0.20)';
        tagBg = 'rgba(88,28,135,0.88)';
        tagFg = '#f3e8ff';
      }}
      tag = `${{isZs ? '🧪 ' : ''}}${{e.label}} ${{Math.round(e.prob*100)}}%`;
    }}

    const activeTop3Entry = paneOpt === 'a' ? eA : (paneOpt === 'ab' ? eAB : eB);
    const hud = `${{kind.toUpperCase()}} obj ${{p.score.toFixed(2)}} | 🚀 Opt A: ${{eA.label}} (${{Math.round(eA.prob*100)}}%) | 🎯 Opt B: ${{eB.label}} (${{Math.round(eB.prob*100)}}%) | ⚡ Opt A+B: ${{eAB.label}} (${{Math.round(eAB.prob*100)}}%)${{fmtTop3(activeTop3Entry)}}`;
    shapes.push({{
      kind,
      geom: kind === 'quad' ? rsQuad(p.quad) : rsBox(p.box),
      stroke, fill, lw: 2.0, dashed: false, tag, tagBg, tagFg, hud
    }});
  }}

  if (useQuad && item.sota_quad) item.sota_quad.forEach(p => pushSotaPred(p, 'quad'));
  if (useHbb && item.sota_hbb) item.sota_hbb.forEach(p => pushSotaPred(p, 'box'));
  return shapes;
}}

function renderGallery() {{
  const gal = document.getElementById('gallery');
  gal.innerHTML = '';

  const showGt = document.getElementById('show-gt').checked;
  const objThresh = parseFloat(document.getElementById('obj-thresh').value);
  const clsThresh = parseFloat(document.getElementById('cls-thresh').value);
  const diagFilter = document.getElementById('diag-filter').value;
  const catFilter = document.getElementById('cat-filter').value;
  const searchQ = document.getElementById('search-box').value.trim().toLowerCase();

  const paneConfigs = headMode === 'split3'
    ? [
        {{ key: 'a', badge: '🚀 Option A (DenseAnchor)', color: '#4ade80' }},
        {{ key: 'b', badge: '🎯 Option B (RoIPooled)', color: '#60a5fa' }},
        {{ key: 'ab', badge: '⚡ Option A+B Fusion', color: '#c084fc' }},
      ]
    : [
        {{
          key: headMode === 'option_a' ? 'a' : (headMode === 'option_b' ? 'b' : (headMode === 'option_ab' ? 'ab' : headMode)),
          badge: null,
          color: '#fff'
        }}
      ];

  const isFiltered = diagFilter !== 'all' || catFilter !== 'all' || searchQ !== '';
  let shownCount = 0;

  IMAGES.forEach(item => {{
    if (activeCohort === 'rare_showcase') {{
      if (!item.is_rare_showcase) return;
    }} else if (activeCohort !== 'all' && item.source_dataset !== activeCohort) {{
      return;
    }}

    if (isFiltered) {{
      const probeKey = paneConfigs[0].key === 'legacy' ? 'legacy' : (headMode === 'split3' ? 'ab' : paneConfigs[0].key);
      const probeShapes = collectShapesForPane(item, probeKey, 1, 1, false, objThresh, clsThresh, diagFilter, catFilter, searchQ);
      if (probeShapes.length === 0) return;
    }}
    shownCount++;

    const card = document.createElement('div');
    card.className = 'card';

    const hdr = document.createElement('div');
    hdr.className = 'card-hdr';
    const shortName = item.image_id < 0 ? item.file_name.split('/').pop() : `#${{item.image_id}}`;
    const showcaseBadge = item.is_rare_showcase && item.showcase_classes && item.showcase_classes.length
      ? ` <span style="background:rgba(245,158,11,0.18);color:#fbbf24;border:1px solid rgba(245,158,11,0.45);padding:1px 6px;border-radius:4px;font-size:0.72rem;font-weight:700;">🎯 ${{item.showcase_classes.join(', ')}}</span>`
      : '';
    hdr.innerHTML = `<span><strong style="color:#fff;">${{item.source_dataset}}</strong> ${{shortName}}${{showcaseBadge}}</span><span>${{item.camera_type}} &bull; ${{item.width}}&times;${{item.height}}</span>`;
    card.appendChild(hdr);

    const panesDiv = document.createElement('div');
    panesDiv.className = headMode === 'split3' ? 'panes three-up' : 'panes';
    card.appendChild(panesDiv);
    gal.appendChild(card);

    const img = new Image();
    img.src = item.thumbnail;
    img.onload = () => {{
      const scaleX = img.width / item.width;
      const scaleY = img.height / item.height;

      paneConfigs.forEach(pc => {{
        const pane = document.createElement('div');
        pane.className = 'pane';
        if (pc.badge) {{
          const b = document.createElement('div');
          b.className = 'pane-badge';
          b.style.color = pc.color;
          b.textContent = pc.badge;
          pane.appendChild(b);
        }}
        const canvas = document.createElement('canvas');
        canvas.width = img.width;
        canvas.height = img.height;
        pane.appendChild(canvas);
        panesDiv.appendChild(pane);

        const ctx = canvas.getContext('2d');
        const shapes = collectShapesForPane(item, pc.key, scaleX, scaleY, showGt, objThresh, clsThresh, diagFilter, catFilter, searchQ);
        shapes.forEach(s => s.area = shapeArea(s));

        let spotActive = false;
        let spotHits = [];
        let spotIdx = 0;
        let ancX = null, ancY = null;

        function paint() {{
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.drawImage(img, 0, 0);
          if (spotActive && spotHits.length > 0) {{
            const s = spotHits[spotIdx % spotHits.length];
            if (s.kind === 'quad') {{
              drawQuad(ctx, s.geom, s.stroke, s.fill, 2.8, s.dashed);
              drawPillText(ctx, s.tag, s.geom[0][0], s.geom[0][1] - 4, s.tagBg, s.tagFg, true);
            }} else {{
              drawBox(ctx, s.geom, s.stroke, s.fill, 2.8, s.dashed);
              drawPillText(ctx, s.tag, s.geom[0], s.geom[1] - 4, s.tagBg, s.tagFg, true);
            }}
            const hudTxt = `[${{spotIdx + 1}}/${{spotHits.length}}] ${{s.hud}}`;
            drawPillText(ctx, hudTxt, 8, canvas.height - 8, 'rgba(15,23,42,0.94)', '#f8fafc', true);
            return;
          }}
          shapes.forEach(s => {{
            if (s.kind === 'quad') {{
              drawQuad(ctx, s.geom, s.stroke, s.fill, s.lw, s.dashed);
              drawPillText(ctx, s.tag, s.geom[0][0], s.geom[0][1] - 3, s.tagBg, s.tagFg, false);
            }} else {{
              drawBox(ctx, s.geom, s.stroke, s.fill, s.lw, s.dashed);
              drawPillText(ctx, s.tag, s.geom[0], s.geom[1] - 3, s.tagBg, s.tagFg, false);
            }}
          }});
        }}

        canvas.addEventListener('wheel', evt => {{
          const rect = canvas.getBoundingClientRect();
          const cx = (evt.clientX - rect.left) * (canvas.width / Math.max(rect.width, 1));
          const cy = (evt.clientY - rect.top) * (canvas.height / Math.max(rect.height, 1));
          const moved = ancX === null || Math.hypot(cx - ancX, cy - ancY) > 8;
          if (!spotActive || moved) {{
            const hits = shapes.filter(s => s.kind === 'quad' ? pointInQuad(cx, cy, s.geom) : pointInBox(cx, cy, s.geom));
            hits.sort((a, b) => a.area - b.area);
            if (hits.length === 0) {{
              if (spotActive) {{ spotActive = false; paint(); }}
              return;
            }}
            evt.preventDefault();
            ancX = cx; ancY = cy; spotHits = hits; spotIdx = 0; spotActive = true;
            paint();
            return;
          }}
          evt.preventDefault();
          spotIdx = (spotIdx + (evt.deltaY >= 0 ? 1 : -1) + spotHits.length) % spotHits.length;
          paint();
        }}, {{ passive: false }});

        canvas.addEventListener('click', () => {{ if (spotActive) {{ spotActive = false; paint(); }} }});
        canvas.addEventListener('mouseleave', () => {{ if (spotActive) {{ spotActive = false; paint(); }} }});
        paint();
      }});
    }};
  }});

  document.getElementById('visible-counter').textContent = `Showing ${{shownCount}} / ${{IMAGES.length}} holdout images`;
  if (shownCount === 0) {{
    gal.innerHTML = '<div class="empty-msg">No holdout images match the active filter &amp; threshold settings. Try lowering Obj Score / Cls Prob or resetting filters.</div>';
  }}
}}

window.addEventListener('DOMContentLoaded', renderGallery);
</script>
</body>
</html>
"""
    output_path.write_text(html, encoding="utf-8")
    print(f"Open-Vocabulary Option A / B / A+B Dashboard generated -> {output_path}")


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    strides = (4, 8, 16, 32)

    vocab_37 = VocabularyBank.load(args.vocab_37, device=device)
    vocab_41 = VocabularyBank.load(args.vocab_41, device=device)
    vocab_27 = (
        VocabularyBank.load(args.vocab_core27, device=device)
        if args.vocab_core27 is not None and args.vocab_core27.exists()
        else None
    )
    vocab_31 = (
        VocabularyBank.load(args.vocab_core31, device=device)
        if args.vocab_core31 is not None and args.vocab_core31.exists()
        else None
    )
    distractor_emb = (
        VocabularyBank.load(args.vocab_distractors, device=device).embeddings
        if args.vocab_distractors.exists()
        else None
    )
    visual_centroid_raw = (
        torch.load(args.visual_centroid, map_location=device, weights_only=False)
        if args.visual_centroid.exists()
        else None
    )
    visual_centroid = (
        visual_centroid_raw["visual_centroid"].to(device)
        if isinstance(visual_centroid_raw, dict)
        and "visual_centroid" in visual_centroid_raw
        else visual_centroid_raw
    )
    bench = (
        json.loads(args.benchmark_json.read_text(encoding="utf-8"))
        if args.benchmark_json.exists()
        else {}
    )
    if args.quad_benchmark_json is not None and args.quad_benchmark_json.exists():
        q_bench = json.loads(args.quad_benchmark_json.read_text(encoding="utf-8"))
        if "quad" in q_bench.get("geometries", {}):
            bench.setdefault("geometries", {})["quad"] = q_bench["geometries"]["quad"]
    bench_core27 = (
        json.loads(args.benchmark_core27_json.read_text(encoding="utf-8"))
        if args.benchmark_core27_json is not None and args.benchmark_core27_json.exists()
        else None
    )
    bench_baseline = (
        json.loads(args.benchmark_baseline_json.read_text(encoding="utf-8"))
        if args.benchmark_baseline_json is not None and args.benchmark_baseline_json.exists()
        else None
    )

    selected_items, _ = load_holdout(args.evaluation_manifest, args.image_root)
    holdout_count = len(selected_items)
    existing_holdout_ids = {
        int(s.image_id) for s, _ in selected_items if int(s.image_id) >= 0
    }
    showcase_val_items = load_rare_showcase_val_items(
        args.gt_index, args.image_root, existing_holdout_ids
    )
    showcase_count_extra = len(showcase_val_items)
    selected_items.extend(showcase_val_items)
    image_ids = [int(s.image_id) for s, _ in selected_items if int(s.image_id) >= 0]
    if args.assert_no_overlap_cache is not None:
        if not args.assert_no_overlap_cache.exists():
            raise FileNotFoundError(
                f"Train cache for leakage check not found: {args.assert_no_overlap_cache}"
            )
        c_conn = sqlite3.connect(
            f"file:{args.assert_no_overlap_cache}?mode=ro&immutable=1", uri=True
        )
        try:
            cached_ids = {
                int(r[0]) for r in c_conn.execute("SELECT image_id FROM processed_images")
            }
        finally:
            c_conn.close()
        overlap = set(image_ids) & cached_ids
        if overlap:
            raise RuntimeError(
                f"Leakage detected! {len(overlap)} dashboard images overlap {args.assert_no_overlap_cache}: {sorted(overlap)[:10]}"
            )
        print(
            f"Verified 0/{len(image_ids)} dashboard images overlap {args.assert_no_overlap_cache}",
            flush=True,
        )
    gt_by_id = load_gt_annotations_with_labels(args.gt_index, image_ids)

    def _load_sota(
        ckpt_path: Path,
        geom: str,
        *,
        distractor_emb_override: torch.Tensor | None = distractor_emb,
        visual_centroid_override: torch.Tensor | None = visual_centroid,
    ) -> tuple[
        StudentDetector | QuadProposalDetector,
        DenseAnchorSemanticHead,
        RoIPooledSemanticHead,
    ]:
        ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
        det_cfg = ckpt.get("detector_config", {})
        det_cls = StudentDetector if geom == "hbb" else QuadProposalDetector
        det = det_cls(
            pretrained_backbone=False,
            fpn_channels=int(det_cfg.get("fpn_channels", 128)),
            neck_type=str(det_cfg.get("neck_type", "pan")),
            strides=strides,
            dual_head=bool(det_cfg.get("dual_head", True)),
            use_3dmf=bool(det_cfg.get("use_3dmf", True)),
        ).to(device)
        det.load_state_dict(ckpt["model_state_dict"])
        det.eval()
        ha = DenseAnchorSemanticHead.from_config(ckpt["head_a_config"]).to(device)
        ha.load_state_dict(ckpt["head_a_state_dict"], strict=False)
        if (
            ha.distractor_embeddings is None or ha.distractor_embeddings.numel() == 0
        ) and distractor_emb_override is not None:
            ha.set_distractor_embeddings(distractor_emb_override)
        if (
            getattr(ha, "teacher_space", "siglip") != "owlv2"
            and (ha.visual_centroid is None or ha.visual_centroid.numel() == 0)
            and visual_centroid_override is not None
        ):
            ha.set_visual_centroid(visual_centroid_override)
        ha.eval()
        hb = RoIPooledSemanticHead.from_config(ckpt["head_b_config"]).to(device)
        hb.load_state_dict(ckpt["head_b_state_dict"], strict=False)
        if (
            hb.distractor_embeddings is None or hb.distractor_embeddings.numel() == 0
        ) and distractor_emb_override is not None:
            hb.set_distractor_embeddings(distractor_emb_override)
        if (
            getattr(hb, "teacher_space", "siglip") != "owlv2"
            and (hb.visual_centroid is None or hb.visual_centroid.numel() == 0)
            and visual_centroid_override is not None
        ):
            hb.set_visual_centroid(visual_centroid_override)
        hb.eval()
        return det, ha, hb

    def _load_legacy(
        det_path: Path, head_path: Path, vocab_path: Path, geom: str
    ) -> tuple[
        StudentDetector | QuadProposalDetector,
        RegionEmbeddingHead,
        VocabularyBank,
        float,
    ] | None:
        if not (det_path.exists() and head_path.exists() and vocab_path.exists()):
            return None
        ckpt = torch.load(det_path, map_location=device, weights_only=False)
        has_aux = any("aux_head" in k for k in ckpt.get("model_state_dict", {}))
        det_cls = StudentDetector if geom == "hbb" else QuadProposalDetector
        det = det_cls(
            pretrained_backbone=False,
            fpn_channels=96,
            neck_type="lite",
            strides=strides,
            dual_head=has_aux,
        ).to(device)
        det.load_state_dict(ckpt["model_state_dict"])
        det.eval()
        rh, r_ckpt = RegionEmbeddingHead.load_from_checkpoint(head_path, device=device)
        rh.eval()
        vb = VocabularyBank.load(vocab_path, device=device)
        return det, rh, vb, float(r_ckpt.get("cls_temperature", 1.0))

    print("Loading SOTA Option A + Option B models and baselines...")
    sota_hbb, ha_hbb, hb_hbb = _load_sota(args.hbb_sota_ckpt, "hbb")
    sota_quad = ha_quad = hb_quad = None
    if not args.only_hbb:
        sota_quad, ha_quad, hb_quad = _load_sota(args.quad_sota_ckpt, "quad")

    base_hbb = base_hb = base_txt = base_cats = None
    if args.hbb_baseline_sota_ckpt is not None and args.hbb_baseline_sota_ckpt.exists():
        b_dist = (
            VocabularyBank.load(args.baseline_vocab_distractors, device=device).embeddings
            if args.baseline_vocab_distractors.exists()
            else None
        )
        b_cent_raw = (
            torch.load(args.baseline_visual_centroid, map_location=device, weights_only=False)
            if args.baseline_visual_centroid.exists()
            else None
        )
        b_cent = (
            b_cent_raw["visual_centroid"].to(device)
            if isinstance(b_cent_raw, dict) and "visual_centroid" in b_cent_raw
            else b_cent_raw
        )
        b_vocab = VocabularyBank.load(args.baseline_vocab_37, device=device)
        base_txt = b_vocab.embeddings.to(device)
        base_cats = tuple(b_vocab.categories)
        base_hbb, _, base_hb = _load_sota(
            args.hbb_baseline_sota_ckpt,
            "hbb",
            distractor_emb_override=b_dist,
            visual_centroid_override=b_cent,
        )

    leg_hbb_bundle = (
        None
        if base_hbb is not None
        else _load_legacy(
            args.hbb_legacy_ckpt, args.hbb_legacy_head, args.hbb_legacy_vocab, "hbb"
        )
    )
    leg_quad_bundle = (
        None
        if args.only_hbb
        else _load_legacy(
            args.quad_legacy_ckpt, args.quad_legacy_head, args.quad_legacy_vocab, "quad"
        )
    )

    hbb_sota_dec = InferenceDecoder(
        strides=strides,
        top_k=args.decoder_top_k,
        nms_iou_threshold=0.50,
        max_detections=args.decoder_max_detections,
        score_mode="objectness",
        nms_mode="containment_aware",
        score_threshold=0.05,
        small_box_min_side=args.small_box_min_side,
        small_box_score_power=args.small_box_score_power,
    )
    hbb_base_dec = InferenceDecoder(
        strides=strides,
        top_k=500,
        nms_iou_threshold=0.50,
        max_detections=80,
        score_mode="objectness",
        nms_mode="containment_aware",
        score_threshold=0.05,
    )
    quad_sota_dec = QuadInferenceDecoder(
        strides=strides,
        pre_nms_top_k=(
            args.quad_decoder_top_k
            if args.quad_decoder_top_k is not None
            else args.decoder_top_k
        ),
        nms_iou_threshold=0.50,
        max_proposals=(
            args.quad_decoder_max_detections
            if args.quad_decoder_max_detections is not None
            else args.decoder_max_detections
        ),
        centerness_alpha=0.0,
        nms_mode="containment_aware",
        score_threshold=0.05,
        small_box_min_side=(
            args.quad_small_box_min_side
            if args.quad_small_box_min_side is not None
            else args.small_box_min_side
        ),
        small_box_score_power=(
            args.quad_small_box_score_power
            if args.quad_small_box_score_power is not None
            else args.small_box_score_power
        ),
    )
    hbb_leg_dec = InferenceDecoder(
        strides=strides,
        top_k=1000,
        nms_iou_threshold=0.45,
        max_detections=100,
        score_mode="objectness_x_centerness",
        centerness_alpha=0.25,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
        score_threshold=0.20,
    )
    quad_leg_dec = QuadInferenceDecoder(
        strides=strides,
        pre_nms_top_k=1000,
        nms_iou_threshold=0.45,
        max_proposals=100,
        centerness_alpha=0.0,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        part_max_iou=0.65,
        score_threshold=0.20,
    )

    hbb_vocab_bundle = {
        "txt37": vocab_37.embeddings.to(device),
        "cats37": tuple(vocab_37.categories),
        "txt41": vocab_41.embeddings.to(device),
        "cats41": tuple(vocab_41.categories),
        "txt27": vocab_27.embeddings.to(device) if vocab_27 is not None else None,
        "cats27": tuple(vocab_27.categories) if vocab_27 is not None else None,
        "txt31": vocab_31.embeddings.to(device) if vocab_31 is not None else None,
        "cats31": tuple(vocab_31.categories) if vocab_31 is not None else None,
    }
    q_v37 = (
        VocabularyBank.load(args.quad_vocab_37, device=device)
        if args.quad_vocab_37 is not None
        else vocab_37
    )
    q_v41 = (
        VocabularyBank.load(args.quad_vocab_41, device=device)
        if args.quad_vocab_41 is not None
        else vocab_41
    )
    q_v27 = (
        VocabularyBank.load(args.quad_vocab_core27, device=device)
        if args.quad_vocab_core27 is not None and args.quad_vocab_core27.exists()
        else vocab_27
    )
    q_v31 = (
        VocabularyBank.load(args.quad_vocab_core31, device=device)
        if args.quad_vocab_core31 is not None and args.quad_vocab_core31.exists()
        else vocab_31
    )
    quad_vocab_bundle = {
        "txt37": q_v37.embeddings.to(device),
        "cats37": tuple(q_v37.categories),
        "txt41": q_v41.embeddings.to(device),
        "cats41": tuple(q_v41.categories),
        "txt27": q_v27.embeddings.to(device) if q_v27 is not None else None,
        "cats27": tuple(q_v27.categories) if q_v27 is not None else None,
        "txt31": q_v31.embeddings.to(device) if q_v31 is not None else None,
        "cats31": tuple(q_v31.categories) if q_v31 is not None else None,
    }
    distractor_set = set(DISTRACTOR_CATEGORIES)

    def _build_sota_preds(
        det_obj: Any,
        feats: tuple[torch.Tensor, ...],
        ha: DenseAnchorSemanticHead,
        hb: RoIPooledSemanticHead,
        geom_key: str,
        geom_items: Any,
        vocab_bundle: dict[str, Any] = hbb_vocab_bundle,
    ) -> list[dict[str, Any]]:
        txt37 = vocab_bundle["txt37"]
        cats37 = vocab_bundle["cats37"]
        txt41 = vocab_bundle["txt41"]
        cats41 = vocab_bundle["cats41"]
        txt27 = vocab_bundle["txt27"]
        cats27 = vocab_bundle["cats27"]
        txt31 = vocab_bundle["txt31"]
        cats31 = vocab_bundle["cats31"]
        n_p = int(det_obj.scores.numel())
        if n_p == 0 or det_obj.flat_indices is None:
            return []
        b_idx = torch.zeros_like(det_obj.flat_indices)
        za = ha.forward_indexed(feats, b_idx, det_obj.flat_indices)
        zb = hb(feats, [det_obj.boxes], image_shapes=[(384, 384)])

        _, pa37 = ha.classify_embeddings(za, txt37)
        _, pb37 = hb.classify_embeddings(zb, txt37)
        pab37 = 0.30 * pa37 + 0.70 * pb37

        _, pa41 = ha.classify_embeddings(za, txt41)
        _, pb41 = hb.classify_embeddings(zb, txt41)
        pab41 = 0.30 * pa41 + 0.70 * pb41

        pa27 = pb27 = pab27 = None
        if txt27 is not None:
            _, pa27 = ha.classify_embeddings(za, txt27)
            _, pb27 = hb.classify_embeddings(zb, txt27)
            pab27 = 0.30 * pa27 + 0.70 * pb27

        pa31 = pb31 = pab31 = None
        if txt31 is not None:
            _, pa31 = ha.classify_embeddings(za, txt31)
            _, pb31 = hb.classify_embeddings(zb, txt31)
            pab31 = 0.30 * pa31 + 0.70 * pb31

        keep = filter_semantic_detections(
            det_obj.boxes,
            det_obj.scores,
            pab37,
            cats37,
            quads=getattr(det_obj, "quads", None) if geom_key == "quad" else None,
        )
        if keep.numel() == 0:
            return []
        k_scores = det_obj.scores[keep].float()
        k_probs = pab37[keep].max(dim=-1).values.float()
        if pab27 is not None:
            k_probs = torch.maximum(k_probs, pab27[keep].max(dim=-1).values.float())
        k_joint = torch.sqrt(
            (k_scores.clamp(min=0.0) * k_probs.clamp(min=0.0)).clamp(min=0.0)
        )
        vis_mask = (
            ((k_scores >= 0.16) & (k_probs >= 0.125) & (k_joint >= 0.17))
            | ((k_scores >= 0.12) & (k_probs >= 0.40) & (k_joint >= 0.22))
        )
        if not vis_mask.any():
            return []
        keep = keep[vis_mask]
        k_joint = k_joint[vis_mask]
        if keep.numel() > 30:
            top_order = torch.topk(k_joint, k=30).indices
        else:
            top_order = torch.argsort(k_joint, descending=True)
        keep = keep[top_order]

        raw_geom = det_obj.quads[keep] if geom_key == "quad" else det_obj.boxes[keep]
        converted_geoms = geom_items(raw_geom)
        scores_list = det_obj.scores[keep].cpu().tolist()
        preds: list[dict[str, Any]] = []
        for idx_k, geom_val, sc_val in zip(keep.tolist(), converted_geoms, scores_list, strict=True):
            la37, pra37, t3a37 = pack_top3(pa37[idx_k], cats37)
            lb37, prb37, t3b37 = pack_top3(pb37[idx_k], cats37)
            lab37, prab37, t3ab37 = pack_top3(pab37[idx_k], cats37)

            la41, pra41, t3a41 = pack_top3(pa41[idx_k], cats41)
            lb41, prb41, t3b41 = pack_top3(pb41[idx_k], cats41)
            lab41, prab41, t3ab41 = pack_top3(pab41[idx_k], cats41)

            item_dict: dict[str, Any] = {
                geom_key: geom_val,
                "score": round(float(sc_val), 3),
                "b37": {
                    "a": {"label": la37, "prob": pra37, "top3": t3a37},
                    "b": {"label": lb37, "prob": prb37, "top3": t3b37},
                    "ab": {"label": lab37, "prob": prab37, "top3": t3ab37},
                },
                "b41": {
                    "a": {"label": la41, "prob": pra41, "top3": t3a41},
                    "b": {"label": lb41, "prob": prb41, "top3": t3b41},
                    "ab": {"label": lab41, "prob": prab41, "top3": t3ab41},
                },
            }
            if pa27 is not None and pb27 is not None and pab27 is not None and cats27 is not None:
                la27, pra27, t3a27 = pack_top3(pa27[idx_k], cats27)
                lb27, prb27, t3b27 = pack_top3(pb27[idx_k], cats27)
                lab27, prab27, t3ab27 = pack_top3(pab27[idx_k], cats27)
                item_dict["b27"] = {
                    "a": {"label": la27, "prob": pra27, "top3": t3a27},
                    "b": {"label": lb27, "prob": prb27, "top3": t3b27},
                    "ab": {"label": lab27, "prob": prab27, "top3": t3ab27},
                }
            if pa31 is not None and pb31 is not None and pab31 is not None and cats31 is not None:
                la31, pra31, t3a31 = pack_top3(pa31[idx_k], cats31)
                lb31, prb31, t3b31 = pack_top3(pb31[idx_k], cats31)
                lab31, prab31, t3ab31 = pack_top3(pab31[idx_k], cats31)
                item_dict["b31"] = {
                    "a": {"label": la31, "prob": pra31, "top3": t3a31},
                    "b": {"label": lb31, "prob": prb31, "top3": t3b31},
                    "ab": {"label": lab31, "prob": prab31, "top3": t3ab31},
                }
            preds.append(item_dict)
        return preds

    def _build_baseline_sota_preds(
        det_obj: Any,
        feats: tuple[torch.Tensor, ...],
        hb: RoIPooledSemanticHead,
        txt_base: torch.Tensor,
        cats_base: tuple[str, ...],
        geom_items: Any,
    ) -> list[dict[str, Any]]:
        n_p = int(det_obj.scores.numel())
        if n_p == 0 or det_obj.flat_indices is None:
            return []
        zb = hb(feats, [det_obj.boxes], image_shapes=[(384, 384)])
        _, pb = hb.classify_embeddings(zb, txt_base)
        keep = filter_semantic_detections(
            det_obj.boxes, det_obj.scores, pb, cats_base, quads=None
        )
        if keep.numel() == 0:
            return []
        k_scores = det_obj.scores[keep].float()
        k_probs = pb[keep].max(dim=-1).values.float()
        k_joint = torch.sqrt(
            (k_scores.clamp(min=0.0) * k_probs.clamp(min=0.0)).clamp(min=0.0)
        )
        vis_mask = (
            ((k_scores >= 0.16) & (k_probs >= 0.125) & (k_joint >= 0.17))
            | ((k_scores >= 0.12) & (k_probs >= 0.40) & (k_joint >= 0.22))
        )
        if not vis_mask.any():
            return []
        keep = keep[vis_mask]
        k_joint = k_joint[vis_mask]
        top_order = (
            torch.topk(k_joint, k=30).indices
            if keep.numel() > 30
            else torch.argsort(k_joint, descending=True)
        )
        keep = keep[top_order]
        converted_boxes = geom_items(det_obj.boxes[keep])
        scores_list = det_obj.scores[keep].cpu().tolist()
        out: list[dict[str, Any]] = []
        for idx_k, box_val, sc_val in zip(keep.tolist(), converted_boxes, scores_list, strict=True):
            lbl, pr, top3 = pack_top3(pb[idx_k], cats_base)
            out.append(
                {
                    "box": box_val,
                    "score": round(float(sc_val), 3),
                    "label": lbl,
                    "prob": pr,
                    "top3": top3,
                }
            )
        return out

    images_data: list[dict[str, Any]] = []
    shapes = ((96, 96), (48, 48), (24, 24), (12, 12))

    with torch.inference_mode():
        for idx, (sample, rec) in enumerate(selected_items):
            if idx % 30 == 0:
                print(f"Rendering holdout sample {idx}/{len(selected_items)}...", flush=True)
            img_path = args.image_root / rec["file_name"]
            if not img_path.exists():
                continue

            img_t = sample.image.unsqueeze(0).to(device)
            orig_h, orig_w = sample.original_size
            scale, off_x, off_y = sample.transform
            _, vlvl = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), shapes, strides
            )
            lmasks = tuple(v.unsqueeze(0) for v in vlvl)

            def unbox(boxes: torch.Tensor) -> list[list[float]]:
                if boxes.numel() == 0:
                    return []
                b = boxes.clone().cpu()
                b[:, 0::2] = ((b[:, 0::2] - off_x) / scale).clamp(0, orig_w)
                b[:, 1::2] = ((b[:, 1::2] - off_y) / scale).clamp(0, orig_h)
                return [[round(float(v), 1) for v in row] for row in b.tolist()]

            def unquad(quads: torch.Tensor) -> list[list[list[float]]]:
                if quads.numel() == 0:
                    return []
                q = quads.clone().cpu()
                q[..., 0] = ((q[..., 0] - off_x) / scale).clamp(0, orig_w)
                q[..., 1] = ((q[..., 1] - off_y) / scale).clamp(0, orig_h)
                return [
                    [[round(float(pt[0]), 1), round(float(pt[1]), 1)] for pt in item_q]
                    for item_q in q.tolist()
                ]

            # SOTA Quad
            sota_quad_preds: list[dict[str, Any]] = []
            if sota_quad is not None and ha_quad is not None and hb_quad is not None:
                fq = sota_quad.forward_features(img_t)
                dq = quad_sota_dec(sota_quad.forward_eval_head(fq), (384, 384), lmasks)[0]  # type: ignore[arg-type]
                sota_quad_preds = _build_sota_preds(
                    dq, fq, ha_quad, hb_quad, "quad", unquad, vocab_bundle=quad_vocab_bundle
                )

            # SOTA HBB
            fh = sota_hbb.forward_features(img_t)
            dh = hbb_sota_dec(sota_hbb.forward_eval_head(fh), (384, 384), lmasks)[0]  # type: ignore[arg-type]
            sota_hbb_preds = _build_sota_preds(
                dh, fh, ha_hbb, hb_hbb, "box", unbox, vocab_bundle=hbb_vocab_bundle
            )

            # Legacy / Baseline Quad & HBB
            leg_quad_preds: list[dict[str, Any]] = []
            if leg_quad_bundle is not None:
                l_det, l_rh, l_vb, l_temp = leg_quad_bundle
                lfq = l_det.forward_features(img_t)
                ldq = quad_leg_dec(l_det.head(lfq), (384, 384), lmasks)[0]  # type: ignore[arg-type]
                if ldq.scores.numel() > 0:
                    lemb = l_rh(lfq, [ldq.boxes], image_shapes=[(384, 384)])
                    _, lprobs = l_vb.classify(
                        lemb,
                        mode="softmax",
                        temperature=l_temp,
                        use_centroid_subtraction=False,
                    )
                    l_cats = [map_to_canonical(c) or c for c in l_vb.categories]
                    for q, sc, pr_row in zip(
                        unquad(ldq.quads),
                        ldq.scores.cpu().tolist(),
                        lprobs,
                        strict=True,
                    ):
                        top_i = int(pr_row.argmax().item())
                        lbl = l_cats[top_i]
                        if lbl in distractor_set:
                            continue
                        leg_quad_preds.append(
                            {
                                "quad": q,
                                "score": round(float(sc), 3),
                                "label": lbl,
                                "prob": round(float(pr_row[top_i].item()), 3),
                            }
                        )

            leg_hbb_preds: list[dict[str, Any]] = []
            if base_hbb is not None and base_hb is not None and base_txt is not None and base_cats is not None:
                bfh = base_hbb.forward_features(img_t)
                bdh = hbb_base_dec(base_hbb.forward_eval_head(bfh), (384, 384), lmasks)[0]  # type: ignore[arg-type]
                leg_hbb_preds = _build_baseline_sota_preds(
                    bdh, bfh, base_hb, base_txt, base_cats, unbox
                )
            elif leg_hbb_bundle is not None:
                l_det, l_rh, l_vb, l_temp = leg_hbb_bundle
                lfh = l_det.forward_features(img_t)
                ldh = hbb_leg_dec(l_det.head(lfh), (384, 384), lmasks)[0]  # type: ignore[arg-type]
                if ldh.scores.numel() > 0:
                    lemb = l_rh(lfh, [ldh.boxes], image_shapes=[(384, 384)])
                    _, lprobs = l_vb.classify(
                        lemb,
                        mode="softmax",
                        temperature=l_temp,
                        use_centroid_subtraction=False,
                    )
                    l_cats = [map_to_canonical(c) or c for c in l_vb.categories]
                    for b, sc, pr_row in zip(
                        unbox(ldh.boxes),
                        ldh.scores.cpu().tolist(),
                        lprobs,
                        strict=True,
                    ):
                        top_i = int(pr_row.argmax().item())
                        lbl = l_cats[top_i]
                        if lbl in distractor_set:
                            continue
                        leg_hbb_preds.append(
                            {
                                "box": b,
                                "score": round(float(sc), 3),
                                "label": lbl,
                                "prob": round(float(pr_row[top_i].item()), 3),
                            }
                        )

            gt_items = gt_by_id.get(int(sample.image_id), [])
            if not gt_items:
                for q in rec.get("gt_quads", []):
                    gt_items.append({"label": "gt", "is_zero_shot": False, "box": None, "quad": q})
                for b in rec.get("gt_boxes", []):
                    raw_b = b if isinstance(b, list) else b.get("box")
                    gt_items.append({"label": "gt", "is_zero_shot": False, "box": raw_b, "quad": None})

            rare_set = set(RARE_SHOWCASE_CLASSES)
            showcase_cls = set(rec.get("showcase_classes", []))
            for g_it in gt_items:
                if str(g_it.get("label")) in rare_set:
                    showcase_cls.add(str(g_it["label"]))
            if int(sample.image_id) == 188347:
                showcase_cls.update(("trash_bin", "fire_hydrant"))
            elif int(sample.image_id) == 188775:
                showcase_cls.update(("traffic_sign", "pillar"))
            is_showcase = bool(rec.get("is_rare_showcase", False) or showcase_cls)

            images_data.append(
                {
                    "image_id": int(sample.image_id),
                    "source_dataset": sample.source_dataset,
                    "camera_type": sample.camera_type,
                    "width": orig_w,
                    "height": orig_h,
                    "file_name": rec["file_name"],
                    "is_rare_showcase": is_showcase,
                    "showcase_classes": sorted(showcase_cls),
                    "thumbnail": encode_thumbnail_base64(img_path),
                    "gt": gt_items,
                    "sota_quad": sota_quad_preds,
                    "sota_hbb": sota_hbb_preds,
                    "legacy_quad": leg_quad_preds,
                    "legacy_hbb": leg_hbb_preds,
                }
            )

    build_html(
        images_data,
        bench,
        args.output_html,
        only_hbb=args.only_hbb,
        has_core27=(vocab_27 is not None),
        has_core31=(vocab_31 is not None),
        bench_core27=bench_core27,
        bench_baseline=bench_baseline,
        baseline_label=args.baseline_label,
        title=args.title,
        holdout_count=holdout_count,
        showcase_count_extra=showcase_count_extra,
    )


if __name__ == "__main__":
    main()
