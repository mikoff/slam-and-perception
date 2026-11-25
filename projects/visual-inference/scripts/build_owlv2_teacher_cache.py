#!/usr/bin/env python3
"""Phase 1: Resumable OWLv2-Large auto-labeling + 768-D embedding cache builder.

Builds a leak-free, balanced 12k training cache (`artifacts/cache/owlv2_teacher_12k.sqlite`)
with:
  - Native OWLv2-Large 768-D VocabularyBanks (37-seen, 41-zeroshot, 7-distractors).
  - Non-destructive annotation hygiene (`annotation_hygiene.py`).
  - Pedestrian crowd-split replacement + missing-class OWLv2 pseudo-labels +
    matched 768-D fp16 `class_embeds` extracted in a single batched pass.
  - Visual centroid, quantitative QA benchmark (`owlv2_teacher_12k_qa_summary.json`),
    interactive QA viewer (`owlv2_teacher_12k_qa.html`), and composite QA PNG sheets.
"""

from __future__ import annotations

import argparse
import base64
from collections import defaultdict
from html import escape
import io
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch
from torch.utils.data import DataLoader, Dataset
from transformers import Owlv2ImageProcessor

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
)
from student_detector.ontology import (
    CORE_27_CATEGORIES,
    DISTRACTOR_CATEGORIES,
    OPEN_VOCAB_CATEGORIES,
    ZERO_SHOT_EVAL_CATEGORIES,
    map_to_canonical,
)
from student_detector.owlv2_teacher import (
    OWLV2_MODEL_ID,
    OWLV2_MODEL_REVISION,
    OwlDetection,
    OwlTeacherCacheReader,
    OwlTeacherCacheWriter,
    Owlv2TeacherExtractor,
    canonicalize_gt_labels,
    merge_clean_gt_and_owlv2,
)
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT
from student_detector.vocabulary import VocabularyBank

BASE_12K_QUOTAS: dict[str, int] = {
    "nuimages": 2400,
    "bdd100k_images_100k": 2400,
    "woodscape_rgb_fisheye": 1800,
    "ade20k": 1800,
    "open_images_subset": 1800,
    "coco_2017": 1800,
}

BASE_30K_QUOTAS: dict[str, int] = {
    "nuimages": 6000,
    "bdd100k_images_100k": 6000,
    "coco_2017": 5000,
    "open_images_subset": 4500,
    "ade20k": 4500,
    "woodscape_rgb_fisheye": 4000,
}

SCARCE_CLASS_QUOTAS_30K: tuple[tuple[str, dict[str, int]], ...] = (
    ("stroller", {"nuimages": 300}),
    ("wheelchair", {"open_images_subset": 200, "nuimages": 50}),
    ("trash_bin", {"open_images_subset": 150, "ade20k": 700}),
    ("fire_hydrant", {"open_images_subset": 100, "coco_2017": 900}),
    ("pillar", {"ade20k": 850}),
    ("signboard", {"open_images_subset": 100, "ade20k": 1400}),
    ("bench", {"open_images_subset": 50, "ade20k": 700, "coco_2017": 900}),
    (
        "bus",
        {
            "open_images_subset": 400,
            "ade20k": 300,
            "woodscape_rgb_fisheye": 800,
            "nuimages": 900,
            "bdd100k_images_100k": 900,
            "coco_2017": 700,
        },
    ),
    (
        "truck",
        {
            "open_images_subset": 950,
            "ade20k": 450,
            "woodscape_rgb_fisheye": 900,
            "nuimages": 1000,
            "bdd100k_images_100k": 1000,
            "coco_2017": 700,
        },
    ),
    ("dog", {"coco_2017": 800}),
    ("license_plate", {"open_images_subset": 1500}),
    (
        "luggage_and_bags",
        {"open_images_subset": 250, "ade20k": 550, "coco_2017": 700},
    ),
    ("construction_barrier", {"nuimages": 800}),
    ("traffic_cone", {"nuimages": 800}),
    ("fence", {"ade20k": 700, "woodscape_rgb_fisheye": 800}),
)

DATASET_ORDER: tuple[str, ...] = tuple(BASE_12K_QUOTAS.keys())


def _collect_exclusions(
    gt_index: Path,
    image_root: Path,
    holdout_manifest: Path,
) -> tuple[set[int], set[str]]:
    """Collect all excluded validation & holdout image IDs and resolved canonical paths."""
    excluded_ids: set[int] = set()
    excluded_paths: set[str] = set()

    if holdout_manifest.exists():
        raw = json.loads(holdout_manifest.read_text())
        for key in ("images", "excluded_validation_images"):
            for rec in raw.get(key, []):
                if rec.get("image_id") is not None:
                    excluded_ids.add(int(rec["image_id"]))
                fn = rec.get("file_name")
                if fn:
                    excluded_paths.add(str((image_root / str(fn)).resolve()))

    val_ds = HumanGTHBBDataset(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="val",
        cohort_size=8000,
        sanitize_annotations=False,
    )
    for rec in val_ds.images:
        excluded_ids.add(int(rec["image_id"]))
        excluded_paths.add(str((image_root / str(rec["file_name"])).resolve()))

    return excluded_ids, excluded_paths


def _sample_even_strided(
    candidates: list[dict[str, Any]],
    count: int,
) -> list[dict[str, Any]]:
    """Deterministically pick `count` evenly spaced records from a sorted pool."""
    if count <= 0 or not candidates:
        return []
    if count >= len(candidates):
        return list(candidates)
    return [candidates[int(i * len(candidates) / count)] for i in range(count)]


def _query_scarce_class_candidates(
    conn: sqlite3.Connection,
    meta_by_id: dict[int, dict[str, Any]],
) -> dict[str, dict[str, dict[int, float]]]:
    """Map scarce_class -> source_dataset -> {image_id: max_relative_box_area}."""
    scarce_classes = {cls_name for cls_name, _ in SCARCE_CLASS_QUOTAS_30K}
    raw_names = [
        str(r[0])
        for r in conn.execute(
            "SELECT DISTINCT category_name FROM annotations WHERE ignore_region = 0"
        ).fetchall()
    ]
    raw_to_scarce = {
        rn: canon
        for rn in raw_names
        if (canon := map_to_canonical(rn)) in scarce_classes
    }
    cls_ds_rel: dict[str, dict[str, dict[int, float]]] = defaultdict(
        lambda: defaultdict(dict)
    )
    if not raw_to_scarce:
        return cls_ds_rel

    placeholders = ",".join("?" for _ in raw_to_scarce)
    rows = conn.execute(
        f"""
        SELECT a.image_id, a.category_name, MAX((a.x2 - a.x1) * (a.y2 - a.y1))
        FROM annotations a
        WHERE a.ignore_region = 0 AND a.category_name IN ({placeholders})
        GROUP BY a.image_id, a.category_name
        """,
        list(raw_to_scarce.keys()),
    ).fetchall()
    for img_id_raw, rc_raw, max_area in rows:
        iid = int(img_id_raw)
        rec = meta_by_id.get(iid)
        if rec is None:
            continue
        canon = raw_to_scarce[str(rc_raw)]
        img_area = max(float(rec["width"] * rec["height"]), 1.0)
        rel_area = float(max_area) / img_area
        ds_map = cls_ds_rel[canon][rec["source_dataset"]]
        if rel_area > ds_map.get(iid, -1.0):
            ds_map[iid] = rel_area
    return cls_ds_rel


def select_leakfree_cohort(
    gt_index: Path,
    image_root: Path,
    holdout_manifest: Path,
    num_images: int = 12000,
    seed_ids: set[int] | None = None,
    prioritize_scarce: bool = True,
) -> list[dict[str, Any]]:
    """Select a balanced, leak-free training cohort preserving seed IDs and scarce classes."""
    excluded_ids, excluded_paths = _collect_exclusions(gt_index, image_root, holdout_manifest)
    train_8k_ds = HumanGTHBBDataset(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="train",
        cohort_size=8000,
        sanitize_annotations=False,
    )
    priority_8k_ids = {int(r["image_id"]) for r in train_8k_ds.images} - excluded_ids

    if num_images == 30000:
        quotas = dict(BASE_30K_QUOTAS)
    elif num_images == 12000:
        quotas = dict(BASE_12K_QUOTAS)
    else:
        ref_quotas = BASE_30K_QUOTAS if num_images > 12000 else BASE_12K_QUOTAS
        base_total = sum(ref_quotas.values())
        quotas = {
            ds: max(1, int(round(q * num_images / base_total)))
            for ds, q in ref_quotas.items()
        }
        quotas[DATASET_ORDER[0]] += num_images - sum(quotas.values())

    with sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True) as conn:
        rows = conn.execute(
            """
            SELECT image_id, file_name, width, height, source_dataset, row_index
            FROM images
            ORDER BY row_index
            """
        ).fetchall()

        meta_by_id: dict[int, dict[str, Any]] = {}
        all_by_ds: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for img_id, fn, w, h, src, row_idx in rows:
            iid = int(img_id)
            if iid in excluded_ids or src not in quotas:
                continue
            img_path = image_root / str(fn)
            if not img_path.exists():
                continue
            if str(img_path.resolve()) in excluded_paths:
                continue
            rec = {
                "image_id": iid,
                "file_name": str(fn),
                "width": int(w),
                "height": int(h),
                "source_dataset": str(src),
                "row_index": int(row_idx),
            }
            meta_by_id[iid] = rec
            all_by_ds[str(src)].append(rec)

        selected_by_ds: dict[str, list[dict[str, Any]]] = defaultdict(list)
        seen_ids: set[int] = set()

        def _try_pick(rec: dict[str, Any]) -> bool:
            iid = rec["image_id"]
            ds = rec["source_dataset"]
            if iid in seen_ids or len(selected_by_ds[ds]) >= quotas[ds]:
                return False
            seen_ids.add(iid)
            selected_by_ds[ds].append(rec)
            return True

        # Tier 0: Preserve existing cached seed IDs first within each dataset's quota.
        if seed_ids:
            seed_recs = sorted(
                (meta_by_id[i] for i in seed_ids if i in meta_by_id),
                key=lambda r: int(r["row_index"]),
            )
            for rec in seed_recs:
                _try_pick(rec)

        # Tier 1: Scarce-class prioritization (50% largest close-ups + 50% strided diversity).
        if prioritize_scarce and num_images > 12000:
            cls_ds_rel = _query_scarce_class_candidates(conn, meta_by_id)
            for canon_class, ds_caps in SCARCE_CLASS_QUOTAS_30K:
                for ds, cap in ds_caps.items():
                    avail = cls_ds_rel.get(canon_class, {}).get(ds, {})
                    if not avail:
                        continue
                    already = sum(1 for r in selected_by_ds[ds] if r["image_id"] in avail)
                    rem_quota = quotas[ds] - len(selected_by_ds[ds])
                    need = min(max(0, cap - already), rem_quota)
                    if need <= 0:
                        continue
                    unpicked = [meta_by_id[i] for i in avail if i not in seen_ids]
                    if not unpicked:
                        continue
                    n_closeup = need // 2
                    if n_closeup > 0:
                        by_closeup = sorted(
                            unpicked,
                            key=lambda r: (-avail[r["image_id"]], int(r["row_index"])),
                        )
                        for rec in by_closeup[:n_closeup]:
                            _try_pick(rec)
                    n_strided = min(need - n_closeup, quotas[ds] - len(selected_by_ds[ds]))
                    if n_strided > 0:
                        rem_unpicked = sorted(
                            (r for r in unpicked if r["image_id"] not in seen_ids),
                            key=lambda r: int(r["row_index"]),
                        )
                        for rec in _sample_even_strided(rem_unpicked, n_strided):
                            _try_pick(rec)

    # Tier 2: Fill remaining quota per dataset (8k priority first, then strided remainder).
    for ds in DATASET_ORDER:
        target_q = quotas[ds]
        rem = target_q - len(selected_by_ds[ds])
        if rem > 0:
            prio_pool = [
                r
                for r in all_by_ds[ds]
                if r["image_id"] in priority_8k_ids and r["image_id"] not in seen_ids
            ]
            for rec in prio_pool[:rem]:
                _try_pick(rec)
        rem = target_q - len(selected_by_ds[ds])
        if rem > 0:
            rest_pool = [r for r in all_by_ds[ds] if r["image_id"] not in seen_ids]
            for rec in _sample_even_strided(rest_pool, rem):
                _try_pick(rec)
        if len(selected_by_ds[ds]) < target_q:
            raise RuntimeError(
                f"Insufficient non-excluded images for {ds}: wanted {target_q}, got {len(selected_by_ds[ds])}"
            )

    selected = [rec for ds in DATASET_ORDER for rec in selected_by_ds[ds]]
    selected.sort(key=lambda r: int(r["row_index"]))

    assert len(selected) == num_images, f"Expected {num_images}, got {len(selected)}"
    assert not ({r["image_id"] for r in selected} & excluded_ids), "ID leakage detected!"
    return selected


def load_clean_gt_for_image(
    conn: sqlite3.Connection,
    image_id: int,
    source_dataset: str,
) -> tuple[list[list[float]], list[str | None], list[str]]:
    """Fetch and sanitize active human GT boxes & canonical labels for one image."""
    ann_rows = conn.execute(
        """
        SELECT x1, y1, x2, y2, ignore_region, category_name, geometry_tier
        FROM annotations
        WHERE image_id = ?
        ORDER BY rowid
        """,
        (int(image_id),),
    ).fetchall()
    hygiene_rows = [
        RawAnnotationRow(
            x1=float(r[0]),
            y1=float(r[1]),
            x2=float(r[2]),
            y2=float(r[3]),
            ignore_region=int(r[4]),
            category_name=str(r[5]),
            geometry_tier=str(r[6]),
        )
        for r in ann_rows
    ]
    dec = sanitize_image_annotations(hygiene_rows, source_dataset=source_dataset)
    boxes = [
        [hygiene_rows[i].x1, hygiene_rows[i].y1, hygiene_rows[i].x2, hygiene_rows[i].y2]
        for i in dec.kept_positive_indices
    ]
    raw_names = [hygiene_rows[i].category_name for i in dec.kept_positive_indices]
    canon_names = canonicalize_gt_labels(raw_names)
    return boxes, canon_names, raw_names


class _OwlImagePreprocessDataset(Dataset[dict[str, Any]]):
    """Worker-friendly CPU image loader + Owlv2ImageProcessor wrapper."""

    def __init__(
        self,
        records: list[dict[str, Any]],
        image_root: Path,
        model_id: str = OWLV2_MODEL_ID,
        revision: str | None = OWLV2_MODEL_REVISION,
    ) -> None:
        self.records = records
        self.image_root = image_root
        self.image_processor = Owlv2ImageProcessor.from_pretrained(
            model_id, revision=revision
        )

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, Any]:
        rec = self.records[index]
        img_path = self.image_root / rec["file_name"]
        with Image.open(img_path) as im:
            rgb = im.convert("RGB")
            orig_w, orig_h = rgb.size
            proc_out = self.image_processor(images=rgb, return_tensors="pt")
        return {
            "image_id": int(rec["image_id"]),
            "source_dataset": str(rec["source_dataset"]),
            "width": int(orig_w),
            "height": int(orig_h),
            "pixel_values": proc_out["pixel_values"].squeeze(0),
        }


def _collate_owl_batch(batch: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "image_id": [b["image_id"] for b in batch],
        "source_dataset": [b["source_dataset"] for b in batch],
        "width": [b["width"] for b in batch],
        "height": [b["height"] for b in batch],
        "pixel_values": torch.stack([b["pixel_values"] for b in batch], dim=0),
    }


def _size_bucket_384(box: list[float], w: int, h: int) -> str:
    scale = 384.0 / float(max(w, h, 1))
    min_side = min(max(0.0, box[2] - box[0]), max(0.0, box[3] - box[1])) * scale
    if min_side < 6.0:
        return "[0,6)"
    if min_side < 12.0:
        return "[6,12)"
    if min_side < 24.0:
        return "[12,24)"
    if min_side < 64.0:
        return "[24,64)"
    return "[64,inf)"


def run_qa_and_centroid(
    *,
    cache_db_path: Path,
    gt_index: Path,
    image_root: Path,
    vocab_37_path: Path,
    centroid_path: Path,
    summary_json_path: Path,
    qa_html_path: Path,
    qa_panels_dir: Path,
) -> dict[str, Any]:
    """Compute OWLv2 centroid, teacher Top-1 alignment benchmark, and QA panels/HTML."""
    vocab_37 = VocabularyBank.load(vocab_37_path, device="cpu")
    text_emb = vocab_37.embeddings.float()
    text_emb = text_emb / text_emb.norm(dim=-1, keepdim=True).clamp(min=1e-6)
    cat_to_idx = {c: i for i, c in enumerate(vocab_37.categories)}

    reader = OwlTeacherCacheReader(cache_db_path)
    gt_conn = sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True)
    img_meta_map = {
        int(r[0]): str(r[1])
        for r in gt_conn.execute("SELECT image_id, file_name FROM images").fetchall()
    }

    centroid_sum = np.zeros((768,), dtype=np.float64)
    centroid_count = 0

    ds_stats: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    cls_clean_gt: dict[str, int] = defaultdict(int)
    cls_superseded_gt: dict[str, int] = defaultdict(int)
    cls_owl_added: dict[str, int] = defaultdict(int)

    # Native OWLv2 Teacher embedding Top-1 / Top-3 on matched Clean Human GT (IoU >= 0.40)
    align_overall = {"total": 0, "top1": 0, "top3": 0}
    align_by_ds: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    align_by_size: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    align_by_cls: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))

    crowd_candidates: list[tuple[int, int, str, int]] = []
    strat_candidates: dict[str, list[tuple[int, int]]] = defaultdict(list)

    print("[QA] Scanning full SQLite cache for centroid & alignment metrics...", flush=True)
    for rec in reader.iter_records():
        img_id = rec.image_id
        src = rec.source_dataset
        boxes, canon_names, _raw_names = load_clean_gt_for_image(gt_conn, img_id, src)
        superseded_set = set(rec.superseded_gt_indices)

        ds_stats[src]["images"] += 1
        ds_stats[src]["clean_gt_before_split"] += len(boxes)
        ds_stats[src]["crowd_gt_split"] += rec.stats.get("crowd_gt_split", 0)
        ds_stats[src]["crowd_owl_peds_added"] += rec.stats.get("crowd_owl_peds_added", 0)
        ds_stats[src]["missing_class_added"] += rec.stats.get("missing_class_added", 0)
        ds_stats[src]["filtered_stuff_area"] += rec.stats.get("filtered_stuff_area", 0)
        ds_stats[src]["filtered_woodscape_hood"] += rec.stats.get("filtered_woodscape_hood", 0)
        ds_stats[src]["filtered_gt_overlap"] += rec.stats.get("filtered_gt_overlap", 0)

        for g_idx, cname in enumerate(canon_names):
            if cname is None or cname not in cat_to_idx:
                continue
            if g_idx in superseded_set:
                cls_superseded_gt[cname] += 1
            else:
                cls_clean_gt[cname] += 1

        for lbl in rec.owl_labels:
            cls_owl_added[lbl] += 1

        # Match GT embeddings (non-superseded, IoU >= 0.40)
        if rec.gt_embeddings.shape[0] == len(boxes) and len(boxes) > 0:
            gt_emb_f = torch.from_numpy(rec.gt_embeddings.astype(np.float32))
            gt_emb_f = gt_emb_f / gt_emb_f.norm(dim=-1, keepdim=True).clamp(min=1e-6)
            sims = gt_emb_f @ text_emb.t()
            for g_idx in range(len(boxes)):
                iou_v = float(rec.gt_match_ious[g_idx])
                cname = canon_names[g_idx]
                if iou_v < 0.40 or g_idx in superseded_set:
                    continue
                vec = gt_emb_f[g_idx].numpy().astype(np.float64)
                centroid_sum += vec
                centroid_count += 1
                if cname is None or cname not in cat_to_idx:
                    continue
                t_idx = cat_to_idx[cname]
                top3 = sims[g_idx].topk(3).indices.tolist()
                hit1 = int(top3[0] == t_idx)
                hit3 = int(t_idx in top3)
                sz = _size_bucket_384(boxes[g_idx], rec.width, rec.height)

                align_overall["total"] += 1
                align_overall["top1"] += hit1
                align_overall["top3"] += hit3
                align_by_ds[src]["total"] += 1
                align_by_ds[src]["top1"] += hit1
                align_by_ds[src]["top3"] += hit3
                align_by_size[sz]["total"] += 1
                align_by_size[sz]["top1"] += hit1
                align_by_size[sz]["top3"] += hit3
                align_by_cls[cname]["total"] += 1
                align_by_cls[cname]["top1"] += hit1

        if rec.owl_embeddings.shape[0] > 0:
            owl_f = rec.owl_embeddings.astype(np.float64)
            owl_norms = np.linalg.norm(owl_f, axis=-1, keepdims=True)
            owl_f = owl_f / np.clip(owl_norms, 1e-6, None)
            centroid_sum += owl_f.sum(axis=0)
            centroid_count += int(owl_f.shape[0])

        c_split = rec.stats.get("crowd_gt_split", 0)
        c_added = rec.stats.get("crowd_owl_peds_added", 0)
        if c_split >= 1 and c_added >= 2:
            crowd_candidates.append((c_added, c_split, src, img_id))
        else:
            strat_candidates[src].append((-(len(boxes) + len(rec.owl_labels)), img_id))

    # Save OWLv2 visual centroid
    if centroid_count > 0:
        centroid_vec = centroid_sum / max(np.linalg.norm(centroid_sum), 1e-8)
        centroid_t = torch.from_numpy(centroid_vec.astype(np.float32))
    else:
        centroid_t = torch.zeros((768,), dtype=torch.float32)
    centroid_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"visual_centroid": centroid_t, "count": centroid_count}, centroid_path)

    per_class_report: dict[str, dict[str, Any]] = {}
    for c in OPEN_VOCAB_CATEGORIES:
        clean_n = cls_clean_gt[c]
        added_n = cls_owl_added[c]
        sup_n = cls_superseded_gt[c]
        a_tot = align_by_cls[c]["total"]
        a_top1 = round(align_by_cls[c]["top1"] / a_tot, 4) if a_tot > 0 else None
        per_class_report[c] = {
            "clean_gt_kept": clean_n,
            "crowd_gt_superseded": sup_n,
            "owl_added": added_n,
            "total_targets": clean_n + added_n,
            "owlv2_gt_top1": a_top1,
            "owlv2_gt_eval_n": a_tot,
        }

    summary = {
        "total_cached_images": reader.count(),
        "centroid_sample_count": centroid_count,
        "dataset_stats": {
            ds: {
                **dict(ds_stats[ds]),
                "final_total_targets": ds_stats[ds]["clean_gt_before_split"]
                - ds_stats[ds]["crowd_gt_split"]
                + ds_stats[ds]["crowd_owl_peds_added"]
                + ds_stats[ds]["missing_class_added"],
            }
            for ds in DATASET_ORDER
            if ds in ds_stats
        },
        "owlv2_native_embedding_alignment_iou40": {
            "overall": {
                "total": align_overall["total"],
                "top1": round(align_overall["top1"] / max(align_overall["total"], 1), 4),
                "top3": round(align_overall["top3"] / max(align_overall["total"], 1), 4),
            },
            "by_dataset": {
                ds: {
                    "total": d["total"],
                    "top1": round(d["top1"] / max(d["total"], 1), 4),
                    "top3": round(d["top3"] / max(d["total"], 1), 4),
                }
                for ds, d in sorted(align_by_ds.items())
            },
            "by_size_384": {
                sz: {
                    "total": d["total"],
                    "top1": round(d["top1"] / max(d["total"], 1), 4),
                    "top3": round(d["top3"] / max(d["total"], 1), 4),
                }
                for sz, d in sorted(align_by_size.items())
            },
        },
        "per_class_counts_37": per_class_report,
    }
    summary_json_path.write_text(json.dumps(summary, indent=2))

    # Pick 12 crowd-split QA examples (prioritizing dataset diversity first, then max split count)
    crowd_candidates.sort(key=lambda x: (-x[0], -x[1], x[3]))
    picked_crowd: list[int] = []
    seen_qa: set[int] = set()
    for ds in DATASET_ORDER:
        for _c_add, _c_sp, src, iid in crowd_candidates:
            if src == ds and iid not in seen_qa:
                picked_crowd.append(iid)
                seen_qa.add(iid)
                break
    for _c_add, _c_sp, _src, iid in crowd_candidates:
        if len(picked_crowd) >= 12:
            break
        if iid not in seen_qa:
            picked_crowd.append(iid)
            seen_qa.add(iid)

    # Pick 12 stratified non-crowd examples (2 per dataset)
    picked_strat: list[int] = []
    for ds in DATASET_ORDER:
        cands = sorted(strat_candidates.get(ds, []))
        added_ds = 0
        for _neg, iid in cands:
            if iid not in seen_qa:
                picked_strat.append(iid)
                seen_qa.add(iid)
                added_ds += 1
                if added_ds >= 2:
                    break

    qa_items = _build_qa_payload_and_sheets(
        reader=reader,
        gt_conn=gt_conn,
        img_meta_map=img_meta_map,
        image_root=image_root,
        crowd_ids=picked_crowd,
        strat_ids=picked_strat,
        qa_panels_dir=qa_panels_dir,
    )
    _write_qa_html(qa_items, summary, qa_html_path)

    gt_conn.close()
    reader.close()
    return summary


def _load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for p in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ):
        if Path(p).exists():
            return ImageFont.truetype(p, size=size)
    return ImageFont.load_default()


def _render_qa_tile(item: dict[str, Any], tile_w: int = 920) -> Image.Image:
    font_hdr = _load_font(15)
    font_tag = _load_font(12)
    header_h = 30
    with Image.open(item["image_path"]) as im:
        rgb = im.convert("RGB")
    scale = tile_w / max(rgb.width, 1)
    tile_h = max(1, int(round(rgb.height * scale)))
    resized = rgb.resize((tile_w, tile_h), Image.Resampling.BILINEAR)
    panel = Image.new("RGB", (tile_w, tile_h + header_h), (16, 22, 34))
    panel.paste(resized, (0, header_h))
    draw = ImageDraw.Draw(panel)
    title_bg = (120, 53, 15) if item["qa_group"] == "crowd_split" else (22, 78, 52)
    draw.rectangle([0, 0, tile_w, header_h], fill=title_bg)
    draw.text(
        (8, 7),
        f"[{item['qa_group']}] {item['source_dataset']} #{item['image_id']} | GT={len(item['gt_kept'])} SupSplit={len(item['gt_superseded'])} +SplitPed={len(item['owl_crowd_peds'])} +Missing={len(item['owl_missing'])}",
        fill=(248, 250, 252),
        font=font_hdr,
    )

    draw_ops: list[tuple[list[float], str, tuple[int, int, int]]] = []
    for b, lbl in item["gt_kept"]:
        draw_ops.append((b, f"GT:{lbl}", (74, 222, 128)))
    for b, lbl in item["gt_superseded"]:
        draw_ops.append((b, f"SPLIT_OLD:{lbl}", (250, 204, 21)))
    for b, lbl, sc in item["owl_crowd_peds"]:
        draw_ops.append((b, f"SPLIT_PED:{lbl} {int(sc*100)}%", (34, 211, 238)))
    for b, lbl, sc in item["owl_missing"]:
        draw_ops.append((b, f"+OWL:{lbl} {int(sc*100)}%", (232, 121, 249)))

    for b, txt, col in draw_ops:
        x1, y1, x2, y2 = b[0] * scale, b[1] * scale + header_h, b[2] * scale, b[3] * scale + header_h
        draw.rectangle([x1, y1, x2, y2], outline=col, width=2)
        tb = draw.textbbox((0, 0), txt, font=font_tag)
        tw, th = tb[2] - tb[0], tb[3] - tb[1]
        ty = max(header_h + 1, y1 - th - 4)
        tx = max(1, min(x1, tile_w - tw - 6))
        draw.rectangle([tx, ty, tx + tw + 6, ty + th + 4], fill=(10, 14, 24))
        draw.text((tx + 3, ty + 2), txt, fill=col, font=font_tag)
    return panel


def _save_sheet(tiles: list[Image.Image], out_path: Path) -> None:
    if not tiles:
        return
    cols = 2
    rows = (len(tiles) + cols - 1) // cols
    tile_w = tiles[0].width
    row_heights = [
        max(tiles[r * cols + c].height for c in range(cols) if r * cols + c < len(tiles))
        for r in range(rows)
    ]
    sheet = Image.new("RGB", (tile_w * cols, sum(row_heights)), (10, 14, 22))
    y_off = 0
    for r in range(rows):
        for c in range(cols):
            idx = r * cols + c
            if idx < len(tiles):
                sheet.paste(tiles[idx], (c * tile_w, y_off))
        y_off += row_heights[r]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)


def _build_qa_payload_and_sheets(
    *,
    reader: OwlTeacherCacheReader,
    gt_conn: sqlite3.Connection,
    img_meta_map: dict[int, str],
    image_root: Path,
    crowd_ids: list[int],
    strat_ids: list[int],
    qa_panels_dir: Path,
) -> list[dict[str, Any]]:
    qa_items: list[dict[str, Any]] = []
    for grp_name, id_list in (("crowd_split", crowd_ids), ("stratified", strat_ids)):
        for iid in id_list:
            rec = reader.get_image(iid)
            if rec is None or iid not in img_meta_map:
                continue
            boxes, canon_names, _raw = load_clean_gt_for_image(gt_conn, iid, rec.source_dataset)
            sup_set = set(rec.superseded_gt_indices)
            gt_kept = [
                (boxes[i], canon_names[i] or "unknown")
                for i in range(len(boxes))
                if i not in sup_set
            ]
            gt_sup = [
                (boxes[i], canon_names[i] or "unknown")
                for i in range(len(boxes))
                if i in sup_set
            ]
            owl_crowd = []
            owl_miss = []
            for j in range(len(rec.owl_labels)):
                entry = ([round(float(v), 1) for v in rec.owl_boxes[j].tolist()], rec.owl_labels[j], float(rec.owl_scores[j]))
                if bool(rec.owl_crowd_mask[j]):
                    owl_crowd.append(entry)
                else:
                    owl_miss.append(entry)
            qa_items.append(
                {
                    "image_id": iid,
                    "qa_group": grp_name,
                    "source_dataset": rec.source_dataset,
                    "width": rec.width,
                    "height": rec.height,
                    "image_path": str(image_root / img_meta_map[iid]),
                    "gt_kept": gt_kept,
                    "gt_superseded": gt_sup,
                    "owl_crowd_peds": owl_crowd,
                    "owl_missing": owl_miss,
                }
            )

    crowd_tiles = [_render_qa_tile(it) for it in qa_items if it["qa_group"] == "crowd_split"][:6]
    strat_tiles = [_render_qa_tile(it) for it in qa_items if it["qa_group"] == "stratified"][:6]
    _save_sheet(crowd_tiles, qa_panels_dir / "crowd_splits_qa.png")
    _save_sheet(strat_tiles, qa_panels_dir / "stratified_qa.png")
    return qa_items


def _write_qa_html(
    qa_items: list[dict[str, Any]],
    summary: dict[str, Any],
    out_path: Path,
) -> None:
    payload = []
    for it in qa_items:
        with Image.open(it["image_path"]) as im:
            rgb = im.convert("RGB")
            scale = min(1.0, 900.0 / max(rgb.width, 1))
            if scale < 1.0:
                rgb = rgb.resize((int(rgb.width * scale), int(rgb.height * scale)), Image.Resampling.BILINEAR)
            buf = io.BytesIO()
            rgb.save(buf, format="JPEG", quality=82)
            thumb = "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")
        payload.append(
            {
                "image_id": it["image_id"],
                "qa_group": it["qa_group"],
                "source_dataset": it["source_dataset"],
                "width": it["width"],
                "height": it["height"],
                "gt_kept": it["gt_kept"],
                "gt_superseded": it["gt_superseded"],
                "owl_crowd_peds": it["owl_crowd_peds"],
                "owl_missing": it["owl_missing"],
                "thumb": thumb,
            }
        )

    ds_rows = ""
    for ds, d in summary["dataset_stats"].items():
        al = summary["owlv2_native_embedding_alignment_iou40"]["by_dataset"].get(ds, {})
        ds_rows += (
            f"<tr><td>{escape(ds)}</td><td>{d['images']}</td><td>{d['clean_gt_before_split']}</td>"
            f"<td>{d['crowd_gt_split']} -> {d['crowd_owl_peds_added']}</td>"
            f"<td>+{d['missing_class_added']}</td><td>{d['final_total_targets']}</td>"
            f"<td>{al.get('top1', 0.0)*100:.1f}%</td></tr>"
        )

    ov = summary["owlv2_native_embedding_alignment_iou40"]["overall"]
    html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>OWLv2 {summary['total_cached_images']} Cache QA Report</title>
<style>
body{{font-family:system-ui,sans-serif;background:#0b1020;color:#e2e8f0;margin:0;padding:18px}}
table{{border-collapse:collapse;font-size:.82rem;background:#131a2e;margin-bottom:14px}}
th,td{{border:1px solid #26314a;padding:5px 10px;text-align:right}} th:first-child,td:first-child{{text-align:left}}
.chip{{background:#1e293b;color:#cbd5e1;border:1px solid #334155;padding:4px 10px;border-radius:999px;cursor:pointer;margin-right:6px}}
.chip.active{{background:#0284c7;color:#fff}}
.grid{{display:grid;grid-template-columns:1fr 1fr;gap:12px}}
.card{{background:#131a2e;border:1px solid #26314a;border-radius:8px;padding:10px}}
canvas{{width:100%;height:auto;border-radius:6px;display:block}}
</style></head><body>
<h1>Phase 1 OWLv2 Cache QA ({summary['total_cached_images']} Images | Native Embedding GT Top-1={ov['top1']*100:.1f}% Top-3={ov['top3']*100:.1f}%)</h1>
<table><tr><th>Dataset</th><th>Imgs</th><th>Clean GT</th><th>Crowd Split (GT->OWL)</th><th>+Missing</th><th>Total Targets</th><th>OWLv2 GT Top-1</th></tr>{ds_rows}</table>
<div style="margin-bottom:12px">
  <button class="chip active" onclick="setGrp('all', this)">All ({len(payload)})</button>
  <button class="chip" onclick="setGrp('crowd_split', this)">Crowd Splits (12)</button>
  <button class="chip" onclick="setGrp('stratified', this)">Stratified (12)</button>
  <span style="font-size:.82rem;color:#94a3b8;margin-left:12px">Green=Clean GT | Yellow dashed=Replaced Group Ped GT | Cyan=Individual Split Ped | Purple=+Missing OWLv2</span>
</div>
<div class="grid" id="gal"></div>
<script>
const ITEMS = {json.dumps(payload)};
let activeGrp = 'all';
function setGrp(g, btn) {{
  activeGrp = g;
  document.querySelectorAll('.chip').forEach(b=>b.classList.remove('active'));
  btn.classList.add('active');
  render();
}}
function render() {{
  const gal = document.getElementById('gal'); gal.innerHTML = '';
  for (const it of ITEMS.filter(x => activeGrp === 'all' || x.qa_group === activeGrp)) {{
    const card = document.createElement('div'); card.className = 'card';
    card.innerHTML = `<div style="display:flex;justify-content:space-between;margin-bottom:6px;font-size:.86rem"><strong>[${{it.qa_group}}] ${{it.source_dataset}} #${{it.image_id}}</strong><span>GT=${{it.gt_kept.length}} SplitOld=${{it.gt_superseded.length}} +SplitPed=${{it.owl_crowd_peds.length}} +Miss=${{it.owl_missing.length}}</span></div><canvas></canvas>`;
    const cv = card.querySelector('canvas');
    const img = new Image(); img.src = it.thumb;
    img.onload = () => {{
      cv.width = img.width; cv.height = img.height;
      const sx = cv.width / it.width, sy = cv.height / it.height;
      const ctx = cv.getContext('2d'); ctx.drawImage(img, 0, 0);
      const drawB = (b, txt, col, dash) => {{
        ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.setLineDash(dash ? [6,4] : []);
        ctx.strokeRect(b[0]*sx, b[1]*sy, (b[2]-b[0])*sx, (b[3]-b[1])*sy); ctx.setLineDash([]);
        ctx.font = 'bold 11px sans-serif'; const tw = ctx.measureText(txt).width;
        const ty = Math.max(14, b[1]*sy - 14);
        ctx.fillStyle = 'rgba(10,14,24,0.86)'; ctx.fillRect(b[0]*sx, ty, tw+6, 14);
        ctx.fillStyle = col; ctx.fillText(txt, b[0]*sx+3, ty+11);
      }};
      it.gt_kept.forEach(([b, l]) => drawB(b, 'GT:'+l, '#4ade80', false));
      it.gt_superseded.forEach(([b, l]) => drawB(b, 'OLD_CROWD:'+l, '#facc15', true));
      it.owl_crowd_peds.forEach(([b, l, s]) => drawB(b, 'PED '+Math.round(s*100)+'%', '#22d3ee', false));
      it.owl_missing.forEach(([b, l, s]) => drawB(b, '+OWL:'+l+' '+Math.round(s*100)+'%', '#e879f9', false));
    }};
    gal.appendChild(card);
  }}
}}
window.addEventListener('DOMContentLoaded', render);
</script></body></html>"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument(
        "--holdout-manifest",
        type=Path,
        default=Path("artifacts/dashboard_holdout_8k_8k_full.json"),
    )
    parser.add_argument("--num-images", type=int, default=12000)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--output-db",
        type=Path,
        default=Path("artifacts/cache/owlv2_teacher_12k.sqlite"),
    )
    parser.add_argument(
        "--vocab-dir",
        type=Path,
        default=Path("artifacts/sota_open_vocab"),
    )
    parser.add_argument(
        "--qa-only",
        action="store_true",
        help="Skip extraction and only regenerate centroid/QA from existing SQLite cache.",
    )
    parser.add_argument(
        "--patch-wide-pedestrians",
        action="store_true",
        help="Re-run OWLv2 only on already-cached images containing wide pedestrian GT boxes.",
    )
    parser.add_argument(
        "--seed-from-db",
        type=Path,
        default=None,
        help="Optional existing SQLite cache to preserve & clone via sqlite3.backup before scaling.",
    )
    parser.add_argument(
        "--dry-run-cohort",
        action="store_true",
        help="Select cohort and print dataset + scarce-class stats without loading OWLv2.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    return parser.parse_args()


def _select_wide_pedestrian_patch_records(
    gt_index: Path,
    completed_ids: set[int],
) -> list[dict[str, Any]]:
    """Find cached image records that have >= 1 wide pedestrian GT box eligible for crowd split."""
    if not completed_ids:
        return []
    with sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE patch_cids (image_id INTEGER PRIMARY KEY)")
        conn.executemany(
            "INSERT INTO patch_cids VALUES (?)", [(int(i),) for i in completed_ids]
        )
        rows = conn.execute(
            """
            SELECT DISTINCT img.image_id, img.file_name, img.width, img.height, img.source_dataset, img.row_index
            FROM annotations a
            JOIN patch_cids p ON a.image_id = p.image_id
            JOIN images img ON a.image_id = img.image_id
            WHERE a.ignore_region = 0
              AND LOWER(TRIM(a.category_name)) IN ('person', 'pedestrian', 'adult', 'child')
              AND (a.x2 - a.x1) > 1.2 * (a.y2 - a.y1)
              AND (a.x2 - a.x1) > 60.0
            ORDER BY img.row_index
            """
        ).fetchall()
    return [
        {
            "image_id": int(r[0]),
            "file_name": str(r[1]),
            "width": int(r[2]),
            "height": int(r[3]),
            "source_dataset": str(r[4]),
            "row_index": int(r[5]),
        }
        for r in rows
    ]


def _print_dry_run_cohort_report(
    cohort: list[dict[str, Any]],
    seed_ids: set[int],
    gt_index: Path,
) -> None:
    """Print dataset breakdown and scarce-class coverage for a dry-run cohort."""
    ds_counts: dict[str, int] = defaultdict(int)
    cohort_meta: dict[int, dict[str, Any]] = {}
    for rec in cohort:
        ds_counts[rec["source_dataset"]] += 1
        cohort_meta[int(rec["image_id"])] = rec

    with sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True) as conn:
        cls_ds_rel = _query_scarce_class_candidates(conn, cohort_meta)

    already_cached = sum(1 for r in cohort if int(r["image_id"]) in seed_ids)
    remaining_cnt = len(cohort) - already_cached
    print(f"Dataset breakdown ({len(cohort)} total):", flush=True)
    for ds in DATASET_ORDER:
        print(f"  {ds:24s}: {ds_counts.get(ds, 0)}", flush=True)
    print(f"Already cached (seeded): {already_cached} | Remaining: {remaining_cnt}", flush=True)
    print("Scarce-class image coverage in selected cohort:", flush=True)
    for canon_class, _ in SCARCE_CLASS_QUOTAS_30K:
        per_ds = {ds: len(iids) for ds, iids in sorted(cls_ds_rel.get(canon_class, {}).items())}
        tot = sum(per_ds.values())
        print(f"  {canon_class:22s}: total={tot:5d} | {per_ds}", flush=True)
    print("Zero val/holdout overlap verified.", flush=True)


def _ensure_core27_banks(
    vocab_37_path: Path,
    vocab_41_path: Path,
    vocab_core27_path: Path,
    vocab_31_core27_zs_path: Path,
) -> None:
    if vocab_37_path.exists() and not vocab_core27_path.exists():
        VocabularyBank.load(vocab_37_path, device="cpu").subset(CORE_27_CATEGORIES).save(
            vocab_core27_path
        )
    if vocab_41_path.exists() and not vocab_31_core27_zs_path.exists():
        VocabularyBank.load(vocab_41_path, device="cpu").subset(
            CORE_27_CATEGORIES + ZERO_SHOT_EVAL_CATEGORIES
        ).save(vocab_31_core27_zs_path)


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    vocab_37_path = args.vocab_dir / "owlv2_vocabulary_bank_37classes.pt"
    vocab_41_path = args.vocab_dir / "owlv2_vocabulary_bank_41classes_zeroshot.pt"
    vocab_core27_path = args.vocab_dir / "owlv2_vocabulary_bank_core27.pt"
    vocab_31_core27_zs_path = args.vocab_dir / "owlv2_vocabulary_bank_31classes_core27_zeroshot.pt"
    vocab_dist_path = args.vocab_dir / "owlv2_vocabulary_bank_distractors.pt"
    centroid_path = args.vocab_dir / "owlv2_visual_centroid.pt"
    summary_json_path = args.output_db.with_name(f"{args.output_db.stem}_qa_summary.json")
    qa_html_path = args.output_db.with_name(f"{args.output_db.stem}_qa.html")
    qa_panels_dir = (
        args.output_db.parent / "qa_panels"
        if args.output_db.stem == "owlv2_teacher_12k"
        else args.output_db.parent / f"qa_panels_{args.output_db.stem.split('_')[-1]}"
    )

    if args.dry_run_cohort:
        seed_ids: set[int] = set()
        for cand_db in (args.output_db, args.seed_from_db):
            if cand_db is not None and cand_db.exists():
                r = OwlTeacherCacheReader(cand_db)
                seed_ids.update(r.get_image_ids())
                r.close()
        cohort = select_leakfree_cohort(
            args.gt_index,
            args.image_root,
            args.holdout_manifest,
            num_images=args.num_images,
            seed_ids=seed_ids,
            prioritize_scarce=True,
        )
        print(
            f"[Dry-Run] Selected {len(cohort)} leak-free images across {len(BASE_30K_QUOTAS)} datasets.",
            flush=True,
        )
        _print_dry_run_cohort_report(cohort, seed_ids, args.gt_index)
        return

    if not args.qa_only:
        if (
            not args.output_db.exists()
            and args.seed_from_db is not None
            and args.seed_from_db.exists()
        ):
            args.output_db.parent.mkdir(parents=True, exist_ok=True)
            print(
                f"[Phase 1] Cloning WAL-consistent seed cache {args.seed_from_db} -> {args.output_db}...",
                flush=True,
            )
            with sqlite3.connect(f"file:{args.seed_from_db}?mode=ro", uri=True) as src_conn:
                with sqlite3.connect(args.output_db) as dst_conn:
                    src_conn.backup(dst_conn)

        writer = OwlTeacherCacheWriter(args.output_db)
        completed_ids = writer.get_completed_ids()
        if args.patch_wide_pedestrians:
            remaining = _select_wide_pedestrian_patch_records(args.gt_index, completed_ids)
            print(
                f"[Patch] Found {len(remaining)} cached images with wide pedestrian GT boxes to re-extract.",
                flush=True,
            )
        else:
            cohort = select_leakfree_cohort(
                args.gt_index,
                args.image_root,
                args.holdout_manifest,
                num_images=args.num_images,
                seed_ids=completed_ids,
                prioritize_scarce=True,
            )
            print(
                f"[Phase 1] Selected {len(cohort)} leak-free images across {len(BASE_12K_QUOTAS)} datasets.",
                flush=True,
            )
            writer.set_metadata("model_id", OWLV2_MODEL_ID)
            writer.set_metadata("target_images", str(len(cohort)))
            remaining = [r for r in cohort if int(r["image_id"]) not in completed_ids]
            print(
                f"[Phase 1] Already cached: {len(completed_ids)} | Remaining to process: {len(remaining)}",
                flush=True,
            )

        need_banks = not (
            vocab_37_path.exists() and vocab_41_path.exists() and vocab_dist_path.exists()
        )
        if remaining or need_banks:
            extractor = Owlv2TeacherExtractor(device=device)
            if need_banks:
                print("[Phase 1] Building native OWLv2 768-D VocabularyBanks...", flush=True)
                extractor.build_vocabulary_bank(OPEN_VOCAB_CATEGORIES).save(vocab_37_path)
                extractor.build_vocabulary_bank(
                    OPEN_VOCAB_CATEGORIES + ZERO_SHOT_EVAL_CATEGORIES
                ).save(vocab_41_path)
                extractor.build_vocabulary_bank(DISTRACTOR_CATEGORIES).save(vocab_dist_path)

            if remaining:
                ds = _OwlImagePreprocessDataset(remaining, args.image_root, model_id=OWLV2_MODEL_ID)
                loader = DataLoader(
                    ds,
                    batch_size=args.batch_size,
                    shuffle=False,
                    num_workers=args.workers,
                    prefetch_factor=2 if args.workers > 0 else None,
                    collate_fn=_collate_owl_batch,
                )
                gt_conn = sqlite3.connect(f"file:{args.gt_index}?mode=ro&immutable=1", uri=True)
                t_start = time.perf_counter()
                processed_now = 0
                running_crowd = 0
                running_added = 0

                for batch_idx, batch in enumerate(loader):
                    b_ids: list[int] = batch["image_id"]
                    b_srcs: list[str] = batch["source_dataset"]
                    b_sizes = list(zip(batch["height"], batch["width"], strict=True))

                    gt_boxes_batch: list[list[list[float]]] = []
                    gt_canon_batch: list[list[str | None]] = []
                    for iid, src in zip(b_ids, b_srcs, strict=True):
                        boxes, canon, _raw = load_clean_gt_for_image(gt_conn, iid, src)
                        gt_boxes_batch.append(boxes)
                        gt_canon_batch.append(canon)

                    ext_list = extractor.extract_batch(
                        batch["pixel_values"],
                        b_sizes,
                        gt_boxes_batch,
                        min_score=0.25,
                    )

                    for i, iid in enumerate(b_ids):
                        ext = ext_list[i]
                        owl_dets: list[OwlDetection] = ext["owl_detections"]
                        merged = merge_clean_gt_and_owlv2(
                            gt_boxes_batch[i],
                            gt_canon_batch[i],
                            owl_dets,
                            source_dataset=b_srcs[i],
                            img_w=int(batch["width"][i]),
                            img_h=int(batch["height"][i]),
                        )
                        writer.write_image(
                            image_id=iid,
                            source_dataset=b_srcs[i],
                            width=int(batch["width"][i]),
                            height=int(batch["height"][i]),
                            merged=merged,
                            gt_match_ious=ext["gt_match_ious"],
                            gt_embeddings=ext["gt_embeddings"],
                            all_owl_detections=owl_dets,
                            all_owl_embeddings=ext["owl_embeddings"],
                        )
                        running_crowd += merged.stats["crowd_gt_split"]
                        running_added += merged.stats["missing_class_added"]
                        processed_now += 1

                    if (batch_idx + 1) % 25 == 0:
                        writer.commit()
                    if processed_now % 500 < args.batch_size and processed_now >= 500:
                        elapsed = max(time.perf_counter() - t_start, 1e-3)
                        rate = processed_now / elapsed
                        eta_min = (len(remaining) - processed_now) / max(rate, 1e-6) / 60.0
                        print(
                            f"  [{processed_now}/{len(remaining)}] {rate:.2f} img/s | ETA={eta_min:.1f}m | crowd_splits={running_crowd} | +missing={running_added}",
                            flush=True,
                        )

                gt_conn.close()
                del extractor
                if device.type == "cuda":
                    torch.cuda.empty_cache()
        writer.close()

    _ensure_core27_banks(
        vocab_37_path, vocab_41_path, vocab_core27_path, vocab_31_core27_zs_path
    )
    summary = run_qa_and_centroid(
        cache_db_path=args.output_db,
        gt_index=args.gt_index,
        image_root=args.image_root,
        vocab_37_path=vocab_37_path,
        centroid_path=centroid_path,
        summary_json_path=summary_json_path,
        qa_html_path=qa_html_path,
        qa_panels_dir=qa_panels_dir,
    )
    ov = summary["owlv2_native_embedding_alignment_iou40"]["overall"]
    print(
        f"\n[Phase 1 Done] Cached={summary['total_cached_images']} imgs | CentroidN={summary['centroid_sample_count']} | "
        f"OWLv2 GT Top-1={ov['top1']*100:.2f}% Top-3={ov['top3']*100:.2f}%",
        flush=True,
    )


if __name__ == "__main__":
    main()
