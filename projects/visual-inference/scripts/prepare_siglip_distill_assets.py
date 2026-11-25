#!/usr/bin/env python3
"""Rebuild open-vocabulary + distractor banks and build SAM SigLIP distillation assets.

1. Rebuilds `vocabulary_bank_37classes.pt`, `vocabulary_bank_41classes_zeroshot.pt`,
   and `vocabulary_bank_distractors.pt` from live SigLIP text encoder.
2. Strictly restricts extraction to the 8,458 training image IDs present in
   `artifacts/cache/siglip_gt_crops_8k.pt` and asserts zero resolved-path
   overlap with `excluded_validation_images` (800) and `images` (210) in
   `artifacts/dashboard_holdout_8k_8k_full.json`.
3. Queries up to `max_per_image` diverse, high-stability (`>= 0.90`),
   non-suppressed (`is_suppressed = 0`), non-GT-overlapping (`IoU < 0.25`)
   SAM discovery regions per image from `sam_discovery_enriched.sqlite`,
   encodes unflipped + horizontally flipped 224x224 native crops with
   `SiglipTeacher` in FP16, and saves `artifacts/cache/siglip_sam_discovery_8k.pt`.
4. Scores the cached SAM discovery regions against `[vocab_37; vocab_distractors]`
   and merges up to 350 high-confidence teacher-verified regions per target class
   into `artifacts/cache/open_world_pseudo_gt_8k.pt`.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sqlite3
from typing import Any

import numpy as np
from PIL import Image
import torch
from torch import Tensor
import torch.nn.functional as F

from student_detector.annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from student_detector.human_gt_data import DEFAULT_ENRICHED_GT_INDEX_PATH, DEFAULT_IMAGE_ROOT
from student_detector.ontology import (
    DISTRACTOR_CATEGORIES,
    OPEN_VOCAB_CATEGORIES,
    ZERO_SHOT_EVAL_CATEGORIES,
)
from student_detector.siglip import SiglipTeacher
from student_detector.vocabulary import DEFAULT_CATEGORY_SYNONYMS, VocabularyBank

TARGET_PSEUDO_GT_CLASSES: tuple[str, ...] = (
    "window",
    "tree",
    "building",
    "stairs",
    "parking_slot",
    "trash_bin",
    "construction_barrier",
    "pillar",
    "pole",
    "canopy",
    "signboard",
    "traffic_sign",
    "traffic_light",
    "fire_hydrant",
    "fence",
    "stroller",
)

DRIVING_SOURCES: frozenset[str] = frozenset(
    {
        "bdd100k_images_100k",
        "nuimages",
        "woodscape_rgb_fisheye",
        "roboflow_ev_parking",
    }
)

EXTRA_DISTRACTOR_SYNONYMS: dict[str, tuple[str, ...]] = {
    "road_surface": (
        "asphalt road surface",
        "paved street pavement",
        "sidewalk ground surface",
        "driveway asphalt",
    ),
    "sky": (
        "open sky",
        "clouds and sky",
        "daytime sky",
        "overcast sky",
    ),
}


def compute_box_iou_np(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    """Compute pairwise [N, M] IoU between xyxy numpy arrays."""
    if boxes_a.shape[0] == 0 or boxes_b.shape[0] == 0:
        return np.zeros((boxes_a.shape[0], boxes_b.shape[0]), dtype=np.float32)
    ax1, ay1, ax2, ay2 = boxes_a[:, 0:1], boxes_a[:, 1:2], boxes_a[:, 2:3], boxes_a[:, 3:4]
    bx1, by1, bx2, by2 = boxes_b[:, 0][None, :], boxes_b[:, 1][None, :], boxes_b[:, 2][None, :], boxes_b[:, 3][None, :]
    ix1 = np.maximum(ax1, bx1)
    iy1 = np.maximum(ay1, by1)
    ix2 = np.minimum(ax2, bx2)
    iy2 = np.minimum(ay2, by2)
    inter = np.maximum(0.0, ix2 - ix1) * np.maximum(0.0, iy2 - iy1)
    area_a = np.maximum(0.0, ax2 - ax1) * np.maximum(0.0, ay2 - ay1)
    area_b = np.maximum(0.0, bx2 - bx1) * np.maximum(0.0, by2 - by1)
    union = area_a + area_b - inter
    return np.where(union > 0.0, inter / np.maximum(union, 1e-6), 0.0).astype(np.float32)


def resolve_image_path(file_name: str, image_root: Path) -> Path:
    """Resolve an image path relative to `image_root` or return absolute path."""
    p = Path(file_name)
    return p if p.is_absolute() else (image_root / p)


def verify_zero_cohort_overlap(
    gt_db_path: Path,
    train_ids: list[int],
    holdout_manifest: Path,
    image_root: Path,
) -> dict[int, tuple[str, int, int]]:
    """Verify 0 resolved path overlap between `train_ids` and val+holdout cohorts."""
    raw = json.loads(holdout_manifest.read_text())
    excluded_paths: set[Path] = set()
    for key in ("excluded_validation_images", "images"):
        for rec in raw.get(key, []):
            fn = rec["file_name"] if isinstance(rec, dict) else str(rec)
            excluded_paths.add(resolve_image_path(fn, image_root).resolve())

    meta_by_id: dict[int, tuple[str, int, int]] = {}
    with sqlite3.connect(f"file:{gt_db_path}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE p_train_ids (image_id INTEGER PRIMARY KEY)")
        conn.executemany("INSERT INTO p_train_ids VALUES (?)", [(i,) for i in train_ids])
        rows = conn.execute(
            """
            SELECT i.image_id, i.file_name, i.width, i.height
            FROM images i
            JOIN p_train_ids t ON i.image_id = t.image_id
            """
        ).fetchall()

    overlap = 0
    for img_id, file_name, w, h in rows:
        resolved = resolve_image_path(str(file_name), image_root).resolve()
        if resolved in excluded_paths:
            overlap += 1
        meta_by_id[int(img_id)] = (str(file_name), int(w), int(h))

    if overlap > 0:
        raise RuntimeError(f"Found {overlap} overlapping image paths with validation/holdout!")
    if len(meta_by_id) != len(train_ids):
        raise RuntimeError(f"Expected {len(train_ids)} image rows in GT DB, found {len(meta_by_id)}.")
    print(
        f"Verified 0 path overlap across {len(meta_by_id)} train IDs vs "
        f"{len(excluded_paths)} excluded val/holdout paths.",
        flush=True,
    )
    return meta_by_id


def parse_quad_or_box(
    quad_json: str | None,
    box: list[float],
    width: int,
    height: int,
) -> list[list[float]]:
    """Parse stored 4-point quad or construct an axis-aligned quad from xyxy box."""
    x1, y1, x2, y2 = box
    if quad_json:
        try:
            parsed = json.loads(quad_json)
            if isinstance(parsed, list) and len(parsed) == 4 and all(isinstance(pt, list) and len(pt) == 2 for pt in parsed):
                return [
                    [
                        float(max(0.0, min(float(width), float(pt[0])))),
                        float(max(0.0, min(float(height), float(pt[1])))),
                    ]
                    for pt in parsed
                ]
        except Exception:
            pass
    return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]


def select_sam_candidates(
    *,
    gt_db_path: Path,
    sam_db_path: Path,
    meta_by_id: dict[int, tuple[str, int, int]],
    max_per_image: int = 6,
    min_stability: float = 0.90,
    min_area: float = 400.0,
    min_side: float = 12.0,
    max_img_frac: float = 0.70,
    max_aspect: float = 8.0,
    max_gt_iou: float = 0.25,
    intra_iou_thresh: float = 0.55,
) -> dict[int, list[tuple[list[float], list[list[float]]]]]:
    """Select up to `max_per_image` diverse non-GT SAM discovery regions per train image."""
    train_ids = sorted(meta_by_id.keys())
    gt_boxes_by_id: dict[int, list[list[float]]] = defaultdict(list)
    raw_gt_by_id: dict[int, tuple[str, list[RawAnnotationRow]]] = {}
    with sqlite3.connect(f"file:{gt_db_path}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE s_tids (image_id INTEGER PRIMARY KEY)")
        conn.executemany("INSERT INTO s_tids VALUES (?)", [(i,) for i in train_ids])
        for img_id, x1, y1, x2, y2, ign, cat_name, tier, src_ds in conn.execute(
            """
            SELECT a.image_id, a.x1, a.y1, a.x2, a.y2, a.ignore_region, a.category_name, a.geometry_tier, i.source_dataset
            FROM annotations a
            JOIN s_tids t ON a.image_id = t.image_id
            JOIN images i ON a.image_id = i.image_id
            WHERE a.ignore_region = 0
            ORDER BY a.image_id, a.rowid
            """
        ):
            iid = int(img_id)
            if iid not in raw_gt_by_id:
                raw_gt_by_id[iid] = (str(src_ds), [])
            raw_gt_by_id[iid][1].append(
                RawAnnotationRow(
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                    ignore_region=int(ign),
                    category_name=str(cat_name or ""),
                    geometry_tier=str(tier or "unknown"),
                )
            )
    for img_id, (src_ds, h_rows) in raw_gt_by_id.items():
        decision = sanitize_image_annotations(h_rows, source_dataset=src_ds)
        gt_boxes_by_id[img_id] = [
            [h_rows[idx].x1, h_rows[idx].y1, h_rows[idx].x2, h_rows[idx].y2]
            for idx in decision.kept_positive_indices
        ]

    gt_arrays: dict[int, np.ndarray] = {
        img_id: np.asarray(bxs, dtype=np.float32) if bxs else np.zeros((0, 4), dtype=np.float32)
        for img_id, bxs in gt_boxes_by_id.items()
    }

    raw_sam_by_id: dict[int, list[tuple[float, list[float], list[list[float]]]]] = defaultdict(list)
    with sqlite3.connect(f"file:{sam_db_path}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE sam_tids (image_id INTEGER PRIMARY KEY)")
        conn.executemany("INSERT INTO sam_tids VALUES (?)", [(i,) for i in train_ids])
        rows = conn.execute(
            """
            SELECT a.image_id, a.bbox_x1, a.bbox_y1, a.bbox_x2, a.bbox_y2, a.quad_json, a.stability_score
            FROM annotations a
            JOIN sam_tids t ON a.image_id = t.image_id
            WHERE a.is_discovered = 1
              AND a.is_suppressed = 0
              AND COALESCE(a.stability_score, 0.0) >= ?
              AND COALESCE(a.matched_gt_iou, 0.0) < ?
              AND (a.bbox_x2 - a.bbox_x1) >= ?
              AND (a.bbox_y2 - a.bbox_y1) >= ?
              AND (a.bbox_x2 - a.bbox_x1) * (a.bbox_y2 - a.bbox_y1) >= ?
            """,
            (min_stability, max_gt_iou, min_side, min_side, min_area),
        ).fetchall()

    for img_id_raw, x1_r, y1_r, x2_r, y2_r, q_json, stab_r in rows:
        img_id = int(img_id_raw)
        _, img_w, img_h = meta_by_id[img_id]
        x1 = max(0.0, min(float(img_w), float(x1_r)))
        y1 = max(0.0, min(float(img_h), float(y1_r)))
        x2 = max(0.0, min(float(img_w), float(x2_r)))
        y2 = max(0.0, min(float(img_h), float(y2_r)))
        bw = x2 - x1
        bh = y2 - y1
        area = bw * bh
        if bw < min_side or bh < min_side or area < min_area:
            continue
        if area > max_img_frac * float(max(img_w * img_h, 1)):
            continue
        aspect = max(bw / bh, bh / bw)
        if aspect > max_aspect:
            continue
        box = [x1, y1, x2, y2]
        quad = parse_quad_or_box(q_json, box, img_w, img_h)
        raw_sam_by_id[img_id].append((float(stab_r), box, quad))

    selected_by_id: dict[int, list[tuple[list[float], list[list[float]]]]] = {}
    total_regions = 0
    for img_id, candidates in raw_sam_by_id.items():
        # Sort by stability descending (tie-break larger boxes slightly less greedy via deterministic area)
        candidates.sort(key=lambda item: (-item[0], -((item[1][2] - item[1][0]) * (item[1][3] - item[1][1]))))
        cand_boxes = np.asarray([c[1] for c in candidates], dtype=np.float32)
        gt_arr = gt_arrays.get(img_id)
        if gt_arr is not None and gt_arr.shape[0] > 0:
            max_ious = compute_box_iou_np(cand_boxes, gt_arr).max(axis=1)
        else:
            max_ious = np.zeros((cand_boxes.shape[0],), dtype=np.float32)

        chosen_boxes: list[np.ndarray] = []
        chosen_items: list[tuple[list[float], list[list[float]]]] = []
        for idx, (_, box, quad) in enumerate(candidates):
            if float(max_ious[idx]) >= max_gt_iou:
                continue
            b_np = cand_boxes[idx : idx + 1]
            if chosen_boxes:
                chosen_arr = np.concatenate(chosen_boxes, axis=0)
                if float(compute_box_iou_np(b_np, chosen_arr).max()) >= intra_iou_thresh:
                    continue
            chosen_boxes.append(b_np)
            chosen_items.append((box, quad))
            if len(chosen_items) >= max_per_image:
                break

        if chosen_items:
            selected_by_id[img_id] = chosen_items
            total_regions += len(chosen_items)

    print(
        f"Selected {total_regions} diverse SAM candidates across {len(selected_by_id)} images "
        f"(avg {total_regions / max(len(selected_by_id), 1):.2f}/img).",
        flush=True,
    )
    return selected_by_id


def _load_image_crops_worker(
    task: tuple[int, Path, list[tuple[list[float], list[list[float]]]]],
) -> tuple[int, list[list[float]], list[list[list[float]]], np.ndarray, np.ndarray]:
    """Open one image and extract uint8 [N, 224, 224, 3] unflipped & flipped crop pairs."""
    img_id, img_path, items = task
    with Image.open(img_path) as im:
        pil_img = im.convert("RGB")
        orig_w, orig_h = pil_img.size

        u_crops: list[np.ndarray] = []
        f_crops: list[np.ndarray] = []
        kept_boxes: list[list[float]] = []
        kept_quads: list[list[list[float]]] = []
        for box, quad in items:
            x1, y1, x2, y2 = box
            bw = x2 - x1
            bh = y2 - y1
            cx1 = max(0, int(round(x1 - 0.15 * bw)))
            cy1 = max(0, int(round(y1 - 0.15 * bh)))
            cx2 = min(orig_w, int(round(x2 + 0.15 * bw)))
            cy2 = min(orig_h, int(round(y2 + 0.15 * bh)))
            if (cx2 - cx1) < 4 or (cy2 - cy1) < 4:
                continue
            crop = pil_img.crop((cx1, cy1, cx2, cy2)).resize((224, 224), Image.Resampling.BICUBIC)
            crop_flip = crop.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            u_crops.append(np.asarray(crop, dtype=np.uint8))
            f_crops.append(np.asarray(crop_flip, dtype=np.uint8))
            kept_boxes.append(box)
            kept_quads.append(quad)

    if not u_crops:
        return (
            img_id,
            [],
            [],
            np.empty((0, 224, 224, 3), dtype=np.uint8),
            np.empty((0, 224, 224, 3), dtype=np.uint8),
        )
    return img_id, kept_boxes, kept_quads, np.stack(u_crops, axis=0), np.stack(f_crops, axis=0)


@torch.no_grad()
def encode_uint8_batches(
    teacher: SiglipTeacher,
    u_arrays: list[np.ndarray],
    f_arrays: list[np.ndarray],
    batch_size: int = 256,
) -> tuple[Tensor, Tensor]:
    """Encode stacked uint8 crops in FP16 on GPU, returning FP16 CPU L2-normalized embeddings."""
    u_all = np.concatenate(u_arrays, axis=0)
    f_all = np.concatenate(f_arrays, axis=0)
    n_total = u_all.shape[0]
    u_embs: list[Tensor] = []
    f_embs: list[Tensor] = []
    device = teacher.device

    for start in range(0, n_total, batch_size):
        end = min(start + batch_size, n_total)
        u_t = torch.from_numpy(u_all[start:end]).to(device=device, dtype=torch.float16).permute(0, 3, 1, 2)
        f_t = torch.from_numpy(f_all[start:end]).to(device=device, dtype=torch.float16).permute(0, 3, 1, 2)
        u_t = u_t.div_(127.5).sub_(1.0)
        f_t = f_t.div_(127.5).sub_(1.0)
        both = torch.cat([u_t, f_t], dim=0)
        with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=(device.type == "cuda")):
            out = teacher.model.get_image_features(pixel_values=both)
            feat = out.pooler_output if hasattr(out, "pooler_output") and out.pooler_output is not None else out[0]
        norm = F.normalize(feat.to(torch.float32), dim=-1).to(dtype=torch.float16).cpu()
        m = end - start
        u_embs.append(norm[:m])
        f_embs.append(norm[m:])

    return torch.cat(u_embs, dim=0), torch.cat(f_embs, dim=0)


def build_and_save_vocab_banks(teacher: SiglipTeacher, vocab_dir: Path) -> tuple[VocabularyBank, VocabularyBank]:
    """Rebuild 37-class, 41-class, and 7-class distractor vocabulary banks."""
    vocab_dir.mkdir(parents=True, exist_ok=True)
    bank_37 = VocabularyBank.from_teacher(
        teacher,
        OPEN_VOCAB_CATEGORIES,
        synonyms=DEFAULT_CATEGORY_SYNONYMS,
    )
    bank_37_path = vocab_dir / "vocabulary_bank_37classes.pt"
    bank_37.save(bank_37_path)

    bank_41 = VocabularyBank.from_teacher(
        teacher,
        OPEN_VOCAB_CATEGORIES + ZERO_SHOT_EVAL_CATEGORIES,
        synonyms=DEFAULT_CATEGORY_SYNONYMS,
    )
    bank_41_path = vocab_dir / "vocabulary_bank_41classes_zeroshot.pt"
    bank_41.save(bank_41_path)

    merged_dist_synonyms = {**DEFAULT_CATEGORY_SYNONYMS, **EXTRA_DISTRACTOR_SYNONYMS}
    bank_dist = VocabularyBank.from_teacher(
        teacher,
        DISTRACTOR_CATEGORIES,
        synonyms=merged_dist_synonyms,
    )
    bank_dist_path = vocab_dir / "vocabulary_bank_distractors.pt"
    bank_dist.save(bank_dist_path)

    print(
        f"Saved vocab banks: {bank_37_path} ({len(bank_37.categories)}), "
        f"{bank_41_path} ({len(bank_41.categories)}), "
        f"{bank_dist_path} ({len(bank_dist.categories)})",
        flush=True,
    )
    return bank_37, bank_dist


def compute_and_save_visual_centroid(
    sam_cache: dict[int, dict[str, Any]],
    vocab_dir: Path,
    alpha: float = 0.45,
) -> Tensor:
    """Compute scaled mean unit embedding from SAM discovery regions and save to vocab_dir."""
    parts = [
        F.normalize(v["unflipped"].float(), dim=-1)
        for v in sam_cache.values()
        if v.get("unflipped") is not None and v["unflipped"].numel() > 0
    ]
    if not parts:
        centroid = torch.zeros((1, 768), dtype=torch.float32)
    else:
        all_u = torch.cat(parts, dim=0)
        centroid = alpha * all_u.mean(dim=0, keepdim=True)
    vocab_dir.mkdir(parents=True, exist_ok=True)
    cent_path = vocab_dir / "visual_centroid.pt"
    torch.save(centroid, cent_path)
    print(
        f"Saved visual centroid (alpha={alpha:.2f}, norm={float(centroid.norm()):.4f}) -> {cent_path}",
        flush=True,
    )
    return centroid


def _rect_quad(box: list[float]) -> list[list[float]]:
    """Return canonical axis-aligned 4-point clockwise quad [[x1,y1],[x2,y1],[x2,y2],[x1,y2]]."""
    return [
        [float(box[0]), float(box[1])],
        [float(box[2]), float(box[1])],
        [float(box[2]), float(box[3])],
        [float(box[0]), float(box[3])],
    ]


def _load_source_datasets_by_id(gt_db_path: Path | None) -> dict[int, str]:
    """Load mapping from image_id to source_dataset from the GT index SQLite DB."""
    if gt_db_path is None or not gt_db_path.exists():
        return {}
    with sqlite3.connect(f"file:{gt_db_path}?mode=ro&immutable=1", uri=True) as conn:
        rows = conn.execute("SELECT image_id, source_dataset FROM images").fetchall()
    return {int(iid): str(src) for iid, src in rows}


def merge_pseudo_gt_cache(
    *,
    sam_cache: dict[int, dict[str, Any]],
    pseudo_gt_path: Path,
    bank_37: VocabularyBank,
    bank_dist: VocabularyBank,
    visual_centroid: Tensor | None = None,
    meta_by_id: dict[int, tuple[str, int, int]] | None = None,
    gt_db_path: Path | None = None,
    max_new_per_class: int = 250,
    min_top1_cos: float = 0.048,
    min_margin: float = 0.008,
    max_existing_iou: float = 0.30,
) -> None:
    """Score cached SAM discovery crops against [vocab_37; distractors] and merge verified pseudo-GT."""
    _ = meta_by_id  # Reserved for image metadata lookups.
    source_by_id = _load_source_datasets_by_id(gt_db_path)
    existing: dict[int, dict[str, Any]] = {}
    if pseudo_gt_path.exists():
        loaded = torch.load(pseudo_gt_path, map_location="cpu", weights_only=False)
        existing = {int(k): dict(v) for k, v in loaded.items()}

    # Idempotency: strip any previously merged SAM discovery boxes (and driving "stairs" noise) while keeping original driving pseudo-GT.
    stripped_sam = 0
    for img_id in list(existing.keys()):
        cur = existing[img_id]
        cur_boxes = cur.get("boxes", [])
        cur_cats = cur.get("categories", ())
        sam_boxes = sam_cache.get(img_id, {}).get("boxes", ())
        is_driving = source_by_id.get(img_id, "") in DRIVING_SOURCES
        if not cur_boxes:
            continue
        sam_arr = np.asarray(sam_boxes, dtype=np.float32) if sam_boxes else np.zeros((0, 4), dtype=np.float32)
        keep_indices: list[int] = []
        for idx_b, b in enumerate(cur_boxes):
            c_b = cur_cats[idx_b] if idx_b < len(cur_cats) else ""
            if is_driving and c_b == "stairs":
                stripped_sam += 1
                continue
            if sam_arr.shape[0] > 0:
                b_arr = np.asarray(b, dtype=np.float32)
                max_diff = np.abs(sam_arr - b_arr[None, :]).max(axis=1)
                if bool((max_diff < 1e-2).any()):
                    stripped_sam += 1
                    continue
            keep_indices.append(idx_b)
        if not keep_indices:
            del existing[img_id]
        elif len(keep_indices) < len(cur_boxes):
            cur["boxes"] = [cur_boxes[i] for i in keep_indices]
            cur["categories"] = [cur["categories"][i] for i in keep_indices]
            cur["unflipped"] = cur["unflipped"][keep_indices]
            cur["flipped"] = cur["flipped"][keep_indices]
            if "quads" in cur and len(cur["quads"]) == len(cur_boxes):
                cur["quads"] = [_rect_quad(cur_boxes[i]) for i in keep_indices]
    if stripped_sam > 0:
        print(
            f"Stripped {stripped_sam} prior SAM-merged entries; retained {sum(len(v['boxes']) for v in existing.values())} original pseudo-GT objects.",
            flush=True,
        )

    all_text = torch.cat([bank_37.embeddings.float(), bank_dist.embeddings.float()], dim=0)
    if visual_centroid is not None and visual_centroid.numel() > 0:
        c_f = visual_centroid.float()
        if c_f.ndim == 1:
            c_f = c_f.unsqueeze(0)
        prompt_bias = (c_f @ all_text.t()).clamp(min=0.0)
    else:
        prompt_bias = torch.zeros((1, all_text.shape[0]), dtype=torch.float32)

    fg_count = len(bank_37.categories)
    target_set = set(TARGET_PSEUDO_GT_CLASSES)
    cat_names = list(bank_37.categories)

    candidates_by_class: dict[str, list[tuple[float, int, list[float], list[list[float]], Tensor, Tensor]]] = defaultdict(list)

    for img_id, rec in sam_cache.items():
        u = rec["unflipped"]
        f = rec["flipped"]
        if u.numel() == 0:
            continue
        emb = F.normalize(u.float() + f.float(), dim=-1)
        sims = (emb @ all_text.t()) - prompt_bias  # [N, 37 + 7]
        top2_vals, top2_idx = torch.topk(sims, k=2, dim=-1)

        ex_rec = existing.get(img_id)
        ex_boxes = np.asarray(ex_rec["boxes"], dtype=np.float32) if (ex_rec and ex_rec.get("boxes")) else np.zeros((0, 4), dtype=np.float32)

        src_name = source_by_id.get(img_id, "")
        for i in range(emb.shape[0]):
            t1_idx = int(top2_idx[i, 0].item())
            if t1_idx >= fg_count:
                continue
            cname = cat_names[t1_idx]
            if cname not in target_set:
                continue
            if cname == "stairs" and src_name in DRIVING_SOURCES:
                continue
            t1_val = float(top2_vals[i, 0].item())
            margin = float((top2_vals[i, 0] - top2_vals[i, 1]).item())
            if t1_val < min_top1_cos or margin < min_margin:
                continue
            box = rec["boxes"][i]
            if ex_boxes.shape[0] > 0:
                if float(compute_box_iou_np(np.asarray([box], dtype=np.float32), ex_boxes).max()) >= max_existing_iou:
                    continue
            rank_score = margin + 0.5 * t1_val
            candidates_by_class[cname].append(
                (rank_score, img_id, box, _rect_quad(box), u[i : i + 1], f[i : i + 1])
            )

    added_counts: dict[str, int] = {}
    new_by_img: dict[int, list[tuple[str, list[float], list[list[float]], Tensor, Tensor]]] = defaultdict(list)
    for cname in TARGET_PSEUDO_GT_CLASSES:
        cands = candidates_by_class.get(cname, [])
        cands.sort(key=lambda x: -x[0])
        taken = cands[:max_new_per_class]
        added_counts[cname] = len(taken)
        for _, img_id, box, quad, u_t, f_t in taken:
            new_by_img[img_id].append((cname, box, quad, u_t, f_t))

    for img_id, items in new_by_img.items():
        if img_id in existing and len(existing[img_id].get("boxes", ())) > 0:
            cur = existing[img_id]
            cur_quads = [_rect_quad(b) for b in cur["boxes"]]
            cur["boxes"] = list(cur["boxes"]) + [it[1] for it in items]
            cur["quads"] = cur_quads + [it[2] for it in items]
            cur["categories"] = list(cur["categories"]) + [it[0] for it in items]
            cur["unflipped"] = torch.cat([cur["unflipped"]] + [it[3] for it in items], dim=0)
            cur["flipped"] = torch.cat([cur["flipped"]] + [it[4] for it in items], dim=0)
        else:
            existing[img_id] = {
                "boxes": [it[1] for it in items],
                "quads": [it[2] for it in items],
                "categories": [it[0] for it in items],
                "unflipped": torch.cat([it[3] for it in items], dim=0),
                "flipped": torch.cat([it[4] for it in items], dim=0),
            }

    # Ensure every entry in merged pseudo-GT has canonical axis-aligned 4-point quads.
    for v in existing.values():
        v["quads"] = [_rect_quad(b) for b in v.get("boxes", ())]

    torch.save(existing, pseudo_gt_path)
    total_objs = sum(len(v["categories"]) for v in existing.values())
    print(f"Merged open-world pseudo-GT -> {pseudo_gt_path}: {len(existing)} images, {total_objs} objects.", flush=True)
    print(f"Added per class (up to {max_new_per_class}): {added_counts}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare SigLIP distillation assets and vocab banks.")
    parser.add_argument("--gt-crops-cache", type=Path, default=Path("artifacts/cache/siglip_gt_crops_8k.pt"))
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--sam-db", type=Path, default=Path("artifacts/pseudo_labels/sam_discovery_enriched.sqlite"))
    parser.add_argument("--holdout-manifest", type=Path, default=Path("artifacts/dashboard_holdout_8k_8k_full.json"))
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument("--vocab-dir", type=Path, default=Path("artifacts/sota_open_vocab"))
    parser.add_argument("--sam-cache-out", type=Path, default=Path("artifacts/cache/siglip_sam_discovery_8k.pt"))
    parser.add_argument("--pseudo-gt-cache", type=Path, default=Path("artifacts/cache/open_world_pseudo_gt_8k.pt"))
    parser.add_argument("--max-per-image", type=int, default=6)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--merge-only", action="store_true", help="Load existing sam-cache-out and vocab banks and merge pseudo-GT.")
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.merge_only:
        device = torch.device(args.device)
        teacher = SiglipTeacher(device=device)
        bank_37, bank_dist = build_and_save_vocab_banks(teacher, args.vocab_dir)
        del teacher
        if device.type == "cuda":
            torch.cuda.empty_cache()
        sam_cache = torch.load(args.sam_cache_out, map_location="cpu", weights_only=False)
        visual_centroid = compute_and_save_visual_centroid(sam_cache, args.vocab_dir)
        merge_pseudo_gt_cache(
            sam_cache=sam_cache,
            pseudo_gt_path=args.pseudo_gt_cache,
            bank_37=bank_37,
            bank_dist=bank_dist,
            visual_centroid=visual_centroid,
            gt_db_path=args.gt_index,
        )
        return
    gt_cache = torch.load(args.gt_crops_cache, map_location="cpu", weights_only=False)
    train_ids = sorted(int(k) for k in gt_cache["offsets"].keys())

    meta_by_id = verify_zero_cohort_overlap(
        args.gt_index, train_ids, args.holdout_manifest, args.image_root
    )
    selected_by_id = select_sam_candidates(
        gt_db_path=args.gt_index,
        sam_db_path=args.sam_db,
        meta_by_id=meta_by_id,
        max_per_image=args.max_per_image,
    )

    device = torch.device(args.device)
    teacher = SiglipTeacher(device=device)
    bank_37, bank_dist = build_and_save_vocab_banks(teacher, args.vocab_dir)

    tasks: list[tuple[int, Path, list[tuple[list[float], list[list[float]]]]]] = []
    for img_id in sorted(selected_by_id.keys()):
        fn, _, _ = meta_by_id[img_id]
        img_path = resolve_image_path(fn, args.image_root)
        if img_path.is_file():
            tasks.append((img_id, img_path, selected_by_id[img_id]))

    sam_cache: dict[int, dict[str, Any]] = {}
    chunk_size = 512  # ~3,000 crops per flush chunk -> low peak RAM
    args.sam_cache_out.parent.mkdir(parents=True, exist_ok=True)

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for start in range(0, len(tasks), chunk_size):
            chunk = tasks[start : start + chunk_size]
            results = list(pool.map(_load_image_crops_worker, chunk))
            u_list: list[np.ndarray] = []
            f_list: list[np.ndarray] = []
            meta_chunk: list[tuple[int, list[list[float]], list[list[list[float]]], int]] = []
            for img_id, boxes, quads, u_arr, f_arr in results:
                n = len(boxes)
                if n == 0:
                    continue
                u_list.append(u_arr)
                f_list.append(f_arr)
                meta_chunk.append((img_id, boxes, quads, n))

            if not u_list:
                continue
            u_embs, f_embs = encode_uint8_batches(
                teacher, u_list, f_list, batch_size=args.batch_size
            )
            cursor = 0
            for img_id, boxes, quads, n in meta_chunk:
                sam_cache[img_id] = {
                    "boxes": boxes,
                    "quads": quads,
                    "unflipped": u_embs[cursor : cursor + n],
                    "flipped": f_embs[cursor : cursor + n],
                }
                cursor += n

            done_imgs = min(start + chunk_size, len(tasks))
            total_encoded = sum(len(v["boxes"]) for v in sam_cache.values())
            print(
                f"Encoded SAM discovery chunk {done_imgs}/{len(tasks)} images "
                f"({total_encoded} regions total)...",
                flush=True,
            )

    torch.save(sam_cache, args.sam_cache_out)
    print(f"Saved SAM discovery SigLIP cache -> {args.sam_cache_out}", flush=True)

    visual_centroid = compute_and_save_visual_centroid(sam_cache, args.vocab_dir)
    merge_pseudo_gt_cache(
        sam_cache=sam_cache,
        pseudo_gt_path=args.pseudo_gt_cache,
        bank_37=bank_37,
        bank_dist=bank_dist,
        visual_centroid=visual_centroid,
        meta_by_id=meta_by_id,
        gt_db_path=args.gt_index,
    )


if __name__ == "__main__":
    main()
