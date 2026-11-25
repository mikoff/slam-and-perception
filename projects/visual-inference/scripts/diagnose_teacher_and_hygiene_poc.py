#!/usr/bin/env python3
"""Option B PoC: Annotation Hygiene Audit + 72-Image OWLv2 Teacher & Merged Target Preview.

Evaluates:
  1. Quantitative before/after impact of `annotation_hygiene.py` across the 8k cohort
     and class-agnostic localization recall on `cohort_val_800` (`nuimages`, `woodscape_rgb_fisheye`).
  2. Stratified 72-image comparison (8 audited diagnostic images + 64 diverse holdout images)
     running tuned `google/owlv2-large-patch14-ensemble` (and optional `grounding-dino-base`)
     against Cleaned GT, Current SOTA Student Opt-B HBB (384x384), and a live
     `Merged Training Target Preview` (Clean GT + OWLv2 Missing-Class Pseudo-Labels).
  3. Renders 2x2 comparison PNGs for the core 8 images, an interactive paginated HTML viewer,
     and `poc_summary.json`.
"""

from __future__ import annotations

import argparse
import base64
from collections import defaultdict
import gc
from html import escape
import io
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

from PIL import Image, ImageDraw, ImageFont
import torch
from torch import Tensor
from torch.utils.data import DataLoader
from torchvision.ops import box_iou, nms
from transformers import (
    AutoModelForZeroShotObjectDetection,
    AutoProcessor,
    Owlv2ForObjectDetection,
    Owlv2Processor,
)

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from student_detector.augmentation import canvas_box_to_native
from student_detector.config import AugmentationConfig
from student_detector.data import ProposalTransform, collate_proposal_samples
from student_detector.decoder import InferenceDecoder
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
    get_stage2_cohort_ids,
)
from student_detector.model import StudentDetector
from student_detector.ontology import (
    SOURCE_ANNOTATED_CATEGORIES,
    map_to_canonical,
)
from student_detector.open_vocab_heads import RoIPooledSemanticHead, filter_semantic_detections
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT
from student_detector.targets import point_validity_from_pixel_mask
from student_detector.vocabulary import VocabularyBank

TARGET_IMAGE_IDS: tuple[int, ...] = (
    189270,
    256307,
    188347,
    188775,
    188919,
    501374,
    500459,
    600267,
)

DATASET_ORDER: tuple[str, ...] = (
    "nuimages",
    "woodscape_rgb_fisheye",
    "bdd100k_images_100k",
    "ade20k",
    "open_images_subset",
    "coco_2017",
)

RARE_INFRA_CATEGORIES: frozenset[str] = frozenset(
    {
        "trash_bin",
        "fire_hydrant",
        "traffic_sign",
        "traffic_light",
        "signboard",
        "traffic_cone",
        "construction_barrier",
        "pole",
        "bench",
        "dog",
        "stroller",
        "wheelchair",
        "pillar",
        "luggage_and_bags",
        "cart",
    }
)

STUFF_CATEGORIES: frozenset[str] = frozenset({"building", "tree", "fence"})

TEACHER_PROMPT_GROUPS: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = (
    (
        "vehicles_people",
        (
            ("car", "car"),
            ("truck", "truck"),
            ("bus", "bus"),
            ("person", "pedestrian"),
            ("bicycle", "cyclist"),
            ("motorcycle", "cyclist"),
            ("stroller", "stroller"),
            ("wheelchair", "wheelchair"),
            ("dog", "dog"),
        ),
    ),
    (
        "road_infra_signs",
        (
            ("traffic sign", "traffic_sign"),
            ("stop sign", "traffic_sign"),
            ("traffic light", "traffic_light"),
            ("traffic cone", "traffic_cone"),
            ("trash bin", "trash_bin"),
            ("garbage can", "trash_bin"),
            ("orange road barricade", "construction_barrier"),
            ("plastic traffic jersey barrier", "construction_barrier"),
            ("street light", "street_light"),
            ("utility pole", "pole"),
            ("fire hydrant", "fire_hydrant"),
            ("bench", "bench"),
            ("signboard", "signboard"),
            ("fence", "fence"),
        ),
    ),
    (
        "architecture_indoor",
        (
            ("building", "building"),
            ("tree", "tree"),
            ("window", "window"),
            ("door", "door"),
            ("pillar", "pillar"),
            ("chair", "chair"),
            ("table", "table"),
            ("cabinet", "cabinet"),
            ("luggage", "luggage_and_bags"),
            ("shopping cart", "cart"),
        ),
    ),
)

OWLV2_MODEL_ID = "google/owlv2-large-patch14-ensemble"
GDINO_MODEL_ID = "IDEA-Research/grounding-dino-base"


def compute_ap(records: list[tuple[float, int]], total_gt: int) -> float:
    """Compute 101-point interpolated AP given confidence-sorted TP/FP flags."""
    if total_gt <= 0 or not records:
        return 0.0
    sorted_recs = sorted(records, key=lambda x: -x[0])
    tp_cum = 0
    fp_cum = 0
    precisions: list[float] = []
    recalls: list[float] = []
    for _, is_tp in sorted_recs:
        if is_tp:
            tp_cum += 1
        else:
            fp_cum += 1
        precisions.append(tp_cum / max(tp_cum + fp_cum, 1))
        recalls.append(tp_cum / total_gt)
    ap = 0.0
    for step in range(101):
        r_thresh = step / 100.0
        p_at_r = max((p for p, r in zip(precisions, recalls, strict=True) if r >= r_thresh), default=0.0)
        ap += p_at_r
    return ap / 101.0


def audit_cohort_hygiene(gt_index: Path) -> dict[str, dict[str, Any]]:
    """Run non-destructive hygiene statistics across all images in the 8k cohort."""
    cohort_ids = sorted(get_stage2_cohort_ids(gt_index, target_count=8000))
    grouped: dict[int, tuple[str, list[RawAnnotationRow]]] = {}
    with sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE poc_cids (image_id INTEGER PRIMARY KEY)")
        conn.executemany("INSERT INTO poc_cids VALUES (?)", [(i,) for i in cohort_ids])
        rows = conn.execute(
            """
            SELECT a.image_id, i.source_dataset, a.x1, a.y1, a.x2, a.y2,
                   a.ignore_region, a.category_name, a.geometry_tier
            FROM annotations a
            JOIN poc_cids c ON a.image_id = c.image_id
            JOIN images i ON a.image_id = i.image_id
            ORDER BY a.image_id, a.rowid
            """
        ).fetchall()
    for r in rows:
        img_id = int(r[0])
        if img_id not in grouped:
            grouped[img_id] = (str(r[1]), [])
        grouped[img_id][1].append(
            RawAnnotationRow(
                x1=float(r[2]),
                y1=float(r[3]),
                x2=float(r[4]),
                y2=float(r[5]),
                ignore_region=int(r[6]),
                category_name=str(r[7]),
                geometry_tier=str(r[8]),
            )
        )

    agg: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for _img_id, (src_ds, h_rows) in grouped.items():
        dec = sanitize_image_annotations(h_rows, source_dataset=src_ds)
        agg[src_ds]["images"] += 1
        for k, v in dec.stats.items():
            agg[src_ds][k] += v

    summary: dict[str, dict[str, Any]] = {}
    for src_ds, d in sorted(agg.items()):
        raw_p = max(d["raw_positives"], 1)
        kept_p = d["kept_positives"]
        summary[src_ds] = {
            **dict(d),
            "reduction_pct": round(100.0 * (d["raw_positives"] - kept_p) / raw_p, 2),
        }
    return summary


def evaluate_val_recall_hygiene_delta(
    *,
    ckpt_path: Path,
    gt_index: Path,
    image_root: Path,
    device: torch.device,
) -> dict[str, dict[str, dict[str, Any]]]:
    """Compare StudentDetector HBB localization recall before vs after GT hygiene on val."""
    ds_raw = HumanGTHBBDataset(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="val",
        input_size=384,
        cohort_size=8000,
        sanitize_annotations=False,
    )
    ds_clean = HumanGTHBBDataset(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="val",
        input_size=384,
        cohort_size=8000,
        sanitize_annotations=True,
    )
    target_sources = {"nuimages", "woodscape_rgb_fisheye"}
    ds_raw.images = [r for r in ds_raw.images if r["source_dataset"] in target_sources]
    ds_clean.images = [r for r in ds_clean.images if r["source_dataset"] in target_sources]

    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    det_cfg = ckpt.get("detector_config", {})
    strides = tuple(int(s) for s in det_cfg.get("strides", (4, 8, 16, 32)))
    detector = StudentDetector(
        pretrained_backbone=False,
        fpn_channels=int(det_cfg.get("fpn_channels", 128)),
        neck_type=str(det_cfg.get("neck_type", "pan")),
        strides=strides,
        dual_head=bool(det_cfg.get("dual_head", True)),
        use_3dmf=bool(det_cfg.get("use_3dmf", True)),
    ).to(device)
    detector.load_state_dict(ckpt["model_state_dict"])
    detector.eval()

    decoder = InferenceDecoder(
        strides=strides,
        top_k=500,
        nms_iou_threshold=0.50,
        max_detections=100,
        score_mode="objectness",
        nms_mode="containment_aware",
        score_threshold=0.01,
    )

    cached_preds: dict[int, tuple[Tensor, Tensor]] = {}
    loader = DataLoader(ds_clean, batch_size=16, shuffle=False, num_workers=2, collate_fn=collate_proposal_samples)
    with torch.no_grad():
        for images, samples in loader:
            images = images.to(device, non_blocking=True)
            feats = detector.forward_features(images)
            raw_out = detector.forward_eval_head(feats)
            shapes = tuple((t.shape[-2], t.shape[-1]) for t in raw_out.objectness)
            lvl_masks = [point_validity_from_pixel_mask(s.valid_mask.to(device), shapes, strides)[1] for s in samples]
            batched_masks = tuple(torch.stack([m[lvl] for m in lvl_masks], dim=0) for lvl in range(len(strides)))
            dets = decoder(raw_out, (384, 384), batched_masks)
            for s, d in zip(samples, dets, strict=True):
                cached_preds[int(s.image_id)] = (d.boxes.detach().cpu(), d.scores.detach().cpu())

    del detector
    gc.collect()
    if device.type == "cuda":
        torch.cuda.empty_cache()

    results: dict[str, dict[str, dict[str, Any]]] = {}
    for label, ds in (("before_raw", ds_raw), ("after_clean", ds_clean)):
        by_src: dict[str, dict[str, Any]] = {
            src: {
                "images": 0,
                "num_gt": 0,
                "rec50": 0,
                "rec75": 0,
                "matched_gt_030": 0,
                "dup_gt_030": 0,
                "ap_recs": [],
            }
            for src in sorted(target_sources)
        }
        for idx in range(len(ds)):
            s = ds[idx]
            src = s.source_dataset
            acc = by_src[src]
            acc["images"] += 1
            pred_b, pred_s = cached_preds[int(s.image_id)]
            gt_b = s.boxes.cpu()
            ign_b = s.ignore_boxes.cpu()
            n_gt = int(gt_b.shape[0])
            n_p = int(pred_b.shape[0])
            acc["num_gt"] += n_gt
            if n_p == 0:
                continue
            iou_m = box_iou(pred_b, gt_b) if n_gt > 0 else torch.zeros((n_p, 0))
            ign_m = box_iou(pred_b, ign_b) if ign_b.shape[0] > 0 else torch.zeros((n_p, 0))
            if n_gt > 0:
                max_g = iou_m.max(dim=0).values
                acc["rec50"] += int((max_g >= 0.50).sum().item())
                acc["rec75"] += int((max_g >= 0.75).sum().item())
                hits = (iou_m[pred_s >= 0.30] >= 0.50).sum(dim=0)
                acc["matched_gt_030"] += int((hits >= 1).sum().item())
                acc["dup_gt_030"] += int((hits >= 2).sum().item())
            gt_used = torch.zeros(n_gt, dtype=torch.bool)
            for p_i in range(n_p):
                sc = float(pred_s[p_i].item())
                if n_gt > 0:
                    row = iou_m[p_i].clone()
                    row[gt_used] = -1.0
                    b_iou, b_idx = row.max(dim=0)
                    if float(b_iou.item()) >= 0.50:
                        gt_used[b_idx] = True
                        acc["ap_recs"].append((sc, 1))
                        continue
                if ign_m.shape[1] > 0 and float(ign_m[p_i].max().item()) >= 0.50:
                    continue
                acc["ap_recs"].append((sc, 0))

        results[label] = {}
        for src, acc in by_src.items():
            d_gt = max(acc["num_gt"], 1)
            results[label][src] = {
                "images": acc["images"],
                "num_gt": acc["num_gt"],
                "recall_100_50": round(acc["rec50"] / d_gt, 4),
                "recall_100_75": round(acc["rec75"] / d_gt, 4),
                "dup_gt_rate": round(acc["dup_gt_030"] / max(acc["matched_gt_030"], 1), 4),
                "class_agnostic_ap50": round(compute_ap(acc["ap_recs"], acc["num_gt"]), 4),
            }
    return results


def select_poc_image_ids(
    gt_index: Path,
    holdout_manifest: Path,
    image_root: Path,
    num_images: int = 72,
) -> tuple[list[int], set[int]]:
    """Select `num_images` stratified IDs: 8 pinned core IDs + rare/diverse holdout images."""
    per_ds_quota = max(1, num_images // len(DATASET_ORDER))
    raw_manifest = json.loads(holdout_manifest.read_text()) if holdout_manifest.exists() else {}
    ho_records = (
        raw_manifest["images"]
        if isinstance(raw_manifest, dict) and "images" in raw_manifest
        else (raw_manifest if isinstance(raw_manifest, list) else [])
    )
    cand_ids = [int(r["image_id"]) for r in ho_records]
    all_check_ids = sorted(set(TARGET_IMAGE_IDS) | set(cand_ids))

    existing_meta: dict[int, tuple[str, str]] = {}
    cats_by_id: dict[int, set[str]] = defaultdict(set)
    with sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True) as conn:
        conn.execute("CREATE TEMP TABLE poc_sel (image_id INTEGER PRIMARY KEY)")
        conn.executemany("INSERT INTO poc_sel VALUES (?)", [(i,) for i in all_check_ids])
        for img_id, fn, src in conn.execute(
            """
            SELECT img.image_id, img.file_name, img.source_dataset
            FROM images img
            JOIN poc_sel s ON img.image_id = s.image_id
            """
        ):
            if (image_root / str(fn)).exists():
                existing_meta[int(img_id)] = (str(fn), str(src))
        for img_id, raw_cat in conn.execute(
            """
            SELECT a.image_id, a.category_name
            FROM annotations a
            JOIN poc_sel s ON a.image_id = s.image_id
            WHERE a.ignore_region = 0
            """
        ):
            canon = map_to_canonical(str(raw_cat))
            if canon is not None:
                cats_by_id[int(img_id)].add(canon)

    core_ids = [i for i in TARGET_IMAGE_IDS if i in existing_meta]
    core_set = set(core_ids)
    counts: dict[str, int] = defaultdict(int)
    for cid in core_ids:
        counts[existing_meta[cid][1]] += 1

    if num_images <= len(core_ids):
        return core_ids[:num_images], core_set

    pool_by_ds: dict[str, list[tuple[int, int, int]]] = defaultdict(list)
    for img_id in cand_ids:
        if img_id in core_set or img_id not in existing_meta:
            continue
        _fn, src = existing_meta[img_id]
        cats = cats_by_id.get(img_id, set())
        rare_hits = len(cats & RARE_INFRA_CATEGORIES)
        pool_by_ds[src].append((-rare_hits, -len(cats), img_id))

    for src in pool_by_ds:
        pool_by_ds[src].sort()

    picked_by_ds: dict[str, list[int]] = defaultdict(list)
    used_ids = set(core_ids)
    for src in DATASET_ORDER:
        need = max(0, per_ds_quota - counts[src])
        for _neg_rare, _neg_all, img_id in pool_by_ds.get(src, []):
            if need <= 0:
                break
            if img_id not in used_ids:
                picked_by_ds[src].append(img_id)
                used_ids.add(img_id)
                need -= 1

    selected: list[int] = list(core_ids)
    # Interleave remaining holdout picks across datasets for page-1 variety.
    cursors = {src: 0 for src in DATASET_ORDER}
    while len(selected) < num_images:
        progressed = False
        for src in DATASET_ORDER:
            idx = cursors[src]
            lst = picked_by_ds.get(src, [])
            if idx < len(lst):
                selected.append(lst[idx])
                cursors[src] += 1
                progressed = True
                if len(selected) >= num_images:
                    break
        if not progressed:
            break

    # Top up if any dataset had fewer candidates or non-multiple target count.
    if len(selected) < num_images:
        leftover = [
            (r, c, img_id)
            for src in DATASET_ORDER
            for r, c, img_id in pool_by_ds.get(src, [])
            if img_id not in used_ids
        ]
        leftover.sort()
        for _r, _c, img_id in leftover:
            selected.append(img_id)
            used_ids.add(img_id)
            if len(selected) >= num_images:
                break

    return selected, core_set


def load_diagnostic_images(
    gt_index: Path,
    image_root: Path,
    target_ids: list[int],
    core_set: set[int],
) -> list[dict[str, Any]]:
    """Load metadata, file path, and GT Hygiene Diff buckets for selected images."""
    placeholders = ",".join("?" for _ in target_ids)
    with sqlite3.connect(f"file:{gt_index}?mode=ro&immutable=1", uri=True) as conn:
        img_rows = conn.execute(
            f"""
            SELECT image_id, file_name, width, height, source_dataset, camera_type
            FROM images
            WHERE image_id IN ({placeholders})
            """,
            target_ids,
        ).fetchall()
        img_by_id = {int(r[0]): r for r in img_rows}
        ann_rows = conn.execute(
            f"""
            SELECT image_id, x1, y1, x2, y2, ignore_region, category_name, geometry_tier
            FROM annotations
            WHERE image_id IN ({placeholders})
            ORDER BY image_id, rowid
            """,
            target_ids,
        ).fetchall()

    raw_by_id: dict[int, list[RawAnnotationRow]] = defaultdict(list)
    for r in ann_rows:
        raw_by_id[int(r[0])].append(
            RawAnnotationRow(
                x1=float(r[1]),
                y1=float(r[2]),
                x2=float(r[3]),
                y2=float(r[4]),
                ignore_region=int(r[5]),
                category_name=str(r[6]),
                geometry_tier=str(r[7]),
            )
        )

    out: list[dict[str, Any]] = []
    for img_id in target_ids:
        ir = img_by_id[img_id]
        src_ds = str(ir[4])
        img_path = image_root / str(ir[1])
        h_rows = raw_by_id.get(img_id, [])
        dec = sanitize_image_annotations(h_rows, source_dataset=src_ds)
        dropped_set = set(dec.dropped_indices)
        demoted_set = set(dec.demoted_to_ignore_indices)
        kept_set = set(dec.kept_positive_indices)

        gt_kept: list[dict[str, Any]] = []
        gt_dropped: list[dict[str, Any]] = []
        gt_demoted: list[dict[str, Any]] = []
        for idx, hr in enumerate(h_rows):
            if hr.ignore_region != 0:
                continue
            canon = map_to_canonical(hr.category_name) or hr.category_name
            entry = {
                "box": [round(hr.x1, 1), round(hr.y1, 1), round(hr.x2, 1), round(hr.y2, 1)],
                "label": canon,
                "raw_label": hr.category_name,
                "tier": hr.geometry_tier,
            }
            if idx in kept_set:
                gt_kept.append(entry)
            elif idx in dropped_set:
                gt_dropped.append(entry)
            elif idx in demoted_set:
                gt_demoted.append(entry)

        out.append(
            {
                "image_id": img_id,
                "is_core": img_id in core_set,
                "file_name": str(ir[1]),
                "image_path": str(img_path),
                "width": int(ir[2]),
                "height": int(ir[3]),
                "source_dataset": src_ds,
                "camera_type": str(ir[5]),
                "hygiene_stats": dec.stats,
                "gt_kept": gt_kept,
                "gt_dropped": gt_dropped,
                "gt_demoted": gt_demoted,
            }
        )
    return out


def run_student_option_b_inference(
    items: list[dict[str, Any]],
    *,
    ckpt_path: Path,
    vocab_37_path: Path,
    vocab_dist_path: Path,
    centroid_path: Path,
    device: torch.device,
) -> tuple[dict[int, list[dict[str, Any]]], float]:
    """Run current SOTA HBB Option B inference on selected PoC images."""
    vocab_37 = VocabularyBank.load(vocab_37_path, device=device)
    dist_emb = VocabularyBank.load(vocab_dist_path, device=device).embeddings if vocab_dist_path.exists() else None
    vc_raw = torch.load(centroid_path, map_location=device, weights_only=False) if centroid_path.exists() else None
    vc = vc_raw["visual_centroid"].to(device) if isinstance(vc_raw, dict) and "visual_centroid" in vc_raw else vc_raw

    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    det_cfg = ckpt.get("detector_config", {})
    strides = tuple(int(s) for s in det_cfg.get("strides", (4, 8, 16, 32)))
    detector = StudentDetector(
        pretrained_backbone=False,
        fpn_channels=int(det_cfg.get("fpn_channels", 128)),
        neck_type=str(det_cfg.get("neck_type", "pan")),
        strides=strides,
        dual_head=bool(det_cfg.get("dual_head", True)),
        use_3dmf=bool(det_cfg.get("use_3dmf", True)),
    ).to(device)
    detector.load_state_dict(ckpt["model_state_dict"])
    detector.eval()

    head_b = RoIPooledSemanticHead.from_config(ckpt["head_b_config"]).to(device)
    head_b.load_state_dict(ckpt["head_b_state_dict"], strict=False)
    if dist_emb is not None and (head_b.distractor_embeddings is None or head_b.distractor_embeddings.numel() == 0):
        head_b.set_distractor_embeddings(dist_emb)
    if vc is not None and (head_b.visual_centroid is None or head_b.visual_centroid.numel() == 0):
        head_b.set_visual_centroid(vc)
    head_b.eval()

    decoder = InferenceDecoder(
        strides=strides,
        top_k=500,
        nms_iou_threshold=0.50,
        max_detections=100,
        score_mode="objectness",
        nms_mode="containment_aware",
        score_threshold=0.05,
    )
    transform = ProposalTransform(384, AugmentationConfig(), training=False, tiny_area=4.0, tiny_min_side=2.0)
    text_37 = vocab_37.embeddings.to(device)
    preds_by_id: dict[int, list[dict[str, Any]]] = {}
    latencies_ms: list[float] = []

    with torch.no_grad():
        for idx, item in enumerate(items):
            with Image.open(item["image_path"]) as im:
                rgb = im.convert("RGB")
                transformed = transform(rgb, torch.empty((0, 4)), torch.empty((0, 4)), seed=42)
            img_t = transformed[0].unsqueeze(0).to(device)
            valid_mask = transformed[4].to(device)
            tf_meta = transformed[5]

            if device.type == "cuda":
                torch.cuda.synchronize()
            t0 = time.perf_counter()

            feats = detector.forward_features(img_t)
            raw_out = detector.forward_eval_head(feats)
            shapes = tuple((t.shape[-2], t.shape[-1]) for t in raw_out.objectness)
            lvl_masks = point_validity_from_pixel_mask(valid_mask, shapes, strides)[1]
            batched_masks = tuple(m.unsqueeze(0) for m in lvl_masks)
            det = decoder(raw_out, (384, 384), batched_masks)[0]

            vocab_cats = tuple(vocab_37.categories)
            kept_preds: list[dict[str, Any]] = []
            if det.boxes.shape[0] > 0:
                z_b = head_b(feats, [det.boxes], image_shapes=[(384, 384)])
                _, probs_b = head_b.classify_embeddings(z_b, text_37)
                keep = filter_semantic_detections(
                    det.boxes,
                    det.scores,
                    probs_b,
                    vocab_cats,
                )
                for p_i in keep.tolist():
                    prob_row = probs_b[p_i]
                    top_p, cls_i = prob_row.max(dim=-1)
                    obj_s = float(det.scores[p_i].item())
                    cls_p = float(top_p.item())
                    joint = float((obj_s * cls_p) ** 0.5)
                    nb = canvas_box_to_native(
                        det.boxes[p_i],
                        tf_meta,
                        (item["height"], item["width"]),
                        horizontal_flip=False,
                    )
                    kept_preds.append(
                        {
                            "box": [round(float(v), 1) for v in nb],
                            "label": vocab_cats[int(cls_i.item())],
                            "score": round(joint, 3),
                            "obj": round(obj_s, 3),
                            "prob": round(cls_p, 3),
                        }
                    )

            if device.type == "cuda":
                torch.cuda.synchronize()
            dt = (time.perf_counter() - t0) * 1000.0
            if idx > 0:
                latencies_ms.append(dt)
            kept_preds.sort(key=lambda x: -x["score"])
            preds_by_id[int(item["image_id"])] = kept_preds

    del detector, head_b
    gc.collect()
    if device.type == "cuda":
        torch.cuda.empty_cache()
    avg_ms = sum(latencies_ms) / max(len(latencies_ms), 1)
    return preds_by_id, round(avg_ms, 2)


def _postprocess_teacher_boxes(
    raw_boxes: Tensor,
    raw_scores: Tensor,
    raw_canon_labels: list[str],
    img_w: int,
    img_h: int,
    *,
    per_class_iou: float = 0.45,
    cross_class_iou: float = 0.75,
) -> list[dict[str, Any]]:
    """Clamp, filter <4px noise, run per-class NMS and cross-class duplicate suppression."""
    if raw_boxes.numel() == 0:
        return []
    boxes = raw_boxes.float().cpu().clone()
    scores = raw_scores.float().cpu().clone()
    boxes[:, 0::2].clamp_(0.0, float(img_w))
    boxes[:, 1::2].clamp_(0.0, float(img_h))
    bw = boxes[:, 2] - boxes[:, 0]
    bh = boxes[:, 3] - boxes[:, 1]
    valid = (bw >= 4.0) & (bh >= 4.0)
    if not bool(valid.any().item()):
        return []
    boxes = boxes[valid]
    scores = scores[valid]
    labels = [lbl for lbl, ok in zip(raw_canon_labels, valid.tolist(), strict=True) if ok]

    by_cls: dict[str, list[int]] = defaultdict(list)
    for idx, lbl in enumerate(labels):
        by_cls[lbl].append(idx)

    stage1_indices: list[int] = []
    for idxs in by_cls.values():
        idx_t = torch.tensor(idxs, dtype=torch.long)
        keep_local = nms(boxes[idx_t], scores[idx_t], per_class_iou)
        stage1_indices.extend(idx_t[keep_local].tolist())

    stage1_indices.sort(key=lambda i: -float(scores[i].item()))
    keep_t = nms(boxes[stage1_indices], scores[stage1_indices], cross_class_iou)
    final_indices = [stage1_indices[k] for k in keep_t.tolist()]
    return [
        {
            "box": [round(float(v), 1) for v in boxes[i].tolist()],
            "label": labels[i],
            "score": round(float(scores[i].item()), 3),
        }
        for i in final_indices
    ]


def _all_teacher_prompt_pairs() -> list[tuple[str, str]]:
    pairs: list[tuple[str, str]] = []
    for _grp_name, grp_pairs in TEACHER_PROMPT_GROUPS:
        pairs.extend(grp_pairs)
    return pairs


def _match_phrase_to_canonical(raw_label: str, phrase_to_canon: dict[str, str]) -> str:
    norm = raw_label.strip().lower()
    if norm in phrase_to_canon:
        return phrase_to_canon[norm]
    for phrase in sorted(phrase_to_canon.keys(), key=len, reverse=True):
        if phrase in norm:
            return phrase_to_canon[phrase]
    return map_to_canonical(norm) or norm.replace(" ", "_")


def run_owlv2_teacher(
    items: list[dict[str, Any]],
    device: torch.device,
) -> tuple[dict[int, list[dict[str, Any]]], float]:
    """Run google/owlv2-large-patch14-ensemble on selected images and release VRAM."""
    print(f"[Teacher 1] Loading {OWLV2_MODEL_ID} ({len(items)} images)...", flush=True)
    processor = Owlv2Processor.from_pretrained(OWLV2_MODEL_ID)
    dtype = torch.float16 if device.type == "cuda" else torch.float32
    model = Owlv2ForObjectDetection.from_pretrained(OWLV2_MODEL_ID, torch_dtype=dtype).to(device)
    model.eval()

    pairs = _all_teacher_prompt_pairs()
    phrases = [p for p, _ in pairs]
    phrase_to_canon = dict(pairs)
    preds_by_id: dict[int, list[dict[str, Any]]] = {}
    latencies_ms: list[float] = []

    with torch.no_grad():
        for idx, item in enumerate(items):
            with Image.open(item["image_path"]) as im:
                rgb = im.convert("RGB")
                inputs = processor(text=[phrases], images=rgb, return_tensors="pt")
            model_inputs = {
                k: (v.to(device=device, dtype=dtype) if v.is_floating_point() else v.to(device=device))
                for k, v in inputs.items()
            }
            if device.type == "cuda":
                torch.cuda.synchronize()
            t0 = time.perf_counter()
            outputs = model(**model_inputs)
            target_sizes = torch.tensor([(rgb.height, rgb.width)], device=device)
            res = processor.post_process_grounded_object_detection(
                outputs=outputs,
                target_sizes=target_sizes,
                threshold=0.14,
                text_labels=[phrases],
            )[0]
            if device.type == "cuda":
                torch.cuda.synchronize()
            dt = (time.perf_counter() - t0) * 1000.0
            if idx > 0:
                latencies_ms.append(dt)

            raw_lbls = res.get("text_labels")
            if raw_lbls is not None:
                canon_lbls = [_match_phrase_to_canonical(str(s), phrase_to_canon) for s in raw_lbls]
            else:
                canon_lbls = [phrase_to_canon[phrases[int(li.item())]] for li in res["labels"]]
            preds_by_id[int(item["image_id"])] = _postprocess_teacher_boxes(
                res["boxes"],
                res["scores"],
                canon_lbls,
                rgb.width,
                rgb.height,
            )
            if (idx + 1) % 24 == 0 or (idx + 1) == len(items):
                print(f"  OWLv2 progress: {idx + 1}/{len(items)}", flush=True)

    del model, processor
    gc.collect()
    if device.type == "cuda":
        torch.cuda.empty_cache()
    return preds_by_id, round(sum(latencies_ms) / max(len(latencies_ms), 1), 2)


def run_grounding_dino_teacher(
    items: list[dict[str, Any]],
    device: torch.device,
) -> tuple[dict[int, list[dict[str, Any]]], float, str]:
    """Run IDEA-Research/grounding-dino-base (3-group prompt batching) and release VRAM."""
    print(f"[Teacher 2] Loading {GDINO_MODEL_ID} ({len(items)} images)...", flush=True)
    processor = AutoProcessor.from_pretrained(GDINO_MODEL_ID)
    dtype = torch.float16 if device.type == "cuda" else torch.float32
    model = AutoModelForZeroShotObjectDetection.from_pretrained(
        GDINO_MODEL_ID, torch_dtype=dtype
    ).to(device)
    model.eval()
    used_dtype = str(dtype)

    preds_by_id: dict[int, list[dict[str, Any]]] = {}
    latencies_ms: list[float] = []

    with torch.no_grad():
        for idx, item in enumerate(items):
            with Image.open(item["image_path"]) as im:
                rgb = im.convert("RGB")
                all_boxes: list[Tensor] = []
                all_scores: list[Tensor] = []
                all_labels: list[str] = []

                if device.type == "cuda":
                    torch.cuda.synchronize()
                t0 = time.perf_counter()

                for _grp_name, grp_pairs in TEACHER_PROMPT_GROUPS:
                    grp_phrases = [p for p, _ in grp_pairs]
                    phrase_to_canon = dict(grp_pairs)
                    inputs = processor(images=rgb, text=[grp_phrases], return_tensors="pt")
                    try:
                        model_inputs = {
                            k: (
                                v.to(device=device, dtype=dtype)
                                if v.is_floating_point()
                                else v.to(device=device)
                            )
                            for k, v in inputs.items()
                        }
                        outputs = model(**model_inputs)
                    except (RuntimeError, TypeError) as err:
                        if dtype != torch.float32:
                            print(f"[Teacher 2] fp16 fallback to fp32 ({err})", flush=True)
                            dtype = torch.float32
                            used_dtype = "torch.float32 (fallback)"
                            model = model.to(dtype=torch.float32)
                            model_inputs = {
                                k: (
                                    v.to(device=device, dtype=torch.float32)
                                    if v.is_floating_point()
                                    else v.to(device=device)
                                )
                                for k, v in inputs.items()
                            }
                            outputs = model(**model_inputs)
                        else:
                            raise

                    res = processor.post_process_grounded_object_detection(
                        outputs=outputs,
                        input_ids=inputs.input_ids.to(device),
                        threshold=0.25,
                        text_threshold=0.20,
                        target_sizes=[(rgb.height, rgb.width)],
                    )[0]
                    if res["boxes"].numel() > 0:
                        all_boxes.append(res["boxes"].float().cpu())
                        all_scores.append(res["scores"].float().cpu())
                        raw_txt = res.get("text_labels") or res.get("labels") or []
                        for lbl in raw_txt:
                            all_labels.append(_match_phrase_to_canonical(str(lbl), phrase_to_canon))

                if device.type == "cuda":
                    torch.cuda.synchronize()
                dt = (time.perf_counter() - t0) * 1000.0
                if idx > 0:
                    latencies_ms.append(dt)

                preds_by_id[int(item["image_id"])] = (
                    _postprocess_teacher_boxes(
                        torch.cat(all_boxes, dim=0),
                        torch.cat(all_scores, dim=0),
                        all_labels,
                        rgb.width,
                        rgb.height,
                    )
                    if all_boxes
                    else []
                )

    del model, processor
    gc.collect()
    if device.type == "cuda":
        torch.cuda.empty_cache()
    return preds_by_id, round(sum(latencies_ms) / max(len(latencies_ms), 1), 2), used_dtype


def _box_iou_xyxy(a: list[float], b: list[float]) -> float:
    ix1 = max(a[0], b[0])
    iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2])
    iy2 = min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0.0:
        return 0.0
    aa = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    ab = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / max(aa + ab - inter, 1e-6)


def compute_merged_training_targets(
    gt_kept: list[dict[str, Any]],
    owl_preds: list[dict[str, Any]],
    source_dataset: str,
    img_w: int,
    img_h: int,
    *,
    owl_thresh: float = 0.26,
    max_stuff_area: float = 0.55,
    hide_stuff: bool = False,
    same_cls_gt_iou: float = 0.35,
    any_gt_iou: float = 0.55,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Merge authoritative Clean Human GT with non-overlapping OWLv2 missing-class boxes."""
    annotated = SOURCE_ANNOTATED_CATEGORIES.get(source_dataset, frozenset())
    img_area = float(max(img_w * img_h, 1))
    owl_added: list[dict[str, Any]] = []

    for pred in owl_preds:
        if float(pred["score"]) < owl_thresh:
            continue
        lbl = str(pred["label"])
        if hide_stuff and lbl in STUFF_CATEGORIES:
            continue
        box = pred["box"]
        area_frac = (max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])) / img_area
        if lbl in STUFF_CATEGORIES and area_frac > max_stuff_area:
            continue
        if lbl in annotated:
            continue

        same_iou = max(
            (_box_iou_xyxy(box, g["box"]) for g in gt_kept if g["label"] == lbl),
            default=0.0,
        )
        any_iou = max((_box_iou_xyxy(box, g["box"]) for g in gt_kept), default=0.0)
        if same_iou >= same_cls_gt_iou or any_iou >= any_gt_iou:
            continue
        owl_added.append(pred)

    return gt_kept, owl_added


def _load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for p in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ):
        if Path(p).exists():
            return ImageFont.truetype(p, size=size)
    return ImageFont.load_default()


def _draw_dashed_rect(
    draw: ImageDraw.ImageDraw,
    box: list[float],
    color: tuple[int, int, int],
    width: int = 2,
    dash: int = 8,
) -> None:
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    for sx, sy, ex, ey in ((x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)):
        length = max(abs(ex - sx), abs(ey - sy))
        if length <= 0:
            continue
        dx = (ex - sx) / length
        dy = (ey - sy) / length
        pos = 0
        while pos < length:
            end = min(pos + dash, length)
            draw.line(
                [(sx + dx * pos, sy + dy * pos), (sx + dx * end, sy + dy * end)],
                fill=color,
                width=width,
            )
            pos += dash * 2


def _render_quadrant(
    base_rgb: Image.Image,
    title: str,
    title_bg: tuple[int, int, int],
    boxes: list[tuple[list[float], str, tuple[int, int, int], bool]],
    quad_w: int,
    header_h: int,
    font_hdr: ImageFont.ImageFont,
    font_tag: ImageFont.ImageFont,
) -> Image.Image:
    scale = quad_w / max(base_rgb.width, 1)
    quad_h = max(1, int(round(base_rgb.height * scale)))
    resized = base_rgb.resize((quad_w, quad_h), Image.Resampling.BILINEAR)
    panel = Image.new("RGB", (quad_w, quad_h + header_h), (18, 22, 32))
    panel.paste(resized, (0, header_h))
    draw = ImageDraw.Draw(panel)
    draw.rectangle([0, 0, quad_w, header_h], fill=title_bg)
    draw.text((10, 8), title, fill=(248, 250, 252), font=font_hdr)

    for raw_b, text, color, dashed in boxes:
        bx1 = raw_b[0] * scale
        by1 = raw_b[1] * scale + header_h
        bx2 = raw_b[2] * scale
        by2 = raw_b[3] * scale + header_h
        if dashed:
            _draw_dashed_rect(draw, [bx1, by1, bx2, by2], color=color, width=2)
        else:
            draw.rectangle([bx1, by1, bx2, by2], outline=color, width=2)
        if text:
            tb = draw.textbbox((0, 0), text, font=font_tag)
            tw, th = tb[2] - tb[0], tb[3] - tb[1]
            ty = max(header_h + 1, by1 - th - 5)
            tx = max(1, min(bx1, quad_w - tw - 6))
            draw.rectangle([tx, ty, tx + tw + 6, ty + th + 4], fill=(12, 17, 29))
            draw.text((tx + 3, ty + 2), text, fill=color, font=font_tag)
    return panel


def render_core_panels(
    items: list[dict[str, Any]],
    student_preds: dict[int, list[dict[str, Any]]],
    owl_preds: dict[int, list[dict[str, Any]]],
    gdino_preds: dict[int, list[dict[str, Any]]] | None,
    out_dir: Path,
) -> list[str]:
    """Render 2x2 PNG panels for the pinned core diagnostic images."""
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

        s_list = [p for p in student_preds.get(img_id, []) if p["score"] >= 0.20][:25]
        o_list = [p for p in owl_preds.get(img_id, []) if p["score"] >= 0.26][:40]

        tl = _render_quadrant(
            rgb,
            f"1. GT Hygiene #{img_id} ({item['source_dataset']}) | Kept={st['kept_positives']}/{st['raw_positives']} DupDrop={st['dropped_dup_hbb']+st['dropped_occluded_quad_fragments']} WSDemote={st['woodscape_demoted_noisy_class']+st['woodscape_demoted_tiny_fragment']+st['woodscape_demoted_nested_fragment']}",
            (22, 78, 52),
            gt_draw,
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )
        tr = _render_quadrant(
            rgb,
            f"2. Current SOTA Student Opt-B HBB (384x384, joint>=0.20, n={len(s_list)})",
            (30, 58, 138),
            [(p["box"], f"{p['label']} {int(p['score']*100)}%", (96, 165, 250), False) for p in s_list],
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )
        bl = _render_quadrant(
            rgb,
            f"3. OWLv2-Large Tuned Prompts (score>=0.26, n={len(o_list)})",
            (107, 33, 168),
            [(p["box"], f"{p['label']} {int(p['score']*100)}%", (232, 121, 249), False) for p in o_list],
            quad_w,
            header_h,
            font_hdr,
            font_tag,
        )

        if gdino_preds is not None:
            g_list = [p for p in gdino_preds.get(img_id, []) if p["score"] >= 0.28][:40]
            br = _render_quadrant(
                rgb,
                f"4. Grounding-DINO-Base Teacher (score>=0.28, n={len(g_list)})",
                (146, 64, 14),
                [(p["box"], f"{p['label']} {int(p['score']*100)}%", (251, 191, 36), False) for p in g_list],
                quad_w,
                header_h,
                font_hdr,
                font_tag,
            )
        else:
            gt_m, owl_added = compute_merged_training_targets(
                item["gt_kept"],
                owl_preds.get(img_id, []),
                item["source_dataset"],
                item["width"],
                item["height"],
                owl_thresh=0.26,
                max_stuff_area=0.55,
                hide_stuff=False,
            )
            merged_draw = [(g["box"], f"GT:{g['label']}", (74, 222, 128), False) for g in gt_m] + [
                (p["box"], f"+OWL:{p['label']} {int(p['score']*100)}%", (34, 211, 238), False)
                for p in owl_added[:35]
            ]
            br = _render_quadrant(
                rgb,
                f"4. Merged Target Preview (GT={len(gt_m)} + OWL Missing={len(owl_added)})",
                (14, 116, 144),
                merged_draw,
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


def compute_owlv2_addition_stats(
    items: list[dict[str, Any]],
    owl_preds: dict[int, list[dict[str, Any]]],
    *,
    owl_thresh: float = 0.26,
) -> dict[str, dict[str, Any]]:
    """Summarize per-dataset missing-class pseudo-label addition rates across PoC images."""
    acc: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for it in items:
        src = str(it["source_dataset"])
        img_id = int(it["image_id"])
        raw_owl = [p for p in owl_preds.get(img_id, []) if float(p["score"]) >= owl_thresh]
        gt_m, added = compute_merged_training_targets(
            it["gt_kept"],
            owl_preds.get(img_id, []),
            src,
            it["width"],
            it["height"],
            owl_thresh=owl_thresh,
        )
        acc[src]["images"] += 1
        acc[src]["clean_gt"] += len(gt_m)
        acc[src]["owl_raw"] += len(raw_owl)
        acc[src]["owl_added"] += len(added)
        acc[src]["construction_barrier"] += sum(1 for p in raw_owl if p["label"] == "construction_barrier")

    out: dict[str, dict[str, Any]] = {}
    for src in DATASET_ORDER:
        if src not in acc:
            continue
        d = acc[src]
        n = max(d["images"], 1)
        out[src] = {
            "images": d["images"],
            "avg_clean_gt": round(d["clean_gt"] / n, 2),
            "avg_owl_raw_026": round(d["owl_raw"] / n, 2),
            "avg_missing_added_026": round(d["owl_added"] / n, 2),
            "construction_barrier_026": d["construction_barrier"],
        }
    return out


def build_poc_html(
    items: list[dict[str, Any]],
    student_preds: dict[int, list[dict[str, Any]]],
    owl_preds: dict[int, list[dict[str, Any]]],
    gdino_preds: dict[int, list[dict[str, Any]]] | None,
    hygiene_summary: dict[str, Any],
    recall_delta: dict[str, Any],
    owl_addition_stats: dict[str, Any],
    latencies: dict[str, Any],
    out_path: Path,
) -> None:
    """Write paginated interactive HTML dashboard for all selected PoC images."""
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

        st_p = student_preds.get(img_id, [])
        ow_p = owl_preds.get(img_id, [])
        gd_p = gdino_preds.get(img_id, []) if gdino_preds is not None else None
        for g in item["gt_kept"]:
            all_classes.add(g["label"])
        for p in st_p + ow_p + (gd_p or []):
            all_classes.add(p["label"])

        entry: dict[str, Any] = {
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
            "student": st_p,
            "owlv2": ow_p,
            "thumb": thumb,
        }
        if gd_p is not None:
            entry["gdino"] = gd_p
        payload.append(entry)

    rows_html = ""
    for src, s in hygiene_summary.items():
        rows_html += (
            f"<tr><td>{escape(src)}</td><td>{s['images']}</td><td>{s['raw_positives']}</td>"
            f"<td>{s['kept_positives']}</td><td>-{s['reduction_pct']}%</td>"
            f"<td>{s['dropped_dup_hbb']}</td><td>{s['dropped_occluded_quad_fragments']}</td>"
            f"<td>{s['woodscape_demoted_noisy_class']+s['woodscape_demoted_tiny_fragment']+s['woodscape_demoted_nested_fragment']}</td></tr>"
        )

    recall_rows = ""
    for src in ("nuimages", "woodscape_rgb_fisheye"):
        b = recall_delta["before_raw"][src]
        a = recall_delta["after_clean"][src]
        recall_rows += (
            f"<tr><td>{escape(src)}</td><td>{b['num_gt']} &rarr; {a['num_gt']}</td>"
            f"<td>{b['recall_100_50']*100:.1f}% &rarr; {a['recall_100_50']*100:.1f}%</td>"
            f"<td>{b['dup_gt_rate']*100:.1f}% &rarr; {a['dup_gt_rate']*100:.1f}%</td>"
            f"<td>{b['class_agnostic_ap50']*100:.1f}% &rarr; {a['class_agnostic_ap50']*100:.1f}%</td></tr>"
        )

    owl_rows = ""
    for src, s in owl_addition_stats.items():
        owl_rows += (
            f"<tr><td>{escape(src)}</td><td>{s['images']}</td><td>{s['avg_clean_gt']:.2f}</td>"
            f"<td>{s['avg_owl_raw_026']:.2f}</td><td>+{s['avg_missing_added_026']:.2f}</td>"
            f"<td>{s['construction_barrier_026']}</td></tr>"
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
    src_ann_json = json.dumps({k: sorted(v) for k, v in SOURCE_ANNOTATED_CATEGORIES.items()})
    stuff_json = json.dumps(sorted(STUFF_CATEGORIES))
    has_gdino = gdino_preds is not None
    gdino_latency_str = (
        f" | Grounding-DINO-Base={latencies['gdino_ms']}ms" if has_gdino and latencies.get("gdino_ms") else ""
    )
    gdino_slider_html = (
        '<label>G-DINO &ge; <span id="gv">0.28</span> <input type="range" id="gt" min="0.25" max="0.80" step="0.02" value="0.28" oninput="gv.textContent=Number(this.value).toFixed(2);render(false)"></label>'
        if has_gdino
        else '<label>Max Stuff Area &le; <span id="mav">55%</span> <input type="range" id="ma" min="0.15" max="1.00" step="0.05" value="0.55" oninput="mav.textContent=Math.round(Number(this.value)*100)+\'%\';render(false)"></label>'
        '<label><input type="checkbox" id="hide-stuff" onchange="render(false)"> Hide Stuff (building/tree/fence)</label>'
    )

    html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Option B PoC ({len(payload)} Images) — Hygiene + OWLv2 Teacher</title>
<style>
body{{font-family:system-ui,sans-serif;background:#0b1020;color:#e2e8f0;margin:0;padding:18px}}
h1{{margin:0 0 8px;font-size:1.3rem}}
.tables{{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:12px}}
table{{border-collapse:collapse;font-size:.80rem;background:#131a2e}}
th,td{{border:1px solid #26314a;padding:4px 9px;text-align:right}} th:first-child,td:first-child{{text-align:left}}
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
<h1>Option B Proof-of-Concept ({len(payload)} Stratified Images): DB Hygiene + Tuned OWLv2-Large</h1>
<div style="font-size:.84rem;color:#94a3b8;margin-bottom:10px">Warm Latency (RTX 3060): Student={latencies['student_ms']}ms | OWLv2-Large={latencies['owlv2_ms']}ms{gdino_latency_str} | Holdout Unseen + Core 8</div>
<div class="tables">
  <table><tr><th>Dataset (8k cohort)</th><th>Imgs</th><th>Raw GT</th><th>Clean GT</th><th>Delta</th><th>Dup HBB</th><th>Occ Quad</th><th>WS Demote</th></tr>{rows_html}</table>
  <table><tr><th>Val Recall (Zero Retrain)</th><th>GT Count</th><th>R@100@50</th><th>Dup GT</th><th>Agn AP@50</th></tr>{recall_rows}</table>
  <table><tr><th>PoC OWLv2 (@0.26)</th><th>Imgs</th><th>Clean GT/img</th><th>OWLv2/img</th><th>+Missing/img</th><th>Barrier FPs</th></tr>{owl_rows}</table>
</div>
<div class="controls">
  <div class="row">{ds_chips}</div>
  <div class="row">
    <label>Class: <select id="cls-filter" onchange="render(true)">{cls_options}</select></label>
    <label>Image ID / Search: <input type="text" id="q-search" placeholder="e.g. 189270" oninput="render(true)" style="width:130px"></label>
    <label><input type="checkbox" id="show-dropped" checked onchange="render(false)"> Show Dropped/Demoted GT</label>
    <label>Student &ge; <span id="sv">0.20</span> <input type="range" id="st" min="0.05" max="0.80" step="0.02" value="0.20" oninput="sv.textContent=Number(this.value).toFixed(2);render(false)"></label>
    <label>OWLv2 &ge; <span id="ov">0.26</span> <input type="range" id="ot" min="0.14" max="0.65" step="0.02" value="0.26" oninput="ov.textContent=Number(this.value).toFixed(2);render(false)"></label>
    {gdino_slider_html}
    <label><input type="checkbox" id="show-all" onchange="render(true)"> Show All</label>
  </div>
</div>
<div class="pager"><span id="count-info"></span><div><button class="btn" id="prev-btn" onclick="stepPage(-1)">Prev</button> <span id="page-info" style="margin:0 8px;font-size:.86rem"></span> <button class="btn" id="next-btn" onclick="stepPage(1)">Next</button></div></div>
<div id="gal"></div>
<div class="pager"><span></span><div><button class="btn" id="prev-btn2" onclick="stepPage(-1)">Prev</button> <span id="page-info2" style="margin:0 8px;font-size:.86rem"></span> <button class="btn" id="next-btn2" onclick="stepPage(1)">Next</button></div></div>
<script>
const ITEMS = {json.dumps(payload)};
const SOURCE_ANN = {src_ann_json};
const STUFF_SET = new Set({stuff_json});
const HAS_GDINO = {json.dumps(has_gdino)};
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
function boxIou(a, b) {{
  const ix1 = Math.max(a[0], b[0]), iy1 = Math.max(a[1], b[1]), ix2 = Math.min(a[2], b[2]), iy2 = Math.min(a[3], b[3]);
  const inter = Math.max(0, ix2 - ix1) * Math.max(0, iy2 - iy1);
  if (inter <= 0) return 0;
  const aa = Math.max(0, a[2]-a[0])*Math.max(0, a[3]-a[1]), ab = Math.max(0, b[2]-b[0])*Math.max(0, b[3]-b[1]);
  return inter / Math.max(aa + ab - inter, 1e-6);
}}
function computeMerged(it, ot, maxStuff, hideStuff) {{
  const ann = new Set(SOURCE_ANN[it.source_dataset] || []);
  const areaImg = Math.max(it.width * it.height, 1);
  const added = [];
  for (const p of it.owlv2) {{
    if (p.score < ot) continue;
    if (hideStuff && STUFF_SET.has(p.label)) continue;
    const af = (Math.max(0, p.box[2]-p.box[0]) * Math.max(0, p.box[3]-p.box[1])) / areaImg;
    if (STUFF_SET.has(p.label) && af > maxStuff) continue;
    if (ann.has(p.label)) continue;
    let maxSame = 0, maxAny = 0;
    for (const g of it.gt_kept) {{
      const u = boxIou(p.box, g.box);
      if (u > maxAny) maxAny = u;
      if (g.label === p.label && u > maxSame) maxSame = u;
    }}
    if (maxSame >= 0.35 || maxAny >= 0.55) continue;
    added.push(p);
  }}
  return added;
}}
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
  const maxStuff = HAS_GDINO ? 1.0 : parseFloat(document.getElementById('ma').value);
  const hideStuff = HAS_GDINO ? false : document.getElementById('hide-stuff').checked;
  const gThresh = HAS_GDINO ? parseFloat(document.getElementById('gt').value) : 0.28;

  const filtered = ITEMS.filter(it => {{
    if (activeDs === 'core' && !it.is_core) return false;
    if (activeDs !== 'all' && activeDs !== 'core' && it.source_dataset !== activeDs) return false;
    if (q && !String(it.image_id).includes(q) && !it.source_dataset.toLowerCase().includes(q)) return false;
    if (clsF !== 'all') {{
      const hasGt = it.gt_kept.some(g => g.label === clsF);
      const hasOwl = it.owlv2.some(p => p.score >= ot && p.label === clsF);
      const hasStu = it.student.some(p => p.score >= st && p.label === clsF);
      if (!hasGt && !hasOwl && !hasStu) return false;
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
      const sBoxes = it.student.filter(p=>p.score>=st).slice(0,25).map(p=>({{box:p.box,color:'#60a5fa',dash:false,txt:`${{p.label}} ${{Math.round(p.score*100)}}%`}}));
      const oBoxes = it.owlv2.filter(p=>p.score>=ot && (!hideStuff || !STUFF_SET.has(p.label))).slice(0,40).map(p=>({{box:p.box,color:'#e879f9',dash:false,txt:`${{p.label}} ${{Math.round(p.score*100)}}%`}}));
      drawPane(panes[0], img, `1. GT Hygiene (${{it.hygiene_stats.kept_positives}}/${{it.hygiene_stats.raw_positives}})`, 'rgba(22,78,52,0.9)', gtBoxes);
      drawPane(panes[1], img, `2. Student Opt-B HBB 384x384 (n=${{sBoxes.length}})`, 'rgba(30,58,138,0.9)', sBoxes);
      drawPane(panes[2], img, `3. OWLv2-Large Tuned (n=${{oBoxes.length}})`, 'rgba(107,33,168,0.9)', oBoxes);
      if (HAS_GDINO) {{
        const gBoxes = (it.gdino || []).filter(p=>p.score>=gThresh).slice(0,40).map(p=>({{box:p.box,color:'#fbbf24',dash:false,txt:`${{p.label}} ${{Math.round(p.score*100)}}%`}}));
        drawPane(panes[3], img, `4. Grounding-DINO-Base (n=${{gBoxes.length}})`, 'rgba(146,64,14,0.9)', gBoxes);
      }} else {{
        const added = computeMerged(it, ot, maxStuff, hideStuff);
        const mBoxes = it.gt_kept.map(x=>({{box:x.box,color:'#4ade80',dash:false,txt:`GT:${{x.label}}`}})).concat(
          added.slice(0,35).map(p=>({{box:p.box,color:'#22d3ee',dash:false,txt:`+OWL:${{p.label}} ${{Math.round(p.score*100)}}%`}}))
        );
        drawPane(panes[3], img, `4. Merged Target Preview (GT=${{it.gt_kept.length}} + OWL Missing=${{added.length}})`, 'rgba(14,116,144,0.9)', mBoxes);
      }}
    }};
    gal.appendChild(card);
  }}
}}
window.addEventListener('DOMContentLoaded', () => render(true));
</script></body></html>"""
    out_path.write_text(html)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument(
        "--holdout-manifest",
        type=Path,
        default=Path("artifacts/dashboard_holdout_8k_8k_full.json"),
    )
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument("--num-images", type=int, default=72)
    parser.add_argument("--include-gdino", action="store_true")
    parser.add_argument(
        "--hbb-ckpt",
        type=Path,
        default=Path("artifacts/sota_8k_hbb/checkpoints/sota_joint_best.pt"),
    )
    parser.add_argument(
        "--vocab-37",
        type=Path,
        default=Path("artifacts/sota_open_vocab/vocabulary_bank_37classes.pt"),
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
        "--output-dir",
        type=Path,
        default=Path("artifacts/poc_teacher_hygiene"),
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    panels_dir = args.output_dir / "panels"

    print("=== Part A1: 8k Cohort Annotation Hygiene Audit ===", flush=True)
    hygiene_summary = audit_cohort_hygiene(args.gt_index)
    for src, s in hygiene_summary.items():
        print(
            f"  {src:22s}: imgs={s['images']:4d} | raw={s['raw_positives']:5d} -> kept={s['kept_positives']:5d} (-{s['reduction_pct']:5.2f}%) "
            f"| dup_hbb={s['dropped_dup_hbb']:4d} occ_frag={s['dropped_occluded_quad_fragments']:4d} "
            f"ws_cls={s['woodscape_demoted_noisy_class']:4d} ws_tiny={s['woodscape_demoted_tiny_fragment']:5d} ws_nest={s['woodscape_demoted_nested_fragment']:4d}",
            flush=True,
        )

    print("\n=== Part A2: Val Recall Hygiene Delta (nuimages + woodscape) ===", flush=True)
    recall_delta = evaluate_val_recall_hygiene_delta(
        ckpt_path=args.hbb_ckpt,
        gt_index=args.gt_index,
        image_root=args.image_root,
        device=device,
    )
    for src in ("nuimages", "woodscape_rgb_fisheye"):
        b, a = recall_delta["before_raw"][src], recall_delta["after_clean"][src]
        print(
            f"  {src:22s}: GT {b['num_gt']}->{a['num_gt']} | R@100@50 {b['recall_100_50']*100:.2f}%->{a['recall_100_50']*100:.2f}% "
            f"| dup_gt {b['dup_gt_rate']*100:.2f}%->{a['dup_gt_rate']*100:.2f}% | agn_AP50 {b['class_agnostic_ap50']*100:.2f}%->{a['class_agnostic_ap50']*100:.2f}%",
            flush=True,
        )

    selected_ids, core_set = select_poc_image_ids(
        args.gt_index,
        args.holdout_manifest,
        args.image_root,
        num_images=args.num_images,
    )
    print(f"\n=== Part B: Stratified PoC Inference on {len(selected_ids)} Images (core={len(core_set)}) ===", flush=True)
    items = load_diagnostic_images(args.gt_index, args.image_root, selected_ids, core_set)

    student_preds, student_ms = run_student_option_b_inference(
        items,
        ckpt_path=args.hbb_ckpt,
        vocab_37_path=args.vocab_37,
        vocab_dist_path=args.vocab_distractors,
        centroid_path=args.visual_centroid,
        device=device,
    )
    owl_preds, owl_ms = run_owlv2_teacher(items, device=device)

    gdino_preds: dict[int, list[dict[str, Any]]] | None = None
    gdino_ms: float | None = None
    gdino_dtype = "skipped"
    if args.include_gdino:
        gdino_preds, gdino_ms, gdino_dtype = run_grounding_dino_teacher(items, device=device)

    owl_addition_stats = compute_owlv2_addition_stats(items, owl_preds, owl_thresh=0.26)
    print("\n=== OWLv2 Missing-Class Addition Rate (@0.26) ===", flush=True)
    for src, s in owl_addition_stats.items():
        print(
            f"  {src:22s}: imgs={s['images']:2d} | clean_gt/img={s['avg_clean_gt']:5.2f} "
            f"| owl_raw/img={s['avg_owl_raw_026']:5.2f} | +missing/img={s['avg_missing_added_026']:5.2f} "
            f"| barrier_026={s['construction_barrier_026']}",
            flush=True,
        )

    print(
        f"\nWarm GPU Latency (RTX 3060): Student={student_ms:.1f}ms | OWLv2-Large={owl_ms:.1f}ms"
        + (f" | Grounding-DINO-Base={gdino_ms:.1f}ms [{gdino_dtype}]" if gdino_ms is not None else ""),
        flush=True,
    )

    print("\n=== Part C: Rendering Core Panels & Interactive 72-Image HTML ===", flush=True)
    panel_paths = render_core_panels(items, student_preds, owl_preds, gdino_preds, panels_dir)
    latencies = {
        "student_ms": student_ms,
        "owlv2_ms": owl_ms,
        "gdino_ms": gdino_ms,
        "gdino_dtype": gdino_dtype,
    }
    build_poc_html(
        items,
        student_preds,
        owl_preds,
        gdino_preds,
        hygiene_summary,
        recall_delta,
        owl_addition_stats,
        latencies,
        args.output_dir / "dashboard_teacher_poc.html",
    )

    summary_payload = {
        "num_images": len(items),
        "cohort_8k_hygiene": hygiene_summary,
        "val_recall_delta": recall_delta,
        "owlv2_addition_stats_026": owl_addition_stats,
        "latencies_ms": latencies,
        "panel_paths": panel_paths,
        "images": [
            {
                "image_id": it["image_id"],
                "is_core": it["is_core"],
                "source_dataset": it["source_dataset"],
                "width": it["width"],
                "height": it["height"],
                "hygiene_stats": it["hygiene_stats"],
                "gt_kept_count": len(it["gt_kept"]),
                "student_count_020": sum(1 for p in student_preds[it["image_id"]] if p["score"] >= 0.20),
                "owlv2_count_026": sum(1 for p in owl_preds[it["image_id"]] if p["score"] >= 0.26),
                "merged_added_026": len(
                    compute_merged_training_targets(
                        it["gt_kept"],
                        owl_preds[it["image_id"]],
                        it["source_dataset"],
                        it["width"],
                        it["height"],
                        owl_thresh=0.26,
                    )[1]
                ),
                "student_top": student_preds[it["image_id"]][:15],
                "owlv2_top": owl_preds[it["image_id"]][:20],
            }
            for it in items
        ],
    }
    (args.output_dir / "poc_summary.json").write_text(json.dumps(summary_payload, indent=2))
    print(f"Done! Artifacts saved under {args.output_dir}", flush=True)


if __name__ == "__main__":
    main()
