#!/usr/bin/env python3
"""Rigorous head-to-head benchmark: Option A vs Option B vs Option A+B vs Legacy.

Evaluates both HBB and Quad open-vocabulary student pipelines across:
  1. cohort_val (800 unseen images from the 8k cohort, 0% overlap with train)
  2. holdout_210 (210 isolated holdout images from dashboard_holdout_8k_8k_full.json)

Measures:
  - Overlap & Localization: boxes/img @0.30, overlap pairs/img (IoU > 0.25),
    multi-hit duplicate GT %, pure background clutter %, Recall@10/100@50, Recall@100@75, AP@50.
  - Seen Open-Vocabulary Classification (37 OPEN_VOCAB_CATEGORIES):
    Matched-proposal Top-1/Top-3, Oracle-GT-box Top-1/Top-3 (RoI heads),
    End-to-End Open-Vocabulary mAP@50, Background Clutter FP Rejection Rate,
    Per-class accuracy breakdown, and Signboard -> Window/Door confusion rate.
  - True Zero-Shot Unseen-Class Generalization (41-class bank appending book, painting,
    cushion, flowerpot with zero retraining): 4-way unseen Top-1 & 41-way Top-1/Top-5.
  - Edge Compute & Latency Profile: parameter counts, 4-thread CPU latency (Pi 5 proxy),
    and CUDA latency for Detector-only, Option A (indexed & dense), Option B, A+B, and Legacy.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import copy
import json
from pathlib import Path
import time
from typing import Any

import torch
from torch import Tensor
from torch.utils.data import DataLoader
from torchvision.ops import box_iou

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.decoder import InferenceDecoder
from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
    HumanGTQuadDataset,
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
from student_detector.quad_geometry import pairwise_quad_iou
from student_detector.region_head import RegionEmbeddingHead
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    collate_proposal_samples,
    collate_quad_proposal_samples,
)
from student_detector.targets import point_validity_from_pixel_mask
from student_detector.vocabulary import VocabularyBank

KEY_BREAKDOWN_CLASSES: tuple[str, ...] = (
    "car",
    "pedestrian",
    "cyclist",
    "traffic_sign",
    "signboard",
    "traffic_light",
    "traffic_cone",
    "trash_bin",
    "pillar",
    "pole",
    "street_light",
    "window",
    "door",
    "building",
    "tree",
    "chair",
    "table",
    "cabinet",
)


def compute_average_precision(
    records: list[tuple[float, int]],
    total_gt: int,
) -> float:
    """Compute standard 101-point interpolated Average Precision."""
    if total_gt <= 0 or not records:
        return 0.0
    sorted_recs = sorted(records, key=lambda x: x[0], reverse=True)
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
    for r_thresh in [i / 100.0 for i in range(101)]:
        p_at_r = 0.0
        for p_val, r_val in zip(precisions, recalls, strict=True):
            if r_val >= r_thresh and p_val > p_at_r:
                p_at_r = p_val
        ap += p_at_r
    return ap / 101.0


def build_split_datasets(
    geometry: str,
    gt_index: Path,
    image_root: Path,
    holdout_manifest: Path,
    input_size: int = 384,
    cohort_size: int = 8000,
) -> dict[str, Any]:
    """Build cohort_val (800) and holdout_210 datasets with unified transforms."""
    ds_cls = HumanGTHBBDataset if geometry == "hbb" else HumanGTQuadDataset
    val_ds = ds_cls(
        cohort_db_path=gt_index,
        gt_index_path=gt_index,
        image_root=image_root,
        split="val",
        input_size=input_size,
        cohort_size=cohort_size,
    )
    splits: dict[str, Any] = {"cohort_val_800": val_ds}

    if holdout_manifest.exists():
        raw_manifest = json.loads(holdout_manifest.read_text())
        ho_imgs = (
            raw_manifest["images"]
            if isinstance(raw_manifest, dict) and "images" in raw_manifest
            else raw_manifest
        )
        holdout_ds = copy.copy(val_ds)
        holdout_ds._connection = None
        holdout_ds.images = [
            {
                "image_id": int(r["image_id"]),
                "file_name": str(r["file_name"]),
                "source_dataset": str(r["source_dataset"]),
                "width": int(r["width"]),
                "height": int(r["height"]),
            }
            for r in ho_imgs
        ]
        splits["holdout_210"] = holdout_ds
    return splits


def load_sota_bundle(
    checkpoint_path: Path,
    geometry: str,
    device: torch.device,
    *,
    distractor_embeddings: Tensor | None = None,
    visual_centroid: Tensor | None = None,
) -> tuple[
    StudentDetector | QuadProposalDetector,
    DenseAnchorSemanticHead,
    RoIPooledSemanticHead,
    dict[str, Any],
]:
    """Load SOTA joint checkpoint containing detector, Option A head, and Option B head."""
    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=False)
    det_cfg = ckpt.get(
        "detector_config",
        {
            "fpn_channels": 128,
            "neck_type": "pan",
            "strides": (4, 8, 16, 32),
            "dual_head": True,
            "use_3dmf": True,
        },
    )
    strides = tuple(int(s) for s in det_cfg.get("strides", (4, 8, 16, 32)))
    det_cls = StudentDetector if geometry == "hbb" else QuadProposalDetector
    detector = det_cls(
        pretrained_backbone=False,
        fpn_channels=int(det_cfg.get("fpn_channels", 128)),
        neck_type=str(det_cfg.get("neck_type", "pan")),
        strides=strides,
        dual_head=bool(det_cfg.get("dual_head", True)),
        use_3dmf=bool(det_cfg.get("use_3dmf", True)),
    ).to(device)
    detector.load_state_dict(ckpt["model_state_dict"])
    detector.eval()

    head_a = DenseAnchorSemanticHead.from_config(ckpt["head_a_config"]).to(device)
    head_a.load_state_dict(ckpt["head_a_state_dict"], strict=False)
    if (
        head_a.distractor_embeddings is None or head_a.distractor_embeddings.numel() == 0
    ) and distractor_embeddings is not None:
        head_a.set_distractor_embeddings(distractor_embeddings)
    if (
        head_a.visual_centroid is None or head_a.visual_centroid.numel() == 0
    ) and visual_centroid is not None:
        head_a.set_visual_centroid(visual_centroid)
    head_a.eval()

    head_b = RoIPooledSemanticHead.from_config(ckpt["head_b_config"]).to(device)
    head_b.load_state_dict(ckpt["head_b_state_dict"], strict=False)
    if (
        head_b.distractor_embeddings is None or head_b.distractor_embeddings.numel() == 0
    ) and distractor_embeddings is not None:
        head_b.set_distractor_embeddings(distractor_embeddings)
    if (
        head_b.visual_centroid is None or head_b.visual_centroid.numel() == 0
    ) and visual_centroid is not None:
        head_b.set_visual_centroid(visual_centroid)
    head_b.eval()
    return detector, head_a, head_b, ckpt


def evaluate_sota_split(
    *,
    detector: StudentDetector | QuadProposalDetector,
    head_a: DenseAnchorSemanticHead,
    head_b: RoIPooledSemanticHead,
    dataset: Any,
    vocab_37: VocabularyBank,
    vocab_41: VocabularyBank,
    geometry: str,
    input_size: int,
    device: torch.device,
    batch_size: int = 16,
    workers: int = 4,
) -> dict[str, Any]:
    """Evaluate SOTA Detector + Option A, Option B, and Option A+B Fusion on one split."""
    strides = (4, 8, 16, 32)
    collate_fn = (
        collate_proposal_samples if geometry == "hbb" else collate_quad_proposal_samples
    )
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=workers,
        collate_fn=collate_fn,
    )
    if geometry == "hbb":
        decoder: Any = InferenceDecoder(
            strides=strides,
            top_k=500,
            nms_iou_threshold=0.50,
            max_detections=100,
            score_mode="objectness",
            nms_mode="containment_aware",
            score_threshold=0.01,
        )
    else:
        decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=500,
            nms_iou_threshold=0.50,
            max_proposals=100,
            centerness_alpha=0.0,
            nms_mode="containment_aware",
            score_threshold=0.01,
        )

    cat_to_idx = {cat: idx for idx, cat in enumerate(OPEN_VOCAB_CATEGORIES)}
    zs_to_41 = {
        cat: len(OPEN_VOCAB_CATEGORIES) + i
        for i, cat in enumerate(ZERO_SHOT_EVAL_CATEGORIES)
    }
    zs_start = len(OPEN_VOCAB_CATEGORIES)
    text_37 = vocab_37.embeddings.to(device)
    text_41 = vocab_41.embeddings.to(device)

    total_imgs = 0
    total_gt = 0
    recalled_10 = 0
    recalled_100_50 = 0
    recalled_100_75 = 0
    boxes_030 = 0
    overlap_pairs_025 = 0
    post_sem_overlap_pairs_025 = 0
    matched_gt_030 = 0
    dup_gt_030 = 0
    total_preds_030 = 0
    clutter_preds_030 = 0
    agn_ap_records: list[tuple[float, int]] = []

    variants = ("option_a", "option_b", "option_ab")
    prop_top1 = {v: 0 for v in variants}
    prop_top3 = {v: 0 for v in variants}
    prop_total = 0

    oracle_b_top1 = 0
    oracle_b_top3 = 0
    oracle_total = 0

    per_class_total: dict[str, int] = defaultdict(int)
    per_class_correct: dict[str, dict[str, int]] = {
        v: defaultdict(int) for v in variants
    }
    ab_agreement_total = 0
    ab_agreement_hits = 0
    predicted_cat_counts_020: dict[str, dict[str, int]] = {
        v: defaultdict(int) for v in variants
    }
    signboard_total = 0
    signboard_to_win_door = {v: 0 for v in variants}
    win_door_indices = {
        cat_to_idx["window"],
        cat_to_idx["door"],
    }

    clutter_rejected = {v: 0 for v in variants}
    ov_gt_counts: dict[int, int] = defaultdict(int)
    ov_ap_records: dict[str, dict[int, list[tuple[float, int]]]] = {
        v: defaultdict(list) for v in variants
    }

    # Zero-shot counters (matched proposals IoU >= 0.50 + oracle RoI for Option B)
    zs_prop_total = 0
    zs_4way_top1 = {v: 0 for v in variants}
    zs_41way_top1 = {v: 0 for v in variants}
    zs_41way_top5 = {v: 0 for v in variants}
    zs_oracle_b_total = 0
    zs_oracle_b_4way = 0
    zs_oracle_b_41way_top1 = 0
    zs_oracle_b_41way_top5 = 0
    seen_on_41_top1 = {v: 0 for v in variants}

    with torch.no_grad():
        for images, samples in loader:
            images = images.to(device, non_blocking=True)
            features = detector.forward_features(images)
            raw_out = detector.forward_eval_head(features)
            if geometry == "hbb":
                shapes = tuple(
                    (t.shape[-2], t.shape[-1]) for t in raw_out.objectness  # type: ignore[union-attr]
                )
            else:
                shapes = tuple(
                    (t.shape[-2], t.shape[-1]) for t in raw_out.quality  # type: ignore[union-attr]
                )
            level_masks_per_b = [
                point_validity_from_pixel_mask(
                    s.valid_mask.to(device), shapes, strides
                )[1]
                for s in samples
            ]
            batched_masks = tuple(
                torch.stack([m[lvl] for m in level_masks_per_b], dim=0)
                for lvl in range(len(strides))
            )
            dets = decoder(raw_out, (input_size, input_size), batched_masks)
            tower_a = head_a.forward_tower(features)

            # Batch Option B RoI inference across all proposals + oracle GT boxes
            prop_boxes_list = [d.boxes for d in dets]
            z_b_all = head_b(
                features,
                prop_boxes_list,
                image_shapes=[(input_size, input_size)] * len(samples),
            )
            gt_boxes_batch = [
                s.boxes.to(device)
                if geometry == "hbb"
                else RegionEmbeddingHead.boxes_from_quads(s.quads.to(device))
                for s in samples
            ]
            z_b_oracle_all = head_b(
                features,
                gt_boxes_batch,
                image_shapes=[(input_size, input_size)] * len(samples),
            )

            b_cursor = 0
            oracle_cursor = 0
            for b_i, (sample, det) in enumerate(zip(samples, dets, strict=True)):
                total_imgs += 1
                n_p = int(det.scores.numel())
                z_b = z_b_all[b_cursor : b_cursor + n_p]
                b_cursor += n_p

                if geometry == "hbb":
                    gt_geom = sample.boxes.to(device)
                    ign_geom = sample.ignore_boxes.to(device)
                    pred_geom = det.boxes
                    iou_fn = box_iou
                else:
                    gt_geom = sample.quads.to(device)
                    ign_geom = sample.ignore_quads.to(device)
                    pred_geom = det.quads
                    iou_fn = pairwise_quad_iou

                n_gt = int(gt_geom.shape[0])
                total_gt += n_gt
                z_b_gt = z_b_oracle_all[oracle_cursor : oracle_cursor + n_gt]
                oracle_cursor += n_gt

                # Canonical GT class mapping
                gt_canon = [
                    map_to_canonical(c)
                    for c in sample.category_names
                ]
                gt_cls_idx = [
                    cat_to_idx[c] if c in cat_to_idx else -1 for c in gt_canon
                ]
                gt_zs_idx = [
                    zs_to_41.get(normalize_category_name(raw_c), -1)
                    if canon is None
                    else -1
                    for raw_c, canon in zip(
                        sample.category_names, gt_canon, strict=True
                    )
                ]
                for c_idx in gt_cls_idx:
                    if c_idx >= 0:
                        ov_gt_counts[c_idx] += 1

                # Oracle Option B accuracy (pure RoI semantic quality on exact GT boxes)
                if n_gt > 0:
                    _, probs_b_gt_37 = head_b.classify_embeddings(z_b_gt, text_37)
                    _, probs_b_gt_41 = head_b.classify_embeddings(z_b_gt, text_41)
                    for g_i, c_idx in enumerate(gt_cls_idx):
                        if c_idx >= 0:
                            oracle_total += 1
                            top3_g = probs_b_gt_37[g_i].topk(3).indices.tolist()
                            if top3_g[0] == c_idx:
                                oracle_b_top1 += 1
                            if c_idx in top3_g:
                                oracle_b_top3 += 1
                        zs_idx = gt_zs_idx[g_i]
                        if zs_idx >= 0:
                            zs_oracle_b_total += 1
                            zs_sub = int(probs_b_gt_41[g_i, zs_start:].argmax().item())
                            if zs_start + zs_sub == zs_idx:
                                zs_oracle_b_4way += 1
                            top5_41 = probs_b_gt_41[g_i].topk(5).indices.tolist()
                            if top5_41[0] == zs_idx:
                                zs_oracle_b_41way_top1 += 1
                            if zs_idx in top5_41:
                                zs_oracle_b_41way_top5 += 1

                # Localization & overlap stats
                iou_mat = (
                    iou_fn(pred_geom, gt_geom)
                    if (n_p > 0 and n_gt > 0)
                    else torch.zeros((n_p, n_gt), device=device)
                )
                ign_mat = (
                    iou_fn(pred_geom, ign_geom)
                    if (n_p > 0 and ign_geom.shape[0] > 0)
                    else torch.zeros((n_p, 0), device=device)
                )

                if n_gt > 0 and n_p > 0:
                    recalled_100_50 += int((iou_mat.max(dim=0).values >= 0.50).sum().item())
                    recalled_100_75 += int((iou_mat.max(dim=0).values >= 0.75).sum().item())
                    recalled_10 += int(
                        (iou_mat[: min(10, n_p)].max(dim=0).values >= 0.50).sum().item()
                    )

                # Class-agnostic AP@50 greedy matching
                gt_used_agn = torch.zeros(n_gt, dtype=torch.bool, device=device)
                for p_i in range(n_p):
                    sc = float(det.scores[p_i].item())
                    if n_gt > 0:
                        ious_p = iou_mat[p_i].clone()
                        ious_p[gt_used_agn] = -1.0
                        best_iou, best_g = ious_p.max(dim=0)
                        if float(best_iou.item()) >= 0.50:
                            gt_used_agn[best_g] = True
                            agn_ap_records.append((sc, 1))
                            continue
                    if ign_mat.shape[1] > 0 and float(ign_mat[p_i].max().item()) >= 0.50:
                        continue
                    agn_ap_records.append((sc, 0))

                keep_030 = det.scores >= 0.30
                n_030 = int(keep_030.sum().item())
                boxes_030 += n_030
                total_preds_030 += n_030
                if n_030 > 1:
                    sub_geom = pred_geom[keep_030]
                    pw = iou_fn(sub_geom, sub_geom)
                    tri = torch.triu(
                        torch.ones_like(pw, dtype=torch.bool), diagonal=1
                    )
                    overlap_pairs_025 += int((pw[tri] > 0.25).sum().item())
                if n_gt > 0 and n_030 > 0:
                    hits_per_gt = (iou_mat[keep_030] >= 0.50).sum(dim=0)
                    matched_mask = hits_per_gt >= 1
                    matched_gt_030 += int(matched_mask.sum().item())
                    dup_gt_030 += int((hits_per_gt >= 2).sum().item())

                if n_p == 0 or det.flat_indices is None:
                    continue

                b_idx_t = torch.full(
                    (n_p,), b_i, dtype=torch.long, device=device
                )
                z_a = head_a.forward_indexed(
                    features,
                    b_idx_t,
                    det.flat_indices,
                    tower_features=tower_a,
                )
                _, probs_a_37 = head_a.classify_embeddings(z_a, text_37)
                _, probs_b_37 = head_b.classify_embeddings(z_b, text_37)
                probs_ab_37 = 0.30 * probs_a_37 + 0.70 * probs_b_37

                _, probs_a_41 = head_a.classify_embeddings(z_a, text_41)
                _, probs_b_41 = head_b.classify_embeddings(z_b, text_41)
                probs_ab_41 = 0.30 * probs_a_41 + 0.70 * probs_b_41

                probs_map_37 = {
                    "option_a": probs_a_37,
                    "option_b": probs_b_37,
                    "option_ab": probs_ab_37,
                }
                probs_map_41 = {
                    "option_a": probs_a_41,
                    "option_b": probs_b_41,
                    "option_ab": probs_ab_41,
                }

                # 1. Matched-proposal classification (seen 37-class + unseen 41-class zero-shot)
                if n_gt > 0:
                    best_ious, best_p = iou_mat.max(dim=0)
                    for g_i in range(n_gt):
                        if float(best_ious[g_i].item()) < 0.50:
                            continue
                        p_idx = int(best_p[g_i].item())
                        c_idx = gt_cls_idx[g_i]
                        c_name = gt_canon[g_i]
                        if c_idx >= 0 and c_name is not None:
                            prop_total += 1
                            per_class_total[c_name] += 1
                            if c_name == "signboard":
                                signboard_total += 1
                            ab_agreement_total += 1
                            if int(probs_a_37[p_idx].argmax().item()) == int(probs_b_37[p_idx].argmax().item()):
                                ab_agreement_hits += 1
                            for v_name, p_mat in probs_map_37.items():
                                top3_v = p_mat[p_idx].topk(3).indices.tolist()
                                if top3_v[0] == c_idx:
                                    prop_top1[v_name] += 1
                                    per_class_correct[v_name][c_name] += 1
                                if c_idx in top3_v:
                                    prop_top3[v_name] += 1
                                if c_name == "signboard" and top3_v[0] in win_door_indices:
                                    signboard_to_win_door[v_name] += 1
                            for v_name, p_mat41 in probs_map_41.items():
                                if int(p_mat41[p_idx].argmax().item()) == c_idx:
                                    seen_on_41_top1[v_name] += 1

                        zs_idx = gt_zs_idx[g_i]
                        if zs_idx >= 0:
                            zs_prop_total += 1
                            for v_name, p_mat41 in probs_map_41.items():
                                zs_sub = int(p_mat41[p_idx, zs_start:].argmax().item())
                                if zs_start + zs_sub == zs_idx:
                                    zs_4way_top1[v_name] += 1
                                top5_v = p_mat41[p_idx].topk(5).indices.tolist()
                                if top5_v[0] == zs_idx:
                                    zs_41way_top1[v_name] += 1
                                if zs_idx in top5_v:
                                    zs_41way_top5[v_name] += 1

                # 2. Pure background clutter rejection (@0.30 proposals with max_gt_iou < 0.15)
                if n_030 > 0:
                    max_gt_iou = (
                        iou_mat[keep_030].max(dim=1).values
                        if n_gt > 0
                        else torch.zeros(n_030, device=device)
                    )
                    max_ign_iou = (
                        ign_mat[keep_030].max(dim=1).values
                        if ign_mat.shape[1] > 0
                        else torch.zeros(n_030, device=device)
                    )
                    clut_mask = (max_gt_iou < 0.15) & (max_ign_iou < 0.25)
                    n_clut = int(clut_mask.sum().item())
                    clutter_preds_030 += n_clut
                    if n_clut > 0:
                        for v_name, p_mat in probs_map_37.items():
                            max_sig = p_mat[keep_030][clut_mask].max(dim=-1).values
                            clutter_rejected[v_name] += int((max_sig < 0.35).sum().item())

                # 3. End-to-End Open-Vocabulary mAP@50 (geometric-mean joint score = sqrt(s_obj * p_cls))
                for v_name, p_mat in probs_map_37.items():
                    top_prob, top_cls = p_mat.max(dim=-1)
                    joint_scores = torch.sqrt((det.scores * top_prob).clamp(min=0.0))
                    keep_ov = filter_semantic_detections(
                        det.boxes,
                        det.scores,
                        p_mat,
                        OPEN_VOCAB_CATEGORIES,
                        quads=det.quads if geometry == "quad" else None,
                    )
                    if v_name == "option_ab" and keep_ov.numel() > 1:
                        vis_mask = (det.scores[keep_ov] >= 0.25) & (top_prob[keep_ov] >= 0.12)
                        if int(vis_mask.sum().item()) > 1:
                            kept_geom = pred_geom[keep_ov][vis_mask]
                            pw_sem = iou_fn(kept_geom, kept_geom)
                            tri_sem = torch.triu(
                                torch.ones_like(pw_sem, dtype=torch.bool), diagonal=1
                            )
                            post_sem_overlap_pairs_025 += int((pw_sem[tri_sem] > 0.25).sum().item())
                    if keep_ov.numel() > 0:
                        active_sem = keep_ov[(joint_scores[keep_ov] >= 0.20) & (top_prob[keep_ov] >= 0.20)]
                        for idx_act in active_sem.tolist():
                            act_cls = int(top_cls[idx_act].item())
                            if 0 <= act_cls < len(OPEN_VOCAB_CATEGORIES):
                                predicted_cat_counts_020[v_name][OPEN_VOCAB_CATEGORIES[act_cls]] += 1
                    if keep_ov.numel() > 0:
                        keep_ov = keep_ov[joint_scores[keep_ov] >= 0.02]
                        if keep_ov.numel() > 1:
                            keep_ov = keep_ov[
                                torch.argsort(joint_scores[keep_ov], descending=True, stable=True)
                            ]
                    if keep_ov.numel() > 100:
                        keep_ov = keep_ov[:100]
                    assert isinstance(keep_ov, Tensor)
                    gt_used_cls: dict[int, set[int]] = defaultdict(set)
                    for idx_t in keep_ov.tolist():
                        c_pred = int(top_cls[idx_t].item())
                        sc_pred = float(joint_scores[idx_t].item())
                        matched_tp = False
                        best_g_idx = -1
                        best_g_iou = 0.50
                        for g_i in range(n_gt):
                            if gt_cls_idx[g_i] != c_pred or g_i in gt_used_cls[c_pred]:
                                continue
                            val_iou = float(iou_mat[idx_t, g_i].item())
                            if val_iou >= best_g_iou:
                                best_g_iou = val_iou
                                best_g_idx = g_i
                        if best_g_idx >= 0:
                            gt_used_cls[c_pred].add(best_g_idx)
                            matched_tp = True
                        ov_ap_records[v_name][c_pred].append(
                            (sc_pred, 1 if matched_tp else 0)
                        )

    # Aggregate mAP@50 across active seen classes
    active_classes = [c for c, cnt in ov_gt_counts.items() if cnt > 0]
    ov_map50: dict[str, float] = {}
    for v_name in variants:
        aps = [
            compute_average_precision(ov_ap_records[v_name][c], ov_gt_counts[c])
            for c in active_classes
        ]
        ov_map50[v_name] = sum(aps) / max(len(aps), 1)

    per_class_report: dict[str, Any] = {}
    for c_name in OPEN_VOCAB_CATEGORIES:
        tot_c = per_class_total[c_name]
        pred_ab = predicted_cat_counts_020["option_ab"].get(c_name, 0)
        if tot_c > 0 or pred_ab > 0:
            per_class_report[c_name] = {
                "count": tot_c,
                "predicted_count_020": pred_ab,
                "option_a_top1": round(per_class_correct["option_a"][c_name] / tot_c, 4) if tot_c > 0 else None,
                "option_b_top1": round(per_class_correct["option_b"][c_name] / tot_c, 4) if tot_c > 0 else None,
                "option_ab_top1": round(per_class_correct["option_ab"][c_name] / tot_c, 4) if tot_c > 0 else None,
            }
    zero_gt_detected = {
        c_name: predicted_cat_counts_020["option_ab"].get(c_name, 0)
        for c_name in OPEN_VOCAB_CATEGORIES
        if per_class_total[c_name] == 0 and predicted_cat_counts_020["option_ab"].get(c_name, 0) > 0
    }

    d_img = max(total_imgs, 1)
    d_gt = max(total_gt, 1)
    d_prop = max(prop_total, 1)
    d_clut = max(clutter_preds_030, 1)
    d_zs = max(zs_prop_total, 1)

    return {
        "num_images": total_imgs,
        "num_gt": total_gt,
        "localization": {
            "boxes_per_img_030": round(boxes_030 / d_img, 2),
            "overlap_pairs_025": round(overlap_pairs_025 / d_img, 2),
            "post_semantic_nms_overlap_pairs_025": round(post_sem_overlap_pairs_025 / d_img, 2),
            "dup_gt_rate": round(dup_gt_030 / max(matched_gt_030, 1), 4),
            "clutter_prop_rate_030": round(clutter_preds_030 / max(total_preds_030, 1), 4),
            "recall_10_50": round(recalled_10 / d_gt, 4),
            "recall_100_50": round(recalled_100_50 / d_gt, 4),
            "recall_100_75": round(recalled_100_75 / d_gt, 4),
            "class_agnostic_ap50": round(
                compute_average_precision(agn_ap_records, total_gt), 4
            ),
        },
        "semantics_37_seen": {
            "evaluated_recalled_gt": prop_total,
            "option_a": {
                "matched_prop_top1": round(prop_top1["option_a"] / d_prop, 4),
                "matched_prop_top3": round(prop_top3["option_a"] / d_prop, 4),
                "seen_on_41bank_top1": round(seen_on_41_top1["option_a"] / d_prop, 4),
                "open_vocab_map50": round(ov_map50["option_a"], 4),
                "clutter_rejection_rate": round(clutter_rejected["option_a"] / d_clut, 4),
                "signboard_to_window_door_err": round(
                    signboard_to_win_door["option_a"] / max(signboard_total, 1), 4
                ),
            },
            "option_b": {
                "matched_prop_top1": round(prop_top1["option_b"] / d_prop, 4),
                "matched_prop_top3": round(prop_top3["option_b"] / d_prop, 4),
                "oracle_gt_box_top1": round(oracle_b_top1 / max(oracle_total, 1), 4),
                "oracle_gt_box_top3": round(oracle_b_top3 / max(oracle_total, 1), 4),
                "seen_on_41bank_top1": round(seen_on_41_top1["option_b"] / d_prop, 4),
                "open_vocab_map50": round(ov_map50["option_b"], 4),
                "clutter_rejection_rate": round(clutter_rejected["option_b"] / d_clut, 4),
                "signboard_to_window_door_err": round(
                    signboard_to_win_door["option_b"] / max(signboard_total, 1), 4
                ),
            },
            "option_ab": {
                "matched_prop_top1": round(prop_top1["option_ab"] / d_prop, 4),
                "matched_prop_top3": round(prop_top3["option_ab"] / d_prop, 4),
                "seen_on_41bank_top1": round(seen_on_41_top1["option_ab"] / d_prop, 4),
                "open_vocab_map50": round(ov_map50["option_ab"], 4),
                "clutter_rejection_rate": round(clutter_rejected["option_ab"] / d_clut, 4),
                "signboard_to_window_door_err": round(
                    signboard_to_win_door["option_ab"] / max(signboard_total, 1), 4
                ),
            },
            "signboard_instances": signboard_total,
            "option_a_b_top1_agreement": round(ab_agreement_hits / max(ab_agreement_total, 1), 4),
            "active_predicted_categories_37": len(predicted_cat_counts_020["option_ab"]),
            "zero_gt_categories_detected": zero_gt_detected,
            "predicted_category_counts_020": dict(predicted_cat_counts_020["option_ab"]),
            "per_class_top1": per_class_report,
        },
        "zero_shot_4_unseen": {
            "unseen_categories": list(ZERO_SHOT_EVAL_CATEGORIES),
            "recalled_unseen_gt": zs_prop_total,
            "oracle_unseen_gt": zs_oracle_b_total,
            "option_a": {
                "unseen_4way_top1": round(zs_4way_top1["option_a"] / d_zs, 4),
                "full_41way_top1": round(zs_41way_top1["option_a"] / d_zs, 4),
                "full_41way_top5": round(zs_41way_top5["option_a"] / d_zs, 4),
            },
            "option_b": {
                "unseen_4way_top1": round(zs_4way_top1["option_b"] / d_zs, 4),
                "full_41way_top1": round(zs_41way_top1["option_b"] / d_zs, 4),
                "full_41way_top5": round(zs_41way_top5["option_b"] / d_zs, 4),
                "oracle_4way_top1": round(
                    zs_oracle_b_4way / max(zs_oracle_b_total, 1), 4
                ),
                "oracle_41way_top1": round(
                    zs_oracle_b_41way_top1 / max(zs_oracle_b_total, 1), 4
                ),
                "oracle_41way_top5": round(
                    zs_oracle_b_41way_top5 / max(zs_oracle_b_total, 1), 4
                ),
            },
            "option_ab": {
                "unseen_4way_top1": round(zs_4way_top1["option_ab"] / d_zs, 4),
                "full_41way_top1": round(zs_41way_top1["option_ab"] / d_zs, 4),
                "full_41way_top5": round(zs_41way_top5["option_ab"] / d_zs, 4),
            },
        },
    }


def evaluate_legacy_split(
    *,
    detector_ckpt_path: Path,
    region_head_ckpt_path: Path,
    vocab_path: Path,
    dataset: Any,
    geometry: str,
    input_size: int,
    device: torch.device,
    batch_size: int = 16,
    workers: int = 4,
) -> dict[str, Any]:
    """Evaluate legacy 8k_full detector + RegionEmbeddingHead (v3/v4) on the exact same split."""
    strides = (4, 8, 16, 32)
    det_ckpt = torch.load(detector_ckpt_path, map_location=device, weights_only=False)
    has_aux = any("aux_head" in k for k in det_ckpt.get("model_state_dict", {}))
    if geometry == "hbb":
        detector: StudentDetector | QuadProposalDetector = StudentDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=has_aux,
        ).to(device)
        decoder: Any = InferenceDecoder(
            strides=strides,
            top_k=1000,
            nms_iou_threshold=0.45,
            max_detections=100,
            score_mode="objectness_x_centerness",
            centerness_alpha=0.25,
            nms_mode="part_aware",
            part_scale_ratio=0.4,
            part_max_iou=0.65,
            score_threshold=0.05,
        )
        collate_fn: Any = collate_proposal_samples
    else:
        detector = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
            dual_head=has_aux,
        ).to(device)
        decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=1000,
            nms_iou_threshold=0.45,
            max_proposals=100,
            centerness_alpha=0.0,
            nms_mode="part_aware",
            part_scale_ratio=0.4,
            part_max_iou=0.65,
            score_threshold=0.05,
        )
        collate_fn = collate_quad_proposal_samples

    detector.load_state_dict(det_ckpt["model_state_dict"])
    detector.eval()

    region_head, r_ckpt = RegionEmbeddingHead.load_from_checkpoint(
        region_head_ckpt_path, device=device
    )
    region_head.eval()
    cls_temp = float(r_ckpt.get("cls_temperature", 1.0))
    vocab = VocabularyBank.load(vocab_path, device=device)
    legacy_cats = [map_to_canonical(c) or c for c in vocab.categories]
    distractor_set = set(DISTRACTOR_CATEGORIES)
    cat_to_idx = {cat: idx for idx, cat in enumerate(OPEN_VOCAB_CATEGORIES)}

    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=workers,
        collate_fn=collate_fn,
    )

    total_imgs = 0
    total_gt = 0
    recalled_10 = 0
    recalled_100_50 = 0
    recalled_100_75 = 0
    boxes_030 = 0
    overlap_pairs_025 = 0
    matched_gt_030 = 0
    dup_gt_030 = 0
    total_preds_030 = 0
    clutter_preds_030 = 0
    clutter_rejected = 0
    agn_ap_records: list[tuple[float, int]] = []

    prop_total = 0
    prop_top1 = 0
    prop_top3 = 0
    oracle_total = 0
    oracle_top1 = 0
    signboard_total = 0
    signboard_to_win_door = 0
    ov_gt_counts: dict[int, int] = defaultdict(int)
    ov_ap_records: dict[int, list[tuple[float, int]]] = defaultdict(list)

    with torch.no_grad():
        for images, samples in loader:
            images = images.to(device, non_blocking=True)
            features = detector.forward_features(images)
            raw_out = detector.forward_eval_head(features)
            if geometry == "hbb":
                shapes = tuple(
                    (t.shape[-2], t.shape[-1]) for t in raw_out.objectness  # type: ignore[union-attr]
                )
            else:
                shapes = tuple(
                    (t.shape[-2], t.shape[-1]) for t in raw_out.quality  # type: ignore[union-attr]
                )
            level_masks_per_b = [
                point_validity_from_pixel_mask(
                    s.valid_mask.to(device), shapes, strides
                )[1]
                for s in samples
            ]
            batched_masks = tuple(
                torch.stack([m[lvl] for m in level_masks_per_b], dim=0)
                for lvl in range(len(strides))
            )
            dets = decoder(raw_out, (input_size, input_size), batched_masks)

            prop_boxes_list = [d.boxes for d in dets]
            embs_all = region_head(
                features,
                prop_boxes_list,
                image_shapes=[(input_size, input_size)] * len(samples),
            )
            gt_boxes_batch = [
                s.boxes.to(device)
                if geometry == "hbb"
                else RegionEmbeddingHead.boxes_from_quads(s.quads.to(device))
                for s in samples
            ]
            embs_gt_all = region_head(
                features,
                gt_boxes_batch,
                image_shapes=[(input_size, input_size)] * len(samples),
            )

            cursor = 0
            gt_cursor = 0
            for sample, det in zip(samples, dets, strict=True):
                total_imgs += 1
                n_p = int(det.scores.numel())
                embs_p = embs_all[cursor : cursor + n_p]
                cursor += n_p

                if geometry == "hbb":
                    gt_geom = sample.boxes.to(device)
                    ign_geom = sample.ignore_boxes.to(device)
                    pred_geom = det.boxes
                    iou_fn = box_iou
                else:
                    gt_geom = sample.quads.to(device)
                    ign_geom = sample.ignore_quads.to(device)
                    pred_geom = det.quads
                    iou_fn = pairwise_quad_iou

                n_gt = int(gt_geom.shape[0])
                total_gt += n_gt
                embs_gt = embs_gt_all[gt_cursor : gt_cursor + n_gt]
                gt_cursor += n_gt

                gt_canon = [
                    map_to_canonical(c)
                    for c in sample.category_names
                ]
                gt_cls_idx = [
                    cat_to_idx[c] if c in cat_to_idx else -1 for c in gt_canon
                ]
                for c_idx in gt_cls_idx:
                    if c_idx >= 0:
                        ov_gt_counts[c_idx] += 1

                if n_gt > 0:
                    _, gt_probs = vocab.classify(
                        embs_gt,
                        mode="softmax",
                        temperature=cls_temp,
                        use_centroid_subtraction=False,
                    )
                    for g_i, c_name in enumerate(gt_canon):
                        if c_name in cat_to_idx:
                            oracle_total += 1
                            pred_c = legacy_cats[int(gt_probs[g_i].argmax().item())]
                            if pred_c == c_name:
                                oracle_top1 += 1

                iou_mat = (
                    iou_fn(pred_geom, gt_geom)
                    if (n_p > 0 and n_gt > 0)
                    else torch.zeros((n_p, n_gt), device=device)
                )
                ign_mat = (
                    iou_fn(pred_geom, ign_geom)
                    if (n_p > 0 and ign_geom.shape[0] > 0)
                    else torch.zeros((n_p, 0), device=device)
                )

                if n_gt > 0 and n_p > 0:
                    recalled_100_50 += int((iou_mat.max(dim=0).values >= 0.50).sum().item())
                    recalled_100_75 += int((iou_mat.max(dim=0).values >= 0.75).sum().item())
                    recalled_10 += int(
                        (iou_mat[: min(10, n_p)].max(dim=0).values >= 0.50).sum().item()
                    )

                gt_used_agn = torch.zeros(n_gt, dtype=torch.bool, device=device)
                for p_i in range(n_p):
                    sc = float(det.scores[p_i].item())
                    if n_gt > 0:
                        ious_p = iou_mat[p_i].clone()
                        ious_p[gt_used_agn] = -1.0
                        best_iou, best_g = ious_p.max(dim=0)
                        if float(best_iou.item()) >= 0.50:
                            gt_used_agn[best_g] = True
                            agn_ap_records.append((sc, 1))
                            continue
                    if ign_mat.shape[1] > 0 and float(ign_mat[p_i].max().item()) >= 0.50:
                        continue
                    agn_ap_records.append((sc, 0))

                keep_030 = det.scores >= 0.30
                n_030 = int(keep_030.sum().item())
                boxes_030 += n_030
                total_preds_030 += n_030
                if n_030 > 1:
                    sub_geom = pred_geom[keep_030]
                    pw = iou_fn(sub_geom, sub_geom)
                    tri = torch.triu(
                        torch.ones_like(pw, dtype=torch.bool), diagonal=1
                    )
                    overlap_pairs_025 += int((pw[tri] > 0.25).sum().item())
                if n_gt > 0 and n_030 > 0:
                    hits_per_gt = (iou_mat[keep_030] >= 0.50).sum(dim=0)
                    matched_gt_030 += int((hits_per_gt >= 1).sum().item())
                    dup_gt_030 += int((hits_per_gt >= 2).sum().item())

                if n_p == 0:
                    continue

                _, probs = vocab.classify(
                    embs_p,
                    mode="softmax",
                    temperature=cls_temp,
                    use_centroid_subtraction=False,
                )

                if n_gt > 0:
                    best_ious, best_p = iou_mat.max(dim=0)
                    for g_i in range(n_gt):
                        if float(best_ious[g_i].item()) < 0.50:
                            continue
                        c_name = gt_canon[g_i]
                        if c_name not in cat_to_idx:
                            continue
                        p_idx = int(best_p[g_i].item())
                        prop_total += 1
                        top3_i = probs[p_idx].topk(3).indices.tolist()
                        top3_names = [legacy_cats[idx_k] for idx_k in top3_i]
                        if top3_names[0] == c_name:
                            prop_top1 += 1
                        if c_name in top3_names:
                            prop_top3 += 1
                        if c_name == "signboard":
                            signboard_total += 1
                            if top3_names[0] in {"window", "door"}:
                                signboard_to_win_door += 1

                if n_030 > 0:
                    max_gt_iou = (
                        iou_mat[keep_030].max(dim=1).values
                        if n_gt > 0
                        else torch.zeros(n_030, device=device)
                    )
                    max_ign_iou = (
                        ign_mat[keep_030].max(dim=1).values
                        if ign_mat.shape[1] > 0
                        else torch.zeros(n_030, device=device)
                    )
                    clut_mask = (max_gt_iou < 0.15) & (max_ign_iou < 0.25)
                    n_clut = int(clut_mask.sum().item())
                    clutter_preds_030 += n_clut
                    if n_clut > 0:
                        sub_p = probs[keep_030][clut_mask]
                        top_v, top_i = sub_p.max(dim=-1)
                        for k_c in range(n_clut):
                            c_pred_name = vocab.categories[int(top_i[k_c].item())]
                            if c_pred_name in distractor_set or float(top_v[k_c].item()) < 0.35:
                                clutter_rejected += 1

                # End-to-end mAP@50 across shared categories
                gt_used_cls: dict[int, set[int]] = defaultdict(set)
                top_prob, top_i = probs.max(dim=-1)
                joint_sc = det.scores * top_prob
                order = torch.argsort(joint_sc, descending=True)
                for idx_t in order.tolist():
                    c_pred_name = legacy_cats[int(top_i[idx_t].item())]
                    if c_pred_name not in cat_to_idx:
                        continue
                    c_pred = cat_to_idx[c_pred_name]
                    sc_pred = float(joint_sc[idx_t].item())
                    matched_tp = False
                    best_g_idx = -1
                    best_g_iou = 0.50
                    for g_i in range(n_gt):
                        if gt_cls_idx[g_i] != c_pred or g_i in gt_used_cls[c_pred]:
                            continue
                        val_iou = float(iou_mat[idx_t, g_i].item())
                        if val_iou >= best_g_iou:
                            best_g_iou = val_iou
                            best_g_idx = g_i
                    if best_g_idx >= 0:
                        gt_used_cls[c_pred].add(best_g_idx)
                        matched_tp = True
                    ov_ap_records[c_pred].append((sc_pred, 1 if matched_tp else 0))

    active_classes = [c for c, cnt in ov_gt_counts.items() if cnt > 0]
    aps = [
        compute_average_precision(ov_ap_records[c], ov_gt_counts[c])
        for c in active_classes
    ]

    d_img = max(total_imgs, 1)
    d_gt = max(total_gt, 1)
    d_prop = max(prop_total, 1)
    return {
        "num_images": total_imgs,
        "num_gt": total_gt,
        "localization": {
            "boxes_per_img_030": round(boxes_030 / d_img, 2),
            "overlap_pairs_025": round(overlap_pairs_025 / d_img, 2),
            "dup_gt_rate": round(dup_gt_030 / max(matched_gt_030, 1), 4),
            "clutter_prop_rate_030": round(
                clutter_preds_030 / max(total_preds_030, 1), 4
            ),
            "recall_10_50": round(recalled_10 / d_gt, 4),
            "recall_100_50": round(recalled_100_50 / d_gt, 4),
            "recall_100_75": round(recalled_100_75 / d_gt, 4),
            "class_agnostic_ap50": round(
                compute_average_precision(agn_ap_records, total_gt), 4
            ),
        },
        "semantics_37_seen": {
            "evaluated_recalled_gt": prop_total,
            "matched_prop_top1": round(prop_top1 / d_prop, 4),
            "matched_prop_top3": round(prop_top3 / d_prop, 4),
            "oracle_gt_box_top1": round(oracle_top1 / max(oracle_total, 1), 4),
            "open_vocab_map50": round(sum(aps) / max(len(aps), 1), 4),
            "clutter_rejection_rate": round(
                clutter_rejected / max(clutter_preds_030, 1), 4
            ),
            "signboard_to_window_door_err": round(
                signboard_to_win_door / max(signboard_total, 1), 4
            ),
        },
    }


def profile_edge_latency_and_params(
    *,
    detector: StudentDetector | QuadProposalDetector,
    head_a: DenseAnchorSemanticHead,
    head_b: RoIPooledSemanticHead,
    vocab_37: VocabularyBank,
    geometry: str,
    legacy_region_head_ckpt: Path | None = None,
    input_size: int = 384,
) -> dict[str, Any]:
    """Measure parameter counts, 4-thread CPU latency (Pi 5 proxy), and CUDA latency."""
    strides = (4, 8, 16, 32)
    # Inference parameters exclude auxiliary training-only head
    det_infer_params = sum(
        p.numel()
        for n, p in detector.named_parameters()
        if not n.startswith("aux_head.")
    )
    params_a = sum(p.numel() for p in head_a.parameters())
    params_b = sum(p.numel() for p in head_b.parameters())

    legacy_params = 0
    if legacy_region_head_ckpt is not None and legacy_region_head_ckpt.exists():
        leg_h, _ = RegionEmbeddingHead.load_from_checkpoint(
            legacy_region_head_ckpt, device="cpu"
        )
        legacy_params = sum(p.numel() for p in leg_h.parameters())

    if geometry == "hbb":
        decoder: Any = InferenceDecoder(
            strides=strides,
            top_k=500,
            nms_iou_threshold=0.50,
            max_detections=30,
            score_mode="objectness",
            nms_mode="containment_aware",
            score_threshold=0.05,
        )
    else:
        decoder = QuadInferenceDecoder(
            strides=strides,
            pre_nms_top_k=500,
            nms_iou_threshold=0.50,
            max_proposals=30,
            centerness_alpha=0.0,
            nms_mode="containment_aware",
            score_threshold=0.05,
        )

    def _time_on_device(dev_str: str, warmup: int, iters: int) -> dict[str, float]:
        dev = torch.device(dev_str)
        det_d = copy.deepcopy(detector).to(dev).eval()
        ha_d = copy.deepcopy(head_a).to(dev).eval()
        hb_d = copy.deepcopy(head_b).to(dev).eval()
        txt = vocab_37.embeddings.to(dev)
        x = torch.randn(1, 3, input_size, input_size, device=dev)
        synth_boxes = torch.tensor(
            [[16.0 + i * 5.0, 20.0 + i * 4.0, 96.0 + i * 5.0, 110.0 + i * 4.0] for i in range(25)],
            dtype=torch.float32,
            device=dev,
        )
        synth_flat = torch.arange(0, 250, 10, dtype=torch.long, device=dev)
        synth_batch = torch.zeros_like(synth_flat)

        def sync() -> None:
            if dev.type == "cuda":
                torch.cuda.synchronize()

        def measure(fn: Any) -> float:
            with torch.no_grad():
                for _ in range(warmup):
                    fn()
                sync()
                t0 = time.perf_counter()
                for _ in range(iters):
                    fn()
                sync()
                return (time.perf_counter() - t0) * 1000.0 / iters

        ms_det = measure(
            lambda: decoder(det_d.head(det_d.forward_features(x)), (input_size, input_size))
        )

        with torch.no_grad():
            feats = det_d.forward_features(x)

        ms_head_a_indexed = measure(
            lambda: ha_d.classify_embeddings(
                ha_d.forward_indexed(feats, synth_batch, synth_flat), txt
            )
        )
        ms_head_a_dense = measure(lambda: ha_d.forward_dense(feats))
        ms_head_b_roi = measure(
            lambda: hb_d.classify_embeddings(
                hb_d(feats, [synth_boxes], image_shapes=[(input_size, input_size)]), txt
            )
        )

        def _full_a() -> Any:
            f = det_d.forward_features(x)
            d = decoder(det_d.head(f), (input_size, input_size))[0]
            idx = d.flat_indices if d.flat_indices is not None and d.flat_indices.numel() > 0 else synth_flat
            b_idx = torch.zeros_like(idx)
            return ha_d.classify_embeddings(ha_d.forward_indexed(f, b_idx, idx), txt)

        def _full_b() -> Any:
            f = det_d.forward_features(x)
            d = decoder(det_d.head(f), (input_size, input_size))[0]
            bxs = d.boxes if d.boxes.numel() > 0 else synth_boxes
            return hb_d.classify_embeddings(
                hb_d(f, [bxs], image_shapes=[(input_size, input_size)]), txt
            )

        def _full_ab() -> Any:
            f = det_d.forward_features(x)
            d = decoder(det_d.head(f), (input_size, input_size))[0]
            idx = d.flat_indices if d.flat_indices is not None and d.flat_indices.numel() > 0 else synth_flat
            bxs = d.boxes if d.boxes.numel() > 0 else synth_boxes
            _, pa = ha_d.classify_embeddings(
                ha_d.forward_indexed(f, torch.zeros_like(idx), idx), txt
            )
            _, pb = hb_d.classify_embeddings(
                hb_d(f, [bxs], image_shapes=[(input_size, input_size)]), txt
            )
            return 0.5 * (pa + pb)

        return {
            "detector_only_ms": round(ms_det, 2),
            "head_a_indexed_25props_ms": round(ms_head_a_indexed, 2),
            "head_a_full_dense_ms": round(ms_head_a_dense, 2),
            "head_b_roi_25props_ms": round(ms_head_b_roi, 2),
            "end_to_end_option_a_ms": round(measure(_full_a), 2),
            "end_to_end_option_b_ms": round(measure(_full_b), 2),
            "end_to_end_option_ab_ms": round(measure(_full_ab), 2),
        }

    prev_threads = torch.get_num_threads()
    torch.set_num_threads(4)
    try:
        cpu_timings = _time_on_device("cpu", warmup=4, iters=20)
    finally:
        torch.set_num_threads(prev_threads)

    cuda_timings = (
        _time_on_device("cuda", warmup=10, iters=40)
        if torch.cuda.is_available()
        else {}
    )

    return {
        "parameters_millions": {
            "sota_detector_inference": round(det_infer_params / 1e6, 3),
            "option_a_head": round(params_a / 1e6, 3),
            "option_b_head": round(params_b / 1e6, 3),
            "sota_total_option_a": round((det_infer_params + params_a) / 1e6, 3),
            "sota_total_option_b": round((det_infer_params + params_b) / 1e6, 3),
            "sota_total_option_ab": round(
                (det_infer_params + params_a + params_b) / 1e6, 3
            ),
            "legacy_region_head": round(legacy_params / 1e6, 3),
        },
        "cpu_4thread_latency_ms": cpu_timings,
        "cuda_latency_ms": cuda_timings,
    }


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
        "--gt-index",
        type=Path,
        default=DEFAULT_ENRICHED_GT_INDEX_PATH,
    )
    parser.add_argument(
        "--holdout-manifest",
        type=Path,
        default=Path("artifacts/dashboard_holdout_8k_8k_full.json"),
    )
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("artifacts/sota_open_vocab/benchmark_option_a_vs_b.json"),
    )
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    vocab_37 = VocabularyBank.load(args.vocab_37, device=device)
    vocab_41 = VocabularyBank.load(args.vocab_41, device=device)
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

    legacy_configs = {
        "hbb": {
            "det_ckpt": Path(
                "artifacts/finetune_8k_hbb_8k_full/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"
            ),
            "v3_head": Path(
                "artifacts/distill_8k_region_head_v3/checkpoints/region_head_best.pt"
            ),
            "v3_vocab": Path(
                "artifacts/distill_8k_region_head_v3/vocabulary_bank_37classes.pt"
            ),
            "v4_head": Path(
                "artifacts/distill_8k_region_head_v4/checkpoints/region_head_best.pt"
            ),
            "v4_vocab": Path(
                "artifacts/distill_8k_region_head_v4/vocabulary_bank_37classes.pt"
            ),
        },
        "quad": {
            "det_ckpt": Path(
                "artifacts/finetune_8k_quad_8k_full/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
            ),
            "v3_head": Path(
                "artifacts/distill_8k_region_head_quad_v3/checkpoints/region_head_best.pt"
            ),
            "v3_vocab": Path(
                "artifacts/distill_8k_region_head_quad_v3/vocabulary_bank_37classes.pt"
            ),
            "v4_head": Path(
                "artifacts/distill_8k_region_head_quad_v4/checkpoints/region_head_best.pt"
            ),
            "v4_vocab": Path(
                "artifacts/distill_8k_region_head_quad_v4/vocabulary_bank_37classes.pt"
            ),
        },
    }

    report: dict[str, Any] = {
        "device": str(device),
        "seen_categories_count": len(OPEN_VOCAB_CATEGORIES),
        "zero_shot_unseen_categories": list(ZERO_SHOT_EVAL_CATEGORIES),
        "geometries": {},
    }

    sota_ckpts = {"hbb": args.hbb_sota_ckpt, "quad": args.quad_sota_ckpt}

    for geom, ckpt_path in sota_ckpts.items():
        if not ckpt_path.exists():
            print(f"[Benchmark] Skipping {geom.upper()}: checkpoint not found at {ckpt_path}")
            continue
        print(f"\n{'=' * 72}\nBenchmarking {geom.upper()} Open-Vocabulary Models ({ckpt_path})\n{'=' * 72}")
        detector, head_a, head_b, ckpt_meta = load_sota_bundle(
            ckpt_path,
            geom,
            device,
            distractor_embeddings=distractor_emb,
            visual_centroid=visual_centroid,
        )
        splits = build_split_datasets(
            geom, args.gt_index, args.image_root, args.holdout_manifest
        )

        geom_res: dict[str, Any] = {
            "sota_checkpoint": str(ckpt_path),
            "sota_best_epoch": int(ckpt_meta.get("epoch", -1)),
            "edge_profile": profile_edge_latency_and_params(
                detector=detector,
                head_a=head_a,
                head_b=head_b,
                vocab_37=vocab_37,
                geometry=geom,
                legacy_region_head_ckpt=legacy_configs[geom]["v4_head"],
            ),
            "splits": {},
        }

        for split_name, ds in splits.items():
            print(f"  -> Evaluating SOTA (Option A, Option B, Option A+B) on {split_name} ({len(ds)} images)...")
            sota_m = evaluate_sota_split(
                detector=detector,
                head_a=head_a,
                head_b=head_b,
                dataset=ds,
                vocab_37=vocab_37,
                vocab_41=vocab_41,
                geometry=geom,
                input_size=384,
                device=device,
                batch_size=args.batch_size,
                workers=args.workers,
            )
            split_entry: dict[str, Any] = {"sota": sota_m, "legacy": {}}

            leg_cfg = legacy_configs[geom]
            if leg_cfg["det_ckpt"].exists():
                for tag in ("v3", "v4"):
                    h_path = leg_cfg[f"{tag}_head"]
                    v_path = leg_cfg[f"{tag}_vocab"]
                    if h_path.exists() and v_path.exists():
                        print(f"  -> Evaluating Legacy ({tag}) on {split_name}...")
                        split_entry["legacy"][f"8k_full_region_head_{tag}"] = (
                            evaluate_legacy_split(
                                detector_ckpt_path=leg_cfg["det_ckpt"],
                                region_head_ckpt_path=h_path,
                                vocab_path=v_path,
                                dataset=ds,
                                geometry=geom,
                                input_size=384,
                                device=device,
                                batch_size=args.batch_size,
                                workers=args.workers,
                            )
                        )

            geom_res["splits"][split_name] = split_entry

            loc = sota_m["localization"]
            sem = sota_m["semantics_37_seen"]
            zs = sota_m["zero_shot_4_unseen"]
            print(
                f"     [{geom.upper()} | {split_name}] "
                f"R@100={loc['recall_100_50']*100:.1f}% AP50={loc['class_agnostic_ap50']*100:.1f}% "
                f"Boxes@0.3={loc['boxes_per_img_030']:.1f} OvlpPairs={loc['overlap_pairs_025']:.2f} "
                f"DupGT={loc['dup_gt_rate']*100:.1f}%"
            )
            print(
                f"     Seen-37 Top1: OptA={sem['option_a']['matched_prop_top1']*100:.1f}% "
                f"(mAP50={sem['option_a']['open_vocab_map50']*100:.1f}%) | "
                f"OptB={sem['option_b']['matched_prop_top1']*100:.1f}% "
                f"(Oracle={sem['option_b']['oracle_gt_box_top1']*100:.1f}%, "
                f"mAP50={sem['option_b']['open_vocab_map50']*100:.1f}%) | "
                f"OptAB={sem['option_ab']['matched_prop_top1']*100:.1f}% "
                f"(mAP50={sem['option_ab']['open_vocab_map50']*100:.1f}%)"
            )
            print(
                f"     ZeroShot-4 (4-way / 41-way Top5, n={zs['recalled_unseen_gt']}): "
                f"OptA={zs['option_a']['unseen_4way_top1']*100:.1f}% / {zs['option_a']['full_41way_top5']*100:.1f}% | "
                f"OptB={zs['option_b']['unseen_4way_top1']*100:.1f}% / {zs['option_b']['full_41way_top5']*100:.1f}% | "
                f"OptAB={zs['option_ab']['unseen_4way_top1']*100:.1f}% / {zs['option_ab']['full_41way_top5']*100:.1f}%"
            )
            if split_entry["legacy"]:
                for l_name, l_m in split_entry["legacy"].items():
                    l_loc = l_m["localization"]
                    l_sem = l_m["semantics_37_seen"]
                    print(
                        f"     Legacy [{l_name}]: R@100={l_loc['recall_100_50']*100:.1f}% "
                        f"Boxes@0.3={l_loc['boxes_per_img_030']:.1f} OvlpPairs={l_loc['overlap_pairs_025']:.2f} "
                        f"DupGT={l_loc['dup_gt_rate']*100:.1f}% | "
                        f"Top1={l_sem['matched_prop_top1']*100:.1f}% "
                        f"(Oracle={l_sem['oracle_gt_box_top1']*100:.1f}%, "
                        f"mAP50={l_sem['open_vocab_map50']*100:.1f}%)"
                    )

        ep = geom_res["edge_profile"]
        print(
            f"  -> Edge Latency (4-thread CPU): Det={ep['cpu_4thread_latency_ms']['detector_only_ms']:.2f}ms | "
            f"E2E OptA={ep['cpu_4thread_latency_ms']['end_to_end_option_a_ms']:.2f}ms | "
            f"E2E OptB={ep['cpu_4thread_latency_ms']['end_to_end_option_b_ms']:.2f}ms | "
            f"E2E OptAB={ep['cpu_4thread_latency_ms']['end_to_end_option_ab_ms']:.2f}ms"
        )
        report["geometries"][geom] = geom_res

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2))
    print(f"\nSaved complete head-to-head benchmark report -> {args.output_json}")


if __name__ == "__main__":
    main()
