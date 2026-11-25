#!/usr/bin/env python3
"""Head-to-head benchmark: Original OWLv2-Large teacher vs distilled Student on holdout_210.

Reproduces the `holdout_210` comparison measuring:
  - OWLv2-Large (1008x1008) Core-27 & Full-37 Frozen R@100, Agnostic AP50,
    Matched-Prop Top-1/Top-3, Open-Vocab mAP@50, ZeroShot-4, per-class Top-1, and GPU ms.
  - Student HBB (`owlv2_30k_hbb_v3`) Opt-B on the exact same holdout split & canvas.
  - Direct Distillation Fidelity between Student Opt-B proposals and confident OWLv2
    teacher boxes (`>= --teacher-score-threshold`), plus Oracle-GT 768-D cosine similarity.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import time
from typing import Any

import numpy as np
from PIL import Image
import torch
from torch import Tensor
import torch.nn.functional as F
from torchvision.ops import box_iou, nms
from transformers.image_transforms import center_to_corners_format
from transformers.models.owlv2.image_processing_owlv2 import _scale_boxes

try:
    from scripts.benchmark_open_vocab_options import (
        build_split_datasets,
        compute_average_precision,
        load_sota_bundle,
    )
except ModuleNotFoundError:
    from benchmark_open_vocab_options import (  # type: ignore[no-redef]
        build_split_datasets,
        compute_average_precision,
        load_sota_bundle,
    )
from student_detector.decoder import InferenceDecoder
from student_detector.human_gt_data import DEFAULT_ENRICHED_GT_INDEX_PATH
from student_detector.ontology import (
    CLOSED_MERGE_MAP,
    ZERO_SHOT_EVAL_CATEGORIES,
    map_to_canonical,
    normalize_category_name,
)
from student_detector.owlv2_teacher import (
    OWLV2_MODEL_ID,
    OWLV2_MODEL_REVISION,
    Owlv2TeacherExtractor,
)
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT
from student_detector.targets import point_validity_from_pixel_mask
from student_detector.vocabulary import VocabularyBank


def _init_preset_accumulators() -> dict[str, Any]:
    return {
        "total_gt": 0,
        "rec50": 0,
        "rec75": 0,
        "agn_ap_records": [],
        "prop_total": 0,
        "prop_top1": 0,
        "prop_top3": 0,
        "per_cls_tot": defaultdict(int),
        "per_cls_hit": defaultdict(int),
        "ov_gt_counts": defaultdict(int),
        "ov_ap_records": defaultdict(list),
        "zs_total": 0,
        "zs_4way": 0,
        "zs_full_top5": 0,
    }


def _decode_owlv2_for_bank(
    extractor: Owlv2TeacherExtractor,
    image_feats: Tensor,
    all_boxes_canvas: Tensor,
    all_class_embeds: Tensor,
    bank_emb: Tensor,
    *,
    iou_threshold: float = 0.50,
    pre_nms_top_k: int = 500,
    max_detections: int = 100,
) -> tuple[Tensor, Tensor, Tensor, Tensor, Tensor]:
    """Run OWLv2 `class_predictor` against a normalized bank and NMS-decode top boxes."""
    q_emb = bank_emb.unsqueeze(0).to(device=image_feats.device, dtype=extractor.dtype)
    pred_logits, _ = extractor.model.class_predictor(image_feats, q_emb, query_mask=None)
    probs = torch.sigmoid(pred_logits[0].float())
    max_scores, _ = probs.max(dim=-1)

    bw = all_boxes_canvas[:, 2] - all_boxes_canvas[:, 0]
    bh = all_boxes_canvas[:, 3] - all_boxes_canvas[:, 1]
    valid = (bw >= 2.0) & (bh >= 2.0)
    if not valid.any():
        empty_b = all_boxes_canvas.new_empty((0, 4))
        return (
            empty_b,
            probs.new_empty((0,)),
            probs.new_empty((0, probs.shape[1])),
            all_class_embeds.new_empty((0, all_class_embeds.shape[1])),
            torch.empty((0,), dtype=torch.long, device=all_boxes_canvas.device),
        )

    valid_idx = torch.where(valid)[0]
    cand_count = min(pre_nms_top_k, int(valid_idx.numel()))
    order = torch.argsort(max_scores[valid_idx], descending=True, stable=True)[:cand_count]
    top_idx = valid_idx[order]

    cand_boxes = all_boxes_canvas[top_idx]
    cand_scores = max_scores[top_idx]
    keep = nms(cand_boxes, cand_scores, iou_threshold)[:max_detections]
    kept_idx = top_idx[keep]
    return (
        all_boxes_canvas[kept_idx],
        max_scores[kept_idx],
        probs[kept_idx],
        all_class_embeds[kept_idx],
        kept_idx,
    )


def _accumulate_preset(
    acc: dict[str, Any],
    *,
    det_boxes: Tensor,
    det_scores: Tensor,
    det_probs_seen: Tensor,
    det_probs_zs: Tensor,
    seen_cats: tuple[str, ...],
    zs_cats: tuple[str, ...],
    gt_boxes: Tensor,
    ign_boxes: Tensor,
    gt_raw_names: tuple[str, ...],
) -> None:
    """Update localization, Open-Vocab mAP@50, and ZeroShot-4 metrics for one preset."""
    n_p = int(det_boxes.shape[0])
    n_gt = int(gt_boxes.shape[0])
    acc["total_gt"] += n_gt

    cat_to_idx = {c: i for i, c in enumerate(seen_cats)}
    merge_map = (
        CLOSED_MERGE_MAP
        if ("street_light" not in cat_to_idx and "pole" in cat_to_idx)
        else {}
    )
    zs_to_idx = {c: zs_cats.index(c) for c in ZERO_SHOT_EVAL_CATEGORIES if c in zs_cats}
    zs_start = min(zs_to_idx.values()) if zs_to_idx else len(zs_cats)

    gt_canon = [
        merge_map.get(c, c) if c is not None else None
        for c in (map_to_canonical(raw_c) for raw_c in gt_raw_names)
    ]
    gt_cls_idx = [cat_to_idx[c] if c in cat_to_idx else -1 for c in gt_canon]
    gt_zs_idx = [
        zs_to_idx.get(normalize_category_name(raw_c), -1) if canon is None else -1
        for raw_c, canon in zip(gt_raw_names, gt_canon, strict=True)
    ]
    for c_i in gt_cls_idx:
        if c_i >= 0:
            acc["ov_gt_counts"][c_i] += 1

    iou_mat = (
        box_iou(det_boxes, gt_boxes)
        if (n_p > 0 and n_gt > 0)
        else det_boxes.new_zeros((n_p, n_gt))
    )
    ign_mat = (
        box_iou(det_boxes, ign_boxes)
        if (n_p > 0 and ign_boxes.shape[0] > 0)
        else det_boxes.new_zeros((n_p, 0))
    )

    if n_p > 0 and n_gt > 0:
        best_gt_iou = iou_mat.max(dim=0).values
        acc["rec50"] += int((best_gt_iou >= 0.50).sum().item())
        acc["rec75"] += int((best_gt_iou >= 0.75).sum().item())

    gt_used = torch.zeros(n_gt, dtype=torch.bool, device=det_boxes.device)
    for p_i in range(n_p):
        sc = float(det_scores[p_i].item())
        if n_gt > 0:
            row = iou_mat[p_i].clone()
            row[gt_used] = -1.0
            best_v, best_g = row.max(dim=0)
            if float(best_v.item()) >= 0.50:
                gt_used[best_g] = True
                acc["agn_ap_records"].append((sc, 1))
                continue
        if ign_mat.shape[1] > 0 and float(ign_mat[p_i].max().item()) >= 0.50:
            continue
        acc["agn_ap_records"].append((sc, 0))

    if n_p > 0 and n_gt > 0:
        best_ious, best_p = iou_mat.max(dim=0)
        for g_i in range(n_gt):
            if float(best_ious[g_i].item()) < 0.50:
                continue
            p_idx = int(best_p[g_i].item())
            c_idx = gt_cls_idx[g_i]
            if c_idx >= 0:
                c_name = seen_cats[c_idx]
                acc["prop_total"] += 1
                acc["per_cls_tot"][c_name] += 1
                top3 = det_probs_seen[p_idx].topk(min(3, len(seen_cats))).indices.tolist()
                if top3[0] == c_idx:
                    acc["prop_top1"] += 1
                    acc["per_cls_hit"][c_name] += 1
                if c_idx in top3:
                    acc["prop_top3"] += 1
            zs_idx = gt_zs_idx[g_i]
            if zs_idx >= 0:
                acc["zs_total"] += 1
                sub = int(det_probs_zs[p_idx, zs_start:].argmax().item())
                if zs_start + sub == zs_idx:
                    acc["zs_4way"] += 1
                top5 = det_probs_zs[p_idx].topk(min(5, len(zs_cats))).indices.tolist()
                if zs_idx in top5:
                    acc["zs_full_top5"] += 1

    if n_p > 0:
        gt_used_cls: dict[int, set[int]] = defaultdict(set)
        top_prob, top_c = det_probs_seen.max(dim=-1)
        joint_sc = torch.sqrt((det_scores.clamp(min=0.0) * top_prob.clamp(min=0.0)).clamp(min=0.0))
        order = torch.argsort(joint_sc, descending=True)
        for p_i in order.tolist():
            c_pred = int(top_c[p_i].item())
            sc_pred = float(joint_sc[p_i].item())
            best_g = -1
            best_iou = 0.50
            for g_i in range(n_gt):
                if gt_cls_idx[g_i] != c_pred or g_i in gt_used_cls[c_pred]:
                    continue
                val_iou = float(iou_mat[p_i, g_i].item())
                if val_iou >= best_iou:
                    best_iou = val_iou
                    best_g = g_i
            if best_g >= 0:
                gt_used_cls[c_pred].add(best_g)
                acc["ov_ap_records"][c_pred].append((sc_pred, 1))
                continue
            if ign_mat.shape[1] > 0 and float(ign_mat[p_i].max().item()) >= 0.50:
                continue
            acc["ov_ap_records"][c_pred].append((sc_pred, 0))


def _finalize_preset(acc: dict[str, Any]) -> dict[str, Any]:
    d_gt = max(acc["total_gt"], 1)
    d_prop = max(acc["prop_total"], 1)
    d_zs = max(acc["zs_total"], 1)
    active_classes = [c for c, cnt in acc["ov_gt_counts"].items() if cnt > 0]
    aps = [
        compute_average_precision(acc["ov_ap_records"][c], acc["ov_gt_counts"][c])
        for c in active_classes
    ]
    per_cls_top1 = {
        c: round(acc["per_cls_hit"][c] / max(acc["per_cls_tot"][c], 1), 4)
        for c in sorted(acc["per_cls_tot"])
    }
    return {
        "total_gt": acc["total_gt"],
        "recall_100_50": round(acc["rec50"] / d_gt, 4),
        "recall_100_75": round(acc["rec75"] / d_gt, 4),
        "class_agnostic_ap50": round(
            compute_average_precision(acc["agn_ap_records"], acc["total_gt"]), 4
        ),
        "evaluated_recalled_gt": acc["prop_total"],
        "matched_prop_top1": round(acc["prop_top1"] / d_prop, 4),
        "matched_prop_top3": round(acc["prop_top3"] / d_prop, 4),
        "open_vocab_map50": round(sum(aps) / max(len(aps), 1), 4),
        "zeroshot_count": acc["zs_total"],
        "zeroshot_4way_top1": round(acc["zs_4way"] / d_zs, 4),
        "zeroshot_full_top5": round(acc["zs_full_top5"] / d_zs, 4),
        "per_class_matched_top1": per_cls_top1,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--student-ckpt",
        type=Path,
        default=Path("artifacts/owlv2_30k_hbb_v3/checkpoints/owlv2_hbb_best.pt"),
    )
    parser.add_argument(
        "--student-vocab-core27",
        type=Path,
        default=Path("artifacts/owlv2_30k_hbb_v3/owlv2_adapted_vocab_core27.pt"),
    )
    parser.add_argument(
        "--teacher-vocab-core27",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_core27.pt"),
    )
    parser.add_argument(
        "--teacher-vocab-31",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_31classes_core27_zeroshot.pt"),
    )
    parser.add_argument(
        "--teacher-vocab-37",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_37classes.pt"),
    )
    parser.add_argument(
        "--teacher-vocab-41",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_41classes_zeroshot.pt"),
    )
    parser.add_argument(
        "--vocab-distractors",
        type=Path,
        default=Path("artifacts/sota_open_vocab/owlv2_vocabulary_bank_distractors.pt"),
    )
    parser.add_argument("--gt-index", type=Path, default=DEFAULT_ENRICHED_GT_INDEX_PATH)
    parser.add_argument("--image-root", type=Path, default=DEFAULT_IMAGE_ROOT)
    parser.add_argument(
        "--holdout-manifest",
        type=Path,
        default=Path("artifacts/dashboard_holdout_8k_8k_full.json"),
    )
    parser.add_argument("--teacher-score-threshold", type=float, default=0.28)
    parser.add_argument("--small-box-score-power", type=float, default=1.25)
    parser.add_argument("--small-box-min-side", type=float, default=16.0)
    parser.add_argument("--decoder-top-k", type=int, default=1000)
    parser.add_argument("--decoder-max-detections", type=int, default=100)
    parser.add_argument("--max-images", type=int, default=None)
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("artifacts/owlv2_30k_hbb_v3/benchmark_teacher_vs_student_holdout210.json"),
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
    splits = build_split_datasets(
        "hbb",
        args.gt_index,
        args.image_root,
        args.holdout_manifest,
        only_holdout=True,
    )
    if "holdout_210" not in splits:
        raise RuntimeError(f"Holdout split unavailable from {args.holdout_manifest}")
    holdout_ds = splits["holdout_210"]

    t_v27 = VocabularyBank.load(args.teacher_vocab_core27, device=device)
    t_v31 = VocabularyBank.load(args.teacher_vocab_31, device=device)
    t_v37 = VocabularyBank.load(args.teacher_vocab_37, device=device)
    t_v41 = VocabularyBank.load(args.teacher_vocab_41, device=device)
    s_v27 = VocabularyBank.load(args.student_vocab_core27, device=device)
    dist_emb = (
        VocabularyBank.load(args.vocab_distractors, device=device).embeddings
        if args.vocab_distractors.exists()
        else None
    )

    print(f"Loading OWLv2-Large ({OWLV2_MODEL_ID}@{OWLV2_MODEL_REVISION[:8]}) & Student {args.student_ckpt}...")
    extractor = Owlv2TeacherExtractor(device=device, revision=OWLV2_MODEL_REVISION)
    teacher_params_m = round(sum(p.numel() for p in extractor.model.parameters()) / 1e6, 2)
    detector, _, head_b, _ = load_sota_bundle(
        args.student_ckpt,
        "hbb",
        device,
        distractor_embeddings=dist_emb,
    )
    student_optb_params_m = round(
        (
            sum(p.numel() for n, p in detector.named_parameters() if not n.startswith("aux_head."))
            + sum(p.numel() for p in head_b.parameters())
        )
        / 1e6,
        2,
    )

    strides = (4, 8, 16, 32)
    decoder = InferenceDecoder(
        strides=strides,
        top_k=args.decoder_top_k,
        nms_iou_threshold=0.50,
        max_detections=args.decoder_max_detections,
        score_mode="objectness",
        nms_mode="containment_aware",
        score_threshold=0.01,
        small_box_min_side=args.small_box_min_side,
        small_box_score_power=args.small_box_score_power,
    )

    t_acc_27 = _init_preset_accumulators()
    t_acc_37 = _init_preset_accumulators()
    s_acc_27 = _init_preset_accumulators()

    teacher_conf_total = 0
    student_recalled_teacher = 0
    matched_agree_top1 = 0
    matched_agree_top3 = 0
    matched_cos_sims: list[float] = []
    oracle_cos_sims: list[float] = []
    teacher_lat_ms: list[float] = []
    student_lat_ms: list[float] = []

    limit = len(holdout_ds) if args.max_images is None else min(len(holdout_ds), args.max_images)
    shapes = ((96, 96), (48, 48), (24, 24), (12, 12))

    with torch.no_grad():
        for idx in range(limit):
            sample = holdout_ds[idx]
            img_path = args.image_root / str(sample.file_name)
            with Image.open(img_path) as raw_im:
                rgb = raw_im.convert("RGB")
                orig_w, orig_h = rgb.size
            pv = extractor.processor.image_processor(images=rgb, return_tensors="pt")["pixel_values"]
            pv_dev = pv.to(device=device, dtype=extractor.dtype)

            if device.type == "cuda":
                torch.cuda.synchronize()
            t0 = time.perf_counter()
            fmap, _ = extractor.model.image_embedder(pixel_values=pv_dev)
            image_feats = fmap.reshape(1, -1, fmap.shape[-1])
            _, raw_ce = extractor.model.class_predictor(image_feats, None, None)
            all_ce = F.normalize(raw_ce[0].float(), dim=-1)
            pred_boxes = extractor.model.box_predictor(image_feats, fmap)
            target_t = torch.tensor([[orig_h, orig_w]], device=device, dtype=torch.float32)
            native_boxes = _scale_boxes(
                center_to_corners_format(pred_boxes.float()), target_t
            )[0]
            native_boxes[:, 0::2].clamp_(0.0, float(orig_w))
            native_boxes[:, 1::2].clamp_(0.0, float(orig_h))

            scale, off_x, off_y = sample.transform
            all_canvas = native_boxes.clone()
            all_canvas[:, 0::2] = (all_canvas[:, 0::2] * scale + off_x).clamp(0.0, 384.0)
            all_canvas[:, 1::2] = (all_canvas[:, 1::2] * scale + off_y).clamp(0.0, 384.0)

            tb_27, ts_27, tp_27, te_27, kidx_27 = _decode_owlv2_for_bank(
                extractor, image_feats, all_canvas, all_ce, t_v27.embeddings
            )
            if device.type == "cuda":
                torch.cuda.synchronize()
            t_ms = (time.perf_counter() - t0) * 1000.0
            if idx >= 2 or limit <= 2:
                teacher_lat_ms.append(t_ms)

            tb_37, ts_37, tp_37, _, kidx_37 = _decode_owlv2_for_bank(
                extractor, image_feats, all_canvas, all_ce, t_v37.embeddings
            )
            # Re-score Core-27 & Full-37 boxes against 31/41 banks for aligned ZeroShot-4 lookup.
            if kidx_27.numel() > 0:
                log_31, _ = extractor.model.class_predictor(
                    image_feats, t_v31.embeddings.unsqueeze(0).to(dtype=extractor.dtype)
                )
                tp_31_matched = torch.sigmoid(log_31[0, kidx_27].float())
            else:
                tp_31_matched = tp_27.new_empty((0, len(t_v31.categories)))
            if kidx_37.numel() > 0:
                log_41, _ = extractor.model.class_predictor(
                    image_feats, t_v41.embeddings.unsqueeze(0).to(dtype=extractor.dtype)
                )
                tp_41_matched = torch.sigmoid(log_41[0, kidx_37].float())
            else:
                tp_41_matched = tp_37.new_empty((0, len(t_v41.categories)))

            gt_b = sample.boxes.to(device)
            ign_b = sample.ignore_boxes.to(device)
            _accumulate_preset(
                t_acc_27,
                det_boxes=tb_27,
                det_scores=ts_27,
                det_probs_seen=tp_27,
                det_probs_zs=tp_31_matched,
                seen_cats=tuple(t_v27.categories),
                zs_cats=tuple(t_v31.categories),
                gt_boxes=gt_b,
                ign_boxes=ign_b,
                gt_raw_names=sample.category_names,
            )
            _accumulate_preset(
                t_acc_37,
                det_boxes=tb_37,
                det_scores=ts_37,
                det_probs_seen=tp_37,
                det_probs_zs=tp_41_matched,
                seen_cats=tuple(t_v37.categories),
                zs_cats=tuple(t_v41.categories),
                gt_boxes=gt_b,
                ign_boxes=ign_b,
                gt_raw_names=sample.category_names,
            )

            # Student HBB Opt-B forward pass
            img_dev = sample.image.unsqueeze(0).to(device)
            _, lvl_masks = point_validity_from_pixel_mask(
                sample.valid_mask.to(device), shapes, strides
            )
            batched_masks = tuple(m.unsqueeze(0) for m in lvl_masks)
            if device.type == "cuda":
                torch.cuda.synchronize()
            s0 = time.perf_counter()
            s_feats = detector.forward_features(img_dev)
            s_out = detector.forward_eval_head(s_feats)
            s_det = decoder(s_out, (384, 384), batched_masks)[0]  # type: ignore[arg-type]
            z_b = head_b(s_feats, [s_det.boxes], image_shapes=[(384, 384)])
            _, sb_27 = head_b.classify_embeddings(z_b, s_v27.embeddings)
            if device.type == "cuda":
                torch.cuda.synchronize()
            s_ms = (time.perf_counter() - s0) * 1000.0
            if idx >= 2 or limit <= 2:
                student_lat_ms.append(s_ms)
            _, sb_31 = head_b.classify_embeddings(z_b, t_v31.embeddings)

            _accumulate_preset(
                s_acc_27,
                det_boxes=s_det.boxes,
                det_scores=s_det.scores,
                det_probs_seen=sb_27,
                det_probs_zs=sb_31,
                seen_cats=tuple(s_v27.categories),
                zs_cats=tuple(t_v31.categories),
                gt_boxes=gt_b,
                ign_boxes=ign_b,
                gt_raw_names=sample.category_names,
            )

            # Direct Distillation Fidelity vs confident OWLv2 boxes (>= threshold)
            conf_m = ts_27 >= args.teacher_score_threshold
            t_cb = tb_27[conf_m]
            t_cp = tp_27[conf_m]
            t_ce = te_27[conf_m]
            teacher_conf_total += int(t_cb.shape[0])
            if t_cb.shape[0] > 0 and s_det.boxes.shape[0] > 0:
                ts_iou = box_iou(t_cb, s_det.boxes)
                best_ious, best_s = ts_iou.max(dim=1)
                for ti in range(int(t_cb.shape[0])):
                    if float(best_ious[ti].item()) < 0.50:
                        continue
                    si = int(best_s[ti].item())
                    student_recalled_teacher += 1
                    t_top1 = int(t_cp[ti].argmax().item())
                    s_top3 = sb_27[si].topk(3).indices.tolist()
                    if s_top3[0] == t_top1:
                        matched_agree_top1 += 1
                    if t_top1 in s_top3:
                        matched_agree_top3 += 1
                    matched_cos_sims.append(float((z_b[si] * t_ce[ti]).sum().item()))

            if gt_b.shape[0] > 0:
                z_b_gt = head_b(s_feats, [gt_b], image_shapes=[(384, 384)])
                best_patch = box_iou(gt_b, all_canvas).argmax(dim=1)
                t_gt_emb = all_ce[best_patch]
                cos_vals = (z_b_gt * t_gt_emb).sum(dim=-1).tolist()
                oracle_cos_sims.extend(float(v) for v in cos_vals)

    report = {
        "split": "holdout_210",
        "evaluated_images": limit,
        "teacher": {
            "model_id": OWLV2_MODEL_ID,
            "revision": OWLV2_MODEL_REVISION,
            "input_size": 1008,
            "params_millions": teacher_params_m,
            "cuda_ms_bsz1": round(float(np.mean(teacher_lat_ms)), 2) if teacher_lat_ms else 0.0,
            "core27_frozen": _finalize_preset(t_acc_27),
            "full37_frozen": _finalize_preset(t_acc_37),
        },
        "student_hbb": {
            "checkpoint": str(args.student_ckpt),
            "input_size": 384,
            "params_millions_opt_b": student_optb_params_m,
            "cuda_ms_bsz1_opt_b": round(float(np.mean(student_lat_ms)), 2) if student_lat_ms else 0.0,
            "core27_adapted": _finalize_preset(s_acc_27),
        },
        "distillation_fidelity": {
            "teacher_score_threshold": args.teacher_score_threshold,
            "teacher_confident_boxes": teacher_conf_total,
            "student_matched_boxes": student_recalled_teacher,
            "student_recall_of_teacher_50": round(
                student_recalled_teacher / max(teacher_conf_total, 1), 4
            ),
            "matched_label_agreement_top1": round(
                matched_agree_top1 / max(student_recalled_teacher, 1), 4
            ),
            "matched_label_agreement_top3": round(
                matched_agree_top3 / max(student_recalled_teacher, 1), 4
            ),
            "matched_embedding_cosine_mean": round(float(np.mean(matched_cos_sims)), 4)
            if matched_cos_sims
            else 0.0,
            "matched_embedding_cosine_median": round(float(np.median(matched_cos_sims)), 4)
            if matched_cos_sims
            else 0.0,
            "oracle_gt_count": len(oracle_cos_sims),
            "oracle_gt_embedding_cosine_mean": round(float(np.mean(oracle_cos_sims)), 4)
            if oracle_cos_sims
            else 0.0,
            "oracle_gt_embedding_cosine_median": round(float(np.median(oracle_cos_sims)), 4)
            if oracle_cos_sims
            else 0.0,
        },
    }

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    t27 = report["teacher"]["core27_frozen"]
    s27 = report["student_hbb"]["core27_adapted"]
    df = report["distillation_fidelity"]
    print(
        f"[OWLv2 Teacher vs Student on {limit} imgs]\n"
        f"  Teacher (1008x1008, {teacher_params_m}M, {report['teacher']['cuda_ms_bsz1']}ms): "
        f"R@100={t27['recall_100_50']*100:.1f}% AP50={t27['class_agnostic_ap50']*100:.1f}% "
        f"Top1={t27['matched_prop_top1']*100:.1f}% mAP50={t27['open_vocab_map50']*100:.1f}%\n"
        f"  Student (384x384, {student_optb_params_m}M, {report['student_hbb']['cuda_ms_bsz1_opt_b']}ms): "
        f"R@100={s27['recall_100_50']*100:.1f}% AP50={s27['class_agnostic_ap50']*100:.1f}% "
        f"Top1={s27['matched_prop_top1']*100:.1f}% mAP50={s27['open_vocab_map50']*100:.1f}%\n"
        f"  Distillation Fidelity: Recall@50={df['student_recall_of_teacher_50']*100:.1f}% "
        f"Agree={df['matched_label_agreement_top1']*100:.1f}%/{df['matched_label_agreement_top3']*100:.1f}% "
        f"CosMean={df['matched_embedding_cosine_mean']:.4f} (Oracle={df['oracle_gt_embedding_cosine_mean']:.4f})\n"
        f"Saved report -> {args.output_json}",
        flush=True,
    )


if __name__ == "__main__":
    main()
