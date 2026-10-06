"""Executable pipeline for Phase 4.0: Experiment and Teacher Baselines."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import time
from typing import Any

from PIL import Image
import torch
from torch.utils.data import DataLoader, Subset

from student_detector.checkpoints import load_checkpoint, load_model_state_strict
from student_detector.config import load_phase3_config
from student_detector.crop_extractor import CropPolicy, extract_object_crop
from student_detector.data import select_source_mixture_indices
from student_detector.head import QuadDetectorOutput
from student_detector.model import QuadProposalDetector
from student_detector.ontology import (
    NOVEL_CATEGORIES,
    UNSUPPORTED_CATEGORIES,
    is_compatible,
)
from student_detector.proposal_contract import SourceTransform
from student_detector.proposal_evaluation import valid_output_levels
from student_detector.quad_data import QuadProposalDataset, collate_quad_proposal_samples
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou
from student_detector.semantic_evaluation_data import (
    SemanticImageRecord,
    load_semantic_image_records,
    select_stratified_semantic_cohort,
)
from student_detector.siglip import SiglipTeacher


def _resolve_project_path(p: Path | str) -> Path:
    path = Path(p)
    if path.is_file() or path.is_dir():
        return path.resolve()
    for prefix in (
        Path.cwd(),
        Path("/home/sashamikoff/git/skillup/projects/visual-inference"),
        Path("/home/sashamikoff/git/skillup"),
    ):
        candidate = prefix / path
        if candidate.is_file() or candidate.is_dir():
            return candidate.resolve()
    return path.resolve()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path("configs/benchmarks/phase4_0_experiment_contract.json"),
        help="Path to phase4_0_experiment_contract.json",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/correction_phase4_quad_p3_local_rtx3060_step20000_v1.yaml"),
        help="Path to base quad detector config",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/home/sashamikoff/git/skillup/data/visual-inference-datasets/output"),
        help="Root path to visual inference dataset",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/phase4/baselines"),
        help="Output directory for reports",
    )
    parser.add_argument(
        "--num-images",
        type=int,
        default=800,
        help="Number of images to evaluate",
    )
    parser.add_argument(
        "--stratified",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Include stratified rare-class images in the evaluation cohort",
    )
    parser.add_argument(
        "--proposals-per-image",
        type=int,
        default=25,
        help="Max proposals per image to pass through SigLIP teacher",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cpu",
        help="Device to run inference on (cpu or cuda)",
    )
    return parser.parse_args()


def compute_ap(recalls: list[float], precisions: list[float]) -> float:
    """Compute average precision using 11-point interpolation."""
    if not recalls or not precisions:
        return 0.0
    rec = [0.0] + recalls + [1.0]
    prec = [0.0] + precisions + [0.0]
    for i in range(len(prec) - 2, -1, -1):
        prec[i] = max(prec[i], prec[i + 1])
    ap = 0.0
    for t in [i / 10.0 for i in range(11)]:
        p = max([p for r, p in zip(rec, prec, strict=True) if r >= t] or [0.0])
        ap += p / 11.0
    return ap


def run_gt_crops_baseline(
    teacher: SiglipTeacher,
    image_records: list[SemanticImageRecord],
    crop_policy: CropPolicy,
    categories: list[str],
    text_bank: torch.Tensor,
) -> dict[str, Any]:
    """Evaluate SigLIP recognition directly on ground-truth object crops."""
    print(f"--- Running GT Crops Baseline (policy: {crop_policy.mode}, margin={crop_policy.context_margin}) ---", flush=True)
    t0 = time.time()

    per_class_total: Counter[str] = Counter()
    per_class_top1: Counter[str] = Counter()
    per_class_top3: Counter[str] = Counter()
    per_size_total: Counter[str] = Counter()
    per_size_top1: Counter[str] = Counter()
    failure_gallery: list[dict[str, Any]] = []

    crops_batch: list[Image.Image] = []
    metadata_batch: list[tuple[str, str, str, int, tuple[float, float, float, float]]] = []

    for img_rec in image_records:
        if not img_rec.image_path.is_file():
            continue
        try:
            pil_img = Image.open(img_rec.image_path).convert("RGB")
        except Exception:
            continue

        for gt in img_rec.positive_ground_truths:
            canonical = gt.canonical_category
            if not canonical or canonical not in categories:
                continue

            extracted = extract_object_crop(pil_img, gt.quad, crop_policy)
            if not extracted.is_valid:
                continue

            crops_batch.append(extracted.image)
            metadata_batch.append((canonical, gt.size_bin, img_rec.file_name, gt.annotation_id, gt.bbox))

    print(f"Extracted {len(crops_batch)} valid GT crops across {len(image_records)} images. Encoding with SigLIP...", flush=True)

    if crops_batch:
        img_embeddings = teacher.encode_images(crops_batch, batch_size=32)
        logits, probs = teacher.compute_similarity(img_embeddings, text_bank)

        for i, (canonical, size_bin, file_name, ann_id, bbox) in enumerate(metadata_batch):
            class_probs = probs[i]
            sorted_indices = torch.argsort(class_probs, descending=True).tolist()
            top1_cat = categories[sorted_indices[0]]
            top3_cats = [categories[idx] for idx in sorted_indices[:3]]

            per_class_total[canonical] += 1
            per_size_total[size_bin] += 1

            is_top1 = is_compatible(top1_cat, canonical)
            is_top3 = any(is_compatible(c, canonical) for c in top3_cats)

            if is_top1:
                per_class_top1[canonical] += 1
                per_size_top1[size_bin] += 1
            else:
                if len(failure_gallery) < 25:
                    failure_gallery.append({
                        "annotation_id": ann_id,
                        "file_name": file_name,
                        "ground_truth": canonical,
                        "predicted_top1": top1_cat,
                        "top3": top3_cats,
                        "confidence": float(class_probs[sorted_indices[0]].item()),
                        "size_bin": size_bin,
                        "bbox": list(bbox),
                    })

            if is_top3:
                per_class_top3[canonical] += 1

    elapsed = time.time() - t0
    class_results: dict[str, Any] = {}
    for cat in sorted(set(list(per_class_total.keys()) + categories)):
        tot = per_class_total[cat]
        t1 = per_class_top1[cat]
        t3 = per_class_top3[cat]
        class_results[cat] = {
            "instances": tot,
            "top1_accuracy": round(t1 / tot, 4) if tot > 0 else 0.0,
            "top3_accuracy": round(t3 / tot, 4) if tot > 0 else 0.0,
        }

    size_results: dict[str, Any] = {}
    for s_bin in ("tiny", "small", "medium", "large"):
        tot = per_size_total[s_bin]
        t1 = per_size_top1[s_bin]
        size_results[s_bin] = {
            "instances": tot,
            "top1_accuracy": round(t1 / tot, 4) if tot > 0 else 0.0,
        }

    overall_tot = sum(per_class_total.values())
    overall_t1 = sum(per_class_top1.values())

    return {
        "crop_policy": {
            "mode": crop_policy.mode,
            "context_margin": crop_policy.context_margin,
            "target_size": list(crop_policy.target_size),
        },
        "elapsed_seconds": round(elapsed, 2),
        "total_evaluated_crops": overall_tot,
        "overall_top1_accuracy": round(overall_t1 / overall_tot, 4) if overall_tot > 0 else 0.0,
        "per_class": class_results,
        "per_size": size_results,
        "failure_gallery": failure_gallery,
    }


def run_proposal_baseline(
    detector: QuadProposalDetector,
    teacher: SiglipTeacher,
    image_records: list[SemanticImageRecord],
    dataset: QuadProposalDataset,
    selected_indices: list[int],
    categories: list[str],
    text_bank: torch.Tensor,
    crop_policy: CropPolicy,
    max_proposals_per_image: int,
    nms_threshold: float,
    score_threshold: float,
    device: torch.device,
) -> dict[str, Any]:
    """Evaluate detector proposals and subsequent SigLIP classification."""
    print(f"--- Running Actual Proposals SigLIP Baseline (NMS: {nms_threshold}, Score: {score_threshold}) ---", flush=True)
    t0 = time.time()

    decoder = QuadInferenceDecoder(
        strides=(8, 16, 32),
        pre_nms_top_k=300,
        nms_iou_threshold=nms_threshold,
        max_proposals=100,
    )

    subset = Subset(dataset, selected_indices)
    loader = DataLoader(
        subset,
        batch_size=8,
        shuffle=False,
        num_workers=0,
        collate_fn=collate_quad_proposal_samples,
    )

    image_rec_by_id = {rec.image_id: rec for rec in image_records}

    # Tracking per-class GT recall
    gt_total_per_class: Counter[str] = Counter()
    gt_recalled_050_per_class: Counter[str] = Counter()
    gt_recalled_075_per_class: Counter[str] = Counter()
    gt_total_per_size: Counter[str] = Counter()
    gt_recalled_050_per_size: Counter[str] = Counter()

    # Tracking proposals and end-to-end detection
    class_detections: dict[str, list[tuple[float, bool]]] = defaultdict(list)
    conditional_correct = 0
    conditional_total = 0

    seen_images = 0
    with torch.no_grad():
        for images, samples in loader:
            images_dev = images.to(device)
            out = detector(images_dev)
            out = QuadDetectorOutput(
                quality=tuple(item.float() for item in out.quality),
                corner_offsets=tuple(item.float() for item in out.corner_offsets),
            )
            masks = tuple(
                zip(
                    *(
                        valid_output_levels(
                            sample.valid_mask,
                            out.quality,
                            (8, 16, 32),
                            device,
                        )
                        for sample in samples
                    )
                )
            )
            valid_masks = tuple(torch.cat(lvl, dim=0) for lvl in masks)
            detections = decoder(
                out,
                (images.shape[-2], images.shape[-1]),
                valid_masks,
            )

            for sample, det in zip(samples, detections, strict=True):
                img_rec = image_rec_by_id.get(sample.image_id)
                if not img_rec or not img_rec.image_path.is_file():
                    continue

                source_transform = SourceTransform(
                    scale=sample.transform[0],
                    offset_x=sample.transform[1],
                    offset_y=sample.transform[2],
                    source_size=sample.original_size,
                    model_size=(384, 384),
                )
                source_quads = source_transform.model_to_source(det.quads)
                scores = det.scores.cpu()

                # Tally GT recall for this image
                pos_gts = [gt for gt in img_rec.positive_ground_truths if gt.canonical_category in categories]
                gt_matched_050 = set()

                if pos_gts and source_quads.shape[0] > 0:
                    gt_tensor = torch.tensor(
                        [gt.quad for gt in pos_gts], dtype=torch.float32
                    )
                    ious = pairwise_quad_iou(gt_tensor, source_quads.cpu())  # [N_gt, N_prop]

                    for gt_idx, gt in enumerate(pos_gts):
                        cat = str(gt.canonical_category)
                        gt_total_per_class[cat] += 1
                        gt_total_per_size[gt.size_bin] += 1

                        max_iou = float(ious[gt_idx].max().item())
                        if max_iou >= 0.50:
                            gt_recalled_050_per_class[cat] += 1
                            gt_recalled_050_per_size[gt.size_bin] += 1
                            gt_matched_050.add(gt_idx)
                        if max_iou >= 0.75:
                            gt_recalled_075_per_class[cat] += 1
                else:
                    for gt in pos_gts:
                        gt_total_per_class[str(gt.canonical_category)] += 1
                        gt_total_per_size[gt.size_bin] += 1

                # Extract and classify proposal crops
                num_eval_proposals = min(source_quads.shape[0], max_proposals_per_image)
                if num_eval_proposals > 0:
                    try:
                        pil_img = Image.open(img_rec.image_path).convert("RGB")
                    except Exception:
                        continue

                    prop_crops = []
                    prop_quads_list = []
                    for p_idx in range(num_eval_proposals):
                        p_quad = source_quads[p_idx].tolist()
                        crop_res = extract_object_crop(pil_img, p_quad, crop_policy)
                        if crop_res.is_valid:
                            prop_crops.append(crop_res.image)
                            prop_quads_list.append((p_idx, p_quad))

                    if prop_crops:
                        p_embeds = teacher.encode_images(prop_crops, batch_size=32)
                        p_logits, p_probs = teacher.compute_similarity(p_embeds, text_bank)

                        for crop_i, (p_idx, p_quad) in enumerate(prop_quads_list):
                            p_score = float(scores[p_idx].item())
                            cat_probs = p_probs[crop_i]

                            # Rank score S_ic = q_i * sim_ic
                            top_prob, top_cat_idx = cat_probs.max(dim=-1)
                            top_cat = categories[top_cat_idx.item()]
                            combined_score = p_score * float(top_prob.item())

                            # Check if proposal localized any GT
                            is_tp = False
                            if pos_gts:
                                p_quad_t = torch.tensor([p_quad], dtype=torch.float32)
                                p_ious = pairwise_quad_iou(torch.tensor([gt.quad for gt in pos_gts], dtype=torch.float32), p_quad_t).squeeze(-1)
                                best_gt_iou, best_gt_idx = p_ious.max(dim=0)
                                if float(best_gt_iou.item()) >= 0.50:
                                    best_gt_cat = str(pos_gts[best_gt_idx.item()].canonical_category)
                                    conditional_total += 1
                                    if is_compatible(top_cat, best_gt_cat):
                                        conditional_correct += 1
                                        is_tp = True

                            class_detections[top_cat].append((combined_score, is_tp))

                seen_images += 1
                if seen_images % 100 == 0:
                    print(f"Processed proposals for {seen_images}/{len(selected_indices)} images...", flush=True)

    elapsed = time.time() - t0
    print(f"Completed proposal evaluation in {elapsed:.1f}s", flush=True)

    # Compute recall metrics
    recall_results: dict[str, Any] = {}
    for cat in sorted(set(list(gt_total_per_class.keys()) + categories)):
        tot = gt_total_per_class[cat]
        r50 = gt_recalled_050_per_class[cat]
        r75 = gt_recalled_075_per_class[cat]
        recall_results[cat] = {
            "instances": tot,
            "recall_iou050": round(r50 / tot, 4) if tot > 0 else 0.0,
            "recall_iou075": round(r75 / tot, 4) if tot > 0 else 0.0,
        }

    size_recall: dict[str, Any] = {}
    for s_bin in ("tiny", "small", "medium", "large"):
        tot = gt_total_per_size[s_bin]
        r50 = gt_recalled_050_per_size[s_bin]
        size_recall[s_bin] = {
            "instances": tot,
            "recall_iou050": round(r50 / tot, 4) if tot > 0 else 0.0,
        }

    # Compute per-class AP50
    class_ap50: dict[str, float] = {}
    for cat in categories:
        dets = class_detections.get(cat, [])
        dets.sort(key=lambda x: x[0], reverse=True)
        tot_gt = gt_total_per_class[cat]
        if tot_gt == 0 or not dets:
            class_ap50[cat] = 0.0
            continue
        tp_cum = 0
        recalls = []
        precisions = []
        for rank, (score, is_tp) in enumerate(dets, start=1):
            if is_tp:
                tp_cum += 1
            recalls.append(tp_cum / tot_gt)
            precisions.append(tp_cum / rank)
        class_ap50[cat] = round(compute_ap(recalls, precisions), 4)

    cond_acc = round(conditional_correct / conditional_total, 4) if conditional_total > 0 else 0.0

    return {
        "elapsed_seconds": round(elapsed, 2),
        "evaluated_images": seen_images,
        "proposal_recall_per_class": recall_results,
        "proposal_recall_per_size": size_recall,
        "conditional_recognition_accuracy": cond_acc,
        "conditional_samples": conditional_total,
        "end_to_end_ap50": class_ap50,
        "mean_ap50": round(sum(class_ap50.values()) / max(len(class_ap50), 1), 4),
    }


def make_readiness_decision(
    category: str,
    gt_instances: int,
    top1_acc: float,
    recall050: float,
    is_base: bool,
    criteria: dict[str, Any],
) -> dict[str, Any]:
    """Determine per-class readiness gate decision for Phase 4 distillation."""
    min_instances = int(criteria.get("min_evidence_instances", 15))

    if category in UNSUPPORTED_CATEGORIES:
        return {
            "status": "UNSUPPORTED",
            "reason": "Missing extent annotations / occupancy semantics not established",
        }

    if gt_instances < min_instances:
        return {
            "status": "SCARCE_EVIDENCE",
            "reason": f"Only {gt_instances} instances available (minimum required: {min_instances})",
        }

    rule = criteria.get("base_class" if is_base else "novel_class", {})
    min_acc = float(rule.get("min_teacher_top1_accuracy", 0.60))
    min_rec = float(rule.get("min_proposal_recall_iou050", 0.40))

    if top1_acc < min_acc:
        return {
            "status": "BLOCKED_RECOGNITION",
            "reason": f"Teacher Top-1 accuracy {top1_acc:.2%} below threshold {min_acc:.2%}",
        }
    if recall050 < min_rec:
        return {
            "status": "BLOCKED_RECALL",
            "reason": f"Detector Proposal Recall@0.50 {recall050:.2%} below threshold {min_rec:.2%}",
        }

    return {
        "status": "READY",
        "reason": "Meets both teacher recognition and proposal coverage criteria",
    }


def main() -> None:
    args = _parse_args()
    contract_path = _resolve_project_path(args.contract)
    with contract_path.open("r", encoding="utf-8") as f:
        contract = json.load(f)

    device = torch.device(args.device if (args.device == "cuda" and torch.cuda.is_available()) else "cpu")
    print(f"Executing Phase 4.0 baselines on device: {device}")

    dataset_root = _resolve_project_path(args.dataset_root)
    output_dir = _resolve_project_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Verify Checkpoints
    primary_meta = contract["detector"]["primary"]
    control_meta = contract["detector"]["control"]
    primary_ckpt = _resolve_project_path(primary_meta["checkpoint"])
    control_ckpt = _resolve_project_path(control_meta["checkpoint"])

    with primary_ckpt.open("rb") as f:
        primary_hash = hashlib.sha256(f.read()).hexdigest()
    assert primary_hash == primary_meta["checkpoint_sha256"], "Primary checkpoint SHA mismatch!"
    print(f"Verified primary detector checkpoint: {primary_meta['name']} (SHA-256 match)")

    with control_ckpt.open("rb") as f:
        control_hash = hashlib.sha256(f.read()).hexdigest()
    assert control_hash == control_meta["checkpoint_sha256"], "Control checkpoint SHA mismatch!"
    print(f"Verified control detector checkpoint: {control_meta['name']} (SHA-256 match)")

    # 2. Setup Dataset & Cohort
    config = load_phase3_config(args.config.resolve())
    dataset = QuadProposalDataset(
        dataset_root / "annotations/proposals_val.json",
        dataset_root,
        dataset_root / "indexes/quad_val.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=False,
    )

    # Selected frozen 800 cohort
    frozen_800_indices = select_source_mixture_indices(
        dataset.records, config.data.source_weights, args.num_images
    )
    frozen_identity = json.dumps(
        [(dataset.records[i].source_dataset, dataset.records[i].image_id) for i in frozen_800_indices],
        separators=(",", ":"),
    ).encode()
    computed_subset_sha = hashlib.sha256(frozen_identity).hexdigest()
    print(f"Cohort validation subset SHA: {computed_subset_sha} (Matches target: {computed_subset_sha == contract['dataset']['frozen_800_subset_sha256']})")

    # Add stratified rare class images if requested
    target_eval_cats = list(contract["vocabulary"]["base_classes"]) + list(contract["vocabulary"]["novel_classes"])
    if args.stratified:
        stratified_image_ids = select_stratified_semantic_cohort(
            dataset_root,
            target_categories=target_eval_cats,
            quota_per_category=30,
        )
        record_map = {rec.image_id: idx for idx, rec in enumerate(dataset.records)}
        stratified_indices = [record_map[img_id] for img_id in stratified_image_ids if img_id in record_map]
        eval_indices = sorted(set(frozen_800_indices + stratified_indices))
    else:
        eval_indices = frozen_800_indices

    cohort_image_ids = [dataset.records[i].image_id for i in eval_indices]
    semantic_records = load_semantic_image_records(dataset_root, cohort_image_ids)
    print(f"Loaded {len(semantic_records)} semantic image records for evaluation.")

    # 3. Setup SigLIP Teacher and Text Bank
    teacher = SiglipTeacher(
        model_name=contract["teacher"]["model_name"],
        revision=contract["teacher"]["revision"],
        device=device,
    )
    eval_categories = target_eval_cats + list(contract["vocabulary"]["distractors"])
    text_bank, text_bank_meta = teacher.build_text_bank(
        eval_categories,
        templates=contract["teacher"]["default_prompts"],
    )
    print(f"Built unit-normalized text bank for {len(eval_categories)} categories: {text_bank.shape}")

    # 4. Run GT Crop Baselines Across Crop Policies
    policies = {
        "tight_source": CropPolicy(mode="axis_aligned", context_margin=0.0, use_original_resolution=True),
        "contextual_source": CropPolicy(mode="axis_aligned", context_margin=0.15, use_original_resolution=True),
        "contextual_resized": CropPolicy(mode="axis_aligned", context_margin=0.15, use_original_resolution=False),
    }

    gt_results_by_policy: dict[str, Any] = {}
    for pol_name, pol in policies.items():
        gt_results = run_gt_crops_baseline(
            teacher=teacher,
            image_records=semantic_records,
            crop_policy=pol,
            categories=eval_categories,
            text_bank=text_bank,
        )
        gt_results_by_policy[pol_name] = gt_results
        print(f"Policy '{pol_name}' Top-1 Accuracy: {gt_results['overall_top1_accuracy']:.2%}")

    # Select winning crop policy (highest top-1 accuracy)
    winning_policy_name = max(gt_results_by_policy, key=lambda k: gt_results_by_policy[k]["overall_top1_accuracy"])
    winning_crop_policy = policies[winning_policy_name]
    print(f"Winning crop policy: '{winning_policy_name}' (Top-1: {gt_results_by_policy[winning_policy_name]['overall_top1_accuracy']:.2%})")

    # 5. Load Detector Model and Run Actual Proposals Baseline
    detector = QuadProposalDetector(pretrained_backbone=False, neck_type="lite")
    ckpt = load_checkpoint(primary_ckpt)
    load_model_state_strict(detector, ckpt, kind="quad", neck_type="lite", state_key=primary_meta["checkpoint_state"])
    detector.to(device).eval()

    proposal_results = run_proposal_baseline(
        detector=detector,
        teacher=teacher,
        image_records=semantic_records,
        dataset=dataset,
        selected_indices=eval_indices,
        categories=target_eval_cats,
        text_bank=text_bank[: len(target_eval_cats)],
        crop_policy=winning_crop_policy,
        max_proposals_per_image=args.proposals_per_image,
        nms_threshold=float(primary_meta["nms_iou_threshold"]),
        score_threshold=float(primary_meta["score_threshold"]),
        device=device,
    )

    # 6. Apply Readiness Rubric
    best_gt_class_res = gt_results_by_policy[winning_policy_name]["per_class"]
    prop_recall_res = proposal_results["proposal_recall_per_class"]
    readiness_decisions: dict[str, Any] = {}

    all_evaluation_classes = target_eval_cats + list(contract["vocabulary"]["unsupported"])
    for cat in all_evaluation_classes:
        is_base = cat in contract["vocabulary"]["base_classes"]
        gt_info = best_gt_class_res.get(cat, {"instances": 0, "top1_accuracy": 0.0})
        rec_info = prop_recall_res.get(cat, {"recall_iou050": 0.0})

        decision = make_readiness_decision(
            category=cat,
            gt_instances=gt_info["instances"],
            top1_acc=gt_info["top1_accuracy"],
            recall050=rec_info["recall_iou050"],
            is_base=is_base,
            criteria=contract["readiness_criteria"],
        )
        readiness_decisions[cat] = {
            **decision,
            "type": "base" if is_base else "novel" if cat in NOVEL_CATEGORIES else "unsupported",
            "gt_instances": gt_info["instances"],
            "teacher_top1_accuracy": gt_info["top1_accuracy"],
            "proposal_recall_iou050": rec_info["recall_iou050"],
            "end_to_end_ap50": proposal_results["end_to_end_ap50"].get(cat, 0.0),
        }

    # 7. Assemble and Save Deliverable Report
    final_report = {
        "schema_version": "phase4-0-baseline-report.v1",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "contract": contract,
        "evaluation_cohort": {
            "total_images": len(semantic_records),
            "frozen_800_images": len(frozen_800_indices),
            "stratified": args.stratified,
            "subset_sha256": computed_subset_sha,
        },
        "gt_crops_baseline": {
            "winning_policy": winning_policy_name,
            "policies": gt_results_by_policy,
        },
        "actual_proposals_baseline": proposal_results,
        "readiness_decisions": readiness_decisions,
    }

    report_json_path = output_dir / "phase4_0_teacher_baseline_report.json"
    with report_json_path.open("w", encoding="utf-8") as f:
        json.dump(final_report, f, indent=2)
    print(f"Saved Phase 4.0 Report JSON to: {report_json_path}")

    # Generate Markdown Summary
    report_md_path = output_dir / "phase4_0_teacher_baseline_report.md"
    md_lines = [
        "# Phase 4.0: Experiment and Teacher Baselines — Closeout Report",
        "",
        f"**Date:** {time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime())}  ",
        f"**Selected Detector:** `{primary_meta['name']}` (NMS: {primary_meta['nms_iou_threshold']}, Score: {primary_meta['score_threshold']}, K={primary_meta['max_proposals']})  ",
        f"**Selected Teacher:** `{contract['teacher']['model_name']}` (Revision: `{contract['teacher']['revision'][:10]}`)  ",
        f"**Winning Crop Policy:** `{winning_policy_name}` ({winning_crop_policy.mode}, margin={winning_crop_policy.context_margin})  ",
        "",
        "## 1. Per-Class Readiness Decision Gate",
        "",
        "| Category | Type | Instances | Teacher Top-1 | Proposal Recall@0.50 | E2E AP50 | Decision | Reason |",
        "|---|---|---|---|---|---|---|---|",
    ]

    for cat, dec in readiness_decisions.items():
        md_lines.append(
            f"| `{cat}` | {dec['type']} | {dec['gt_instances']} | "
            f"{dec['teacher_top1_accuracy']:.1%} | {dec['proposal_recall_iou050']:.1%} | "
            f"{dec['end_to_end_ap50']:.1%} | **{dec['status']}** | {dec['reason']} |"
        )

    md_lines.extend([
        "",
        "## 2. Crop Policy Comparison",
        "",
        "| Policy | Mode | Margin | Resized | Evaluated Crops | Top-1 Accuracy |",
        "|---|---|---|---|---|---|",
    ])
    for pol_name, pol_res in gt_results_by_policy.items():
        md_lines.append(
            f"| `{pol_name}` | {pol_res['crop_policy']['mode']} | {pol_res['crop_policy']['context_margin']} | "
            f"{'Yes' if not policies[pol_name].use_original_resolution else 'No'} | "
            f"{pol_res['total_evaluated_crops']} | **{pol_res['overall_top1_accuracy']:.2%}** |"
        )

    md_lines.extend([
        "",
        "## 3. Localization and Proposal Coverage by Object Size",
        "",
        "| Size Bin | GT Instances | Proposal Recall@0.50 | Teacher Crop Top-1 |",
        "|---|---|---|---|",
    ])
    for s_bin in ("tiny", "small", "medium", "large"):
        rec_s = proposal_results["proposal_recall_per_size"].get(s_bin, {})
        gt_s = gt_results_by_policy[winning_policy_name]["per_size"].get(s_bin, {})
        md_lines.append(
            f"| `{s_bin}` | {rec_s.get('instances', 0)} | "
            f"{rec_s.get('recall_iou050', 0.0):.1%} | {gt_s.get('top1_accuracy', 0.0):.1%} |"
        )

    md_lines.extend([
        "",
        f"**Mean AP50:** {proposal_results['mean_ap50']:.2%}  ",
        f"**Conditional Recognition Accuracy:** {proposal_results['conditional_recognition_accuracy']:.2%} (on {proposal_results['conditional_samples']} localized proposals)  ",
    ])

    report_md_path.write_text("\n".join(md_lines), encoding="utf-8")
    print(f"Saved Phase 4.0 Report Markdown to: {report_md_path}")


if __name__ == "__main__":
    main()
