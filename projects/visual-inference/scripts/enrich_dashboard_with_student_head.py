#!/usr/bin/env python3
"""Enrich open-vocabulary proposals with Student RegionEmbeddingHead classifications.

Runs the trained StudentDetector + RegionEmbeddingHead over all holdout images
to generate direct student-side open-vocabulary predictions, enabling side-by-side
Student vs SigLIP Teacher comparison in the dashboard.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from PIL import Image
import torch

from student_detector.augmentation import (
    AugmentationConfig,
    apply_image_augmentation,
    sample_augmentation_parameters,
)
from student_detector.decoder import InferenceDecoder
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.ontology import DISTRACTOR_CATEGORIES
from student_detector.open_vocab_heads import (
    DenseAnchorSemanticHead,
    RoIPooledSemanticHead,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.region_head import RegionEmbeddingHead
from student_detector.sam_student_data import DEFAULT_IMAGE_ROOT
from student_detector.vocabulary import VocabularyBank


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-json",
        type=Path,
        default=Path("artifacts/open_vocabulary_images_data.json"),
        help="Path to open_vocabulary_images_data.json",
    )
    parser.add_argument(
        "--hbb-student-checkpoint",
        type=Path,
        default=Path("artifacts/finetune_8k_hbb_8k_full/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"),
        help="Path to HBB student checkpoint",
    )
    parser.add_argument(
        "--quad-student-checkpoint",
        type=Path,
        default=Path("artifacts/finetune_8k_quad_8k_full/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"),
        help="Path to Quad student checkpoint",
    )
    parser.add_argument(
        "--hbb-region-head-checkpoint",
        type=Path,
        default=Path("artifacts/distill_8k_region_head/checkpoints/region_head_best.pt"),
        help="Path to trained HBB RegionEmbeddingHead checkpoint",
    )
    parser.add_argument(
        "--quad-region-head-checkpoint",
        type=Path,
        default=Path("artifacts/distill_8k_region_head_quad/checkpoints/region_head_best.pt"),
        help="Path to trained Quad RegionEmbeddingHead checkpoint",
    )
    parser.add_argument(
        "--vocabulary-path",
        type=Path,
        default=Path("artifacts/distill_8k_region_head/vocabulary_bank_37classes.pt"),
        help="Path to serialized VocabularyBank",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Root path containing raw images",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Inference device",
    )
    parser.add_argument(
        "--use-centroid-subtraction",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable centroid centering subtraction during classification",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)
    strides = (4, 8, 16, 32)
    print(f"Enriching dashboard data using Student Region Head on {device}...")

    def _load_pipeline(
        stu_ckpt_path: Path,
        reg_ckpt_path: Path,
        geom: str,
    ) -> tuple[
        StudentDetector | QuadProposalDetector,
        DenseAnchorSemanticHead | None,
        RoIPooledSemanticHead | None,
        RegionEmbeddingHead | None,
        float,
        torch.Tensor | None,
    ]:
        ckpt = torch.load(stu_ckpt_path, map_location=device, weights_only=False)
        state = ckpt.get("model_state_dict", {})
        det_cfg = ckpt.get("detector_config", {})
        has_aux = bool(det_cfg.get("dual_head", any("aux_head" in k for k in state)))
        has_3dmf = bool(det_cfg.get("use_3dmf", any("filter_3dmf" in k for k in state)))
        neck_t = str(
            det_cfg.get(
                "neck_type",
                "pan" if any("down_convs" in k for k in state) else "lite",
            )
        )
        fpn_ch = int(det_cfg.get("fpn_channels", 128 if neck_t == "pan" else 96))
        cls_det = StudentDetector if geom == "hbb" else QuadProposalDetector
        det = cls_det(
            pretrained_backbone=False,
            neck_type=neck_t,
            strides=strides,
            fpn_channels=fpn_ch,
            dual_head=has_aux,
            use_3dmf=has_3dmf,
        ).to(device)
        det.load_state_dict(state)
        det.eval()

        if "head_a_config" in ckpt and "head_b_config" in ckpt:
            ha = DenseAnchorSemanticHead.from_config(ckpt["head_a_config"]).to(device)
            ha.load_state_dict(ckpt["head_a_state_dict"])
            ha.eval()
            hb = RoIPooledSemanticHead.from_config(ckpt["head_b_config"]).to(device)
            hb.load_state_dict(ckpt["head_b_state_dict"])
            hb.eval()
            return det, ha, hb, None, 1.0, None

        reg_head, r_ckpt = RegionEmbeddingHead.load_from_checkpoint(
            reg_ckpt_path, device=device
        )
        temp = float(r_ckpt.get("cls_temperature", 1.0))
        cent = r_ckpt.get("centroid")
        if cent is not None:
            cent = cent.to(device)
        return det, None, None, reg_head, temp, cent

    # 1. Load Student Detector + Semantic Heads (SOTA Joint or Legacy RegionHead)
    hbb_student, hbb_ha, hbb_hb, hbb_region_head, hbb_temperature, hbb_centroid = (
        _load_pipeline(args.hbb_student_checkpoint, args.hbb_region_head_checkpoint, "hbb")
    )
    quad_student, quad_ha, quad_hb, quad_region_head, quad_temperature, quad_centroid = (
        _load_pipeline(args.quad_student_checkpoint, args.quad_region_head_checkpoint, "quad")
    )
    hbb_sota_dec = InferenceDecoder(
        strides=strides,
        top_k=500,
        nms_iou_threshold=0.50,
        max_detections=100,
        score_mode="objectness",
        nms_mode="containment_aware",
        score_threshold=0.05,
    )
    quad_sota_dec = QuadInferenceDecoder(
        strides=strides,
        pre_nms_top_k=500,
        nms_iou_threshold=0.50,
        max_proposals=100,
        centerness_alpha=0.0,
        nms_mode="containment_aware",
        score_threshold=0.05,
    )

    # 3. Load Vocabulary Bank
    vocab = VocabularyBank.load(args.vocabulary_path, device=device)
    distractor_set = set(DISTRACTOR_CATEGORIES)
    print(f"Calibration temperatures: HBB T={hbb_temperature:.2f}, Quad T={quad_temperature:.2f}")
    if args.use_centroid_subtraction and (hbb_centroid is not None or vocab.centroid is not None):
        print("Centroid centering: Enabled for anisotropic manifold correction.")
    else:
        print("Centroid centering: Disabled (preserving native class clusters).")

    # 4. Load Data JSON
    with open(args.data_json) as f:
        images_data: list[dict[str, Any]] = json.load(f)

    cfg = AugmentationConfig()
    params = sample_augmentation_parameters(cfg, 384, training=False, seed=42)

    t0 = time.time()
    total_hbb = 0
    total_quad = 0
    hbb_labeled = 0
    quad_labeled = 0
    hbb_agree = 0
    quad_agree = 0

    print(f"Running student inference across {len(images_data)} holdout images...")
    for idx, item in enumerate(images_data):
        img_path = args.image_root / item["file_name"]
        if not img_path.exists():
            continue

        try:
            pil_img = Image.open(img_path).convert("RGB")
        except Exception as e:
            print(f"Warning: Failed to open {img_path}: {e}")
            continue

        aug = apply_image_augmentation(pil_img, 384, params)
        scale, off_x, off_y = aug.transform
        img_t = aug.tensor.unsqueeze(0).to(device)

        with torch.no_grad():
            txt_emb = vocab.embeddings.to(device)

            def _score_boxes(
                feats: tuple[torch.Tensor, ...],
                b_t: torch.Tensor,
                stu_det: StudentDetector | QuadProposalDetector,
                dec: Any,
                ha: DenseAnchorSemanticHead | None,
                hb: RoIPooledSemanticHead | None,
                reg_head: RegionEmbeddingHead | None,
                temp: float,
                cent: torch.Tensor | None,
            ) -> tuple[torch.Tensor, torch.Tensor | None, torch.Tensor | None]:
                if hb is not None:
                    zb = hb(feats, [b_t], image_shapes=[(384, 384)])
                    _, pb = hb.classify_embeddings(zb, txt_emb)
                    pa: torch.Tensor | None = None
                    if ha is not None:
                        live = dec(stu_det.head(feats), (384, 384))[0]
                        if (
                            live.flat_indices is not None
                            and live.flat_indices.numel() > 0
                            and live.boxes.numel() > 0
                        ):
                            from torchvision.ops import box_iou

                            best_live = box_iou(b_t, live.boxes).argmax(dim=1)
                            matched_flat = live.flat_indices[best_live]
                            za = ha.forward_indexed(
                                feats, torch.zeros_like(matched_flat), matched_flat
                            )
                            _, pa = ha.classify_embeddings(za, txt_emb)
                    p_ab = 0.5 * (pa + pb) if pa is not None else pb
                    return p_ab, pa, pb

                assert reg_head is not None
                embs = reg_head(feats, [b_t])
                if cent is not None:
                    vocab.centroid = cent
                _, probs_leg = vocab.classify(
                    embs,
                    mode="softmax",
                    temperature=temp,
                    use_centroid_subtraction=args.use_centroid_subtraction,
                )
                return probs_leg, None, None

            def _write_preds(
                preds_list: list[dict[str, Any]],
                probs_main: torch.Tensor,
                probs_a: torch.Tensor | None,
                probs_b: torch.Tensor | None,
            ) -> tuple[int, int, int]:
                cnt_tot = 0
                cnt_lbl = 0
                cnt_agr = 0
                k_top = min(3, int(probs_main.shape[-1]))
                top_val, top_idx = probs_main.topk(k_top, dim=-1)
                for j, p in enumerate(preds_list):
                    cnt_tot += 1
                    stu_lbl = vocab.categories[int(top_idx[j, 0].item())]
                    stu_prob = round(float(top_val[j, 0].item()), 3)
                    p["stu_label"] = stu_lbl
                    p["stu_prob"] = stu_prob
                    p["stu_is_distractor"] = stu_lbl in distractor_set
                    p["stu_top3"] = [
                        (
                            vocab.categories[int(top_idx[j, k].item())],
                            round(float(top_val[j, k].item()), 3),
                        )
                        for k in range(k_top)
                    ]
                    p["stu_joint"] = round(float(p["score"]) * stu_prob, 4)
                    if probs_a is not None:
                        va, ia = probs_a[j].topk(k_top)
                        la = vocab.categories[int(ia[0].item())]
                        p["opt_a_label"] = la
                        p["opt_a_prob"] = round(float(va[0].item()), 3)
                        p["opt_a_is_distractor"] = la in distractor_set
                        p["opt_a_top3"] = [
                            (vocab.categories[int(ia[k].item())], round(float(va[k].item()), 3))
                            for k in range(k_top)
                        ]
                    if probs_b is not None:
                        vb, ib = probs_b[j].topk(k_top)
                        lb = vocab.categories[int(ib[0].item())]
                        p["opt_b_label"] = lb
                        p["opt_b_prob"] = round(float(vb[0].item()), 3)
                        p["opt_b_is_distractor"] = lb in distractor_set
                        p["opt_b_top3"] = [
                            (vocab.categories[int(ib[k].item())], round(float(vb[k].item()), 3))
                            for k in range(k_top)
                        ]
                    if p.get("label") is not None:
                        cnt_lbl += 1
                        cnt_agr += int(p["label"] == stu_lbl)
                return cnt_tot, cnt_lbl, cnt_agr

            # Process HBB proposals
            hbb_preds = item.get("hbb_after", [])
            if hbb_preds:
                hbb_feats = hbb_student.forward_features(img_t)
                b_canvas = [
                    [
                        p["box"][0] * scale + off_x,
                        p["box"][1] * scale + off_y,
                        p["box"][2] * scale + off_x,
                        p["box"][3] * scale + off_y,
                    ]
                    for p in hbb_preds
                ]
                b_t = torch.tensor(b_canvas, dtype=torch.float32, device=device)
                pm, pa, pb = _score_boxes(
                    hbb_feats,
                    b_t,
                    hbb_student,
                    hbb_sota_dec,
                    hbb_ha,
                    hbb_hb,
                    hbb_region_head,
                    hbb_temperature,
                    hbb_centroid,
                )
                c1, c2, c3 = _write_preds(hbb_preds, pm, pa, pb)
                total_hbb += c1
                hbb_labeled += c2
                hbb_agree += c3

            # Process Quad proposals
            quad_preds = item.get("quad_after", [])
            if quad_preds:
                quad_feats = quad_student.forward_features(img_t)
                qb_canvas = []
                for p in quad_preds:
                    q = p["quad"]
                    qx = [pt[0] for pt in q]
                    qy = [pt[1] for pt in q]
                    qb_canvas.append(
                        [
                            min(qx) * scale + off_x,
                            min(qy) * scale + off_y,
                            max(qx) * scale + off_x,
                            max(qy) * scale + off_y,
                        ]
                    )
                qb_t = torch.tensor(qb_canvas, dtype=torch.float32, device=device)
                qpm, qpa, qpb = _score_boxes(
                    quad_feats,
                    qb_t,
                    quad_student,
                    quad_sota_dec,
                    quad_ha,
                    quad_hb,
                    quad_region_head,
                    quad_temperature,
                    quad_centroid,
                )
                qc1, qc2, qc3 = _write_preds(quad_preds, qpm, qpa, qpb)
                total_quad += qc1
                quad_labeled += qc2
                quad_agree += qc3

    dt = time.time() - t0
    print(f"Finished student inference in {dt:.2f}s ({dt / len(images_data) * 1000:.1f} ms/img).")
    # Agreement is measured only on proposals the teacher actually labeled.
    print(f"HBB: {total_hbb} proposals, {hbb_labeled} teacher-labeled, "
          f"top-1 match {hbb_agree / max(hbb_labeled, 1) * 100:.2f}%")
    print(f"Quad: {total_quad} proposals, {quad_labeled} teacher-labeled, "
          f"top-1 match {quad_agree / max(quad_labeled, 1) * 100:.2f}%")

    # Save enriched data JSON
    with open(args.data_json, "w", encoding="utf-8") as f:
        json.dump(images_data, f)
    print(f"Updated {args.data_json} with student region head predictions!")


if __name__ == "__main__":
    main()
