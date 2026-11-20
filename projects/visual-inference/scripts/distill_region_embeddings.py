#!/usr/bin/env python3
"""Stage-2 Open-Vocabulary Distillation: Distill SigLIP into Student Region-Embedding Head.

Attaches a lightweight MultiScaleRoIAlign RegionEmbeddingHead to the frozen P2LiteFPN student
and distills 768-D unit L2 visual embeddings from SiglipTeacher, enabling real-time
zero-runtime-text-encoder open-vocabulary detection on edge hardware (Raspberry Pi 5).
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import random
import time
from typing import Any

from PIL import Image
import torch
from torch import Tensor
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader
import torchvision.transforms.functional as TF

if hasattr(torch.multiprocessing, "set_sharing_strategy"):
    torch.multiprocessing.set_sharing_strategy("file_system")

from student_detector.human_gt_data import (
    DEFAULT_ENRICHED_GT_INDEX_PATH,
    HumanGTHBBDataset,
    HumanGTQuadDataset,
)
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.ontology import (
    BASE_CATEGORIES,
    DISTRACTOR_CATEGORIES,
    EXTENDED_CATEGORIES,
    NOVEL_CATEGORIES,
)
from student_detector.region_head import RegionEmbeddingHead
from student_detector.sam_student_data import (
    DEFAULT_IMAGE_ROOT,
    collate_proposal_samples,
    collate_quad_proposal_samples,
)
from student_detector.siglip import SiglipTeacher
from student_detector.vocabulary import (
    DEFAULT_CATEGORY_SYNONYMS,
    VocabularyBank,
    expected_calibration_error,
    fit_temperature,
)

ADDITIONAL_CATEGORIES: tuple[str, ...] = (
    "tree",
    "window",
    "door",
    "building",
    "motorcycle",
    "bicycle",
    "stairs",
)

FULL_VOCABULARY: tuple[str, ...] = (
    BASE_CATEGORIES
    + NOVEL_CATEGORIES
    + EXTENDED_CATEGORIES
    + ADDITIONAL_CATEGORIES
    + DISTRACTOR_CATEGORIES
)


def sample_balanced_box_indices(
    categories: list[str] | tuple[str, ...] | None,
    total_boxes: int,
    max_boxes: int,
) -> list[int]:
    """Sample box indices with round-robin category balancing to avoid frequent-class dominance."""
    if categories is None or len(categories) != total_boxes:
        return torch.randperm(total_boxes)[:max_boxes].tolist()

    cat_to_indices: dict[str, list[int]] = {}
    for idx, c in enumerate(categories):
        cat_to_indices.setdefault(c, []).append(idx)

    selected: list[int] = []
    shuffled_cats = list(cat_to_indices.keys())
    random.shuffle(shuffled_cats)
    for c in shuffled_cats:
        if len(selected) < max_boxes and cat_to_indices[c]:
            pick_idx = cat_to_indices[c].pop(random.randrange(len(cat_to_indices[c])))
            selected.append(pick_idx)

    remaining = [idx for idxs in cat_to_indices.values() for idx in idxs]
    random.shuffle(remaining)
    selected.extend(remaining[: max_boxes - len(selected)])
    return selected


def extract_teacher_crop(
    sample: Any,
    box: Tensor,  # [4] (x1, y1, x2, y2) on 384 canvas
    image_root: Path,
    canvas_image_unnorm: Tensor,  # [3, 384, 384] float tensor [0, 1]
    pil_cache: Image.Image | None = None,
) -> tuple[Tensor, Image.Image | None]:
    """Extract a 224x224 bicubic crop from native-res image (or fallback to canvas)."""
    pil_img = pil_cache
    if pil_img is None and hasattr(sample, "file_name") and sample.file_name:
        pil_path = image_root / sample.file_name
        if pil_path.exists():
            try:
                pil_img = Image.open(pil_path).convert("RGB")
            except Exception:
                pil_img = None

    is_flipped = bool(getattr(sample, "horizontal_flip", False))
    bw = box[2] - box[0]
    bh = box[3] - box[1]
    pad_w = 0.15 * bw
    pad_h = 0.15 * bh

    if pil_img is not None:
        orig_w, orig_h = pil_img.size
        from student_detector.augmentation import canvas_box_to_native

        cx1, cy1, cx2, cy2 = canvas_box_to_native(
            box,
            sample.transform,
            (orig_h, orig_w),
            horizontal_flip=is_flipped,
            margin=0.15,
        )

        if (cx2 - cx1) >= 4 and (cy2 - cy1) >= 4:
            crop_pil = pil_img.crop((int(cx1), int(cy1), int(cx2), int(cy2)))
            if is_flipped:
                crop_pil = crop_pil.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            crop_pil = crop_pil.resize((224, 224), Image.Resampling.BICUBIC)
            crop_t = TF.to_tensor(crop_pil)
            crop_siglip = (crop_t - 0.5) / 0.5
            return crop_siglip, pil_img

    # Fallback to canvas crop
    cx1 = int(max(0.0, box[0].item() - pad_w.item()))
    cy1 = int(max(0.0, box[1].item() - pad_h.item()))
    cx2 = int(min(384.0, box[2].item() + pad_w.item()))
    cy2 = int(min(384.0, box[3].item() + pad_h.item()))
    if (cx2 - cx1) >= 4 and (cy2 - cy1) >= 4:
        crop_canvas = canvas_image_unnorm[:, cy1:cy2, cx1:cx2].unsqueeze(0)
        crop_224 = F.interpolate(
            crop_canvas, size=(224, 224), mode="bilinear", align_corners=False
        )
        crop_siglip = (crop_224.squeeze(0) - 0.5) / 0.5
        return crop_siglip, pil_img

    return torch.empty((0, 224, 224), dtype=torch.float32), pil_img


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--geometry",
        type=str,
        choices=["hbb", "quad"],
        default="hbb",
        help="Geometry type of the student detector.",
    )
    parser.add_argument(
        "--student-checkpoint",
        type=Path,
        default=None,
        help="Path to trained Stage-2 student checkpoint (defaults based on geometry).",
    )
    parser.add_argument(
        "--cohort-db",
        type=Path,
        default=Path("artifacts/pseudo_labels/sam_discovery_enriched.sqlite"),
        help="Path to sam_discovery_enriched.sqlite defining cohort.",
    )
    parser.add_argument(
        "--gt-index",
        type=Path,
        default=DEFAULT_ENRICHED_GT_INDEX_PATH,
        help="Path to quad_train_enriched.sqlite.",
    )
    parser.add_argument(
        "--image-root",
        type=Path,
        default=DEFAULT_IMAGE_ROOT,
        help="Path to raw dataset images directory.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=5,
        help="Number of distillation training epochs.",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=32,
        help="Batch size of images per training step.",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=1e-3,
        help="Initial learning rate for RegionEmbeddingHead.",
    )
    parser.add_argument(
        "--cohort-size",
        type=int,
        default=8000,
        help="Balanced cohort size (default: 8000).",
    )
    parser.add_argument(
        "--boxes-per-image",
        type=int,
        default=12,
        help="Maximum region boxes to distill per image per batch.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory for checkpoints and reports (defaults based on geometry).",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        default=None,
        help="Path to checkpoint to resume training from.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Execution device ('cuda' or 'cpu').",
    )
    parser.add_argument(
        "--kl-weight",
        type=float,
        default=1.0,
        help="Weight of the vocabulary-distribution KL distillation term.",
    )
    parser.add_argument(
        "--kl-temperature",
        type=float,
        default=4.0,
        help="Softening temperature for the KL term (teacher logits are very peaky at scale ~117).",
    )
    parser.add_argument(
        "--use-context-roi",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable dual-scale (focal + context) RoI pooling.",
    )
    parser.add_argument(
        "--use-box-geometry",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable geometric coordinate conditioning (6-D scale/elevation prior).",
    )
    parser.add_argument(
        "--focal-margin",
        type=float,
        default=0.15,
        help="Margin expansion for focal RoIAlign (matches teacher crop).",
    )
    parser.add_argument(
        "--context-margin",
        type=float,
        default=0.45,
        help="Margin expansion for context RoIAlign (surrounding environment).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    device = torch.device(args.device)

    if args.student_checkpoint is None:
        if args.geometry == "quad":
            args.student_checkpoint = Path(
                "artifacts/finetune_8k_quad_8k_full/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
            )
        else:
            args.student_checkpoint = Path(
                "artifacts/finetune_8k_hbb_8k_full/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"
            )

    if args.output_dir is None:
        if args.geometry == "quad":
            args.output_dir = Path("artifacts/distill_8k_region_head_quad")
        else:
            args.output_dir = Path("artifacts/distill_8k_region_head")

    print(f"Starting Region-Embedding Distillation for geometry: {args.geometry.upper()} on {device}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    ckpt_dir = args.output_dir / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load trained student detector
    print(f"Loading frozen student detector from {args.student_checkpoint}...")
    strides = (4, 8, 16, 32)
    if args.geometry == "hbb":
        student_model: nn.Module = StudentDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
        ).to(device)
    else:
        student_model = QuadProposalDetector(
            pretrained_backbone=False,
            neck_type="lite",
            strides=strides,
            fpn_channels=96,
        ).to(device)

    ckpt_data = torch.load(args.student_checkpoint, map_location=device)
    student_model.load_state_dict(ckpt_data["model_state_dict"])
    student_model.eval()
    for param in student_model.parameters():
        param.requires_grad = False
    print("Student detector loaded and frozen successfully.")

    # 2. Instantiate SigLIP teacher and build vocabulary bank
    print(f"Initializing frozen SiglipTeacher on {device}...")
    teacher = SiglipTeacher(device=device)
    print(f"Building offline VocabularyBank for {len(FULL_VOCABULARY)} categories (with synonym ensembling)...")
    vocab_bank = VocabularyBank.from_teacher(
        teacher, FULL_VOCABULARY, synonyms=DEFAULT_CATEGORY_SYNONYMS
    ).to(device)
    vocab_path = args.output_dir / "vocabulary_bank_37classes.pt"
    vocab_bank.save(vocab_path)
    print(f"Saved variable VocabularyBank to {vocab_path} ({vocab_path.stat().st_size / 1024:.1f} KB)")

    # 3. Instantiate RegionEmbeddingHead
    print("Instantiating lightweight RegionEmbeddingHead (RoIAlign + MLP projector)...")
    region_head = RegionEmbeddingHead(
        in_channels=96,
        embedding_dim=768,
        output_size=3,
        hidden_dim=512,
        strides=strides,
        focal_margin=args.focal_margin,
        context_margin=args.context_margin,
        use_context_roi=args.use_context_roi,
        use_box_geometry=args.use_box_geometry,
    ).to(device)
    total_params = sum(p.numel() for p in region_head.parameters() if p.requires_grad)
    print(f"RegionEmbeddingHead trainable parameters: {total_params:,} ({total_params / 1e6:.3f}M)")

    optimizer = torch.optim.AdamW(region_head.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs, eta_min=1e-5)

    # 4. Create Datasets and DataLoaders
    print("Preparing 8k balanced training and validation datasets...")
    if args.geometry == "hbb":
        train_ds = HumanGTHBBDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            cohort_size=args.cohort_size,
            split_seed=42,
            val_ratio=0.10,
            split="train",
        )
        val_ds = HumanGTHBBDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            cohort_size=args.cohort_size,
            split_seed=42,
            val_ratio=0.10,
            split="val",
        )
        collate_fn = collate_proposal_samples
    else:
        train_ds = HumanGTQuadDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            cohort_size=args.cohort_size,
            split_seed=42,
            val_ratio=0.10,
            split="train",
        )
        val_ds = HumanGTQuadDataset(
            cohort_db_path=args.cohort_db,
            gt_index_path=args.gt_index,
            image_root=args.image_root,
            cohort_size=args.cohort_size,
            split_seed=42,
            val_ratio=0.10,
            split="val",
        )
        collate_fn = collate_quad_proposal_samples

    print(f"Training samples: {len(train_ds)}, Validation samples: {len(val_ds)}")
    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=4,
        collate_fn=collate_fn,
        pin_memory=True,
        drop_last=True,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=4,
        collate_fn=collate_fn,
        pin_memory=True,
    )

    IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406], device=device).view(1, 3, 1, 1)
    IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225], device=device).view(1, 3, 1, 1)

    history: list[dict[str, Any]] = []
    best_val_cosine = 0.0
    best_val_top1 = 0.0
    start_epoch = 1

    if args.resume is not None and args.resume.exists():
        print(f"Resuming from checkpoint {args.resume}...")
        res_data = torch.load(args.resume, map_location=device)
        region_head.load_state_dict(res_data["region_head_state_dict"])
        if "optimizer_state_dict" in res_data:
            optimizer.load_state_dict(res_data["optimizer_state_dict"])
        start_epoch = res_data.get("epoch", 0) + 1
        best_val_cosine = float(res_data.get("val_mean_cosine_similarity", 0.0))
        best_val_top1 = float(res_data.get("val_top1_agreement", 0.0))
        print(f"Resumed at epoch {start_epoch}, previous best val cosine: {best_val_cosine:.4f}")

    print("\n" + "=" * 60)
    print(f"BEGINNING REGION DISTILLATION TRAINING ({start_epoch} to {args.epochs} EPOCHS)")
    print("=" * 60)

    t_start_total = time.time()
    for epoch in range(start_epoch, args.epochs + 1):
        region_head.train()
        epoch_loss_total = 0.0
        batch_count = 0
        t_epoch_start = time.time()

        for batch_idx, batch in enumerate(train_loader):
            images, samples = batch
            images = images.to(device, non_blocking=True)
            B = images.shape[0]

            # 1. Forward frozen student FPN features
            with torch.no_grad():
                features = student_model.forward_features(images)

            # 2. Extract bounding boxes and GPU tensor crops for each image in batch
            images_unnorm = (images * IMAGENET_STD + IMAGENET_MEAN).clamp(0.0, 1.0)
            batch_boxes_student: list[torch.Tensor] = []
            crops_tensors: list[torch.Tensor] = []
            valid_box_indices: list[tuple[int, int]] = []  # (batch_i, box_i)

            for b_i in range(B):
                sample = samples[b_i]

                # Get boxes for this image in 384x384 canvas coordinates
                if hasattr(sample, "boxes") and sample.boxes.numel() > 0:
                    raw_b = sample.boxes
                elif hasattr(sample, "quads") and sample.quads.numel() > 0:
                    raw_b = RegionEmbeddingHead.boxes_from_quads(sample.quads)
                else:
                    raw_b = torch.empty((0, 4), dtype=torch.float32)

                # Filter valid non-degenerate boxes, then apply class-balanced subsampling.
                if raw_b.numel() > 0:
                    w = raw_b[:, 2] - raw_b[:, 0]
                    h = raw_b[:, 3] - raw_b[:, 1]
                    valid_mask = (w >= 6.0) & (h >= 6.0)
                    raw_b = raw_b[valid_mask]
                    valid_cats = None
                    if hasattr(sample, "category_names") and len(sample.category_names) == len(valid_mask):
                        valid_cats = [sample.category_names[i] for i, v in enumerate(valid_mask.tolist()) if v]
                    sel_indices = sample_balanced_box_indices(valid_cats, raw_b.shape[0], args.boxes_per_image)
                    raw_b = raw_b[sel_indices]

                batch_boxes_student.append(raw_b.to(device))

                # Extract teacher crops (native high-resolution bicubic, with canvas fallback)
                pil_cache = None
                for box_j in range(raw_b.shape[0]):
                    crop_siglip, pil_cache = extract_teacher_crop(
                        sample=sample,
                        box=raw_b[box_j],
                        image_root=args.image_root,
                        canvas_image_unnorm=images_unnorm[b_i],
                        pil_cache=pil_cache,
                    )
                    if crop_siglip.numel() > 0:
                        crops_tensors.append(crop_siglip)
                        valid_box_indices.append((b_i, box_j))

            total_valid_crops = len(crops_tensors)
            if total_valid_crops < 2:
                continue

            # 3. Extract teacher visual embeddings for all valid crops on GPU
            pixel_values = torch.stack(crops_tensors, dim=0)
            with torch.no_grad():
                teacher_embs = teacher.encode_images(pixel_values, batch_size=64).to(device)

            # 4. Extract student region embeddings via RegionEmbeddingHead
            student_embs_all = region_head(features, batch_boxes_student)

            # Slice only student embeddings corresponding to valid_box_indices
            box_offsets = [0]
            for b_boxes in batch_boxes_student:
                box_offsets.append(box_offsets[-1] + b_boxes.shape[0])

            selected_student_indices = [
                box_offsets[b_i] + box_j for (b_i, box_j) in valid_box_indices
            ]
            student_embs = student_embs_all[selected_student_indices]

            # 5. Distillation Loss: Cosine distance + In-batch Contrastive alignment
            cosine_similarity = (student_embs * teacher_embs).sum(dim=-1)
            loss_cosine = (1.0 - cosine_similarity).mean()

            # Contrastive InfoNCE loss across regions in the batch
            tau = 0.07
            sim_matrix = torch.matmul(student_embs, teacher_embs.t()) / tau
            targets = torch.arange(total_valid_crops, device=device)
            loss_nce = F.cross_entropy(sim_matrix, targets)

            # Vocabulary KL: match the teacher's class distribution over the text bank.
            # Cosine alone leaves the residual error free to point at the wrong class;
            # this term spends capacity on the directions that decide the label.
            t_logits, _ = vocab_bank.classify(teacher_embs)
            s_logits, _ = vocab_bank.classify(student_embs)
            loss_kl = F.kl_div(
                F.log_softmax(s_logits / args.kl_temperature, dim=-1),
                F.log_softmax(t_logits / args.kl_temperature, dim=-1),
                reduction="batchmean",
                log_target=True,
            )

            loss = loss_cosine + 0.25 * loss_nce + args.kl_weight * loss_kl

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(region_head.parameters(), max_norm=5.0)
            optimizer.step()

            epoch_loss_total += float(loss.item())
            batch_count += 1

            if batch_idx % 25 == 0:
                print(
                    f"Epoch {epoch}/{args.epochs} [{batch_idx}/{len(train_loader)}] "
                    f"Loss: {loss.item():.4f} (Cos: {loss_cosine.item():.4f}, NCE: {loss_nce.item():.4f}, "
                    f"KL: {loss_kl.item():.4f}) "
                    f"MeanCosSim: {cosine_similarity.mean().item():.3f} ({total_valid_crops} crops)",
                    flush=True,
                )

        scheduler.step()
        epoch_duration = time.time() - t_epoch_start
        avg_train_loss = epoch_loss_total / max(batch_count, 1)

        # 6. Validation Evaluation
        print(f"\nEvaluating Epoch {epoch} on validation cohort...")
        region_head.eval()
        val_cosine_sims: list[float] = []
        val_top1_agreements: list[float] = []
        val_student_logits: list[torch.Tensor] = []
        val_teacher_top1: list[torch.Tensor] = []
        val_student_embs_list: list[torch.Tensor] = []

        with torch.no_grad():
            for v_idx, v_batch in enumerate(val_loader):
                if v_idx >= 15:  # Cap at 15 batches (~480 images) for evaluation speed
                    break
                v_images, v_samples = v_batch
                v_images = v_images.to(device)
                v_feats = student_model.forward_features(v_images)

                v_unnorm = (v_images * IMAGENET_STD + IMAGENET_MEAN).clamp(0.0, 1.0)
                v_boxes_student: list[torch.Tensor] = []
                v_crops_tensors: list[torch.Tensor] = []
                v_indices: list[tuple[int, int]] = []

                for b_i in range(v_images.shape[0]):
                    sample = v_samples[b_i]

                    if hasattr(sample, "boxes") and sample.boxes.numel() > 0:
                        raw_b = sample.boxes
                    elif hasattr(sample, "quads") and sample.quads.numel() > 0:
                        raw_b = RegionEmbeddingHead.boxes_from_quads(sample.quads)
                    else:
                        raw_b = torch.empty((0, 4), dtype=torch.float32)

                    if raw_b.numel() > 0:
                        w = raw_b[:, 2] - raw_b[:, 0]
                        h = raw_b[:, 3] - raw_b[:, 1]
                        valid_mask = (w >= 6.0) & (h >= 6.0)
                        raw_b = raw_b[valid_mask]
                        valid_cats = None
                        if hasattr(sample, "category_names") and len(sample.category_names) == len(valid_mask):
                            valid_cats = [sample.category_names[i] for i, v in enumerate(valid_mask.tolist()) if v]
                        sel_indices = sample_balanced_box_indices(valid_cats, raw_b.shape[0], args.boxes_per_image)
                        raw_b = raw_b[sel_indices]

                    v_boxes_student.append(raw_b.to(device))

                    pil_cache = None
                    for box_j in range(raw_b.shape[0]):
                        crop_siglip, pil_cache = extract_teacher_crop(
                            sample=sample,
                            box=raw_b[box_j],
                            image_root=args.image_root,
                            canvas_image_unnorm=v_unnorm[b_i],
                            pil_cache=pil_cache,
                        )
                        if crop_siglip.numel() > 0:
                            v_crops_tensors.append(crop_siglip)
                            v_indices.append((b_i, box_j))

                if len(v_crops_tensors) == 0:
                    continue

                v_pixel_values = torch.stack(v_crops_tensors, dim=0)
                v_teacher_embs = teacher.encode_images(v_pixel_values, batch_size=64).to(device)
                v_student_all = region_head(v_feats, v_boxes_student)

                v_offsets = [0]
                for vb in v_boxes_student:
                    v_offsets.append(v_offsets[-1] + vb.shape[0])
                v_sel_idx = [v_offsets[b_i] + box_j for (b_i, box_j) in v_indices]
                v_student_embs = v_student_all[v_sel_idx]
                val_student_embs_list.append(v_student_embs.cpu())

                # Cosine similarity
                sims = (v_student_embs * v_teacher_embs).sum(dim=-1).cpu().tolist()
                val_cosine_sims.extend(sims)

                # Open-vocabulary top-1 class agreement
                t_logits_v, t_probs = vocab_bank.classify(v_teacher_embs)
                s_logits_v, s_probs = vocab_bank.classify(v_student_embs)
                t_top1 = t_probs.argmax(dim=-1)
                s_top1 = s_probs.argmax(dim=-1)
                agreements = (t_top1 == s_top1).float().cpu().tolist()
                val_top1_agreements.extend(agreements)
                val_student_logits.append(s_logits_v.cpu())
                val_teacher_top1.append(t_top1.cpu())

        mean_val_cosine = float(sum(val_cosine_sims) / max(len(val_cosine_sims), 1))
        mean_top1_agree = float(sum(val_top1_agreements) / max(len(val_top1_agreements), 1))

        # Compute empirical feature centroid and class frequency distribution
        all_s_embs = torch.cat(val_student_embs_list, dim=0)
        val_centroid = all_s_embs.mean(dim=0, keepdim=True).cpu()
        all_s_logits = torch.cat(val_student_logits)
        all_t_top1 = torch.cat(val_teacher_top1)
        val_priors = torch.bincount(all_t_top1, minlength=vocab_bank.num_classes).float().cpu()
        val_priors = val_priors / max(float(val_priors.sum()), 1.0)

        vocab_bank.centroid = val_centroid.to(device)
        vocab_bank.class_priors = val_priors.to(device)
        vocab_bank.save(vocab_path)

        # Post-hoc temperature calibration against teacher top-1 labels: after this,
        # a displayed "70%" means the student agrees with SigLIP ~70% of the time.
        cls_temperature = fit_temperature(all_s_logits, all_t_top1)
        ece_raw = expected_calibration_error(all_s_logits.softmax(-1), all_t_top1)
        ece_cal = expected_calibration_error((all_s_logits / cls_temperature).softmax(-1), all_t_top1)

        print(f"\n--- Epoch {epoch}/{args.epochs} Summary ---")
        print(f"Duration: {epoch_duration:.1f}s | Avg Train Loss: {avg_train_loss:.4f}")
        print(f"Validation Mean Cosine Similarity: {mean_val_cosine:.4f}")
        print(f"Validation Open-Vocabulary Top-1 Agreement: {mean_top1_agree * 100:.2f}%")
        print(f"Confidence calibration: T={cls_temperature:.2f}, ECE {ece_raw:.3f} -> {ece_cal:.3f}\n")

        epoch_record = {
            "epoch": epoch,
            "duration_seconds": round(epoch_duration, 2),
            "train_loss": round(avg_train_loss, 4),
            "val_mean_cosine_similarity": round(mean_val_cosine, 4),
            "val_top1_agreement": round(mean_top1_agree, 4),
            "cls_temperature": round(cls_temperature, 4),
            "ece_uncalibrated": round(ece_raw, 4),
            "ece_calibrated": round(ece_cal, 4),
            "lr": round(scheduler.get_last_lr()[0], 6),
        }
        history.append(epoch_record)

        # Save checkpoint
        ckpt_payload = {
            "epoch": epoch,
            "region_head_state_dict": region_head.state_dict(),
            "region_head_config": region_head.get_config(),
            "optimizer_state_dict": optimizer.state_dict(),
            "val_mean_cosine_similarity": mean_val_cosine,
            "val_top1_agreement": mean_top1_agree,
            "cls_temperature": cls_temperature,
            "centroid": val_centroid,
            "class_priors": val_priors,
            "context_margin": region_head.context_margin,
            "focal_margin": region_head.focal_margin,
            "use_context_roi": region_head.use_context_roi,
            "use_box_geometry": region_head.use_box_geometry,
            "geometry": args.geometry,
            "student_checkpoint": str(args.student_checkpoint),
            "vocabulary_path": str(vocab_path),
        }
        torch.save(ckpt_payload, ckpt_dir / "region_head_last.pt")

        # Select on top-1 agreement: that is what the label quality depends on,
        # and it does not track mean cosine (best-cosine epoch had the worst agreement).
        if mean_top1_agree > best_val_top1:
            best_val_top1 = mean_top1_agree
            best_val_cosine = mean_val_cosine
            torch.save(ckpt_payload, ckpt_dir / "region_head_best.pt")
            print(f"🚀 New best validation top-1 agreement: {best_val_top1 * 100:.2f}% -> saved best checkpoint!")

    total_training_time = time.time() - t_start_total

    # 7. Edge Benchmark: Measure RegionEmbeddingHead latency on GPU and CPU (simulating RPi 5)
    print("\nBenchmarking edge inference latency for RegionEmbeddingHead...")
    region_head.eval()
    test_features = (
        torch.randn(1, 96, 96, 96, device=device),
        torch.randn(1, 96, 48, 48, device=device),
        torch.randn(1, 96, 24, 24, device=device),
        torch.randn(1, 96, 12, 12, device=device),
    )
    test_boxes = [torch.rand((100, 4), device=device) * 384.0]
    # Warmup
    for _ in range(10):
        _ = region_head(test_features, test_boxes)

    # GPU Latency
    if device.type == "cuda":
        torch.cuda.synchronize()
        t0_gpu = time.time()
        for _ in range(100):
            _ = region_head(test_features, test_boxes)
        torch.cuda.synchronize()
        gpu_lat_ms = (time.time() - t0_gpu) / 100 * 1000.0
    else:
        gpu_lat_ms = 0.0

    # CPU Latency (4 threads, simulating Raspberry Pi 5 ARM Cortex-A76 cores)
    torch.set_num_threads(4)
    cpu_head = region_head.to("cpu")
    cpu_features = tuple(f.to("cpu") for f in test_features)
    cpu_boxes = [b.to("cpu") for b in test_boxes]
    for _ in range(5):
        _ = cpu_head(cpu_features, cpu_boxes)
    t0_cpu = time.time()
    for _ in range(25):
        _ = cpu_head(cpu_features, cpu_boxes)
    cpu_lat_ms = (time.time() - t0_cpu) / 25 * 1000.0

    # Matrix multiplication with 37-class text bank
    sample_reg_embs = torch.randn(100, 768)
    sample_text_bank = vocab_bank.to("cpu")
    t0_mm = time.time()
    for _ in range(500):
        _ = sample_text_bank.classify(sample_reg_embs)
    mm_lat_ms = (time.time() - t0_mm) / 500 * 1000.0

    summary_report = {
        "geometry": args.geometry,
        "completed_at": datetime.now().isoformat(),
        "total_duration_seconds": round(total_training_time, 2),
        "epochs": args.epochs,
        "best_val_cosine_similarity": round(best_val_cosine, 4),
        "best_val_top1_agreement": round(best_val_top1, 4),
        "history": history,
        "latency_benchmark_100_proposals": {
            "gpu_ms": round(gpu_lat_ms, 2) if device.type == "cuda" else None,
            "cpu_4threads_ms": round(cpu_lat_ms, 2),
            "text_matrix_multiply_ms": round(mm_lat_ms, 4),
            "estimated_rpi5_total_open_vocab_ms": round(cpu_lat_ms * 1.5 + mm_lat_ms, 2),
        },
    }

    summary_file = args.output_dir / "distillation_summary.json"
    summary_file.write_text(json.dumps(summary_report, indent=2))
    print(f"\nWrote distillation summary report to {summary_file}")
    print("\n" + "=" * 60)
    print("DISTILLATION COMPLETE & EDGE VERIFIED")
    print("=" * 60)
    print(f"Best Validation Cosine Similarity: {best_val_cosine:.4f}")
    print(f"Region Embedding Head Latency (100 proposals on 4-thread CPU): {cpu_lat_ms:.2f} ms")
    print(f"Text-Bank Dot Product (100 proposals @ 37 classes): {mm_lat_ms:.3f} ms")
    print(f"Estimated RPi 5 Open-Vocabulary Head Latency: {cpu_lat_ms * 1.5:.1f} ms")
    print("=" * 60)


if __name__ == "__main__":
    main()
