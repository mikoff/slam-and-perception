"""OWLv2-Large unified detection & 768-D embedding teacher infrastructure.

Provides:
  - Tuned open-vocabulary prompt vocabulary and native 768-D VocabularyBank builder.
  - Pedestrian crowd-split replacement rule (splitting merged group GT boxes into
    individual OWLv2 pedestrian detections while blocking single-person torso/leg splits).
  - Missing-class merger combining cleaned Human GT with OWLv2 pseudo-labels
    (including capped stuff areas and WoodScape ego-hood false-positive suppression).
  - Fast batched GPU extraction (`image_embedder + class_predictor + box_predictor`)
    with startup parity verification and WAL-mode SQLite resume cache.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sqlite3
from typing import Any

import numpy as np
import torch
from torch import Tensor
from torchvision.ops import box_iou, nms
from transformers import Owlv2ForObjectDetection, Owlv2Processor
from transformers.image_transforms import center_to_corners_format
from transformers.models.owlv2.image_processing_owlv2 import _scale_boxes

from .ontology import SOURCE_ANNOTATED_CATEGORIES, map_to_canonical
from .vocabulary import DEFAULT_CATEGORY_SYNONYMS, VocabularyBank

OWLV2_MODEL_ID: str = "google/owlv2-large-patch14-ensemble"
OWLV2_MODEL_REVISION: str = "95e26936e865f87db1742128404b3c035d47d89d"

OWLV2_DETECTION_PROMPTS: tuple[tuple[str, str], ...] = (
    # Vehicles & People
    ("car", "car"),
    ("truck", "truck"),
    ("bus", "bus"),
    ("person", "pedestrian"),
    ("bicycle", "cyclist"),
    ("motorcycle", "cyclist"),
    ("stroller", "stroller"),
    ("wheelchair", "wheelchair"),
    ("dog", "dog"),
    # Road Infra & Signs
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
    ("bollard", "pole"),
    ("metal bollard", "pole"),
    ("fire hydrant", "fire_hydrant"),
    ("bench", "bench"),
    ("signboard", "signboard"),
    ("fence", "fence"),
    # Architecture & Indoor/Objects
    ("building", "building"),
    ("tree", "tree"),
    ("window", "window"),
    ("door", "door"),
    ("pillar", "pillar"),
    ("concrete column", "pillar"),
    ("building column", "pillar"),
    ("chair", "chair"),
    ("table", "table"),
    ("cabinet", "cabinet"),
    ("luggage", "luggage_and_bags"),
    ("shopping cart", "cart"),
)

OWLV2_STUFF_CATEGORIES: frozenset[str] = frozenset({"building", "tree", "fence"})
OWLV2_DEFAULT_THRESH: float = 0.28
OWLV2_CLASS_THRESHOLDS: dict[str, float] = {
    "construction_barrier": 0.34,
    "pillar": 0.26,
    "fence": 0.32,
    "building": 0.30,
    "tree": 0.30,
}
OWLV2_MAX_STUFF_AREA: float = 0.50
WOODSCAPE_EGO_HOOD_MIN_Y_FRAC: float = 0.58
WOODSCAPE_EGO_HOOD_MIN_AREA_FRAC: float = 0.12
CROWD_PED_MIN_SCORE: float = 0.27
OWLV2_PATCH_BOLLARD_PILLAR_PROMPTS: tuple[tuple[str, str], ...] = (
    ("bollard", "pole"),
    ("metal bollard", "pole"),
    ("pillar", "pillar"),
    ("concrete column", "pillar"),
    ("building column", "pillar"),
)
OWLV2_PATCH_CLASS_THRESHOLDS: dict[str, float] = {
    "pole": 0.28,
    "pillar": 0.26,
}
OWLV2_PATCH_MAX_AREA_FRAC: dict[str, float] = {
    "pole": 0.15,
    "pillar": 0.25,
}

OWLV2_SYNONYM_OVERRIDES: dict[str, tuple[str, ...]] = {
    "construction_barrier": (
        "orange road barricade",
        "plastic traffic jersey barrier",
        "road barricade",
        "construction barrier",
    ),
    "trash_bin": (
        "trash bin",
        "garbage can",
        "waste container",
        "litter bin",
    ),
}


@dataclass(frozen=True)
class OwlDetection:
    """Single post-NMS detection from OWLv2 in native image pixel coordinates."""

    box: tuple[float, float, float, float]
    label: str
    score: float
    patch_index: int


@dataclass(frozen=True)
class MergedImageTargets:
    """Result of merging cleaned Human GT with OWLv2 crowd splits & missing classes."""

    superseded_gt_indices: tuple[int, ...]
    added_owl_detections: tuple[OwlDetection, ...]
    crowd_split_owl_mask: tuple[bool, ...]
    stats: dict[str, int]


@dataclass(frozen=True)
class CachedOwlImageRecord:
    """Deserialized per-image cache entry from `owlv2_teacher_12k.sqlite`."""

    image_id: int
    source_dataset: str
    width: int
    height: int
    superseded_gt_indices: tuple[int, ...]
    gt_match_ious: np.ndarray
    gt_embeddings: np.ndarray
    owl_boxes: np.ndarray
    owl_labels: tuple[str, ...]
    owl_scores: np.ndarray
    owl_crowd_mask: np.ndarray
    owl_embeddings: np.ndarray
    stats: dict[str, int]


def _box_area(b: Sequence[float]) -> float:
    return max(0.0, float(b[2]) - float(b[0])) * max(0.0, float(b[3]) - float(b[1]))


def _inter_area(a: Sequence[float], b: Sequence[float]) -> float:
    ix1 = max(float(a[0]), float(b[0]))
    iy1 = max(float(a[1]), float(b[1]))
    ix2 = min(float(a[2]), float(b[2]))
    iy2 = min(float(a[3]), float(b[3]))
    return max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)


def _iou(a: Sequence[float], b: Sequence[float]) -> float:
    inter = _inter_area(a, b)
    if inter <= 0.0:
        return 0.0
    union = _box_area(a) + _box_area(b) - inter
    return inter / max(union, 1e-6)


def _iomin(a: Sequence[float], b: Sequence[float]) -> float:
    inter = _inter_area(a, b)
    if inter <= 0.0:
        return 0.0
    return inter / max(min(_box_area(a), _box_area(b)), 1e-6)


def merge_clean_gt_and_owlv2(
    gt_boxes: Sequence[Sequence[float]],
    gt_canonical_labels: Sequence[str | None],
    owl_detections: Sequence[OwlDetection],
    source_dataset: str,
    img_w: int,
    img_h: int,
) -> MergedImageTargets:
    """Merge cleaned Human GT with OWLv2 crowd-split pedestrians and missing-class boxes.

    /// @brief Replaces group-labeled pedestrian GT boxes with individual OWLv2 pedestrians
    /// when strict geometric invariants hold, then adds high-confidence OWLv2 detections for
    /// classes unannotated by `source_dataset`.
    /// @param gt_boxes Kept cleaned Human GT boxes in native `[x1, y1, x2, y2]` coordinates.
    /// @param gt_canonical_labels Canonical category names aligned 1-to-1 with `gt_boxes`.
    /// @param owl_detections Post-NMS OWLv2 detections sorted descending by score.
    /// @param source_dataset Canonical source dataset identifier.
    /// @param img_w Native image width in pixels.
    /// @param img_h Native image height in pixels.
    /// @return MergedImageTargets specifying superseded GT indices and added OWLv2 detections.
    """
    superseded_gt: list[int] = []
    claimed_owl_indices: set[int] = set()
    added_detections: list[OwlDetection] = []
    crowd_mask: list[bool] = []
    stats: dict[str, int] = {
        "missing_class_added": 0,
        "crowd_gt_split": 0,
        "crowd_owl_peds_added": 0,
        "filtered_stuff_area": 0,
        "filtered_woodscape_hood": 0,
        "filtered_gt_overlap": 0,
    }

    # Step A: Pedestrian Crowd-Split Replacement
    ped_owl_indices = sorted(
        [
            idx
            for idx, d in enumerate(owl_detections)
            if d.label == "pedestrian" and float(d.score) >= CROWD_PED_MIN_SCORE
        ],
        key=lambda i: -float(owl_detections[i].score),
    )

    for g_idx, (g_box, g_lbl) in enumerate(
        zip(gt_boxes, gt_canonical_labels, strict=True)
    ):
        if g_lbl != "pedestrian":
            continue
        g_w = max(0.0, float(g_box[2]) - float(g_box[0]))
        g_h = max(0.0, float(g_box[3]) - float(g_box[1]))
        g_area = g_w * g_h
        if g_w <= 1.0 or g_h <= 1.0:
            continue

        cands: list[int] = []
        for o_idx in ped_owl_indices:
            if o_idx in claimed_owl_indices:
                continue
            ob = owl_detections[o_idx].box
            o_w = max(0.0, float(ob[2]) - float(ob[0]))
            o_h = max(0.0, float(ob[3]) - float(ob[1]))
            o_area = o_w * o_h
            if (
                _iomin(ob, g_box) >= 0.70
                and o_area <= 0.65 * g_area
                and o_h >= 0.45 * g_h
                and o_w <= 0.72 * g_w
            ):
                cands.append(o_idx)

        kept_cands: list[int] = []
        for o_idx in cands:
            ob = owl_detections[o_idx].box
            if any(_iou(ob, owl_detections[k].box) >= 0.35 for k in kept_cands):
                continue
            kept_cands.append(o_idx)

        if len(kept_cands) >= 2:
            cxs = [0.5 * (owl_detections[k].box[0] + owl_detections[k].box[2]) for k in kept_cands]
            if (max(cxs) - min(cxs)) >= 0.25 * g_w:
                superseded_gt.append(g_idx)
                stats["crowd_gt_split"] += 1
                stats["crowd_owl_peds_added"] += len(kept_cands)
                for k in kept_cands:
                    claimed_owl_indices.add(k)
                    added_detections.append(owl_detections[k])
                    crowd_mask.append(True)

    # Step B: Missing-Class Pseudo-Label Addition
    superseded_set = set(superseded_gt)
    remaining_gt = [
        (gt_boxes[i], gt_canonical_labels[i])
        for i in range(len(gt_boxes))
        if i not in superseded_set
    ]
    annotated = SOURCE_ANNOTATED_CATEGORIES.get(source_dataset, frozenset())
    img_area = float(max(img_w * img_h, 1))
    img_h_f = float(max(img_h, 1))

    ordered_owl_indices = sorted(
        range(len(owl_detections)),
        key=lambda i: -float(owl_detections[i].score),
    )
    for o_idx in ordered_owl_indices:
        if o_idx in claimed_owl_indices:
            continue
        det = owl_detections[o_idx]
        min_thresh = OWLV2_CLASS_THRESHOLDS.get(det.label, OWLV2_DEFAULT_THRESH)
        if float(det.score) < min_thresh:
            continue

        area_frac = _box_area(det.box) / img_area
        cy_frac = (0.5 * (det.box[1] + det.box[3])) / img_h_f

        if det.label in OWLV2_STUFF_CATEGORIES and area_frac > OWLV2_MAX_STUFF_AREA:
            stats["filtered_stuff_area"] += 1
            continue

        if (
            source_dataset == "woodscape_rgb_fisheye"
            and det.label in {"construction_barrier", "fence", "building"}
            and cy_frac >= WOODSCAPE_EGO_HOOD_MIN_Y_FRAC
            and area_frac >= WOODSCAPE_EGO_HOOD_MIN_AREA_FRAC
        ):
            stats["filtered_woodscape_hood"] += 1
            continue

        if det.label in annotated:
            continue

        max_same_iou = max(
            (_iou(det.box, gb) for gb, gl in remaining_gt if gl == det.label),
            default=0.0,
        )
        max_any_iou = max((_iou(det.box, gb) for gb, _gl in remaining_gt), default=0.0)
        if max_same_iou >= 0.35 or max_any_iou >= 0.55:
            stats["filtered_gt_overlap"] += 1
            continue

        dup_same = max(
            (_iou(det.box, prev.box) for prev in added_detections if prev.label == det.label),
            default=0.0,
        )
        dup_any = max((_iou(det.box, prev.box) for prev in added_detections), default=0.0)
        if dup_same >= 0.45 or dup_any >= 0.55:
            continue

        added_detections.append(det)
        crowd_mask.append(False)
        stats["missing_class_added"] += 1

    return MergedImageTargets(
        superseded_gt_indices=tuple(superseded_gt),
        added_owl_detections=tuple(added_detections),
        crowd_split_owl_mask=tuple(crowd_mask),
        stats=stats,
    )


def merge_patch_owl_detections(
    gt_boxes: Sequence[Sequence[float]],
    gt_canonical_labels: Sequence[str | None],
    existing_owl_boxes: Sequence[Sequence[float]],
    existing_owl_labels: Sequence[str],
    patch_owl_detections: Sequence[OwlDetection],
    source_dataset: str,
    img_w: int,
    img_h: int,
) -> tuple[tuple[OwlDetection, ...], dict[str, int]]:
    """Filter and deduplicate targeted bollard/pillar OWLv2 patch detections.

    /// @brief Filters targeted patch detections by score threshold, source dataset
    /// annotation coverage, area/WoodScape ego-hood guards, and overlap against
    /// existing clean GT and already-cached OWLv2 boxes.
    /// @param gt_boxes Kept cleaned Human GT boxes (excluding superseded rows) in pixel `[x1, y1, x2, y2]`.
    /// @param gt_canonical_labels Canonical category names aligned 1-to-1 with `gt_boxes`.
    /// @param existing_owl_boxes Already-cached OWLv2 boxes in native pixel `[x1, y1, x2, y2]`.
    /// @param existing_owl_labels Canonical category names aligned 1-to-1 with `existing_owl_boxes`.
    /// @param patch_owl_detections Candidate post-NMS detections from the targeted patch pass.
    /// @param source_dataset Canonical source dataset identifier.
    /// @param img_w Native image width in pixels.
    /// @param img_h Native image height in pixels.
    /// @return Tuple of `(accepted_detections, patch_filter_stats)`.
    """
    annotated = SOURCE_ANNOTATED_CATEGORIES.get(source_dataset, frozenset())
    stats: dict[str, int] = {
        "patch_added_pole": 0,
        "patch_added_pillar": 0,
        "patch_filtered_area": 0,
        "patch_filtered_woodscape_hood": 0,
        "patch_filtered_gt_overlap": 0,
        "patch_filtered_owl_overlap": 0,
    }
    gt_pairs = list(zip(gt_boxes, gt_canonical_labels, strict=True))
    existing_plus_added: list[tuple[Sequence[float], str]] = list(
        zip(existing_owl_boxes, existing_owl_labels, strict=True)
    )
    img_area = float(max(img_w * img_h, 1))
    img_h_f = float(max(img_h, 1))

    accepted: list[OwlDetection] = []
    for det in sorted(patch_owl_detections, key=lambda d: -float(d.score)):
        thresh = OWLV2_PATCH_CLASS_THRESHOLDS.get(det.label)
        if thresh is None or float(det.score) < thresh:
            continue
        if det.label in annotated:
            continue

        area_frac = _box_area(det.box) / img_area
        if area_frac > OWLV2_PATCH_MAX_AREA_FRAC[det.label]:
            stats["patch_filtered_area"] += 1
            continue

        cy_frac = (0.5 * (det.box[1] + det.box[3])) / img_h_f
        if (
            source_dataset == "woodscape_rgb_fisheye"
            and cy_frac >= WOODSCAPE_EGO_HOOD_MIN_Y_FRAC
            and area_frac >= WOODSCAPE_EGO_HOOD_MIN_AREA_FRAC
        ):
            stats["patch_filtered_woodscape_hood"] += 1
            continue

        max_gt_same = max(
            (_iou(det.box, gb) for gb, gl in gt_pairs if gl == det.label),
            default=0.0,
        )
        max_gt_any = max((_iou(det.box, gb) for gb, _gl in gt_pairs), default=0.0)
        if max_gt_same >= 0.35 or max_gt_any >= 0.55:
            stats["patch_filtered_gt_overlap"] += 1
            continue

        max_owl_same = max(
            (_iou(det.box, ob) for ob, ol in existing_plus_added if ol == det.label),
            default=0.0,
        )
        max_owl_any = max((_iou(det.box, ob) for ob, _ol in existing_plus_added), default=0.0)
        if max_owl_same >= 0.45 or max_owl_any >= 0.55:
            stats["patch_filtered_owl_overlap"] += 1
            continue

        accepted.append(det)
        existing_plus_added.append((det.box, det.label))
        stats[f"patch_added_{det.label}"] += 1

    return tuple(accepted), stats


def build_owlv2_vocabulary_bank(
    model: Owlv2ForObjectDetection,
    processor: Owlv2Processor,
    categories: Sequence[str],
    device: torch.device,
    *,
    synonyms: dict[str, Sequence[str]] | None = None,
) -> VocabularyBank:
    """Build a 768-D VocabularyBank from OWLv2-Large's native text encoder.

    /// @param model Loaded Owlv2ForObjectDetection instance.
    /// @param processor Matching Owlv2Processor tokenizer/image processor.
    /// @param categories Ordered sequence of canonical category names.
    /// @param device Execution device for encoding.
    /// @param synonyms Optional custom synonym dictionary per category.
    /// @return VocabularyBank aligned 1-to-1 with OWLv2's `class_embeds` space.
    """
    merged_synonyms: dict[str, Sequence[str]] = {**DEFAULT_CATEGORY_SYNONYMS, **OWLV2_SYNONYM_OVERRIDES}
    if synonyms is not None:
        merged_synonyms.update(synonyms)

    cat_list = tuple(categories)
    all_phrases: list[str] = []
    spans: list[tuple[int, int]] = []
    for cat in cat_list:
        syns = list(merged_synonyms.get(cat, (cat.replace("_", " "),)))
        start = len(all_phrases)
        all_phrases.extend(syns)
        spans.append((start, len(all_phrases)))

    with torch.no_grad():
        phrase_vecs: list[Tensor] = []
        chunk_size = 64
        for s_idx in range(0, len(all_phrases), chunk_size):
            chunk = all_phrases[s_idx : s_idx + chunk_size]
            inp = processor(text=chunk, return_tensors="pt", padding=True)
            inp_dev = {k: v.to(device) for k, v in inp.items()}
            txt_out = model.owlv2.get_text_features(**inp_dev)
            feats = txt_out.pooler_output if hasattr(txt_out, "pooler_output") else txt_out
            feats_f = feats.float()
            feats_f = feats_f / feats_f.norm(dim=-1, keepdim=True).clamp(min=1e-6)
            phrase_vecs.append(feats_f.cpu())
        all_vecs = torch.cat(phrase_vecs, dim=0)

        protos: list[Tensor] = []
        for start, end in spans:
            mean_v = all_vecs[start:end].mean(dim=0)
            protos.append(mean_v / mean_v.norm().clamp(min=1e-6))

    bank_tensor = torch.stack(protos, dim=0).to(torch.float32)
    return VocabularyBank(
        categories=cat_list,
        embeddings=bank_tensor,
        logit_scale=math.log(20.0),
        logit_bias=-5.0,
    )


class Owlv2TeacherExtractor:
    """Fast batched OWLv2-Large teacher extractor with precomputed text queries."""

    def __init__(
        self,
        device: torch.device | str = "cuda",
        *,
        model_id: str = OWLV2_MODEL_ID,
        revision: str | None = OWLV2_MODEL_REVISION,
        prompts: Sequence[tuple[str, str]] = OWLV2_DETECTION_PROMPTS,
        dtype: torch.dtype | None = None,
    ) -> None:
        self.device = torch.device(device)
        self.model_id = model_id
        self.revision = revision
        self.prompts = tuple(prompts)
        self.phrases = [p for p, _ in self.prompts]
        self.canon_by_qidx = [c for _, c in self.prompts]
        self.dtype = (
            dtype
            if dtype is not None
            else (torch.float16 if self.device.type == "cuda" else torch.float32)
        )

        self.processor = Owlv2Processor.from_pretrained(model_id, revision=revision)
        self.model = Owlv2ForObjectDetection.from_pretrained(
            model_id, revision=revision, torch_dtype=self.dtype
        ).to(self.device)
        self.model.eval()
        for param in self.model.parameters():
            param.requires_grad = False

        self._parity_checked = False
        with torch.no_grad():
            dummy_rgb = np.zeros((32, 32, 3), dtype=np.uint8)
            inp = self.processor(text=[self.phrases], images=[dummy_rgb], return_tensors="pt")
            inp_dev = {
                k: (
                    v.to(device=self.device, dtype=self.dtype)
                    if v.is_floating_point()
                    else v.to(device=self.device)
                )
                for k, v in inp.items()
            }
            out = self.model(**inp_dev)
            self.query_embeds_single = out.text_embeds[0:1].detach()
            q_ids = inp_dev["input_ids"].reshape(1, len(self.phrases), -1)
            self.query_mask_single = (q_ids[..., 0] > 0).detach()
            self._init_input_ids = inp_dev["input_ids"].detach()
            self._init_attention_mask = inp_dev["attention_mask"].detach()

    def build_vocabulary_bank(self, categories: Sequence[str]) -> VocabularyBank:
        """Build native OWLv2 text VocabularyBank reusing this loaded model."""
        return build_owlv2_vocabulary_bank(
            self.model,
            self.processor,
            categories,
            self.device,
        )

    @torch.no_grad()
    def extract_batch(
        self,
        pixel_values: Tensor,
        target_sizes: Sequence[tuple[int, int]],
        gt_boxes_batch: Sequence[Sequence[Sequence[float]]],
        *,
        min_score: float = 0.25,
        per_class_iou: float = 0.45,
        cross_class_iou: float = 0.75,
    ) -> list[dict[str, Any]]:
        """Run batched OWLv2 vision + box/class heads and extract per-box 768-D embeddings."""
        bsz = int(pixel_values.shape[0])
        pv = pixel_values.to(device=self.device, dtype=self.dtype, non_blocking=True)
        feature_map, _ = self.model.image_embedder(pixel_values=pv)
        _, h_p, w_p, d_h = feature_map.shape
        image_feats = feature_map.reshape(bsz, h_p * w_p, d_h)

        q_emb = self.query_embeds_single.expand(bsz, -1, -1)
        q_mask = self.query_mask_single.expand(bsz, -1)
        pred_logits, class_embeds = self.model.class_predictor(image_feats, q_emb, q_mask)
        pred_boxes = self.model.box_predictor(image_feats, feature_map)

        if not self._parity_checked:
            full_out = self.model(
                pixel_values=pv,
                input_ids=self._init_input_ids.repeat(bsz, 1),
                attention_mask=self._init_attention_mask.repeat(bsz, 1),
            )
            diff_logits = float((full_out.logits - pred_logits).abs().max().item())
            diff_probs = float(
                (full_out.logits.float().sigmoid() - pred_logits.float().sigmoid())
                .abs()
                .max()
                .item()
            )
            diff_boxes = float((full_out.pred_boxes - pred_boxes).abs().max().item())
            if diff_boxes >= 1e-3 or diff_probs >= 5e-3 or diff_logits >= 0.25:
                raise RuntimeError(
                    "OWLv2 modular head parity check failed: "
                    f"diff_logits={diff_logits:.5f}, diff_probs={diff_probs:.6f}, diff_boxes={diff_boxes:.5f}"
                )
            self._parity_checked = True

        target_tensor = torch.tensor(list(target_sizes), device=self.device, dtype=torch.float32)
        corners = center_to_corners_format(pred_boxes.float())
        scaled_boxes_all = _scale_boxes(corners, target_tensor)

        results: list[dict[str, Any]] = []
        for b_idx in range(bsz):
            img_h, img_w = int(target_sizes[b_idx][0]), int(target_sizes[b_idx][1])
            boxes_b = scaled_boxes_all[b_idx].clone()
            boxes_b[:, 0::2].clamp_(0.0, float(img_w))
            boxes_b[:, 1::2].clamp_(0.0, float(img_h))

            ce_b = class_embeds[b_idx].float()
            ce_b = ce_b / ce_b.norm(dim=-1, keepdim=True).clamp(min=1e-6)
            ce_b_fp16 = ce_b.to(torch.float16).cpu()

            logits_b = pred_logits[b_idx].float()
            max_logits, best_q = logits_b.max(dim=-1)
            scores_b = torch.sigmoid(max_logits)

            cand_mask = scores_b >= min_score
            bw = boxes_b[:, 2] - boxes_b[:, 0]
            bh = boxes_b[:, 3] - boxes_b[:, 1]
            cand_mask = cand_mask & (bw >= 4.0) & (bh >= 4.0)

            cand_indices = torch.where(cand_mask)[0]
            if cand_indices.numel() > 0:
                c_boxes = boxes_b[cand_indices].cpu()
                c_scores = scores_b[cand_indices].cpu()
                c_qidxs = best_q[cand_indices].cpu().tolist()
                c_patches = cand_indices.cpu().tolist()
                c_labels = [self.canon_by_qidx[qi] for qi in c_qidxs]

                by_cls: dict[str, list[int]] = defaultdict(list)
                for loc_i, lbl in enumerate(c_labels):
                    by_cls[lbl].append(loc_i)

                stage1: list[int] = []
                for locs in by_cls.values():
                    lt = torch.tensor(locs, dtype=torch.long)
                    keep_l = nms(c_boxes[lt], c_scores[lt], per_class_iou)
                    stage1.extend(lt[keep_l].tolist())

                stage1.sort(key=lambda li: -float(c_scores[li].item()))
                keep_cross = nms(c_boxes[stage1], c_scores[stage1], cross_class_iou)
                final_locs = [stage1[k] for k in keep_cross.tolist()]

                owl_dets = [
                    OwlDetection(
                        box=tuple(round(float(v), 2) for v in c_boxes[li].tolist()),  # type: ignore[arg-type]
                        label=c_labels[li],
                        score=round(float(c_scores[li].item()), 4),
                        patch_index=int(c_patches[li]),
                    )
                    for li in final_locs
                ]
                kept_patch_t = torch.tensor(
                    [d.patch_index for d in owl_dets], dtype=torch.long
                )
                owl_embeds = ce_b_fp16[kept_patch_t].numpy()
            else:
                owl_dets = []
                owl_embeds = np.zeros((0, 768), dtype=np.float16)

            gt_list = list(gt_boxes_batch[b_idx])
            if gt_list:
                gt_t = torch.tensor(gt_list, dtype=torch.float32, device=self.device)
                iou_mat = box_iou(gt_t, boxes_b)
                best_ious, best_patches = iou_mat.max(dim=1)
                gt_match_ious = best_ious.detach().cpu().to(torch.float16).numpy()
                gt_embeds = ce_b_fp16[best_patches.detach().cpu()].numpy()
            else:
                gt_match_ious = np.zeros((0,), dtype=np.float16)
                gt_embeds = np.zeros((0, 768), dtype=np.float16)

            results.append(
                {
                    "owl_detections": owl_dets,
                    "owl_embeddings": owl_embeds,
                    "gt_match_ious": gt_match_ious,
                    "gt_embeddings": gt_embeds,
                }
            )

        return results


class OwlTeacherCacheWriter:
    """Crash-resumable WAL SQLite writer for OWLv2 boxes & 768-D embeddings."""

    def __init__(self, db_path: Path | str) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.db_path)
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.execute("PRAGMA synchronous=NORMAL")
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            )
            """
        )
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS processed_images (
                image_id INTEGER PRIMARY KEY,
                source_dataset TEXT NOT NULL,
                width INTEGER NOT NULL,
                height INTEGER NOT NULL,
                superseded_gt_json TEXT NOT NULL,
                gt_match_ious_blob BLOB NOT NULL,
                gt_embeddings_blob BLOB NOT NULL,
                num_gt INTEGER NOT NULL,
                owl_boxes_blob BLOB NOT NULL,
                owl_labels_json TEXT NOT NULL,
                owl_scores_blob BLOB NOT NULL,
                owl_crowd_mask_blob BLOB NOT NULL,
                owl_embeddings_blob BLOB NOT NULL,
                num_owl INTEGER NOT NULL,
                stats_json TEXT NOT NULL
            )
            """
        )
        self.conn.commit()

    def set_metadata(self, key: str, value: str) -> None:
        self.conn.execute(
            "INSERT OR REPLACE INTO metadata (key, value) VALUES (?, ?)",
            (key, value),
        )
        self.conn.commit()

    def get_completed_ids(self) -> set[int]:
        return {
            int(r[0])
            for r in self.conn.execute("SELECT image_id FROM processed_images").fetchall()
        }

    def get_image(self, image_id: int) -> CachedOwlImageRecord | None:
        """Fetch and decode one cached image record from the writable connection.

        /// @param image_id Integer image identifier in `processed_images`.
        /// @return Decoded CachedOwlImageRecord, or None if not present.
        """
        r = self.conn.execute(
            """
            SELECT image_id, source_dataset, width, height,
                   superseded_gt_json, gt_match_ious_blob, gt_embeddings_blob, num_gt,
                   owl_boxes_blob, owl_labels_json, owl_scores_blob, owl_crowd_mask_blob,
                   owl_embeddings_blob, num_owl, stats_json
            FROM processed_images
            WHERE image_id = ?
            """,
            (int(image_id),),
        ).fetchone()
        return OwlTeacherCacheReader._decode_row(r) if r is not None else None

    def append_owl_detections(
        self,
        *,
        image_id: int,
        added_detections: Sequence[OwlDetection],
        added_embeddings: np.ndarray,
        extra_stats: dict[str, int] | None = None,
    ) -> int:
        """Append new OWLv2 detections & FP16 embeddings to an existing cached image row.

        /// @param image_id Existing `processed_images.image_id` to update in place.
        /// @param added_detections Newly accepted OwlDetection entries to append.
        /// @param added_embeddings Float array of shape `[N, 768]` aligned with `added_detections`.
        /// @param extra_stats Optional counter deltas merged into `stats_json`.
        /// @return Number of newly appended OWLv2 detections.
        """
        rec = self.get_image(image_id)
        if rec is None:
            raise KeyError(f"Cached image_id={image_id} not found in {self.db_path}")

        n_add = len(added_detections)
        if n_add == 0 and not extra_stats:
            return 0

        emb_arr = np.asarray(added_embeddings, dtype=np.float32)
        if emb_arr.shape != (n_add, 768):
            raise ValueError(
                f"Expected added_embeddings shape ({n_add}, 768), got {emb_arr.shape}"
            )
        if not np.isfinite(emb_arr).all():
            raise ValueError(f"Non-finite values found in added_embeddings for image_id={image_id}")

        if n_add > 0:
            new_boxes = np.asarray([d.box for d in added_detections], dtype=np.float32)
            if not np.isfinite(new_boxes).all():
                raise ValueError(f"Non-finite values found in added boxes for image_id={image_id}")
            new_labels = [d.label for d in added_detections]
            new_scores = np.asarray([d.score for d in added_detections], dtype=np.float32)
            new_crowd = np.zeros((n_add,), dtype=np.uint8)
            new_embeds = np.ascontiguousarray(emb_arr, dtype=np.float16)

            if rec.owl_boxes.shape[0] > 0:
                owl_boxes = np.concatenate([rec.owl_boxes.astype(np.float32), new_boxes], axis=0)
                owl_labels = list(rec.owl_labels) + new_labels
                owl_scores = np.concatenate([rec.owl_scores.astype(np.float32), new_scores], axis=0)
                owl_crowd = np.concatenate(
                    [rec.owl_crowd_mask.astype(np.uint8), new_crowd], axis=0
                )
                owl_embeds = np.concatenate([rec.owl_embeddings.astype(np.float16), new_embeds], axis=0)
            else:
                owl_boxes = new_boxes
                owl_labels = new_labels
                owl_scores = new_scores
                owl_crowd = new_crowd
                owl_embeds = new_embeds
        else:
            owl_boxes = rec.owl_boxes.astype(np.float32)
            owl_labels = list(rec.owl_labels)
            owl_scores = rec.owl_scores.astype(np.float32)
            owl_crowd = rec.owl_crowd_mask.astype(np.uint8)
            owl_embeds = rec.owl_embeddings.astype(np.float16)

        stats = dict(rec.stats)
        if n_add > 0:
            stats["missing_class_added"] = int(stats.get("missing_class_added", 0)) + n_add
        if extra_stats:
            for k, v in extra_stats.items():
                stats[k] = int(stats.get(k, 0)) + int(v)

        self.conn.execute(
            """
            UPDATE processed_images
            SET owl_boxes_blob = ?,
                owl_labels_json = ?,
                owl_scores_blob = ?,
                owl_crowd_mask_blob = ?,
                owl_embeddings_blob = ?,
                num_owl = ?,
                stats_json = ?
            WHERE image_id = ?
            """,
            (
                np.ascontiguousarray(owl_boxes, dtype=np.float32).tobytes(),
                json.dumps(owl_labels),
                np.ascontiguousarray(owl_scores, dtype=np.float32).tobytes(),
                np.ascontiguousarray(owl_crowd, dtype=np.uint8).tobytes(),
                np.ascontiguousarray(owl_embeds, dtype=np.float16).tobytes(),
                int(owl_boxes.shape[0]),
                json.dumps(stats),
                int(image_id),
            ),
        )
        return n_add

    def write_image(
        self,
        *,
        image_id: int,
        source_dataset: str,
        width: int,
        height: int,
        merged: MergedImageTargets,
        gt_match_ious: np.ndarray,
        gt_embeddings: np.ndarray,
        all_owl_detections: Sequence[OwlDetection],
        all_owl_embeddings: np.ndarray,
    ) -> None:
        """Serialize one image's kept GT embeddings & merged OWLv2 targets."""
        patch_to_row = {det.patch_index: i for i, det in enumerate(all_owl_detections)}
        added = merged.added_owl_detections
        if added:
            owl_boxes = np.asarray([d.box for d in added], dtype=np.float32)
            owl_labels = [d.label for d in added]
            owl_scores = np.asarray([d.score for d in added], dtype=np.float32)
            owl_crowd = np.asarray(merged.crowd_split_owl_mask, dtype=np.uint8)
            row_idxs = [patch_to_row[d.patch_index] for d in added]
            owl_embeds = np.ascontiguousarray(all_owl_embeddings[row_idxs], dtype=np.float16)
        else:
            owl_boxes = np.zeros((0, 4), dtype=np.float32)
            owl_labels = []
            owl_scores = np.zeros((0,), dtype=np.float32)
            owl_crowd = np.zeros((0,), dtype=np.uint8)
            owl_embeds = np.zeros((0, 768), dtype=np.float16)

        gt_ious_arr = np.ascontiguousarray(gt_match_ious, dtype=np.float16)
        gt_emb_arr = np.ascontiguousarray(gt_embeddings, dtype=np.float16)
        num_gt = int(gt_emb_arr.shape[0])
        num_owl = int(owl_boxes.shape[0])

        self.conn.execute(
            """
            INSERT OR REPLACE INTO processed_images (
                image_id, source_dataset, width, height,
                superseded_gt_json, gt_match_ious_blob, gt_embeddings_blob, num_gt,
                owl_boxes_blob, owl_labels_json, owl_scores_blob, owl_crowd_mask_blob,
                owl_embeddings_blob, num_owl, stats_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                int(image_id),
                source_dataset,
                int(width),
                int(height),
                json.dumps(list(merged.superseded_gt_indices)),
                gt_ious_arr.tobytes(),
                gt_emb_arr.tobytes(),
                num_gt,
                owl_boxes.tobytes(),
                json.dumps(owl_labels),
                owl_scores.tobytes(),
                owl_crowd.tobytes(),
                owl_embeds.tobytes(),
                num_owl,
                json.dumps(merged.stats),
            ),
        )

    def commit(self) -> None:
        self.conn.commit()

    def close(self, *, checkpoint_wal: bool = False) -> None:
        """Commit pending transactions and optionally flatten WAL journal back into DB."""
        self.conn.commit()
        if checkpoint_wal:
            self.conn.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            self.conn.execute("PRAGMA journal_mode=DELETE")
        self.conn.close()


class OwlTeacherCacheReader:
    """Read-only accessor for `owlv2_teacher_12k.sqlite`."""

    def __init__(self, db_path: Path | str) -> None:
        self.db_path = Path(db_path)
        self.conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)

    def count(self) -> int:
        row = self.conn.execute("SELECT COUNT(*) FROM processed_images").fetchone()
        return int(row[0]) if row else 0

    def get_image_ids(self) -> list[int]:
        return [
            int(r[0])
            for r in self.conn.execute(
                "SELECT image_id FROM processed_images ORDER BY image_id"
            ).fetchall()
        ]

    @staticmethod
    def _decode_row(r: tuple[Any, ...]) -> CachedOwlImageRecord:
        num_gt = int(r[7])
        num_owl = int(r[13])
        gt_ious = np.frombuffer(r[5], dtype=np.float16).reshape(num_gt).copy()
        gt_embeds = np.frombuffer(r[6], dtype=np.float16).reshape(num_gt, 768).copy()
        owl_boxes = np.frombuffer(r[8], dtype=np.float32).reshape(num_owl, 4).copy()
        owl_labels = tuple(str(x) for x in json.loads(str(r[9])))
        owl_scores = np.frombuffer(r[10], dtype=np.float32).reshape(num_owl).copy()
        owl_crowd = np.frombuffer(r[11], dtype=np.uint8).reshape(num_owl).astype(bool).copy()
        owl_embeds = np.frombuffer(r[12], dtype=np.float16).reshape(num_owl, 768).copy()
        return CachedOwlImageRecord(
            image_id=int(r[0]),
            source_dataset=str(r[1]),
            width=int(r[2]),
            height=int(r[3]),
            superseded_gt_indices=tuple(int(x) for x in json.loads(str(r[4]))),
            gt_match_ious=gt_ious,
            gt_embeddings=gt_embeds,
            owl_boxes=owl_boxes,
            owl_labels=owl_labels,
            owl_scores=owl_scores,
            owl_crowd_mask=owl_crowd,
            owl_embeddings=owl_embeds,
            stats=json.loads(str(r[14])),
        )

    def get_image(self, image_id: int) -> CachedOwlImageRecord | None:
        r = self.conn.execute(
            """
            SELECT image_id, source_dataset, width, height,
                   superseded_gt_json, gt_match_ious_blob, gt_embeddings_blob, num_gt,
                   owl_boxes_blob, owl_labels_json, owl_scores_blob, owl_crowd_mask_blob,
                   owl_embeddings_blob, num_owl, stats_json
            FROM processed_images
            WHERE image_id = ?
            """,
            (int(image_id),),
        ).fetchone()
        return self._decode_row(r) if r is not None else None

    def iter_records(self) -> Any:
        cursor = self.conn.execute(
            """
            SELECT image_id, source_dataset, width, height,
                   superseded_gt_json, gt_match_ious_blob, gt_embeddings_blob, num_gt,
                   owl_boxes_blob, owl_labels_json, owl_scores_blob, owl_crowd_mask_blob,
                   owl_embeddings_blob, num_owl, stats_json
            FROM processed_images
            ORDER BY image_id
            """
        )
        for r in cursor:
            yield self._decode_row(r)

    def close(self) -> None:
        self.conn.close()


def canonicalize_gt_labels(raw_names: Sequence[str]) -> list[str | None]:
    """Map raw SQLite annotation names to canonical vocabulary categories."""
    return [map_to_canonical(n) for n in raw_names]


__all__ = [
    "CROWD_PED_MIN_SCORE",
    "CachedOwlImageRecord",
    "MergedImageTargets",
    "OWLV2_CLASS_THRESHOLDS",
    "OWLV2_DEFAULT_THRESH",
    "OWLV2_DETECTION_PROMPTS",
    "OWLV2_MAX_STUFF_AREA",
    "OWLV2_MODEL_ID",
    "OWLV2_MODEL_REVISION",
    "OWLV2_PATCH_BOLLARD_PILLAR_PROMPTS",
    "OWLV2_PATCH_CLASS_THRESHOLDS",
    "OWLV2_PATCH_MAX_AREA_FRAC",
    "OWLV2_STUFF_CATEGORIES",
    "OWLV2_SYNONYM_OVERRIDES",
    "OwlDetection",
    "OwlTeacherCacheReader",
    "OwlTeacherCacheWriter",
    "Owlv2TeacherExtractor",
    "WOODSCAPE_EGO_HOOD_MIN_AREA_FRAC",
    "WOODSCAPE_EGO_HOOD_MIN_Y_FRAC",
    "build_owlv2_vocabulary_bank",
    "canonicalize_gt_labels",
    "merge_clean_gt_and_owlv2",
    "merge_patch_owl_detections",
]
