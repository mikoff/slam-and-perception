"""Run SAM Automatic Mask Generator object discovery pilot on the benchmark cohort.

Extracts dual geometries (axis-aligned bounding boxes and oriented quadrilaterals),
applies containment suppression on GPU, and performs hybrid merging against human
ground-truth annotations.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import signal
import sqlite3
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image
import torch
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

from student_detector.data import _integer_quotas
from student_detector.discovery_storage import (
    get_dataset_summary,
    get_processed_image_ids,
    init_discovery_db,
    save_image_batch,
)
from student_detector.semantic_evaluation_data import (
    SemanticGroundTruthRecord,
    SemanticImageRecord,
    load_semantic_image_records,
)

DEFAULT_COHORT_WEIGHTS: dict[str, float] = {
    "coco_2017": 0.50,
    "nuimages": 0.18,
    "bdd100k_images_100k": 0.12,
    "woodscape_rgb_fisheye": 0.20,
}

TARGET_COHORT_SHA256: str = "e705233ada6e63cb58c259b610417a6226dd9cf158acc7737842ecc758e15806"

STAGE2_COHORT_WEIGHTS: dict[str, float] = {
    "coco_2017": 0.50,
    "bdd100k_images_100k": 0.20,
    "nuimages": 0.20,
    "woodscape_rgb_fisheye": 0.10,
}

STAGE2_COHORT_SHA256: str = "2d35c2fdc1cc9559e735267947f017347eeaed7a79f7eb1254c1f953aeecaf44"


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Run SAM AMG object discovery pilot on benchmark cohort."
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/home/sashamikoff/git/skillup/data/visual-inference-datasets/output"),
        help="Root path to visual inference dataset",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("artifacts/models/sam_vit_b_01ec64.pth"),
        help="Path to SAM ViT-B checkpoint",
    )
    parser.add_argument(
        "--output-db",
        type=Path,
        default=Path("artifacts/pseudo_labels/sam_discovery.sqlite"),
        help="Destination SQLite database for streaming discovered pseudo-labels",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=None,
        help="Optional destination JSON file for discovered pseudo-labels",
    )
    parser.add_argument(
        "--index-db",
        type=Path,
        default=None,
        help="Path to source SQLite index (e.g. quad_train.sqlite or quad_val.sqlite)",
    )
    parser.add_argument(
        "--split",
        type=str,
        default="train",
        help="Split label assigned to processed images in SQLite ('train' or 'val')",
    )
    parser.add_argument(
        "--num-images",
        type=int,
        default=800,
        help="Total number of cohort images to process",
    )
    parser.add_argument(
        "--points-per-side",
        type=int,
        default=16,
        help="Sampling point grid per side for SAM AMG (e.g. 16, 24, 32)",
    )
    parser.add_argument(
        "--points-per-batch",
        type=int,
        default=128,
        help="Number of prompt points evaluated per GPU batch by SAM mask decoder (default 128)",
    )
    parser.add_argument(
        "--pred-iou-thresh",
        type=float,
        default=0.85,
        help="Minimum predicted mask IoU threshold for SAM",
    )
    parser.add_argument(
        "--stability-score-thresh",
        type=float,
        default=0.88,
        help="Minimum mask stability score threshold for SAM",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=256.0,
        help="Minimum mask pixel area floor (default 16x16 = 256 px)",
    )
    parser.add_argument(
        "--containment-thresh",
        type=float,
        default=0.85,
        help="Suppression threshold for child masks contained in parent masks",
    )
    parser.add_argument(
        "--gt-duplicate-iou-thresh",
        type=float,
        default=0.50,
        help="IoU threshold above which a SAM mask duplicates an existing human GT",
    )
    parser.add_argument(
        "--gt-novel-iou-thresh",
        type=float,
        default=0.15,
        help="IoU threshold below which a SAM mask is accepted as a novel discovered object",
    )
    parser.add_argument(
        "--max-parent-child-ratio",
        type=float,
        default=12.0,
        help="Max scale disparity before parent is deemed backdrop (default 12.0)",
    )
    parser.add_argument(
        "--max-backdrop-fraction",
        type=float,
        default=0.15,
        help="Image area fraction above which parent mask is deemed backdrop (default 0.15)",
    )
    parser.add_argument(
        "--min-vertical-aspect",
        type=float,
        default=2.2,
        help="Minimum aspect ratio (H/W) to protect vertical structural elements (default 2.2)",
    )
    parser.add_argument(
        "--min-horizontal-aspect",
        type=float,
        default=2.5,
        help="Minimum aspect ratio (W/H) to protect horizontal road markings (default 2.5)",
    )
    parser.add_argument(
        "--min-element-dim",
        type=float,
        default=25.0,
        help="Minimum dimension in pixels to qualify for aspect protection (default 25.0)",
    )
    parser.add_argument(
        "--max-area-fraction",
        type=float,
        default=0.40,
        help="Maximum image area fraction for a foreground object proposal (default 0.40)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Device to run inference on (cuda or cpu)",
    )
    parser.add_argument(
        "--save-interval",
        type=int,
        default=25,
        help="Durable progress save interval in images",
    )
    parser.add_argument(
        "--all-images",
        action="store_true",
        help="Process ALL images in the index database (full 267.8k dataset rollout)",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=250,
        help="Number of image records to load into memory per streaming batch (default 250)",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Optional upper bound on processed images for quick pilot smoke-testing",
    )
    return parser.parse_args()


def select_cohort_image_ids(
    dataset_root: Path,
    count: int = 800,
    source_weights: dict[str, float] | None = None,
    index_path: Path | None = None,
    all_images: bool = False,
) -> tuple[list[int], str]:
    """Select the deterministic benchmark cohort IDs directly from SQLite.

    /// @param dataset_root Root directory of the dataset.
    /// @param count Total number of cohort images.
    /// @param source_weights Domain proportions.
    /// @param index_path Optional explicit index path.
    /// @param all_images If True, select all images in index_path without quota limits.
    /// @return Tuple of (selected_image_ids, computed_subset_sha256).
    """
    if index_path is not None:
        db_path = index_path
        weights = source_weights or (
            STAGE2_COHORT_WEIGHTS if count > 800 else DEFAULT_COHORT_WEIGHTS
        )
    elif all_images or count > 800 or count <= 0:
        db_path = dataset_root / "indexes/quad_train.sqlite"
        weights = source_weights or STAGE2_COHORT_WEIGHTS
    else:
        db_path = dataset_root / "indexes/quad_val.sqlite"
        weights = source_weights or DEFAULT_COHORT_WEIGHTS

    with sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True) as connection:
        rows = connection.execute(
            "SELECT image_id, source_dataset FROM images ORDER BY row_index"
        ).fetchall()

    if all_images or count <= 0:
        selected_ids = [int(r[0]) for r in rows]
        selected_tuples = [(str(r[1]), int(r[0])) for r in rows]
        identity = json.dumps(selected_tuples, separators=(",", ":")).encode("utf-8")
        computed_sha = hashlib.sha256(identity).hexdigest()
        return selected_ids, computed_sha

    quotas = _integer_quotas(weights, count)

    selected_tuples: list[tuple[str, int]] = []
    selected_ids: list[int] = []
    observed = {k: 0 for k in quotas}

    for img_id, source in rows:
        if source in quotas and observed[source] < quotas[source]:
            selected_tuples.append((source, img_id))
            selected_ids.append(img_id)
            observed[source] += 1
        if len(selected_ids) == count:
            break

    identity = json.dumps(selected_tuples, separators=(",", ":")).encode("utf-8")
    computed_sha = hashlib.sha256(identity).hexdigest()
    return selected_ids, computed_sha


def order_quad_clockwise(points: np.ndarray) -> list[list[float]]:
    """Order quadrilateral vertices clockwise starting from the top-left vertex.

    /// @param points Float array of shape [4, 2].
    /// @return Ordered list of 4 [x, y] coordinates.
    """
    pts = points.astype(np.float32)
    center = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    sort_idx = np.argsort(angles)
    sorted_pts = pts[sort_idx]

    # Find the top-left-most point (minimum x + y)
    sums = sorted_pts[:, 0] + sorted_pts[:, 1]
    tl_idx = int(np.argmin(sums))
    reordered = np.roll(sorted_pts, -tl_idx, axis=0)
    return [[float(round(p[0], 2)), float(round(p[1], 2))] for p in reordered]


def extract_mask_geometries(
    mask_bool: np.ndarray,
    approx_epsilon: float = 1.2,
) -> tuple[list[float], list[list[float]], list[list[float]], float]:
    """Extract axis-aligned bounding box, oriented quad, polygon outline, and area.

    /// @param mask_bool Boolean mask array of shape [H, W].
    /// @param approx_epsilon Douglas-Peucker epsilon for polygon simplification.
    /// @return Tuple of (bbox [x1, y1, x2, y2], quad [[x, y], ...], polygon [[x, y], ...], area).
    """
    mask_uint8 = mask_bool.astype(np.uint8)
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return [0.0, 0.0, 0.0, 0.0], [[0.0, 0.0]] * 4, [], 0.0

    # Primary contour
    c = max(contours, key=cv2.contourArea)
    area = float(np.sum(mask_bool))

    # 1. Axis-Aligned Bounding Box (HBB)
    x, y, w, h = cv2.boundingRect(c)
    bbox = [float(x), float(y), float(x + w), float(y + h)]

    # 2. Oriented Quadrilateral (Quad)
    rect = cv2.minAreaRect(c)
    box_pts = cv2.boxPoints(rect)
    quad = order_quad_clockwise(box_pts)

    # 3. Compact Polygon Outline
    poly_approx = cv2.approxPolyDP(c, epsilon=approx_epsilon, closed=True)
    polygon = [
        [float(round(pt[0][0], 1)), float(round(pt[0][1], 1))]
        for pt in poly_approx
    ]

    return bbox, quad, polygon, area


def suppress_contained_masks_gpu(
    masks: list[dict[str, Any]],
    containment_threshold: float = 0.85,
    max_parent_child_ratio: float = 12.0,
    max_backdrop_fraction: float = 0.15,
    min_vertical_aspect: float = 2.2,
    min_horizontal_aspect: float = 2.5,
    min_element_dim: float = 25.0,
    device: str = "cuda",
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Suppress smaller masks substantially contained in larger masks via GPU tensors.

    Applies scale and aspect ratio protections to preserve independent structural
    elements (columns, pillars, poles) and road markings (lines, curbs, stripes)
    even when enclosed by massive environmental backdrops (buildings, asphalt).

    /// @param masks List of SAM output mask dictionaries.
    /// @param containment_threshold Threshold ratio of intersection over child area.
    /// @param max_parent_child_ratio Maximum scale disparity before parent is deemed backdrop.
    /// @param max_backdrop_fraction Area fraction of image above which parent is backdrop.
    /// @param min_vertical_aspect Aspect ratio (H/W) threshold for vertical structural elements.
    /// @param min_horizontal_aspect Aspect ratio (W/H) threshold for horizontal elements.
    /// @param min_element_dim Minimum pixel dimension for aspect-based element protection.
    /// @param device Device for tensor computation ('cuda' or 'cpu').
    /// @return Tuple of (kept_masks, suppressed_masks).
    """
    if not masks:
        return [], []

    # Sort masks descending by area
    sorted_masks = sorted(masks, key=lambda m: float(m["area"]), reverse=True)
    n = len(sorted_masks)
    if n <= 1:
        return sorted_masks, []

    # Stack on specified device
    mask_tensor = torch.from_numpy(
        np.stack([m["segmentation"] for m in sorted_masks], axis=0)
    ).to(device=device, dtype=torch.bool)

    H, W = mask_tensor.shape[1], mask_tensor.shape[2]
    image_total_area = float(H * W)

    areas = mask_tensor.sum(dim=(1, 2)).float()
    is_suppressed = [False] * n
    suppression_meta: dict[int, str] = {}

    for i in range(n):
        if is_suppressed[i]:
            continue
        cands = [j for j in range(i + 1, n) if not is_suppressed[j]]
        if not cands:
            continue

        parent_area = float(areas[i].item())
        is_parent_backdrop = (parent_area / max(image_total_area, 1.0)) >= max_backdrop_fraction

        cand_tensor = mask_tensor[cands]
        intersections = (cand_tensor & mask_tensor[i]).sum(dim=(1, 2)).float()
        ratios = intersections / torch.clamp(areas[cands], min=1.0)

        for k, ratio in enumerate(ratios):
            ratio_val = float(ratio.item())
            if ratio_val < containment_threshold:
                continue

            cand_idx = cands[k]
            cand_m = sorted_masks[cand_idx]
            cand_area = float(areas[cand_idx].item())
            scale_ratio = parent_area / max(cand_area, 1.0)

            # Determine candidate bounding dimensions
            if "bbox" in cand_m:
                cw = float(cand_m["bbox"][2])
                ch = float(cand_m["bbox"][3])
            else:
                ys, xs = np.where(cand_m["segmentation"])
                if len(xs) > 0:
                    cw = float(xs.max() - xs.min() + 1)
                    ch = float(ys.max() - ys.min() + 1)
                else:
                    cw, ch = 0.0, 0.0

            aspect_v = ch / max(cw, 1e-3)
            aspect_h = cw / max(ch, 1e-3)
            max_dim = max(cw, ch)
            min_dim = max(min(cw, ch), 1e-3)
            diag_aspect = max_dim / min_dim

            # Protection 1: Parent is environmental backdrop (road asphalt, building facade, sky)
            if is_parent_backdrop:
                continue

            # Protection 2: Scale disparity ceiling (parent is >> child; environmental context)
            if scale_ratio >= max_parent_child_ratio:
                continue

            # Protection 3: Vertical structural element (pillar, column, pole, lamppost)
            if aspect_v >= min_vertical_aspect and ch >= min_element_dim:
                continue

            # Protection 4: Horizontal / elongated element (road markings, curbs, crosswalks)
            if (aspect_h >= min_horizontal_aspect and cw >= min_element_dim) or (
                diag_aspect >= 3.0 and max_dim >= min_element_dim * 1.2
            ):
                continue

            # If none of the protections apply, candidate is genuinely an internal part of parent
            is_suppressed[cand_idx] = True
            suppression_meta[cand_idx] = (
                f"contained_{ratio_val:.2f}_in_larger_mask_{i}"
            )

    kept: list[dict[str, Any]] = []
    suppressed: list[dict[str, Any]] = []

    for idx, m in enumerate(sorted_masks):
        if is_suppressed[idx]:
            m_copy = dict(m)
            m_copy["suppression_reason"] = suppression_meta.get(
                idx, "contained_in_parent"
            )
            suppressed.append(m_copy)
        else:
            kept.append(m)

    return kept, suppressed


def compute_box_iou_matrix(boxes1: np.ndarray, boxes2: np.ndarray) -> np.ndarray:
    """Compute pairwise IoU matrix between boxes1 [N, 4] and boxes2 [M, 4]."""
    if len(boxes1) == 0 or len(boxes2) == 0:
        return np.zeros((len(boxes1), len(boxes2)), dtype=np.float32)

    x1 = np.maximum(boxes1[:, None, 0], boxes2[None, :, 0])
    y1 = np.maximum(boxes1[:, None, 1], boxes2[None, :, 1])
    x2 = np.minimum(boxes1[:, None, 2], boxes2[None, :, 2])
    y2 = np.minimum(boxes1[:, None, 3], boxes2[None, :, 3])

    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    a1 = np.maximum(0.0, boxes1[:, 2] - boxes1[:, 0]) * np.maximum(
        0.0, boxes1[:, 3] - boxes1[:, 1]
    )
    a2 = np.maximum(0.0, boxes2[:, 2] - boxes2[:, 0]) * np.maximum(
        0.0, boxes2[:, 3] - boxes2[:, 1]
    )
    union = a1[:, None] + a2[None, :] - intersection
    return intersection / np.maximum(union, 1e-6)


def match_sam_with_human_gt(
    sam_bboxes: list[list[float]],
    gt_records: Sequence[SemanticGroundTruthRecord],
    duplicate_iou_thresh: float = 0.50,
    novel_iou_thresh: float = 0.15,
) -> list[dict[str, Any]]:
    """Classify SAM proposals based on overlap with existing human ground truth.

    /// @param sam_bboxes List of [x1, y1, x2, y2] bounding boxes.
    /// @param gt_records Human ground truth records for the image.
    /// @param duplicate_iou_thresh Overlap threshold for duplicate of human GT.
    /// @param novel_iou_thresh Overlap threshold for novel physical instance.
    /// @return List of matching results per SAM proposal.
    """
    if not sam_bboxes:
        return []

    positive_gts = [gt for gt in gt_records if gt.is_positive]
    if not positive_gts:
        return [
            {
                "status": "novel",
                "max_iou": 0.0,
                "matched_gt_id": None,
                "matched_gt_category": None,
            }
            for _ in sam_bboxes
        ]

    sam_arr = np.array(sam_bboxes, dtype=np.float32)
    gt_arr = np.array([gt.bbox for gt in positive_gts], dtype=np.float32)
    iou_mat = compute_box_iou_matrix(sam_arr, gt_arr)  # [S, G]

    results: list[dict[str, Any]] = []
    for s_idx in range(len(sam_bboxes)):
        row = iou_mat[s_idx]
        best_gt_idx = int(np.argmax(row))
        max_iou = float(row[best_gt_idx])
        matched_gt = positive_gts[best_gt_idx]

        if max_iou >= duplicate_iou_thresh:
            status = "duplicate_gt"
        elif max_iou <= novel_iou_thresh:
            status = "novel"
        else:
            status = "borderline_gt"

        results.append(
            {
                "status": status,
                "max_iou": round(max_iou, 4),
                "matched_gt_id": matched_gt.annotation_id,
                "matched_gt_category": matched_gt.canonical_category
                or matched_gt.source_category,
            }
        )

    return results


def process_image_discovery(
    record: SemanticImageRecord,
    mask_generator: SamAutomaticMaskGenerator,
    containment_threshold: float,
    duplicate_iou_thresh: float,
    novel_iou_thresh: float,
    max_area_fraction: float,
    max_parent_child_ratio: float = 12.0,
    max_backdrop_fraction: float = 0.15,
    min_vertical_aspect: float = 2.2,
    min_horizontal_aspect: float = 2.5,
    min_element_dim: float = 25.0,
    device: str = "cuda",
) -> dict[str, Any]:
    """Run full discovery, geometry extraction, containment, and GT matching for one image."""
    img_pil = Image.open(record.image_path).convert("RGB")
    img_np = np.array(img_pil)
    image_total_area = float(record.width * record.height)

    # 1. SAM Automatic Mask Generation with mixed precision
    with torch.inference_mode(), torch.autocast(
        device_type="cuda" if device.startswith("cuda") else "cpu",
        dtype=torch.float16 if device.startswith("cuda") else torch.bfloat16,
    ):
        raw_masks = mask_generator.generate(img_np)

    # 2. GPU Containment Suppression with scale and aspect protection
    kept_masks, suppressed_masks = suppress_contained_masks_gpu(
        raw_masks,
        containment_threshold=containment_threshold,
        max_parent_child_ratio=max_parent_child_ratio,
        max_backdrop_fraction=max_backdrop_fraction,
        min_vertical_aspect=min_vertical_aspect,
        min_horizontal_aspect=min_horizontal_aspect,
        min_element_dim=min_element_dim,
        device=device,
    )

    # 3. Extract Dual Geometries for Kept Masks
    kept_geometries = []
    for m in kept_masks:
        bbox, quad, polygon, area = extract_mask_geometries(m["segmentation"])
        kept_geometries.append(
            {
                "raw_mask": m,
                "bbox": bbox,
                "quad": quad,
                "polygon": polygon,
                "area": area,
            }
        )

    # 4. Hybrid Matching with Human Ground Truth
    kept_bboxes = [g["bbox"] for g in kept_geometries]
    gt_match_results = match_sam_with_human_gt(
        kept_bboxes,
        record.ground_truths,
        duplicate_iou_thresh=duplicate_iou_thresh,
        novel_iou_thresh=novel_iou_thresh,
    )

    # 5. Format Annotations
    annotations: list[dict[str, Any]] = []
    ann_id = 1

    # Ingest Verified Human GT
    for gt in record.ground_truths:
        annotations.append(
            {
                "id": ann_id,
                "source": "human_gt",
                "category": gt.canonical_category or gt.source_category,
                "source_category": gt.source_category,
                "bbox": [float(round(v, 2)) for v in gt.bbox],
                "quad": [
                    [float(round(p[0], 2)), float(round(p[1], 2))]
                    for p in gt.quad
                ],
                "polygon": [],
                "area": float(round(gt.area, 2)),
                "is_positive": gt.is_positive,
                "is_discovered": False,
                "is_suppressed": False,
                "suppression_reason": None,
                "sam_stability_score": None,
                "sam_predicted_iou": None,
            }
        )
        ann_id += 1

    # Ingest SAM Discovered & Kept Instances
    novel_count = 0
    duplicate_count = 0
    borderline_count = 0
    stuff_count = 0

    for geom, match_res in zip(kept_geometries, gt_match_results, strict=True):
        status = match_res["status"]
        raw_m = geom["raw_mask"]
        area_frac = geom["area"] / max(image_total_area, 1.0)

        if area_frac > max_area_fraction:
            stuff_count += 1
            is_suppressed = True
            suppression_reason = f"exceeds_max_area_fraction_{area_frac:.2f}"
            category = "stuff_background"
        elif status == "novel":
            novel_count += 1
            is_suppressed = False
            suppression_reason = None
            category = "unlabeled_object"
        elif status == "duplicate_gt":
            duplicate_count += 1
            is_suppressed = True
            suppression_reason = f"duplicates_gt_{match_res['matched_gt_id']}_{match_res['matched_gt_category']}"
            category = f"duplicate_{match_res['matched_gt_category']}"
        else:  # borderline_gt
            borderline_count += 1
            is_suppressed = True
            suppression_reason = f"borderline_gt_overlap_{match_res['max_iou']:.2f}"
            category = "borderline_overlap"

        annotations.append(
            {
                "id": ann_id,
                "source": "sam_discovered",
                "category": category,
                "source_category": "sam_amg",
                "bbox": [float(round(v, 2)) for v in geom["bbox"]],
                "quad": geom["quad"],
                "polygon": geom["polygon"],
                "area": float(round(geom["area"], 2)),
                "is_positive": not is_suppressed,
                "is_discovered": status == "novel",
                "is_suppressed": is_suppressed,
                "suppression_reason": suppression_reason,
                "sam_stability_score": float(round(raw_m["stability_score"], 4)),
                "sam_predicted_iou": float(round(raw_m["predicted_iou"], 4)),
                "matched_gt_id": match_res["matched_gt_id"],
                "matched_gt_category": match_res["matched_gt_category"],
                "matched_gt_iou": match_res["max_iou"],
            }
        )
        ann_id += 1

    # Optionally extract geometries for suppressed internal parts for visual inspection
    for s_mask in suppressed_masks:
        bbox, quad, polygon, area = extract_mask_geometries(s_mask["segmentation"])
        annotations.append(
            {
                "id": ann_id,
                "source": "sam_discovered",
                "category": "contained_subpart",
                "source_category": "sam_amg",
                "bbox": [float(round(v, 2)) for v in bbox],
                "quad": quad,
                "polygon": polygon,
                "area": float(round(area, 2)),
                "is_positive": False,
                "is_discovered": False,
                "is_suppressed": True,
                "suppression_reason": s_mask.get(
                    "suppression_reason", "contained_in_parent"
                ),
                "sam_stability_score": float(round(s_mask["stability_score"], 4)),
                "sam_predicted_iou": float(round(s_mask["predicted_iou"], 4)),
                "matched_gt_id": None,
                "matched_gt_category": None,
                "matched_gt_iou": 0.0,
            }
        )
        ann_id += 1

    return {
        "image_id": record.image_id,
        "source_dataset": record.source_dataset,
        "file_name": record.file_name,
        "width": record.width,
        "height": record.height,
        "metrics": {
            "human_gt_count": len(record.ground_truths),
            "human_gt_positive_count": len(record.positive_ground_truths),
            "raw_sam_masks_count": len(raw_masks),
            "containment_suppressed_count": len(suppressed_masks),
            "kept_sam_masks_count": len(kept_masks),
            "novel_discovered_count": novel_count,
            "duplicate_gt_count": duplicate_count,
            "borderline_gt_count": borderline_count,
            "stuff_suppressed_count": stuff_count,
        },
        "annotations": annotations,
    }


def main() -> None:
    """Entry point for SAM discovery pipeline."""
    args = parse_args()

    db_path = args.output_db.resolve() if args.output_db else None
    json_path = args.output_json.resolve() if args.output_json else None
    if json_path is not None:
        json_path.parent.mkdir(parents=True, exist_ok=True)
    if db_path is not None:
        db_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. Determine index path
    if args.index_db is not None:
        index_path = args.index_db.resolve()
    elif args.all_images or args.num_images > 800:
        index_path = args.dataset_root.resolve() / "indexes/quad_train.sqlite"
    else:
        index_path = args.dataset_root.resolve() / "indexes/quad_val.sqlite"

    run_title = "FULL DATASET (ALL IMAGES)" if args.all_images else f"{args.num_images} IMAGES"
    print("==================================================================")
    print(f"SAM OBJECT DISCOVERY RUN ({run_title}, SPLIT: {args.split.upper()})")
    print("==================================================================")
    print(f"Dataset root:       {args.dataset_root.resolve()}")
    print(f"Index DB:           {index_path}")
    print(f"SAM checkpoint:     {args.checkpoint.resolve()}")
    if db_path:
        print(f"Output SQLite DB:   {db_path}")
    if json_path:
        print(f"Output JSON:        {json_path}")
    print(f"Compute device:     {args.device}")
    print(f"Points per side:    {args.points_per_side} ({args.points_per_side**2} prompt grid)")
    print(f"Points per batch:   {args.points_per_batch}")
    print(f"Pred IoU threshold: {args.pred_iou_thresh}")
    print(f"Stability threshold:{args.stability_score_thresh}")
    print(f"Min area floor:     {args.min_area} px")
    print(f"Containment thresh: {args.containment_thresh}")
    print(f"Batch commit size:  {args.save_interval} images")
    print(f"Streaming chunk:    {args.chunk_size} images")
    print("------------------------------------------------------------------")

    # 2. Select benchmark cohort / all images
    cohort_ids, cohort_sha = select_cohort_image_ids(
        args.dataset_root,
        count=args.num_images,
        index_path=index_path,
        all_images=args.all_images,
    )
    print(f"Selected {len(cohort_ids)} cohort images directly from {index_path.name}.")
    print(f"Cohort SHA-256:     {cohort_sha}")
    if not args.all_images:
        if args.num_images == 800 and index_path.name == "quad_val.sqlite":
            assert (
                cohort_sha == TARGET_COHORT_SHA256
            ), f"Cohort SHA mismatch! Expected {TARGET_COHORT_SHA256}, got {cohort_sha}"
            print("Cohort validation:  PASSED (Exact match with benchmark contract)")
        elif args.num_images == 10000 and index_path.name == "quad_train.sqlite":
            assert (
                cohort_sha == STAGE2_COHORT_SHA256
            ), f"Stage 2 SHA mismatch! Expected {STAGE2_COHORT_SHA256}, got {cohort_sha}"
            print("Cohort validation:  PASSED (Exact match with Stage 2 contract)")

    # 3. Initialize SQLite and resume tracking
    sqlite_conn = None
    processed_ids: set[int] = set()
    discovered_images: list[dict[str, Any]] = []

    if db_path is not None:
        sqlite_conn = init_discovery_db(db_path)
        processed_ids = get_processed_image_ids(db_path)
        print(
            f"Resuming from SQLite: Found {len(processed_ids)} already processed images in {db_path}."
        )

    if json_path is not None and json_path.exists():
        try:
            existing_data = json.loads(json_path.read_text(encoding="utf-8"))
            if isinstance(existing_data, dict) and "images" in existing_data:
                discovered_images = existing_data["images"]
                json_processed = {img["image_id"] for img in discovered_images}
                processed_ids.update(json_processed)
                print(
                    f"Resuming from JSON: Loaded {len(discovered_images)} images from {json_path}."
                )
        except Exception as e:
            print(f"Warning: Could not read existing output JSON: {e}.")

    remaining_ids = [cid for cid in cohort_ids if cid not in processed_ids]
    if args.max_images is not None and args.max_images < len(remaining_ids):
        remaining_ids = remaining_ids[: args.max_images]
        print(f"Restricted to first {len(remaining_ids)} remaining images by --max-images flag.")

    total_target = len(cohort_ids)
    print(f"Total target cohort:     {total_target} images")
    print(f"Already completed:       {len(processed_ids)} images")
    print(f"Pending to process:      {len(remaining_ids)} images")

    if not remaining_ids:
        print("==================================================================")
        print("ALL TARGET IMAGES ALREADY PROCESSED IN DATABASE! NOTHING TO DO.")
        print("==================================================================")
        return

    # 4. Initialize SAM Model and Mask Generator on GPU
    print(f"Loading SAM ViT-B model onto {args.device}...")
    t_model = time.time()
    sam = sam_model_registry["vit_b"](checkpoint=str(args.checkpoint.resolve()))
    sam.to(device=args.device)
    mask_generator = SamAutomaticMaskGenerator(
        model=sam,
        points_per_side=args.points_per_side,
        points_per_batch=args.points_per_batch,
        pred_iou_thresh=args.pred_iou_thresh,
        stability_score_thresh=args.stability_score_thresh,
        min_mask_region_area=int(args.min_area),
        box_nms_thresh=0.7,
        crop_n_layers=0,
    )
    print(f"SAM loaded in {time.time() - t_model:.2f}s.")

    # 5. Signal handling for graceful pause / stop
    stop_requested = False

    def handle_stop(signum: int, frame: Any) -> None:
        nonlocal stop_requested
        print(
            "\n[SAM DISCOVERY] Graceful stop requested. Finishing current batch and saving to SQLite..."
        )
        stop_requested = True

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    # 6. Stream and Process Images in manageable chunks
    print(f"Starting discovery run across {len(remaining_ids)} pending images...")
    start_time = time.time()
    total_novel_discovered = sum(
        img.get("metrics", {}).get("novel_discovered_count", 0) for img in discovered_images
    )
    total_human_gts = sum(
        img.get("metrics", {}).get("human_gt_count", 0) for img in discovered_images
    )

    batch_buffer: list[dict[str, Any]] = []
    num_processed_this_run = 0

    chunk_size = max(1, args.chunk_size)
    for chunk_start in range(0, len(remaining_ids), chunk_size):
        if stop_requested:
            break
        chunk_ids = remaining_ids[chunk_start : chunk_start + chunk_size]
        records = load_semantic_image_records(
            args.dataset_root, image_ids=chunk_ids, index_path=index_path
        )
        for rec in records:
            if stop_requested:
                break
            step_t0 = time.time()
            res = process_image_discovery(
                record=rec,
                mask_generator=mask_generator,
                containment_threshold=args.containment_thresh,
                duplicate_iou_thresh=args.gt_duplicate_iou_thresh,
                novel_iou_thresh=args.gt_novel_iou_thresh,
                max_area_fraction=args.max_area_fraction,
                max_parent_child_ratio=args.max_parent_child_ratio,
                max_backdrop_fraction=args.max_backdrop_fraction,
                min_vertical_aspect=args.min_vertical_aspect,
                min_horizontal_aspect=args.min_horizontal_aspect,
                min_element_dim=args.min_element_dim,
                device=args.device,
            )
            step_elapsed = time.time() - step_t0

            res["split"] = args.split
            discovered_images.append(res)
            batch_buffer.append(res)
            processed_ids.add(rec.image_id)
            num_processed_this_run += 1
            total_novel_discovered += res["metrics"]["novel_discovered_count"]
            total_human_gts += res["metrics"]["human_gt_count"]

            # Progress reporting
            num_done = len(processed_ids)
            fps = num_processed_this_run / max(time.time() - start_time, 1e-4)
            eta_sec = (total_target - num_done) / max(fps, 1e-4)

            print(
                f"[{num_done:06d}/{total_target:06d}] "
                f"Img {rec.image_id:06d} ({rec.source_dataset[:8]}) | "
                f"Raw: {res['metrics']['raw_sam_masks_count']:02d} -> "
                f"Kept: {res['metrics']['kept_sam_masks_count']:02d} "
                f"(Novel: {res['metrics']['novel_discovered_count']:02d}, "
                f"GT Dup: {res['metrics']['duplicate_gt_count']:02d}, "
                f"Contained: {res['metrics']['containment_suppressed_count']:02d}) | "
                f"{step_elapsed:.2f}s | "
                f"ETA: {eta_sec/3600.0:.1f}h"
            )

            # Transactional SQLite commit
            if len(batch_buffer) >= args.save_interval:
                if sqlite_conn is not None:
                    save_image_batch(sqlite_conn, batch_buffer, default_split=args.split)
                    print(
                        f"--- Transaction committed: {len(batch_buffer)} images to SQLite (Total: {num_done}) ---"
                    )
                batch_buffer.clear()

                if json_path is not None:
                    payload = {
                        "schema_version": "sam-discovery.v2",
                        "cohort_sha256": cohort_sha,
                        "points_per_side": args.points_per_side,
                        "summary": {
                            "total_images": len(discovered_images),
                            "total_human_gt": total_human_gts,
                            "total_novel_discovered": total_novel_discovered,
                        },
                        "images": discovered_images,
                    }
                    tmp_json = json_path.with_suffix(".tmp.json")
                    tmp_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
                    tmp_json.replace(json_path)

    # Final flush
    if batch_buffer and sqlite_conn is not None:
        save_image_batch(sqlite_conn, batch_buffer, default_split=args.split)
        print(
            f"--- Final transaction committed: {len(batch_buffer)} images to SQLite (Total: {len(processed_ids)}) ---"
        )
        batch_buffer.clear()

    if json_path is not None and discovered_images:
        payload = {
            "schema_version": "sam-discovery.v2",
            "cohort_sha256": cohort_sha,
            "points_per_side": args.points_per_side,
            "summary": {
                "total_images": len(discovered_images),
                "total_human_gt": total_human_gts,
                "total_novel_discovered": total_novel_discovered,
            },
            "images": discovered_images,
        }
        tmp_json = json_path.with_suffix(".tmp.json")
        tmp_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        tmp_json.replace(json_path)

    total_time = time.time() - start_time
    print("==================================================================")
    if stop_requested:
        print("DISCOVERY RUN PAUSED / STOPPED GRACEFULLY")
    else:
        print("DISCOVERY RUN COMPLETE!")
    print(f"Total in database:      {len(processed_ids)} images")
    print(f"Processed this run:     {num_processed_this_run} images")
    print(f"Total novel discovered: {total_novel_discovered} objects")
    print(f"Total human GTs:        {total_human_gts} annotations")
    print(f"Total run time:         {total_time:.2f}s ({total_time/60.0:.1f}m)")
    if db_path:
        print(f"Database saved to:      {db_path}")
        summary = get_dataset_summary(db_path)
        print(f"Final DB Summary:       {summary}")
    print("==================================================================")


if __name__ == "__main__":
    main()
