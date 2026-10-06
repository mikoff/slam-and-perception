"""Unit tests for SAM object discovery pilot primitives and geometries."""

from __future__ import annotations

import numpy as np
import pytest

from scripts.run_sam_pilot_discovery import (
    compute_box_iou_matrix,
    extract_mask_geometries,
    match_sam_with_human_gt,
    order_quad_clockwise,
    suppress_contained_masks_gpu,
)
from student_detector.semantic_evaluation_data import SemanticGroundTruthRecord


def test_order_quad_clockwise() -> None:
    # Unordered quad points
    pts = np.array([[10.0, 50.0], [50.0, 50.0], [10.0, 10.0], [50.0, 10.0]])
    ordered = order_quad_clockwise(pts)
    assert len(ordered) == 4
    # Top-left vertex should be ordered first
    assert ordered[0] == [10.0, 10.0]
    # Check that vertices are clockwise: [10, 10] -> [50, 10] -> [50, 50] -> [10, 50]
    assert ordered[1] == [50.0, 10.0]
    assert ordered[2] == [50.0, 50.0]
    assert ordered[3] == [10.0, 50.0]


def test_extract_mask_geometries_circle() -> None:
    mask = np.zeros((100, 100), dtype=bool)
    # Draw solid square in center
    mask[20:60, 30:70] = True
    bbox, quad, polygon, area = extract_mask_geometries(mask)

    assert bbox == [30.0, 20.0, 70.0, 60.0]
    assert len(quad) == 4
    assert len(polygon) >= 4
    assert area == pytest.approx(1600.0, abs=10.0)


def test_gpu_containment_suppression() -> None:
    # 1. Non-backdrop parent (area 1200 / 10000 = 12% < 15%) and true internal subpart (area 300, ratio 4x <= 12)
    parent = np.zeros((100, 100), dtype=bool)
    parent[20:60, 20:50] = True  # area 1200, width 30, height 40

    # True compact subpart (e.g. car door, square aspect)
    subpart = np.zeros((100, 100), dtype=bool)
    subpart[25:40, 25:40] = True  # area 225, width 15, height 15 (aspect 1.0)

    # Separate object
    separate = np.zeros((100, 100), dtype=bool)
    separate[0:5, 0:5] = True  # area 25 (0% contained)

    masks = [
        {"segmentation": subpart, "area": 225.0, "bbox": [25.0, 25.0, 15.0, 15.0]},
        {"segmentation": parent, "area": 1200.0, "bbox": [20.0, 20.0, 30.0, 40.0]},
        {"segmentation": separate, "area": 25.0, "bbox": [0.0, 0.0, 5.0, 5.0]},
    ]

    kept, suppressed = suppress_contained_masks_gpu(
        masks, containment_threshold=0.85, device="cpu"
    )

    assert len(kept) == 2
    assert len(suppressed) == 1
    assert suppressed[0]["area"] == 225.0
    assert "contained" in suppressed[0]["suppression_reason"]


def test_gpu_containment_structural_and_road_protections() -> None:
    # Backdrop parent (road surface, 40% of image area)
    road_backdrop = np.zeros((100, 100), dtype=bool)
    road_backdrop[50:90, 0:100] = True  # area 4000, 40% of image

    # Horizontal road marking (width 50, height 5, aspect 10.0 >= 2.5)
    road_marking = np.zeros((100, 100), dtype=bool)
    road_marking[60:65, 20:70] = True  # area 250, 100% inside road
    road_marking_box = [20.0, 60.0, 50.0, 5.0]

    # Vertical pillar in front of non-backdrop building (H=30, W=8, aspect 3.75 >= 2.2)
    building = np.zeros((100, 100), dtype=bool)
    building[0:40, 0:30] = True  # area 1200, 12% of image
    pillar = np.zeros((100, 100), dtype=bool)
    pillar[5:35, 5:13] = True  # area 240, 100% inside building
    pillar_box = [5.0, 5.0, 8.0, 30.0]

    masks = [
        {"segmentation": road_backdrop, "area": 4000.0, "bbox": [0.0, 50.0, 100.0, 40.0]},
        {"segmentation": road_marking, "area": 250.0, "bbox": road_marking_box},
        {"segmentation": building, "area": 1200.0, "bbox": [0.0, 0.0, 30.0, 40.0]},
        {"segmentation": pillar, "area": 240.0, "bbox": pillar_box},
    ]

    kept, suppressed = suppress_contained_masks_gpu(
        masks, containment_threshold=0.85, device="cpu"
    )

    # Both the road marking (horizontal) and pillar (vertical) should be preserved!
    assert len(suppressed) == 0
    assert len(kept) == 4


def test_compute_box_iou_matrix() -> None:
    b1 = np.array([[0.0, 0.0, 10.0, 10.0]], dtype=np.float32)
    b2 = np.array([[0.0, 0.0, 10.0, 10.0], [5.0, 0.0, 15.0, 10.0]], dtype=np.float32)
    iou = compute_box_iou_matrix(b1, b2)
    assert iou.shape == (1, 2)
    assert iou[0, 0] == pytest.approx(1.0)
    assert iou[0, 1] == pytest.approx(50.0 / 150.0)


def test_match_sam_with_human_gt() -> None:
    gt_record = SemanticGroundTruthRecord(
        annotation_id=101,
        image_id=1,
        source_dataset="test",
        source_category="car",
        canonical_category="car",
        quad=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
        bbox=(0.0, 0.0, 10.0, 10.0),
        ignore_region=0,
        geometry_tier="tier1",
        size_bin="medium",
        area=100.0,
    )

    sam_boxes = [
        [0.0, 0.0, 10.0, 10.0],  # Exact match -> duplicate_gt (IoU = 1.0 >= 0.50)
        [20.0, 20.0, 30.0, 30.0],  # No overlap -> novel (IoU = 0.0 <= 0.15)
        [5.0, 0.0, 15.0, 10.0],  # Partial overlap -> borderline_gt (IoU = 0.33)
    ]

    results = match_sam_with_human_gt(
        sam_boxes,
        [gt_record],
        duplicate_iou_thresh=0.50,
        novel_iou_thresh=0.15,
    )

    assert len(results) == 3
    assert results[0]["status"] == "duplicate_gt"
    assert results[0]["matched_gt_id"] == 101
    assert results[1]["status"] == "novel"
    assert results[2]["status"] == "borderline_gt"
