from __future__ import annotations

import torch

from student_detector.decoder import InferenceDecoder, class_agnostic_nms
from student_detector.head import DetectorOutput


def test_nms_is_class_agnostic() -> None:
    boxes = torch.tensor(
        [[0.0, 0.0, 10.0, 10.0], [1.0, 1.0, 11.0, 11.0], [20.0, 20.0, 30.0, 30.0]]
    )
    scores = torch.tensor([0.9, 0.8, 0.7])
    keep = class_agnostic_nms(boxes, scores, iou_threshold=0.5)
    assert keep.tolist() == [0, 2]


def test_nms_breaks_equal_score_ties_by_input_order() -> None:
    boxes = torch.tensor([[0.0, 0.0, 10.0, 10.0], [1.0, 1.0, 11.0, 11.0]])
    keep = class_agnostic_nms(boxes, torch.tensor([0.5, 0.5]), iou_threshold=0.5)
    assert keep.tolist() == [0]


def test_decoder_clips_and_limits_proposals() -> None:
    shapes = ((4, 4), (2, 2), (1, 1))
    objectness = tuple(torch.full((1, 1, h, w), 10.0) for h, w in shapes)
    centerness = tuple(torch.full((1, 1, h, w), 10.0) for h, w in shapes)
    distances = tuple(torch.full((1, 4, h, w), 1000.0) for h, w in shapes)
    output = DetectorOutput(objectness, distances, centerness)

    result = InferenceDecoder(top_k=10, max_detections=3)(output, (32, 32))[0]
    assert result.boxes.shape[0] <= 3
    assert result.levels is not None
    assert result.location_indices is not None
    assert result.levels.shape == result.scores.shape
    assert result.location_indices.shape == result.scores.shape
    assert result.pre_nms_boxes is not None
    assert result.pre_nms_scores is not None
    assert torch.all(result.boxes >= 0)
    assert torch.all(result.boxes <= 32)


def test_decoder_rejects_zero_area_boxes_after_clipping() -> None:
    shapes = ((1, 1), (1, 1), (1, 1))
    objectness = tuple(torch.full((1, 1, h, w), 10.0) for h, w in shapes)
    centerness = tuple(torch.full((1, 1, h, w), 10.0) for h, w in shapes)
    distances = tuple(torch.zeros((1, 4, h, w)) for h, w in shapes)
    result = InferenceDecoder(top_k=3)(
        DetectorOutput(objectness, distances, centerness), (32, 32)
    )[0]
    assert result.candidate_count == 3
    assert result.invalid_candidate_count == 3
    assert result.boxes.shape == (0, 4)


def test_decoder_masks_padding_before_topk() -> None:
    shapes = ((2, 2), (1, 1), (1, 1))
    objectness = tuple(torch.zeros((1, 1, h, w)) for h, w in shapes)
    objectness[0][0, 0, 0, 0] = 20
    centerness = tuple(torch.zeros((1, 1, h, w)) for h, w in shapes)
    distances = tuple(torch.ones((1, 4, h, w)) for h, w in shapes)
    valid = (
        torch.tensor([[[False, True], [True, True]]]),
        torch.ones((1, 1, 1), dtype=torch.bool),
        torch.ones((1, 1, 1), dtype=torch.bool),
    )
    result = InferenceDecoder(top_k=1)(
        DetectorOutput(objectness, distances, centerness),
        (16, 16),
        valid,
    )[0]
    assert result.boxes.shape == (1, 4)
    assert result.levels is not None and result.levels.tolist() == [3]
    assert result.location_indices is not None
    # The invalid high-score point at (4, 4) must not win top-K.
    assert result.boxes[0, 0] > 4


def test_objectness_score_mode_ignores_dormant_centerness() -> None:
    shapes = ((1, 2), (1, 1), (1, 1))
    objectness = (
        torch.tensor([[[[8.0, 4.0]]]]),
        torch.full((1, 1, 1, 1), -20.0),
        torch.full((1, 1, 1, 1), -20.0),
    )
    centerness = (
        torch.tensor([[[[-20.0, 8.0]]]]),
        torch.zeros((1, 1, 1, 1)),
        torch.zeros((1, 1, 1, 1)),
    )
    distances = tuple(torch.ones((1, 4, h, w)) for h, w in shapes)
    output = DetectorOutput(objectness, distances, centerness)
    objectness_result = InferenceDecoder(top_k=1, score_mode="objectness")(
        output, (16, 16)
    )[0]
    product_result = InferenceDecoder(top_k=1, score_mode="objectness_x_centerness")(
        output, (16, 16)
    )[0]
    assert objectness_result.boxes[0, 0] < product_result.boxes[0, 0]

    # Verify centerness_alpha = 0.0 matches score_mode="objectness"
    alpha_zero_result = InferenceDecoder(
        top_k=1, score_mode="objectness_x_centerness", centerness_alpha=0.0
    )(output, (16, 16))[0]
    assert torch.allclose(objectness_result.scores, alpha_zero_result.scores)

    # Verify centerness_alpha = 0.5 produces scores between alpha=0 and alpha=1
    alpha_half_result = InferenceDecoder(
        top_k=1, score_mode="objectness_x_centerness", centerness_alpha=0.5
    )(output, (16, 16))[0]
    assert alpha_half_result.scores[0] > 0


def test_nms_part_aware_preserves_distinct_subparts() -> None:
    # Box 0: Parent object (whole building), area = 100 * 100 = 10000
    # Box 1: Duplicate box of parent, area = 100 * 100 = 10000, IoU ~ 0.8
    # Box 2: Sub-part (pillar/column inside building), area = 15 * 50 = 750, scale_ratio = 750 / 10000 = 0.075 <= 0.4
    boxes = torch.tensor(
        [
            [0.0, 0.0, 100.0, 100.0],
            [2.0, 2.0, 102.0, 102.0],
            [10.0, 20.0, 25.0, 70.0],
        ]
    )
    scores = torch.tensor([0.95, 0.85, 0.90])

    # Hard NMS with low threshold would suppress overlapping pillar if IoU intersects
    # Under part_aware, duplicate box 1 is suppressed, but sub-part 2 is preserved!
    keep, kept_scores = class_agnostic_nms(
        boxes,
        scores,
        iou_threshold=0.05,  # Deliberately strict to test scale containment
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        return_scores=True,
    )
    assert 0 in keep.tolist()
    assert 1 not in keep.tolist()  # Duplicate suppressed
    assert 2 in keep.tolist()  # Subpart preserved!


def test_nms_soft_decays_duplicate_scores() -> None:
    boxes = torch.tensor(
        [
            [0.0, 0.0, 10.0, 10.0],
            [1.0, 1.0, 11.0, 11.0],
        ]
    )
    scores = torch.tensor([0.9, 0.8])
    keep, kept_scores = class_agnostic_nms(
        boxes, scores, iou_threshold=0.5, nms_mode="soft", return_scores=True
    )
    assert keep.tolist() == [0, 1]
    # Kept score for box 1 should be decayed
    assert kept_scores[1] < 0.8


def test_nms_part_aware_configurable_part_max_iou() -> None:
    # Box 0: [0, 0, 10, 10], area = 100
    # Box 1: [0, 3, 10, 10], area = 70. Intersection = 70, Union = 100 -> IoU = 0.70
    boxes = torch.tensor(
        [
            [0.0, 0.0, 10.0, 10.0],
            [0.0, 3.0, 10.0, 10.0],
        ]
    )
    scores = torch.tensor([0.9, 0.8])

    # When iou_threshold=0.75, standard part check (IoU > 0.75) won't trigger.
    # With default part_max_iou=0.65, IoU 0.70 > 0.65 suppresses box 1.
    keep_default = class_agnostic_nms(
        boxes,
        scores,
        iou_threshold=0.75,
        nms_mode="part_aware",
        part_max_iou=0.65,
    )
    assert keep_default.tolist() == [0]

    # With higher configured part_max_iou=0.80, IoU 0.70 <= 0.80 preserves box 1.
    keep_relaxed = class_agnostic_nms(
        boxes,
        scores,
        iou_threshold=0.75,
        nms_mode="part_aware",
        part_max_iou=0.80,
    )
    assert keep_relaxed.tolist() == [0, 1]
