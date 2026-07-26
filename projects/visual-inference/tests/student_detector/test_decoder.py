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


def test_decoder_clips_and_limits_proposals() -> None:
    shapes = ((4, 4), (2, 2), (1, 1))
    objectness = tuple(torch.full((1, 1, h, w), 10.0) for h, w in shapes)
    centerness = tuple(torch.full((1, 1, h, w), 10.0) for h, w in shapes)
    distances = tuple(torch.full((1, 4, h, w), 1000.0) for h, w in shapes)
    output = DetectorOutput(objectness, distances, centerness)

    result = InferenceDecoder(top_k=10, max_detections=3)(output, (32, 32))[0]
    assert result.boxes.shape[0] <= 3
    assert torch.all(result.boxes >= 0)
    assert torch.all(result.boxes <= 32)


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
    distances = tuple(
        torch.ones((1, 4, h, w)) for h, w in shapes
    )
    output = DetectorOutput(objectness, distances, centerness)
    objectness_result = InferenceDecoder(
        top_k=1, score_mode="objectness"
    )(output, (16, 16))[0]
    product_result = InferenceDecoder(
        top_k=1, score_mode="objectness_x_centerness"
    )(output, (16, 16))[0]
    assert objectness_result.boxes[0, 0] < product_result.boxes[0, 0]
