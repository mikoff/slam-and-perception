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
