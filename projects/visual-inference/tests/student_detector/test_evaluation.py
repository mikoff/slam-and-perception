from __future__ import annotations

import torch

from student_detector.decoder import Detection
from student_detector.evaluation import EvaluationImage, evaluate_proposals


def test_perfect_proposals_have_perfect_recall_and_ap():
    box = torch.tensor([[10.0, 10.0, 30.0, 30.0]])
    metrics = evaluate_proposals([
        EvaluationImage(
            image_id=1,
            domain="general",
            camera_type="perspective",
            image_size=(64, 64),
            ground_truth=box,
            ignore_boxes=torch.empty((0, 4)),
            detection=Detection(box.clone(), torch.tensor([0.9])),
        )
    ])
    assert metrics["ap/50"] == 1
    assert metrics["ap/75"] == 1
    assert metrics["recall/10@50"] == 1
    assert metrics["recall/general/100@50"] == 1


def test_detection_inside_ignore_region_is_not_false_positive():
    ignored = torch.tensor([[0.0, 0.0, 8.0, 8.0]])
    gt = torch.tensor([[20.0, 20.0, 40.0, 40.0]])
    detection = Detection(
        torch.cat((ignored, gt)),
        torch.tensor([0.99, 0.9]),
    )
    metrics = evaluate_proposals([
        EvaluationImage(
            image_id=1,
            domain="fisheye",
            camera_type="fisheye",
            image_size=(64, 64),
            ground_truth=gt,
            ignore_boxes=ignored,
            detection=detection,
        )
    ])
    assert metrics["ap/50"] == 1
