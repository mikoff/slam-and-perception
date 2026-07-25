"""Run the Phase-2 completion checks without downloading pretrained weights."""

from __future__ import annotations

import torch

from student_detector import ATSSAssigner, StudentDetector
from student_detector.geometry import decode_ltrb, encode_ltrb


def main() -> None:
    torch.manual_seed(0)
    model = StudentDetector(pretrained_backbone=False).eval()
    print(f"parameters: {sum(parameter.numel() for parameter in model.parameters()):,}")

    for image_size in (320, 384):
        example = torch.randn(1, 3, image_size, image_size)
        with torch.inference_mode():
            output = model(example)
        shapes = [tuple(tensor.shape) for tensor in output.objectness]
        print(f"forward {image_size}x{image_size}: {shapes}")

    example = torch.randn(1, 3, 384, 384)
    exported = torch.export.export(model, (example,))
    print(f"torch.export: OK ({len(exported.graph.nodes)} graph nodes)")

    boxes = torch.tensor([[31.0, 41.0, 91.0, 121.0], [140.0, 100.0, 300.0, 330.0]])
    assignment = ATSSAssigner().assign(
        boxes, ((48, 48), (24, 24), (12, 12)), (384, 384)
    )
    positives_per_gt = [
        int((assignment.matched_gt_indices == index).sum()) for index in range(len(boxes))
    ]
    print(f"ATSS positives per GT: {positives_per_gt}")

    points = torch.tensor([[20.0, 30.0], [100.0, 80.0]])
    target_boxes = torch.tensor(
        [[12.0, 18.0, 44.0, 50.0], [90.0, 60.0, 130.0, 120.0]]
    )
    reconstructed = decode_ltrb(points, encode_ltrb(points, target_boxes))
    torch.testing.assert_close(reconstructed, target_boxes)
    print("perfect target round-trip: OK")


if __name__ == "__main__":
    main()
