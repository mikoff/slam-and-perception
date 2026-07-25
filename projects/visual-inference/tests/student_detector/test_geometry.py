from __future__ import annotations

import torch

from student_detector.geometry import decode_ltrb, encode_ltrb, make_grid_points


def test_perfect_distance_targets_reproduce_boxes() -> None:
    points = torch.tensor([[20.0, 30.0], [100.0, 80.0]])
    boxes = torch.tensor([[12.0, 18.0, 44.0, 50.0], [90.0, 60.0, 130.0, 120.0]])
    distances = encode_ltrb(points, boxes)
    torch.testing.assert_close(decode_ltrb(points, distances), boxes)


def test_grid_points_are_cell_centres() -> None:
    points, slices = make_grid_points(((2, 2), (1, 1)), (8, 16))
    torch.testing.assert_close(
        points,
        torch.tensor([[4.0, 4.0], [12.0, 4.0], [4.0, 12.0], [12.0, 12.0], [8.0, 8.0]]),
    )
    assert slices == (slice(0, 4), slice(4, 5))
