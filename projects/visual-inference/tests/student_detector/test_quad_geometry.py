from __future__ import annotations

import torch

from student_detector.quad_geometry import (
    aligned_quad_iou,
    canonicalize_quad,
    equivalent_quad_traversals,
    fit_quad_from_points,
    polygon_nms,
    points_inside_convex_quad,
    quad_from_bbox,
    quad_validity,
)


def test_canonicalization_and_eight_traversals() -> None:
    target = torch.tensor([[0.0, 0.0], [10.0, 1.0], [9.0, 8.0], [1.0, 7.0]])
    canonical = canonicalize_quad(torch.flip(target, dims=(0,)))
    assert quad_validity(canonical)
    assert equivalent_quad_traversals(canonical).shape == (8, 4, 2)
    torch.testing.assert_close(
        aligned_quad_iou(
            equivalent_quad_traversals(canonical),
            canonical.expand(8, -1, -1),
        ),
        torch.ones(8),
    )


def test_points_inside_and_fit_cover_cloud() -> None:
    quad = quad_from_bbox([2.0, 3.0, 12.0, 13.0])
    points = torch.tensor([[4.0, 5.0], [11.0, 12.0], [20.0, 20.0]])
    assert points_inside_convex_quad(points, quad).tolist() == [True, True, False]
    fitted = fit_quad_from_points(points[:2].new_tensor([
        [2.0, 3.0], [12.0, 3.0], [12.0, 13.0], [2.0, 13.0],
        [7.0, 6.0],
    ]))
    assert quad_validity(fitted)
    assert points_inside_convex_quad(points[:2], fitted).all()


def test_polygon_nms_suppresses_only_overlapping_quads() -> None:
    quads = torch.stack((
        quad_from_bbox([0.0, 0.0, 10.0, 10.0]),
        quad_from_bbox([1.0, 1.0, 11.0, 11.0]),
        quad_from_bbox([20.0, 20.0, 30.0, 30.0]),
    ))
    keep = polygon_nms(quads, torch.tensor([0.9, 0.8, 0.7]), 0.5)
    assert keep.tolist() == [0, 2]
