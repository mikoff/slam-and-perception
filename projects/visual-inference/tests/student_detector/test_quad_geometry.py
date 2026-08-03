from __future__ import annotations

import torch

from student_detector.quad_geometry import (
    aligned_quad_iou,
    canonicalize_quad,
    canonicalize_quads,
    convex_polygon_iou,
    equivalent_quad_traversals,
    fit_quad_from_points,
    pairwise_quad_iou,
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
    batch = torch.stack((target, torch.flip(target, dims=(0,))))
    torch.testing.assert_close(
        canonicalize_quads(batch), canonical.expand_as(batch)
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


def test_vectorized_iou_matches_reference_clipping() -> None:
    generator = torch.Generator().manual_seed(17)
    centers = torch.rand((32, 2), generator=generator) * 40
    sizes = torch.rand((32, 2), generator=generator) * 12 + 2
    angles = torch.rand((32,), generator=generator) * torch.pi
    local = torch.tensor([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    local = local[None] * sizes[:, None] / 2
    cosine, sine = angles.cos(), angles.sin()
    rotation = torch.stack((cosine, -sine, sine, cosine), dim=1).reshape(-1, 2, 2)
    first = local @ rotation.transpose(1, 2) + centers[:, None]
    second = torch.roll(first, 1, dims=0) + torch.tensor([1.5, -0.75])
    expected = torch.stack([
        convex_polygon_iou(a, b) for a, b in zip(first, second, strict=True)
    ])
    torch.testing.assert_close(aligned_quad_iou(first, second), expected, atol=2e-5, rtol=2e-5)


def test_pairwise_iou_has_aligned_diagonal_and_symmetry() -> None:
    quads = torch.stack((
        quad_from_bbox([0.0, 0.0, 10.0, 10.0]),
        quad_from_bbox([3.0, 2.0, 8.0, 14.0]),
        quad_from_bbox([20.0, 20.0, 30.0, 30.0]),
    ))
    matrix = pairwise_quad_iou(quads, quads, pair_chunk_size=2)
    torch.testing.assert_close(matrix, matrix.T)
    torch.testing.assert_close(matrix.diagonal(), torch.ones(3))
