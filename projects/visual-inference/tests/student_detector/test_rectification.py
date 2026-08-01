from __future__ import annotations

import torch

from student_detector.rectification import (
    extract_rectified_patches,
    homography_from_four_points,
)


def test_homography_maps_all_four_correspondences() -> None:
    source = torch.tensor([[
        [0.0, 0.0], [9.0, 0.0], [9.0, 9.0], [0.0, 9.0],
    ]])
    destination = torch.tensor([[
        [2.0, 1.0], [8.0, 2.0], [9.0, 8.0], [1.0, 9.0],
    ]])
    matrix = homography_from_four_points(source, destination)
    homogeneous = torch.cat(
        (source, torch.ones((1, 4, 1))), dim=-1
    )
    mapped = torch.matmul(matrix, homogeneous.transpose(1, 2)).transpose(1, 2)
    mapped = mapped[..., :2] / mapped[..., 2:]
    torch.testing.assert_close(mapped, destination, atol=1e-5, rtol=0)


def test_full_image_rectangle_rectifies_to_identity() -> None:
    image = torch.arange(3 * 8 * 10, dtype=torch.float32).reshape(3, 8, 10)
    quadrilateral = torch.tensor([[
        [0.0, 0.0], [9.0, 0.0], [9.0, 7.0], [0.0, 7.0],
    ]])
    patch = extract_rectified_patches(
        image, quadrilateral, output_size=(8, 10)
    )
    torch.testing.assert_close(
        patch[0], image, atol=2e-4, rtol=0
    )


def test_empty_quad_batch_returns_fixed_empty_patch_tensor() -> None:
    patches = extract_rectified_patches(
        torch.zeros(3, 16, 16),
        torch.empty(0, 4, 2),
        output_size=224,
    )
    assert patches.shape == (0, 3, 224, 224)
