"""Full-projective quadrilateral patch extraction outside detector forward."""

from __future__ import annotations

import torch
import torch.nn.functional as functional
from torch import Tensor

from .geometry import valid_quadrilateral_mask


def homography_from_four_points(source: Tensor, destination: Tensor) -> Tensor:
    """Solve batched 3x3 homographies mapping source points to destination."""
    if source.shape[-2:] != (4, 2) or destination.shape != source.shape:
        raise ValueError("source and destination must have shape [N, 4, 2]")
    source = source.float()
    destination = destination.float()
    x, y = source.unbind(dim=-1)
    u, v = destination.unbind(dim=-1)
    zeros = torch.zeros_like(x)
    ones = torch.ones_like(x)
    rows_u = torch.stack(
        (x, y, ones, zeros, zeros, zeros, -u * x, -u * y),
        dim=-1,
    )
    rows_v = torch.stack(
        (zeros, zeros, zeros, x, y, ones, -v * x, -v * y),
        dim=-1,
    )
    matrix = torch.stack((rows_u, rows_v), dim=-2).reshape(-1, 8, 8)
    values = torch.stack((u, v), dim=-1).reshape(-1, 8, 1)
    coefficients = torch.linalg.solve(matrix, values).squeeze(-1)
    return torch.cat(
        (coefficients, torch.ones_like(coefficients[:, :1])), dim=-1
    ).reshape(-1, 3, 3)


def _normalize_grid(
    x: Tensor,
    y: Tensor,
    *,
    width: int,
    height: int,
    align_corners: bool,
) -> Tensor:
    if align_corners:
        normalized_x = 2 * x / max(width - 1, 1) - 1
        normalized_y = 2 * y / max(height - 1, 1) - 1
    else:
        normalized_x = 2 * (x + 0.5) / width - 1
        normalized_y = 2 * (y + 0.5) / height - 1
    return torch.stack((normalized_x, normalized_y), dim=-1)


def extract_rectified_patches(
    image: Tensor,
    quadrilaterals: Tensor,
    *,
    output_size: int | tuple[int, int] = 224,
    align_corners: bool = False,
    padding_mode: str = "border",
) -> Tensor:
    """Warp TL,TR,BR,BL quads into fixed patches using full homographies.

    ``image`` is ``[C, H, W]`` and quadrilaterals are ``[K, 4, 2]`` in image
    pixel coordinates. This utility intentionally remains outside the exported
    detector and uses output-to-input projective sampling.
    """
    if image.ndim != 3:
        raise ValueError("image must have shape [C, H, W]")
    if quadrilaterals.ndim != 3 or quadrilaterals.shape[-2:] != (4, 2):
        raise ValueError("quadrilaterals must have shape [K, 4, 2]")
    if isinstance(output_size, int):
        output_height = output_width = output_size
    else:
        output_height, output_width = output_size
    if output_height < 2 or output_width < 2:
        raise ValueError("output dimensions must be at least 2")
    count = quadrilaterals.shape[0]
    if count == 0:
        return image.new_empty(
            (0, image.shape[0], output_height, output_width)
        )
    if not valid_quadrilateral_mask(quadrilaterals.float()).all():
        raise ValueError("quadrilaterals must be finite clockwise convex quads")

    destination = quadrilaterals.new_tensor([
        [0.0, 0.0],
        [output_width - 1.0, 0.0],
        [output_width - 1.0, output_height - 1.0],
        [0.0, output_height - 1.0],
    ]).unsqueeze(0).expand(count, -1, -1)
    # grid_sample needs, for every output pixel, its source-image position.
    output_to_input = homography_from_four_points(
        destination, quadrilaterals
    )
    y, x = torch.meshgrid(
        torch.arange(
            output_height, device=image.device, dtype=torch.float32
        ),
        torch.arange(
            output_width, device=image.device, dtype=torch.float32
        ),
        indexing="ij",
    )
    homogeneous = torch.stack(
        (x, y, torch.ones_like(x)), dim=-1
    ).reshape(-1, 3)
    mapped = torch.matmul(
        output_to_input, homogeneous.t().unsqueeze(0)
    ).transpose(1, 2)
    denominator = mapped[..., 2].clamp(min=1e-7)
    source_x = (mapped[..., 0] / denominator).reshape(
        count, output_height, output_width
    )
    source_y = (mapped[..., 1] / denominator).reshape(
        count, output_height, output_width
    )
    grid = _normalize_grid(
        source_x,
        source_y,
        width=image.shape[-1],
        height=image.shape[-2],
        align_corners=align_corners,
    ).to(dtype=image.dtype)
    return functional.grid_sample(
        image.unsqueeze(0).expand(count, -1, -1, -1),
        grid,
        mode="bilinear",
        padding_mode=padding_mode,
        align_corners=align_corners,
    )


__all__ = ["extract_rectified_patches", "homography_from_four_points"]
