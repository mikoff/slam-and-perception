"""Reference geometry operations for class-agnostic quadrilateral proposals."""

from __future__ import annotations

import math
from collections.abc import Iterable

import torch
from torch import Tensor


_EPS = 1e-7


def _quad_tensor(quad: Tensor) -> Tensor:
    value = torch.as_tensor(quad)
    if value.shape[-2:] != (4, 2):
        raise ValueError("quad must have shape [..., 4, 2]")
    return value


def quad_signed_area(quad: Tensor) -> Tensor:
    """Return signed image-coordinate area; positive means clockwise."""
    value = _quad_tensor(quad)
    return 0.5 * (
        (value[..., :, 0] * torch.roll(value[..., :, 1], -1, -1)).sum(-1)
        - (value[..., :, 1] * torch.roll(value[..., :, 0], -1, -1)).sum(-1)
    )


def quad_area(quad: Tensor) -> Tensor:
    return quad_signed_area(quad).abs()


def quad_centroid(quad: Tensor) -> Tensor:
    """Return the area centroid of a non-degenerate quadrilateral."""
    value = _quad_tensor(quad)
    following = torch.roll(value, -1, dims=-2)
    cross = value[..., :, 0] * following[..., :, 1] - following[..., :, 0] * value[..., :, 1]
    denominator = (3.0 * cross.sum(dim=-1)).unsqueeze(-1)
    numerator = ((value + following) * cross.unsqueeze(-1)).sum(dim=-2)
    mean = value.mean(dim=-2)
    nondegenerate = denominator.abs() > _EPS
    safe_denominator = torch.where(nondegenerate, denominator, torch.ones_like(denominator))
    centroid = numerator / safe_denominator
    return torch.where(nondegenerate, centroid, mean)


def _cross(a: Tensor, b: Tensor) -> Tensor:
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def quad_validity(
    quad: Tensor,
    *,
    min_area: float = 1e-4,
    min_edge: float = 1e-4,
) -> Tensor:
    """Return a mask for finite, convex, non-self-intersecting quads."""
    value = _quad_tensor(quad)
    finite = torch.isfinite(value).all(dim=(-1, -2))
    edges = torch.roll(value, -1, -2) - value
    edge_lengths = torch.linalg.vector_norm(edges, dim=-1)
    edge_ok = (edge_lengths >= min_edge).all(dim=-1)
    signed = quad_signed_area(value)
    area_ok = signed.abs() >= min_area
    turns = _cross(edges, torch.roll(edges, -1, -2))
    convex = (turns > min_edge * min_edge).all(dim=-1) | (
        turns < -min_edge * min_edge
    ).all(dim=-1)
    return finite & edge_ok & area_ok & convex


def canonicalize_quad(quad: Tensor) -> Tensor:
    """Canonicalize one quad clockwise, starting at its top-left vertex."""
    value = _quad_tensor(quad)
    if value.ndim != 2:
        raise ValueError("canonicalize_quad accepts one [4, 2] quad")
    if not torch.isfinite(value).all():
        raise ValueError("quad contains non-finite coordinates")
    center = value.mean(dim=0)
    angles = torch.atan2(value[:, 1] - center[1], value[:, 0] - center[0])
    ordered = value[torch.argsort(angles)]
    if quad_signed_area(ordered) < 0:
        ordered = torch.flip(ordered, dims=(0,))
    # Image y points down. The smallest y, then smallest x, is the stable
    # top-left start for rectangles and the nearest equivalent start for
    # rotated/perspective quads.
    start = min(range(4), key=lambda index: (
        float(ordered[index, 1]), float(ordered[index, 0])
    ))
    return torch.roll(ordered, -start, dims=0)


def equivalent_quad_traversals(quad: Tensor) -> Tensor:
    """Return the four cyclic and four reversed target traversals."""
    canonical = canonicalize_quad(quad)
    forward = torch.stack([torch.roll(canonical, -i, dims=0) for i in range(4)])
    reverse = torch.flip(canonical, dims=(0,))
    backward = torch.stack([torch.roll(reverse, -i, dims=0) for i in range(4)])
    return torch.cat((forward, backward), dim=0)


def quad_offsets(quad: Tensor, point: Tensor, stride: float) -> Tensor:
    """Encode four image points as eight signed stride-normalized offsets."""
    value = _quad_tensor(quad)
    return ((value - point[..., None, :]) / float(stride)).reshape(
        *value.shape[:-2], 8
    )


def decode_quad_offsets(point: Tensor, offsets: Tensor, stride: float) -> Tensor:
    """Decode eight signed offsets relative to one feature-grid point."""
    value = torch.as_tensor(offsets)
    if value.shape[-1] != 8:
        raise ValueError("offsets must have shape [..., 8]")
    decoded = value.reshape(*value.shape[:-1], 4, 2)
    scale = torch.as_tensor(stride, device=value.device, dtype=value.dtype)
    while scale.ndim < decoded.ndim:
        scale = scale.unsqueeze(-1)
    return decoded * scale + point[..., None, :]


def points_inside_convex_quad(points: Tensor, quad: Tensor) -> Tensor:
    """Return [num_points] inclusion mask for a convex quad."""
    points = torch.as_tensor(points)
    value = canonicalize_quad(quad)
    edges = torch.roll(value, -1, 0) - value
    relative = points[:, None, :] - value[None, :, :]
    turns = _cross(edges[None, :, :], relative)
    return (turns >= -_EPS).all(dim=1) | (turns <= _EPS).all(dim=1)


def _line_intersection(start: Tensor, end: Tensor, clip_start: Tensor, clip_end: Tensor) -> Tensor:
    direction = end - start
    clip_direction = clip_end - clip_start
    denominator = _cross(direction, clip_direction)
    numerator = _cross(clip_start - start, clip_direction)
    t = numerator / denominator.clamp(min=_EPS) if denominator >= 0 else numerator / denominator.clamp(max=-_EPS)
    return start + t * direction


def clip_convex_polygon(subject: Tensor, clip: Tensor) -> Tensor:
    """Sutherland-Hodgman clipping for clockwise image-coordinate polygons."""
    subject_value = torch.as_tensor(subject)
    clip_value = canonicalize_quad(clip)
    if subject_value.ndim != 2 or subject_value.shape[1] != 2:
        raise ValueError("subject must have shape [N, 2]")
    vertices: list[Tensor] = [item for item in subject_value]
    for index in range(clip_value.shape[0]):
        if not vertices:
            break
        clip_start = clip_value[index]
        clip_end = clip_value[(index + 1) % clip_value.shape[0]]
        output: list[Tensor] = []
        previous = vertices[-1]
        previous_inside = bool(_cross(clip_end - clip_start, previous - clip_start) >= -_EPS)
        for current in vertices:
            current_inside = bool(_cross(clip_end - clip_start, current - clip_start) >= -_EPS)
            if current_inside != previous_inside:
                output.append(_line_intersection(previous, current, clip_start, clip_end))
            if current_inside:
                output.append(current)
            previous, previous_inside = current, current_inside
        vertices = output
    if not vertices:
        return subject_value.new_empty((0, 2))
    return torch.stack(vertices)


def polygon_signed_area(polygon: Tensor) -> Tensor:
    value = torch.as_tensor(polygon)
    if value.ndim < 2 or value.shape[-1] != 2:
        raise ValueError("polygon must have shape [..., N, 2]")
    if value.shape[-2] < 3:
        return value.new_zeros(value.shape[:-2])
    return 0.5 * (
        (value[..., :, 0] * torch.roll(value[..., :, 1], -1, -1)).sum(-1)
        - (value[..., :, 1] * torch.roll(value[..., :, 0], -1, -1)).sum(-1)
    )


def convex_polygon_iou(first: Tensor, second: Tensor) -> Tensor:
    """Exact IoU for two convex polygons, implemented without custom CUDA."""
    first_value = canonicalize_quad(first)
    second_value = canonicalize_quad(second)
    return _ordered_convex_polygon_iou(first_value, second_value)


def _ordered_convex_polygon_iou(first_value: Tensor, second_value: Tensor) -> Tensor:
    """IoU for already canonicalized clockwise convex quads."""
    intersection = clip_convex_polygon(first_value, second_value)
    intersection_area = polygon_signed_area(intersection).abs()
    union = quad_area(first_value) + quad_area(second_value) - intersection_area
    return intersection_area / union.clamp(min=torch.finfo(first_value.dtype).eps)


def aligned_quad_iou(first: Tensor, second: Tensor) -> Tensor:
    """Compute exact IoU for aligned [N, 4, 2] pairs."""
    first_value = _quad_tensor(first)
    second_value = _quad_tensor(second)
    if first_value.shape != second_value.shape:
        raise ValueError("aligned quad tensors must have equal shape")
    return torch.stack([
        convex_polygon_iou(a, b) for a, b in zip(first_value, second_value, strict=True)
    ]) if first_value.shape[0] else first_value.new_empty((0,))


def polygon_nms(quads: Tensor, scores: Tensor, iou_threshold: float) -> Tensor:
    """Reference class-agnostic NMS for convex quadrilaterals."""
    value = _quad_tensor(quads)
    if value.shape[0] != scores.shape[0]:
        raise ValueError("quads and scores must have equal length")
    if value.shape[0] == 0:
        return torch.empty((0,), dtype=torch.long, device=value.device)
    # Canonicalize once.  Re-canonicalizing inside every pairwise IoU call
    # makes the reference implementation needlessly quadratic in Python.
    nms_value = value.detach().cpu()
    nms_scores = scores.detach().cpu()
    canonical = torch.stack([canonicalize_quad(item) for item in nms_value])
    valid = torch.tensor([bool(quad_validity(item)) for item in canonical], dtype=torch.bool)
    bounds = torch.stack((canonical[..., 0].amin(dim=1), canonical[..., 1].amin(dim=1),
                          canonical[..., 0].amax(dim=1), canonical[..., 1].amax(dim=1)), dim=1)
    order = torch.argsort(nms_scores, descending=True, stable=True)
    kept: list[int] = []
    for index in order.tolist():
        if not valid[index]:
            continue
        suppressed = False
        for other in kept:
            # Axis-aligned envelope IoU is an admissible overlap upper bound
            # only for disjoint boxes (zero means exact polygon IoU is zero).
            left = max(float(bounds[index, 0]), float(bounds[other, 0]))
            top = max(float(bounds[index, 1]), float(bounds[other, 1]))
            right = min(float(bounds[index, 2]), float(bounds[other, 2]))
            bottom = min(float(bounds[index, 3]), float(bounds[other, 3]))
            if right <= left or bottom <= top:
                continue
            if float(_ordered_convex_polygon_iou(canonical[index], canonical[other])) > iou_threshold:
                suppressed = True
                break
        if not suppressed:
            kept.append(index)
    return torch.tensor(kept, dtype=torch.long, device=value.device)


def fit_quad_from_points(points: Tensor) -> Tensor:
    """Fit a containing oriented rectangle using the point cloud's PCA frame."""
    value = torch.as_tensor(points)
    if value.ndim != 2 or value.shape[1] != 2 or value.shape[0] < 3:
        raise ValueError("at least three [x, y] points are required")
    if not torch.isfinite(value).all():
        raise ValueError("points contain non-finite coordinates")
    center = value.mean(dim=0)
    centered = value - center
    covariance = centered.transpose(0, 1) @ centered / max(value.shape[0] - 1, 1)
    eigenvalues, eigenvectors = torch.linalg.eigh(covariance)
    major = eigenvectors[:, torch.argmax(eigenvalues)]
    major = major / torch.linalg.vector_norm(major).clamp(min=_EPS)
    minor = torch.stack((-major[1], major[0]))
    projected = torch.stack((centered @ major, centered @ minor), dim=1)
    lower = projected.min(dim=0).values
    upper = projected.max(dim=0).values
    corners_local = value.new_tensor([
        [0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]
    ])
    corners_local = lower + corners_local * (upper - lower)
    fitted = center + corners_local[:, :1] * major + corners_local[:, 1:] * minor
    return canonicalize_quad(fitted)


def quad_from_bbox(box: Tensor | Iterable[float]) -> Tensor:
    value = torch.as_tensor(box, dtype=torch.float32)
    if value.shape != (4,):
        raise ValueError("box must have shape [4] as xyxy")
    x1, y1, x2, y2 = value
    return value.new_tensor([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
