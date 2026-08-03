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


def canonicalize_quads(quad: Tensor) -> Tensor:
    """Canonicalize quads clockwise, starting at each top-left vertex."""
    value = _quad_tensor(quad)
    if not torch.isfinite(value).all():
        raise ValueError("quad contains non-finite coordinates")
    original_shape = value.shape
    flat = value.reshape(-1, 4, 2)
    center = flat.mean(dim=1)
    angles = torch.atan2(
        flat[..., 1] - center[:, None, 1],
        flat[..., 0] - center[:, None, 0],
    )
    order = torch.argsort(angles, dim=1, stable=True)
    ordered = torch.gather(flat, 1, order.unsqueeze(-1).expand(-1, -1, 2))
    reverse = quad_signed_area(ordered) < 0
    ordered = torch.where(
        reverse[:, None, None], torch.flip(ordered, dims=(1,)), ordered
    )
    minimum_y = ordered[..., 1].amin(dim=1, keepdim=True)
    top_x = ordered[..., 0].masked_fill(ordered[..., 1] != minimum_y, torch.inf)
    start = top_x.argmin(dim=1)
    indices = (
        torch.arange(4, device=value.device).unsqueeze(0) + start.unsqueeze(1)
    ).remainder(4)
    canonical = torch.gather(
        ordered, 1, indices.unsqueeze(-1).expand(-1, -1, 2)
    )
    return canonical.reshape(original_shape)


def canonicalize_quad(quad: Tensor) -> Tensor:
    """Canonicalize one quad clockwise, starting at its top-left vertex."""
    value = _quad_tensor(quad)
    if value.ndim != 2:
        raise ValueError("canonicalize_quad accepts one [4, 2] quad")
    return canonicalize_quads(value)


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


def _points_inside_ordered_quads(points: Tensor, quads: Tensor) -> Tensor:
    """Test batched points against batched convex, perimeter-ordered quads."""
    edges = torch.roll(quads, -1, dims=-2) - quads
    relative = points.unsqueeze(-2) - quads.unsqueeze(-3)
    turns = _cross(edges.unsqueeze(-3), relative)
    return (turns >= -_EPS).all(dim=-1) | (turns <= _EPS).all(dim=-1)


def _aligned_intersection_area(first: Tensor, second: Tensor) -> Tensor:
    """Vectorized exact intersection area for aligned convex quad pairs."""
    if first.shape != second.shape or first.ndim != 3:
        raise ValueError("quad pairs must both have shape [N, 4, 2]")
    if first.shape[0] == 0:
        return first.new_empty((0,))

    first_edges = torch.roll(first, -1, dims=1) - first
    second_edges = torch.roll(second, -1, dims=1) - second
    first_start = first[:, :, None, :]
    second_start = second[:, None, :, :]
    first_direction = first_edges[:, :, None, :]
    second_direction = second_edges[:, None, :, :]
    difference = second_start - first_start
    denominator = _cross(first_direction, second_direction)
    nonparallel = denominator.abs() > _EPS
    safe_denominator = torch.where(
        nonparallel, denominator, torch.ones_like(denominator)
    )
    first_fraction = _cross(difference, second_direction) / safe_denominator
    second_fraction = _cross(difference, first_direction) / safe_denominator
    intersects = (
        nonparallel
        & (first_fraction >= -_EPS)
        & (first_fraction <= 1 + _EPS)
        & (second_fraction >= -_EPS)
        & (second_fraction <= 1 + _EPS)
    )
    intersections = first_start + first_fraction.unsqueeze(-1) * first_direction

    candidates = torch.cat(
        (first, second, intersections.flatten(start_dim=1, end_dim=2)), dim=1
    )
    candidate_mask = torch.cat(
        (
            _points_inside_ordered_quads(first, second),
            _points_inside_ordered_quads(second, first),
            intersects.flatten(start_dim=1),
        ),
        dim=1,
    )
    counts = candidate_mask.sum(dim=1)
    centers = (candidates * candidate_mask.unsqueeze(-1)).sum(dim=1) / counts.clamp(
        min=1
    ).unsqueeze(-1)
    angles = torch.atan2(
        candidates[..., 1] - centers[:, None, 1],
        candidates[..., 0] - centers[:, None, 0],
    )
    angles = angles.masked_fill(~candidate_mask, torch.inf)
    order = torch.argsort(angles, dim=1, stable=True)
    ordered = torch.gather(candidates, 1, order.unsqueeze(-1).expand(-1, -1, 2))

    sequential_cross = _cross(ordered[:, :-1], ordered[:, 1:])
    positions = torch.arange(
        sequential_cross.shape[1], device=first.device
    ).unsqueeze(0)
    sequential_cross = (
        sequential_cross * (positions < (counts - 1).unsqueeze(1))
    ).sum(dim=1)
    last_index = (counts - 1).clamp(min=0)
    last = ordered.gather(
        1, last_index[:, None, None].expand(-1, 1, 2)
    ).squeeze(1)
    closing_cross = _cross(last, ordered[:, 0])
    area = 0.5 * (sequential_cross + closing_cross).abs()
    return torch.where(counts >= 3, area, torch.zeros_like(area))


def aligned_quad_iou(first: Tensor, second: Tensor) -> Tensor:
    """Compute exact IoU for aligned [N, 4, 2] pairs."""
    first_value = _quad_tensor(first)
    second_value = _quad_tensor(second)
    if first_value.shape != second_value.shape:
        raise ValueError("aligned quad tensors must have equal shape")
    intersection = _aligned_intersection_area(first_value, second_value)
    union = quad_area(first_value) + quad_area(second_value) - intersection
    return intersection / union.clamp(min=torch.finfo(first_value.dtype).eps)


_compiled_aligned_quad_iou = torch.compile(
    aligned_quad_iou, fullgraph=True, dynamic=False
)


def warmup_compiled_quad_iou(device: torch.device) -> None:
    """Compile the fixed-size CUDA overlap kernel outside measured inference."""
    if device.type != "cuda":
        return
    sample = torch.tensor(
        [[0.0, 0.0], [8.0, 0.0], [8.0, 8.0], [0.0, 8.0]], device=device
    ).expand(16_384, -1, -1)
    _compiled_aligned_quad_iou(sample, sample)


def pairwise_quad_iou(
    first: Tensor,
    second: Tensor,
    *,
    pair_chunk_size: int = 16_384,
    minimum_iou: float = 0.0,
) -> Tensor:
    """Compute an exact [N, M] convex-quad IoU matrix in bounded batches.

    Values that cannot exceed ``minimum_iou`` are safely left at zero. This is
    useful for NMS, where only threshold crossings matter.
    """
    first_value = _quad_tensor(first)
    second_value = _quad_tensor(second)
    if first_value.ndim != 3 or second_value.ndim != 3:
        raise ValueError("quad sets must have shape [N, 4, 2] and [M, 4, 2]")
    if pair_chunk_size < 1:
        raise ValueError("pair_chunk_size must be positive")
    if not 0 <= minimum_iou <= 1:
        raise ValueError("minimum_iou must be in [0, 1]")
    rows, columns = first_value.shape[0], second_value.shape[0]
    if rows == 0 or columns == 0:
        return first_value.new_zeros((rows, columns))
    first_min = first_value.amin(dim=1)
    first_max = first_value.amax(dim=1)
    second_min = second_value.amin(dim=1)
    second_max = second_value.amax(dim=1)
    overlap_width = (
        torch.minimum(first_max[:, None, 0], second_max[None, :, 0])
        - torch.maximum(first_min[:, None, 0], second_min[None, :, 0])
    ).clamp(min=0)
    overlap_height = (
        torch.minimum(first_max[:, None, 1], second_max[None, :, 1])
        - torch.maximum(first_min[:, None, 1], second_min[None, :, 1])
    ).clamp(min=0)
    envelope_intersection = overlap_width * overlap_height
    first_area = quad_area(first_value)
    second_area = quad_area(second_value)
    maximum_intersection = torch.minimum(
        envelope_intersection,
        torch.minimum(first_area[:, None], second_area[None, :]),
    )
    iou_upper_bound = maximum_intersection / (
        first_area[:, None] + second_area[None, :] - maximum_intersection
    ).clamp(min=torch.finfo(first_value.dtype).eps)
    envelope_overlap = iou_upper_bound > minimum_iou
    flat_indices = envelope_overlap.flatten().nonzero().flatten()
    result = first_value.new_zeros((rows * columns,))
    if flat_indices.numel() == 0:
        return result.reshape(rows, columns)
    first_indices = torch.div(flat_indices, columns, rounding_mode="floor")
    second_indices = flat_indices.remainder(columns)
    values: list[Tensor] = []
    for start in range(0, flat_indices.shape[0], pair_chunk_size):
        chunk_first = first_value[first_indices[start : start + pair_chunk_size]]
        chunk_second = second_value[second_indices[start : start + pair_chunk_size]]
        chunk_length = chunk_first.shape[0]
        if first_value.device.type == "cuda":
            if pair_chunk_size != 16_384:
                raise ValueError("CUDA pair_chunk_size must remain 16384")
            if chunk_length < pair_chunk_size:
                padding = pair_chunk_size - chunk_length
                chunk_first = torch.cat((chunk_first, chunk_first[:1].expand(padding, -1, -1)))
                chunk_second = torch.cat((chunk_second, chunk_second[:1].expand(padding, -1, -1)))
            chunk_iou = _compiled_aligned_quad_iou(chunk_first, chunk_second)[:chunk_length]
        else:
            chunk_iou = aligned_quad_iou(chunk_first, chunk_second)
        values.append(chunk_iou)
    result[flat_indices] = torch.cat(values)
    return result.reshape(rows, columns)


def polygon_nms(
    quads: Tensor,
    scores: Tensor,
    iou_threshold: float,
    *,
    max_output: int | None = None,
) -> Tensor:
    """Exact class-agnostic NMS using one batched polygon-overlap matrix."""
    value = _quad_tensor(quads)
    if value.shape[0] != scores.shape[0]:
        raise ValueError("quads and scores must have equal length")
    if value.shape[0] == 0:
        return torch.empty((0,), dtype=torch.long, device=value.device)
    valid = quad_validity(value)
    overlaps = pairwise_quad_iou(
        value, value, minimum_iou=iou_threshold
    ).detach().cpu()
    order = torch.argsort(scores.detach().cpu(), descending=True, stable=True)
    valid_cpu = valid.detach().cpu()
    kept: list[int] = []
    suppressed = torch.zeros(value.shape[0], dtype=torch.bool)
    for index in order.tolist():
        if not valid_cpu[index] or suppressed[index]:
            continue
        kept.append(index)
        suppressed |= overlaps[index] > iou_threshold
        # Scores are visited in descending order. Once the requested output
        # budget is full, later candidates cannot enter that prefix.
        if max_output is not None and len(kept) >= max_output:
            break
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
