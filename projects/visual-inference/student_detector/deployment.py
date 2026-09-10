"""Raw detector export helpers and static deployment-cost accounting."""

from __future__ import annotations

from collections.abc import Sequence

import torch
from torch import Tensor, nn

from .head import DetectorOutput, QuadDetectorOutput


def flatten_detector_output(
    output: DetectorOutput | QuadDetectorOutput,
) -> tuple[Tensor, ...]:
    """Flatten either fixed raw output contract into serializable tensors."""
    if isinstance(output, DetectorOutput):
        return (*output.objectness, *output.box_distances, *output.centerness)
    if isinstance(output, QuadDetectorOutput):
        return (*output.quality, *output.corner_offsets)
    raise TypeError(f"unsupported detector output: {type(output).__name__}")


class DetectorExportView(nn.Module):
    """Expose the raw detector graph with a plain-tuple output tree."""

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.model = model

    def forward(self, images: Tensor) -> tuple[Tensor, ...]:
        return flatten_detector_output(self.model(images))


def count_macs(model: nn.Module, example: Tensor) -> int:
    """Count convolution and linear multiply-accumulates for one forward pass."""
    total = 0

    def convolution_hook(
        module: nn.Module, _inputs: Sequence[Tensor], output: Tensor
    ) -> None:
        nonlocal total
        if not isinstance(module, nn.Conv2d):
            raise TypeError("convolution hook attached to a non-convolution module")
        kernel_height, kernel_width = module.kernel_size
        total += (
            output.numel()
            * (module.in_channels // module.groups)
            * kernel_height
            * kernel_width
        )

    def linear_hook(
        module: nn.Module, _inputs: Sequence[Tensor], output: Tensor
    ) -> None:
        nonlocal total
        if not isinstance(module, nn.Linear):
            raise TypeError("linear hook attached to a non-linear module")
        total += output.numel() * module.in_features

    handles = []
    for module in model.modules():
        if isinstance(module, nn.Conv2d):
            handles.append(module.register_forward_hook(convolution_hook))
        elif isinstance(module, nn.Linear):
            handles.append(module.register_forward_hook(linear_hook))
    try:
        with torch.inference_mode():
            model(example)
    finally:
        for handle in handles:
            handle.remove()
    return total


def parity_metrics(
    reference: Sequence[Tensor], candidate: Sequence[Tensor]
) -> dict[str, float | int | bool]:
    """Return strict shape plus FP32 numerical-parity evidence."""
    if len(reference) != len(candidate):
        return {
            "tensor_count": len(candidate),
            "shape_match": False,
            "max_absolute_error": float("inf"),
            "max_relative_error": float("inf"),
            "allclose_atol_1e-5_rtol_1e-5": False,
        }
    shape_match = all(
        left.shape == right.shape for left, right in zip(reference, candidate)
    )
    if not shape_match:
        return {
            "tensor_count": len(candidate),
            "shape_match": False,
            "max_absolute_error": float("inf"),
            "max_relative_error": float("inf"),
            "allclose_atol_1e-5_rtol_1e-5": False,
        }
    max_absolute = 0.0
    max_relative = 0.0
    allclose = True
    for left, right in zip(reference, candidate, strict=True):
        left = left.detach()
        right = right.detach()
        difference = (left - right).abs()
        max_absolute = max(max_absolute, float(difference.amax()))
        relative = difference / left.abs().clamp_min(1e-6)
        max_relative = max(max_relative, float(relative.amax()))
        allclose = allclose and torch.allclose(left, right, atol=1e-5, rtol=1e-5)
    return {
        "tensor_count": len(candidate),
        "shape_match": True,
        "max_absolute_error": max_absolute,
        "max_relative_error": max_relative,
        "allclose_atol_1e-5_rtol_1e-5": allclose,
    }


__all__ = [
    "DetectorExportView",
    "count_macs",
    "flatten_detector_output",
    "parity_metrics",
]
