"""Offline gradient influence measurements for proposal-loss families."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
import statistics

import torch
from torch import Tensor


_FAMILIES = ("quality", "localization")
_WEIGHTINGS = ("unweighted", "weighted")


def _norm(gradients: Sequence[Tensor]) -> Tensor:
    squared = sum(gradient.float().square().sum() for gradient in gradients)
    return squared.sqrt()


def _dot(left: Sequence[Tensor], right: Sequence[Tensor]) -> Tensor:
    return sum(
        (left_item.float() * right_item.float()).sum()
        for left_item, right_item in zip(left, right, strict=True)
    )


def _safe_ratio(numerator: Tensor, denominator: Tensor) -> Tensor:
    return torch.where(
        denominator > 0,
        numerator / denominator.clamp(min=torch.finfo(torch.float32).eps),
        denominator.new_zeros(()),
    )


def measure_gradient_influence(
    family_terms: Mapping[str, Tensor],
    sites: Mapping[str, Sequence[Tensor]],
) -> dict[str, float]:
    """Measure family gradients at named non-leaf activation tensors.

    The function uses ``autograd.grad`` and therefore never populates model
    parameter ``.grad`` fields or mutates optimizer state.
    """
    expected = {
        f"{family}/{weighting}"
        for family in _FAMILIES
        for weighting in _WEIGHTINGS
    }
    missing = expected - set(family_terms)
    if missing:
        raise ValueError(f"gradient family terms are missing: {sorted(missing)}")
    if not sites or any(not tensors for tensors in sites.values()):
        raise ValueError("gradient sites must contain at least one tensor")

    site_names = tuple(sites)
    flat_sites = tuple(tensor for name in site_names for tensor in sites[name])
    if any(not tensor.requires_grad for tensor in flat_sites):
        raise ValueError("every gradient-site tensor must require gradients")
    offsets: dict[str, slice] = {}
    start = 0
    for name in site_names:
        stop = start + len(sites[name])
        offsets[name] = slice(start, stop)
        start = stop

    gradients: dict[str, tuple[Tensor, ...]] = {}
    ordered_terms = tuple(sorted(expected))
    for index, name in enumerate(ordered_terms):
        values = torch.autograd.grad(
            family_terms[name],
            flat_sites,
            retain_graph=index < len(ordered_terms) - 1,
            allow_unused=True,
        )
        gradients[name] = tuple(
            torch.zeros_like(site) if gradient is None else gradient
            for site, gradient in zip(flat_sites, values, strict=True)
        )

    result = {
        f"loss/{name}": float(value.detach().float().cpu())
        for name, value in family_terms.items()
        if name in expected
    }
    for site_name, selected in offsets.items():
        site_gradients = {
            name: values[selected] for name, values in gradients.items()
        }
        for family in _FAMILIES:
            for weighting in _WEIGHTINGS:
                key = f"{family}/{weighting}"
                result[f"gradient/{site_name}/{key}_norm"] = float(
                    _norm(site_gradients[key]).detach().cpu()
                )

        quality = site_gradients["quality/weighted"]
        localization = site_gradients["localization/weighted"]
        total = tuple(
            quality_item + localization_item
            for quality_item, localization_item in zip(
                quality, localization, strict=True
            )
        )
        quality_norm = _norm(quality)
        localization_norm = _norm(localization)
        total_norm = _norm(total)
        cosine = _safe_ratio(
            _dot(quality, localization), quality_norm * localization_norm
        )
        result[f"gradient/{site_name}/total/weighted_norm"] = float(
            total_norm.detach().cpu()
        )
        result[f"gradient/{site_name}/quality/weighted_to_total_ratio"] = float(
            _safe_ratio(quality_norm, total_norm).detach().cpu()
        )
        result[
            f"gradient/{site_name}/localization/weighted_to_total_ratio"
        ] = float(_safe_ratio(localization_norm, total_norm).detach().cpu())
        result[
            f"gradient/{site_name}/quality_localization/weighted_cosine"
        ] = float(cosine.detach().cpu())
    if not all(math.isfinite(value) for value in result.values()):
        raise FloatingPointError("non-finite gradient influence measurement")
    return result


def summarize_gradient_measurements(
    measurements: Sequence[Mapping[str, float]],
) -> dict[str, object]:
    """Summarize repeated calibration batches including population variance."""
    if not measurements:
        raise ValueError("at least one gradient measurement is required")
    keys = set(measurements[0])
    if any(set(measurement) != keys for measurement in measurements[1:]):
        raise ValueError("gradient measurements must have identical metric keys")
    summary: dict[str, dict[str, float]] = {}
    for key in sorted(keys):
        values = [float(measurement[key]) for measurement in measurements]
        summary[key] = {
            "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "variance": statistics.pvariance(values),
            "minimum": min(values),
            "maximum": max(values),
        }
    return {"batches": len(measurements), "metrics": summary}


__all__ = ["measure_gradient_influence", "summarize_gradient_measurements"]
