"""Deterministic selection and review states for one hard-negative mining round."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
import hashlib
from typing import Any

from .data import ImageRecord


def _integer_quotas(weights: dict[str, float], count: int) -> dict[str, int]:
    if count < 1:
        raise ValueError("count must be positive")
    positive = {key: float(value) for key, value in weights.items() if value > 0}
    total = sum(positive.values())
    if not positive or total <= 0:
        raise ValueError("at least one source weight must be positive")
    exact = {key: count * value / total for key, value in positive.items()}
    quotas = {key: int(value) for key, value in exact.items()}
    remainder = count - sum(quotas.values())
    order = sorted(positive, key=lambda key: (-(exact[key] - quotas[key]), key))
    for key in order[:remainder]:
        quotas[key] += 1
    return quotas


def select_hashed_pool_indices(
    records: Sequence[ImageRecord],
    source_weights: dict[str, float],
    count: int,
    *,
    seed: int,
) -> list[int]:
    """Choose an order-independent, source-stratified mining pool."""
    quotas = _integer_quotas(source_weights, count)
    ranked: dict[str, list[tuple[bytes, int]]] = defaultdict(list)
    for index, record in enumerate(records):
        if record.source_dataset not in quotas:
            continue
        identity = f"{seed}:{record.source_dataset}:{record.image_id}".encode()
        ranked[record.source_dataset].append((hashlib.sha256(identity).digest(), index))
    selected: list[int] = []
    for source, quota in sorted(quotas.items()):
        candidates = sorted(ranked[source])
        if len(candidates) < quota:
            raise RuntimeError(
                f"mining pool lacks {quota - len(candidates)} images for {source}"
            )
        selected.extend(index for _, index in candidates[:quota])
    return sorted(selected, key=lambda index: records[index].row_index)


def review_classification(
    *,
    best_object_iou: float,
    ignore_fraction: float,
    trusted_background_fraction: float,
    semantic_state: str | None,
    semantic_fraction: float,
    unmatched_iou: float,
    region_fraction: float,
) -> str | None:
    """Classify a high-score proposal without asserting unlabeled space is negative."""
    if best_object_iou >= unmatched_iou:
        return None
    if ignore_fraction >= region_fraction:
        return "ignore_region"
    if trusted_background_fraction >= region_fraction:
        return "certified_trusted_background"
    if semantic_state == "positive" and semantic_fraction >= region_fraction:
        return "possible_labeled_object"
    return "unverified_background"


def hamming_distance(left: str, right: str) -> int:
    """Return bit distance between equal-width hexadecimal perceptual hashes."""
    if len(left) != len(right):
        raise ValueError("perceptual hashes must have equal width")
    return (int(left, 16) ^ int(right, 16)).bit_count()


def deduplicate_and_cap(
    candidates: Sequence[dict[str, Any]],
    *,
    per_image: int,
    per_domain: dict[str, int],
    perceptual_hamming: int,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    """Keep highest scores subject to image/domain caps and visual deduplication."""
    if per_image < 1 or perceptual_hamming < 0:
        raise ValueError("caps must be positive and Hamming distance nonnegative")
    ordered = sorted(
        candidates,
        key=lambda item: (-float(item["score"]), str(item["candidate_id"])),
    )
    kept: list[dict[str, Any]] = []
    image_counts: dict[tuple[str, int], int] = defaultdict(int)
    domain_counts: dict[str, int] = defaultdict(int)
    hashes: list[str] = []
    rejected = defaultdict(int)
    for candidate in ordered:
        domain = str(candidate["domain"])
        image_key = (str(candidate["source_dataset"]), int(candidate["image_id"]))
        if image_counts[image_key] >= per_image:
            rejected["per_image_cap"] += 1
            continue
        if domain not in per_domain:
            raise KeyError(f"missing mining cap for domain {domain!r}")
        if domain_counts[domain] >= per_domain[domain]:
            rejected["per_domain_cap"] += 1
            continue
        crop_hash = str(candidate["crop_dhash"])
        if any(
            hamming_distance(crop_hash, prior) <= perceptual_hamming for prior in hashes
        ):
            rejected["near_duplicate_crop"] += 1
            continue
        kept.append(dict(candidate))
        hashes.append(crop_hash)
        image_counts[image_key] += 1
        domain_counts[domain] += 1
    return kept, dict(sorted(rejected.items()))


__all__ = [
    "deduplicate_and_cap",
    "hamming_distance",
    "review_classification",
    "select_hashed_pool_indices",
]
