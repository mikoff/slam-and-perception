"""Shared non-destructive human-GT annotation hygiene rules.

Fixes two verified corruption patterns in `quad_train_enriched.sqlite` in memory
without modifying the read-only SQLite file or disturbing 1-to-1 `gt_indices`
cache alignment:
1. Cross-tier `source_hbb` + `fitted_quad` duplicates and occluded multi-fragment
   sub-pieces in `nuimages` and `coco_2017`.
2. Semantic-mask connected-component slivers, nested fragments, and catch-all
   classes in `woodscape_rgb_fisheye`.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass

CROSS_TIER_DEDUP_DATASETS: frozenset[str] = frozenset({"nuimages", "coco_2017"})
WOODSCAPE_DATASET: str = "woodscape_rgb_fisheye"
WOODSCAPE_IGNORE_RAW_CATEGORIES: frozenset[str] = frozenset(
    {
        "trafficsign_indistingushable",
        "other_wheeled_transport",
        "movable_object",
    }
)
WOODSCAPE_MIN_SIDE: float = 16.0
WOODSCAPE_MIN_AREA: float = 256.0
WOODSCAPE_POLE_MIN_MAJOR: float = 32.0
WOODSCAPE_POLE_MIN_AREA: float = 320.0
WOODSCAPE_NESTED_IOMIN: float = 0.70


@dataclass(frozen=True)
class RawAnnotationRow:
    """Minimal raw annotation tuple evaluated by annotation hygiene rules."""

    x1: float
    y1: float
    x2: float
    y2: float
    ignore_region: int
    category_name: str
    geometry_tier: str = "unknown"


@dataclass(frozen=True)
class HygieneDecision:
    """Per-image annotation filtering and demotion result preserving row order."""

    kept_positive_indices: tuple[int, ...]
    demoted_to_ignore_indices: tuple[int, ...]
    dropped_indices: tuple[int, ...]
    stats: dict[str, int]


def _box_area(row: RawAnnotationRow) -> float:
    w = max(float(row.x2) - float(row.x1), 0.0)
    h = max(float(row.y2) - float(row.y1), 0.0)
    return w * h


def _intersection_area(a: RawAnnotationRow, b: RawAnnotationRow) -> float:
    ix1 = max(float(a.x1), float(b.x1))
    iy1 = max(float(a.y1), float(b.y1))
    ix2 = min(float(a.x2), float(b.x2))
    iy2 = min(float(a.y2), float(b.y2))
    iw = max(ix2 - ix1, 0.0)
    ih = max(iy2 - iy1, 0.0)
    return iw * ih


def _box_iou(a: RawAnnotationRow, b: RawAnnotationRow) -> float:
    inter = _intersection_area(a, b)
    if inter <= 0.0:
        return 0.0
    union = _box_area(a) + _box_area(b) - inter
    return inter / max(union, 1e-6)


def _box_iomin(a: RawAnnotationRow, b: RawAnnotationRow) -> float:
    inter = _intersection_area(a, b)
    if inter <= 0.0:
        return 0.0
    return inter / max(min(_box_area(a), _box_area(b)), 1e-6)


def sanitize_image_annotations(
    rows: Sequence[RawAnnotationRow],
    source_dataset: str,
) -> HygieneDecision:
    """Apply cross-tier deduplication and WoodScape fragment hygiene rules.

    /// @brief Cleans active (`ignore_region == 0`) annotations in memory while leaving
    /// non-positive (`ignore_region != 0`) rows untouched.
    /// @param rows Ordered sequence of RawAnnotationRow for a single image.
    /// @param source_dataset Canonical dataset identifier (e.g. `nuimages`, `woodscape_rgb_fisheye`).
    /// @return HygieneDecision containing kept, demoted-to-ignore, and dropped indices plus stats.
    """
    active_indices: list[int] = [
        idx for idx, row in enumerate(rows) if int(row.ignore_region) == 0
    ]
    raw_positives = len(active_indices)
    dropped: set[int] = set()
    demoted: set[int] = set()

    stats: dict[str, int] = {
        "raw_positives": raw_positives,
        "kept_positives": 0,
        "dropped_dup_hbb": 0,
        "dropped_occluded_quad_fragments": 0,
        "woodscape_demoted_noisy_class": 0,
        "woodscape_demoted_tiny_fragment": 0,
        "woodscape_demoted_nested_fragment": 0,
    }

    if not active_indices:
        return HygieneDecision(
            kept_positive_indices=(),
            demoted_to_ignore_indices=(),
            dropped_indices=(),
            stats=stats,
        )

    # Rule 1: Cross-tier deduplication & occluded-fragment collapse
    if source_dataset in CROSS_TIER_DEDUP_DATASETS:
        hbb_indices = [
            i for i in active_indices if rows[i].geometry_tier == "source_hbb"
        ]
        quad_indices = [
            i
            for i in active_indices
            if rows[i].geometry_tier not in {"source_hbb", "unknown", ""}
        ]
        if hbb_indices and quad_indices:
            children_by_hbb: dict[int, list[int]] = defaultdict(list)
            for q_idx in quad_indices:
                q_row = rows[q_idx]
                q_cat = q_row.category_name.strip().lower()
                best_h: int | None = None
                best_key = (-1.0, -1.0)
                for h_idx in hbb_indices:
                    h_row = rows[h_idx]
                    if h_row.category_name.strip().lower() != q_cat:
                        continue
                    iou_val = _box_iou(h_row, q_row)
                    iomin_val = _box_iomin(h_row, q_row)
                    if iou_val >= 0.30 or iomin_val >= 0.70:
                        score_key = (iou_val, iomin_val)
                        if score_key > best_key:
                            best_key = score_key
                            best_h = h_idx
                if best_h is not None:
                    children_by_hbb[best_h].append(q_idx)

            for h_idx, children in children_by_hbb.items():
                if len(children) == 1 and _box_iou(rows[h_idx], rows[children[0]]) >= 0.50:
                    dropped.add(h_idx)
                    stats["dropped_dup_hbb"] += 1
                elif children:
                    for c_idx in children:
                        if c_idx not in dropped:
                            dropped.add(c_idx)
                            stats["dropped_occluded_quad_fragments"] += 1

    # Rule 2: WoodScape semantic-mask fragment & catch-all sanitization
    if source_dataset == WOODSCAPE_DATASET:
        surviving: list[int] = []
        for idx in active_indices:
            if idx in dropped:
                continue
            row = rows[idx]
            cat = row.category_name.strip().lower()
            w = max(float(row.x2) - float(row.x1), 0.0)
            h = max(float(row.y2) - float(row.y1), 0.0)
            area = w * h
            if cat in WOODSCAPE_IGNORE_RAW_CATEGORIES:
                demoted.add(idx)
                stats["woodscape_demoted_noisy_class"] += 1
            elif cat == "pole":
                if max(w, h) < WOODSCAPE_POLE_MIN_MAJOR or area < WOODSCAPE_POLE_MIN_AREA:
                    demoted.add(idx)
                    stats["woodscape_demoted_tiny_fragment"] += 1
                else:
                    surviving.append(idx)
            else:
                if min(w, h) < WOODSCAPE_MIN_SIDE or area < WOODSCAPE_MIN_AREA:
                    demoted.add(idx)
                    stats["woodscape_demoted_tiny_fragment"] += 1
                else:
                    surviving.append(idx)

        by_cat: dict[str, list[int]] = defaultdict(list)
        for idx in surviving:
            by_cat[rows[idx].category_name.strip().lower()].append(idx)

        for cat_indices in by_cat.values():
            ordered = sorted(
                cat_indices,
                key=lambda i: (-_box_area(rows[i]), i),
            )
            accepted: list[int] = []
            for idx in ordered:
                cur_row = rows[idx]
                if any(
                    _box_iomin(cur_row, rows[kept_idx]) >= WOODSCAPE_NESTED_IOMIN
                    for kept_idx in accepted
                ):
                    demoted.add(idx)
                    stats["woodscape_demoted_nested_fragment"] += 1
                else:
                    accepted.append(idx)

    kept_positive_indices = tuple(
        idx for idx in active_indices if idx not in dropped and idx not in demoted
    )
    demoted_to_ignore_indices = tuple(sorted(demoted))
    dropped_indices = tuple(sorted(dropped))
    stats["kept_positives"] = len(kept_positive_indices)

    return HygieneDecision(
        kept_positive_indices=kept_positive_indices,
        demoted_to_ignore_indices=demoted_to_ignore_indices,
        dropped_indices=dropped_indices,
        stats=stats,
    )


__all__ = [
    "CROSS_TIER_DEDUP_DATASETS",
    "HygieneDecision",
    "RawAnnotationRow",
    "WOODSCAPE_DATASET",
    "WOODSCAPE_IGNORE_RAW_CATEGORIES",
    "WOODSCAPE_MIN_AREA",
    "WOODSCAPE_MIN_SIDE",
    "WOODSCAPE_NESTED_IOMIN",
    "WOODSCAPE_POLE_MIN_AREA",
    "WOODSCAPE_POLE_MIN_MAJOR",
    "sanitize_image_annotations",
]
