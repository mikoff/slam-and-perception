"""Canonical ontology definitions and taxonomy audit for Phase 4.0."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Literal

import torch
from torch import Tensor

CategoryReadiness = Literal["evaluable", "scarce", "unsupported", "distractor"]

BASE_CATEGORIES: tuple[str, ...] = (
    "car",
    "pedestrian",
    "cyclist",
    "traffic_cone",
    "bus",
    "truck",
)

NOVEL_CATEGORIES: tuple[str, ...] = (
    "stroller",
    "dog",
    "debris",
    "wheelchair",
    "construction_barrier",
)

DISTRACTOR_CATEGORIES: tuple[str, ...] = (
    "road_surface",
    "sky",
    "vegetation",
    "building_wall",
    "ego_platform_bodywork",
    "indoor_lamp_or_light_fixture",
    "bed_and_bedding",
)

UNSUPPORTED_CATEGORIES: tuple[str, ...] = ()

EXTENDED_CATEGORIES: tuple[str, ...] = (
    "pillar",
    "trash_bin",
    "street_light",
    "parking_slot",
    "parking_meter",
    "fire_hydrant",
    "canopy",
    "cart",
    "golf_cart",
    "luggage_and_bags",
    "signboard",
    "traffic_sign",
    "traffic_light",
    "pole",
    "bench",
    "license_plate",
    "window",
    "door",
    "building",
    "tree",
    "stairs",
    "fence",
    "chair",
    "table",
    "cabinet",
    "wheeled_transport",
)

# Mutually-exclusive foreground vocabulary for SOTA sigmoid open-vocabulary detection
# (excludes colliding scene-stuff distractors and duplicate two-wheeler aliases).
OPEN_VOCAB_CATEGORIES: tuple[str, ...] = (
    BASE_CATEGORIES + NOVEL_CATEGORIES + EXTENDED_CATEGORIES
)

# Real SQLite categories intentionally held out from closed-set GT supervision
# to benchmark true zero-shot open-vocabulary generalization at test time.
ZERO_SHOT_EVAL_CATEGORIES: tuple[str, ...] = (
    "book",
    "painting",
    "cushion",
    "flowerpot",
)

# Per-dataset annotation capability matrix for Federated Open-Vocabulary Loss:
# maps each source_dataset to the canonical OPEN_VOCAB_CATEGORIES it actually labels.
# Mined background/clutter proposals from dataset `d` MUST NOT apply negative BCE
# gradients to categories outside `SOURCE_ANNOTATED_CATEGORIES[d]`.
SOURCE_ANNOTATED_CATEGORIES: dict[str, frozenset[str]] = {
    "bdd100k_images_100k": frozenset(
        {
            "car",
            "pedestrian",
            "cyclist",
            "bus",
            "truck",
            "traffic_light",
            "traffic_sign",
        }
    ),
    "nuimages": frozenset(
        {
            "car",
            "pedestrian",
            "cyclist",
            "bus",
            "truck",
            "traffic_cone",
            "construction_barrier",
            "stroller",
            "debris",
            "wheelchair",
            "wheeled_transport",
        }
    ),
    "woodscape_rgb_fisheye": frozenset(
        {
            "car",
            "pedestrian",
            "cyclist",
            "bus",
            "truck",
            "traffic_light",
            "traffic_sign",
            "pole",
            "fence",
            "wheeled_transport",
        }
    ),
    "coco_2017": frozenset(
        {
            "car",
            "pedestrian",
            "cyclist",
            "bus",
            "truck",
            "traffic_light",
            "parking_meter",
            "fire_hydrant",
            "bench",
            "dog",
            "luggage_and_bags",
            "chair",
            "table",
        }
    ),
    "ade20k": frozenset(
        {
            "car",
            "pedestrian",
            "cyclist",
            "bus",
            "truck",
            "pillar",
            "trash_bin",
            "canopy",
            "signboard",
            "traffic_light",
            "pole",
            "bench",
            "luggage_and_bags",
            "door",
            "fence",
            "chair",
            "table",
            "cabinet",
        }
    ),
    "open_images_subset": frozenset(
        {
            "car",
            "pedestrian",
            "cyclist",
            "bus",
            "truck",
        }
    ),
    "roboflow_ev_parking": frozenset(
        {
            "car",
        }
    ),
}


def build_source_annotated_mask(
    source_datasets: Sequence[str],
    categories: Sequence[str] = OPEN_VOCAB_CATEGORIES,
    *,
    device: torch.device | str | None = None,
) -> Tensor:
    """Build a boolean mask [N, C] indicating which classes are annotated per sample source."""
    num_samples = len(source_datasets)
    num_classes = len(categories)
    mask = torch.ones((num_samples, num_classes), dtype=torch.bool)
    for row_idx, src in enumerate(source_datasets):
        annotated = SOURCE_ANNOTATED_CATEGORIES.get(src)
        if annotated is not None:
            for col_idx, cat in enumerate(categories):
                if cat not in annotated:
                    mask[row_idx, col_idx] = False
    if device is not None:
        mask = mask.to(device=device)
    return mask

READINESS_STATUS: dict[str, CategoryReadiness] = {
    "car": "evaluable",
    "pedestrian": "evaluable",
    "cyclist": "evaluable",
    "traffic_cone": "evaluable",
    "bus": "evaluable",
    "truck": "evaluable",
    "stroller": "evaluable",
    "dog": "evaluable",
    "debris": "evaluable",
    "construction_barrier": "evaluable",
    "wheelchair": "evaluable",
    "empty_parking_bay": "evaluable",
    "parking_slot": "evaluable",
    "pillar": "evaluable",
    "trash_bin": "evaluable",
    "street_light": "evaluable",
    "parking_meter": "evaluable",
    "fire_hydrant": "evaluable",
    "canopy": "evaluable",
    "cart": "evaluable",
    "golf_cart": "evaluable",
    "luggage_and_bags": "evaluable",
    "signboard": "evaluable",
    "pole": "evaluable",
    "bench": "evaluable",
    "license_plate": "evaluable",
    "road_surface": "distractor",
    "sky": "distractor",
    "vegetation": "distractor",
    "building_wall": "distractor",
    "ego_platform_bodywork": "distractor",
    "indoor_lamp_or_light_fixture": "distractor",
    "bed_and_bedding": "distractor",
}

CANONICAL_MAPPINGS: dict[str, str] = {
    # Person / Pedestrian
    "person": "pedestrian",
    "adult": "pedestrian",
    "child": "pedestrian",
    "construction worker": "pedestrian",
    "pedestrian_adult": "pedestrian",
    "pedestrian_child": "pedestrian",
    "pedestrian_other": "pedestrian",
    "pedestrian_construction_worker": "pedestrian",
    "pedestrian_police_officer": "pedestrian",
    "pedestrian": "pedestrian",
    # Cyclist / Two-wheeler
    "bicycle": "cyclist",
    "bike": "cyclist",
    "motorcycle": "cyclist",
    "motor": "cyclist",
    "rider": "cyclist",
    # Vehicles
    "car": "car",
    "van": "car",
    "truck": "truck",
    "bus": "bus",
    "rigid": "bus",
    "bus_rigid": "bus",
    "bus_articulated": "bus",
    # Safety / Infrastructure
    "trafficcone": "traffic_cone",
    "traffic_cone": "traffic_cone",
    "barrier": "construction_barrier",
    "construction_barrier": "construction_barrier",
    "construction": "construction_barrier",
    # Novel Mobility & Hazards
    "stroller": "stroller",
    "dog": "dog",
    "debris": "debris",
    "wheelchair": "wheelchair",
    # Distractors
    "road_surface": "road_surface",
    "drivable_surface": "road_surface",
    "sky": "sky",
    "nature": "vegetation",
    "vegetation": "vegetation",
    "green_strip": "vegetation",
    "ego": "ego_platform_bodywork",
    "ego_vehicle": "ego_platform_bodywork",
    "ego_platform_bodywork": "ego_platform_bodywork",
    "building_wall": "building_wall",
    "fence": "fence",
    "fence_or_wall": "fence",
    "structure": "building_wall",
    "explicit_stuff": "road_surface",
    # Extended Multi-Domain Infrastructure
    "empty_parking_bay": "parking_slot",
    "parking_space": "parking_slot",
    "parking_slot": "parking_slot",
    "pillar": "pillar",
    "column": "pillar",
    "trash_bin": "trash_bin",
    "ashcan": "trash_bin",
    "waste_container": "trash_bin",
    "street_light": "street_light",
    "streetlight": "street_light",
    "lamp": "indoor_lamp_or_light_fixture",
    "sconce": "indoor_lamp_or_light_fixture",
    "chandelier": "indoor_lamp_or_light_fixture",
    "bed": "bed_and_bedding",
    "pillow": "bed_and_bedding",
    "mattress": "bed_and_bedding",
    "canopy": "canopy",
    "awning": "canopy",
    "signboard": "signboard",
    "billboard": "signboard",
    "trade_name": "signboard",
    "traffic_sign": "traffic_sign",
    "traffic sign": "traffic_sign",
    "trafficsign_indistingushable": "traffic_sign",
    "stop_sign": "traffic_sign",
    "stop sign": "traffic_sign",
    "street_sign": "traffic_sign",
    "traffic_light": "traffic_light",
    "traffic light": "traffic_light",
    "unknown_traffic_light": "traffic_light",
    "pole": "pole",
    "bench": "bench",
    "cart": "cart",
    "golf_cart": "golf_cart",
    "parking_meter": "parking_meter",
    "fire_hydrant": "fire_hydrant",
    "license_plate": "license_plate",
    "luggage_and_bags": "luggage_and_bags",
    "suitcase": "luggage_and_bags",
    "backpack": "luggage_and_bags",
    "handbag": "luggage_and_bags",
    "bag": "luggage_and_bags",
    "window": "window",
    "windowpane": "window",
    "door": "door",
    "double_door": "door",
    "building": "building",
    "house": "building",
    "edifice": "building",
    "skyscraper": "building",
    "tree": "tree",
    "trees": "tree",
    "palm": "tree",
    "stairs": "stairs",
    "stairway": "stairs",
    "steps": "stairs",
    "chair": "chair",
    "armchair": "chair",
    "swivel_chair": "chair",
    "seat": "chair",
    "sofa": "chair",
    "couch": "chair",
    "table": "table",
    "desk": "table",
    "coffee_table": "table",
    "dining_table": "table",
    "cabinet": "cabinet",
    "chest_of_drawers": "cabinet",
    "wardrobe": "cabinet",
    "shelf": "cabinet",
    "bookcase": "cabinet",
    "other_wheeled_transport": "wheeled_transport",
    "wheeled_transport": "wheeled_transport",
    "trailer": "wheeled_transport",
}

COMPATIBLE_LABELS: dict[str, tuple[str, ...]] = {
    "car": ("car", "van"),
    "pedestrian": ("pedestrian", "person", "adult", "child", "construction worker"),
    "cyclist": ("cyclist", "bicycle", "bike", "motorcycle", "rider"),
    "traffic_cone": ("traffic_cone", "trafficcone"),
    "bus": ("bus", "rigid"),
    "truck": ("truck",),
    "stroller": ("stroller",),
    "dog": ("dog",),
    "debris": ("debris",),
    "wheelchair": ("wheelchair",),
    "construction_barrier": ("construction_barrier", "barrier"),
    "parking_slot": ("parking_slot", "parking_space", "empty_parking_bay"),
    "pillar": ("pillar", "column"),
    "trash_bin": ("trash_bin", "ashcan", "waste_container"),
    "street_light": ("street_light", "streetlight"),
    "canopy": ("canopy", "awning", "sunshade"),
    "cart": ("cart", "shopping_cart", "golf_cart"),
    "road_surface": ("road_surface", "road", "street"),
    "sky": ("sky",),
    "vegetation": ("vegetation", "trees", "grass"),
    "building_wall": ("building_wall", "wall"),
    "fence": ("fence", "fencing"),
    "ego_platform_bodywork": ("ego_platform_bodywork", "car hood"),
}


def normalize_category_name(raw_name: str) -> str:
    """Normalize a category string to standard lowercase format."""
    normalized = raw_name.strip().lower()
    return normalized.replace("-", "_").replace(" ", "_")


def map_to_canonical(raw_name: str) -> str | None:
    """Map a raw dataset category to its canonical evaluation concept.

    /// @brief Resolves raw or dataset category string to canonical evaluation label.
    /// @param raw_name Category string from source dataset or SQLite index.
    /// @return Canonical category name, or None if unmapped.
    """
    cleaned = raw_name.strip().lower()
    if cleaned in CANONICAL_MAPPINGS:
        return CANONICAL_MAPPINGS[cleaned]
    norm = normalize_category_name(cleaned)
    if norm in CANONICAL_MAPPINGS:
        return CANONICAL_MAPPINGS[norm]
    # Check direct canonical membership
    all_known = (
        BASE_CATEGORIES
        + NOVEL_CATEGORIES
        + EXTENDED_CATEGORIES
        + DISTRACTOR_CATEGORIES
        + UNSUPPORTED_CATEGORIES
    )
    if norm in all_known:
        return norm
    return None


def is_compatible(prediction: str, ground_truth: str) -> bool:
    """Check if prediction is semantically compatible with ground-truth concept.

    /// @brief Evaluates whether a predicted class label is compatible with GT.
    /// @param prediction Predicted category name.
    /// @param ground_truth Canonical ground truth category name.
    /// @return True if prediction matches or is in compatible labels set.
    """
    if prediction == ground_truth:
        return True
    compatible = COMPATIBLE_LABELS.get(ground_truth, ())
    return prediction in compatible or map_to_canonical(prediction) == ground_truth


@dataclass(frozen=True)
class ConceptAuditRecord:
    """Audit summary for one semantic concept."""

    concept: str
    readiness: CategoryReadiness
    is_base: bool
    is_novel: bool
    is_unsupported: bool
    validation_instances: int
    frozen800_instances: int
    compatible_sources: tuple[str, ...]


__all__ = [
    "BASE_CATEGORIES",
    "build_source_annotated_mask",
    "CANONICAL_MAPPINGS",
    "CategoryReadiness",
    "COMPATIBLE_LABELS",
    "ConceptAuditRecord",
    "DISTRACTOR_CATEGORIES",
    "EXTENDED_CATEGORIES",
    "is_compatible",
    "map_to_canonical",
    "normalize_category_name",
    "NOVEL_CATEGORIES",
    "OPEN_VOCAB_CATEGORIES",
    "READINESS_STATUS",
    "SOURCE_ANNOTATED_CATEGORIES",
    "UNSUPPORTED_CATEGORIES",
    "ZERO_SHOT_EVAL_CATEGORIES",
]
