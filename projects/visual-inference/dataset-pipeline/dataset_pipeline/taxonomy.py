from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .reports import read_json


class UnmappedCategoryError(ValueError):
    pass


@dataclass(frozen=True)
class MappingResult:
    source_category: str
    normalized: str
    canonical: str | None
    category_id: int | None
    ignored: bool
    ignore_region: bool


class Taxonomy:
    def __init__(self, data: dict[str, Any]):
        self.data = dict(data)
        self.ignore_token = data.get("ignore_token", "__ignore__")
        self.ignore_region_token = data.get("tokens", {}).get(
            "ignore_region", "__ignore_region__"
        )
        self.ignore_tokens = set(data.get("tokens", {}).values()) | {self.ignore_token}
        order = data.get("canonical_id_order") or data["semantic_category_id_order"]
        self.data["canonical_id_order"] = order
        if len(order) != len(set(order)):
            raise ValueError("canonical_id_order contains duplicates")
        self.category_ids = {name: index + 1 for index, name in enumerate(order)}
        self.ignore_region_category_id = len(self.category_ids) + 1
        self.identity_datasets = set(data.get("identity_datasets", []))
        guards = data.get("mapping_guards", {})
        required_guard_terms = {
            "nuimages.other": ("pedestrian",),
            "woodscape_rgb_fisheye.construction": ("construction",),
        }
        for guard, terms in required_guard_terms.items():
            statement = str(guards.get(guard, "")).lower()
            if not all(term in statement for term in terms):
                raise ValueError(f"Taxonomy mapping guard {guard!r} is missing required semantic assertions")
        for dataset, mapping in data.get("dataset_mappings", {}).items():
            unknown = {self._mapping_target(value) for value in mapping.values()} - set(order) - self.ignore_tokens
            if unknown:
                raise ValueError(f"{dataset} maps to unknown canonical categories: {sorted(unknown)}")

    def _mapping_target(self, value: Any) -> str:
        if isinstance(value, str):
            return value
        if isinstance(value, dict):
            if str(value.get("action", "")).startswith("ignore"):
                return self.ignore_token
            if value.get("action") == "keep" and isinstance(value.get("concept"), str):
                return value["concept"]
        raise ValueError(f"Invalid taxonomy mapping entry: {value!r}")

    @classmethod
    def load(cls, path: str | Path) -> "Taxonomy":
        return cls(read_json(Path(path)))

    @property
    def categories(self) -> list[dict[str, Any]]:
        categories = [
            {"id": category_id, "name": name, "supercategory": "object"}
            for name, category_id in self.category_ids.items()
        ]
        categories.append({
            "id": self.ignore_region_category_id,
            "name": self.ignore_region_token,
            "supercategory": "ignore",
            "ignore_region": True,
        })
        return categories

    @property
    def configured_category_ids(self) -> set[int]:
        return {*self.category_ids.values(), self.ignore_region_category_id}

    def normalize(self, name: str) -> str:
        rules = self.data.get("name_normalization", {})
        value = name.strip() if rules.get("trim", True) else name
        value = value.lower() if rules.get("lowercase", True) else value
        replacement = rules.get("replace_non_alphanumeric_with", "_")
        value = re.sub(r"[^a-zA-Z0-9]+", replacement, value)
        if rules.get("collapse_repeated_underscores", True):
            value = re.sub(r"_+", "_", value)
        return value.strip("_") if rules.get("strip_leading_trailing_underscores", True) else value

    def map(self, dataset: str, source_name: str, description: str | None = None) -> MappingResult:
        normalized = self.normalize(source_name)
        if dataset in self.identity_datasets:
            canonical = normalized
            if canonical not in self.category_ids:
                raise UnmappedCategoryError(f"{dataset}: unexpected identity category {source_name!r}")
        else:
            mapping = self.data.get("dataset_mappings", {}).get(dataset)
            if mapping is None or normalized not in mapping:
                raise UnmappedCategoryError(f"{dataset}: unmapped category {source_name!r} ({normalized!r})")
            canonical = self._mapping_target(mapping[normalized])
        if canonical == self.ignore_region_token:
            return MappingResult(
                source_name,
                normalized,
                self.ignore_region_token,
                self.ignore_region_category_id,
                False,
                True,
            )
        if canonical in self.ignore_tokens:
            return MappingResult(source_name, normalized, None, None, True, False)
        self._check_guard(dataset, normalized, description)
        return MappingResult(
            source_name,
            normalized,
            canonical,
            self.category_ids[canonical],
            False,
            False,
        )

    def _check_guard(self, dataset: str, normalized: str, description: str | None) -> None:
        if dataset == "nuimages" and normalized == "other":
            if description and "pedestrian" not in description.lower():
                raise ValueError("nuimages 'other' guard failed: metadata must describe a pedestrian class")
        if dataset == "woodscape_rgb_fisheye" and normalized == "construction":
            if description and not any(token in description.lower() for token in ("vehicle", "instance", "construction")):
                raise ValueError("WoodScape 'construction' guard contradicts construction-vehicle assumption")

    def validate_classes(self, dataset: str, classes: list[dict[str, Any]]) -> list[MappingResult]:
        return [self.map(dataset, item["title"], item.get("description")) for item in classes]
