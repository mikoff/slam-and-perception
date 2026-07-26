from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class DatasetConfig:
    name: str
    archive: Path
    extracted_dir: Path
    project_subpath: str | None = None


@dataclass(frozen=True)
class Config:
    path: Path
    workspace_root: Path
    datasets: dict[str, DatasetConfig]
    storage: dict[str, Any]
    splits: dict[str, list[str]]
    validation: dict[str, Any]
    taxonomy_path: Path

    @property
    def reports(self) -> Path:
        return self.workspace_root / "reports"

    def selected(self, name: str | None = None) -> list[DatasetConfig]:
        if name is None:
            return list(self.datasets.values())
        if name not in self.datasets:
            raise ValueError(f"Unknown dataset {name!r}; choose from {', '.join(self.datasets)}")
        return [self.datasets[name]]


DEFAULT_STORAGE = {
    "image_link_mode": "symlink",
    "relative_symlinks": True,
    "delete_intermediate_projects_after_export": True,
}
DEFAULT_SPLITS = {
    "train_aliases": ["train", "training", "train2017"],
    "val_aliases": ["val", "valid", "validation", "val2017"],
    "test_aliases": ["test", "test2017"],
}
DEFAULT_VALIDATION = {
    "preview_count": 100,
    "random_seed": 42,
    "clip_boxes_to_image": True,
    "validation_fraction": 0.1,
    "source_test_as_validation": ["woodscape_rgb_fisheye"],
}


def _options(data: dict[str, Any], name: str, defaults: dict[str, Any]) -> dict[str, Any]:
    supplied = data.get(name) or {}
    if not isinstance(supplied, dict):
        raise ValueError(f"{name} must be a mapping")
    unknown = set(supplied) - set(defaults)
    if unknown:
        raise ValueError(f"Unknown {name} options: {', '.join(sorted(unknown))}")
    return {**defaults, **supplied}


def load_config(path: str | Path) -> Config:
    config_path = Path(path).expanduser().resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if "workspace_root" not in data or not isinstance(data.get("datasets"), dict):
        raise ValueError("Configuration requires workspace_root and a datasets mapping")
    root_value = Path(data["workspace_root"]).expanduser()
    root = (config_path.parent / root_value).resolve() if not root_value.is_absolute() else root_value.resolve()
    datasets: dict[str, DatasetConfig] = {}
    for name, item in data["datasets"].items():
        archive = Path(item["archive"]).expanduser()
        if not archive.is_absolute():
            archive = (root / archive).resolve()
        extracted = Path(item.get("extracted_dir", f"raw/{name}"))
        if not extracted.is_absolute():
            extracted = (root / extracted).resolve()
        try:
            extracted.relative_to(root)
        except ValueError as exc:
            raise ValueError(f"{name}.extracted_dir must be inside workspace_root") from exc
        datasets[name] = DatasetConfig(name, archive.resolve(), extracted, item.get("project_subpath"))
    taxonomy = Path(data.get("taxonomy", config_path.parent / "automotive_taxonomy_mapping.json"))
    if not taxonomy.is_absolute():
        taxonomy = (config_path.parent / taxonomy).resolve()
    storage = _options(data, "storage", DEFAULT_STORAGE)
    splits = _options(data, "splits", DEFAULT_SPLITS)
    validation = _options(data, "validation", DEFAULT_VALIDATION)
    if storage["image_link_mode"] not in {"symlink", "hardlink", "copy"}:
        raise ValueError("storage.image_link_mode must be symlink, hardlink, or copy")
    if not isinstance(validation["preview_count"], int) or validation["preview_count"] < 1:
        raise ValueError("validation.preview_count must be a positive integer")
    fraction = validation["validation_fraction"]
    if not isinstance(fraction, (int, float)) or not 0 < fraction < 1:
        raise ValueError("validation.validation_fraction must be between 0 and 1")
    if (
        not isinstance(validation["source_test_as_validation"], list)
        or any(not isinstance(name, str) for name in validation["source_test_as_validation"])
    ):
        raise ValueError("validation.source_test_as_validation must be a list of dataset names")
    if any(
        not isinstance(aliases, list) or not aliases
        or any(not isinstance(alias, str) or not alias for alias in aliases)
        for aliases in splits.values()
    ):
        raise ValueError("split aliases must be non-empty lists of strings")
    return Config(config_path, root, datasets, storage, splits, validation, taxonomy)


def ensure_workspace(config: Config) -> None:
    for relative in (
        "raw", "intermediate/filtered", "intermediate/detection",
        "intermediate/coco", "output/annotations", "output/images/train",
        "output/images/val", "reports/previews",
    ):
        (config.workspace_root / relative).mkdir(parents=True, exist_ok=True)
