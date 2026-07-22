from __future__ import annotations

from pathlib import Path
from typing import Any

from .config import Config, DatasetConfig
from .progress import Progress
from .reports import read_json, write_json


def _dataset_dirs(root: Path) -> list[Path]:
    result = []
    for child in root.iterdir():
        if child.is_dir() and (child / "img").is_dir() and (child / "ann").is_dir():
            result.append(child)
    return sorted(result)


def validate_project(path: Path, sdk_check: bool = True) -> tuple[bool, str]:
    try:
        data = read_json(path / "meta.json")
        if not isinstance(data.get("classes", []), list):
            return False, "meta.json has invalid classes"
        if not _dataset_dirs(path):
            return False, "no dataset with img and ann directories"
        if sdk_check:
            import supervisely as sly
            sly.Project(str(path), sly.OpenMode.READ)
    except Exception as exc:
        return False, str(exc)
    return True, "valid"


def discover_candidates(root: Path, sdk_check: bool = True) -> list[dict[str, Any]]:
    result = []
    for meta in sorted(root.rglob("meta.json")):
        valid, reason = validate_project(meta.parent, sdk_check=sdk_check)
        result.append({"path": str(meta.parent.resolve()), "valid": valid, "reason": reason})
    return result


def _manifest_project(dataset: DatasetConfig) -> Path | None:
    manifest_path = dataset.extracted_dir / ".dataset_pipeline_manifest.json"
    if not manifest_path.exists():
        return None
    try:
        selected = read_json(manifest_path).get("discovered_project_root")
        path = Path(selected).resolve() if selected else None
        if path is None:
            return None
        path.relative_to(dataset.extracted_dir.resolve())
        valid, _ = validate_project(path, sdk_check=False)
        return path if valid else None
    except (OSError, ValueError, TypeError, AttributeError):
        return None


def project_root(
    dataset: DatasetConfig, sdk_check: bool = True,
    candidates: list[dict[str, Any]] | None = None,
) -> Path:
    if dataset.project_subpath:
        path = (dataset.extracted_dir / dataset.project_subpath).resolve()
        try:
            path.relative_to(dataset.extracted_dir.resolve())
        except ValueError as exc:
            raise ValueError(f"{dataset.name}.project_subpath escapes extracted_dir") from exc
        valid, reason = validate_project(path, sdk_check)
        if not valid:
            raise RuntimeError(f"Configured project is invalid: {path}: {reason}")
        return path
    if candidates is None and (manifest_project := _manifest_project(dataset)):
        return manifest_project
    candidates = candidates if candidates is not None else discover_candidates(dataset.extracted_dir, sdk_check)
    valid = [Path(item["path"]) for item in candidates if item["valid"]]
    if len(valid) != 1:
        raise RuntimeError(f"{dataset.name}: expected exactly one valid Supervisely project; found {valid}. Set project_subpath.")
    return valid[0]


def discover_all(config: Config, dataset_name: str | None = None, sdk_check: bool = True) -> list[dict[str, Any]]:
    report = []
    for dataset in config.selected(dataset_name):
        candidates = discover_candidates(dataset.extracted_dir, sdk_check)
        selected = None
        error = None
        try:
            selected = str(project_root(dataset, sdk_check, candidates))
            manifest_path = dataset.extracted_dir / ".dataset_pipeline_manifest.json"
            if manifest_path.exists():
                manifest = read_json(manifest_path)
                manifest["discovered_project_root"] = selected
                write_json(manifest_path, manifest)
        except RuntimeError as exc:
            error = str(exc)
        report.append({"dataset": dataset.name, "selected": selected, "candidates": candidates, "error": error})
    write_json(config.reports / "project_discovery.json", report)
    if any(item["error"] for item in report):
        raise RuntimeError("\n".join(item["error"] for item in report if item["error"]))
    return report


def inspect_project(path: Path) -> dict[str, Any]:
    meta = read_json(path / "meta.json")
    splits, extensions, geometries = [], set(), set()
    image_count = annotated = annotation_count = 0
    invalid = []
    progress = Progress(f"Inspecting project {path.name}", "images")
    for dataset in _dataset_dirs(path):
        split = {"name": dataset.name, "images": 0, "annotations": 0}
        for image in sorted((dataset / "img").iterdir()):
            if not image.is_file():
                continue
            image_count += 1
            split["images"] += 1
            extensions.add(image.suffix.lower())
            ann = dataset / "ann" / f"{image.name}.json"
            if not ann.exists():
                invalid.append(str(ann))
                progress.add()
                continue
            try:
                data = read_json(ann)
                labels = data.get("objects", [])
                annotated += bool(labels)
                annotation_count += len(labels)
                split["annotations"] += len(labels)
                geometries.update(label.get("geometryType", "unknown") for label in labels)
            except (OSError, ValueError):
                invalid.append(str(ann))
            progress.add()
        splits.append(split)
    progress.finish()
    physical = 0
    storage_progress = Progress(f"Measuring project {path.name}", "files")
    for item in path.rglob("*"):
        if item.is_file() and not item.is_symlink():
            physical += item.stat().st_size
            storage_progress.add()
    storage_progress.finish()
    return {
        "project_name": path.name, "project_path": str(path), "splits": splits,
        "image_count": image_count, "annotated_image_count": annotated,
        "annotation_count": annotation_count, "object_classes": meta.get("classes", []),
        "geometry_types": sorted(geometries), "image_extensions": sorted(extensions),
        "source_storage_size": physical, "invalid_or_missing_annotations": invalid,
    }


def inspect_projects(config: Config, taxonomy: Any, dataset_name: str | None = None, sdk_check: bool = True) -> list[dict[str, Any]]:
    report, source, ignored, unmapped = [], {}, {}, {}
    for dataset in config.selected(dataset_name):
        item = inspect_project(project_root(dataset, sdk_check))
        classes = item["object_classes"]
        source[dataset.name] = sorted(c["title"] for c in classes)
        try:
            mappings = taxonomy.validate_classes(dataset.name, classes)
            ignored[dataset.name] = sorted(m.source_category for m in mappings if m.ignored)
            if not any(not m.ignored for m in mappings):
                raise ValueError(f"{dataset.name}: no retained object classes")
        except Exception as exc:
            unmapped[dataset.name] = [str(exc)]
        report.append({"dataset": dataset.name, **item})
    write_json(config.reports / "source_classes.json", source)
    write_json(config.reports / "ignored_classes.json", ignored)
    write_json(config.reports / "unmapped_classes.json", unmapped)
    write_json(config.reports / "project_inspection.json", report)
    if unmapped:
        raise ValueError(f"Taxonomy validation failed: {unmapped}")
    return report
