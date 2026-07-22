from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

from .config import Config
from .reports import read_json, write_json


def physical_size(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(item.stat(follow_symlinks=False).st_blocks * 512 for item in path.rglob("*"))


def disk_usage(config: Config) -> dict[str, Any]:
    archives = sum(item.archive.stat().st_size for item in config.datasets.values() if item.archive.exists())
    raw = sum(physical_size(item.extracted_dir) for item in config.datasets.values())
    intermediate = config.workspace_root / "intermediate"
    output = config.workspace_root / "output"
    previews = config.reports / "previews"
    symlinks = [p for root in (intermediate, output / "images") if root.exists() for p in root.rglob("*") if p.is_symlink()]
    physical_intermediate_images = sum(
        p.stat().st_blocks * 512
        for project_type in ("filtered", "detection")
        for image_dir in (intermediate / project_type).rglob("img")
        if image_dir.is_dir()
        for p in image_dir.iterdir()
        if p.is_file() and not p.is_symlink()
    )
    estimated_copy = sum(p.resolve().stat().st_size for p in symlinks if p.exists())
    result = {
        "archive_storage": archives, "extracted_raw_storage": raw,
        "intermediate_annotation_storage": sum(p.stat().st_size for p in intermediate.rglob("*") if p.is_file() and not p.is_symlink()),
        "actual_physical_intermediate_image_storage": physical_intermediate_images,
        "final_annotation_storage": physical_size(output / "annotations"),
        "final_preview_storage": physical_size(previews),
        "apparent_symlink_tree_size": sum(p.lstat().st_size for p in symlinks),
        "physical_blocks_used_by_symlink_trees": sum(p.lstat().st_blocks * 512 for p in symlinks),
        "estimated_image_copy_storage": estimated_copy,
        "estimated_disk_space_saved": max(0, estimated_copy - sum(p.lstat().st_blocks * 512 for p in symlinks)),
    }
    write_json(config.reports / "disk_usage.json", result)
    return result


def cleanup(config: Config, dry_run: bool = False) -> list[dict[str, Any]]:
    if not config.storage["delete_intermediate_projects_after_export"]:
        return []
    validation = config.reports / "validation_summary.json"
    preview = config.reports / "previews" / "index.html"
    final_annotations = [
        config.workspace_root / "output" / "annotations" / f"instances_{split}.json"
        for split in ("train", "val")
    ]
    required = [
        validation,
        preview,
        config.reports / "archive_inventory.json",
        *final_annotations,
    ]
    if not all(path.exists() for path in required):
        raise RuntimeError("Cleanup prerequisites are missing")
    if not read_json(validation).get("valid"):
        raise RuntimeError("Cleanup requires successful final validation")
    newest_output = max(path.stat().st_mtime_ns for path in final_annotations)
    if validation.stat().st_mtime_ns < newest_output or preview.stat().st_mtime_ns < newest_output:
        raise RuntimeError("Cleanup reports are older than the final annotations; rerun validation and preview")
    targets = [
        config.workspace_root / "intermediate" / "filtered",
        config.workspace_root / "intermediate" / "detection",
    ]
    plan = [
        {"path": str(path), "estimated_reclaimed_bytes": physical_size(path), "reason": "validated derived Supervisely project"}
        for path in targets if path.exists()
    ]
    write_json(config.reports / "cleanup_plan.json", plan)
    if not dry_run:
        for path in targets:
            if path.exists():
                shutil.rmtree(path)
    return plan
