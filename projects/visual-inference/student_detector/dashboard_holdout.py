"""Audit dashboard images against the entire current two-stage training cohort."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sqlite3
from typing import Any

from PIL import Image
import torch

from .config import AugmentationConfig
from .data import ProposalSample, ProposalTransform
from .human_gt_data import get_stage2_cohort_ids


def file_sha256(path: Path) -> str:
    """Hash file contents without loading a whole image archive into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def training_inventory(
    gt_index: Path,
    image_root: Path,
    cohort_size: int,
    val_ratio: float = 0.0,
    seed: int = 42,
) -> list[dict[str, Any]]:
    """Exclude Stage-1 training cohort images (defaulting to whole cohort when val_ratio=0)."""
    ids = get_stage2_cohort_ids(gt_index, cohort_size)
    with sqlite3.connect(f"file:{gt_index}?mode=ro", uri=True) as conn:
        rows = conn.execute(
            "SELECT image_id, file_name, source_dataset FROM images ORDER BY image_id"
        ).fetchall()
    all_records = [
        {"image_id": int(iid), "file_name": str(name), "source_dataset": str(source)}
        for iid, name, source in rows
        if int(iid) in ids
    ]
    if val_ratio > 0.0:
        from .sam_student_data import split_cohort_indices

        train_indices, _ = split_cohort_indices(
            all_records, val_ratio=val_ratio, seed=seed
        )
        records = [all_records[i] for i in train_indices]
    else:
        records = all_records

    with ThreadPoolExecutor(max_workers=4) as pool:
        hashes = pool.map(
            file_sha256, (image_root / row["file_name"] for row in records)
        )
        for row, digest in zip(records, hashes, strict=True):
            row["sha256"] = digest
    return records


def validation_inventory(pseudo_labels: Path, image_root: Path) -> list[dict[str, Any]]:
    """Hash images passed to both stages as their external validation set."""
    data = json.loads(pseudo_labels.read_text())
    records = [
        {
            "image_id": None,
            "file_name": str(row["file_name"]),
            "source_dataset": str(row["source_dataset"]),
        }
        for row in data["images"]
    ]
    with ThreadPoolExecutor(max_workers=4) as pool:
        hashes = pool.map(
            file_sha256, (image_root / row["file_name"] for row in records)
        )
        for row, digest in zip(records, hashes, strict=True):
            row["sha256"] = digest
    return records


def audit_records(
    records: list[dict[str, Any]],
    training: list[dict[str, Any]],
    image_root: Path,
) -> None:
    """Reject training IDs, aliased files, byte duplicates, and duplicate cards."""
    ids = {r["image_id"] for r in training if r["image_id"] is not None}
    paths = {(image_root / r["file_name"]).resolve() for r in training}
    hashes = {r["sha256"] for r in training}
    selected_ids: set[int] = set()
    selected_hashes: set[str] = set()
    for row in records:
        path = (image_root / row["file_name"]).resolve()
        digest = file_sha256(path)
        if row["image_id"] in ids or path in paths or digest in hashes:
            raise ValueError(f"Training overlap: {row['image_id']} ({path})")
        if row["image_id"] in selected_ids or digest in selected_hashes:
            raise ValueError(f"Duplicate evaluation image: {row['image_id']}")
        if row.get("sha256", digest) != digest:
            raise ValueError(f"Evaluation image changed: {path}")
        row["sha256"] = digest
        selected_ids.add(row["image_id"])
        selected_hashes.add(digest)


def load_holdout(
    manifest_path: Path, image_root: Path
) -> tuple[list[tuple[ProposalSample, dict[str, Any]]], dict[str, Any]]:
    """Re-audit a frozen manifest and apply the same inference letterboxing."""
    manifest = json.loads(manifest_path.read_text())
    val_ratio = float(manifest.get("pipeline_val_ratio", 0.0))
    seed = int(manifest.get("split_seed", 42))
    training = training_inventory(
        Path(manifest["gt_index"]),
        image_root,
        manifest["cohort_size"],
        val_ratio=val_ratio,
        seed=seed,
    )
    if training != manifest["excluded_training_images"]:
        raise ValueError("Training inventory changed; rebuild the evaluation manifest")
    validation = validation_inventory(
        Path(manifest["validation_pseudo_labels"]), image_root
    )
    if validation != manifest["excluded_validation_images"]:
        raise ValueError(
            "Validation inventory changed; rebuild the evaluation manifest"
        )
    records = manifest["images"]
    audit_records(records, training + validation, image_root)
    transform = ProposalTransform(
        384, AugmentationConfig(), training=False, tiny_area=4, tiny_min_side=2
    )
    items = []
    for row in records:
        with Image.open(image_root / row["file_name"]) as image:
            transformed = transform(
                image, torch.empty((0, 4)), torch.empty((0, 4)), seed=42
            )
        items.append(
            (
                ProposalSample(
                    image=transformed[0],
                    boxes=transformed[1],
                    ignore_boxes=transformed[2],
                    valid_mask=transformed[4],
                    image_id=row["image_id"],
                    source_dataset=row["source_dataset"],
                    domain="general",
                    camera_type=row["camera_type"],
                    background_supervision=False,
                    original_size=(row["height"], row["width"]),
                    transform=transformed[5],
                ),
                row,
            )
        )
    return items, manifest

