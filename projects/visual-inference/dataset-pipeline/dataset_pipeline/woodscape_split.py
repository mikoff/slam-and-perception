"""Deterministic labeled WoodScape holdout from timestamp-defined sequences."""

from __future__ import annotations

import ast
import hashlib
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .reports import read_json


@dataclass(frozen=True)
class SplitAssignment:
    generated_split: str
    sequence_identity: str


def _vehicle_timestamp(annotation: dict[str, Any]) -> int:
    value = next(
        (
            tag.get("value")
            for tag in annotation.get("tags", [])
            if tag.get("name") == "vehicle info"
        ),
        None,
    )
    if not isinstance(value, str):
        raise ValueError("WoodScape RGB annotation is missing vehicle info")
    try:
        parsed = ast.literal_eval(value)
        return int(parsed["timestamp"])
    except (ValueError, SyntaxError, KeyError, TypeError) as exc:
        raise ValueError("WoodScape vehicle info has an invalid timestamp") from exc


def build_woodscape_split(
    project: Path,
    validation_fraction: float,
    seed: int,
    *,
    sequence_gap: int = 1_000_000,
) -> tuple[dict[tuple[str, str], SplitAssignment], dict[str, Any]]:
    """Assign labeled RGB sequences to train/val and test data to qualitative."""
    rgb_rows: list[tuple[int, str]] = []
    for annotation_path in sorted((project / "train" / "ann").glob("rgb_*.json")):
        rgb_rows.append(
            (_vehicle_timestamp(read_json(annotation_path)), annotation_path.stem)
        )
    if not rgb_rows:
        raise ValueError("WoodScape has no labeled RGB training annotations")

    timestamps = sorted({timestamp for timestamp, _ in rgb_rows})
    sequence_by_timestamp: dict[int, str] = {}
    sequence_index = 0
    previous = None
    for timestamp in timestamps:
        if previous is not None and timestamp - previous > sequence_gap:
            sequence_index += 1
        sequence_by_timestamp[timestamp] = f"rgb-sequence-{sequence_index:04d}"
        previous = timestamp

    images_by_sequence: dict[str, list[str]] = defaultdict(list)
    for timestamp, annotation_stem in rgb_rows:
        image_name = annotation_stem
        images_by_sequence[sequence_by_timestamp[timestamp]].append(image_name)
    target = round(len(rgb_rows) * validation_fraction)
    ordered_sequences = sorted(
        images_by_sequence,
        key=lambda sequence: (
            hashlib.sha256(f"{seed}:{sequence}".encode()).hexdigest(),
            sequence,
        ),
    )
    # Select whole sequences while staying at or below the requested image
    # count.  A hash-order greedy selection can choose one abnormally long
    # drive and turn a 10% holdout into most of the corpus.  This bounded
    # subset-sum is cheap for WoodScape (194 sequences, target about 823) and
    # remains deterministic because hash order resolves equivalent sums.
    parents: dict[int, tuple[int, str] | None] = {0: None}
    for sequence in ordered_sequences:
        size = len(images_by_sequence[sequence])
        for prior in sorted(parents, reverse=True):
            candidate = prior + size
            if candidate <= target and candidate not in parents:
                parents[candidate] = (prior, sequence)
    validation_images = max(parents)
    validation_sequences: set[str] = set()
    cursor = validation_images
    while cursor:
        parent = parents[cursor]
        if parent is None:
            break
        cursor, sequence = parent
        validation_sequences.add(sequence)
    if not validation_sequences:
        sequence = min(
            images_by_sequence,
            key=lambda item: (len(images_by_sequence[item]), item),
        )
        validation_sequences.add(sequence)
        validation_images = len(images_by_sequence[sequence])

    assignments: dict[tuple[str, str], SplitAssignment] = {}
    for sequence, image_names in images_by_sequence.items():
        generated = "val" if sequence in validation_sequences else "train"
        for image_name in image_names:
            assignments[("train", image_name)] = SplitAssignment(generated, sequence)
    for image_path in sorted((project / "train" / "img").glob("soiling_*")):
        camera = image_path.stem.rsplit("_", 1)[-1]
        assignments[("train", image_path.name)] = SplitAssignment(
            "train", f"soiling-train-{camera}"
        )
    for image_path in sorted((project / "test" / "img").iterdir()):
        if image_path.is_file():
            assignments[("test", image_path.name)] = SplitAssignment(
                "test", f"qualitative-test:{image_path.stem}"
            )

    train_sequences = set(images_by_sequence) - validation_sequences
    report = {
        "schema_version": "woodscape-sequence-split.v1",
        "source": "labeled raw train RGB annotations",
        "timestamp_sequence_gap": sequence_gap,
        "validation_fraction_requested": validation_fraction,
        "seed": seed,
        "rgb_images": len(rgb_rows),
        "sequence_count": len(images_by_sequence),
        "train_sequence_count": len(train_sequences),
        "validation_sequence_count": len(validation_sequences),
        "train_rgb_images": sum(
            len(images_by_sequence[sequence]) for sequence in train_sequences
        ),
        "validation_rgb_images": validation_images,
        "validation_fraction_actual": validation_images / len(rgb_rows),
        "sequence_overlap": sorted(train_sequences & validation_sequences),
        "qualitative_test_images": sum(split == "test" for split, _ in assignments),
        "validation_sequences": sorted(validation_sequences),
    }
    return assignments, report
