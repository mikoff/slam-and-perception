#!/usr/bin/env python3
"""Build a cryptographic contract for a generated proposal dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sqlite3
import subprocess
from pathlib import Path
from typing import Any


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact(path: Path, *, sha256: str | None = None) -> dict[str, Any]:
    return {
        "bytes": path.stat().st_size,
        "sha256": sha256 or _sha256(path),
    }


def _git(repo: Path, *arguments: str) -> str:
    return subprocess.run(
        ("git", *arguments),
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _tool_inventory(project: Path) -> dict[str, str]:
    roots = (
        project / "dataset-pipeline" / "dataset_pipeline",
        project / "student_detector",
    )
    files = [
        path
        for root in roots
        for path in root.rglob("*.py")
        if "__pycache__" not in path.parts
    ]
    files.extend(
        (
            project / "dataset-pipeline" / "pyproject.toml",
            project / "dataset-pipeline" / "uv.lock",
            project / "pyproject.toml",
            project / "uv.lock",
            Path(__file__).resolve(),
        )
    )
    return {
        path.relative_to(project).as_posix(): _sha256(path)
        for path in sorted(set(files))
    }


def _index_summary(path: Path) -> tuple[dict[str, Any], str]:
    with sqlite3.connect(path) as connection:
        metadata = dict(connection.execute("SELECT key, value FROM metadata"))
        images = int(connection.execute("SELECT COUNT(*) FROM images").fetchone()[0])
        states = {
            str(state): int(count)
            for state, count in connection.execute(
                "SELECT ignore_region, COUNT(*) FROM annotations GROUP BY ignore_region"
            )
        }
    signature = str(metadata["source_signature"])
    if not signature.startswith("sha256:"):
        raise ValueError(f"index has an invalid source signature: {path}")
    return {
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
        "schema_version": metadata["schema_version"],
        "images": images,
        "annotation_states": {
            "positive": states.get("0", 0),
            "ignore": states.get("1", 0),
            "trusted_background": states.get("2", 0),
        },
        "source_signature": signature,
    }, signature.removeprefix("sha256:")


def build_contract(
    candidate: Path,
    dataset_id: str,
    pipeline_config: Path,
    project: Path,
) -> dict[str, Any]:
    reports = candidate / "reports"
    annotations = candidate / "output" / "annotations"
    indexes = candidate / "output" / "indexes"
    archive_inventory = json.loads(
        (reports / "archive_inventory.json").read_text(encoding="utf-8")
    )
    train_index, train_manifest_hash = _index_summary(indexes / "quad_train.sqlite")
    val_index, val_manifest_hash = _index_summary(indexes / "quad_val.sqlite")
    taxonomy = project / "automotive_taxonomy_mapping.json"
    review = project / "dataset-pipeline/configs/proposal_object_contract_review.csv"
    tool_files = _tool_inventory(project)
    tool_digest = hashlib.sha256(
        json.dumps(tool_files, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    official = {
        "train2017": {
            "path": "raw/coco_2017/official/annotations/instances_train2017.json",
            "sha256": "610fce4944abdeb15354cc765333805529359d12d88f2f711393ca586901d01d",
        },
        "val2017": {
            "path": "raw/coco_2017/official/annotations/instances_val2017.json",
            "sha256": "e8c7f7908f1d7278341fae127d0da654f102f11bd7b21d8aeefa635b8c810b6f",
        },
    }
    return {
        "schema_version": "phase3-dataset-contract.v1",
        "dataset_id": dataset_id,
        "publication_status": "candidate_not_published",
        "source_archives": {
            item["dataset"]: {
                "bytes": item["compressed_size"],
                "member_count": item["member_count"],
                "sha256": item["sha256"],
            }
            for item in archive_inventory
        },
        "official_annotations": official,
        "policy": {
            "proposal_object_contract": "proposal-object-contract.v1",
            "taxonomy": _artifact(taxonomy),
            "review_decisions": _artifact(review),
            "pipeline_config": _artifact(pipeline_config),
        },
        "splits": {
            "woodscape_sequence_split": _artifact(
                reports / "woodscape_sequence_split.json"
            ),
            "merged_split_report": _artifact(reports / "split_report.json"),
        },
        "artifacts": {
            "instances_train.json": _artifact(annotations / "instances_train.json"),
            "instances_val.json": _artifact(annotations / "instances_val.json"),
            "proposals_train.json": _artifact(
                annotations / "proposals_train.json", sha256=train_manifest_hash
            ),
            "proposals_val.json": _artifact(
                annotations / "proposals_val.json", sha256=val_manifest_hash
            ),
            "quad_train.sqlite": train_index,
            "quad_val.sqlite": val_index,
            "trusted_background.json": _artifact(
                reports / "trusted_background.json"
            ),
        },
        "tool": {
            "git_commit": _git(project.parents[1], "rev-parse", "HEAD"),
            "working_tree_dirty": bool(
                _git(project.parents[1], "status", "--porcelain", "--", str(project))
            ),
            "source_tree_sha256": tool_digest,
            "source_files": tool_files,
        },
    }


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--pipeline-config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    candidate = args.candidate.resolve()
    project = Path(__file__).resolve().parents[1]
    output = args.output or candidate / "dataset_contract.json"
    _write_json(
        output,
        build_contract(
            candidate,
            args.dataset_id,
            args.pipeline_config.resolve(),
            project,
        ),
    )
    print(output)


if __name__ == "__main__":
    main()
