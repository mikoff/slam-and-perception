"""Fail-closed local identity checks for Phase 4 training inputs."""

from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

import yaml

from .config import Phase3Config
from .provenance import sha256_file


def _mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _json_object(path: Path, name: str) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Phase 4 {name} is missing: {path}")
    return _mapping(json.loads(path.read_text(encoding="utf-8")), name)


def _index_metadata(path: Path) -> tuple[dict[str, str], int]:
    if not path.is_file():
        raise FileNotFoundError(f"Phase 4 prebuilt index is missing: {path}")
    with sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True) as connection:
        metadata = dict(connection.execute("SELECT key, value FROM metadata"))
        images = int(connection.execute("SELECT COUNT(*) FROM images").fetchone()[0])
    return metadata, images


def _require_equal(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        raise ValueError(
            f"Phase 4 {name} mismatch: expected {expected!r}, found {actual!r}"
        )


def verify_phase4_launch_inputs(
    config: Phase3Config,
    *,
    training_contract_path: Path,
    initialization_path: Path | None,
    initialization_state: str | None,
    initialization_required: bool,
) -> dict[str, str]:
    """Verify the immutable dataset and fresh-start checkpoint before GPU setup."""
    training_contract = _mapping(
        yaml.safe_load(training_contract_path.read_text(encoding="utf-8")),
        "training contract",
    )
    expected_dataset = _mapping(training_contract.get("dataset"), "dataset contract")
    expected_initialization = _mapping(
        training_contract.get("initialization"), "initialization contract"
    )

    output = config.data.image_root.resolve()
    workspace = output.parent
    _require_equal("dataset output directory", output, workspace / "output")
    _require_equal("dataset workspace name", workspace.name, expected_dataset["id"])
    expected_paths = {
        "training manifest path": output / "annotations/proposals_train.json",
        "validation manifest path": output / "annotations/proposals_val.json",
        "training index directory": output / "indexes",
    }
    actual_paths = {
        "training manifest path": config.data.train_annotations.resolve(),
        "validation manifest path": config.data.val_annotations.resolve(),
        "training index directory": config.data.index_dir.resolve(),
    }
    for name, expected in expected_paths.items():
        _require_equal(name, actual_paths[name], expected)

    dataset_contract_path = workspace / "dataset_contract.json"
    dataset_contract = _json_object(dataset_contract_path, "dataset contract")
    dataset_contract_sha256 = sha256_file(dataset_contract_path)
    _require_equal(
        "dataset contract SHA-256",
        dataset_contract_sha256,
        expected_dataset["contract_sha256"],
    )
    _require_equal(
        "dataset contract schema",
        dataset_contract.get("schema_version"),
        "phase3-dataset-contract.v1",
    )
    _require_equal(
        "dataset identifier", dataset_contract.get("dataset_id"), expected_dataset["id"]
    )

    cloud_manifest_path = workspace / "reports/cloud_bundle/dataset-manifest.json"
    cloud_manifest = _json_object(cloud_manifest_path, "cloud dataset manifest")
    _require_equal(
        "cloud dataset manifest SHA-256",
        sha256_file(cloud_manifest_path),
        expected_dataset["manifest_sha256"],
    )
    _require_equal(
        "cloud manifest dataset identifier",
        cloud_manifest.get("dataset_id"),
        expected_dataset["id"],
    )
    _require_equal(
        "cloud manifest dataset contract SHA-256",
        cloud_manifest.get("dataset_contract_sha256"),
        expected_dataset["contract_sha256"],
    )

    artifacts = _mapping(dataset_contract.get("artifacts"), "dataset artifacts")
    signatures: dict[str, str] = {}
    for split in ("train", "val"):
        manifest_path = output / f"annotations/proposals_{split}.json"
        index_path = output / f"indexes/quad_{split}.sqlite"
        manifest_contract = _mapping(
            artifacts.get(f"proposals_{split}.json"), f"{split} manifest contract"
        )
        index_contract = _mapping(
            artifacts.get(f"quad_{split}.sqlite"), f"{split} index contract"
        )
        if not manifest_path.is_file():
            raise FileNotFoundError(f"Phase 4 {split} manifest is missing: {manifest_path}")
        _require_equal(
            f"{split} manifest size", manifest_path.stat().st_size, manifest_contract["bytes"]
        )
        metadata, images = _index_metadata(index_path)
        _require_equal(f"{split} index size", index_path.stat().st_size, index_contract["bytes"])
        _require_equal(
            f"{split} index schema", metadata.get("schema_version"), "8:proposal-manifest"
        )
        _require_equal(
            f"{split} index source signature",
            metadata.get("source_signature"),
            index_contract["source_signature"],
        )
        _require_equal(f"{split} index image count", images, index_contract["images"])
        _require_equal(
            f"{split} manifest/index binding",
            metadata.get("source_signature"),
            f"sha256:{manifest_contract['sha256']}",
        )
        signatures[split] = str(manifest_contract["sha256"])

    if initialization_required and initialization_path is None:
        raise ValueError("Phase 4 fresh start requires --initialize-from")
    if initialization_path is not None:
        resolved_initialization = initialization_path.resolve()
        if not resolved_initialization.is_file():
            raise FileNotFoundError(
                f"Phase 4 initialization is missing: {resolved_initialization}"
            )
        _require_equal(
            "initialization SHA-256",
            sha256_file(resolved_initialization),
            expected_initialization["artifact_sha256"],
        )
        _require_equal(
            "initialization state",
            initialization_state,
            expected_initialization["checkpoint_state"],
        )

    return {
        "dataset_id": str(expected_dataset["id"]),
        "dataset_contract_sha256": dataset_contract_sha256,
        "train_manifest_sha256": signatures["train"],
        "validation_manifest_sha256": signatures["val"],
        "index_schema": "8:proposal-manifest",
    }


__all__ = ["verify_phase4_launch_inputs"]
