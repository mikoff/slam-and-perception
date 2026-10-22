"""Fail-fast image isolation checks shared by detector training stages."""

from __future__ import annotations

from collections.abc import Collection, Mapping, Sequence
import hashlib
from pathlib import Path
from typing import Any


ImageRecord = Mapping[str, Any]


def _resolved_path(record: ImageRecord, image_root: Path) -> Path:
    return (image_root / str(record["file_name"])).resolve()


def _identity(record: ImageRecord) -> tuple[str, int] | None:
    image_id = record.get("image_id")
    source = record.get("source_dataset")
    if image_id is None or source is None:
        return None
    return str(source), int(image_id)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def assert_same_image_records(
    expected: Sequence[ImageRecord],
    actual: Sequence[ImageRecord],
    *,
    image_root: Path,
    expected_name: str,
    actual_name: str,
) -> None:
    """Require two dataset constructions to select the same image identities."""
    expected_paths = {_resolved_path(record, image_root) for record in expected}
    actual_paths = {_resolved_path(record, image_root) for record in actual}
    if expected_paths != actual_paths or len(expected) != len(actual):
        raise ValueError(
            f"Split mismatch between {expected_name} and {actual_name}: "
            f"records={len(expected)} vs {len(actual)}, "
            f"{len(expected_paths - actual_paths)} missing, "
            f"{len(actual_paths - expected_paths)} unexpected"
        )


def audit_training_isolation(
    training: Sequence[ImageRecord],
    validation_sets: Mapping[str, Sequence[ImageRecord]],
    *,
    image_root: Path,
    verify_hashes: bool = False,
    identity_validation_sets: Collection[str] | None = None,
) -> dict[str, Any]:
    """Reject identity, path, or optional exact-content validation leakage.

    Image IDs are compared only for named sets where both sides use the same
    ID namespace. Exported validation JSON files often renumber images locally,
    so their IDs cannot safely be compared with the training database.
    """
    root = image_root.resolve()
    training_paths = {_resolved_path(record, root) for record in training}
    training_ids = {
        identity for record in training if (identity := _identity(record)) is not None
    }
    hash_cache: dict[Path, str] = {}

    def hashes(paths: set[Path]) -> set[str]:
        result: set[str] = set()
        for path in paths:
            if not path.is_file():
                raise FileNotFoundError(f"Isolation audit image not found: {path}")
            if path not in hash_cache:
                hash_cache[path] = _file_sha256(path)
            result.add(hash_cache[path])
        return result

    training_hashes = hashes(training_paths) if verify_hashes else set()
    report: dict[str, Any] = {
        "training_images": len(training_paths),
        "hash_verification": verify_hashes,
        "validation": {},
    }
    for name, records in validation_sets.items():
        validation_paths = {_resolved_path(record, root) for record in records}
        validation_ids = {
            identity
            for record in records
            if (identity := _identity(record)) is not None
        }
        path_overlap = training_paths & validation_paths
        compare_identities = (
            identity_validation_sets is None or name in identity_validation_sets
        )
        identity_overlap = (
            training_ids & validation_ids if compare_identities else set()
        )
        hash_overlap = (
            training_hashes & hashes(validation_paths) if verify_hashes else set()
        )
        report["validation"][name] = {
            "images": len(validation_paths),
            "identity_check": compare_identities,
            "identity_overlap": len(identity_overlap),
            "path_overlap": len(path_overlap),
            "sha256_overlap": len(hash_overlap),
        }
        if identity_overlap or path_overlap or hash_overlap:
            raise ValueError(
                f"Training/validation overlap in {name}: "
                f"identities={len(identity_overlap)}, "
                f"paths={len(path_overlap)}, sha256={len(hash_overlap)}"
            )
    return report
