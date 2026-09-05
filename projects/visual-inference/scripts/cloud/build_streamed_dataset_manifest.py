"""Build the manifest for a dataset archive uploaded by a native pipeline."""

from __future__ import annotations

import argparse
import json
import os
import re
import tempfile
from pathlib import Path

if __package__:
    from .dataset_staging import REQUIRED_DATASET_FILES, STREAMED_MANIFEST_SCHEMA
else:
    from dataset_staging import REQUIRED_DATASET_FILES, STREAMED_MANIFEST_SCHEMA

SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
SHA256 = re.compile(r"[0-9a-f]{64}")


def build_manifest(
    *,
    root: Path,
    dataset_id: str,
    archive_key: str,
    archive_size: int,
    archive_sha256: str,
    extracted_size: int,
    dataset_contract_sha256: str,
    output: Path,
) -> Path:
    """Write an atomic manifest for an already-uploaded archive."""
    if SAFE_ID.fullmatch(dataset_id) is None:
        raise ValueError("dataset ID contains unsupported characters")
    expected_key = f"datasets/{dataset_id}/dataset.tar.gz"
    if archive_key != expected_key:
        raise ValueError(f"archive key must be {expected_key}")
    if archive_size <= 0:
        raise ValueError("archive size must be positive")
    if SHA256.fullmatch(archive_sha256) is None:
        raise ValueError("archive SHA-256 must be 64 lowercase hexadecimal characters")
    if extracted_size <= 0:
        raise ValueError("extracted size must be positive")
    if SHA256.fullmatch(dataset_contract_sha256) is None:
        raise ValueError(
            "dataset contract SHA-256 must be 64 lowercase hexadecimal characters"
        )

    root = root.resolve()
    if not root.is_dir():
        raise NotADirectoryError(root)
    for relative in REQUIRED_DATASET_FILES:
        if not (root / relative).is_file():
            raise FileNotFoundError(f"required dataset file is missing: {relative}")
    manifest = {
        "schema_version": STREAMED_MANIFEST_SCHEMA,
        "dataset_id": dataset_id,
        "dataset_contract_sha256": dataset_contract_sha256,
        "archive": {
            "key": archive_key,
            "size": archive_size,
            "sha256": archive_sha256,
        },
        "extracted_size": extracted_size,
        "required_files": list(REQUIRED_DATASET_FILES),
    }

    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", suffix=".json", dir=output.parent, encoding="utf-8", delete=False
    ) as stream:
        staged = Path(stream.name)
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    try:
        os.replace(staged, output)
    finally:
        staged.unlink(missing_ok=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--archive-key", required=True)
    parser.add_argument("--archive-size", type=int, required=True)
    parser.add_argument("--archive-sha256", required=True)
    parser.add_argument("--extracted-size", type=int, required=True)
    parser.add_argument("--dataset-contract-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = build_manifest(
        root=args.root,
        dataset_id=args.dataset_id,
        archive_key=args.archive_key,
        archive_size=args.archive_size,
        archive_sha256=args.archive_sha256,
        extracted_size=args.extracted_size,
        dataset_contract_sha256=args.dataset_contract_sha256,
        output=args.output,
    )
    print(f"Built streamed dataset manifest {manifest}")


if __name__ == "__main__":
    main()
