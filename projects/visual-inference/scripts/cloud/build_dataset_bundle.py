"""Build the immutable archive and manifest consumed by cloud dataset staging."""

from __future__ import annotations

import argparse
import json
import os
import tarfile
import tempfile
from pathlib import Path

if __package__:
    from .dataset_staging import MANIFEST_SCHEMA, sha256_file
else:
    from dataset_staging import MANIFEST_SCHEMA, sha256_file


def _dataset_files(root: Path) -> list[Path]:
    files = sorted(path for path in root.rglob("*") if path.is_file())
    if not files:
        raise ValueError("dataset root contains no files")
    for path in files:
        if path.is_symlink():
            raise ValueError(f"dataset bundles cannot contain symlinks: {path}")
    return files


def build_bundle(*, root: Path, dataset_id: str, output: Path) -> tuple[Path, Path]:
    """Write one archive and matching manifest, replacing neither on failure."""
    root = root.resolve()
    output = output.resolve()
    if not root.is_dir():
        raise NotADirectoryError(root)
    if output == root or root in output.parents:
        raise ValueError("output directory must be outside the dataset root")
    for required in ("indexes/quad_train.sqlite", "indexes/quad_val.sqlite"):
        if not (root / required).is_file():
            raise FileNotFoundError(f"prebuilt cloud index is missing: {required}")

    files = _dataset_files(root)
    output.mkdir(parents=True, exist_ok=True)
    archive = output / "dataset.tar.gz"
    manifest_path = output / "dataset-manifest.json"
    with tempfile.TemporaryDirectory(prefix=f".{dataset_id}.", dir=output) as name:
        temporary = Path(name)
        temporary_archive = temporary / archive.name
        with tarfile.open(temporary_archive, mode="w:gz") as bundle:
            for path in files:
                bundle.add(
                    path, arcname=path.relative_to(root).as_posix(), recursive=False
                )

        manifest = {
            "schema_version": MANIFEST_SCHEMA,
            "dataset_id": dataset_id,
            "archive": {
                "key": f"datasets/{dataset_id}/{archive.name}",
                "size": temporary_archive.stat().st_size,
                "sha256": sha256_file(temporary_archive),
            },
            "files": [
                {
                    "path": path.relative_to(root).as_posix(),
                    "size": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
                for path in files
            ],
        }
        temporary_manifest = temporary / manifest_path.name
        temporary_manifest.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(temporary_archive, archive)
        os.replace(temporary_manifest, manifest_path)
    return archive, manifest_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    archive, manifest = build_bundle(
        root=args.root, dataset_id=args.dataset_id, output=args.output
    )
    print(f"Built {archive}")
    print(f"Built {manifest}")


if __name__ == "__main__":
    main()
