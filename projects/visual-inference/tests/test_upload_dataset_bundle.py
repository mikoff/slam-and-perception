from __future__ import annotations

import io
import tarfile
from pathlib import Path

from scripts.cloud.upload_dataset_bundle import (
    dataset_files,
    file_records,
    write_archive,
)


def test_archive_dereferences_files_and_records_exact_bytes(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    target = tmp_path / "raw.jpg"
    target.write_bytes(b"pixels")
    (root / "images").mkdir(parents=True)
    (root / "indexes").mkdir()
    (root / "images/example.jpg").symlink_to(target)
    (root / "indexes/quad_train.sqlite").write_bytes(b"train")
    (root / "indexes/quad_val.sqlite").write_bytes(b"val")
    files = dataset_files(root)
    records = file_records(root, files)
    archive = io.BytesIO()
    size, digest = write_archive(root, files, archive)
    assert size == len(archive.getvalue())
    assert len(digest) == 64
    assert {record["path"] for record in records} == {
        "images/example.jpg",
        "indexes/quad_train.sqlite",
        "indexes/quad_val.sqlite",
    }
    archive.seek(0)
    with tarfile.open(fileobj=archive, mode="r:gz") as bundle:
        member = bundle.getmember("images/example.jpg")
        assert member.isreg()
        extracted = bundle.extractfile(member)
        assert extracted is not None
        assert extracted.read() == b"pixels"
