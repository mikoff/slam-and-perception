from __future__ import annotations

import hashlib
import io
import json
import tarfile
from pathlib import Path

import pytest

from scripts.cloud.build_dataset_bundle import build_bundle
from scripts.cloud.dataset_staging import (
    required_staging_bytes,
    stage_dataset,
    verify_bucket_protection,
)


class _Objects:
    def __init__(self, values: dict[str, bytes]) -> None:
        self.values = values
        self.downloads = 0

    def download(self, uri: str, destination: Path) -> None:
        self.downloads += 1
        destination.write_bytes(self.values[uri])


def _archive(path: str = "images/example.txt") -> bytes:
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode="w:gz") as bundle:
        payload = b"pixels"
        info = tarfile.TarInfo(path)
        info.size = len(payload)
        bundle.addfile(info, io.BytesIO(payload))
    return stream.getvalue()


def _objects(dataset_id: str, archive: bytes) -> _Objects:
    manifest = {
        "schema_version": "visual-inference-dataset.v1",
        "dataset_id": dataset_id,
        "archive": {
            "key": f"datasets/{dataset_id}/dataset.tar.gz",
            "size": len(archive),
            "sha256": hashlib.sha256(archive).hexdigest(),
        },
        "files": [
            {
                "path": "images/example.txt",
                "size": 6,
                "sha256": hashlib.sha256(b"pixels").hexdigest(),
            }
        ],
    }
    return _Objects(
        {
            f"s3://bucket/datasets/{dataset_id}/dataset-manifest.json": json.dumps(
                manifest
            ).encode(),
            f"s3://bucket/datasets/{dataset_id}/dataset.tar.gz": archive,
        }
    )


def test_disk_requirement_counts_archive_and_extracted_files() -> None:
    objects = _objects("v1", b"archive")
    manifest = json.loads(
        objects.values["s3://bucket/datasets/v1/dataset-manifest.json"]
    )
    assert required_staging_bytes(manifest) == len(b"archive") + len(b"pixels")


def test_dataset_is_verified_and_atomically_reused(tmp_path: Path) -> None:
    objects = _objects("v1", _archive())
    first = stage_dataset(
        bucket="bucket", dataset_id="v1", destination_root=tmp_path, aws=objects
    )
    second = stage_dataset(
        bucket="bucket", dataset_id="v1", destination_root=tmp_path, aws=objects
    )
    assert first == second
    assert (first / "images/example.txt").read_bytes() == b"pixels"
    assert objects.downloads == 3


def test_dataset_rejects_unsafe_archive_member(tmp_path: Path) -> None:
    archive = _archive("../escape.txt")
    objects = _objects("unsafe", archive)
    with pytest.raises(ValueError, match="unsafe dataset path"):
        stage_dataset(
            bucket="bucket",
            dataset_id="unsafe",
            destination_root=tmp_path,
            aws=objects,
        )
    assert not (tmp_path.parent / "escape.txt").exists()


def test_dataset_rejects_archive_checksum_mismatch(tmp_path: Path) -> None:
    objects = _objects("bad", _archive())
    archive_uri = "s3://bucket/datasets/bad/dataset.tar.gz"
    objects.values[archive_uri] += b"corrupt"
    with pytest.raises(ValueError, match="size mismatch"):
        stage_dataset(
            bucket="bucket", dataset_id="bad", destination_root=tmp_path, aws=objects
        )


def test_bundle_builder_round_trips_through_staging(tmp_path: Path) -> None:
    source = tmp_path / "source"
    (source / "indexes").mkdir(parents=True)
    (source / "indexes/quad_train.sqlite").write_bytes(b"train-index")
    (source / "indexes/quad_val.sqlite").write_bytes(b"val-index")
    (source / "images").mkdir()
    (source / "images/example.txt").write_bytes(b"pixels")
    archive, manifest = build_bundle(
        root=source, dataset_id="release-1", output=tmp_path / "bundle"
    )
    objects = _Objects(
        {
            "s3://bucket/datasets/release-1/dataset-manifest.json": manifest.read_bytes(),
            "s3://bucket/datasets/release-1/dataset.tar.gz": archive.read_bytes(),
        }
    )
    staged = stage_dataset(
        bucket="bucket",
        dataset_id="release-1",
        destination_root=tmp_path / "staged",
        aws=objects,
    )
    assert (staged / "indexes/quad_train.sqlite").read_bytes() == b"train-index"


def test_bundle_builder_requires_prebuilt_indexes(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "image.jpg").write_bytes(b"pixels")
    with pytest.raises(FileNotFoundError, match="quad_train.sqlite"):
        build_bundle(root=source, dataset_id="v1", output=tmp_path / "bundle")


class _BucketConfiguration:
    def __init__(
        self,
        *,
        versioning: str = "Enabled",
        current_days: int = 30,
        noncurrent_days: int = 30,
    ) -> None:
        self.versioning = versioning
        self.current_days = current_days
        self.noncurrent_days = noncurrent_days

    def json(self, *arguments: str) -> dict[str, object]:
        if "get-bucket-versioning" in arguments:
            return {"Status": self.versioning}
        return {
            "Rules": [
                {
                    "ID": "checkpoint-recovery",
                    "Status": "Enabled",
                    "Filter": {"Prefix": "runs/"},
                    "Expiration": {"Days": self.current_days},
                    "NoncurrentVersionExpiration": {
                        "NoncurrentDays": self.noncurrent_days
                    },
                    "AbortIncompleteMultipartUpload": {"DaysAfterInitiation": 1},
                }
            ]
        }


def test_bucket_protection_accepts_versioned_retained_checkpoints() -> None:
    verify_bucket_protection(
        bucket="bucket",
        aws=_BucketConfiguration(),  # type: ignore[arg-type]
    )


def test_bucket_protection_requires_enabled_versioning() -> None:
    with pytest.raises(ValueError, match="versioning"):
        verify_bucket_protection(
            bucket="bucket",
            aws=_BucketConfiguration(versioning="Suspended"),  # type: ignore[arg-type]
        )


@pytest.mark.parametrize(("current_days", "noncurrent_days"), [(1, 30), (30, 1)])
def test_bucket_protection_rejects_short_checkpoint_retention(
    current_days: int, noncurrent_days: int
) -> None:
    with pytest.raises(ValueError, match="at least 7 days"):
        verify_bucket_protection(
            bucket="bucket",
            aws=_BucketConfiguration(  # type: ignore[arg-type]
                current_days=current_days,
                noncurrent_days=noncurrent_days,
            ),
        )


def test_bucket_protection_requires_multipart_cleanup_for_runs() -> None:
    class MissingAbort(_BucketConfiguration):
        def json(self, *arguments: str) -> dict[str, object]:
            value = super().json(*arguments)
            if "get-bucket-lifecycle-configuration" in arguments:
                rules = value["Rules"]
                assert isinstance(rules, list)
                rules[0].pop("AbortIncompleteMultipartUpload")
            return value

    with pytest.raises(ValueError, match="multipart"):
        verify_bucket_protection(
            bucket="bucket",
            aws=MissingAbort(),  # type: ignore[arg-type]
        )


def test_bucket_protection_requires_multipart_cleanup_to_cover_all_runs() -> None:
    class NarrowAbort(_BucketConfiguration):
        def json(self, *arguments: str) -> dict[str, object]:
            value = super().json(*arguments)
            if "get-bucket-lifecycle-configuration" in arguments:
                rules = value["Rules"]
                assert isinstance(rules, list)
                rules[0]["Filter"] = {"Prefix": "runs/one-run/"}
            return value

    with pytest.raises(ValueError, match="multipart"):
        verify_bucket_protection(
            bucket="bucket",
            aws=NarrowAbort(),  # type: ignore[arg-type]
        )
