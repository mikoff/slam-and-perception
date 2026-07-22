import json
from pathlib import Path

import pytest

from dataset_pipeline.archives import UnsafeArchiveError, extract_one, inspect_archive, inspect_archives
from dataset_pipeline.config import ensure_workspace
from dataset_pipeline.config import DatasetConfig
from conftest import make_tar


def test_archive_inspection_and_reuse(tmp_path):
    archive = make_tar(tmp_path / "data.tar", {"project/meta.json": b'{"classes":[]}', "project/train/img/a.jpg": b"x"})
    dataset = DatasetConfig("sample", archive, tmp_path / "raw")
    report = inspect_archive(dataset)
    assert report["contains_meta_json"] and report["member_count"] == 2
    first = extract_one(dataset)
    second = extract_one(dataset)
    assert first["status"] == "extracted"
    assert second["status"] == "reused"
    assert json.loads((dataset.extracted_dir / ".dataset_pipeline_manifest.json").read_text())["archive_sha256"]


def test_rejects_traversal(tmp_path):
    archive = make_tar(tmp_path / "bad.tar", {"../escape": b"bad"})
    dataset = DatasetConfig("bad", archive, tmp_path / "raw")
    assert inspect_archive(dataset)["suspicious_entries"][0]["reason"] == "path_traversal"
    with pytest.raises(UnsafeArchiveError):
        extract_one(dataset)


def test_changed_fingerprint_requires_force(tmp_path):
    archive = make_tar(tmp_path / "data.tar", {"one": b"1"})
    dataset = DatasetConfig("sample", archive, tmp_path / "raw")
    extract_one(dataset)
    make_tar(archive, {"two": b"2"})
    with pytest.raises(RuntimeError, match="force-extract"):
        extract_one(dataset)


def test_reuse_does_not_rehash_unchanged_archive(tmp_path, monkeypatch):
    archive = make_tar(tmp_path / "data.tar", {"one": b"1"})
    dataset = DatasetConfig("sample", archive, tmp_path / "raw")
    extract_one(dataset)
    monkeypatch.setattr("dataset_pipeline.archives.sha256", lambda _: pytest.fail("unexpected hash"))
    assert extract_one(dataset)["status"] == "reused"


def test_all_can_reuse_cached_archive_inspection(tmp_path, config_factory, monkeypatch):
    archive = make_tar(tmp_path / "data.tar", {"one": b"1"})
    config = config_factory({"sample": {"archive": archive}})
    ensure_workspace(config)
    inspect_archives(config)
    monkeypatch.setattr("dataset_pipeline.archives.inspect_archive", lambda _: pytest.fail("unexpected scan"))
    assert inspect_archives(config, reuse_cached=True)[0]["dataset"] == "sample"
