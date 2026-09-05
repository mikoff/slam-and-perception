from __future__ import annotations

import pytest

from dataset_pipeline.config import load_config


def _config(tmp_path, extra: str = ""):
    path = tmp_path / "config.yaml"
    path.write_text(
        f"workspace_root: {tmp_path / 'workspace'}\ndatasets: {{}}\n{extra}",
        encoding="utf-8",
    )
    return path


def test_rejects_unknown_options(tmp_path):
    with pytest.raises(ValueError, match="Unknown storage options"):
        load_config(_config(tmp_path, "storage:\n  typo: true\n"))


def test_rejects_invalid_validation_range(tmp_path):
    with pytest.raises(ValueError, match="between 0 and 1"):
        load_config(_config(tmp_path, "validation:\n  validation_fraction: 2\n"))


def test_loads_pinned_official_annotations_inside_workspace(tmp_path):
    workspace = tmp_path / "workspace"
    config_path = tmp_path / "official.yaml"
    config_path.write_text(
        f"workspace_root: {workspace}\n"
        "datasets:\n"
        "  coco_2017:\n"
        f"    archive: {tmp_path / 'coco.tar'}\n"
        "    extracted_dir: raw/coco_2017\n"
        "    official_annotations:\n"
        "      train2017:\n"
        "        path: raw/coco_2017/official/instances_train2017.json\n"
        f"        sha256: {'a' * 64}\n",
        encoding="utf-8",
    )

    config = load_config(config_path)
    source = config.datasets["coco_2017"].official_annotations["train2017"]

    assert source.path == (
        workspace / "raw/coco_2017/official/instances_train2017.json"
    )
    assert source.sha256 == "a" * 64


def test_rejects_invalid_official_annotation_hash(tmp_path):
    config_path = tmp_path / "official.yaml"
    config_path.write_text(
        f"workspace_root: {tmp_path / 'workspace'}\n"
        "datasets:\n"
        "  coco_2017:\n"
        f"    archive: {tmp_path / 'coco.tar'}\n"
        "    extracted_dir: raw/coco_2017\n"
        "    official_annotations:\n"
        "      train2017:\n"
        "        path: raw/coco_2017/official/instances_train2017.json\n"
        "        sha256: not-a-hash\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="64 hex characters"):
        load_config(config_path)
