import json
import tarfile
from pathlib import Path

import pytest

import dataset_pipeline.coco_export as coco_export_module
from dataset_pipeline.archives import extract_one
from dataset_pipeline.coco_export import export_project
from dataset_pipeline.config import DatasetConfig
from dataset_pipeline.detection_conversion import rectangle_object
from dataset_pipeline.discovery import discover_candidates, project_root
from dataset_pipeline.supervisely_filter import filter_project
from conftest import make_project


def test_synthetic_archive_to_coco(tmp_path, taxonomy, monkeypatch):
    source = make_project(tmp_path / "build" / "project", "car", "polygon")
    archive = tmp_path / "project.tar"
    with tarfile.open(archive, "w") as tar:
        tar.add(source, arcname="nested/project")
    dataset = DatasetConfig("bdd100k_images_100k", archive, tmp_path / "raw")
    extract_one(dataset)
    candidates = discover_candidates(dataset.extracted_dir, sdk_check=False)
    assert [item for item in candidates if item["valid"]]
    project = Path(candidates[0]["path"])
    filtered = tmp_path / "filtered"
    filter_project(dataset, project, filtered, taxonomy)
    annotation_path = filtered / "train/ann/one.jpg.json"
    annotation = json.loads(annotation_path.read_text())
    annotation["objects"][0], _ = rectangle_object(annotation["objects"][0], 20, 10, True)
    annotation_path.write_text(json.dumps(annotation))
    output = tmp_path / "coco"
    outputs = export_project(filtered, output, dataset.name, taxonomy)
    data = json.loads(outputs[0].read_text())
    assert data["annotations"][0]["bbox"] == [1.0, 2.0, 8.0, 6.0]
    assert data["images"][0]["source_image_id"] == "train/img/one.jpg"
    assert Path(data["images"][0]["raw_image_path"]).is_file()
    (output / "stale.json").write_text("stale")
    export_project(filtered, output, dataset.name, taxonomy)
    assert not (output / "stale.json").exists()

    existing = outputs[0].read_bytes()
    original_write_json = coco_export_module.write_json

    def fail_manifest(path, value, **kwargs):
        if path.name == "export_manifest.json":
            raise OSError("simulated export failure")
        original_write_json(path, value, **kwargs)

    monkeypatch.setattr(coco_export_module, "write_json", fail_manifest)
    with pytest.raises(OSError, match="simulated"):
        export_project(filtered, output, dataset.name, taxonomy)
    assert outputs[0].read_bytes() == existing


def test_project_root_reuses_the_discovery_manifest(tmp_path, monkeypatch):
    extracted = tmp_path / "raw"
    project = make_project(extracted / "project")
    (extracted / ".dataset_pipeline_manifest.json").write_text(json.dumps({
        "discovered_project_root": str(project.resolve()),
    }))
    dataset = DatasetConfig("bdd100k_images_100k", tmp_path / "unused.tar", extracted)
    def unexpected_rediscovery(*_):
        raise AssertionError("unexpected rediscovery")

    monkeypatch.setattr("dataset_pipeline.discovery.discover_candidates", unexpected_rediscovery)
    assert project_root(dataset, sdk_check=False) == project.resolve()
