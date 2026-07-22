from __future__ import annotations

import io
import json
import tarfile
from pathlib import Path

import pytest
from PIL import Image

from dataset_pipeline.config import load_config
from dataset_pipeline.taxonomy import Taxonomy


@pytest.fixture
def taxonomy() -> Taxonomy:
    return Taxonomy.load(Path(__file__).parents[1] / "configs" / "automotive_taxonomy_mapping.json")


@pytest.fixture
def config_factory(tmp_path):
    def create(datasets: dict | None = None):
        datasets = datasets or {}
        cfg = tmp_path / "config.yaml"
        cfg.write_text(
            "workspace_root: " + str(tmp_path / "workspace") + "\n"
            "taxonomy: " + str(Path(__file__).parents[1] / "configs" / "automotive_taxonomy_mapping.json") + "\n"
            "datasets:\n" + "".join(
                f"  {name}:\n    archive: {item['archive']}\n    extracted_dir: raw/{name}\n"
                for name, item in datasets.items()
            ),
            encoding="utf-8",
        )
        return load_config(cfg)
    return create


def make_project(root: Path, class_title: str = "car", geometry: str = "rectangle") -> Path:
    root.mkdir(parents=True)
    (root / "meta.json").write_text(json.dumps({
        "projectType": "images",
        "classes": [{"title": class_title, "shape": geometry, "geometryType": geometry, "color": "#ff0000"}],
        "tags": [],
    }))
    image_dir, ann_dir = root / "train" / "img", root / "train" / "ann"
    image_dir.mkdir(parents=True)
    ann_dir.mkdir()
    Image.new("RGB", (20, 10), "white").save(image_dir / "one.jpg")
    obj = {
        "classTitle": class_title, "geometryType": geometry,
        "points": {"exterior": [[1, 2], [8, 7]], "interior": []},
        "tags": [],
    }
    (ann_dir / "one.jpg.json").write_text(json.dumps({
        "description": "", "size": {"width": 20, "height": 10}, "tags": [], "objects": [obj],
    }))
    return root


def make_tar(path: Path, members: dict[str, bytes]) -> Path:
    with tarfile.open(path, "w") as archive:
        for name, value in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(value)
            archive.addfile(info, io.BytesIO(value))
    return path

