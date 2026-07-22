import json
from pathlib import Path

from PIL import Image

from dataset_pipeline.coco_merge import merge_exports, normalize_split
from dataset_pipeline.config import ensure_workspace


def _source_export(config, taxonomy, dataset="bdd100k_images_100k", split="train", filename="same.jpg"):
    raw = config.workspace_root / "raw" / dataset / "project" / split / "img" / filename
    raw.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (20, 10), "white").save(raw)
    data = {
        "images": [{
            "id": 9, "file_name": filename, "width": 20, "height": 10,
            "source_dataset": dataset, "source_split": split, "source_image_id": "stable-1",
            "source_file_name": filename, "raw_image_path": str(raw),
        }],
        "annotations": [{
            "id": 8, "image_id": 9, "category_id": taxonomy.category_ids["car"],
            "bbox": [1.0, 2.0, 7.0, 5.0], "area": 35.0, "iscrowd": 0, "segmentation": [],
            "source_dataset": dataset, "source_annotation_id": "a", "source_category": "car", "attributes": {},
        }],
        "categories": taxonomy.categories,
    }
    path = config.workspace_root / "intermediate" / "coco" / dataset / f"instances_{split}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def test_split_aliases():
    aliases = {"train_aliases": ["training"], "val_aliases": ["valid"], "test_aliases": ["test"]}
    assert normalize_split("Training", aliases) == "train"
    assert normalize_split("VALID", aliases) == "val"


def test_merge_is_deterministic_and_links_raw(config_factory, taxonomy):
    config = config_factory({
        "bdd100k_images_100k": {"archive": "/tmp/unused.tar"},
        "nuimages": {"archive": "/tmp/unused2.tar"},
    })
    ensure_workspace(config)
    _source_export(config, taxonomy, split="train")
    _source_export(config, taxonomy, dataset="bdd100k_images_100k", split="val", filename="val.jpg")
    merge_exports(config, taxonomy)
    train_path = config.workspace_root / "output/annotations/instances_train.json"
    before = train_path.read_bytes()
    merge_exports(config, taxonomy)
    assert train_path.read_bytes() == before
    data = json.loads(before)
    assert data["images"][0]["id"] == 1
    link = config.workspace_root / "output" / data["images"][0]["file_name"]
    assert link.is_symlink()
    assert "/raw/" in str(link.resolve())
