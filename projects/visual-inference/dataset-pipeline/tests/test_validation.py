import json

from PIL import Image

from dataset_pipeline.validation import validate_coco


def test_validation_rejects_invalid_box(tmp_path, taxonomy):
    image = tmp_path / "image.jpg"
    Image.new("RGB", (10, 10)).save(image)
    data = {
        "images": [{"id": 1, "file_name": "image.jpg", "raw_image_path": str(image), "width": 10, "height": 10}],
        "annotations": [{
            "id": 1, "image_id": 1, "category_id": taxonomy.category_ids["car"],
            "bbox": [9, 9, 3, 3], "area": 9, "iscrowd": 0, "segmentation": [],
        }],
        "categories": taxonomy.categories,
    }
    path = tmp_path / "instances.json"
    path.write_text(json.dumps(data))
    errors, _, _ = validate_coco(path, tmp_path, taxonomy)
    assert any("outside" in item["error"] for item in errors)


def test_validation_accepts_valid_source(tmp_path, taxonomy):
    image = tmp_path / "image.jpg"
    Image.new("RGB", (10, 10)).save(image)
    data = {
        "images": [{"id": 1, "file_name": "image.jpg", "raw_image_path": str(image), "width": 10, "height": 10}],
        "annotations": [{
            "id": 1, "image_id": 1, "category_id": taxonomy.category_ids["car"],
            "bbox": [1, 1, 3, 3], "area": 9, "iscrowd": 0, "segmentation": [],
        }],
        "categories": taxonomy.categories,
    }
    path = tmp_path / "instances.json"
    path.write_text(json.dumps(data))
    errors, _, _ = validate_coco(path, tmp_path, taxonomy)
    assert errors == []


def test_intermediate_validation_does_not_reopen_images(tmp_path, taxonomy):
    data = {
        "images": [{
            "id": 1, "file_name": "missing.jpg", "raw_image_path": str(tmp_path / "missing.jpg"),
            "width": 10, "height": 10, "source_dataset": "sample",
        }],
        "annotations": [],
        "categories": taxonomy.categories,
    }
    path = tmp_path / "instances.json"
    path.write_text(json.dumps(data))
    errors, _, _ = validate_coco(path, tmp_path, taxonomy)
    assert errors == []
