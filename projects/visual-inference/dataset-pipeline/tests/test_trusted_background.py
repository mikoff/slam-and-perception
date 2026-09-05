from dataset_pipeline.trusted_background import derive_trusted_background


def _record(quad, state, category):
    xs = [point[0] for point in quad]
    ys = [point[1] for point in quad]
    return {
        "bbox": [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)],
        "quad": quad,
        "state": state,
        "supervision_state": state,
        "source_category": category,
    }


def test_positive_and_ignore_precedence_remove_trusted_pixels(config_factory, taxonomy):
    manifest = {
        "images": [
            {
                "image_id": 1,
                "source_dataset": "bdd100k_images_100k",
                "source_split": "train",
                "source_image_id": "one",
                "width": 64,
                "height": 64,
                "positive": [
                    _record([[0, 0], [31, 0], [31, 31], [0, 31]], "positive", "car")
                ],
                "ignore": [
                    _record(
                        [[32, 0], [63, 0], [63, 31], [32, 31]],
                        "ignore",
                        "grouped_vehicles",
                    )
                ],
                "trusted_background": [
                    _record(
                        [[0, 0], [63, 0], [63, 63], [0, 63]],
                        "trusted_background",
                        "drivable_area",
                    )
                ],
            }
        ]
    }

    config = config_factory({"bdd100k_images_100k": {"archive": "/tmp/unused.tar"}})
    report = derive_trusted_background(config, taxonomy, {"train": manifest})

    trusted = manifest["images"][0]["trusted_background"]
    assert {tuple(row["bbox"]) for row in trusted} == {
        (0.0, 32.0, 32.0, 32.0),
        (32.0, 32.0, 32.0, 32.0),
    }
    assert report["counts"]["positive_trusted_overlap_pixels_after_precedence"] == 0
    assert report["counts"]["ignore_trusted_overlap_pixels_after_precedence"] == 0
    key = "bdd100k_images_100k:drivable_area"
    assert report["source_category_instances"][key] == 1
    assert report["source_category_area_pixels"][key] > 0
