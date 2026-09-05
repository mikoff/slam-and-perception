import json

from PIL import Image

from dataset_pipeline.woodscape_split import build_woodscape_split


def _rgb(project, name, timestamp):
    image = project / "train/img" / name
    image.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (8, 8), "white").save(image)
    annotation = project / "train/ann" / f"{name}.json"
    annotation.parent.mkdir(parents=True, exist_ok=True)
    annotation.write_text(
        json.dumps(
            {
                "tags": [
                    {
                        "name": "vehicle info",
                        "value": repr({"timestamp": timestamp}),
                    }
                ]
            }
        )
    )


def test_sequence_split_is_disjoint_and_raw_test_is_qualitative(tmp_path):
    project = tmp_path / "woodscape"
    _rgb(project, "rgb_0001.png", 100)
    _rgb(project, "rgb_0002.png", 200)
    _rgb(project, "rgb_0003.png", 2_000_000)
    _rgb(project, "rgb_0004.png", 2_000_100)
    test_image = project / "test/img/rgb_test.png"
    test_image.parent.mkdir(parents=True)
    Image.new("RGB", (8, 8), "white").save(test_image)

    assignments, report = build_woodscape_split(project, 0.5, 7)

    train_sequences = {
        value.sequence_identity
        for (source_split, _), value in assignments.items()
        if source_split == "train" and value.generated_split == "train"
    }
    validation_sequences = {
        value.sequence_identity
        for (source_split, _), value in assignments.items()
        if source_split == "train" and value.generated_split == "val"
    }
    assert train_sequences
    assert validation_sequences
    assert train_sequences.isdisjoint(validation_sequences)
    assert report["sequence_overlap"] == []
    assert report["validation_rgb_images"] == 2
    assert assignments[("test", "rgb_test.png")].generated_split == "test"
