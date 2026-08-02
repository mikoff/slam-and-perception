from __future__ import annotations

import json

from student_detector.config import load_phase3_config
from student_detector.quad_data import QuadProposalDataset
from student_detector.quad_microfixture import FIXTURE_SCHEMA, create_g6_microfixture


def test_g6_microfixture_is_deterministic_and_complete(tmp_path) -> None:
    first = create_g6_microfixture(tmp_path / "g6")
    second = create_g6_microfixture(tmp_path / "g6")
    assert first == second
    assert first["fixture_schema"] == FIXTURE_SCHEMA

    manifest = json.loads((tmp_path / "g6" / "proposals_g6.json").read_text())
    positives = [record for image in manifest["images"] for record in image["positive"]]
    assert {record["geometry_tier"] for record in positives} == {
        "source_hbb", "rotated_rect", "source_quad", "fitted_quad"
    }
    assert {record["object_condition"] for record in positives} == {
        "whole_object", "perspective_object", "thin_object", "nested_part"
    }
    assert any(image["ignore"] for image in manifest["images"])
    assert any(image["background_supervision"] and not image["positive"] for image in manifest["images"])
    assert any(not image["background_supervision"] and not image["positive"] for image in manifest["images"])

    config = load_phase3_config(first["config"])
    dataset = QuadProposalDataset(
        config.data.quad_train_annotations,
        config.data.image_root,
        config.data.index_dir / "fixture.sqlite",
        config.data,
        config.augmentation,
        training=False,
    )
    samples = [dataset[index] for index in range(len(dataset))]
    assert sum(sample.quads.shape[0] for sample in samples) == 6
    assert {condition for sample in samples for condition in sample.object_conditions} == {
        "whole_object", "perspective_object", "thin_object", "nested_part"
    }
