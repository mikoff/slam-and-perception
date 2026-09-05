import json

from dataset_pipeline.supervision_audit import generate_woodscape_supervision_audit
from conftest import make_project


def test_generates_deterministic_generated_state_audit(tmp_path):
    project = make_project(tmp_path / "filtered", "ego_platform_bodywork", "polygon")
    annotation_path = project / "train/ann/one.jpg.json"
    annotation = json.loads(annotation_path.read_text())
    obj = annotation["objects"][0]
    obj["sourceCategory"] = "ego_vehicle"
    obj["sourceAnnotationId"] = "ego-1"
    obj["supervisionState"] = "trusted_background"
    annotation_path.write_text(json.dumps(annotation))

    report = generate_woodscape_supervision_audit(
        project, tmp_path / "audit", count_per_category_state=1
    )

    assert report["available_counts"] == {"ego_vehicle:trusted_background": 1}
    assert report["rendered_cards"] == 1
    assert (tmp_path / "audit/index.html").exists()
    assert len(list((tmp_path / "audit/assets").glob("*.jpg"))) == 1
