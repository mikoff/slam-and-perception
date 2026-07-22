import json

from dataset_pipeline.reports import write_json


def test_compact_json_is_valid_and_not_indented(tmp_path):
    path = tmp_path / "data.json"
    write_json(path, {"items": [1, 2]}, compact=True)
    assert json.loads(path.read_text()) == {"items": [1, 2]}
    assert "\n  " not in path.read_text()
