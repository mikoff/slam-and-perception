from __future__ import annotations

import json
import sqlite3
from pathlib import Path

from scripts.analyze_small_object_policy import analyze_index, write_report
from student_detector.config import load_phase3_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _quad(x: float, y: float, width: float, height: float) -> str:
    return json.dumps(
        [[x, y], [x + width, y], [x + width, y + height], [x, y + height]]
    )


def test_small_object_audit_separates_retention_from_grid_opportunity(tmp_path):
    index = tmp_path / "proposals.sqlite"
    with sqlite3.connect(index) as connection:
        connection.executescript(
            """
            CREATE TABLE images (
                image_id INTEGER PRIMARY KEY,
                width INTEGER,
                height INTEGER,
                source_dataset TEXT
            );
            CREATE TABLE annotations (
                image_id INTEGER,
                x1 REAL, y1 REAL, x2 REAL, y2 REAL,
                ignore_region INTEGER,
                category_name TEXT,
                quad_json TEXT,
                attributes_json TEXT
            );
            INSERT INTO images VALUES (1, 1280, 720, 'bdd100k_images_100k');
            """
        )
        rows = [
            (1, 0, 0, 100, 100, 0, "car", _quad(0, 0, 100, 100), '{"distance":15}'),
            (1, 101, 101, 106, 106, 0, "car", _quad(101, 101, 5, 5), "{}"),
            (1, 200, 200, 300, 202, 0, "car", _quad(200, 200, 100, 2), "{}"),
        ]
        connection.executemany(
            "INSERT INTO annotations VALUES (?,?,?,?,?,?,?,?,?)", rows
        )
    config = load_phase3_config(PROJECT_ROOT / "configs/phase3.yaml")

    report = analyze_index(
        index, config, PROJECT_ROOT / "automotive_taxonomy_mapping.json"
    )
    output = tmp_path / "report"
    write_report(report, output)

    overall = next(group for group in report["groups"] if group["scope"] == "overall")
    assert overall["objects"] == 3
    assert overall["hbb_retained"] == 1
    assert overall["quad_retained"] == 2
    assert overall["distance_bands"] == {"10-20m": 1, "unavailable": 2}
    assert overall["p3_opportunity"] <= overall["any_grid_opportunity"]
    assert (output / "small_object_policy.json").exists()
    assert (output / "histograms.csv").exists()
    assert (output / "index.html").exists()
