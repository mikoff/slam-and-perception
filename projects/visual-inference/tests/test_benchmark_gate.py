from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.run_limited_benchmark import compare_primary_metrics


def _summary(value: float) -> dict[str, object]:
    return {
        "runs": [
            {
                "neck_type": "lite",
                "seed": 42,
                "state": "raw",
                "ar/100": value,
                "recall/100@0.50": value,
                "recall/100@0.75": value,
                "matched_iou/median": value,
            }
        ]
    }


def _write(path: Path, value: float) -> Path:
    path.write_text(json.dumps(_summary(value)), encoding="utf-8")
    return path


def test_metric_gate_uses_exact_four_decimal_contract(tmp_path: Path) -> None:
    baseline = _write(tmp_path / "baseline.json", 0.12344)
    compare_primary_metrics(_write(tmp_path / "same.json", 0.123441), baseline)
    with pytest.raises(ValueError, match="regression"):
        compare_primary_metrics(_write(tmp_path / "changed.json", 0.12346), baseline)
