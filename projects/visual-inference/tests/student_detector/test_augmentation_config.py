from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from student_detector.config import load_phase3_config


def _write_config(tmp_path: Path, augmentation: dict[str, object]) -> Path:
    path = tmp_path / "phase3.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "data": {
                    "train_annotations": "train.json",
                    "val_annotations": "val.json",
                    "image_root": "images",
                    "index_dir": "indexes",
                },
                "augmentation": augmentation,
            }
        ),
        encoding="utf-8",
    )
    return path


def test_unknown_augmentation_field_is_rejected(tmp_path: Path) -> None:
    path = _write_config(tmp_path, {"perspective_probability": 0.1})
    with pytest.raises(ValueError, match="invalid AugmentationConfig fields"):
        load_phase3_config(path)


@pytest.mark.parametrize(
    "augmentation",
    [
        {"noise_probability": 1.1},
        {"translation_fraction": -0.1},
        {"scale_min": 1.2, "scale_max": 0.8},
    ],
)
def test_invalid_augmentation_ranges_are_rejected(
    tmp_path: Path, augmentation: dict[str, object]
) -> None:
    path = _write_config(tmp_path, augmentation)
    with pytest.raises(ValueError, match="augmentation"):
        load_phase3_config(path)


def test_data_paths_expand_environment_variables(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dataset_root = tmp_path / "dataset"
    monkeypatch.setenv("TEST_DATASET_ROOT", str(dataset_root))
    path = _write_config(tmp_path, {})
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw["data"]["image_root"] = "${TEST_DATASET_ROOT}"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    config = load_phase3_config(path)

    assert config.data.image_root == dataset_root
