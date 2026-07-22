from __future__ import annotations

import pytest

from dataset_pipeline.config import load_config


def _config(tmp_path, extra: str = ""):
    path = tmp_path / "config.yaml"
    path.write_text(
        f"workspace_root: {tmp_path / 'workspace'}\n"
        "datasets: {}\n"
        f"{extra}",
        encoding="utf-8",
    )
    return path


def test_rejects_unknown_options(tmp_path):
    with pytest.raises(ValueError, match="Unknown storage options"):
        load_config(_config(tmp_path, "storage:\n  typo: true\n"))


def test_rejects_invalid_validation_range(tmp_path):
    with pytest.raises(ValueError, match="between 0 and 1"):
        load_config(_config(tmp_path, "validation:\n  validation_fraction: 2\n"))
