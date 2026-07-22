import os

import pytest

from dataset_pipeline.cleanup import cleanup
from dataset_pipeline.config import ensure_workspace
from dataset_pipeline.reports import write_json


def test_cleanup_rejects_reports_older_than_output(config_factory):
    config = config_factory({"sample": {"archive": "/tmp/unused.tar"}})
    ensure_workspace(config)
    validation = config.reports / "validation_summary.json"
    preview = config.reports / "previews/index.html"
    inventory = config.reports / "archive_inventory.json"
    write_json(validation, {"valid": True})
    preview.write_text("preview")
    write_json(inventory, [])
    for split in ("train", "val"):
        write_json(config.workspace_root / f"output/annotations/instances_{split}.json", {})

    with pytest.raises(RuntimeError, match="older"):
        cleanup(config, dry_run=True)

    newest = max(
        (config.workspace_root / f"output/annotations/instances_{split}.json").stat().st_mtime_ns
        for split in ("train", "val")
    ) + 1
    os.utime(validation, ns=(newest, newest))
    os.utime(preview, ns=(newest, newest))
    assert cleanup(config, dry_run=True)
