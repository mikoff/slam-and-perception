import pytest

from dataset_pipeline.cli import COMMANDS, PIPELINE_COMMANDS, main


def test_diagnostic_commands_are_not_in_default_pipeline():
    assert "inspect-projects" in COMMANDS and "inspect-projects" not in PIPELINE_COMMANDS
    assert "disk-usage" in COMMANDS and "disk-usage" not in PIPELINE_COMMANDS


def test_rejects_unsupported_dry_run():
    with pytest.raises(SystemExit):
        main(["all", "--dry-run"])


def test_rejects_nonpositive_image_limit():
    with pytest.raises(SystemExit):
        main(["filter", "--limit-images", "0"])
