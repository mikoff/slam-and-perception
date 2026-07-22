from pathlib import Path

import pytest

from dataset_pipeline.links import LinkError, link_image, verify_link


def test_relative_symlink_reuse_and_wrong_target(tmp_path):
    source = tmp_path / "raw" / "x.jpg"
    source.parent.mkdir()
    source.write_bytes(b"image")
    destination = tmp_path / "out" / "x.jpg"
    assert link_image(source, destination, "symlink", True) == "new_symlink"
    assert not destination.readlink().is_absolute()
    assert link_image(source, destination, "symlink", True) == "reused_symlink"
    assert verify_link(destination, source)[0]
    other = tmp_path / "raw" / "other.jpg"
    other.write_bytes(b"other")
    with pytest.raises(LinkError):
        link_image(other, destination)


def test_broken_link_is_rejected(tmp_path):
    destination = tmp_path / "bad.jpg"
    destination.symlink_to("missing.jpg")
    source = tmp_path / "source.jpg"
    source.write_bytes(b"x")
    with pytest.raises(LinkError, match="broken"):
        link_image(source, destination)

