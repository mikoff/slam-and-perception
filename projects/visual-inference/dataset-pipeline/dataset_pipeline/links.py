from __future__ import annotations

import os
import shutil
from pathlib import Path


class LinkError(RuntimeError):
    pass


def link_image(source: Path, destination: Path, mode: str = "symlink", relative: bool = True, force: bool = False) -> str:
    source = source.resolve(strict=True)
    if not source.is_file():
        raise LinkError(f"Image source is not a regular file: {source}")
    if any(suffix in {".tar", ".tgz", ".gz", ".zst"} for suffix in source.suffixes):
        raise LinkError(f"Refusing to link an archive: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink():
        try:
            if destination.resolve(strict=True) == source:
                return "reused_symlink"
        except FileNotFoundError:
            pass
        if not force:
            raise LinkError(f"Existing symlink is broken or incorrect: {destination}")
        destination.unlink()
    elif destination.exists():
        if mode == "hardlink" and os.path.samefile(source, destination):
            return "reused_hardlink"
        if not force:
            raise LinkError(f"Destination already exists: {destination}")
        if destination.is_dir():
            raise LinkError(f"Destination is a directory: {destination}")
        destination.unlink()
    if mode == "symlink":
        target = os.path.relpath(source, destination.parent) if relative else str(source)
        destination.symlink_to(target)
        action = "new_symlink"
    elif mode == "hardlink":
        if source.stat().st_dev != destination.parent.stat().st_dev:
            raise LinkError("Hard links require source and destination on the same filesystem")
        os.link(source, destination)
        action = "hardlink"
    elif mode == "copy":
        shutil.copy2(source, destination)
        action = "copied"
    else:
        raise ValueError("mode must be symlink, hardlink, or copy")
    if not destination.resolve(strict=True).is_file():
        raise LinkError(f"Created image link does not resolve to a file: {destination}")
    return action


def verify_link(path: Path, expected: Path | None = None) -> tuple[bool, str]:
    if not path.is_symlink():
        return False, "not_symlink"
    try:
        resolved = path.resolve(strict=True)
    except FileNotFoundError:
        return False, "broken"
    if not resolved.is_file():
        return False, "not_regular_file"
    if expected is not None and resolved != expected.resolve(strict=True):
        return False, "incorrect_target"
    return True, str(resolved)
