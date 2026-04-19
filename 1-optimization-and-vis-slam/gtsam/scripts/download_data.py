#!/usr/bin/env python3
"""Download nuScenes-mini and convert to MCAP for the gtsam SLAM project."""

import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
DATA_ROOT = REPO_ROOT / "data" / "nuscenes"
MCAP_DIR = DATA_ROOT / "mcap"

CONVERTER_IMAGE = "nuscenes2mcap-local"
CONVERTER_REPO = "https://github.com/foxglove/nuscenes2mcap.git"

# nuScenes files — all require a free account at https://www.nuscenes.org/nuscenes#download
# if the script cannot download them automatically (nuScenes returns an HTML login page).
FILES = [
    # (name, url, archive, extract_to, marker, is_tar)
    (
        "nuscenes-mini",
        "https://www.nuscenes.org/data/v1.0-mini.tgz",
        DATA_ROOT / "v1.0-mini.tgz",
        DATA_ROOT,
        DATA_ROOT / "v1.0-mini",
        True,
    ),
    (
        "CAN bus",
        "https://www.nuscenes.org/data/can_bus.zip",
        DATA_ROOT / "can_bus.zip",
        DATA_ROOT,
        DATA_ROOT / "can_bus",
        False,
    ),
    (
        # zip contains expansion/ basemap/ prediction/ — must land inside maps/
        "Map expansion",
        "https://www.nuscenes.org/data/nuScenes-map-expansion-v1.3.zip",
        DATA_ROOT / "nuScenes-map-expansion-v1.3.zip",
        DATA_ROOT / "maps",
        DATA_ROOT / "maps" / "expansion",
        False,
    ),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _is_valid_archive(path: Path) -> bool:
    try:
        with open(path, "rb") as f:
            magic = f.read(4)
        return magic[:2] == b"\x1f\x8b" or magic[:4] == b"PK\x03\x04"
    except OSError:
        return False


def _host_path(container_path: Path) -> str:
    """Return the host-side path for a container path.

    With Docker-outside-of-Docker, volume mounts must use host paths.
    Reads /proc/1/mountinfo to find the host bind-mount root for /workspace.
    Falls back to the original path when not running inside a devcontainer.
    """
    container_path = container_path.resolve()
    try:
        with open("/proc/1/mountinfo") as f:
            for line in f:
                parts = line.split()
                if parts[4] == "/workspace":
                    rel = container_path.relative_to("/workspace")
                    return str(Path(parts[3]) / rel)
    except (OSError, ValueError):
        pass
    return str(container_path)


# ---------------------------------------------------------------------------
# Download & extract
# ---------------------------------------------------------------------------

def download(name: str, url: str, dest: Path) -> bool:
    if dest.exists():
        if _is_valid_archive(dest):
            print(f"  {name}: already downloaded")
            return True
        print(f"  {name}: removing invalid file")
        dest.unlink()

    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"  {name}: downloading {dest.name} ...")
    try:
        urllib.request.urlretrieve(url, dest)
    except urllib.error.HTTPError as e:
        if e.code in (401, 403):
            _manual_download_hint(name, dest)
            return False
        raise

    if not _is_valid_archive(dest):
        dest.unlink()
        _manual_download_hint(name, dest)
        return False
    return True


def _manual_download_hint(name: str, dest: Path) -> None:
    print(f"  {name}: manual download required (nuScenes account needed)")
    print("    → https://www.nuscenes.org/nuscenes#download")
    print(f"    → place {dest.name} at {dest}")


def extract(name: str, archive: Path, dest_dir: Path, marker: Path, is_tar: bool) -> None:
    if marker.exists():
        print(f"  {name}: already extracted")
        return
    dest_dir.mkdir(parents=True, exist_ok=True)
    print(f"  {name}: extracting {archive.name} ...")
    if is_tar:
        with tarfile.open(archive, "r:gz") as tf:
            tf.extractall(path=dest_dir, filter="data")
    else:
        with zipfile.ZipFile(archive) as zf:
            zf.extractall(dest_dir)


# ---------------------------------------------------------------------------
# MCAP conversion
# ---------------------------------------------------------------------------

def ensure_converter_image() -> None:
    result = subprocess.run(
        ["docker", "images", "-q", CONVERTER_IMAGE],
        capture_output=True, text=True, check=True,
    )
    if result.stdout.strip():
        print("  Converter image already built")
        return
    with tempfile.TemporaryDirectory() as tmp:
        clone = Path(tmp) / "nuscenes2mcap"
        print("  Cloning nuscenes2mcap ...")
        subprocess.run(["git", "clone", "--depth=1", CONVERTER_REPO, str(clone)], check=True)
        print("  Building converter image ...")
        subprocess.run(["docker", "build", "-t", CONVERTER_IMAGE, str(clone)], check=True)


def convert_to_mcap(dataset_name: str) -> None:
    existing = list(MCAP_DIR.glob(f"NuScenes-{dataset_name}-*.mcap"))
    if existing:
        print(f"  MCAP: already converted ({len(existing)} scene(s))")
        return

    if not shutil.which("docker") or subprocess.run(
        ["docker", "info"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ).returncode != 0:
        print("  MCAP: Docker unavailable — mount /var/run/docker.sock and rebuild the devcontainer")
        sys.exit(1)

    ensure_converter_image()
    MCAP_DIR.mkdir(parents=True, exist_ok=True)

    print(f"  MCAP: converting '{dataset_name}' → {MCAP_DIR.relative_to(REPO_ROOT)}/")
    subprocess.run(
        [
            "docker", "run", "--rm",
            "--user", f"{os.getuid()}:{os.getgid()}",
            "-v", f"{_host_path(DATA_ROOT)}:/data",
            "-v", f"{_host_path(MCAP_DIR)}:/output",
            CONVERTER_IMAGE,
            "python3", "convert_to_mcap.py",
            "--data-dir", "/data",
            "--dataset-name", dataset_name,
            "--output-dir", "/output",
        ],
        check=True,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    missing = []

    for name, url, archive, extract_to, marker, is_tar in FILES:
        if not download(name, url, archive):
            missing.append(archive.name)
        else:
            extract(name, archive, extract_to, marker, is_tar)

    if missing:
        print(f"\nBlocked on {len(missing)} missing file(s). Re-run after downloading them.")
        sys.exit(1)

    convert_to_mcap("v1.0-mini")
    print("Done.")


if __name__ == "__main__":
    main()
