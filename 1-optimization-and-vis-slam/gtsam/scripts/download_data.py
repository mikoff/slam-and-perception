#!/usr/bin/env python3
"""Download nuScenes-mini dataset and convert to MCAP for the gtsam SLAM project."""

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
MCAP_OUTPUT_DIR = DATA_ROOT / "mcap"
CONVERTER_IMAGE = "nuscenes2mcap-local"
CONVERTER_REPO = "https://github.com/foxglove/nuscenes2mcap.git"

DATASETS = {
    "nuscenes-mini": {
        "url": "https://www.nuscenes.org/data/v1.0-mini.tgz",
        "archive": DATA_ROOT / "v1.0-mini.tgz",
        "extracted_marker": DATA_ROOT / "v1.0-mini",
        "dataset_name": "v1.0-mini",
    },
}

# Required by the MCAP converter (CAN bus data + map tiles).
CONVERTER_DEPS = [
    {
        "name": "CAN bus",
        "url": "https://www.nuscenes.org/data/can_bus.zip",
        "archive": DATA_ROOT / "can_bus.zip",
        "extract_to": DATA_ROOT,
        "marker": DATA_ROOT / "can_bus",
    },
    {
        "name": "Map expansion",
        "url": "https://www.nuscenes.org/data/nuScenes-map-expansion-v1.3.zip",
        "archive": DATA_ROOT / "nuScenes-map-expansion-v1.3.zip",
        "extract_to": DATA_ROOT / "maps",
        "marker": DATA_ROOT / "maps" / "expansion",
    },
]


def _is_valid_archive(path: Path) -> bool:
    """Check whether the file starts with a valid gzip or zip magic number."""
    try:
        with open(path, "rb") as f:
            magic = f.read(4)
        return magic[:2] == b"\x1f\x8b" or magic[:4] == b"PK\x03\x04"
    except OSError:
        return False


def download(url: str, dest: Path) -> bool:
    """Download url to dest. Returns False if auth is required."""
    if dest.exists():
        if _is_valid_archive(dest):
            print(f"  Already downloaded: {dest.name}")
            return True
        print(f"  Removing invalid download: {dest.name}")
        dest.unlink()
    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"  Downloading {dest.name} ...")
    try:
        urllib.request.urlretrieve(url, dest)
    except urllib.error.HTTPError as e:
        if e.code in (401, 403):
            _print_manual_download(dest)
            return False
        raise
    if not _is_valid_archive(dest):
        dest.unlink()
        _print_manual_download(dest)
        return False
    return True


def _print_manual_download(dest: Path) -> None:
    print(f"  [MANUAL DOWNLOAD REQUIRED] {dest.name}")
    print(f"    nuScenes requires a free account to download data files.")
    print(f"    1. Sign up / log in at https://www.nuscenes.org/nuscenes#download")
    print(f"    2. Download: {dest.name}")
    print(f"    3. Place the file at: {dest}")


def extract_tar(archive: Path, dest_dir: Path, marker: Path) -> None:
    if marker.exists():
        print(f"  Already extracted: {marker.name}/")
        return
    print(f"  Extracting {archive.name} ...")
    with tarfile.open(archive, "r:gz") as tar:
        tar.extractall(path=dest_dir, filter="data")


def extract_zip(archive: Path, dest_dir: Path, marker: Path) -> None:
    if marker.exists():
        print(f"  Already extracted: {marker.name}/")
        return
    print(f"  Extracting {archive.name} ...")
    dest_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as zf:
        zf.extractall(dest_dir)


def check_docker() -> bool:
    if shutil.which("docker") is None:
        return False
    return subprocess.run(
        ["docker", "info"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def ensure_converter_image() -> None:
    result = subprocess.run(
        ["docker", "images", "-q", CONVERTER_IMAGE],
        capture_output=True,
        text=True,
        check=True,
    )
    if result.stdout.strip():
        print(f"  Converter image already built: {CONVERTER_IMAGE}")
        return
    with tempfile.TemporaryDirectory() as tmpdir:
        clone_dir = Path(tmpdir) / "nuscenes2mcap"
        print("  Cloning nuscenes2mcap converter ...")
        subprocess.run(
            ["git", "clone", "--depth=1", CONVERTER_REPO, str(clone_dir)],
            check=True,
        )
        print(f"  Building converter image '{CONVERTER_IMAGE}' ...")
        subprocess.run(
            ["docker", "build", "-t", CONVERTER_IMAGE, str(clone_dir)],
            check=True,
        )


def _host_path(container_path: Path) -> str:
    """Translate a path inside this devcontainer to the corresponding host path.

    When using Docker-outside-of-Docker (socket passthrough), volume mounts must
    reference paths as they exist on the *host*, not inside this container.
    We read /proc/1/mountinfo to find how /workspace was bind-mounted.
    """
    container_path = container_path.resolve()
    try:
        with open("/proc/1/mountinfo") as f:
            for line in f:
                parts = line.split()
                mount_point = parts[4]
                # Find the separator " - " then the fs root is parts[3]
                mount_root = parts[3]
                if mount_point == "/workspace":
                    rel = container_path.relative_to("/workspace")
                    return str(Path(mount_root) / rel)
    except (OSError, ValueError):
        pass
    # Fallback: assume paths match (not inside a devcontainer)
    return str(container_path)


def convert_to_mcap(dataset_name: str) -> None:
    existing = list(MCAP_OUTPUT_DIR.glob(f"NuScenes-{dataset_name}-*.mcap"))
    if existing:
        print(f"  Already converted: {len(existing)} scene(s)")
        return

    if not check_docker():
        print(
            "\n  [WARNING] Docker is not available.\n"
            "  The MCAP conversion requires Docker.\n"
            "  Mount the host Docker socket into the devcontainer and rebuild.\n"
        )
        sys.exit(1)

    ensure_converter_image()
    MCAP_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"  Converting '{dataset_name}' to MCAP → {MCAP_OUTPUT_DIR.relative_to(REPO_ROOT)}/")
    host_data = _host_path(DATA_ROOT)
    host_output = _host_path(MCAP_OUTPUT_DIR)
    subprocess.run(
        [
            "docker", "run", "--rm",
            "--user", f"{os.getuid()}:{os.getgid()}",
            "-v", f"{host_data}:/data",
            "-v", f"{host_output}:/output",
            CONVERTER_IMAGE,
            "python3", "convert_to_mcap.py",
            "--data-dir", "/data",
            "--dataset-name", dataset_name,
            "--output-dir", "/output",
        ],
        check=True,
    )


def main() -> None:
    missing = []

    # 1. Download and extract the main dataset
    for name, ds in DATASETS.items():
        print(f"{name}: data root → {DATA_ROOT.relative_to(REPO_ROOT)}/")
        ok = download(ds["url"], ds["archive"])
        if not ok:
            missing.append(ds["archive"].name)
            continue
        extract_tar(ds["archive"], DATA_ROOT, ds["extracted_marker"])

    # 2. Download and extract converter dependencies (CAN bus + maps)
    for dep in CONVERTER_DEPS:
        print(f"{dep['name']}: data root → {DATA_ROOT.relative_to(REPO_ROOT)}/")
        ok = download(dep["url"], dep["archive"])
        if not ok:
            missing.append(dep["archive"].name)
            continue
        extract_zip(dep["archive"], dep["extract_to"], dep["marker"])

    if missing:
        print(
            f"\n  [ERROR] {len(missing)} file(s) require manual download from nuScenes.\n"
            "  Re-run this script after placing them in the correct locations.\n"
        )
        sys.exit(1)

    # 3. Convert to MCAP via Docker
    for ds in DATASETS.values():
        convert_to_mcap(ds["dataset_name"])

    print("Done.")

if __name__ == "__main__":
    main()