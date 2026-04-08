#!/usr/bin/env python3
"""Download iSAM2 benchmark datasets for the symforce SLAM project."""

import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]

DATASETS = {
    "iSAM2": {
        "base_url": "https://raw.githubusercontent.com/qh-huang/iSAM/master/data",
        "target": REPO_ROOT / "data" / "iSAM2",
        "files": [
            "README",
            "city10000.txt",
            "cityTrees10000.txt",
            "manhattanOlson3500.txt",
            "sphere2500.txt",
            "sphere400.txt",
            "torus10000.txt",
            "torus2000Points.txt",
            "victoriaPark.txt",
            "groundtruth/city10000_groundtruth.txt",
            "groundtruth/cityTrees10000_groundtruth.txt",
            "groundtruth/manhattanOlson3500_groundtruth.txt",
            "groundtruth/sphere2500_groundtruth.txt",
        ],
    },
}


def download(url: str, dest: Path) -> None:
    if dest.exists():
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"  {dest.name}")
    urllib.request.urlretrieve(url, dest)


def main() -> None:
    for name, ds in DATASETS.items():
        target = ds["target"]
        base = ds["base_url"]
        existing = sum(1 for f in ds["files"] if (target / f).exists())
        if existing == len(ds["files"]):
            print(f"{name}: already complete ({existing} files)")
            continue
        print(f"{name}: downloading to {target.relative_to(REPO_ROOT)}/")
        for f in ds["files"]:
            download(f"{base}/{f}", target / f)
    print("Done.")


if __name__ == "__main__":
    main()
