#!/usr/bin/env python3
"""Discover and run all sub-project download_data.py scripts."""

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def main() -> None:
    scripts = sorted(
        p
        for p in REPO_ROOT.rglob("scripts/download_data.py")
        if p != Path(__file__).resolve()
    )
    if not scripts:
        print("No sub-project downloaders found.")
        return

    failed = []
    for script in scripts:
        project = script.relative_to(REPO_ROOT).parts[0]
        print(f"=== {project} ===")
        result = subprocess.run([sys.executable, str(script)])
        if result.returncode != 0:
            failed.append(project)
        print()

    if failed:
        print(f"FAILED: {', '.join(failed)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
