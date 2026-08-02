"""Create the deterministic mixed-geometry G6 training fixture."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from student_detector.quad_microfixture import create_g6_microfixture


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/phase3/fixtures/g6"),
    )
    args = parser.parse_args()
    print(json.dumps(create_g6_microfixture(args.output_dir), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
