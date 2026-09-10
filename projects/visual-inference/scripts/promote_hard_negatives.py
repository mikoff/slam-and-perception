"""Apply a frozen conservative policy to a mining candidate manifest."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import yaml

from student_detector.hard_negative_promotion import build_default_supplement
from student_detector.provenance import sha256_file


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--decisions-output", type=Path, required=True)
    return parser.parse_args()


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TypeError(f"{path} must contain a JSON object")
    return value


def main() -> None:
    args = _args()
    policy = yaml.safe_load(args.policy.read_text(encoding="utf-8"))
    if not isinstance(policy, dict):
        raise TypeError("promotion policy must be a mapping")
    candidate_sha256 = sha256_file(args.candidates)
    if candidate_sha256 != policy["candidate_manifest_sha256"]:
        raise ValueError("candidate manifest hash does not match promotion policy")
    supplement, decisions = build_default_supplement(_json(args.candidates), policy)
    supplement["inputs"] = {
        "candidate_manifest": str(args.candidates.resolve()),
        "candidate_manifest_sha256": candidate_sha256,
        "promotion_policy": str(args.policy.resolve()),
        "promotion_policy_sha256": sha256_file(args.policy),
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(supplement, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    decisions_output = args.decisions_output.resolve()
    decisions_output.parent.mkdir(parents=True, exist_ok=True)
    with decisions_output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(decisions[0]))
        writer.writeheader()
        writer.writerows(decisions)
    print(
        json.dumps(
            {
                "supplement": str(output),
                "decisions": str(decisions_output),
                **supplement["counts"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
