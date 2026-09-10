"""Apply the frozen proposal-selection policy to two utility reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

from student_detector.provenance import sha256_file
from student_detector.selection_policy import compare_selection_reports


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
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
        raise TypeError("selection policy must be a mapping")
    if policy.get("schema_version") != "proposal-selection-policy.v1":
        raise ValueError("unsupported selection policy")
    result = compare_selection_reports(
        _json(args.baseline), _json(args.candidate), policy
    )
    result["inputs"] = {
        "policy": str(args.policy.resolve()),
        "policy_sha256": sha256_file(args.policy),
        "baseline": str(args.baseline.resolve()),
        "baseline_sha256": sha256_file(args.baseline),
        "candidate": str(args.candidate.resolve()),
        "candidate_sha256": sha256_file(args.candidate),
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "output": str(output),
                "status": result["status"],
                "failed_gates": [
                    gate["name"]
                    for gate in (*result["gates"], *result["absolute_gates"])
                    if gate["status"] == "fail"
                ],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
