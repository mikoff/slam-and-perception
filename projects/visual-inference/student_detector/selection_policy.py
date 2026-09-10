"""Goal-aligned selection decisions for matched proposal-utility reports."""

from __future__ import annotations

from typing import Any


def _value_at(document: dict[str, Any], path: str) -> Any:
    value: Any = document
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise KeyError(f"report does not contain {path!r}")
        value = value[part]
    return value


def _metric(report: dict[str, Any], group: str, metric: str) -> float:
    value = _value_at(report, f"hbb.groups.{group}")
    if not isinstance(value, dict) or metric not in value:
        raise KeyError(f"HBB group {group!r} does not contain metric {metric!r}")
    result = float(value[metric])
    if not (-float("inf") < result < float("inf")):
        raise ValueError(f"metric {group}/{metric} is not finite")
    return result


def _delta_status(
    delta_pp: float,
    *,
    direction: str,
    warning_delta_pp: float,
    failure_delta_pp: float,
) -> str:
    if direction == "higher":
        if failure_delta_pp > warning_delta_pp:
            raise ValueError("higher-is-better failure threshold must be lower")
        if delta_pp < failure_delta_pp:
            return "fail"
        if delta_pp < warning_delta_pp:
            return "inconclusive"
        return "pass"
    if direction == "lower":
        if failure_delta_pp < warning_delta_pp:
            raise ValueError("lower-is-better failure threshold must be higher")
        if delta_pp > failure_delta_pp:
            return "fail"
        if delta_pp > warning_delta_pp:
            return "inconclusive"
        return "pass"
    raise ValueError(f"unsupported direction {direction!r}")


def compare_selection_reports(
    baseline: dict[str, Any],
    candidate: dict[str, Any],
    policy: dict[str, Any],
) -> dict[str, Any]:
    """Classify a candidate as pass, fail, inconclusive, or invalid."""
    contract = policy["contract"]
    contract_checks = []
    for path in contract["required_equal"]:
        baseline_value = _value_at(baseline, path)
        candidate_value = _value_at(candidate, path)
        contract_checks.append(
            {
                "path": path,
                "equal": baseline_value == candidate_value,
            }
        )
    for path, expected in contract.get("required_values", {}).items():
        contract_checks.append(
            {
                "path": path,
                "equal": (
                    _value_at(baseline, path) == expected
                    and _value_at(candidate, path) == expected
                ),
            }
        )
    if not all(check["equal"] for check in contract_checks):
        return {
            "schema_version": "proposal-selection-result.v1",
            "status": "invalid_contract",
            "contract_checks": contract_checks,
            "objective": None,
            "gates": [],
            "absolute_gates": [],
            "uncertainty": policy["uncertainty"],
        }

    objective_policy = policy["objective"]
    objective_baseline = _metric(
        baseline, objective_policy["group"], objective_policy["metric"]
    )
    objective_candidate = _metric(
        candidate, objective_policy["group"], objective_policy["metric"]
    )
    objective_delta_pp = 100.0 * (objective_candidate - objective_baseline)
    if objective_delta_pp < float(objective_policy["failure_delta_pp"]):
        objective_status = "fail"
    elif objective_delta_pp < float(objective_policy["minimum_improvement_pp"]):
        objective_status = "inconclusive"
    else:
        objective_status = "pass"
    objective = {
        "name": objective_policy["name"],
        "group": objective_policy["group"],
        "metric": objective_policy["metric"],
        "baseline": objective_baseline,
        "candidate": objective_candidate,
        "delta_pp": objective_delta_pp,
        "status": objective_status,
    }

    gates = []
    for gate in policy["relative_gates"]:
        baseline_value = _metric(baseline, gate["group"], gate["metric"])
        candidate_value = _metric(candidate, gate["group"], gate["metric"])
        delta_pp = 100.0 * (candidate_value - baseline_value)
        gates.append(
            {
                "name": gate["name"],
                "group": gate["group"],
                "metric": gate["metric"],
                "direction": gate["direction"],
                "baseline": baseline_value,
                "candidate": candidate_value,
                "delta_pp": delta_pp,
                "warning_delta_pp": float(gate["warning_delta_pp"]),
                "failure_delta_pp": float(gate["failure_delta_pp"]),
                "status": _delta_status(
                    delta_pp,
                    direction=gate["direction"],
                    warning_delta_pp=float(gate["warning_delta_pp"]),
                    failure_delta_pp=float(gate["failure_delta_pp"]),
                ),
            }
        )

    absolute_gates = []
    for gate in policy.get("absolute_gates", []):
        candidate_value = _metric(candidate, gate["group"], gate["metric"])
        maximum = float(gate["maximum"])
        absolute_gates.append(
            {
                "name": gate["name"],
                "group": gate["group"],
                "metric": gate["metric"],
                "candidate": candidate_value,
                "maximum": maximum,
                "status": "pass" if candidate_value <= maximum else "fail",
            }
        )

    statuses = [objective_status]
    statuses.extend(gate["status"] for gate in gates)
    statuses.extend(gate["status"] for gate in absolute_gates)
    if "fail" in statuses:
        status = "fail"
    elif all(item == "pass" for item in statuses):
        status = "pass"
    else:
        status = "inconclusive"
    return {
        "schema_version": "proposal-selection-result.v1",
        "status": status,
        "contract_checks": contract_checks,
        "objective": objective,
        "gates": gates,
        "absolute_gates": absolute_gates,
        "uncertainty": policy["uncertainty"],
        "report_only": policy.get("report_only", []),
    }


__all__ = ["compare_selection_reports"]
