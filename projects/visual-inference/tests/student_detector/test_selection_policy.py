from __future__ import annotations

from copy import deepcopy

from student_detector.selection_policy import compare_selection_reports


def _policy() -> dict[str, object]:
    return {
        "contract": {
            "required_values": {"schema_version": "utility.v1"},
            "required_equal": ["validation_subset_sha256"],
        },
        "objective": {
            "name": "ar100",
            "group": "aggregate",
            "metric": "ar/100",
            "minimum_improvement_pp": 0.25,
            "failure_delta_pp": -0.25,
        },
        "relative_gates": [
            {
                "name": "recall",
                "group": "aggregate",
                "metric": "recall/100@0.50",
                "direction": "higher",
                "warning_delta_pp": -0.1,
                "failure_delta_pp": -0.5,
            },
            {
                "name": "false",
                "group": "aggregate",
                "metric": "false/unmatched_fraction",
                "direction": "lower",
                "warning_delta_pp": 0.2,
                "failure_delta_pp": 0.5,
            },
        ],
        "absolute_gates": [
            {
                "name": "budget",
                "group": "aggregate",
                "metric": "proposals/100_per_image",
                "maximum": 100,
            }
        ],
        "uncertainty": {"method": "aggregate_only"},
    }


def _report() -> dict[str, object]:
    return {
        "schema_version": "utility.v1",
        "validation_subset_sha256": "same",
        "hbb": {
            "groups": {
                "aggregate": {
                    "ar/100": 0.15,
                    "recall/100@0.50": 0.4,
                    "false/unmatched_fraction": 0.45,
                    "proposals/100_per_image": 98.0,
                }
            }
        },
    }


def test_clear_improvement_passes_all_gates() -> None:
    baseline = _report()
    candidate = deepcopy(baseline)
    candidate["hbb"]["groups"]["aggregate"]["ar/100"] = 0.153  # type: ignore[index]

    result = compare_selection_reports(baseline, candidate, _policy())

    assert result["status"] == "pass"


def test_near_boundary_is_inconclusive() -> None:
    baseline = _report()
    candidate = deepcopy(baseline)
    candidate["hbb"]["groups"]["aggregate"]["ar/100"] = 0.151  # type: ignore[index]

    result = compare_selection_reports(baseline, candidate, _policy())

    assert result["status"] == "inconclusive"
    assert result["objective"]["status"] == "inconclusive"  # type: ignore[index]


def test_clear_guardrail_regression_fails() -> None:
    baseline = _report()
    candidate = deepcopy(baseline)
    metrics = candidate["hbb"]["groups"]["aggregate"]  # type: ignore[index]
    metrics["ar/100"] = 0.153
    metrics["recall/100@0.50"] = 0.39

    result = compare_selection_reports(baseline, candidate, _policy())

    assert result["status"] == "fail"
    assert result["gates"][0]["status"] == "fail"


def test_lower_is_better_warning_is_inconclusive() -> None:
    baseline = _report()
    candidate = deepcopy(baseline)
    metrics = candidate["hbb"]["groups"]["aggregate"]  # type: ignore[index]
    metrics["ar/100"] = 0.153
    metrics["false/unmatched_fraction"] = 0.453

    result = compare_selection_reports(baseline, candidate, _policy())

    assert result["status"] == "inconclusive"
    assert result["gates"][1]["status"] == "inconclusive"


def test_contract_mismatch_is_invalid() -> None:
    baseline = _report()
    candidate = deepcopy(baseline)
    candidate["validation_subset_sha256"] = "different"

    result = compare_selection_reports(baseline, candidate, _policy())

    assert result["status"] == "invalid_contract"
    assert result["objective"] is None
