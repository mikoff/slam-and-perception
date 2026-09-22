from __future__ import annotations

from html.parser import HTMLParser

import pytest

from scripts.build_quad_comparison_dashboard import (
    _html_document,
    _validate_shared_cohort,
)


def _report(subset: str = "subset") -> dict[str, object]:
    return {
        "validation_manifest_sha256": "manifest",
        "validation_subset_sha256": subset,
        "source_counts": {"a": 400, "b": 400},
    }


def test_shared_cohort_rejects_different_image_identity() -> None:
    with pytest.raises(ValueError, match="validation_subset_sha256"):
        _validate_shared_cohort([_report(), _report("other")])


def test_dashboard_is_self_contained_and_escapes_script_terminator() -> None:
    document = _html_document({"label": "unsafe </script> value", "models": {}})
    parser = HTMLParser()
    parser.feed(document)

    assert "https://" not in document
    assert "<\\/script>" in document
    assert "__DASHBOARD_DATA__" not in document
    assert 'id="dashboard-data"' in document
