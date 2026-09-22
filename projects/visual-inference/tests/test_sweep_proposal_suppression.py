from __future__ import annotations

from scripts.sweep_proposal_suppression import _select_validation_indices


def test_all_validation_selection_preserves_manifest_order() -> None:
    records = [object(), object(), object()]

    selected = _select_validation_indices(
        records,
        {"unavailable_source": 1.0},
        100,
        all_images=True,
    )

    assert selected == [0, 1, 2]
