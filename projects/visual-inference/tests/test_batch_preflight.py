from __future__ import annotations

import signal
import subprocess
from types import SimpleNamespace

import pytest
import torch

from scripts.benchmark_batch_size import (
    _build_targets,
    _run_candidate,
    parse_candidates,
    recommend_batch,
)


def test_preflight_uses_keyword_only_target_device() -> None:
    received: dict[str, object] = {}

    class Builder:
        def __call__(
            self,
            samples: list[object],
            shapes: tuple[tuple[int, int], ...],
            *,
            device: torch.device,
        ) -> str:
            received.update(samples=samples, shapes=shapes, device=device)
            return "targets"

    samples = [object()]
    prediction = SimpleNamespace(quality=[torch.zeros(1, 1, 4, 5)])
    device = torch.device("cpu")

    assert _build_targets(Builder(), samples, prediction, device) == "targets"  # type: ignore[arg-type]
    assert received == {
        "samples": samples,
        "shapes": ((4, 5),),
        "device": device,
    }


def test_candidate_timeout_terminates_the_worker_process_group(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Process:
        pid = 123
        waits = 0

        def wait(self, timeout: int | None = None) -> int:
            self.waits += 1
            if self.waits == 1:
                raise subprocess.TimeoutExpired(["worker"], timeout)
            return -signal.SIGTERM

    process = Process()
    signals: list[tuple[int, signal.Signals]] = []
    monkeypatch.setattr(subprocess, "Popen", lambda *_args, **_kwargs: process)
    monkeypatch.setattr(
        "scripts.benchmark_batch_size.os.killpg",
        lambda pid, sig: signals.append((pid, sig)),
    )

    with pytest.raises(subprocess.TimeoutExpired):
        _run_candidate(["worker"], timeout=5)

    assert signals == [(123, signal.SIGTERM)]


def test_candidate_parser_sorts_and_deduplicates() -> None:
    assert parse_candidates("64, 16,32,64") == [16, 32, 64]
    with pytest.raises(ValueError, match="positive"):
        parse_candidates("0,16")


def test_recommendation_uses_smallest_safe_batch_on_plateau() -> None:
    results = [
        {
            "status": "pass",
            "batch_size": 32,
            "images_per_second": 80.0,
            "memory_headroom_fraction": 0.70,
        },
        {
            "status": "pass",
            "batch_size": 64,
            "images_per_second": 100.0,
            "memory_headroom_fraction": 0.50,
        },
        {
            "status": "pass",
            "batch_size": 96,
            "images_per_second": 104.0,
            "memory_headroom_fraction": 0.30,
        },
        {
            "status": "pass",
            "batch_size": 128,
            "images_per_second": 106.0,
            "memory_headroom_fraction": 0.10,
        },
        {"status": "oom", "batch_size": 160},
    ]

    assert recommend_batch(results, minimum_headroom=0.15, plateau_fraction=0.95) == 64


def test_recommendation_fails_closed_without_memory_headroom() -> None:
    results = [
        {
            "status": "pass",
            "batch_size": 128,
            "images_per_second": 100.0,
            "memory_headroom_fraction": 0.05,
        }
    ]
    assert (
        recommend_batch(results, minimum_headroom=0.15, plateau_fraction=0.95) is None
    )


def test_recommendation_fails_closed_on_infrastructure_error() -> None:
    results = [
        {
            "status": "pass",
            "batch_size": 16,
            "images_per_second": 70.0,
            "memory_headroom_fraction": 0.90,
        },
        {
            "status": "error",
            "batch_size": 32,
            "error": "Pin memory thread exited unexpectedly",
        },
    ]

    assert (
        recommend_batch(results, minimum_headroom=0.15, plateau_fraction=0.95) is None
    )
