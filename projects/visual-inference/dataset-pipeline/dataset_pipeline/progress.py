from __future__ import annotations

import logging
import time


class Progress:
    """Log periodic progress without coupling work to a specific iterator."""

    def __init__(self, label: str, unit: str = "items", interval: float = 30.0) -> None:
        self.label = label
        self.unit = unit
        self.interval = interval
        self.count = 0
        self.started = self.last_report = time.monotonic()

    def add(self, count: int = 1) -> None:
        self.count += count
        now = time.monotonic()
        if now - self.last_report >= self.interval:
            self._log(now)
            self.last_report = now

    def finish(self) -> None:
        self._log(time.monotonic())

    def _log(self, now: float) -> None:
        elapsed = max(now - self.started, 1e-9)
        logging.info(
            "%s: %s %s in %.1fs (%.1f %s/s)",
            self.label, f"{self.count:,}", self.unit, elapsed,
            self.count / elapsed, self.unit,
        )
