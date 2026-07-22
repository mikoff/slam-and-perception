from __future__ import annotations

from collections import deque
from concurrent.futures import ProcessPoolExecutor
from itertools import islice
from typing import Callable, Iterable, Iterator, TypeVar


Input = TypeVar("Input")
Output = TypeVar("Output")


def batches(items: Iterable[Input], size: int = 100) -> Iterator[list[Input]]:
    iterator = iter(items)
    while batch := list(islice(iterator, size)):
        yield batch


def process_map(
    function: Callable[[Input], Output], items: Iterable[Input], workers: int,
) -> Iterator[Output]:
    """Map with deterministic ordering and a small, bounded pending queue."""
    if workers == 1:
        yield from map(function, items)
        return

    with ProcessPoolExecutor(max_workers=workers) as executor:
        pending = deque()
        for item in items:
            pending.append(executor.submit(function, item))
            if len(pending) >= workers * 2:
                yield pending.popleft().result()
        while pending:
            yield pending.popleft().result()
