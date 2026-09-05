"""Time and measurement helpers shared by the whole subsystem.

Two rules earn their place here rather than being repeated in every module.

**Milliseconds are the lifecycle unit, nanoseconds are the clock unit** (§19). Every
timestamp that crosses a module boundary is an integer number of nanoseconds on the
host monotonic clock — the same domain ``SensorTimestamp`` is reported in and the same
domain controld timestamps the motor history with. Every *duration* in a configuration
or a diagnostic is in milliseconds, because §19's thresholds are written that way and
because a threshold in seconds silently changes meaning every time the inference rate
changes. ``ms_from_ns`` is the one conversion, and it refuses to go backwards past
zero: a negative age (a re-ordered or delayed metadata batch, §52's "reordered/delayed
metadata" case) must read as "brand new", not as a huge age that retires a track.

**Percentiles are computed from a bounded window** (§40). The pipeline publishes p50 /
p95 / p99 / max per stage; an unbounded histogram is a memory leak on a daemon that is
expected to run for days, and a percentile over the whole lifetime hides exactly the
thing worth seeing — that the last thirty seconds are worse than the last hour.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, List, Optional

NS_PER_MS = 1_000_000.0


def ms_from_ns(newer_ns: int, older_ns: int) -> float:
    """Elapsed milliseconds between two monotonic stamps, floored at zero."""
    if newer_ns <= 0 or older_ns <= 0:
        return -1.0  # unknown: never guessed as "0 ms" (which reads as "instant")
    delta = newer_ns - older_ns
    return delta / NS_PER_MS if delta > 0 else 0.0


def clamp_age_ms(age_ms: float) -> float:
    """A clock that went backwards reads as "just measured", not as "ancient"."""
    return 0.0 if age_ms < 0.0 else age_ms


@dataclass
class Summary:
    """A percentile report. §40 asks for exactly these four numbers."""

    count: int = 0
    p50: float = 0.0
    p95: float = 0.0
    p99: float = 0.0
    max: float = 0.0

    def to_dict(self) -> dict:
        return {"count": self.count, "p50": round(self.p50, 3),
                "p95": round(self.p95, 3), "p99": round(self.p99, 3),
                "max": round(self.max, 3)}


def percentile(sorted_values: List[float], q: float) -> float:
    """Nearest-rank percentile over an ALREADY SORTED list.

    Nearest rank rather than interpolation: with 200 samples, "p99" is the worst
    sample, and a report that claims a number between two samples invites the reader
    to believe a resolution the data does not have.
    """
    if not sorted_values:
        return 0.0
    if q <= 0.0:
        return sorted_values[0]
    if q >= 1.0:
        return sorted_values[-1]
    # Nearest rank rounds UP into the sample set, so p95 of 20 values is the 19th.
    rank = int(q * len(sorted_values) + 0.999999) - 1
    rank = max(0, min(rank, len(sorted_values) - 1))
    return sorted_values[rank]


class RingStats:
    """Bounded window of measurements with §40's percentile report.

    ``capacity`` samples are kept in insertion order; the summary sorts on demand, so
    recording stays O(1) on the frame path and reporting stays off it.
    """

    __slots__ = ("name", "_values", "_capacity", "_total", "_sum")

    def __init__(self, name: str, capacity: int = 512) -> None:
        if capacity < 2:
            raise ValueError("RingStats capacity must be at least 2")
        self.name = name
        self._values: deque = deque(maxlen=int(capacity))
        self._capacity = int(capacity)
        self._total = 0
        self._sum = 0.0

    def record(self, value: float) -> None:
        self._values.append(float(value))
        self._total += 1
        self._sum += float(value)

    @property
    def total(self) -> int:
        """Recordings ever seen, including samples that have aged out of the window."""
        return self._total

    @property
    def mean(self) -> float:
        return self._sum / self._total if self._total else 0.0

    def summary(self) -> Summary:
        values = sorted(self._values)
        if not values:
            return Summary()
        return Summary(
            count=len(values),
            p50=percentile(values, 0.50),
            p95=percentile(values, 0.95),
            p99=percentile(values, 0.99),
            max=values[-1],
        )

    def to_dict(self) -> dict:
        out = self.summary().to_dict()
        out["name"] = self.name
        out["window"] = self._capacity
        out["total"] = self._total
        return out


def summarize(values: Iterable[float]) -> Summary:
    """One-shot percentile report for a finished run (evaluator, replay harness)."""
    ordered = sorted(float(v) for v in values)
    if not ordered:
        return Summary()
    return Summary(
        count=len(ordered),
        p50=percentile(ordered, 0.50),
        p95=percentile(ordered, 0.95),
        p99=percentile(ordered, 0.99),
        max=ordered[-1],
    )


def optional_summary(values: Optional[Iterable[float]]) -> Optional[Summary]:
    return None if values is None else summarize(values)
