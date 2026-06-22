"""Latency statistics (archi §10, step 8) — pure logic, stdlib only.

The architecture requires measuring end-to-end latency (capture -> detection -> obs
-> policy -> command) and checking it against the latency modelled at training
(sim-to-real §5.6): if the real latency exceeds it, the randomisation must be
widened and the policy retrained. This module accumulates timing samples and
reports count / mean / min / max / p50 / p95 / p99 so the ``latency_report`` node
(and tests) can summarise a run without numpy.

Running count/sum/min/max are exact over the whole session; percentiles come from
a bounded ring buffer (``window`` most-recent samples) to keep memory flat on a
long run.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional


class LatencyStats:
    def __init__(self, window: int = 20000) -> None:
        if window < 1:
            raise ValueError(f"window must be >= 1, got {window}")
        self._window: deque[float] = deque(maxlen=window)
        self._count = 0
        self._sum = 0.0
        self._min: Optional[float] = None
        self._max: Optional[float] = None

    def add(self, value_s: float) -> None:
        v = float(value_s)
        self._window.append(v)
        self._count += 1
        self._sum += v
        self._min = v if self._min is None else min(self._min, v)
        self._max = v if self._max is None else max(self._max, v)

    @property
    def count(self) -> int:
        return self._count

    @property
    def mean(self) -> float:
        return self._sum / self._count if self._count else 0.0

    @property
    def min(self) -> float:
        return self._min if self._min is not None else 0.0

    @property
    def max(self) -> float:
        return self._max if self._max is not None else 0.0

    def percentile(self, p: float) -> float:
        """Nearest-rank percentile (``0 <= p <= 100``) over the ring-buffer window."""
        if not (0.0 <= p <= 100.0):
            raise ValueError(f"p must be in [0, 100], got {p}")
        if not self._window:
            return 0.0
        ordered = sorted(self._window)
        # nearest-rank: index = ceil(p/100 * n) - 1, clamped to [0, n-1]
        idx = math.ceil((p / 100.0) * len(ordered)) - 1
        idx = min(max(idx, 0), len(ordered) - 1)
        return ordered[idx]

    def summary(self) -> dict:
        return {
            "count": self._count,
            "mean": self.mean,
            "min": self.min,
            "max": self.max,
            "p50": self.percentile(50),
            "p95": self.percentile(95),
            "p99": self.percentile(99),
        }

    def format_ms(self, label: str = "") -> str:
        s = self.summary()
        if not s["count"]:
            return f"{label}: no samples"
        return (
            f"{label}: n={s['count']} "
            f"mean={s['mean'] * 1e3:.2f} p50={s['p50'] * 1e3:.2f} "
            f"p95={s['p95'] * 1e3:.2f} p99={s['p99'] * 1e3:.2f} "
            f"max={s['max'] * 1e3:.2f} ms"
        )
