"""Boolean-signal flap detector (pure logic, no ROS imports).

Used on ``CatchTelemetry.command_enabled``: with a single live_catch_node the
flag only changes on operator action (a few times per session). Several
transitions inside a short sliding window therefore means two live_catch_node
instances publish interleaved telemetry with opposite command states — the
virtual-ball stack still running next to a manual live_catch launch
(2026-07-09 incident). The Web UI surfaces this instead of rendering a
flickering command status.
"""

from __future__ import annotations

from collections import deque


class FlapDetector:
    """Report True while a boolean signal toggles repeatedly.

    ``min_transitions`` transitions within ``window_s`` seconds => flapping.
    The state decays on its own: once transitions age out of the window the
    detector reports False again.
    """

    def __init__(self, window_s: float = 2.0, min_transitions: int = 3) -> None:
        if window_s <= 0.0:
            raise ValueError(f"window_s must be > 0, got {window_s!r}")
        if min_transitions < 1:
            raise ValueError(f"min_transitions must be >= 1, got {min_transitions!r}")
        self.window_s = float(window_s)
        self.min_transitions = int(min_transitions)
        self._last: bool | None = None
        self._transitions: deque[float] = deque()

    def observe(self, value: bool, now_s: float) -> bool:
        """Feed one sample; returns the current flapping verdict."""
        value = bool(value)
        if self._last is not None and value != self._last:
            self._transitions.append(float(now_s))
        self._last = value
        return self.flapping(now_s)

    def flapping(self, now_s: float) -> bool:
        cutoff = float(now_s) - self.window_s
        while self._transitions and self._transitions[0] < cutoff:
            self._transitions.popleft()
        return len(self._transitions) >= self.min_transitions
