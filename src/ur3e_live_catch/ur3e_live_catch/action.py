"""Action -> joint target mapping (archi §4.3.4), wired at step 6.

Two modes behind a config flag (user decision; default ``faithful``):

  faithful : reproduce the trained policy exactly. The simulation commanded an
             ABSOLUTE, UNCLIPPED target ``q_target = action * action_scale``
             (verified on the rollouts: ``joint_position_target_rad ==
             action_normalized * 0.5``). The observation feedback (comp 9) stores
             the RAW action. Safety (clip + rate-limit) is a SEPARATE, independent
             layer (archi §9) — so fidelity and safety are reconciled, not traded.

  safe     : the literal doc formula ``q + clamp(action, -1, 1) * v_safe * dt``
             (bounded incremental). Stores the CLIPPED action as comp 9. Safest,
             but diverges from the trained policy (likely needs retraining).

This module only maps; it does NOT enforce limits — :mod:`safety` does.
"""

from __future__ import annotations

from typing import Optional, Sequence

ACTION_SCALE = 0.5  # policy_metadata.json: joint_position_target_rad = action * 0.5
DT_STEP = 1.0 / 60.0


def _clip(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


class ActionMapper:
    def __init__(
        self,
        mode: str = "faithful",
        *,
        action_scale: float = ACTION_SCALE,
        v_safe: Optional[Sequence[float]] = None,
        dt: float = DT_STEP,
    ) -> None:
        if mode not in ("faithful", "safe"):
            raise ValueError(f"mode must be 'faithful' or 'safe', got {mode!r}")
        if mode == "safe" and v_safe is None:
            raise ValueError("mode 'safe' requires v_safe (per-joint rad/s)")
        self.mode = mode
        self.action_scale = float(action_scale)
        self.v_safe = list(v_safe) if v_safe is not None else None
        self.dt = float(dt)
        self._prev_action: list[float] = [0.0] * 6

    @property
    def prev_action(self) -> list[float]:
        """The value to feed back as observation component 9 next tick."""
        return list(self._prev_action)

    def map(self, action: Sequence[float], q: Sequence[float]) -> list[float]:
        """Return the 6-D joint target and record the comp-9 feedback action."""
        if len(action) != 6 or len(q) != 6:
            raise ValueError("action and q must each have 6 elements")
        if self.mode == "faithful":
            target = [float(a) * self.action_scale for a in action]
            self._prev_action = [float(a) for a in action]  # raw
        else:
            clipped = [_clip(float(a), -1.0, 1.0) for a in action]
            assert self.v_safe is not None
            target = [q[i] + clipped[i] * self.v_safe[i] * self.dt for i in range(6)]
            self._prev_action = clipped
        return target
