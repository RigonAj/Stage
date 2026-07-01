"""Action -> joint target mapping (archi §4.3.4), wired at step 6.

Three modes behind a config flag (user decision; default ``faithful``):

  faithful : legacy absolute-action contract. The simulation commanded an
             ABSOLUTE, UNCLIPPED target ``q_target = action * action_scale``
             (verified on the rollouts: ``joint_position_target_rad ==
             action_normalized * 0.5``). The observation feedback (comp 9) stores
             the RAW action. Safety (clip + rate-limit) is a SEPARATE, independent
             layer (archi §9) — so fidelity and safety are reconciled, not traded.

  safe     : the literal doc formula ``q + clamp(action, -1, 1) * v_safe * dt``
             (bounded incremental). Stores the CLIPPED action as comp 9. Safest,
             but diverges from the trained policy (likely needs retraining).

  incremental : current Isaac FirstTraining contract. The target is integrated
             from the previous joint-position target, using clipped actions,
             per-joint velocity, acceleration and position limits. Stores the
             CLIPPED action as comp 9, matching ``firsttraining_env``.

This module only maps; it does NOT enforce limits — :mod:`safety` does.
"""

from __future__ import annotations

import math
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
        a_safe: Optional[Sequence[float]] = None,
        position_lower: Optional[Sequence[float]] = None,
        position_upper: Optional[Sequence[float]] = None,
        dt: float = DT_STEP,
    ) -> None:
        if mode not in ("faithful", "safe", "incremental"):
            raise ValueError(f"mode must be 'faithful', 'safe' or 'incremental', got {mode!r}")
        if mode in ("safe", "incremental") and v_safe is None:
            raise ValueError(f"mode {mode!r} requires v_safe (per-joint rad/s)")
        self.mode = mode
        self.action_scale = float(action_scale)
        self.v_safe = self._vector_or_none(v_safe, "v_safe")
        self.a_safe = self._vector_or_none(a_safe, "a_safe")
        self.position_lower = self._vector_or_none(position_lower, "position_lower")
        self.position_upper = self._vector_or_none(position_upper, "position_upper")
        self.dt = float(dt)
        self._prev_action: list[float] = [0.0] * 6
        self._prev_target: Optional[list[float]] = None
        self._prev_cmd_vel: list[float] = [0.0] * 6

    @staticmethod
    def _vector_or_none(values: Optional[Sequence[float]], name: str) -> Optional[list[float]]:
        if values is None:
            return None
        if len(values) != 6:
            raise ValueError(f"{name} must contain 6 values")
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain finite values")
        return result

    @property
    def prev_action(self) -> list[float]:
        """The value to feed back as observation component 9 next tick."""
        return list(self._prev_action)

    def reset(self, reference_q: Optional[Sequence[float]] = None) -> None:
        """Reset stateful action feedback and incremental integration memory."""
        self._prev_action = [0.0] * 6
        self._prev_cmd_vel = [0.0] * 6
        self._prev_target = None if reference_q is None else self._vector_or_none(reference_q, "reference_q")

    def map(self, action: Sequence[float], q: Sequence[float]) -> list[float]:
        """Return the 6-D joint target and record the comp-9 feedback action."""
        if len(action) != 6 or len(q) != 6:
            raise ValueError("action and q must each have 6 elements")
        if self.mode == "faithful":
            target = [float(a) * self.action_scale for a in action]
            self._prev_action = [float(a) for a in action]  # raw
        elif self.mode == "safe":
            clipped = [_clip(float(a), -1.0, 1.0) for a in action]
            assert self.v_safe is not None
            target = [q[i] + clipped[i] * self.v_safe[i] * self.dt for i in range(6)]
            self._prev_action = clipped
        else:
            target = self._map_incremental(action, q)
        return target

    def _map_incremental(self, action: Sequence[float], q: Sequence[float]) -> list[float]:
        """Mirror ``firsttraining_env._pre_physics_step`` for current exports."""
        assert self.v_safe is not None
        clipped_action = [_clip(float(a), -1.0, 1.0) for a in action]
        reference_q = list(q) if self._prev_target is None else list(self._prev_target)
        target = [0.0] * 6

        for i in range(6):
            desired_cmd_vel = clipped_action[i] * self.v_safe[i]
            a_safe = self.a_safe[i] if self.a_safe is not None else None
            lower = self.position_lower[i] if self.position_lower is not None else None
            upper = self.position_upper[i] if self.position_upper is not None else None

            if a_safe is not None and lower is not None and upper is not None:
                accel_step = a_safe * self.dt
                distance_to_lower = max(reference_q[i] - lower, 0.0)
                distance_to_upper = max(upper - reference_q[i], 0.0)
                max_negative_stop_vel = max(
                    -accel_step + math.sqrt(accel_step * accel_step + 2.0 * a_safe * distance_to_lower),
                    0.0,
                )
                max_positive_stop_vel = max(
                    -accel_step + math.sqrt(accel_step * accel_step + 2.0 * a_safe * distance_to_upper),
                    0.0,
                )
                desired_cmd_vel = _clip(desired_cmd_vel, -max_negative_stop_vel, max_positive_stop_vel)
                max_delta_v = accel_step
                cmd_vel_delta = _clip(
                    desired_cmd_vel - self._prev_cmd_vel[i],
                    -max_delta_v,
                    max_delta_v,
                )
                cmd_vel = _clip(
                    self._prev_cmd_vel[i] + cmd_vel_delta,
                    -self.v_safe[i],
                    self.v_safe[i],
                )
                next_target = reference_q[i] + cmd_vel * self.dt
                next_target = _clip(next_target, lower, upper)
                self._prev_cmd_vel[i] = (next_target - reference_q[i]) / self.dt
            else:
                cmd_vel = _clip(desired_cmd_vel, -self.v_safe[i], self.v_safe[i])
                next_target = reference_q[i] + cmd_vel * self.dt
                self._prev_cmd_vel[i] = cmd_vel
            target[i] = next_target

        self._prev_target = list(target)
        self._prev_action = clipped_action
        return target
