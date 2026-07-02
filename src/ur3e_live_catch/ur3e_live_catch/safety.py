"""Safety layer: clip + rate-limit + watchdog (archi §4.3.5, §9), wired at step 6.

Independent of the policy output: bounds are enforced robot-side regardless of
the action mode. Current Isaac exports supply bounds through
``policy_metadata.json``; legacy/manual fallback bounds can still be built from
``ur_description/.../joint_limits.yaml`` and ``v_safe_factor``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

Vec = Sequence[float]

TWO_PI = 2.0 * math.pi


def start_pose_violations(
    q: Vec,
    joint_names: Sequence[str],
    limit_rad: float,
) -> list[str]:
    """Sanity-check the measured pose before the FIRST command of a session.

    The UR controller keeps its own internal joint representation; after a
    power cycle or a wound wrist, ``/joint_states`` can report a joint on a
    ±2π-shifted branch (e.g. wrist_3 = 6.2832 while the pendant shows ~0°).
    Echoing that pose back as a command then looks like a full-turn jump to the
    driver, which rejects it ("Velocity ... exceeding the joint velocity
    limits") and latches "Ignoring commands until a valid command is received".

    Returns one human-readable message per violating joint (empty = safe).
    """
    if len(q) != len(joint_names):
        raise ValueError("q and joint_names must have the same length")
    violations: list[str] = []
    for name, value in zip(joint_names, q):
        if abs(value) <= limit_rad:
            continue
        wrapped = math.remainder(value, TWO_PI)  # principal value in (-pi, pi]
        violations.append(
            f"{name} measured at {value:+.4f} rad ({math.degrees(value):+.1f} deg), "
            f"beyond start-pose limit ±{limit_rad:.2f} rad; principal angle is "
            f"{wrapped:+.4f} rad — likely a ±2π-wrapped branch. Jog/unwind the "
            f"joint (or reboot the arm) until /joint_states matches the pendant."
        )
    return violations


def _clip(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


@dataclass(frozen=True)
class JointBound:
    min_position: float
    max_position: float
    v_safe: float          # rad/s
    a_safe: float          # rad/s^2


@dataclass
class SafetyReport:
    target: list[float]
    position_clamped: list[bool] = field(default_factory=list)
    rate_limited: list[bool] = field(default_factory=list)
    accel_limited: list[bool] = field(default_factory=list)

    @property
    def any_limited(self) -> bool:
        return any(self.position_clamped) or any(self.rate_limited) or any(self.accel_limited)


class SafetyLimiter:
    """Clip to joint range, then rate-limit to v_safe*dt and accel to a_safe*dt."""

    def __init__(self, bounds: Sequence[JointBound], dt: float) -> None:
        if len(bounds) != 6:
            raise ValueError("expected 6 joint bounds")
        self.bounds = list(bounds)
        self.dt = float(dt)
        self._prev_vel: list[float] = [0.0] * 6

    def reset(self) -> None:
        self._prev_vel = [0.0] * 6

    def limit(self, target: Vec, q: Vec) -> SafetyReport:
        if len(target) != 6 or len(q) != 6:
            raise ValueError("target and q must each have 6 elements")
        report = SafetyReport(target=[0.0] * 6,
                              position_clamped=[False] * 6,
                              rate_limited=[False] * 6,
                              accel_limited=[False] * 6)
        for i, b in enumerate(self.bounds):
            desired = float(target[i])
            # 1) position clip
            clipped = _clip(desired, b.min_position, b.max_position)
            report.position_clamped[i] = clipped != desired

            # 2) velocity (rate) limit: |delta| <= v_safe * dt
            max_step = b.v_safe * self.dt
            delta = clipped - q[i]
            if delta > max_step:
                delta = max_step
                report.rate_limited[i] = True
            elif delta < -max_step:
                delta = -max_step
                report.rate_limited[i] = True

            # 3) acceleration limit: |vel - prev_vel| <= a_safe * dt
            vel = delta / self.dt if self.dt > 0 else 0.0
            max_dvel = b.a_safe * self.dt
            dvel = vel - self._prev_vel[i]
            if dvel > max_dvel:
                vel = self._prev_vel[i] + max_dvel
                report.accel_limited[i] = True
            elif dvel < -max_dvel:
                vel = self._prev_vel[i] - max_dvel
                report.accel_limited[i] = True

            delta = vel * self.dt
            self._prev_vel[i] = vel
            report.target[i] = q[i] + delta
        return report


class Watchdog:
    """Dead-man checks: stale perception, blown loop budget, tracking error (§9)."""

    def __init__(self, stale_after_s: float, loop_budget_s: float, max_tracking_error: float) -> None:
        self.stale_after_s = float(stale_after_s)
        self.loop_budget_s = float(loop_budget_s)
        self.max_tracking_error = float(max_tracking_error)

    def check(
        self,
        *,
        perception_age_s: float,
        loop_time_s: float,
        tracking_error: Optional[float] = None,
    ) -> tuple[bool, list[str]]:
        """Return ``(ok, reasons)``. ``ok=False`` => caller triggers a controlled stop."""
        reasons: list[str] = []
        if perception_age_s > self.stale_after_s:
            reasons.append(f"stale_perception({perception_age_s:.3f}s>{self.stale_after_s:.3f}s)")
        if loop_time_s > self.loop_budget_s:
            reasons.append(f"loop_overrun({loop_time_s:.4f}s>{self.loop_budget_s:.4f}s)")
        if tracking_error is not None and tracking_error > self.max_tracking_error:
            reasons.append(f"tracking_error({tracking_error:.4f}>{self.max_tracking_error:.4f})")
        return (not reasons), reasons
