"""Excitation safety guards (doc §3 warning box, §6.1, §9) — PURE (stdlib only).

Operates on plain numbers so it is unit-testable off-robot; ``run_sweep`` feeds it
the per-joint limits obtained from :func:`ur3e_live_catch.limits.load_ur3e_joint_limits`.
Two non-negotiable checks before any motion:
  - the whole swing stays inside ``[q_min+margin, q_max-margin]`` (away from stops);
  - for the chirp, the peak speed ``2*pi*f1*A`` stays below half the velocity limit
    (``assert 2*pi*f1*A < 0.5*velocity_limit`` — doc §6.1); for the ramp, the
    commanded speed stays below half the velocity limit.
"""

from __future__ import annotations

from dataclasses import dataclass

from .signals import peak_velocity


class ExcitationUnsafe(ValueError):
    """An excitation would leave the joint limits or exceed the speed guard."""


@dataclass(frozen=True)
class SweepParams:
    """One sweep's signal parameters (only the fields for ``signal`` are used)."""

    signal: str
    amplitude: float = 0.0          # rad (step/chirp swing; sign allowed)
    f0: float = 0.1                 # Hz (chirp)
    f1: float = 3.0                 # Hz (chirp)
    duration: float = 20.0          # s  (chirp)
    velocity: float = 0.1           # rad/s (ramp)
    distance: float = 0.2           # rad (ramp move length, sign allowed)

    def validate(self) -> None:
        if self.signal not in ("step", "chirp", "ramp"):
            raise ExcitationUnsafe(f"unknown signal {self.signal!r}")
        if self.signal in ("step", "chirp") and self.amplitude == 0.0:
            raise ExcitationUnsafe("amplitude must be non-zero for step/chirp")
        if self.signal == "chirp":
            if not (0.0 < self.f0 <= self.f1):
                raise ExcitationUnsafe(f"chirp requires 0 < f0 <= f1, got f0={self.f0}, f1={self.f1}")
            if self.duration <= 0.0:
                raise ExcitationUnsafe("chirp duration must be positive")
        if self.signal == "ramp":
            if self.velocity <= 0.0:
                raise ExcitationUnsafe("ramp velocity must be positive")
            if self.distance == 0.0:
                raise ExcitationUnsafe("ramp distance must be non-zero")


def _swing_bounds(signal: str, center: float, params: SweepParams) -> tuple[float, float]:
    """Min/max joint position the excitation will command around ``center``."""
    if signal in ("step", "chirp"):
        a = abs(params.amplitude)
        return center - a, center + a
    # ramp: center -> center + distance (one direction)
    end = center + params.distance
    return min(center, end), max(center, end)


def check_excitation(
    center: float,
    params: SweepParams,
    *,
    q_min: float,
    q_max: float,
    velocity_limit: float,
    margin: float = 0.1,
    velocity_fraction: float = 0.5,
) -> dict[str, float]:
    """Validate a sweep; raise :class:`ExcitationUnsafe` or return computed metrics.

    ``velocity_limit`` is the joint's nominal max speed (rad/s); the guard caps the
    excitation at ``velocity_fraction`` of it (default 50 %, doc §6.1).
    """
    params.validate()
    if margin < 0.0:
        raise ExcitationUnsafe("margin must be nonnegative")
    if not (0.0 < velocity_fraction <= 1.0):
        raise ExcitationUnsafe("velocity_fraction must be in (0, 1]")
    if velocity_limit <= 0.0:
        raise ExcitationUnsafe("velocity_limit must be positive")

    lo, hi = _swing_bounds(params.signal, center, params)
    if lo < q_min + margin or hi > q_max - margin:
        raise ExcitationUnsafe(
            f"swing [{lo:.3f}, {hi:.3f}] rad leaves the safe band "
            f"[{q_min + margin:.3f}, {q_max - margin:.3f}] (limits [{q_min:.3f}, {q_max:.3f}], "
            f"margin {margin})"
        )

    cap = velocity_fraction * velocity_limit
    metrics = {"swing_min": lo, "swing_max": hi, "velocity_cap": cap}
    if params.signal == "chirp":
        v_peak = peak_velocity(params.amplitude, params.f1)
        if v_peak >= cap:
            raise ExcitationUnsafe(
                f"chirp peak speed 2*pi*f1*A = {v_peak:.3f} rad/s exceeds "
                f"{velocity_fraction:.0%} of the velocity limit ({cap:.3f} rad/s); "
                f"reduce amplitude (A ~ 1/f) or f1"
            )
        metrics["peak_velocity"] = v_peak
    elif params.signal == "ramp":
        if params.velocity >= cap:
            raise ExcitationUnsafe(
                f"ramp speed {params.velocity:.3f} rad/s exceeds {velocity_fraction:.0%} of the "
                f"velocity limit ({cap:.3f} rad/s)"
            )
        metrics["peak_velocity"] = params.velocity
    return metrics
