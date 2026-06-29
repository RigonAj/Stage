"""Excitation signal generators (doc §3, §6.1) — PURE (stdlib only, no rclpy/numpy).

One joint at a time, centred on a safe mid-range pose. Three signals:
  - ``step``  : near-instant set-point change via the trajectory controller
                (hold q0, jump to q0+A, settle) -> rise time / overshoot.
  - ``ramp``  : constant-velocity move -> stationary torque vs speed (friction).
  - ``chirp`` : linear-sweep sine ``q0 + A*sin(phase)`` streamed at 60 Hz through
                forward_position_controller (the deployment path, doc §1) -> FRF.

The chirp instantaneous phase is the integral of ``2*pi*f(t)`` with a linear
frequency sweep ``f(t) = f0 + (f1-f0)*t/T``:
    ``phase(t) = 2*pi*(f0*t + 0.5*(f1-f0)*t^2/T)``.
"""

from __future__ import annotations

import math
from typing import Sequence

TWO_PI = 2.0 * math.pi


def peak_velocity(amplitude: float, f_max: float) -> float:
    """Peak joint speed of an A-amplitude sine at ``f_max`` Hz: ``2*pi*f*A`` (doc §3)."""
    return TWO_PI * f_max * abs(amplitude)


def peak_acceleration(amplitude: float, f_max: float) -> float:
    """Peak joint acceleration of an A-amplitude sine at ``f_max`` Hz: ``(2*pi*f)^2*A``."""
    return (TWO_PI * f_max) ** 2 * abs(amplitude)


def chirp_instant_frequency(t: float, f0: float, f1: float, duration: float) -> float:
    """Instantaneous (linear-sweep) frequency at time ``t`` in [0, duration]."""
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    return f0 + (f1 - f0) * t / duration


def chirp_phase(t: float, f0: float, f1: float, duration: float) -> float:
    """Instantaneous phase ``2*pi*(f0*t + 0.5*(f1-f0)*t^2/T)`` (integral of 2*pi*f)."""
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    return TWO_PI * (f0 * t + 0.5 * (f1 - f0) * t * t / duration)


def chirp_value(t: float, q0: float, amplitude: float, f0: float, f1: float, duration: float) -> float:
    """Commanded joint position of the chirp at time ``t``: ``q0 + A*sin(phase)``."""
    return q0 + amplitude * math.sin(chirp_phase(t, f0, f1, duration))


def step_profile(
    q0: float,
    amplitude: float,
    *,
    dwell_s: float = 1.0,
    rise_s: float = 0.05,
    settle_s: float = 2.0,
) -> tuple[list[float], list[float]]:
    """Four-point set-point profile for a near-step through the trajectory action.

    Hold ``q0`` (``dwell_s``), change to ``q0+amplitude`` over a short ``rise_s``
    (so the controller sees a fast set-point change), then hold to observe the
    settling. Returns ``(times, values)`` with times strictly increasing from 0.
    """
    if dwell_s < 0.0 or rise_s <= 0.0 or settle_s <= 0.0:
        raise ValueError("dwell_s>=0, rise_s>0, settle_s>0 required")
    q1 = q0 + amplitude
    times = [0.0, dwell_s, dwell_s + rise_s, dwell_s + rise_s + settle_s]
    values = [q0, q0, q1, q1]
    return times, values


def ramp_profile(
    q0: float,
    distance: float,
    velocity: float,
    *,
    dwell_s: float = 0.5,
) -> tuple[list[float], list[float]]:
    """Constant-velocity move ``q0 -> q0+distance`` at ``|velocity|`` (friction, doc §4.6).

    Returns ``(times, values)``: dwell at ``q0`` then the constant-speed segment.
    """
    if velocity <= 0.0:
        raise ValueError("velocity must be positive")
    if distance == 0.0:
        raise ValueError("distance must be non-zero")
    if dwell_s < 0.0:
        raise ValueError("dwell_s must be nonnegative")
    move_s = abs(distance) / velocity
    times = [0.0, dwell_s, dwell_s + move_s]
    values = [q0, q0, q0 + distance]
    return times, values


def chirp_profile(
    q0: float,
    amplitude: float,
    f0: float,
    f1: float,
    duration: float,
    *,
    rate: float = 60.0,
) -> tuple[list[float], list[float]]:
    """Sample the chirp at ``rate`` Hz over [0, duration]. Returns ``(times, values)``.

    ``rate`` defaults to 60 Hz to match ``ur3e_live_catch.streaming`` (the
    deployment path). Used by tests and as the command reference for the fit.
    """
    if rate <= 0.0:
        raise ValueError("rate must be positive")
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    n = int(round(duration * rate))
    times = [k / rate for k in range(n + 1)]
    values = [chirp_value(t, q0, amplitude, f0, f1, duration) for t in times]
    return times, values


def embed(
    base_pose: Sequence[float],
    joint_index: int,
    scalar_values: Sequence[float],
) -> list[tuple[float, ...]]:
    """Lift per-joint scalar set-points into full poses: ``base_pose`` with the
    excited joint replaced by each scalar. Returns a list of 6-tuples."""
    base = [float(v) for v in base_pose]
    if not (0 <= joint_index < len(base)):
        raise IndexError(f"joint_index {joint_index} out of range for pose of length {len(base)}")
    out: list[tuple[float, ...]] = []
    for v in scalar_values:
        row = list(base)
        row[joint_index] = float(v)
        out.append(tuple(row))
    return out
