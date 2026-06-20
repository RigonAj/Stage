"""SafetyLimiter (clip + rate + accel) and Watchdog (archi §4.3.5, §9)."""

import math

import pytest

from ur3e_live_catch.safety import JointBound, SafetyLimiter, Watchdog

DT = 1.0 / 60.0


def _bounds(v_safe=1.0, a_safe=1e9, lo=-6.28, hi=6.28):
    return [JointBound(lo, hi, v_safe, a_safe) for _ in range(6)]


def test_position_clip():
    lim = SafetyLimiter(_bounds(v_safe=1e9, a_safe=1e9, lo=-1.0, hi=1.0), DT)
    rep = lim.limit([5.0] * 6, [0.9] * 6)
    assert all(t == pytest.approx(1.0) for t in rep.target)
    assert all(rep.position_clamped)


def test_rate_limit():
    v_safe = 1.0
    lim = SafetyLimiter(_bounds(v_safe=v_safe, a_safe=1e9), DT)
    rep = lim.limit([10.0] * 6, [0.0] * 6)  # huge jump
    step = v_safe * DT
    assert all(t == pytest.approx(step) for t in rep.target)
    assert all(rep.rate_limited)


def test_accel_limit_from_rest():
    a_safe = 2.0
    lim = SafetyLimiter(_bounds(v_safe=1e9, a_safe=a_safe), DT)
    rep = lim.limit([10.0] * 6, [0.0] * 6)
    # from rest, velocity can grow by at most a_safe*dt this tick -> step = a_safe*dt*dt
    expect = a_safe * DT * DT
    assert all(t == pytest.approx(expect) for t in rep.target)
    assert all(rep.accel_limited)


def test_no_limit_when_within_bounds():
    lim = SafetyLimiter(_bounds(v_safe=100.0, a_safe=1e9), DT)
    rep = lim.limit([0.001] * 6, [0.0] * 6)
    assert not rep.any_limited


def test_watchdog_triggers():
    wd = Watchdog(stale_after_s=0.1, loop_budget_s=0.02, max_tracking_error=0.05)
    ok, reasons = wd.check(perception_age_s=0.0, loop_time_s=0.001, tracking_error=0.0)
    assert ok and not reasons

    ok, reasons = wd.check(perception_age_s=0.2, loop_time_s=0.001, tracking_error=0.0)
    assert not ok and any("stale" in r for r in reasons)

    ok, reasons = wd.check(perception_age_s=0.0, loop_time_s=0.05, tracking_error=0.0)
    assert not ok and any("overrun" in r for r in reasons)

    ok, reasons = wd.check(perception_age_s=0.0, loop_time_s=0.001, tracking_error=0.5)
    assert not ok and any("tracking" in r for r in reasons)
