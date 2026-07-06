"""Joint-limit -> SafetyLimiter bounds (archi §4.3.5, §9; sim-to-real §2.1)."""

import math

import pytest

from ur3e_live_catch.joint_order import JOINT_ORDER
from ur3e_live_catch.limits import (
    JointLimit,
    build_joint_bounds,
    nominal_ur3e_limits,
    scale_bounds,
    v_safe_vector,
)


def test_nominal_v_safe_is_half_the_urdf_limit():
    """v_safe = nominal max_velocity * 0.5 -> 1.571/1.571/1.571/3.142/3.142/3.142."""
    bounds = build_joint_bounds(nominal_ur3e_limits())
    v_safe = v_safe_vector(bounds)
    expect = [math.pi / 2, math.pi / 2, math.pi / 2, math.pi, math.pi, math.pi]
    assert v_safe == pytest.approx(expect)
    # matches the documented numeric caps
    assert v_safe == pytest.approx([1.5708, 1.5708, 1.5708, 3.1416, 3.1416, 3.1416], abs=1e-3)


def test_bounds_are_ordered_canonically():
    limits = {
        name: JointLimit(min_position=-1.0 - i, max_position=1.0 + i, max_velocity=2.0 + i)
        for i, name in enumerate(JOINT_ORDER)
    }
    bounds = build_joint_bounds(limits, v_safe_factor=0.5, a_safe=5.0)
    assert len(bounds) == 6
    for i, b in enumerate(bounds):
        assert b.min_position == pytest.approx(-1.0 - i)
        assert b.max_position == pytest.approx(1.0 + i)
        assert b.v_safe == pytest.approx((2.0 + i) * 0.5)
        assert b.a_safe == pytest.approx(5.0)


def test_missing_joint_raises():
    limits = dict(nominal_ur3e_limits())
    del limits["wrist_3_joint"]
    with pytest.raises(KeyError):
        build_joint_bounds(limits)


def test_non_finite_velocity_raises():
    limits = dict(nominal_ur3e_limits())
    limits["elbow_joint"] = JointLimit(-1.0, 1.0, math.inf)
    with pytest.raises(ValueError):
        build_joint_bounds(limits)


def test_bad_factor_and_accel():
    with pytest.raises(ValueError):
        build_joint_bounds(nominal_ur3e_limits(), v_safe_factor=0.0)
    with pytest.raises(ValueError):
        build_joint_bounds(nominal_ur3e_limits(), v_safe_factor=1.5)
    with pytest.raises(ValueError):
        build_joint_bounds(nominal_ur3e_limits(), a_safe=0.0)


def test_scale_bounds_halves_velocity_and_accel_only():
    bounds = build_joint_bounds(nominal_ur3e_limits(), v_safe_factor=1.0, a_safe=10.0)
    scaled = scale_bounds(bounds, 0.5)
    for b, s in zip(bounds, scaled):
        assert s.v_safe == pytest.approx(b.v_safe * 0.5)
        assert s.a_safe == pytest.approx(b.a_safe * 0.5)
        assert s.min_position == pytest.approx(b.min_position)
        assert s.max_position == pytest.approx(b.max_position)


def test_scale_bounds_can_overdrive_velocity_and_accel_for_limit_tests():
    bounds = build_joint_bounds(nominal_ur3e_limits(), v_safe_factor=1.0, a_safe=10.0)
    scaled = scale_bounds(bounds, 4.0)
    for b, s in zip(bounds, scaled):
        assert s.v_safe == pytest.approx(b.v_safe * 4.0)
        assert s.a_safe == pytest.approx(b.a_safe * 4.0)
        assert s.min_position == pytest.approx(b.min_position)
        assert s.max_position == pytest.approx(b.max_position)


def test_scale_bounds_identity_and_validation():
    bounds = build_joint_bounds(nominal_ur3e_limits())
    assert scale_bounds(bounds, 1.0) == list(bounds)
    with pytest.raises(ValueError):
        scale_bounds(bounds, 0.0)
    with pytest.raises(ValueError):
        scale_bounds(bounds, 4.1)
