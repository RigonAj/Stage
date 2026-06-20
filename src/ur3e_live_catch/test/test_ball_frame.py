"""Frame transform + velocity filter + staleness (archi §4.3.1, §12 parité repère)."""

import math

import pytest

from ur3e_live_catch.ball_frame import (
    BallFrameTransformer, BallVelocityFilter, FrameError, RigidTransform,
)


def test_identity_when_base():
    t = BallFrameTransformer(base_frame="base")
    assert t.to_base([1.0, 2.0, 3.0], "base") == (1.0, 2.0, 3.0)


def test_mm_to_m_conversion():
    t = BallFrameTransformer(base_frame="base", units="mm")
    assert t.to_base([1000.0, -2000.0, 3000.0], "base") == pytest.approx((1.0, -2.0, 3.0))


def test_empty_frame_rejected():
    t = BallFrameTransformer()
    with pytest.raises(FrameError):
        t.to_base([0.0, 0.0, 0.0], "")


def test_unknown_frame_without_transform_rejected():
    t = BallFrameTransformer(base_frame="base")
    with pytest.raises(FrameError):
        t.to_base([1.0, 0.0, 0.0], "camera_optical")  # no transform supplied


def test_rotation_plus_translation():
    # 90 deg about +z maps (1,0,0) -> (0,1,0); then translate by (10,0,0).
    s = math.sin(math.pi / 4)
    tf = RigidTransform(translation=(10.0, 0.0, 0.0), quaternion=(0.0, 0.0, s, s))
    t = BallFrameTransformer(base_frame="base")
    out = t.to_base([1.0, 0.0, 0.0], "camera_optical", transform=tf)
    assert out == pytest.approx((10.0, 1.0, 0.0), abs=1e-9)


def test_velocity_finite_difference_exact_with_alpha_one():
    f = BallVelocityFilter(ema_alpha=1.0)
    assert f.update([0.0, 0.0, 0.0], 0.0) == (0.0, 0.0, 0.0)  # first call
    v = f.update([0.1, 0.2, -0.3], 0.1)  # dt = 0.1 s (realistic ~30 Hz range)
    assert v == pytest.approx((1.0, 2.0, -3.0))


def test_velocity_invariant_to_constant_translation():
    # Adding a constant offset (env origin) must not change the velocity.
    f = BallVelocityFilter(ema_alpha=1.0)
    f.update([5.0, 5.0, 5.0], 0.0)
    v = f.update([6.0, 5.0, 5.0], 0.5)
    assert v == pytest.approx((2.0, 0.0, 0.0))


def test_staleness_flag():
    t = BallFrameTransformer(base_frame="base", stale_after_s=0.1)
    t.process([0.0, 0.0, 0.0], "base", stamp_s=10.0)
    assert t.is_stale(10.05) is False
    assert t.is_stale(10.2) is True
