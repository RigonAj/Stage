"""Frame transform + velocity filter + staleness (archi §4.3.1, §12 parité repère)."""

import math

import pytest

from ur3e_live_catch.ball_frame import (
    BallFrameTransformer, BallVelocityFilter, FrameError, RigidTransform,
    producer_velocity,
)


def test_identity_when_base_link():
    t = BallFrameTransformer()
    assert t.to_base([1.0, 2.0, 3.0], "base_link") == (1.0, 2.0, 3.0)


def test_mm_to_m_conversion():
    t = BallFrameTransformer(units="mm")
    assert t.to_base([1000.0, -2000.0, 3000.0], "base_link") == pytest.approx((1.0, -2.0, 3.0))


def test_empty_frame_rejected():
    t = BallFrameTransformer()
    with pytest.raises(FrameError):
        t.to_base([0.0, 0.0, 0.0], "")


def test_unknown_frame_without_transform_rejected():
    t = BallFrameTransformer()
    with pytest.raises(FrameError):
        t.to_base([1.0, 0.0, 0.0], "camera_optical")  # no transform supplied


def test_rotation_plus_translation():
    # 90 deg about +z maps (1,0,0) -> (0,1,0); then translate by (10,0,0).
    s = math.sin(math.pi / 4)
    tf = RigidTransform(translation=(10.0, 0.0, 0.0), quaternion=(0.0, 0.0, s, s))
    t = BallFrameTransformer()
    out = t.to_base([1.0, 0.0, 0.0], "camera_optical", transform=tf)
    assert out == pytest.approx((10.0, 1.0, 0.0), abs=1e-9)


def test_base_to_base_link_rotation_is_explicit_tf():
    # UR base -> base_link is 180 deg about Z: (x, y, z) -> (-x, -y, z).
    tf = RigidTransform(translation=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 1.0, 0.0))
    t = BallFrameTransformer()
    assert t.to_base([1.0, 2.0, 3.0], "base", transform=tf) == pytest.approx((-1.0, -2.0, 3.0))


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
    t = BallFrameTransformer(stale_after_s=0.1)
    t.process([0.0, 0.0, 0.0], "base_link", stamp_s=10.0)
    assert t.is_stale(10.05) is False
    assert t.is_stale(10.2) is True


def test_reset_velocity_clears_ema_between_throws():
    t = BallFrameTransformer()
    t.process([0.0, 0.0, 0.0], "base_link", stamp_s=0.0)
    _, vel = t.process([1.0, 0.0, 0.0], "base_link", stamp_s=0.1)
    assert vel != (0.0, 0.0, 0.0)  # previous flight left EMA state behind
    t.reset_velocity()
    _, vel = t.process([5.0, 5.0, 5.0], "base_link", stamp_s=1.0)
    assert vel == (0.0, 0.0, 0.0)  # first sample of the new throw re-seeds
    # Staleness bookkeeping survives the velocity reset.
    assert t.is_stale(1.05) is False


def test_producer_velocity_zero_means_not_provided():
    assert producer_velocity((0.0, 0.0, 0.0), "base_link", "base_link") is None


def test_producer_velocity_base_frame_passthrough():
    v = producer_velocity((1.0, -2.0, 3.0), "base_link", "base_link")
    assert v == pytest.approx((1.0, -2.0, 3.0))


def test_producer_velocity_rotated_translation_ignored():
    # 90 deg about +z maps (1,0,0) -> (0,1,0); translation must NOT apply to
    # a free vector like velocity.
    s = math.sin(math.pi / 4)
    tf = RigidTransform(translation=(10.0, 0.0, 0.0), quaternion=(0.0, 0.0, s, s))
    v = producer_velocity((1.0, 0.0, 0.0), "camera_optical", "base_link", tf)
    assert v == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_producer_velocity_unknown_frame_without_transform():
    assert producer_velocity((1.0, 0.0, 0.0), "camera_optical", "base_link") is None


def test_producer_velocity_rejects_implausible_speed():
    # Consumer guard: an upstream bug must fall back to the EMA filter.
    assert producer_velocity((30.0, 0.0, 0.0), "base_link", "base_link",
                             max_speed=12.0) is None
    v = producer_velocity((5.0, 0.0, 0.0), "base_link", "base_link", max_speed=12.0)
    assert v == pytest.approx((5.0, 0.0, 0.0))
    assert producer_velocity((float("nan"), 1.0, 0.0), "base_link", "base_link") is None
