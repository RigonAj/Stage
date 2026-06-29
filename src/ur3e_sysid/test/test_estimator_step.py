import numpy as np

from ur3e_sysid import estimator


def test_recover_underdamped_step():
    wn, zeta = 8.0, 0.3
    t = np.arange(0.0, 4.0, 0.001)
    y = estimator.second_order_step(wn, zeta, t)
    assert y.max() > 1.0  # has overshoot
    wn_fit, zeta_fit, r2 = estimator.fit_step(t, y)
    assert abs(wn_fit - wn) / wn < 0.05
    assert abs(zeta_fit - zeta) < 0.03
    assert r2 > 0.99


def test_recover_overdamped_step():
    wn, zeta = 5.0, 1.5
    t = np.arange(0.0, 5.0, 0.002)
    y = estimator.second_order_step(wn, zeta, t)
    assert y.max() <= 1.0 + 1e-3  # no overshoot
    wn_fit, zeta_fit, r2 = estimator.fit_step(t, y)
    assert zeta_fit >= 1.0
    assert abs(wn_fit - wn) / wn < 0.15
    assert r2 > 0.98


def test_normalisation_offset_invariance():
    # fit_step subtracts t[0]; a non-zero start time must not change the result.
    wn, zeta = 10.0, 0.25
    t = np.arange(2.0, 5.0, 0.001)
    y = estimator.second_order_step(wn, zeta, t - t[0])
    wn_fit, zeta_fit, _ = estimator.fit_step(t, y)
    assert abs(wn_fit - wn) / wn < 0.05
    assert abs(zeta_fit - zeta) < 0.03
