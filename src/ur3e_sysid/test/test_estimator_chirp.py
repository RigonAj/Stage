"""Pivot test: synthesise a known 2nd-order-plus-delay response to a chirp command
and check that fit_chirp recovers (wn, zeta, L) with R^2 >= 0.95 (doc §4.3, §12)."""

import math

import numpy as np
import pytest
from scipy.signal import TransferFunction, lsim

from ur3e_sysid import estimator


def _synth(wn, zeta, L, *, fs=200.0, T=20.0, f0=0.2, f1=4.0, amp=0.05, noise=0.0, seed=0):
    t = np.arange(0.0, T, 1.0 / fs)
    phase = 2 * math.pi * (f0 * t + 0.5 * (f1 - f0) * t * t / T)
    cmd = amp * np.sin(phase)
    cmd_delayed = np.interp(t - L, t, cmd, left=0.0)
    sys = TransferFunction([wn * wn], [1.0, 2.0 * zeta * wn, wn * wn])
    _, y, _ = lsim(sys, U=cmd_delayed, T=t)
    if noise:
        y = y + np.random.default_rng(seed).normal(0.0, noise, size=y.shape)
    return t, cmd, y, f0, f1


def test_recover_second_order_with_delay():
    wn, zeta, L = 15.0, 0.4, 0.02
    t, cmd, y, f0, f1 = _synth(wn, zeta, L)
    wn_f, zeta_f, L_f, r2 = estimator.fit_chirp(t, cmd, y, f0, f1)
    assert r2 >= 0.95
    assert abs(wn_f - wn) / wn < 0.1
    assert abs(zeta_f - zeta) < 0.1
    assert abs(L_f - L) < 0.01


def test_recover_no_delay():
    wn, zeta, L = 12.0, 0.6, 0.0
    t, cmd, y, f0, f1 = _synth(wn, zeta, L)
    wn_f, zeta_f, L_f, r2 = estimator.fit_chirp(t, cmd, y, f0, f1)
    assert r2 >= 0.95
    assert abs(wn_f - wn) / wn < 0.1
    assert abs(zeta_f - zeta) < 0.1
    assert L_f < 0.01


def test_robust_to_light_noise():
    wn, zeta, L = 15.0, 0.4, 0.02
    t, cmd, y, f0, f1 = _synth(wn, zeta, L, noise=1e-3, seed=3)
    wn_f, zeta_f, L_f, r2 = estimator.fit_chirp(t, cmd, y, f0, f1)
    assert r2 >= 0.9
    assert abs(wn_f - wn) / wn < 0.15


def test_measure_fs_from_timestamps():
    t = np.arange(0.0, 1.0, 1.0 / 250.0)
    assert math.isclose(estimator.measure_fs(t), 250.0, rel_tol=1e-6)
