"""Offline gain/latency/friction estimator (doc §4) — PURE math (numpy/scipy).

Fits, per joint, the closed-loop surrogate of the ``position command -> position``
response:
  - :func:`fit_step`   step response   -> (wn, zeta)        (doc §4.2)
  - :func:`fit_chirp`  FRF of the chirp -> (wn, zeta, L)     (doc §4.3, recommended)
  - :func:`fit_friction` ramps          -> (Fc, Fv)          (doc §4.6, secondary)
  - :func:`reconcile`  consolidate chirp (wide-band, priority) vs step.

``(wn, zeta, L)`` are inertia-independent; the gains follow with ``I_eff`` (doc §4.1):
``K = I_eff*wn^2``, ``D = 2*zeta*wn*I_eff``. No rclpy/ROS — unit-tested on synthetic
signals.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import coherence, csd, welch

EPS = 1e-12


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def r_squared(y: Sequence[float], y_hat: Sequence[float]) -> float:
    """Coefficient of determination of ``y_hat`` against ``y`` (real signals)."""
    y = np.asarray(y, dtype=float)
    y_hat = np.asarray(y_hat, dtype=float)
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    if ss_tot <= EPS:
        return 0.0
    return 1.0 - ss_res / ss_tot


def frf_r_squared(H: np.ndarray, H_hat: np.ndarray) -> float:
    """R^2 for a complex frequency-response fit."""
    H = np.asarray(H)
    H_hat = np.asarray(H_hat)
    ss_res = float(np.sum(np.abs(H - H_hat) ** 2))
    ss_tot = float(np.sum(np.abs(H - np.mean(H)) ** 2))
    if ss_tot <= EPS:
        return 0.0
    return 1.0 - ss_res / ss_tot


def measure_fs(t: Sequence[float]) -> float:
    """Sampling rate from timestamps: ``1 / median(diff(t))`` (doc §5 — never assume 500 Hz)."""
    t = np.asarray(t, dtype=float)
    if t.size < 2:
        raise ValueError("need at least two samples to measure fs")
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"non-positive median sample interval: {dt}")
    return 1.0 / float(dt)


def second_order_step(wn: float, zeta: float, t: Sequence[float]) -> np.ndarray:
    """Unit step response of ``wn^2/(s^2 + 2*zeta*wn*s + wn^2)`` from rest."""
    t = np.asarray(t, dtype=float)
    t = np.clip(t, 0.0, None)
    if wn <= 0.0:
        return np.zeros_like(t)
    if zeta < 1.0 - 1e-6:                       # underdamped
        wd = wn * math.sqrt(1.0 - zeta * zeta)
        phi = math.acos(max(-1.0, min(1.0, zeta)))
        env = np.exp(-zeta * wn * t) / math.sqrt(1.0 - zeta * zeta)
        return 1.0 - env * np.sin(wd * t + phi)
    if zeta > 1.0 + 1e-6:                        # overdamped (two real poles)
        root = wn * math.sqrt(zeta * zeta - 1.0)
        s1, s2 = -zeta * wn + root, -zeta * wn - root
        b = wn * wn / (s1 * (s1 - s2))
        c = wn * wn / (s2 * (s2 - s1))
        return 1.0 + b * np.exp(s1 * t) + c * np.exp(s2 * t)
    return 1.0 - (1.0 + wn * t) * np.exp(-wn * t)  # critically damped


# --------------------------------------------------------------------------- #
# step fit (doc §4.2)
# --------------------------------------------------------------------------- #
def _rise_time_10_90(t: np.ndarray, y: np.ndarray) -> float:
    def crossing(level: float) -> float:
        idx = np.argmax(y >= level)
        if y[idx] < level:
            return float(t[-1])
        if idx == 0:
            return float(t[0])
        t0, t1, y0, y1 = t[idx - 1], t[idx], y[idx - 1], y[idx]
        return float(t0 + (level - y0) * (t1 - t0) / (y1 - y0)) if y1 != y0 else float(t1)

    return max(crossing(0.9) - crossing(0.1), EPS)


def fit_overdamped(t: Sequence[float], y: Sequence[float]) -> tuple[float, float, float]:
    """Fit (wn, zeta>=1) to a non-overshooting step via least squares on the model."""
    t = np.asarray(t, dtype=float) - float(t[0])
    y = np.asarray(y, dtype=float)
    wn0 = 2.2 / _rise_time_10_90(t, y)
    res = least_squares(
        lambda p: second_order_step(p[0], p[1], t) - y,
        x0=[max(wn0, 1e-2), 1.2],
        bounds=([1e-3, 1.0], [np.inf, 12.0]),
    )
    wn, zeta = float(res.x[0]), float(res.x[1])
    return wn, zeta, r_squared(y, second_order_step(wn, zeta, t))


def fit_step(t: Sequence[float], y_norm: Sequence[float]) -> tuple[float, float, float]:
    """Step response -> (wn, zeta, R^2). ``y_norm`` normalised 0 (pre-step) -> 1 (final),
    ``t`` starting at the step onset."""
    t = np.asarray(t, dtype=float) - float(t[0])
    y = np.asarray(y_norm, dtype=float)
    if t.size < 3:
        raise ValueError("step fit needs at least 3 samples")
    y_peak = float(np.max(y))
    if y_peak <= 1.0 + 1e-3:                     # no overshoot -> overdamped/critical
        return fit_overdamped(t, y)
    mp = y_peak - 1.0                            # relative overshoot (doc §4.2)
    t_peak = float(t[int(np.argmax(y))])
    ln_mp = math.log(mp)
    zeta = -ln_mp / math.sqrt(math.pi ** 2 + ln_mp ** 2)
    zeta = min(max(zeta, 1e-3), 0.999)
    wn = math.pi / (t_peak * math.sqrt(1.0 - zeta * zeta)) if t_peak > 0.0 else 0.0
    return wn, zeta, r_squared(y, second_order_step(wn, zeta, t))


# --------------------------------------------------------------------------- #
# chirp / FRF fit (doc §4.3)
# --------------------------------------------------------------------------- #
def _resample_uniform(t, x, y, fs):
    t = np.asarray(t, dtype=float)
    grid = np.arange(t[0], t[-1], 1.0 / fs)
    return (
        np.interp(grid, t, np.asarray(x, dtype=float)),
        np.interp(grid, t, np.asarray(y, dtype=float)),
    )


def _phase_slope_delay(w: np.ndarray, H: np.ndarray) -> float:
    """Estimate pure delay from the slope of unwrapped phase vs omega (>=0)."""
    if w.size < 2:
        return 0.0
    phase = np.unwrap(np.angle(H))
    slope = np.polyfit(w, phase, 1)[0]           # phase ~ -L*w  -> slope = -L
    return float(max(0.0, -slope))


def _bandwidth_3db(f: np.ndarray, H: np.ndarray) -> float:
    mag = np.abs(H)
    if mag.size == 0 or mag[0] <= EPS:
        return float(f[np.argmax(mag)]) if mag.size else 1.0
    thr = mag[0] / math.sqrt(2.0)
    below = np.where(mag <= thr)[0]
    return float(f[below[0]]) if below.size else float(f[-1])


def fit_chirp(
    t: Sequence[float],
    q_cmd: Sequence[float],
    q: Sequence[float],
    f0: float,
    f1: float,
    *,
    fs: float | None = None,
    coherence_min: float = 0.8,
    nperseg: int | None = None,
) -> tuple[float, float, float, float]:
    """Chirp FRF -> (wn, zeta, L, R^2). Fits a 2nd-order-plus-delay model on the band
    ``[f0, f1]`` where coherence exceeds ``coherence_min`` (doc §4.3)."""
    t = np.asarray(t, dtype=float)
    if fs is None:
        fs = measure_fs(t)
    qc, qq = _resample_uniform(t, q_cmd, q, fs)
    n = qc.size
    if n < 16:
        raise ValueError("chirp fit needs more samples")
    if nperseg is None:
        target = max(256, int(round(fs / max(f0, 0.05))))
        nperseg = int(min(n, target))
    noverlap = nperseg // 2

    f, suu = welch(qc, fs, nperseg=nperseg, noverlap=noverlap, detrend="constant")
    _, suy = csd(qc, qq, fs, nperseg=nperseg, noverlap=noverlap, detrend="constant")
    _, coh = coherence(qc, qq, fs, nperseg=nperseg, noverlap=noverlap)
    with np.errstate(divide="ignore", invalid="ignore"):
        H = np.where(suu > EPS, suy / suu, 0.0)      # H1 = conj(Qcmd).Q / |Qcmd|^2

    band = (f >= f0) & (f <= f1) & (coh > coherence_min) & (f > 0.0)
    if int(np.count_nonzero(band)) < 4:              # relax coherence if too sparse
        band = (f >= f0) & (f <= f1) & (f > 0.0)
    if int(np.count_nonzero(band)) < 4:
        raise ValueError("not enough excited spectral lines in [f0, f1] to fit the FRF")

    w = 2.0 * math.pi * f[band]
    Hb = H[band]

    def model(theta, omega):
        wn, zeta, L = theta
        return wn * wn / (wn * wn - omega ** 2 + 2.0 * zeta * wn * 1j * omega) * np.exp(-1j * omega * L)

    def resid(theta):
        e = model(theta, w) - Hb
        return np.concatenate([e.real, e.imag])

    wn0 = 2.0 * math.pi * _bandwidth_3db(f[band], Hb)
    L0 = min(max(_phase_slope_delay(w, Hb), 0.0), 0.099)   # keep x0 inside the L bounds
    theta0 = [max(wn0, 1e-2), 0.7, L0]
    res = least_squares(
        resid, x0=theta0, bounds=([1e-3, 1e-3, 0.0], [np.inf, 2.0, 0.1])
    )
    wn, zeta, L = float(res.x[0]), float(res.x[1]), float(res.x[2])
    r2 = frf_r_squared(Hb, model(res.x, w))
    return wn, zeta, L, r2


# --------------------------------------------------------------------------- #
# friction (doc §4.6) and reconciliation
# --------------------------------------------------------------------------- #
def fit_friction(velocities: Sequence[float], torques: Sequence[float]) -> tuple[float, float, float]:
    """Fit ``tau = Fc*sign(v) + Fv*v`` to (gravity-removed) stationary torques.

    Returns (Fc, Fv, R^2). Needs >= 2 points; both speed signs improve conditioning.
    """
    v = np.asarray(velocities, dtype=float)
    tau = np.asarray(torques, dtype=float)
    if v.size < 2:
        raise ValueError("friction fit needs at least 2 points")
    A = np.column_stack([np.sign(v), v])
    coef, *_ = np.linalg.lstsq(A, tau, rcond=None)
    fc, fv = float(coef[0]), float(coef[1])
    return fc, fv, r_squared(tau, A @ coef)


def _rel_gap(a: float, b: float) -> float:
    denom = max(abs(a), abs(b), EPS)
    return abs(a - b) / denom


def reconcile(
    wn_c: float,
    zeta_c: float,
    wn_s: float,
    zeta_s: float,
    *,
    wn_tol: float = 0.2,
    zeta_tol: float = 0.3,
) -> tuple[float, float, list[str]]:
    """Consolidate chirp (wide-band -> priority) with step; warn on disagreement (doc §4.3)."""
    warnings: list[str] = []
    if _rel_gap(wn_c, wn_s) > wn_tol:
        warnings.append(f"wn step vs chirp differ by {_rel_gap(wn_c, wn_s):.0%} (>{wn_tol:.0%})")
    if _rel_gap(zeta_c, zeta_s) > zeta_tol:
        warnings.append(f"zeta step vs chirp differ by {_rel_gap(zeta_c, zeta_s):.0%} (>{zeta_tol:.0%})")
    if warnings:
        warnings.append("check linearity / amplitude")
    return wn_c, zeta_c, warnings
