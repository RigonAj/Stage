"""Measurement purity (min_input_confidence) and anisotropic depth model.

Plan items 1.1/2.1 of docs/Robot_Control/plan_amelioration_perception_transmission.md:
coasted (extrapolated) producer samples must never feed the fit, and realistic
Trace depth noise (1/trail-width, ~10x the lateral noise) must neither block
the start gate nor drag the fit when the camera geometry is known.
"""

import math
import random

from ur3e_live_catch.ball_regression import (
    ACCEPTED,
    BallRegression,
    IGNORED,
    REJECTED,
    RegressionConfig,
    TRACKING,
)

G = 9.81
P0 = (-0.4, 1.65, 0.85)
V0 = (-0.05, -4.25, 0.7)
CAMERA = (0.0, 2.5, 0.8)
T_START = 100.0  # arbitrary absolute clock origin


def _truth(t: float):
    return (
        P0[0] + V0[0] * t,
        P0[1] + V0[1] * t,
        P0[2] + V0[2] * t - 0.5 * G * t * t,
    )


def _ray_from_camera(pos):
    d = tuple(pos[i] - CAMERA[i] for i in range(3))
    n = math.sqrt(sum(v * v for v in d))
    return tuple(v / n for v in d)


def _perp(ray):
    """Any unit vector perpendicular to the ray (lateral noise direction)."""
    axis = (0.0, 0.0, 1.0) if abs(ray[2]) < 0.9 else (1.0, 0.0, 0.0)
    cx = ray[1] * axis[2] - ray[2] * axis[1]
    cy = ray[2] * axis[0] - ray[0] * axis[2]
    cz = ray[0] * axis[1] - ray[1] * axis[0]
    n = math.sqrt(cx * cx + cy * cy + cz * cz)
    return (cx / n, cy / n, cz / n)


def _noisy_flight(rng, depth_std, lateral_std, rate_hz=120.0, duration=0.30):
    """(stamp, noisy position) samples with noise expressed along/across the ray."""
    samples = []
    steps = int(duration * rate_hz)
    for i in range(steps + 1):
        t = i / rate_hz
        pos = _truth(t)
        ray = _ray_from_camera(pos)
        lat = _perp(ray)
        d = rng.gauss(0.0, depth_std)
        s = rng.gauss(0.0, lateral_std)
        noisy = tuple(pos[k] + d * ray[k] + s * lat[k] for k in range(3))
        samples.append((T_START + t, noisy))
    return samples


# --- 1.1 measurement purity ----------------------------------------------------


def test_low_confidence_sample_is_ignored():
    reg = BallRegression(RegressionConfig())
    assert reg.add_sample(T_START, _truth(0.0), confidence=0.7) == IGNORED
    assert reg.state == "idle"


def test_full_confidence_sample_is_accepted():
    reg = BallRegression(RegressionConfig())
    assert reg.add_sample(T_START, _truth(0.0), confidence=1.0) == ACCEPTED
    assert reg.state == "collecting"


def test_confidence_threshold_is_configurable():
    reg = BallRegression(RegressionConfig(min_input_confidence=0.5))
    assert reg.add_sample(T_START, _truth(0.0), confidence=0.7) == ACCEPTED


def test_default_confidence_keeps_legacy_callers_working():
    reg = BallRegression(RegressionConfig())
    assert reg.add_sample(T_START, _truth(0.0)) == ACCEPTED


def test_coast_burst_never_starts_a_flight():
    # A tracker coasting with decaying confidence must leave the state machine
    # idle even over a long, self-consistent burst.
    reg = BallRegression(RegressionConfig())
    for i in range(30):
        t = i / 60.0
        conf = max(0.0, 1.0 - t / 0.5) - 1e-3  # strictly below 1.0
        assert reg.add_sample(T_START + t, _truth(t), confidence=conf) == IGNORED
    assert reg.state == "idle"
    assert reg.fit is None


# --- 2.1 anisotropic depth model -------------------------------------------------


def _run_flight(reg, samples, with_camera=True):
    outcomes = []
    for t, pos in samples:
        outcomes.append(
            reg.add_sample(t, pos, camera_pos_base=CAMERA if with_camera else None)
        )
        reg.step(t)  # advance time-driven transitions alongside the feed
    return outcomes


def test_scale_one_with_camera_matches_isotropic_fit():
    # depth_sigma_scale=1.0 must reproduce the legacy behavior bit-for-bit,
    # camera provided or not.
    samples = _noisy_flight(random.Random(7), depth_std=0.01, lateral_std=0.002)
    reg_ray = BallRegression(RegressionConfig(depth_sigma_scale=1.0))
    reg_iso = BallRegression(RegressionConfig(depth_sigma_scale=1.0))
    _run_flight(reg_ray, samples, with_camera=True)
    _run_flight(reg_iso, samples, with_camera=False)
    assert reg_ray.fit is not None and reg_iso.fit is not None
    for a, b in zip(reg_ray.fit.v0, reg_iso.fit.v0):
        assert a == b
    assert reg_ray.fit.rms == reg_iso.fit.rms


def test_isotropic_gating_is_fragile_under_depth_noise():
    # 6 cm depth noise (realistic Trace 1/width error) makes the ISOTROPIC
    # pipeline fragile: depending on the noise draw the rms start gate never
    # opens (no pop) or in-flight samples get rejected as outliers. The scaled
    # metric removes both failure modes on the same draws (next test pins the
    # quality; this one pins the robustness contrast).
    for seed in (3, 7, 11, 42):
        samples = _noisy_flight(random.Random(seed), depth_std=0.06, lateral_std=0.002)

        iso = BallRegression(RegressionConfig(depth_sigma_scale=1.0))
        iso_outcomes = _run_flight(iso, samples)
        iso_fragile = iso.state != TRACKING or REJECTED in iso_outcomes
        assert iso_fragile, f"seed {seed}: isotropic unexpectedly clean"

        aniso = BallRegression(RegressionConfig(depth_sigma_scale=8.0))
        aniso_outcomes = _run_flight(aniso, samples)
        assert aniso.state == TRACKING, f"seed {seed}: anisotropic did not pop"
        assert REJECTED not in aniso_outcomes, f"seed {seed}: anisotropic rejected samples"


def test_depth_scale_recovers_pop_and_velocity_under_depth_noise():
    samples = _noisy_flight(random.Random(3), depth_std=0.06, lateral_std=0.002)
    reg = BallRegression(RegressionConfig(depth_sigma_scale=8.0))
    outcomes = _run_flight(reg, samples)
    assert reg.state == TRACKING
    # Depth wobble must not read as outliers: essentially everything accepted.
    assert outcomes.count(ACCEPTED) >= 0.9 * len(outcomes)
    fit = reg.fit
    assert fit is not None
    # rms now reads in lateral-equivalent metres: well under the start gate.
    assert fit.rms < RegressionConfig().max_rms_m
    # The approach velocity (vy, the interception-critical component) is
    # recovered despite the depth axis being the noisy one.
    assert abs(fit.v0[1] - V0[1]) < 0.5
    assert abs(fit.v0[0] - V0[0]) < 0.5
    est = reg.step(samples[-1][0])
    assert est.valid


def test_camera_at_sample_position_falls_back_to_isotropic():
    reg = BallRegression(RegressionConfig(depth_sigma_scale=8.0))
    # Degenerate ray (|pos - camera| ~ 0) must not crash nor produce NaNs.
    assert reg.add_sample(T_START, CAMERA, camera_pos_base=CAMERA) == ACCEPTED


# --- runtime lead tuning (latency compensation, plan 2.4 provisional) ------------


def test_set_lead_time_shifts_evaluation_mid_flight():
    import pytest

    samples = _noisy_flight(random.Random(5), depth_std=0.0, lateral_std=0.0)
    reg = BallRegression(RegressionConfig(lead_time_s=0.0))
    _run_flight(reg, samples, with_camera=False)
    assert reg.state == TRACKING
    t_mid = T_START + 0.15  # mid-flight: t_mid + lead stays above the ground

    no_lead = reg.step(t_mid)
    reg.set_lead_time(0.2)
    with_lead = reg.step(t_mid)
    assert no_lead.valid and with_lead.valid
    # The lead only moves the evaluation point along the SAME fit.
    fit = reg.fit
    for got, want in zip(with_lead.position, fit.position(t_mid + 0.2)):
        assert abs(got - want) < 1e-9
    # On a clean parabola the led position matches the true future state.
    truth = _truth(0.15 + 0.2)
    for got, want in zip(with_lead.position, truth):
        assert abs(got - want) < 1e-6

    for bad in (-0.1, 1.5, float("nan")):
        with pytest.raises(ValueError):
            reg.set_lead_time(bad)
    assert reg.lead_time_s == 0.2  # rejected values leave the lead untouched


def test_lead_terminates_flight_when_predicted_ball_grounds():
    # Documented side effect of a large lead: the ground check runs at the
    # EVALUATION time, so a 0.2 s lead ends the flight ~0.2 s before the real
    # ball grounds. Operators tuning lead_time_s must know the trade-off.
    samples = _noisy_flight(random.Random(5), depth_std=0.0, lateral_std=0.0)
    reg = BallRegression(RegressionConfig(lead_time_s=0.2))
    _run_flight(reg, samples, with_camera=False)
    t_end = samples[-1][0]  # truth grounds at ~t0+0.49 s; eval sees it at ~0.29 s
    est = reg.step(t_end)
    assert not est.valid
    assert reg.state == "ended"
