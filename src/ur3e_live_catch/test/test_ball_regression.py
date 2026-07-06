"""Ballistic regression: fit accuracy, start gate, coast, outliers, restarts.

Pure-logic tests (stdlib only). The synthetic throw uses the test_ball_node /
Isaac FirstTraining midpoint defaults so the timing constants (gate budget,
flight duration ~0.48 s, ground_z) are the deployment ones.
"""

import math
import random

import pytest

from ur3e_live_catch.ball_regression import (
    ABORTED,
    ACCEPTED,
    COASTING,
    COLLECTING,
    ENDED,
    IDLE,
    IGNORED,
    REJECTED,
    RESTARTED,
    TRACKING,
    BallRegression,
    RegressionConfig,
)

G = 9.81
P0 = (-0.4, 1.65, 0.85)   # Isaac midpoint throw (test_ball_node defaults)
V0 = (-0.05, -4.25, 0.7)
T_BASE = 100.0            # absolute stamps: the fit must not care about epoch
RATE = 30.0               # test_ball_node raw cadence
DT = 1.0 / RATE


def parabola(tau, p0=P0, v0=V0):
    return (
        p0[0] + v0[0] * tau,
        p0[1] + v0[1] * tau,
        p0[2] + v0[2] * tau - 0.5 * G * tau * tau,
    )


def feed_flight(reg, n, noise=0.0, rng=None, start=T_BASE, p0=P0, v0=V0):
    """Feed n samples at 30 Hz; returns the outcomes list."""
    outcomes = []
    for i in range(n):
        t = start + i * DT
        pos = parabola(t - start, p0, v0)
        if noise > 0.0:
            pos = tuple(c + rng.gauss(0.0, noise) for c in pos)
        outcomes.append(reg.add_sample(t, pos))
    return outcomes


def test_clean_parabola_recovers_exact_coefficients():
    reg = BallRegression()
    feed_flight(reg, 6)
    assert reg.state == TRACKING
    fit = reg.fit
    assert fit.p0 == pytest.approx(P0, abs=1e-9)
    assert fit.v0 == pytest.approx(V0, abs=1e-9)
    t = T_BASE + 0.3
    est = reg.step(t)
    assert est.valid
    assert est.position == pytest.approx(parabola(0.3), abs=1e-9)


def test_velocity_is_fit_derivative():
    reg = BallRegression()
    feed_flight(reg, 6)
    t = T_BASE + 0.25
    est = reg.step(t)
    assert est.velocity == pytest.approx(
        (V0[0], V0[1], V0[2] - G * 0.25), abs=1e-9
    )


def test_start_gate_needs_min_support():
    reg = BallRegression()  # min_samples=4, min_span_s=0.06
    feed_flight(reg, 3)     # span 0.067 s but only 3 samples
    assert reg.state == COLLECTING
    assert reg.step(T_BASE + 2 * DT).valid is False
    reg.add_sample(T_BASE + 3 * DT, parabola(3 * DT))
    assert reg.state == TRACKING
    # Pop budget: first valid estimate within (min_samples-1)/rate + one tick.
    pop_latency = 3 * DT + 1.0 / 60.0
    est = reg.step(T_BASE + pop_latency)
    assert est.valid
    assert pop_latency <= 0.12  # ~117 ms at the 30 Hz test cadence


@pytest.mark.parametrize("seed", range(10))
def test_noisy_parabola_pops_and_stays_accurate(seed):
    rng = random.Random(seed)
    reg = BallRegression()
    feed_flight(reg, 12, noise=0.02, rng=rng)  # 0.37 s of support
    # Also guards the ballistic monitor: genuine free fall must never abort.
    assert reg.state == TRACKING
    fit = reg.fit
    assert fit.rms <= 0.04  # <= 2 sigma
    tau = 11 * DT
    est = reg.step(T_BASE + tau)
    assert est.valid
    assert est.position == pytest.approx(parabola(tau), abs=0.04)
    v_true = (V0[0], V0[1], V0[2] - G * tau)
    err = math.dist(est.velocity, v_true)
    assert err < 0.8


def test_outlier_is_rejected_and_fit_unchanged():
    reg = BallRegression()
    feed_flight(reg, 8)
    before = reg.step(T_BASE + 0.3).position
    tau = 8 * DT
    bad = tuple(c + 0.5 for c in parabola(tau))
    assert reg.add_sample(T_BASE + tau, bad) == REJECTED
    assert reg.state == TRACKING
    after = reg.step(T_BASE + 0.3).position
    assert after == pytest.approx(before, abs=1e-9)


def test_dropout_coasts_on_frozen_fit_then_ends():
    # max_coast_s shortened so the coast timeout fires while the predicted
    # ball is still well above ground (isolates it from the ground rule).
    reg = BallRegression(RegressionConfig(max_coast_s=0.15))
    feed_flight(reg, 4)  # last sample at tau=0.1
    # Starved past coast_after_s=0.10 -> COASTING, prediction continues.
    est = reg.step(T_BASE + 0.25)
    assert reg.state == COASTING
    assert est.valid
    assert est.position == pytest.approx(parabola(0.25), abs=1e-9)
    # Still coasting before max_coast_s elapses (z(0.38)=0.41 > ground).
    assert reg.step(T_BASE + 0.38).valid
    # Coast timeout (0.17 s > 0.15) with the ball at z(0.42)=0.28 -> ENDED.
    est = reg.step(T_BASE + 0.42)
    assert reg.state == ENDED
    assert est.valid is False
    assert est.velocity == (0.0, 0.0, 0.0)


def test_sample_after_coast_resumes_tracking():
    reg = BallRegression()
    feed_flight(reg, 4)
    reg.step(T_BASE + 0.25)
    assert reg.state == COASTING
    assert reg.add_sample(T_BASE + 0.3, parabola(0.3)) == ACCEPTED
    assert reg.state == TRACKING


def test_ground_termination_isaac_parity():
    reg = BallRegression()
    feed_flight(reg, 12)
    # Analytic flight grounds (z<0.05) at tau ~ 0.481 s.
    est = reg.step(T_BASE + 0.49)
    assert reg.state == ENDED
    assert est.valid is False
    # Refractory: floor bounces are ignored, then the node returns to IDLE.
    assert reg.add_sample(T_BASE + 0.55, (0.0, 0.5, 0.06)) == IGNORED
    reg.step(T_BASE + 0.49 + 0.31)
    assert reg.state == IDLE


def test_new_throw_restarts_without_contamination():
    reg = BallRegression()
    feed_flight(reg, 5)  # flight A tracked
    assert reg.state == TRACKING
    p0b, v0b = (0.3, 1.9, 1.0), (0.2, -4.0, 0.5)
    start_b = T_BASE + 0.2
    outcomes = feed_flight(reg, 4, start=start_b, p0=p0b, v0=v0b)
    assert outcomes[:3] == [REJECTED] * 3
    assert outcomes[3] == RESTARTED
    # The reject streak seeds the new flight; 4 clean samples re-pass the gate.
    assert reg.state == TRACKING
    tau = 0.15
    est = reg.step(start_b + tau)
    assert est.position == pytest.approx(parabola(tau, p0b, v0b), abs=1e-9)
    assert est.velocity[1] == pytest.approx(v0b[1], abs=1e-9)


def test_static_cluster_never_pops():
    reg = BallRegression()
    for i in range(17):  # > 0.5 s of a static spurious cluster
        reg.add_sample(T_BASE + i * DT, (0.5, 1.5, 0.5))
        # The horizontal-speed gate blocks the pop the whole time, and the
        # ballistic monitor aborts the track once the span is long enough
        # (constant height is not free fall). Later samples may re-seed a new
        # collection after the refractory, but nothing ever becomes valid.
        assert reg.step(T_BASE + i * DT).valid is False
    assert reg.state != TRACKING
    assert reg.last_flight_summary["reason"] == "non_ballistic"


def test_collect_timeout_resets_to_idle():
    reg = BallRegression()
    reg.add_sample(T_BASE, parabola(0.0))
    assert reg.state == COLLECTING
    reg.step(T_BASE + 0.35)  # > collect_timeout_s=0.3
    assert reg.state == IDLE


def test_confidence_decays_while_coasting():
    reg = BallRegression()
    feed_flight(reg, 4)
    c_track = reg.step(T_BASE + 0.15).confidence
    assert c_track > 0.0
    c1 = reg.step(T_BASE + 0.25).confidence  # coasting starts
    c2 = reg.step(T_BASE + 0.40).confidence
    assert reg.state == COASTING
    assert c1 > c2 > 0.0


def test_non_finite_and_stale_samples_ignored():
    reg = BallRegression()
    assert reg.add_sample(T_BASE, (float("nan"), 0.0, 0.0)) == IGNORED
    assert reg.state == IDLE
    feed_flight(reg, 4)
    assert reg.add_sample(T_BASE - 1.0, parabola(0.0)) == IGNORED


def test_config_is_deployment_tunable():
    cfg = RegressionConfig(min_samples=6, min_span_s=0.1)
    reg = BallRegression(cfg)
    feed_flight(reg, 5)
    assert reg.state == COLLECTING  # stricter gate honored


def test_hand_drift_aborts_as_non_ballistic():
    # A hand crossing the FOV at 1 m/s, constant height: passes the horizontal
    # speed gate and pops, but is NOT in free fall — the ballistic consistency
    # monitor must kill the flight shortly after.
    reg = BallRegression()
    outcomes = []
    for i in range(10):
        tau = i * DT
        outcomes.append(reg.add_sample(T_BASE + tau, (0.5 + 1.0 * tau, 1.5, 0.5)))
    assert ABORTED in outcomes
    assert reg.state == ENDED
    assert reg.last_flight_summary["reason"] == "non_ballistic"
    assert reg.step(T_BASE + 10 * DT).valid is False
    # Refractory expires back to IDLE like any ended flight.
    reg.step(T_BASE + 10 * DT + 0.35)
    assert reg.state == IDLE


def test_pop_corridor_blocks_flights_born_near_robot():
    # A perfectly ballistic track that stays within min_pop_distance_m of the
    # base must never start a flight (hands/deflections next to the arm).
    reg = BallRegression()
    p0, v0 = (0.2, 0.1, 0.3), (1.0, 0.0, 0.5)
    for i in range(7):  # tau <= 0.2 s: distance stays under 0.5 m
        tau = i * DT
        reg.add_sample(T_BASE + tau, parabola(tau, p0, v0))
        assert reg.step(T_BASE + tau).valid is False
    assert reg.state == COLLECTING


def test_decimation_ignores_khz_bursts():
    reg = BallRegression()
    assert reg.add_sample(T_BASE, parabola(0.0)) == ACCEPTED
    assert reg.add_sample(T_BASE + 0.001, parabola(0.001)) == IGNORED
    assert reg.add_sample(T_BASE + 0.005, parabola(0.005)) == ACCEPTED


def test_flight_summary_reports_ground_end():
    reg = BallRegression()
    feed_flight(reg, 12)
    reg.step(T_BASE + 0.49)  # grounds the flight
    summary = reg.last_flight_summary
    assert summary["reason"] == "ground"
    assert summary["n_accepted"] == 12
    assert summary["pop_latency_s"] == pytest.approx(3 * DT)
    assert summary["pop_position"][1] > 1.2  # popped inside the Isaac corridor
    assert reg.flights_ended == 1
