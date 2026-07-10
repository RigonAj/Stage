"""Ballistic regression over raw ball detections (Isaac "pop" parity).

Pure logic, no rclpy / no numpy (package rule, see ``ball_frame.py``): the node
wrapper feeds base_link samples in and reads a publishable estimate out.

Purpose: make the real ball look to the policy like an Isaac spawned ball.
Isaac gives the policy a clean position AND velocity from the spawn tick; the
raw tracker gives bursty, noisy single-frame detections. This module fits the
ballistic model with FIXED gravity in base_link (z-up),

    x(tau) = x0 + vx*tau
    y(tau) = y0 + vy*tau
    z(tau) = z0 + vz*tau - 0.5*g*tau**2      tau = t - t0 (first accepted sample)

so with z' = z + 0.5*g*tau**2 every axis is a closed-form weighted linear fit
(6 free parameters total). Robustness comes from IRLS with a Cauchy weight on
the joint 3-D residual times a mild recency weight, mirroring the C++
``WeightedLinearFit`` used for GUI diagnostics in Ball_Tracking_Cpp.

State machine (all timeouts in the config):

    IDLE -> COLLECTING   first finite sample
    COLLECTING -> TRACKING  start gate: enough support (min_samples, min_span_s),
                            fit residual below max_rms_m, plausible speed,
                            above ground -- nothing valid is published before this
    COLLECTING -> IDLE   collect_timeout_s without samples, or span exceeded
                         without ever passing the gate (stray cluster)
    TRACKING -> COASTING no accepted sample for coast_after_s (dropout) or the
                         fitted ball is within freeze_distance_m of the base
                         (near-robot occlusion): fit frozen, prediction continues
    COASTING -> TRACKING a sample passes the acceptance gate again
    TRACKING/COASTING -> COLLECTING  reject_streak_n consecutive rejected but
                         mutually consistent samples = a NEW throw: restart
    TRACKING/COASTING -> ENDED  predicted z below ground_z_m (Isaac
                         ball_on_ground parity), flight/coast timeout
    ENDED -> IDLE        refractory_s elapsed (floor bounces ignored)

The node publishes valid=False heartbeats outside TRACKING/COASTING, matching
the trigger-mode ``test_ball_node`` contract, and evaluates the fit at "now"
(+ optional lead) — deliberate latency compensation: the estimate is the state
of the ball at evaluation time, not at measurement time.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Optional, Sequence

Vec3 = tuple[float, float, float]

# Regression states (exported for the node log / tests).
IDLE = "idle"
COLLECTING = "collecting"
TRACKING = "tracking"
COASTING = "coasting"
ENDED = "ended"

# add_sample outcomes.
ACCEPTED = "accepted"
REJECTED = "rejected"
RESTARTED = "restarted"
IGNORED = "ignored"
ABORTED = "aborted"  # flight invalidated by the ballistic consistency monitor


@dataclass
class RegressionConfig:
    gravity_m_s2: float = 9.81
    # Measurement purity (plan 1.1): drop producer samples below this
    # confidence. The tracker publishes 1.0 on real fits and a decaying value
    # while coasting (extrapolated, model-generated points) — those must never
    # feed the fit as measurements.
    min_input_confidence: float = 1.0
    # Anisotropic noise model (plan 2.1): Trace depth (trail width -> 1/width)
    # is far noisier than the lateral image position. When the producer's
    # camera position is known, per-sample residuals and per-axis weights are
    # scaled so a deviation ALONG the camera->ball ray counts depth_sigma_scale
    # times less than a lateral one (sigma_depth = scale * sigma_lateral).
    # 1.0 = isotropic (previous behavior); the fit rms and every rms-based gate
    # (max_rms_m, gate_k) then read in lateral-equivalent metres, so realistic
    # depth noise no longer blocks the start gate or triggers restarts.
    depth_sigma_scale: float = 1.0
    # fit
    max_samples: int = 240            # full-flight buffer (0.3-0.5 s flights)
    irls_iterations: int = 2
    sigma_floor_m: float = 0.01       # keeps Cauchy weights sane on clean data
    recency_lambda: float = 1.0       # 0 disables recency weighting
    # per-sample acceptance (TRACKING/COASTING)
    gate_floor_m: float = 0.10
    gate_k: float = 3.0
    reorder_tolerance_s: float = 0.01
    # Decimation: the real tracker can emit trace samples at event-batch rate
    # (kHz bursts); overlapping windows add no information at that cadence and
    # a per-sample IRLS refit would not keep up in Python.
    min_sample_interval_s: float = 0.003
    # publish-start gate ("enough support")
    min_samples: int = 4
    min_span_s: float = 0.06
    max_rms_m: float = 0.035
    # HORIZONTAL speed floor: the gravity linearization can fake a vertical
    # velocity (~g*span/2) for a static cluster, but never a horizontal one.
    min_speed_m_s: float = 0.5
    max_speed_m_s: float = 10.0
    require_approach: bool = False    # optionally demand vy < 0 (toward robot)
    ground_z_m: float = 0.05          # Isaac ball_on_ground parity
    # Pop corridor: a flight may only START while the fitted ball is at least
    # this far from base_link origin. Real throws always begin far away; only
    # spurious clusters (hands, arm reflections, hoop-rim deflections) are
    # born near the robot — and a near-robot pop moves the arm toward them.
    min_pop_distance_m: float = 0.6
    # Ballistic consistency monitor: once the support span is long enough,
    # compare the fixed-gravity z fit against a FREE quadratic z fit. A ball in
    # free fall gives a curvature of -g/2 (ratio ~1); a hand or drifting
    # cluster does not, so the fixed model misses while the free one does not.
    ballistic_check_span_s: float = 0.15
    ballistic_rms_ratio: float = 2.0
    ballistic_rms_floor_m: float = 0.015  # ignore the ratio while both fits are this good
    # state-machine timing
    collect_timeout_s: float = 0.3
    max_collect_span_s: float = 0.5
    coast_after_s: float = 0.10
    freeze_distance_m: float = 0.35
    reject_streak_n: int = 4
    max_coast_s: float = 0.25
    coast_conf_tau_s: float = 0.15
    max_flight_s: float = 1.0
    refractory_s: float = 0.3
    lead_time_s: float = 0.0


@dataclass(frozen=True)
class BallisticFit:
    """Fitted flight: evaluate anywhere in absolute time."""

    t0: float
    p0: Vec3
    v0: Vec3
    g: float
    rms: float
    n: int
    span: float

    def position(self, t_abs: float) -> Vec3:
        tau = t_abs - self.t0
        return (
            self.p0[0] + self.v0[0] * tau,
            self.p0[1] + self.v0[1] * tau,
            self.p0[2] + self.v0[2] * tau - 0.5 * self.g * tau * tau,
        )

    def velocity(self, t_abs: float) -> Vec3:
        tau = t_abs - self.t0
        return (self.v0[0], self.v0[1], self.v0[2] - self.g * tau)

    def speed(self, t_abs: float) -> float:
        vx, vy, vz = self.velocity(t_abs)
        return math.sqrt(vx * vx + vy * vy + vz * vz)


@dataclass(frozen=True)
class Estimate:
    """Publishable output of ``BallRegression.step``."""

    position: Vec3
    velocity: Vec3
    valid: bool
    confidence: float
    state: str


def _ols_quad(taus: Sequence[float],
              values: Sequence[float]) -> Optional[tuple[float, float, float]]:
    """Unweighted least squares for value = a*tau^2 + b*tau + c."""
    n = len(taus)
    if n < 3:
        return None
    s1 = s2 = s3 = s4 = 0.0
    sy = sty = st2y = 0.0
    for tau, y in zip(taus, values):
        t2 = tau * tau
        s1 += tau
        s2 += t2
        s3 += t2 * tau
        s4 += t2 * t2
        sy += y
        sty += tau * y
        st2y += t2 * y
    # Normal equations [[s4,s3,s2],[s3,s2,s1],[s2,s1,n]] . [a,b,c] = [st2y,sty,sy]
    det = (s4 * (s2 * n - s1 * s1)
           - s3 * (s3 * n - s1 * s2)
           + s2 * (s3 * s1 - s2 * s2))
    if abs(det) < 1e-12:
        return None
    a = (st2y * (s2 * n - s1 * s1)
         - s3 * (sty * n - sy * s1)
         + s2 * (sty * s1 - sy * s2)) / det
    b = (s4 * (sty * n - sy * s1)
         - st2y * (s3 * n - s1 * s2)
         + s2 * (s3 * sy - sty * s2)) / det
    c = (s4 * (s2 * sy - s1 * sty)
         - s3 * (s3 * sy - s2 * sty)
         + st2y * (s3 * s1 - s2 * s2)) / det
    return a, b, c


def _wls_line(taus: Sequence[float], values: Sequence[float],
              weights: Sequence[float]) -> Optional[tuple[float, float]]:
    """Weighted least squares for value = p + v*tau; returns (p, v)."""
    sw = st = sy = stt = sty = 0.0
    for tau, y, w in zip(taus, values, weights):
        sw += w
        st += w * tau
        sy += w * y
        stt += w * tau * tau
        sty += w * tau * y
    d = sw * stt - st * st
    if abs(d) < 1e-12:
        return None
    v = (sw * sty - st * sy) / d
    p = (sy - v * st) / sw if sw > 0.0 else 0.0
    return p, v


class BallRegression:
    """Rolling ballistic fit + flight state machine (see module docstring)."""

    def __init__(self, config: Optional[RegressionConfig] = None) -> None:
        self._cfg = config or RegressionConfig()
        self._state = IDLE
        # (stamp, position, camera->ball unit ray or None) — the ray carries the
        # per-sample depth direction for the anisotropic noise model.
        self._samples: list[tuple[float, Vec3, Optional[Vec3]]] = []
        self._fit: Optional[BallisticFit] = None
        self._track_started_t: Optional[float] = None
        self._last_sample_t: Optional[float] = None    # last stored sample stamp
        self._last_seen_t: Optional[float] = None      # last processed stamp (decimation)
        self._last_accepted_t: Optional[float] = None  # last fit-updating stamp
        self._freeze_t: Optional[float] = None         # step-clock coast start
        self._idle_after: Optional[float] = None       # step-clock refractory end
        self._reject_streak: list[tuple[float, Vec3, Optional[Vec3]]] = []
        # per-flight bookkeeping for the end-of-flight summary
        self._first_sample_t: Optional[float] = None
        self._pop_t: Optional[float] = None
        self._pop_position: Optional[Vec3] = None
        self._n_accepted = 0
        self._n_rejected = 0
        # survives reset(): describes the PREVIOUS flight
        self._last_summary: Optional[dict] = None
        self._flights_ended = 0

    # --- public API -----------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def fit(self) -> Optional[BallisticFit]:
        return self._fit

    @property
    def last_flight_summary(self) -> Optional[dict]:
        """Summary of the most recently ended flight (survives reset)."""
        return self._last_summary

    @property
    def flights_ended(self) -> int:
        return self._flights_ended

    @property
    def lead_time_s(self) -> float:
        return self._cfg.lead_time_s

    def set_lead_time(self, lead_s: float) -> None:
        """Runtime lead tuning (latency compensation, operator-adjustable).

        Safe mid-flight: the lead only shifts the evaluation time of ``step``,
        it never touches samples, fit or state machine. Bounded to [0, 1] s —
        beyond that the extrapolation exceeds any plausible flight remainder.
        """
        lead = float(lead_s)
        if not math.isfinite(lead) or not (0.0 <= lead <= 1.0):
            raise ValueError(f"lead_time_s must be in [0, 1] s, got {lead_s!r}")
        self._cfg.lead_time_s = lead

    def reset(self) -> None:
        self._state = IDLE
        self._samples.clear()
        self._fit = None
        self._track_started_t = None
        self._last_sample_t = None
        self._last_seen_t = None
        self._last_accepted_t = None
        self._freeze_t = None
        self._idle_after = None
        self._reject_streak.clear()
        self._first_sample_t = None
        self._pop_t = None
        self._pop_position = None
        self._n_accepted = 0
        self._n_rejected = 0

    def add_sample(self, t_s: float, pos_base: Sequence[float],
                   confidence: float = 1.0,
                   camera_pos_base: Optional[Sequence[float]] = None) -> str:
        """Feed one raw detection (base_link, meters, producer stamp seconds).

        ``confidence`` is the producer's value; anything below
        ``min_input_confidence`` is IGNORED (coasted/extrapolated points are
        not measurements). ``camera_pos_base`` (camera origin in base_link)
        enables the anisotropic depth model; None keeps the sample isotropic.
        """
        pos = (float(pos_base[0]), float(pos_base[1]), float(pos_base[2]))
        if not all(math.isfinite(v) for v in (t_s, *pos)):
            return IGNORED
        if confidence < self._cfg.min_input_confidence - 1e-6:
            return IGNORED  # model-generated (coast) point, not a measurement
        ray = self._camera_ray(pos, camera_pos_base)
        if self._state == ENDED:
            return IGNORED  # refractory: floor bounces / lingering clusters
        if self._last_seen_t is not None and \
                t_s - self._last_seen_t < self._cfg.min_sample_interval_s:
            return IGNORED  # decimation of kHz-rate raw sources
        if self._last_sample_t is not None and \
                t_s < self._last_sample_t - self._cfg.reorder_tolerance_s:
            return IGNORED  # stale out-of-order stamp
        self._last_seen_t = t_s

        if self._state == IDLE:
            self._start_collecting([(t_s, pos, ray)])
            return ACCEPTED

        if self._state == COLLECTING:
            self._store(t_s, pos, ray)
            self._n_accepted += 1
            self._refit()
            if not self._ballistic_consistency_ok():
                return self._abort_flight(t_s)
            self._try_start_tracking()
            return ACCEPTED

        # TRACKING / COASTING: gate against the current (possibly frozen) fit,
        # in the scaled metric (a depth-direction deviation counts less).
        assert self._fit is not None
        residual = self._scaled_dist(pos, self._fit.position(t_s), ray)
        threshold = max(self._cfg.gate_floor_m, self._cfg.gate_k * self._fit.rms)
        if residual <= threshold:
            self._reject_streak.clear()
            self._store(t_s, pos, ray)
            self._n_accepted += 1
            self._refit()
            self._last_accepted_t = t_s
            if not self._ballistic_consistency_ok():
                return self._abort_flight(t_s)
            if self._state == COASTING:
                self._state = TRACKING  # occlusion/dropout gap ended
                self._freeze_t = None
            return ACCEPTED

        self._reject_streak.append((t_s, pos, ray))
        self._n_rejected += 1
        if len(self._reject_streak) >= self._cfg.reject_streak_n and \
                self._streak_is_consistent():
            self._finalize_flight("restarted")
            seed = list(self._reject_streak)
            self._start_collecting(seed)  # new throw replaces the old flight
            self._refit()
            self._try_start_tracking()
            return RESTARTED
        return REJECTED

    def step(self, now_s: float) -> Estimate:
        """Advance time-driven transitions and return the publishable estimate.

        ``now_s`` must share the clock of the sample stamps (the node guards
        skew with max_stamp_age_s). Evaluation happens at now + lead_time_s.
        """
        t_eval = now_s + self._cfg.lead_time_s

        if self._state == ENDED:
            if self._idle_after is not None and now_s >= self._idle_after:
                self.reset()
            return self._invalid()

        if self._state == COLLECTING:
            if self._collect_expired(now_s):
                self._finalize_flight("collect_timeout")
                self.reset()
            return self._invalid()

        if self._state == TRACKING:
            assert self._fit is not None
            starved = (self._last_accepted_t is not None
                       and now_s - self._last_accepted_t > self._cfg.coast_after_s)
            near_robot = _norm(self._fit.position(t_eval)) < self._cfg.freeze_distance_m
            if starved or near_robot:
                self._state = COASTING
                self._freeze_t = now_s

        if self._state in (TRACKING, COASTING):
            assert self._fit is not None
            end_reason = self._flight_over(now_s, t_eval)
            if end_reason is not None:
                self._finalize_flight(end_reason)
                self._state = ENDED
                self._idle_after = now_s + self._cfg.refractory_s
                return self._invalid()
            return self._valid_estimate(now_s, t_eval)

        return self._invalid()  # IDLE

    # --- internals -------------------------------------------------------------

    @staticmethod
    def _camera_ray(pos: Vec3, camera_pos_base: Optional[Sequence[float]]) -> Optional[Vec3]:
        """Unit camera->ball direction in base_link (the depth axis), or None."""
        if camera_pos_base is None:
            return None
        d = (pos[0] - float(camera_pos_base[0]),
             pos[1] - float(camera_pos_base[1]),
             pos[2] - float(camera_pos_base[2]))
        n = _norm(d)
        if not math.isfinite(n) or n < 1e-6:
            return None
        return (d[0] / n, d[1] / n, d[2] / n)

    def _axis_variance_factors(self, ray: Optional[Vec3]) -> Vec3:
        """Diagonal of the per-sample noise covariance in sigma_lateral^2 units.

        Full covariance is sigma_lat^2 * (I + (scale^2 - 1) * ray*ray^T); the
        per-axis solver keeps only the diagonal (exact when the ray aligns with
        a base axis, first-order otherwise).
        """
        s2 = self._cfg.depth_sigma_scale * self._cfg.depth_sigma_scale
        if ray is None or s2 <= 1.0:
            return (1.0, 1.0, 1.0)
        e = s2 - 1.0
        return (1.0 + e * ray[0] * ray[0],
                1.0 + e * ray[1] * ray[1],
                1.0 + e * ray[2] * ray[2])

    def _scaled_dist(self, pos: Sequence[float], pred: Sequence[float],
                     ray: Optional[Vec3]) -> float:
        """Residual distance in lateral-equivalent metres (Mahalanobis, diagonal)."""
        var = self._axis_variance_factors(ray)
        return math.sqrt((pos[0] - pred[0]) ** 2 / var[0]
                         + (pos[1] - pred[1]) ** 2 / var[1]
                         + (pos[2] - pred[2]) ** 2 / var[2])

    def _start_collecting(self, seed: list[tuple[float, Vec3, Optional[Vec3]]]) -> None:
        self._samples = list(seed[-self._cfg.max_samples:])
        self._fit = None
        self._state = COLLECTING
        self._track_started_t = None
        self._last_sample_t = self._samples[-1][0]
        self._last_seen_t = self._samples[-1][0]
        self._last_accepted_t = self._samples[-1][0]
        self._freeze_t = None
        self._reject_streak.clear()
        self._first_sample_t = self._samples[0][0]
        self._pop_t = None
        self._pop_position = None
        self._n_accepted = len(self._samples)
        self._n_rejected = 0

    def _store(self, t_s: float, pos: Vec3, ray: Optional[Vec3]) -> None:
        self._samples.append((t_s, pos, ray))
        if len(self._samples) > self._cfg.max_samples:
            del self._samples[0]
        self._last_sample_t = t_s
        if self._state == COLLECTING:
            self._last_accepted_t = t_s

    def _refit(self) -> None:
        fit = self._fit_samples()
        if fit is not None:
            self._fit = fit

    def _fit_samples(self) -> Optional[BallisticFit]:
        samples = self._samples
        if len(samples) < 2:
            return None
        t0 = samples[0][0]
        taus = [t - t0 for t, _, _ in samples]
        span = taus[-1] - taus[0]
        g = self._cfg.gravity_m_s2
        xs = [p[0] for _, p, _ in samples]
        ys = [p[1] for _, p, _ in samples]
        # z' = z + 0.5*g*tau^2 linearizes the fixed-gravity axis.
        zs = [p[2] + 0.5 * g * tau * tau for (_, p, _), tau in zip(samples, taus)]
        var_factors = [self._axis_variance_factors(ray) for _, _, ray in samples]

        weights = self._recency_weights(taus, span)
        fit = self._solve(taus, xs, ys, zs, weights, var_factors, t0, g, span)
        if fit is None:
            return None
        for _ in range(max(0, self._cfg.irls_iterations)):
            weights = self._irls_weights(samples, taus, span, fit)
            refit = self._solve(taus, xs, ys, zs, weights, var_factors, t0, g, span)
            if refit is None:
                break
            fit = refit
        return fit

    def _solve(self, taus, xs, ys, zs, weights, var_factors, t0: float, g: float,
               span: float) -> Optional[BallisticFit]:
        # Per-axis WLS weights = common (recency*Cauchy) weight / axis variance:
        # depth-noisy directions pull each 1-D fit less (identical to the
        # previous isotropic solve when depth_sigma_scale is 1).
        wx = [w / v[0] for w, v in zip(weights, var_factors)]
        wy = [w / v[1] for w, v in zip(weights, var_factors)]
        wz = [w / v[2] for w, v in zip(weights, var_factors)]
        ax = _wls_line(taus, xs, wx)
        ay = _wls_line(taus, ys, wy)
        az = _wls_line(taus, zs, wz)
        if ax is None or ay is None or az is None:
            return None
        p0 = (ax[0], ay[0], az[0])
        v0 = (ax[1], ay[1], az[1])
        # Weighted RMS of the joint residual in the SCALED metric
        # (lateral-equivalent metres), so gate_k/max_rms_m compare consistently.
        sw = sr = 0.0
        for tau, x, y, z, w, v in zip(taus, xs, ys, zs, weights, var_factors):
            fx = p0[0] + v0[0] * tau
            fy = p0[1] + v0[1] * tau
            fz = p0[2] + v0[2] * tau  # both sides in z' space: gravity cancels
            r2 = ((x - fx) ** 2 / v[0]
                  + (y - fy) ** 2 / v[1]
                  + (z - fz) ** 2 / v[2])
            sw += w
            sr += w * r2
        rms = math.sqrt(sr / sw) if sw > 0.0 else float("inf")
        return BallisticFit(t0=t0, p0=p0, v0=v0, g=g, rms=rms,
                            n=len(taus), span=span)

    def _recency_weights(self, taus: Sequence[float], span: float) -> list[float]:
        lam = self._cfg.recency_lambda
        if lam <= 0.0 or span <= 0.0:
            return [1.0] * len(taus)
        newest = taus[-1]
        return [math.exp(-lam * (newest - tau) / span) for tau in taus]

    def _irls_weights(self, samples, taus, span, fit: BallisticFit) -> list[float]:
        # Residuals in the scaled metric: a depth-direction wobble is expected
        # noise, not an outlier, so it must not inflate the Cauchy scale nor be
        # clipped as hard as a lateral miss.
        residuals = [self._scaled_dist(p, fit.position(t), ray) for t, p, ray in samples]
        scale = max(1e-6, self._cfg.sigma_floor_m, 1.4826 * median(residuals))
        recency = self._recency_weights(taus, span)
        return [rw / (1.0 + (r / scale) ** 2) for r, rw in zip(residuals, recency)]

    def _try_start_tracking(self) -> None:
        cfg = self._cfg
        fit = self._fit
        if fit is None or len(self._samples) < cfg.min_samples:
            return
        if fit.span < cfg.min_span_s or fit.rms > cfg.max_rms_m:
            return
        t_now = self._samples[-1][0]
        vx, vy, _ = fit.velocity(t_now)
        if math.hypot(vx, vy) < cfg.min_speed_m_s:
            return  # static/vertical artifact: no real throw lacks vx/vy
        if fit.speed(t_now) > cfg.max_speed_m_s:
            return
        if cfg.require_approach and fit.velocity(t_now)[1] >= 0.0:
            return
        pos_now = fit.position(t_now)
        if pos_now[2] < cfg.ground_z_m:
            return
        if _norm(pos_now) < cfg.min_pop_distance_m:
            return  # pop corridor: flights never start next to the robot
        self._state = TRACKING
        self._track_started_t = t_now
        self._pop_t = t_now
        self._pop_position = pos_now
        self._freeze_t = None
        self._reject_streak.clear()

    def _ballistic_consistency_ok(self) -> bool:
        """Reject tracks that are not in free fall (hands, drifting clusters).

        Compares the unweighted fixed-gravity z fit against a FREE quadratic z
        fit over the same samples. A ballistic ball recovers a curvature of
        -g/2 either way (ratio ~1); anything else makes the fixed model miss
        while the free one still fits.
        """
        cfg = self._cfg
        fit = self._fit
        if fit is None or fit.span < cfg.ballistic_check_span_s:
            return True
        t0 = self._samples[0][0]
        taus = [t - t0 for t, _, _ in self._samples]
        zs = [p[2] for _, p, _ in self._samples]
        g = cfg.gravity_m_s2
        fixed = _wls_line(taus, [z + 0.5 * g * tau * tau for z, tau in zip(zs, taus)],
                          [1.0] * len(taus))
        free = _ols_quad(taus, zs)
        if fixed is None or free is None:
            return True
        rms_fixed = _rms(zs, [fixed[0] + fixed[1] * tau - 0.5 * g * tau * tau
                              for tau in taus])
        if rms_fixed <= cfg.ballistic_rms_floor_m:
            return True  # the free-fall model explains the data; nothing to test
        rms_free = _rms(zs, [free[0] * tau * tau + free[1] * tau + free[2]
                             for tau in taus])
        return rms_fixed <= cfg.ballistic_rms_ratio * max(rms_free, 1e-6)

    def _abort_flight(self, t_s: float) -> str:
        """Non-ballistic track: end the flight and hold the refractory."""
        self._finalize_flight("non_ballistic")
        self._state = ENDED
        self._idle_after = t_s + self._cfg.refractory_s
        return ABORTED

    def _finalize_flight(self, reason: str) -> None:
        fit = self._fit
        self._last_summary = {
            "reason": reason,
            "first_sample_t": self._first_sample_t,
            "pop_t": self._pop_t,
            "pop_latency_s": (None if self._pop_t is None or self._first_sample_t is None
                              else self._pop_t - self._first_sample_t),
            "pop_position": self._pop_position,
            "n_accepted": self._n_accepted,
            "n_rejected": self._n_rejected,
            "fit_n": None if fit is None else fit.n,
            "fit_span_s": None if fit is None else fit.span,
            "fit_rms_m": None if fit is None else fit.rms,
        }
        self._flights_ended += 1

    def _streak_is_consistent(self) -> bool:
        """A physically plausible burst far from the fit means a NEW throw."""
        streak = self._reject_streak
        if len(streak) < 2:
            return False
        for (t_a, p_a, _), (t_b, p_b, _) in zip(streak, streak[1:]):
            dt = t_b - t_a
            if dt <= 0.0:
                return False
            if _dist(p_a, p_b) / dt > self._cfg.max_speed_m_s:
                return False
        return True

    def _collect_expired(self, now_s: float) -> bool:
        if self._last_sample_t is None:
            return True
        if now_s - self._last_sample_t > self._cfg.collect_timeout_s:
            return True
        first_t = self._samples[0][0] if self._samples else self._last_sample_t
        return self._last_sample_t - first_t > self._cfg.max_collect_span_s

    def _flight_over(self, now_s: float, t_eval: float) -> Optional[str]:
        assert self._fit is not None
        if self._fit.position(t_eval)[2] < self._cfg.ground_z_m:
            return "ground"  # Isaac ball_on_ground parity
        if self._track_started_t is not None and \
                now_s - self._track_started_t > self._cfg.max_flight_s:
            return "max_flight"
        if self._state == COASTING and self._freeze_t is not None and \
                now_s - self._freeze_t > self._cfg.max_coast_s:
            return "max_coast"
        return None

    def _valid_estimate(self, now_s: float, t_eval: float) -> Estimate:
        assert self._fit is not None
        conf = min(1.0, self._fit.n / 10.0)
        if self._cfg.max_rms_m > 0.0:
            conf *= max(0.0, 1.0 - self._fit.rms / self._cfg.max_rms_m)
        if self._state == COASTING and self._freeze_t is not None:
            conf *= math.exp(-(now_s - self._freeze_t) / self._cfg.coast_conf_tau_s)
        return Estimate(
            position=self._fit.position(t_eval),
            velocity=self._fit.velocity(t_eval),
            valid=True,
            confidence=max(0.0, min(1.0, conf)),
            state=self._state,
        )

    def _invalid(self) -> Estimate:
        return Estimate(position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0),
                        valid=False, confidence=0.0, state=self._state)


def _rms(values: Sequence[float], predictions: Sequence[float]) -> float:
    total = sum((v - p) ** 2 for v, p in zip(values, predictions))
    return math.sqrt(total / len(values)) if values else 0.0


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
