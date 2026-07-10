# Perception Robustness And Flight Lifecycle

> Sources: Independent perception/control review, 2026-07-10; first real Trace command-test analysis, 2026-07-09; perception-transmission improvement plan, 2026-07-09; current left-policy metadata, 2026-07-06
> Raw: [Independent review](../../docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md); [First real test analysis](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md); [Improvement plan](../../docs/Robot_Control/plan_amelioration_perception_transmission.md); [Trace publisher](../../src/Ball_Tracking_Cpp/src/publisher_member_function.cpp); [Ball regression](../../src/ur3e_live_catch/ur3e_live_catch/ball_regression.py); [Regression node](../../src/ur3e_live_catch/ur3e_live_catch/ball_regression_node.py); [Live node](../../src/ur3e_live_catch/ur3e_live_catch/live_catch_node.py); [Bring-up config](../../src/ur3e_live_catch/config/live_catch.yaml); [Left model metadata](../../data/models/latest-left/policy_metadata.json)

## Overview

The 2026-07-09 camera + Trace + real-robot attempt did not constitute a valid
perception or control-quality test. Its twitching robot and flapping command UI
are strongly explained by duplicate `live_catch_node` and ball producers: idle
`valid=false` messages repeatedly stopped the robot and reset policy state. The
single-stack fix addresses that incident, but no real rosbag or 3D ground truth
yet demonstrates robust perception.

The recommended architecture remains:

```text
Trace measurements in camera_optical
  -> transform to base_link
  -> one ballistic/flight estimator
  -> current ball state + velocity at 60 Hz
  -> policy + independent safety + 500 Hz command streaming
```

The next priority is not more robot speed. It is an explicit, testable contract
for time, uncertainty and the beginning/end of one flight.

## Verified And Unverified State

Verified locally on 2026-07-10:

- the four affected ROS packages build;
- `ur3e_live_catch`: 127 tests pass, 1 skips;
- `ur3e_web_ui`: 51 tests pass;
- producer-conflict detection, measurement-purity gating, anisotropic weighting
  and offline regression replay are present in code.

Not yet verified:

- real Trace 3D accuracy throughout the catch volume;
- false-positive rate with hand/robot/reflections;
- real raw sample gaps and time-to-first-valid state;
- true event-to-command latency;
- fixed-gravity fit under the physical ball's drag/spin;
- one uninterrupted real command flight after the duplicate-producer fix.

The hardest anisotropic unit tests use 120 Hz synthetic data. Deployment Trace
is GUI-capped to at most 60 Hz, so 30/60 Hz noise + dropout coverage and real
captures are still required.

## P0 Findings

### Historical provisional 0.2 s lead (default reverted)

`BallRegression.step(now)` already evaluates old measurements at `now`, so
`lead_time_s=0` compensates measurement age inside the model. The former
extra 0.2 s default presented a future ball with current robot joints, moved the
policy outside its training timing and ended the flight on predicted ground
about 0.2 s early. On 2026-07-10 `live_catch.yaml` was reverted to 0.0 by
default. Only add a separately named prediction horizon after measurement,
replay and training validation.

### Timestamp contract mismatch

The documented timestamp semantics do not match implementation:

- the tracker re-anchors the first event published after a gap to current ROS
  time, which preserves within-flight intervals but hides fixed processing age;
- the regression evaluates at `now + lead` but stamps the output at `now`.

Therefore current `perception_age_s` cannot measure true source latency and is
near zero even when the state has a 0.2 s future horizon. Robust contracts need
at least `measurement_stamp`, `state_stamp` and `publish_stamp`; watchdog
freshness must use the last real measurement time.

### Binary measurement quality

Fresh Trace windows publish confidence 1.0 regardless of width dispersion,
support or fit conditioning. `min_input_confidence=1.0` correctly drops tracker
coast, but cannot reject a marginal fresh measurement. Regression output
confidence decays during coast, yet `live_catch_node` currently consumes only
`valid`. Trace should publish quality/covariance, and control should use a
hysteretic uncertainty gate rather than alternate instantly between valid and
invalid.

### Command authority exclusivity

Producer diagnostics run every 2 s. Ball-topic conflicts block command output,
but duplicate live-node/telemetry warnings do not. A robust arm service should
synchronously reject any duplicate command authority or contract producer.

## Recommended Flight Lifecycle

Each throw should carry a monotonic `flight_id` and one explicit phase:

| Phase | Estimator rule | Control rule |
|---|---|---|
| `IDLE` | No candidate; maintain noise statistics. | Stable hold; reset once. |
| `CANDIDATE` | Event/width/motion candidate in the launch region. | No motion. |
| `RELEASED` | Approach, support and intercept-feasibility gates pass. | Begin only when control-valid. |
| `TRACKING` | Measurements accepted; uncertainty bounded. | Normal PPO + safety. |
| `COASTING` | Short occlusion; frozen fit and growing covariance. | Continue only within time/uncertainty bounds. |
| `PASSED_OR_IMPACT` | Hoop plane crossed, ball moves away, or near-hoop deflection. | Hold without a target jump. |
| `ENDED` | Ground, timeout, excessive uncertainty or infeasible flight. | Hold; optional slow return between throws. |
| `REFRACTORY` | Ignore bounce, tail events and ball retrieval. | No re-arm until a new launch candidate. |

Current radial gates (`min_pop_distance_m`, `freeze_distance_m`) are only rough
proxies for task geometry. The start gate should use the loaded model envelope,
closing speed toward the hoop plane, predicted time-to-plane and reachable
intersection. The end gate should include crossing/passing that plane, not only
predicted ground and timeouts.

## Left-Policy Envelope

The locally deployed `latest-left` metadata declares:

- spawn position `x=[0.2,0.6]`, `y=[1.2,2.1]`, `z=[0.5,1.2]` m;
- velocity `vx=[-0.6,0.7]`, `vy=[-5.0,-4.0]`, `vz=[0.2,1.5]` m/s;
- `hold_side=left`, position-noise metadata 0.05 m and
  `disk_radius_m=0.1`.

Early real command tests should use throws within this envelope plus a small
margin. `require_approach=false` and the regression's broad 0.5–10 m/s speed
range otherwise allow out-of-distribution movers to reach the policy before
the later ballistic check aborts them.

## Trace Robustness Priorities

- Decouple acquisition/Trace/publication from raylib rendering.
- Expose and persist ROI, polarity, memory, edge refinement and width smoothing
  as ROS parameters; current defaults are full-frame ROI, negative polarity,
  edge refine off and width smoothing off.
- Acquire widely, then follow the accepted ball with a dynamic image-space
  tube; mask robot/hoop events using TF/URDF projection or a calibrated mask.
- Evaluate positive/negative polarity candidates rather than rely on one
  lighting-dependent default.
- Derive depth uncertainty from width dispersion. Because depth is inverse
  width, its variance changes with distance.
- Replace the diagonal base-axis anisotropy approximation with the full
  camera-ray covariance for gating and robust GLS fitting.
- Make the fixed-gravity consistency test use the same covariance/robust
  weights; require repeated evidence before aborting a real flight.
- Measure the physical ball radius and mass, then test whether drag/spin makes
  fixed-gravity, linear-horizontal motion inaccurate.

## Smooth-Control Priorities

The metadata-driven incremental mapper, independent velocity/acceleration
limits and 500 Hz interpolation are good foundations. Remaining points:

- `v_safe_scale=0.5` is safe bring-up but differs from the full-speed dynamics
  in the left model metadata;
- observed saturated policy actions mean smoothness currently comes mainly from
  the limiter, so action saturation and limiter duty cycles must be telemetered;
- do not add an untrained action low-pass filter as a quick fix; measure command
  jerk/tracking first, then use a jerk-aware interpolator and/or retrain with
  real limits, latency, noise, dropout and actuator dynamics;
- reset policy/control memory once per flight transition, not on every low-level
  heartbeat;
- keep short coast continuous only while intercept uncertainty remains bounded.

## Next Validation Gate

Start with command off and verify the default `lead_time_s=0`. Record H5 events plus a rosbag with
`ball_state_raw`, `ball_state`, `/joint_states`, TF, telemetry, command topic and
parameter events. Run valid throws across depth plus negative tests (robot-only,
hand-only, wrong direction, occlusion, bounce/retrieval).

Before command-mode real throws, require at least:

- no false pop in negative tests;
- time to first control-valid state p95 no worse than 100 ms;
- no nominal active-flight measurement gap over the 100 ms stale bound;
- near-zero `non_ballistic` aborts on genuine throws;
- physically validated camera-to-base and width-to-depth accuracy;
- unique producers/command authority and no driver/watchdog rejection.

Provisional downstream targets from the review are intercept-point p95 error
within 5 cm, velocity p95 error within 0.3 m/s and plane-time p95 error within
25 ms. They must be confirmed against real ground truth before becoming
release criteria.

## See Also

- [Trace Ball Tracking](trace-ball-tracking.md)
- [Real Perception Trace Test Runbook](real-perception-trace-test.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Safety And Commanding](../live-catch/safety-and-commanding.md)
- [Observation Latency And Models](../sim-to-real/observation-latency-and-models.md)
- [Isaac Training Environment](../sim-to-real/isaac-training-environment.md)
