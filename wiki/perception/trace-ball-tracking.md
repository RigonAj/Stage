# Trace Ball Tracking

> Sources: Repository README, 2026-06-29; project synthesis, 2026-06-29; trace pose publication, 2026-07-03; full perception pipeline detail + ROI-gated accumulation + lead/coast prediction, 2026-07-08; ball radius ROS launch parameter, 2026-07-09; sampled display path, 2026-07-09; explicit camera_calibration_file parameter, 2026-07-09; GUI-framerate publish cadence analysis, 2026-07-09; independent robustness/timestamp review, 2026-07-10
> Raw: [README](../../README.md); [Synthese projet](../../docs/Context/synthese_projet.md); [Publisher node](../../src/Ball_Tracking_Cpp/src/publisher_member_function.cpp); [Trace analysis](../../src/Ball_Tracking_Cpp/src/TraceAnalysis.cpp); [Camera front-end](../../src/Ball_Tracking_Cpp/src/Camera.cpp); [Gui accumulation/panel](../../src/Ball_Tracking_Cpp/include/Ball_Tracking_Cpp/Gui.h); [Live-catch launch](../../src/ur3e_live_catch/launch/live_catch.launch.py); [Live-catch config](../../src/ur3e_live_catch/config/live_catch.yaml); [Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md); [Perception/control review](../../docs/Robot_Control/revue_perception_robuste_controle_fluide_2026-07-10.md)

## Overview

`Ball_Tracking_Cpp` estimates the 3D position of a fast ball from a DVXplorer
event camera. The primary algorithm is **Trace**: a thrown ball leaves a
motion-blurred trail of events. Instead of trying to fit a circle to one
instantaneous blob, Trace treats the whole trail as a **ribbon**, measures the
trail's apparent **width perpendicular to motion**, and converts that apparent
diameter into **depth** with the pinhole model. The trail also gives the ball's
2D path, so one trace window yields a short 3D trajectory, not a single point.

Why width-based depth: an event camera has no native depth. A single blob's
radius is noisy and ambiguous (partial detections, polarity asymmetry). The
trail width, measured over many events and smoothed along the trajectory, is a
far more stable size cue, and size → depth is direct once intrinsics and the
known ball radius are fixed.

The design goal that shapes every stage below: **robustness over instantaneous
latency**. Perception runs upstream of a real robot, so a wrong pose is worse
than a slightly late one. Almost every step is a robust estimator (medians,
MAD, LOESS, IRLS, RANSAC) rather than a least-squares mean.

## Frames and Units

Getting these right is the top correctness risk; the code keeps three
conventions and converts explicitly.

- **Image pixels**: DVXplorer 640×480, origin top-left, `x` right, `y` down.
- **`camera_optical` (mm)**: pinhole camera frame, `x` right, `y` down, `z`
  forward (depth). `estimateBallPoseFromCircle` and the depth stage produce mm
  here.
- **Internal "world" (m)**: `ToMeters` remaps camera mm `{x,y,z}` → `{x, z, -y}`
  in metres (a z-up, y-forward convention used by the 3D views and the
  trajectory fit). See `util.hpp` `ToMeters`.

The node converts the internal world estimate back to `camera_optical` mm before
publishing (`traceWorldToCameraMm`, the exact inverse of `ToMeters`), then to
metres for `BallState.position`. The published frame is `camera_optical`
(parameter `camera_frame_id`); downstream hand-eye/TF takes it to `base_link`.

## Pipeline — Front-End (per timer tick, `Camera.cpp`)

The ROS node timer is declared at 1 ms (`publisher_member_function.cpp`,
`timer_callback`), but the loop is **effectively capped at the GUI framerate**:
`gui.Update()` renders with raylib `SetTargetFPS(60)` and `EndDrawing` blocks
to hold the frame time, serializing acquisition, Trace analysis AND
publication behind the render (2026-07-09 analysis). A heavy render (3D view,
high `Max Events`) therefore drops the `BallState` cadence below 60 Hz;
decoupling publish from render is an open task. Each tick:

1. **Acquire** — `NextBatch()` pulls the next DVXplorer event batch (or the
   reader feeds recorded `.h5` events in file mode). Live batches are often
   empty between real bursts; the loop keeps the last view instead of redrawing.
2. **Denoise** — `Filter()` runs dv-processing's background-activity noise
   filter (1 ms activity window, set in the `DvCamera` constructor). This drops
   isolated sensor noise that would otherwise widen the ribbon.
3. **Rolling filtered window** — `KeepRecentFiltered(timeslice)` keeps a
   rolling window of filtered events (default `timeslice` ≈ 484 ms). This is
   the full pre-subsampling window used by undistortion, Trace accumulation and
   fallback logic.
4. **Undistort** — `Undistort()` produces two parallel streams: the **raw**
   filtered points and the **undistorted** points (`cv::undistortPoints`, or
   `cv::fisheye` when the calibration is fisheye), each clamped to image bounds.
   The trace can consume either (`Trace use raw input`, default undistorted).
   Undistortion runs on the **full** filtered window on purpose — the trace
   needs full trail density, so it must run before subsampling.
5. **Subsample for display + cluster** — `Echantillon(maxevent)` decimates the
   filtered window into `Samples`. The GUI 2D texture draws only these sampled
   events to avoid lag; `Cluster(box, alpha, bandwidth, minNb)` also consumes
   them for the legacy circle fitter. Trace still accumulates from the full raw
   or undistorted filtered streams before this display subsampling.

## Pipeline — Trace Accumulation (`Gui::AppendTraceEvents`)

The trace is built from a **rolling time window** of events, not one frame:

- **Window** = the last `trace_memory_ms` of events (default **40 ms**, slider
  up to 3000 ms). Older events age out each tick; a hard cap of 120 000 events
  bounds memory.
- **Spatial gate = fixed work-ROI (since 2026-07-08).** Only events inside a
  user-set rectangle (`WorkRoiX/Y/W/H`, image pixels, drawn as an orange box on
  the 2D views) are accumulated. This replaced the old circle-derived motion
  window, so the trace now runs with **circle fitting OFF** (the default) and
  the operator can crop out the region where the robot arm itself generates
  events. Default ROI is the full frame (no crop).
- **Reset** on a backward time jump > 1 ms (new recording/seek); a forward gap
  does not reset, so a new throw simply refills the window as the old events age
  out.

## Pipeline — Ribbon Fit (`FitTraceRibbon`)

This turns a cloud of accumulated events into three smooth curves (upper /
middle / lower edges of the trail) plus its extent.

1. **Gate** — return invalid unless ≥ **500** raw events, ≥ **250** radial
   inliers, ribbon length ≥ **35 px**, ≥ **7** valid bins. This is the built-in
   "let the trace form first" guard: nothing is fitted (and nothing published)
   until the trail has real support.
2. **Robust center + radial trim** — median center, drop events beyond the
   0.97 quantile of squared radius. Kills stray far events before orientation.
3. **Orientation** — closed-form 2×2 covariance eigen-decomposition
   (`FitTracePca`) gives the global principal direction. Because a real throw
   curves, `BuildTemporalPcaSlices` splits the window into time slices
   (`trace_pca_period_ms`, default ~36 ms), runs PCA per slice with chained
   sign consistency, and `CombineTemporalPcaDirection` blends them — a better
   tangent for a curved trail than one global PCA.
4. **Ribbon coordinates** — project every inlier to `(s, h)`: `s` along the
   trajectory, `h` perpendicular. `sMin/sMax` are the 0.03/0.97 quantiles of
   `s` (trim the sparse tips).
5. **Per-bin edges** — bin along `s` (`trace_line_bin_width_px`). For each bin,
   `EstimateSupportedEdges` finds the lowest/highest `h` that still has
   `localSupport` neighbours within `supportRadiusPx` (a density-supported
   extreme, not the raw min/max — one stray event cannot set an edge), then
   expands by a small `borderRatio` margin. Yields low/middle/high samples and a
   local width per bin.
6. **Sample cleaning** — `FilterRibbonSamplesRansac` + `FilterCoherentRibbonSamples`
   drop bins whose edges/width disagree with a global fit (outlier bins from
   occlusion or a second mover).
7. **Curve models** — `MakeLocalQuadraticModel` builds a **LOESS** model for
   each of the three edge sample sets: `FitLocalQuadraticAt` does a locally
   weighted quadratic (tricube weights within bandwidth `trace_line_window_px`,
   Gaussian fallback outside, ridge-regularised). Evaluating a curve at any `s`
   is a local robust regression, so the ribbon follows curvature without a
   brittle global polynomial.
8. **Edge refine (optional, `trace_edge_refine`, default OFF)** — a second pass
   re-detects edges inside a narrow band around the fitted curves and, crucially,
   **trims ≈ one ball radius off each end** (`capMarginPx`): at the leading and
   trailing tips the cross-section is a *chord* of the disc appearing/vanishing,
   not the apparent diameter, which would bias width (and therefore depth). This
   is the main defence against end-of-trace depth error.

## Pipeline — Width → 3D (`AnalyzeTrace3D`)

1. **Width per time-slice** — `EstimateTraceWidths` samples the ribbon in time
   (`trace_width_step_px` controls the count, 3–80). At each sample it collects
   local events, measures the supported edge width in the geometric-normal
   direction, and optionally reconciles it with the fitted-curve envelope width.
   `FilterTraceWidthSpikes` removes single-slice spikes.
2. **Width smoothing (optional, `trace_width_smoothing`, default OFF)** —
   `SmoothTraceWidthProfile` replaces each width by a robust polynomial
   `width(t)` (degree 1–2, IRLS with 2 reweighting passes, event-count base
   weights, clamped to 0.55–1.7× median). Rationale: depth ∝ 1/width, so
   pixel-level width noise turns directly into depth noise; a throw's depth is
   smooth in time, so a smooth `width(t)` is physically justified.
3. **Depth from width** — `TraceImagePointToWorldMeters`:
   `depth = fEff · (2·ballRadius) / widthPx`, where
   `fEff = √((fx·nx)² + (fy·ny)²)` is the focal length projected onto the
   width direction `(nx,ny)` (handles `fx ≠ fy`). The center pixel is then
   back-projected to `(x,y)` at that depth. The ball radius defaults to 20 mm,
   can be initialized from the ROS parameter / launch argument `ball_radius_mm`,
   and remains adjustable live from the **Option panel** "Ball radius (mm)"
   slider (`Ui::BallRadiusMm()`, clamped 1–100 mm). The node pushes it into the
   pose calibration on every input change and per-frame tracker settings, so
   depth rescales without a rebuild. Intrinsics come from the
   `camera_calibration_file` parameter; the live-catch launch defaults this to
   `recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`
   for the current real-test setup. Requires a ready calibration.
4. **3D outlier filter** — `FilterTraceWorldOutliers` runs two robust passes:
   a temporal-jump test (drop points that jump > 7× the median step while their
   neighbours bridge ≤ 3.5×), then a fit-residual test (fit linear `x,y` and
   quadratic `z(t)`, drop residuals > median + 6·MAD). It never removes more
   than half the points.
5. **Assemble** — the surviving `(worldPoint, time)` pairs are the window's 3D
   trajectory. `Trace3DAnalysis.currentWorld` (the mid-window point) is kept for
   the GUI display; the ROS output uses the full trajectory (next section).

## Pipeline — Trajectory Output and Prediction (node)

Since 2026-07-08 the node publishes a **trajectory prediction**, not a single
sample (Option A — prediction lives in C++):

1. It reads the whole window trajectory (`Gui::CurrentTraceTrajectory`:
   worldPoints + times + window origin).
2. It fits per-axis quadratics `world(t)` (`SolveWeightedPolynomialFit`,
   degree 2 with ≥5 points else linear).
3. It publishes the position evaluated at **`latest_sample + trace_lead_ms`**.
   The stamp is the evaluation event time (so `lead > 0` stamps in the future,
   matching the regression-node convention). `lead = 0` publishes the latest
   position with no mid-window lag.
4. **Coast** — when a fresh window stops arriving (ball out of ROI / trail gone),
   the last fit is extrapolated forward for up to **`trace_hold_ms`**, with
   `BallState.confidence` decaying linearly to 0. Coast is also driven from the
   timer's early-return paths so event gaps keep it alive; a *total* event
   blackout that stops the render timer suspends it (with the robot moving there
   are always events).

Deduplication: a window is only re-fitted when its latest sample stamp
strictly advances, so repeated render frames on the same events do not
re-publish.

## Output Contract

- **Topic**: `ball_state` (parameter `ball_state_topic`; the live-catch launch
  re-points it to `ball_state_raw` when the regression publisher is enabled).
- **Message**: `ur3e_catch_msgs/BallState`, `position` in metres,
  `header.frame_id = camera_frame_id` (`camera_optical`, enforced non-empty).
- **Timestamp limitation (audit 2026-07-10)**: `eventStampToRosTime` anchors
  the first published event after a >0.5 s gap to current ROS time. Relative
  event timing inside the throw is preserved, but fixed acquisition/processing
  age is hidden; raw `now - stamp` is therefore not a true end-to-end latency
  measurement yet.
- **`velocity`** is left `(0,0,0)` = "not provided"; the downstream consumer
  recomputes it. Filling it from the fit derivative is a possible follow-up.
- **`confidence`** = 1.0 on every live valid fit, decaying during coast. The
  live value is binary validity, not yet a score derived from width dispersion,
  support or fit conditioning.
- **`pose_source`**: `"trace"` (bring-up config, `live_catch.yaml`) uses the
  pipeline above; `"circle"` (code default for a bare `ros2 run`) is the legacy
  per-detection circle-fit pose. Legacy `ball_position_3d_mm` is a fallback path.

## Design Choices and Rationale

- **Trace instead of circle fit**: apparent width over a long trail is a
  stabler depth cue than a single blob radius; it also yields the 2D path for a
  short trajectory in one window.
- **Rolling time window (not per-frame)**: a single event frame is too sparse
  for a ribbon; a window gives the trail its length. Short default (40 ms) keeps
  the mid-window lag small; widen it to capture more of a flight (needed for a
  reliable prediction — see risks).
- **ROI gate instead of circle motion window**: decouples the primary algorithm
  from the legacy circle fitter and lets the operator crop the robot's own
  events. The trade: without a follow-window, Trace assumes the ball is the
  dominant mover inside the ROI — hence the ROI to exclude the robot.
- **Robust everywhere**: median center + radial trim, density-supported edges,
  RANSAC/coherent bin cleaning, LOESS curves, IRLS width smoothing, MAD-based 3D
  outlier rejection. Fast throws produce messy events; means would chase
  outliers.
- **Temporal PCA slices**: a curved throw is not a straight line; per-slice PCA
  recovers the changing tangent.
- **End-cap trim + width smoothing (optional)**: both target the two physical
  error sources — chord-vs-diameter bias at the tips and 1/width depth
  sensitivity. Off by default to keep the raw view faithful; turn on for depth
  accuracy.
- **Mid-window sample vs trajectory prediction**: the GUI shows the robust
  mid-window point; the robot output fits and extrapolates so the operator can
  ask for a future position (interception) and coast through occlusions.
- **Prediction in C++ (Option A)**: single low-latency process, all knobs as
  live sliders. Alternative: the Stage `ball_regression_node` does a full-flight
  ballistic fit in `base_link`; the two are alternative prediction sites, not
  meant to be stacked.

## Tuning Parameters (GUI sliders)

| Slider | Default | Effect |
|---|---|---|
| `Trace ms` (`trace_memory_ms`) | 40 | Accumulation window length. Longer = more trail/support and better prediction, more mid-window lag. |
| `Lead ms` (`trace_lead_ms`, ROS param since 2026-07-09) | 0 | Publish position predicted at latest + lead. Pinned to 0 by the live-catch launch when `use_ball_regression:=true`: the measurement layer must not publish extrapolated points into the regression. |
| `Hold ms` (`trace_hold_ms`, ROS param since 2026-07-09) | 0 | Coast duration after the ball leaves the ROI. Pinned to 0 under the regression for the same reason (coast points carry confidence < 1 and the regression drops them anyway). |
| `ROI x/y/w/h` | full frame | Fixed work-ROI; crop out the robot region. |
| `Max Events` | 1000 | Display/clustering event budget. Lower it if the UI lags; Trace accumulation still uses the full filtered stream. |
| `Ball radius (mm)` (`ball_radius_mm`, ROS launch arg + Option panel) | 20 | Physical ball radius used by the width→depth model; initialized at launch, live-adjustable, clamped 1–100 mm. |
| `camera_calibration_file` (ROS parameter / launch arg) | `recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml` in live-catch launch | OpenCV XML intrinsics used by undistortion and Trace depth. Verify the tracker log before real throws. |
| `Bin width px` | 4 | `s`-binning granularity for edges. |
| `Local window` (`trace_line_window_px`) | ~66 | LOESS bandwidth for the ribbon curves. |
| `Width step px` | 8 | Spacing of width samples along time. |
| `PCA ms` | ~36 | Temporal PCA slice period. |
| `Support div / min / max / radius` | 28 / 3 / 9 / 1.75 | Edge support density controls. |
| `Border %` | 3.5 | Edge margin expansion. |
| `Edge refine` | OFF | Second-pass edges + end-cap trim (fixes tip depth bias). |
| `Width fit` (smoothing) | OFF (Raw) | Robust `width(t)` smoothing (reduces depth noise). |
| `Trace use raw input` | OFF (Undist) | Feed raw vs undistorted points. |
| `Circle fit` | OFF | Legacy circle path; not needed for Trace. |

Only lead, hold, radius and calibration are currently launch-initialized. ROI,
polarity, memory, edge refinement and width smoothing remain GUI-local and
reset to defaults on restart (full-frame ROI, negative polarity, 40 ms, edge
refine OFF, width smoothing OFF). This is a reproducibility gap for real bags.

## Risks

- **Depth ∝ 1/width**: highly sensitive to pixel-width error. Width smoothing
  and edge refine mitigate; a mis-scaled `ballRadius` or bad intrinsics bias
  depth directly.
- **Frame mixing**: image / `camera_optical` / world / `base_link` must never be
  silently mixed. The `ToMeters` ↔ `traceWorldToCameraMm` inverse pair is the
  single frame-convention definition on the ROS path.
- **Prediction extrapolation**: lead/coast extrapolate the per-axis quadratics
  in `camera_optical` (no gravity model). A large `lead` from a short
  `trace_memory_ms` window is a long, noisy extrapolation — widen the window
  before trusting a +100 ms prediction.
- **Trailing end-cap bias**: the ball entering/leaving frame is a chord, not the
  diameter; the last samples degrade unless `trace_edge_refine` is ON.
- **Single-mover assumption**: without a follow-window, two movers inside the
  ROI corrupt the ribbon — keep the ROI tight around the ball's path and off the
  robot.
- **Publish cadence tied to the render loop**: fresh-window publishes happen at
  most once per GUI frame (`SetTargetFPS(60)`, dedup by latest sample stamp),
  irregularly; coast publishes in bursts every callback. `ball_state_raw` is
  therefore NOT a guaranteed 60 Hz — the downstream `ball_regression_node`
  resamples it to a clean 60 Hz `ball_state` and is the recommended feed for
  the policy ([Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md)).
- **Quality is not propagated**: a marginal but valid fresh ribbon gets the
  same confidence 1.0 as a well-conditioned one. Regression measurement-purity
  gating removes coast points only; it cannot reject live depth fits without a
  covariance/quality contract.
- **Fixed polarity/manual ROI**: the negative-polarity, full-frame defaults are
  lighting- and scene-dependent. Robust deployment should persist a profile,
  compare polarity candidates, then follow an acquired ball with a dynamic ROI
  while masking robot events.

## Main Code

- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp`: ROS node loop,
  ROI drawing, trajectory fit, lead prediction, coast and `BallState` publish.
- `src/Ball_Tracking_Cpp/src/TraceAnalysis.cpp`: pure Trace computation
  (`FitTraceRibbon`, `EstimateTraceWidths`, `AnalyzeTrace3D`,
  `TraceImagePointToWorldMeters`, `FilterTraceWorldOutliers`).
- `src/Ball_Tracking_Cpp/src/Camera.cpp`: acquisition, denoise, undistort,
  subsample, DBSCAN cluster.
- `src/Ball_Tracking_Cpp/src/Gui.cpp` + `include/.../Gui.h`: accumulation,
  work-ROI, panel sliders, trajectory getter, visual diagnostics.
- `include/Ball_Tracking_Cpp/util.hpp`: `ToMeters` frame remap, circle-pose
  fallback.
- `src/Ball_Tracking_Cpp/src/BallTracker.cpp`: legacy circle fitting.

## See Also

- [Real Perception Trace Test Runbook](real-perception-trace-test.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Perception Robustness And Flight Lifecycle](perception-robustness-flight-lifecycle.md)
