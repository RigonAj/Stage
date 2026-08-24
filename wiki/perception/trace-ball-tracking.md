# Trace Ball Tracking

> Sources: Repository README, 2026-06-29; project synthesis, 2026-06-29; trace pose publication, 2026-07-03; full perception pipeline detail + ROI-gated accumulation + lead/coast prediction, 2026-07-08; ball radius ROS launch parameter, 2026-07-09; sampled display path, 2026-07-09; explicit camera_calibration_file parameter, 2026-07-09; GUI-framerate publish cadence analysis, 2026-07-09; independent robustness/timestamp review, 2026-07-10; input-source/polarity/recording/replay ROS parameters + trace-status heartbeat + offline real-throw validation, 2026-07-16; latency optimization (render-decoupled analysis, incremental undistortion, gated clustering), 2026-07-17
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

The ROS node timer runs at 1 ms (`publisher_member_function.cpp`,
`timer_callback`). **Since 2026-07-17 the Trace analysis and the publish are
decoupled from the render**: `Gui::RefreshTraceAnalysis()` reruns the trace
pipeline from the timer tick as soon as the accumulated events (or a relevant
slider) change — rate-limited by the `trace_analysis_period_ms` ROS parameter
(default 4 ms) — and `publishTracePose()` runs *before* `gui.Update()`. A
fresh pose therefore no longer waits up to 16.7 ms for the next 60 FPS frame
(the pre-2026-07-17 behavior, where `UpdateTraceAnalysis()` only ran inside
`Draw()`). Rendering itself stays gated at 60 FPS and still shares the thread,
so a heavy render frame can delay the *next* tick by a few ms. Each tick:

1. **Acquire** — `NextBatch()` pulls the next DVXplorer event batch (or the
   reader feeds recorded `.h5` events in file mode). Live batches are often
   empty between real bursts; the loop keeps the last view instead of redrawing.
   **Input source is ROS-initialized since 2026-07-16** (`use_reader`, default
   `false` = live camera). Before that, the GUI constructor hardcoded
   `reader_mode = true`: every launch started in **File mode and silently
   processed no camera events** until the operator clicked "Reader → Camera" —
   the root cause of the 2026-07-16 real-ball session producing zero valid
   Trace samples. File mode with no events now logs a throttled warning
   instead of staying silent.
2. **Denoise** — `Filter()` runs dv-processing's background-activity noise
   filter (1 ms activity window, set in the `DvCamera` constructor). This drops
   isolated sensor noise that would otherwise widen the ribbon.
3. **Undistort + rolling window (live, incremental since 2026-07-17)** —
   `UndistortLiveIncremental(timeslice)` undistorts **only the fresh filtered
   batch** (`cv::undistortPoints`, or `cv::fisheye` when the calibration is
   fisheye) and merges it into a rolling undistorted window (default
   `timeslice` ≈ 484 ms) via shallow `dv::EventStore` slices. This replaced
   the old `KeepRecentFiltered` + full-window `Undistort()` pair, which
   re-undistorted the entire ~484 ms window every tick — the main per-tick CPU
   cost and the cause of lag spikes during event bursts. The window resets on
   a backward time jump or a calibration/input switch. **Reader mode** keeps
   the full-window `Undistort()` recompute, but only reruns it when the
   playback file/position/window actually changed (a paused reader no longer
   re-reads the H5 and re-undistorts every tick). Both paths expose the raw
   and undistorted streams; the trace consumes either (`Trace use raw input`,
   default undistorted) and, in live mode, is fed just the fresh batch (its
   timestamp dedup made re-scanning the whole window per tick pure overhead).
4. **Subsample for display + cluster** — `Echantillon(maxevent)` decimates the
   filtered window into `Samples`. The GUI 2D texture draws only these sampled
   events to avoid lag. `Cluster(box, alpha, bandwidth, minNb)` (DBSCAN) and
   the tracker-cluster conversion **only run when circle fitting is ON or a
   non-Trace view is displayed** (since 2026-07-17): in the live-catch
   configuration (Trace view, circle fitting OFF) the whole DBSCAN path is
   skipped every tick.

## Pipeline — Trace Accumulation (`Gui::AppendTraceEvents`)

The trace is built from a **rolling time window** of events, not one frame:

- **Window** = the last `trace_memory_ms` of events (default **150 ms** since
  2026-07-17, slider 1–500 ms; was 40 ms). A hard cap of 120 000 events bounds
  memory. **Compaction is lazy since 2026-07-17**: aged-out events form a
  sorted prefix that is erased only once it dominates the buffer (the old
  full rewrite ran at 1 kHz over the whole accumulation and froze the loop
  during event bursts); the analysis applies the exact window cutoff itself
  when reading the buffer, so the fitted window stays exact.
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
strictly advances, so repeated ticks on the same accumulated events do not
re-publish. Upstream of that, `RefreshTraceAnalysis()` itself dedups: it only
reruns the ribbon/3D analysis when the accumulation changed or a
trace-relevant setting changed (settings signature), and at most once per
`trace_analysis_period_ms` (ROS parameter, default 4 ms — so up to ~250
fresh publishes/s during a throw, vs at most 60/s before 2026-07-17). Two
cost bounds protect the loop during event bursts: the analysis input is
stride-subsampled to at most **24 000 points** (bins still get hundreds of
events each; on the 2026-07-09 replay the ribbon is unchanged, 298 px), and
the effective analysis period adapts to twice the measured duration of the
previous run (≤ 50 ms), so a heavy fit throttles itself instead of starving
acquisition and rendering.

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
  for a ribbon; a window gives the trail its length. A short window keeps
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
| `Trace ms` (`trace_memory_ms`) | 150 (was 40 until 2026-07-17; slider 1–500 ms) | Accumulation window length. Longer = more trail/support and better prediction, more mid-window lag. On the 2026-07-09 replay, 150 ms gives a 298 px ribbon (vs ~118 px at 40 ms); the ribbon fit input is capped at 24 000 stride-subsampled points, so a longer window no longer inflates the per-run cost. |
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

Launch-initialized parameters: lead, hold, radius, calibration, and since
2026-07-16 the input source (`use_reader`, default live camera), scripted
replay (`reader_file`), trace polarity (`trace_polarity_mode`, default `all`)
and event recording (`record`/`record_file`). ROI, memory, edge refinement and
width smoothing remain GUI-local and reset to defaults on restart (full-frame
ROI, 150 ms, edge refine OFF, width smoothing OFF). This is a smaller but
still open reproducibility gap for real bags.

## Node I/O Parameters and Diagnosis (2026-07-16)

New `ball_tracking_cpp` ROS parameters, all set in `live_catch.yaml` and
overridable per launch:

| Parameter | Default | Effect |
|---|---|---|
| `use_reader` | `false` | `false` = live DVXplorer camera at startup, `true` = File/reader playback. The GUI Reader button still toggles it live. |
| `reader_file` | `""` | Scripted replay: a `recordings/` H5 file that forces reader mode and autoplays from t=0. Empty = no replay. |
| `default_reader_file` | `"realtest.h5"` | Reader-UI preselection only (no mode change): the GUI Read-file box starts on this `recordings/` file, so File → Play replays the last recording in one click. Ignored when `reader_file` forces a replay. |
| `trace_polarity_mode` | `"all"` | Trace polarity filter (`all`/`positive`/`negative`). The old hardcoded GUI default was `negative`, which starved the ribbon of support depending on the ball's contrast direction. |
| `record` | `false` | Recording is **manual** (GUI REC toggle); `true` arms it from launch. Reader mode never records. Default was briefly `true` on 2026-07-16; reverted after a data-loss incident (empty session recording truncated the previous file). |
| `record_file` | `"realtest.h5"` | Recording target; plain names land under `recordings/`. When a writer opens, an existing non-empty target is **archived as `<name>_YYYYMMDD_HHMMSS.h5`**, never truncated. |

The node also logs a 2 s **trace-status heartbeat**
(`trace status: events=… (peak …) ribbon=… world_pts=… 3d=… published=…`)
tracking stage peaks between prints, so a live session where the ribbon never
validates now shows *which* stage starves instead of logging nothing
(the 2026-07-16 failure signature was a log with startup lines only).

Offline validation 2026-07-16: replaying the 2026-07-09 real-throw recording
(212 354 events, 9.4 s, `recordings/realtest_2026-07-09_backup.h5`) through
`pose_source=trace` + polarity `all` produced an event peak of ~41 000, a
valid ribbon (~119 px) and 12–13 valid `BallState` samples with a physically
coherent approach trajectory; chained through `ball_regression_node` + the
hand-eye TF it yielded a full `idle → collecting → tracking → ended` flight
(10 accepted measurements, 0 rejected, RMS 0.013 m) and 27 `valid=true` fitted
samples on `/ball_state` in `base_link`. The C++ Trace path is therefore
functional on real data; live validation with physical throws remains.

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
- **Publish cadence is event-driven, not fixed-rate**: since 2026-07-17
  fresh-window publishes follow new event batches (rate-limited by
  `trace_analysis_period_ms`, default 4 ms) instead of the 60 FPS render gate,
  so `ball_state_raw` is faster but still irregular — the downstream
  `ball_regression_node` resamples it to a clean 60 Hz `ball_state` and
  remains the recommended feed for the policy
  ([Analyse pipeline commande](../../docs/Robot_Control/analyse_pipeline_commande_trace_2026-07-09.md)).
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

- [Trace Vs Circle Fitting Benchmark](trace-vs-circle-benchmark.md)
- [Real Perception Trace Test Runbook](real-perception-trace-test.md)
- [Live Catch Loop](../live-catch/live-catch-loop.md)
- [Message Contracts And Topics](../live-catch/message-contracts-and-topics.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Frames And Transforms](../calibration/frames-and-transforms.md)
- [Perception Robustness And Flight Lifecycle](perception-robustness-flight-lifecycle.md)
