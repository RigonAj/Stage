# Trace Vs Circle Fitting Benchmark

> Sources: Offline benchmark executable and campaign, 2026-08-04; EventGen benchmark chain, 2026-08-04
> Raw: [Offline benchmark](../../src/Ball_Tracking_Cpp/src/OfflineBenchmark.cpp); [Benchmark CLI](../../src/Ball_Tracking_Cpp/src/ball_tracker_h5_benchmark.cpp); [Sequence sidecars](../../src/Ball_Tracking_Cpp/src/SequenceData.cpp); [Trace analysis](../../src/Ball_Tracking_Cpp/src/TraceAnalysis.cpp); [Circle tracker](../../src/Ball_Tracking_Cpp/src/BallTracker.cpp); [Package build](../../src/Ball_Tracking_Cpp/CMakeLists.txt)

## Overview

`ball_tracker_h5_benchmark` replays a recorded or synthetic `.h5` event
sequence through **both** production perception pipelines and scores each
against Isaac Sim ground truth. It is the quantitative counterpart of the
qualitative Trace/circle comparison: same events, same intrinsics, same
ground truth, one CSV per method.

The two branches call the live code, nothing is reimplemented:

- **Trace**: `BuildTracePointsFromFloatSource` → `FitTraceRibbon` →
  `AnalyzeTrace3D` (see [Trace Ball Tracking](trace-ball-tracking.md)).
- **Circle**: `DvCamera::Echantillon` / `Cluster` → `BallTracker::Update`.

## Why The Previous Numbers Are Void

A first harness (`TraceBenchmark.cpp`, `ball_tracker_h5_benchmark.cpp`) existed
until commit `889a684` deleted it. It never called Trace: it ran one global PCA
over the whole 530 ms window and took quantile edges, so it reported the centre
of the entire trajectory arc instead of an instantaneous position. Everything it
produced — including the `benchmark_fast_throw_0500` report of **RMSE 2D
118 px / RMSE 3D 1.30 m** — is an artefact of that stand-in and must not be
cited. The deletion also silently disabled the CMake target, because the build
guards it behind `if(EXISTS ...)`.

## Correctness Contracts

Three conventions decide whether the numbers mean anything.

**Intrinsics.** Every sequence carries `camera/intrinsics.json`
(`fx=fy=520`, `cx=320`, `cy=240`, no distortion). The benchmark loads it
through the shared `LoadCalibrationFromIntrinsicsJson` and **aborts** if it is
missing or invalid. There is deliberately no fallback: silently reusing the real
DVXplorer calibration would corrupt every depth estimate. The chosen values are
echoed into `tracker_output/runtime.json` so a run can be audited after the
fact.

**Frames.** Both pipelines already emit the raylib world convention
`(x_cam, z_cam, -y_cam)` in metres. The CSV converts back to OpenCV camera
metres — `x_cam = world.x`, `y_cam = -world.z`, `z_cam = world.y` — to match the
ground-truth columns.

**Timestamps.** Each method stamps its estimate with its own event time, not the
sampling-grid time: Trace uses `Trace3DAnalysis::currentWorldTimestampUs` (the
mid-ribbon sample) and circle fitting uses `BallTrackerResult::poseTimestampUs`
(the newest event of the accepted slice). The evaluator interpolates ground
truth at that instant, which is what makes the comparison fair.

**Ball radius.** Read from `metadata.json` (`ball.radius_m = 0.02`), converted
to the millimetre *radius* the APIs expect; depth uses `2 × radius`.

## Operating Points

Each method is scored at its own best setting, so neither is handicapped.

- **Trace** runs the live production defaults (`Ui()` in `Gui.h`), notably
  `trace_memory_ms = 150`. A sweep over 30–400 ms on the detailed sequences
  stays within RMSE 3D 0.050–0.059 m, so the default is representative rather
  than cherry-picked.
- **Circle fitting** does *not* use its GUI defaults. The 484 ms inspection
  window spans far more trail than ball, the fitted radius inflates and depth
  collapses (RMSE 3D 0.74 m); a window sweep bottoms out at 10–20 ms
  (RMSE 3D 0.215 m). `min_nb` also had to drop from 40 to 5: on the sparse fast
  throws (~70 events per 15 ms window) DBSCAN formed no core point at all and
  the method scored **zero** detections. At 5 it detects on every sequence and
  the dense sequences are unchanged (0.2148 m vs 0.2149 m).

Settings live in
`EventGen/ball_event_dataset_v0/benchmark/configs/tracker_methods.yaml`.

## Results

Both methods at their own best operating point, Trace including the edge-
correction fix described in the next section.

### Nominal regime - 3 detailed sequences

Depth ~1.0-1.4 m, apparent ball diameter ~15-21 px. This is the regime the live
catch stack works in.

| Metric | Trace | Circle fitting |
|---|---|---|
| Detection rate | **0.73** | 0.47 |
| RMSE 3D | **0.045 m** | 0.136 m |
| RMSE depth | **0.043 m** | 0.128 m |
| RMSE 2D | **5.4 px** | 9.2 px |

Trace is 3x more accurate in 3D and available on 1.6x more samples.

### Far / fast regime - 138 sequences of `benchmark_fast_throw_0500`

Depth 1.8-3.4 m, speed 4.4-9.6 m/s, apparent ball diameter only 6-11 px.

| Metric | Trace | Circle fitting |
|---|---|---|
| Detection rate | **0.737** | 0.013 |
| RMSE 2D | **2.16 px** | 7.26 px |
| RMSE depth | **0.153 m** | 0.643 m |
| RMSE 3D | **0.157 m** | 0.685 m |
| Mean runtime | 1.65 ms | **0.008 ms** |

Trace wins every accuracy metric in both regimes while emitting ~57x more
estimates at range. Circle fitting is two orders of magnitude cheaper per
sample, which is its only remaining advantage.

Two cautions on the far-regime numbers:

- **Depth is intrinsically ill-conditioned here.** At 2.5 m the ball is 8.2 px
  across, so a 1 px diameter error moves depth by ~0.31 m. Reaching 0.157 m
  means the width estimate is now accurate to well under a pixel; there is
  little headroom left without a different depth cue.
- **Circle's RMSE is a conditional statistic.** At a 1.3 % detection rate it
  only fires when a cluster happens to be clean, so its error distribution is a
  heavily filtered subset. Always quote its detection rate beside it.

## The Apparent-Size Bias And Its Fix

The first campaign exposed a defect, not an algorithmic verdict. Comparing
measured apparent size against the true diameter, sample by sample:

| Regime | Ball diameter | Trace measured / true | Circle measured / true |
|---|---|---|---|
| Nominal (3 sequences) | 15-21 px | 0.96 | 1.13 |
| Far / fast (138 sequences) | 6-11 px | **0.65** | 1.98 |

Trace's ribbon width was near-unbiased on a large ball and lost 35 % on a small
one, while its image-plane track stayed excellent (2.2 px). The failure was in
the width estimator alone, and it was **scale-dependent**.

### Two mechanisms, both shrinking the measured width

`EstimateSupportedEdges` corrects each edge outward, because events sit at
integer pixel centres while the ball's border lies beyond the outermost centre.
That correction was written as a **fraction of the measured width**
(`rawWidth * borderRatio`, 3.5 %). Two things are wrong with it:

1. **Pixel quantisation is a constant** (~0.5 px per side), not a fraction. A
   20 px ball gets 0.70 px per side, right by coincidence; a 6.7 px ball gets
   0.23 px, far too little.
2. **Support-radius erosion.** `supportedLow`/`supportedHigh` require
   `localSupport` events within `supportRadiusPx` of a candidate edge, so on a
   sparse trail the outermost events are discarded and the edge moves inward by
   an amount set by **event density**, not by ball size. This term is why no
   single constant served both regimes.

### The fix

`TraceSupportEdgeSettings` gained two fields; **both default to 0, so live
behaviour is unchanged unless they are set**:

- `borderPixels` - a constant, for the quantisation term.
- `borderSpacingFactor` - multiplies the **measured** median sample gap at the
  edge (`EdgeSampleSpacing`). For samples with mean spacing `s` the true edge
  sits about `s` beyond the outermost one, so this term self-calibrates: large
  on a sparse trail, negligible on a dense one.

At `borderPixels = 0.75`, `borderSpacingFactor = 1.75` - one setting, both
regimes:

| | RMSE 3D before | after | width ratio | detection rate |
|---|---|---|---|---|
| Nominal (live regime) | 0.058 m | **0.045 m** | 0.96 -> **1.00** | unchanged |
| Far / fast (138 seq) | 1.716 m | **0.157 m** | 0.65 -> **0.99** | 0.737, unchanged |

An 11x gain at range and 22 % at close range, at identical detection rate, and
the apparent-size measurement is now unbiased in both. Trace beats circle
fitting on every metric in both regimes, which is what the design predicted all
along.

### What the parameter search added: nothing

`scripts/tune_trace_params.py` was run on 30 far-regime training sequences and
moved 13 of 15 parameters. Ablating on the 10 held-out sequences:

| Configuration | RMSE 3D | detection rate |
|---|---|---|
| baseline production | 1.175 m | 0.87 |
| baseline + constant edge correction only | 0.107 m | 0.87 |
| full tuning **without** the edge correction | 1.562 m | 0.71 |
| full tuning | 0.082 m | 0.71 |

91 % of the gain comes from the edge correction alone, and **without it the
fully tuned profile is worse than the baseline**: the other parameters were
being contorted to partly compensate a bias only the edge correction removes,
at a cost of 16 points of detection rate. Two practical conclusions: fix the
estimator rather than tune around it, and do not ship a 13-parameter
per-distance profile.

The same search on the nominal regime returned **-12 % on held-out data** and
tripped the script's own overfitting warning - with only 2 training sequences
it fit noise. That is the expected behaviour of the guard, not a failure of it.

## Why Circle Fitting Almost Stops Detecting At Range

On the far/fast sequences circle fitting emits an estimate on ~3 % of samples —
one single point on `sequence_000001`. DBSCAN is not the cause: clusters are
produced for 2766 of 2795 samples, median 26 events. 92 % of samples reach
`BallTracker::Update` with a cluster and leave without a pose.

Three distinct mechanisms, in order of impact:

1. **The polarity-symmetry validation.** `PolarFilter` requires both polarities
   present and a count imbalance `|countN - countP| / (countN + countP)` below
   `symCoef` (0.29 at the GUI default). v2e output on these sequences is
   85 % / 15 % imbalanced, so a 26-event cluster typically holds ~22 / ~4 and is
   rejected. The check encodes a real physical prior — a resolved ball shows
   opposite leading and trailing edges — but that prior breaks down once the
   ball is 6–11 px across.
2. **The stateful depth-jump gate.** A pose whose depth moves more than
   `depthJumpGateMm` (250 mm) from the previous one is rejected *without
   updating the reference*, so once depth noise exceeds the threshold the
   tracker latches and emits nothing more for the rest of the throw. That is
   exactly the single-point signature. Opening it to 3000 mm raises detections
   from 36 to 127 over 20 sequences — real, but still only 5 %. It is now
   exposed as `BallTrackerSettings::depthJumpGateMm`, default 250 unchanged.
3. **Benchmark fidelity bug (fixed).** The offline runner passed the raw GUI
   slider values for `sym_coef` / `sym_coef2`, where `Ui::Sym_coef()` divides
   them by 100 before the tracker sees them — a count fraction and an angle in
   radians. Both symmetry gates were therefore inert in the first campaign.
   With them active, nominal-regime circle fitting improves to RMSE 3D 0.136 m
   at a 0.47 detection rate (from 0.215 m at 0.74): the gate is doing its job,
   trading recall for precision.

## Related Work

`uzh-rpg/event-based_object_catching_anymal` (Forrai et al., ICRA'23) catches
objects at up to 15 m/s from 4 m with 83 % success, running at 100 Hz on a
Jetson Orin from a VGA event camera. Its packages are
`rpg_dynamic_obstacle_detection` and `rpg_ransac_parabola`, i.e. clustering plus
a **RANSAC-fitted parabola** as trajectory model. The published material does not
document how depth is obtained, so no claim is made here about their mechanism.

Two observations for this project:

- Their operating range (4 m, VGA) puts the ball at roughly 5 px, i.e. deeper
  into the regime where apparent-size depth is ill-conditioned than anything
  measured above. That is circumstantial evidence that depth-from-size is not
  their route at range.
- A RANSAC parabola regularises *scatter*, not *bias*. Our measured residual
  after the border fix is a systematic under-estimate, so a ballistic prior
  would fit a biased parabola rather than rescue it. Fixing the edge estimator
  remains the correct lever; the trajectory prior is complementary, and this
  repository already has one in `ur3e_live_catch`.

## Running It

Build (the CMake target re-enables itself once the sources exist):

```bash
source env.sh
build
```

Single sequence:

```bash
./build/ball_tracker_h5_benchmark \
  --events-h5 sequences/sequence_0001/events_v2e/events_filtered.h5 \
  --ground-truth sequences/sequence_0001/labels/ground_truth.csv \
  --camera sequences/sequence_0001/camera/intrinsics.json \
  --metadata sequences/sequence_0001/metadata.json \
  --output-trace /tmp/det_trace.csv \
  --output-circle /tmp/det_circle.csv \
  --runtime-output /tmp/runtime.json \
  --mode both
```

`--mode` accepts `trace`, `circle` or `both`. `--output` remains an alias of
`--output-trace`, and `--headless` is accepted and ignored: the benchmark links
raylib for its vector types but never opens a window.

Full campaign, from the EventGen repository:

```bash
python3 benchmark/scripts/run_tracker_batch.py --benchmark benchmark/datasets/benchmark_fast_throw_0500 --resume --jobs 8
python3 benchmark/scripts/evaluate_sequence.py --benchmark benchmark/datasets/benchmark_fast_throw_0500 --all --jobs 8
python3 benchmark/scripts/aggregate_results.py --benchmark benchmark/datasets/benchmark_fast_throw_0500
python3 benchmark/scripts/make_report.py --benchmark benchmark/datasets/benchmark_fast_throw_0500
```

## Report Figures

`scripts/plot_report_figures.py` rebuilds the two computed figures of the stage
report from data already on disk, without re-running the perception pipeline:

```bash
python3 scripts/plot_report_figures.py trace-convergence
python3 scripts/plot_report_figures.py intrinsic-poses
```

`trace-convergence` reads `evaluation/matched_samples_trace.csv` of one
benchmark sequence (default `sequence_000110` of `benchmark_fast_throw_0500`)
and plots ground truth, per-sample estimates and the trajectory model refitted
on 30 / 60 / 100 % of the detections. `intrinsic-poses` reads the intrinsic
calibration report and plots the tilt distribution and image coverage of the
retained views. Both write into `images/`.

## Tuning Parameters

`scripts/tune_trace_params.py` searches parameters for either method against
the same ground truth, by driving the benchmark executable:

```bash
python3 scripts/tune_trace_params.py --benchmark /home/rigon/Documents/EventGen/ball_event_dataset_v0/benchmark/datasets/benchmark_fast_throw_0500 --limit 40 --trials 100 --rounds 2 --jobs 8 --out /tmp/tune_far.json
```

Random search then coordinate descent, with two guards that decide whether the
output can be trusted:

- **Train/test split.** Sequences are split before the search; the tuned
  parameters are re-scored once on the held-out set. Quote the test number, not
  the train number, and treat a large gap as overfitting.
- **UI clamp bounds.** The search space matches the `Ui` clamps in `Gui.h`, so a
  tuned value can be dialled into the live GUI. The offline runner does not
  clamp, so a value outside those bounds would behave differently live.
- Detection rate is a hard floor (`--min-detection-rate`), not a weighted term:
  a candidate that detects almost nothing otherwise posts a flattering RMSE on
  the few easy samples it kept.

`--method circle` searches the circle-fitting space instead.

Outputs per sequence: `tracker_output/detections_{trace,circle}.csv`,
`tracker_output/runtime.json`, `evaluation/matched_samples_{trace,circle}.csv`
and a `evaluation/sequence_metrics.json` holding a `methods` block per method.
`aggregate_results/all_metrics.csv` carries one row per (sequence, method).

## Known Limits

- Only 138 of the 500 fast-throw sequences have `events_filtered.h5`; the rest
  stop at the video or raw-events stage and would need a v2e re-run.
- The whole fast-throw dataset sits in the ill-conditioned depth regime. A
  nearer, slower benchmark would be needed to characterise Trace between the
  two regimes measured here.
- Circle fitting's low detection rate on sparse sequences makes its RMSE a
  conditional statistic; detection rate must always be quoted next to it.

## See Also

- [Benchmark Trace vs circle (document detaille, figures)](../../docs/benchmark_trace_vs_circle.md)
- [Trace Ball Tracking](trace-ball-tracking.md)
- [Real Perception Trace Test Runbook](real-perception-trace-test.md)
- [Perception Robustness And Flight Lifecycle](perception-robustness-flight-lifecycle.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
