# Trace vs Circle Fitting Comparison Benchmark

> Sources: Offline comparison harness and metrics script, 2026-08-04; simulated sequence dataset `ball_event_dataset_v0` (Isaac Sim + v2e), 2026-08-04
> Raw: [Benchmark](../../src/Ball_Tracking_Cpp/src/method_comparison_benchmark.cpp); [Trace runtime](../../src/Ball_Tracking_Cpp/src/TraceRuntime.cpp); [Sequence dataset](../../src/Ball_Tracking_Cpp/src/SequenceDataset.cpp); [Metrics script](../../scripts/compare_trace_vs_circle.py); [Circle tracker](../../src/Ball_Tracking_Cpp/src/BallTracker.cpp); [Build](../../src/Ball_Tracking_Cpp/CMakeLists.txt)

## Overview

`method_comparison_benchmark` replays a simulated sequence through **both** 3D
estimators — Trace and the legacy circle fitting — and writes, for every
estimate, the position next to the time-interpolated ground truth.
`scripts/compare_trace_vs_circle.py` turns those CSVs into RMSE, bias and
coverage tables.

Why it exists: the project's claim that Trace beats circle fitting on fast
balls was argued qualitatively (apparent-radius sensitivity, visual 3D plots)
but never measured against a reference. The Isaac Sim + v2e sequences carry a
per-instant ground-truth ball position, so the claim can be turned into
numbers.

**Status 2026-08-04**: the harness and the metrics script are written and
syntax-checked; the metrics script is exercised end-to-end on synthetic
detections. **No number has been produced yet** — that requires building on a
machine with dv-processing/raylib and the dataset present.

## Design Rule: Drive The Real Code

A previous benchmark (`TraceBenchmark.cpp` + `ball_tracker_h5_benchmark.cpp`,
removed 2026-06-08) carried its **own copy** of the trace maths. It stopped
measuring the shipped algorithm as soon as the two diverged, which is exactly
what happened when the trace pipeline was extracted into `TraceAnalysis.cpp`.
Its dead CMake block survived until this change.

The new harness therefore calls the same code the live node calls:

| Stage | Shared implementation |
|---|---|
| Trace accumulation | `TraceAccumulator::Append` (`TraceRuntime.hpp`) |
| Trace ribbon + 3D | `RunTraceAnalysis` → `FitTraceRibbon`, `AnalyzeTrace3D` |
| Undistortion | `DvCamera::Undistort` |
| Circle clustering | `DvCamera::Echantillon`, `DvCamera::Cluster` |
| Circle fit + pose | `BuildTrackerClusters`, `BallTracker::Update`, `estimateBallPoseFromCircle` |
| Sequence I/O | `sequence_dataset::*` (`SequenceDataset.hpp`) |

Two extractions made this possible, both pure moves with no behavior change:

- **`TraceRuntime`** — the rolling accumulator and the analysis call sequence
  used to live in `Gui`, whose constructor calls `InitWindow()`. `Gui` now
  holds a `TraceAccumulator` and calls `RunTraceAnalysis`, so the pipeline runs
  headless.
- **`SequenceDataset`** — sidecar discovery, `camera/intrinsics.json`,
  `labels/ground_truth.csv` and `metadata.json` reading moved out of the
  `Gui.cpp` anonymous namespace. `Gui` delegates to them.

`BuildTrackerClusters` (DBSCAN clusters → tracker input) moved from the ROS
node to `BallTracker.hpp` for the same reason.

## Dataset Contract

A sequence directory is accepted when it contains **both**
`labels/ground_truth.csv` and `camera/intrinsics.json`; the event file is
looked up as `events_v2e/events.h5`, then a few known variants, then any
`*.h5`/`*.bin` under the sequence. Rejected directories are logged with the
reason.

| File | Used for |
|---|---|
| `camera/intrinsics.json` | `fx, fy, cx, cy`, distortion, image size |
| `labels/ground_truth.csv` | `timestamp_s`, `ball_{x,y,z}_cam_m`, optional `visible` |
| `metadata.json` | `ball.radius_m` |
| `events_v2e/events.h5` | event stream |

`EventReader` already reads both H5 layouts: the compound `PackedEvent`
dataset written by the recorder, and the integer N×4 `[t, x, y, p]` dataset
that v2e produces. No conversion step is needed.

## Intrinsics And Ball Radius Are Sequence-Local

This is the correctness pivot of the whole comparison. Depth is `f · D / size`
in both methods, so a wrong focal length or a wrong ball radius biases every
single measurement:

| | Simulated sequence | Live defaults |
|---|---|---|
| Intrinsics | `fx = fy = 520`, `cx = 320`, `cy = 240`, 640×480, no distortion | DVXplorer mire XML |
| Ball radius | `metadata.json` → 0.02 m | `ball_radius_mm` = 20 mm, but `BallTrackerSettings` defaults to **60 mm** |

The harness reads both from the sequence and refuses to run a sequence with no
radius unless `--ball-radius-mm` is given. Both values, and the file they came
from, are written to `run_manifest.json` — check them there before trusting a
result. A ~3× depth error means the 60 mm tracker default leaked through; a ~2×
error means a radius/diameter mix-up.

## Frames

Estimates and ground truth are both written in **`camera_optical` metres**
(x right, y down, z = depth), so the `z` column is directly comparable to
`ball_z_cam_m`. Internally the trace 3D output and the loaded ground truth
live in the internal world convention (`ToMeters`: camera `{x,y,z}` → `{x, z,
-y}`); `sequence_dataset::WorldToCameraOptical` is the inverse applied on the
way out.

## One Estimate Per Instant

Both methods run on sliding windows, so consecutive runs re-estimate the same
instants: the circle window spans ~484 ms and slides one tick, and a trace
window spans 150 ms. Emitting every sample of every run would inflate every
statistic with correlated duplicates and would not compare like with like.

Default `--emit newest` keeps **one estimate per distinct measurement
instant**:

- Trace: within each analysis run, only world points strictly newer than the
  last emitted instant — the freshest estimate available, which is what a
  real-time consumer would use.
- Circle: a pose is written only when `poseTimestampUs` strictly advances.

`--emit all` keeps everything (with `run_index`) for studying within-window
behavior.

## Metrics

`scripts/compare_trace_vs_circle.py` (standard library only; matplotlib
optional for figures) reports per sequence and aggregated:

- **RMSE 3D** and per-axis **RMSE X / Y / depth** — depth is the discriminating
  axis, it is the one derived from an apparent size;
- **median and p95** error, so a few aberrations cannot carry the comparison;
- **signed depth bias**, which quantifies the systematic depth offset the
  report describes qualitatively for circle fitting;
- **coverage** (fraction of the labelled flight within ±20 ms of an estimate),
  **estimates/s** and **first-estimate latency** — without these, RMSE is
  meaningless when one method emits 500 estimates and the other 12;
- **RMSE vs true depth** in 0.5 m buckets;
- mean runtime per estimate.

Estimates landing where the ground truth is flagged invisible are skipped by
default (`--include-invisible` to keep them).

## Commands

```bash
source env.sh
build

# One sequence, to check the intrinsics the run picked up
./build/ball_tracking_cpp/method_comparison_benchmark \
  --sequence sequences/sequence_0001 \
  --out /tmp/cmp_one

# Whole dataset
./build/ball_tracking_cpp/method_comparison_benchmark \
  --dataset-root /home/rigon/Documents/EventGen/ball_event_dataset_v0 \
  --out evaluation/method_comparison/run1

python3 scripts/compare_trace_vs_circle.py evaluation/method_comparison/run1
```

Outputs land in the `--out` directory: `detections_<sequence>_<method>.csv`,
`run_manifest.json`, then `comparison_summary.csv`, `comparison_summary.md` and
`figures/`. The root `evaluation/` directory is gitignored.

## Time Base Pitfall

Ground-truth times start at 0 (`frame_id / fps`). Event timestamps are used
raw by default (`--time-base file`), matching what the GUI overlay does. If the
v2e stream does not start at 0 the two do not line up and almost nothing
matches; the harness detects this (fewer than a quarter of estimates matched)
and prints both spans plus a suggestion to use `--time-base zero`.

## Risks

- **Parameter fairness**: each method runs with its own live defaults, which is
  what actually ships, not a jointly tuned optimum. Every parameter is in the
  manifest and overridable, so a sensitivity study is a rerun, not a rebuild.
- **Window semantics differ**: the circle path consumes a ~484 ms trailing
  window subsampled to 1000 events before DBSCAN, the trace path a 150 ms
  accumulation capped at 24 000 points. Coverage and estimates/s exist to keep
  that asymmetry visible instead of hiding it inside RMSE.
- **Simulation only**: v2e events are not DVXplorer events. A win here bounds
  the geometric quality of the algorithms; it says nothing about real noise,
  lighting or polarity behavior.
- **`size_px` is per window, not per point**: the 3D outlier filter runs after
  the per-sample widths are consumed, so per-point widths cannot be recovered;
  the column carries the window's ribbon width (or the circle radius).

## See Also

- [Trace Ball Tracking](trace-ball-tracking.md)
- [Real Perception Trace Test Runbook](real-perception-trace-test.md)
- [Perception Robustness And Flight Lifecycle](perception-robustness-flight-lifecycle.md)
- [Camera And Hand-Eye Calibration](../calibration/camera-and-handeye-calibration.md)
- [Testing And Commands](../operations/testing-and-commands.md)
