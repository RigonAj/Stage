# Intrinsic Calibration Runbook (DVXplorer Event Mire)

> Sources: calibration Python architecture, 2026-06-29; env.sh calibration aliases, 2026-07-09
> Raw: [Calibration scripts architecture](../../docs/Context/calibration_python_architecture.md); [Capture tool](../../scripts/event_mire_calibration.py); [Solver](../../scripts/calibrate_intrinsics_from_mire.py); [Aliases](../../env.sh)

## Overview

Operator checklist to redo the **DVXplorer intrinsic calibration** with the
blinking event mire. Two steps: capture mire observations with
`event_mire_calibration.py`, then solve the OpenCV intrinsics with
`calibrate_intrinsics_from_mire.py`. Intrinsics must be validated before any
hand-eye session — hand-eye never beats the intrinsics it uses (see
[Extrinsic Calibration Runbook](extrinsic-calibration-runbook.md)).

## Working Directories

Everything lives under one directory (created by the scripts if missing):

```text
recordings/mire_calibration/
```

| File pattern | Produced by | Role |
| --- | --- | --- |
| `mire_observation_*.json` | capture (`Calib`) | one accepted view: `object_mm ↔ camera_px` matches — **input to the solver** |
| `mire_overlay_*.png` | capture (`Calib`) | control image, verify labels land on real dots |
| `calibration_test_*.json/.png` | `Test calib` / F9 | reprojection check of an existing XML |
| `square_test_*.json/.png` | `Test carré` / F10 | independent-geometry validation |
| `intrinsics_from_mire*.xml` | solver | **output** OpenCV intrinsics used by the C++ code |
| `intrinsics_from_mire*_report.json` | solver | readable report: RMS, per-view errors, warnings, flags |
| `handeye/handeye_samples_*.json` | phone/hand-eye mode | extrinsic samples (not intrinsic) |

- Capture output dir: `--output-dir` (default `recordings/mire_calibration`).
- Solver input dir: `--input-dir` (default same); outputs: `--output-xml` /
  `--output-json`.
- The `calib` alias writes to a distinct name so it does not clobber older XMLs:
  `intrinsics_from_mire_robust_constrained.xml` (+ `_report.json`).

## Prerequisites

- **DVXplorer plugged in via USB.** The capture tool opens it directly through
  `dv_processing` — no ROS camera driver is involved.
- **Measure the active screen size in mm with a caliper.** The EDID can be
  wrong, and the mire object coordinates depend on real mm/px.
- No robot stack is needed for pure intrinsics (only for the phone/hand-eye
  variant).

## Environment And Aliases

`env.sh` defines the calibration helpers. Source it once, then use the alias:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
```

| Alias | Expands to |
| --- | --- |
| `calib` / `calib_intrinsics` | `calibrate_intrinsics_from_mire.py --robust --use-intrinsic-guess --zero-tangent-dist --fix-k3 --output-xml …_robust_constrained.xml --output-json …_report.json --camera-name DVXplorer_mire_robust "$@"` |

The capture tool has no alias — call the script directly.

## Step 0 — Pre-Flight

List detected monitors and run the built-in self-tests:

```bash
python3 scripts/event_mire_calibration.py --list-monitors
python3 scripts/event_mire_calibration.py --self-test
python3 -m py_compile scripts/event_mire_calibration.py scripts/calibrate_intrinsics_from_mire.py
```

## Step 1 — Capture Mire Observations

Launch the interactive Qt tool with the **measured** screen size (example
values 344 × 194 mm):

```bash
python3 scripts/event_mire_calibration.py \
  --monitor 1 \
  --screen-width-mm 344 \
  --screen-height-mm 194 \
  --accum-ms 240
```

Useful variants:

```bash
# start on a specific mire pattern (mire | grid_5x4 | grid_7x5 | grid_9x6)
python3 scripts/event_mire_calibration.py --monitor 1 \
  --screen-width-mm 344 --screen-height-mm 194 --pattern grid_7x5

# enable background-noise filter (support ≈ 1/cutoff, 500 Hz ≈ 2 ms)
python3 scripts/event_mire_calibration.py --monitor 1 \
  --screen-width-mm 344 --screen-height-mm 194 --noise-filter --noise-cutoff-hz 500

# longer accumulation if blobs are weak
python3 scripts/event_mire_calibration.py --accum-ms 500
```

Per view: press **`Calib`** to record. Aim for **10–20 varied captures** —
center, corners, edges, near, far, tilted. After each capture open the
`mire_overlay_*.png` and confirm every label sits on the real dot.

**Oblique views are supported.** The blob→dot association fits a homography
from candidate convex-hull grid corners and refines it by iterated closest
point (`associate_blobs_to_layout`), so it is robust to in-plane rotation
(camera roll), perspective keystone and PCA corner-collapse cases. Tilt and
rotate freely for pose diversity; the `--self-test` locks this in with
strongly tilted/rolled synthetic poses plus a PCA-collapse regression case. If
a capture is still rejected with `could not fit grid homography`, inspect the
overlay for merged, missing or spurious blobs, then shorten accumulation,
enable/tune the noise filter or use a sparser pattern.

The default association acceptance is now the active target size
(`--min-matched 0` means all points in the selected pattern), so `grid_7x5`
uses 35 expected matches rather than the 19-point asymmetric default.

Validation buttons against an existing XML: **`Test calib` / F9** (solvePnP +
reprojection) and **`Test carré` / F10** (4-point squares, different geometry).

## Step 2 — Solve The Intrinsics

Default (uses every `mire_observation_*.json` in the input dir):

```bash
python3 scripts/calibrate_intrinsics_from_mire.py
```

Recommended robust + constrained run via the alias:

```bash
calib
# equivalently, only one mire type:
calib --pattern grid_7x5
```

Manual robust call with an explicit threshold:

```bash
python3 scripts/calibrate_intrinsics_from_mire.py --robust --ransac-threshold-px 0.5
```

Change the paths explicitly:

```bash
python3 scripts/calibrate_intrinsics_from_mire.py \
  --input-dir recordings/mire_calibration \
  --output-xml recordings/mire_calibration/intrinsics_from_mire.xml \
  --output-json recordings/mire_calibration/intrinsics_from_mire_report.json
```

Key flags: `--robust` (RANSAC-style view selection), `--use-intrinsic-guess`,
`--zero-tangent-dist` (p1=p2=0), `--fix-k3`, `--pattern all|mire|grid_*`.

## Acceptance Checklist

- Overlays show every center on the right dot.
- Observations cover several sensor zones and 2–3 apparent scales.
- Global RMS is coherent with the expected precision (prior XMLs ≈ 0.49 px).
- No single per-view RMS dominates.
- The XML/JSON were regenerated **after** the latest captures.
- The XML consumed by the C++ code is the one you just produced.

## See Also

- [Camera And Hand-Eye Calibration](camera-and-handeye-calibration.md)
- [Extrinsic Calibration Runbook](extrinsic-calibration-runbook.md)
- [Frames And Transforms](frames-and-transforms.md)
