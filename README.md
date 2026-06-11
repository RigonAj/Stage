# Event-Based 3D Ball Tracking

ROS 2 C++ project for detecting and tracking a fast-moving ball with a DVXplorer event camera.

The application reads asynchronous camera events (live or from recorded/simulated files), filters noise, follows the ball in the image, and estimates its 3D position. The main estimation algorithm is the **Trace** algorithm: instead of fitting a circle on the instantaneous ball contour, it measures the width of the event trail left by the fast ball and converts that width into depth. The legacy circle-fitting tracker is still available as an optional mode for comparison.

## Why Trace Is the Main Algorithm

Fast ball motion creates a long event trail. Fitting a circle on that trail overestimates the apparent radius, and since depth is inversely proportional to the radius, the ball looks too close. The depth sensitivity is roughly `Z / width_px` per pixel of error: at 1.47 m with a ~21 px apparent diameter, a 1 px error already means ~7 cm of depth error.

The Trace algorithm measures the trail width perpendicular to the motion from many local slices, which makes the apparent diameter far less sensitive to isolated events or a badly fitted circle.

## Trace Processing Pipeline

1. Acquire event batches from a DVXplorer camera or from a recorded/simulated file (H5/bin).
2. Filter background activity with `dv-processing`.
3. Undistort event coordinates using the camera calibration (real OpenCV XML, or the per-sequence `camera/intrinsics.json` for simulated data).
4. Accumulate recent events inside a moving window that follows the ball trajectory (`Trace ms` memory, polarity filter).
5. Estimate the trail direction with a global PCA plus temporal PCA slices, and transform events into a local trace frame:

   ```text
   s = position along the trail
   h = position normal to the trail
   ```

6. Split the trail into bins along `s` and detect, in each bin, the two *supported edges*: the extreme `h` values that have enough close neighbours (isolated events cannot create an edge).
7. Reject incoherent bins, then fit three local curves: upper edge, middle line, lower edge.
8. Measure the local trail width along the normal of the middle line, and reject isolated width spikes.
9. Convert each center point plus width into a 3D position:

   ```text
   Z = f_eff * real_diameter / width_px
   f_eff = sqrt((fx*nx)^2 + (fy*ny)^2)
   X = ((u - cx) / fx) * Z
   Y = ((v - cy) / fy) * Z
   ```

10. Filter 3D outliers and fit the trajectory, then publish the ball position on the ROS 2 topic `ball_position_3d_mm`.

The full algorithm is documented visually in `trace_algorithm_explanation.html` (detailed explanation, parameters, diagnostics) and `algo_trace_graph.html` (C++ pipeline graph).

## Trajectory Fit

Each valid 3D estimate is added to the trajectory history. The trajectory model is:

```text
X(t) = a*t + b
Y(t) = a*t + b
Z(t) = a*t^2 + b*t + c
```

An optional **weighted regression** mode (`Weighted reg` toggle) keeps the same model but weights each sample by recency (`exp(-3 * age)`) and by robustness (`1 / (1 + (residual/scale)^2)`), so recent coherent measurements dominate and outliers stop dragging the curve.

## Circle Fitting (Optional Mode)

The original tracker is enabled with the `Circle fit` toggle (off by default). It clusters events with DBSCAN, fits a circle on the best cluster, validates it with the polarity symmetry of the events, and estimates depth from the apparent radius:

```text
Z = fx * R / r
```

It works well when the ball projection stays close to a circle, but its depth depends directly on a single radius value (`dZ/dr = -Z/r`), which is why the Trace method replaced it for fast throws. Its pipeline is documented in `algo_circle_fitting_graph.html`.

## Views

The Raylib/raygui interface provides five views: `2D` (events, clusters, fitted circle), `3D` (estimated trajectory in blue, simulated ground truth in red, 25 cm grid squares), `TOP` (top view and depth-bias analysis), `RMSE` (internal trajectory consistency), and `Trace` (ribbon fit, edge curves, width measurements, all tuning sliders).

## Simulated Sequences

Simulated sequences (Isaac Sim video converted to events with v2e) live in the local `sequences/` folder, each with `camera/intrinsics.json`, `labels/ground_truth.csv` and `metadata.json`. The reader automatically loads the per-sequence calibration (`fx = fy = 520`, no distortion, ball radius 0.02 m). The `Option` panel switches the reader source between `Sequences` and `Recordings`.

## Repository Layout

```text
.
├── env.sh                              build/run/calib/deps shell helpers
├── calibration_camera_DVXplorer_*.xml  real camera intrinsics
├── trace_algorithm_explanation.html    Trace algorithm documentation
├── algo_trace_graph.html               Trace pipeline graph
├── algo_circle_fitting_graph.html      circle-fitting pipeline graph
├── Stage_summary.tex / .pdf            internship report (both methods, validation, calibration)
├── images/                             report figures
├── scripts/                            calibration and utility scripts
├── sequences/                          local simulated sequences (git-ignored)
├── recordings/                         local camera recordings (git-ignored)
├── src/
│   └── Ball_Tracking_Cpp/
│       ├── include/Ball_Tracking_Cpp/
│       └── src/
└── README.md
```

Generated folders such as `build/`, `install/`, `log/`, `.deps/` are ignored by Git.

## Main Files

- `src/Ball_Tracking_Cpp/src/Gui.cpp`: **Trace algorithm** (ribbon fit, supported edges, width measurement, 3D conversion) and the 2D/3D/TOP/RMSE/Trace views.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp`: ROS 2 node loop, calibration selection, trace feeding, publication.
- `src/Ball_Tracking_Cpp/src/Camera.cpp`: camera acquisition, filtering, undistortion, sampling, and DBSCAN clustering.
- `src/Ball_Tracking_Cpp/src/BallTracker.cpp`: optional circle-fitting tracker, cluster validation, classic 3D pose estimation.
- `src/Ball_Tracking_Cpp/src/EventWriter.cpp`: H5/bin reading and writing, v2e `(N,4)` event format support.
- `src/Ball_Tracking_Cpp/include/Ball_Tracking_Cpp/RegressionAccumulator.hpp`: linear and quadratic regressions.

## Dependencies

- Linux
- ROS 2
- CMake
- GCC/G++
- OpenCV
- Eigen3
- fmt
- TBB
- Raylib
- raygui
- libusb
- HDF5
- `dv-processing`
- DVXplorer camera support

## Ubuntu 24.04 Dependency Setup

Check what is missing first:

```bash
scripts/install_dependencies_ubuntu24.sh --check
```

Install missing dependencies after the check:

```bash
scripts/install_dependencies_ubuntu24.sh --install
```

Or source the environment helper and use the aliases:

```bash
source env.sh
deps-check
deps-install
```

## Build and Run

From the workspace root:

```bash
source env.sh
build
run
```

`build` sets up the ROS environment and builds the C++ package with colcon; `run` starts the `talker` node with the GUI.

## Notes

Depth estimation is sensitive to the width measured in pixels: a small pixel error can create a large depth error, especially when the ball is far from the camera. The Trace view exposes every parameter of the supported-edge detector (`Support div/min/max`, `Support radius px`, `Border %`) and of the ribbon fit so this measurement can be inspected and tuned. See `trace_algorithm_explanation.html` for the tuning guide.
