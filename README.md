# Event-Based 3D Ball Tracking

ROS 2 C++ project for detecting and tracking a moving ball with a DVXplorer event camera.

The application reads asynchronous camera events, filters noise, clusters event points, estimates the 3D position of the ball, and displays the result in 2D, 3D, top-view, RMSE, and trace-analysis views.

## Processing Pipeline

1. Acquire event batches from a DVXplorer camera or from a recorded file.
2. Filter background activity with `dv-processing`.
3. Undistort event coordinates using the OpenCV calibration file.
4. Keep a recent time window and downsample events for real-time processing.
5. Cluster events with DBSCAN.
6. Fit a circle on the best ball cluster.
7. Estimate the 3D position from the apparent circle radius.
8. Accumulate 3D positions and fit a trajectory.
9. Display the tracking result with Raylib/raygui.

## 3D Position Estimation

The main tracker estimates the ball position from the fitted image circle.

For a ball with known real radius `R` and detected image radius `r`, depth is estimated as:

```text
Z = fx * R / r
X = ((u - cx) / fx) * Z
Y = ((v - cy) / fy) * Z
```

Where:

- `(u, v)` is the circle center in pixels;
- `r` is the detected circle radius in pixels;
- `fx`, `fy`, `cx`, `cy` come from the OpenCV camera calibration;
- `X`, `Y`, `Z` are returned in millimeters by the tracker.

The displayed 3D world coordinates are converted to meters for the Raylib scene.

## Trajectory Fit

Each valid 3D estimate is added to the trajectory history. The current trajectory model is:

```text
X(t) = a*t + b
Y(t) = a*t + b
Z(t) = a*t^2 + b*t + c
```

This keeps the lateral/depth components simple while allowing a parabolic component for the vertical motion.

## Trace View

Fast ball motion can create a long event trail. In that case, fitting a circle can overestimate the apparent radius and make the ball look too close.

The `Trace` view provides a second geometric analysis:

1. follow the detected ball using a moving window;
2. accumulate recent raw or undistorted events inside that window;
3. estimate the trail direction with PCA;
4. transform events into a local trace frame:

```text
s = position along the trail
h = position normal to the trail
```

5. split the trail along `s`;
6. detect dense upper/lower borders in each slice using a 1D histogram on `h`;
7. fit upper, middle, and lower curves;
8. measure local trail width on the normal direction;
9. convert center point plus width in pixels into 3D points.

For trace-based depth, the measured width is treated as the apparent diameter:

```text
Z = f_effective * real_diameter / width_px
```

The trace algorithm is documented in `trace_algorithm_explanation.html`.

## Repository Layout

```text
.
├── build.sh
├── calibration_camera_DVXplorer_DXA00265-2026_04_23_13_33_50.xml
├── images/
├── scripts/
├── src/
│   ├── Ball_Tracking_Cpp/
│   │   ├── include/Ball_Tracking_Cpp/
│   │   └── src/
│   └── ball_tracking/
├── Stage.pdf
├── trace_algorithm_explanation.html
└── README.md
```

Generated folders such as `build/`, `install/`, `log/`, `.deps/`, local recordings, and local agent notes are ignored by Git.

## Main Files

- `src/Ball_Tracking_Cpp/src/Camera.cpp`: camera acquisition, filtering, undistortion, sampling, and DBSCAN clustering.
- `src/Ball_Tracking_Cpp/src/BallTracker.cpp`: circle fitting, cluster validation, 3D pose estimation, and trajectory fitting.
- `src/Ball_Tracking_Cpp/src/Gui.cpp`: 2D/3D visualization, top view, RMSE view, and trace-ribbon analysis.
- `src/Ball_Tracking_Cpp/include/Ball_Tracking_Cpp/RegressionAccumulator.hpp`: linear and quadratic regressions.
- `src/Ball_Tracking_Cpp/src/publisher_member_function.cpp`: ROS 2 node loop and GUI/tracker integration.

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

## Build

From the workspace root:

```bash
source env.sh
build
```

The helper script sets up the ROS environment and builds the C++ package with colcon.

## Notes

Depth estimation is sensitive to the apparent radius or width measured in pixels. A small pixel error can create a large depth error, especially when the ball is far from the camera. The trace view exists to inspect and improve this measurement when fast motion creates an elongated event trail.
