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

10. Filter 3D outliers and fit the trajectory, then publish the ball state on the ROS 2 topic `ball_state` (`ur3e_catch_msgs/BallState`). The legacy `ball_position_3d_mm` topic can still be emitted for compatibility.

The full algorithm is documented visually in `docs/trace_algorithm_explanation.html` (detailed explanation, parameters, diagnostics) and `docs/Context/algo_trace_graph.html` (C++ pipeline graph).

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

It works well when the ball projection stays close to a circle, but its depth depends directly on a single radius value (`dZ/dr = -Z/r`), which is why the Trace method replaced it for fast throws. Its pipeline is documented in `docs/Context/algo_circle_fitting_graph.html`.

## Views

The Raylib/raygui interface provides five views: `2D` (events, clusters, fitted circle), `3D` (estimated trajectory in blue, simulated ground truth in red, 25 cm grid squares), `TOP` (top view and depth-bias analysis), `RMSE` (internal trajectory consistency), and `Trace` (ribbon fit, edge curves, width measurements, all tuning sliders).

## Simulated Sequences

Simulated sequences (Isaac Sim video converted to events with v2e) live in the local `sequences/` folder, each with `camera/intrinsics.json`, `labels/ground_truth.csv` and `metadata.json`. The reader automatically loads the per-sequence calibration (`fx = fy = 520`, no distortion, ball radius 0.02 m). The `Option` panel switches the reader source between `Sequences` and `Recordings`.

## Repository Layout

```text
.
├── env.sh                              build/run/calib/deps shell helpers
├── calibration_camera_DVXplorer_*.xml  real camera intrinsics
├── docs/trace_algorithm_explanation.html          Trace algorithm documentation
├── docs/Context/algo_trace_graph.html             Trace pipeline graph
├── docs/Context/algo_circle_fitting_graph.html    circle-fitting pipeline graph
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

## UR3e Live-Catch Inference Stack

Build the robot UI and live-catch packages before launching the inference stack:

```bash
source env.sh
colcon build --symlink-install --packages-select \
  ur3e_catch_msgs ur3e_live_catch ur3e_rollout_replay ur3e_web_ui
source install/setup.bash
```

Recommended one-command bring-up with fake hardware:

```bash
source env.sh
ur3e_catch_stack --fake
```

Recommended one-command bring-up on the real UR3e:

```bash
source env.sh
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 ur3e_catch_stack --real
```

`ur3e_catch_stack` launches the UR driver, MoveIt, `live_catch_node`,
`test_ball_node` in trigger mode, the Isaac-matched `wrist_3_link -> hoop_center`
TF, and the Web UI at `http://127.0.0.1:8080`. The live-catch node loads the
default policy from `data/models/policy_deterministic.onnx` with TorchScript
fallback, publishes telemetry, and stays in dry-run by default
(`enable_command=false`), so no robot command is emitted until the Web UI Test
tab explicitly enables command mode.

Open the Web UI, select the **Test** tab, choose `latest` or `best` if needed,
then use **Launch virtual ball** or **Isaac random**. To stop the stack:

```bash
ur3e_catch_stop
```

Status, 2026-07-02: the virtual-ball policy stream has been validated on the
real UR3e according to user hardware testing. The robot follows and holds after
the virtual ball grounds, but the response is still slow under conservative
bring-up limits (`v_safe_scale=0.5`); watchdog, tuning, real perception latency
and camera/hoop TF validation remain open before real-ball interception.

Useful variants:

```bash
# Use a specific exported policy.
ur3e_catch_stack --fake --model-path data/models/latest/policy_deterministic.onnx

# Expose the UI on the LAN or use another local port.
UR3E_UI_HOST=0.0.0.0 ur3e_catch_stack --fake
ur3e_catch_stack --fake --port 8081

# Start in command mode only after workspace/E-stop checks.
ur3e_catch_stack --real --model-path data/models/latest/policy_deterministic.onnx --enable-command
```

Direct ROS launch equivalents:

```bash
# Fake hardware + virtual ball + inference + UI.
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py use_fake_hardware:=true

# Real robot + virtual ball + inference + UI.
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py \
  robot_ip:=192.168.0.5 reverse_ip:=192.168.0.3 use_fake_hardware:=false

# If a separate UR3e driver stack is already running, start only live-catch + test ball.
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_test_ball:=true trigger_mode:=true publish_frame:=base_link enable_command:=false
ros2 service call /test_ball_node/throw std_srvs/srv/Trigger {}
```

## Notes

Depth estimation is sensitive to the width measured in pixels: a small pixel error can create a large depth error, especially when the ball is far from the camera. The Trace view exposes every parameter of the supported-edge detector (`Support div/min/max`, `Support radius px`, `Border %`) and of the ribbon fit so this measurement can be inspected and tuned. See `docs/trace_algorithm_explanation.html` for the tuning guide.
