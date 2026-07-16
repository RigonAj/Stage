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

## Essential Commands

Run commands from the workspace root:

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
```

### 1. Launch Perception

Starts the DVXplorer Trace application and its graphical interface:

```bash
run
```

### 2. Launch the UR3e Robot and Web UI

Starts the real UR3e driver, MoveIt and the Web UI at
<http://127.0.0.1:8080>:

```bash
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 ur3e_stack
```

This targets the physical robot. Check the workspace, reduced-speed mode and
E-stop before sending any motion from the UI. Stop the stack with:

```bash
ur3e_stop
```

### 3. Publish the Hand-Eye Camera TF

Keep this command running in a separate terminal while using calibrated
perception:

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

Validate the transform from another sourced terminal:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical
```

### 4. Launch the Integrated Live-Catch Stack

The stack uses exactly one ball source. For a safe integration check, start
fake hardware with a virtual ball:

```bash
ur3e_catch_stack --fake
```

To launch a **virtual ball on the real UR3e** with the documented left-hand
mount and matching policy, omit `--tracker`:

```bash
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 \
  ur3e_catch_stack --real \
  --hold-side left \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

Both variants start with robot commanding disabled. Open
<http://127.0.0.1:8080>, select the **Test** tab and use **Launch virtual
ball**. The `--tracker` option selects the real DVXplorer instead and therefore
disables all virtual-ball controls. Stop either stack with:

```bash
ur3e_catch_stop
```

### 5. Real-Ball Perception Test (Robot Disarmed)

First validation of real Trace perception after the 2026-07-16 fix: the
tracker now starts on the **live camera** by default (`use_reader:=false`),
uses polarity `all` and prints a `trace status` heartbeat every 2 s. Event
recording is **manual**: press **REC** in the GUI to capture throws into
`recordings/realtest.h5` (or arm it from launch with `record:=true`). When a
recording starts, an existing non-empty target file is archived with a
timestamp suffix — never truncated. `--ball-radius` expects the **radius** in
millimetres: `45.0` corresponds to a Ø 90 mm ball (if your ball measures
45 mm across, use `22.5`). Trace depth scales directly with this value.

**Terminal A — hand-eye camera TF** (keep running for the whole session):

```bash
cd ~/Dv-Rosws/Dv-Rosws && source env.sh
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

**Terminal B — the single live-catch stack with the real tracker** (dry-run;
robot command stays disabled):

```bash
cd ~/Dv-Rosws/Dv-Rosws && source env.sh
ur3e_catch_stop
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 \
  ur3e_catch_stack --real --tracker \
  --hold-side left \
  --ball-radius 45.0 \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

Verify in the tracker output **before any throw**:

- `Input source: live camera (use_reader), trace polarity filter: all`
- `Calibration loaded from recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml`
- `Ball radius set to 45.0 mm`
- a `trace status: ...` heartbeat line every 2 s

To record the throws, press **REC** in the GUI (the terminal then prints
`Writer ready at recordings/realtest.h5`).

**Terminal C — one-time checks, then watch the raw tracker boundary:**

```bash
cd ~/Dv-Rosws/Dv-Rosws && source env.sh && source install/setup.bash
ros2 run tf2_ros tf2_echo base_link camera_optical              # TF must resolve
ros2 topic info /ball_state --verbose                           # exactly ONE publisher
ros2 topic echo /catch_telemetry --once | grep command_enabled  # expect: false
ros2 topic echo /ball_state_raw
```

**Terminal D — watch the fitted 60 Hz policy input:**

```bash
cd ~/Dv-Rosws/Dv-Rosws && source env.sh && source install/setup.bash
ros2 topic echo /ball_state
```

Optional ROS-level capture for offline analysis:

```bash
ros2 bag record -o rosbags/real_$(date +%Y%m%d_%H%M%S) \
  /ball_state_raw /ball_state /catch_telemetry /joint_states /tf /tf_static
```

**Procedure** — the robot stays disarmed for the whole session:

1. Throw the ball 5–10 times through the camera field of view.
2. During each flight, the Terminal B heartbeat must show `peak` rising above
   ~10 000 events and the `published=` counter incrementing. If not, the
   heartbeat names the starving stage (events → ribbon → 3d).
3. Terminal C must show `valid: true` bursts in `camera_optical` with a
   plausible positive depth (`position.z`); Terminal D must show `valid: true`
   fitted samples in `base_link` with a non-zero `velocity`.
4. Recordings survive by themselves: every time a recording (re)starts on an
   existing non-empty file, the previous one is archived as
   `realtest_YYYYMMDD_HHMMSS.h5` in `recordings/`.
5. Only when steps 2–3 are repeatable, arm command mode from the Web UI
   **Test** tab (<http://127.0.0.1:8080>) with a low `v_safe_scale` and the
   E-stop at hand.

A failed session can be replayed offline without the camera or robot
(`use_reader:=true reader_file:=<file>.h5`); the known-good reference is
`recordings/realtest_2026-07-09_backup.h5`. In the GUI, the reader Read-file
box starts preselected on `realtest.h5`, so **Reader → File** then **Play**
replays the last recording in one click. See
[docs/COMMANDS.md](docs/COMMANDS.md).

See [docs/COMMANDS.md](docs/COMMANDS.md) for dependency installation, builds,
calibration capture/solve, real Trace integration, diagnostics, replay,
system-identification, tests and every useful command.

## Notes

Depth estimation is sensitive to the width measured in pixels: a small pixel error can create a large depth error, especially when the ball is far from the camera. The Trace view exposes every parameter of the supported-edge detector (`Support div/min/max`, `Support radius px`, `Border %`) and of the ribbon fit so this measurement can be inspected and tuned. See `docs/trace_algorithm_explanation.html` for the tuning guide.
