# Project Command Reference

Detailed setup, calibration, launch, diagnostics, replay, system-identification
and test commands for this workspace.

> Real-robot commands can move the UR3e. Validate with fake hardware first,
> reduce the pendant speed, clear the workspace and keep an operator at the
> E-stop.

[Back to the project README](../README.md)

## Dependencies

- Linux and ROS 2 Humble.
- CMake, GCC/G++, OpenCV, Eigen3, fmt, TBB, Raylib/raygui, libusb and HDF5.
- `dv-processing` and DVXplorer camera support.
- Python dependencies used by the robot tools, notably FastAPI, Uvicorn,
  NumPy/SciPy and either ONNX Runtime or Torch for policy inference.

## Commands

All commands below are run from the workspace root unless stated otherwise.
Commands that target the real UR3e can move the arm: first use fake hardware,
keep the pendant speed reduced, clear the workspace and keep an operator at the
E-stop. The live-catch stacks start safely with `enable_command=false` unless
`--enable-command` is passed explicitly.

Detailed operator procedures are available for [real Trace perception](Robot_Control/procedure_test_perception_trace.md),
[intrinsic calibration](../wiki/calibration/intrinsic-calibration-runbook.md),
[extrinsic calibration](Robot_Control/procedure_calibration_extrinseque.md)
and the [real commanded catch session](Robot_Control/procedure_lancement_reel_trace_commande.md).

### Environment

```bash
cd ~/Dv-Rosws/Dv-Rosws
source env.sh
```

`env.sh` sources ROS and defines the main helpers. After a manual `colcon
build`, source the generated workspace before calling `ros2` directly:

```bash
source install/setup.bash
```

The most useful helpers are:

| Helper | Purpose |
| --- | --- |
| `build`, `run` | Build and launch the C++ perception GUI |
| `calib`, `calib_intrinsics` | Solve DVXplorer intrinsics with the recommended robust settings |
| `deps-check`, `deps-install` | Check or install Ubuntu dependencies |
| `ur3e_stack`, `ur3e_stop` | Start/stop the UR driver, MoveIt and Web UI |
| `ur3e_catch_stack`, `ur3e_catch_stop` | Start/stop the complete live-catch stack |
| `ur3e_ui`, `ur3e_ui_lan` | Start only the Web UI locally or on the LAN |
| `ur3e_validate`, `ur3e_replay_dry` | Validate a rollout or prepare a no-motion replay |
| `ur3e_controllers`, `ur3e_joints_once` | Inspect controllers or one joint-state message |
| `ur3e_test` | Run the rollout-replay and Web UI package tests |
| `compile-report` | Rebuild `Stage_summary.pdf` |

### Dependency Setup (Ubuntu 24.04)

Always preview missing dependencies before installing them:

```bash
scripts/install_dependencies_ubuntu24.sh --check
scripts/install_dependencies_ubuntu24.sh --install

# Equivalent helpers after `source env.sh`:
deps-check
deps-install
```

Install the current ROS 2 Humble Universal Robots driver once, then preview and
apply the wired-network configuration if this PC is connected to the UR3e:

```bash
./scripts/setup_ur_current_driver.sh
./scripts/configure_ur3e_ethernet.sh
./scripts/configure_ur3e_ethernet.sh --apply
ip route get 192.168.0.5
ping -c 3 192.168.0.5
```

The network script defaults to PC `192.168.0.3/24` and robot `192.168.0.5`.
Override its documented `UR3E_ETH_*` variables when using another interface or
address.

### Build

Build the perception application and its message dependency:

```bash
source env.sh
build
source install/setup.bash
```

`build` intentionally builds only `ur3e_catch_msgs` and
`ball_tracking_cpp`, using GCC 13 for the C++ package. Build the robot-side
packages separately:

```bash
colcon build --symlink-install --packages-select \
  ur3e_catch_msgs ur3e_live_catch ur3e_rollout_replay ur3e_web_ui ur3e_sysid
source install/setup.bash
```

Useful targeted builds:

```bash
colcon build --packages-select ball_tracking_cpp
colcon build --symlink-install --packages-select ur3e_catch_msgs ur3e_live_catch
colcon build --symlink-install --packages-select ur3e_rollout_replay ur3e_web_ui
```

### Perception Application

Launch the DVXplorer perception software and its Raylib GUI:

```bash
source env.sh
run
```

Equivalent launchers, useful from desktop shortcuts or another terminal:

```bash
scripts/launch_ball_tracking.sh
scripts/launch_ball_tracking_terminal.sh
ros2 run ball_tracking_cpp talker
```

Launch calibrated Trace perception as a ROS `BallState` publisher. Replace
`BALL_RADIUS_MM` with the measured physical **radius** of the ball:

```bash
BALL_RADIUS_MM=20.0
ros2 run ball_tracking_cpp talker --ros-args \
  --params-file src/ur3e_live_catch/config/live_catch.yaml \
  -p pose_source:=trace \
  -p ball_state_topic:=ball_state \
  -p camera_frame_id:=camera_optical \
  -p camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  -p ball_radius_mm:="$BALL_RADIUS_MM" \
  -p publish_legacy_pose:=false
```

Inspect its ROS output in another sourced terminal:

```bash
ros2 topic echo /ball_state
ros2 topic hz /ball_state
ros2 topic info /ball_state --verbose
```

Recorded and simulated sequences are selected inside the GUI from the Option
panel; the same `run` command is used.

### DVXplorer Intrinsic Calibration

No robot stack is needed. Plug in the camera, measure the monitor's active
width and height with a caliper, then run the calibration self-checks:

```bash
python3 scripts/event_mire_calibration.py --list-monitors
python3 scripts/event_mire_calibration.py --self-test
python3 -m py_compile \
  scripts/event_mire_calibration.py scripts/calibrate_intrinsics_from_mire.py
```

Capture 10-20 varied views. The `344 x 194 mm` values below are examples and
must be replaced by the measured screen dimensions:

```bash
python3 scripts/event_mire_calibration.py \
  --monitor 1 \
  --screen-width-mm 344 \
  --screen-height-mm 194 \
  --pattern grid_7x5 \
  --accum-ms 240
```

Use the GUI's `Calib` button for every accepted view, then solve with the
recommended robust and constrained settings:

```bash
source env.sh
calib --pattern grid_7x5
```

Equivalent explicit commands and useful diagnostics:

```bash
python3 scripts/calibrate_intrinsics_from_mire.py
python3 scripts/calibrate_intrinsics_from_mire.py \
  --robust --ransac-threshold-px 0.5
python3 scripts/calibrate_intrinsics_from_mire.py \
  --input-dir recordings/mire_calibration \
  --output-xml recordings/mire_calibration/intrinsics_from_mire.xml \
  --output-json recordings/mire_calibration/intrinsics_from_mire_report.json
```

Before hand-eye calibration, validate the selected XML in the capture GUI with
`Test calib` (F9) and `Test carré` (F10).

### Camera-to-Robot Extrinsic Calibration

Start `ur3e_stack` first so `base -> tool0` and `/joint_states` exist. In a
second terminal, source the same ROS domain and run the solver/collector
self-tests:

```bash
source env.sh
source install/setup.bash
python3 scripts/solve_handeye.py --self-test
python3 scripts/event_mire_calibration.py --self-test
```

With the phone mire mounted on `tool0`, launch the phone server and DVXplorer
collector:

```bash
scripts/run_handeye_session.sh
```

After collecting 15-20 diverse accepted poses, select the latest session and
solve `T_base_camera`:

```bash
ls -1t recordings/mire_calibration/handeye/handeye_samples_*.json
HAND_EYE_SAMPLES="$(ls -1t recordings/mire_calibration/handeye/handeye_samples_*.json | head -n 1)"
python3 scripts/solve_handeye.py "$HAND_EYE_SAMPLES" \
  --output-yaml calibration/handeye_result.yaml
```

Publish the resulting static camera transform in a terminal that remains open:

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml
```

Validate it from another sourced terminal:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical
```

Useful publication variants:

```bash
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml --print-only
python3 scripts/publish_camera_tf.py calibration/handeye_result.yaml --with-mire
```

Do not move the camera after calibration; any displacement invalidates the
extrinsic result.

### UR3e Driver and Web UI

Start the real driver, MoveIt and the UI at <http://127.0.0.1:8080>:

```bash
source env.sh
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 ur3e_stack
```

Useful variants and stop commands:

```bash
ur3e_stack --no-moveit
ur3e_stack --port 8081
ur3e_stack_lan --no-moveit
ur3e_controllers
ur3e_joints_once
ur3e_stop
```

Run fake hardware and the UI separately for motion-free UI tests:

```bash
# Terminal 1
source env.sh
ur3e_fake_driver

# Terminal 2
source env.sh
ur3e_ui
```

Install an optional local desktop launcher for the Web UI:

```bash
ur3e_install_web_app
```

### Live-Catch Stack

`ur3e_catch_stack` launches the UR driver, MoveIt, the selected ball source,
`live_catch_node`, the hoop TF and the Web UI. It starts in dry-run: open the
Test tab to launch a virtual ball or inspect a real throw, and enable command
only after all physical and ROS checks pass.

Fake hardware with a virtual ball:

```bash
source env.sh
ur3e_catch_stack --fake
```

Real UR3e with a virtual ball, still initially in dry-run:

```bash
UR3E_ROBOT_IP=192.168.0.5 UR3E_REVERSE_IP=192.168.0.3 \
  ur3e_catch_stack --real
```

Real DVXplorer Trace perception inside the single real-robot stack:

```bash
ur3e_catch_stop
ur3e_catch_stack --real --tracker \
  --hold-side left \
  --ball-radius 20.0 \
  --camera-calib recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  --model-path data/models/latest-left/policy_deterministic.onnx
```

`--hold-side`, the physical racket mount, the model metadata and the Web UI
selection must agree. Never start a separate `live_catch.launch.py` beside this
combined stack: two `live_catch_node` or ball publishers create a producer
conflict and command emission is blocked.

Other useful stack options:

```bash
ur3e_catch_stack --help
ur3e_catch_stack --fake --model-path data/models/latest/policy_deterministic.onnx
ur3e_catch_stack --fake --port 8081
UR3E_UI_HOST=0.0.0.0 ur3e_catch_stack --fake
ur3e_catch_stack --real --tracker --no-regression   # raw tracker debug only
ur3e_catch_stop
```

The recommended way to arm the real robot is the Web UI confirmation gate. The
CLI also supports `--enable-command`, but it starts already armed and must only
be used after the complete real-session checklist has passed.

Direct ROS launch equivalents:

```bash
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py \
  use_fake_hardware:=true
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py \
  robot_ip:=192.168.0.5 reverse_ip:=192.168.0.3 use_fake_hardware:=false

# Use only when a separate driver already exists; remains dry-run.
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_test_ball:=true trigger_mode:=true \
  publish_frame:=base_link enable_command:=false
ros2 service call /test_ball_node/throw std_srvs/srv/Trigger {}

# Show every supported launch argument.
ros2 launch ur3e_live_catch virtual_ball_robot.launch.py --show-args
ros2 launch ur3e_live_catch live_catch.launch.py --show-args
```

### Perception-to-Policy Dry-Run and Diagnostics

Run real Trace plus inference without starting the robot driver or sending
commands:

```bash
ros2 launch ur3e_live_catch live_catch.launch.py \
  use_tracker:=true \
  use_ball_regression:=true \
  enable_command:=false \
  model_path:=data/models/latest-left/policy_deterministic.onnx \
  camera_calibration_file:=recordings/mire_calibration/intrinsics_from_mire_robust_constrained.xml \
  ball_radius_mm:=20.0
```

Inspect frames, publishers, telemetry, controller state and the zero-lead
bring-up baseline:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical
ros2 run tf2_ros tf2_echo base_link hoop_center
ros2 topic info /ball_state --verbose
ros2 topic info /catch_telemetry --verbose
ros2 topic echo /catch_telemetry
ros2 control list_controllers
ros2 param get /ball_regression_node lead_time_s
ros2 run ur3e_live_catch latency_report
```

Record real raw detections and tune the ballistic regression offline:

```bash
BAG_DIRECTORY="recordings/ball_regression_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$BAG_DIRECTORY" /ball_state_raw /tf_static
```

Stop the recording with `Ctrl+C`, then replay the created bag directory:

```bash
python3 scripts/replay_ball_regression.py "$BAG_DIRECTORY" \
  --set depth_sigma_scale=8.0 --set max_rms_m=0.05
```

### Rollout Validation and Replay

Validate episode 0 and inspect a retimed replay without sending a trajectory:

```bash
source env.sh
ur3e_validate --episode 0
ros2 run ur3e_rollout_replay ur3e_replay_send --episode 0
```

For a no-driver test, provide a six-joint current pose explicitly:

```bash
ur3e_replay_dry --episode 0
```

The replay defaults to `--source realized`, the motion reached in simulation.
Use `--source target` only for diagnostics. `--execute` sends the trajectory and
must only be added after validation, preview and the real-robot safety gates:

```bash
ros2 run ur3e_rollout_replay ur3e_replay_validate \
  --episode 0 --source realized
ros2 run ur3e_rollout_replay ur3e_replay_send \
  --episode 0 --source realized --execute
```

The Web UI Rollout tab is the preferred path because it validates, previews and
asks for explicit confirmation before execution.

### UR3e System Identification

System identification can move the real robot. Validate the command and safety
gates first with fake hardware and `--dry-run`:

```bash
ros2 run ur3e_sysid run_sweep \
  --joint elbow --signal chirp \
  --f0 0.1 --f1 3.0 --amplitude 0.02 --duration 20 \
  --out-dir recordings/sysid --dry-run
```

Remove `--dry-run` only for an approved, clear real-robot session. Repeat the
step/chirp/ramp measurements one joint at a time, then fit the recorded gains:

```bash
ros2 run ur3e_sysid run_sweep \
  --joint elbow --signal chirp \
  --f0 0.1 --f1 3.0 --amplitude 0.02 --duration 20 \
  --out-dir recordings/sysid
ros2 run ur3e_sysid fit_gains \
  --in-dir recordings/sysid \
  --out ur3e_actuator_identified.yaml
```

Use `ros2 run ur3e_sysid run_sweep --help` and `ros2 run ur3e_sysid fit_gains
--help` for the step/ramp options, payload metadata and joint subsets.

### Tests and Static Checks

Run the package-local Python suites from the workspace root:

```bash
(cd src/ur3e_live_catch && \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
(cd src/ur3e_rollout_replay && \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
(cd src/ur3e_web_ui && \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
(cd src/ur3e_sysid && \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q)
```

Calibration and wiki checks:

```bash
python3 scripts/event_mire_calibration.py --self-test
python3 scripts/solve_handeye.py --self-test
python3 scripts/lint_llm_wiki.py
python3 scripts/update_agent_wiki.py
```

Build the internship report when its LaTeX sources change:

```bash
source env.sh
compile-report
```
